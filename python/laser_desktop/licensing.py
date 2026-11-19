# -*- coding: utf-8 -*-
"""启动闸门：**在创建窗口、读模型之前**校验离线许可证。

为什么要有这一层（而不是直接用 :mod:`licensing`）
--------------------------------------------------
``licensing`` 是**签名与判定**的底层：它只回答"这份文件是不是我们签的、
这台机器对不对、过没过期"，出错时抛 :class:`licensing.LicenseError`，消息写给
工程师看。这个模块是**现场闸门**，两件底层不该管的事由它负责：

1. **找文件**：许可证放哪（环境变量 / 数据目录 / ``%PROGRAMDATA%``）、公钥放哪
   （随包资产 / 内嵌常量）；
2. **说人话**：把每一种失败翻译成"现在该做什么"，并且**绝不把 traceback 抛给
   用户**——现场看不懂堆栈，看到了也只会拍张照片发过来。

判定顺序刻意是"越便宜越先"：找文件 → 公钥 → 读文件 → 验签 → 有效期 →
机器绑定。前一步不过就不必再算后面（验签要塞一个 4 KB 的公钥运算，机器指纹
要枚举网卡，最慢的放最后）。

这个闸门的作用
--------------
**"防顺手拷走 + 可追溯"，不是不可破解的 DRM。** Python 程序可以被改、MAC 可以
用驱动伪造——真挡不住存心破解的人。它挡的是"把安装包拷到另一台机器直接用"，
并让每个安装都能对上客户名与序列号。所以这里**没有**混淆、反调试、时间炸弹：
那些只会让现场运维更难，也不会真的挡住谁。

由此带来的两条运维约定（是流程，不是 bug）：

* **换网卡 / 换主板 = 重新签发**：指纹变了本机就不再是"授权绑定的机器"；
* **续期也要重新签发**：签名覆盖 ``expires``，手改日期只会让整份证失效。

底层能力留在 :mod:`licensing`（本模块不重复实现验签/指纹算法），已知边界写
在那边的 ``hwid`` 模块说明里。

``LASER_SKIP_LICENSE`` 为什么只在源码模式生效
--------------------------------------------
开发时天天改代码，手边不该非要有一份授权；但**交付包不能靠设个环境变量绕过**，
否则这道门在第一台客户机上就被运维顺手关掉了。所以跳过需要同时满足：
显式请求（``skip=True`` 或 ``LASER_SKIP_LICENSE=1``）**且** ``not is_frozen()``。
冻结包里即使设了变量也照常校验，并在日志里写明"该变量被忽略"。
"""
from __future__ import annotations

import datetime as _dt
import json
import logging
import os
from pathlib import Path
from typing import NamedTuple

from . import paths

_LOGGER = logging.getLogger(__name__)

#: 直接指定许可证文件的位置（U 盘、共享盘、临时排障都用它，免得去改安装目录）。
ENV_LICENSE = "LASER_LICENSE"

#: 开发期跳过校验的开关；**只在未冻结时生效**（见模块说明）。
ENV_SKIP = "LASER_SKIP_LICENSE"

#: 许可证文件名。放 ``data_root()`` 或 ``%PROGRAMDATA%\\LaserCalibration\\``。
LICENSE_NAME = "license.json"

#: 公钥文件名（随包发布；私钥**绝不**进仓库、绝不进安装包）。
PUBLIC_KEY_NAME = "license_public.pem"

#: 冻结包里放许可证的目录名：``%PROGRAMDATA%\\LaserCalibration``。
PROGRAM_DATA_APP = "LaserCalibration"

#: 许可证校验失败时的退出码。与普通错误 1 区分，运维看到 3 就知道是授权问题。
EXIT_LICENSE = 3

#: 内嵌公钥（PEM 字节）。**冻结包里必须非空**，由 ``packaging/desktop.spec``
#: 在构建时读仓库根的 ``license_public.pem`` 生成 ``laser_desktop/_embedded_key.py``
#: 注入；源码运行时留 ``None``，退回读文件。
EMBEDDED_PUBLIC_KEY_PEM: bytes | None = None


def _embedded_public_key() -> bytes | None:
    """构建期注入的公钥（``laser_desktop/_embedded_key.py``，不进仓库）。

    惰性导入而不是模块级导入：源码树里若残留一份旧的生成模块，模块级导入会让
    开发机静默用上"上一把公钥"，症状是"明明重签了却验不过"。
    """
    try:
        from ._embedded_key import PUBLIC_KEY_PEM   # noqa: WPS433 - 构建期产物
    except Exception:                               # noqa: BLE001
        return None
    return bytes(PUBLIC_KEY_PEM) if PUBLIC_KEY_PEM else None

#: 找不到许可证时的话术（``%s`` 是它应该在的位置）。
#: 找不到许可证时的话术。
#:
#: **不在界面上给出文件该放哪**：那句话读起来像"你自己拷一个文件进去就行"，
#: 现场照着做出来的往往是无效文件（许可证绑定本机网卡指纹，不是通用文件），
#: 反而把问题拖长。放哪个路径只写进 ``detail``（支持与日志用），主提示只说
#: "联系供应商"——分发时由供应商把授权随安装包放好或当场导入。
_MESSAGE_NOT_FOUND = "未找到许可证文件。请联系供应商获取授权。"

#: 源码运行时追加的一句：把开发者的出路写在错误里，而不是藏在文档里。
#: 冻结包**不**追加——现场看到”设法绕过校验”的提示只会让人误以为绕过是正当操作。
_MESSAGE_DEV_HINT = ("（开发调试：可设环境变量 %s=1 跳过校验；"
                     "冻结后的安装包不受该变量影响）")

def _not_found_message() -> str:
    """找不到证时的话术：源码运行额外告诉开发者怎么跳过。

    路径**不进**这句话（见 :data:`_MESSAGE_NOT_FOUND`），只进 ``detail``。
    """
    message = _MESSAGE_NOT_FOUND
    if not paths.is_frozen():
        message += _MESSAGE_DEV_HINT % ENV_SKIP
    return message

#: 公钥缺失时的话术。装包不完整是唯一合理原因，所以直接让人重装。
_MESSAGE_NO_PUBLIC_KEY = ("程序缺少内置公钥，无法校验许可证（安装包可能不完整）。"
                          "请重新安装。")


class GateResult(NamedTuple):
    """闸门结论。

    Attributes:
        ok: 能不能放行。
        message: 给人看的一句话。失败时是**可直接照做**的中文，成功时是摘要
            （谁被授权、到什么时候）。
        detail: 日志/``--self-check``/对话框的"详细信息"用：路径、指纹、有效期、
            具体是底层哪一步拒绝的。**不弹给最终用户**当主文案。
    """

    ok: bool
    message: str
    detail: str


# ---------------------------------------------------------------------------
# 找许可证 / 找公钥
# ---------------------------------------------------------------------------
def _program_data_dir() -> Path:
    """所有用户共享的程序数据目录（Windows 的 ``%PROGRAMDATA%``）。

    优先读环境变量：测试与"换盘符"的现场都靠它，别把 ``C:\\ProgramData`` 写死
    到无法覆盖。
    """
    base = os.environ.get("PROGRAMDATA")
    if base:
        return Path(base)
    return Path(r"C:\ProgramData")


def license_candidates() -> list[Path]:
    """按优先级列出许可证**可能**的位置（第 0 个就是"应该放哪"）。

    顺序：``LASER_LICENSE`` 指定的文件 → ``data_root()/license.json`` →
    （仅冻结时）``%PROGRAMDATA%/LaserCalibration/license.json``。

    第二个是"当前用户装一份"（免管理员权限），第三个是"整机装一份"；打包后
    用户数据目录在 ``%LOCALAPPDATA%``，运维多半会放 ``%PROGRAMDATA%``，所以两个
    都认。源码运行时没有第三个——开发机上不该从系统目录里捡到一份陈年授权。
    """
    candidates = []
    override = os.environ.get(ENV_LICENSE)
    if override:
        candidates.append(Path(override).expanduser())
    try:
        candidates.append(paths.data_root() / LICENSE_NAME)
    except Exception:                     # noqa: BLE001 - 路径解析不该让启动崩
        _LOGGER.warning("data_root() 解析失败，跳过该候选位置", exc_info=True)
    if paths.is_frozen():
        candidates.append(_program_data_dir() / PROGRAM_DATA_APP / LICENSE_NAME)
    return candidates


def find_license_file() -> Path | None:
    """找到那份真的存在的许可证；一份都没有就返回 ``None``。

    Returns:
        Path | None: 命中的文件（已确认是**普通文件**，不是目录）。

    Note:
        ``LASER_LICENSE`` 指了一个不存在的路径时**继续往下找**：写错环境变量的
        人手里往往还有一份放在默认位置的授权，替他省一次排障。
    """
    for candidate in license_candidates():
        try:
            if candidate.is_file():
                return candidate
        except OSError:                   # 网络盘掉线、权限不足都算"没找到"
            continue
    return None


def preferred_path() -> Path:
    """许可证**应该**放的位置（找不到文件时用它写提示）。"""
    candidates = license_candidates()
    return candidates[0] if candidates else Path(LICENSE_NAME)


def public_key_candidates() -> list[Path]:
    """公钥可能在哪。

    冻结后 ``asset_root()`` 是 ``_internal/assets``，spec 把公钥收在
    ``assets/license_public.pem``；源码运行时 ``asset_root()`` 是仓库根，于是
    第二个候选正好是仓库里那份 ``data/assets/license_public.pem``（构建机放
    公钥的地方，可删可重建）。两个都列出来，报先命中的那个。
    """
    try:
        root = paths.asset_root()
    except Exception:                     # noqa: BLE001 - 资产根解析不了就等于没公钥
        _LOGGER.warning("asset_root() 解析失败，视为没有随包公钥", exc_info=True)
        return []
    return [root / PUBLIC_KEY_NAME, root / "data" / "assets" / PUBLIC_KEY_NAME]


def public_key() -> bytes | None:
    """内嵌公钥（PEM 字节）；取不到时返回 ``None``。

    返回 ``None`` 不是异常：调用方要据此说"安装包可能不完整，请重新安装"，
    这比抛一个 ``FileNotFoundError`` 有用得多。

    **冻结（打包）后只认"编进代码里的"那把**，不再读随包文件。原因是一条实测
    可用的绕过：随包公钥是一个可替换的**文件**，攻击者把自己的公钥拷进去
    （`_internal/assets/license_public.pem`），再用自己的私钥签一张 2099 年的
    许可证，校验就会通过——整个过程没碰一行代码。公钥编进字节码后，同样的目的
    就得先去改代码，那是另一个门槛（见 ``docs/开发参考.md`` §4.6.3）。

    源码运行时仍允许读文件：开发机不该为了跑一次界面就先去生成密钥对。
    """
    if EMBEDDED_PUBLIC_KEY_PEM:
        return EMBEDDED_PUBLIC_KEY_PEM
    if paths.is_frozen():
        # 冻结包只认编进代码的公钥；缺了就是构建漏了一步，不退回读文件
        return _embedded_public_key()
    for candidate in public_key_candidates():
        try:
            if candidate.is_file():
                data = candidate.read_bytes()
                if data.strip():
                    return data
        except OSError:
            continue
    return None


# ---------------------------------------------------------------------------
# 判定
# ---------------------------------------------------------------------------
def _skipped() -> GateResult:
    """开发期跳过的结论（调用方已经确认过"未冻结"）。"""
    return GateResult(True,
                      "已跳过许可证校验（源码运行 + %s=1）" % ENV_SKIP,
                      detail="跳过开关生效条件：显式请求且未冻结（sys.frozen 为空）。"
                             "冻结的安装包会忽略这个开关。")


def _want_skip(skip: bool) -> bool:
    """这次调用是否在**请求**跳过（还没判断冻结与否）。"""
    if skip:
        return True
    return os.environ.get(ENV_SKIP, "").strip() == "1"


def skip_active() -> bool:
    """当前是否处于"开发期跳过"：显式请求**且**未冻结。

    给 ``--self-check`` 用：它要能区分"真的校验通过了"与"被跳过了"，否则自检
    会把一个没校验过的包报成 ``[OK]``。
    """
    return _want_skip(False) and not paths.is_frozen()


def _serialization():
    """延迟 import ``cryptography`` 的 serialization 模块（没装返回 ``None``）。

    延迟到调用点才 import：不想让 ``--print-paths`` 这种纯路径查询因为少一个
    密码库就直接崩（它还要负责把“公钥缺失”报出来）。
    """
    try:
        from cryptography.hazmat.primitives import serialization
    except ImportError:
        return None
    return serialization


def _load_key(pem: bytes, serialization):
    """PEM 字节 → Ed25519 公钥对象。

    Raises:
        LicenseError: 公钥内容坏了（依赖缺失由调用方先判，不在这里）。
    """
    from licensing.license import LicenseError

    try:
        return serialization.load_pem_public_key(pem)
    except Exception as error:            # noqa: BLE001 - 坏 PEM 一律算公钥不可用
        raise LicenseError(f"公钥内容不是合法的 PEM：{error}") from error


def _check_signature(payload: dict, path: Path, key) -> GateResult | None:
    """验签；通过返回 ``None``，不通过返回结论。"""
    from licensing.license import LicenseError, verify_signature

    # 再读一次只为取 sig 段。底层 load_license() 已经把"JSON 坏了 / 缺 payload
    # 或 sig / 版本不支持"都校验过了，这里不会失败；而把版本校验和字段校验在
    # 闸门里复刻一遍，等于让两处判定慢慢走散。
    try:
        signature = json.loads(path.read_text(encoding="utf-8"))["sig"]
    except Exception as error:            # noqa: BLE001 - 文件被换走/被占用
        return GateResult(
            False,
            f"许可证文件读取失败（{type(error).__name__}: {error}）。"
            "请确认文件完整，或重新向供应商索取 license.json。",
            detail=f"{path}：读第二次（取签名段）时失败")
    try:
        verify_signature(payload, signature, key)
    except LicenseError as error:
        return GateResult(
            False,
            "许可证签名校验失败：这份文件不是本程序签发的，或内容被改过"
            "（客户名、有效期、绑定的机器，改动任何一个字段都会这样）。"
            "请向供应商重新索取 license.json。",
            detail=f"{path}：{error}")
    return None


def _check_expiry(payload: dict, today: _dt.date) -> GateResult | None:
    """有效期；通过（或永久）返回 ``None``。

    ``expires`` 为 ``None`` 表示永久授权。日期比较写成"今天 > 到期日"，
    与底层 :func:`licensing.license._check_expiry` 同义——这里重写一遍不是为了
    另立一套判定，而是因为**只有知道那个日期才能写出"已于 X 到期"**。
    """
    expires = payload.get("expires")
    if not expires:
        return None
    try:
        deadline = _dt.date.fromisoformat(str(expires))
    except ValueError:
        return GateResult(
            False,
            f"许可证里的有效期格式不对（{expires!r}），无法判断是否到期。"
            "请联系供应商重新签发。",
            detail=f"payload['expires'] = {expires!r}，预期 YYYY-MM-DD 或 null")
    if today > deadline:
        return GateResult(
            False,
            f"许可证已于 {deadline.isoformat()} 到期，请联系供应商续签。",
            detail=f"expires={deadline.isoformat()}，今天 {today.isoformat()}，"
                   f"共过期 {(today - deadline).days} 天")
    return None


def _bound_fingerprints(payload: dict) -> set:
    """授权里绑定的指纹集合（统一小写）。

    载荷被改坏了（``fingerprints`` 不是列表）时这里会抛，由 :func:`check` 兜住——
    这正是“永不把异常摊给用户”要靠外层的原因。
    """
    return {str(value).strip().lower()
            for value in payload.get("fingerprints") or []}


def _required_matches(payload: dict) -> int:
    """要先命中几个指纹才算同一台机器（``min_match``，至少 1）。"""
    return max(1, int(payload.get("min_match") or 1))


def _check_machine(payload: dict, local: list[str]) -> GateResult | None:
    """机器绑定；通过返回 ``None``。

    本机指纹**必须出现在提示里**：现场唯一的自救动作就是"把这两行发给供应商"，
    让他们照着重新签发。底层 :class:`LicenseError` 有意不带本机指纹（它不该
    外泄硬件信息），所以这句人话只能由闸门拼。
    """
    from licensing.license import LicenseError, check_machine

    allowed = _bound_fingerprints(payload)
    required = _required_matches(payload)
    if not local:
        return GateResult(
            False,
            "读不到本机的物理网卡，无法按网卡校验许可证（在虚拟机里运行、或网卡被"
            "禁用都会这样）。请在实体机上运行；若确实只能用这台机器，请联系供应商。",
            detail=f"machine_fingerprints() 返回空；授权绑定 {len(allowed)} 个指纹，"
                   f"min_match={required}")
    try:
        check_machine(payload, local)
    except LicenseError as error:
        matched = sorted(set(local) & allowed)
        listed = ", ".join(local)
        return GateResult(
            False,
            f"本机不是授权绑定的机器（命中 {len(matched)}/{required}）。"
            f"本机网卡指纹：{listed}。请把这两行发给供应商重新签发。",
            detail=f"{error}\n本机指纹（逐行，直接复制发给供应商）：\n"
                   + "\n".join(f"  {value}" for value in local)
                   + f"\n授权绑定 {len(allowed)} 个；min_match={required}；"
                     f"已命中 {len(matched)} 个")
    return None


def _detail_line(path: Path, local: list[str]) -> str:
    """一行日志用的现场信息（路径 + 冻结状态 + 指纹数）。"""
    return (f"许可证 {path}；frozen={paths.is_frozen()}；"
            f"本机物理网卡指纹 {len(local)} 个")


def check(licenses_path=None, *, skip=False, public_key_pem=None,
          local_fingerprints=None, today=None) -> GateResult:
    """完整判定：找不到文件 / 公钥缺失 / 读不了 / 签名不对 / 已过期 / 指纹不匹配。

    Args:
        licenses_path: 显式指定许可证文件；``None`` 时走 :func:`find_license_file`。
        skip: 请求跳过校验（仅开发期有效，见模块说明）。
        public_key_pem: 公钥 PEM 字节（测试注入用）；``None`` 时走
            :func:`public_key`。
        local_fingerprints: 本机指纹（测试注入用）；``None`` 时现算。
        today: 今天的日期（测试注入用）；``None`` 时取系统日期。

    Returns:
        GateResult: **永不抛异常**。任何底层异常都被翻译成 ``ok=False`` 的
        中文结论——现场看不懂 traceback，崩给他看也解决不了问题。
    """
    try:
        return _check(licenses_path, skip=skip, public_key_pem=public_key_pem,
                      local_fingerprints=local_fingerprints, today=today)
    except Exception as error:            # noqa: BLE001 - 闸门的契约就是不抛
        _LOGGER.exception("许可证校验出现意外异常")
        return GateResult(
            False,
            f"许可证校验出现意外错误（{type(error).__name__}: {error}）。"
            "请把这条信息发给供应商。",
            detail=f"未预期的异常；licenses_path={licenses_path!r}")


def _check(licenses_path, *, skip, public_key_pem, local_fingerprints,
           today) -> GateResult:
    """:func:`check` 的实现（外面那层只负责兜异常）。"""
    from licensing.license import LicenseError, load_license
    from licensing.hwid import machine_fingerprints

    # 0. 跳过：显式请求 **且** 未冻结才算数。
    if _want_skip(skip):
        if paths.is_frozen():
            _LOGGER.warning("冻结包里 %s / skip 被忽略：安装包不允许靠环境变量绕过"
                            "许可证校验", ENV_SKIP)
        else:
            _LOGGER.info("许可证校验已跳过（源码运行 + %s）", ENV_SKIP)
            return _skipped()

    # 1. 找文件。
    if licenses_path is not None:
        path = Path(licenses_path).expanduser()
        exists = False
        try:
            exists = path.is_file()
        except OSError:
            exists = False
        if not exists:
            return GateResult(
                False, _not_found_message(),
                detail=f"显式指定的许可证路径不存在或不是文件：{path}；"
                       f"候选位置：{[str(item) for item in license_candidates()]}")
    else:
        found = find_license_file()
        if found is None:
            target = preferred_path()
            return GateResult(
                False, _not_found_message(),
                detail="没有找到许可证；按优先级查过："
                       + "、".join(str(item) for item in license_candidates()))
        path = found

    # 2. 公钥。缺公钥时不必读文件——校验根本做不了。
    pem = public_key_pem if public_key_pem is not None else public_key()
    if not pem:
        return GateResult(
            False, _MESSAGE_NO_PUBLIC_KEY,
            detail="内嵌常量 EMBEDDED_PUBLIC_KEY_PEM 为空，且这些位置没有 "
                   + PUBLIC_KEY_NAME + "："
                   + "、".join(str(item) for item in public_key_candidates()))
    serialization = _serialization()
    if serialization is None:
        return GateResult(
            False,
            "程序缺少授权校验组件（cryptography），安装包可能不完整。"
            "请重新安装本程序；若仍不行，请联系供应商要一份完整安装包。",
            detail="import cryptography.hazmat.primitives.serialization 失败；"
                   "依赖见 requirements-license.txt")
    try:
        key = _load_key(pem, serialization)
    except LicenseError as error:
        return GateResult(
            False,
            "程序里的公钥读不出来（安装包可能已损坏）。请重新安装。",
            detail=f"{error}；公钥来源：" + (
                "内嵌常量" if public_key_pem is None and EMBEDDED_PUBLIC_KEY_PEM
                else "随包文件 " + "、".join(
                    str(item) for item in public_key_candidates())))

    # 3. 读文件 + 格式/版本。
    try:
        payload = load_license(path)
    except LicenseError as error:
        return GateResult(
            False,
            f"许可证文件读不了：{error}。请确认拿到的就是供应商发的那份"
            "license.json（没被改名、没被截断），必要时重新索取。",
            detail=f"{path}；{error}")
    except OSError as error:              # load_license 只兜了 FileNotFound
        return GateResult(
            False,
            f"许可证文件读不了（{type(error).__name__}: {error}）。"
            "请确认文件没被占用、磁盘没掉线。",
            detail=f"{path}；{error}")

    # 4. 验签。
    failed = _check_signature(payload, path, key)
    if failed is not None:
        # 用 info 而不是 warning：调用方（入口/自检）会把这个结论原样报给用户，
        # 再 warning 一遍（logging 的 lastResort 也是 stderr）就变成同一个失败
        # 连打两遍，看上去像出了两个错。
        _LOGGER.info("许可证验签失败：%s（%s）", failed.message, failed.detail)
        return failed

    # 5. 有效期（比机器指纹便宜：不枚举网卡、不起 PowerShell）。
    today = today or _dt.date.today()
    failed = _check_expiry(payload, today)
    if failed is not None:
        _LOGGER.info("许可证已过期：%s", failed.message)
        return failed

    # 6. 机器绑定（最贵的一步：要枚举物理网卡，Windows 上走 PowerShell）。
    local = (list(local_fingerprints) if local_fingerprints is not None
             else machine_fingerprints())
    failed = _check_machine(payload, local)
    if failed is not None:
        _LOGGER.info("许可证与机器不匹配：%s", failed.message)
        return failed

    # 7. 通过：摘要给用户，细节进日志。
    licensee = str(payload.get("licensee") or "（未填客户名）")
    serial = str(payload.get("serial") or "（无序列号）")
    expires = payload.get("expires")
    features = list(payload.get("features") or [])
    matched = len(set(local) & _bound_fingerprints(payload))
    required = _required_matches(payload)
    message = (f"授权给 {licensee}（序列 {serial}），"
               f"有效期至 {expires or '永久'}")
    detail = (f"{_detail_line(path, local)}；命中 {matched}/{required} 个绑定指纹；"
              f"功能 {features or '（未限定）'}；签发日 {payload.get('issued')}")
    _LOGGER.info("许可证校验通过：%s（%s）", message, detail)
    return GateResult(True, message, detail)
