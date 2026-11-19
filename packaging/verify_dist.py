#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""构建产物自检：只看 ``dist/LaserCalibration/``，不 import 仓库里的任何代码。

    python packaging/verify_dist.py                     # 默认查 dist/LaserCalibration
    python packaging/verify_dist.py --skip-self-check    # 只查文件，不启动 exe
    python packaging/verify_dist.py --installer dist\\setup\\LaserCalibration-setup-0.9.0.exe

为什么单独写一个、而不是塞进 pytest
------------------------------------
这个脚本要回答的问题在**打包之后**才存在：PyInstaller 有没有把 QtWebEngine 的
孑进程与资源装进去、有没有把 ``tests/`` 和 ``robot_sim/`` 一起带进去、体积是不是
离谱。它跑在 Windows 构建机上（那份 venv 里没有仓库的测试依赖），所以只用标准库，
也不 import ``laser_desktop``——一旦 import，就变成"用同一份代码证明自己没错"。

判定分三级
----------
``[OK]``   通过。
``[FAIL]`` 硬失败，退出码 1。缺一个都意味着包是坏的（大多表现为"图区白屏"）。
         也包括"包里出现了私钥"——那等于把签发权一起发出去了。
``[WARN]`` 只提示，不影响退出码：体积超预算、多出来的调试资源、两份 plotly.js 不一致、
         包里没有授权公钥（包能打出来，但客户机上装上也用不了）。
           ——体积超了还能用，白屏不能用，所以两者不同级。

退出码：0 全通过（可能有 WARN）；1 有 FAIL。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

#: 解压目录的体积预期（MB）。计划 §5.4：瘦身后 395–490 MB。
EXPECTED_MIN_MB = 395
EXPECTED_MAX_MB = 490
#: 安装包的体积预期（MB）：155–200 MB（lzma2，压缩比按 webengine wheel 实测推算）。
INSTALLER_MIN_MB = 155
INSTALLER_MAX_MB = 200
#: 孑进程用的 Chromium 内核 dll，正常 203.5 MB；小于这个数说明包被裁过头了。
WEBENGINE_CORE_MIN_MB = 180
#: 保留的 locale 上限（zh-CN / zh-TW / en-US）。
MAX_LOCALES = 5

OK, FAIL, WARN = "OK", "FAIL", "WARN"


class Report:
    """收集结果并统一打印（顺序：先全部检查，最后给结论）。"""

    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str]] = []

    def add(self, level: str, title: str, detail: str = "") -> None:
        self.rows.append((level, title, detail))
        print(f"  [{level:4s}] {title}" + (f" —— {detail}" if detail else ""))

    def ok(self, title: str, detail: str = "") -> None:
        self.add(OK, title, detail)

    def fail(self, title: str, detail: str = "") -> None:
        self.add(FAIL, title, detail)

    def warn(self, title: str, detail: str = "") -> None:
        self.add(WARN, title, detail)

    def count(self, level: str) -> int:
        return sum(1 for row in self.rows if row[0] == level)


def section(title: str) -> None:
    print()
    print("=" * 74)
    print(title)
    print("=" * 74)


# ---------------------------------------------------------------------------
# 尺寸工具
# ---------------------------------------------------------------------------
def dir_size(path: Path) -> int:
    """目录下所有普通文件的字节数（不跟随符号链接）。"""
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            full = os.path.join(root, name)
            try:
                if os.path.islink(full):
                    continue
                total += os.path.getsize(full)
            except OSError:
                pass
    return total


def mb(value: int) -> float:
    return value / 1024.0 / 1024.0


def biggest_files(root: Path, limit: int = 8) -> list[tuple[int, str]]:
    """最大的几个文件（体积异常时用来看是谁胖了）。"""
    found = []
    for dirpath, _dirs, files in os.walk(root):
        for name in files:
            full = os.path.join(dirpath, name)
            try:
                if os.path.islink(full):
                    continue
                found.append((os.path.getsize(full),
                              os.path.relpath(full, root).replace("\\", "/")))
            except OSError:
                pass
    found.sort(reverse=True)
    return found[:limit]


def find_by_name(root: Path, name: str, max_hits: int = 6) -> list[Path]:
    """在整棵树里按文件名找（用来"在哪儿都行、但必须有一份"的东西）。"""
    hits = []
    for dirpath, _dirs, files in os.walk(root):
        if name in files:
            hits.append(Path(dirpath) / name)
            if len(hits) >= max_hits:
                break
    return hits


# ---------------------------------------------------------------------------
# 1. 必需文件
# ---------------------------------------------------------------------------
def check_required(dist: Path, internal: Path, rep: Report) -> None:
    section("1. 必需文件（缺一个都可能是白屏）")

    exe = dist / "LaserCalibration.exe"
    if exe.is_file():
        rep.ok("可执行文件", f"{exe.name}（{mb(exe.stat().st_size):.1f} MB）")
    else:
        rep.fail("可执行文件", f"没有 {exe}")

    # QtWebEngine 的孑进程：hook 收到 _internal/PySide6/ 下。这里既认准确位置、
    # 也接受"在别处但存在"，只是把实际位置打出来（PyInstaller 换版本挪过位置）。
    checks = [
        # (说明, 期望相对路径, 文件名)
        ("QtWebEngine 孑进程", "PySide6/QtWebEngineProcess.exe",
         "QtWebEngineProcess.exe"),
        ("Chromium 资源包", "PySide6/resources/qtwebengine_resources.pak",
         "qtwebengine_resources.pak"),
        ("高分屏资源包 100%", "PySide6/resources/qtwebengine_resources_100p.pak",
         "qtwebengine_resources_100p.pak"),
        ("高分屏资源包 200%", "PySide6/resources/qtwebengine_resources_200p.pak",
         "qtwebengine_resources_200p.pak"),
        ("V8 快照", "PySide6/resources/v8_context_snapshot.bin",
         "v8_context_snapshot.bin"),
        ("ICU 数据", "PySide6/resources/icudtl.dat", "icudtl.dat"),
        ("孑进程的 qt.conf", "PySide6/qt.conf", "qt.conf"),
        ("中文语言包", "PySide6/translations/qtwebengine_locales/zh-CN.pak",
         "zh-CN.pak"),
        ("QtWebEngine 内核 dll", "PySide6/Qt6WebEngineCore.dll",
         "Qt6WebEngineCore.dll"),
    ]
    for title, relative, name in checks:
        expected = internal / relative.replace("/", os.sep)
        if expected.is_file():
            size = expected.stat().st_size
            rep.ok(title, f"_internal/{relative}（{mb(size):.1f} MB）")
            continue
        # 位置与预期不同也接受（PyInstaller 换版本可能挪位置），但要报出实际位置。
        hits = find_by_name(internal, name)
        if hits:
            rep.ok(title, "位置与预期不同，实际在："
                   + "、".join(str(h.relative_to(internal)).replace("\\", "/")
                              for h in hits[:3]))
        else:
            rep.fail(title, f"缺 _internal/{relative}（整棵树里也没有 {name}）")

    # 两份 plotly.min.js：一份是运行时真正被 get_plotlyjs() 读的
    # （plotly 包的 package_data），一份是我们自己的只读资产（assets/plotly）。
    runtime_js = internal / "plotly" / "package_data" / "plotly.min.js"
    asset_js = internal / "assets" / "plotly" / "plotly.min.js"
    for title, path, why in (
            ("plotly.min.js（运行时读这份）", runtime_js,
             "plotly.offline.get_plotlyjs() 走 pkgutil 读它，缺了壳页里的 "
             "<script src> 是空的"),
            ("plotly.min.js（随包资产）", asset_js,
             "离线画图的显式副本，paths.asset_root()/plotly/plotly.min.js"),
    ):
        if path.is_file():
            rep.ok(title, f"{mb(path.stat().st_size):.2f} MB —— {why}")
        else:
            rep.fail(title, f"缺 {path.relative_to(internal)} —— {why}")

    if runtime_js.is_file() and asset_js.is_file():
        same = runtime_js.read_bytes() == asset_js.read_bytes()
        if same:
            rep.ok("两份 plotly.min.js 一致", "同源，离线图不会出现两套渲染")
        else:
            rep.warn("两份 plotly.min.js 不一致",
                     "assets/plotly 那份可能过期了（不影响运行，但现场看到的是"
                     "内置的那份）")

    # 机器人模型：spec 只收 data/configs 顶层的 *.urdf，至少要两个。
    configs = internal / "assets" / "data" / "configs"
    urdfs = sorted(p.name for p in configs.glob("*.urdf")) if configs.is_dir() else []
    if len(urdfs) >= 2:
        rep.ok("机器人模型（*.urdf）", f"{len(urdfs)} 个：{'、'.join(urdfs)}")
    elif urdfs:
        rep.fail("机器人模型（*.urdf）", f"只有 {len(urdfs)} 个：{urdfs}，预期 ≥ 2")
    else:
        rep.fail("机器人模型（*.urdf）", f"缺 {configs} 下的 *.urdf")

    dh = sorted(p.name for p in configs.glob("*_dh.json")) if configs.is_dir() else []
    if dh:
        rep.ok("DH 覆盖文件（*_dh.json）", "、".join(dh))
    else:
        rep.fail("DH 覆盖文件（*_dh.json）", f"缺 {configs} 下的 *_dh.json")

    # matplotlib 的 mpl-data：plot.png 要字体，缺了 50 点解算会在出图那步炸。
    mpl = internal / "matplotlib" / "mpl-data" / "fonts" / "ttf" / "DejaVuSans.ttf"
    if mpl.is_file():
        rep.ok("matplotlib mpl-data", "DejaVuSans.ttf 在")
    else:
        rep.fail("matplotlib mpl-data",
                 "缺 DejaVuSans.ttf —— core 写 plot.png 会 FileNotFoundError")


# ---------------------------------------------------------------------------
# 1b. 离线授权：公钥可以在（缺了只警告），私钥绝不能出现
# ---------------------------------------------------------------------------
#: 授权公钥在包里的位置。
#:
#: 注意：**冻结运行时不读这个文件**。公钥在构建时被写进
#: ``laser_desktop/_embedded_key.py`` 编进 PYZ——因为随包文件可以被替换成攻击者
#: 自己的公钥，再配一张自签的证就绕过去了（实测过）。这里保留一项检查，是因为
#: 包里有这份副本便于人工核对版本；它**不是**校验依据，缺了也不影响授权。
PUBLIC_KEY_NAME = "license_public.pem"

#: PEM 私钥的头部特征。
PRIVATE_KEY_MARKERS = (b"PRIVATE KEY-----",)

#: 扫内容时只看这么小的文件。私钥是几百字节的 PEM 文本，没必要为了找它去读
#: 那 203 MB 的 Qt6WebEngineCore.dll（扫全树体积会让这个脚本慢上好几十秒）。
KEY_SCAN_MAX_BYTES = 64 * 1024


def _looks_like_private_key(path: Path) -> bool:
    """文件内容像不像 PEM 私钥（只看前 64 KB）。"""
    try:
        if path.stat().st_size > KEY_SCAN_MAX_BYTES:
            return False
        with path.open("rb") as handle:
            head = handle.read(KEY_SCAN_MAX_BYTES)
    except OSError:
        return False
    return any(marker in head for marker in PRIVATE_KEY_MARKERS)


def check_license_key(dist: Path, internal: Path, rep: Report) -> None:
    section("1b. 离线授权：公钥可以在，私钥绝不能出现")

    key = internal / "assets" / PUBLIC_KEY_NAME
    if key.is_file():
        # PEM 是文本，但读成 bytes 更宽容（带 BOM / CRLF 都无所谓）。
        blob = key.read_bytes()
        if b"PUBLIC KEY-----" in blob:
            rep.ok("授权公钥", f"_internal/assets/{PUBLIC_KEY_NAME}（{len(blob)} B）")
        else:
            rep.fail("授权公钥",
                     f"{key.relative_to(internal)} 在，但内容不像 PEM 公钥——"
                     "构建时是不是把私钥或者别的文件拷过来了？")
    else:
        # 副本缺失不是问题（冻结运行只认编进代码里的那把）；但如果**内置也没编进去**，
        # 客户机上启动会报“程序缺少内置公钥”——那一步由下面第 4 节的 `--self-check`
        # 输出来判定，这里只提示一句。
        rep.ok("包里没有公钥副本（正常）",
               "冻结运行时只用编进代码的那把；是否编进去了由 --self-check 的第 7 项判定")

    # 私钥泄露是灾难级的：拿到它就能自己签任意授权，而且无法撤销。
    #
    # 判据要**准**而不是广：早先"文件名里有 private 就算"直接误报了三个真文件——
    #   api-ms-win-crt-private-l1-1-0.dll（Windows CRT 的私有 API 库）
    #   QmlAssetDownloaderPrivateplugin_init.cpp.obj（Qt 的编译中间产物）
    # 名字里带 private 的合法二进制太多了。现在只认两类：
    #   1) 内容像 PEM 私钥（扫前 64 KB 找 "PRIVATE KEY-----"）——这是硬证据；
    #   2) 扩展名是 .pem/.key 且文件名带 private/id_rsa/id_ed25519——这才是我们的私钥命名。
    files = [path for path in dist.rglob("*") if path.is_file()]

    def _private_by_name(path: Path) -> bool:
        name = path.name.lower()
        if path.suffix.lower() not in (".pem", ".key"):
            return False
        return ("private" in name or name.startswith("id_rsa")
                or name.startswith("id_ed25519") or "privkey" in name)

    by_name = sorted({p for p in files if _private_by_name(p)})
    by_content = sorted({p for p in files if _looks_like_private_key(p)})
    leaked = sorted(set(by_name) | set(by_content))
    if leaked:
        rep.fail("包里出现了私钥",
                 "、".join(str(p.relative_to(dist)).replace("\\", "/")
                          for p in leaked[:3])
                 + " —— 私钥泄露等于签发权泄露，这个包不能用，改完 spec 重打")
    else:
        rep.ok("没有私钥",
               f"扫了 {len(files)} 个文件：没有 PEM 私钥内容、"
               f"也没有 *.pem/*.key 的私钥命名")


# ---------------------------------------------------------------------------
# 2. 不该出现的东西
# ---------------------------------------------------------------------------
def check_forbidden(dist: Path, rep: Report) -> None:
    section("2. 不该打进去的东西")

    # 仿真器资产：不是机器人模型，list_robots() 试加载必然 KeyError，
    # 只会让下拉框多三个"不可用"条目（spec 里用 glob 只收 *.urdf / *_dh.json）。
    sim = [Path(p) for p in dist.rglob("robot_sim")
           if p.is_dir() and "robot_sim" in p.name]
    if sim:
        rep.fail("robot_sim/ 被打进去了",
                 "、".join(str(p.relative_to(dist)) for p in sim[:3]))
    else:
        rep.ok("没有 data/configs/robot_sim/", "仿真器控制器 JSON 不属于交付资产")

    # tests 目录 / 测试模块：+几 MB 而且会把用户的运行目录搞混。
    #
    # 只认**我们自己的** tests：第三方库自带 `matplotlib/testing`、`plotly/testing`
    # 这类目录是它们的正常组成部分，不是"打包漏了排除"。早先按目录名匹配，于是
    # `_internal/matplotlib/testing` 被当成我们的测试目录，报了一条假 FAIL。
    ours = {"laser_web", "laser_desktop", "excitation_trajectory",
            "robot_model", "handeye", "licensing"}
    test_dirs = []
    for path in dist.rglob("*"):
        if not path.is_dir() or path.name not in ("tests", "test"):
            continue
        parts = [part.lower() for part in path.relative_to(dist).parts]
        # 顶层 tests/（仓库根拷进来的）或 _internal/<我们的包>/tests
        if "tests" == parts[-1] or "test" == parts[-1]:
            if len(parts) == 1 or parts[0] in ours or any(p in ours for p in parts):
                test_dirs.append(path)
    if test_dirs:
        rep.fail("tests/ 被打进去了",
                 "、".join(str(p.relative_to(dist)) for p in test_dirs[:3]))
    else:
        rep.ok("没有 tests/ 目录", "包括 matplotlib/testing 这类第三方自带目录也不算")

    test_modules = [p for p in dist.rglob("@test_*.pyc")] + \
                   [p for p in dist.rglob("test_*.pyc")]
    if test_modules:
        rep.fail("测试模块 .pyc 被打进去了",
                 "、".join(str(p.relative_to(dist)) for p in test_modules[:3]))
    else:
        rep.ok("没有 test_*.pyc", "")

    # pytest 的缓存在 __pycache__ 里；构建机上的仓库根如果有 .pytest_cache，
    # 说明有人从仓库里直接拷过东西。
    caches = [p for p in dist.rglob("*")
              if p.is_dir() and p.name in (".pytest_cache", ".ruff_cache", ".mypy_cache")]
    if caches:
        rep.fail("测试/检查工具的缓存目录被打进去了",
                 "、".join(str(p.relative_to(dist)) for p in caches[:3]))
    else:
        rep.ok("没有 .pytest_cache / .ruff_cache / .mypy_cache", "")

    # __pycache__ 本身不算错（--onedir 里有些包就靠它），但里面不该有 .pyc 的测试。
    pycache = list(dist.rglob("__pycache__"))
    if pycache:
        rep.warn(f"有 {len(pycache)} 个 __pycache__ 目录",
                 "一般无害；若里面有测试模块请查 spec 的 excludes")

    # 源码（*.py）不应出现：我们用的是默认的"编译进 PYZ"，露在磁盘上的 .py
    # 多半是误收集（例如 hook 里的 include_py_files=True）。
    stray_py = [p for p in dist.rglob("*.py") if p.is_file()]
    if stray_py:
        rep.warn(f"有 {len(stray_py)} 个 .py 源码文件露在外面",
                 "、".join(str(p.relative_to(dist)) for p in stray_py[:3]))
    else:
        rep.ok("没有裸露的 .py 源码", "")


# ---------------------------------------------------------------------------
# 3. 体积
# ---------------------------------------------------------------------------
def check_sizes(dist: Path, internal: Path, installer: Path | None,
                rep: Report) -> None:
    section("3. 体积（超出只警告，不判失败）")

    total = dir_size(dist)
    total_mb = mb(total)
    detail = (f"{total_mb:.0f} MB，预期 {EXPECTED_MIN_MB}–{EXPECTED_MAX_MB} MB")
    if total_mb > EXPECTED_MAX_MB:
        rep.warn("解压目录偏大", detail + "；多半是 QtWebEngine 没瘦身"
                 "（debug 资源 +84 MB、多余 locale +44 MB）")
    elif total_mb < EXPECTED_MIN_MB:
        rep.warn("解压目录偏小", detail + "；先确认第 1 节全 OK，再怀疑被裁过头")
    else:
        rep.ok("解压目录体积", detail)

    core = internal / "PySide6" / "Qt6WebEngineCore.dll"
    if core.is_file():
        size_mb = mb(core.stat().st_size)
        if size_mb < WEBENGINE_CORE_MIN_MB:
            rep.warn("QtWebEngine 内核 dll 偏小",
                     f"{size_mb:.0f} MB < {WEBENGINE_CORE_MIN_MB} MB，"
                     "可能不是完整的 WebEngine 构建")
        else:
            rep.ok("QtWebEngine 内核 dll", f"{size_mb:.0f} MB")

    # 瘦身是否真生效：debug 资源与 devtools 都该是 0。
    debug = [p for p in internal.rglob("*.debug.pak")] + \
            [p for p in internal.rglob("*.debug.bin")]
    if debug:
        rep.warn(f"debug 资源没剪掉（{len(debug)} 个，共 "
                 f"{mb(sum(p.stat().st_size for p in debug)):.0f} MB）",
                 "spec 的 _slim() 没生效？不影响运行，只是白背 84 MB")
    else:
        rep.ok("QtWebEngine debug 资源已剪掉", "省 84 MB")

    devtools = list(internal.rglob("qtwebengine_devtools_resources.pak"))
    if devtools:
        rep.warn("devtools 资源还在",
                 f"{mb(devtools[0].stat().st_size):.0f} MB；只有远程调试才用")
    else:
        rep.ok("devtools 资源已剪掉", "省 12 MB")

    locales_dir = internal / "PySide6" / "translations" / "qtwebengine_locales"
    if locales_dir.is_dir():
        locales = sorted(p.name for p in locales_dir.glob("*.pak"))
        if len(locales) > MAX_LOCALES:
            rep.warn("语言包太多", f"{len(locales)} 个：{'、'.join(locales[:8])}…")
        else:
            rep.ok("语言包已裁剪", "、".join(locales))
        if "zh-CN.pak" not in locales:
            rep.fail("缺 zh-CN.pak", "界面/Chromium 提示会退回英文")

    if installer is None:
        rep.ok("安装包体积", "跳过（还没打 setup.exe；打完用 --installer 再查）")
        return
    if not installer.is_file():
        rep.fail("安装包体积", f"没有 {installer}")
        return
    size_mb = mb(installer.stat().st_size)
    detail = f"{size_mb:.0f} MB，预期 {INSTALLER_MIN_MB}–{INSTALLER_MAX_MB} MB"
    if size_mb > INSTALLER_MAX_MB:
        rep.warn("安装包偏大", detail + "；先看解压目录是不是也偏大")
    elif size_mb < INSTALLER_MIN_MB:
        rep.warn("安装包偏小", detail + "；可疑，确认解压目录是完整的")
    else:
        rep.ok("安装包体积", detail)

    print()
    print("  目录里最大的 8 个文件：")
    for size, name in biggest_files(dist):
        print(f"    {mb(size):8.1f} MB  {name}")


# ---------------------------------------------------------------------------
# 4. 跑一次 --self-check
# ---------------------------------------------------------------------------
def check_self_check(dist: Path, timeout: int, rep: Report) -> None:
    section("4. 启动自检（LaserCalibration.exe --self-check）")

    exe = dist / "LaserCalibration.exe"
    if not exe.is_file():
        rep.fail("启动自检", "没有 exe，跳过")
        return

    # 关键：把输出编码固定成 UTF-8。冻结后的 Python 在管道上默认按系统代码页
    # （中文 Windows 是 cp936）写，这里按 utf-8 解码才不会变成乱码。
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    print(f"  运行：{exe} --self-check（超时 {timeout} s）")
    try:
        completed = subprocess.run(
            [str(exe), "--self-check"],
            capture_output=True, timeout=timeout, env=env,
            cwd=str(dist),                 # 从安装目录起跑，路径解析与现场一致
        )
    except subprocess.TimeoutExpired:
        rep.fail("启动自检超时", f">{timeout} s 没有退出；图视图/WebEngine 可能卡住")
        return
    except OSError as error:
        rep.fail("启动自检起不来", str(error))
        return

    stdout = completed.stdout.decode("utf-8", "replace")
    stderr = completed.stderr.decode("utf-8", "replace")
    for line in (stdout + stderr).splitlines():
        print("    " + line)

    fails = [line for line in stdout.splitlines() if "[FAIL]" in line]
    oks = [line for line in stdout.splitlines() if "[OK]" in line]
    # [SKIP] 不是失败：构建机上没装许可证是常态（第 7 项），把它算进 OK 数会
    # 让“N 项全 OK”与实际项数对不上。
    skips = [line for line in stdout.splitlines() if "[SKIP]" in line]
    if completed.returncode == 0 and not fails:
        if skips:
            rep.ok("启动自检",
                   f"{len(oks)} 项 OK、{len(skips)} 项跳过"
                   f"（{skips[0].split(']')[-1].strip()}），退出码 0")
        else:
            rep.ok("启动自检", f"{len(oks)} 项全 OK，退出码 0")
    else:
        rep.fail("启动自检",
                 f"退出码 {completed.returncode}，FAIL {len(fails)} 项"
                 + ("：" + fails[0].strip() if fails else ""))


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="检查 PyInstaller 产物（Windows 构建机上跑）")
    parser.add_argument("--dist", default=os.path.join("dist", "LaserCalibration"),
                        help="解压目录（默认 dist/LaserCalibration）")
    parser.add_argument("--installer", default=None,
                        help="setup.exe 路径（可选；不给就只查解压目录）")
    parser.add_argument("--skip-self-check", action="store_true",
                        help="不启动 exe（只想快速查文件时用）")
    parser.add_argument("--timeout", type=int, default=300,
                        help="--self-check 的超时秒数（默认 300）")
    args = parser.parse_args(argv)

    dist = Path(args.dist).expanduser().resolve()
    installer = (Path(args.installer).expanduser().resolve()
                 if args.installer else None)

    print(f"产物目录：{dist}")
    if not dist.is_dir():
        print(f"[FAIL] 没有这个目录：{dist}\n"
              f"  先在 Windows 构建机上跑：pyinstaller packaging\\desktop.spec --noconfirm")
        return 1

    internal = dist / "_internal"
    rep = Report()
    if not internal.is_dir():
        rep.fail("_internal/ 目录",
                 "没有它说明不是 PyInstaller 6 的 onedir 产物（或 contents_directory 被改了）")
    check_required(dist, internal, rep)
    check_license_key(dist, internal, rep)
    check_forbidden(dist, rep)
    check_sizes(dist, internal, installer, rep)
    if args.skip_self_check:
        section("4. 启动自检")
        rep.ok("启动自检", "已按 --skip-self-check 跳过")
    else:
        check_self_check(dist, args.timeout, rep)

    section("结论")
    print(f"  OK {rep.count(OK)} 项，WARN {rep.count(WARN)} 项，FAIL {rep.count(FAIL)} 项")
    if rep.count(FAIL):
        print("\n  有 FAIL：这个包不能用（多半是白屏或起不来）。逐条看上面的说明，")
        print("  改 spec 后重新 pyinstaller；不要手动往 dist/ 里拷文件——那不是修。")
        return 1
    print("\n  全部必需项通过。下一步：打 Inno Setup 安装包，然后在**干净机**上装一遍。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
