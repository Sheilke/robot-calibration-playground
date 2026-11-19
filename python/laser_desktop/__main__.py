#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桌面版入口。

    PYTHONPATH=python python3 -m laser_desktop                    # 正常启动
    PYTHONPATH=python python3 -m laser_desktop --print-paths      # 只打印路径解析结果
    PYTHONPATH=python python3 -m laser_desktop --self-check       # 启动自检（不显示窗口）

``--self-check`` 为什么要有
--------------------------
打包后的故障几乎都长成同一副样子：**窗口能开，图区白白的**。没有报错、没有日志，
现场只能说"打不开图"。这个模式把七个环节逐个点名，每项 ``[OK]``/``[FAIL]``，
任一 FAIL 退出码 1：

1. Qt 绑定与版本（交付包必须是 PySide6；PyQt5 是 GPL，只在 Linux 本地验证用）
2. ``plotly.min.js`` 能不能读到（离线画图的前提）
3. ``data/configs`` 里能真正加载的机器人模型数
4. ``data_root()`` 能不能写（装到只读目录的典型症状）
5. **真起一个 PlotView**，画出 8 个点，再从壳页里读回 ``window.__state()``
   （这一项过了，就说明 QtWebEngine、孑进程、plotly.js、通道全都活着）
6. 能不能读到物理网卡（离线授权要绑 MAC）
7. 许可证本身（签名 / 有效期 / 机器绑定）；**没有许可证不算 FAIL**

它**不显示主窗口**，所以 ``QT_QPA_PLATFORM=offscreen`` 下也能跑；
**不加** ``--disable-gpu``（那会把 WebGL 一起关掉，查不出真问题）。
构建脚本 ``packaging/build_windows.ps1`` 与 ``packaging/verify_dist.py`` 都会
调用它——安装包"能装"跟"能出图"是两件事。

注意：``qt_compat`` 必须**在 QApplication 之前**导入（Qt5 的 WebEngine 硬性要求），
所以这里的 import 顺序是刻意的。

许可证闸门（``--self-check`` / ``--print-paths`` 例外）
-----------------------------------------------------
正常启动时先过 :func:`laser_desktop.licensing.check`：不通过就退出码 **3**
（与普通错误码 1 区分，运维一看就知道是授权问题）。它刻意放在
**建 QApplication 之前、import ``laser_web.core`` 之前**：

* 早于建窗口 —— 源码运行失败时只往 stderr 打一行就退，不必为一个注定要被
  关掉的窗口付 Qt 的启动成本；冻结包里才建临时 app 弹框（现场没控制台）；
* 早于读模型 —— 授权不过就不该碰数据（读模型、建 ``runs/`` 都会留痕迹）。

两个诊断开关**不过闸门**——它们是没证时唯一的排障手段——但都会把许可证
状态报出来。
"""
from __future__ import annotations

import logging
import sys

from . import licensing, paths
from .qt_compat import (BINDING, DEV_ONLY_BINDING, QtWidgets,
                        apply_chinese_font, configure_webengine_environment)

_LOGGER = logging.getLogger(__name__)

#: ``--self-check`` 等壳页加载的上限（秒）。EDR 全盘扫描时 Windows 上首读
#: 203 MB 的 Qt6WebEngineCore.dll 确实会慢，给足余量但不至于无限等。
SELF_CHECK_LOAD_TIMEOUT = 25.0
#: 从壳页回读 ``window.__state()`` 的上限（秒）。
SELF_CHECK_JS_TIMEOUT = 15.0


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)

    # 两个诊断开关先返回：它们是**没许可证时唯一能用的东西**，所以不过闸门——
    # 但都要把许可证状态报出来（否则就变成"查授权要先有授权"）。
    if "--print-paths" in args:
        _print_paths()
        return 0

    if "--self-check" in args:
        return self_check()

    # ----- 许可证闸门：必须在这里 -----
    # **早于 QApplication**：源码运行时失败只往 stderr 打两行就退，不必为一个
    #   注定要被关掉的窗口付 Qt 的启动成本（冻结包里才建一个临时 app 去弹框，
    #   因为 QMessageBox 没有 QApplication 就没法显示）。
    # **早于 laser_web.core / MainWindow**：授权不过就不该碰模型目录；读模型、
    #   建 runs/ 目录都会在盘上留下痕迹，那不该发生在未授权的机器上。
    gate = licensing.check()
    if not gate.ok:
        return _refuse(gate)
    _LOGGER.info("许可证：%s（%s）", gate.message, gate.detail)

    configure_webengine_environment()      # 必须在建 QApplication 之前
    paths.ensure_writable_roots()

    app = QtWidgets.QApplication(args)
    family = apply_chinese_font(app)

    from .main_window import MainWindow
    window = MainWindow()
    window.show()
    print(f"桌面版启动：Qt 绑定 {BINDING}"
          + ("（仅供本地验证）" if DEV_ONLY_BINDING else "")
          + f"，中文字体 {family}；许可证 {gate.message}")
    return app.exec()


def _print_paths() -> None:
    """``--print-paths``：路径解析结果 + 许可证状态（诊断工具，不过闸门）。"""
    for name, value in paths.describe().items():
        print(f"{name:8s} {value}")
    print()
    print("许可证")
    found = licensing.find_license_file()
    print(f"  {'文件':6s} {found if found is not None else '未找到'}")
    print(f"  {'应放':6s} {licensing.preferred_path()}")
    result = licensing.check()
    print(f"  {'状态':6s} {'通过' if result.ok else '未通过'}：{result.message}")
    if result.detail:
        print(f"  {'详细':6s} {result.detail}")


def _refuse(gate: licensing.GateResult) -> int:
    """许可证不通过：**冻结包弹框 + 源码模式只打 stderr**，一律退出码 3。

    为什么区分两种：开发时天天改代码，弹窗会打断手上的事（而且它挡着终端输出）；
    现场没控制台（``--windowed`` 打包），不弹框就只剩"双击了一下，什么都没发生"。

    退出码 :data:`licensing.EXIT_LICENSE` （3）与普通错误 1 分开：运维在快捷方式、
    计划任务或 Inno 的日志里看到 3，就知道是授权问题，不用再猜。
    """
    if paths.is_frozen():
        # 冻结包是 --windowed（没控制台）：stderr 被入口脚本接成了 os.devnull，
        # 所以日志记录 + 对话框是唯一两条痕迹，两个都要留。
        _LOGGER.warning("许可证校验未通过：%s（%s）", gate.message, gate.detail)
        _show_refusal_box(gate)
        return licensing.EXIT_LICENSE

    # 源码模式：开发者看终端。这里**不**再走 logging——lastResort 也是写 stderr，
    # 同一个失败会连打两遍，看上去像出了两个错。
    stream = sys.stderr if sys.stderr is not None else None
    if stream is not None:
        print(f"许可证校验未通过：{gate.message}", file=stream)
        if gate.detail:
            print(f"（详细信息）{gate.detail}", file=stream)
        try:
            stream.flush()
        except (OSError, ValueError):       # pragma: no cover - 流已关闭
            pass
    return licensing.EXIT_LICENSE


def _show_refusal_box(gate: licensing.GateResult) -> None:
    """冻结包里的失败对话框（正文是人话，堆栈折叠在"详细信息"里）。

    这里必须自己建一个 ``QApplication``：弹框是 Qt 控件，没有 app 就没有事件
    循环，``QMessageBox.critical`` 会直接倒（或什么都不显）。建完就退，不复用。
    任何失败（无显示器、Qt 起不来）都退回"只有 stderr"——弹不出框不能又漏掉
    退出码。
    """
    try:
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        try:
            apply_chinese_font(app)        # 不然中文标题是方块，等于没提示
        except Exception:                  # noqa: BLE001 - 字体不影响能不能退出
            pass
        box = QtWidgets.QMessageBox()
        box.setIcon(QtWidgets.QMessageBox.Critical)
        box.setWindowTitle("许可证校验未通过")
        box.setText(gate.message)
        if gate.detail:
            box.setDetailedText(gate.detail)
        box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        # PyQt5/PySide6 的 exec 名字不一样（老的只认 exec_），两个都试一下。
        show = getattr(box, "exec", None) or getattr(box, "exec_", None)
        if show is not None:
            show()
    except Exception:                      # noqa: BLE001 - 弹框失败也要退码 3
        _LOGGER.exception("许可证失败对话框显示不出来（已退回到控制台提示）")


# ---------------------------------------------------------------------------
# --self-check
# ---------------------------------------------------------------------------
class _Report:
    """自检结果：逐项打印 + 结束时落一份日志（现场求助时直接发这个文件）。

    每项是 ``(ok, 标题, 细节)``，``ok`` 有三种值：``True`` 通过、``False`` 失败、
    ``None`` 跳过。跳过既不是通过也不是失败：它**不参与退出码**（典型例子是
    在构建机上检查许可证——那里本来就不该有绑定到构建机的授权）。
    """

    def __init__(self) -> None:
        self.results = []                  # [(ok, 标题, 细节)]
        self._lines = []

    def emit(self, text: str = "") -> None:
        self._lines.append(text)
        stream = sys.stdout if sys.stdout is not None else sys.stderr
        if stream is None:                 # pragma: no cover - 极端情况（无任何流）
            return
        print(text, file=stream)
        try:
            stream.flush()
        except (OSError, ValueError):       # pragma: no cover - 流已关闭
            pass

    def record(self, ok: bool, title: str, detail: str = "") -> bool:
        self.results.append((bool(ok), title, detail))
        tail = f"  {detail}" if detail else ""
        self.emit(f"{'[OK]  ' if ok else '[FAIL]'} {title}{tail}")
        return bool(ok)

    def record_skip(self, title: str, detail: str = "") -> None:
        """记一项"跳过"：打 ``[SKIP]``，**不算失败**。"""
        self.results.append((None, title, detail))
        tail = f"  {detail}" if detail else ""
        self.emit(f"[SKIP] {title}{tail}")

    @property
    def failures(self) -> list:
        # 必须写 ``is False``：跳过的 ``None`` 也是假值，用 ``not`` 会把跳过算成失败。
        return [item for item in self.results if item[0] is False]

    @property
    def skipped(self) -> list:
        return [item for item in self.results if item[0] is None]

    def save(self, folder) -> None:
        """把整份输出写到 ``data_root()/self-check.log``（写不了就算了）。"""
        try:
            folder.mkdir(parents=True, exist_ok=True)
            (folder / "self-check.log").write_text(
                "\n".join(self._lines) + "\n", encoding="utf-8")
        except OSError:                     # pragma: no cover - 只读环境
            pass


def _pump(app, seconds: float) -> None:
    """跑一小会儿事件循环（不阻塞回调，也不需要嵌套 QEventLoop）。"""
    import time
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.02)


def _wait_until(app, predicate, timeout: float) -> bool:
    """等 predicate() 变真，最多 timeout 秒（期间照常处理 Qt 事件）。"""
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        app.processEvents()
        time.sleep(0.02)
    return bool(predicate())


def _check_binding(report: _Report) -> None:
    """1. Qt 绑定与版本。

    交付形态必须是 PySide6（LGPL）：PyQt5 是 GPL，随包分发会踩许可证。
    所以只有**已经打包**（``sys.frozen``）却用了 PyQt5 时才算 FAIL；
    源码运行时在 Linux 上用 PyQt5 验证界面是既有做法（见 ``qt_compat``）。
    """
    from .qt_compat import QtCore
    qt_version = QtCore.qVersion()
    # 版本号必须取**当前实际用的那个绑定**：本机的 venv 里同时装着 PySide6
    # （Linux wheel 没 QtWebEngine，于是退回 PyQt5），直接 import PySide6 报版本
    # 会打出“PyQt5 6.12.0”这种自相矛盾的东西。
    version = "?"
    if BINDING == "PySide6":
        try:
            import PySide6
            version = PySide6.__version__
        except ImportError:                 # pragma: no cover - 不该发生
            pass
    else:
        try:
            from PyQt5.QtCore import PYQT_VERSION_STR
            version = PYQT_VERSION_STR
        except ImportError:                 # pragma: no cover - 不该发生
            pass

    detail = f"{BINDING} {version}（Qt {qt_version}）"
    if DEV_ONLY_BINDING:
        detail += "，仅本地验证"
        if paths.is_frozen():
            report.record(
                False, "1. Qt 绑定与版本",
                detail + " —— 交付包用了 PyQt5（GPL）！重新用 PySide6 打包")
            return
    report.record(True, "1. Qt 绑定与版本", detail)


def _plotly_asset_candidates() -> list:
    """随包 ``plotly.min.js`` 可能的位置。

    冻结时资产根是 ``_MEIPASS/assets``（spec 把它们放在 ``assets/plotly/``）；
    源码运行时 ``asset_root()`` 是仓库根，那份副本在 ``data/assets/plotly/``
    （构建脚本生成，可删可重建）。两个都列出来，报先命中的那个。
    """
    return [paths.asset_root() / "plotly" / "plotly.min.js",
            paths.repo_root() / "data" / "assets" / "plotly" / "plotly.min.js"]


def _check_plotly(report: _Report) -> None:
    """2. ``plotly.min.js`` 能不能读到。

    运行时壳页靠 ``plotly.offline.get_plotlyjs()`` 取 JS（读 plotly 包的
    ``package_data/plotly.min.js``），所以**这一份**才是硬指标；
    随包的 ``assets/plotly/plotly.min.js`` 是显式副本，有就一起报出来。
    """
    from . import plot_view

    candidates = _plotly_asset_candidates()
    asset = next((path for path in candidates if path.is_file()), None)
    if asset is not None:
        asset_note = f"随包副本 {asset.stat().st_size / 1024 / 1024:.2f} MB"
    else:
        asset_note = "随包副本缺失（不影响出图，运行时用的是 plotly 包自带那份）"
    try:
        text = plot_view.plotly_js_text()
    except Exception as error:              # noqa: BLE001 - 报告用
        report.record(False, "2. plotly.min.js 可读", f"{type(error).__name__}: {error}")
        return
    size = len(text.encode("utf-8")) if isinstance(text, str) else len(text)
    if size < 3.5 * 1024 * 1024:
        report.record(False, "2. plotly.min.js 可读",
                      f"只有 {size / 1024:.0f} KB（预期约 4.6 MB）："
                      "plotly 包数据没打全，图会白屏")
        return
    report.record(True, "2. plotly.min.js 可读",
                  f"{size / 1024 / 1024:.2f} MB；{asset_note}")


def _check_models(report: _Report) -> None:
    """3. ``data/configs`` 里能真正加载的机器人模型数。

    除了数个数，还要确认**下拉框里的相对路径喂回 core 之后能定位到文件**。
    冻结后 ``core._REPO_ROOT = Path(__file__).parents[2]`` 指向的是**安装目录**
    （``dist/LaserCalibration``），而模型在 ``_internal/assets`` 下；于是
    ``_resolve_model_path("data/configs/ch2000_origin.urdf")`` 拼出一个不存在的
    路径——现场的表现为"每次生成都失败"，日志里一行 FileNotFoundError。
    这一项就是这个坑的探针（打包时入口脚本里的 _fix_frozen_repo_root() 会补上）。
    """
    from pathlib import Path

    from laser_web import core

    folder = paths.config_dir()
    try:
        robots = core.list_robots(paths.asset_root())
    except Exception as error:              # noqa: BLE001 - 报告用
        report.record(False, "3. 机器人模型", f"{type(error).__name__}: {error}")
        return
    usable = [item for item in robots if item.get("usable")]
    if not usable:
        report.record(False, "3. 机器人模型",
                      f"0 个可加载（目录 {folder}，共 {len(robots)} 个文件）")
        return

    # 相对路径 -> 绝对路径的解析（下拉框交给 core 的正是这个相对路径）。
    resolve = getattr(core, "_resolve_model_path", None)
    if resolve is not None:
        try:
            resolved = str(resolve(usable[0]["path"]))
        except Exception as error:          # noqa: BLE001 - 报告用
            report.record(False, "3. 机器人模型",
                          f"路径解析抛异常：{type(error).__name__}: {error}")
            return
        if not Path(resolved).is_file():
            report.record(
                False, "3. 机器人模型",
                f"{len(usable)} 个模型能读出，但相对路径解析到了不存在的文件："
                f"{resolved}（冻结后 core._REPO_ROOT 指向安装目录，需要入口脚本"
                f"里的 _fix_frozen_repo_root）")
            return

    names = "、".join(item["label"] for item in usable[:4])
    report.record(True, "3. 机器人模型",
                  f"{len(usable)} 个可加载（共 {len(robots)} 个文件）：{names}"
                  + ("…" if len(usable) > 4 else ""))


def _check_writable(report: _Report) -> None:
    """4. ``data_root()`` 能不能写。

    装的目录只读没问题（模型本来就是只读资产），但 ``runs/`` 与缓存必须能写；
    写不了就是"解算完什么都没存下来"那种最气人的故障。
    """
    try:
        paths.ensure_writable_roots()
        probe = paths.data_root() / ".self-check-probe"
        probe.write_text("ok", encoding="utf-8")
        content = probe.read_text(encoding="utf-8")
        probe.unlink()
    except OSError as error:
        report.record(False, "4. data_root() 可写", f"{type(error).__name__}: {error}")
        return
    report.record(content == "ok", "4. data_root() 可写",
                  f"{paths.data_root()}（runs={paths.runs_root()}，"
                  f"cache={paths.cache_root()}）")


def _check_plot_view(report: _Report, app) -> None:
    """5. 真起一个 ``PlotView``：8 个点 → 壳页 → 读回 ``window.__state()``。

    这一项同时验证：QtWebEngine 的 Python 绑定、孑进程（QtWebEngineProcess）、
    离线 ``plotly.min.js``、壳页 HTML、``runJavaScript`` 通路。任何一环断掉，
    现场的表现都是"图区空白"，而这里会指名道姓。
    """
    from laser_web import figures
    from .plot_view import PlotView

    view = None
    try:
        rows = [{"gp": index,
                 "pos": [200.0 + 40.0 * index, -150.0 + 25.0 * index,
                         600.0 + 15.0 * (index % 3)],
                 "j3_abs": 60.0 + index, "j5_abs": 55.0 + index, "ok": True}
                for index in range(8)]
        figure = figures.waypoint_figure(rows)
        view = PlotView()
        view.resize(720, 480)
        view.load_shell()
        if not _wait_until(app, lambda: view.loaded, SELF_CHECK_LOAD_TIMEOUT):
            report.record(False, "5. 图视图（8 个点）",
                          "壳页加载超时/失败：" + str(view.last_error or "loadFinished 没来"))
            return
        view.set_figure(figure)
        _pump(app, 1.0)

        box = {}
        script = "window.__state ? window.__state() : 'NO-STATE-FN'"
        view.page().runJavaScript(script, 0, lambda value: box.setdefault("state", value))
        if not _wait_until(app, lambda: "state" in box, SELF_CHECK_JS_TIMEOUT):
            report.record(False, "5. 图视图（8 个点）", "回读 window.__state() 超时")
            return
        state = str(box.get("state"))
        if state == "points=8":
            report.record(True, "5. 图视图（8 个点）",
                          f"window.__state() -> {state}；壳页加载 {view.load_count} 次")
            return
        report.record(False, "5. 图视图（8 个点）",
                      f"window.__state() -> {state!r}（预期 'points=8'）"
                      + (f"；{view.last_error}" if view.last_error else ""))
    except Exception as error:              # noqa: BLE001 - 报告用：这一项不能把自检带崩
        report.record(False, "5. 图视图（8 个点）", f"{type(error).__name__}: {error}")
    finally:
        if view is not None:
            try:
                view.shutdown()
            except Exception:               # noqa: BLE001 - 拆不掉也不能崩
                pass


def _check_hwid(report: _Report) -> None:
    """6. 能不能读到物理网卡（离线授权按 MAC 指纹绑定）。"""
    try:
        from licensing.hwid import physical_macs
    except Exception as error:              # noqa: BLE001 - 报告用
        report.record(False, "6. hwid（物理网卡）", f"导入失败：{type(error).__name__}: {error}")
        return
    try:
        macs = physical_macs()
    except Exception as error:              # noqa: BLE001 - 枚举硬件不该让自检崩
        report.record(False, "6. hwid（物理网卡）", f"{type(error).__name__}: {error}")
        return
    if not macs:
        report.record(False, "6. hwid（物理网卡）",
                      "读不到物理网卡（虚拟机？网卡被禁用？）——授权会绑定失败")
        return
    report.record(True, "6. hwid（物理网卡）", f"{len(macs)} 个：{', '.join(macs[:3])}")


def _check_license(report: _Report) -> None:
    """7. 许可证本身（签名 / 有效期 / 机器绑定）。

    **这一项不因为"没有许可证"而判失败**，两个理由都成立：

    * 构建机与开发机上本来就不该有绑定到自己机器的授权；判失败会让每次干净
      构建、每台开发机都红一条，结果是所有人都学会忽略这一项；
    * ``packaging/verify_dist.py`` 会在构建机上跑 ``--self-check`` 并要求退出码
      为 0——把"没证"做成 FAIL，等于把安装包的构建卡死在一个装完才有的东西上。

    真正该报 FAIL 的是"证就在那儿但通不过"：签名不对、过期、绑到别的机器。
    那三种正是发错版本、过期未续、装错机器的现场症状，值得为它红一次。
    """
    if licensing.skip_active():
        report.record_skip("7. 许可证",
                           "已按 %s 跳过（仅源码运行有效，冻结包不认）"
                           % licensing.ENV_SKIP)
        return

    result = licensing.check()
    if result.ok:
        report.record(True, "7. 许可证", result.message)
        return
    if licensing.find_license_file() is None:
        if paths.is_frozen():
            report.record_skip(
                "7. 许可证",
                "本机没有许可证（构建机上正常；客户机必须放一份，见 --print-paths）")
        else:
            report.record_skip("7. 许可证", "源码运行，未提供许可证")
        return
    report.record(False, "7. 许可证", result.message)


def self_check() -> int:
    """``--self-check`` 的实现：不显示窗口，逐项打印，任一 FAIL 返回 1。"""
    configure_webengine_environment()      # 必须在建 QApplication 之前

    report = _Report()
    report.emit("桌面版启动自检（不显示主窗口）")
    for name, value in paths.describe().items():
        report.emit(f"  {name:8s} {value}")
    report.emit("")
    # 注意：这里**不**调 ensure_writable_roots()。可写性本身就是第 4 项，
    # 在检查之前建目录失败的话，现场只会看到一段 traceback 而不是 [FAIL]。

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    try:
        apply_chinese_font(app)
    except Exception:                       # noqa: BLE001 - 字体不影响自检
        pass

    _check_binding(report)
    _check_plotly(report)
    _check_models(report)
    _check_writable(report)
    _check_plot_view(report, app)
    _check_hwid(report)
    _check_license(report)

    report.emit("")
    if report.failures:
        report.emit(f"结果：{len(report.failures)} 项 FAIL / 共 {len(report.results)} 项")
        for _ok, title, detail in report.failures:
            report.emit(f"  - {title}：{detail}")
        code = 1
    else:
        passed = len(report.results) - len(report.skipped)
        if report.skipped:
            report.emit(f"结果：{passed} 项通过，{len(report.skipped)} 项跳过")
            for _ok, title, detail in report.skipped:
                report.emit(f"  - {title}：{detail}")
        else:
            report.emit(f"结果：全部 {passed} 项通过")
        code = 0
    # 日志落在哪：冻结时放用户数据目录（现场最容易找到），源码运行时放 .cache\
    # ——不能往仓库根扔日志，那会让 git status 脏掉。
    log_dir = paths.data_root() if paths.is_frozen() else paths.cache_root()
    report.save(log_dir)
    report.emit(f"（日志已写到 {log_dir / 'self-check.log'}）")
    return code


if __name__ == "__main__":
    sys.exit(main())
