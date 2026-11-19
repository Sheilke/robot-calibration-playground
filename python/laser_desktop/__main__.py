#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桌面版入口。

    PYTHONPATH=python python3 -m laser_desktop                    # 正常启动
    PYTHONPATH=python python3 -m laser_desktop --print-paths      # 只打印路径解析结果
    PYTHONPATH=python python3 -m laser_desktop --self-check       # 启动自检（不显示窗口）

``--self-check`` 为什么要有
--------------------------
打包后的故障几乎都长成同一副样子：**窗口能开，图区白白的**。没有报错、没有日志，
现场只能说"打不开图"。这个模式把六个环节逐个点名，每项 ``[OK]``/``[FAIL]``，
任一 FAIL 退出码 1：

1. Qt 绑定与版本（交付包必须是 PySide6；PyQt5 是 GPL，只在 Linux 本地验证用）
2. ``plotly.min.js`` 能不能读到（离线画图的前提）
3. ``data/configs`` 里能真正加载的机器人模型数
4. ``data_root()`` 能不能写（装到只读目录的典型症状）
5. **真起一个 PlotView**，画出 8 个点，再从壳页里读回 ``window.__state()``
   （这一项过了，就说明 QtWebEngine、孑进程、plotly.js、通道全都活着）
6. 能不能读到物理网卡（离线授权要绑 MAC）

它**不显示主窗口**，所以 ``QT_QPA_PLATFORM=offscreen`` 下也能跑；
**不加** ``--disable-gpu``（那会把 WebGL 一起关掉，查不出真问题）。
构建脚本 ``packaging/build_windows.ps1`` 与 ``packaging/verify_dist.py`` 都会
调用它——安装包"能装"跟"能出图"是两件事。

注意：``qt_compat`` 必须**在 QApplication 之前**导入（Qt5 的 WebEngine 硬性要求），
所以这里的 import 顺序是刻意的。
"""
from __future__ import annotations

import sys

from . import paths
from .qt_compat import (BINDING, DEV_ONLY_BINDING, QtWidgets,
                        apply_chinese_font, configure_webengine_environment)

#: ``--self-check`` 等壳页加载的上限（秒）。EDR 全盘扫描时 Windows 上首读
#: 203 MB 的 Qt6WebEngineCore.dll 确实会慢，给足余量但不至于无限等。
SELF_CHECK_LOAD_TIMEOUT = 25.0
#: 从壳页回读 ``window.__state()`` 的上限（秒）。
SELF_CHECK_JS_TIMEOUT = 15.0


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)

    if "--print-paths" in args:
        for name, value in paths.describe().items():
            print(f"{name:8s} {value}")
        return 0

    if "--self-check" in args:
        return self_check()

    configure_webengine_environment()      # 必须在建 QApplication 之前
    paths.ensure_writable_roots()

    app = QtWidgets.QApplication(args)
    family = apply_chinese_font(app)

    from .main_window import MainWindow
    window = MainWindow()
    window.show()
    print(f"桌面版启动：Qt 绑定 {BINDING}"
          + ("（仅供本地验证）" if DEV_ONLY_BINDING else "")
          + f"，中文字体 {family}")
    return app.exec()


# ---------------------------------------------------------------------------
# --self-check
# ---------------------------------------------------------------------------
class _Report:
    """自检结果：逐项打印 + 结束时落一份日志（现场求助时直接发这个文件）。"""

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

    @property
    def failures(self) -> list:
        return [item for item in self.results if not item[0]]

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

    report.emit("")
    if report.failures:
        report.emit(f"结果：{len(report.failures)} 项 FAIL / 共 {len(report.results)} 项")
        for _ok, title, detail in report.failures:
            report.emit(f"  - {title}：{detail}")
        code = 1
    else:
        report.emit(f"结果：全部 {len(report.results)} 项通过")
        code = 0
    # 日志落在哪：冻结时放用户数据目录（现场最容易找到），源码运行时放 .cache\
    # ——不能往仓库根扔日志，那会让 git status 脏掉。
    log_dir = paths.data_root() if paths.is_frozen() else paths.cache_root()
    report.save(log_dir)
    report.emit(f"（日志已写到 {log_dir / 'self-check.log'}）")
    return code


if __name__ == "__main__":
    sys.exit(main())
