"""在 Windows 目标机上跑一遍，回答打包前必须先知道的几件事。

用法（把整个仓库拷到目标机，或用 U 盘带这一个文件 + 已建好的 venv 路径）::

    python scripts\\check_windows_env.py

它**只读项目文件**，不改任何配置与数据；唯一会写盘的是第 4 项的可写性实测：
在 ``%LOCALAPPDATA%`` （或系统临时目录）下建一个探针目录 ``laser-desktop-probe``，
写一个临时文件再立即删掉（目录也删），不碰其他任何路径。输出可以直接贴回来。

为什么需要它
------------
桌面端用 PySide6 + ``QWebEngineView`` 显示 Plotly 图。PySide6 把 Qt 拆成了
Addons / Essentials 两个包，**QtWebEngine 只可能在 Addons 里**，而不同版本、
不同安装方式（pip / conda / 离线 whl）带的东西并不一样。这一条决定了打包
方案：Addons 里有 → PyInstaller 直接捆；没有 → 得另找 QtWebEngine 来源，
整个 S7 的时间估算要重算。

顺带确认几个会咬人的环境事实：

* 显卡与 ANGLE 后端 —— ``QWebEngineView`` 没有 WebGL 时 Plotly 的 3D 图
  是空白，而不是报错（我们在 Linux 上踩过：``--disable-gpu`` 会直接关掉
  WebGL，而界面看上去"正常"）。
* 长路径 / 中文路径 / 目录可写 —— 安装到 ``%LOCALAPPDATA%`` 还是
  ``Program Files`` 会影响首次启动建目录的权限（可写性靠一个用完就删的探针
  目录实测，见上）。
* 高 DPI —— Qt6 默认按缩放比渲染，截图里的字号和列宽会和这里不一样。
* 绘图依赖 —— ``iso.png`` 是 matplotlib（Agg 后端）画的，它不在 requirements
  的必装项里：缺了不影响主流程，但「ISO 五点」的导出会少一张图。
"""

from __future__ import annotations

import json
import os
import platform
import struct
import sys
import tempfile
from pathlib import Path


def _line(title: str) -> None:
    print()
    print("=" * 68)
    print(title)
    print("=" * 68)


def check_python() -> dict:
    _line("1. Python 与解释器")
    info = {
        "version": platform.python_version(),
        "implementation": platform.python_implementation(),
        "bits": struct.calcsize("P") * 8,
        "executable": sys.executable,
        "frozen": bool(getattr(sys, "frozen", False)),
    }
    print(f"Python      : {info['version']} ({info['implementation']}, {info['bits']} bit)")
    print(f"解释器       : {info['executable']}")
    print(f"已打包运行?  : {info['frozen']}")
    if info["bits"] != 64:
        print("  !! 32 位解释器：QtWebEngine 只提供 64 位，请换 64 位 Python")
    if tuple(int(p) for p in platform.python_version().split(".")[:2]) < (3, 10):
        print("  !! 低于 3.10：本项目的类型标注语法需要 3.10+")
    return info


def check_pyside() -> dict:
    _line("2. PySide6 与 QtWebEngine（决定打包路线）")
    info: dict = {"pyside6": None, "essentials": None, "addons": None,
                  "qtwebenginewidgets": False, "qtwebenginecore": False,
                  "error": None}
    try:
        import PySide6
        info["pyside6"] = PySide6.__version__
        print(f"PySide6     : {PySide6.__version__}")
    except Exception as exc:                      # noqa: BLE001 - 报告用
        info["error"] = f"import PySide6 失败: {exc}"
        print(f"PySide6     : 未安装（{exc}）")
        return info

    for name in ("PySide6-Essentials", "PySide6-Addons"):
        try:
            from importlib.metadata import version
            info[name.split("-")[1].lower()] = version(name)
            print(f"{name:<20}: {version(name)}")
        except Exception:                          # noqa: BLE001 - 报告用
            print(f"{name:<20}: 未安装（元数据里没有）")

    try:
        from PySide6.QtWebEngineWidgets import QWebEngineView  # noqa: F401
        info["qtwebenginewidgets"] = True
        print("QtWebEngineWidgets : OK  <-- 打包可以直接捆 Addons")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["error"] = f"import QWebEngineView 失败: {exc}"
        print(f"QtWebEngineWidgets : 不可用（{exc}）")
        print("  !! 这一条为 False 时，PyInstaller 打出来的 exe 里没有 WebEngine，")
        print("     3D 图会是空白。需要装 PySide6-Addons 或改用 PyQt5 + QtWebEngine。")

    try:
        from PySide6.QtWebEngineCore import QWebEngineSettings  # noqa: F401
        info["qtwebenginecore"] = True
        print("QtWebEngineCore    : OK")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        print(f"QtWebEngineCore    : 不可用（{exc}）")
    return info


def check_webgl() -> dict:
    """真的起一个 QWebEngineView，读 WebGL 的 renderer 字符串。"""
    _line("3. WebGL 实测（Plotly 3D 依赖它）")
    info: dict = {"ok": False, "renderer": None, "error": None}
    try:
        from PySide6.QtWebEngineWidgets import QWebEngineView
        from PySide6.QtWidgets import QApplication
        from PySide6.QtCore import QTimer
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["error"] = str(exc)
        print(f"跳过：{exc}")
        return info

    app = QApplication.instance() or QApplication(sys.argv[:1])
    view = QWebEngineView()
    view.resize(640, 480)
    view.setHtml(
        "<canvas id='c'></canvas><script>"
        "var c=document.getElementById('c');"
        "var gl=c.getContext('webgl')||c.getContext('experimental-webgl');"
        "window.__r='';"
        "if(gl){var d=gl.getExtension('WEBGL_debug_renderer_info');"
        "window.__r=gl.getParameter(d?d.UNMASKED_RENDERER_WEBGL:gl.RENDERER);}"
        "</script>")
    view.show()
    result: dict = {}

    def read_back() -> None:
        def done(value) -> None:
            result["renderer"] = value
        view.page().runJavaScript("window.__r", done)

    QTimer.singleShot(2500, read_back)

    def finish() -> None:
        app.quit()

    QTimer.singleShot(3500, finish)
    app.exec()

    renderer = result.get("renderer") or ""
    info["renderer"] = renderer or None
    info["ok"] = bool(renderer)
    if renderer:
        print(f"WebGL renderer : {renderer}")
        if "SwiftShader" in renderer or "Software" in renderer:
            print("  注意：软件渲染。能用，但 3000 点以上的散点图会明显拖慢。")
    else:
        print("WebGL renderer : 空 <-- 3D 图会白屏，必须在打包里解决")
    return info


def check_plotting() -> dict:
    """绘图依赖：``iso.png`` 走 matplotlib（Agg 后端）+ ``plot_points`` 的惰性 import。

    ``matplotlib`` 不在必装项里（见 ``requirements*.txt``），所以缺了也不能让
    ``laser_web`` / 桌面端起不来：两边都是**函数内 import**。这里就按同一路径
    验一遍：能 import → ``iso.png`` 有；不能 → 只是少一张图。

    ``matplotlib.use("Agg")`` 必须在``import pyplot`` 之前，而一旦别处已经
    import 了 pyplot，``use()`` 可能抛（或静默无效）。所以这条检查只在这里
    做一次，且不依赖 ``DISPLAY``。
    """
    _line("5. 绘图依赖（iso.png / matplotlib Agg）")
    info: dict = {"matplotlib": None, "backend": None, "mplot3d": False,
                  "plot_points": False, "error": None}
    try:
        import matplotlib
        info["matplotlib"] = matplotlib.__version__
        print(f"matplotlib  : {matplotlib.__version__}")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["error"] = f"import matplotlib 失败: {exc}"
        print(f"matplotlib  : 未安装（{exc}）")
        print("  !! 缺它不影响主流程，但「ISO 五点」的 iso.png 会导出失败。")
    if info["matplotlib"]:
        try:
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt
            info["backend"] = str(matplotlib.get_backend())
            print(f"后端        : {info['backend']}")
            plt.close("all")
            if info["backend"].lower() != "agg":
                print("  !! 后端不是 Agg：无显示环境（服务/远程）下会报错或卡住。")
            # ``projection="3d"`` 需要 ``mpl_toolkits.mplot3d`` 注册；有些发行版的
            # matplotlib 被拆包（二维能用、三维报错），这一句能抓到。图只在内存里，
            # 不落盘。
            figure = plt.figure()
            figure.add_subplot(111, projection="3d")
            plt.close(figure)
            info["mplot3d"] = True
            print("3D 子图     : OK（mpl_toolkits.mplot3d 就位）")
        except Exception as exc:                   # noqa: BLE001 - 报告用
            info["error"] = f"matplotlib Agg 不可用: {exc}"
            print(f"后端        : 不可用（{exc}）")

    # ``iso9283_cube.plot_points`` 自己才 import matplotlib / pyplot（惰性）：
    # 这里只查它能不能被 import、以及它声明的依赖是不是真的能装上。
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from excitation_trajectory import iso9283_cube
        info["plot_points"] = callable(getattr(iso9283_cube, "plot_points", None))
        print(f"plot_points : {'OK' if info['plot_points'] else '缺'}")
        if info["plot_points"] and info["matplotlib"]:
            import matplotlib.pyplot as plt                # noqa: F401 - 可用性实测
            print("  → iso.png 能画（lazy import 就位）")
        elif info["plot_points"]:
            print("  !! plot_points 在，但 matplotlib 缺 → iso.png 会失败。")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["error"] = info["error"] or f"import iso9283_cube 失败: {exc}"
        print(f"plot_points : 不可用（{exc}）")
    return info


def check_paths() -> dict:
    _line("4. 目录与路径")
    local = os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()
    info = {"localappdata": local, "writable": None, "cwd": os.getcwd(),
            "non_ascii": not os.getcwd().isascii(), "long": len(os.getcwd()) > 120}
    print(f"LOCALAPPDATA : {local}")
    # 可写性实测：建一个探针目录 → 写一个文件 → 删文件 → 删目录。
    # 这是本脚本**唯一**会被动盘的地方（其余均只读），已写进 docstring。
    try:
        probe = Path(local) / "laser-desktop-probe"
        probe.mkdir(parents=True, exist_ok=True)
        (probe / "t.txt").write_text("ok", encoding="utf-8")
        (probe / "t.txt").unlink()
        probe.rmdir()
        info["writable"] = True
        print("可写        : 是")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["writable"] = False
        print(f"可写        : 否（{exc}）  !! 程序需要在这里存 runs/ 与日志")
    print(f"当前目录     : {os.getcwd()}")
    if info["non_ascii"]:
        print("  注意：路径含非 ASCII 字符，打包与首次启动都要验一遍。")
    if info["long"]:
        print("  注意：路径接近/超过 120 字符，Windows 长路径可能出问题。")
    return info


def check_project() -> dict:
    """从仓库里跑时，顺手确认数据文件在。"""
    _line("6. 项目数据（不在仓库里跑就跳过）")
    root = Path(__file__).resolve().parents[1]
    info: dict = {}
    # GP 模板是公司资料，不入库：运行时由用户在「工具 → GP 导出」上传，
    # 所以这里只查模型与代码，不把模板算作缺失。
    for rel in ("data/configs/ch2000_origin.urdf",
                "data/configs/hyundai_general6s.urdf",
                "python/laser_web/core.py",
                "python/laser_desktop/main_window.py"):
        path = root / rel
        info[rel] = path.is_file()
        print(f"{'OK ' if path.is_file() else '缺失'} {rel}")
    print("GP 模板     : 不入库，运行时上传（工具页「GP 导出」）")
    try:
        sys.path.insert(0, str(root / "python"))
        from laser_web import core                       # noqa: F401
        robots = core.list_robots()
        usable = [r for r in robots if r.get("usable")]
        info["usable_robots"] = len(usable)
        print(f"可用机器人模型 : {len(usable)} / {len(robots)}")
    except Exception as exc:                       # noqa: BLE001 - 报告用
        info["core_error"] = str(exc)
        print(f"core 导入失败：{exc}")
    return info


def main() -> int:
    print("laser-desktop 打包前环境检查")
    print(f"平台: {platform.platform()}")
    report = {
        "python": check_python(),
        "pyside": check_pyside(),
        "webgl": check_webgl(),
        "paths": check_paths(),
        "plotting": check_plotting(),
        "project": check_project(),
    }
    _line("结论")
    pyside = report["pyside"]
    webgl = report["webgl"]
    plotting = report["plotting"]
    if pyside["qtwebenginewidgets"] and webgl["ok"]:
        print("可以按计划打包：QtWebEngine 在，WebGL 能跑。")
        verdict = 0
    elif pyside["qtwebenginewidgets"]:
        print("QtWebEngine 在，但 WebGL 起不来 —— 先解决显卡/ANGLE，再打包。")
        verdict = 1
    else:
        print("QtWebEngine 不在 —— 打包路线要改，请把上面输出贴回来。")
        verdict = 1
    if not (plotting["plot_points"] and plotting["mplot3d"]
            and str(plotting["backend"] or "").lower() == "agg"):
        print("注意：绘图依赖不齐 —— 不影响主流程，但 ISO 的 iso.png 导不出来。")
    print()
    print("--- 下面这段贴回来即可 ---")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return verdict


if __name__ == "__main__":
    raise SystemExit(main())
