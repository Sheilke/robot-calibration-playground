"""离屏冒烟：验证桌面窗口能建、壳页能加载、图真进了 WebEngine，并截图。

为什么它不在 pytest 里
--------------------
它要真的建 ``QWebEngineView`` 并等 Chromium 渲染，在没有显示器的 CI 上偶发超时；
而"窗口能不能起"这件事本来更适合人工/发布前跑一次。单元测试只覆盖不依赖显示器
的部分（``tests/test_paths.py``、``tests/test_plot_view.py``）。

跑法（Linux 无显示器；Windows 直接跑不加 QT_QPA_PLATFORM）::

    PYTHONPATH=python QT_QPA_PLATFORM=offscreen python3 -m laser_desktop.smoke_offscreen

退出码：0 全部通过；非 0 见打印的失败项。截图写到 ``/tmp/desktop_s1.png``。
"""
import sys
from laser_desktop.qt_compat import configure_webengine_environment, QtWidgets, QtCore, BINDING
configure_webengine_environment()
app = QtWidgets.QApplication(sys.argv)
from laser_desktop.main_window import MainWindow, build_preview_figure
from laser_desktop import paths

window = MainWindow()
window.resize(1180, 780)
window.show()

state = {"steps": []}
def step(name, ok, detail=""):
    state["steps"].append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

step("窗口构建", window.isVisible() is not None, f"绑定 {BINDING}")
step("四页注册", window.stack.count() == 4, f"count={window.stack.count()}")
step("导航项", window.nav.count() == 4, "、".join(window.nav.item(i).text() for i in range(window.nav.count())))
step("可写目录", (paths.cache_root() / "plot" / "plotly.min.js").is_file(),
     f"{paths.cache_root()}", )

def after_load(ok):
    step("壳页加载", ok)
    if not ok:
        app.exit(2); return
    window.plot.set_figure(build_preview_figure())
    QtCore.QTimer.singleShot(2500, check_state)

def check_state():
    window.plot.page().runJavaScript("window.__state()", 0, on_state)

def on_state(result):
    step("图已进 WebEngine", result == "points=50", str(result))
    QtCore.QTimer.singleShot(600, grab)

def grab():
    ok = window.grab().save("/tmp/desktop_s1.png")
    import os
    size = os.path.getsize("/tmp/desktop_s1.png") if os.path.exists("/tmp/desktop_s1.png") else 0
    step("截图", bool(ok) and size > 5000, f"{size} 字节 → /tmp/desktop_s1.png")
    print("\n=== 结论 ===")
    bad = [n for n, o, _ in state["steps"] if not o]
    print("  全部通过" if not bad else "  未通过：" + "、".join(bad))
    app.exit(0 if not bad else 3)

window.plot.loadFinished.connect(after_load)
QtCore.QTimer.singleShot(60000, lambda: (print("  超时"), app.exit(4)))
app.exec()
