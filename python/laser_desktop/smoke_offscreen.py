"""离屏冒烟：验证桌面窗口能建、壳页能加载、图真进了 WebEngine，并截图。

为什么它不在 pytest 里
--------------------
它要真的建 ``QWebEngineView`` 并等 Chromium 渲染，在没有显示器的 CI 上偶发超时；
而"窗口能不能起"这件事本来更适合人工/发布前跑一次。单元测试只覆盖不依赖显示器
的部分（``tests/test_paths.py``、``tests/test_plot_view.py``、
``tests/test_generate_page.py``）。

S2 起额外检查：生成页的预览卡片/网格组里**真的有总点数**（不是空标签），
截图存到 ``/tmp/desktop_s2.png``（只校验体积 > 5 KB，不做 OCR）。

跑法（Linux 无显示器；Windows 直接跑不加 QT_QPA_PLATFORM）::

    PYTHONPATH=python QT_QPA_PLATFORM=offscreen python3 -m laser_desktop.smoke_offscreen

退出码：0 全部通过；非 0 见打印的失败项。
"""
import os
import sys

from laser_desktop.qt_compat import configure_webengine_environment, QtWidgets, QtCore, BINDING
configure_webengine_environment()
app = QtWidgets.QApplication(sys.argv)
from laser_desktop.main_window import MainWindow, build_preview_figure
from laser_desktop import paths

#: 截图落盘位置（每次覆盖）。
SHOT = "/tmp/desktop_s2.png"

window = MainWindow()
window.resize(1180, 780)
window.show()

state = {"steps": []}
def step(name, ok, detail=""):
    state["steps"].append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

# 真检查，不是 ``is not None``（后者对 bool 永远成立）：offscreen 下 ``show()``
# 之后 ``isVisible()`` 就是 ``True``，同时确认 ``centralWidget()`` 真的挂上了。
step("窗口构建",
     window.isVisible() is True and window.centralWidget() is not None,
     f"visible={window.isVisible()} "
     f"central={type(window.centralWidget()).__name__} 绑定 {BINDING}")
step("四页注册", window.stack.count() == 4, f"count={window.stack.count()}")
step("导航项", window.nav.count() == 4, "、".join(window.nav.item(i).text() for i in range(window.nav.count())))
step("可写目录", (paths.cache_root() / "plot" / "plotly.min.js").is_file(),
     f"{paths.cache_root()}", )

page = window.generate
points_text = page.metric_labels["points"].text()
step("预览卡片有总点数", "点" in points_text and page.total.count == 50,
     f"卡片={points_text!r} 网格组={page.total_value.text()!r} "
     f"格点={page.metric_labels['grid'].text()!r}")
step("预览卡片体积/条件数", "m³" in page.metric_labels["volume"].text()
     and page.metric_labels["design"].text() != "—",
     f"体积={page.metric_labels['volume'].text()!r} "
     f"条件数={page.metric_labels['design'].text()!r}")
step("生成按钮可用（参数合法）", page.generate_button.isEnabled(),
     f"enable={page.generate_button.isEnabled()}")
step("状态栏含绑定/数据目录", BINDING in window.statusBar().currentMessage()
     and str(paths.data_root()) in window.statusBar().currentMessage(),
     window.statusBar().currentMessage()[:80])

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
    ok = window.grab().save(SHOT)
    size = os.path.getsize(SHOT) if os.path.exists(SHOT) else 0
    # 预览卡片里的总点数已经在上面那条断言里验过文字了；这里只要求截图像素够
    # （卡片真的画出来了），不做 OCR
    step("截图（含预览卡片总点数）", bool(ok) and size > 5000,
         f"{size} 字节 > 5000 → {SHOT}")
    print("\n=== 结论 ===")
    bad = [n for n, o, _ in state["steps"] if not o]
    print("  全部通过" if not bad else "  未通过：" + "、".join(bad))
    app.exit(0 if not bad else 3)

window.plot.loadFinished.connect(after_load)
QtCore.QTimer.singleShot(60000, lambda: (print("  超时"), app.exit(4)))
app.exec()
