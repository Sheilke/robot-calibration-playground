#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给《操作说明书》的「桌面版」一章生成截图（**真跑一遍，不摆样子**）。

    QT_QPA_PLATFORM=offscreen PYTHONPATH=python \\
        .venv/bin/python scripts/make_desktop_shots.py

产出（覆盖写）：

    docs/images/desktop/generate.png   生成页：真跑一次 50 点解算，停在路点表
    docs/images/desktop/repair.png     修复页：真跑一次「定位段」，停在定位页签
    docs/images/desktop/iso.png        ISO 页：真跑一次 200 mm 立方体的五点
    docs/images/desktop/tools.png      工具页：运行目录列表里是上面这几次运行

为什么单独一个脚本
------------------
截图必须来自**当前代码**，而且要能重来一次：手点出来的图过两周就对不上代码了。
脚本做的每一步都是界面上的真实操作（填表单 → 点按钮 → 等任务结束 → 切页签），
没有任何"构造一份假结果塞进表格"的分支。

数据目录默认落在 ``/tmp/laser-desktop-shots``（可用 ``LASER_HOME`` 覆盖）：
截图过程会产生 3 个运行目录，不往仓库 ``runs/`` 里堆。

Linux 上没有 ``xvfb`` 也能出图：``QT_QPA_PLATFORM=offscreen`` 下
``QWidget.grab()`` 会把 Plotly 的 WebGL 画布一起抓下来（见 ``smoke_offscreen``
的实测结论）。Windows 上直接跑这个脚本也行，只是路径不同。

退出码：0 四张都写出来了；1 任何一步失败（打印失败在哪一步）。
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PYTHON_SRC = REPO / "python"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

#: 截图落盘目录（相对仓库根）。
SHOT_DIR = REPO / "docs" / "images" / "desktop"

#: 窗口尺寸：与 ``page_layout.DEFAULT_WINDOW_SIZE`` 同一档（三段都舒展开）。
WINDOW = (1600, 950)

#: 每一步的等待上限（秒）：解算/定位/ISO 都在 1 s 量级，给足余量。
RUN_TIMEOUT = 180

#: 出结果后等 WebEngine 把图画完的时间（秒）。``Plotly.react`` 是异步的，
#: 抓早了图区是白的（smoke 里实测 0.8 s 够，这里留 1.5 s）。
PLOT_SETTLE = 1.5

#: 渲染进程的默认数据目录（不污染仓库 runs/）。
os.environ.setdefault("LASER_HOME", "/tmp/laser-desktop-shots")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop.qt_compat import QtCore, QtWidgets, configure_webengine_environment  # noqa: E402

configure_webengine_environment()          # 必须在建 QApplication 之前

_APP = QtWidgets.QApplication([])

from laser_desktop.main_window import MainWindow             # noqa: E402


def pump(seconds: float) -> None:
    """跑一小段事件循环（``QTimer`` 到点才触发，光 sleep 不够）。"""
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        _APP.processEvents()
        time.sleep(0.01)


def wait_until(predicate, timeout: float, label: str) -> bool:
    """等 ``predicate()`` 变真（期间照常处理 Qt 事件）。

    Returns:
        bool: 等到为 ``True``（超时打印一行并返回 ``False``）。
    """
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        if predicate():
            return True
        _APP.processEvents()
        time.sleep(0.02)
    print(f"  [超时] {label}（{timeout:.0f} s）")
    return False


def settle_plot(window, extra: float = PLOT_SETTLE) -> None:
    """等 WebEngine 把图真画出来（否则图区是一片白）。

    两道门：壳页 ``loadFinished``（``PlotHost.loaded``）与壳页里的 WebGL 自检
    （``webgl_ok`` 由 ``None`` 变成 ``True``/``False``）——后者要等 Chromium
    起来、JS 跑完，首张图实测 3–5 s。状态栏那句“WebGL：检查中”就是这两道门
    都没过的样子。

    Args:
        window: 主窗口（要它那个共享的 ``PlotHost``）。
        extra: 自检通过后再等多久（Plotly.react 本身是异步的）。
    """
    wait_until(lambda: bool(window.plot.loaded), 30.0, "图控件壳页加载")
    wait_until(lambda: window.plot.webgl_ok is not None, 30.0, "WebGL 自检")
    pump(extra)


def grab(window, name: str) -> Path:
    """等图渲染好，再把主窗口存成 PNG（顺手报体积）。"""
    settle_plot(window)
    # 留在原地的控件不动焦点：一旦把焦点挪到导航（或 activateWindow），
    # Fusion 风格会把整个表单换成非活动调色板（浅灰底），截图就不像现场屏幕
    # 看到的白底了。导航的选中底色由各页自己切页时重画（见 shot_generate）。
    pump(0.15)
    SHOT_DIR.mkdir(parents=True, exist_ok=True)
    path = SHOT_DIR / name
    if not window.grab().save(str(path)):
        raise SystemExit(f"截图保存失败：{path}")
    print(f"  截图 {path.relative_to(REPO)}  {path.stat().st_size / 1024:.0f} KB")
    return path


# ---------------------------------------------------------------------------
# 四个页面
# ---------------------------------------------------------------------------
def shot_generate(window) -> None:
    """生成页：表单默认值（站点 50 点）→ 生成并解算 → 停在「路点」表。"""
    print("· 生成页")
    # 先切走再切回：启动时就在第 0 行，直接 setCurrentRow(0) 不会触发选中重绘，
    # 截图上第一项没底色，看不出这是当前页（人为切一次就有）。
    window.nav.setCurrentRow(1)
    pump(0.1)
    window.nav.setCurrentRow(0)
    page = window.generate
    pump(0.3)
    print(f"  表单：{page.total.text}")
    job_id = page.start_scan()
    if not job_id:
        raise SystemExit("  起解算任务失败")
    if not wait_until(lambda: page.run_state in ("done", "failed", "cancelled"),
                      RUN_TIMEOUT, "解算跑完"):
        raise SystemExit("  解算超时")
    if page.run_state != "done":
        raise SystemExit(f"  解算没有成功：{page.run_state}")
    page.result_tabs.setCurrentWidget(page.waypoints_page)
    pump(PLOT_SETTLE)
    stats = (page.last_result or {}).get("stats") or {}
    print(f"  结果：成功 {stats.get('solved')}/{stats.get('total')}，"
          f"可行={stats.get('feasible')}")
    grab(window, "generate.png")


def shot_repair(window) -> None:
    """修复页：站点 50 点 + 一组退化读数 → 定位段 → 停在「定位」页签。"""
    print("· 修复页")
    window.nav.setCurrentRow(1)
    page = window.repair
    pump(0.3)
    fields = page.fields
    # 现场读数：J5 = 0 的退化停机（J4/J6 共线），与冒烟脚本用的是同一组。
    for joint, value in enumerate((0.0, 0.0, 0.0, 55.72, 0.0, -55.72)):
        page.q_rows[0]["fields"][joint].setValue(value)
    fields["stopped_at"].setValue(41)
    pump(0.2)
    result = page.locate()                  # 同步调用，实测 0.3–0.5 s
    if not result or not result.get("ok"):
        raise SystemExit(f"  定位段失败：{(result or {}).get('error')}")
    page.result_tabs.setCurrentWidget(page.locate_page)
    # 像用户拖分割条一样把结果区拉高：定位页签里除了徒标还有一张段表，
    # 默认比例下只能露到表头。用 _apply_sizes（不走用户拖动那条路），
    # 所以不会把比例写进 QSettings。
    page._viewer_splitter._apply_sizes([110, 200, 620])
    pump(0.3)
    print(f"  判定：{'、'.join(badge.text() for badge in page.kind_badges)}；"
          f"段定位 {page.profile_table.rowCount()} 行")
    grab(window, "repair.png")


def shot_iso(window) -> None:
    """ISO 页：站点位置 200 mm 立方体 → 生成五点 → 停在「五点」表。"""
    print("· ISO 页")
    window.nav.setCurrentRow(2)
    page = window.iso
    pump(0.3)
    fields = page.fields
    fields["cube_bounds"].setChecked(True)
    for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                       ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
        fields[key].setValue(value)
    page.refresh_preview()
    pump(0.2)
    job_id = page.start_iso()
    if not job_id:
        raise SystemExit("  起 ISO 任务失败")
    if not wait_until(lambda: page.run_state in ("done", "failed", "cancelled"),
                      RUN_TIMEOUT, "ISO 五点跑完"):
        raise SystemExit("  ISO 超时")
    if page.run_state != "done":
        raise SystemExit(f"  ISO 没有成功：{page.run_state}")
    page.result_tabs.setCurrentWidget(page.points_page)
    pump(PLOT_SETTLE)
    stats = (page.last_result or {}).get("stats") or {}
    print(f"  结果：可达 {stats.get('solved')}/{stats.get('total')}，"
          f"边长 {stats.get('cube_side')} mm")
    grab(window, "iso.png")


def shot_tools(window) -> None:
    """工具页：刷新运行目录（列表里是刚才那几次真运行）。"""
    print("· 工具页")
    window.nav.setCurrentRow(3)
    page = window.tools
    pump(0.3)
    page.refresh_runs()
    pump(0.5)
    page.scroll.verticalScrollBar().setValue(0)
    pump(0.3)
    print(f"  运行目录 {len(page.runs)} 个：{page.runs_count_label.text()}")
    grab(window, "tools.png")


def main() -> int:
    """建窗口 → 四页各跑一次 → 退出。"""
    print(f"窗口 {WINDOW[0]}×{WINDOW[1]}，数据目录 {os.environ['LASER_HOME']}")
    window = MainWindow()
    window.resize(*WINDOW)
    window.show()
    pump(0.5)
    # 每次截图都要可复现：分割条回默认比例（用户拖过的比例存在 QSettings 里，
    # 上一次拖窄了这次就不一样了）。三页各有一个竖分割器。
    for page in (window.generate, window.repair, window.iso):
        for splitter in (page._viewer_splitter, page._body_splitter):
            splitter.reset_ratios()
    pump(0.3)

    shot_generate(window)
    shot_repair(window)
    shot_iso(window)
    shot_tools(window)

    print("完成：四张都在 " + str(SHOT_DIR.relative_to(REPO)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
