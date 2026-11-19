"""离屏冒烟：验证桌面窗口能建、壳页能加载、图真进了 WebEngine，并截图。

为什么它不在 pytest 里
--------------------
它要真的建 ``QWebEngineView`` 并等 Chromium 渲染，在没有显示器的 CI 上偶发超时；
而"窗口能不能起"这件事本来更适合人工/发布前跑一次。单元测试只覆盖不依赖显示器
的部分（``tests/test_paths.py``、``tests/test_plot_view.py``、
``tests/test_generate_page.py``、``tests/test_job_bridge.py`` …）。

S2 起额外检查：生成页的预览卡片/网格组里**真的有总点数**（不是空标签）。
S3 起再额外检查：**真跑一次 8 点小案子**（走 ``JobBridge`` → ``core.run_scan``），
断言终态 ``done`` 且 ``solved == 8``、结果表 8 行，并把结果图推给 WebEngine 后
截图存到 ``/tmp/desktop_s3.png``（只校验体积 > 5 KB，不做 OCR）。
S4 起再补一段**修复页**检查：切到「奇异修复」页 → 填 8 点小案子 + 现场退化
读数 → 真跑一次「定位段」（``core.locate_segment``），断言判定徽标与段定位表
有内容、切页（共用同一个 PlotView）耗时远低于 200 ms，并截图存到
``/tmp/desktop_s4.png``。
S5 起最后补两段：

* **ISO 五点页**：切到该页 → 立方体来源两种模式各切 2 次（可见组必须恰好 1 个，
  钉死网页版 32136c6 那类“两组都隐藏”的坑）→ 填 200 mm 立方体（站点位置）→
  真跑一次（``core.run_iso9283``，实测 < 1 s），断言终态 ``done``、路点表 5 行、
  ``|J5|min`` 有值，截图存 ``/tmp/desktop_s5_iso.png``；
* **工具页**：运行表非空、模板文件的摘要块说清楚自己看到的是什么（找到了就
  是 64 位十六进制 sha256；找不到就明说「—」并列出搜索路径 + 环境变量名，
  绝不允许“两眼一抹黑但显示一个 sha256”），截图存 ``/tmp/desktop_s5_tools.png``。

S5 评审后补的一条回归（T1）：**切立方体来源必须同时换预览**——切到「自动搜索」
后预览卡必须是搜索箱（不是上一种来源的边长 550 mm）。

S4 评审后的三条回归（都是“界面状态与盘子里的文件自相矛盾”那类问题）：

* **Q2**：``xs/ys/zs = 200/200/200``（800 万点）时「修复」必须不可点，且
  ``start_repair()`` 必须被硬守卫拦住（不真的去 plan 800 万点）；
* **Q3**：真跑一次修复（原域可行 → ``mode=none``）拿到图之后，再提交一次新任务，
  ``last_figure`` 必须当场清成 ``None``、画布换成「正在修复」说明图；
* **Q1**：那次新任务故意造成“所有候选都不可行”（``j5_min=90`` +
  ``max_candidates=1``）→ 断言 ``last_result`` 非空、``run_label`` 显示真实
  run_id、**只有** repair.json 的导出按钮亮着。

S6c 最后一段（**三页 × 两种窗口尺寸，六张截图**）：主操作按钮必须在固定
操作行里完整可见（``visibleRegion() == rect()``）、1600×950 下预览卡五项数值
不用滚、图 ≥260 / 表 8 行（1280×800 是图 ≥200 / 表 ≥5 行）。截图存
``/tmp/desktop_s6d_<page>_<1600|1280>.png``。

S6e 最后一段（**三页 × 默认/拖动，六张截图**）：左导航 / 表单 / 右栏三列里，
表单与右栏之间现在是可拖的横向分割器（``page_layout.BodySplitter``）。
断言：两段之间真是 ``QSplitter``（水平、手柄 8 px、不可折叠）、默认左栏
≈ 540 px、拖到最左贴住表单的最小宽度且数值框不被压窄、横向拖动不动竖分割
比例。截图存 ``/tmp/desktop_s6e_<page>_<default|dragged>.png``
（``dragged`` 就是左栏拖窄、右栏放宽那一下）。

S6 评审后的三处必改（S6d，全部用真 WebEngine 探针量，不是看代码）：

* **R1**：图上高亮只增不减——``Plotly.restyle`` 就地改写 ``gd.data[t].marker
  .size``，上次的 12 会变成新基准。这里直接读回 ``marker.size`` 数组：选 3 →
  只有第 3 个 12；改选 7 → 只有第 7 个 12；``__select([])`` → 全部回基准 5；
* **R2**：状态栏跨页残留——切页后临时区的原文必须带页面前缀，且常驻信息
  （绑定/数据目录/WebGL）在临时消息 10 s 超时之后仍在（那条消息故意等 12 s 再读）；
* **R3**：``RightSplitter`` 用户拖过之后，``protect_plot`` 不许再把图区抳回
  260（探针：置 ``_user_moved`` → ``setSizes`` → 图区与预览卡保持用户比例）。

“图进了 WebEngine”的判据是 **trace 名 + 标题**，不是点数量：预览图同样是 8 个
点，只看点数量区分不了“结果图被残留的预览去抖盖掉”（S3 评审 P1 就是这么溜过去
的）。结果图 trace 名是 ``成功 8 点 · 按路径顺序``，预览图无名、标题以“路点预览”
开头。

跑法（Linux 无显示器；Windows 直接跑不加 QT_QPA_PLATFORM）::

    PYTHONPATH=python QT_QPA_PLATFORM=offscreen python3 -m laser_desktop.smoke_offscreen

退出码：0 全部通过；非 0 见打印的失败项。
"""
import json
import os
import subprocess
import sys
import tempfile
import time

from laser_desktop.qt_compat import configure_webengine_environment, QtWidgets, QtCore, BINDING
configure_webengine_environment()
app = QtWidgets.QApplication(sys.argv)
from laser_desktop.main_window import MainWindow, build_preview_figure
from laser_desktop import page_layout
from laser_desktop import paths
from laser_desktop import tools_page

#: 截图落盘位置（每次覆盖）。
SHOT = "/tmp/desktop_s3.png"
#: S4（奇异修复页）截图落盘位置。
SHOT_S4 = "/tmp/desktop_s4.png"
#: S5（ISO 五点页 / 工具页）截图落盘位置。
SHOT_S5_ISO = "/tmp/desktop_s5_iso.png"
SHOT_S5_TOOLS = "/tmp/desktop_s5_tools.png"
#: S6b（默认窗口下的三页）截图落盘位置。
SHOT_S6 = {"generate": "/tmp/desktop_s6b_generate.png",
           "iso": "/tmp/desktop_s6b_iso.png",
           "repair": "/tmp/desktop_s6b_repair.png"}
#: S6b 的最小窗口（现场老笔记本）截图：证明左侧表单不被横向裁掉。
SHOT_S6_SMALL = "/tmp/desktop_s6b_small.png"
#: S6d（S6 评审后）的六张截图：三页 × 两种窗口尺寸（文件名里带尺寸）。
SHOT_S6C = {(kind, size_label): f"/tmp/desktop_s6d_{kind}_{size_label}.png"
            for kind in ("generate", "iso", "repair")
            for size_label in ("1600", "1280")}
#: S6e（三页的左右两栏横向可拖）的六张截图：默认 / 把左栏拖到最窄。
SHOT_S6E = {(kind, label): f"/tmp/desktop_s6e_{kind}_{label}.png"
            for kind in ("generate", "iso", "repair")
            for label in ("default", "dragged")}
#: S6b 的**默认**窗口（= MainWindow 的默认尺寸）：一屏 8 行的口径。
S6_WIDTH, S6_HEIGHT = 1600, 950
#: 最小窗口：允许纵向滚动，但 5 行仍要露出来。
S6_SMALL_WIDTH, S6_SMALL_HEIGHT = 1280, 800
#: 默认窗口下一屏至少要看到几行（生成页 8 点全见；ISO 只有 5 点）。
S6_MIN_ROWS = 8
#: 最小窗口下一屏至少要看到几行。
S6_SMALL_MIN_ROWS = 5
#: 真解算的超时（秒）：8 点小案子实测 < 1 s，给足余量。
RUN_TIMEOUT = 180
#: 切页耗时的上限（毫秒）：两张图共用一个 PlotView，切页不该重建 WebEngine。
PAGE_SWITCH_MS = 200

#: 取当前图的关键信息：点数 + ``data[0].name`` + 标题。
#: 预览图无名（``name`` 是 ``null``/空串）、标题以“路点预览”开头；结果图的
#: trace 名是 ``成功 N 点 · 按路径顺序``。光看点数分不清两张图（评审第 8 条）。
FIGURE_STATE_JS = """(function () {
  if (typeof gd === 'undefined' || gd === null || !gd.data || !gd.data.length) {
    return 'empty';
  }
  var name = gd.data[0].name;
  return JSON.stringify({
    points: (gd.data[0].x || []).length,
    name: (name === undefined || name === null) ? '' : String(name),
    title: String((gd.layout.title && gd.layout.title.text) || '')
  });
})()"""


def figure_state(raw) -> dict:
    """解析 :data:`FIGURE_STATE_JS` 的返回值（``empty`` / 坏 JSON 给空 dict）。"""
    try:
        return json.loads(str(raw or ""))
    except (TypeError, ValueError):
        return {}

window = MainWindow()
# 窗口就用 MainWindow 的默认尺寸（1600×950，S6b）：三段（预览 / 图 / 结果）
# 都够用，结果表一屏能露 8 行；1280×800 那档在 S6b 小节单独量。
window.resize(page_layout.DEFAULT_WINDOW_SIZE[0], page_layout.DEFAULT_WINDOW_SIZE[1])
window.show()


def qtwebengine_processes() -> int:
    """当前 ``QtWebEngineProcess`` 个数（P4；没有 pgrep 时返回 -1）。"""
    try:
        result = subprocess.run(["pgrep", "-f", "QtWebEngineProcess"],
                                capture_output=True, text=True, check=False)
    except (FileNotFoundError, OSError):        # pragma: no cover - 平台差异
        return -1
    return len([line for line in result.stdout.splitlines() if line.strip()])


def visible_rows(table) -> int:
    """表格视口里**完整**可见的行数。

    用 ``viewport().visibleRegion()`` 而不是只看视口高度：表格被父容器（页签、
    分割器）裁掉时，视口自己的高度仍是 200，``visualItemRect`` 也在那 200 里，
    可屏幕上只剩 1 行——S6 第一次"看起来改好了"就是这么骗过去的。
    """
    region = table.viewport().visibleRegion().boundingRect()
    count = 0
    for row in range(table.rowCount()):
        item = table.item(row, 0)
        if item is None:
            continue
        rect = table.visualItemRect(item)
        if rect.top() >= region.top() and rect.bottom() < region.bottom():
            count += 1
    return count


state = {"steps": [], "startup_procs": qtwebengine_processes()}
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
# P5：还没有图要看时不该建 WebEngineView（更不该拉 Chromium 子进程）。
# 这一行在 app.exec() 之前跑：预览去抖（300 ms）还没到点。
step("启动时不建图视图（P5 懒创建）",
     window.plot.created is False and window.plot.load_count == 0,
     f"created={window.plot.created} load_count={window.plot.load_count}")
step("启动时 QtWebEngineProcess ≤ 1（P5/P4）",
     state["startup_procs"] <= 1,
     f"{state['startup_procs']} 个（没装 pgrep 时为 -1）")

page = window.generate
points_text = page.metric_labels["points"].text()
step("预览卡片有总点数", "点" in points_text and page.total.count == 50,
     f"卡片={points_text!r} 网格组={page.total_value.text()!r} "
     f"格点={page.metric_labels['grid'].text()!r}")
step("预览卡片边长/条件数", page.metric_labels["edges"].text().endswith(" m")
     and page.metric_labels["edges"].text().count("×") == 2
     and page.metric_labels["design"].text() != "—",
     f"边长={page.metric_labels['edges'].text()!r} "
     f"条件数={page.metric_labels['design'].text()!r}")
step("生成按钮可用（参数合法）", page.generate_button.isEnabled(),
     f"enable={page.generate_button.isEnabled()}")
step("状态栏含绑定/数据目录", BINDING in window._status_label.text()
     and str(paths.data_root()) in window._status_label.toolTip(),
     f"{window._status_label.text()[:60]}（tooltip 含完整数据目录）")

def after_load(ok):
    step("壳页加载", ok)
    if not ok:
        app.exit(2); return
    window.plot.set_figure(build_preview_figure())
    QtCore.QTimer.singleShot(2500, check_state)

def check_state():
    window.plot.page().runJavaScript(FIGURE_STATE_JS, 0, on_state)


def on_state(raw):
    view = figure_state(raw)          # 不叫 state：模块里那个是步骤清单
    ok = (view.get("points") == 50
          and view.get("name") == ""
          and str(view.get("title") or "").startswith("路点预览"))
    step("预览图已进 WebEngine", ok,
         f"points={view.get('points')} name={view.get('name')!r} "
         f"title={str(view.get('title'))[:24]!r}")
    QtCore.QTimer.singleShot(300, start_run)


def configure_small_case(page):
    """把表单改成 8 点小案子（S3 线程安全冒烟的固定参数）。"""
    fields = page.fields
    fields["domain_axes"].setChecked(True)          # 六个轴边界
    for key, value in (("x_min", 900.0), ("x_max", 1100.0),
                       ("y_min", -100.0), ("y_max", 100.0),
                       ("z_min", 500.0), ("z_max", 700.0)):
        fields[key].setValue(value)
    fields["grid_count"].setChecked(True)          # 目标点数
    fields["target_count"].setValue(8)
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))
    fields["q_seed"].setText("43,-30,-36,42,-89,-1")
    fields["j5_min"].setValue(20.0)
    fields["seed"].setValue(0)


def start_run():
    """S3：真跑一次 8 点小案子（后台线程 → 信号 → 结果区）。"""
    configure_small_case(page)
    step("表单改成 8 点小案子", page.total.count == 8, page.total.text)
    job_id = page.start_scan()
    step("起解算任务", bool(job_id),
         f"job={job_id} 进度={page.progress_bar.format()} "
         f"取消按钮={page.cancel_button.isVisible()}")
    if not job_id:
        print("\n=== 结论 ===\n  未通过：起解算任务")
        app.exit(2); return
    state["deadline"] = time.time() + RUN_TIMEOUT
    QtCore.QTimer.singleShot(100, poll_run)


def poll_run():
    if page.run_state in ("done", "failed", "cancelled"):
        after_run(); return
    if time.time() > state.get("deadline", 0):
        step("解算完成", False, f"超时 {RUN_TIMEOUT} s")
        after_run(); return
    QtCore.QTimer.singleShot(100, poll_run)


def after_run():
    result = page.last_result or {}
    stats = result.get("stats") or {}
    step("解算终态 done", page.run_state == "done", f"state={page.run_state}")
    step("solved == 8", stats.get("solved") == 8 and stats.get("total") == 8,
         f"{stats.get('solved')}/{stats.get('total')}")
    step("结果表 8 行", page.waypoint_table.rowCount() == 8,
         f"路点={page.waypoint_table.rowCount()} "
         f"段={page.segment_table.rowCount()}")
    step("进度条到 100%", page.progress_bar.value() == 1000
         and "完成" in page.progress_bar.format(), page.progress_bar.format())
    step("导出按钮可用", all(button.isEnabled()
         for button in page.export_buttons.values()),
         ",".join(f"{key}={button.isEnabled()}"
                  for key, button in page.export_buttons.items()))
    # 8 点小案子实测不可行（条件数未达标），所以状态行是“完成（不可行）”+原因，
    # 这里只要求“完成”二字与可见性；颜色/原因在前面失败分支以外不强制。
    step("状态行显示完成", "完成" in page.status_label.text()
         and not page.status_label.isHidden(), page.status_label.text()[:60])
    page.result_tabs.setCurrentWidget(page.waypoints_page)
    # P1 的复现开关：把“点按钮前排着的旧去抖”补响一次。修好了就该是空转
    # （结果渲染后表单没变→不覆盖）；没修就会在 300 ms 内推一张预览图，
    # 下面按 trace 名判的检查会当场变红（不赌解算时长是否 < 300 ms）。
    page._debounce.start()
    QtCore.QTimer.singleShot(1200, check_result_state)


def check_result_state():
    """结果图真的换成了解算结果：是结果图那条 trace，而不是预览图。"""
    window.plot.page().runJavaScript(FIGURE_STATE_JS, 0, on_result_state)


def on_result_state(raw):
    view = figure_state(raw)
    name = str(view.get("name") or "")
    title = str(view.get("title") or "")
    ok = (view.get("points") == 8
          and name == "成功 8 点 · 按路径顺序"
          and not title.startswith("路点预览"))
    step("结果图已进 WebEngine", ok,
         f"points={view.get('points')} name={name!r} title={title[:24]!r}")
    QtCore.QTimer.singleShot(600, grab_s3)


def grab_s3():
    ok = window.grab().save(SHOT)
    size = os.path.getsize(SHOT) if os.path.exists(SHOT) else 0
    # 按钮/表格的文字已经在上面的断言里验过了；这里只要求截图像素够
    # （结果区真的画出来了），不做 OCR
    step("截图（生成页 + 结果区）", bool(ok) and size > 5000,
         f"{size} 字节 > 5000 → {SHOT}")
    QtCore.QTimer.singleShot(200, start_repair_page)


# ---------------------------------------------------------------------------
# S4：奇异修复页（真跑一次「定位段」）
# ---------------------------------------------------------------------------
def configure_repair_case(repair, stopped_at=4):
    """把修复页改成 8 点小案子 + 现场退化读数（S4 冒烟的固定参数）。

    Args:
        repair: 修复页。
        stopped_at: 已走到第几点；``0`` 表示不给（只判整链，原域可行）。
    """
    fields = repair.fields
    fields["domain_axes"].setChecked(True)
    for key, value in (("x_min", 900.0), ("x_max", 1100.0),
                       ("y_min", -100.0), ("y_max", 100.0),
                       ("z_min", 500.0), ("z_max", 700.0)):
        fields[key].setValue(value)
    fields["grid_count"].setChecked(True)
    fields["target_count"].setValue(8)
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))
    fields["q_seed"].setText("43,-30,-36,42,-89,-1")
    fields["j5_min"].setValue(20.0)
    fields["seed"].setValue(0)
    fields["max_candidates"].setValue(400)      # 回到 CLI 默认值
    for joint, value in enumerate((0.0, 0.0, 0.0, 55.72, 0.0, -55.72)):
        repair.q_rows[0]["fields"][joint].setValue(value)
    fields["stopped_at"].setValue(int(stopped_at))


def start_repair_page():
    """S4：切到「奇异修复」页 → 真跑一次「定位段」→ 截图。"""
    window.nav.setCurrentRow(1)
    repair = window.repair
    step("切到奇异修复页", window.stack.currentWidget() is repair,
         type(window.stack.currentWidget()).__name__)

    # 两张图共用同一个 PlotView：来回切 10 次的总耗时要远低于 200 ms/次
    started = time.time()
    for _ in range(10):
        window.nav.setCurrentRow(0)
        window.nav.setCurrentRow(1)
    cost = (time.time() - started) * 1000.0 / 20.0
    step("切页 <200 ms（共用 PlotView）", cost < PAGE_SWITCH_MS,
         f"平均 {cost:.1f} ms/次")

    configure_repair_case(repair)
    step("修复页表单落到 8 点", repair.total.count == 8, repair.total.text)
    result = repair.locate()               # 同步，实测 0.2–0.5 s
    step("定位段 ok", bool(result and result.get("ok")),
         (result or {}).get("verdict") or (result or {}).get("error") or "")
    kinds = [badge.text() for badge in repair.kind_badges]
    step("判定徽标有内容", bool(kinds), "、".join(kinds))
    step("段定位表有内容", repair.profile_table.rowCount() > 0,
         f"{repair.profile_table.rowCount()} 行，"
         f"结论={str(repair.verdict_label.text())[:24]!r}")
    repair.result_tabs.setCurrentWidget(repair.locate_page)
    QtCore.QTimer.singleShot(400, grab_s4)


def grab_s4():
    ok = window.grab().save(SHOT_S4)
    size = os.path.getsize(SHOT_S4) if os.path.exists(SHOT_S4) else 0
    step("截图（奇异修复页）", bool(ok) and size > 5000,
         f"{size} 字节 > 5000 → {SHOT_S4}")
    QtCore.QTimer.singleShot(200, start_repair_run)


# ---------------------------------------------------------------------------
# S4 评审回归：Q2（点数门禁）/ Q3（换任务清图）/ Q1（ok=False 结果接管）
# ---------------------------------------------------------------------------
def figure_label(figure) -> str:
    """图的简短标识：结果图有 trace 名，说明图只有一段注解。"""
    if figure is None:
        return "None"
    traces = getattr(figure, "data", ()) or ()
    if traces:
        return "trace:" + str(getattr(traces[0], "name", "") or "")
    annotations = getattr(figure.layout, "annotations", ()) or ()
    return "note:" + (str(annotations[0].text) if annotations else "")


def check_point_limit_gate(repair):
    """Q2：点数超上限时「修复」不可点，且提交被硬守卫拦住。"""
    fields = repair.fields
    fields["grid_explicit"].setChecked(True)
    for key in ("xs", "ys", "zs"):
        fields[key].setValue(200)
    fields["z_insert"].setValue(0)
    blocked = repair.start_repair() is None
    detail = (f"{repair.total.count} 点 over={repair.total.over} "
              f"enable={repair.repair_button.isEnabled()} 提交被拦={blocked}")
    return (not repair.repair_button.isEnabled()) and blocked, detail


def start_repair_run():
    """S4 真修复：先验 Q2 门禁，再真跑一次（原域可行 → mode=none）。"""
    repair = window.repair
    ok, detail = check_point_limit_gate(repair)
    step("超上限时不可点/不许提交（Q2）", ok, detail)

    configure_repair_case(repair, stopped_at=0)     # 不给 stopped_at：只判整链
    step("恢复 8 点小案子", repair.total.count == 8,
         f"{repair.total.text} 可点={repair.repair_button.isEnabled()}")
    job_id = repair.start_repair()
    step("起修复任务", bool(job_id), f"job={job_id} 进度={repair.progress_bar.format()}")
    if not job_id:
        print("\n=== 结论 ===\n  未通过：起修复任务")
        app.exit(2); return
    state["deadline"] = time.time() + RUN_TIMEOUT
    QtCore.QTimer.singleShot(100, poll_repair_run)


def poll_repair_run():
    repair = window.repair
    if repair.run_state in ("done", "failed", "cancelled"):
        after_repair_run(); return
    if time.time() > state.get("deadline", 0):
        step("修复完成", False, f"超时 {RUN_TIMEOUT} s")
        after_repair_run(); return
    QtCore.QTimer.singleShot(100, poll_repair_run)


def after_repair_run():
    """第一次修复跑完：有结果图才算真的渲染完；然后验 Q3 + 起 Q1 那次任务。"""
    repair = window.repair
    result = repair.last_result or {}
    step("修复终态 done", repair.run_state == "done",
         f"state={repair.run_state} ok={result.get('ok')} "
         f"{str(result.get('error') or '')[:32]}")
    step("修复完成后画布上是结果图", figure_label(repair.last_figure).startswith("trace:"),
         figure_label(repair.last_figure)[:40])
    step("修复产物四个导出全开", all(button.isEnabled()
         for button in repair.export_buttons.values()),
         ",".join(f"{key}={button.isEnabled()}"
                  for key, button in repair.export_buttons.items()))

    # ---- Q3：提交新任务时必须当场清掉上一张图 ----
    repair.fields["j5_min"].setValue(90.0)          # |J5| 全不达标
    repair.fields["max_candidates"].setValue(1)     # 只评估一个候选
    old = repair.last_figure
    job_id = repair.start_repair()
    cleared = repair.last_figure is None
    placeholder = figure_label(repair.current_figure())
    step("起新任务清掉上一张图（Q3）",
         bool(job_id) and cleared and placeholder != figure_label(old),
         f"job={job_id} last_figure={repair.last_figure} "
         f"current={placeholder[:28]}")
    if not job_id:
        finish(); return
    state["q1_run_id"] = job_id
    state["deadline"] = time.time() + RUN_TIMEOUT
    QtCore.QTimer.singleShot(100, poll_infeasible)


def poll_infeasible():
    repair = window.repair
    if repair.run_state in ("done", "failed", "cancelled"):
        after_infeasible(); return
    if time.time() > state.get("deadline", 0):
        step("不可行修复跑完", False, f"超时 {RUN_TIMEOUT} s")
        after_infeasible(); return
    QtCore.QTimer.singleShot(100, poll_infeasible)


def after_infeasible():
    """Q1：``ok=False`` 但 repair.json 在盘上 → 结果、运行目录、导出都要给。"""
    repair = window.repair
    result = repair.last_result or {}
    run_id = state.get("q1_run_id")
    step("不可行结果被接住（Q1）", bool(result) and result.get("ok") is False,
         f"ok={result.get('ok')} err={str(result.get('error') or '')[:34]}")
    step("run_label 显示真实运行目录（Q1）",
         bool(run_id) and run_id in repair.run_label.text()
         and "还没有运行目录" not in repair.run_label.text(),
         repair.run_label.text()[:56])
    enabled = {key: button.isEnabled()
               for key, button in repair.export_buttons.items()}
    step("只点亮 repair.json 导出（Q1）",
         enabled.get("repair") is True
         and not any(enabled.get(key) for key in ("scan_json", "joints", "plot")),
         ",".join(f"{key}={value}" for key, value in enabled.items()))
    step("报错原文红字在结果区",
         "所有候选都不可行" in repair.status_label.text()
         and "color:#c0392b" in repair.status_label.styleSheet(),
         repair.status_label.text()[:40].replace("\n", " "))
    QtCore.QTimer.singleShot(200, start_iso_page)


# ---------------------------------------------------------------------------
# S5：ISO 五点页（切两次来源 + 真跑 200 mm 立方体）
# ---------------------------------------------------------------------------
def configure_iso_case(iso) -> None:
    """把 ISO 页改成 200 mm 立方体（站点位置，5 点全可解）。"""
    fields = iso.fields
    fields["cube_bounds"].setChecked(True)
    for key, value in (("x0", 925.0), ("x1", 1125.0), ("y0", -100.0),
                       ("y1", 100.0), ("z0", 505.0), ("z1", 705.0)):
        fields[key].setValue(value)
    fields["layout"].setCurrentIndex(fields["layout"].findData("vertices"))
    fields["plane"].setCurrentIndex(fields["plane"].findData("xz"))
    fields["rpy_a"].setValue(3.0)
    fields["rpy_b"].setValue(73.0)
    fields["rpy_c"].setValue(-167.0)
    fields["euler"].setCurrentIndex(fields["euler"].findData("xyz-intrinsic"))
    fields["q_seed"].setText("43,-30,-36,42,-89,-1")
    fields["j5_min"].setValue(20.0)
    iso.refresh_preview()


def visible_iso_panels(iso) -> list[int]:
    """当前可见的立方体来源面板下标（``isVisible``；窗口已显示）。"""
    stack = iso.mode_stack
    return [index for index in range(stack.count())
            if stack.widget(index).isVisible()]


def start_iso_page():
    """S5：切到「ISO 五点」页 → 切两次立方体来源 → 真跑一次 → 截图。"""
    window.nav.setCurrentRow(2)
    iso = window.iso
    step("切到 ISO 五点页", window.stack.currentWidget() is iso,
         type(window.stack.currentWidget()).__name__)

    seen = []
    for _round in range(2):
        iso.fields["cube_auto"].setChecked(True)
        seen.append(len(visible_iso_panels(iso)))
        iso.fields["cube_bounds"].setChecked(True)
        seen.append(len(visible_iso_panels(iso)))
    step("立方体来源切两次仍恰好 1 组可见",
         seen == [1, 1, 1, 1], f"每次可见组数={seen}")

    # T1：切来源必须**同时换预览**（预览卡 / 预览页签 / 提示全跟着走）
    iso.fields["cube_auto"].setChecked(True)
    auto = iso.preview or {}
    step("切到自动搜索后预览卡是搜索箱",
         auto.get("mode") == "auto"
         and iso.preview_labels["cube"].text() == "待真跑（自动搜索）"
         and "边长" not in iso.preview_labels["cube"].text()
         and "搜索范围" in iso.preview_hint.text()
         and iso.preview_table.rowCount() == 0,
         f"mode={auto.get('mode')} 卡片={iso.preview_labels['cube'].text()} "
         f"提示={iso.preview_hint.text()[:36]}… "
         f"五点行={iso.preview_table.rowCount()}")
    iso.fields["cube_bounds"].setChecked(True)
    bounds = iso.preview or {}
    step("切回两点输入后预览卡是立方体",
         bounds.get("mode") == "bounds"
         and iso.preview_labels["cube"].text().startswith("边长")
         and iso.preview_table.rowCount() == 5,
         f"mode={bounds.get('mode')} 卡片={iso.preview_labels['cube'].text()} "
         f"五点行={iso.preview_table.rowCount()}")

    configure_iso_case(iso)
    preview = iso.preview or {}
    step("ISO 预览算出 200 mm 立方体",
         preview.get("ok") is True and preview.get("side") == 200.0
         and len(preview.get("candidates") or []) == 9,
         f"边长={preview.get('side')} 面心={len(preview.get('faces') or [])} "
         f"候选点={len(preview.get('candidates') or [])} "
         f"五点={len(preview.get('points') or [])}")
    job_id = iso.start_iso()
    step("起 ISO 五点任务", bool(job_id),
         f"job={job_id} 进度={iso.progress_bar.format()}")
    if not job_id:
        finish(); return
    state["deadline"] = time.time() + RUN_TIMEOUT
    QtCore.QTimer.singleShot(100, poll_iso_run)


def poll_iso_run():
    iso = window.iso
    if iso.run_state in ("done", "failed", "cancelled"):
        after_iso_run(); return
    if time.time() > state.get("deadline", 0):
        step("ISO 五点跑完", False, f"超时 {RUN_TIMEOUT} s")
        after_iso_run(); return
    QtCore.QTimer.singleShot(100, poll_iso_run)


def after_iso_run():
    """ISO 真跑：5 行路点、|J5|min 有值、产物按钮全开。"""
    iso = window.iso
    result = iso.last_result or {}
    stats = result.get("stats") or {}
    step("ISO 终态 done", iso.run_state == "done" and result.get("ok") is True,
         f"state={iso.run_state} ok={result.get('ok')} "
         f"err={str(result.get('error') or '')[:32]}")
    step("ISO 路点表 5 行", iso.point_table.rowCount() == 5,
         f"{stats.get('solved')}/{stats.get('total')} 行={iso.point_table.rowCount()}")
    step("ISO |J5|min 有值",
         stats.get("j5_abs_min") is not None
         and iso.stat_labels["j5"].text() != "|J5|min —",
         f"{iso.stat_labels['j5'].text()} · {iso.stat_labels['solved'].text()}")
    step("ISO 导出按钮全开",
         all(button.isEnabled() for button in iso.export_buttons.values()),
         ",".join(f"{key}={button.isEnabled()}"
                  for key, button in iso.export_buttons.items()))
    step("ISO 进度条到 100%", iso.progress_bar.value() == 1000,
         iso.progress_bar.format())
    iso.result_tabs.setCurrentWidget(iso.points_page)
    QtCore.QTimer.singleShot(400, grab_s5_iso)


def grab_s5_iso():
    ok = window.grab().save(SHOT_S5_ISO)
    size = os.path.getsize(SHOT_S5_ISO) if os.path.exists(SHOT_S5_ISO) else 0
    step("截图（ISO 五点页）", bool(ok) and size > 5000,
         f"{size} 字节 > 5000 → {SHOT_S5_ISO}")
    QtCore.QTimer.singleShot(200, start_tools_page)


# ---------------------------------------------------------------------------
# S5：工具页（运行表 + 模板 sha256）
# ---------------------------------------------------------------------------
def start_tools_page():
    """S5：切到「工具」页 → 运行表非空、模板 sha256 显示 → 截图。"""
    window.nav.setCurrentRow(3)
    tools = window.tools
    step("切到工具页", window.stack.currentWidget() is tools,
         type(window.stack.currentWidget()).__name__)
    step("工具页运行表非空",
         len(tools.runs) > 0 and tools.runs_table.rowCount() == len(tools.runs),
         f"{len(tools.runs)} 个运行目录 · {tools.runs_count_label.text()}")
    info = tools.template_file or {}
    summary = (tools.template_path_label.text() + " · "
               + tools.template_size_label.text() + " · "
               + tools.template_sha_label.text())
    if info.get("exists"):
        digest = tools.template_sha_label.text().replace("sha256", "").strip()
        step("工具页显示模板 sha256",
             len(digest) == 64
             and all(char in "0123456789abcdef" for char in digest),
             summary)
    else:
        # 模板不入库（也不再有 ~/Documents/backup 兜底）：这时界面必须明说「—」
        # 并把搜索路径与环境变量名列出来——这正是 T1 之外另一条评审意见。
        step("工具页明说找不到模板（不谎报 sha256）",
             "—" in tools.template_sha_label.text()
             and "—" in tools.template_size_label.text()
             and tools_page.TEMPLATE_ENV in tools.template_path_label.text(),
             summary)
    # 截图要能看见被检查的两块（运行目录 + 模板文件），所以先滚到底
    bar = tools.scroll.verticalScrollBar()
    bar.setValue(bar.maximum())
    QtCore.QTimer.singleShot(400, grab_s5_tools)


def grab_s5_tools():
    ok = window.grab().save(SHOT_S5_TOOLS)
    size = os.path.getsize(SHOT_S5_TOOLS) if os.path.exists(SHOT_S5_TOOLS) else 0
    step("截图（工具页）", bool(ok) and size > 5000,
         f"{size} 字节 > 5000 → {SHOT_S5_TOOLS}")
    QtCore.QTimer.singleShot(200, start_s6)


# ---------------------------------------------------------------------------
# S6b：默认 200×950 一屏 8 行 / 1280×800 表单不横向裁 / P1 双向 / P2 不重载 /
#       P3 三组耗时 / P4 子进程
# ---------------------------------------------------------------------------
def wait_events(seconds: float = 0.3) -> None:
    """跑一小段事件循环（布局要两轮才量得准）。"""
    deadline = time.time() + seconds
    while time.time() < deadline:
        app.processEvents()
        time.sleep(0.01)


def start_s6():
    """S6b：先把窗口拉回默认的 1600×950，再逐页量“一屏几行”并截图。"""
    window.nav.setCurrentRow(0)
    window.resize(S6_WIDTH, S6_HEIGHT)
    QtCore.QTimer.singleShot(500, s6_generate)


def s6_report_rows(name, page, table, want_rows=None, size=None):
    """量结果表：高度、可见行数、分割器分配（三页同一口径）。

    ``want_rows`` 不传就按"默认窗口 8 行"算，但不超过表里实际行数（ISO 只有
    5 个点，不能要求 8 行）。``size`` 是期望的窗口尺寸（默认取 1600×950）。
    """
    want_rows = S6_MIN_ROWS if want_rows is None else int(want_rows)
    want_rows = min(want_rows, max(1, table.rowCount()))
    width, height = (S6_WIDTH, S6_HEIGHT) if size is None else size
    wait_events(0.35)                    # 刚切过页签/刚出结果：等布局落定
    rows = visible_rows(table)
    sizes = page._viewer_splitter.sizes()
    ok = (rows >= want_rows
          and table.height() >= 180
          and window.width() == width and window.height() == height)
    step(f"{name}一屏可见 ≥{want_rows} 行", ok,
         f"表高={table.height()} viewport={table.viewport().height()} "
         f"可视={table.viewport().visibleRegion().boundingRect().height()} "
         f"行数={table.rowCount()} 可见={rows} 分割={sizes} "
         f"窗口={window.width()}×{window.height()} "
         f"窗口最小={window.minimumSizeHint().width()}×"
         f"{window.minimumSizeHint().height()}")
    # P2：三段高度（预览 / 图 / 结果）一起报出来
    step(f"{name}三段高度（预览/图/结果）", len(sizes) == 3,
         f"{sizes}（图 ≥{page_layout.PLOT_MIN_HEIGHT}、"
         f"结果 ≥{page_layout.RESULT_MIN_HEIGHT} 是默认窗口的验收线）")
    return rows


def s6_grab(name):
    shot = SHOT_S6[name]
    ok = window.grab().save(shot)
    size = os.path.getsize(shot) if os.path.exists(shot) else 0
    step(f"截图（S6b {name}，默认 1600×950）",
         bool(ok) and size > 5000, f"{size} 字节 → {shot}")


def s6_generate():
    page = window.generate
    page.result_tabs.setCurrentWidget(page.waypoints_page)
    s6_report_rows("生成页", page, page.waypoint_table)
    s6_grab("generate")
    QtCore.QTimer.singleShot(150, s6_iso)


def s6_iso():
    window.nav.setCurrentRow(2)
    QtCore.QTimer.singleShot(300, lambda: s6_iso_check(window.iso))


def s6_iso_check(iso):
    iso.result_tabs.setCurrentWidget(iso.points_page)
    s6_report_rows("ISO 页", iso, iso.point_table)
    s6_grab("iso")
    QtCore.QTimer.singleShot(150, s6_repair)


def s6_repair():
    """修复页：Q1 那次“所有候选都不可行”之后计划表是空的，再真跑一次。

    这里不用构造数据：截图要能对着真结果看（否则"运行 …/tmp/s6-smoke" 一眼就是假的）。
    """
    window.nav.setCurrentRow(1)
    repair = window.repair
    configure_repair_case(repair, stopped_at=0)
    job_id = repair.start_repair()
    step("S6 再跑一次修复（出 8 行计划）", bool(job_id), f"job={job_id}")
    if not job_id:
        s6_repair_check(); return
    state["deadline"] = time.time() + RUN_TIMEOUT
    QtCore.QTimer.singleShot(100, poll_s6_repair)


def poll_s6_repair():
    if window.repair.run_state in ("done", "failed", "cancelled"):
        QtCore.QTimer.singleShot(300, s6_repair_check)
        return
    if time.time() > state.get("deadline", 0):
        step("S6 修复跑完", False, f"超时 {RUN_TIMEOUT} s")
        s6_repair_check(); return
    QtCore.QTimer.singleShot(100, poll_s6_repair)


def s6_repair_check():
    repair = window.repair
    repair.result_tabs.setCurrentWidget(repair.plan_page)
    step("S6b 修复结果表有 8 行", repair.waypoint_table.rowCount() == 8,
         f"行数={repair.waypoint_table.rowCount()} 状态={repair.run_state}")
    s6_report_rows("修复页", repair, repair.waypoint_table)
    s6_grab("repair")
    QtCore.QTimer.singleShot(150, s6_small_window)


def s6_small_window():
    """P1/P4/P6：把窗口缩到最小的 1280×800，证明表单不被横向裁、5 行仍在。"""
    window.resize(S6_SMALL_WIDTH, S6_SMALL_HEIGHT)
    window.nav.setCurrentRow(1)                  # 修复页：旧版裁得最厉害的
    wait_events(0.4)
    repair = window.repair
    repair.result_tabs.setCurrentWidget(repair.plan_page)
    wait_events(0.4)
    # P4-①：表单没有横向滚动条
    scroll = repair.form_scroll
    bar = scroll.horizontalScrollBar()
    step("1280×800 表单无横向滚动条（P4）", bar.maximum() == 0,
         f"maximum={bar.maximum()} 视口={scroll.viewport().width()} "
         f"内容最小={scroll.widget().minimumSizeHint().width()}")
    # P4-②：数值框宽度 ≥ sizeHint（"500.0 mr"那种切字不会再出现）
    repair.fields["domain_axes"].setChecked(True)     # 「六个轴边界」面板
    wait_events(0.3)
    detail = []
    ok = True
    for key in ("y_min", "z_min"):
        box = repair.fields[key]
        good = box.width() >= box.sizeHint().width()
        ok = ok and good
        detail.append(f"{key}={box.width()}≥{box.sizeHint().width()}" if good
                      else f"{key}={box.width()}<{box.sizeHint().width()}(切字)")
    step("1280×800 修复页 Y/Z 下界数值框完整可见（P4）", ok, "；".join(detail))
    # P4-③：「删除本组」按钮完整可见
    button = repair.q_rows[0]["remove_button"]
    inside = (button.mapTo(scroll.widget(), button.rect().topLeft()).x() >= 0
              and button.width() >= button.sizeHint().width())
    step("1280×800「删除本组」按钮完整可见（P4）", inside,
         f"按钮 {button.width()}≥{button.sizeHint().width()}，"
         f"视口={scroll.viewport().width()}")
    # P6：最小窗口下 5 行
    s6_report_rows("修复页（1280×800）", repair, repair.waypoint_table,
                   want_rows=S6_SMALL_MIN_ROWS,
                   size=(S6_SMALL_WIDTH, S6_SMALL_HEIGHT))
    shot = SHOT_S6_SMALL
    saved = window.grab().save(shot)
    size = os.path.getsize(shot) if os.path.exists(shot) else 0
    step("截图（1280×800 修复页，表单不裁）", bool(saved) and size > 5000,
         f"{size} 字节 → {shot}")
    window.resize(S6_WIDTH, S6_HEIGHT)
    QtCore.QTimer.singleShot(400, s6_back_to_generate)


def s6_back_to_generate():
    window.nav.setCurrentRow(0)
    QtCore.QTimer.singleShot(300, s6_p1_js_select)


def js(script, callback):
    window.plot.page().runJavaScript(script, 0, callback)


def s6_p1_js_select():
    """P1 正向：JS 侧调 ``__select(['5'])`` → 表格第 5 行被选中。"""
    js("window.__select(['5'])", lambda _raw: QtCore.QTimer.singleShot(
        500, s6_p1_js_select_check))


def s6_p1_js_select_check():
    page = window.generate
    row = page.waypoint_table.currentRow()
    step("图上选点 → 表格选中第 5 行（P1）",
         row == 4 and "GP0005" in page.last_status,
         f"currentRow={row} 状态={page.last_status!r}")
    js("window.__keys()", s6_p1_keys)


def s6_p1_keys(raw):
    """每个 trace 的 ``customdata[0]`` 必须是 GP 号（失败点那条也要有）。"""
    try:
        keys = json.loads(str(raw or "[]"))
    except (TypeError, ValueError):
        keys = []
    flat = []
    for trace_keys in keys:
        for key in trace_keys:
            if key is not None and key != "":
                flat.append(key)
    step("图里每个点都带 GP（customdata[0]，P1）",
         sorted(int(key) for key in flat) == list(range(1, 9)),
         f"trace 数={len(keys)} 键={flat[:10]}")
    # P1 反向：表格选第 3 行 → JS 侧 __select 收到 "3"
    window.generate.waypoint_table.setCurrentCell(2, 0)
    QtCore.QTimer.singleShot(400, s6_p1_table_select)


def s6_p1_table_select():
    js("window.__selectedKeys()", s6_p1_table_select_check)


def s6_p1_table_select_check(raw):
    try:
        keys = json.loads(str(raw or "[]"))
    except (TypeError, ValueError):
        keys = []
    page = window.generate
    step("表格选行 → 图上高亮同一个 GP（P1）",
         keys == ["3"] and "GP0003" in page.last_status,
         f"JS selected={keys} 状态={page.last_status!r}")
    QtCore.QTimer.singleShot(100, s6_p2_start)


def s6_p2_start():
    """P2：连续换 10 次图 —— 页不重载、相机不动（loadFinished 计数不变）。"""
    js("Plotly.relayout(gd, {'scene.camera': {eye: {x: 1.7, y: 1.2, z: 0.9}}}); 'ok'",
       lambda _raw: QtCore.QTimer.singleShot(400, s6_p2_push))


def s6_p2_push():
    page = window.generate
    state["p2_count"] = state.get("p2_count", 0)
    if state["p2_count"] >= 10:
        QtCore.QTimer.singleShot(800, s6_p2_check)
        return
    state["p2_count"] += 1
    values = page.values()
    values["rpy"] = [3.0, 73.0 + state["p2_count"], -167.0]
    window.plot.set_figure(build_preview_figure(values))
    QtCore.QTimer.singleShot(120, s6_p2_push)


CAMERA_JS = """(function () {
  try {
    var scene = gd._fullLayout.scene;
    var cam = (scene && scene.camera) || (gd.layout.scene && gd.layout.scene.camera);
    return JSON.stringify(cam || null);
  } catch (error) { return '"err"'; }
})()"""


def s6_p2_check():
    step("连续换图 10 次不重载壳页（P2）",
         window.plot.load_count == 1,
         f"load_count={window.plot.load_count}")
    js(CAMERA_JS, s6_p2_camera)


def s6_p2_camera(raw):
    try:
        camera = json.loads(str(raw or "null"))
    except (TypeError, ValueError):
        camera = None
    eye = ((camera or {}).get("eye") or {}) if isinstance(camera, dict) else {}
    ok = (abs(float(eye.get("x", 0)) - 1.7) < 1e-6
          and abs(float(eye.get("y", 0)) - 1.2) < 1e-6
          and abs(float(eye.get("z", 0)) - 0.9) < 1e-6)
    step("换图后相机视角不动（P2）", ok, f"camera={camera}")
    QtCore.QTimer.singleShot(100, s6_p3_start)


def plan_figure(count: int):
    """用 ``core`` 生成 ``count`` 点计划（``skip_ik=True``，不跑 IK）。"""
    from laser_desktop import generate_page as gen
    from laser_web import core, figures

    previous = os.environ.get("LASER_WEB_RUNS")
    os.environ["LASER_WEB_RUNS"] = tempfile.mkdtemp(prefix="s6-runs-")
    try:
        values = gen.form_defaults()
        values["grid_mode"] = "count"
        values["target_count"] = count
        values["domain_axes"] = values["domain_mode"] = "axes"
        params = gen.form_to_params(values)
        params["skip_ik"] = True
        result = core.run_scan(params)
    finally:
        if previous is None:
            os.environ.pop("LASER_WEB_RUNS", None)
        else:
            os.environ["LASER_WEB_RUNS"] = previous
    rows = result.get("waypoints") or []
    return figures.waypoint_figure(rows, all_ok=True), len(rows)


def s6_p3_start():
    """P3：50 / 200 / 1000 点各测一次 ``Plotly.react`` 的 JS 侧耗时。"""
    state["p3"] = []
    state["p3_queue"] = [50, 200, 1000]
    s6_p3_next()


def s6_p3_next():
    queue = state["p3_queue"]
    if not queue:
        s6_p3_report()
        return
    count = queue.pop(0)
    started = time.time()
    figure, points = plan_figure(count)
    step(f"生成 {count} 点计划（skip_ik，P3）", points >= count * 0.9,
         f"{points} 点，plan+建图 {1000 * (time.time() - started):.0f} ms")
    state["p3_started"] = time.time()

    def measured(ms):
        state["p3"].append((count, points, ms))
        QtCore.QTimer.singleShot(100, s6_p3_next)

    window.plot.measure_react(figure, measured, timeout_s=30.0)


def s6_p3_report():
    lines = [f"{count} 点 → {ms:.1f} ms" for count, _points, ms in state["p3"]
             if ms is not None]
    thousand = [ms for count, _points, ms in state["p3"] if count == 1000]
    slowest = max([ms for _c, _p, ms in state["p3"] if ms is not None] or [0.0])
    step("1000 点 react < 1500 ms（P3）",
         bool(thousand) and thousand[0] is not None and thousand[0] < 1500.0,
         "；".join(lines) + f"（最慢 {slowest:.0f} ms）")
    QtCore.QTimer.singleShot(200, s6_p4)


def s6_p4():
    count = qtwebengine_processes()
    step("QtWebEngineProcess ≤ 2（P4）", 0 < count <= 2, f"{count} 个")
    step("三页共用一个图视图（P5）",
         window.plot.created and window.plot.view is not None,
         f"created={window.plot.created} 视图={type(window.plot.view).__name__} "
         f"load_count={window.plot.load_count}")
    QtCore.QTimer.singleShot(200, s6c_start)


# ---------------------------------------------------------------------------
# S6c：主操作按钮的固定行 + 预览卡按内容拿高度（三页 × 两种窗口尺寸，六张截图）
# ---------------------------------------------------------------------------
def label_fully_visible(widget) -> bool:
    """控件在屏幕上完整可见（可见区域 == 整个 rect）。"""
    if not widget.isVisible():
        return False
    return widget.visibleRegion().boundingRect() == widget.rect()


def buttons_of(page) -> dict:
    """当前页的主操作按钮（S6c 只认这三个）。"""
    if page is window.repair:
        return {"定位段": page.locate_button, "修复": page.repair_button}
    return {page.generate_button.text(): page.generate_button}


def metric_labels_of(page) -> dict:
    """预览卡里的五项数值（生成 / 修复 = 体积…欧拉约定；ISO = 立方体…截面）。"""
    if page is window.iso:
        keys = ("cube", "apothem", "center", "layout", "plane")
        return {key: page.preview_labels[key] for key in keys}
    keys = ("volume", "grid", "points", "design", "euler")
    return {key: page.metric_labels[key] for key in keys}


def s6c_start():
    """先把三种尺寸×三页排成一队（1600 与 1280 各一遍）。"""
    state["s6c_queue"] = [
        (size_label, size, kind, row)
        for size_label, size in (("1600", (S6_WIDTH, S6_HEIGHT)),
                                 ("1280", (S6_SMALL_WIDTH, S6_SMALL_HEIGHT)))
        for kind, row in (("generate", 0), ("iso", 2), ("repair", 1))]
    s6c_next()


def s6c_next():
    queue = state.get("s6c_queue") or []
    if not queue:
        s6c_done()
        return
    size_label, size, kind, row = queue.pop(0)
    window.resize(*size)
    window.nav.setCurrentRow(row)
    QtCore.QTimer.singleShot(600, lambda: s6c_check(size_label, size, kind))


def s6c_check(size_label, size, kind):
    """量一页的一档：预览卡 / 图 / 表 / 按钮 + 截图。"""
    page = window.stack.currentWidget()
    if kind == "generate":
        tab = page.waypoints_page
    elif kind == "iso":
        tab = page.points_page
    else:
        tab = page.plan_page
    page.result_tabs.setCurrentWidget(tab)
    wait_events(0.5)
    width, height = size
    label = f"{kind} {width}×{height}"
    splitter = page._viewer_splitter
    preview, plot, results = splitter.sizes()
    scroll = splitter.widget(0)
    card = splitter.preview_card
    table = page.point_table if kind == "iso" else page.waypoint_table
    rows = visible_rows(table)
    want_rows = (S6_MIN_ROWS if (size == (S6_WIDTH, S6_HEIGHT)) else
                 S6_SMALL_MIN_ROWS)
    want_rows = min(want_rows, max(1, table.rowCount()))
    want_plot = (page_layout.PLOT_MIN_HEIGHT if size == (S6_WIDTH, S6_HEIGHT)
                 else page_layout.PLOT_FLOOR_HEIGHT)

    # ① 预览卡：1600 下内容不滚（五项数值一眼看到），1280 下允许滚
    metrics = metric_labels_of(page)
    hidden = [key for key, widget in metrics.items()
              if not label_fully_visible(widget)]
    bar = scroll.verticalScrollBar().maximum()
    if size == (S6_WIDTH, S6_HEIGHT):
        # 判据是"五项数值一眼全见"；卡片尾巴（core 打印的两行提示）可能还要
        # 滚 8–11 px，那不影响判断可行/不可行，数字照实报出来。
        ok = not hidden
        step(f"S6c {label} 预览卡五项数值无滚动可见", ok,
             f"预览卡={preview} px（内容 {card.sizeHint().height()}），"
             f"还需滚 {bar} px，看不到的项={hidden or '无'}")
    else:
        step(f"S6c {label} 预览卡可滚动（允许）", preview >= page_layout.PREVIEW_MIN_HEIGHT,
             f"预览卡={preview} px，滚动 {bar} px，看不到的项={hidden or '无'}")

    # ② 图区
    step(f"S6c {label} 图区 ≥{want_plot}", plot >= want_plot, f"图={plot} px")

    # ③ 表的可见行数
    step(f"S6c {label} 一屏 ≥{want_rows} 行", rows >= want_rows,
         f"可见={rows} 行（表高 {table.height()}，结果区={results} px）")

    # ④ 主操作按钮：可见区域 == 整块，且 height ≥ sizeHint
    detail = []
    ok = True
    for text, button in buttons_of(page).items():
        region = button.visibleRegion().boundingRect()
        good = (button.isVisible() and region == button.rect()
                and button.height() >= button.sizeHint().height())
        ok = ok and good
        if good:
            detail.append(f"{text} {button.width()}×{button.height()}"
                          f"（region={region.width()}×{region.height()}，"
                          f"sizeHint 高={button.sizeHint().height()}）")
        else:
            detail.append(f"{text} 不完整：region={region}，"
                          f"rect={button.rect()}")
    step(f"S6c {label} 主操作按钮完整可见", ok, "；".join(detail))

    # ⑤ 截图
    shot = SHOT_S6C[(kind, size_label)]
    saved = window.grab().save(shot)
    size_bytes = os.path.getsize(shot) if os.path.exists(shot) else 0
    step(f"截图（S6c {label}）", bool(saved) and size_bytes > 5000,
         f"{size_bytes} 字节 → {shot}")
    QtCore.QTimer.singleShot(150, s6c_next)


def s6c_done():
    """回到默认尺寸，接着跑 S6d（评审三处必改的探针）。"""
    window.resize(S6_WIDTH, S6_HEIGHT)
    window.nav.setCurrentRow(0)
    wait_events(0.3)
    s6d_start()


# ---------------------------------------------------------------------------
# S6d：S6 评审的三处必改（真 WebEngine 探针）
# ---------------------------------------------------------------------------
def marker_sizes_js(callback):
    """读壳页里每个 trace 的 ``marker.size``（高亮只能靠这个数组证明）。"""
    js("window.__markerSizes()", callback)


def s6d_start():
    """先在生成页把高亮清干净，读出基准尺寸。"""
    window.nav.setCurrentRow(0)
    wait_events(0.5)
    page = window.generate
    page.result_tabs.setCurrentWidget(page.waypoints_page)
    wait_events(0.3)
    js("window.__select([])", lambda _raw: QtCore.QTimer.singleShot(
        350, lambda: marker_sizes_js(s6d_r1_base)))


def s6d_r1_base(raw):
    """R1-0：清空后每个点都回到基准（结果图的 marker.size=5）。"""
    try:
        sizes = json.loads(str(raw or "[]"))
    except (TypeError, ValueError):
        sizes = []
    base = sizes[0] if sizes else []
    rows = window.generate.waypoint_table.rowCount()
    if isinstance(base, (int, float)):          # 还没 restyle 过：标量
        base = [base] * rows
    state["base"] = list(base)
    step("R1 基准 marker.size（清空后全回 5）",
         len(base) == 8 and all(value == 5 for value in base), f"{base}")
    s6d_r1_select("3", 3, s6d_r1_after_3)


def s6d_r1_select(key, index, next_step):
    """选一个点，350 ms 后读回 marker.size 数组。"""
    js("window.__select(['%s'])" % key, lambda _raw: QtCore.QTimer.singleShot(
        350, lambda: marker_sizes_js(
            lambda raw: s6d_r1_check(raw, index, next_step))))


def s6d_r1_check(raw, index, next_step):
    """断言“只有第 index 个点是 12”，并把数组原文打出来。"""
    try:
        sizes = json.loads(str(raw or "[]"))[0]
    except (TypeError, ValueError, IndexError):
        sizes = []
    expected = list(state.get("base") or [])
    if len(expected) >= index:
        expected[index - 1] = 12
    step(f"R1 选 {index} → 只有第 {index} 个点是 12", sizes == expected,
         f"数组={sizes}")
    next_step()


def s6d_r1_after_3():
    """R1-②：改选 7（上一次的 3 必须缩回去）。"""
    s6d_r1_select("7", 7, s6d_r1_after_7)


def s6d_r1_after_7():
    """R1-③：清空 → 全部回基准 5，回显也回空。"""
    js("window.__select([])", lambda _raw: QtCore.QTimer.singleShot(
        350, lambda: marker_sizes_js(s6d_r1_after_clear)))


def s6d_r1_after_clear(raw):
    try:
        sizes = json.loads(str(raw or "[]"))[0]
    except (TypeError, ValueError, IndexError):
        sizes = []
    base = list(state.get("base") or [])
    step("R1 清空 → 全部回到基准 5", sizes == base, f"数组={sizes}")
    js("window.__selectedKeys()", s6d_r2)


def s6d_r2(selected_raw):
    """R2：切页后状态栏原文带页面前缀、不残留；切回来高亮要补回。"""
    try:
        shown = json.loads(str(selected_raw or "[]"))
    except (TypeError, ValueError):
        shown = None
    step("R1 清空后回显也为空", shown == [], f"__selectedKeys={shown}")
    page = window.generate
    page.waypoint_table.setCurrentCell(4, 0)        # GP0005
    wait_events(0.4)
    message = window.statusBar().currentMessage()
    step("R2 页面消息带页面前缀", message.startswith("生成："), f"状态栏={message!r}")
    window.nav.setCurrentRow(2)                     # 切到 ISO 五点
    wait_events(0.4)
    iso_message = window.statusBar().currentMessage()
    step("R2 切页后不残留上一页的话",
         "GP0005" not in iso_message
         and (not iso_message or iso_message.startswith("ISO：")),
         f"状态栏={iso_message!r}（ISO 页）")
    window.nav.setCurrentRow(0)                     # 切回来
    wait_events(0.8)
    step("R2 切回生成页状态栏", window.statusBar().currentMessage().startswith("生成："),
         f"状态栏={window.statusBar().currentMessage()!r}")
    state["r2_start"] = time.time()
    marker_sizes_js(s6d_r2_highlight_back)


def s6d_r2_highlight_back(raw):
    """R2c：切回来之后图上的选中点必须重新是 12（react 会把它重置成 5）。"""
    try:
        sizes = json.loads(str(raw or "[]"))[0]
    except (TypeError, ValueError, IndexError):
        sizes = []
    expected = list(state.get("base") or [])
    if len(expected) >= 5:
        expected[4] = 12                        # GP0005 是第 5 行
    step("R2 切回后图上的选中点补回来了", sizes == expected, f"数组={sizes}")
    QtCore.QTimer.singleShot(12000, s6d_r2_after_timeout)


def s6d_r2_after_timeout():
    """R2：临时消息 10 s 超时后，常驻信息（永久控件）必须还在。"""
    message = window.statusBar().currentMessage()
    label = window._status_label.text()
    tooltip = window._status_label.toolTip()
    elapsed = time.time() - state.get("r2_start", time.time())
    step("R2 临时消息到点自己清掉", message == "",
         f"{elapsed:.1f} s 后临时区={message!r}")
    step("R2 12 s 后常驻信息仍在",
         BINDING in label and str(paths.data_root()) in tooltip,
         f"永久控件={label!r}")
    s6d_r3()


def s6d_r3():
    """R3：``_user_moved`` 之后，``protect_plot`` 不许把图区抢回 260。"""
    window.nav.setCurrentRow(1)
    wait_events(0.5)
    splitter = window.repair._viewer_splitter
    total = sum(splitter.sizes())
    results_min = max(page_layout.RESULT_MIN_HEIGHT,
                      splitter.widget(2).minimumSizeHint().height())
    target = [total - results_min - 210, 210, results_min]
    splitter._user_moved = True
    splitter.setSizes(target)
    wait_events(0.6)
    sizes = splitter.sizes()
    step("R3 用户拖过后图区不被抢回 260",
         sizes[1] < page_layout.PLOT_MIN_HEIGHT and sizes[0] >= target[0] - 8,
         f"目标={target} → 实际={sizes}")
    step("R3 plot_floor 走硬下限 200",
         splitter.plot_floor() == page_layout.PLOT_FLOOR_HEIGHT,
         f"plot_floor={splitter.plot_floor()}")
    step("R3 protect_plot 不再抢高度", splitter.protect_plot() is False,
         f"protect_plot 返回 False，分割={splitter.sizes()}")
    QtCore.QTimer.singleShot(200, s6d_done)


def s6d_done():
    window.resize(S6_WIDTH, S6_HEIGHT)
    window.nav.setCurrentRow(0)
    wait_events(0.3)
    s6e_start()


# ---------------------------------------------------------------------------
# S6e：三页的左右两栏横向可拖（默认 / 拖到最窄，六张截图）
# ---------------------------------------------------------------------------
def s6e_start():
    """S6e：三页 ×（默认 / 把左栏拖到最窄）的横向分割器，六张截图。"""
    window.resize(S6_WIDTH, S6_HEIGHT)
    state["s6e_queue"] = [("generate", 0), ("iso", 2), ("repair", 1)]
    s6e_next()


def s6e_next():
    queue = state.get("s6e_queue") or []
    if not queue:
        finish()
        return
    kind, row = queue.pop(0)
    window.nav.setCurrentRow(row)
    QtCore.QTimer.singleShot(500, lambda: s6e_default(kind))


def s6e_table_tab(page):
    """截图里要看得到结果表：切到带表的页签。"""
    if page is window.iso:
        return page.points_page
    if page is window.repair:
        return page.plan_page
    return page.waypoints_page


def s6e_grab(kind, label):
    """存一张 S6e 截图。"""
    shot = SHOT_S6E[(kind, label)]
    saved = window.grab().save(shot)
    size_bytes = os.path.getsize(shot) if os.path.exists(shot) else 0
    step(f"截图（S6e {kind} {label}）", bool(saved) and size_bytes > 5000,
         f"{size_bytes} 字节 → {shot}")


def s6e_default(kind):
    """① 默认宽度（可复现）+ 两段下限的硬指标 + ``default`` 截图。"""
    page = window.stack.currentWidget()
    page.result_tabs.setCurrentWidget(s6e_table_tab(page))
    body = page._body_splitter
    body.reset_ratios()            # 截图要可复现：回默认比例（不读用户存过的）
    wait_events(0.5)
    sizes = body.sizes()
    left_min, right_min = body.segment_minimums()
    natural = body.natural_left_width()
    form = page.form_scroll
    vertical = page._right_splitter
    state["s6e_vsizes"] = list(vertical.sizes())
    step(f"S6e {kind} 两栏之间确实是可拖分割器",
         isinstance(body, QtWidgets.QSplitter)
         and body.orientation() == QtCore.Qt.Horizontal
         and body.count() == 2
         and body.handleWidth() == page_layout.BODY_HANDLE_WIDTH
         and not body.childrenCollapsible(),
         f"手柄={body.handleWidth()} px，不可折叠={not body.childrenCollapsible()}，"
         f"两段={body.count()}")
    share = sizes[0] / float(window.width())
    step(f"S6e {kind} 默认宽度（左栏 ≈ 表单自然宽度，窗口宽的 {share:.0%}）",
         sizes[0] >= natural and sizes[0] >= left_min
         and 0.30 < share < 0.45 and sizes[1] > sizes[0],
         f"左={sizes[0]}（自然宽 {natural}，下限 {left_min}，表单 {form.width()}）"
         f" 右={sizes[1]}（下限 {right_min}），可拖幅 {sizes[0] - left_min} px")
    s6e_grab(kind, "default")
    QtCore.QTimer.singleShot(200, lambda: s6e_dragged(kind, page))


def s6e_dragged(kind, page):
    """② 把左栏拖到最窄（右栏放宽）：不裁数值框 + 竖分割不动 + 截图。"""
    body = page._body_splitter
    form = page.form_scroll
    vertical = page._right_splitter
    before_v = state.get("s6e_vsizes") or []
    left_min = body.widget(0).minimumWidth()
    body.moveSplitter(1, 1)        # 当真用户拖到最左边（handle 1 拖到 x=1）
    wait_events(0.5)
    sizes = body.sizes()
    step(f"S6e {kind} 拖到最左后左段贴住下限",
         left_min <= sizes[0] <= left_min + 2,
         f"左={sizes[0]}（下限 {left_min}） 右={sizes[1]}"
         f" 比例={body.live_ratios()} 用户拖过={body.user_moved}")
    boxes = {key: widget for key, widget in page.fields.items()
             if isinstance(widget, QtWidgets.QAbstractSpinBox)
             and widget.isVisible()}
    clipped = {key: (box.width(), box.sizeHint().width())
               for key, box in boxes.items()
               if box.width() < box.sizeHint().width()}
    step(f"S6e {kind} 左栏最窄时数值框完整",
         not clipped and form.horizontalScrollBar().maximum() == 0,
         f"数值框 {len(boxes)} 个，被压窄={clipped or '无'}，"
         f"表单横滚条={form.horizontalScrollBar().maximum()}"
         f"（视口 {form.viewport().width()}，内容最小 "
         f"{form.widget().minimumSizeHint().width()}）")
    after_v = list(vertical.sizes())
    step(f"S6e {kind} 横向拖动不动竖分割比例", after_v == before_v,
         f"竖分割 {before_v} → {after_v}，``protect_plot`` "
         f"{'不抢' if not vertical.protect_plot() else '还在抢'}")
    shot = SHOT_S6E[(kind, "dragged")]
    s6e_grab(kind, "dragged")
    body.reset_ratios()            # 下一张默认截图仍要可复现
    wait_events(0.2)
    QtCore.QTimer.singleShot(150, s6e_next)


def finish():
    # P5：关窗前按"先删 page 再删 view"的顺序拆掉 WebEngine——不拆的话 Qt 在
    # 进程退出时会打 "Release of profile requested but WebEnginePage still not
    # deleted. Expect troubles !"（交付到 Windows 就是关窗报错）。
    window.close()
    app.processEvents()
    step("关窗后图视图已拆（P5，stderr 无 not deleted）",
         not window.plot.created,
         f"created={window.plot.created} load_count={window.plot.load_count}")
    print("\n=== 结论 ===")
    bad = [name for name, ok, _ in state["steps"] if not ok]
    print("  全部通过" if not bad else "  未通过：" + "、".join(bad))
    app.exit(0 if not bad else 3)

window.plot.loadFinished.connect(after_load)
# 硬超时：比解算超时留更多余量（壳页 + 两次 JS 往返 + 报图各要几百毫秒）
QtCore.QTimer.singleShot(max(60000, (RUN_TIMEOUT + 30) * 1000),
                         lambda: (print("  超时"), app.exit(4)))
app.exec()
