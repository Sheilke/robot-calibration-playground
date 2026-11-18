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
import sys
import time

from laser_desktop.qt_compat import configure_webengine_environment, QtWidgets, QtCore, BINDING
configure_webengine_environment()
app = QtWidgets.QApplication(sys.argv)
from laser_desktop.main_window import MainWindow, build_preview_figure
from laser_desktop import paths
from laser_desktop import tools_page

#: 截图落盘位置（每次覆盖）。
SHOT = "/tmp/desktop_s3.png"
#: S4（奇异修复页）截图落盘位置。
SHOT_S4 = "/tmp/desktop_s4.png"
#: S5（ISO 五点页 / 工具页）截图落盘位置。
SHOT_S5_ISO = "/tmp/desktop_s5_iso.png"
SHOT_S5_TOOLS = "/tmp/desktop_s5_tools.png"
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
# 窗口尺寸按“现场笔记本最大化”取：结果区展开后图还能有 ~350 px 高
# （默认窗口 1180×780 在 S3 之后会显得拘谨，但那是 MainWindow 的默认，不改）
window.resize(1360, 900)
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
    finish()


def finish():
    print("\n=== 结论 ===")
    bad = [name for name, ok, _ in state["steps"] if not ok]
    print("  全部通过" if not bad else "  未通过：" + "、".join(bad))
    app.exit(0 if not bad else 3)

window.plot.loadFinished.connect(after_load)
# 硬超时：比解算超时留更多余量（壳页 + 两次 JS 往返 + 报图各要几百毫秒）
QtCore.QTimer.singleShot(max(60000, (RUN_TIMEOUT + 30) * 1000),
                         lambda: (print("  超时"), app.exit(4)))
app.exec()
