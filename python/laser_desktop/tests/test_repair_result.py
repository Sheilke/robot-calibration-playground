#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复页结果区（S4）：**构造**的 ``locate_segment`` / ``run_repair`` 返回值。

要验的是“把结果铺到界面上”这一层，不跑 IK：

* 判定徽标：``kind`` -> 中文名（``_KIND_LABEL``）、有效阈值、禁区数量；
* 段定位表：行数 = profiles 数、**报告段黄底**、越界段**红底**；
* 修复前后对比：``mode`` 徽标文案、边界表里变化的轴高亮、采样域缩小百分比、
  局部修复的 ``GP`` / ``delta_rpy`` / ``q`` 前后、``before/after`` 指标对照；
* 候选日志：行数、失败行红字；
* 计划：路点表 / 段诊断表 / 统计标签（复用生成页的渲染器）；
* 导出：``QFileDialog`` 选完目标后**复制**四个产物，源文件仍在。
"""

import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import generate_page, repair_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtGui, QtWidgets
from laser_web import jobs

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-repair-result-tests"])
    return _APP


def color_name(value: str) -> str:
    """十六进制色值 -> Qt 的规范名（``#fde8e8``），用于比较表格底色。"""
    return QtGui.QColor(value).name()


def make_classifications():
    """一个退化读数的分类行（字段与 ``core._classification_rows`` 对齐）。"""
    return [{
        "q_deg": [0.0, 0.0, 0.0, 55.721, 0.0, -55.721],
        "kind": "degenerate",
        "j3_abs": 42.639,
        "j5_abs": 0.0,
        "j5_floor": 75.721,
        "j3_floor": 5.0,
        "same_as_zero_pose": False,
        "note": "J5≈0 时 J4/J6 共线；该读数不能定位位置。",
        "position": [1000.0, 0.0, 600.0],
    }]


def make_profiles():
    """窗口内三段：报告段（index 3）安全、越界段（index 4）|J5| 太低。"""
    return [
        {"index": 2, "gp_from": 3, "gp_to": 4, "ok": True,
         "j3_abs_min": 51.2, "j5_abs_min": 60.5, "crossings": 0,
         "worst_frac": 0.25, "worst_pos": [950.0, -50.0, 550.0]},
        {"index": 3, "gp_from": 4, "gp_to": 5, "ok": True,
         "j3_abs_min": 42.95, "j5_abs_min": 53.09, "crossings": 0,
         "worst_frac": 0.5, "worst_pos": [1000.0, 0.0, 600.0]},
        {"index": 4, "gp_from": 5, "gp_to": 6, "ok": False,
         "j3_abs_min": 3.5, "j5_abs_min": 1.25, "crossings": 1,
         "worst_frac": 0.75, "worst_pos": [1050.0, 50.0, 650.0]},
    ]


def make_locate_result() -> dict:
    """造一份与 ``core.locate_segment`` 成功返回值同构的结果。"""
    profiles = make_profiles()
    return {
        "ok": True,
        "classifications": make_classifications(),
        "effective_thresholds": {"j5_min_deg": 75.721, "j3_min_deg": 5.0,
                                 "j3_seg_min_deg": 20.0},
        "forbidden_positions": [[1000.0, 0.0, 600.0]],
        "stopped_at": 4,
        "profiles": profiles,
        "offenders": [profiles[2]],
        "verdict": "报告段 GP5->GP6 越界：|J3|min=3.500°、|J5|min=1.250°。",
        "unmodified": {"ok": True, "reasons": []},
        "chain": {"j3_crossings": 1, "j5_crossings": 1, "bad_segments":
                  ["GP5->GP6"], "seg_j3_min": 3.5, "seg_j5_min": 1.25},
        "warnings": [],
    }


def make_stats(*, feasible=True) -> dict:
    """与 ``core._summary_stats`` 同键的 stats。"""
    return {"solved": 8, "total": 8, "switches": 0, "dq_l2_sum": 10400.563,
            "j5_abs_min": 53.136, "j3_abs_min": 42.64, "design_cond": 6.691,
            "post_ik_cond": None, "seg_j5_min": 53.088, "seg_j3_min": 42.64,
            "j3_crossings": 0, "j5_crossings": 0, "guard_violations": 0,
            "feasible": feasible}


def make_repair(*, mode="local", moved=False) -> dict:
    """造一份 ``core.run_repair()["repair"]``（mode=none/local/domain 都能拼）。"""
    before_bounds = [750.0, 1300.0, -600.0, 600.0, 270.0, 940.0]
    after_bounds = list(before_bounds)
    volume_loss = 0.0
    rpy_before = [3.0, 73.0, -167.0]
    rpy_after = list(rpy_before)
    local = None
    label = "无改动"
    if mode == "local":
        local = {"index": 3, "gp": 4,
                 "delta_rpy": [1.5, -2.0, 0.25],
                 "q_before": [10.0, -20.0, 30.0, 40.0, 50.0, 60.0],
                 "q_after": [11.0, -21.0, 31.0, 41.0, 49.0, 59.0]}
        label = "重采 GP4 的姿态"
    if mode == "domain" and moved:
        after_bounds = [750.0, 1300.0, -600.0, 520.0, 270.0, 940.0]  # Y 上界 -80
        volume_loss = 0.1234
        rpy_after = [3.0, 70.0, -167.0]
        label = "Y 上界 -80 mm"
    return {
        "mode": mode,
        "label": label,
        "bounds_before": before_bounds,
        "bounds_after": after_bounds,
        "volume_loss": volume_loss,
        "rpy_before": rpy_before,
        "rpy_after": rpy_after,
        "moves": ([{"axis": 3, "delta": -80.0}] if moved else []),
        "local": local,
        "log": [
            {"label": "原域", "tier": 0, "ok": False, "score": 100.0,
             "reasons": ["段内奇异 |J5|=1.25° < 20°"]},
            {"phase": "single", "label": "Y 上界 -80", "tier": 1, "ok": False,
             "score": 30.0, "reasons": ["条件数 30 > 20", "3 个路点无解"]},
            {"phase": "single", "label": "Y 上界 -160", "tier": 1, "ok": True,
             "score": 8.0, "reasons": []},
        ],
        "log_total": 3,
        "before_stats": make_stats(feasible=False),
        "after_stats": make_stats(feasible=True),
        "verdict": "报告段 GP5->GP6 越界：|J3|min=3.500°、|J5|min=1.250°。",
        "classifications": make_classifications(),
        "forbidden_positions": [[1000.0, 0.0, 600.0]],
        "profiles": make_profiles(),
        "offenders": [4],
        "stopped_at": 4,
        "effective_thresholds": {"j5_min_deg": 75.721, "j3_min_deg": 5.0,
                                 "j3_seg_min_deg": 20.0},
    }


def make_repair_result(run_dir: Path, *, mode="local", moved=False,
                       artifacts=True) -> dict:
    """造一份与 ``core.run_repair`` 同构的返回值（含 ``repair`` 块）。"""
    waypoints = [
        {"gp": 1, "index": 0, "pos": [900.0, -100.0, 500.0],
         "rpy_pert": [0.0, 0.0, 0.0], "q": [10.0, -20.0, 30.0, 40.0, 50.0, 60.0],
         "j3_abs": 42.64, "j5_abs": 53.14, "config": [0, 1, 0],
         "dq_l2": 0.0, "ok": True},
        {"gp": 2, "index": 1, "pos": [900.0, 100.0, 500.0],
         "rpy_pert": [1.0, 2.0, 3.0], "q": None, "j3_abs": None,
         "j5_abs": None, "config": None, "dq_l2": None, "ok": False},
    ]
    segments = [
        {"gp_from": 1, "gp_to": 2, "j3_abs_min": 42.64, "j5_abs_min": 53.09,
         "crosses": False, "worst_frac": 0.5,
         "worst_pos": [900.0, 0.0, 500.0]},
        {"gp_from": 2, "gp_to": 3, "j3_abs_min": 1.5, "j5_abs_min": 0.5,
         "crosses": True, "worst_frac": 0.25,
         "worst_pos": [950.0, 100.0, 550.0]},
    ]
    result = {"ok": True, "run_id": "20260101-000000-repair",
              "run_dir": str(run_dir), "stats": make_stats(),
              "meta": {"kind": "repair"},
              "waypoints": waypoints, "segments": segments,
              "warnings": ["局部修复：GP4 换成 delta_rpy=[1.5, -2.0, 0.25]"],
              "artifacts": {}, "repair": make_repair(mode=mode, moved=moved)}
    if artifacts:
        names = {"scan_json": "scan.json", "joints": "joints.txt",
                 "plot": "plot.png", "repair": "repair.json"}
        for key, name in names.items():
            (run_dir / name).write_text(f"{key} 的内容\n", encoding="utf-8")
        result["artifacts"] = dict(names)
    return result


class PageCase(unittest.TestCase):
    """共用：一个挂临时登记表的页面 + 一个临时 ``run_dir``。"""

    def setUp(self):
        app()
        self.run_dir = Path(tempfile.mkdtemp(prefix="repair-run-"))
        self.root = Path(tempfile.mkdtemp(prefix="repair-runs-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = repair_page.RepairPage(plot=None, bridge=self.bridge)

    def tearDown(self):
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=30)
        shutil.rmtree(self.run_dir, ignore_errors=True)
        shutil.rmtree(self.root, ignore_errors=True)


class LocateRenderTest(PageCase):
    """``locate_segment`` 的结果铺到「定位」页签。"""

    def test_kind_badges_and_threshold_chips(self):
        self.page.render_locate(make_locate_result())
        self.assertEqual([badge.text() for badge in self.page.kind_badges],
                         ["退化读数"])
        texts = [label.text() for label in self.page.threshold_labels]
        self.assertIn("有效阈值 |J5|≥75.72° · |J3|≥5.00°", texts)
        self.assertIn("段内 |J3|≥20.00°", texts, "有段内肘部裕度时要单独显示")
        self.assertIn("位置禁区 1 个", texts)
        # 判定说明：**只有**算法给的 note（页面不再复述一遁），另加一句引导语
        hints = "\n".join(label.text() for label in self.page.hint_labels)
        self.assertIn(repair_page.VERDICT_GUIDE, hints)
        self.assertIn("J4/J6 共线", hints)
        self.assertIn("报告段 GP5->GP6 越界", self.page.verdict_label.text())
        self.assertIn("color:#8a6d3b", self.page.verdict_label.styleSheet(),
                      "越界结论不该用“没问题”的绿色")

    @staticmethod
    def verdict_texts(page) -> list:
        """判定区里所有 ``QLabel`` 的文本（徽标容器里的也算）。"""
        texts = []

        def walk(layout):
            for index in range(layout.count()):
                widget = layout.itemAt(index).widget()
                if isinstance(widget, QtWidgets.QLabel):
                    texts.append(widget.text())
                elif widget is not None and widget.layout() is not None:
                    walk(widget.layout())

        walk(page.verdict_holder.layout())
        return texts

    def test_explanation_is_shown_only_once(self):
        """S4 评审：页面自己的解释 + ``note`` 说的是同一件事，只许留一处。

        ``note`` 留下了（它有数据来源），页面那句改成了 :data:`VERDICT_GUIDE`
        的一句引导语——两句话的关键词各只出现一次。
        """
        self.page.render_locate(make_locate_result())
        joined = "\n".join(self.verdict_texts(self.page))
        self.assertEqual(joined.count("J4/J6 共线"), 1, joined)
        self.assertEqual(joined.count(repair_page.VERDICT_GUIDE), 1, joined)
        # 页面不复述算法的话（那句以“退化读数：J5≈0 …”开头的长解释已删）
        self.assertNotIn("退化读数：J5≈0", joined)

    def test_unknown_kind_falls_back_to_the_raw_text_and_a_grey_badge(self):
        """算法新加的判定：徽标显示原文 + 灰色，不抛异常（S4 评审第 8 条）。"""
        result = make_locate_result()
        result["classifications"] = [{
            "q_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            "kind": "weird",
            "j3_abs": 12.5,
            "j5_abs": 45.0,
            "note": "算法新加的判定：本页还不认识它。",
            "position": [0.0, 0.0, 0.0],
        }]
        self.page.render_locate(result)          # 不抛异常即是本节的一半
        self.assertEqual([badge.text() for badge in self.page.kind_badges],
                         ["weird"], "未知判定要原样显示，不吞信息")
        self.assertIn("background:#9e9e9e",
                      self.page.kind_badges[0].styleSheet(), "未知判定用灰徽标")
        self.assertIn("算法新加的判定：本页还不认识它。",
                      "\n".join(label.text()
                               for label in self.page.hint_labels))
        self.assertIn("q=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]",
                      "\n".join(label.text()
                               for label in self.page.detail_labels))
        # 纯逻辑那一层也跟着兼容
        self.assertEqual(repair_page.kind_text("weird"), "weird")

    def test_verdict_uses_green_when_nothing_needs_to_change(self):
        result = make_locate_result()
        result.update({"verdict": "窗口内各段都满足阈值，且整链无越界：不动采样域。",
                       "offenders": [], "profiles": []})
        self.page.render_locate(result)
        self.assertIn("color:#1e8449", self.page.verdict_label.styleSheet())
        self.assertEqual(self.page.profile_table.rowCount(), 0)
        self.assertIn("不动采样域", self.page.status_label.text())

    def test_profile_rows_reported_and_offender_highlights(self):
        result = make_locate_result()
        self.page.render_locate(result)
        table = self.page.profile_table
        self.assertEqual(table.rowCount(), len(result["profiles"]))
        self.assertEqual(table.columnCount(),
                         len(repair_page.PROFILE_HEADERS))

        reported = color_name(repair_page._REPORTED_BG)
        offender = color_name(repair_page._OFFENDER_BG)
        # 第 2 行是 index 3 == stopped_at - 1：报告段（黄底）
        self.assertEqual(table.item(1, 0).text(), "GP04 → GP05")
        self.assertEqual(table.item(1, 0).background().color().name(), reported)
        # 第 3 行是 offender（红底 + 深红字）
        self.assertEqual(table.item(2, 0).text(), "GP05 → GP06")
        self.assertEqual(table.item(2, 0).background().color().name(), offender)
        self.assertEqual(table.item(2, 0).foreground().color().name(),
                         color_name(repair_page._OFFENDER_FG))
        # 第 1 行既不是报告段也不是 offender：没有底色
        self.assertNotEqual(table.item(0, 0).background().color().name(),
                            reported)
        self.assertNotEqual(table.item(0, 0).background().color().name(),
                            offender)
        # 穿零数 / 最差点 / t 三列真的填了
        self.assertEqual(table.item(2, 3).text(), "1")
        self.assertEqual(table.item(2, 4).text(), "1050.0, 50.0, 650.0")
        self.assertEqual(table.item(2, 5).text(), "0.750")

    def test_offender_row_wins_when_it_is_also_the_reported_one(self):
        """报告段本身越界时：红底优先（与网页版行模板的优先级一致）。"""
        result = make_locate_result()
        result["offenders"] = [result["profiles"][1]]
        self.page.render_locate(result)
        table = self.page.profile_table
        self.assertEqual(table.item(1, 0).background().color().name(),
                         color_name(repair_page._OFFENDER_BG))

    def test_locate_failure_is_red_and_keeps_the_text(self):
        self.page.render_locate({"ok": False, "error": "参数校验失败：示例原文",
                                 "warnings": ["警告一"]})
        self.assertEqual(self.page.profile_table.rowCount(), 0)
        self.assertEqual(self.page.kind_badges, [])
        text = self.page.status_label.text()
        self.assertIn("参数校验失败：示例原文", text)
        self.assertIn("· 警告一", text)
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())


class RepairRenderTest(PageCase):
    """``run_repair`` 的结果铺到「修复」「计划」页签。"""

    def test_local_mode_badge_volume_and_local_block(self):
        result = make_repair_result(self.run_dir, mode="local")
        self.page.render_result(result)

        self.assertEqual(self.page.mode_badge.text(), "局部修复（只换端点姿态）")
        self.assertFalse(self.page.mode_badge.isHidden())
        self.assertEqual(self.page.label_value.text(), "重采 GP4 的姿态")
        self.assertEqual(self.page.volume_label.text(), "采样域缩小 0.00%")

        # 局部修复：GP / delta_rpy / q 前后
        self.assertFalse(self.page.local_group.isHidden())
        self.assertIn("GP4", self.page.local_labels["gp"].text())
        self.assertEqual(self.page.local_labels["delta_rpy"].text(),
                         "+1.50, -2.00, +0.25")
        self.assertEqual(self.page.local_labels["q_before"].text(),
                         "10.00, -20.00, 30.00, 40.00, 50.00, 60.00")
        self.assertEqual(self.page.local_labels["q_after"].text(),
                         "11.00, -21.00, 31.00, 41.00, 49.00, 59.00")
        # 本地修复不动边界：6 行全都不是高亮色
        table = self.page.bound_table
        self.assertEqual(table.rowCount(), 6)
        changed = color_name(repair_page._CHANGED_BG)
        for row in range(6):
            self.assertNotEqual(table.item(row, 0).background().color().name(),
                                changed, f"第 {row} 行不该高亮")
        self.assertIn("（未改）", self.page.rpy_label.text())

    def test_domain_mode_highlights_changed_axis_and_volume_loss(self):
        result = make_repair_result(self.run_dir, mode="domain", moved=True)
        self.page.render_result(result)

        self.assertEqual(self.page.mode_badge.text(), "整体挪动采样域")
        self.assertEqual(self.page.volume_label.text(), "采样域缩小 12.34%")
        table = self.page.bound_table
        changed = color_name(repair_page._CHANGED_BG)
        highlighted = [row for row in range(6)
                       if table.item(row, 0).background().color().name() == changed]
        self.assertEqual(highlighted, [3], "只该高亮 Y 上界那一行")
        self.assertEqual(table.item(3, 0).text(), "Y 上界")
        self.assertEqual(table.item(3, 1).text(), "600.0")
        self.assertEqual(table.item(3, 2).text(), "520.0")
        self.assertEqual(table.item(3, 3).text(), "-80.0")
        # 域被挪过：参考姿态块给出前后
        self.assertIn("→", self.page.rpy_label.text())
        # 没有 local 块
        self.assertTrue(self.page.local_group.isHidden())

    def test_before_after_stats_table(self):
        self.page.render_result(make_repair_result(self.run_dir))
        table = self.page.stat_table
        self.assertEqual(table.rowCount(), 9)
        items = [table.item(row, 0).text() for row in range(9)]
        self.assertEqual(items[0], "解出路点")
        self.assertIn("段内 |J5|min °", items)
        self.assertIn("穿零 J3 / J5", items)
        self.assertEqual(items[-1], "可行")
        # 修复前不可行 -> 否；修复后可行 -> 是
        self.assertEqual(table.item(0, 1).text(), "8/8")
        self.assertEqual(table.item(8, 1).text(), "否")
        self.assertEqual(table.item(8, 2).text(), "是")
        self.assertEqual(table.item(1, 1).text(), "53.136")
        self.assertEqual(table.item(1, 2).text(), "53.136")

    def test_candidate_log_rows_and_failed_marker(self):
        self.page.render_result(make_repair_result(self.run_dir))
        table = self.page.log_table
        self.assertEqual(table.rowCount(), 3)
        self.assertEqual(table.columnCount(), len(repair_page.LOG_HEADERS))
        self.assertEqual(table.item(0, 1).text(), "原域")     # 阶段缺省写原域
        self.assertEqual(table.item(0, 2).text(), "✗")
        self.assertEqual(table.item(1, 1).text(), "Y 上界 -80")
        self.assertEqual(table.item(1, 3).text(), "30.00")
        self.assertIn("条件数", table.item(1, 4).text())
        self.assertEqual(table.item(2, 2).text(), "✓")
        self.assertEqual(table.item(2, 4).text(), "—")
        # 失败行红字（成功行没有）
        failed = color_name(repair_page._OFFENDER_FG)
        self.assertEqual(table.item(0, 0).foreground().color().name(), failed)
        self.assertNotEqual(table.item(2, 0).foreground().color().name(), failed)
        self.assertIn("候选日志（3/3 条", self.page.log_caption.text())

    def test_plan_tables_stat_labels_and_figure(self):
        result = make_repair_result(self.run_dir)
        figures = []

        class StubPlot(QtWidgets.QWidget):
            def set_figure(self, figure):
                figures.append(figure)

        page = repair_page.RepairPage(plot=StubPlot(), bridge=self.bridge)
        try:
            page.render_result(result)
            self.assertEqual(page.waypoint_table.rowCount(), 2)
            self.assertEqual(page.segment_table.rowCount(), 2)
            self.assertEqual(page.stat_labels["solved"].text(), "成功 8/8")
            self.assertEqual(page.stat_labels["feasible"].text(), "可行")
            self.assertIn(result["run_id"], page.run_label.text())
            # 失败路点整行红底（复用生成页的渲染器）
            self.assertEqual(
                page.waypoint_table.item(1, 0).background().color().name(),
                color_name(generate_page._FAILED_BG))
            # 修复后的计划图（复用 result_figure -> 网页版 figures）
            self.assertEqual(len(figures), 1)
            self.assertIs(page.last_figure, figures[-1])
            self.assertIn("成功", str(figures[-1].data[0].name))
            # 导出按钮：四个产物都在，全开
            self.assertEqual(set(page.export_buttons), {"scan_json", "joints",
                                                        "plot", "repair"})
            self.assertTrue(all(button.isEnabled()
                                for button in page.export_buttons.values()))
            # 「计划」页签里能看到不可行/备注行
            self.assertIn("局部修复", page.plan_status_label.text())
        finally:
            page.shutdown()
            page.deleteLater()

    def test_export_buttons_disabled_without_artifacts(self):
        self.page.render_result(make_repair_result(self.run_dir, artifacts=False))
        self.assertFalse(any(button.isEnabled()
                             for button in self.page.export_buttons.values()))

    def test_none_mode_replaces_the_bound_table_with_one_sentence(self):
        """``mode=none`` = 整份没动：前后必然相同的 6 行数字换成一句话（S4 第 5 条）。"""
        result = make_repair_result(self.run_dir, mode="none")
        self.page.render_result(result)
        self.assertEqual(self.page.mode_badge.text(), "不动采样域")
        self.assertTrue(self.page.bound_table.isHidden())
        self.assertFalse(self.page.bound_note.isHidden())
        self.assertEqual(self.page.bound_note.text(),
                         repair_page.BOUND_UNCHANGED_TEXT)
        self.assertEqual(self.page.bound_table.rowCount(), 0,
                         "不摆 6 行纯占位数字")

    def test_other_modes_still_show_the_bound_table(self):
        """只有 none 才换：local / domain 的边界表（与变化高亮）照旧。"""
        self.page.render_result(make_repair_result(self.run_dir, mode="none"))
        self.page.render_repair(make_repair(mode="local"))
        self.assertFalse(self.page.bound_table.isHidden())
        self.assertTrue(self.page.bound_note.isHidden())
        self.assertEqual(self.page.bound_table.rowCount(), 6)

    def test_infeasible_result_still_renders_the_repair_block(self):
        """所有候选都不可行时 core 也返回 ``repair``（复盘用）：对比 + 日志要画。"""
        result = make_repair_result(self.run_dir, artifacts=False)
        result.update({"ok": False, "error": "所有候选都不可行：段内奇异"})
        self.page._on_finished({"state": "done", "result": result, "error": None,
                                "run_id": result["run_id"],
                                "run_dir": result["run_dir"]})
        self.assertEqual(self.page.run_state, "done")
        self.assertIn("所有候选都不可行", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertEqual(self.page.log_table.rowCount(), 3)
        self.assertEqual(self.page.bound_table.rowCount(), 6)

    def test_infeasible_result_takes_over_the_run_directory(self):
        """Q1：``ok=False`` 但 ``repair.json`` 在盘上 → 导出入口 + 真实运行目录。

        ``core`` 的原文就写着“详见 repair.json”，界面却把 ````last_result``
        扔掉、四个导出按钮全灰、还说“还没有运行目录”——现场无处可看。
        """
        result = make_repair_result(self.run_dir, mode="domain", moved=True)
        result.update({"ok": False,
                       "error": "所有候选都不可行（详见 repair.json）",
                       "artifacts": {"repair": "repair.json"}})
        for name in ("scan.json", "joints.txt", "plot.png"):
            (self.run_dir / name).unlink()      # 不可行时 core 只落 repair.json

        self.page._on_finished({"state": "done", "result": result, "error": None,
                                "run_id": result["run_id"],
                                "run_dir": result["run_dir"]})

        self.assertEqual(self.page.last_result, result, "结果要接住（Q1）")
        self.assertIn(result["run_id"], self.page.run_label.text())
        self.assertIn(str(self.run_dir), self.page.run_label.text())
        self.assertNotIn("还没有运行目录", self.page.run_label.text())
        enabled = {key: button.isEnabled()
                  for key, button in self.page.export_buttons.items()}
        self.assertEqual(enabled, {"scan_json": False, "joints": False,
                                  "plot": False, "repair": True},
                         "只有真实存在的 repair.json 该亮")
        self.assertIn("详见 repair.json", self.page.status_label.text())

    def test_cancelled_result_offers_the_kept_repair_json(self):
        """取消同样要接管：提示语自己说“已写出 repair.json 的目录会保留”。"""
        (self.run_dir / "repair.json").write_text("{}\n", encoding="utf-8")
        self.page._on_finished({"state": "cancelled", "result": None,
                                "error": None, "run_id": "20260101-000000-repair",
                                "run_dir": str(self.run_dir)})
        self.assertIsNotNone(self.page.last_result)
        self.assertIn("20260101-000000-repair", self.page.run_label.text())
        self.assertTrue(self.page.export_buttons["repair"].isEnabled())
        self.assertFalse(self.page.export_buttons["scan_json"].isEnabled())
        self.assertIn("已取消", self.page.status_label.text())


class DataGapTest(PageCase):
    """缺数据的显示：不把“没数据”说成 0，也不把缺键变成崩溃（S4 建议 2/3/4）。"""

    def test_missing_volume_loss_is_a_dash_not_zero(self):
        repair = make_repair(mode="domain", moved=True)
        repair["volume_loss"] = None
        self.page.render_repair(repair)
        self.assertEqual(self.page.volume_label.text(), "采样域缩小 —")
        self.assertNotIn("0.00%", self.page.volume_label.text())
        repair.pop("volume_loss")               # 键干脆不存在也一样
        self.page.render_repair(repair)
        self.assertEqual(self.page.volume_label.text(), "采样域缩小 —")

    def test_zero_volume_loss_still_reads_zero(self):
        """真的 0 损失还是 0.00%（“没数据”与“零损失”不能混为一谈）。"""
        self.page.render_repair(make_repair(mode="local"))
        self.assertEqual(self.page.volume_label.text(), "采样域缩小 0.00%")

    def test_missing_delta_rpy_does_not_crash(self):
        repair = make_repair(mode="local")
        repair["local"] = {"index": 3, "gp": 4, "q_before": None,
                           "q_after": None}      # 没有 delta_rpy 这个键
        self.page.render_repair(repair)
        self.assertEqual(self.page.local_labels["delta_rpy"].text(), "—")
        self.assertEqual(self.page.local_labels["q_before"].text(), "—")

    def test_null_log_total_is_not_printed(self):
        repair = make_repair()
        repair["log_total"] = None               # 键存在但为 None
        self.page.render_repair(repair)
        caption = self.page.log_caption.text()
        self.assertNotIn("None", caption)
        self.assertIn("候选日志（3/3 条", caption)


class ResetResultAreaTest(PageCase):
    """Q3：起新任务时结果区（含图）不能留着上一次的东西冒充新结果。"""

    def test_new_task_clears_the_figure_and_pushes_a_placeholder(self):
        figures = []

        class StubPlot(QtWidgets.QWidget):
            def set_figure(self, figure):
                figures.append(figure)

        page = repair_page.RepairPage(plot=StubPlot(), bridge=self.bridge)
        try:
            page.render_result(make_repair_result(self.run_dir))
            self.assertIsNotNone(page.last_figure)
            old = page.last_figure
            self.assertEqual(len(figures), 1)

            page._reset_result_area()

            self.assertIsNone(page.last_figure, "上一次的图必须清掉（Q3）")
            self.assertEqual(len(figures), 2, "画布要被换成“正在修复”的说明图")
            self.assertIsNot(figures[-1], old)
            self.assertEqual(figures[-1].layout.title.text, "路点预览（未出图）")
            self.assertIn("正在修复", figures[-1].layout.annotations[0].text)
            # 切页回来重推的也不是旧图；任务在跑时说“正在修复”而不是“还没有结果”
            self.assertIsNot(page.current_figure(), old)
            page.run_state = "running"
            self.assertIn("正在修复",
                          page.current_figure().layout.annotations[0].text)
        finally:
            page.shutdown()
            page.deleteLater()


class ExportTest(PageCase):
    """导出：``QFileDialog`` 选目标 -> 复制（源文件必须还在）。"""

    def setUp(self):
        super().setUp()
        self.target_dir = Path(tempfile.mkdtemp(prefix="repair-export-"))
        self.page.render_result(make_repair_result(self.run_dir))

    def tearDown(self):
        shutil.rmtree(self.target_dir, ignore_errors=True)
        super().tearDown()

    def _export(self, key: str, target: Path):
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=(str(target), "")):
            self.page._on_export(key)

    def test_all_four_artifacts_are_copied_byte_for_byte(self):
        for key, name in (("scan_json", "scan.json"), ("joints", "joints.txt"),
                          ("plot", "plot.png"), ("repair", "repair.json")):
            target = self.target_dir / f"导出的-{name}"
            self._export(key, target)
            source = self.run_dir / name
            self.assertTrue(target.is_file(), f"{name} 没复制出来")
            self.assertEqual(target.read_bytes(), source.read_bytes())
            self.assertTrue(source.is_file(), f"{name} 被搬走了（runs/ 是归档）")
            self.assertIn("已导出", self.page.last_status)

    def test_default_filename_carries_the_run_id(self):
        result = self.page.last_result
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=("", "")) as dialog:
            self.page._on_export("repair")
        default = Path(dialog.call_args[0][2]).name
        self.assertEqual(default, f"{result['run_id']}-repair.json")

    def test_cancelled_dialog_writes_nothing(self):
        with mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                               return_value=("", "")):
            self.page._on_export("scan_json")
        self.assertEqual(list(self.target_dir.iterdir()), [])
        self.assertNotIn("已导出", self.page.last_status)

    def test_missing_suffix_is_appended(self):
        target = self.target_dir / "没有扩展名"
        self._export("plot", target)
        self.assertTrue((self.target_dir / "没有扩展名.png").is_file())

    def test_missing_source_file_reports_red(self):
        (self.run_dir / "repair.json").unlink()
        self._export("repair", self.target_dir / "repair.json")
        self.assertFalse((self.target_dir / "repair.json").exists())
        self.assertIn("源文件不存在", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())


if __name__ == "__main__":
    unittest.main()
