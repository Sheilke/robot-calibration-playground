#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""修复页真跑（S4）：真定位一次 + 真修复一次（走 ``JobBridge``）。

两个真案子都基于 8 点小案子（域 ``900..1100 / -100..100 / 500..700``、
``target_count 8``、``rpy 3 73 -167``、``euler xyz-intrinsic``、
``q-seed 43,-30,-36,42,-89,-1``）：

1. **定位**：``singular_q=[[0,0,0,55.721,0,-55.721]]`` + ``stopped_at=4``，
   实测约 0.2 s。断言 ``ok``、``kind == "degenerate"``、``verdict`` 非空、
   段表行数落在窗口 ``2*window+1`` 之内；
2. **修复**：同一个域不填「已走到第几点」（原域可行 → ``mode == "none"``），
   实测 0.5 s 内跑完。断言 ``finished.state == "done"``、页面显示
   ``mode``、四个产物按钮全开、进度条到 100%。

线程安全沿用 S3 的判据：``sys.excepthook`` / ``threading.excepthook`` 抓到的
异常必须为空，且 ``finished`` 信号在**主线程**发出（跨线程碰控件会在这里炸）。
"""

import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import repair_page
from laser_desktop.job_bridge import JobBridge
from laser_desktop.qt_compat import QtGui, QtWidgets
from laser_web import jobs

#: 真跑的上限（秒）：两个案子实测都在 1 s 内，给足余量（慢机器/CI）。
TIMEOUT = 180.0

#: 现场读数（2026-11-05 的退化读数：J5≈0，J4/J6 共线）。
SINGULAR_Q = [0.0, 0.0, 0.0, 55.721, 0.0, -55.721]

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-repair-run-tests"])
    return _APP


def wait_until(predicate, timeout: float = TIMEOUT) -> bool:
    """转事件循环直到 ``predicate()`` 为真（超时返回 False）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app().processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    app().processEvents()
    return bool(predicate())


def configure_small_case(page, *, stopped_at=0) -> None:
    """把表单改成 8 点小案子（并填一组奇异读数）。"""
    fields = page.fields
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
    for joint, value in enumerate(SINGULAR_Q):
        page.q_rows[0]["fields"][joint].setValue(value)
    fields["stopped_at"].setValue(stopped_at)


class RealRunTest(unittest.TestCase):
    """真定位 + 真修复：从按钮到结果表、图、导出、``runs/<id>/`` 全链路。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="repair-real-"))
        self.registry = jobs.JobRegistry(root=self.root)
        self.bridge = JobBridge(registry=self.registry, flush_ms=50)
        self.page = repair_page.RepairPage(plot=None, bridge=self.bridge)
        # 跨线程异常会进 excepthook：一个都不许有
        self.caught = []
        self._old_excepthook = sys.excepthook
        self._old_thread_hook = threading.excepthook
        sys.excepthook = lambda *args: self.caught.append(args)
        threading.excepthook = lambda args: self.caught.append(args)

    def tearDown(self):
        sys.excepthook = self._old_excepthook
        threading.excepthook = self._old_thread_hook
        self.page.shutdown()
        self.page.deleteLater()
        self.registry.shutdown(timeout=TIMEOUT)
        shutil.rmtree(self.root, ignore_errors=True)

    # ---------------------------------------------------------------- 定位
    def test_real_locate_on_the_eight_point_case(self):
        configure_small_case(self.page, stopped_at=4)
        self.assertEqual(self.page.total.count, 8,
                         f"表单没落到 8 点：{self.page.total}")
        result = self.page.locate()

        self.assertTrue(result["ok"], f"定位失败：{result.get('error')}")
        self.assertEqual([row["kind"] for row in result["classifications"]],
                         ["degenerate"])
        self.assertTrue(str(result["verdict"]).strip(), "结论不能是空串")
        self.assertEqual(result["stopped_at"], 4)
        window = int(self.page.params()["segment_window"])
        # 8 点 + 报告点 4 → 窗口内 7 段（2*window+1 是上限）
        self.assertEqual(len(result["profiles"]), 2 * window + 1)
        self.assertEqual(self.page.profile_table.rowCount(),
                         len(result["profiles"]))
        self.assertLessEqual(self.page.profile_table.rowCount(), 2 * window + 1)
        # 判定徽标 / 阈值 / 结论都画出来了
        self.assertEqual([badge.text() for badge in self.page.kind_badges],
                         ["退化读数"])
        self.assertTrue(self.page.threshold_labels)
        self.assertIn(result["verdict"], self.page.verdict_label.text())
        self.assertIn("定位完成", self.page.status_label.text())
        # 报告段（stopped_at - 1 == 3）必须是黄底那一行
        reported = QtGui.QColor(repair_page._REPORTED_BG).name()
        rows = [index for index in range(self.page.profile_table.rowCount())
                if self.page.profile_table.item(index, 0)
                .background().color().name() == reported]
        self.assertEqual(len(rows), 1, f"报告段高亮行数不对：{rows}")
        self.assertIn("GP04", self.page.profile_table.item(rows[0], 0).text())
        # 定位不改参数、不落盘
        self.assertFalse(
            any(self.root.iterdir()), "定位段不该创建运行目录")
        # 等待光标已经恢复
        self.assertIsNone(QtWidgets.QApplication.overrideCursor())
        self.assertEqual(self.caught, [], f"有异常：{self.caught}")

    def test_locate_is_refused_while_a_repair_is_running(self):
        """修复任务在跑时「定位段」不许插进来（web 也是这个语义）。"""
        configure_small_case(self.page)
        job_id = self.page.start_repair()
        self.assertIsNotNone(job_id)
        self.assertIsNone(self.page.locate(), "任务在跑时不该同步跑定位")
        self.assertIn("修复任务在跑", self.page.last_status)
        self.assertTrue(wait_until(
            lambda: self.page.run_state in jobs.TERMINAL_STATES),
            "修复没结束")

    # ---------------------------------------------------------------- 修复
    def test_real_repair_through_the_bridge(self):
        configure_small_case(self.page)          # 不给 stopped_at：只判整链
        self.assertIsNone(self.page.params()["stopped_at"])
        finished_on = []
        payloads = []
        self.bridge.finished.connect(
            lambda payload: (finished_on.append(threading.current_thread()),
                             payloads.append(payload)))

        job_id = self.page.start_repair()
        self.assertIsNotNone(job_id, "start_repair 没起任务")
        self.assertEqual(self.page.run_state, "running")
        self.assertFalse(self.page.cancel_button.isHidden())
        self.assertFalse(self.page.repair_button.isEnabled())

        self.assertTrue(
            wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
            f"修复没在 {TIMEOUT}s 内结束：state={self.page.run_state} "
            f"进度={self.page.progress_bar.format()}")
        self.assertEqual(self.page.run_state, "done",
                         f"没跑完：{self.page.status_label.text()}")
        self.assertEqual(len(payloads), 1, "finished 只该发一次")
        self.assertEqual(payloads[0]["state"], "done")
        self.assertEqual(payloads[0]["run_id"], job_id)

        result = self.page.last_result
        self.assertIsNotNone(result, "没有结果")
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["run_id"], job_id)
        repair = result["repair"]
        # 原域本来就可行 → core 选 none；页面显示同一个 mode
        self.assertEqual(repair["mode"], "none")
        self.assertEqual(self.page.mode_badge.text(),
                         repair_page.MODE_TEXT[repair["mode"]])
        self.assertIn(repair_page.MODE_TEXT["none"], self.page.status_label.text())
        self.assertIn("color:#1e8449", self.page.status_label.styleSheet())

        # 计划页签：统计 / 路点表 / 段诊断表 / 图
        self.assertEqual(self.page.waypoint_table.rowCount(), 8)
        self.assertEqual(self.page.segment_table.rowCount(), 7)
        self.assertEqual(self.page.stat_labels["solved"].text(), "成功 8/8")
        self.assertEqual(self.page.stat_labels["feasible"].text(), "可行")
        self.assertIsNotNone(self.page.last_figure)
        self.assertIn("成功 8 点", str(self.page.last_figure.data[0].name))

        # 进度条满 + 取消按钮收起 + 按钮重新可用
        self.assertEqual(self.page.progress_bar.value(), 1000)
        self.assertIn("完成", self.page.progress_bar.format())
        self.assertTrue(self.page.cancel_button.isHidden())
        self.assertTrue(self.page.repair_button.isEnabled())
        self.assertTrue(self.page.locate_button.isEnabled())

        # 四个产物真的落盘，按钮全开
        run_dir = Path(result["run_dir"])
        for name in ("scan.json", "joints.txt", "plot.png", "repair.json"):
            self.assertTrue((run_dir / name).is_file(), f"{name} 没落盘")
        self.assertTrue(all(button.isEnabled()
                            for button in self.page.export_buttons.values()))
        payload = json.loads((run_dir / "repair.json").read_text(encoding="utf-8"))
        # repair.json 里两处 label 不同口径：decision 是“选中的那个域”（这里是原域
        # “原域”），repair.label 是页面徽标文案（mode=none → “无改动”）
        self.assertEqual(payload["decision"]["label"], "原域")
        self.assertEqual(payload["decision"]["tier"], 0)
        self.assertEqual(payload["decision"]["bounds"],
                         [round(value, 3) for value in repair["bounds_after"]])
        self.assertEqual(payload["repair"]["mode"], "none")
        self.assertEqual(payload["repair"]["label"], repair["label"])

        # 线程安全：finished 在主线程，异常清单为空
        self.assertEqual({thread is threading.main_thread()
                          for thread in finished_on}, {True})
        self.assertEqual(self.caught, [], f"有异常：{self.caught}")

    def test_validation_error_from_core_lands_in_the_result_area(self):
        """参数非法：core 的报错原文进结果区（红字），不是弹窗。"""
        configure_small_case(self.page, stopped_at=8)   # 8 点案子只能 1..7
        job_id = self.page.start_repair()
        self.assertIsNotNone(job_id)
        self.assertTrue(
            wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
            "任务没结束")
        self.assertIn("1..7", self.page.status_label.text())
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        # 参数非法也是 ok=False：结果照样接住（Q1 同一条分支），但这次真的
        # 没有运行目录——那就不该点亮任何导出按钮，run_label 也照实说。
        self.assertIsNotNone(self.page.last_result)
        self.assertIn("1..7", self.page.last_result["error"])
        self.assertFalse(any(button.isEnabled()
                             for button in self.page.export_buttons.values()))
        self.assertIn("还没有运行目录", self.page.run_label.text())

    def test_infeasible_repair_keeps_its_repair_json_exportable(self):
        """Q1 实测：所有候选都不可行时 ``core`` 只落 ``repair.json``，界面要接住。

        构造方式（实测 0.5 s）：``j5_min=90`` 让原域与所有候选的 |J5| 都过不了，
        再配 ``max_candidates=1`` 把整体搜索压成一个候选 → 必然
        “所有候选都不可行（详见 repair.json）”。

        旧行为：``last_result=None``、四个导出按钮全灰、``run_label`` 仍写
        “还没有运行目录”——错误原文让你去看 repair.json，却不给入口。
        """
        configure_small_case(self.page)              # 不给 stopped_at：整链判
        self.page.fields["j5_min"].setValue(90.0)
        self.page.fields["max_candidates"].setValue(1)
        self.assertTrue(self.page.repair_button.isEnabled(),
                        "参数合法且点数没超上限，按钮该是可点的")

        job_id = self.page.start_repair()
        self.assertIsNotNone(job_id)
        self.assertTrue(
            wait_until(lambda: self.page.run_state in jobs.TERMINAL_STATES),
            f"修复没在 {TIMEOUT}s 内结束：{self.page.progress_bar.format()}")
        self.assertEqual(self.page.run_state, "done")

        result = self.page.last_result
        self.assertIsNotNone(result, "ok=False 的结果也要接住（Q1）")
        self.assertIs(result["ok"], False, result.get("error"))
        self.assertIn("所有候选都不可行", result["error"])
        self.assertIn("repair.json", result["error"],
                      "原文让现场看 repair.json")
        self.assertEqual(result["run_id"], job_id)

        # 盘上真的有 repair.json（这才是“详见 repair.json”的前提）
        run_dir = Path(result["run_dir"])
        self.assertTrue((run_dir / "repair.json").is_file(),
                        sorted(path.name for path in run_dir.iterdir()))

        # 运行目录：真实 run_id + 目录，不再谎称“还没有运行目录”
        self.assertIn(job_id, self.page.run_label.text())
        self.assertIn(str(run_dir), self.page.run_label.text())
        self.assertNotIn("还没有运行目录", self.page.run_label.text())

        # 导出：只有真存在的 repair.json 该亮
        enabled = {key: button.isEnabled()
                   for key, button in self.page.export_buttons.items()}
        self.assertEqual(enabled,
                         {"scan_json": False, "joints": False,
                          "plot": False, "repair": True})

        # 报错原文红字 + 建议 6 的参数指纹（能对回提交那一版）
        self.assertIn("color:#c0392b", self.page.status_label.styleSheet())
        self.assertIn("参数（提交时）", self.page.status_label.text())
        self.assertIn("j5_min=90.0", self.page.status_label.text())
        # 不可行也把候选日志画出来（复盘用）
        self.assertGreater(self.page.log_table.rowCount(), 0)
        self.assertEqual(self.caught, [], f"有异常：{self.caught}")


if __name__ == "__main__":
    unittest.main()
