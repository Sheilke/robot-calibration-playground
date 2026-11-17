#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""页面层冒烟：模块可导入、关键守卫函数行为正确、页面真能建起来。

真正的浏览器交互（点按钮、弹 toast、拉表格）靠 ``nicegui.testing.User`` 或
人工冒烟，这里不碰；但 ``render()`` 会**真的跑一遍**（无 websocket 的 client），
所以“元素定义顺序/闭包名字写错”这类只在建页面时才炸的问题会被钉住。
"""
import tempfile
import unittest

from laser_web import jobs
from laser_web.pages import (POINT_COUNT_WARN, adopt_running_job,
                             cancelled_result)


class PagesImportTest(unittest.TestCase):
    def test_pages_import_and_expose_render(self):
        from laser_web import app  # noqa: F401 - 页面注册依赖它
        from laser_web.pages import generate, repair, tools

        for module in (generate, repair, tools):
            self.assertTrue(callable(module.render), module.__name__)

    def test_warn_threshold_is_sane(self):
        """软上限要挡住"笔误几十万点"，但不能挡住正常交付的 50 点。"""
        self.assertGreater(POINT_COUNT_WARN, 50)
        self.assertLessEqual(POINT_COUNT_WARN, 20000)


class AdoptRunningJobTest(unittest.TestCase):
    """页面重建后要能接管在跑的任务（否则不可见、不可取消）。"""

    def test_returns_only_running_job_of_requested_kind(self):
        import threading
        import time

        with tempfile.TemporaryDirectory() as tmp:
            registry = jobs.JobRegistry(tmp, keep=5)
            started = threading.Event()

            def slow(job):
                started.set()
                time.sleep(0.4)
                return {"ok": True}

            job_id = registry.submit("scan", slow)
            self.assertTrue(started.wait(2.0))
            adopted = adopt_running_job(registry, "scan")
            self.assertIsNotNone(adopted)
            self.assertEqual(adopted.id, job_id)
            # 类型不匹配时不接管
            self.assertIsNone(adopt_running_job(registry, "repair"))

            time.sleep(0.8)
            self.assertIsNone(adopt_running_job(registry, "scan"))


class CancelledResultTest(unittest.TestCase):
    """取消是中性状态，不能画成红色"失败"（评审第 4 条）。"""

    def test_cancelled_shape(self):
        result = cancelled_result()
        self.assertFalse(result["ok"])
        self.assertTrue(result["cancelled"])
        self.assertEqual(result["warnings"], [])

    def test_generate_modes(self):
        from laser_web.pages import generate

        self.assertEqual(generate._result_mode(None), "empty")
        self.assertEqual(generate._result_mode(cancelled_result()),
                         "cancelled")
        self.assertEqual(generate._result_mode({"ok": False, "error": "x"}),
                         "failed")
        self.assertEqual(generate._result_mode({"ok": True}), "ok")

    def test_repair_modes(self):
        from laser_web.pages import repair

        self.assertEqual(repair._result_mode(None, None, None), "empty")
        self.assertEqual(repair._result_mode(None, None, {"error": "参数非法"}),
                         "locate_failed")
        self.assertEqual(repair._result_mode({"ok": True}, None, None), "ok")
        self.assertEqual(repair._result_mode(None, cancelled_result(), None),
                         "cancelled")
        self.assertEqual(
            repair._result_mode(None, {"ok": False, "error": "x"}, None),
            "failed")
        # 有报错又刚跑完一次失败：结果的优先级更高（结果区只能有一张卡）
        self.assertEqual(
            repair._result_mode(None, {"ok": False, "error": "x"},
                                {"error": "旧报错"}),
            "failed")


class WaypointRowDashTest(unittest.TestCase):
    """失败路点的关节角要显示 ``—``，而不是空白（评审第 7 条）。"""

    def _result(self, ok: bool) -> dict:
        return {"meta": {"skip_ik": False},
                "waypoints": [{"gp": 1, "pos": [1.0, 2.0, 3.0],
                               "q": [1.0] * 6 if ok else None,
                               "config": [1, 0, 0] if ok else None,
                               "j3_abs": 12.5 if ok else None,
                               "j5_abs": 30.0 if ok else None,
                               "dq_l2": 1.25 if ok else None,
                               "ok": ok}]}

    def test_failed_row_keeps_null_cells(self):
        from laser_web.pages import generate

        row = generate._waypoint_rows(self._result(False))[0]
        for key in ("q1", "q2", "q3", "q4", "q5", "q6", "j3", "j5", "dq_l2"):
            self.assertIsNone(row[key], key)
        self.assertEqual(row["cfg"], "—")
        self.assertTrue(row["failed"])

    def test_ok_row_stays_numeric(self):
        """成功的行仍是数字（点表头排序要按大小排，不是字典序）。"""
        from laser_web.pages import generate

        row = generate._waypoint_rows(self._result(True))[0]
        self.assertEqual(row["q1"], 1.0)
        self.assertEqual(row["j3"], 12.5)
        self.assertFalse(row["failed"])

    def test_slot_draws_dash_for_null(self):
        from laser_web.pages import generate

        slot = generate._FAILED_ROW_SLOT
        self.assertIn("'—'", slot)
        self.assertIn("null", slot)


class UploadLimitTest(unittest.TestCase):
    """工具页上传有大小上限，且服务端自己再判一次（评审第 6 条）。"""

    def test_limit_predicate(self):
        from laser_web.pages import tools

        self.assertEqual(tools.UPLOAD_MAX_BYTES, 8 * 1024 * 1024)
        self.assertFalse(tools._upload_too_large(None))       # 大小未知：不拦
        self.assertFalse(tools._upload_too_large(0))
        self.assertFalse(tools._upload_too_large(tools.UPLOAD_MAX_BYTES))
        self.assertTrue(tools._upload_too_large(tools.UPLOAD_MAX_BYTES + 1))


class PageRenderSmokeTest(unittest.TestCase):
    """三个页面的 ``render()`` 真的能建完（不连浏览器，但也不只是 import）。

    这一层能抓住"元素定义顺序/闭包名字写错"这类只在建页面时才炸的问题
    （例如把被 ``@ui.refreshable`` 函数引用的 helper 定义在它后面）。真的
    交互（点按钮、弹 toast）仍由人工冒烟确认。
    """

    def test_all_three_pages_build(self):
        import asyncio

        import nicegui

        from laser_web.pages import generate, repair, tools

        builders = (("/", generate), ("/repair", repair), ("/tools", tools))

        async def build_all() -> dict:
            from nicegui.client import Client
            from nicegui.page import page

            previous = nicegui.core.loop
            nicegui.core.loop = asyncio.get_running_loop()
            counts = {}
            try:
                for path, module in builders:
                    client = Client(page(path, title=path)(lambda: None))
                    with client:
                        module.render()
                    counts[module.__name__] = len(client.elements)
            finally:
                nicegui.core.loop = previous
            return counts

        counts = asyncio.run(build_all())
        for name, count in counts.items():
            self.assertGreater(count, 10, name)


if __name__ == "__main__":
    unittest.main()
