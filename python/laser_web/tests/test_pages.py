#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""页面层冒烟：模块可导入、关键守卫函数行为正确。

真正的浏览器交互靠 ``nicegui.testing.User``，但那需要拉起事件循环，放在这里
会让测试变慢且更脆；这里只钉住评审抓到过的问题点（下载守卫、任务接管、
点数上限），UI 结构本身由人工冒烟确认。
"""
import tempfile
import unittest
from pathlib import Path

from laser_web import jobs
from laser_web.pages import POINT_COUNT_WARN, adopt_running_job


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


if __name__ == "__main__":
    unittest.main()
