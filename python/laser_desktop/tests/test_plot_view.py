#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plotly 壳页与图负载：离线、可复用、不引用 CDN。

真窗口渲染另有人工冒烟脚本（``laser_desktop/smoke_offscreen.py``）；这里只钉
**不依赖显示器**的那部分，它们恰恰是最容易悄悄坏掉的（壳页被改成 CDN、
plotly.min.js 被反复重写、numpy 数据没转换导致 json 抛异常）。
"""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from laser_desktop import paths, plot_view


class ShellTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.patch = mock.patch.object(paths, "cache_root", return_value=self.tmp)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_shell_and_js_are_written_offline(self):
        shell = plot_view.prepare_shell(force=True)
        self.assertTrue(shell.is_file())
        js = self.tmp / "plot" / plot_view.PLOTLY_JS_NAME
        self.assertTrue(js.is_file(), "plotly.min.js 必须随壳页落盘（现场无外网）")
        self.assertGreater(js.stat().st_size, 1_000_000, "plotly.js 体积异常")
        html = shell.read_text(encoding="utf-8")
        # 只引用同目录的本地文件
        self.assertIn(plot_view.PLOTLY_JS_NAME, html)
        self.assertNotIn("cdn.plot.ly", html)
        self.assertNotRegex(html, r"""(src|href)\s*=\s*["']https?://""")

    def test_shell_reuses_files_unless_forced(self):
        plot_view.prepare_shell(force=True)
        js = self.tmp / "plot" / plot_view.PLOTLY_JS_NAME
        shell = self.tmp / "plot" / plot_view.SHELL_NAME
        before = (js.stat().st_mtime_ns, shell.stat().st_mtime_ns)
        plot_view.prepare_shell()
        after = (js.stat().st_mtime_ns, shell.stat().st_mtime_ns)
        self.assertEqual(before, after, "没变化却重写了 4.8 MB 的 plotly.js")

    def test_shell_exposes_the_two_entry_points(self):
        html = plot_view.prepare_shell(force=True).read_text(encoding="utf-8")
        for hook in ("window.__setFigure", "window.__select", "window.__state"):
            self.assertIn(hook, html, f"壳页缺少 {hook}")


class PayloadTest(unittest.TestCase):
    def test_numpy_values_are_serialisable(self):
        """numpy 数据必须能 JSON 化。

        注意 plotly 6+ 会把 numpy 数组编码成 ``{"dtype": ..., "bdata": ...}``
        （base64 紧凑格式），plotly.js 认得；所以这里不假设"一定是 list"，
        只要求：json.dumps 不抛、且数据既不是 numpy 对象也不是缺值。
        """
        import numpy as np
        import plotly.graph_objects as go

        figure = go.Figure(go.Scatter3d(
            x=np.array([0.0, 1.0]), y=np.linspace(0, 1, 2),
            z=np.array([0, 2], dtype=np.int32)))
        payload = plot_view.figure_payload(figure)
        text = json.dumps(payload, ensure_ascii=False)      # 不抛即通过
        for axis in ("x", "y", "z"):
            value = payload["data"][0][axis]
            self.assertIsNotNone(value, axis)
            self.assertIn(type(value).__name__, ("list", "dict"), axis)
            if isinstance(value, list):                     # 老版本 plotly：明文
                self.assertEqual(len(value), 2, axis)
        self.assertIn("layout", payload)
        self.assertLess(len(text), 200_000, "负载不该接近内联 js 的量级")

    def test_payload_keeps_customdata_for_table_linking(self):
        import plotly.graph_objects as go

        figure = go.Figure(go.Scatter3d(
            x=[1, 2], y=[3, 4], z=[5, 6], customdata=[[0], [1]]))
        payload = plot_view.figure_payload(figure)
        self.assertEqual(payload["data"][0]["customdata"], [[0], [1]])

    def test_empty_figure_is_safe(self):
        import plotly.graph_objects as go

        payload = plot_view.figure_payload(go.Figure())
        self.assertEqual(payload["data"], [])


class BindingTest(unittest.TestCase):
    def test_binding_is_known(self):
        from laser_desktop import qt_compat
        self.assertIn(qt_compat.BINDING, ("PySide6", "PyQt5"))
        # 交付必须是 PySide6；PyQt5 只是本机验证
        self.assertEqual(qt_compat.DEV_ONLY_BINDING,
                         qt_compat.BINDING == "PyQt5")

    def test_no_disable_gpu_flag(self):
        """``--disable-gpu`` 会让 WebGL 失效、Plotly 3D 直接白屏（实测）。

        只检查**代码**，不检查 docstring——文档里正解释为什么不能加它。
        """
        import ast
        import inspect
        from laser_desktop import qt_compat

        tree = ast.parse(inspect.getsource(
            qt_compat.configure_webengine_environment).lstrip())
        function = tree.body[0]
        body = function.body
        if body and isinstance(body[0], ast.Expr) and isinstance(
                body[0].value, ast.Constant) and isinstance(body[0].value.value, str):
            body = body[1:]                     # 去掉 docstring
        code = "\n".join(ast.unparse(node) for node in body)
        self.assertNotIn("--disable-gpu", code)


if __name__ == "__main__":
    unittest.main()
