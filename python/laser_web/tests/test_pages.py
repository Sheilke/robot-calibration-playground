#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""页面层冒烟：模块可导入、关键守卫函数行为正确、页面真能建起来。

真正的浏览器交互（点按钮、弹 toast、拉表格）靠 ``nicegui.testing.User`` 或
人工冒烟，这里不碰；但 ``render()`` 会**真的跑一遍**（无 websocket 的 client），
所以“元素定义顺序/闭包名字写错”这类只在建页面时才炸的问题会被钉住。
"""
import re
import tempfile
import unittest
from pathlib import Path

from laser_web import jobs
from laser_web.pages import (POINT_COUNT_WARN, adopt_running_job,
                             cancelled_result)

#: 页面源码目录（回调/初始化写法的静态检查用）。
PAGES_DIR = Path(__file__).resolve().parents[1] / "pages"


class PagesImportTest(unittest.TestCase):
    def test_pages_import_and_expose_render(self):
        from laser_web import app  # noqa: F401 - 页面注册依赖它
        from laser_web.pages import generate, iso9283, repair, tools

        for module in (generate, iso9283, repair, tools):
            self.assertTrue(callable(module.render), module.__name__)

    def test_warn_threshold_is_sane(self):
        """软上限要挡住"笔误几十万点"，但不能挡住正常交付的 50 点。"""
        self.assertGreater(POINT_COUNT_WARN, 50)
        self.assertLessEqual(POINT_COUNT_WARN, 20000)


class EulerLabelTest(unittest.TestCase):
    """界面上不出现 "intrinsic" 字样，一律写"内旋"，但提交的值仍是算法约定名。"""

    def test_labels_are_chinese_and_values_are_canonical(self):
        from laser_web.ui_state import (EULER_LABELS, euler_convention_options,
                                        euler_conventions)

        options = euler_convention_options()
        self.assertEqual(list(options), euler_conventions())
        for value, label in options.items():
            with self.subTest(value=value):
                self.assertIn("内旋", label)
                self.assertNotIn("intrinsic", label.lower())
                self.assertNotIn("extrinsic", label.lower())
                # 值必须保持算法模块里的规范名，后端与 CLI 才对得上
                self.assertTrue(value.endswith("-intrinsic"))
                self.assertEqual(EULER_LABELS[value], label)

    def test_three_orders_only(self):
        from laser_web.ui_state import euler_conventions

        self.assertEqual(euler_conventions(),
                         ["xyz-intrinsic", "zyx-intrinsic", "zyz-intrinsic"])

    def test_unknown_value_falls_back_verbatim(self):
        from laser_web.ui_state import euler_label

        self.assertEqual(euler_label("zyz-extrinsic"), "zyz-extrinsic")

    def test_gp_dropdown_uses_the_same_labels(self):
        from laser_web import core, ui_state

        for value in core.gp_euler_conventions():
            label = ui_state.euler_label(value)
            self.assertIn("内旋", label)
            self.assertNotIn("intrinsic", label.lower())


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
    """四个页面的 ``render()`` 真的能建完（不连浏览器，但也不只是 import）。

    这一层能抓住"元素定义顺序/闭包名字写错"这类只在建页面时才炸的问题
    （例如把被 ``@ui.refreshable`` 函数引用的 helper 定义在它后面）。真的
    交互（点按钮、弹 toast）仍由人工冒烟确认。
    """

    def test_all_four_pages_build(self):
        import asyncio

        import nicegui

        from laser_web.pages import generate, iso9283, repair, tools

        builders = (("/", generate), ("/repair", repair), ("/iso", iso9283),
                    ("/tools", tools))

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


class GridVertexConversionTest(unittest.TestCase):
    """界面填「格点数（行列数）」，算法要「份数」（= 格点数 − 1）。

    现场同事按"这一方向有几个点"来估，而 CLI 历史上用"份数"，两者差 1。
    换算只允许在这一个地方做，避免两个页面各写一遍、有人写成 +1 或漏减。
    """

    def test_conversion_is_off_by_one(self):
        from laser_web.ui_state import segments_to_vertices, vertices_to_segments

        for vertices in (1, 2, 3, 5, 5, 6, 21):
            with self.subTest(vertices=vertices):
                self.assertEqual(vertices_to_segments(vertices), vertices - 1)
                self.assertEqual(
                    segments_to_vertices(vertices_to_segments(vertices)), vertices)

    def test_defaults_are_vertex_counts(self):
        """默认值必须是「格点数」：CLI 默认份数 (1,4,4) → 界面显示 (2,5,5)。"""
        from excitation_trajectory.aabb_snake_scan import DEFAULT_GRID
        from laser_web.ui_state import default_form_values

        values = default_form_values()
        self.assertEqual((values["xs"], values["ys"], values["zs"]),
                         tuple(segments + 1 for segments in DEFAULT_GRID))

    def test_guards_never_go_negative(self):
        from laser_web.ui_state import segments_to_vertices, vertices_to_segments

        self.assertEqual(vertices_to_segments(0), 0)     # 0 格点按 0 份数处理
        self.assertEqual(vertices_to_segments(-3), 0)
        self.assertEqual(segments_to_vertices(-3), 1)    # 至少 1 个格点


class IsoPageRowsTest(unittest.TestCase):
    """ISO 页的表格行与汇总行：只有坐标 + 可达（没有关节量）。"""

    def result(self):
        return {"ok": True,
                "points": [{"name": "P1", "pos": [1000.0, 0.0, 600.0],
                            "reachable": True},
                           {"name": "P2", "pos": [900.0, 100.0, 500.0],
                            "reachable": False}],
                "summary": {"reachable": 1, "total": 2},
                "meta": {"rpy": [3.0, 73.0, -167.0]}}

    def test_rows_have_no_joint_fields(self):
        from laser_web.pages import iso9283

        rows = iso9283._point_rows(self.result())
        self.assertEqual(len(rows), 2)
        for row in rows:
            self.assertEqual(set(row), {"name", "x", "y", "z", "reach",
                                        "failed"})
        self.assertEqual(rows[0]["reach"], "✓ 可达")
        self.assertEqual(rows[1]["reach"], "✗ 不可达")
        self.assertTrue(rows[1]["failed"])

    def test_columns_are_cartesian_only(self):
        from laser_web.pages import iso9283

        labels = [label for _name, label, _field, _align
                  in iso9283._POINT_COLUMNS]
        self.assertEqual(labels, ["P", "X (mm)", "Y (mm)", "Z (mm)", "可达"])
        self.assertNotIn("q1", labels)

    def test_summary_says_reachable(self):
        from laser_web.pages import iso9283

        self.assertEqual(iso9283._summary_text(self.result()), "可达 1/2")
        self.assertEqual(iso9283._summary_text({"ok": False}), "可达 —")
        self.assertEqual(iso9283._HINT,
                         "只算笛卡尔坐标；姿态固定、不加扰动；系统会解一次 IK "
                         "判断每点是否可达，不输出关节角。")


class PageInitUsesCurrentToggleTest(unittest.TestCase):
    """切换型页面初始化必须按 toggle 的**当前值**显示输入块，不能写死模式名。

    真实故障：ISO 页初始化写死 ``switch_mode("center")``，而立方体来源已经没有
    ``center`` 这一项（只有 bounds / auto）→ 打开页面后**两组输入全被隐藏**，
    看起来“这一页没有输入框”。源码扫描守住这条，避免再出现写死旧模式名的写法。

    ISO 页后来又把两种来源一起删了（ISO 9283 只给最大/最小点，没有“布点 /
    截面”可选），所以那里改成断言“没有切换控件”，同一个坑不可能再出现。
    """

    PAGES = ("generate", "repair", "iso9283")

    def _source(self, name: str) -> str:
        return (PAGES_DIR / f"{name}.py").read_text(encoding="utf-8")

    def test_iso_page_shows_no_joint_quantities(self):
        """ISO 页不显示关节量：没有 q_seed / j5_min 输入、没有任务机制。"""
        source = self._source("iso9283")
        for banned in ('fields["q_seed"]', 'fields["j5_min"]',
                       "log_view", "linear_progress", "request_cancel",
                       "iso_cube_figure", "jobs.submit"):
            self.assertNotIn(banned, source, f"ISO 页里还有 {banned}")
        for column in ("q1", "q2", "q3", "q4", "q5", "q6", "j3", "j5", "cfg"):
            self.assertNotIn(f'("{column}",', source, f"表里还有 {column} 列")
        self.assertIn('"reach", "可达", "reach", "center"', source)
        self.assertIn("系统会解一次 IK 判断每点是否可达", source)

    def test_iso_page_has_no_source_toggle_left(self):
        source = self._source("iso9283")
        self.assertNotIn("switch_mode", source)
        self.assertNotIn("switch_layout", source)
        self.assertNotIn('"cube_mode": fields[', source)
        self.assertIn('"cube_mode": _CUBE_MODE', source)

    #: 切换函数 -> 它对应的 toggle 字段名。
    TOGGLES = {"mode": "cube_mode", "domain": "domain_mode", "grid": "grid_mode"}

    def _literal_switches(self, source: str):
        """找出 ``switch_*(字面量)`` 的位置，并判断附近是否把 toggle 设成了同一个值。

        允许「先赋值 toggle、再按它切换」的写法（repair 页的"填入样例"就是这样，
        自洽）；只抓"写死一个模式名，但 toggle 并不指向它"的情况——那才会让可见的
        切换按钮与实际显示的输入块不一致。
        """
        lines = source.splitlines()
        pattern = re.compile(r"switch_(mode|domain|grid)\(\s*[\"']([^\"']+)[\"']")
        problems = []
        for index, line in enumerate(lines):
            match = pattern.search(line)
            if not match:
                continue
            kind, literal = match.group(1), match.group(2)
            field = self.TOGGLES[kind]
            window = lines[max(0, index - 12):index + 4]
            consistent = any(
                re.search(rf'fields\["{field}"\]\.value\s*=\s*[\"\']{re.escape(literal)}',
                          candidate)
                for candidate in window)
            if not consistent:
                problems.append(f"{index + 1}: {line.strip()}（toggle {field} 未设为 {literal!r}）")
        return problems

    def test_switch_calls_agree_with_their_toggle(self):
        """``switch_*(字面量)`` 必须伴随把对应 toggle 设成同一个值。"""
        offenders = []
        for name in self.PAGES:
            for problem in self._literal_switches(self._source(name)):
                offenders.append(f"{name}.py:{problem}")
        self.assertEqual(offenders, [],
                         "切换用字面量但 toggle 不是该值 → 按钮与实际输入块不一致：\n"
                         + "\n".join(offenders))
