#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两份文档与代码同步，且各守本分。

- ``docs/操作说明书.md``：**面向现场操作人员**，只讲界面。新增界面页面必须写进去；
  同时**不得**出现命令行痕迹（``PYTHONPATH``、``--flag`` 之类），否则说明内部细节
  漏到了客户文档里。
- ``docs/开发参考.md``：面向开发者，必须覆盖每个带 ``main()`` 的命令行工具。

判定口径刻意宽松（只查"名字是否出现"），不校验措辞——准确性靠人。
"""
import re
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
DOCS = REPO / "docs"
CUSTOMER_DOC = DOCS / "操作说明书.md"
DEVELOPER_DOC = DOCS / "开发参考.md"

#: 客户文档里不该出现的技术痕迹（正则 -> 说明）。
_JARGON = (
    (re.compile(r"PYTHONPATH"), "PYTHONPATH（命令行环境变量）"),
    (re.compile(r"--[a-z][a-z0-9-]{2,}"), "长选项（如 --target-count）"),
    (re.compile(r"\.py\b"), ".py 文件名"),
    (re.compile(r"intrinsic"), "intrinsic 字样（界面已改叫「内旋」）"),
)


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _cli_modules() -> list:
    """仓库里带 ``def main`` 的可运行模块（相对 ``python/`` 的模块名）。"""
    modules = []
    for path in sorted((REPO / "python").glob("*/*.py")):
        if path.name == "__init__.py":
            continue
        if re.search(r"^def main\(", path.read_text(encoding="utf-8"), re.M):
            modules.append(f"{path.parent.name}.{path.stem}")
    return modules


def _page_modules() -> list:
    return [path.stem for path in
            sorted((REPO / "python" / "laser_web" / "pages").glob("*.py"))
            if not path.name.startswith("__")]


class BothDocsExistTest(unittest.TestCase):
    def test_customer_doc_is_substantial(self):
        self.assertTrue(CUSTOMER_DOC.is_file(), f"缺少操作说明书：{CUSTOMER_DOC}")
        text = _text(CUSTOMER_DOC)
        self.assertGreater(len(text), 6000, "操作说明书太短，可能没写完")
        for heading in ("这个工具能做什么", "打开界面", "界面总览", "生成标定点",
                        "现场报奇异", "ISO 9283", "导出控制器 GP",
                        "怎么读结果", "常见问题", "名词解释"):
            self.assertIn(heading, text, f"操作说明书缺少「{heading}」")

    def test_developer_doc_is_substantial(self):
        self.assertTrue(DEVELOPER_DOC.is_file(), f"缺少开发参考：{DEVELOPER_DOC}")
        text = _text(DEVELOPER_DOC)
        for heading in ("五分钟上手", "环境准备", "命令行工具", "约定与术语", "变更记录"):
            self.assertIn(heading, text, f"开发参考缺少「{heading}」")


class CustomerDocStaysCustomerFacingTest(unittest.TestCase):
    def test_no_command_line_jargon(self):
        text = _text(CUSTOMER_DOC)
        offenders = []
        for pattern, label in _JARGON:
            for match in pattern.finditer(text):
                line = text[:match.start()].count("\n") + 1
                offenders.append(f"第 {line} 行出现 {label}：{match.group(0)}")
        self.assertEqual(
            offenders, [],
            "操作说明书是给现场人员的，不该出现这些：\n" + "\n".join(offenders))

    def test_it_tells_the_operator_what_to_click(self):
        """客户文档必须真的讲操作（出现按钮名与四页标签）。"""
        text = _text(CUSTOMER_DOC)
        for label in ("生成并解算", "预览路点", "定位段", "修复", "生成五点",
                      "生成 GP", "导出"):
            self.assertIn(label, text, f"操作说明书没提按钮/动作「{label}」")


class CustomerDocCoversEveryPageTest(unittest.TestCase):
    TABS = ("生成", "奇异修复", "ISO 五点", "工具")

    def test_every_page_tab_is_described(self):
        text = _text(CUSTOMER_DOC)
        missing = [tab for tab in self.TABS if tab not in text]
        self.assertEqual(missing, [], "这些页面没写进操作说明书：" + "、".join(missing))

    def test_page_modules_still_match_the_tabs(self):
        """页面增删时要同步改这份测试的 TABS 与操作说明书。"""
        self.assertEqual(sorted(_page_modules()),
                         sorted(("generate", "iso9283", "repair", "tools")))


class DeveloperDocCoversEveryCliToolTest(unittest.TestCase):
    def test_every_cli_module_is_documented(self):
        text = _text(DEVELOPER_DOC)
        missing = [name for name in _cli_modules() if name not in text]
        self.assertEqual(missing, [],
                         "这些命令行工具没写进开发参考：" + "、".join(missing))

    def test_cli_module_discovery_is_sane(self):
        modules = _cli_modules()
        for expected in ("excitation_trajectory.aabb_snake_scan",
                         "excitation_trajectory.avoid_singularity",
                         "excitation_trajectory.export_gp",
                         "excitation_trajectory.iso9283_cube",
                         "robot_model.json_to_urdf"):
            self.assertIn(expected, modules)


class ConventionsDocumentedTest(unittest.TestCase):
    def test_euler_conventions_listed_in_developer_doc(self):
        from excitation_trajectory.aabb_snake_scan import EULER_CONVENTIONS

        text = _text(DEVELOPER_DOC)
        for convention in EULER_CONVENTIONS:
            self.assertIn(convention, text, f"开发参考没提 {convention}")

    def test_extrinsic_removal_explained(self):
        text = _text(DEVELOPER_DOC)
        self.assertIn("xyz-extrinsic", text)
        self.assertIn("已从接口移除", text)


if __name__ == "__main__":
    unittest.main()
