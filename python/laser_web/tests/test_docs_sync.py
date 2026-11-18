#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""说明书与代码同步：新增工具/页面却没写进 ``docs/使用说明书.md`` 时测试失败。

为什么值得单独一条测试
----------------------
文档最容易烂的方式不是写错，而是**加了功能忘了写**。这条测试让"同步维护"
变成机械约束：只要新的 CLI 模块或界面页面没在说明书里出现，测试就红，CI/本地
跑一遍就知道该补文档。

判定口径刻意宽松（只查"名字是否出现"），不校验措辞与准确性——那部分靠人。
"""
import re
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
DOC = REPO / "docs" / "使用说明书.md"


def _doc_text() -> str:
    return DOC.read_text(encoding="utf-8")


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
    pages = []
    for path in sorted((REPO / "python" / "laser_web" / "pages").glob("*.py")):
        if path.name.startswith("__"):
            continue
        pages.append(path.stem)
    return pages


class ManualExistsTest(unittest.TestCase):
    def test_manual_exists_and_is_substantial(self):
        self.assertTrue(DOC.is_file(), f"缺少说明书：{DOC}")
        text = _doc_text()
        self.assertGreater(len(text), 8000, "说明书太短，可能没写完")
        for heading in ("五分钟上手", "环境准备", "网页界面", "命令行工具",
                        "常见任务怎么做", "出错了怎么查", "约定与术语", "变更记录"):
            self.assertIn(heading, text, f"说明书缺少「{heading}」章节")


class ManualCoversEveryCliToolTest(unittest.TestCase):
    def test_every_cli_module_is_documented(self):
        text = _doc_text()
        missing = [name for name in _cli_modules() if name not in text]
        self.assertEqual(
            missing, [],
            "这些命令行工具没写进说明书："
            + "、".join(missing)
            + f"\n（文件：{DOC.relative_to(REPO)}）")

    def test_cli_module_discovery_is_sane(self):
        """守卫自身要有牙齿：至少扫到已知的五个工具。"""
        modules = _cli_modules()
        for expected in ("excitation_trajectory.aabb_snake_scan",
                         "excitation_trajectory.avoid_singularity",
                         "excitation_trajectory.export_gp",
                         "excitation_trajectory.iso9283_cube",
                         "robot_model.json_to_urdf"):
            self.assertIn(expected, modules)


class ManualCoversEveryPageTest(unittest.TestCase):
    def test_every_page_is_documented(self):
        text = _doc_text()
        missing = [name for name in _page_modules() if name not in text]
        self.assertEqual(missing, [],
                         "这些界面页面没写进说明书：" + "、".join(missing))

    def test_page_discovery_is_sane(self):
        pages = _page_modules()
        for expected in ("generate", "repair", "tools", "iso9283"):
            self.assertIn(expected, pages)


class ManualCoversConventionsTest(unittest.TestCase):
    def test_euler_conventions_are_listed(self):
        """三种内旋约定必须在术语章节里逐个出现。"""
        from excitation_trajectory.aabb_snake_scan import EULER_CONVENTIONS

        text = _doc_text()
        for convention in EULER_CONVENTIONS:
            self.assertIn(convention, text, f"说明书没提 {convention}")

    def test_extrinsic_removal_is_explained(self):
        """移除定轴是破坏性改动，说明书必须交代怎么迁移。"""
        text = _doc_text()
        self.assertIn("xyz-extrinsic", text)      # 至少说明旧写法会报错
        self.assertIn("已从接口移除", text)

    def test_install_paths_are_documented(self):
        text = _doc_text()
        for item in ("requirements.txt", "requirements-web.txt",
                     "--port=8765", "127.0.0.1"):
            self.assertIn(item, text)


if __name__ == "__main__":
    unittest.main()
