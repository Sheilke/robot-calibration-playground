#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径解析：源码运行 vs 打包运行必须给对目录。

这是桌面化最容易翻车的一条：现有代码到处用 ``Path(__file__).parents[2]`` 反推
仓库根，打包后 ``__file__`` 在 ``_internal/`` 里，推出来是个不存在的路径。
"""
import importlib
import sys
import unittest
from pathlib import Path
from unittest import mock

from laser_desktop import paths

REPO = Path(__file__).resolve().parents[3]


class SourceRunTest(unittest.TestCase):
    def test_repo_root_when_not_frozen(self):
        self.assertFalse(paths.is_frozen())
        self.assertEqual(paths.repo_root(), REPO)
        self.assertEqual(paths.asset_root(), REPO)
        self.assertEqual(paths.config_dir(), REPO / "data" / "configs")
        self.assertEqual(paths.data_root(), REPO)
        self.assertEqual(paths.runs_root(), REPO / "runs")
        self.assertEqual(paths.cache_root(), REPO / ".cache")

    def test_config_dir_actually_holds_models(self):
        """资产路径要真的能指到模型，否则桌面版一启动就"找不到模型"。"""
        self.assertTrue((paths.config_dir() / "hyundai_general6s.urdf").is_file())


class FrozenRunTest(unittest.TestCase):
    """模拟 PyInstaller：``sys.frozen`` + ``sys._MEIPASS``。"""

    def setUp(self):
        self.tmp = Path(self.enterContext(
            __import__("tempfile").TemporaryDirectory()))
        self.bundle = self.tmp / "_internal"
        (self.bundle / "assets" / "data" / "configs").mkdir(parents=True)
        (self.bundle / "assets" / "data" / "configs" / "fake.urdf").write_text("x")

    def test_assets_come_from_bundle_data_goes_to_user_dir(self):
        # 两个平台的用户数据目录环境变量都指到临时目录，避免用例依赖宿主系统
        env = {"LOCALAPPDATA": str(self.tmp), "XDG_DATA_HOME": str(self.tmp)}
        with mock.patch.object(sys, "frozen", True, create=True), \
                mock.patch.object(sys, "_MEIPASS", str(self.bundle), create=True), \
                mock.patch.dict("os.environ", env):
            __import__("os").environ.pop(paths.ENV_HOME, None)
            self.assertTrue(paths.is_frozen())
            self.assertEqual(paths.asset_root(), self.bundle / "assets")
            self.assertEqual(paths.config_dir(),
                             self.bundle / "assets" / "data" / "configs")
            # 可写目录不能落在安装目录里（Program Files 无写权限）
            self.assertEqual(paths.data_root(),
                             self.tmp / paths.APP_ID)
            self.assertNotEqual(paths.runs_root(), self.bundle / "runs")

    def test_env_override_wins_everywhere(self):
        home = self.tmp / "portable"
        with mock.patch.dict("os.environ", {paths.ENV_HOME: str(home)}):
            self.assertEqual(paths.data_root(), home.resolve())
            self.assertEqual(paths.runs_root(), home.resolve() / "runs")
            self.assertEqual(paths.cache_root(), home.resolve() / ".cache")

    def test_ensure_writable_roots_creates_them(self):
        home = self.tmp / "d"
        with mock.patch.dict("os.environ", {paths.ENV_HOME: str(home)}):
            created = paths.ensure_writable_roots()
            for name in ("data", "runs", "cache"):
                self.assertTrue(created[name].is_dir(), name)


class DescribeTest(unittest.TestCase):
    def test_describe_lists_all_roots(self):
        info = paths.describe()
        self.assertEqual(sorted(info),
                         ["assets", "bundle", "cache", "configs", "data",
                          "frozen", "runs"])
        for key in ("assets", "cache", "configs", "data", "runs"):
            self.assertTrue(Path(info[key]).is_absolute(), key)


if __name__ == "__main__":
    unittest.main()
