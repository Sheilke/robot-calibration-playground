# -*- coding: utf-8 -*-
"""``laser_web.paths``：冻结后资产根必须落在 ``_MEIPASS/assets``。

这条测试守的是一个**只在打包后出现**的故障：``core`` 曾经用
``Path(__file__).resolve().parents[2]`` 反推仓库根，源码模式下完全正确，而
PyInstaller 冻结后模块的 ``__file__`` 是 ``<_MEIPASS>/laser_web/core``（**没有**
``.py`` 后缀），``parents[2]`` 数出来是**安装目录**，于是：

* 模型下拉框照常列出 ``data/configs/*.urdf``（`list_robots` 用相对路径）；
* 一点「生成并解算」就 ``FileNotFoundError`` —— 因为拼出来的
  ``<app>/data/configs/…`` 并不存在，资产在 ``<app>/_internal/assets/data/configs``。

源码里跑一万次也复现不了，装到现场第一次就中。所以这里**在子进程里**先把
``sys.frozen``/``sys._MEIPASS`` 设好再导入（``_REPO_ROOT`` 是模块导入时的快照，
在同一进程里事后改 ``sys.frozen`` 是不会生效的——这一点本身也值得记住）。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]

#: 模拟 PyInstaller onedir 的目录形状：``<app>/_internal`` 是 ``_MEIPASS``。
_FAKE_MEIPASS = str(Path("/opt/LaserCalibration/_internal"))


def _run_frozen(code: str) -> str:
    """在子进程里以"冻结形态"导入 ``laser_web``，返回 stdout。"""
    prelude = (
        "import sys, json\n"
        f"sys.frozen = True\n"
        f"sys._MEIPASS = {_FAKE_MEIPASS!r}\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", prelude + code],
        cwd=str(REPO), capture_output=True, text=True,
        env=_child_env(),
        timeout=300,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"子进程失败（{proc.returncode}）：\n{proc.stdout}\n{proc.stderr}")
    return proc.stdout.strip()


def _child_env() -> dict:
    """子进程环境：**继承当前环境**，只把 PYTHONPATH 指到仓库的 python/。

    不要手写 ``{"PATH": "/usr/bin:/bin"}``：在 Windows 上那样会让子进程找不到
    系统目录，导入链里一旦碰到需要它们的东西就整段失败，报出来的却是一个
    和被测逻辑无关的错（本文件最初就是这么写的，在 Linux 上永远看不出来）。
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO / "python")
    return env


class FrozenAssetRootTest(unittest.TestCase):
    """冻结后 `asset_root` 与 `core` 用的是同一个、正确的位置。"""

    def test_asset_root_points_into_meipass(self):
        out = _run_frozen(
            "from laser_web import paths\n"
            "print(paths.asset_root())\n")
        self.assertEqual(out, f"{_FAKE_MEIPASS}/assets")

    def test_core_repo_root_follows_asset_root(self):
        """这条是真正的回归点：`core._REPO_ROOT` 必须等于 `asset_root()`。"""
        out = _run_frozen(
            "from laser_web import paths, core\n"
            "print(json.dumps([str(paths.asset_root()), str(core._REPO_ROOT)]))\n")
        asset_root, core_root = json.loads(out)
        self.assertEqual(core_root, asset_root,
                         "core._REPO_ROOT 没有跟着资产根走，冻结后模型路径会拼错")

    def test_model_path_resolves_under_assets(self):
        out = _run_frozen(
            "from laser_web import core\n"
            "print(core._resolve_model_path('data/configs/ch2000_origin.urdf'))\n")
        self.assertEqual(
            out,
            f"{_FAKE_MEIPASS}/assets/data/configs/ch2000_origin.urdf",
            "模型相对路径没有解到 _internal/assets 下")

    def test_source_mode_unchanged(self):
        """源码模式必须仍等于仓库根（不能为了打包把开发路径改坏）。"""
        proc = subprocess.run(
            [sys.executable, "-c",
             "from laser_web import paths; print(paths.asset_root())"],
            cwd=str(REPO), capture_output=True, text=True,
            env=_child_env(),
            timeout=300,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(Path(proc.stdout.strip()).resolve(), REPO)

    def test_desktop_paths_delegates(self):
        """桌面端 `asset_root()` 与 core 用同一份定义（不许各算一遍）。"""
        proc = subprocess.run(
            [sys.executable, "-c",
             "from laser_web import paths;"
             "from laser_desktop import paths as desktop;"
             "print(paths.asset_root() == desktop.asset_root())"],
            cwd=str(REPO), capture_output=True, text=True,
            env=_child_env(),
            timeout=300,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(proc.stdout.strip(), "True")


if __name__ == "__main__":
    unittest.main()
