#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工具页（S5）：GP 导出 / JSON → URDF / 运行目录清理 / 模板摘要 / 上传限流。

四块各自独立，这里按块验：

1. **GP 导出**：真跑一次 8 点扫描（``core.run_scan`` 写进临时 ``runs/``），再用
   **合成模板**（:data:`SYNTHETIC_TEMPLATE`，三样字段就够）走 ``core.build_gp``；
   断言写了 8 个 GP、模板 sha256 前后一致、副本路径真实存在。现场那份真模板
   只能通过 ``LASER_GP_TEMPLATE`` / :func:`tools_page.find_template` 取到，取不到
   就跳过——**测试不再依赖任何绝对路径**（否则换台机器/CI 必红）；
2. **JSON → URDF**：控制器 JSON 用 ``robot_model/tests`` 里那份最小样例（仓库
   的 ``data/configs/*.json`` 是 DH 参数 / 扫描元数据，不是控制器格式，见报告），
   断言产出 ``.urdf``、关节数与末端位姿都显示出来；
3. **运行目录**：清理**不动**正在跑的任务目录（``exclude`` 生效），没有活跃目录
   时与 ``core.clean_runs`` 完全同口径；
4. **模板文件**：sha256 与字节数来自文件本身，重读前后不变（只读）；
5. **上传限流**：造一个 >16 MB 的文件，``read_limited`` 直接拒绝，**不读进内存**；
   并且**先成功再超限**——拒绝后不许留着上一次的状态（否则点下去就用旧模板
   静默导出）。
"""

import hashlib
import json
import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import tools_page
from laser_desktop.qt_compat import QtWidgets
from laser_web import core, jobs

_APP = None


def app():
    """整个测试会话共用一个 QApplication。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-tools-tests"])
    return _APP


#: 合成的 GP 模板：字段与现场 ``position_R1_template.json`` 同构，但**不如库、
#: 不依赖任何现场文件**。``core.build_gp`` 只要三样：``controllerID`` /
#: ``version``（导出前后必须原样回读）与 ``globalPosition``（每个槽位 14 字段）。
#: 这里给 12 个槽位：8 点扫描写 GP0001..GP0008，GP0009 之后必须一个字不动。
SYNTHETIC_TEMPLATE = {
    "controllerID": "synthetic-controller",
    "version": "1.0",
    "globalPosition": {f"GP{index:04d}": [float(index)] * 14
                       for index in range(1, 13)},
}


def synthetic_template() -> dict:
    """每次给一份**新**的合成模板（测试里会比对“写之前”的那份）。"""
    return json.loads(json.dumps(SYNTHETIC_TEMPLATE))


def synthetic_template_bytes() -> bytes:
    """合成模板的字节（与真模板一样：``json.dumps(..., indent=1)``）。"""
    return (json.dumps(synthetic_template(), ensure_ascii=False, indent=1)
            + "\n").encode("utf-8")


def field_template_path():
    """现场真模板：先看 ``LASER_GP_TEMPLATE``，再走 :func:`tools_page.find_template`。

    Returns:
        Path | None: 找到的那份；都没找到时 ``None``（调用方 ``skipTest``）。
    """
    override = os.environ.get(tools_page.TEMPLATE_ENV)
    if override:
        path = Path(override).expanduser()
        if path.is_file():
            return path
    return tools_page.find_template()

#: 8 点小案子（与 ``laser_web/tests/test_tools.py`` 同一份参数）。
BASE_PARAMS = {"robot": "data/configs/ch2000_origin.urdf",
               "point_min": [900.0, -100.0, 500.0],
               "point_max": [1100.0, 100.0, 700.0],
               "rpy": [3.0, 73.0, -167.0],
               "euler": "xyz-intrinsic",
               "perturb_deg": [20.0, 20.0, 35.0],
               "target_count": 8,
               "q_seed": "43,-30,-36,42,-89,-1",
               "j5_min": 20.0,
               "seed": 0}

#: 最小可用的控制器 JSON（照抄 ``robot_model/tests/test_json_to_urdf.py``）。
MINIMAL_CONTROLLER = {
    "robotType": "R_GENERAL_6S", "totalAxisNum": 6, "eulerAngle": "XYZ",
    "Link&CoupleCoe": {
        "mountMethod": 0,
        "CoupleCoe": {"Couple_Coe_5_6": 0.0},
        "Link": [
            {"a": 159.47, "alpha": 90.0, "d": 446.6, "theta": 0.0},
            {"a": 770.7, "alpha": 0.0, "d": 0.0, "theta": 90.0},
            {"a": 141.05, "alpha": 90.0, "d": 0.0, "theta": 0.0},
            {"a": 0.0, "alpha": 90.0, "d": 1025.36, "theta": 0.0},
            {"a": 0.0, "alpha": -90.0, "d": 0.0, "theta": 0.0},
            {"a": 0.0, "alpha": 0.0, "d": 110.4, "theta": 0.0},
        ],
    },
    "Joint": [{"Direction": -1, "NegSWLimit": -180.0, "PosSWLimit": 180.0,
               "RatedVel": 100.0} for _ in range(6)],
}


def sha256_of(path) -> str:
    """文件内容的 sha256（模板“未被修改”的证据）。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run_scan_into(root: Path) -> dict:
    """往临时 ``runs/`` 里真跑一次 8 点扫描，返回任务结果。"""
    registry = jobs.JobRegistry(root=root)
    try:
        job_id = registry.submit(
            "scan", lambda job: core.run_scan(dict(BASE_PARAMS), job=job))
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            app().processEvents()
            job = registry.get(job_id)
            if job is not None and job.state in jobs.TERMINAL_STATES:
                return job.result or {}
            time.sleep(0.01)
        raise AssertionError("扫描超时")
    finally:
        registry.shutdown(timeout=30)


def wait_until(predicate, timeout: float = 60.0) -> bool:
    """转事件循环直到 ``predicate()`` 为真。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app().processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


class PageCase(unittest.TestCase):
    """共用：临时 ``runs/`` 根 + 临时文件目录 + 一个工具页。"""

    def setUp(self):
        app()
        self.root = Path(tempfile.mkdtemp(prefix="tools-runs-"))
        self.files = Path(tempfile.mkdtemp(prefix="tools-files-"))
        self.page = tools_page.ToolsPage(runs_root=self.root)

    def tearDown(self):
        self.page.deleteLater()
        shutil.rmtree(self.root, ignore_errors=True)
        shutil.rmtree(self.files, ignore_errors=True)

    def write_template(self, name=tools_page.TEMPLATE_FILE_NAME):
        """把合成模板写到临时文件（充当“现场那份”的位置）。

        Returns:
            Path: 临时模板路径。
        """
        path = self.files / name
        path.write_bytes(synthetic_template_bytes())
        return path

    def point_template_block_at(self, path) -> dict:
        """让「模板文件」块指向 ``path``（走 ``LASER_GP_TEMPLATE``）并重算摘要。"""
        patcher = mock.patch.dict(os.environ,
                                  {tools_page.TEMPLATE_ENV: str(path)})
        patcher.start()
        self.addCleanup(patcher.stop)
        return self.page.refresh_template_block()


class UploadLimitTest(unittest.TestCase):
    """上传限流：16 MB 以上直接拒绝，**不读进内存**。"""

    def setUp(self):
        app()
        self.tmp = Path(tempfile.mkdtemp(prefix="tools-upload-"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_limit_matches_spec(self):
        self.assertEqual(tools_page.UPLOAD_MAX_BYTES, 16 * 1024 * 1024)

    def test_oversize_is_rejected_before_reading(self):
        """超限文件：``stat()`` 之后就返回，绝不 ``read_bytes()``。"""
        big = self.tmp / "huge.json"
        with open(big, "wb") as handle:
            handle.truncate(tools_page.UPLOAD_MAX_BYTES + 1)   # 稀疏文件，不占内存
        self.assertGreater(big.stat().st_size, tools_page.UPLOAD_MAX_BYTES)

        with mock.patch.object(Path, "read_bytes",
                               side_effect=AssertionError("超限文件被读进内存了")):
            data, error = tools_page.read_limited(big)
        self.assertIsNone(data)
        self.assertIn("上限", error)
        self.assertIn("未读取", error)

    def test_oversize_is_rejected_in_the_page(self):
        """页面上选到超限模板：红字写进提示，模板状态保持为空。"""
        big = self.tmp / "huge.json"
        with open(big, "wb") as handle:
            handle.truncate(tools_page.UPLOAD_MAX_BYTES + 1)
        page = tools_page.ToolsPage(runs_root=self.tmp / "runs")
        try:
            with mock.patch.object(QtWidgets.QFileDialog, "getOpenFileName",
                                   return_value=(str(big), "")):
                self.assertIsNone(page.load_template())
            self.assertIn("上限", page.template_hint.text())
            self.assertIn("color:#c0392b", page.template_hint.styleSheet())
            self.assertIsNone(page.template_state)
        finally:
            page.deleteLater()

    def test_small_file_is_read(self):
        small = self.tmp / "ok.json"
        small.write_text('{"a": 1}', encoding="utf-8")
        data, error = tools_page.read_limited(small)
        self.assertIsNone(error)
        self.assertEqual(data, b'{"a": 1}')

    def oversize(self, name="huge.json") -> Path:
        """造一个稀疏的 >16 MB 文件（不占内存）。"""
        path = self.tmp / name
        with open(path, "wb") as handle:
            handle.truncate(tools_page.UPLOAD_MAX_BYTES + 1)
        return path

    def test_reject_after_success_clears_template(self):
        """先成功上传一次、再选超限文件：**模板状态必须清空**、按钮禁用。

        旧写法用全新页面测（``template_state`` 本来就是 ``None``），测不出
        “拒绝后仍留着上一个模板”——那会让「生成 GP」一点就用**上一个**模板
        静默导出，而界面上再没有任何地方说明当前装的是哪个模板。
        """
        page = tools_page.ToolsPage(runs_root=self.tmp / "runs")
        try:
            small = self.tmp / "ok-template.json"
            small.write_bytes(synthetic_template_bytes())
            with mock.patch.object(QtWidgets.QFileDialog, "getOpenFileName",
                                   return_value=(str(small), "")):
                self.assertIsNotNone(page.load_template())
            self.assertIsNotNone(page.template_state)
            self.assertTrue(page.generate_gp_button.isEnabled()
                            or page.current_run() is None)

            with mock.patch.object(QtWidgets.QFileDialog, "getOpenFileName",
                                   return_value=(str(self.oversize()), "")):
                self.assertIsNone(page.load_template())
            self.assertIsNone(page.template_state, "拒绝后不许留着上一次的模板")
            hint = page.template_hint.text()
            self.assertIn("尚未上传", hint.splitlines()[0],
                          "第一行要说清当前装的是哪个模板")
            self.assertIn("上限", hint)
            self.assertIn("color:#c0392b", page.template_hint.styleSheet())
            self.assertFalse(page.generate_gp_button.isEnabled(),
                             "拦完必须禁用「生成 GP」")
        finally:
            page.deleteLater()

    def test_reject_after_success_clears_controller(self):
        """JSON → URDF 同理：拒绝后不许留着上一次的控制器 JSON。"""
        page = tools_page.ToolsPage(runs_root=self.tmp / "runs")
        try:
            small = self.tmp / "ok-controller.json"
            small.write_bytes(json.dumps(MINIMAL_CONTROLLER).encode("utf-8"))
            with mock.patch.object(QtWidgets.QFileDialog, "getOpenFileName",
                                   return_value=(str(small), "")):
                self.assertIsNotNone(page.load_controller())
            self.assertIsNotNone(page.controller_state)

            with mock.patch.object(QtWidgets.QFileDialog, "getOpenFileName",
                                   return_value=(str(self.oversize()), "")):
                self.assertIsNone(page.load_controller())
            self.assertIsNone(page.controller_state)
            hint = page.json_hint.text()
            self.assertIn("尚未上传", hint.splitlines()[0])
            self.assertIn("上限", hint)
            self.assertIn("color:#c0392b", page.json_hint.styleSheet())
            self.assertFalse(page.convert_button.isEnabled(),
                             "拦完必须禁用「转换」")
        finally:
            page.deleteLater()


class TemplateBlockTest(PageCase):
    """模板文件块：sha256 与字节数来自文件本身（只读）。

    断言同时不碰绝对路径（只看 ``info["path"]`` 存不存在、是不是文件）——
    早先那条 ``Path(info["path"]) == <写死的现场模版常量>`` 在别人机器上必红：
    ``find_template()`` 可能先命中 ``data_root()`` / ``data/configs``。
    """

    def test_template_digest_is_displayed(self):
        """摘要来自 ``find_template()`` 找到的那份文件。"""
        path = self.write_template()
        info = self.point_template_block_at(path)
        self.assertTrue(info["exists"])
        self.assertTrue(Path(info["path"]).is_file(), info["path"])
        self.assertEqual(Path(info["path"]), path)
        self.assertEqual(info["size_bytes"], path.stat().st_size)
        self.assertEqual(info["sha256"], sha256_of(path))
        self.assertRegex(self.page.template_sha_label.text(), r"sha256 [0-9a-f]{64}")
        self.assertIn(str(path.stat().st_size),
                      self.page.template_size_label.text())
        self.assertIn(str(path), self.page.template_path_label.text())

    def test_reload_does_not_touch_the_file(self):
        """重读前后文件 sha256 不变（这一块只读，不写）。"""
        path = self.write_template()
        before = sha256_of(path)
        self.point_template_block_at(path)
        self.page.refresh_template_block()
        self.page.refresh_template_block()
        self.assertEqual(sha256_of(path), before)
        self.assertEqual(self.page.template_file["sha256"], before)

    def test_missing_template_shows_dash(self):
        """找不到模板时显示 ``—``、搜索路径与环境变量名，不谎报一个 sha256。"""
        with mock.patch.object(tools_page, "template_candidates",
                               return_value=[Path("/nonexistent/x.json")]):
            info = self.page.refresh_template_block()
        self.assertFalse(info["exists"])
        self.assertIn("—", self.page.template_size_label.text())
        self.assertIn("—", self.page.template_sha_label.text())
        self.assertIn("/nonexistent/x.json", self.page.template_path_label.text())
        self.assertIn(tools_page.TEMPLATE_ENV,
                      self.page.template_path_label.text())

    def test_candidates_have_no_home_documents_fallback(self):
        """搜索路径只有三处（环境变量 / 数据目录 / configs），不再兜底
        ``~/Documents/backup``：兜底会让客户机上毫无标识地展示一份旧模板的
        sha256，容易被当成交付基线抄走。"""
        with mock.patch.dict(os.environ):
            os.environ.pop(tools_page.TEMPLATE_ENV, None)
            candidates = tools_page.template_candidates()
        self.assertEqual(
            candidates,
            [tools_page.paths.data_root() / tools_page.TEMPLATE_FILE_NAME,
             tools_page.paths.config_dir() / tools_page.TEMPLATE_FILE_NAME])
        self.assertFalse(any("Documents" in str(path) for path in candidates),
                         candidates)
        with mock.patch.dict(os.environ,
                             {tools_page.TEMPLATE_ENV: "/tmp/x.json"}):
            self.assertEqual(tools_page.template_candidates()[0],
                             Path("/tmp/x.json"), "环境变量仍然排第一")


class GpExportTest(PageCase):
    """GP 导出：真运行 + 模板，模板 sha256 必须前后一致。"""

    def setUp(self):
        super().setUp()
        self.result = run_scan_into(self.root)
        self.assertTrue(self.result.get("ok"), self.result.get("error"))
        self.page.refresh_runs()

    def set_synthetic_template(self) -> dict:
        """用合成模板（不依赖现场文件，任何机器上都跑）。"""
        path = self.write_template()
        data = path.read_bytes()
        state = {"name": path.name, "bytes": data, "path": str(path),
                 "sha256": hashlib.sha256(data).hexdigest()}
        self.page.set_template(state)
        return state

    def set_real_template(self) -> dict:
        """现场真模板：路径**只能**来自 ``LASER_GP_TEMPLATE`` / ``find_template()``。

        取不到就跳过——绝不写死 ``~/Documents/backup/...`` 那种绝对路径
        （CI / 同事机器上必然红）。
        """
        path = field_template_path()
        if path is None:
            self.skipTest(
                f"没有现场模板：请用环境变量 {tools_page.TEMPLATE_ENV} 指定 "
                f"{tools_page.TEMPLATE_FILE_NAME}")
        data = Path(path).read_bytes()
        state = {"name": Path(path).name, "bytes": data, "path": str(path),
                 "sha256": hashlib.sha256(data).hexdigest()}
        self.page.set_template(state)
        return state

    def test_runs_are_listed_with_scan_mark(self):
        self.assertEqual(len(self.page.runs), 1)
        row = self.page.runs[0]
        self.assertEqual(tools_page.gp_source(row), core.ARTIFACT_SCAN_JSON)
        self.assertIn("scan.json✓", tools_page.artifact_marks(row))
        self.assertEqual(self.page.runs_table.rowCount(), 1)
        self.assertEqual(self.page.run_combo.currentData(), row["run_id"])
        self.assertTrue(self.page.generate_gp_button.isEnabled())

    def test_build_gp_keeps_template_untouched(self):
        """合成模板：写 8 个 GP、模板字节不变、第 9 条之后一个字不动。"""
        state = self.set_synthetic_template()
        before = state["sha256"]
        result = self.page.generate_gp()
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["count"], 8)
        self.assertEqual(result["first"], 1)
        self.assertEqual(result["last"], 8)
        # 模板：内存里的字节没变
        self.assertEqual(self.page.template_state["bytes"], state["bytes"])
        self.assertEqual(hashlib.sha256(state["bytes"]).hexdigest(), before)
        self.assertIn("模板 sha256 未变", self.page.gp_template_label.text())
        self.assertIn(before, self.page.gp_template_label.text())
        self.assertIn("color:#1e8449", self.page.gp_template_label.styleSheet())
        # 结果区文案 + 副本
        self.assertIn("已写入 8 个 GP", self.page.gp_count_label.text())
        self.assertIn("GP0001 .. GP0008", self.page.gp_count_label.text())
        self.assertIn("controllerID=", self.page.gp_meta_label.text())
        copy = Path(self.page.gp_path_label.text())
        self.assertTrue(copy.is_file(), "GP 副本应当落在运行目录里")
        self.assertEqual(copy.parent, Path(self.result["run_dir"]))
        self.assertTrue(self.page.gp_download_button.isEnabled())
        self.assertTrue(self.page.gp_export_usable)
        # 写出来的文件：GP0001..GP0008 被改写，每条 14 个字段，其余槽位原样保留
        template_payload = synthetic_template()
        payload = json.loads(copy.read_text(encoding="utf-8"))
        keys = list(template_payload["globalPosition"])
        self.assertEqual(list(payload["globalPosition"]), keys)
        self.assertTrue(all(len(payload["globalPosition"][key]) == 14
                            for key in keys[:8]))
        self.assertNotEqual(payload["globalPosition"]["GP0001"],
                            template_payload["globalPosition"]["GP0001"],
                            "GP0001 应当被写进了新的关节角")
        self.assertEqual(payload["globalPosition"]["GP0009"],
                         template_payload["globalPosition"]["GP0009"],
                         "第 9 条之后不许动")
        # controllerID / version 原样（模板元数据不许改）
        for key in core.GP_TEMPLATE_META_KEYS:
            self.assertEqual(payload[key], template_payload[key], key)

    def test_build_gp_with_field_template(self):
        """有现场真模板时也走一遍（没设 ``LASER_GP_TEMPLATE`` 就跳过）。"""
        state = self.set_real_template()
        before = sha256_of(state["path"])
        result = self.page.generate_gp()
        self.assertTrue(result["ok"], result.get("error"))
        self.assertEqual(result["count"], 8)
        self.assertEqual(sha256_of(state["path"]), before, "模板文件不许被改动")
        self.assertIn("模板 sha256 未变", self.page.gp_template_label.text())
        self.assertTrue(self.page.gp_download_button.isEnabled())

    def test_template_change_is_a_hard_failure(self):
        """模板 sha256 真变了就是硬失败（不只红字）：不许说“已写入”、不许下载。"""
        self.set_synthetic_template()
        copy_path = Path(self.result["run_dir"]) / tools_page.TEMPLATE_FILE_NAME
        copy_path.write_text("{}", encoding="utf-8")
        result = {"ok": True, "path": str(copy_path),
                  "filename": tools_page.TEMPLATE_FILE_NAME, "count": 8,
                  "first": 1, "last": 8,
                  "meta": {"controllerID": "x", "version": "1.0"},
                  "warnings": []}
        self.page.render_gp(result, {"bytes": "a" * 64, "file": "a" * 64},
                            {"bytes": "b" * 64, "file": "b" * 64})
        text = self.page.gp_template_label.text()
        self.assertIn("模板 sha256 变了", text)
        self.assertIn("a" * 64, text)
        self.assertIn("b" * 64, text)
        self.assertIn("color:#c0392b", self.page.gp_template_label.styleSheet())
        # 硬失败的三条：不写“已写入”、不亮下载、报错区明说不要下载
        self.assertNotIn("已写入", self.page.gp_count_label.text())
        self.assertIn("已作废", self.page.gp_count_label.text())
        self.assertFalse(self.page.gp_download_button.isEnabled())
        self.assertIn("不要下载", self.page.gp_error_label.text())
        self.assertFalse(self.page.gp_export_usable)

    def test_gp_error_is_shown_without_dialog(self):
        """``core`` 返回 ``ok=False`` 时把原文写在结果区（红字，不弹窗）。"""
        self.set_synthetic_template()
        with mock.patch.object(core, "build_gp",
                               return_value={"ok": False, "error": "参数无法解析：X",
                                             "warnings": ["w"]}):
            result = self.page.generate_gp()
        self.assertFalse(result["ok"])
        self.assertIn("参数无法解析：X", self.page.gp_error_label.text())
        self.assertIn("color:#c0392b", self.page.gp_error_label.styleSheet())
        self.assertTrue(self.page.gp_error_label.isVisibleTo(self.page))
        self.assertIn("w", self.page.gp_warnings_label.text())

    def test_run_without_scan_is_not_selectable(self):
        """没有 ``scan.json`` / ``iso.json`` 的运行：组合框里灰掉、按钮禁用。"""
        empty = self.root / "20260101-000000-0000"
        empty.mkdir()
        self.page.refresh_runs()
        index = self.page.run_combo.findData(empty.name)
        self.assertGreaterEqual(index, 0, "没有产物的运行也要列出来")
        item = self.page._combo_item(index)
        self.assertIsNotNone(item)
        self.assertFalse(item.isEnabled(), "不能选（灰掉）")
        self.page.run_combo.setCurrentIndex(index)
        self.assertFalse(self.page.generate_gp_button.isEnabled())
        self.assertIn("不能写 GP", self.page.run_hint.text())

    def test_iso_run_is_selectable(self):
        """``iso.json`` 与 ``scan.json`` 同构：ISO 运行也能写 GP。"""
        iso_dir = self.root / "20260101-000001-abcd"
        iso_dir.mkdir()
        (iso_dir / core.ARTIFACT_ISO_JSON).write_text('{"waypoints": []}',
                                                      encoding="utf-8")
        self.page.refresh_runs()
        row = next(row for row in self.page.runs if row["run_id"] == iso_dir.name)
        self.assertEqual(tools_page.gp_source(row), core.ARTIFACT_ISO_JSON)


class ConvertUrdfTest(PageCase):
    """JSON → URDF：产出 ``.urdf``，关节数与末端位姿都显示。"""

    def setUp(self):
        super().setUp()
        self.data = json.dumps(MINIMAL_CONTROLLER).encode("utf-8")
        self.page.set_controller({"name": "Robot_A.json", "bytes": self.data,
                                  "path": "Robot_A.json"})

    def test_convert_produces_urdf(self):
        self.page.model_edit.setText("h6s")
        result = self.page.convert()
        self.assertTrue(result["ok"], result.get("error"))
        self.assertTrue(result["filename"].endswith("_h6s.urdf"),
                        result["filename"])
        produced = Path(result["path"])
        self.assertTrue(produced.is_file())
        self.assertEqual(produced.suffix, ".urdf")
        self.assertIsNotNone(result["self_check_error"])
        self.assertLess(result["self_check_error"], 1e-9)
        self.assertIn("自校验误差", self.page.urdf_name_label.text())
        self.assertTrue(self.page.urdf_download_button.isEnabled())

    def test_joint_count_and_end_pose_are_shown(self):
        self.page.model_edit.setText("h6s")
        result = self.page.convert()
        self.assertTrue(result["ok"], result.get("error"))
        summary = tools_page.urdf_summary(result["path"])
        self.assertEqual(summary["joints"], 7)          # 6 轴 + base 固定关节
        self.assertIsNotNone(summary["ee_position"])
        self.assertEqual(self.page.urdf_joints_label.text(), "关节数 7")
        self.assertIn("位置 (", self.page.urdf_pose_label.text())
        self.assertIn("RPY (", self.page.urdf_pose_label.text())
        self.assertNotEqual(self.page.urdf_pose_label.text().strip(),
                            "末端位姿 —")

    def test_bad_model_shows_cli_error(self):
        """型号非法时把 ``json_to_urdf`` 的 argparse 原文写在结果区。"""
        self.page.model_edit.setText("bad model!")
        result = self.page.convert()
        self.assertFalse(result["ok"])
        self.assertIn("--model", result["error"])
        self.assertIn("--model", self.page.urdf_error_label.text())
        self.assertIn("color:#c0392b", self.page.urdf_error_label.styleSheet())


class CleanRunsTest(PageCase):
    """清理：不动正在跑的任务目录；没有活跃目录时与 core 同口径。"""

    def make_runs(self, count: int) -> list[Path]:
        """造 ``count`` 个运行目录（mtime 递增，名字也递增）。"""
        made = []
        for index in range(count):
            path = self.root / f"2026010{index + 1}-000000-abc{index}"
            path.mkdir()
            (path / "scan.json").write_text("{}", encoding="utf-8")
            os.utime(path, (1_700_000_000 + index, 1_700_000_000 + index))
            made.append(path)
        return made

    def test_active_run_is_excluded(self):
        made = self.make_runs(4)
        active = made[0]                      # 最旧的那个“还在跑”
        page = tools_page.ToolsPage(runs_root=self.root,
                                    active_dirs=lambda: [str(active)])
        try:
            page.refresh_runs()
            self.assertEqual(len(page.runs), 4)
            result = page.clean_runs(keep=1)
            removed = {Path(item).name for item in result["removed"]}
            self.assertEqual(len(removed), 2, result)   # 4 - keep1 - active1
            self.assertNotIn(active.name, removed)
            self.assertTrue(active.is_dir(), "在跑的目录不许删")
            self.assertEqual(result["kept"], 2)         # active + 最新
            # 页面入口回报删了几个；只剩 active + 最新，再清也不该动 active
            page.clean_now(keep=1)
            self.assertIn("已删除", page.clean_status_label.text())
            self.assertTrue(active.is_dir())
            self.assertEqual(len(page.runs), 2)
        finally:
            page.deleteLater()

    def test_without_active_dirs_matches_core(self):
        """没有活跃目录时：与 ``core.clean_runs`` 删的目录完全一样。"""
        made = self.make_runs(4)
        mirror = Path(tempfile.mkdtemp(prefix="tools-runs-mirror-"))
        try:
            for path in made:
                copy = mirror / path.name
                copy.mkdir()
                (copy / "scan.json").write_text("{}", encoding="utf-8")
                stat = path.stat()
                os.utime(copy, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            page_result = self.page.clean_runs(keep=2)
            core_result = core.clean_runs(2, root=mirror)
            self.assertEqual([Path(item).name for item in page_result["removed"]],
                             [Path(item).name for item in core_result["removed"]])
            self.assertEqual(page_result["kept"], core_result["kept"])
        finally:
            shutil.rmtree(mirror, ignore_errors=True)

    def test_clean_reports_zero_when_nothing_to_remove(self):
        self.make_runs(2)
        result = self.page.clean_now(keep=jobs.KEEP_RUNS)
        self.assertEqual(result["removed"], [])
        self.assertIn("已删除 0 个", self.page.clean_status_label.text())


if __name__ == "__main__":                              # pragma: no cover
    unittest.main()
