# -*- coding: utf-8 -*-
"""启动闸门 ``laser_desktop.licensing``：六种失败各说各的人话，且永不抛异常。

为什么这里造**真**许可证
------------------------
这一层唯一的价值就是"签名对不对、这台机器对不对"。把
``verify_license`` mock 掉之后，测试只剩一句 ``assert called``——真到了现场，
错的是签名规范、是 base64、是指纹大小写，而测试全绿。所以每个用例都用
``licensing`` 现生成一对 Ed25519 密钥、现签一份 ``license.json``，走完整条
验签路径（只有"本机指纹"与"今天"两处注入，底层 ``verify_license`` 本来就
支持这两个参数，不是测试专用后门）。

私钥只活在临时目录里（``generate_keypair`` 写在 tmp），仓库里一份都没有。
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from laser_desktop import licensing as gate

try:
    from licensing import license as lic
    from licensing.hwid import machine_fingerprints
    _HAVE_CRYPTO = True
except Exception:                          # pragma: no cover - 依赖缺失
    _HAVE_CRYPTO = False

REPO = Path(__file__).resolve().parents[3]
PY_SRC = REPO / "python"

#: 本机指纹（注入用）：形如真指纹的 32 位十六进制。
LOCAL_A = "0123456789abcdef0123456789abcdef"
LOCAL_B = "fedcba9876543210fedcba9876543210"
#: 另一台机器的指纹（既不是本机，也不在授权里）。
OTHER_A = "11111111111111111111111111111111"
OTHER_B = "22222222222222222222222222222222"

#: 固定"今天"，让过期判定与宿主时钟无关。
TODAY = _dt.date(2026, 11, 18)


def _write_license(folder: Path, private_key, *, fingerprints, min_match=1,
                   expires=None, licensee="XX 焊接有限公司",
                   serial="R1-0001") -> Path:
    """用**真私钥**签一份许可证写到 ``folder/license.json``。"""
    payload = lic.build_payload(licensee, serial, fingerprints,
                                expires=expires, min_match=min_match)
    path = folder / "license.json"
    path.write_bytes(lic.sign_payload(payload, private_key))
    return path


@unittest.skipUnless(_HAVE_CRYPTO, "需要 cryptography（requirements-license.txt）")
class GateTestBase(unittest.TestCase):
    """每个用例一套全新的密钥 + 临时目录。"""

    def setUp(self) -> None:
        self.tmp = Path(self.enterContext(
            tempfile.TemporaryDirectory(prefix="laser-lic-")))
        self.private_path = self.tmp / "private.pem"
        self.public_path = self.tmp / "public.pem"
        lic.generate_keypair(self.private_path, self.public_path)
        self.private_key = lic.load_private_key(self.private_path)
        self.public_pem = self.public_path.read_bytes()

    def issue(self, **kwargs) -> Path:
        kwargs.setdefault("fingerprints", [LOCAL_A])
        return _write_license(self.tmp, self.private_key, **kwargs)

    def check(self, path=None, **kwargs):
        """默认注入本机指纹、公钥与固定日期（只在用例里覆盖需要的那个）。"""
        kwargs.setdefault("public_key_pem", self.public_pem)
        kwargs.setdefault("local_fingerprints", [LOCAL_A])
        kwargs.setdefault("today", TODAY)
        return gate.check(path, **kwargs)


class SuccessTest(GateTestBase):
    """1. 正常证：放行，摘要里有授权对象与到期日。"""

    def test_valid_license_passes_with_summary(self):
        path = self.issue(expires="2027-11-18")
        result = self.check(path)
        self.assertTrue(result.ok, result.message)
        self.assertIn("XX 焊接有限公司", result.message)
        self.assertIn("2027-11-18", result.message)
        self.assertIn("R1-0001", result.message)

    def test_valid_license_detail_carries_forensics(self):
        """详细信息要有路径与命中数——现场求助时直接发这一段。"""
        path = self.issue(expires="2027-11-18")
        result = self.check(path)
        self.assertIn(str(path), result.detail)
        self.assertIn("命中 1/1", result.detail)

    def test_permanent_license_says_no_expiry(self):
        result = self.check(self.issue(expires=None))
        self.assertTrue(result.ok, result.message)
        self.assertIn("永久", result.message)

    def test_min_match_is_honoured(self):
        """绑定两个指纹、要求命中两个：只对上一个不该放行。"""
        path = self.issue(fingerprints=[LOCAL_A, OTHER_A], min_match=2)
        result = self.check(path, local_fingerprints=[LOCAL_A, LOCAL_B])
        self.assertFalse(result.ok, result.message)
        self.assertIn("命中 1/2", result.message)

        ok = self.check(path, local_fingerprints=[LOCAL_A, OTHER_A])
        self.assertTrue(ok.ok, ok.message)


class SignatureTest(GateTestBase):
    """2. 签名被改：验签失败，文案里点出"签名"。"""

    def test_tampered_payload_rejected(self):
        path = self.issue(expires="2027-11-18")
        document = json.loads(path.read_text(encoding="utf-8"))
        document["payload"]["expires"] = "2099-12-31"   # 手改到期日
        path.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
        result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("签名", result.message)

    def test_license_signed_by_another_key_rejected(self):
        """换一对密钥签的证＝不是本程序签发的（公钥不匹配）。"""
        other_private = self.tmp / "other_private.pem"
        lic.generate_keypair(other_private, self.tmp / "other_public.pem")
        foreign_dir = self.tmp / "foreign"
        foreign_dir.mkdir()
        foreign = _write_license(foreign_dir, lic.load_private_key(other_private),
                                 fingerprints=[LOCAL_A], expires="2027-11-18")
        result = self.check(foreign)
        self.assertFalse(result.ok)
        self.assertIn("签名", result.message)

    def test_broken_base64_rejected(self):
        path = self.issue(expires="2027-11-18")
        document = json.loads(path.read_text(encoding="utf-8"))
        document["sig"] = "不是 base64!!!"
        path.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
        result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("签名", result.message)


class MachineTest(GateTestBase):
    """3. 别人家的机器：文案里必须带**本机指纹**（现场要能直接复制发给我们）。"""

    def test_foreign_fingerprints_are_echoed_back(self):
        path = self.issue(fingerprints=[OTHER_A, OTHER_B], min_match=2)
        result = self.check(path, local_fingerprints=[LOCAL_A, LOCAL_B])
        self.assertFalse(result.ok)
        self.assertIn("命中 0/2", result.message)
        self.assertIn(LOCAL_A, result.message)
        self.assertIn(LOCAL_B, result.message)
        # 逐行版本留在 detail 里，"把这两行发给供应商"照抄就行。
        self.assertIn(LOCAL_A, result.detail)
        self.assertIn(LOCAL_B, result.detail)

    def test_no_physical_nic_has_its_own_message(self):
        path = self.issue(expires="2027-11-18")
        result = self.check(path, local_fingerprints=[])
        self.assertFalse(result.ok)
        self.assertIn("网卡", result.message)


class ExpiryTest(GateTestBase):
    """4. 过期：文案里有到期日期。"""

    def test_expired_license_names_the_date(self):
        path = self.issue(expires="2026-12-31")
        result = self.check(path, today=_dt.date(2027, 1, 5))
        self.assertFalse(result.ok)
        self.assertIn("2026-12-31", result.message)
        self.assertIn("到期", result.message)

    def test_expiry_on_the_last_day_still_works(self):
        path = self.issue(expires="2026-12-31")
        result = self.check(path, today=_dt.date(2026, 12, 31))
        self.assertTrue(result.ok, result.message)

    def test_bad_expiry_format_is_reported(self):
        path = self.issue(expires="2026-12-31")
        document = json.loads(path.read_text(encoding="utf-8"))
        document["payload"]["expires"] = "2026/12/31"     # 斜杠不是 ISO
        path.write_bytes(lic.sign_payload(document["payload"], self.private_key))
        result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("有效期", result.message)


class MissingFileTest(GateTestBase):
    """5. 文件不存在：文案里给出**应该放哪**的路径。"""

    def test_explicit_missing_path_is_named(self):
        missing = self.tmp / "不存在" / "license.json"
        result = self.check(missing)
        self.assertFalse(result.ok)
        self.assertIn("联系供应商", result.message)
        self.assertNotIn(str(missing), result.message)
        self.assertIn(str(missing), result.detail)
        self.assertIn("未找到许可证文件", result.message)

    def test_auto_search_names_the_preferred_location(self):
        """不指定路径时，提示的必须是首选落盘位置（用户照做就能过）。"""
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(gate.ENV_LICENSE, None)
            with mock.patch.object(gate.paths, "data_root",
                                   return_value=self.tmp / "数据目录"):
                (self.tmp / "数据目录").mkdir(exist_ok=True)
                self.assertIsNone(gate.find_license_file())
                self.assertEqual(gate.preferred_path(),
                                 self.tmp / "数据目录" / "license.json")
                result = self.check()
        self.assertFalse(result.ok)
        # 主提示**不**给出文件该放哪：那句话读起来像"自己拷一个进去就行"，
        # 而许可证绑定本机指纹，现场照做只会得到一个无效文件。
        self.assertIn("联系供应商", result.message)
        self.assertNotIn(str(self.tmp / "数据目录" / "license.json"),
                         result.message)
        # 路径仍然记录在 detail 里（支持与日志要用），只是不给用户看
        self.assertIn(str(self.tmp / "数据目录" / "license.json"), result.detail)

    def test_env_var_points_at_the_license(self):
        path = self.issue(expires="2027-11-18")
        with mock.patch.dict(os.environ, {gate.ENV_LICENSE: str(path)}):
            self.assertEqual(gate.find_license_file(), path)
            result = self.check()
        self.assertTrue(result.ok, result.message)

    def test_frozen_build_also_looks_in_program_data(self):
        """冻结包会多认一个整机位置：%PROGRAMDATA%/LaserCalibration/license.json。"""
        shared = self.tmp / "程序数据"
        target = shared / gate.PROGRAM_DATA_APP / gate.LICENSE_NAME
        target.parent.mkdir(parents=True)
        target.write_bytes(self.issue(expires="2027-11-18").read_bytes())
        with mock.patch.dict(os.environ, {"PROGRAMDATA": str(shared)}, clear=False):
            os.environ.pop(gate.ENV_LICENSE, None)
            with mock.patch.object(gate.paths, "is_frozen", return_value=True), \
                 mock.patch.object(gate.paths, "data_root",
                                   return_value=self.tmp / "空"):
                (self.tmp / "空").mkdir(exist_ok=True)
                self.assertEqual(gate.find_license_file(), target)
                result = self.check()
        self.assertTrue(result.ok, result.message)

    def test_json_garbage_is_reported_as_unreadable(self):
        path = self.tmp / "license.json"
        path.write_text("这不是 JSON", encoding="utf-8")
        result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("读不了", result.message)


class PublicKeyTest(GateTestBase):
    """6. 公钥缺失：明确说"安装包可能不完整"，并让人重装。"""

    def _empty_asset_root(self):
        """把资产根指到一个空目录：内嵌常量与随包文件都取不到。"""
        empty = self.tmp / "空的资产根"
        empty.mkdir(parents=True, exist_ok=True)
        return mock.patch.object(gate.paths, "asset_root", return_value=empty)

    def test_public_key_returns_none_when_nothing_embedded(self):
        self.assertIsNone(gate.EMBEDDED_PUBLIC_KEY_PEM)
        with self._empty_asset_root():
            self.assertIsNone(gate.public_key())

    def test_missing_public_key_message(self):
        path = self.issue(expires="2027-11-18")
        with self._empty_asset_root():
            result = gate.check(path, public_key_pem=None,
                                local_fingerprints=[LOCAL_A], today=TODAY)
        self.assertFalse(result.ok)
        self.assertIn("公钥", result.message)
        self.assertIn("重新安装", result.message)

    def test_public_key_is_read_from_the_repo_layout(self):
        """源码运行时公钥在 ``data/assets/license_public.pem``（构建机放那儿）。"""
        root = self.tmp / "资产根"
        (root / "data" / "assets").mkdir(parents=True)
        (root / "data" / "assets" / gate.PUBLIC_KEY_NAME).write_bytes(self.public_pem)
        with mock.patch.object(gate.paths, "asset_root", return_value=root):
            self.assertEqual(gate.public_key(), self.public_pem)
            path = self.issue(expires="2027-11-18")
            result = gate.check(path, local_fingerprints=[LOCAL_A], today=TODAY)
        self.assertTrue(result.ok, result.message)

    def test_public_key_is_read_from_the_frozen_layout(self):
        """冻结后公钥在 ``_internal/assets/license_public.pem``。"""
        root = self.tmp / "assets"
        root.mkdir()
        (root / gate.PUBLIC_KEY_NAME).write_bytes(self.public_pem)
        with mock.patch.object(gate.paths, "asset_root", return_value=root):
            self.assertEqual(gate.public_key(), self.public_pem)

    def test_corrupt_public_key_message(self):
        path = self.issue(expires="2027-11-18")
        result = self.check(path, public_key_pem="-----BEGIN PUBLIC KEY-----\n坏\n".encode("utf-8"))
        self.assertFalse(result.ok)
        self.assertIn("公钥", result.message)


class SkipSwitchTest(GateTestBase):
    """7. ``LASER_SKIP_LICENSE``：源码模式跳过，冻结包不认。"""

    def setUp(self) -> None:
        super().setUp()
        env = mock.patch.dict(os.environ, {gate.ENV_SKIP: "1"})
        env.start()
        self.addCleanup(env.stop)

    def test_source_run_can_skip(self):
        self.assertFalse(gate.paths.is_frozen())
        result = gate.check(public_key_pem=self.public_pem)
        self.assertTrue(result.ok)
        self.assertIn("跳过", result.message)

    def test_explicit_skip_argument_also_works(self):
        with mock.patch.dict(os.environ, {gate.ENV_SKIP: "0"}):
            result = gate.check(skip=True, public_key_pem=self.public_pem)
        self.assertTrue(result.ok)
        self.assertIn("跳过", result.message)

    def test_frozen_build_ignores_the_switch(self):
        """核心断言：冻结包里该变量**不起作用**，照常做完整校验。

        起子进程是因为 ``sys.frozen`` 必须在 import/解析之前就位（
        ``paths.is_frozen()`` 是运行时读的，但子进程里验更接近真机）。
        """
        path = self.issue(expires="2027-11-18")
        code = (
            "import json, sys\n"
            "sys.frozen = True\n"
            "sys._MEIPASS = sys.argv[2]\n"
            "from laser_desktop import licensing as gate\n"
            "result = gate.check(sys.argv[1], "
            "public_key_pem=open(sys.argv[3], 'rb').read(), "
            "local_fingerprints=[sys.argv[4]], "
            "today=__import__('datetime').date(2026, 11, 18))\n"
            "print(json.dumps({'ok': result.ok, 'message': result.message}))\n"
        )
        env = {key: value for key, value in os.environ.items()
               if key.upper() != "PYTHONPATH"}
        env.update({"PYTHONPATH": str(PY_SRC), gate.ENV_SKIP: "1"})
        proc = subprocess.run(
            [sys.executable, "-c", code, str(path), str(self.tmp / "_internal"),
             str(self.public_path), LOCAL_A],
            cwd=str(REPO), capture_output=True, text=True, timeout=300, env=env)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        payload = json.loads(proc.stdout.strip().splitlines()[-1])
        self.assertTrue(payload["ok"], payload["message"])
        self.assertNotIn("跳过", payload["message"])
        self.assertIn("XX 焊接有限公司", payload["message"])
        # 变量被忽略这件事必须留下痕迹，否则运维会以为"设了但没用"是 bug。
        self.assertIn("被忽略", proc.stderr)


class NeverRaisesTest(GateTestBase):
    """8. 任何异常都转成 GateResult——现场绝不能看到 traceback。"""

    def test_path_object_that_explodes_is_caught(self):
        class Exploding:
            def __fspath__(self):
                raise RuntimeError("路径解析炸了")

        result = self.check(Exploding())
        self.assertFalse(result.ok)
        self.assertIn("意外", result.message)
        self.assertIn("RuntimeError", result.message)

    def test_unexpected_payload_shape_is_caught(self):
        """载荷里 ``fingerprints`` 不是列表（被手改又重签过）也不能抛。"""
        payload = {"v": lic.LICENSE_VERSION, "licensee": "X", "serial": "S",
                   "issued": "2026-11-18", "expires": None,
                   "fingerprints": 12345, "min_match": "两个", "features": []}
        path = self.tmp / "license.json"
        path.write_bytes(lic.sign_payload(payload, self.private_key))
        result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("意外", result.message)

    def test_missing_cryptography_dependency_is_reported(self):
        path = self.issue(expires="2027-11-18")
        with mock.patch.dict(sys.modules, {"cryptography": None,
                                           "cryptography.hazmat": None,
                                           "cryptography.hazmat.primitives": None,
                                           "cryptography.hazmat.primitives.serialization": None}):
            result = self.check(path)
        self.assertFalse(result.ok)
        self.assertIn("cryptography", result.message)


class _FakeMessageBox:
    """替身 QMessageBox：记下被设了哪些字段（对话框内容就是这里要验的东西）。"""

    Critical = "critical"
    Ok = "ok"
    made: list = []

    def __init__(self) -> None:
        self.icon = None
        self.title = None
        self.text = None
        self.detail = None
        self.buttons = None
        self.executed = False
        _FakeMessageBox.made.append(self)

    def setIcon(self, value):                 # noqa: N802 - 模仿 Qt
        self.icon = value

    def setWindowTitle(self, value):          # noqa: N802
        self.title = value

    def setText(self, value):                 # noqa: N802
        self.text = value

    def setDetailedText(self, value):         # noqa: N802
        self.detail = value

    def setStandardButtons(self, value):      # noqa: N802
        self.buttons = value

    def exec(self):                           # noqa: N802 - PySide6 名字
        self.executed = True


class _FakeApplication:
    """替身 QApplication：只关心"有没有被建过"。"""

    made: list = []

    def __init__(self, *_args, **_kwargs) -> None:
        _FakeApplication.made.append(self)

    @classmethod
    def instance(cls):
        return None


class MainWiringTest(GateTestBase):
    """``__main__`` 的接线：弹窗内容、退出码 3、诊断开关不被闸门拦住。"""

    def setUp(self) -> None:
        super().setUp()
        _FakeMessageBox.made = []
        _FakeApplication.made = []
        from laser_desktop import __main__ as entry
        self.entry = entry
        fake_qt = mock.Mock(QApplication=_FakeApplication,
                            QMessageBox=_FakeMessageBox)
        patches = [
            mock.patch.object(entry, "QtWidgets", fake_qt),
            mock.patch.object(entry, "apply_chinese_font", lambda app: "fake"),
        ]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def _bad_result(self):
        path = self.issue(fingerprints=[OTHER_A, OTHER_B], min_match=2)
        return path, gate.check(path, public_key_pem=self.public_pem,
                                local_fingerprints=[LOCAL_A, LOCAL_B],
                                today=TODAY)

    def test_source_run_refuses_with_code_three_and_no_dialog(self):
        _path, result = self._bad_result()
        stderr = mock.patch("sys.stderr")
        with stderr:
            self.assertEqual(self.entry._refuse(result), gate.EXIT_LICENSE)
        self.assertEqual(_FakeMessageBox.made, [], "源码模式不应该弹框")
        self.assertEqual(_FakeApplication.made, [], "源码模式不建 QApplication")

    def test_frozen_run_shows_the_dialog(self):
        _path, result = self._bad_result()
        with mock.patch.object(self.entry.paths, "is_frozen", return_value=True), \
                mock.patch("sys.stderr"):
            self.assertEqual(self.entry._refuse(result), gate.EXIT_LICENSE)
        self.assertEqual(len(_FakeMessageBox.made), 1)
        box = _FakeMessageBox.made[0]
        self.assertEqual(box.title, "许可证校验未通过")
        self.assertEqual(box.text, result.message)
        self.assertEqual(box.detail, result.detail)
        self.assertTrue(box.executed, "弹窗没进 exec()，现场就看不到任何提示")

    def test_dialog_is_chinese_and_copyable(self):
        """正文全是人话；本机指纹在"详细信息"里，可以选中复制发给供应商。"""
        _path, result = self._bad_result()
        with mock.patch.object(self.entry.paths, "is_frozen", return_value=True), \
                mock.patch("sys.stderr"):
            self.entry._refuse(result)
        box = _FakeMessageBox.made[0]
        self.assertIn(LOCAL_A, box.text)
        self.assertIn(LOCAL_A, box.detail)
        self.assertNotIn("Traceback", box.detail)

    def test_main_returns_three_before_creating_any_window(self):
        """端到端：没证时 ``main()`` 直接 3，**一个 QApplication 都不建**。"""
        with mock.patch.dict(os.environ, {}, clear=False), \
                mock.patch.object(gate.paths, "data_root",
                                  return_value=self.tmp / "无授权"), \
                mock.patch("sys.stderr"):
            os.environ.pop(gate.ENV_LICENSE, None)
            os.environ.pop(gate.ENV_SKIP, None)
            (self.tmp / "无授权").mkdir(exist_ok=True)
            self.assertEqual(self.entry.main([]), gate.EXIT_LICENSE)
        self.assertEqual(_FakeApplication.made, [])

    def test_print_paths_bypasses_the_gate_and_reports_it(self):
        """诊断开关：没证也得能跑（返回 0），但把许可证状态报出来。"""
        from io import StringIO
        buffer = StringIO()
        with mock.patch.dict(os.environ, {}, clear=False), \
                mock.patch.object(gate.paths, "data_root",
                                  return_value=self.tmp / "无授权"), \
                mock.patch("sys.stdout", buffer):
            os.environ.pop(gate.ENV_LICENSE, None)
            os.environ.pop(gate.ENV_SKIP, None)
            (self.tmp / "无授权").mkdir(exist_ok=True)
            self.assertEqual(self.entry.main(["--print-paths"]), 0)
        text = buffer.getvalue()
        self.assertIn("许可证", text)
        self.assertIn("未找到许可证文件", text)
        self.assertIn(str(self.tmp / "无授权" / gate.LICENSE_NAME), text)

    def test_print_paths_reports_a_valid_license(self):
        """绑定到**本机真实指纹**的证：命令行诊断也得能看出"证是好的"。"""
        local = machine_fingerprints()
        if not local:                     # pragma: no cover - 无物理网卡的机器
            self.skipTest("本机没有物理网卡，无法造本机可用的证")
        path = self.issue(fingerprints=local, expires="2027-11-18")
        from io import StringIO
        buffer = StringIO()
        with mock.patch.dict(os.environ, {gate.ENV_LICENSE: str(path)}), \
                mock.patch.object(gate, "public_key", return_value=self.public_pem), \
                mock.patch("sys.stdout", buffer):
            self.assertEqual(self.entry.main(["--print-paths"]), 0)
        self.assertIn("通过", buffer.getvalue())
        self.assertIn("XX 焊接有限公司", buffer.getvalue())


if __name__ == "__main__":
    unittest.main()
