#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""授权文件：签名、防篡改、机器绑定、有效期。"""
import datetime as dt
import tempfile
import unittest
from pathlib import Path

from licensing.hwid import fingerprint
from licensing.license import (LicenseError, build_payload, check_machine,
                               generate_keypair, load_license,
                               load_private_key, load_public_key,
                               sign_payload, verify_license)

MAC_A = "f0:4e:a4:90:4b:2c"
MAC_B = "c8:95:ce:02:5d:cd"
FP_A = fingerprint(MAC_A)
FP_B = fingerprint(MAC_B)


class _Keys(unittest.TestCase):
    """每个用例一套临时密钥（Ed25519 生成很快）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.private_pem = self.dir / "private.pem"
        self.public_pem = self.dir / "public.pem"
        generate_keypair(self.private_pem, self.public_pem)
        self.private = load_private_key(self.private_pem)
        self.public = load_public_key(self.public_pem)

    def tearDown(self):
        self._tmp.cleanup()

    def issue(self, payload, name="t.lic") -> Path:
        path = self.dir / name
        path.write_bytes(sign_payload(payload, self.private))
        return path


class BuildPayloadTest(unittest.TestCase):
    def test_normalises_and_defaults(self):
        payload = build_payload(" XX 公司 ", "R1-0001", [FP_A.upper()],
                                issued="2026-11-18")
        self.assertEqual(payload["licensee"], "XX 公司")
        self.assertEqual(payload["serial"], "R1-0001")
        self.assertEqual(payload["fingerprints"], [FP_A])   # 统一小写
        self.assertEqual(payload["issued"], "2026-11-18")
        self.assertIsNone(payload["expires"])               # 默认永久
        self.assertEqual(payload["min_match"], 1)

    def test_rejects_bad_input(self):
        for kwargs in ({"fingerprints": []},
                       {"fingerprints": [FP_A], "min_match": 0},
                       {"fingerprints": [FP_A], "min_match": 2},
                       {"fingerprints": [FP_A], "expires": "2027/11/18"},
                       {"fingerprints": [FP_A], "features": ["nope"]}):
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError):
                    build_payload("客户", "S1", kwargs.pop("fingerprints"),
                                  **kwargs)


class SignVerifyTest(_Keys):
    def test_round_trip_on_bound_machine(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A, FP_B],
                                issued="2026-11-18")
        path = self.issue(payload)
        summary = verify_license(path, self.public,
                                 local_fingerprints=[FP_A],
                                 today=dt.date(2026, 11, 18))
        self.assertEqual(summary["licensee"], "XX 公司")
        self.assertEqual(summary["serial"], "R1-0001")
        self.assertIsNone(summary["expires"])

    def test_tampering_any_field_breaks_the_signature(self):
        """改客户名/有效期/指纹，都应验签失败（这是授权的核心保证）。"""
        payload = build_payload("XX 公司", "R1-0001", [FP_A], issued="2026-11-18")
        path = self.issue(payload)
        original = path.read_text(encoding="utf-8")
        for old, new in (('"XX 公司"', '"别家 公司"'),
                         ('"R1-0001"', '"R1-9999"'),
                         ("null", '"2099-01-01"')):
            with self.subTest(change=f"{old} -> {new}"):
                path.write_text(original.replace(old, new, 1), encoding="utf-8")
                with self.assertRaises(LicenseError) as caught:
                    verify_license(path, self.public, local_fingerprints=[FP_A],
                                   today=dt.date(2026, 11, 18))
                self.assertIn("签名校验失败", str(caught.exception))
        path.write_text(original, encoding="utf-8")

    def test_wrong_public_key_is_rejected(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A], issued="2026-11-18")
        path = self.issue(payload)
        other_private = self.dir / "other.pem"
        other_public = self.dir / "other.pub"
        generate_keypair(other_private, other_public)
        with self.assertRaises(LicenseError):
            verify_license(path, load_public_key(other_public),
                           local_fingerprints=[FP_A])

    def test_missing_or_broken_file(self):
        with self.assertRaises(LicenseError):
            verify_license(self.dir / "nope.lic", self.public)
        broken = self.dir / "broken.lic"
        broken.write_text("{not json", encoding="utf-8")
        with self.assertRaises(LicenseError):
            verify_license(broken, self.public)
        empty = self.dir / "empty.lic"
        empty.write_text('{"payload": {}, "sig": "AA=="}', encoding="utf-8")
        with self.assertRaises(LicenseError):
            load_license(empty)


class MachineBindingTest(_Keys):
    def test_other_machine_is_refused_with_actionable_message(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A], issued="2026-11-18")
        path = self.issue(payload)
        with self.assertRaises(LicenseError) as caught:
            verify_license(path, self.public, local_fingerprints=[FP_B],
                           today=dt.date(2026, 11, 18))
        message = str(caught.exception)
        self.assertIn("不是授权绑定的机器", message)
        self.assertIn("重新签发", message)

    def test_min_match_controls_tolerance(self):
        """绑两块、min_match=2：只在一块的机器上应被拒（换网卡的容差可控）。"""
        strict = build_payload("严格", "S1", [FP_A, FP_B], min_match=2,
                               issued="2026-11-18")
        with self.assertRaises(LicenseError):
            check_machine(strict, [FP_A])
        self.assertEqual(check_machine(strict, [FP_A, FP_B])["required"], 2)
        # 默认 min_match=1：命中任意一块即可（关 Wi-Fi 不至于失效）
        loose = build_payload("宽松", "S2", [FP_A, FP_B], issued="2026-11-18")
        self.assertEqual(len(check_machine(loose, [FP_B])["matched"]), 1)

    def test_no_physical_nic_is_reported_clearly(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A], issued="2026-11-18")
        with self.assertRaises(LicenseError) as caught:
            check_machine(payload, [])
        self.assertIn("读不到本机物理网卡", str(caught.exception))


class ExpiryTest(_Keys):
    def test_expired_is_refused(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A],
                                expires="2026-12-31", issued="2026-01-01")
        path = self.issue(payload)
        verify_license(path, self.public, local_fingerprints=[FP_A],
                       today=dt.date(2026, 12, 31))        # 当天仍有效
        with self.assertRaises(LicenseError) as caught:
            verify_license(path, self.public, local_fingerprints=[FP_A],
                           today=dt.date(2027, 1, 1))
        self.assertIn("已过期", str(caught.exception))

    def test_perpetual_never_expires(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A],
                                issued="2026-11-18")
        path = self.issue(payload)
        summary = verify_license(path, self.public, local_fingerprints=[FP_A],
                                 today=dt.date(2099, 1, 1))
        self.assertIsNone(summary["expires"])


class VersionTest(_Keys):
    def test_unknown_version_is_refused(self):
        payload = build_payload("XX 公司", "R1-0001", [FP_A], issued="2026-11-18")
        payload["v"] = 99
        path = self.issue(payload)
        with self.assertRaises(LicenseError) as caught:
            verify_license(path, self.public, local_fingerprints=[FP_A])
        self.assertIn("版本", str(caught.exception))
