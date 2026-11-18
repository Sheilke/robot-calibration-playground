#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物理网卡识别与指纹：虚拟/伪造要滤掉，同一块网卡指纹要稳定。"""
import unittest

from licensing.hwid import (FINGERPRINT_PREFIX, describe_machine, fingerprint,
                            is_usable_mac, machine_fingerprints,
                            physical_macs)

#: 各类"不能用"的 MAC 及原因（回归清单，改过滤逻辑时先过这张表）。
REJECTED = {
    "00:00:00:00:00:00": "全零",
    "ff:ff:ff:ff:ff:ff": "广播",
    "01:00:5e:00:00:01": "组播位",
    "02:42:ac:11:00:02": "本地管理位 + Docker OUI",
    "0a:00:27:00:00:01": "本地管理位 + VirtualBox OUI",
    "52:54:00:12:34:56": "QEMU/KVM OUI",
    "00:15:5d:01:02:03": "Hyper-V OUI",
    "00:50:56:c0:00:08": "VMware OUI",
    "00:0c:29:aa:bb:cc": "VMware OUI",
    "00:1c:42:00:00:01": "Parallels OUI",
    "08:00:27:11:22:33": "VirtualBox OUI",
}

#: 两块真实网卡 + 同一块的几种常见写法
ACCEPTED = ("f0:4e:a4:90:4b:2c", "c8:95:ce:02:5d:cd",
            "F0-4E-A4-90-4B-2C", "f04e.a490.4b2c")
SAME_MAC_NOTATIONS = ("f0:4e:a4:90:4b:2c", "F0-4E-A4-90-4B-2C",
                      "f04e.a490.4b2c", "f04ea4904b2c")


class UsableMacTest(unittest.TestCase):
    def test_rejects_virtual_and_spoofed(self):
        for mac, why in REJECTED.items():
            with self.subTest(mac=mac):
                self.assertFalse(is_usable_mac(mac), f"{mac} 应被拒（{why}）")

    def test_accepts_real_looking_physical(self):
        for mac in ACCEPTED:
            with self.subTest(mac=mac):
                self.assertTrue(is_usable_mac(mac))

    def test_rejects_garbage(self):
        for text in ("", "zz:zz", "aa:bb", "not-a-mac", "f0:4e:a4:90:4b"):
            with self.subTest(text=text):
                self.assertFalse(is_usable_mac(text))


class FingerprintTest(unittest.TestCase):
    def test_stable_and_normalised(self):
        """同一块网卡的多种写法必须给同一个指纹（不同网卡给不同的）。"""
        values = {fingerprint(mac) for mac in SAME_MAC_NOTATIONS}
        self.assertEqual(len(values), 1, values)
        self.assertNotEqual(fingerprint("f0:4e:a4:90:4b:2c"),
                            fingerprint("c8:95:ce:02:5d:cd"))

    def test_differs_per_mac(self):
        self.assertNotEqual(fingerprint("f0:4e:a4:90:4b:2c"),
                            fingerprint("c8:95:ce:02:5d:cd"))

    def test_is_irreversible_and_namespaced(self):
        result = fingerprint("f0:4e:a4:90:4b:2c")
        self.assertEqual(len(result), 32)
        self.assertNotIn("f0", result)
        # 换前缀等于让所有旧授权失效——这里钉住前缀值
        self.assertEqual(FINGERPRINT_PREFIX, "laser-lic/v1")

    def test_rejects_invalid(self):
        with self.assertRaises(ValueError):
            fingerprint("not-a-mac")


class ThisMachineTest(unittest.TestCase):
    """本机枚举：不崩、指纹与 MAC 一一对应、虚拟接口被排除。"""

    def test_describe_machine_shape(self):
        info = describe_machine()
        self.assertEqual(sorted(info),
                         ["fingerprints", "machine", "macs", "system"])
        self.assertEqual(len(info["macs"]), len(info["fingerprints"]))
        for mac in info["macs"]:
            self.assertTrue(is_usable_mac(mac), mac)

    def test_fingerprints_match_physical_macs(self):
        self.assertEqual(machine_fingerprints(),
                         sorted(fingerprint(mac) for mac in physical_macs()))

    def test_virtual_interfaces_are_filtered_on_linux(self):
        """本机若有 docker/veth/br-/tailscale 之类，不能出现在结果里。"""
        import platform
        if platform.system() != "Linux":
            self.skipTest("只在 Linux 上校验 /sys 过滤")
        from pathlib import Path
        virtual = []
        for entry in Path("/sys/class/net").iterdir():
            if not (entry / "device").exists():
                try:
                    virtual.append((entry / "address").read_text().strip())
                except OSError:
                    pass
        for mac in virtual:
            if not is_usable_mac(mac):        # 本身就被判定不可用
                continue
            # 即使 MAC 看起来正常，只要接口是虚拟的就不该进结果
            self.assertNotIn(mac, physical_macs(), f"{mac} 是虚拟接口的地址")


if __name__ == "__main__":
    unittest.main()
