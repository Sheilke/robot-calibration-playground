# -*- coding: utf-8 -*-
"""物理网卡 MAC 的枚举与指纹（授权绑定的硬件标识）。

设计要点
--------
1. **只认物理网卡**：虚拟网卡（Hyper-V / VMware / Docker / VPN / 蓝牙 PAN /
   Tailscale 等）的 MAC 一重启就变、或可随手改，不能用来绑机器。
2. **只认烧录地址**：Windows 10/11 支持"随机硬件地址"，读到的当前 MAC 可能是
   随机值；因此优先用 ``PermanentAddress``（网卡固件里的地址）。
   **绝不**读注册表 ``NetworkAddress``（那是可伪造的覆盖值）。
3. **指纹不可逆**：对外只出现 ``sha256(前缀 + MAC)`` 的十六进制，授权文件里不存
   原始 MAC，避免把客户的硬件地址带到别处。
4. **多网卡容忍**：一台机器常有线+无线两块；授权里存一组指纹，校验时只要命中
   ``min_match`` 个就算这台机器（默认 1 个），换线/关 Wi-Fi 不会失效。

已知边界（务必知情）
--------------------
* 这是**威慑与追溯**，不是不可破解的 DRM：Python 程序可以被改，MAC 可以用驱动
  伪造。它挡的是"把安装包拷到另一台机器直接用"，并让授权可追（客户名 + 序列号）。
* 换网卡/换主板需要重新签发授权（属于运维流程，不是 bug）。
"""
from __future__ import annotations

import hashlib
import platform
import re
import subprocess
import sys

#: 指纹前缀：换前缀等于让所有旧指纹失效，改它要慎重。
FINGERPRINT_PREFIX = "laser-lic/v1"

#: 已知虚拟化厂商的 OUI（前三字节）。只作为第二道防线——第一道是"是否物理网卡"。
VIRTUAL_OUIS = frozenset({
    "00:05:69", "00:0c:29", "00:1c:14", "00:50:56",   # VMware
    "08:00:27", "0a:00:27",                            # VirtualBox
    "00:15:5d",                                        # Hyper-V
    "00:1c:42",                                        # Parallels
    "52:54:00",                                        # QEMU / KVM
    "00:16:3e",                                        # Xen
    "02:42:ac", "02:42",                               # Docker bridge
    "0a:58:0a",                                        # Kubernetes / CNI
})

#: 名字里带这些字样的接口一律当虚拟（Linux 侧的补充过滤）。
VIRTUAL_NAME_PATTERNS = re.compile(
    r"^(lo|docker|veth|br-|virbr|vmnet|vboxnet|tun|tap|wg|tailscale|zt|"
    r"mihomo|clash|utun|awdl|llw|bridge|anpi|gif|stf|ipsec|ppp)",
    re.IGNORECASE)


def _normalise(mac: str) -> str:
    """把 ``AA-BB-CC-DD-EE-FF`` / ``aabb.ccdd.eeff`` 等统一成小写冒号形式。

    Raises:
        ValueError: 不是 12 位十六进制。
    """
    digits = re.sub(r"[^0-9A-Fa-f]", "", str(mac)).lower()
    if len(digits) != 12:
        raise ValueError(f"MAC 需 12 位十六进制，当前 {mac!r}")
    return ":".join(digits[index:index + 2] for index in range(0, 12, 2))


def is_usable_mac(mac: str) -> bool:
    """这个 MAC 能不能用来绑机器（非全零/广播、非组播、非本地管理、非虚拟 OUI）。"""
    try:
        normalised = _normalise(mac)
    except ValueError:
        return False
    first = int(normalised[0:2], 16)
    if first & 0x01:                     # 组播位
        return False
    if first & 0x02:                     # 本地管理位：随机化/伪造的典型特征
        return False
    if normalised in ("00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff"):
        return False
    return normalised[:8] not in VIRTUAL_OUIS


def fingerprint(mac: str) -> str:
    """MAC -> 硬件指纹（``sha256(前缀 + 规范化 MAC)`` 的十六进制，32 位）。

    Args:
        mac: 任意常见写法的 MAC。

    Returns:
        str: 不可逆的指纹；同一块网卡任何时候都得到同一个值。
    """
    normalised = _normalise(mac)
    digest = hashlib.sha256(f"{FINGERPRINT_PREFIX}:{normalised}".encode("ascii"))
    return digest.hexdigest()[:32]


def _run(command: list) -> str:
    """跑一个系统命令，失败返回空串（枚举硬件不该让程序崩）。"""
    try:
        completed = subprocess.run(command, capture_output=True, text=True,
                                   timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        return ""
    return completed.stdout if completed.returncode == 0 else ""


def _windows_physical_macs() -> list:
    """Windows：``Get-NetAdapter -Physical`` 的 ``PermanentAddress``。

    用 PowerShell 而不是注册表：注册表里的 ``NetworkAddress`` 是**可写覆盖值**，
    伪造 MAC 就是改它，不能作为依据。``PermanentAddress`` 来自固件/驱动上报，
    Win10 起可用（本工具的目标平台就是 Win10+）。
    """
    script = (
        "Get-NetAdapter -Physical | "
        "Where-Object { $_.Status -ne 'Not Present' } | "
        "ForEach-Object { $_.PermanentAddress }"
    )
    output = _run(["powershell", "-NoProfile", "-NonInteractive",
                   "-Command", script])
    if not output:                        # 老系统兜底（Win10 早期版本）
        output = _run(["wmic", "nic", "where", "PhysicalAdapter=true",
                       "get", "MACAddress", "/value"])
        return [line.split("=", 1)[1].strip() for line in output.splitlines()
                if line.strip().lower().startswith("macaddress=")]
    return [line.strip() for line in output.splitlines() if line.strip()]


def _linux_physical_macs() -> list:
    """Linux：``/sys/class/net/<if>/device`` 存在且名字不像虚拟的才算物理。"""
    from pathlib import Path

    net = Path("/sys/class/net")
    if not net.is_dir():
        return []
    macs = []
    for entry in sorted(net.iterdir()):
        if VIRTUAL_NAME_PATTERNS.match(entry.name):
            continue
        if not (entry / "device").exists():      # 没有 device 链接 = 虚拟接口
            continue
        try:
            address = (entry / "address").read_text(encoding="ascii").strip()
        except OSError:
            continue
        macs.append(address)
    return macs


def physical_macs() -> list:
    """本机所有**物理**网卡的 MAC（规范化、去重、已过滤虚拟/随机/伪造特征）。

    Returns:
        list[str]: 形如 ``["f0:4e:a4:90:4b:2c", ...]``；枚举失败时返回空列表
        （调用方据此给出"读不到网卡"的明确提示，而不是当成"没有授权"）。
    """
    system = platform.system()
    if system == "Windows":
        raw = _windows_physical_macs()
    elif system == "Linux":
        raw = _linux_physical_macs()
    else:                                # macOS 等：/sys 不存在，暂不支持
        raw = []
    result = []
    for mac in raw:
        if not is_usable_mac(mac):
            continue
        normalised = _normalise(mac)
        if normalised not in result:
            result.append(normalised)
    return result


def machine_fingerprints() -> list:
    """本机物理网卡对应的指纹列表（授权里存的就是这些值）。"""
    return sorted(fingerprint(mac) for mac in physical_macs())


def describe_machine() -> dict:
    """给人看/给签发方用的一份硬件摘要。

    Returns:
        dict: ``{"system": "Windows", "macs": [...], "fingerprints": [...]}``。
    """
    macs = physical_macs()
    return {"system": platform.system(), "machine": platform.machine(),
            "macs": macs, "fingerprints": sorted(fingerprint(m) for m in macs)}


if __name__ == "__main__":             # 便于现场快速取证
    info = describe_machine()
    if not info["macs"]:
        print("读不到物理网卡（虚拟机上运行？），请把这条信息发给供应商",
              file=sys.stderr)
    for mac, fp in zip(info["macs"], info["fingerprints"]):
        print(f"{mac}  {fp}")
