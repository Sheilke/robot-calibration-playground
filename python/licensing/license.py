# -*- coding: utf-8 -*-
"""离线授权文件：Ed25519 签名 + 物理网卡 MAC 绑定。

为什么是"签名文件"而不是"联网校验"
----------------------------------
现场 PC 常年不联外网，任何"启动时联网验证"都不可行。所以做成**离线签名**：

* 签发方（我们）持**私钥**，把"客户名 + 序列号 + 绑定的硬件指纹 + 有效期"
  签成一个 ``.lic`` 文件；
* 程序里只内嵌**公钥**，启动时验证签名 + 比对本机指纹；
* 私钥泄露才可能伪造授权，因此私钥只留在签发人的离线机器上（不随程序分发）。

文件格式
--------
``.lic`` 就是 JSON（便于人眼查、也便于运维手改客户名后重新签发）::

    {
      "payload": {
        "v": 1,
        "licensee": "XX 公司",
        "serial": "R1-0001",
        "issued": "2026-11-18",
        "expires": "2027-11-18",          // null = 永久
        "fingerprints": ["3f2a...", ...], // 绑定的物理网卡指纹（不是原始 MAC）
        "min_match": 1,                   // 命中几个即视为同一台机器
        "features": ["iso9283"]           // 预留：按功能授权
      },
      "sig": "base64(Ed25519(payload 的规范 JSON))"
    }

签名覆盖的就是 ``payload`` 的**规范 JSON**（``sort_keys`` + 紧凑分隔符），任何
字段改动都会让签名失效——包括改 ``expires`` 或往 ``fingerprints`` 里加机器。
"""
from __future__ import annotations

import base64
import datetime as _dt
import json
from pathlib import Path

from .hwid import machine_fingerprints

#: 授权文件格式版本。升级要写迁移逻辑，别静默改。
LICENSE_VERSION = 1

#: 允许的功能名（预留给按功能授权；当前只用 ``iso9283`` 这类短名）。
KNOWN_FEATURES = ("iso9283", "handeye", "export_gp")


class LicenseError(Exception):
    """授权不可用。``str(error)`` 就是给人看的原因（中文、可直接显示在界面上）。"""


def canonical_json(payload: dict) -> bytes:
    """签名/验签用的规范 JSON：键排序、紧凑分隔符、UTF-8。"""
    return json.dumps(payload, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False).encode("utf-8")


def _require_crypto():
    """延迟 import，让只用核心算法的人不必装 ``cryptography``。"""
    try:
        from cryptography.exceptions import InvalidSignature
        from cryptography.hazmat.primitives.asymmetric.ed25519 import (
            Ed25519PrivateKey, Ed25519PublicKey)
        from cryptography.hazmat.primitives import serialization
    except ImportError as error:      # pragma: no cover - 依赖缺失属于安装问题
        raise LicenseError(
            "缺少授权校验依赖 cryptography：pip install -r requirements-license.txt"
        ) from error
    return (InvalidSignature, Ed25519PrivateKey, Ed25519PublicKey,
            serialization)


def load_private_key(path) -> object:
    """读签发私钥（PEM）。**签发方专用**，不要随程序分发。"""
    _, Ed25519PrivateKey, _, serialization = _require_crypto()
    data = Path(path).read_bytes()
    return serialization.load_pem_private_key(data, password=None)


def load_public_key(path) -> object:
    """读校验公钥（PEM）。这个是随程序发布的那个。"""
    _, _, Ed25519PublicKey, serialization = _require_crypto()
    data = Path(path).read_bytes()
    return serialization.load_pem_public_key(data)


def generate_keypair(private_path, public_path) -> None:
    """生成一对 Ed25519 密钥（签发方一次性操作）。

    Args:
        private_path: 私钥输出路径（PEM，权限 0600，**必须离线保存**）。
        public_path: 公钥输出路径（PEM，随程序分发/内嵌）。
    """
    _, Ed25519PrivateKey, _, serialization = _require_crypto()
    private_key = Ed25519PrivateKey.generate()
    Path(private_path).write_bytes(private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption()))
    Path(private_path).chmod(0o600)
    Path(public_path).write_bytes(private_key.public_key().public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo))


def build_payload(licensee: str, serial: str, fingerprints, *,
                  expires=None, min_match: int = 1,
                  features=(), issued=None) -> dict:
    """组装授权内容（不含签名）。

    Args:
        licensee: 客户名（追溯用）。
        serial: 授权序列号，唯一。
        fingerprints: 绑定的网卡指纹（``hwid.machine_fingerprints()`` 的输出）。
        expires: ``"YYYY-MM-DD"`` 或 ``None``（永久）。
        min_match: 命中几个指纹算同一台机器；多网卡机器建议 1。
        features: 功能名列表（见 :data:`KNOWN_FEATURES`）。
        issued: 签发日期，默认今天。

    Raises:
        ValueError: 指纹为空、``min_match`` 不合理、日期格式错。
    """
    fingerprints = [str(value).strip().lower() for value in fingerprints if value]
    if not fingerprints:
        raise ValueError("至少要绑定一个网卡指纹")
    if not 1 <= int(min_match) <= len(fingerprints):
        raise ValueError(f"min_match 需在 1..{len(fingerprints)}，当前 {min_match}")
    unknown = [name for name in features if name not in KNOWN_FEATURES]
    if unknown:
        raise ValueError(f"未知功能 {unknown}（可用 {list(KNOWN_FEATURES)}）")
    for label, value in (("issued", issued), ("expires", expires)):
        if value is None:
            continue
        try:
            _dt.date.fromisoformat(str(value))
        except ValueError as error:
            raise ValueError(f"{label} 需为 YYYY-MM-DD，当前 {value!r}") from error
    return {"v": LICENSE_VERSION,
            "licensee": str(licensee).strip(),
            "serial": str(serial).strip(),
            "issued": str(issued or _dt.date.today().isoformat()),
            "expires": None if expires is None else str(expires),
            "fingerprints": fingerprints,
            "min_match": int(min_match),
            "features": list(features)}


def sign_payload(payload: dict, private_key) -> bytes:
    """给 payload 签名，返回 ``.lic`` 文件的字节内容。"""
    _, _, _, _ = _require_crypto()
    signature = private_key.sign(canonical_json(payload))
    document = {"payload": payload,
                "sig": base64.b64encode(signature).decode("ascii")}
    return json.dumps(document, ensure_ascii=False, indent=2).encode("utf-8")


def verify_signature(payload: dict, signature_b64: str, public_key) -> None:
    """验签；不通过抛 :class:`LicenseError`。

    Raises:
        LicenseError: 签名不匹配 / 公钥不对 / 字段被改过。
    """
    InvalidSignature, _, _, _ = _require_crypto()
    try:
        signature = base64.b64decode(signature_b64, validate=True)
    except Exception as error:                # noqa: BLE001 - 内容坏了都算验签失败
        raise LicenseError(f"授权文件签名段不是合法 base64：{error}") from error
    try:
        public_key.verify(signature, canonical_json(payload))
    except InvalidSignature as error:
        raise LicenseError(
            "授权文件签名校验失败：要么不是本程序签发的，要么内容被改过"
            "（客户名、有效期、绑定的机器任何一个字段改动都会这样）") from error


def _check_expiry(payload: dict,*, today=None) -> None:
    """有效期检查（``expires`` 为 ``None`` 表示永久）。"""
    expires = payload.get("expires")
    if not expires:
        return
    today = today or _dt.date.today()
    try:
        deadline = _dt.date.fromisoformat(str(expires))
    except ValueError as error:
        raise LicenseError(f"授权文件里的有效期格式不对：{expires!r}") from error
    if today > deadline:
        raise LicenseError(f"授权已过期（有效期至 {deadline.isoformat()}），"
                           f"请联系供应商续期")


def check_machine(payload: dict, local_fingerprints=None) -> dict:
    """验证本机是否就是授权里绑定的机器。

    Args:
        payload: 已验签的 payload。
        local_fingerprints: 本机指纹（默认现算；测试可注入）。

    Returns:
        dict: ``{"matched": [...], "required": int, "local": int}``。

    Raises:
        LicenseError: 命中数不足（提示换机器要重新签发）。
    """
    local = (machine_fingerprints() if local_fingerprints is None
             else list(local_fingerprints))
    if not local:
        raise LicenseError(
            "读不到本机物理网卡（常见原因：在虚拟机里运行、网卡被禁用）。"
            "授权按网卡绑定，读不到就无法验证")
    allowed = {str(value).strip().lower() for value in payload.get("fingerprints") or []}
    required = max(1, int(payload.get("min_match") or 1))
    matched = sorted(set(local) & allowed)
    if len(matched) < required:
        raise LicenseError(
            f"本机不是授权绑定的机器（命中 {len(matched)}/{required} 个网卡指纹）。"
            f"换机器/换网卡需要重新签发授权，请把本机指纹发给供应商")
    return {"matched": matched, "required": required, "local": len(local)}


def load_license(path) -> dict:
    """读 ``.lic`` 并返回已验签的 payload（**不**做机器与有效期检查）。

    Raises:
        LicenseError: 文件读不了 / JSON 坏了 / 缺字段。
    """
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise LicenseError(f"找不到授权文件：{path}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise LicenseError(f"授权文件读不了或不是合法 JSON：{error}") from error
    payload = document.get("payload")
    signature = document.get("sig")
    if not isinstance(payload, dict) or not isinstance(signature, str):
        raise LicenseError("授权文件缺 payload / sig 字段")
    if int(payload.get("v") or 0) != LICENSE_VERSION:
        raise LicenseError(f"授权文件版本不受支持（{payload.get('v')!r}，"
                           f"本程序支持 {LICENSE_VERSION}）")
    return payload


def verify_license(path, public_key, *, local_fingerprints=None,
                   today=None) -> dict:
    """完整校验：读文件 → 验签 → 查有效期 → 查机器 → 返回摘要。

    Args:
        path: ``.lic`` 路径。
        public_key: 已加载的 Ed25519 公钥对象。
        local_fingerprints: 本机指纹（测试注入）。
        today: 今天的日期（测试注入）。

    Returns:
        dict: ``{"licensee", "serial", "expires", "features", "matched"}``。

    Raises:
        LicenseError: 任何一项不通过（消息可直接显示给用户）。
    """
    payload = load_license(path)
    verify_signature(payload, json.loads(
        Path(path).read_text(encoding="utf-8"))["sig"], public_key)
    _check_expiry(payload, today=today)
    machine = check_machine(payload, local_fingerprints)
    return {"licensee": payload["licensee"], "serial": payload["serial"],
            "expires": payload.get("expires"),
            "features": list(payload.get("features") or []),
            "matched": machine["matched"]}
