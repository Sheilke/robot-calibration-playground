# -*- coding: utf-8 -*-
"""离线授权：Ed25519 签名 + 物理网卡 MAC 绑定。

模块组成：

* ``hwid``    物理网卡 MAC 枚举（滤掉虚拟/随机/伪造）与不可逆指纹；
* ``license`` 授权文件的组装、签名、验签、有效期与机器绑定检查；
* ``__main__`` 命令行：``hwid`` / ``issue`` / ``verify``（签发与现场排查用）。

用法见 ``docs/开发参考.md`` 的「授权签发与校验」一节。
"""
from .hwid import describe_machine, machine_fingerprints, physical_macs
from .license import (LicenseError, build_payload, generate_keypair,
                      load_license, load_private_key, load_public_key,
                      sign_payload, verify_license)

__all__ = ["describe_machine", "machine_fingerprints", "physical_macs",
           "LicenseError", "build_payload", "generate_keypair",
           "load_license", "load_private_key", "load_public_key",
           "sign_payload", "verify_license"]
