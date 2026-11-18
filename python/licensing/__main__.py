#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""授权工具命令行。

三个子命令：

``hwid``
    在**客户机器**上跑，打印本机物理网卡与指纹；把指纹发回供应商用于签发。
``issue``
    在**签发方**机器上跑（需要私钥），生成 ``.lic``。
``verify``
    在任何机器上跑，检查一份 ``.lic`` 是不是本机可用（现场排查用）。

示例::

    # 客户机：取指纹
    PYTHONPATH=python python3 -m licensing hwid

    # 签发方：首次生成密钥（私钥离线保管！）
    PYTHONPATH=python python3 -m licensing issue --keygen private.pem public.pem

    # 签发方：按指纹签发（有效期可省 = 永久）
    PYTHONPATH=python python3 -m licensing issue --key private.pem \\
        --licensee "XX 公司" --serial R1-0001 --expires 2027-11-18 \\
        --fingerprint 3f2a... --fingerprint 91c0... --out R1-0001.lic

    # 现场：验证
    PYTHONPATH=python python3 -m licensing verify R1-0001.lic \\
        --public-key public.pem
"""
from __future__ import annotations

import argparse
import sys

from .hwid import describe_machine
from .license import (LicenseError, build_payload, generate_keypair,
                      load_private_key, load_public_key, sign_payload,
                      verify_license)


def _cmd_hwid(args) -> int:
    """打印本机物理网卡（原始 MAC + 指纹）。"""
    info = describe_machine()
    print(f"系统：{info['system']} / {info['machine']}")
    if not info["macs"]:
        print("读不到物理网卡：常见于虚拟机、或所有网卡都被禁用。", file=sys.stderr)
        return 2
    print(f"物理网卡 {len(info['macs'])} 块：")
    for mac, print_fingerprint in zip(info["macs"], info["fingerprints"]):
        print(f"  {mac}   {print_fingerprint}")
    print("\n把上面的**指纹**（第二列）发给供应商即可签发授权；"
          "原始 MAC 不需要外发。")
    return 0


def _cmd_keygen(args) -> int:
    """生成 Ed25519 密钥对。"""
    generate_keypair(args.keygen[0], args.keygen[1])
    print(f"已生成私钥 {args.keygen[0]}（权限 600，请离线保管，不要随程序分发）")
    print(f"已生成公钥 {args.keygen[1]}（随程序发布/内嵌）")
    return 0


def _cmd_issue(args) -> int:
    """按指纹签发授权文件。"""
    try:
        payload = build_payload(args.licensee, args.serial, args.fingerprint,
                                expires=args.expires,
                                min_match=args.min_match,
                                features=args.feature or (),
                                issued=args.issued)
        document = sign_payload(payload, load_private_key(args.key))
    except ValueError as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2
    out = args.out or f"{args.serial}.lic"
    with open(out, "wb") as handle:
        handle.write(document)
    print(f"已签发 {out}")
    print(f"  客户：{payload['licensee']}   序列号：{payload['serial']}")
    print(f"  有效期：{payload['expires'] or '永久'}   "
          f"绑定指纹 {len(payload['fingerprints'])} 个（命中 "
          f"{payload['min_match']} 个即视为同一台机器）")
    return 0


def _cmd_verify(args) -> int:
    """校验授权文件在本机是否可用。"""
    try:
        summary = verify_license(args.license, load_public_key(args.public_key),
                                 local_fingerprints=(args.local_fingerprint or None))
    except LicenseError as error:
        print(f"不可用：{error}", file=sys.stderr)
        return 2
    print("授权有效")
    print(f"  客户：{summary['licensee']}   序列号：{summary['serial']}")
    print(f"  有效期：{summary['expires'] or '永久'}")
    if summary["features"]:
        print(f"  功能：{'、'.join(summary['features'])}")
    print(f"  命中指纹 {len(summary['matched'])} 个")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m licensing",
        description="离线授权工具：物理网卡绑定 + Ed25519 签名。")
    sub = parser.add_subparsers(dest="command", required=True)

    hwid = sub.add_parser("hwid", help="打印本机物理网卡 MAC 与指纹")
    hwid.set_defaults(handler=_cmd_hwid)

    keygen = sub.add_parser("issue", help="签发授权（需要私钥）")
    keygen.add_argument("--key", help="签发私钥（PEM）")
    keygen.add_argument("--keygen", nargs=2, metavar=("PRIVATE", "PUBLIC"),
                        help="改为生成密钥对：私钥 公钥")
    keygen.add_argument("--licensee", help="客户名")
    keygen.add_argument("--serial", help="授权序列号（唯一）")
    keygen.add_argument("--fingerprint", action="append", default=[],
                        metavar="FP", help="绑定的网卡指纹，可重复给多个")
    keygen.add_argument("--min-match", type=int, default=1, metavar="N",
                        help="命中几个指纹算同一台机器，默认 1")
    keygen.add_argument("--expires", default=None, metavar="YYYY-MM-DD",
                        help="有效期；不给表示永久")
    keygen.add_argument("--issued", default=None, metavar="YYYY-MM-DD",
                        help="签发日期，默认今天")
    keygen.add_argument("--feature", action="append", default=[],
                        metavar="NAME", help="功能授权（预留），可重复")
    keygen.add_argument("--out", default=None, help="输出 .lic 路径")
    keygen.set_defaults(handler=_dispatch_issue)

    verify = sub.add_parser("verify", help="校验授权文件")
    verify.add_argument("license", help=".lic 文件")
    verify.add_argument("--public-key", required=True, help="公钥（PEM）")
    verify.add_argument("--local-fingerprint", action="append", default=[],
                        metavar="FP", help="手工给出本机指纹（默认现算）")
    verify.set_defaults(handler=_cmd_verify)
    return parser


def _dispatch_issue(args) -> int:
    """``issue`` 既要支持 --keygen 也要支持签发，这里分流。"""
    if args.keygen:
        return _cmd_keygen(args)
    missing = [name for name in ("key", "licensee", "serial") if not getattr(args, name)]
    if missing:
        print(f"错误：签发需要 {' / '.join('--' + name.replace('_', '-') for name in missing)}"
              "（或用 --keygen 只生成密钥）", file=sys.stderr)
        return 2
    if not args.fingerprint:
        print("错误：至少要给一个 --fingerprint（在客户机上跑 `hwid` 取）",
              file=sys.stderr)
        return 2
    return _cmd_issue(args)


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
