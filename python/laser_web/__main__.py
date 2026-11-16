# -*- coding: utf-8 -*-
"""``laser_web`` 的命令行入口：启动本地 Web 界面。

用法（在仓库根）::

    PYTHONPATH=python .venv/bin/python -m laser_web --port 8765
    # 浏览器打开 http://127.0.0.1:8765

只监听本机：``--host`` 默认且只接受 ``127.0.0.1``（本工具没有鉴权，不提供
局域网监听选项；将来要开再加，并附“无鉴权”警告）。
"""

from __future__ import annotations

import argparse

#: 唯一允许的监听地址（本机回环）。
HOST = "127.0.0.1"
#: 默认端口。
DEFAULT_PORT = 8765


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="python -m laser_web",
        description="标定路点工具的本地网页界面（M1：骨架 + 采样域预览）。"
                    "服务只监听 127.0.0.1，不提供局域网监听。")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, metavar="PORT",
                        help=f"监听端口，默认 {DEFAULT_PORT}")
    parser.add_argument("--host", default=HOST, metavar="ADDR",
                        help=f"监听地址，只允许 {HOST}（默认）；"
                             "本工具无鉴权，不支持局域网监听")
    return parser


def main(argv: list[str] | None = None) -> int:
    """解析参数并启动界面。

    Args:
        argv: 命令行参数（``None`` 表示 ``sys.argv[1:]``，测试可直接传列表）。

    Returns:
        进程退出码：正常启动后不会返回（服务常驻）。

    Raises:
        SystemExit: 参数非法时由 argparse 抛出（``--host`` 只接受 127.0.0.1）。
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.host != HOST:
        parser.error(f"--host 只允许 {HOST}（本工具仅本机访问，无鉴权），"
                     f"当前为 {args.host!r}")
    if not 1 <= args.port <= 65535:
        parser.error(f"--port 需在 1..65535 之间，当前为 {args.port}")

    # 延迟 import：--help / 参数报错时不必拉起 UI 依赖。
    from laser_web.app import run
    run(host=HOST, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
