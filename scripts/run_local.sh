#!/usr/bin/env bash
# 本地（Linux）启动桌面版，仅供开发与内部查看。
#
# 为什么需要这个脚本
# ------------------
# 交付版启动时会校验离线授权（绑定机器物理网卡），**源码运行也一样校验**：
# 这是故意的 —— 如果源码模式默认放行，就永远测不到"没证的人打开会看到什么"，
# 而那句话恰恰是最需要打磨的。
#
# 代价是本地每次都要带上跳过变量，容易忘。所以把它收在这里，并顺手做两件
# 开发时真的会踩的事：
#
#   * 用 venv 里的解释器（不是系统 python，那边没有 PyQt5/PySide6）；
#   * 把 PYTHONPATH 指到 python/（laser_desktop 是包，不在仓库根）。
#
# 注意：这里能跳过，是因为**源码未冻结**（sys.frozen 为假）。打出来的 exe 不认
# 这个变量 —— 别拿它当后门，它只在开发机上有效。
set -euo pipefail

cd "$(dirname "$0")/.."

export PYTHONPATH=python
export LASER_SKIP_LICENSE=1

# 开发时的临时数据目录：不往仓库 runs/ 里堆测试产物（可用 LASER_HOME 覆盖）
export LASER_HOME="${LASER_HOME:-/tmp/laser-desktop-dev}"

exec .venv/bin/python -m laser_desktop "$@"
