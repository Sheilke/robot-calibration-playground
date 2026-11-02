"""机器人运动学/动力学模型公共接口。

导入本包不会执行任何命令行入口；``Nexbot6R`` 与 ``wrap_180`` 在首次访问时
按需从 :mod:`robot_model.nexbot_ik` 加载。
"""

from importlib import import_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .nexbot_ik import Nexbot6R, wrap_180

__all__ = ("Nexbot6R", "wrap_180")


def __getattr__(name: str):
    """按需导出 Nexbot 6R 运动学公共符号。"""
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(".nexbot_ik", __name__), name)
    globals()[name] = value
    return value
