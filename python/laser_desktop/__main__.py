#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桌面版入口。

    PYTHONPATH=python python3 -m laser_desktop            # 正常启动
    PYTHONPATH=python python3 -m laser_desktop --print-paths   # 只打印路径解析结果

注意：``qt_compat`` 必须**在 QApplication 之前**导入（Qt5 的 WebEngine 硬性要求），
所以这里的 import 顺序是刻意的。
"""
from __future__ import annotations

import sys

from . import paths
from .qt_compat import (BINDING, DEV_ONLY_BINDING, QtWidgets,
                        apply_chinese_font, configure_webengine_environment)


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)

    if "--print-paths" in args:
        for name, value in paths.describe().items():
            print(f"{name:8s} {value}")
        return 0

    configure_webengine_environment()      # 必须在建 QApplication 之前
    paths.ensure_writable_roots()

    app = QtWidgets.QApplication(args)
    family = apply_chinese_font(app)

    from .main_window import MainWindow
    window = MainWindow()
    window.show()
    print(f"桌面版启动：Qt 绑定 {BINDING}"
          + ("（仅供本地验证）" if DEV_ONLY_BINDING else "")
          + f"，中文字体 {family}")
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
