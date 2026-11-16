# -*- coding: utf-8 -*-
"""``laser_web``：标定路点工具的本地 Web 界面（M1：骨架 + 采样域预览）。

分层（与设计文档 §4 一致）：

* ``core``     服务层，只依赖算法模块，**不 import 任何 UI 库**，可脱离界面单测；
* ``pages``    只做“收集参数 -> 调 core -> 渲染”，不出现任何算法公式；
* ``ui_state`` 表单默认值工厂，每次调用返回新 dict，不保存模块级可变状态；
* ``app``      左侧导航 + 三个页面容器 + ``ui.run``。

启动（仅本机）::

    PYTHONPATH=python .venv/bin/python -m laser_web --port 8765
"""
