# -*- coding: utf-8 -*-
"""手眼验证（VPE）工具包。

设计原则：**手眼求解器只保留公司 C++ 一份**（`calibration/eye_on_hand_calibration`
动作），本包只做 C++ 做不到的数据变换与评价，避免算法漂移与权威性分裂。

模块组成（见 plans/handeye-vpe-verification.md）：

* ``se3``               SE(3) 基础运算、手眼矩阵 16 值装载/校验/单位换算
* ``dh``                URDF 基准 + dh_override.json -> 旧/新 DH 的 FK
* ``proto_codec``       公司 calibration/common proto 的动态编译与 .pb 读写
* ``pose_encoder``      T_bh -> 控制器 robot_pt（mm + XYZ 内旋欧拉）
* ``pb_rewrite``        用 FK 重算位姿并改写 .pb（喂给公司 C++ 求解器）
* ``session``           会话 JSON 的加载与校验（含 FK 一致性报告）
* ``vpe``               视觉链推算与 VPE 指标
* ``report``            markdown 报告生成
* ``verify``            VPE 验证 CLI
* ``offline_recompute`` 旧/新 DH 离线重算 CLI（调公司 eye_on_hand_calibration 动作）
"""
