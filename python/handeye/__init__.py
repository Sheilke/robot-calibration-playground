# -*- coding: utf-8 -*-
"""运动学工具子集：手眼验证包中与路点生成共用的部分。

本分支只保留路点生成所需的模块：

* ``se3``           SE(3) 基础运算、变换矩阵装载/校验/单位换算
* ``dh``            URDF 基准 + dh_override -> 改参后的 FK
* ``pose_encoder``  T_bh -> 控制器 robot_pt（mm + XYZ 内旋欧拉）

``export_gp --cartesian fk`` 用 ``dh`` + ``pose_encoder`` 由关节角算笛卡尔位姿；
``se3`` 是两者的公共底座。完整的手眼验证功能（session / vpe / report /
verify / offline_recompute / pb_rewrite / proto_codec）保留在 ``dev_0.1`` 分支。
"""
