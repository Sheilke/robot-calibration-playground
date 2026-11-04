# 手眼验证（VPE）代码实现计划

## Context

《手眼 & 机器人精度 关联分析方案.md》及其补充《手眼验证 Pro.md》（InfoKownledgeBay）提出用激光跟踪仪独立验证手眼标定质量：世界固定放置标准靶球，机器人多位姿用相机**直接辨识靶球**，视觉链推算的球心位置与跟踪仪实测比较得 VPE（Visual Positioning Error）；并用「离线重算实验」（旧/新 DH 各解一次手眼、独立验证子集上比 VPE）评估 DH 标定对手眼精度的贡献。本计划把该验证方法落成本仓库（robot-calibration-playground，纯 Python、无 ROS2）中可执行的代码。

**已与用户确认的决策**：
- 关节读数：用户提供，**模型轴角度**（无需 Direction 映射）；
- ArUco/靶球数据：公司 3srobotics 管线产物，**.pb 文件**（ArucoRecordingResult）；
- 手眼求解：**复用公司 C++ 实现，不重写算法**（`calibration/eye_on_hand_calibration` 动作），离线重算允许依赖 ROS2 容器；
- 标定板布局不可提供 → **按直接辨识标准球方案**：靶球中心为相机系直接测得的 3D 点，无板坐标系、无板位姿拟合；
- 手眼矩阵 X：用户另行标定后提供（作为输入）；
- 模型范围：只做昌宏 CH2000（组4/5）；不做 3 靶球 6-DOF；报告 markdown。

**为何不重写求解器**：C++ 的 eye_on_hand 求解器输入是笛卡尔位姿（`robot_pt`），不知道关节角也不知道 DH。
离线重算真正缺的能力是「用新 DH 从关节角重算位姿」，而不是求解器本身。
因此本包只补：`FK(q, θ)` → 改写 `.pb` 的 robot_pt → 交给现成动作求解 → VPE 评价。

核心公式（eye-on-hand，静态靶球）：

```
^B P_ball^vision(i) = FK(q_i, θ) · ^H T_C · [^C P_ball(i); 1]
e_i = ^B P_ball^vision(i) − (^B P_ball^laser + ^B ΔP)
VPE_i = ‖e_i‖,  VPE_RMSE = sqrt(mean(VPE_i²))
指标：VPE_RMSE / VPE_MAX / VPE_P95 / e_x,e_y,e_z / 平均偏置向量（方向性诊断）
```

离线重算（方案第 5 节）：

```
X_old = C++HandEye(recorded_robot_pt 或 FK(q,θ_old), p_cam)
X_new = C++HandEye(rewrite_pb(FK(q,θ_new)), p_cam)
同一批采样、独立验证子集上比较 VPE(X_new,θ_new) vs VPE(X_old,θ_old)
交叉组合（旧DH×新X 等）仅作诊断输出
```

求解器调用方式（已勘察）：
- 动作 `calibration/eye_on_hand_calibration`（`arc_msgs/action/ArcAction`，goal.data = 序列化的
  `EyeOnHandCalibrationGoal{input_directory}`）——只读目录内 `.pb`，不需要机器人/相机；
- 结果 `EyeOnHandCalibrationResult{success, matrix[16 row-major], compute_error}`，
  序列化在 `ArcAction::Result.data`；同时落盘备份到 `ARC_S_DATA_DIR/assets/camera/`（宿主 `~/arc-s-data/assets/camera`）；
- **矩阵平移单位为 mm**（与 common_config 存储的米制不同，`load_matrix16` 显式换算）。

## 公司仓库勘察结论（决定实现形态）

1. **数据形态**（`arc-s/src/calibration/src/calibration_action.cpp` ~706-845 行）：管线输出 `ArucoRecordingResult`（.pb）：`SingleCalibStatics{robot_pt(笛卡尔位姿), BoardStatics{board_pts:[{id,x,y,z}]}}`——每帧若干相机系 3D 点（marker 中心/球心），无角点、无 solvePnP。
2. **手眼求解 = asmaa 算法**（同文件 ~77-260、1228-1500 行）：按点 id 分组，每组一个 base 系固定点 P_fix；`CalculateRt1` 给定 P_fix 线性 LS 解 `A·X=B`（A=相机点齐次行，B=`T_i⁻¹·P_fix`）；`TasksCpp` 对 P_fix 随机搜索（20³ 网格抽 800，d=1000→×0.7×25）+ 网格搜索（10³，×0.5×4），外层 5 轮每轮 `TrimData` 剔除最差样本；多组按组间平均误差选优并精化。X 语义：`T_BH·X·p_cam` 相机系→base，即 X = ^H T_C。
3. **手眼矩阵存储约定**（arc-common `hand_eye_utils.hpp`）：16 值 row-major、**米制**、正交性校验 tol 1e-4。本仓库 FK 为 mm → 单位换算显式处理。
4. arc-s `handeye_export` 的 .pos 为笛卡尔位姿（无关节角），不作离线重算输入（离线重算必须用关节读数）。

## Approach

新增 `python/handeye/` 包（核心纯 numpy；仅 pb 转换需要 protobuf）：

- `se3.py`：SE(3) 运算（逆/乘/点变换、rpy↔矩阵、16 值 row-major 装载+正交性校验+单位换算）。复用 `robot_model.nexbot_ik.euler_to_rot/rot_to_euler`、`excitation_trajectory.aabb_snake_scan.rotation_vector`。
- `dh.py`：DH 覆盖加载——URDF 基准（`Nexbot6R`，`data/configs/ch2000_origin.urdf`）+ `dh_override.json`（std DH a/alpha/d/theta_offset，mm/deg），产出 θ_old/θ_new 两个 FK；报告覆盖前后 FK 差异。
- `asmaa.py`：公司算法 numpy 移植——`calculate_rt1`/`cam2_fixpoint`/随机+网格搜索/`tasks`（含 TrimData）；机器人位姿输入泛化为 4x4 T_BH 矩阵（公司版由 euler 构造；我们支持 FK(q,θ) 直供——离线重算的关键差异）；固定随机种子可复现。
- `pb_to_json.py`：CLI 转换器 `python -m handeye.pb_to_json <dir|file.pb> -o session_part.json`——解析 ArucoRecordingResult（id/x/y/z + robot_pt），合并用户提供的 q_deg（按序/按 pose_id 对齐）；protobuf 为可选依赖，缺失时给出安装指引；目标点单位（m→mm）显式换算。
- `session.py`：会话 JSON 加载与校验。Schema：
  ```json
  {
    "meta": {"robot_urdf": "...", "dh_override": null|"path", "units": "mm",
             "group": 4, "joint_convention": "model"},
    "reference": {"p_ball_laser_base": [x,y,z], "delta_p_base": null|[x,y,z]},
    "handeye_matrix": null | {"matrix16_rowmajor": [...], "units": "m"},
    "samples": [{"pose_id": 1, "q_deg": [6 值],
                 "frames": [{"target_points": {"<id>": [x,y,z]}}]}]
  }
  ```
  校验：q 与限位、点单位、非有限值、pose_id 唯一；附带 robot_pt（若来自 pb）与自算 FK(q,θ_old) 的一致性报告（控制器 FK vs 本仓库 FK，暴露约定差异）。
- `vpe.py`：视觉链推算 ^B P_ball^vision(i)（多位姿多帧：先按位姿内多帧聚合均值，再统计）+ VPE 指标（RMSE/MAX/P95/逐轴/偏置向量）。
- `report.py`：markdown 报告（分组对比表、逐位姿明细、偏置方向诊断、FK 一致性附录）。
- `verify.py`：CLI `python -m handeye.verify SESSION.json -o report.md`（X 用 session 内用户提供的已标矩阵）。
- `offline_recompute.py`：CLI `python -m handeye.offline_recompute SESSION.json --dh-old O.json --dh-new N.json [--calib-ids ... --valid-ids ... | --split 0.7]`（子集划分 → X_old/X_new → 交叉组合 → VPE 对比表）。
- `tests/`：合成数据端到端——已知 X 与 DH 造静态球观测点（可加噪声）→ asmaa 恢复 X（平移/旋转误差断言）；VPE 指标手算对照；DH 覆盖生效（FK 差异=注入扰动）；多帧聚合；markdown 报告结构。

## Files to add / modify

- 新增：`python/handeye/{__init__,se3,dh,proto_codec,pose_encoder,pb_rewrite,session,vpe,report,verify,offline_recompute}.py`
- 新增：`python/handeye/tests/test_*.py`、`data/configs/dh_override_example.json`、`data/configs/session_example.json`（含合成示例数据）
- 修改：`README.md`（模块说明 + 用法 + 数据准备流程：pb→json→verify/recompute）
- 修改：`requirements.txt`（追加可选说明或 extras：`protobuf` 仅 pb 转换需要）

## Reuse

- `python/robot_model/nexbot_ik.py`：`Nexbot6R`（URDF 加载 + FK + 限位）、`euler_to_rot`、`rot_to_euler`
- `python/excitation_trajectory/aabb_snake_scan.py`：`rotation_vector`
- 公司参考实现（只读对照，不引入依赖）：`arc-s/src/calibration/src/calibration_action.cpp`（asmaa 全算法、pb 数据形态）、`arc-common/src/proto_msgs/proto/calibration/calibration.proto`（schema）、`arc-common/.../hand_eye_utils.hpp`（矩阵校验与米制约定）

## Steps

- [x] 1. `se3.py`（含 X 装载/校验/单位换算）+ 单测
- [x] 2. `dh.py`（URDF+override → 新旧 FK）+ 单测（注入扰动恢复）
- [ ] 3. `proto_codec.py` + `pose_encoder.py` + `pb_rewrite.py`（用 FK 重算 robot_pt 并改写 .pb）+ 单测
- [ ] 4. `session.py`（schema 校验、FK 一致性报告）+ 示例 JSON + 单测
- [ ] 5. `vpe.py`（链式推算 + 多帧聚合 + 指标）+ 手算对照单测
- [ ] 6. `report.py` markdown 输出
- [ ] 7. `verify.py` CLI + 端到端单测（合成 session → 报告）
- [ ] 8. `offline_recompute.py` CLI（子集划分、改写 .pb、调 C++ 动作得 X_old/X_new、交叉组合）+ 单测
- [ ] 9. README 更新（用法 + pb→session 数据准备说明 + 动作调用方式）

## Verification

- `PYTHONPATH=python python3 -m unittest discover -s python/handeye/tests` 全绿；既有 25 项测试不回归
- 合成端到端：真值 X 已知 → asmaa 恢复误差断言；θ_new 注入已知扰动 → VPE 恶化量级与扰动量级一致性；交叉组合表结构断言
- `python -m handeye.verify data/configs/session_example.json -o /tmp/report.md` 产出 markdown 报告
