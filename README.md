# robot-calibration-playground

个人机器人标定方向的开发仓库，用于算法积累与实验记录。当前以 Python 为主，后续计划与公司 ROS2 项目对接。

本仓库为个人开发空间，不代表公司项目；涉及公司内部参数与配置的内容不会提交到此仓库。

## 当前重点

**自动生成利于参数辨识的标定路点（excitation trajectory / informative waypoints）**：根据机器人模型与约束条件，生成使辨识问题条件数更优、参数可辨识性更好的标定路点或激励轨迹。

## 仓库结构

```
robot-calibration-playground/
├── README.md
├── .gitignore
├── requirements.txt              # 当前 Python 运行依赖
├── docs/                         # 文档、笔记、标定理论记录
│   ├── theory/                   # 可辨识性、激励轨迹设计笔记
│   └── experiments/              # 实验记录、标定结果对比
├── python/                       # 当前阶段：纯 Python 开发
│   ├── excitation_trajectory/    # 利于辨识的标定路点生成（不依赖 ROS2）
│   │   ├── aabb_snake_scan.py    # 连续蛇形网格、姿态扰动、链式 IK、3D 可视化
│   │   ├── export_9011.py        # 扫描结果 -> 9011/N9011 点位与逐点行走报文
│   │   ├── generator.py          # 路点生成主逻辑
│   │   ├── optimizer.py          # 可辨识性指标优化
│   │   └── tests/                # 单元测试
│   ├── identification/           # 参数辨识相关
│   ├── handeye/                  # 手眼验证（VPE）与 DH 离线重算（纯 Python）
│   │   ├── se3.py                # SE(3) 运算、手眼矩阵 16 值装载/校验/单位换算
│   │   ├── dh.py                 # URDF 基准 + DH 覆盖 -> 旧/新 DH 的 FK
│   │   ├── proto_codec.py        # 公司 calibration proto 的运行时编译与 .pb 读写
│   │   ├── pose_encoder.py       # T_bh <-> 控制器 robot_pt（mm + XYZ 内旋欧拉）
│   │   ├── pb_rewrite.py         # 用 FK 重算位姿并改写 .pb（喂给公司 C++ 求解器）
│   │   ├── session.py            # 会话 JSON 装载与校验（含 FK 一致性自检）
│   │   ├── vpe.py                # 视觉链推算与 VPE 指标
│   │   ├── report.py             # markdown 报告
│   │   ├── verify.py             # VPE 验证 CLI
│   │   ├── offline_recompute.py  # 旧/新 DH 离线重算 CLI
│   │   └── tests/                # 单元测试
│   ├── robot_model/              # 机器人运动学/动力学模型抽象
│   │   ├── __init__.py           # 运动学公共接口
│   │   ├── nexbot_ik.py          # 标准 URDF + JSON 输入的 6R FK/解析 IK
│   │   └── json_to_urdf.py       # JSON -> 标准 URDF 转换
│   └── common/                   # 工具函数、配置加载、日志
├── ros2_ws/                      # 预留：未来对接公司 ROS2 项目
│   └── src/                      # 后续放 ament_python / ament_cmake 包
└── data/                         # 配置模板与样本（大文件不入库）
    ├── configs/
    │   ├── ch2000_origin.urdf    # CH2000/Nexbot 标准机器人模型
    │   ├── dh_override_example.json  # DH 覆盖参数模板
    │   └── session_example.json      # 手眼验证会话模板（可直接跑通）
    └── samples/                  # 小样本数据
```

## 模块边界

- `python/excitation_trajectory/` 是**纯算法模块**，不依赖 ROS2。输入机器人模型与约束条件，输出路点序列或轨迹参数，可独立运行与单元测试。
  - `aabb_snake_scan.py` 生成连续蛇形 AABB 网格路点，支持姿态扰动、链式 IK 和 3D 可视化。
- `python/robot_model/` 放置机器人运动学/动力学模型抽象，不依赖 ROS2。
  - `nexbot_ik.py` 支持标准 URDF 与 JSON 输入的 Nexbot/CH2000 6R 正运动学和解析逆运动学。
  - `json_to_urdf.py` 将控制器 JSON 转换为标准 URDF。
- `data/configs/ch2000_origin.urdf` 是可直接用于上述 6R 工具的标准 URDF 配置模板。
- `python/identification/`、`python/common/` 同样保持与 ROS2 解耦。
- `python/handeye/` 做手眼标定的**独立验证**（VPE，Visual Positioning Error），**不重写手眼求解算法**——求解器复用公司 `calibration/eye_on_hand_calibration` 动作；本包只补 C++ 做不到的部分：用关节角 + DH 参数算位姿、改写 `.pb`、算 VPE 指标与出报告。
- `ros2_ws/` 仅作为未来 ROS2 对接层的预留位置，当前不放置 ROS2 包。

## ROS2 对接预期

未来 ROS2 节点计划放在 `ros2_ws/src/`，通过 import `python/excitation_trajectory` 的方式复用算法核心，**不重复实现**。ROS2 包只负责话题收发、参数读取、服务调用等对接逻辑。

## 数据与配置

- `data/configs/` 存放配置模板，例如机器人关节限位、激励轨迹参数和标准 URDF。
- 实际标定数据、bag 文件、大体积 CSV 不入库，已在 `.gitignore` 中排除。
- 公司内部专用参数不提交，本地通过覆盖文件或环境变量加载。
- 公司内部 `Robot_A.json` 原始配置不进仓库；执行 JSON -> URDF 转换时，由用户通过显式本地路径提供。

## 环境与依赖

当前阶段依赖见 `requirements.txt`：

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 当前工具使用

以下命令均在仓库根目录执行，不需要 ROS2：

```bash
# 1. AABB 扫描：默认执行姿态扰动和链式 IK
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 --seed 0

# 1b. 姿态激励条件数（一次性设计检查，门禁取设计值，默认阈值 9）
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 \
  --cond-max 9

# 1c. 自定义三轴扰动半宽（deg）：例如 20/15/10，设计条件数为 2.0
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 \
  --perturb-deg 20 15 10 --cond-max 9

# 1b. 指定欧拉约定：xyz-intrinsic = Rx·Ry·Rz（Nexbot 控制器链路）
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 --euler xyz-intrinsic

# 2. 仅预览连续蛇形网格，不求解 IK
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  0 200 0 200 0 200 --xs 2 --ys 2 --zs 2 --skip-ik

# 3. Nexbot/CH2000 FK/解析 IK 随机往返自测
PYTHONPATH=python python -m robot_model.nexbot_ik --selftest 100

# 4. JSON -> 标准 URDF：显式提供内部原始 JSON 和目标文件
PYTHONPATH=python python -m robot_model.json_to_urdf \
  /path/to/Robot_A.json /tmp/ch2000_origin.urdf

# 5. 扫描结果 -> 9011/N9011 点位下载 + 逐点行走报文（DH 标定采点）
PYTHONPATH=python python -m excitation_trajectory.export_9011 \
  scan_result.json --out calib_points_9011.json \
  --messages calib_9011_messages.txt --dwell 6
```

AABB CLI 默认会在当前工作目录生成 `scan_result.json` 和 `scan_path.png`；这两个运行产物已被 `.gitignore` 忽略。JSON 转换器的默认输出是 `data/configs/ch2000_origin.urdf`，但公司内部原始 `Robot_A.json` 始终由调用方显式传入，不会进入仓库。

### 姿态激励条件数（姿态覆盖度诊断）

这是一次性的**设计方案检查**：路点用于最小二乘参数辨识，姿态集必须充分激励三个转动自由度。工具在 **IK 之前**计算两类条件数（`cond(X)=σmax/σmin`，X 为 N×3 激励设计矩阵，`σ` 为奇异值）：

- **设计条件数**（`design_condition_number`）：解析值，**与随机种子、采样次数无关**。各路点扰动独立均匀，切空间协方差 `Σ = J·diag(a²/3)·Jᵀ`（J 为旋转向量对 rpy 的雅可比），设计矩阵满足 `XᵀX ≈ N·Σ`，故 `cond(X) = sqrt(λmax(Σ)/λmin(Σ))`。轴对齐姿态下 `J` 正交，因此 `cond ≈ a_max/a_min`；默认幅度 ±5°/±20°/±15° → ≈4.0，均衡 10/10/10 → 1.0，`--perturb-deg 20 15 10` → ≈2.0，`--perturb-deg 20 15 0.5` → 40（会被拒绝）。
- **实测条件数**（`pose_condition_number`）：对实际采样的姿态集做 SVD，按块报告——姿态块（相对标称参考姿态的 SO(3) log 旋转向量，rad）与位置块（mm），每块给出各自条件数与去中心化后的**绝对 RMS**（用于暴露绝对幅度）。

门禁规则：

- 门禁值 = **姿态块的设计条件数**（与 seed 无关）；`--cond-position` 时与位置块条件数取较大者。实测值是样本估计，只做诊断，不参与门禁（否则同一方案会因 seed 不同时过时不过）；
- **秩亏例外**：实测姿态块为 `inf`（路点过少 / 姿态完全相同）时，无论设计值多小都直接判 `rejected`，因为设计值描述的是方案、不能代表这批样本真的展开；
- 超过 `--cond-max`（默认 9）时打印**可执行建议**：
  `需要 a_min >= a_max/cond_max = 2.22 deg（当前 0.5）；可用 --perturb-deg 调整或提高 --cond-max`，`--strict` 下以退出码 2 中止，否则继续执行；
- `--no-cond-check` 只报告不门禁（`status="reported_only"`）；`--no-perturb` / `--skip-ik` 下姿态全同、实测度量退化，状态记 `not_run`（设计值仍报告，不打印误导性数字）；
- **不再**「超阈值就重采样扰动」：重采样不改变三轴幅度比，只在同一分布内挑样本噪声。要降低条件数只能改设计参数——提高最小轴幅度（`--perturb-deg`），或放宽 `--cond-max`。

语义换算与单位约定：

- 阈值 9 等价于 `cond(XᵀX) = 81`，即约损失 1 位有效数字；需要 2 位有效数字时应把阈值降到约 3.2；
- `--cond-position` 附带的合并 6D 条件数经**等 RMS 归一化**，抹掉了绝对幅度信息，**只能作诊断**（JSON 中标注 `metric: "...-equal-rms-combined"`、`combined_is_diagnostic_only: true`），不作为 6D 激励的证明，也不参与门禁；
- 参考姿态统一取标称 RPY（`--rpy`），消除「参考 = 第一个路点」的顺序依赖（JSON 中 `reference_source: "explicit"`）。

注意：这里度量的是**姿态集激励覆盖度**的条件数，不是 DH 参数辨识回归矩阵的条件数——后者依赖关节角，需在 IK 之后（对成功子集）另行计算。JSON 的 `conditioning` 节给出 `design` / `realized` / `post_ik`（IK 后成功子集，只报告）三段，非有限值一律写 `null`，`--seed` 相同则输出可复现。

## 9011 点位下载与逐点采点

`export_9011.py` 把 `aabb_snake_scan.py` 的扫描结果转成下位 `9011`/`N9011` 的定宽字符串报文，现场“标定软件采点 → 跟踪仪测量”循环用它驱动机器人。

### 为什么用轴坐标

路点里的姿态是逐点随机扰动的，链式 IK 已为本仓选定了具体构型。若下发笛卡尔位姿，控制器会自己解 IK，可能跳到另一个构型（腕翻转/肘上肘下），导致相邻点关节大幅跳变。用**轴坐标**（`coord=1`，模型关节角度）可精确复现选定构型。

### 报文格式（逐字符对齐 C++ 驱动）

| 报文 | 格式 | 说明 |
| --- | --- | --- |
| 下载点位 | `G{count}` + 每点 `coord(1) + 每轴 8 字(2 位小数) + P 编号(4)` | 6 轴即每点 53 字；报文上限 **240 字符**、`G` 头计数仅 1 位 → 自动分包，6 轴每包最多 4 点 |
| 逐点行走 | `T{begin:04d}{end:04d}` + `type(1)+speed(7)+acc(1)+tool(1)+ratio(4)` | 每条 23 字；`begin==end` 即只走 1 点 |
| 退出 | `Q0` | |

外轴站（7/8 轴）：每点记录长度为 `1 + (6+外轴数)×8 + 4`（Nexbot `N9011ReadPos` 6 轴=53、8 轴=69），用 `--ext-axis` 追加，**必须与下位 JOB 的 stride 一致**。

### 关于“每点暂停 6 秒”（关键）

**9011 的 `T` 循环里没有停留/等待指令**（`N9011.JBR` 的 `WHILE` 与 Hyundai `9011.JOB` 的 `FOR` 都是 `MOVE` 完直接下一点）。所以 6 秒暂停**只能由上位发送循环实现**：

1. 发一条**单点** `T`（不要发 `T0001{0048}` 范围，否则机器人连续走完）；
2. 等运动完成（驱动侧 `MovePointAndWait` 的做法是轮询到 `Ready`）；
3. `sleep --dwell` 秒，让跟踪仪完成测量；
4. 再发下一条。

报文文件里在每个点之间插了 `# [n/N] 等运动完成后 sleep 6 s 再发下一条` 注释行作提示；`--dwell` 的值同时写进 JSON 的 `meta.dwell_s`，供发送端读取而不是硬编码。

### 两个必须现场确认的点

- **关节方向映射**：导出的角度是**模型轴角**。若实际控制器 `Direction` 不全为 1（CH2000 本体为全 1，但加外轴/不同站点需核实），需先换算再下发；JSON 的 `meta.note` 已标此项。
- **平滑等级 `--acc`**：默认 `0`（`A=0`，点到点精确停位），采点场景**不要**设大值，否则机器人会在目标点附近平滑掠过而不真正停住。

## 手眼验证（VPE）与 DH 离线重算

用途：用激光跟踪仪作为独立参考，验证「机器人 + 手眼标定」的最终空间定位精度，并回答一个具体问题——**DH 参数标定到底能不能改善手眼精度**。

### 设计原则：不重写求解器

手眼求解器只保留公司 C++ 一份（动作 `calibration/eye_on_hand_calibration`）。原因是它只接受**笛卡尔位姿**，不知道关节角、也不知道 DH；离线重算真正缺的能力是「用新 DH 从关节角重算位姿」。所以本包只做三件事：算位姿、改写 `.pb`、算 VPE。

### 核心公式

```
^B P_ball^vision(i) = FK(q_i, θ) · ^H T_C · [^C P_ball(i); 1]
e_i                  = ^B P_ball^vision(i) − (^B P_ball^laser + ^B ΔP)
VPE_i                = ‖e_i‖,  VPE_RMSE = sqrt(mean(VPE_i²))
```

`^B ΔP` 是建立 laser→base 坐标注册时产生的 Tool0—靶球固定偏置；`^C P_ball(i)` 是相机在该位姿下**直接辨识靶球**得到的球心坐标（无需标定板布局）。

### 用法

```bash
# 1. VPE 验证（X 已标定好）
PYTHONPATH=python python3 -m handeye.verify data/configs/session_example.json \
  -o vpe_report.md --json vpe_report.json

# 2. 用命令行给的 X 覆盖会话内的矩阵（单位默认 mm）
PYTHONPATH=python python3 -m handeye.verify SESSION.json \
  --matrix16 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 --matrix-units mm

# 3. DH 离线重算：同一批采样，只换运动学模型
PYTHONPATH=python python3 -m handeye.offline_recompute \
  --pb-dir /path/to/handeye_pb --session SESSION.json \
  --dh-old old.json --dh-new new.json --split 0.7 --seed 0 \
  --solver-cmd 'docker exec arc-eye-on-hand bash -lc "ros2 run calibration_py eye_on_hand_calibration_solver --input-dir {input_dir} --output-bin {output_bin}"' \
  -o offline.md --json offline.json
```

### 数据准备：`.pb` → 会话 JSON

原始数据是公司管线落盘的 `ArucoRecordingResult`（.pb，含笛卡尔位姿 + 相机系靶球点）。本仓库**不改动**原始 `.pb`，只在内存/临时目录里改写后交给求解器；会话 JSON 由调用方提供，格式见 `data/configs/session_example.json`。

关键字段与约定：

| 字段 | 说明 |
| --- | --- |
| `meta.robot_urdf` | 机器人模型（URDF），相对仓库根或绝对路径 |
| `meta.dh_override` | DH 覆盖 JSON；`null` 表示用 URDF 基准参数 |
| `meta.joint_convention` | 固定 `"model"`（模型轴角度，API 不传 Direction 映射）|
| `meta.euler_convention` | 可选，`"xyz-intrinsic"`（默认，Nexbot 链路）/ `"xyz-extrinsic"`（定轴 RPY），作用于 `recorded_pose` 的 orientation |
| `meta.units` | 固定 `"mm"` |
| `reference.p_ball_laser_base` | 跟踪仪实测球心（换算到基座系）|
| `reference.delta_p_base` | Tool0—靶球固定偏置 `^B ΔP`，可为 `null` |
| `handeye_matrix` | `{"matrix16_rowmajor": [...], "units": "mm"|"m"}`，可为 `null` |
| `samples[].q_deg` | **关节角（模型轴，度）**，必填，必须在 URDF 限位内 |
| `samples[].recorded_pose` | 控制器回传的笛卡尔位姿；用于 FK 一致性自检 |
| `samples[].frames[].target_points` | `{靶球id: [x,y,z] 相机系 mm}`，可多帧、多 id |

### 两条必须知道的自检与陷阱

1. **运动学一致性自检（`--fk-tolerance-mm`，默认 1.0）**：把 `recorded_pose` 与 `FK(q, θ)` 对比。若这里就超阈值，说明 DH、单位或欧拉约定有错，**后续 VPE 结论全部不可信**。典型错误是欧拉约定——控制器/Nexbot 链路用 **XYZ 内旋**（`R = Rx·Ry·Rz`，C++ `kXyzIntrinsic`），本仓库 `pose_encoder` 默认即此约定；若会话里的 `recorded_pose` 来自默认定轴 RPY（`Rz·Ry·Rx`）的工具，在 `meta.euler_convention` 里写 `"xyz-extrinsic"` 即可，两约定同名可选项为 `xyz-intrinsic | xyz-extrinsic`。
2. **手眼矩阵单位有两套**：`common_config` 存的是**米制**（`toolFromCameraMm()` 乘 1000 可证），而动作 `EyeOnHandCalibrationResult.matrix` 返回的是 **mm**（全程用 robot_pt 的 mm 计算）。`se3.load_matrix16` 强制显式声明 `units`，不会静默换算错。

### 离线重算的两个方法学选择

- `--pose-source-old recorded|fk_old`（默认 `recorded`）：生产上的 `X_old` 是用**控制器回传位姿**算的。若强行换成我们的 `FK(θ_old)`，基线就被改动、`ΔVPE` 不再可比。
- 四个组合中 `θ_new×X_old`、`θ_old×X_new` **不具备物理一致性**，报告里只作**诊断**（判断误差主要来自运动学还是手眼矩阵），不作最终性能结论。

### 判读规则

`ΔVPE_RMSE = VPE_old − VPE_new`（正值表示改善），相对基线 `Δ/RMSE_old`：

| 比值 | 判读 |
| --- | --- |
| `≥ 10%` | 新 DH 使手眼定位精度改善超过 10%，支持采用标定后参数 |
| `−10% ~ 10%` | 影响在 ±10% 内，不足以支持结论，建议增加样本或复核标定残差 |
| `< −10%` | 新 DH 使精度变差，需检查 DH 标定结果与零点定义转换 |

### 依赖说明

- 核心（`se3`/`dh`/`session`/`vpe`/`report`/`verify`）：仅 `numpy`
- `.pb` 读写：需要系统 `protoc` + `protobuf`，且能访问公司 `arc-common` 的 proto 根目录（环境变量 `ARC_COMMON_PROTO_ROOT`，默认 `~/ws/3srobotics/arc-common/src/proto_msgs`）。绑定时运行时生成到 `build/proto_gen/`（已 gitignore），**公司 schema 不入库**
- 离线重算的求解步骤：需要能调用公司 ROS2 栈（`--solver-cmd` 模板），本仓库不自动执行 docker

## 实验记录

标定方向中参数设置与实验条件强相关。每次路点生成使用的约束、可辨识性指标（如条件数）、辨识结果记录在 `docs/experiments/` 下，按日期或机器人型号组织。
