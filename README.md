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
│   │   ├── generator.py          # 路点生成主逻辑
│   │   ├── optimizer.py          # 可辨识性指标优化
│   │   └── tests/                # 单元测试
│   ├── identification/           # 参数辨识相关
│   ├── robot_model/              # 机器人运动学/动力学模型抽象
│   │   ├── __init__.py           # 运动学公共接口
│   │   ├── nexbot_ik.py          # 标准 URDF + JSON 输入的 6R FK/解析 IK
│   │   └── json_to_urdf.py       # JSON -> 标准 URDF 转换
│   └── common/                   # 工具函数、配置加载、日志
├── ros2_ws/                      # 预留：未来对接公司 ROS2 项目
│   └── src/                      # 后续放 ament_python / ament_cmake 包
└── data/                         # 配置模板与样本（大文件不入库）
    ├── configs/
    │   └── ch2000_origin.urdf    # CH2000/Nexbot 标准机器人模型
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

# 1c. 自定义三轴扰动半宽（deg）：把最小轴从 3 提高到 10，设计条件数 6.7 -> 2.0
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 \
  --perturb-deg 20 15 10 --cond-max 9

# 2. 仅预览连续蛇形网格，不求解 IK
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  0 200 0 200 0 200 --xs 2 --ys 2 --zs 2 --skip-ik

# 3. Nexbot/CH2000 FK/解析 IK 随机往返自测
PYTHONPATH=python python -m robot_model.nexbot_ik --selftest 100

# 4. JSON -> 标准 URDF：显式提供内部原始 JSON 和目标文件
PYTHONPATH=python python -m robot_model.json_to_urdf \
  /path/to/Robot_A.json /tmp/ch2000_origin.urdf
```

AABB CLI 默认会在当前工作目录生成 `scan_result.json` 和 `scan_path.png`；这两个运行产物已被 `.gitignore` 忽略。JSON 转换器的默认输出是 `data/configs/ch2000_origin.urdf`，但公司内部原始 `Robot_A.json` 始终由调用方显式传入，不会进入仓库。

### 姿态激励条件数（姿态覆盖度诊断）

这是一次性的**设计方案检查**：路点用于最小二乘参数辨识，姿态集必须充分激励三个转动自由度。工具在 **IK 之前**计算两类条件数（`cond(X)=σmax/σmin`，X 为 N×3 激励设计矩阵，`σ` 为奇异值）：

- **设计条件数**（`design_condition_number`）：解析值，**与随机种子、采样次数无关**。各路点扰动独立均匀，切空间协方差 `Σ = J·diag(a²/3)·Jᵀ`（J 为旋转向量对 rpy 的雅可比），设计矩阵满足 `XᵀX ≈ N·Σ`，故 `cond(X) = sqrt(λmax(Σ)/λmin(Σ))`。轴对齐姿态下 `J` 正交，因此 `cond ≈ a_max/a_min`；默认幅度 ±20°/±15°/±3° → ≈6.7，均衡 10/10/10 → 1.0，`--perturb-deg 20 15 10` → ≈2.0，`--perturb-deg 20 15 0.5` → 40（会被拒绝）。
- **实测条件数**（`pose_condition_number`）：对实际采样的姿态集做 SVD，按块报告——姿态块（相对标称参考姿态的 SO(3) log 旋转向量，rad）与位置块（mm），每块给出各自条件数与去中心化后的**绝对 RMS**（用于暴露绝对幅度）。

门禁规则：

- 门禁值 = **姿态块的设计条件数**（与 seed 无关）；`--cond-position` 时与位置块条件数取较大者。实测值是样本估计，只做诊断，不参与门禁（否则同一方案会因 seed 不同时过时不过）；
- **秩亏例外**：实测姿态块为 `inf`（路点过少 / 姿态完全相同）时，无论设计值多小都直接判 `rejected`，因为设计值描述的是方案、不能代表这批样本真的展开；
- 超过 `--cond-max`（默认 9）时打印**可执行建议**：
  `需要 a_min >= a_max/cond_max = 2.22 deg（当前 0.5）；可用 --perturb-deg 调整或提高 --cond-max`，`--strict` 下以退出码 2 中止，否则继续执行；
- `--no-cond-check` 只报告不门禁（`status="reported_only"`）；`--no-perturb` / `--skip-ik` 下姿态全同、实测度量退化，状态记 `not_run`（设计值仍报告，不打印误导性数字）；
- **不再**「超阈值就重采样扰动」：重采样不改变三轴幅度比，只在同一分布内挑样本噪声（默认 20/15/3 下大量 seed 的首采样本本来就都低于阈值）。要降低条件数只能改设计参数——提高最小轴幅度（`--perturb-deg`），或放宽 `--cond-max`。

语义换算与单位约定：

- 阈值 9 等价于 `cond(XᵀX) = 81`，即约损失 1 位有效数字；需要 2 位有效数字时应把阈值降到约 3.2；
- `--cond-position` 附带的合并 6D 条件数经**等 RMS 归一化**，抹掉了绝对幅度信息，**只能作诊断**（JSON 中标注 `metric: "...-equal-rms-combined"`、`combined_is_diagnostic_only: true`），不作为 6D 激励的证明，也不参与门禁；
- 参考姿态统一取标称 RPY（`--rpy`），消除「参考 = 第一个路点」的顺序依赖（JSON 中 `reference_source: "explicit"`）。

注意：这里度量的是**姿态集激励覆盖度**的条件数，不是 DH 参数辨识回归矩阵的条件数——后者依赖关节角，需在 IK 之后（对成功子集）另行计算。JSON 的 `conditioning` 节给出 `design` / `realized` / `post_ik`（IK 后成功子集，只报告）三段，非有限值一律写 `null`，`--seed` 相同则输出可复现。

## 实验记录

标定方向中参数设置与实验条件强相关。每次路点生成使用的约束、可辨识性指标（如条件数）、辨识结果记录在 `docs/experiments/` 下，按日期或机器人型号组织。
