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

# 1b. 姿态可辨识性校验：条件数超阈值即重新采样扰动（默认阈值 5）
PYTHONPATH=python python -m excitation_trajectory.aabb_snake_scan \
  800 1200 -200 200 800 1000 --xs 3 --ys 3 --zs 2 \
  --cond-max 5 --cond-attempts 20

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

### 姿态条件数（可辨识性）校验

路点用于最小二乘参数辨识，姿态集必须充分激励三个转动自由度。工具在 **IK 之前**对姿态集做 SVD：

- 特征：各姿态相对参考姿态的 SO(3) log 映射旋转向量（N×3，去中心化）；
- 条件数 `σmax/σmin` 超过 `--cond-max`（默认 5）则重新采样姿态扰动，最多 `--cond-attempts` 次（默认 20），达标后才进入 IK；
- `--cond-position` 把位置也计入（与姿态等 RMS 归一化），`--no-cond-check` 只报告不重采样；
- 全部尝试均未达标时保留最优采样并告警，`--strict` 下以退出码 2 中止；
- 条件数与采样结果写入 JSON 的 `conditioning` 节，同 `--seed` 可复现。

注意：这里度量的是**位姿集激励**的条件数。若要评估 DH 参数辨识矩阵的条件数（依赖关节角），需在 IK 之后另行计算。

## 实验记录

标定方向中参数设置与实验条件强相关。每次路点生成使用的约束、可辨识性指标（如条件数）、辨识结果记录在 `docs/experiments/` 下，按日期或机器人型号组织。
