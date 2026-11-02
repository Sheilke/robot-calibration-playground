# robot-calibration-playground

个人机器人标定方向的开发仓库，用于算法积累与实验记录。当前以 Python 为主，后续计划与公司 ROS2 项目对接。

本仓库为个人开发空间，不代表公司项目；涉及公司内部参数与配置的内容不会提交到此仓库。

## 当前重点

**自动生成利于参数辨识的标定路点（excitation trajectory / informative waypoints）**：根据机器人模型与约束条件，生成使辨识问题条件数更优、参数可辨识性更好的标定路点或激励轨迹。

## 仓库结构
robot-calibration-playground/
├── README.md
├── .gitignore
├── requirements.txt # 当前 Python 全局依赖
├── docs/ # 文档、笔记、标定理论记录
│ ├── theory/ # 可辨识性、激励轨迹设计笔记
│ └── experiments/ # 实验记录、标定结果对比
├── python/ # 当前阶段：纯 Python 开发
│ ├── excitation_trajectory/ # 利于辨识的标定路点生成
│ │ ├── generator.py # 路点生成主逻辑
│ │ ├── optimizer.py # 可辨识性指标优化
│ │ └── tests/ # 单元测试
│ ├── identification/ # 参数辨识相关
│ ├── robot_model/ # 机器人运动学/动力学模型抽象
│ └── common/ # 工具函数、配置加载、日志
├── ros2_ws/ # 预留：未来对接公司 ROS2 项目
│ └── src/ # 后续放 ament_python / ament_cmake 包
└── data/ # 配置模板与样本（大文件不入库）
├── configs/ # 机器人模型参数、激励轨迹参数模板
└── samples/ # 小样本数据

## 模块边界

- `python/excitation_trajectory/` 是**纯算法模块**，不依赖 ROS2。输入机器人模型与约束条件，输出路点序列或轨迹参数，可独立运行与单元测试。
- `python/identification/`、`python/robot_model/`、`python/common/` 同样保持与 ROS2 解耦。
- `ros2_ws/` 仅作为未来 ROS2 对接层的预留位置，当前不放置 ROS2 包。

## ROS2 对接预期

未来 ROS2 节点计划放在 `ros2_ws/src/`，通过 import `python/excitation_trajectory` 的方式复用算法核心，**不重复实现**。ROS2 包只负责话题收发、参数读取、服务调用等对接逻辑。

## 数据与配置

- `data/configs/` 存放配置模板，例如机器人关节限位、激励轨迹参数。
- 实际标定数据、bag 文件、大体积 CSV 不入库，已在 `.gitignore` 中排除。
- 公司内部专用参数不提交，本地通过覆盖文件或环境变量加载。

## 环境与依赖

当前阶段依赖见 `requirements.txt`：

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
实验记录

标定方向中参数设置与实验条件强相关。每次路点生成使用的约束、可辨识性指标（如条件数）、辨识结果记录在 docs/experiments/ 下，按日期或机器人型号组织。
