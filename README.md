# robot-calibration-playground

个人机器人标定方向的开发仓库，用于算法积累与实验记录。当前以 Python 为主，后续计划与公司 ROS2 项目对接。

本仓库为个人开发空间，不代表公司项目；涉及公司内部参数与配置的内容不会提交到此仓库。

## 当前重点

**自动生成利于参数辨识的标定路点（excitation trajectory / informative waypoints）**：根据机器人模型与约束条件，生成使辨识问题条件数更优、参数可辨识性更好的标定路点，并导出为控制器可用的点位文件。

本分支（`main`）只保留这条主线必需的三个功能：

1. **生成 50 个标定点** —— `aabb_snake_scan.py`：AABB 蛇形网格 + 姿态扰动 + 链式 IK + 条件数门禁；
2. **控制器 JSON → 标准 URDF** —— `json_to_urdf.py`；
3. **50 点写入 GP 文件** —— `export_gp.py`：把关节角写进 GP 模板的 14 字段布局。

其余功能（手眼验证 VPE、9011 报文导出与 TCP 发送、ISO 9283 五点）完整保留在 **`dev_0.1`** 分支，未删除。

## 仓库结构

```
robot-calibration-playground/
├── README.md
├── .gitignore
├── requirements.txt              # numpy + matplotlib
├── python/
│   ├── excitation_trajectory/
│   │   ├── aabb_snake_scan.py    # ① 扫描点生成：蛇形网格、姿态扰动、链式 IK、3D 可视化
│   │   ├── export_gp.py          # ③ 扫描结果 -> GP 文件的 14 字段布局
│   │   └── tests/
│   ├── robot_model/
│   │   ├── nexbot_ik.py          # 6R FK / 解析 IK，URDF 与 JSON 解析
│   │   ├── json_to_urdf.py       # ② 控制器 JSON -> 标准 URDF（--model 后缀）
│   │   └── tests/
│   ├── handeye/                  # 仅保留 export_gp --cartesian fk 所需的三件
│   │   ├── se3.py                # SE(3) 运算、变换矩阵装载/校验/单位换算
│   │   ├── dh.py                 # URDF 基准 + DH 覆盖 -> FK
│   │   └── pose_encoder.py       # T_bh -> 控制器 robot_pt（mm + XYZ 内旋欧拉）
│   ├── identification/           # 预留
│   └── common/                   # 预留
└── data/
    ├── configs/
    │   ├── ch2000_origin.urdf           # CH2000（昌宏）标准模型
    │   ├── hyundai_general6s.urdf        # Hyundai R_GENERAL_6S 标准模型
    │   ├── hyundai_general6s_dh.json     # 上者的 DH 显式副本
    │   ├── hyundai_general6s_scan.json   # 一次 50 点扫描的参数记录
    │   └── robot_sim/                    # 控制器原始 JSON（3 个 CH2000 配置）
    └── samples/
```

> 注意：`python/handeye/` 在本分支只保留 `se3` / `dh` / `pose_encoder` 三个模块，
> 因为 `export_gp --cartesian fk` 需要由关节角算笛卡尔位姿。完整手眼功能在 `dev_0.1`。

## 数据与配置

- `data/configs/` 存放机器人模型与配置模板。
- 实际标定数据、bag 文件、大体积 CSV 不入库，已在 `.gitignore` 中排除。
- 公司内部专用参数不提交，本地通过覆盖文件或环境变量加载。
- 公司内部 `Robot_A.json` 原始配置不进仓库；执行 JSON → URDF 转换时，由用户通过显式本地路径提供。

## 环境与依赖

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
PYTHONPATH=python python -m pytest python -q
```

## 功能一：生成 50 个标定点

```bash
# 最简形式：直接给可达区域的两个对角点 + 目标点数，其余自动
PYTHONPATH=python python3 -m excitation_trajectory.aabb_snake_scan \
  --point-min 750 -600 270 --point-max 1300 600 940 \
  --target-count 50 \
  --robot-config /path/robot_R1.json \
  --rpy 3 73 -167 --euler xyz-intrinsic \
  --q-seed 43,-30,-36,42,-89,-1 \
  --perturb-deg 20 20 35 --j5-min 20 --seed 0 \
  --out position_R1_scan50.json

# 等价写法：给 6 个交错数值 + 显式网格（交付那批 = 48 格点 + 2 个换层插点）
PYTHONPATH=python python3 -m excitation_trajectory.aabb_snake_scan \
  750 1300 -600 600 270 940 --xs 3 --ys 3 --zs 2 --z-insert 1 --seed 0 \
  --urdf data/configs/hyundai_general6s.urdf \
  --rpy 3 73 -167 --euler xyz-intrinsic \
  --q-seed 43,-30,-36,42,-89,-1 --perturb-deg 20 20 35 --j5-min 20
```

### 三种输入形式

| 输入 | 写法 |
| --- | --- |
| AABB 边界 | 6 个交错数值：`x_min x_max y_min y_max z_min z_max` |
| 两个对角点 | `--point-min X Y Z --point-max X Y Z`（与上式互斥；某轴顺序反了会自动按分量归一化并提示） |
| 网格份数 | `--xs/--ys/--zs`（顶点数 = 份数+1）+ `--z-insert`；**默认 `2×5×5`（=50 点）** |
| 目标点数 | `--target-count N`：自动选一组份数恰好凑出 N 点，与 `--xs/--ys/--zs/--z-insert` 互斥 |

路点总数 = `(xs+1)(ys+1)(zs+1) + zs·z_insert`。`--target-count` 只在**精确命中**的组合里挑，优先级：
优先纯网格（插点只动 Z、不增加 XY 覆盖）→ 各轴格距最均匀（最大/最小比接近 1）→ 插点更少 → 层数更多。
例如 50 点选 `2×5×5`；若某轴只能取 1 个值（如 7 点），会打印退化警告。

模型既可用 `--urdf` 也可用等价的 `--robot-config`，两者都接受 `.urdf` 或控制器 `.json`。

主要约束与门禁：

- `--perturb-deg A B C`：三轴姿态扰动半宽（度），决定姿态激励覆盖度；
- `--cond-max`：姿态设计条件数上限，默认 9（等价 `cond(XᵀX)=81`）。门禁取**设计值**（与 seed 无关），实测值只作诊断；
- `--j5-min` / `--j3-min` / `--seg-min`：J5、J3 与相邻点之间的安全裕度;
- `--z-insert N`：在每层竖直过渡处插入 N 个点（不是额外整层）；
- `--skip-ik` 只看网格不求解；`--euler xyz-extrinsic|xyz-intrinsic` 指定欧拉约定，默认外旋。

### 姿态表达（欧拉约定）：轴序与定/动轴都可选

`--euler <轴序>-<定轴|动轴>`，轴序支持 **xyz** 与 **zyx**，共四种组合，大小写不敏感：

| 约定 | 逆解出的旋转矩阵 | 典型用法 |
| --- | --- | --- |
| `xyz-extrinsic` | `R = Rz·Ry·Rx` | 定轴 RPY（**默认**，本仓历史行为） |
| `xyz-intrinsic` | `R = Rx·Ry·Rz` | 动轴 XYZ（Nexbot 控制器 `kXyzIntrinsic`） |
| `zyx-extrinsic` | `R = Rx·Ry·Rz` | 定轴 ZYX |
| `zyx-intrinsic` | `R = Rz·Ry·Rx` | 动轴 ZYX |

同一组 `--rpy` 数值在不同约定下代表**不同的姿态**，所以「参考姿态怎么读」和「轴序」必须与
控制器一致。`--euler` 会写进输出 JSON 的 `meta.euler_convention`；`export_gp --cartesian fk`
有自己的 `--euler`（默认 `xyz-intrinsic`），需与扫描时用的一致。

条件数的语义、门禁规则与单位换算细节，见脚本 docstring 与 `--help`。

## 功能二：控制器 JSON → 标准 URDF

```bash
# 输出名取输入 JSON 的主名：robot_R1.json -> robot_R1.urdf
PYTHONPATH=python python3 -m robot_model.json_to_urdf /path/to/robot_R1.json

# --model 追加型号后缀：robot_R1_CH2000.urdf
PYTHONPATH=python python3 -m robot_model.json_to_urdf /path/to/robot_R1.json \
  --model CH2000

# 显式给输出路径时，后缀加在该名字上：out_CH2000.urdf
PYTHONPATH=python python3 -m robot_model.json_to_urdf /path/to/robot_R1.json \
  out.urdf --model CH2000
```

型号会同时写入 URDF 的头注释（`型号 model=...`）以便追溯；`--model` 只允许字母数字与
`. _ -`，因为要进文件名。

转换器读取控制器 JSON 的 `Link&CoupleCoe` / `Joint` 等字段（`Robot_A.json` 那一套），
输出本仓解析 IK 可用的标准 URDF（球型腕结构前提：`alpha=[90,0,90,90,-90,0]`、`a4=a5=d5=0`），
并在末尾用 200 个随机位姿自校验 DH-FK 与 URDF-FK 是否严格等价。

`data/configs/robot_sim/` 下三份是**仿真器资产**（`robot-sim-robot` 导出格式，顶层键为
`format / id / name / assets / sensors`），**不是控制器配置**，`json_to_urdf` 不读取它们；
保留仅作 CH2000 的 DH 参考：

| 文件 | 说明 |
| --- | --- |
| `ch2000_3sr_grslider_7axis.json` | 7 轴，带导轨 |
| `ch2000_3sr_inverted_7axis_new.json` | 倒装 7 轴；**其 FK 与 `ch2000_origin.urdf` 一致** |
| `ch2000_3sr_inverted_8axis.json` | 倒装 8 轴 |

另外两份与 `ch2000_origin.urdf` 相差一个**固定的右侧法兰 `Rz(180°)`**。本仓目前**没有**
外部轴的 IK 支持；控制器 JSON 转换出的 URDF 也只含六轴本体。

## 功能二·五：奇异点自动重规划

现场跑 50 点时控制器报奇异？把**当时的 6 个关节角**交给这个工具，它会自己判断
该平移、收缩还是改参考姿态，改多少，并输出一份与扫描器同结构的 JSON（可直接
喂给 `export_gp`）。不需要人工分析。

```bash
PYTHONPATH=python python3 -m excitation_trajectory.avoid_singularity \
  --robot data/configs/hyundai_general6s.urdf \
  --point-min 1100 -300 1150 --point-max 1500 300 1500 \
  --rpy 3 73 -167 --euler xyz-intrinsic \
  --q-seed 43,-30,-36,42,-89,-1 --target-count 50 \
  --singular-q 0 -30 -60 0 12 0 \
  --out scan50_repaired.json --report avoid_report.json
```

`--singular-q` 两种写法等价：`--singular-q 0 0 0 55.7 0 -55.7`（6 个数）或
`--singular-q="0,0,0,55.7,0,-55.7"`（逗号串），可重复给多个奇异位形。

### 路径模型：默认按 MOVL 判（真实现场情形）

现场报警几乎都发生在**运动中**——MOVL 沿笛卡尔直线走，关节轨迹**不是**直线，
所以"端点满足裕度"不代表路径安全。工具默认：

* `aabb_snake_scan --seg-mode cartesian`：位置线性 + 姿态测地线插补，逐点求 IK
  （以上一采样解为参考保持构型），统计段内 `|J3|/|J5|` 最小值、穿零、无解点；
* `--seg-mode joint`：关节直线（PTP）模型，保留为历史行为，默认仍是它；
* `avoid_singularity` 的可行性判据**固定用 MOVL**（搜索 11 采样/段，复核 31）。

经验值：50 点、21 采样/段时笛卡尔检查约 0.5 s，一次完整重规划约 13 s。

### 它自己怎么判定

1. 对每个报告的位形算 FK 位置与 `|J3|`/`|J5|`；
2. **与内置判据一致**（`|J5| < j5_min` 或 `|J3| < j3_min`）→ 把实测裕度当经验
   阈值上抬（`max(内置, 实测 + --avoid-margin)`），整体推开这一类位形；
3. **与内置判据不一致**（例如 `|J5|=55.7°` 却报奇异）→ 说明控制器报警条件本仓
   模型没覆盖，退化为**位置禁区**：以该点 FK 位置为球心、`--avoid-radius` 为
   半径，禁止任何路点落入；
4. 评估原域（N 点 IK + 关节裕度 + **MOVL 段内穿零/无解** + 设计条件数），可行就直接输出；
5. 不可行则按**改动量从小到大**枚举候选：单轴平移 → 单侧收缩 → 参考姿态微调 →
   双轴平移组合，第一个可行的再用**二分细化**压到刚好可行（精度 1/16）；
6. 用完整重试次数复核后输出，报告里写明改动链、每个候选的失败原因。

退出码：`0` 已输出可行方案；`2` 全部候选不可行（报告里给失败原因，不输出扫描）。

已验证的行为（`test_avoid_singularity.py`）：原域可行时不做任何改动；奇异位置
落在域角点时自动平移拉开；`--no-rpy-change` / `--no-shrink` 可限制手段。

## 功能三：50 点写入 GP 文件

`export_gp.py` 把扫描结果里的关节角写进控制器 GP 的 14 字段布局，其余字段（笛卡尔
半、标志位、外部轴、以及模板里未受影响的寄存器与版本信息）原样保留。

### 布局

| 索引 | 含义 |
| --- | --- |
| `0..5` | 笛卡尔位姿（默认写 0，与现场样例一致；`--cartesian fk` 可由模型算出） |
| `6` | 标志位（`--flag`，默认 0） |
| `7..12` | **六个关节角（度）** |
| `13` | 外部轴 |

```bash
# 按模板生成 GP0001..GP0050（模板是位置参数，只读不修改）
PYTHONPATH=python python3 -m excitation_trajectory.export_gp \
  scan_result.json position_R1_template.json \
  -o position_R1.json --first 1

# 需要笛卡尔位姿时由模型 FK 填充；姿态按控制器读数写弧度
PYTHONPATH=python python3 -m excitation_trajectory.export_gp \
  scan_result.json position_R1_template.json \
  -o position_R1.json --cartesian fk --cart-units rad --euler xyz-intrinsic
```

- **关节角始终是度**，与 `robot_pt` 线上单位一致；`--cart-units` 只作用于
  `--cartesian fk` 写出的**笛卡尔姿态**，默认 `rad`（示教器读数是弧度）。
- 模板里的 `controllerID` / 版本号 / 未涉及寄存器不做修改；
- `--flag` 的取值需与现场已创建的样例一致，默认 0；
- 输出前会校验关节角是否在 URDF 限位内，越界即报错退出。

## 已入库的机器人模型

| 模型 | URDF | DH 副本 | 关键参数 |
| --- | --- | --- | --- |
| CH2000（昌宏） | `ch2000_origin.urdf` | — | a=`150/780/250`、d=`510.5/-47.5/1048.7/102.6`、限位 J2`[-80,93]` |
| Hyundai R_GENERAL_6S | `hyundai_general6s.urdf` | `hyundai_general6s_dh.json` | a=`159.47/770.7/141.05`、d=`446.6/1025.36/110.4`、限位 J2`[-70,180]` J3`[-80,180]` |

两者 alpha 均为 `[90,0,90,90,-90,0]` 且 `a4=a5=d5=0`，满足本仓解析 IK 的结构前提
（球型腕）。`test_robot_configs.py` 钉住三件事：URDF 可解析且结构合格、DH 副本与
URDF 基准逐分量相等（FK 差 0）、限位内 FK→IK→FK 往返闭合。
