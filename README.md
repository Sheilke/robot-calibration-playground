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
│   │   └── json_to_urdf.py       # ② 控制器 JSON -> 标准 URDF
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
# 基本用法：AABB 范围 + 各轴格点 + 姿态扰动 + 链式 IK
PYTHONPATH=python python3 -m excitation_trajectory.aabb_snake_scan \
  750 1300 -600 600 270 940 --xs 3 --ys 3 --zs 2 --z-insert 1 --seed 0 \
  --urdf data/configs/hyundai_general6s.urdf \
  --rpy 3 73 -167 --euler xyz-intrinsic \
  --q-seed 43,-30,-36,42,-89,-1 --perturb-deg 20 20 35 --j5-min 20
```

上面这条即最终交付的那批点：`xs=3, ys=3, zs=2, z-insert=1` → **48 + 2 = 50 个路点**。
默认在当前目录生成 `scan_result.json` 与 `scan_path.png`（均已 gitignore）。

主要约束与门禁：

- `--perturb-deg A B C`：三轴姿态扰动半宽（度），决定姿态激励覆盖度；
- `--cond-max`：姿态设计条件数上限，默认 9（等价 `cond(XᵀX)=81`）。门禁取**设计值**（与 seed 无关），实测值只作诊断；
- `--j5-min` / `--j3-min` / `--seg-min`：J5、J3 与相邻点之间的安全裕度;
- `--z-insert N`：在每层竖直过渡处插入 N 个点（不是额外整层）；
- `--skip-ik` 只看网格不求解；`--euler xyz-extrinsic|xyz-intrinsic` 指定欧拉约定，默认外旋。

条件数的语义、门禁规则与单位换算细节，见脚本 docstring 与 `--help`。

## 功能二：控制器 JSON → 标准 URDF

```bash
# 显式提供公司原始 JSON 与目标路径
PYTHONPATH=python python3 -m robot_model.json_to_urdf \
  /path/to/Robot_A.json /tmp/model.urdf
```

转换器读取控制器 JSON 的 `DH_model` / 杆长 / 限位 / `Direction`，输出本仓解析 IK
可用的标准 URDF（球型腕结构前提：`alpha=[90,0,90,90,-90,0]`、`a4=a5=d5=0`）。

`data/configs/robot_sim/` 下是三个 CH2000 控制器配置样例：

| 文件 | 说明 |
| --- | --- |
| `ch2000_3sr_grslider_7axis.json` | 7 轴，带导轨 |
| `ch2000_3sr_inverted_7axis_new.json` | 倒装 7 轴；**其 FK 与 `ch2000_origin.urdf` 一致** |
| `ch2000_3sr_inverted_8axis.json` | 倒装 8 轴 |

另外两个与 `ch2000_origin.urdf` 相差一个**固定的右侧法兰 `Rz(180°)`**，如需用它们做
FK/IK，须补上该固定变换。本仓目前**没有**外部轴的 IK 支持，转换出的 URDF 只含六轴本体。

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
  -o position_R1.json --cartesian fk --cart-units rad
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
