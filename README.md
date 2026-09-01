# teleop-system — 机器人遥操作教学项目

> 机器人学课程配套项目:一个**三层架构的通用遥操作框架**,支持 LIBERO 仿真
> (Franka/OSC)与 **UR5e 真机**(ur_rtde/servoL),输入设备覆盖 VR 头显(Pico 4
> Ultra)、Xbox 手柄、录制流重放。主功能:纯遥操作 + 回合数据录制;进阶:随时
> 接管一个 VLA 策略。

---

## 1. 你将学到什么

| 主题 | 对应模块 |
|---|---|
| **分层解耦设计**:N 设备 × M 机器人 = N+M 份代码,而不是 N×M | `teleop_system/types.py` 三个接口数据类型 |
| **SO(3) 旋转数学**:四元数/旋转矩阵/轴角,坐标系变换(XR Y-up → 世界 Z-up) | `teleop_system/geometry.py` |
| **离合式增量映射**:锚定、逐拍增量、迟滞防抖、符号/增益标定 | `teleop_system/mapping/ee_delta.py` |
| **坐标系对齐标定**:站位偏航角求解、Kabsch 轨迹对齐 | `scripts/auto_calibrate_ur5e.py`、`scripts/drag_calibrate_ur5e.py` |
| **真机安全设计**:步长限幅、工作空间盒、目标-实测偏差钳位、看门狗 | `envs/ur5e/teleop_adapter.py` |
| **实时控制**:servoL 伺服语义、控制频率与插值、延迟诊断 | `envs/ur5e/teleop_record.py`、`scripts/net_monitor.py` |
| **数据采集**:回合录制、裁决(保存/作废)、raw tap 事后重映射 | 两个 `teleop_record.py` 入口 |

## 2. 三层架构(本项目的核心思想)

```
输入设备 ──L1 backends──> TeleopState ──L2 mapping──> ControlIntent ──L3 adapter──> 机器人动作
 (Pico/手柄/重放)        (每手位姿+按键)   (离合/增量/标定)  (度量增量+夹爪)   (OSC 或 servoL)
```

| 层 | 职责 | 关键约束 |
|---|---|---|
| **L1 设备后端** `teleop_system/backends/` | 只懂设备:读位姿/模拟量/按键,变换到世界系 | 不懂机器人、离合、接管 |
| **L2 语义映射** `teleop_system/mapping/` | 设备无关+机器人无关:离合锚定→逐拍增量、站位对齐(world_yaw)、符号×增益、接管语义 | 只准依赖 numpy+yaml |
| **L3 机器人适配** `envs/<robot>/teleop_adapter.py` | 唯一懂各机器人动作约定的文件 | LIBERO: OSC `[-1,1]^7`;UR5e: servoL 目标 + 三重安全钳制 |

**换输入设备只写 L1**(一个类实现 `read() -> TeleopState`),**换机器人只写 L3**。
接口由 `teleop_system/types.py` 的三个 dataclass 钉死——先读这个文件再读其他代码。

## 3. 环境安装

```bash
curl -fsSL https://pixi.sh/install.sh | bash        # 装 pixi(一次)
git clone git@github.com:Gwss-s/teleop-system.git && cd teleop-system
pixi install                                        # 核心框架 + UR5e(python3.10/numpy/ur_rtde)
pixi run test                                       # 24 项单测全过 = 安装 OK
pixi shell                                          # 进环境干活

# LIBERO 仿真(可选,重依赖):
pixi install -e sim && pixi run -e sim install-libero
```

VR 头显链路(仅真机/头显实验需要,一次性):PC Service + APK + pybind SDK,
照 `docs/pico_teleop_setup.md` 做。

## 4. 快速上手(按难度递进)

### 4.1 零硬件:重放驱动模拟器(理解数据流)

```bash
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py \
    --dry-run --backend replay --tap <raw_tap.npz> --fast --no-gui
```

`--dry-run` 用内置一阶伺服模拟器代替真机;重放已录的手柄流走完整 L1→L2→L3。

### 4.2 手柄 + 仿真(第一次亲手遥操)

```bash
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0 --backend gamepad
# 左摇杆=XY  RT/LT=Z  右摇杆=pitch/yaw  十字键=roll
# LB按住=离合  X按住=夹爪  B=保存回合  A=作废
```

### 4.3 VR 头显 + 仿真

```bash
# 顺序固定: PC Service(runService.sh) → 本程序 → 头显 app 连 PC 的 IP → Send On
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0
#   grip=移动 松手=保持 trigger=夹爪 | B=保存 A=作废
```

### 4.4 UR5e 真机(必须有指导教师在场)

**⚠ 安全第一**:上真机前**必须**核对 `configs/ur5e.yaml` 的 `workspace` 盒与
实际工作单元一致;首跑用 `--pos-scale 0.4` 低速;急停按钮在手边。
分步验收流程(离线→URSim 仿真→真机)见 `docs/ur5e_setup.md`。

```bash
# ① 标定(首次/换站位/重启头显 app 后必做): 臂逐轴演示,你捏 grip 模仿,自动解出
#    站位偏航角+逐轴符号(原理见 docs/ur5e_setup.md §3 第 4 条)
PYTHONPATH=$PWD python scripts/auto_calibrate_ur5e.py
# ② 遥操+录制
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --pos-scale 0.4
```

**启动头显 app 时人面向机器人**——头显世界系朝向 = app 启动瞬间头的朝向,
面向机器人可让站位偏航角接近 0(偏了也能标出来,但没必要)。

## 5. 文档地图

| 文档 | 内容 |
|---|---|
| `docs/architecture.md` | **系统完整介绍**:三层设计/数据流/设计决策与理由/数据格式 ★新同学从这里开始 |
| `docs/runbook.md` | **日常操作手册**:安装/开工 checklist/录制规范/故障速查 |
| `docs/ur5e_setup.md` | UR5e 部署与分步验收(离线→URSim→真机)★真机前必读 |
| `docs/pico_teleop_setup.md` | Pico 头显部署(PC Service/APK/pybind/组网) |
| `docs/troubleshooting.md` | 踩坑与调试记录(现象→根因→修复→教训) |
| `docs/teleop_usage.md` | 数据通路与延迟判读 |

## 6. 测试与调试

```bash
pixi run test    # 等价回归(映射数学保险丝) + UR5e 安全钳制 + gamepad,共 24 项
```

手感不对时先跑 `python scripts/net_monitor.py --host <头显IP> --no-gui` 体检
链路——**手感劣化的头号原因是链路老化,不是映射**(教训见
`docs/troubleshooting.md` #1);头显摘下会睡眠断流,距离传感器贴胶带。
