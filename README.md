# teleop-system — 机器人遥操作教学项目

> 机器人学课程配套项目。**目标:用 Xbox 手柄和 Pico 4 Ultra VR 头显遥操作
> UR5e 机械臂**(先仿真,后真机),完成回合数据录制。
> Windows 与 Linux 都可以完成全部实验;不需要任何编程环境基础——
> 从"怎么打开终端"开始,每一步文档里都写了。

---

## 怎么开始:按顺序做下面的实验

**本页不含任何命令。** 每个实验的全部操作步骤只写在"跟着做"那一份文档里,
照着做即可;做的过程中卡住,查 `docs/faq.md`(现象→原因→怎么办)。

| 顺序 | 实验 | 需要什么 | 跟着做 | 预计用时 |
|---|---|---|---|---|
| **0** | 环境准备(装 pixi/取代码/装 Docker) | 一台电脑 | `docs/setup.md` 从头到尾 | 40 分钟 |
| **1** | 零硬件:重放数据流,看安全钳制工作 | 无 | `docs/ur5e_setup.md` 实验 1 | 5 分钟 |
| **2** | **手柄遥操仿真 UR5e**(第一次亲手遥操) | Xbox 手柄 | `docs/ur5e_setup.md` 实验 2 | 30 分钟 |
| **3** | **VR 头显遥操仿真 UR5e** | Pico 4 Ultra | 先 `docs/pico_teleop_setup.md`(部署,一次性),再 `docs/ur5e_setup.md` 实验 3 | 1 小时 |
| **4** | **真机 UR5e**(必须有指导教师在场) | 真机 + 教师 | `docs/ur5e_setup.md` 实验 4 | 1 小时 |

## 你将学到什么

| 主题 | 对应模块 |
|---|---|
| **分层解耦设计**:N 设备 × M 机器人 = N+M 份代码,而不是 N×M | `teleop_system/types.py` 三个接口数据类型 |
| **SO(3) 旋转数学**:四元数/旋转矩阵/轴角,坐标系变换(XR Y-up → 世界 Z-up) | `teleop_system/geometry.py` |
| **离合式增量映射**:锚定、逐拍增量、迟滞防抖、符号/增益标定 | `teleop_system/mapping/ee_delta.py` |
| **坐标系对齐标定**:站位偏航角求解、Kabsch 轨迹对齐 | `scripts/auto_calibrate_ur5e.py`、`scripts/drag_calibrate_ur5e.py` |
| **真机安全设计**:步长限幅、工作空间盒、目标-实测偏差钳位、看门狗 | `envs/ur5e/teleop_adapter.py` |
| **实时控制**:servoL 伺服语义、控制频率与插值、延迟诊断 | `envs/ur5e/teleop_record.py`、`scripts/net_monitor.py` |
| **数据采集**:回合录制、裁决(保存/作废)、raw tap 事后重映射 | `envs/ur5e/teleop_record.py` |

## 三层架构(本项目的核心思想)

```
输入设备 ──L1 backends──> TeleopState ──L2 mapping──> ControlIntent ──L3 adapter──> 机器人动作
 (Pico/手柄/重放)        (每手位姿+按键)   (离合/增量/标定)  (物理量+夹爪)      (servoL)
```

**换输入设备只写 L1**(一个类实现 `read() -> TeleopState`),**换机器人只写 L3**。
接口由 `teleop_system/types.py` 的三个 dataclass 钉死——读代码从这个文件开始。
完整设计与理由:`docs/architecture.md`。

## 文档地图

| 文档 | 什么时候读 |
|---|---|
| `docs/setup.md` | **实验 0**:环境准备(从打开终端教起,双系统) |
| `docs/ur5e_setup.md` | **实验 1-4 的主指南**,所有实验命令都在这里 |
| `docs/pico_teleop_setup.md` | 实验 3 之前:部署 VR 头显链路(一次性) |
| `docs/faq.md` | 碰到任何问题时(17 条:现象→原因→怎么办) |
| `docs/architecture.md` | 想弄懂系统怎么设计的(读代码前) |
| `configs/README.md` | 想改配置时:三类配置文件各管什么 |
