# Runbook(日常操作手册)

> 从零安装 → 每日开工 → 录制规范 → 故障速查 → 收工。
> 系统设计看 `architecture.md`;踩坑史看 `troubleshooting.md`。

## 0. 新机器安装

### 0.1 pixi 一键(核心框架 + UR5e,推荐)

```bash
curl -fsSL https://pixi.sh/install.sh | bash        # 装 pixi(一次)
git clone git@github.com:Gwss-s/teleop-system.git && cd teleop-system
pixi install                                        # python3.10 + numpy/yaml/cv2/ur_rtde
pixi run test                                       # 全部单测通过 = 安装 OK
pixi shell                                          # 进环境干活(PYTHONPATH 已配)
```

LIBERO 仿真(可选,重依赖):

```bash
pixi install -e sim                                 # + robosuite 1.4.1 / mujoco 3.1.6
pixi run -e sim install-libero                      # clone LIBERO-plus + pip -e(拉 torch,较大)
```

### 0.2 pixi 覆盖不了的(手动,一次性)

| 组件 | 怎么装 | 什么时候需要 |
|---|---|---|
| XRoboToolkit PC Service 1.0.0(.deb) | `docs/pico_teleop_setup.md` §1.1(需 sudo) | 用 Pico 手柄时 |
| `xrobotoolkit_sdk` pybind 1.0.2 | 同上 §1.2(源码构建脚本);若与 pixi 环境 ABI 不合,用 `backends/remote.py` 跨进程桥 | 用 Pico 手柄时 |
| 头显 APK + 热点 | 同上 §2/§3 | 用 Pico 手柄时 |
| URSim(docker) | `docs/ur5e_setup.md` §2 | 无真机验证 UR5e 时 |
| LIBERO 数据集/资产 | LIBERO-plus 仓库说明 | 跑仿真任务时 |

> 本机历史环境是 conda(`liberoplus_sim`,锁定清单 `docs/liberoplus_sim.freeze.txt`),
> 与 pixi 二选一即可;文档命令里的 `~/miniconda3/envs/liberoplus_sim/bin/python`
> 在 pixi 环境下等价于 `pixi shell` 后的 `python`。

## 1. 每日开工(仿真遥操)

1. **操作员准备**:头显额头/头顶佩戴(摄像头必须看得见手柄)、距离传感器贴胶带、
   app 面板 "Switch w/ A Button" 关闭、面板挪出手柄指向范围;
   **启动头显 app 时人面向机器人**——头显世界系朝向 = app 启动瞬间头的朝向,
   面向机器人可让站位偏航角(world_yaw_deg)接近 0(偏了也能标出来修,但没必要);
2. **链路体检(30 秒仪式)**:`python scripts/net_monitor.py` → 绿(<30ms)开工;
   红/丢包 → `bash scripts/reset_teleop_link.sh` + 头显整机重启 → 复查;
3. **量化验收门**(正式录制前):`python envs/libero/teleop_latency_diag.py --seconds 60`,
   stale<2% 且最长断流<1s;
4. **启动顺序**:PC Service(`runService.sh`)→ 本程序 → 头显 app 连 PC 的 IP → Send On
   (顺序错=连不上;2026-09 部署:PC 与头显同连路由器,app 里输 PC 的 WiFi IP
   如 192.168.3.22;备用热点档案 `hil-hotspot`(10.42.0.1),`nmcli connection up
   hil-hotspot` 启用——注意会断 PC 的 WiFi 上网);
5. 入口(在仓库根,`pixi shell` 或 conda 环境内):

```bash
# 纯遥操+录制(主功能)
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0 --out-dir outputs/teleop_demos
# 标定(换视角/站位时): 1-6 翻符号 +/-/[/] 增益 s 存盘
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_test.py --suite libero_goal --task-id 0 --view both
# 接管模式(需策略服务器,先建隧道: ssh -N -L 5557:localhost:5557 <gpu-host> &)
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/collect_client.py --host localhost --port 5557 --suite libero_goal --task-id 0
```

### 1.1 Xbox 手柄(免头显备选输入)

```bash
PYTHONPATH=$PWD MUJOCO_GL=egl python envs/libero/teleop_record.py \
    --suite libero_goal --task-id 0 --backend gamepad     # UR5e 入口同理
```

| 动作 | 键位 |
|---|---|
| XY 平移 / Z 升降 | 左摇杆 / RT(升) LT(降) |
| pitch/yaw / roll | 右摇杆 / 十字键 ←→ |
| 离合(按住才动,松手即停) | LB |
| 夹爪合(按住) | X |
| 保存 / 作废回合 | B / A(与 Pico 一致) |

速度/死区/键位在 `configs/gamepad.yaml`;方向标定走 `configs/teleop/gamepad_*.yaml`
(自动选择)。首插手柄看启动日志打印的型号名,非 Xbox 布局需在 models: 加表。

## 2. 每日开工(UR5e)

**首次/换场地必做**:核对 `configs/ur5e.yaml` 的 `workspace` 盒与实际单元一致
(桌面高度/围栏/人员位置)——这是最重要的一道防线。

1. 控制柜 Remote Control 模式、payload/TCP 已设、`ping <控制柜IP>` 通
   (2026-09 部署:PC 有线口 `enp132s0` = 192.168.10.1 做 DHCP 服务器,
   控制柜设 DHCP,拿到 192.168.10.18;示教器静态 IP 配置容易不生效,别用);
2. 链路体检同 §1 第 2 步(手柄链路与机器人无关,照样要体检;
   路由器方案下用 `python scripts/net_monitor.py --host <头显IP> --no-gui`);
3. **标定(首次/换站位/重启头显 app 后必做)**——引导式自动标定,机器人逐轴
   单向演示、人捏 grip 朝同方向模仿,自动解出站位偏航角 + 逐轴符号:

```bash
PYTHONPATH=$PWD python scripts/auto_calibrate_ur5e.py
```

   * x 轴手势是解偏航的基准,尽量水平移动;动作幅度平移≥10cm/旋转≥30°;
   * 产物写回 `configs/teleop/pico_ur5e.yaml`(`world_yaw_deg` 绑定 app 启动
     朝向与站位,两者任一变了就重标;启动 app 时面向机器人可让它接近 0);
   * 标完个别轴反了:直接改 yaml 里对应 `pos_sign`/`rot_sign`,或
     `--calibrate`(键 1-6)微调,不必整套重标。
4. 启动:

```bash
# 空载低速首跑(增益减半)
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --robot-host <IP> --pos-scale 0.4
# 手动微调符号/增益(1-6 翻符号,s 存 configs/teleop/pico_ur5e.yaml,保留 world_yaw_deg)
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --robot-host <IP> --calibrate
# 无真机: --dry-run(内置模拟器) / URSim(docs/ur5e_setup.md §2)
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --dry-run --backend replay --tap <raw_tap.npz> --fast --no-gui
```

5. 操作语义:grip=移动,松手=保持,trigger=夹爪,B=保存,A=作废,q/Ctrl-C=干净退出
   (自动 servoStop→stopScript);
6. 看门狗触发([WATCHDOG] 橙字)= 手柄流断:机器人已停,松手重捏恢复;频繁触发按
   `troubleshooting.md` #1/#8 治链路;**头显摘下会睡眠断流,距离传感器贴胶带**;
7. protective stop:入口会提示,示教器解锁后自动恢复,松手重捏继续。

## 3. 录制会话规范

- **裁决**:B=保存(回合无异常画面/穿模/物理爆炸);A=作废(误操作/数据污染);
  LIBERO 成功自动保存、超时自动作废;
- 录制中撞上明显位姿断流(臂无故停顿后猛跟)→ 该回合 A 作废;
- raw tap 默认开启(事后可换 yaml 重映射,`--no-raw-tap` 关);
- 产物位置:`outputs/<入口对应目录>/episode_*.npz` + `raw_tap_*.npz`,格式见
  `architecture.md` §5。

## 4. 故障速查

| 现象 | 先查 | 处置 |
|---|---|---|
| 手感差/延迟大 | `net_monitor.py` | 红→`reset_teleop_link.sh`+头显重启(**不要改映射**) |
| ping 低但输入龄高 | Service/App 层积压 | 重启头显 App→PC Service |
| 连不上手柄 | 启动顺序 | Service→程序→头显连热点→Send On |
| UR5e "RTDE data synchronization" 失败 | URSim 安全确认/版本 | `troubleshooting.md` #9/#10 |
| UR5e 看门狗频繁触发 | 链路 | #1/#8;确认头显没睡眠 |
| 方向错乱(相机扰动任务) | 标定视角 | `teleop_test.py --view wrist` 翻符号存 `_wrist.yaml` |
| 多回合后 MuJoCo "resource not found" | fd 泄漏累积 | 重启入口续采(数据不丢,`--resume-collection`) |

## 5. 收工

- 入口 q/Ctrl-C 退出(UR5e 自动 servoStop+stopScript);
- 长期不用时关 PC Service(单实例,挂着会老化,见 troubleshooting #1);
- URSim 容器:`docker stop ursim`(注意:重建容器需重做安全确认,见 ur5e_setup §2)。
