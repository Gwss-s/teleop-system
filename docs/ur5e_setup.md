# UR5e 真机部署与分步验收

> 对应代码:`envs/ur5e/`(adapter/入口/夹爪)+ `configs/ur5e.yaml`(机器人/安全)
> + `configs/teleop/pico_ur5e.yaml`(映射标定)。
> 官方依据:ur_rtde 文档(sdurobotics.gitlab.io/ur_rtde)与 XRoboToolkit 官方
> UR5e 真机遥操实现(XR-Robotics/XRoboToolkit-Teleop-Sample-Python)。

## 0. 控制结构(为什么和 LIBERO 版不同)

LIBERO 版 L3 输出**逐拍 OSC 增量**(超限部分被丢弃);UR5e 版 L3 维护一个
**持久 servoL 目标**:增量积分进目标,servoL 朝目标伺服,**误差自然保留**
——这正是官方 XRoboToolkit 真机的锚定语义。防"目标甩飞"(历史事故教训)靠三层钳制:

1. **每拍步长限幅**(手甩动/XR 跟踪跳变保护): `max_lin_step`/`max_rot_step`;
2. **工作空间盒**: 目标位置硬钳进 `workspace` 盒;
3. **目标-实测偏差钳位**: 目标最多领先实测 TCP `max_target_offset`(米)/
   `max_target_rot_offset`(弧度)。

频率架构:主循环 20-50Hz(默认 25Hz)算目标,servoL 靠 `lookahead_time`(0.1)/
`gain`(300,官方真机值)在机器人侧 500Hz 插值。**不要把主循环提到 500Hz。**

离合语义(真机复查项):捏 grip=移动;**松手=目标钉在当前实测位姿并主动保持**
(不回零、不追旧目标);看门狗/保护停后必须**松手重捏**才恢复人控。

## 1. 离线验收(零硬件,改完代码必跑)

```bash
PYTHONPATH=$PWD python tests/test_ur5e_adapter.py     # 7 项安全钳制单测
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py \
    --dry-run --backend replay --tap <raw_tap_XXX.npz> --fast --no-gui
# 通过标准: 连续人控段最大单拍步长 <= max_lin_step;最大目标-实测偏差 <= max_target_offset
```

`--dry-run` 用内置一阶伺服模拟器代替真机;`--backend replay` 重放已录手柄流
(LIBERO 采集时留下的 raw_tap npz 直接可用,同一个 L1/L2 栈)。

## 2. URSim 仿真验收(真实 RTDE 接口,无真机;2026-08-31 已走通)

```bash
docker network create --subnet=192.168.56.0/24 ursim_net 2>/dev/null || true
# 用 5.12.x(与 ur_rtde 1.6.5 兼容性经过验证;最新 5.26 会让 ur_rtde 版本解析失败);
# 不加 --rm,安全确认状态存在容器里,stop/start 不丢
docker run -d --name ursim --net ursim_net --ip 192.168.56.101 \
    universalrobots/ursim_e-series:5.12.6      # 默认型号即 UR5e
```

**首次必做——GUI 确认安全配置**(不做的话 RTDEControl 报
"Failed to start RTDE data synchronization";用原始 RTDE 探针能看到真实原因:
"SafetySetup has not been confirmed yet")。浏览器开
`http://192.168.56.101:6080/vnc.html`,在 PolyScope 里:
1. 汉堡菜单 → Settings → Password → Safety:设一个安全密码(URSim 出厂未设,不设无法解锁);
2. Installation → Safety → Robot Limits:输密码 → Unlock → 右下 Apply → "Apply and restart";
3. 重启后出现 "Confirmation of applied Safety Configuration" 屏 → 点 **Confirm Safety Configuration**。

之后上电+松刹车可完全脚本化(免 VNC):

```bash
python - <<'EOF'
import dashboard_client, time
d = dashboard_client.DashboardClient("192.168.56.101"); d.connect()
d.powerOn(); time.sleep(5); d.brakeRelease(); time.sleep(8)
print(d.robotmode())          # 期望 Robotmode: RUNNING
EOF
# 跑全链路(重放已录手柄流,或 --backend pico 用真手柄):
PYTHONPATH=$PWD python envs/ur5e/teleop_record.py \
    --robot-host 192.168.56.101 --backend replay --tap <raw_tap.npz> --no-gui
```

已验收记录(2026-08-31):真实 tap(1419 拍)驱动 URSim UR5e,机器人按流移动约 8cm+大幅转姿,
全程无 protective stop,终态在工作空间盒内;tap 中两段录制时代的陈旧帧断流(>0.3s)
被看门狗正确捕获(servoStop+强制重捏),即安全层在真实劣化数据上按设计工作。

## 3. 真机 bring-up 清单(按顺序)

1. **前置**:示教器开 Remote Control 模式;设置 payload/TCP;外部主机与控制柜同网段
   (`ping <控制柜IP>`);`pip install ur_rtde`(cp3.6-3.12 有轮子)。
2. **⚠ 核对 `configs/ur5e.yaml` 的 `workspace` 盒**——默认值是示例,必须按你的
   实际工作单元(桌面高度/围栏/人员位置)重新给定。入口启动时若当前 TCP 在盒外会
   大字警告。
3. **空载低速首跑**:增益减半 + 步长上限减半:
   ```bash
   PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --robot-host <IP> \
       --pos-scale 0.4   # yaml 默认 0.8(官方真机值)的一半
   ```
   自由空间画方框,验证:松手即停、B/A 裁决、Ctrl-C 干净退出(servoStop+stopScript)。
4. **现场标定**(操作员站位与仿真不同,几乎必做)——首选引导式自动标定:
   ```bash
   PYTHONPATH=$PWD python scripts/auto_calibrate_ur5e.py
   ```
   机器人逐轴**单向**演示(平移 10cm/旋转 46°,停在终点不回位),你捏住 grip 朝
   同方向模仿(平移≥10cm/旋转≥30°),脚本自动解出**站位偏航角 world_yaw_deg**
   (头显世界系朝向 = app 启动瞬间头的朝向,与基座水平轴不对齐时纯符号标定无解
   ——2026-09 现场实测偏航可达 119°)+ 逐轴符号,写回 `configs/teleop/pico_ur5e.yaml`。
   x 轴手势是解偏航的基准,尽量水平移动。**换站位或重启头显 app 后必须重标**;
   启动 app 时面向机器人可让偏航接近 0。个别轴不对时用手动方式微调:
   ```bash
   PYTHONPATH=$PWD python envs/ur5e/teleop_record.py --robot-host <IP> --calibrate
   ```
   键 `1/2/3` 翻 pos 符号、`4/5/6` 翻 rot 符号、`+/-`/`[/]` 调增益、`s` 存回
   (保留 world_yaw_deg)。目标:"手往哪动,末端就往哪动"。
5. **夹爪**(Robotiq,经控制柜 63352 端口,不打断伺服):`configs/ur5e.yaml` 改
   `gripper.type: robotiq` → trigger 模拟量=开合。抓放泡沫块验收。
6. **录制回路**:npz 每回合一个文件(`mode/t_wall/tcp_actual/tcp_target/q/gripper[/cam]`),
   B=保存 A=作废;`--camera <N>` 附带 /dev/videoN 画面(320×240)。

## 4. 安全行为一览(全部默认开启)

| 触发 | 行为 | 恢复 |
|---|---|---|
| 手甩动/跟踪跳变 | 每拍步长限幅(方向保持) | 自动 |
| 目标出盒 | 钳到盒边 | 自动 |
| 机器人跟不上(卡住/限速) | 目标最多领先实测 10cm/0.5rad | 自动 |
| 手柄流断/冻结/输入龄超 `watchdog_timeout` | servoStop + 强制脱开 | 流恢复后**松手重捏** |
| protective/emergency stop | servoStop + 等待解锁 | 示教器解锁 → 自动 reuploadScript → **松手重捏** |
| Ctrl-C / 异常退出 | finally: servoStop → stopScript → 断开 | — |

已知限制(下次会话 backlog):protective stop 频发代码 163 时 UR 要求等 ≥5s 再解锁;
Dashboard 自动解锁(`unlockProtectiveStop`)未接入入口,目前走示教器手动解锁。

## 5. 与 LIBERO 版的参数差异(为什么不能照搬)

| 参数 | LIBERO 仿真 | UR5e 真机 | 原因 |
|---|---|---|---|
| pos_scale | 4.0 | 0.8 | 4.0 是"增量+OSC限幅丢弃"模式的补偿值;持久目标无丢弃,官方真机 0.8 |
| rot_scale | 2.0 | 1.0 | 官方语义:旋转从不放大 |
| pos_sign/rot_sign | 已标定(-1,1,1)/(1,-1,-1) | 全 1 待标定 | 符号取决于操作员站位/参考视角,现场 `--calibrate` |
| 动作空间 | OSC [-1,1]^7 归一化 | 米/弧度度量 servoL 目标 | L2 输出本来就是度量单位,真机不需要归一化 |
