# Pico 4 Ultra 遥操作接入手册(XRoboToolkit)

> 目标:用 Pico 4 Ultra 实现 **仿真/真机遥操作(可选:策略接管)**。
> 本手册每一步都取自官方文档(XR-Robotics 各仓库 README / org 快速开始),提取日期 2026-08-25。
> **普通消费版 Pico 4 Ultra 够用**——官方明确:企业版仅 VST 摄像头透视需要;位姿/按键流不需要。头显系统要求:**User OS > 5.12**。

## 0. 架构与部署阶段

```
Pico 4 Ultra(XRoboToolkit app,APK)
   │  同一 WiFi/局域网,位姿通道 90Hz
   ▼
PC Service(/opt/apps/roboticsservice,.deb 安装)     ← 先启动,单实例
   │  gRPC → pybind
   ▼
Python: import xrobotoolkit_sdk as xrt               ← xrt.init()
   │  6-DoF 位姿 + grip/trigger/按键
   ▼
teleop_system 三层栈(backends/pico_ultra4.py) → LIBERO 仿真 / UR5e 真机
```

| 阶段 | 内容 | 验收 | 状态 |
|---|---|---|---|
| S0 | 官方文档提取 → 本手册 | 每步有出处 | ✅ |
| S1 | PC 侧:PC Service .deb + teleop 样例环境(含 pybind 自动构建) | `import xrobotoolkit_sdk` ✓;service 能启动 | ✅ |
| S2 | 头显:装 APK、连 PC,Python 打印手柄位姿 | 终端实时打印 6-DoF + grip | ✅ |
| S3 | 官方 MuJoCo demo(dual UR5e) | 手柄动 → 仿真臂动 = **能遥操** | ✅ |
| S4 | Pico → LIBERO OSC delta-EE(7 维)映射 | Pico 遥操 LIBERO Panda 抓物 | ✅(`envs/libero/teleop_test.py`) |
| S5 | 三层栈后端(`teleop_system/backends/pico_ultra4.py`) | grip=engaged 边沿正确 | ✅(等价回归测试钉死) |
| S6 | LIBERO 接管客户端(自主流 + grip 接管/交还) | 策略跑动中随抓随还 = **能接管** | ✅(`envs/libero/collect_client.py`) |

## 1. PC 侧安装(S1)

### 1.1 PC Service(需 sudo)
从 GitHub release 下载对应系统版本(有 ubuntu 22.04 与 24.04 两种 .deb):
```bash
mkdir -p ~/xrobotoolkit && cd ~/xrobotoolkit
curl -fLO https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
```
装到 `/opt/apps/roboticsservice/`,启动脚本:
- `runService.sh` — 纯服务进程(我们用这个)
- `run2D.sh` / `run3D.sh` — 官方演示;`runRobotDataRecorder.sh` — 录制器

> 官方约束:**单实例**(同一时间只能跑一个);Qt6 打包,勿从源码编译(那才需要 Qt 6.6.3)。

### 1.2 pybind SDK(构建进 pixi 环境,已验证流程)
```bash
cd ~/xrobotoolkit
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
# 构建 C++ SDK 底层库(系统 gcc/cmake 即可):
mkdir -p tmp && cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK && bash build.sh && cd ../../../..
mkdir -p lib include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
# 装进本仓库的 pixi 环境(在 teleop-system 目录 pixi shell 后执行):
pip install pybind11
export CMAKE_PREFIX_PATH=$(python -m pybind11 --cmakedir)
python setup.py install
```
验收:`python -c "import xrobotoolkit_sdk"` 不报错。
若与消费环境 ABI 不合,退路是 `backends/remote.py` 跨进程桥(TCP 5570)。

## 2. 头显侧(S2,**需要你 + Pico 4 Ultra**)

1. 头显开发者模式 + USB 调试(**"开发者"入口默认隐藏**):
   `设置 → 通用 → 关于本机 → 光标对准"软件版本号"连点约 8 次` → 左侧导航栏底部出现"开发者" → 进入并打开 **USB 调试**
2. PC 装 adb:`sudo apt install android-tools-adb`
3. USB 连接头显(**必须用支持数据的线,纯充电线识别不到——lsusb 里看得到
   "Pico PICO 4 Ultra" 才算通**),安装 APK:
   ```bash
   cd ~/xrobotoolkit
   curl -fLO https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
   adb devices -l        # 头显内接受调试授权
   adb install -g XRoboToolkit-PICO-1.1.1.apk
   ```
   > 装 APK 走 USB,**不需要任何网络**。
4. **组网(两种方案)**:
   - **首选:PC 与头显同连一台自有路由器**(2026-09 已落地)。企业/校园 WLAN 有
     AP 隔离(终端互 ping 不通)且每设备要单独认证,Pico 不要直连它;自有路由器
     LAN 内互通,PC 保持上网。头显 app 里连 PC 的 WiFi IP(`ip addr` 查)。
   - **备用:PC 开热点**(无路由器时):`nmcli connection add type wifi ifname
     <无线网卡> con-name hil-hotspot autoconnect no ssid hil-teleop mode ap
     ipv4.method shared wifi-sec.key-mgmt wpa-psk wifi-sec.psk <密码>`;
     头显连该热点后 app 里输 **10.42.0.1**。注意单网卡机器开热点会断 PC 的
     WiFi 上网。
   - 位姿流仅 ~40kbps,链路质量验收看 `scripts/net_monitor.py --host <头显IP>
     --no-gui`(绿 <30ms)。
5. 首次运行若报 "No entitlement info":把头显连一次公网再开 app(官方 Q&A)

## 3. 运行顺序(每次固定,顺序错会连不上)

```bash
# ① PC:先起服务(保持运行)
/opt/apps/roboticsservice/runService.sh
# ② PC:起消费端(读位姿脚本 / MuJoCo demo / 我们的客户端)
# ③ 头显:开 XRoboToolkit app → 弹出连接提示 → 扳机点选 PC 的 IP
#    (或 Network → Enter 手输 IP)→ 主面板显示 WORKING
# ④ 头显面板:开 Head/Controller tracking,"Send" 拨 On
#    ("Switch w/ A Button":右手柄 A 键暂停/恢复发送)
```
官方 Q&A:Linux 连不上 → 确认同一 WiFi;**先跑 PC 端程序再开头显 app**。

## 4. S3 验收命令(官方 MuJoCo demo,可选)

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git
cd XRoboToolkit-Teleop-Sample-Python   # 按其 README 装依赖
python scripts/simulation/teleop_dual_ur5e_mujoco.py
```
**按住 grip 才动**(松手即停,官方安全设计);扳机=夹爪;B 键=开始/停止录制(.pkl,可转 LeRobot)。
> 更快的验收方式:直接跑本仓库的探针 `python scripts/pico_probe.py`,
> 手柄动起来、grip 值变化即链路通。

## 5. Python API 速查(pybind 模块 `xrobotoolkit_sdk`)

```python
import xrobotoolkit_sdk as xrt
xrt.init()                                  # 先起 PC Service,再 init;数据异步到达,连上前读到 0
p = xrt.get_right_controller_pose()         # [x,y,z,qx,qy,qz,qw],原点=app 启动时头位置
g = xrt.get_right_grip()                    # 模拟量 0.0~1.0(不是布尔!官方样例用 >0.9 判接管)
t = xrt.get_right_trigger()                 # 夹爪
a = xrt.get_A_button(); b = xrt.get_B_button()   # 右手柄;左手柄为 X/Y
ts = xrt.get_time_stamp_ns()
xrt.close()
```
接管语义参考实现:官方 `xrobotoolkit_teleop/common/base_teleop_controller.py` —— `active = grip > 0.9`;激活瞬间快照参考位姿,之后 `delta = (controller_xyz - ref_xyz) * scale`(clutch 相对模式)→ 我们 S5 的 `teleop/pico.py` 按同一模式实现。

## 6. 已知坑(全部来自官方文档/Q&A)

| 坑 | 说明 |
|---|---|
| 顺序 | PC Service 必须先于头显 app 启动 |
| 单实例 | PC Service 只能跑一个 |
| grip 是模拟量 | 判接管用阈值(官方 0.9),不要当布尔 |
| ABI | conda 环境下构建 pybind 前需 `conda install libstdcxx-ng`;pixi 环境实测直接可用(§1.2) |
| 坐标系 | 头显位姿右手系(X右/Y上/Z内);手柄文档标注为左手系同向 → 映射到 MuJoCo 时注意 |
| VST 摄像头 | 需企业版+审批;**仿真遥操作用不到** |
| 端口 | 位姿通路端口未文档化(仅视频 13579/音频 13580);连不上先查同网段与防火墙 |

## 来源
- 快速开始(安装/运行顺序/OS 要求):github.com/XR-Robotics/.github(org profile README)
- PC Service .deb / 源码构建 / JSON 数据格式:github.com/XR-Robotics/XRoboToolkit-PC-Service(release v1.0.0)
- Python API 全表 / 构建:github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind(v1.0.2)
- 遥操作样例 / grip 门控 / Placo QP-IK / 录制:github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python
- APK(v1.1.1):github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases
- 第三方端到端实操(2026-03):akshayparkhi.net "XR-Robotics with Pico 4 Ultra"
</content>
