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

### 1.1 PC Service(需 sudo,**用户执行**)
.deb 已下载到 `/home/gws/sim_ws/xrobotoolkit/`:
```bash
sudo dpkg -i /home/gws/sim_ws/xrobotoolkit/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```
装到 `/opt/apps/roboticsservice/`,启动脚本:
- `runService.sh` — 纯服务进程(我们用这个)
- `run2D.sh` / `run3D.sh` — 官方演示;`runRobotDataRecorder.sh` — 录制器

> 官方约束:**单实例**(同一时间只能跑一个);Qt6 打包,勿从源码编译(那才需要 Qt 6.6.3)。

### 1.2 Teleop 样例 + pybind(无需 sudo,已自动化)
```bash
# 仓库已克隆:/home/gws/sim_ws/xrobotoolkit/XRoboToolkit-Teleop-Sample-Python
conda activate xr-robotics          # python 3.10(pybind 官方 README 的版本)
bash setup_conda.sh --install       # 自动:libstdcxx-ng 修 ABI → 克隆并构建 pybind(内含 PXREARobotSDK C++ 构建)→ R5 包 → pip -e .
```
验收:`python -c "import xrobotoolkit_sdk"` 不报错。

## 2. 头显侧(S2,**需要你 + Pico 4 Ultra**)

1. 头显开发者模式 + USB 调试(**"开发者"入口默认隐藏**):
   `设置 → 通用 → 关于本机 → 光标对准"软件版本号"连点约 8 次` → 左侧导航栏底部出现"开发者" → 进入并打开 **USB 调试**
2. PC 装 adb:`sudo apt install android-tools-adb`
3. USB 连接头显(**必须用支持数据的 USB3 线,自带充电线常识别不到**),安装 APK(已下载好):
   ```bash
   adb devices -l        # 头显内接受调试授权
   adb install -g /home/gws/sim_ws/xrobotoolkit/XRoboToolkit-PICO-1.1.1.apk
   ```
   > 装 APK 走 USB,**不需要任何网络**。
4. **局域网方案(已落地:双网卡并存;公司网 ZPHZ 需域名认证且可能有客户端隔离,Pico 不连它)**:
   - 主板 AX211(`wlo1`,曾因无天线被 blacklist,已解禁:黑名单文件备份在 `~/disable-internal-wifi.conf.bak`)开热点:
     SSID `hil-teleop` / 密码 `hilvla2026` / 2.4G ch6 / PC 热点 IP **10.42.0.1**(NM shared 模式,自带 DHCP+NAT,头显可透过它上外网)
   - USB 网卡(8812au)继续连 ZPHZ(公司网/服务器/互联网),两网并存
   - 无天线对 1-3 米热点可行(路损 ~48dB + 无天线 ~20dB 惩罚,仍有 ~30dB 富余;位姿流仅 ~40kbps)
   - 热点起停:`nmcli connection up hil-hotspot` / `nmcli connection down hil-hotspot`;wlo1 的 ZPHZ 档案已设 autoconnect=no
   - 头显在 app 里连 PC 时输入 **10.42.0.1**(不是 ZPHZ 的 10.253.x)
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

## 4. S3 验收命令(官方 MuJoCo demo)

```bash
conda activate xr-robotics
cd /home/gws/sim_ws/xrobotoolkit/XRoboToolkit-Teleop-Sample-Python
python scripts/simulation/teleop_dual_ur5e_mujoco.py
```
**按住 grip 才动**(松手即停,官方安全设计);扳机=夹爪;B 键=开始/停止录制(.pkl,可转 LeRobot)。

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
| conda ABI | pybind 构建前必须 `conda install libstdcxx-ng`(setup 脚本已包含) |
| setup 脚本会删同名环境 | `--conda` 先 `conda remove --all`;它取"当前 python3"定版本,在 conda base 下会拿错(我们已手动固定 3.10) |
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
