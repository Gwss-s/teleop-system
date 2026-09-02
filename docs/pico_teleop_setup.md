# Pico 4 Ultra 头显部署手册(XRoboToolkit)

> 一次性部署:装完之后日常只需按 §3 的固定顺序启动。每一步都取自
> XR-Robotics 官方文档(出处见文末)。普通消费版 Pico 4 Ultra 即可
> (企业版仅摄像头透视需要,遥操作用不到);头显系统要求 User OS > 5.12。
> Linux 为主线;Windows 看 §5。

## 0. 链路架构

```
Pico 4 Ultra(XRoboToolkit app,APK)
   │  同一 WiFi/局域网,位姿流 90Hz
   ▼
PC Service(官方后台服务)                              ← 先启动,单实例
   │  gRPC → pybind
   ▼
Python: import xrobotoolkit_sdk                       ← 6-DoF 位姿 + grip/trigger/按键
   ▼
teleop_system 三层栈(backends/pico_ultra4.py) → URSim 仿真 / UR5e 真机
```

## 1. PC 侧安装(Linux)

### 1.1 PC Service(需 sudo)

从 GitHub release 下载对应系统版本(有 ubuntu 22.04 与 24.04 两种 .deb):

```bash
mkdir -p ~/xrobotoolkit && cd ~/xrobotoolkit
curl -fLO https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
```

装到 `/opt/apps/roboticsservice/`,日常用 `runService.sh` 启动(纯服务进程;
`run2D.sh`/`run3D.sh` 是官方演示,用不到)。
官方约束:**单实例**(同一时间只能跑一个);Qt6 打包,不要从源码编译。

### 1.2 pybind SDK(构建进 pixi 环境)

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
若与运行环境 ABI 不合,退路是跨进程桥(§5 方案 B 同款,本机也可用)。

## 2. 头显侧

1. 开发者模式 + USB 调试(**"开发者"入口默认隐藏**):
   `设置 → 通用 → 关于本机 → 光标对准"软件版本号"连点约 8 次` →
   左侧导航栏底部出现"开发者" → 进入并打开 **USB 调试**;
2. PC 装 adb:`sudo apt install android-tools-adb`;
3. USB 连接头显装 APK(**必须用支持数据的线,纯充电线识别不到**——`lsusb` 里
   看得到 "Pico PICO 4 Ultra" 才算通;装 APK 走 USB,不需要网络):

   ```bash
   cd ~/xrobotoolkit
   curl -fLO https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
   adb devices -l        # 在头显内接受调试授权
   adb install -g XRoboToolkit-PICO-1.1.1.apk
   ```

4. **组网(两种方案)**:
   - **首选:PC 与头显同连一台自有路由器**。校园/企业 WLAN 有 AP 隔离
     (终端互 ping 不通),Pico 不要直连它;自有路由器 LAN 内互通,PC 还能上网。
     头显 app 里连 PC 的 WiFi IP(`ip addr` 查);
   - **备用:PC 开热点**(无路由器时):
     `nmcli connection add type wifi ifname <无线网卡> con-name hil-hotspot
     autoconnect no ssid hil-teleop mode ap ipv4.method shared
     wifi-sec.key-mgmt wpa-psk wifi-sec.psk <密码>`,头显连该热点后 app 里输
     **10.42.0.1**。注意单网卡机器开热点会断 PC 的 WiFi 上网;
   - 位姿流仅 ~40kbps;链路质量验收:
     `python scripts/net_monitor.py --host <头显IP> --no-gui`(绿 <30ms)。
5. 首次运行若报 "No entitlement info":把头显连一次公网再开 app(官方 Q&A)。

## 3. 日常启动顺序(每次固定,顺序错会连不上)

```bash
# ① PC:先起服务(保持运行)
/opt/apps/roboticsservice/runService.sh
# ② PC:起消费端程序(探针 / 遥操入口)
# ③ 头显:开 XRoboToolkit app → 弹出连接提示 → 扳机点选 PC 的 IP
#    (或 Network → Enter 手输 IP)→ 主面板显示 WORKING
# ④ 头显面板:开 Head/Controller tracking,"Send" 拨 On
#    (把 "Switch w/ A Button" 关掉:A 键要留给"作废回合",不许兼职暂停发送)
```

**链路验收**:`python scripts/pico_probe.py`——手柄动起来、grip 值在变即通。

## 4. Python API 速查(`xrobotoolkit_sdk`)

```python
import xrobotoolkit_sdk as xrt
xrt.init()                                  # 先起 PC Service 再 init;连上前读到 0
p = xrt.get_right_controller_pose()         # [x,y,z,qx,qy,qz,qw],原点=app 启动时头位置
g = xrt.get_right_grip()                    # 模拟量 0.0~1.0(不是布尔!阈值 0.9 判接管)
t = xrt.get_right_trigger()                 # 夹爪
a = xrt.get_A_button(); b = xrt.get_B_button()   # 右手柄;左手柄为 X/Y
ts = xrt.get_time_stamp_ns()
xrt.close()
```

## 5. 🪟 Windows 部署(两条路线任选)

- **方案 A(原生)**:官方提供 Windows 组件——PC Service 用 release 里的
  `XRoboToolkit-PC-Service.win.zip`,pybind 用同仓库的 `setup_windows.bat`
  构建,之后 §2-§4 与 Linux 一致(adb 用
  [platform-tools](https://developer.android.com/tools/releases/platform-tools);
  热点方案不适用,用路由器组网)。
- **方案 B(跨机桥接,构建失败时的退路)**:让实验室 Linux 机跑设备发布端:
  ```bash
  python -m teleop_system.backends.remote --serve --backend pico --port 5570
  ```
  你的 Windows 机与它同一局域网,入口加 `--backend remote --remote-host <Linux机IP>`
  即可正常遥操(位姿流经 TCP 转发,数据格式与本地完全一致)。

## 6. 已知坑速查

| 坑 | 说明 |
|---|---|
| 顺序 | PC Service 必须先于头显 app 启动 |
| 单实例 | PC Service 只能跑一个 |
| grip 是模拟量 | 判接管用阈值 0.9,要捏到底 |
| 头显睡眠断流 | 摘下即睡眠 → 距离传感器贴胶带 |
| 佩戴位置 | 架在额头/头顶时摄像头必须看得见手柄(6DoF 追踪依赖视野) |
| ABI | conda 环境构建 pybind 前需 `conda install libstdcxx-ng`;pixi 环境直接可用 |
| 坐标系 | 头显位姿右手系(X右/Y上/Z内);框架已在 L1 统一变换,写新后端时才需要关心 |
| 端口 | 位姿通路端口未文档化;连不上先查同网段与防火墙 |

## 来源(全部官方)

- 快速开始(安装/运行顺序/OS 要求):github.com/XR-Robotics(org profile README)
- PC Service(.deb / win.zip / 源码):github.com/XR-Robotics/XRoboToolkit-PC-Service release v1.0.0
- Python API / 构建(含 setup_windows.bat):github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind v1.0.2
- APK v1.1.1:github.com/XR-Robotics/XRoboToolkit-Unity-Client releases
