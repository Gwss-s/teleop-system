# Pico 4 Ultra 头显部署手册(一次性)

> 装好之后,日常使用只需 §4 的固定启动顺序。命令框都标了在哪执行;
> 🪟 = 只在 Windows 做,🐧 = 只在 Linux 做,没有标记 = 两个系统都一样。
> 普通消费版 Pico 4 Ultra 即可,头显系统要求 User OS 5.12 以上。
> 部署分三块:**PC 侧(§1)→ 头显侧(§2)→ 组网(§3)**。

## 0. 链路架构(装的是什么)

```
Pico 4 Ultra(装官方 XRoboToolkit app)
   │  同一 WiFi,手柄位姿流 90Hz
   ▼
PC Service(官方后台服务,PC 上运行)            ← 每次先启动,只能开一个
   ▼
xrobotoolkit_sdk(Python 模块,需构建安装)      ← 项目代码从这里读手柄数据
   ▼
本项目三层栈 → URSim 仿真 / UR5e 真机
```

## 1. PC 侧安装

### 🐧 Linux

**装 PC Service**(需要管理员权限):

```bash
mkdir -p ~/xrobotoolkit && cd ~/xrobotoolkit    # 建目录并进入,存放下载的安装包
# 下载官方 PC Service 安装包(Ubuntu 24.04 版;22.04 的机器把文件名里 24.04 改成 22.04)
curl -fLO https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb    # 安装(会装到 /opt/apps/roboticsservice/)
```

以后用 `/opt/apps/roboticsservice/runService.sh` 启动服务。
注意:同一时间只能运行一个 PC Service。

**构建 xrobotoolkit_sdk**(逐行执行;报错见 §6):

```bash
cd ~/xrobotoolkit
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git    # 下载 Python 绑定源码
cd XRoboToolkit-PC-Service-Pybind
mkdir -p tmp && cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git           # 下载底层 SDK 源码
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK && bash build.sh && cd ../../../..    # 编译底层 C++ 库
mkdir -p lib include    # 建放编译产物的目录
# 把编译产物(头文件与 .so)拷到绑定源码期望的位置:
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
```

再装进本项目环境——**另开一个终端,`cd` 进 teleop-system 目录,`pixi shell`**,然后:

```bash
pip install pybind11                                       # 装构建工具
export CMAKE_PREFIX_PATH=$(python -m pybind11 --cmakedir)  # 告诉构建系统去哪找 pybind11
cd ~/xrobotoolkit/XRoboToolkit-PC-Service-Pybind && python setup.py install    # 编译并安装 Python 模块
```

验收:

```bash
python -c "import xrobotoolkit_sdk"    # 不报错即安装成功
```

### 🪟 Windows(两条路线任选)

**路线 A(原生)**:
1. 浏览器打开 `https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases`,
   下载 v1.0.0 里的 `XRoboToolkit-PC-Service.win.zip`,解压到 `C:\lab\xrobotoolkit\`,
   双击运行里面的服务程序;
2. 构建 SDK:下载 `https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind`,
   在 teleop-system 的 pixi shell 里按其 README 运行 `setup_windows.bat`;
3. 验收(pixi shell 里):`python -c "import xrobotoolkit_sdk"` 不报错。

**路线 B(跨机桥,不想在 Windows 上构建 SDK 时用)**:头显链路挂在实验室 Linux 机
上,你的 Windows 电脑只负责收数据——

在实验室 Linux 机上(已按上面 🐧 部署好):

```bash
# 把头显数据通过网络发布出去,供其它电脑接收
python -m teleop_system.backends.remote --serve --backend pico --port 5570
```

你的 Windows 电脑与它连**同一网络**,之后所有遥操命令加
`--backend remote --remote-host <Linux机IP>` 即可,效果与本地完全一致。

## 2. 头显侧(装 app)

1. **打开开发者模式**(入口是隐藏的):头显里 `设置 → 通用 → 关于本机 → 把光标对准
   "软件版本号"用扳机连点约 8 次` → 左侧导航栏底部出现"开发者" → 进去打开 **USB 调试**;
2. **PC 上装 adb 工具**(adb 是给头显传文件的工具):
   - 🐧 `sudo apt install android-tools-adb`
   - 🪟 浏览器下载 `https://developer.android.com/tools/releases/platform-tools`
     (SDK Platform-Tools for Windows),解压到 `C:\lab\platform-tools\`;在这个
     文件夹的地址栏输入 `powershell` 回车,得到一个"就在这个目录里"的终端,下面的
     adb 命令在这里执行;
3. **下载 app 安装包**(APK 文件):
   - 🐧 终端执行:
     ```bash
     # 下载头显端 app 安装包到当前目录
     curl -fLO https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
     ```
   - 🪟 浏览器打开上面那个网址直接下载,把文件放进 `C:\lab\platform-tools\`(和 adb 同目录);
4. **用数据线**连接头显和 PC(纯充电线识别不到),安装:

```bash
adb devices -l                                 # 列出已连接设备;此时戴上头显,在弹窗里点"允许 USB 调试"
adb install -g XRoboToolkit-PICO-1.1.1.apk     # 把 app 安装到头显
```

**预期**:`adb install` 最后打印 `Success`。
首次打开 app 若报 "No entitlement info":让头显连一次公网 WiFi 再开(官方说明)。

## 3. 组网(PC 和头显连同一个 WiFi)

**PC 和头显连接同一个实验室 WiFi 即可**:

> WiFi 名称:`HUAWEI-6G`
> WiFi 密码:`【待填:实验室 WiFi 密码】`

然后查 PC 的 IP(记下来,头显 app 里要输它来连 PC):

- 🐧 终端输 `ip addr`,找无线网卡下 `inet 192.168.x.x` 的那串数字;
- 🪟 PowerShell 输 `ipconfig`,找"无线局域网适配器 WLAN"下的"IPv4 地址"。

**链路质量验收**(在 pixi shell 里;头显 IP 在路由器管理页或头显 WiFi 详情里看):

```bash
python scripts/net_monitor.py --host <头显IP> --no-gui    # 持续测 PC 到头显的延迟;个位数毫秒(<30ms)为达标,Ctrl+C 退出
```

注意:校园网/公司 WiFi 通常有"AP 隔离"(设备之间互相 ping 不通),连得上也用不了
——所以要用实验室自己的路由器。

## 4. 日常启动顺序(每次都按这个来,顺序错就连不上)

```
① PC:启动 PC Service
     🐧 终端执行 /opt/apps/roboticsservice/runService.sh(保持这个终端开着)
     🪟 双击运行解压目录里的 PC Service 程序
② PC:启动消费端程序(下面的探针,或实验 3 的遥操命令)
③ 头显:打开 XRoboToolkit app → 弹出连接列表 → 用扳机点选 PC 的 IP
     (没弹出就选 Network → Enter 手输)→ 主面板显示 WORKING
④ 头显面板:确认 Controller tracking 开着 → 把 "Send" 拨到 On
     (把 "Switch w/ A Button" 关掉:A 键要留给"作废回合"用)
```

**部署验收**(在第②步跑这个;在 pixi shell 里):

```bash
python scripts/pico_probe.py    # 打印手柄位姿和 grip 值;挥手柄数字变化、捏 grip 值升到 1.0 = 链路通
```

✅ 到这里部署完成。**回 `ur5e_setup.md` 实验 3** 开始 VR 遥操。

## 5. Python 接口速查(写代码/调试时参考)

```python
import xrobotoolkit_sdk as xrt
xrt.init()                                  # 初始化(要先起 PC Service 再 init)
p = xrt.get_right_controller_pose()         # 右手柄位姿 [x,y,z,qx,qy,qz,qw]
g = xrt.get_right_grip()                    # 右 grip,0.0~1.0 模拟量(阈值 0.9 判接管)
t = xrt.get_right_trigger()                 # 右扳机(夹爪)
a = xrt.get_A_button(); b = xrt.get_B_button()   # 右手柄 A/B 键
ts = xrt.get_time_stamp_ns()                # 采样时间戳
xrt.close()
```

## 6. 本页已知坑速查

| 现象 | 怎么办 |
|---|---|
| 启动后连不上 | PC Service 必须先于头显 app 启动;且只能开一个 |
| `adb devices` 列表为空 | 装 APK 必须用**数据线**;并在头显弹窗里点了"允许 USB 调试" |
| grip 没反应 | 它是 0~1 模拟量,要**捏到底**(阈值 0.9) |
| 头显放下就断连 | 摘下即睡眠断流 → 给距离传感器(镜片中间上方)贴一小块胶带 |
| 手柄追踪飘 | 头显架在额头/头顶时,前摄像头必须能看见手柄(靠视野追踪) |
| Linux 构建报 GLIBCXX 错 | conda 环境先 `conda install libstdcxx-ng`;pixi 环境无此问题 |
| 就是连不上 | 逐项查:同一 WiFi?能互 ping?PC 防火墙放行?启动顺序对不对(§4)? |

## 来源(全部官方)

- 快速开始/运行顺序:github.com/XR-Robotics(组织主页 README)
- PC Service(.deb / win.zip):github.com/XR-Robotics/XRoboToolkit-PC-Service release v1.0.0
- Python 绑定(含 setup_windows.bat):github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind v1.0.2
- 头显 APK v1.1.1:github.com/XR-Robotics/XRoboToolkit-Unity-Client releases
