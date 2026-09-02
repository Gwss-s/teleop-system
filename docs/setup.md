# 实验 0:环境准备(从打开终端教起)

> 目标:把电脑准备好,做完本文能运行项目测试并看到全部通过。
> 不需要预装 Python、编辑器或任何开发工具——下面的 pixi 会把需要的都装好。
> 全文 🪟 = 只在 Windows 做,🐧 = 只在 Linux 做,没有标记 = 两个系统都一样。

## 0.1 你需要什么

- 一台 Windows 10/11 **或** Ubuntu 20.04 以上的电脑(内存 8GB 以上,磁盘剩余 10GB 以上);
- 能上网(要下载约 2GB 的软件与镜像)。

## 0.2 打开终端(所有命令都在这里输)

终端是一个输命令的黑色/蓝色窗口。

- 🪟 **Windows**:按 `Win` 键 → 输入 `powershell` → 回车。粘贴命令:在窗口里**单击鼠标右键**。
- 🐧 **Ubuntu**:按 `Ctrl+Alt+T`。粘贴命令:`Ctrl+Shift+V`。

用法:命令粘进去按**回车**才执行;执行完会回到提示符等你输下一条;
文档里以 `#` 开头的是注释(说明这条命令在做什么),不用输。

## 0.3 安装 pixi(项目环境管理器,会自动装好 Python)

**🪟 Windows**(在 PowerShell 里):

```powershell
# 下载并运行 pixi 官方安装脚本
powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"
```

**🐧 Linux**(在终端里;若提示"找不到 curl",先 `sudo apt install -y curl`):

```bash
# 下载并运行 pixi 官方安装脚本
curl -fsSL https://pixi.sh/install.sh | bash
```

装完**关闭终端窗口、重新开一个**(不然找不到命令),然后验证:

```bash
pixi --version        # 打印出 pixi 版本号(如 pixi 0.69.0)就算成功,数字不同没关系
```

如果提示"无法识别 pixi / 命令未找到",是因为没有重开终端——关掉重开一个再试。

## 0.4 获取项目代码

**方式 A(推荐,不需要任何工具)**:浏览器打开
`https://github.com/Gwss-s/teleop-system` → 点绿色 **Code** 按钮 →
**Download ZIP** → 解压到一个你找得到的目录(建议 🪟 `C:\lab\`、🐧 主目录 `~`)→
把解压出来的 `teleop-system-main` 文件夹改名成 `teleop-system`。

**方式 B(电脑装了 git 的同学)**:

```bash
# 从 GitHub 克隆项目到当前目录
git clone https://github.com/Gwss-s/teleop-system.git
```

## 0.5 进入项目目录

后面所有操作都要求你**在项目目录里**。

**🪟 Windows**:

```powershell
cd C:\lab\teleop-system      # 进入项目文件夹(路径按你实际解压位置改)
```

**🐧 Linux**:

```bash
cd ~/teleop-system           # 进入项目文件夹(路径按你实际解压位置改)
```

## 0.6 安装依赖并验证

在项目目录里依次执行(第一条要下载依赖,几分钟,看网速):

```bash
pixi install     # 下载并安装项目全部依赖;成功时最后一行是 "The default environment has been installed."
pixi run test    # 运行全部自动测试;成功时看到多行 [ok],最后三行都是 "all ... passed"
```

`pixi run test` 全部通过 = 环境完全就绪。

## 0.7 进入工作环境:pixi shell

```bash
pixi shell       # 进入项目环境;提示符前面出现 (teleop-system) 字样就算进去了。退出输 exit
```

**这是每次做实验前的固定动作**:之后文档里所有 `python ...` 开头的命令,
都必须在这个状态下运行,否则会报"找不到 python / 模块"。

📌 **记住:每次坐下干活的三步** —— ① 开终端 → ② `cd` 进项目目录 → ③ `pixi shell`。

## 0.8 安装 Docker(跑 UR5e 仿真器用;做实验 2 之前装好即可)

**🪟 Windows**:

1. 浏览器打开 `https://www.docker.com/products/docker-desktop/`,下载
   Docker Desktop for Windows 并安装(全部默认选项),装完**重启电脑**;
2. 从开始菜单打开 **Docker Desktop**,等左下角状态变绿(首次要 1-2 分钟);
   它必须**一直在后台运行**,跑仿真前先确认它开着;
3. 重新开一个 PowerShell 验证:

```powershell
docker --version      # 打印出 Docker version 2x.x.x 就算成功
```

**🐧 Linux**(实验室机器一般已装好,先验证;没装再装):

```bash
docker --version                  # 有版本号输出 = 已装好,可跳过下面两条
sudo apt install -y docker.io     # 安装 Docker
sudo usermod -aG docker $USER     # 把当前用户加入 docker 组(免 sudo 用 docker)
```

执行过 `usermod` 的话,要**注销并重新登录**一次才生效。

## 0.9 (可选)Xbox 手柄自查

手柄用 USB 插上电脑(免驱;蓝牙手柄先在系统设置里配对),在 pixi shell 里:

```bash
python scripts/gamepad_axis_dump.py    # 实时打印手柄各轴/键的值;拨动摇杆能看到数字变化即正常,Ctrl+C 退出
```

---

✅ **环境准备完成。** 回到 `README.md` 的实验路线表,开始实验 1。
