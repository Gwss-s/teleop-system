# 实验 0:环境准备(双系统,从打开终端教起)

> 目标:把电脑准备好。做完本文,你能运行本项目的测试并看到全部通过。
> 不需要预装 Python、编辑器或任何开发工具——下面的 pixi 会把需要的都装好。
> 每个命令框上方都标了**在哪里执行**;命令一行一行复制、粘贴、回车即可。

## 0.1 你需要什么

- 一台 Windows 10/11 **或** Ubuntu 20.04 以上的电脑(内存 ≥8GB,磁盘剩余 ≥10GB);
- 能上网(要下载约 2GB 的软件与镜像);
- 本文出现 🪟 标记的框 = 仅 Windows;🐧 = 仅 Linux;没有标记 = 两个系统相同。

## 0.2 打开终端(所有命令都在这里输)

- 🪟 **Windows**:按 `Win` 键 → 输入 `powershell` → 回车,打开的蓝底/黑底窗口
  就是终端(叫 PowerShell)。粘贴:在窗口里**单击鼠标右键**;
- 🐧 **Ubuntu**:按 `Ctrl+Alt+T`。粘贴:`Ctrl+Shift+V`。

终端使用常识(会的可跳过):把命令粘贴进去按**回车**才会执行;命令执行完会回到
提示符;`cd 某目录` 表示"进入某个文件夹";看到以 `#` 开头的行是注释,不用输。

## 0.3 安装 pixi(项目环境管理器,自带 Python)

> 🪟 **Windows**(在 PowerShell 里):
> ```powershell
> powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"
> ```

> 🐧 **Linux**(在终端里;若提示"找不到 curl"先执行 `sudo apt install -y curl`):
> ```bash
> curl -fsSL https://pixi.sh/install.sh | bash
> ```

装完**关闭终端窗口,重新开一个**(否则找不到命令),验证:

```bash
pixi --version        # 预期输出类似: pixi 0.69.0(数字不同没关系)
```

> 卡住了?输入 `pixi` 提示"无法识别/命令未找到" → 你没有重开终端,回去重开一个。

## 0.4 获取项目代码(两种方式任选)

**方式 A(推荐,无需任何工具)**:浏览器打开
`https://github.com/Gwss-s/teleop-system` → 绿色 **Code** 按钮 →
**Download ZIP** → 解压到一个**你找得到的目录**(建议:🪟 `C:\lab\`,🐧 `~/`),
解压后得到文件夹 `teleop-system-main`,把它改名为 `teleop-system`。

**方式 B(装了 git 的同学)**:

```bash
git clone https://github.com/Gwss-s/teleop-system.git
```

然后**进入项目目录**(后面所有操作都要求你在这个目录里):

> 🪟 ```powershell
> cd C:\lab\teleop-system
> ```

> 🐧 ```bash
> cd ~/teleop-system
> ```

## 0.5 安装项目依赖并验证

在项目目录里依次执行(第一条要下载依赖,几分钟,取决于网速):

```bash
pixi install          # 预期最后一行: The default environment has been installed.
pixi run test         # 预期看到多行 [ok] ...,最后三行都是 "all ... passed"
```

`pixi run test` 全部通过 = 你的环境完全就绪。

## 0.6 进入工作环境:pixi shell

```bash
pixi shell            # 提示符前面出现 (teleop-system) 字样 = 已进入环境
```

**这是每次做实验前的固定动作**。之后文档里所有 `python ...` 开头的命令,
都必须在这个状态下执行(否则会报"找不到 python/模块")。退出环境输 `exit`。

> **每次坐下干活的三步**:① 开终端 → ② `cd` 进项目目录 → ③ `pixi shell`。

## 0.7 安装 Docker(跑 UR5e 仿真器用;实验 2 之前装好即可)

> 🪟 **Windows**:
> 1. 浏览器打开 `https://www.docker.com/products/docker-desktop/`,下载
>    Docker Desktop for Windows 并安装(全部默认选项),装完**重启电脑**;
> 2. 开始菜单打开 **Docker Desktop**,等左下角状态变绿(首次要 1-2 分钟);
>    它必须**保持在后台运行**,跑仿真前先确认它开着;
> 3. 验证(重新开一个 PowerShell):
>    ```powershell
>    docker --version      # 预期输出 Docker version 2x.x.x
>    ```

> 🐧 **Linux**(实验室机器一般已装好,先验证;没装再执行安装):
> ```bash
> docker --version                          # 有版本号输出 = 已装好,跳过下面
> sudo apt install -y docker.io             # 安装
> sudo usermod -aG docker $USER             # 允许当前用户使用 docker
> ```
> 执行过 `usermod` 的话,**注销并重新登录**一次才生效。

## 0.8 (可选)Xbox 手柄自查

手柄 USB 插上电脑(免驱;蓝牙手柄先在系统设置里配对),然后在
pixi shell 里:

```bash
python scripts/gamepad_axis_dump.py
# 预期: 打印手柄名称和一行实时数字;拨动摇杆能看到数字变化;Ctrl+C 退出
```

---

✅ **环境准备完成。** 回到 `README.md` 的实验路线表,开始实验 1。
