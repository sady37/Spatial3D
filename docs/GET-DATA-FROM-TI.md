# 从 AWRL6844 取数据 — 操作指南

## 现状与根因（2026-07 实测）

- 芯片：**AWRL6844**（CCS target 树显示 `AWRL68xx`，核心 = Cortex-R5F + C66x DSP），
  属 **L 系列**，用 **MMWAVE-L-SDK**（≠ 经典 mmWave SDK）。
- 两个 UART 口 @115200/@921600 各读 2 秒 → **0 字节**：芯片上没有跑吐数据的固件。
- SDK 未安装，手上是 `MMWAVE_L_SDK_06_00_04_01-Linux-x86-Install.bin`（**Linux 版**）。
- 本机是 **macOS**：TI 的 radar SDK 只发 Windows/Linux，**Mac 装不了 SDK/编译固件**。

> 结论：**雷达要吐数据，必须先在芯片上跑一个「配置 RF + 从 DATA UART 推结果」的固件**。
> 这个固件来自 L-SDK 的 demo。装 SDK / 编译 / 烧录这一段在 **Linux** 上做；
> 读串口 / 解析 / 建模这一段在 **Mac（本仓库 pc/）** 上做。

```
[Linux]  装 L-SDK → 编译 OOB demo → 烧录          ← 让芯片产生数据
   │
   ▼  (雷达上电自动跑 / 或 CCS load-run)
[AWRL6844]  CLI UART @115200  ← 发 .cfg 启动
            DATA UART @921600 → TLV 帧（点云等）
   │
   ▼
[Mac / pc/]  发 .cfg + 读 DATA + 解析 TLV → 体素化 → Open3D
```

## 方案 A：Intel Mac + VirtualBox 虚拟机（当前采用）

主机：Mac mini 2018（Intel x86_64，36GB RAM）→ x86_64 Ubuntu 原生跑，无需模拟。
VM 软件：**VirtualBox 7.2.x**（Fusion 被 Broadcom 下架了 brew cask，改用 VirtualBox）。

### 0. 装 VirtualBox（已完成）
```bash
brew install --cask virtualbox
brew install --cask virtualbox-extension-pack   # USB 直通必需
```
装完在「系统设置 → 隐私与安全性」放行 Oracle 内核扩展并**重启**（否则 VM 起不来）。
验证：`kextstat | grep -i vbox` 有输出即 OK。

### 1. 建 Ubuntu 22.04 VM
下 `ubuntu-22.04.x-desktop-amd64.iso`。GUI 新建，或命令行：
```bash
VBoxManage createvm --name ti-linux --ostype Ubuntu_64 --register
VBoxManage modifyvm ti-linux --cpus 4 --memory 8192 --vram 128 --usbxhci on
VBoxManage createhd --filename ~/VirtualBox\ VMs/ti-linux/ti-linux.vdi --size 40960
VBoxManage storagectl ti-linux --name SATA --add sata
VBoxManage storageattach ti-linux --storagectl SATA --port 0 --device 0 --type hdd \
    --medium ~/VirtualBox\ VMs/ti-linux/ti-linux.vdi
VBoxManage storageattach ti-linux --storagectl SATA --port 1 --device 0 --type dvddrive \
    --medium ~/Downloads/ubuntu-22.04.5-desktop-amd64.iso
VBoxManage startvm ti-linux
```
装完系统后装 Guest Additions（剪贴板/共享目录），并从光驱移除 ISO。

### 2. XDS110 USB 直通
雷达插到 Mac，然后给 VM 加 USB 过滤器（TI 厂商 ID `0451`）：
```bash
VBoxManage usbfilter add 0 --target ti-linux --name xds110 --vendorid 0451
```
VM 里 `lsusb | grep -i texas` 能看到、并出现 `/dev/ttyACM0` `/dev/ttyACM1` 即成功。
（GUI 里：设置 → USB → 勾 USB 3.0(xHCI) → 加设备过滤器选 XDS110。）

### 3. 装 SDK / 编译 / 烧录 → 见下方「Linux 端步骤」

---

## Linux 端步骤

### 1. 安装 MMWAVE-L-SDK
```bash
chmod +x MMWAVE_L_SDK_06_00_04_01-Linux-x86-Install.bin
./MMWAVE_L_SDK_06_00_04_01-Linux-x86-Install.bin
# 默认装到 ~/ti/mmwave_l_sdk_06_00_04_01/
```
同时按 SDK release notes 装依赖：**SysConfig、TI ARM-CLANG 编译器、C6000 编译器、
Node.js**（L-SDK 用 makefile/gmake 构建）。这些 CCS 20 一般会一起拉。

### 2. 编译 out-of-box demo
demo 源码在：
```
~/ti/mmwave_l_sdk_<ver>/examples/mmw_demo/   （具体名以该版本为准）
```
用 SDK 顶层的 `imports.mak` 配好工具链路径后：
```bash
cd ~/ti/mmwave_l_sdk_<ver>/examples/mmw_demo/<board>/<core>
gmake all           # 产出 .appimage / .out
```

### 3. 烧录到 flash
把板子拨到 **flashing / SOP** 模式，用 **UniFlash**（或 SDK 自带
`uart_uniflash.py`）通过 CLI UART 烧 `.appimage`：
```bash
python uart_uniflash.py -p /dev/ttyUSB0 --cfg=<flash_cfg>.cfg
```
烧完拨回 **functional / run** 模式，上电。

> 快速验证（不烧 flash）：也可在 CCS 里 **Load Program** 把 `.out` 直接
> load-and-run 到 R5F，先确认能出数据，再决定要不要固化到 flash。

### 4. 确认在吐数据
上电后 DATA UART 应持续出帧。在 **Mac** 上跑本仓库的诊断工具即可确认（见下）。

## Mac 端（本仓库 pc/）

串口是 USB-CDC，macOS 直接认，无需装 SDK。两个口：
- CLI（配置）：`/dev/cu.usbmodem0000RA441`（推测，@115200）
- DATA（数据）：`/dev/cu.usbmodem0000RA444`（推测，@921600）

> 哪个口是 DATA 以诊断工具实测为准（能收到 TLV magic `02 01 04 03 06 05 08 07` 的就是）。

```bash
cd pc
# 1) 原始帧诊断：确认哪个口在吐数据、看帧头/TLV 结构
.venv/bin/python -m spatial3d.dump --data-port /dev/cu.usbmodem0000RA444

# 2) 发 cfg 启动 + 解析点云（cfg 来自 L-SDK / Radar Toolbox 里对应 demo 的 profile）
.venv/bin/python -m spatial3d.main --data-port /dev/cu.usbmodem0000RA444 \
    --cli-port /dev/cu.usbmodem0000RA441 --cfg path/to/profile.cfg --viz
```

## ⚠️ 待实测确认的点

TLV 具体字段（帧头长度、TLV type ID、点结构）**因 demo 而异**。本仓库 `tlv.py`
先按 TI 标准帧头（40B）+ 标准点云 TLV 实现，并提供 `dump` 诊断模式。等雷达真的
出数据后，用 `dump` 抓真实字节，对照 L-SDK 里该 demo 的 `mmw_output.h` /
`<demo>_output.h` 核对，再在 `tlv.py` 里微调即可。
