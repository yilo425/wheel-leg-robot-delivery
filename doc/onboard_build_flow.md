# 上板流程（Gowin 综合 → 比特流 → 下载）

> 目标板：小梅哥 ACG720 / **GW5AT-138**（`GW5AT-LV138PG484AC1/I0`，PBGA484A）
> 器件版本：`B`（若综合报错可试 `C`）

---

## 0. ⚠️ 两个必须先知道的坑

### 坑 1：中文路径是**硬阻塞**

Gowin 工具链读不了含中文的路径。实测：

```
PWD = C:/Users/??/Documents/...            <- "忆洛" 被吃成 "??"
alog error: failed to open log file: 'C:/Users/??/AppData/LocalLow/...'
```

**后果**：工作目录错乱 → 找不到文件 → 综合失败，
而且错误信息**一个字都没提"路径"**，极难定位。

**做法**：工程放到**纯 ASCII 路径**，例如：

```
D:\robot_build\robot\
```

不要放在 `C:\Users\<中文名>\...` 下面。
（ModelSim 也有同样的问题，那边用 `D:\gowin_lib\work` 绕开了。）

> 试过但**不管用**的绕法：
> - Windows 8.3 短名 —— `C:\Users\忆洛` 本身没有短名（只有子目录有）
> - `subst` 映射盘符 —— 可用，但本项目环境里被拒绝

### 坑 2：`.tcl` 脚本必须**纯 ASCII**

`gw_sh` 用 GBK 读脚本，UTF-8 中文注释会乱码成 `???????`，
严重时直接语法错误。

**所有给 `gw_sh` 的 `.tcl` 里不要写中文。**

---

## 1. 工具位置

| 工具 | 路径 | 用途 |
|---|---|---|
| `gw_sh.exe` | `D:\Gowin\...\IDE\bin\` | 命令行综合/布局布线 |
| `programmer_cli.exe` | `D:\Gowin\...\Programmer\bin\` | **命令行下载** |
| `programmer.exe` | 同上 | 图形界面下载 |

---

## 2. 第一步永远是：**LED 冒烟测试**

**不要一上来就烧整个机器人设计。**

| 直接烧整机 | 先烧 LED |
|---|---|
| 出问题不知道是**工具链**、**引脚**还是**逻辑** | LED 闪 = 三件事一次全排除 |

冒烟测试文件在 [`tools/fpga_build/led_smoke/`](../tools/fpga_build/led_smoke/)：

```
led_smoke/led_test.v      流水灯 + 按键
led_smoke/led_test.cst    clk=Y18  rst=B21  led=M22 N22 L21 K21
led_smoke/led_test.sdc    50MHz 时序约束
led_smoke/build_led.tcl   综合脚本
```

**现象**：4 个 LED 每 0.125s 循环左移一格；按住 S4 复位键全灭。

**跑法**：

```powershell
cd D:\robot_build\led_test
D:\Gowin\...\IDE\bin\gw_sh.exe build_led.tcl
```

看到 `RESULT_OK` 就成了。产物：

```
D:\robot_build\led_test\led_test\impl\pnr\led_test.fs
```

> **实例记录**：本项目第一次跑通时，`led_test.fs` = **34,690,867 B**（约 34.7 MB）。
> 138K 器件的比特流就是这个量级，不是异常。

---

## 3. 检查引脚有没有真的按要求布

综合完**一定要看一眼** `impl/pnr/<name>.pin.html`。

**为什么**：`.cst` 写错了（比如两个信号填了同一个球号）Gowin **不一定会报错**，
但引脚报告是布局布线的**实际结果**。

```powershell
# 快速核对
Select-String -Path 'D:\robot_build\led_test\led_test\impl\pnr\led_test.pin.html' `
  -Pattern 'clk_50m|rst_key_n|led\['
```

期望看到（已实测确认）：

```
clk_50m    Y18/4
rst_key_n  B21/2
led[0]     M22/3
led[1]     N22/3
led[2]     L21/3
led[3]     K21/3
```

---

## 4. 时序约束（SDC）

**没有 SDC 会得到这个警告**：

```
WARN (TA1132): 'clk_50m' was determined to be a clock but was not created.
```

**意思是工具压根没检查时序。** LED 无所谓，**整车设计绝对不能这样**。

`led_smoke/led_test.sdc` 内容：

```tcl
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]
set_input_delay  -clock clk_50m -max 5 [get_ports {rst_key_n}]
set_output_delay -clock clk_50m -max 5 [get_ports {led[*]}]
```

> ⚠️ SDC 用的是 **Tcl 语法**，注释是 `#` 不是 `//`。

加上之后 `TA1132` 消失 ✓（已实测）

---

## 5. 下载到板子

### 先确认硬件

```powershell
programmer_cli.exe --scan
```

**正常**：

```
Target Cable: Gowin USB Cable(FT2CH)/0/None/null@2.5MHz
```

**如果接着是**：

```
Error: No Gowin devices found!
```

→ **先查电源**。下载器认到了但芯片不响应，最常见就是：

| 检查 | |
|---|---|
| 板子电源开关开了吗？电源灯亮吗？ | |
| USB 插的是**下载口**（JTAG/PROG）不是串口口？ | |
| USB 供电够吗？（FPGA 上电电流不小，试试外接电源） | |

### 下载

```powershell
# operation_index 2 = 烧到 SRAM（掉电丢失，调试用，可以反复烧）
programmer_cli.exe --device GW5AT-138 --fsFile <路径>\led_test.fs --operation_index 2

# operation_index 6 = 烧到外部 Flash（掉电保留，成品用）
programmer_cli.exe --device GW5AT-138 --fsFile <路径>\led_test.fs --operation_index 6
```

**调试期用 2** —— SRAM 掉电就没了，不用反复擦 Flash。

---

## 6. 整机设计怎么综合

[`build_all.tcl`](../tools/fpga_build/build_all.tcl) 是整机版本。用之前：

1. 把 `robot_project` 复制到纯 ASCII 路径（比如 `D:\robot_build\robot\src`）
2. 检查 `build_all.tcl` 顶部的 `PROJ_ROOT` / `SRC_ROOT`
3. 跑

```powershell
D:\Gowin\...\IDE\bin\gw_sh.exe build_all.tcl
```

---

## 7. 已验证的事实（本项目实测）

| 项 | 结果 |
|---|---|
| `gw_sh` 能命令行综合 | ✅ |
| 器件 `GW5AT-LV138PG484AC1/I0` + `-device_version B` | ✅ **必须带 `-device_version`**，否则报 "The target device does not have device version" |
| 布局布线 + 比特流生成 | ✅ |
| 加了 SDC 后 `TA1132` 消失 | ✅ |
| 引脚报告与 `.cst` 一致 | ✅ |
| `programmer_cli --scan` 认到下载器 | ✅（`Gowin USB Cable(FT2CH)`）|
| **Y18 是全局时钟脚且直连 PLL** | ✅ `MGCLKT_4/BPLL2_T_FB0/BPLL3_T_FB0` —— **`clk_gen` 的 PLL 输入就接它** |
