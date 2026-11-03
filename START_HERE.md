# 从这里开始 👈

> 你是这个项目的队友，刚拿到这份代码。**看完这一页就能开工。**

---

## 第 0 步：你是谁？

| 你是 | 你负责 | 先看这个文件 |
|---|---|---|
| **A** | FPGA 实时控制（Verilog） | [`handoff/A_fpga.md`](handoff/A_fpga.md) |
| **B** | ARM 视觉与通信 | [`handoff/B_arm.md`](handoff/B_arm.md) |
| **C** | 机械与集成 | [`handoff/C_mech.md`](handoff/C_mech.md) |

**三个人都要读一遍**：[`doc/interface_delivery.md`](doc/interface_delivery.md)（接口全貌）。

---

## 第 1 步：先跑这 3 条命令（**不用装任何东西**）

**双击运行这一个就行：**

```
tools\run_checks.ps1        ← 右键 → 使用 PowerShell 运行
```

它会自动做完 12 项检查（**不需要 ModelSim，不需要开发板，不需要任何硬件**）：

```
[1/3] 找 Python          ← 自动找到真 Python（跳过 Windows 自带的假 python）
[2/3] 静态检查 + 行为模型  ← 12 项，全是秒级
[3/3] 汇总               ← 全过就告诉你"可以开工了"
```

**应该看到 `通过 : 12`。**

### 如果它报"找不到能用的 Python 3"

**Windows 自带一个 0 字节的 `python` 占位程序** —— 敲 `python --version` 不报错也不输出，
很容易以为"Python 装好了"。解决方法：

1. 去 <https://www.python.org/downloads/> 下载 Python 3.10+
2. 安装时**务必勾选 `Add python.exe to PATH`**
3. 装完关掉所有 PowerShell 窗口，重新运行

**或者**：设置 → 应用 → 高级应用设置 → 应用执行别名，把 `python.exe` / `python3.exe` 关掉。

---

### 想手动跑也行（想看懂每一步的话）

```powershell
cd tools
python check_encoding.py ..              # 编码/BOM/纯ASCII
python rtl_lint.py ..\rtl ..\sim         # RTL 静态检查
python verify_uart_frame_model.py        # UART 帧协议模型（39 项）
cd ..\arm
python uart_proto.py                     # 协议库自测（31 项）
python fpga_link.py --dry-run            # 看帧长什么样
```

**这一步的意义**：确认 Python 环境没问题，并且**看懂协议长什么样**。

---

## 第 2 步：把你的 AI 接进来

**先发这段**（它会读必要文件、建立上下文）：

```
【项目背景】
轮足复合式 FPGA 实时感知与自主控制机器人，参加 2026 全国大学生嵌入式芯片
与系统设计竞赛（高云 FPGA 赛道）。三人分工：A 做 FPGA 实时控制，
B 做 ARM 视觉与通信，C 做机械与集成。

【请先读这几个文件，再动手】
1. START_HERE.md               —— 我在项目里的位置
2. doc/interface_delivery.md   —— 接口全貌（重点读 §1 §2 §4 §5）
3. handoff/<我的角色>.md        —— 我的任务清单和提示词
4. doc/design_guide.md         —— 每个模块内部怎么运转、为什么这么设计
5. doc/progress_review.md      —— 踩过的坑与修复过程（避免重复踩）

【当前状态】
FPGA 侧已完成 8 个模块 / 271 项仿真检查全部 PASS，整数链路闭环。
uart_frame 已写完验证但还没接进 top.v。ARM 板还没买。机械接口信息待确认。

【现在请你】
读完后用不超过 15 行总结：
  ① 我在整条链路里负责哪一段
  ② 我的第一个可执行任务是什么
  ③ 我有什么被别人的进度阻塞
然后等我确认，不要直接改代码。
```

**然后**打开 `handoff/<你的角色>.md`，里面有**针对具体任务的提示词**，直接复制用。

---

## 第 3 步：什么时候需要装工具

| 你想做什么 | 需要装什么 | 看哪个文档 |
|---|---|---|
| 跑 Python 检查 / 协议模型 | **只要 Python** | 上面第 1 步 |
| **跑 Verilog 仿真** | ModelSim + Gowin 原语库 | [`doc/runbook_verify_toolchain.md`](doc/runbook_verify_toolchain.md) |
| **综合上板** | Gowin IDE | [`doc/toolchain_gowin_modelsim.md`](doc/toolchain_gowin_modelsim.md) |
| 装 ARM 侧环境 | Python + pyserial | [`arm/README.md`](arm/README.md) |

**关键**：
- **工程路径不要带中文** —— ModelSim 2020.4 在中文路径下建不了 work 库
- 跑仿真前先建 `sim/toolchain/toolchain.local.ps1`（复制 `.template` 改路径）

---

## 常见卡点（先看这里，别急着问）

| 现象 | 原因 | 怎么办 |
|---|---|---|
| `python` 不是命令 | 没装或没加 PATH | 装 Python 3，勾选 "Add to PATH" |
| 仿真报 `Error loading design` | 只有一句话没细节 | 跑 `vopt` 才给真因；见 `handoff/A_fpga.md` 的排查提示词 |
| `Instantiation of 'PLL' failed` | 没编译 Gowin 原语库 | 跑 `setup_gowin_modelsim.ps1` |
| `.ps1` 报诡异的语法错 | 文件丢了 UTF-8 BOM | 跑 `python tools/check_encoding.py .` |
| 串口收到乱码 | **没共地** | TX / RX / **GND** 三根线都要接 |
| 不知道改哪个文件 | — | 问你的 AI，**但先让它读上面第 2 步那 5 个文件** |

---

## 项目现在能做什么

| ✅ 已完成（仿真验证） | ⏳ 待完成 |
|---|---|
| **9 个模块 / 287 项检查全 PASS** | 引脚分配：填 `constraints/pin_table.csv` 后跑 `python tools/gen_cst.py` |
| **UART 已接入**（vision_rx + telemetry_tx） | TOF / IMU / 舵机驱动（等选型） |
| UART 帧协议（含超时/校验/异常保护） | ARM 视觉（板未到） |
| 机械接口需求清单 | 机械设计与装配 |

**B 的协议层可以在 PC 上完全独立跑通，不用等任何硬件。**

---

## 遇到问题

1. **先看**上面的"常见卡点"
2. **再问你的 AI** —— 把**完整报错**贴给它，不要只贴一行
3. **还不行就在群里问**，附上你的环境和完整报错
