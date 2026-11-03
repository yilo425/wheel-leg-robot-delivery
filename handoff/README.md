# handoff/ —— 三人任务分工与提示词

> **这是发给队友的那一份。** 每人只需要看自己那个文件。
>
> 团队三人：**A** = FPGA 实时控制 · **B** = ARM 视觉与通信 · **C** = 机械与集成

---

## 1. 你该看哪个文件

| 你是谁 | 看这个 | 里面有什么 |
|---|---|---|
| **A（FPGA）** | [`A_fpga.md`](A_fpga.md) | 剩余 FPGA 任务 + **会话重启提示词** + 5 个场景提示词 |
| **B（ARM）** | [`B_arm.md`](B_arm.md) | 任务里程碑 + 4 个场景提示词 + 协议速查 |
| **C（机械）** | [`C_mech.md`](C_mech.md) | **13 项待填接口表** + 任务清单 + 4 个场景提示词 |

**所有人都要读**：[`doc/interface_delivery.md`](../doc/interface_delivery.md) —— 接口全貌。

---

## 2. ⚠️ 当前三个阻塞项（按紧急程度）

| # | 阻塞项 | 卡住谁 | 谁来解决 |
|---|---|---|---|
| 1 | **ARM 板还没买** | B 的全部工作 | **B —— 今天下单** |
| 2 | **机械接口表 13 项没填** | A 的 6 个模块 | **C —— 今明两天** |
| 3 | **引脚分配表没拿到** | 上板 | C |

**其中第 1 项有时间成本（采购周期），最紧急。**

---

## 3. 接下来 48 小时

| 时间 | A（FPGA） | B（ARM） | C（机械） |
|---|---|---|---|
| **今天** | `uart_frame` 接入 `top.v`（`A_fpga.md` 提示词 2） | **下单买板** + 跑 `arm/uart_proto.py` 自测 | **填 13 项表**（`C_mech.md` §1） |
| **明天** | 扩展 `tb_top_integration` + `clk_gen`(PLL) | PC 上完成收发闭环（虚拟串口） | 拿引脚表 + 确定底盘方案 |
| **后天** | 更新 `design_guide.md` | 写颜色识别（`detect_from_image.py`） | 出采购清单并下单 |

**核心原则**：**A 不等 B/C，B 和 C 也不等 A。** 协议已冻结，三条线可以并行。

---

## 4. 三个人的工作各自独立到什么程度

```
A（FPGA）  ──── uart_frame 接入 top.v ────┐
                                          │
B（ARM）   ──── 协议代码（PC 可测）───────┤──── 联调（等到这一步才需要见面）
                                          │
C（机械）  ──── 接口表 + 底盘 + 装配 ─────┘
```

| 谁 | 现在能独立做到什么程度 | 什么时候必须等别人 |
|---|---|---|
| **A** | 除 `servo_lift`/`tof_driver`/`imu_driver` 外全部 | 上板时等 C 的引脚表 |
| **B** | **协议层 + PC 端视觉算法全部** | 联调时等 A 的 A1 完成 |
| **C** | 除联调外全部 | 装电机时等 A 的引脚分配 |

---

## 5. 每个人怎么自己验证（不需要别人的环境）

### B：协议库自测（**秒级，不需要 ModelSim，不需要硬件**）

```bash
cd arm
python uart_proto.py          # 期望: RESULT: PASS (31 项)
python fpga_link.py --dry-run # 看帧结构、BCC 复核、坐标变化
```

### A：完整仿真（**分钟级，需要 ModelSim**）

```bash
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1
```

### C：不需要跑代码，看文档即可

---

## 6. 团队协作规则（[`doc/team_sync_guide.md`](../doc/team_sync_guide.md)）

**三条纪律**：

1. **每天开工先 `git pull`**
2. **只有 8 个 tb 全 PASS 才合并进 `main`**
3. **改协议 = 独立 commit + 双方确认 + 升版本号**

**验证记录**：每次跑完仿真，`run_tb.ps1` 会自动在 `verify/` 下生成一份报告。
文件名带【用户+机器+commit】，三个人各生成各的，**永远不会冲突**。

```bash
git log --oneline verify/    # 看谁对哪个版本验证过什么
```

---

## 7. Git 分支约定

```
main          ← 稳定版，任何时候 checkout 都能跑通全部仿真
 ├── fpga     ← A 的日常分支
 ├── vision   ← B 的日常分支
 └── mech     ← C 的日常分支
```

```bash
git clone <仓库地址>
cd robot_project
git checkout vision     # B
git checkout mech       # C
```

---

## 8. 每个提示词怎么用

`A_fpga.md` / `B_arm.md` / `C_mech.md` 里的提示词都是**可以直接整段复制**的。

**用法**：

1. 打开你自己的 DSH
2. **先发"开场提示词"** —— 它会让 DSH 读必要文件、建立上下文
3. DSH 总结完之后，**再发具体任务的提示词**

**为什么要有"开场提示词"**：DSH 的**会话上下文不共享** ——
你的 DSH 不知道我们之前讨论过什么。开场提示词就是用来补这个上下文的。

**注意**：提示词里凡是 `<...>` 的地方都是**要你自己填的**。

---

## 9. 文件全貌

```
robot_project/
├── README.md                 工程总览 + 文档导航
├── .gitignore / .gitattributes
│
├── rtl/                      ★ A 负责（Verilog 源码）
├── sim/                      ★ A 负责（Testbench + 一键脚本）
├── constraints/top.cst       ★ C 提供引脚，A 填
│
├── arm/                      ★ B 负责（可以直接用的协议代码）
│   ├── uart_proto.py           协议库（已自测 31/31）
│   ├── fpga_link.py            联调工具（不需要摄像头）
│   └── README.md
│
├── tools/                    静态检查 + Python 行为模型（A 用）
├── verify/                   ★ 三人都要提交（验证记录）
│
├── handoff/                  ★ 本目录 —— 任务分工与提示词
│   ├── README.md
│   ├── A_fpga.md
│   ├── B_arm.md
│   └── C_mech.md
│
└── doc/                      文档
    ├── interface_delivery.md    ★ 所有人都先读这个
    ├── team_sync_guide.md       ★ 协作规则
    ├── design_guide.md          模块内部设计（A 的活）
    ├── uart_frame_protocol.md   协议定义（B 的活）
    ├── interface_spec.md        模块级接口
    ├── progress_review.md       踩过的坑（防重复踩）
    ├── runbook_verify_toolchain.md  环境配置手册
    └── toolchain_gowin_modelsim.md  工具链原理
```

---

## 10. 时间线

| 日期 | 里程碑 |
|---|---|
| **2026-10-05**（今天） | 三人各自开工：A 接入 UART、B 下单、C 填表 |
| **2026-10-10** | A：FPGA 代码基本完成；B：板到货，摄像头点亮；C：底盘装好 |
| **2026-10-20** | **★ 红线**：视觉必须能出结果，否则降级到纯颜色/形状识别 |
| **2026-10-25** | 全系统联调通过，能跑完整任务 |
| **2026-11-01** | 演示视频、报告、PPT 完成 |
| **2026-11-04** | **提交截止** |
| **2026-11-20~22** | 决赛 |

---

## 11. 需要帮助时

| 情况 | 怎么做 |
|---|---|
| 卡住了 | 在群里说，**附完整报错**（不要只贴一行） |
| 要改协议 | **先问 A 和 B**，见 `team_sync_guide.md` §7 |
| 改了代码 | **跑一遍验证，提交 `verify/` 报告** |
| 发现文档和代码不一致 | **改文档并提交** —— 文档不更新等于没做 |
