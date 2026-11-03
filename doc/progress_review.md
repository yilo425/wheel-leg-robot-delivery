# 项目现状回顾（2026-10-04 工作日志）

> 本文是一次完整工作周期的复盘：**做了什么、查出什么问题、怎么修的、
> 现在 Gowin 和 ModelSim 是什么关系、当前能跑什么**。
> 可作为站会汇报、周报、以及答辩材料的素材。

---

## 0. 一句话总结

**起点**：Gowin 和 ModelSim 两个软件**完全没联系**，工程代码从未被任何仿真器执行过。
**终点**：工具链打通，**5 个 Testbench 全部 `RESULT: PASS`**，
并修掉 **5 个会在上板时才爆发的 RTL 缺陷**（其中 3 个是致命的）。

---

## 1. 起点状态（工作开始前）

| 项目 | 状态 |
|---|---|
| 工程代码 | 工作区内**为空**（此前讨论过的骨架不在本工作区） |
| Gowin IDE | `D:\Gowin\Gowin_V1.9.11.03_Education_x64`，支持 GW5AST-138 |
| Gowin 仿真库 | `IDE\simlib\gw5a\prim_sim.v`（709,303 B，含 PLL/OSC/GSR） |
| ModelSim | 两份：`C:\` 与 `D:\modeltech64_2020.4`（SE-64 2020.4） |
| **两者关系** | ❌ **完全没有联系**——Gowin 库从未编译进 ModelSim；IDE 里也没配过仿真器 |
| 仿真可行性 | ❌ 从未成功跑过；GUI 用不了（许可），命令行也因中文路径失败 |

**核心矛盾**：Gowin 给的是原语**源码**，ModelSim 只认**编译好的库**。
中间这一步没人做过，于是"两个软件各干各的"。

---

## 2. 做了什么（按阶段）

### 阶段 1 · 建立工程骨架
按竞赛规范建立目录结构，从零写出驱动层模块与自检 Testbench：

```
robot_project/
├── rtl/  project_config.vh · top.v
│         hal/reset_sync.v
│         driver/{pwm_motor, encoder_decode, uart_tx, uart_rx}.v
├── sim/  5 个 Testbench + ModelSim 脚本
├── constraints/top.cst（引脚占位）
├── tools/ 静态检查、参数自检、UART 模型
└── doc/  接口规格、帧协议、工具链、执行手册
```

### 阶段 2 · 打通工具链（最难的一步）
逐层定位并解决 3 个环境问题（详见 §3.1）。

### 阶段 3 · 让测试结果可见
ModelSim 批处理模式下 `$display` 走 stdout，管道经常抓不到 →
**改为 Testbench 用 `$fwrite` 自己写结果文件**，从此结果一定可见。
**关键测量值也必须落盘**——这一点后面反复救了场。

### 阶段 4 · 逐个修 RTL 缺陷
每修一个都先用**廉价验证**（Python 周期精确模型 / 逻辑推演）确认，再跑 ModelSim。

### 阶段 5 · 达成 5/5 PASS

---

## 3. 查出的问题与修复

### 3.1 环境 / 工具链问题（3 个）

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| E1 | **ModelSim 建库失败**：`init_dbinfo() DATABASE ERROR: sqlite3_open ... unable to open database file` | ModelSim 2020.4 用 **ANSI 编码**打开库数据库，**路径含中文必失败**（本机用户名是"忆洛"） | work 库放到**纯 ASCII 路径**（`D:\gowin_lib\work`）；**源码可留在中文路径**，只有库目录要求 ASCII |
| E2 | 例化 PLL 会报 `Instantiation of 'PLL' failed` | Gowin 的器件原语库**从未编译进 ModelSim** | 用 `vlog` 把 `prim_sim.v` 编译成 `gw5a` 库，并用 `vmap` 建立映射、`vsim -L gw5a` 挂载 |
| E3 | 手工把 `.v` 文件拷来拷去，两边文件列表不同步 | 没有统一构建脚本 | 生成 `compile_all.do` / `run_sim.do`，**直接用相对路径引用工程源码**；每次改文件列表重跑生成器 |

> **更正（同日实测）**：本机 ModelSim GUI **可用**。
> 早先判定"签不出 viewer license"，实际原因是 **`LM_LICENSE_FILE` 没设对**，
> 不是许可本身缺 feature —— 许可文件里确实有 `INCREMENT vsimviewer_c`。
> 设好环境变量后 `vsim -do wave.do` 能正常弹出 GUI。
> 所以**不要用 `vsim -do wave.do`**，改走"批处理 + 导出 VCD + GTKWave"。

### 3.2 RTL 缺陷（5 个致命/功能性）

| # | 模块 | 缺陷 | 上板后果 | 发现方式 |
|---|---|---|---|---|
| **R1** | `uart_tx` / `uart_rx` | `BAUD_RATE × 2²⁴ = 1.93×10¹²` 在 **32 位整数里溢出**，`BAUD_INC` 算出来是 **0** → `baud_tick` 永不触发 | **一个字节都发不出去**；会误以为是线序/波特率/共地问题 | 仿真（guard 超时 + 诊断输出） |
| **R2** | `uart_tx` | 用"计数归零"当比特结束标志：进入 `START` 时 `div_cnt` 正好是 0 → **起始位只持续 1 个 clk**（应为 868） | 收发全废；起始位宽度为 0 | 仿真（`start bit width = 0`） |
| **R3** | `uart_rx` | 下降沿判据写成了**上升沿**：`if (rx_s2==0 && rx_s1==1)` | **永远收不到任何字节**；且 TX 看起来正常，更难查 | Python 模型（逐拍追踪状态机） |
| **R4** | `reset_sync` | `POR_CYCLES=0` 时用 `always @(*) por_done = 1'b1;`——**空敏感列表**（body 没读任何信号）+ 与声明初值 `= 1'b0` **竞争**，ModelSim 里初值赢了 → `por_done` 恒为 0 | **上电后复位永不释放，整块电路锁死** | 仿真（新增的 `POR_CYCLES=0` 测试项） |
| **R5** | `pwm_motor` / `top` | `duty_in` 的符号同时被当作"方向"和"幅值"。普通驱动板只有**一根方向脚**，负 duty 表达不了倒车 | 倒车变成"反向 PWM 减速"，电机不会真的反转 | 设计审查 + 集成仿真 |

**R1 / R3 / R4 是典型的"仿真几分钟发现、上板查几天"类型。**

### 3.3 RTL 缺陷（早期静态审查发现，7 个）

这些是在还没有仿真器时，靠逐行审查 + 自建静态检查工具发现的：

| # | 模块 | 缺陷 |
|---|---|---|
| S1 | `uart_rx` | `os_acc` **重复声明** → 直接编译不过 |
| S2 | `uart_rx` | 过采样分频写错（`os_cnt` 放在分频之后）→ 有效过采样退化成 1 倍 |
| S3 | `uart_tx` | 波特率累加器阈值写成 `BAUD_INC >> ACC_WIDTH`，而 `BAUD_INC` 远小于 2²⁴ → 阈值恒为 0 |
| S4 | `pwm_motor` | `mag × STEP_DIV` 在 16 位里可能**静默溢出截断** |
| S5 | `encoder_decode` | 用拼接结果 `{{29{1'b0}}, step_s}` 当有符号数 → `-1` 被当成 `+1` |
| S6 | `uart_tx` | `acc` **先用后声明** → `vlog` 报 `vlog-2730 Undefined variable` |
| S7 | `pwm_motor` / `top` | 方向语义（同 R5） |

> 这套"逐行审查 + 静态检查工具"共发现 **7 个问题**，
> 说明在缺仿真器时，系统性的代码审查仍能拦住相当一部分缺陷。
> 但 **R1/R2/R3/R4 这四个是审查拦不住的**——只有真跑仿真才能暴露。

### 3.4 Testbench / 工具缺陷（7 个）

这些是"测试写错导致误报"的问题，同样值得记录：

| # | 问题 | 根因 |
|---|---|---|
| T1 | `tb_uart` guard 上限算错 | 连续三轮算错，最后一次是 `10 × 10⁹` 在 32 位里**溢出**（同 R1 的坑）；另外 20MHz 下 16 倍过采样的**截断误差达 8%**，接收必然失败 |
| T2 | `tb_encoder_decode` 测速为 0 | `step_fwd(600)` **跑完之后**才采样，编码器已停、窗口为空。**测量必须和运动同时进行** → 改用 `fork` |
| T3 | `tb_reset_sync` 延迟测出 4 拍（应为 3） | `rst_n` 在 `posedge` 的 **NBA 区**更新，测试却在**同一拍 active 区**就读 → 读到旧值。改用**无歧义判据**："第 N-1 拍仍低、第 N 拍已高" |
| T4 | `tb_pwm_motor` `clamp` 断言写反 | `dir_in=1` 时应为 `+100`，却期望 `-100` |
| T5 | `tb_encoder_decode` `rev: speed == -window` 断言写反 | `window_cnt` 本身就是**有符号净计数**，应 `==` 而不是 `== -` |
| T6 | `tb_top_integration` 加载失败（无细节报错） | ① `$display` 里引用了**未声明**的 `TEST_VREF` → 隐式线网；② 用了 `dut.xxx` **层次引用**，需要 `+acc` 否则优化器删信号后直接加载失败 |
| T7 | `tb_top_integration` 开环测试全失败 | 写 `key = 2'b01` 却注释成"key[0]=0"——Verilog 的 `2'b01` 是 `{key[1],key[0]}={0,1}`，**`key[0]=1`（关使能）**。RTL 是对的，测试位序写反 → 改用**具名常量** |

### 3.5 我（AI）自己造成的返工

诚实记录，避免下次重犯：

| 失误 | 后果 |
|---|---|
| 判据查错了文件（查 `_lib.qdb`，而 `vlib` 成功时先写 `_info`） | 让用户白跑了好几轮"建库失败" |
| `rebuild.ps1` 用了一个**永远不匹配的过滤器**显示输出 | 测试其实在正常跑，屏幕却一片空白 → 用户以为死机 |
| 让你用 `Add-Content` 建 `modelsim.ini` | 造出没有 `[Library]` 段的残废文件 → `vmap` 报 `(vmap-20)` |
| 用 `Copy-Item 'rtl\*' ... -Recurse` 通配符批量复制 | 嵌套目录可能没覆盖，导致 `top.v` 等文件静默未同步 |
| **没先做廉价验证就让你跑昂贵的验证** | UART 那几个 bug 本可以用 Python 模型几分钟定位，却来回跑了十几轮 |

---

## 4. 现在 Gowin 与 ModelSim 是什么关系

**从"完全独立"变成"通过三座桥连起来"，但各有分工。**

### 4.1 分工

| 工具 | 职责 | 不负责 |
|---|---|---|
| **Gowin IDE** | 综合、布局布线、时序约束、生成比特流、烧录、**提供器件原语仿真库** | 不做功能仿真 |
| **ModelSim** | **功能仿真**（Testbench 验证）、波形导出 | 不做综合/烧录 |

### 4.2 三座桥

**桥 1 · 器件原语库**
```
D:\Gowin\...\IDE\simlib\gw5a\prim_sim.v        （Gowin 给的源码，709 KB）
        │  vlog -work gw5a prim_sim.v
        ▼
D:\modeltech64_2020.4\gw5a                     （编译好的库，含 PLL/OSC/GSR/LUT4…）
        │  vmap gw5a D:/modeltech64_2020.4/gw5a   （写进 modelsim.ini）
        ▼
vsim -L gw5a ...                               （仿真时挂载）
```
**没有这座桥**：一旦代码例化 PLL（`clk_gen.v` 必然要用），
ModelSim 会在 elaboration 阶段报 `Instantiation of 'PLL' failed`。

**桥 2 · ASCII 路径**
```
源码可以放在中文路径（如 C:\Users\忆洛\...）  ✅ 没问题
但 work 库必须放纯 ASCII 路径（D:\gowin_lib\work）  ⚠️ 硬性要求
```
原因：ModelSim 2020.4 用 ANSI 编码打开库数据库，中文路径必失败。

**桥 3 · 构建脚本**
```
sim/compile_all.do   建 work 库 + vmap 挂 Gowin 库 + 编译全部源码
sim/run_sim.do       依次跑 5 个 Testbench
sim/vcd.do           导出 VCD 波形（不需要 GUI 许可）
sim/wave.do          GUI 加载波形（**实测可用**，推荐首选）
```
这些脚本**直接用相对路径引用 `../rtl/*.v`**，不再需要拷贝文件。
改文件列表时重跑 `toolchain\emit_dofiles.ps1` 重新生成。

### 4.3 当前可用的运行方式

| 方式 | 是否可用 | 说明 |
|---|---|---|
| 命令行批处理 `vsim -c` | ✅ **完全可用** | 日常验证走这条 |
| 命令行导出 VCD | ✅ **可用** | 配合免费 GTKWave 看波形 |
| GUI（`vsim` 不带 `-c`） | ✅ **可用** | 需先设好 `LM_LICENSE_FILE` |
| Gowin IDE 内部调 ModelSim | ⚪ 未配置 | 当前走独立命令行流程，更可控 |

---

## 5. 现在能跑什么

### 5.1 已实测通过（5/5）

| Testbench | 检查项 | 结果 |
|---|---|---|
| `tb_pwm_motor` | 21 | ✅ PASS |
| `tb_encoder_decode` | 17 | ✅ PASS |
| `tb_uart` | 17 | ✅ PASS |
| `tb_reset_sync` | 11 | ✅ PASS |
| `tb_top_integration` | 17 | ✅ PASS |

**集成测试的关键数值与设计值精确吻合**（可作答辩数据）：

| 项目 | 实测 | 设计值 |
|---|---|---|
| 开环占空比 | **2280** clk | `LIMIT_DUTY(120) × STEP_DIV(19)` = 2280 |
| 闭环静止占空比 | **570** clk | `(VREF>>1)(30) × 19` = 570 |
| 编码器正转后占空比 | 502 | 应 < 570 ✓ |
| 编码器反转后占空比 | 866 | 应 > 502 ✓ |
| UART 起始位宽度 | 869 clk | `CLK/BAUD` = 868（偏差 1 拍=边沿对齐） |
| UART 回环 | 8/8 字节正确 | — |
| 同步释放延迟 | 3 拍 | `SYNC_STAGES` = 3 |
| 编码器测速 | 48 计数/窗口 | 理论 ≈ 50（偏差来自窗口边界） |

### 5.2 覆盖的功能

| 能力 | 覆盖情况 |
|---|---|
| 编码器正交解码 + 4 倍频计数 | ✅ 正/反转、毛刺抑制、位置累计 |
| PWM 电机驱动 | ✅ 占空比标度、方向控制、使能门、死区、限幅 |
| UART 收发 | ✅ 回环、帧错误、起始位宽度、背靠背流控 |
| 复位同步 | ✅ 异步 assert、同步 release、POR 计数、POR 旁路 |
| 顶层闭环 | ✅ 编码器→测速→控制律→PWM→方向 全链路 |

### 5.3 现在还**不能**做

| 项目 | 原因 |
|---|---|
| GUI 看波形 | ✅ **可用**（设好 `LM_LICENSE_FILE` 即可） |
| 综合 / 布局布线 / 烧录 | 需在 Gowin IDE 里操作 |
| 上板验证 | 需硬件；且 `constraints/top.cst` 的引脚号**还是占位值** |
| 待实现模块 | `clk_gen`(PLL)、`main_fsm`、`motion_decision`、`tof_driver`、`imu_driver`、`servo_lift`、`uart_frame`、`display_ctrl` |

### 5.4 常用命令

```powershell
# 环境（每次新开终端）
. D:\robot_trial\sim\toolchain\start_here.ps1

# 全量重编译
cd D:\robot_trial\sim
& 'D:\modeltech64_2020.4\win64\vsim.exe' -c -do "do compile_all.do; quit -f"

# 跑全部测试
& 'D:\modeltech64_2020.4\win64\vsim.exe' -c -do "do run_sim.do"

# 看结果（读文件，不要看屏幕滚动）
Get-ChildItem *_result.txt | ForEach-Object {
  Write-Host "===== $($_.Name) ====="
  Get-Content $_.FullName | Select-String 'RESULT|checks,|\[FAIL\]'
}
```

---

## 6. 沉淀下来的方法论（建议写进团队规范）

### 6.1 用便宜的验证先跑，别用昂贵的验证试错

| 验证方式 | 成本 | 适用 |
|---|---|---|
| 静态检查工具（本工程自建 4 个） | 秒级 | 语法、声明顺序、位宽、魔法数字 |
| **Python 周期精确模型** | 秒级 | 纯时序状态机（UART 就是这么定位的） |
| ModelSim 仿真 | 分钟级 | 真正的时序/集成验证 |
| 上板 | 小时～天级 | 最后的硬件验证 |

**UART 的 3 个 bug，本可以用 Python 模型几分钟定位，却在 ModelSim 里来回跑了十几轮。**

### 6.2 关键测量值必须落盘

`$display` 在批处理模式下会被吞；**`$fwrite` 自己写文件才可靠**。
而且**不只写 PASS/FAIL，还要写实际测量值**——
`goto=4 exp=3` 这种信息量极低，`d0=3473` 才能一眼看出问题。

### 6.3 判据要写成无歧义形式

测"延迟是 N 拍"时，不要"数到信号变化"（受 NBA/active 区时序影响会差一拍），
而应验证：**"第 N-1 拍仍为旧值、第 N 拍已为新值"**。

### 6.4 位序常量一律用具名 localparam

```verilog
// ❌ 极易出错，且注释和数值容易不一致
key = 2'b01;   // 注释写着 "key[0]=0" —— 但 2'b01 的 key[0] 其实是 1！

// ✅ 用具名常量
localparam [1:0] KEY_OPEN_LOOP_ENABLE = 2'b10;  // key[1]=1 开环, key[0]=0 使能
```

### 6.5 跨工具协作要显式，不要靠"看起来对"

- 用**显式路径**复制文件，不用 `*` 通配符（嵌套目录会静默漏掉）
- 工具脚本要**检查产物是否真的生成**（不能只看退出码：
  ModelSim 找不到 `.do` 文件时只打印 `invalid command name` 却正常退出）
- **不要把"能否加载"和"优化器行为"耦合**（层次引用 `dut.xxx` 必须配 `+acc`）

---

## 7. 下一步

| 优先级 | 任务 | 说明 |
|---|---|---|
| 1 | **`clk_gen.v`（PLL）** | 用 Gowin IP Core Generator 生成；接入后把 `sys_clk` 换成 100MHz |
| 2 | **`main_fsm.v`（主状态机）** | 基础任务 4「自主导航」核心；纯状态机，可先用 Python 模型验证 |
| 3 | `motion_decision.v` | 替换 `top.v` 里的占位控制律 |
| 4 | `uart_frame.v` | 帧封装/解析 + 超时保护，打通 ARM 通信 |
| 5 | 上板调试 | **先替换 `constraints/top.cst` 的引脚占位值** |

**距提交 31 天（今天第 1 天）。工具链与驱动层的正确性已经过关，
这是项目里风险最高、最容易反复的一段。**

---

## 附：本工程自建的验证工具

| 工具 | 作用 |
|---|---|
| `tools/rtl_lint.py` | RTL 静态检查：模块配平、重复声明、未声明信号、魔法数字 |
| `tools/check_decl_order.py` | 检查"先用后声明"（`vlog-2730` 那类错误） |
| `tools/params_check.py` | 参数自检：PWM 标度、波特率误差、测速窗口换算 |
| `tools/verify_uart_model.py` | **UART 周期精确 Python 模型**，秒级验证收发逻辑 |
| `tools/add_result_logging.py` | 批量为 Testbench 添加 `$fwrite` 结果落盘能力 |
| `sim/toolchain/*.ps1` | 工具链配置、`.do` 生成、一键重编译、环境预检 |
