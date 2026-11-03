# A —— FPGA 实时控制

> **你的定位**：整条"感知 → 决策 → 驱动"链路都在 FPGA 内完成，**这是项目的技术核心**。
> **你的优势**：剩余 FPGA 工作**基本不阻塞**，可以立刻推进。

---

## 0. 当前状态（一句话）

**8 个模块 / 271 项仿真检查全部 PASS，整数链路已闭环。**
剩下的是：接上通信口、生成时钟、补外设驱动、上板。

```
已完成 ✅                        待完成 ⏳
├─ reset_sync      11/11        ├─ uart_frame 接入 top.v   ← 无阻塞，最优先
├─ pwm_motor ×4    21/21        ├─ clk_gen (PLL 100MHz)
├─ encoder_decode×4 17/17       ├─ top.cst 引脚分配        ← 等 C
├─ uart_tx/rx      17/17        ├─ servo_lift              ← 等 C 第10/11项
├─ uart_frame      58/58        ├─ tof_driver              ← 等选型
├─ main_fsm        63/63        ├─ imu_driver              ← 等选型
├─ motion_decision 32/32        ├─ display_ctrl            ← 等选型
└─ top 集成        52/52        └─ 上板 bring-up
```

---

## 1. 任务清单（按优先级）

### P0：无阻塞，立刻可做

| # | 任务 | 验收标准 | 阻塞于 |
|---|---|---|---|
| **A1** | **`uart_frame` 接入 `top.v`** | `tb_top_integration` 扩展后全 PASS，且能收发真实帧 | **无** |
| **A2** | **`clk_gen.v`**（Gowin PLL，50→100MHz） | 仿真能跑，综合无 warning | 需 Gowin IP Core Generator |
| **A3** | 更新 `doc/design_guide.md` 的 `uart_frame` 章节 | 与新代码一致 | A1 |
| **A4** | 扩展 `tb_top_integration` 覆盖 UART 回路 | 检查数从 52 增加到 ~70 | A1 |

### P1：等 C 的关键几项

| # | 任务 | 等 C 的哪一项 |
|---|---|---|
| **A5** | `servo_lift.v` | **第 10 项（`lift_done` 来源）+ 第 11 项（舵机型号）** |
| **A6** | `motion_decision` 参数标定（轮径/轮距/PPR） | 第 1~5 项 |
| **A7** | `pwm_motor` 参数标定（死区/频率） | 第 8 项 |
| **A8** | `top.cst` 填真实引脚 | 第 13 项 |

### P2：等选型

| # | 任务 | 等什么 |
|---|---|---|
| **A9** | `tof_driver.v` | TOF 型号与接口 |
| **A10** | `imu_driver.v` | IMU 型号与接口 |
| **A11** | `display_ctrl.v` | 屏型号 |

### P3：上板

| # | 任务 | 前置 |
|---|---|---|
| **A12** | LED 流水灯上板，确认板子活着 | 引脚分配 |
| **A13** | 编码器上板（手转轮子看计数） | 机械装好 |
| **A14** | 电机上板（**架空轮子**） | 驱动板接好 |
| **A15** | 急停实测（按 key[1]，看 `m_en` 是否 0 拍断） | A12 |
| **A16** | 与 B 的 ARM 板联调 UART | A1 + B 的板到货 |
| **A17** | 落地测试 | 全部 |

---

## 2. 建议的推进顺序

```
今天     A1（uart_frame 接入 top.v）  ← 收益最大，因为它让 B 能开始联调
明天     A4（扩展集成测试）+ A2（PLL）
等 C 表  A5（servo_lift）+ A6/A7（参数标定）
等选型   A9/A10/A11
```

**A1 为什么最优先**：它是**唯一同时解锁"B 能联调"和"A 能上板"的改动**。

---

## 3. 可直接粘贴给 DSH 的提示词

### 提示词 1：会话重启（**上下文被压缩后、或开新会话时用**）

```
【项目背景】
轮足复合式 FPGA 实时感知与自主控制机器人，2026 全国大学生嵌入式芯片与系统设计竞赛
（高云半导体 FPGA 创新设计赛道），本科组。
平台：高云 ACG720（GW5AST-138）/ 小梅哥开发板。
队伍三人：A 做 FPGA 实时控制（我是 A），B 做 ARM 视觉与通信，C 做机械与集成。

【架构】
摄像头(ARM) ──UART──► FPGA ──► 电机驱动板 ──► 4 个电机
                       │
     TOF/IMU ──────────┤
     编码器×4 ─────────┘
所有实时控制（传感器采集、电机闭环、避障状态机、升降腿）都在 FPGA 内完成。

【请先读这些文件，再动手】
1. doc/interface_delivery.md   —— 接口全貌与当前缺口，先读这个
2. doc/design_guide.md         —— ★ 每个模块内部怎么运转、为什么这么设计
3. doc/progress_review.md      —— ★ 踩过的坑与修复过程（避免重复踩）
4. sim/toolchain/run_tb.ps1    —— 一键仿真
5. doc/team_sync_guide.md      —— Git 工作流与接口冻结规则

【当前状态】
- 8 个模块 / 271 项仿真检查全部 PASS，整数链路已闭环
- uart_frame.v 已写完并验证 58/58，但**还没接进 top.v**（下一步就是做这个）
- top.cst 里引脚全是 PIN_xx 占位值
- 机械接口信息还没拿到，servo_lift / tof_driver / imu_driver 被阻塞

【硬约束 —— 这些是这个工程反复踩过的坑】
1. Verilog-2001，**不允许对 parameter/localparam 做位选**
   必须写 localparam [7:0] X_8 = X; 再用 X_8
2. **不要用 2'bXY 这种位序** —— 写 {1'b1, 1'b0} 明确表达
3. 所有中间运算用 32 位，防溢出（本工程出现过 4 次 32 位溢出 bug）
4. .do 文件必须是**纯 ASCII**（Tcl 按 ANSI 读，中文会乱码）
5. 含中文的 .ps1 必须有 UTF-8 BOM
6. 例化参数名写错时 vlog **不报错**，只有 vopt / vsim 才会炸 ——
   改完例化先跑 tools/check_instantiations.py
7. 验证顺序：静态检查(Python 秒级) → 行为模型(Python 秒级) → ModelSim(分钟级) → 上板
8. **不要过滤工具输出** —— 过滤会把真正的错误一起滤掉

【现在请你】
读完文件后，用不超过 25 行总结：
  ① 这个工程现在能做什么、不能做什么
  ② 下一步最该做什么、为什么
  ③ 有哪些坑我特别容易再踩一遍
然后等我确认，不要直接改代码。
```

### 提示词 2：A1 —— `uart_frame` 接入 `top.v`

```
把 rtl/app/uart_frame.v 接进 rtl/top.v。

【目标】
让 ARM 能通过 UART 把视觉结果交给 FPGA，同时 FPGA 能回传状态。

【具体要求】
1) top.v 新增 2 个端口：
      output wire uart_tx,     // 接 ARM 的 RX
      input  wire uart_rx      // 接 ARM 的 TX

2) 例化 uart_frame，参数与 tb_uart_frame.v 保持一致
   （CLK_FREQ_HZ / BAUD_RATE / FRAME_TIMEOUT_MS / MAX_PAYLOAD）

3) **接收方向**：解析 ARM 发来的 0x02 VISION_RESULT 帧
   - 载荷格式：目标数(1B) + [序号(1B) 类别(1B) x中心(2B) y中心(2B) 置信度(1B)] × n
   - 取**置信度最高**的目标（或第一个，你判断哪个更合理，说明理由）
   - 输出到 top 已有的端口：
       vision_class[7:0]  ← 目标类别
       vision_valid       ← 收到新帧时给一个脉冲或保持若干周期
       steer[15:0]        ← 由 x 中心换算：<21845 左转 / >43690 右转 / 中间直行
   - ⚠️ 注意 steer 是**有符号**的，正=右转

4) **发送方向**：周期发 0x10 TELEMETRY 帧（建议 10Hz）
   - 载荷 13 字节：状态(1) + 速度L(2) + 速度R(2) + 距离(2) + roll/pitch/yaw(2×3)
   - 速度取 encoder_decode 的 speed_cnt（有符号补码大端）
   - 距离取 dist_mm；姿态暂时填 0（IMU 还没做）

5) **失效安全**（重要）：
   - ARM 不发帧时，vision_valid 必须为 0，main_fsm 会按"无障碍但限速"处理
   - 不要因为 UART 没数据就改变现有行为

6) 同步更新：
   - constraints/top.cst 加 uart_tx/uart_rx 的占位条目
   - sim/tb_top_integration.v 扩展：用软件串口灌一个 VISION_RESULT，断言 vision_class/steer 正确
   - doc/interface_delivery.md §1.3 的"缺失接口"说明改成已完成

【约束】
- 不要改 uart_frame.v 的接口（B 侧已经按它写了 arm/uart_proto.py 并自测通过）
- 不要改 main_fsm 的行为
- 所有中间运算用 32 位

【验收】
python tools/rtl_lint.py rtl sim            -> errors=0
python tools/check_encoding.py .              -> RESULT: PASS
python tools/check_instantiations.py rtl sim -> 0 错误
powershell -File sim/toolchain/run_tb.ps1    -> 全部 tb PASS，检查数比之前多

先给我改动方案（哪些文件、加什么信号、怎么验证），我确认后再改。
```

### 提示词 3：A2 —— `clk_gen.v`（PLL）

```
生成 rtl/hal/clk_gen.v —— 把板载 50MHz 变成系统用的 100MHz。

【背景】
- 当前所有 tb 都用 100MHz（CLK_PERIOD=10ns），但板上只有 50MHz 晶振
- Gowin 的 PLL 是**原语**，不能用纯 RTL 描述，要在 Gowin IDE 里用
  IP Core Generator 生成模板，或者用 prim_sim.v 里的 PLL 原语

【要求】
1) 先不要猜写法。先做这三件事：
   a) 在 D:\Gowin\...\IDE\simlib\gw5a\prim_sim.v 里找 PLL / rPLL 的模块定义
   b) 查 doc/toolchain_gowin_modelsim.md 里关于 Gowin 原语仿真的说明
   c) 告诉我：GW5AST-138 上 PLL 的正确原语名和端口是什么
2) 给我两个方案对比：
   方案 A：手工写原语例化（可控，但要查手册）
   方案 B：用 Gowin IDE 的 IP Core Generator 生成（官方推荐，但我需要知道具体步骤）
3) 封装成 clk_gen.v，对外只暴露：
      input  clk_50m, rst_n
      output clk_sys (100MHz)
      output locked
4) 加一个"PLL 未锁定时保持复位"的处理（locked 信号接进 reset_sync）

【验收】
- ModelSim 能仿真（挂 gw5a 库，用 prim_sim.v）
- 综合无 warning

先给我调查结果和两个方案，我确认后再写代码。
```

### 提示词 4：写新模块的通用模板

```
我要写一个新的 FPGA 模块：<模块名>，功能是 <一句话描述>。

【这个工程的模块写作规范 —— 请严格遵守】
1) 文件头注释块包含：模块名、一句话功能、端口表、参数表、设计要点、已知陷阱
2) 参数化：所有位宽/频率/阈值都用 parameter，不要写魔法数字
3) 三段式状态机（组合算次态 / 时序更新 / 组合输出）或明确的组合+时序分离
4) **所有等待都必须有超时** —— 没有超时就会永久卡死
5) 组合逻辑输出和寄存器输出要分清，注释里标出来
6) 上电默认值要安全（比如电机使能默认 0）

【必须先做的验证，顺序不能变】
1) 静态检查：python tools/rtl_lint.py rtl sim
              python tools/check_encoding.py .
              python tools/check_instantiations.py rtl sim
              python tools/check_literal_overflow.py rtl sim
2) **写一个 Python 行为模型**并验证（参考 tools/verify_main_fsm_model.py 的风格）
   —— 这一步是秒级的，能抓住状态机转移和算术符号错误
3) 写 Testbench（参考 sim/tb_main_fsm.v 的风格）
   - 每个检查都要有名字、期望值、实际值
   - 结果用 $fwrite 写进 <模块>_result.txt
   - 末尾打印 "=== tb_<模块>: N checks, M errors ===" 和 "RESULT: PASS/FAIL"
   - 在 sim/toolchain/emit_dofiles.ps1 的列表里注册新文件
4) 跑 ModelSim

【注意】
- 不要跳过第 2 步直接写 Testbench —— 本工程的 motion_decision 转向符号错误
  就是 Python 模型 30 秒发现的，ModelSim 要几分钟一轮
- 例化参数名/端口名写错时 vlog 不报错，只有 vopt 才炸

请先给我：
  ① 端口列表和参数列表
  ② 状态机设计（如果有）
  ③ 打算怎么超时保护
  ④ Python 模型要验证哪些点
我确认后再写代码。
```

### 提示词 5：出问题了怎么排查

```
仿真/上板出问题了。现象是：
<贴完整的报错或现象>

【排查纪律 —— 请按这个顺序，不要跳步】
1) **先拿完整输出**，不要让我贴片段，也不要自己过滤
2) 如果是 "Error loading design"，**先跑 vopt**：
      vopt -work work -L gw5a +acc <tb> -o <tb>_opt
   vlog/vsim 都不给细节，只有 vopt 会指出真正的错误
3) 先判断是**编译期**还是**精化期（elaboration）**还是**运行期**
   - 编译期：vlog 报错，看语法
   - 精化期：vlog 通过但 vsim 说加载失败 → 参数名/端口名/位宽
   - 运行期：能跑但结果不对 → 逻辑问题
4) 如果某个 tb 失败，**先确认它的 _result.txt 是本次跑的**
   （tb 加载失败时，结果文件会保留上一次的内容 —— 本工程踩过这个坑）
5) 给我一个**最小复现**：能去掉的都去掉，剩下最小的失败案例

【重要】
不要基于"看着像原因"就改代码。本工程今天有两次因为猜错方向浪费了时间：
  - 看到"对 localparam 做位选"就认定是故障原因，结果别的模块也在用同样写法且都通过
  - 看到"参数位选"就改，结果真因是 compile_all.do 里没这个文件

先量数据，再改代码。
```

---

## 4. 你要读/维护的文件

| 文件 | 用途 |
|---|---|
| [`doc/interface_delivery.md`](../doc/interface_delivery.md) | 接口全貌；A1 完成后要更新 §1.3 |
| [`doc/design_guide.md`](../doc/design_guide.md) | 每个模块的设计说明；新模块要补章节 |
| [`doc/progress_review.md`](../doc/progress_review.md) | 缺陷复盘；修了新 bug 要追加 |
| [`doc/team_sync_guide.md`](../doc/team_sync_guide.md) | Git 工作流、接口冻结规则 |
| `sim/toolchain/emit_dofiles.ps1` | 新增 RTL/tb 要在这里注册 |
| [`verify/`](../verify/) | 每次跑完仿真提交验证报告 |

---

## 5. 答辩素材（**边做边攒，别到最后补**）

这个工程里已经积累了不少**能直接放进 PPT** 的东西：

| 素材 | 在哪 |
|---|---|
| **急停 0 拍断使能的波形** | `tb_main_fsm`，`estop` 与 `motor_enable` 同一时刻跳变 |
| **TX 状态机优先级遮蔽的 bug** | 字节间隔正常但内容全错 —— 示波器看不出来的 bug |
| **4 次 32 位溢出** | `progress_review.md` |
| **验证分层：静态→模型→仿真→上板** | `design_guide.md`，Python 模型 30 秒 vs ModelSim 几分钟 |
| **fail-safe 设计** | 视觉失效不停车、传感器故障限速、所有等待有超时 |
| **271 项仿真检查全过** | `verify/` 里的报告 |

**建议每完成一个阶段就截一张波形图存进 `doc/images/`。**

---

## 6. 给 A 的提醒

| 提醒 | 原因 |
|---|---|
| **不要等 B 和 C** | A1/A2 完全无阻塞，先做 |
| **不要为了"分工好看"把 RTL 分给别人** | 他们要先补上下文，损耗大于收益 |
| **每天开工先 `git pull`** | 见 `team_sync_guide.md` |
| **改协议必须和 B 确认** | 唯一的耦合点 |
| **跑完仿真提交 `verify/` 报告** | 让队友知道进度 |
