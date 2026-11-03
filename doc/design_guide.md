# 设计说明文档

> 目的：说清**每个模块为什么这么设计、内部怎么运转、函数/任务怎么用**。
> 面向两类读者：接手代码的组员 + 答辩时需要讲清"为什么这么设计"的场合。
>
> 配套阅读：[interface_spec.md](interface_spec.md)（接口契约）、
> [progress_review.md](progress_review.md)（缺陷复盘）、
> [team_handoff.md](team_handoff.md)（分工与阻塞项）。

---

## 目录

1. [总体设计](#1-总体设计)
2. [`reset_sync`](#2-reset_sync)
3. [`pwm_motor`](#3-pwm_motor)
4. [`encoder_decode`](#4-encoder_decode)
5. [`uart_tx`](#5-uart_tx)
6. [`uart_rx`](#6-uart_rx)
7. [`main_fsm`](#7-main_fsm)
8. [`motion_decision`](#8-motion_decision)
9. [`top`](#9-top)
10. [跨模块约定与换算](#10-跨模块约定与换算)
11. [验证体系](#11-验证体系)
12. [Testbench 里的任务](#12-testbench-里的任务)
13. [关键设计决策记录](#13-关键设计决策记录)

---

## 1. 总体设计

### 1.1 分层架构

```
┌──────────────────────────────────────────────────────────────┐
│ top.v — 只做例化与连线，不写复杂逻辑                          │
├──────────────────────────────────────────────────────────────┤
│ app/     应用层   main_fsm（行为决策）                        │
│                   motion_decision（运动决策/速度闭环）        │
├──────────────────────────────────────────────────────────────┤
│ driver/  驱动层   pwm_motor · encoder_decode · uart_tx/rx     │
├──────────────────────────────────────────────────────────────┤
│ hal/     硬件抽象 reset_sync（时序与复位）                    │
└──────────────────────────────────────────────────────────────┘
```

**分层的判据是"变更原因"**：改避障策略只动 `main_fsm`；换电机只动
`motion_decision` 的参数；换驱动板只动 `pwm_motor`。三层互不牵连。

### 1.2 数据流

```
   TOF ─┐
  视觉 ─┼─► main_fsm ──► motion_decision ──► pwm_motor ×4 ──► 驱动板 ──► 电机
  按键 ─┘   (意图)         (每轮目标速度)      (载波)
  舵机 ─┘        ▲              ▲
                 │              │
            越障反馈        encoder_decode ×4 ◄── 编码器
```

**关键点：`main_fsm` 输出的不是 PWM，而是"意图"**（`speed_scale` = 停/慢/满）。
把"要不要走"和"轮子转多快"分开，是这套设计最重要的一条边界。

### 1.3 三条贯穿全篇的主线

#### 主线 1：让错误无法藏身

今天实测数据：**15 个 bug 里 11 个是工具/测试发现的，只有 2 个靠读代码看出来。**

三招：

| 招 | 做法 | 例子 |
|---|---|---|
| **分层** | 每层单一职责，改动不扩散 | 底盘形式未定，`motion_decision` 已能写完 —— 转向只体现在 4 个系数 |
| **自证** | 代码/测试自己断言前提 | `main_fsm` 断言距离阈值递进；`tb_top_integration` 断言 key 位序 |
| **廉价验证优先** | 静态检查 → Python 模型 → ModelSim → 上板 | `motion_decision` 的转向符号错误，Python 模型 30 秒发现 |

#### 主线 2：安全逻辑不依赖上层写对

- 急停走**组合逻辑**（0 拍），不进状态机 —— 状态机跑飞时急停仍有效
- 传感器失效时**限速**而非停车 —— 既安全又不会永久卡死
- 所有等待**都有超时** —— 绝不允许状态机无限期停在某状态

#### 主线 3：绕开硬件/工具的不确定性

| 问题 | 对策 |
|---|---|
| 异步输入可能有毛刺 | 2 级同步器 + 可选数字滤波 |
| 驱动板只有一根方向脚 | 把控制量**符号拆成"幅值 + 方向脚"** |
| ModelSim 批处理下 stdout 抓不到 | Testbench 用 `$fwrite` **自己写结果文件** |
| 32 位整数会静默溢出 | 中间量一律放宽到 32 位；专门的检查工具 |

### 1.4 全局约定（改代码前必读）

#### 符号约定

| 信号 | 约定 |
|---|---|
| `speed_cnt > 0` | 该轮**正转** |
| `duty_in`（pwm_motor） | **幅值**，恒 ≥ 0；方向由 `dir_in` 单独给 |
| `dir_in / dir_out` | 1 = 正转 |
| `steer > 0` | **右转**（右转 = 左轮加速、右轮减速） |
| 所有 `rst_n` | **低有效** |
| 按键 `key` | **低有效**（0 = 按下） |

#### 位序约定（今天踩了两次的坑）

```verilog
// ❌ 2'bXY 里 X 是 bit1、Y 是 bit0，和"从左往右读"的方向相反
key = 2'b10;   // 这其实是 key[1]=1, key[0]=0

// ✅ 用拼接写法，左边就是高位，一眼能对
key = {1'b1, 1'b0};   // {key[1], key[0]}
```

#### 位宽约定

**所有中间运算放宽到 32 位有符号，只在输出时收窄。** 原因：

```verilog
115200 * 16777216 = 1.93e12   // 32 位里回绕成 0 → 硬件上一个字节都发不出去
CLK_FREQ_HZ * AVOID_TIMEOUT_MS = 50e6 * 5000 = 2.5e11  // 溢出 → 判断结果全错
#5_000_000_000                // 无尺寸字面量超 32 位 → vlog 拒绝编译
```

#### Verilog-2001 的两条硬限制

```verilog
// ① 不允许对 parameter / localparam 做位选或部分选择
wire x = DIST_STOP_MM[15:0];        // ❌ 非法
localparam [15:0] DIST_STOP_W = DIST_STOP_MM;  // ✅ 先转成定宽常量

// ② .do 文件里的注释必须是纯 ASCII（Tcl 按 ANSI 代码页读脚本）
```

---

## 2. `reset_sync`

**文件**：`rtl/hal/reset_sync.v`　**层次**：HAL

### 2.1 职责

把**异步复位**变成**异步 assert / 同步 release**，并提供一个**上电复位（POR）**
保持窗口，掩盖电源爬升和 PLL 锁定过程。

### 2.2 端口

| 方向 | 名称 | 说明 |
|---|---|---|
| in | `clk` | 同步时钟 |
| in | `arst_n` | 异步复位输入，低有效 |
| out | `rst_n` | 同步后的复位，低有效 |

### 2.3 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `SYNC_STAGES` | 3 | 同步链级数（决定释放延迟） |
| `POR_CYCLES` | 256 | 上电后额外保持的周期数；**0 = 关闭 POR** |

### 2.4 内部结构与运行逻辑

```
arst_n ──┬──────────────────────────────┐
         │                              │
         │    ┌──────────────┐          │
         └───►│ POR 计数器    │──por_done┘
              │ (POR_CYCLES) │          │
              └──────────────┘          ▼
                                   arst_n_eff = arst_n & por_ok
                                        │
                                        ▼
                       ┌───────────────────────────┐
                       │ 同步链 SYNC_STAGES 级      │
                       │ 异步 assert / 同步 release │
                       └───────────────────────────┘
                                        │
                                        ▼
                                      rst_n
```

**逐段说明**

1. **POR 计数器**
   ```
   在 arst_n 为低时清零；释放后开始计数；
   计满 POR_CYCLES 后 por_done 置 1。
   注意：por_done 在 arst_n 拉低时会被清零 ——
        所以每按一次复位键，POR 都要重新走一遍。
   ```

2. **POR 旁路（`POR_CYCLES == 0`）**
   ```verilog
   wire por_ok = (POR_CYCLES == 0) ? 1'b1 : por_done;
   wire arst_n_eff = arst_n & por_ok;
   ```
   **这里曾经有一个会让电路死锁的 bug**：
   ```verilog
   // ❌ 原写法
   reg por_done = 1'b0;              // 声明初始化器
   always @(*) por_done = 1'b1;      // 空敏感列表！且与初始化器竞争
   ```
   `always @(*)` 的 body 里没有读任何信号 → 敏感列表为空 → 只在 time 0 执行一次；
   而声明初值也在 time 0 赋值。ModelSim 里**初值赢了** → `por_done` 恒为 0
   → `arst_n_eff` 恒为 0 → **复位永不释放，整块电路锁死**。

   现在改成**用常量选择器旁路**，不依赖任何求值顺序。

3. **同步链**
   ```verilog
   always @(posedge clk or negedge arst_n_eff) begin
       if (!arst_n_eff) sync_chain <= 0;                       // 异步 assert
       else sync_chain <= {sync_chain[SYNC_STAGES-2:0], 1'b1}; // 同步 release
   end
   assign rst_n = sync_chain[SYNC_STAGES-1];
   ```
   释放要经过 `SYNC_STAGES` 个时钟沿才会传播到 `rst_n`（实测 3 拍）。

### 2.5 关键设计点

| 点 | 原因 |
|---|---|
| assert 走异步路径 | 复位必须立即生效，不能等时钟 |
| release 走同步路径 | 避免不同触发器释放时刻不一致导致亚稳态 |
| POR 可关闭 | 仿真时缩短时间；但**关掉时不能锁死**（见上面的 bug） |

---

## 3. `pwm_motor`

**文件**：`rtl/driver/pwm_motor.v`　**层次**：driver

### 3.1 职责

把**占空比幅值 + 方向**翻译成一路 PWM 载波 + 一根方向脚。
**不含任何控制律**，纯粹是执行器。

### 3.2 端口

| 方向 | 名称 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk` / `rst_n` | 1 | 时钟 / 低有效复位 |
| in | `enabled` | 1 | 0 时立即停输出（不等状态机） |
| in | `duty_in` | 16 有符号 | **幅值**（负值也按幅值处理，兼容旧用法） |
| in | `dir_in` | 1 | 1 = 正转 |
| out | `pwm_out` | 1 | PWM 载波，高有效 |
| out | `dir_out` | 1 | 方向脚（打一拍输出） |
| out | `pwm_cnt` | CNT_WIDTH | 载波计数（调试用） |
| out | `duty_actual` | 16 有符号 | 实际生效的带符号占空比 |

### 3.3 参数与推导

```verilog
PERIOD_CNT = CLK_FREQ_HZ / PWM_FREQ_HZ;    // 一个载波周期的 clk 数
DUTY_FULL  = PERIOD_CNT / DUTY_GAIN;       // duty_in 的满量程刻度
STEP_DIV   = PERIOD_CNT / DUTY_FULL;       // 每个刻度对应的 clk 数
```

**举例**（50MHz / 20kHz / GAIN=128）：`PERIOD_CNT=2500`，`DUTY_FULL=131`，
`STEP_DIV=19`。所以 `duty_in=120` → 比较值 `2280` 个 clk → 实测就是 2280。

### 3.4 内部结构与运行逻辑

```
duty_in ──► [取幅值] ──► [钳位到 DUTY_FULL] ──► [零点死区] ──► mag_want
                                                                  │
                                              ┌───────────────────┘
                                              ▼
                                     ┌─────────────────┐
                                     │ 打一拍 (mag_r)   │
                                     └─────────────────┘
                                              │
                                              ▼
                                   cmp_r = mag_r × STEP_DIV
                                   （满量程时直接钳到 PERIOD_CNT）
                                              │
                  cnt_r（自由运行的载波计数）──┼──► pwm_out = (cnt_r < cmp_r)
```

**逐段说明**

1. **载波计数器 `cnt_r`**
   ```verilog
   if (cnt_r >= PERIOD_CNT - 1) cnt_r <= 0;
   else                         cnt_r <= cnt_r + 1;
   ```

2. **指令解算（组合）`mag_want`**
   - `enabled=0` → 直接 0
   - `duty_in > 0` → 取值
   - `duty_in < 0` → 取绝对值 `(~duty_in)+1`（兼容旧调用方）
   - 钳位：`> DUTY_FULL` 截到 `DUTY_FULL`
   - 死区：`<= ZERO_DEADBAND` 直接归 0（避免极小占空比让电机嗡嗡响却不转）

3. **方向打拍 `dir_r`**
   ```verilog
   if (mag_want != 0 || !enabled) dir_r <= dir_in;   // 无输出时不换向
   ```
   **为什么加这个条件**：电机停着的时候不要动方向脚，避免驱动板在
   零占空比时产生不必要的继电器/桥臂动作。

4. **比较值 `cmp_r`（32 位保存）**
   ```verilog
   if (mag_r >= DUTY_FULL_16) cmp_r <= PERIOD_CNT;   // 100% 饱和
   else                       cmp_r <= mag_r * STEP_DIV;
   ```
   **两个坑**：
   - `mag * STEP_DIV` 若用 16 位保存会**静默溢出截断** → 中间积用 32 位
   - 满量程时若按 `DUTY_FULL*STEP_DIV` 算，得到的是 `4992` 而不是 `5000`
     → **永远到不了 100%**，所以单独钳到 `PERIOD_CNT`

5. **输出与回读**
   ```verilog
   assign pwm_out = (cnt_r < cmp_r) ? 1'b1 : 1'b0;
   assign duty_actual = dir_r ? mag_r : -mag_r;
   ```

### 3.5 为什么方向不由 duty_in 的符号决定

**普通直流驱动板只有一根方向脚**，占空比符号表达不了"倒车"。
如果直接把负 duty 当反向 PWM，电机只会**被反向 PWM 减速，不会真的反转**。

所以约定：**幅值（≥0）走 `duty_in`，方向走 `dir_in`**。

---

## 4. `encoder_decode`

**文件**：`rtl/driver/encoder_decode.v`　**层次**：driver

### 4.1 职责

增量式编码器的**正交解码（4 倍频）** + **定窗测速**。
输出带符号的位置累计和窗口速度，**不做 PID**（PID 在 `motion_decision`）。

### 4.2 端口

| 方向 | 名称 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk` / `rst_n` | 1 | 时钟 / 复位 |
| in | `enc_a` / `enc_b` | 1 | 编码器 A/B 相（**异步输入**） |
| out | `pos_cnt` | 32 有符号 | 累计位置（4 倍频计数） |
| out | `speed_cnt` | 24 有符号 | 本窗口内的净计数 |
| out | `dir` | 1 | 最近一次跳变的方向 |
| out | `speed_valid` | 1 | 窗口刷新脉冲 |
| out | `window_cnt` | 32 | 窗口净计数（调试） |

### 4.3 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `FILTER_CYCLES` | 8 | 输入滤波窗口；**0 = 关闭** |
| `SPEED_WINDOW_CYCLES` | 1_000_000 | 测速窗口长度（clk） |
| `POS_WIDTH` / `SPEED_WIDTH` | 32 / 24 | 输出位宽 |

### 4.4 内部结构与运行逻辑

```
enc_a ─►[2级同步器]─►a_s2 ─┐
enc_b ─►[2级同步器]─►b_s2 ─┴─►[数字滤波]─►ab_filt─►[4倍频查表]─►steps
                                   ▲                          │
                              [保持计数]                 ┌────┴────┐
                                                        ▼         ▼
                                                   pos_cnt    方向锁存
                                                   (累计)
                                              ┌─────────┴─────────┐
                                              ▼                   ▼
                                         fwd_acc             rev_acc
                                         (正转累加)          (反转累加)
                                              └─────────┬─────────┘
                                                        ▼
                                        窗口结束: net = fwd - rev → speed_cnt
```

**逐段说明**

1. **两级同步器**
   ```verilog
   a_s1 <= enc_a;  a_s2 <= a_s1;
   ```
   `enc_a/enc_b` 是**异步输入**（外部编码器），必须先同步再使用，否则亚稳态。

2. **数字滤波（可选）**
   ```
   新电平必须【连续保持 FILTER_CYCLES 拍】才被采信。
   电平一变就重新计时 → 短于窗口的毛刺被完全吃掉。
   FILTER_CYCLES=0 时用组合直通（仿真时可以关掉）。
   ```

3. **4 倍频正交解码查表**
   ```
   前向序列:  00 → 01 → 11 → 10 → 00
   索引 idx = {prev_ab, cur_ab}
   ```
   | 索引 | 含义 | steps |
   |---|---|---|
   | `0001 0111 1110 1000` | 前向跳变 | `2'd1` |
   | `0010 0100 1101 1011` | 后向跳变 | `2'd2` |
   | 其它 | 静止 / 非法跳变（两位同时变） | `2'd0` |

   **为什么用 2 位编码 `1=+1 / 2=-1` 而不是直接写 -1**：
   ```verilog
   // ❌ 拼接后整体当有符号数，符号扩展容易踩坑
   wire signed [31:0] step = {{31{1'b0}}, step_s};
   // 结果 -1 会被当成 +1（本工程最早就是这么错的）

   // ✅ 拆成"幅值 + 方向标志"
   wire [31:0] step_mag = (steps == 0) ? 0 : 1;
   wire        step_up  = (steps == 1);
   ```

4. **位置累计与方向锁存**
   ```verilog
   if (step_act) begin
       pos_cnt <= step_up ? (pos_cnt + step_mag) : (pos_cnt - step_mag);
       dir     <= step_up;
   end
   ```
   只有真正发生跳变时才动，静止时计数保持不变。

5. **定窗测速（正反向分开累加）**
   ```verilog
   if (step_act) begin
       if (step_up) fwd_acc <= fwd_acc + 1;
       else         rev_acc <= rev_acc + 1;
   end
   if (speed_valid) begin
       window_cnt <= net_cnt;                    // net = fwd - rev
       speed_cnt  <= net_cnt[SPEED_WIDTH-1:0];
       cyc_cnt <= 0; fwd_acc <= 0; rev_acc <= 0;
   end
   ```
   **为什么正反向分开累加，而不是用一个有符号累加器**：
   彻底避免"无符号 + 有符号"混用时的符号扩展问题。两个都是纯无符号计数，
   只在窗口结束相减时才按有符号解释。

### 4.5 速度换算

```
v(转/秒) = speed_cnt / (4 × PPR) / (WINDOW / CLK_FREQ_HZ)

例：PPR=13，4 倍频 = 52，窗口 = 10ms
    v(rps) = speed_cnt / 0.52
```

---

## 5. `uart_tx`

**文件**：`rtl/driver/uart_tx.v`　**层次**：driver

### 5.1 职责

把并行字节按 UART 帧格式（起始位 + 数据位 + 可选校验 + 停止位）串行发出。

### 5.2 端口

| 方向 | 名称 | 说明 |
|---|---|---|
| in | `clk` / `rst_n` | 时钟 / 复位 |
| in | `start` | **单拍脉冲**：启动一次发送 |
| in | `data_in` | 待发字节 |
| out | `tx` | 串行输出（空闲为高） |
| out | `busy` | 忙（**上层必须查这个再发**） |
| out | `done` | 发送完成脉冲 |

### 5.3 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `CLK_FREQ_HZ` / `BAUD_RATE` | 50M / 115200 | 决定每比特多少 clk |
| `DATA_BITS` | 8 | 数据位 |
| `PARITY_EN` | 0 | 0=无 1=偶 2=奇 |

### 5.4 内部结构与运行逻辑

```
IDLE ──start──► START ──bit_end──► DATA ──(移完 N 位)──► STOP ──bit_end──► IDLE
  │                │                 │                     │
  tx=1           tx=0              tx=shift[0]           tx=1
 busy=0         busy=1            shift <= {1'b1, shift>>1}
```

**逐段说明**

1. **波特率整数分频**
   ```verilog
   localparam integer CLK_PER_BIT    = CLK_FREQ_HZ / BAUD_RATE;
   localparam [31:0]  CLK_PER_BIT_M1 = CLK_PER_BIT - 1;
   wire bit_end = (div_cnt == CLK_PER_BIT_M1);
   ```
   **这里踩过一个致命坑**：
   ```verilog
   // ❌ 原写法：把"计数归零"当比特结束
   wire baud_tick = (div_cnt == 0);
   if (state == IDLE) div_cnt <= 0;   // 和上面组合，灾难
   ```
   进入 `START` 时 `div_cnt` 正好是 0 → `baud_tick` 立刻为真
   → **起始位只持续 1 个 clk**（应为 868）→ 整个字节 11 拍就"发完"了。

   **正确写法**：把标志放在"计满一个比特"的位置（`div_cnt == CLK_PER_BIT-1`），
   而不是"计数刚开始"的位置。

2. **状态机**
   - `IDLE`：`tx=1`；收到 `start` → 装载 `shift`，`tx=0`，进 `START`
   - `START`：等 `bit_end` → 输出 `shift[0]`，移位，进 `DATA`
   - `DATA`：每 `bit_end` 移出一位；移完 `DATA_BITS + PARITY_EN` 位 → `STOP`
   - `STOP`：`bit_end` 时 `tx=1`，`busy=0`，`done=1`，回 `IDLE`

3. **组帧**
   ```verilog
   wire [DATA_BITS:0] frame = PARITY_EN
       ? {(PARITY_EN==1 ? ^data_in : ~^data_in), data_in}
       : {1'b1, data_in};
   ```

### 5.5 关键设计点

**`busy` 必须被上层检查。** 接收器在**停止位中点**就报 done，
而发送器要等**停止位结束**才回 IDLE，中间有半个比特的窗口。
此时给 `start` 会被**静默忽略**。

这个特性导致的测试现象很有迷惑性：**字节严格交替成功/失败**。

---

## 6. `uart_rx`

**文件**：`rtl/driver/uart_rx.v`　**层次**：driver

### 6.1 职责

从 `rx` 线上异步接收 UART 帧，做 16 倍过采样、中点采样、起始位复核、
帧错误与校验错误检测。

### 6.2 端口

| 方向 | 名称 | 说明 |
|---|---|---|
| in | `rx` | 串行输入（异步） |
| out | `data_out` | 收到的字节 |
| out | `done` | 收到完整字节（单拍） |
| out | `frame_err` | 停止位不是高电平 |
| out | `parity_err` | 校验错 |
| out | `rx_busy` | 正在接收 |

### 6.3 内部结构与运行逻辑

```
rx ─►[2级同步器]─►s1,s2
                    │
            ┌───────┴──────────────────────────────┐
            │  过采样分频: OS_DIV = CLK/(BAUD×16)   │
            │  tick_cnt 0..15, 中点 = HALF-1        │
            └───────┬──────────────────────────────┘
                    ▼
IDLE ──下降沿──► RECHK ──中点仍为低──► DATA ──(收满)──► PAR ──► STOP ──► IDLE
                   │                    │              │        │
              假起始→IDLE           逐位中点采样    校验比对   停止位检查
```

**逐段说明**

1. **过采样分频**
   ```verilog
   localparam integer OS_RATE = 16;
   localparam integer OS_DIV  = CLK_FREQ_HZ / (BAUD_RATE * OS_RATE);
   localparam integer HALF    = OS_RATE / 2;
   wire os_tick = (div_cnt == 0);
   wire mid_bit = (tick_cnt == HALF - 1);
   ```
   **约束**：`OS_DIV >= 16`，即 `CLK_FREQ_HZ >= BAUD_RATE × 256`（≈29.5MHz@115200）。
   低于此值截断误差会大到接收失败（20MHz 时 `OS_DIV=10`，误差 8%，实测必失败）。

2. **起始位下降沿检测 —— 曾经写反**
   ```verilog
   // ❌ 原写法：这是【上升沿】
   if (rx_s2 == 1'b0 && rx_s1 == 1'b1)
   // ✅ 正确：2 拍前高、1 拍前低
   if (rx_s2 == 1'b1 && rx_s1 == 1'b0)
   ```
   **后果**：接收器把"比特结束"当成"起始位开始"，起始位复核必然读到高电平
   → 判假起始 → 退回 IDLE → **永远收不到任何字节**。
   而且因为 TX 看起来完全正常，这个 bug 极难查。

3. **状态机**
   - `IDLE`：`tick_cnt=0`；检测到下降沿 → `RECHK`
   - `RECHK`：等中点，若仍为低 → 确认起始位 → `DATA`；否则假起始 → `IDLE`
   - `DATA`：每个中点采样一位，`data_idx` 递增
   - `PAR`（可选）：比对校验位
   - `STOP`：中点应为高；是 → `done`；否 → `frame_err`

### 6.4 关键设计点

| 点 | 原因 |
|---|---|
| 16 倍过采样 + 中点采样 | 容忍波特率误差和边沿抖动 |
| **起始位复核（RECHK）** | 单次毛刺不会误触发一整帧 |
| 同步器 + 边沿检测 | `rx` 是异步输入 |

---

## 7. `main_fsm`

**文件**：`rtl/app/main_fsm.v`　**层次**：app

### 7.1 职责

**行为决策**：决定"现在该走、该停、该减速、还是该抬腿"。
输出的是**意图**（`speed_scale`），不是 PWM。

### 7.2 端口

| 方向 | 名称 | 说明 |
|---|---|---|
| in | `start` | 单拍脉冲：启动导航 |
| in | `estop` | **高有效急停，最高优先级** |
| in | `manual_sel` | 手动模式 |
| in | `dist_mm` / `dist_valid` / `dist_err` | TOF 测距 |
| in | `vision_class` / `vision_valid` | ARM 视觉类别 |
| in | `goal_reached` | 到达终点 |
| in | `lift_done` / `lift_fault` | 舵机反馈 |
| out | `state` | 当前状态 |
| out | `motor_enable` | 电机使能（**急停 0 拍断**） |
| out | `speed_scale` | 0=停 1=慢 2=满 |
| out | `lift_cmd` / `lift_phase` | 越障控制 |
| out | `avoid_active` / `lift_active` | 状态指示 |
| out | `error_flag` | 曾因超时/故障安全停车 |

### 7.3 状态定义

| 编码 | 状态 | 含义 |
|---|---|---|
| 3'd0 | `IDLE` | 待机，电机断使能 |
| 3'd1 | `MANUAL` | 手动模式 |
| 3'd2 | `AUTO` | 自主导航 |
| 3'd3 | `AVOID` | 避障 |
| 3'd4 | `LIFT` | 越障（3 个子阶段） |
| 3'd5 | `GOAL` | 到达终点 |
| 3'd6 | `ESTOP` | 急停 |

`LIFT` 子阶段：`UP`（抬腿）→ `CROSS`（慢速通过）→ `DOWN`（放腿）

### 7.4 内部结构与运行逻辑（三段式）

```
【段1】状态寄存器 + 4 个分阶段超时计数器
           │
           ▼
【段2】次态组合逻辑（带优先级）
           │
           ▼
【段3】输出组合逻辑（Moore，只依赖 state / lift_phase）
```

#### 段 1：状态寄存器与计时器

```verilog
state      <= nstate;
lift_phase <= nphase;

t_avoid <= cnt_avoid ? (t_avoid + 1) : 0;
t_up    <= cnt_up    ? (t_up    + 1) : 0;
t_cross <= cnt_cross ? (t_cross + 1) : 0;
t_down  <= cnt_down  ? (t_down  + 1) : 0;
```

**四个独立计数器**，每个只在对应阶段累加，离开即清零。
分成四个而不是一个，是因为 `LIFT` 的三个子阶段需要各自的超时阈值。

#### 段 2：次态逻辑（优先级）

```verilog
nstate = state;  nphase = lift_phase;

if (estop) begin
    nstate = S_ESTOP;              // ① 急停最高优先级，覆盖一切
end else begin
    case (state)
        S_IDLE  : if (manual_sel) ... else if (start) → S_AUTO
        S_MANUAL: if (!manual_sel) → S_IDLE
        S_AUTO  : if (obst_stop) → S_AVOID
                  else if (low_obst) → S_LIFT
                  else if (goal_reached) → S_GOAL
        S_AVOID : if (low_obst) → S_LIFT
                  else if (path_clear) → S_AUTO
                  else if (超时) → S_ESTOP
        S_LIFT  : case (lift_phase)
                    LP_UP   : lift_done → LP_CROSS；超时/故障 → S_ESTOP
                    LP_CROSS: path_clear 或超时 → LP_DOWN
                    LP_DOWN : lift_done → S_AUTO；超时/故障 → S_ESTOP
                  endcase
        S_GOAL  : if (start) → S_AUTO
        S_ESTOP : if (start) → S_IDLE      // 必须再按一次，防自启动
    endcase
end
```

**优先级设计**：`急停 > 越障/避障 > 正常导航 > 上层指令`

#### 段 3：输出逻辑

```verilog
// 安全默认值：一律先当"停"
motor_en_r = 0;  speed_scale = SPD_STOP;  lift_cmd = 0;

case (state)
    S_IDLE / S_GOAL / S_ESTOP : 全 0
    S_MANUAL : motor_en=1, speed=SLOW
    S_AUTO   : motor_en=1, speed = obst_slow ? SLOW : FULL
    S_AVOID  : motor_en = ~obst_stop, speed = obst_stop ? STOP : SLOW
    S_LIFT   : case (lift_phase)
                 LP_UP   : lift_cmd=1, motor_en=0, speed=STOP
                 LP_CROSS: lift_cmd=1, motor_en=1, speed=SLOW
                 LP_DOWN : lift_cmd=0, motor_en=0, speed=STOP
               endcase
endcase
```

**为什么输出用组合逻辑而不是寄存器**：
Moore 输出只依赖状态，组合实现**延迟为 0**，且不会因为多打一拍导致
"状态变了但输出还没变"的错位。

### 7.5 环境判定

```verilog
wire obst_stop  = dist_ok && (dist_r < DIST_STOP_W);    // 太近
wire obst_slow  = !dist_ok || (dist_r < DIST_SLOW_W);   // 偏近 或 读数不可信
wire path_clear = dist_ok && (dist_r > DIST_CLEAR_W);   // 通畅
wire low_obst   = vision_valid && (vision_class == VIS_LOW_OBST);
```

**`dist_ok` 的锁存逻辑（fail-safe 的核心）**：

```verilog
if (dist_valid) begin
    dist_r  <= dist_mm;  dist_ok <= 1'b1;   // 收到有效读数
end else if (dist_err) begin
    dist_ok <= 1'b0;                        // 失效 → 标为不可信
end
```

- 读数失效时 **不清旧值**（便于诊断），只清有效标志
- `!dist_ok` 被算进 `obst_slow` → **限速前进**，而不是停车
- **为什么不直接停车**：传感器一抖机器人就永久卡死。限速既安全又保可用性

**注意：TOF 读数需要"先锁存、下一拍生效"** —— 这是刻意的，
避免组合路径直穿异步传感器输入。

### 7.6 急停的 0 拍实现

```verilog
assign motor_enable = motor_en_r & ~estop;
```

**这是本模块最重要的时序特性**：

| 方案 | 延迟 | 风险 |
|---|---|---|
| 状态机进 ESTOP 再输出 | 1+ 拍 | 状态机跑飞则失效 |
| **组合与 estop 相与** | **0 拍** | 无 —— 纯组合路径 |

这也是**FPGA 相对软件方案的核心卖点**：软件方案要经过中断响应 +
任务调度 + 串口传输，延迟是毫秒级且**不确定**。

### 7.7 `error_flag` 的判定（曾经的 bug）

```verilog
// ❌ 原写法
else if ((nstate == S_ESTOP) && !estop) error_flag <= 1'b1;
```
**问题**：人为急停松开 estop 后，状态还停在 ESTOP 而 estop 已变低
→ **把"人为急停"误报成"异常停车"**。

```verilog
// ✅ 只在【进入 ESTOP 的那一拍】判定
else if ((state != S_ESTOP) && (nstate == S_ESTOP) && !estop) error_flag <= 1'b1;
```

### 7.8 参数自检

```verilog
initial begin
    if (MS_TICKS < 1) $display("... FATAL: CLK_FREQ_HZ 太低");
    if (!(DIST_STOP_MM < DIST_SLOW_MM && DIST_SLOW_MM < DIST_CLEAR_MM))
        $display("... FATAL: 距离阈值必须递进");
    if ((CLK_FREQ_HZ * 1.0 * AVOID_TIMEOUT_MS / 1000.0) >= CNT_MAX_R)
        $display("... WARN: 超时换算后超出 32 位");
end
```

**注意用实数比较** —— 自检代码自己踩过 32 位溢出的坑：
`50e6 × 5000 = 2.5e11` 在 integer 里会回绕，判断结果完全错误。

---

## 8. `motion_decision`

**文件**：`rtl/app/motion_decision.v`　**层次**：app

### 8.1 职责

把 `main_fsm` 的**意图**翻译成**每轮具体的目标速度**，用编码器反馈做比例闭环，
输出每轮的占空比幅值 + 方向。

### 8.2 端口

| 方向 | 名称 | 说明 |
|---|---|---|
| in | `motor_enable` | 来自 `main_fsm`；0 时强制 duty=0 |
| in | `speed_scale` | 0=停 1=慢 2=满 |
| in | `steer` | 16 有符号，**正 = 右转** |
| in | `spd_fb0..3` | 24 有符号，每轮速度反馈 |
| out | `duty0..3` | 16 位幅值 |
| out | `dir0..3` | 方向 |
| out | `limited` | 至少一路触发限幅 |
| out | `target0..3` | 当前目标速度（观测用） |

### 8.3 运行逻辑

```
speed_scale ──► base = 0 / VREF/2 / VREF
                  │
steer ──► bias = steer >>> STEER_SHIFT
                  │
                  ▼
        target[g] = base + bias × STEER_COEF[g]
                  │
spd_fb[g] ────────┤
                  ▼
        err  = target - fb
        prop = err >>> KP_SHIFT
        sat  = clamp(prop, ±LIMIT_DUTY)
                  │
                  ▼
        duty[g] = |sat|      dir[g] = (sat >= 0)
```

### 8.4 为什么按"每轮独立"设计

**底盘形式还没定**（差速 / 麦克纳姆 / 四轮独立）。
所以本模块不假设任何具体底盘，只提供：

```verilog
target[g] = base + bias * STEER_COEF[g]
```

| 底盘 | 系数 |
|---|---|
| 差速 | 左两轮 `+1`，右两轮 `-1`（默认） |
| 麦克纳姆 / 全向 | 按运动学矩阵改这四个系数 |

**换底盘只改参数、不改逻辑。**

### 8.5 转向符号（写反过的坑）

```verilog
// steer > 0 表示【右转】，右转要靠【左轮加速、右轮减速】
// 所以：左轮系数 = +1，右轮系数 = -1
```

**最初写成左 -1、右 +1** → `steer>0` 时左轮减速 → **转成左转，方向反了**。

**系数之和为 0**，所以转向不会附带整体加减速（自检里有断言）。

### 8.6 内部函数 `abs_duty`

```verilog
function [15:0] abs_duty;
    input signed [31:0] v;
    reg   signed [31:0] a;
    begin
        a = (v < 0) ? -v : v;      // 取绝对值
        if (a < DB32)       abs_duty = 16'd0;        // 死区
        else if (a > LIM32) abs_duty = LIMIT_W;      // 钳位
        else                abs_duty = a[15:0];      // 正常
    end
endfunction
```

**三级处理**：绝对值 → 死区 → 钳位。
死区避免"极小占空比让电机嗡嗡响却不转"。

**调用方式**（四路显式展开，不用循环）：
```verilog
duty0 = abs_duty(sat0);
duty1 = abs_duty(sat1);
duty2 = abs_duty(sat2);
duty3 = abs_duty(sat3);
```

**为什么不用 `for` 循环**：组合 `always` 里的 `for` + 循环变量在部分综合器上
行为不一致。通道少，写开反而更好查。

### 8.7 位宽策略

```verilog
// 全部中间量用 32 位有符号
wire signed [31:0] base, bias, tgt0..3, err0..3, prop0..3, sat0..3;
```

**原因**：本工程因"位宽/截断"踩了 4 次坑。中间运算放宽到 32 位，
只在最后输出 `duty` 时收窄到 16 位。

### 8.8 关使能的处理

```verilog
wire en = motor_enable && (speed_scale != SPD_STOP);
if (!en) begin duty0..3 = 0; limited = 0; end
```

**不依赖上层是否记得清零，本模块自己兜底。**

---

## 9. `top`

**文件**：`rtl/top.v`　**层次**：集成

### 9.1 职责

**只做例化与连线，不写复杂逻辑。**

### 9.2 连接结构

```
① 时钟（hal/clk_gen.v；默认旁路 50MHz，填 PLL 后 100MHz）
② reset_sync ──► rst_n
③ encoder_decode ×4（generate）
④ 按键边沿检测（key[0] 下降沿 → start 脉冲）
⑤ main_fsm
⑥ motion_decision
⑦ pwm_motor ×4（generate）
⑧ LED / 状态输出
⑨ 待接入模块的端口（全部引出）
```

### 9.3 按键信号整形

```verilog
reg key0_d;
always @(posedge sys_clk or negedge rst_n)
    if (!rst_n) key0_d <= 1'b1;      // 释放态为高
    else        key0_d <= key[0];

wire start_pulse = key0_d & ~key[0];             // 下降沿 = 按下
wire estop_sig   = ESTOP_FROM_KEY ? ~key[1] : 1'b0;
```

| 信号 | 语义 | 处理 |
|---|---|---|
| `start` | **脉冲** | 必须做边沿检测 —— 直接给电平会让状态机每拍都"启动" |
| `estop` | **电平** | 直接取反 —— 急停是"按住就一直停" |

### 9.4 `ESTOP_FROM_KEY` 参数

```verilog
parameter integer ESTOP_FROM_KEY = 1;   // 0 = 忽略按键急停
```

**为什么需要这个开关**：调试初期如果板上没焊 `key[1]`，读到的电平不确定，
会把状态机**一直顶在 ESTOP**，现象是"什么都没反应"。

### 9.5 待接入模块的端口已全部引出

`sensors`（TOF/视觉/舵机）的端口**已经在 top 上引出**，接上去即可，
**不用再改 `top.v`**。这是刻意的：避免"看起来接好了其实没接"的假集成。

### 9.6 LED 指示（上板调试顺序）

| LED | 含义 |
|---|---|
| `led[0]` | 复位释放，板子活着 |
| `led[1]` | 电机使能中 |
| `led[2]` | 正在避障或越障 |
| `led[3]` | 进入过安全停车 |

---

## 10. 跨模块约定与换算

### 10.1 速度量纲传递

```
编码器 ──(4倍频计数)──► encoder_decode.speed_cnt
                              │  单位：计数/窗口
                              ▼
                    motion_decision.spd_fb
                              │  与 VREF 同量纲比较
                              ▼
                       duty（归一化占空比刻度）
                              │
                              ▼
                       pwm_motor.duty_in
                              │  × STEP_DIV
                              ▼
                        比较值（clk 数）
```

**关键**：`VREF`（满速目标）必须与 `encoder_decode` 的 `speed_cnt` **同量纲**。
两者都改时要注意同步。

### 10.2 典型参数链（50MHz 基准）

| 环节 | 参数 | 值 |
|---|---|---|
| PWM | `PWM_FREQ_HZ` | 20 kHz → `PERIOD_CNT=2500` |
| PWM | `DUTY_GAIN` | 128 → `DUTY_FULL=131`，`STEP_DIV=19` |
| 测速 | `SPEED_WINDOW_CYCLES` | 500000（10ms） |
| 决策 | `VREF` | 60（满速计数/窗口） |
| 决策 | `KP_SHIFT` | 1（增益 1/2） |
| 决策 | `LIMIT_DUTY` | 120 |

**验算**：静止时 `err=60`，`prop=30`，`duty=30`，比较值 `30×19=570` clk。
**实测 570，完全吻合。**

---

## 11. 验证体系

### 11.1 四层验证（成本递增）

| 层 | 工具 | 成本 | 能抓什么 |
|---|---|---|---|
| 1 | 静态检查（Python） | 秒 | 语法、命名、位宽、参数名、隐式线网 |
| 2 | **行为模型（Python）** | 秒 | 状态机转移、算术与符号约定 |
| 3 | ModelSim 仿真 | 分钟 | 真实时序、模块间接口、复位 |
| 4 | 上板 | 小时~天 | 电气、机械、真实传感器 |

### 11.2 为什么要有第 2 层

`main_fsm` 和 `motion_decision` 都是**先用 Python 模型跑通**才碰 ModelSim 的。

**实际收益**：

| 问题 | 若直接跑 ModelSim | 实际（Python 模型） |
|---|---|---|
| `motion_decision` 转向符号写反 | 一堆零散 FAIL，来回好几轮 | **30 秒定位** |
| `main_fsm` `error_flag` 误报急停 | 同上 | **场景 5 直接暴露** |

### 11.3 静态检查工具清单

| 工具 | 检查内容 |
|---|---|
| `rtl_lint.py` | 模块配平、重复声明、未声明信号、魔法数字 |
| `check_encoding.py` | 编码/BOM/纯ASCII（.ps1 缺 BOM 会报假语法错；.do 必须 ASCII） |
| `check_decl_order.py` | "先用后声明"（`vlog-2730` 那类） |
| `check_literal_overflow.py` | **32 位溢出**（字面量、常量乘法、延迟值） |
| `check_instantiations.py` | **例化参数名/端口名**（`vopt-2732` 那类） |
| `find_undeclared.py` | 隐式线网（会导致 elaboration 失败） |
| `params_check.py` | PWM 标度、波特率误差、测速窗口换算 |

**`check_instantiations.py` 是最有价值的一个** —— 它拦住的那类错误，
`vlog` 报 `Errors: 0`，`vsim` 只给一句 `Error loading design`。

### 11.4 测试结果的可见性设计

```verilog
integer log_fd;
initial log_fd = $fopen("<tb>_result.txt", "w");
// 检查时同时写文件
task check; ...
    $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
endtask
```

**为什么不用 `$display`**：ModelSim 批处理模式下 `$display` 走 stdout，
管道经常抓不到 → 跑完了但看不到 PASS/FAIL。

**关键测量值也必须落盘**：`got=4 exp=3` 信息量极低；
`d0=3473`（=4×868）才能一眼看出"数错了 4 个比特"。

---

## 12. Testbench 里的任务

### 12.1 通用任务

| 任务 | 作用 | 用法 |
|---|---|---|
| `check(name, got, exp)` | 断言并落盘 | `check("3.1 state=AUTO", state_dbg, S_AUTO);` |
| `step` | 推一拍（等时钟沿 + `#1` 让 NBA 生效） | `step;` |

**`step` 里的 `#1` 很关键**：寄存器在 `posedge` 的 **NBA 区**更新，
如果在同一拍的 active 区就读，会读到旧值。这个坑导致过
"同步释放延迟测出 4 拍而不是 3 拍"。

### 12.2 `tb_top_integration` 的任务

| 任务 | 作用 | 实现要点 |
|---|---|---|
| `feed_dist(mm)` | 送一次有效测距读数 | 拉高 `dist_valid` 一拍；**下一拍才生效** |
| `press_start` | 按一次 key[0] | 拉低 3 拍再拉高，产生下降沿脉冲 |
| `measure_pwm0` | 测一个载波周期的高电平拍数 | 先等上升沿对齐，再数 `PERIOD_CNT` 拍 |
| `drive_enc(a,b)` | 驱动正交编码器一相 | 保持 `STEP_CYCLES` 拍 |
| `enc_fwd(n)` | 正转 n 步（4 相循环） | `00→01→11→10` |

**`measure_pwm0` 为什么先等上升沿**：不等的话测到的窗口跨越两个载波周期，
拍数会有偏差。

### 12.3 测量时的并发问题

```verilog
// ❌ 错：先转完再测
enc_fwd(400);
measure_pwm0;                    // 测速窗口已归零 → 测到静止值

// ✅ 对：fork 让驱动和测量并行
fork
    enc_fwd(400);
    begin
        repeat (SPEED_WIN + 100) step;
        measure_pwm0;
        duty_moving = high_cnt;
    end
join
```

### 12.4 位序自检（今天加的）

```verilog
check("C3 KEY_ESTOP 的 bit1=0 (key[1] 按下 = 急停)",
      (KEY_ESTOP >> 1) & 1'b1, 0);
check("C4 KEY_ESTOP 的 bit0=1 (key[0] 松开，不能误触发启动)",
      KEY_ESTOP         & 1'b1, 1);
```

**为什么加**：位序错误在本工程踩了两次，且都是"编译通过、跑通、结果莫名其妙"。
**与其靠人读对，不如让 tb 自己断言。**

**注意用移位+掩码取位** —— Verilog-2001 不允许 `localparam` 位选。

---

## 13. 关键设计决策记录

| # | 决策 | 备选方案 | 为什么选它 |
|---|---|---|---|
| 1 | `main_fsm` 输出"意图"而非 PWM | 直接算占空比 | 改避障策略不影响电机标度 |
| 2 | 急停走组合逻辑 | 状态机进入 ESTOP | 0 拍延迟；状态机跑飞时仍有效 |
| 3 | 传感器失效时限速 | 停车 | 不会因一次丢数就永久卡死 |
| 4 | 所有等待都有超时 | 无限等待 | 卡死比报错更糟 |
| 5 | 符号拆成"幅值+方向脚" | duty 带符号 | 驱动板只有一根方向脚 |
| 6 | 测速正反向分开累加 | 一个有符号累加器 | 避开符号扩展问题 |
| 7 | `motion_decision` 按每轮独立设计 | 直接写差速 | 底盘形式未定，换底盘不改逻辑 |
| 8 | 中间量全用 32 位 | 按需收窄 | 位宽坑踩了 4 次 |
| 9 | Testbench 用 `$fwrite` 落盘 | `$display` | 批处理下 stdout 抓不到 |
| 10 | 关键测量值也落盘 | 只写 PASS/FAIL | 否则事后无法定位 |
| 11 | 判据用无歧义写法 | 数到信号变化 | 受 NBA/active 区时序影响 |
| 12 | 位序常量用 `{}` 拼接 | `2'bXY` | 拼接左边就是高位，不会读反 |
| 13 | 代码自断言前提条件 | 靠人记住 | 配错时立刻暴露 |
| 14 | 先用 Python 模型验证 | 直接跑 ModelSim | 秒级 vs 分钟级 |
| 15 | 静态检查先于编译 | 靠编译报错 | `vlog` 查不出参数名/端口名 |

---

## 附录：常用命令

```powershell
# 环境（新开终端）
. D:\robot_trial\sim\toolchain\start_here.ps1

# 一键：预检 → 清库 → 编译 → 运行 → 结果
cd D:\robot_trial\sim
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1 tb_top_integration
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1              # 全部

# 静态检查（不需要 ModelSim）
cd robot_project
python tools\check_instantiations.py rtl sim
python tools\check_literal_overflow.py rtl sim
python tools\rtl_lint.py rtl sim
python tools\check_decl_order.py
python tools\params_check.py

# 行为模型（不需要 ModelSim）
python tools\verify_main_fsm_model.py
python tools\verify_motion_decision_model.py
python tools\verify_uart_model.py --clk 100000000

# 遇到 "Error loading design" 时 —— 用 vopt 拿真因
& 'D:\modeltech64_2020.4\win64\vopt.exe' -work work -L gw5a +acc tb_xxx -o tb_xxx_opt
```

---

## 附录：`Error loading design` 排查流程

这句话**不给行号、不给名字**。今天被它拦了 4 次，真因分别是：

| # | 真因 | 特征 | 怎么发现 |
|---|---|---|---|
| 1 | 未声明标识符（隐式线网） | `$display` 里引用了没声明的名字 | 逐行读 + `find_undeclared.py` |
| 2 | 无尺寸字面量超 32 位 | `#5_000_000_000` | 单独 `vlog` 该文件 |
| 3 | `$argv` 传参被 vsim 污染 | `RUN -c` | 看日志里的 tb 名 |
| 4 | **例化参数名写错** | `.DUTY_WIDTH` 应为 `.CNT_WIDTH` | **`vopt`** |

**排查顺序**：

```
1. 用 vopt 拿真因        ← 最快，先做这个
2. 单独 vlog 可疑文件     ← 抓编译期问题
3. 检查 tb 是否真的加载了（看结果文件是新的还是旧的）
4. 静态检查工具跑一遍
```

**第 3 条特别重要**：加载失败时**结果文件保持上一次的旧内容**，
很容易误以为"通过了"。**必须核对检查项数量是否与当前 tb 一致。**
