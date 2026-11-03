# FPGA 模块接口说明

> 工程：轮足复合式 FPGA 实时感知与自主控制机器人
> 平台：小梅哥 ACG720 / **GW5AT-138**（`GW5AT-LV138PG484AC1/I0`，PBGA484A 封装）
> 　　⚠️ 是 **GW5AT**，不是 GW5AST —— 两个器件 ID 差一位（0x0011081B vs 0x0001081B），
> 　　定案过程见 `doc/board_pinout.md` 第 2 节
> 约定：所有模块统一 `clk` + `rst_n`（低有效，异步复位同步释放）

本文档是"接口先行"的落地物：**先在这里定死接口，再写代码**。
任何接口变更必须先改本文档，再改 RTL，并在每日站会上同步给 A/B/C 三方。

---

## 1. 全局约定

| 项目 | 约定 |
|---|---|
| 时钟 | 统一 `sys_clk`，由 `hal/clk_gen.v` 产生。默认旁路 = 板载 50MHz；填好 PLL 后 100MHz。**换频率时必须同步改 `top.CLK_FREQ_HZ`**，有断言盯着 |
| 复位 | `rst_n` 低有效，由 `hal/reset_sync.v` 产生（异步 assert / 同步 release） |
| 命名 | 输入 `i_`/无前缀，输出 `o_` 或无前缀；本工程统一不加前缀，靠端口方向区分 |
| 参数化 | 时钟频率、波特率、PWM 频率、窗口长度一律 `parameter`，禁止魔法数字 |
| 厂商隔离 | 只有 `hal/` 下的文件可以例化 `gowin_pll` 等原语，业务模块一律不碰 |
| 跨时钟域 | 必须用双时钟 FIFO 或至少 2 级同步器，**禁止**直接打拍 |
| 状态机 | 三段式（状态寄存器 / 次态组合 / 输出逻辑），必须有 `default` 分支 |
| 验证 | 每个模块先过 Testbench 仿真再上板 |

### 1.1 已实现模块清单

| 模块 | 路径 | Testbench | 状态 |
|---|---|---|---|
| `reset_sync` | `rtl/hal/reset_sync.v` | `sim/tb_reset_sync.v` | RTL 完成 · TB 就绪 |
| `pwm_motor` | `rtl/driver/pwm_motor.v` | `sim/tb_pwm_motor.v` | RTL 完成 · TB 就绪 |
| `encoder_decode` | `rtl/driver/encoder_decode.v` | `sim/tb_encoder_decode.v` | RTL 完成 · TB 就绪 |
| `uart_tx` | `rtl/driver/uart_tx.v` | `sim/tb_uart.v` | RTL 完成 · TB 就绪 |
| `uart_rx` | `rtl/driver/uart_rx.v` | `sim/tb_uart.v` | RTL 完成 · TB 就绪 |
| `top`（集成） | `rtl/top.v` | `sim/tb_top_integration.v` | 链路已接好 |

> "TB 就绪" = 自检 Testbench 已写好且含 PASS/FAIL 判定，但**尚未在仿真器里实际执行**
> （开发机 ModelSim 受中文路径限制跑不起来）。首次回归请以仿真输出为准，
> 有不通过项先查 Testbench 的采样相位与测量窗口，再查 RTL。

---

## 2. 各模块接口

### 2.1 `reset_sync`

```
reset_sync #(
    .SYNC_STAGES (3),      // 同步链级数
    .POR_CYCLES  (256)     // 上电后额外拉长复位的周期数, 0=关闭
) u_reset_sync (
    .clk    (sys_clk),
    .arst_n (arst_n),      // 异步复位输入, 低有效
    .rst_n  (rst_n)        // 同步释放后的复位, 低有效
);
```

**约束：** 所有业务模块的复位必须来自本模块输出，模块内部不得自行同步复位。

---

### 2.2 `pwm_motor`（单路直流电机）

```
pwm_motor #(
    .CLK_FREQ_HZ   (50_000_000),  // 系统时钟
    .PWM_FREQ_HZ   (20_000),      // 载波频率
    .DUTY_GAIN     (128),         // duty_in 满量程缩放系数
    .CNT_WIDTH     (16),
    .ZERO_DEADBAND (0)            // 零点死区, 上板实测后填
) u_pwm_motor (
    .clk         (sys_clk),
    .rst_n       (rst_n),
    .enabled     (en),            // 0 = 强制停机
    .duty_in     (duty),          // signed [15:0], 只表示幅值
    .dir_in      (dir_cmd),       // 方向指令: 1 = 正转
    .pwm_out     (pwm),
    .dir_out     (dir),           // 方向脚输出(= 生效的 dir_in)
    .pwm_cnt     (),
    .duty_actual ()
);
```

> **方向为什么不看 duty_in 的符号？**
> 普通直流驱动板只有**一根**方向脚 + 一根 PWM 脚。若把负的 duty 解释成
> "反向 PWM"，电机只会被反向电压减速，不会真的倒车。
> 所以本工程约定：`duty_in` 只给幅值（恒 ≥ 0），方向由 `dir_in` 单独给。
> 上层做 PI 时，把输出拆成"幅值 + 符号"两步再分别送进来。

**占空比语义（关键，上层写控制律必须按这个来）：**

| 量 | 计算式 |
|---|---|
| 载波周期 | `PERIOD_CNT = CLK_FREQ_HZ / PWM_FREQ_HZ` |
| 每单位 duty 对应的 clk 数 | `STEP_DIV = PERIOD_CNT / DUTY_GAIN` |
| duty 满量程 | `DUTY_FULL = PERIOD_CNT / STEP_DIV` |
| 实际比较值 | `cmp = duty_in * STEP_DIV`（满量程时饱和到 `PERIOD_CNT`，保证 100% 可达） |

50MHz / 20kHz / DUTY_GAIN=128 时：`PERIOD_CNT=2500`，`STEP_DIV=19`，`DUTY_FULL=131`。
所以 `duty_in=65` 约等于 50% 占空比。

**行为保证：**

1. `enabled=0` 时输出恒 0，`dir_out` 跟随 `dir_in`（便于上电即确定方向）。
2. `duty_in=0` 时输出恒 0，且**不换向**（防止 0 附近抖动打齿）。
3. `|duty_in| <= ZERO_DEADBAND` 时输出 0（用于抵消驱动板的零点漂移）。
4. `duty_in` 超量程时钳位，不会回绕。
5. `duty_in` 为负时按**绝对值**当幅值处理（兼容旧写法），符号不影响 `dir_out`。

---

### 2.3 `encoder_decode`（正交解码 + 测速）

```
encoder_decode #(
    .FILTER_CYCLES       (8),          // 输入滤波窗口; 0=关闭
    .SPEED_WINDOW_CYCLES (500_000),    // 测速窗口 (clk 周期)
    .POS_WIDTH           (32),
    .SPEED_WIDTH         (24)
) u_encoder_decode (
    .clk         (sys_clk),
    .rst_n       (rst_n),
    .enc_a       (enc_a),        // 异步输入
    .enc_b       (enc_b),
    .pos_cnt     (pos_cnt),      // signed [31:0] 累计位置
    .speed_cnt   (speed_cnt),    // signed [23:0] 窗口内 4 倍频计数
    .dir         (dir),          // 1 = 正转
    .speed_valid (speed_valid),  // 窗口刷新脉冲, 1 clk
    .window_cnt  ()              // 调试用
);
```

**速度换算公式（交给上层或 ARM 换算）：**

```
转速(rps) = speed_cnt / (4 * PPR * 减速比) / (SPEED_WINDOW_CYCLES / CLK_FREQ_HZ)
```

例：PPR=13、4 倍频=52、窗口 10ms → `rps = speed_cnt / 0.52`。

**行为保证：**

1. `enc_a/enc_b` 先过 2 级同步器，再过滤波，**不会**直接进业务逻辑。
2. `FILTER_CYCLES > 0` 时，单相电平必须连续保持该周期数才被采信。
   因此**每相脉冲宽度必须大于 `FILTER_CYCLES`**，否则会被当作毛刺滤掉。
   低速时这一点尤其要注意：算一下最低速下每相宽度再定 `FILTER_CYCLES`。
3. 静止时 `speed_cnt` 为 0，不会因非法跳变累积计数。

---

### 2.4 `uart_tx` / `uart_rx`

```
uart_tx #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD_RATE   (115_200),
    .DATA_BITS   (8),
    .PARITY_EN   (0),          // 0=无 1=偶 2=奇
    .ACC_WIDTH   (24)
) u_uart_tx (
    .clk (sys_clk), .rst_n (rst_n),
    .start (tx_start),         // 单周期脉冲
    .data_in (tx_byte),
    .tx (tx_line), .busy (tx_busy), .done (tx_done)
);

uart_rx #(
    .CLK_FREQ_HZ (50_000_000),
    .BAUD_RATE   (115_200),
    .DATA_BITS   (8),
    .PARITY_EN   (0)
) u_uart_rx (
    .clk (sys_clk), .rst_n (rst_n),
    .rx (rx_line),
    .data_out (rx_byte), .done (rx_done),
    .frame_err (rx_frame_err), .parity_err (rx_parity_err), .rx_busy (rx_busy)
);
```

**行为保证：**

1. 波特率用累加器做分数分频，`CLK_FREQ_HZ/BAUD_RATE` 除不尽也不会帧内漂移。
2. `uart_rx` 用 16 倍过采样 + 比特中点判决，起点后第 8 拍复核起始位。
3. `done` 为单周期脉冲，且**仅在无 `frame_err`/`parity_err` 时**给出。
4. 参数限制：必须满足 `CLK_FREQ_HZ / (BAUD_RATE * 16) >= 1`，
   否则过采样退化为每拍一 tick，可靠余量不足（例：1MHz + 115200 不满足，
   只能用于仿真加速，不能上板）。

---

## 3. 待实现模块的接口约定（先定死，再实现）

### 3.1 `tof_driver`　✅ 已实现（`rtl/driver/tof_driver.v`）

```
tof_driver #(
    .CLK_FREQ_HZ     (50_000_000),
    .SAMPLE_MS       (50),        // 采样周期, 单位 **ms**（名字是 SAMPLE_MS）
    .TIMEOUT_MS      (100),       // 单次采样总超时
    .I2C_SCL_HZ      (400_000),   // 透传给 i2c_master
    .I2C_TIMEOUT_MS  (10),        // 单笔 I2C 传输超时
    .BOOT_WAIT_MS    (10),        // 上电/释放 XSHUT 后的等待
    .DIST_MAX_MM     (2000),      // 超过它判为无效读数
    .DEV_ADDR        (7'h29)      // VL53L0X 出厂 7 位地址
) u_tof (
    .clk (sys_clk), .rst_n (rst_n),
    .dist_mm    (dist_mm),        // [15:0] 距离, 单位 mm（保持最近一次**有效**值）
    .dist_valid (dist_valid),     // 单拍脉冲：本次读数有效
    .dist_err   (dist_err),       // 单拍脉冲：超时/无应答/读数不合理
    // 物理接口：I2C（开漏，需外部上拉）
    .scl (i2c_scl), .sda (i2c_sda)
);
```

**与本文档早期版本的两点差异**（看代码时以代码为准）：

| 早期写法 | 实际实现 | 原因 |
|---|---|---|
| `SAMPLE_PERIOD (5_000_000)`（周期用**时钟数**） | `SAMPLE_MS (50)`（周期用 **ms**） | 时钟数要跟着 `CLK_FREQ_HZ` 改，换 PLL 时极易漏改；ms 更不容易错 |
| 只有 `CLK_FREQ_HZ / SAMPLE_PERIOD / DIST_MAX_MM` | 多了 `TIMEOUT_MS` / `I2C_*` / `BOOT_WAIT_MS` / `DEV_ADDR` | 所有等待都必须有超时（项目约定）；`DEV_ADDR` 是为将来第二块 TOF 改地址留的 |

**在 top.v 里的接法**：见 `rtl/top.v` §1.5 —— 真实驱动由 `TOF_ENABLE=1`（默认）
例化；`TOF_ENABLE=0` 时退回 `DIST_STUB_MM` 常量桩（只用于分段联调）。

**读数失效时的行为**（这是安全相关的一条，别改）：
`dist_err` 只清"数据可信"标志，**不刷新 `dist_mm`**，`main_fsm` 侧按"限速"
处理而不是急停 —— 否则 TOF 偶尔读个 0 就会把正常行驶的车一巴掌拍停。

### 3.2 `imu_driver`

```
imu_driver #(
    .CLK_FREQ_HZ (50_000_000),
    .SAMPLE_PERIOD (500_000)      // 100Hz
) u_imu (
    .clk (sys_clk), .rst_n (rst_n),
    .roll  (roll),                // signed [15:0], 0.01 度/LSB
    .pitch (pitch),
    .yaw   (yaw),
    .imu_valid (imu_valid)
);
```

### 3.3 `servo_lift`（升降腿舵机）

```
servo_lift #(
    .CLK_FREQ_HZ (50_000_000),
    .PWM_FREQ_HZ (50),            // 舵机标准 50Hz
    .MIN_US      (500),           // 0.5ms
    .MAX_US      (2500)           // 2.5ms
) u_servo (
    .clk (sys_clk), .rst_n (rst_n),
    .angle_cmd (angle_cmd),       // [15:0] 0..1000 对应 0..180 度
    .servo_pwm (servo_pwm)
);
```

### 3.4 `main_fsm`（主状态机）

文件：`rtl/app/main_fsm.v`　Testbench：`sim/tb_main_fsm.v`
RTL 已完成，**5/5 仿真通过**。

#### 3.4.1 状态定义（必须与 PPT/报告一致）

| 编码 | 状态 | 含义 |
|---|---|---|
| 3'd0 | `IDLE` | 上电待机，电机使能关闭 |
| 3'd1 | `MANUAL` | 遥控/按键手动控制 |
| 3'd2 | `AUTO` | 自主导航，按路径点行进 |
| 3'd3 | `AVOID` | 避障：减速/停车/绕行 |
| 3'd4 | `LIFT` | 越障：抬腿/跨越/放下 |
| 3'd5 | `GOAL` | 到达终点，停车 |
| 3'd6 | `ESTOP` | 急停：立即断使能 |

`LIFT` 内部子阶段：

| 编码 | 子阶段 | 动作 |
|---|---|---|
| 2'd0 | `UP` | 抬腿。`lift_cmd=1`，电机停 |
| 2'd1 | `CROSS` | 慢速通过。`lift_cmd=1` 保持抬起，电机慢速 |
| 2'd2 | `DOWN` | 放腿。`lift_cmd=0`，电机停 |

#### 3.4.2 端口

| 方向 | 名称 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk` / `rst_n` | 1 | 同步时钟 / 低有效复位 |
| in | `start` | 1 | 单拍脉冲：启动导航 |
| in | `estop` | 1 | **高有效急停，最高优先级** |
| in | `manual_sel` | 1 | 手动模式选择 |
| in | `dist_mm` | 16 | TOF 距离（mm） |
| in | `dist_valid` | 1 | 单拍脉冲：本次读数有效 |
| in | `dist_err` | 1 | 单拍脉冲：测距失败 |
| in | `vision_class` | 8 | ARM 视觉类别码 |
| in | `vision_valid` | 1 | 单拍脉冲：本帧结果有效 |
| in | `goal_reached` | 1 | 到达终点 |
| in | `lift_done` | 1 | 舵机到位脉冲 |
| in | `lift_fault` | 1 | 舵机故障 |
| out | `state` | 3 | 当前状态 |
| out | `motor_enable` | 1 | 电机使能（**急停 0 拍断**） |
| out | `speed_scale` | 2 | 0=停 1=慢 2=正常（**意图**，非最终 PWM） |
| out | `lift_cmd` | 2 | 1=抬腿 0=放腿 |
| out | `lift_phase` | 2 | 越障子阶段 |
| out | `avoid_active` / `lift_active` | 1 | 状态指示（给显示/上报） |
| out | `error_flag` | 1 | 因超时/故障进入过安全停车 |

> **职责边界**：本模块只做**行为决策**（走/停/减速/抬腿），
> 输出的是**意图**。具体"轮子转多快、PID 怎么算"由 `motion_decision` 负责。

#### 3.4.3 关键行为约定

| 编号 | 约定 | 为什么 |
|---|---|---|
| B1 | **急停 0 拍断使能**：`motor_enable = 内部使能 & ~estop`（纯组合） | FPGA 硬实时的核心卖点；软件方案要经中断+调度+串口，延迟毫秒级且不确定 |
| B2 | 急停解除后**必须再按一次 `start`** 才恢复 | 防止意外自启动 |
| B3 | 传感器失效时 **fail-safe 限速**（不是停车） | 既不冒进，也不会因一次读数丢失就永久停机 |
| B4 | 所有等待都有**超时保护** → 进 `ESTOP` | 绝不允许状态机无限期停在某个状态 |
| B5 | `error_flag` 只在**进入 ESTOP 的那一拍**判定，且排除人为急停 | 否则急停松开后会把人为急停误报成异常 |
| B6 | TOF 读数**先锁存、下一拍生效** | 避免组合路径直穿传感器输入（防毛刺） |

#### 3.4.4 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `CLK_FREQ_HZ` | 50_000_000 | 系统时钟 |
| `DIST_STOP_MM` | 200 | 低于此值 → 停车避障 |
| `DIST_SLOW_MM` | 500 | 低于此值 → 减速 |
| `DIST_CLEAR_MM` | 800 | 高于此值 → 路径通畅 |
| `AVOID_TIMEOUT_MS` | 5000 | 避障超时 → 安全停车 |
| `LIFT_TIMEOUT_MS` | 3000 | 抬腿/放腿超时 → **降级为绕行**（AVOID），连续 3 次才停车 |
| `LIFT_LOCKOUT_MS` | 3000 | 失败后禁止再抬腿的时长 |
| `LIFT_CLEAR_MS` | 5000 | 干净多久把失败计数清零 |
| `LIFT_RETRY_MAX` | 3 | 连续失败几次才真进 ESTOP |
| `CROSS_TIMEOUT_MS` | 2000 | 跨越超时 → 收腿 |
| `VIS_LOW_OBST` | 8'h04 | 低矮障碍类别码 |

> **约束**：必须满足 `DIST_STOP_MM < DIST_SLOW_MM < DIST_CLEAR_MM`。
> RTL 里有 elaboration 期自检，配错会在编译时提示。

#### 3.4.5 状态转移图

```
                    start
        ┌──────────────────────────┐
        │                          ▼
     ┌──────┐  manual_sel      ┌──────┐  goal_reached  ┌──────┐
     │ IDLE │◄────────────────►│ AUTO │───────────────►│ GOAL │
     └──────┘                  └──────┘                └──────┘
        ▲                       │   ▲                      │
        │ start(解除后)          │   │ path_clear           │ start
        │                  obst_stop│  │                      │
        │                       ▼   │                       │
        │                   ┌───────┴─┐  low_obst           │
        │                   │  AVOID  │─────────┐           │
        │                   └─────────┘         │           │
        │                       │               ▼           │
        │            avoid_timeout          ┌─────────┐      │
        │                       │           │  LIFT   │      │
        │                       │           │ UP→CROSS│      │
        │                       │           │  →DOWN  │      │
        │                       │           └─────────┘      │
        │                       │                │           │
        │                       ▼                ▼           │
        │                   ┌────────────────────────┐       │
        └───────────────────│        ESTOP           │◄──────┘
                   start    └────────────────────────┘
                            （estop 信号 / 任何超时 / 舵机故障）
```

---

### 3.5 `motion_decision`（运动决策）

文件：`rtl/app/motion_decision.v`　Testbench：`sim/tb_motion_decision.v`

在链路中的位置：

```
sensors ─► main_fsm ─► motion_decision ─► pwm_motor ─► 电机
(TOF/视觉)  (行为决策)   (本模块: 速度闭环)   (载波生成)
                              ▲
                        encoder_decode
                        (每轮速度反馈)
```

`main_fsm` 给的是**意图**（停/慢/满），本模块把它变成**每轮具体的目标速度**，
用编码器反馈做比例闭环，输出每轮的 duty 幅值 + 方向脚。

#### 3.5.1 端口

| 方向 | 名称 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk` / `rst_n` | 1 | 时钟 / 复位 |
| in | `motor_enable` | 1 | 来自 `main_fsm`，0 = 强制 duty=0 |
| in | `speed_scale` | 2 | 来自 `main_fsm`：0=停 1=慢 2=满 |
| in | `steer` | 16 有符号 | 转向指令，**正 = 右转** |
| in | `spd_fb0..3` | 24 有符号 | 每轮速度反馈（来自 `encoder_decode`） |
| in | `spd_valid` | 1 | 测速窗口更新脉冲 |
| out | `duty0..3` | 16 | 每轮占空比**幅值** |
| out | `dir0..3` | 1 | 每轮方向（1=正转） |
| out | `limited` | 1 | 至少一路触发限幅 |
| out | `target0..3` | 16 有符号 | 当前目标速度（便于观测/上报） |

#### 3.5.2 控制律

```
base   = (speed_scale==2) ? VREF : (speed_scale==1) ? VREF>>>1 : 0
bias   = steer >>> STEER_SHIFT              // 算术右移，负数保号
target[g] = base + bias * STEER_COEF[g]
duty[g]   = clamp( (target[g] - spd_fb[g]) >>> KP_SHIFT, ±LIMIT_DUTY ) 的绝对值
dir[g]    = 该命令的符号
```

用**算术右移**代替除法：省 DSP、延迟固定 —— 这是 FPGA 相对软件方案的一个卖点。

#### 3.5.3 转向符号约定（**极易搞反，务必照此实现**）

| 要求 | 实现 |
|---|---|
| `steer > 0` 表示**右转** | 右转 = **左轮加速、右轮减速** |
| 所以左轮系数 = **+1**，右轮系数 = **-1** | `STEER_COEF0/1 = +1`（左），`STEER_COEF2/3 = -1`（右） |

> ⚠️ 最初写成左 -1、右 +1，结果 `steer>0` 时左轮减速 —— **转成左转，方向反了**。
> 系数之和为 0，所以转向不会附带整体加减速。

#### 3.5.4 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `VREF` | 60 | 满速对应的编码器净计数/窗口 |
| `KP_SHIFT` | 1 | `duty = err >>> KP_SHIFT`，越大增益越小 |
| `LIMIT_DUTY` | 120 | 每轮占空比上限（标度值） |
| `ZERO_DEADBAND` | 1 | 小于此值视为 0，避免电机嗡嗡抖 |
| `STEER_COEF0..3` | +1,+1,-1,-1 | 每轮转向系数（差速底盘） |
| `STEER_SHIFT` | 4 | 转向量衰减 |

#### 3.5.5 为什么按"每轮独立"设计

底盘驱动形式**还没定**（差速 / 麦克纳姆 / 四轮独立）。
所以本模块不假设任何具体底盘，只提供 `target[g] = base + bias*COEF[g]`：

- **差速底盘**：左两轮 +1，右两轮 -1（默认值）
- **麦克纳姆/全向**：按运动学矩阵改这四个系数
  （将来若需要二维平移，再加一路 `steer2` 即可）

**换底盘时只改参数、不改逻辑**，避免返工。

---

1. **只给 FPGA 供电**，烧 LED 流水灯，确认板子活着、时钟在跑。
2. 连信号线，**不接电机电源**，用逻辑分析仪抓 PWM/方向/编码器波形。
3. 接电机电源，**低速短时**测试（占空比 ≤ 30%，每次 ≤ 2 秒）。
4. 确认转向与编码器符号一致后，再上闭环。
5. 每次上电测试前确认 **电机、FPGA、ARM、电池共地**。
