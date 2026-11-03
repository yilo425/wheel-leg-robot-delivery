//=============================================================================
// module : tof_driver
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : VL53L0X 激光测距驱动（I2C，周期采样，输出 dist_mm/valid/err）。
//
//-----------------------------------------------------------------------------
// ★★ 初始化做到了哪一级（2026-10-15 升级：简化 3 条 -> 官方序列）★★
//-----------------------------------------------------------------------------
//   上一版只写 3 个寄存器（0x88/0x44/0x45），当时以为"把官方 tuning 表贴进
//   INIT 表就行，其余逻辑不用改"。**那个前提是错的**：
//   官方单次测距流程除了 tuning 表，还需要
//     ① 读 0x91 得到 stop_variable，并在**每次启动测距前**把这一串写完；
//     ② 两处读-改-写（0x60 |= 0x12、0x84 &= ~0x10）；
//     ③ 参考校准（VHV + phase 两段，每段都要轮询 0x13）。
//   所以这一版做的是"初始化时序升级"，不是"贴一张表"。
//
//   来源（★ 单一来源、未经第二份交叉核对 —— 如实标注，不要当成"已与官方核对"）：
//     https://raw.githubusercontent.com/TheLostLambda/ESP32-VL53L0X/master/vl53l0x.c
//   该文件开头写明 "Based on https://github.com/pololu/vl53l0x-arduino"，
//   表中内容即 ST API 的 DefaultTuningSettings() / VL53L0X_load_tuning_settings()。
//   ⚠️ 想抓第二份独立来源做交叉核对时网络超时，本版**只对照了这一份**。
//      如果上板行为和预期不符，第一步应该是拿官方 API 的表逐条再核一遍。
//
//   ⚠️ 参考校准那段（0x01=0x01 -> 0x00=0x41 -> 轮询 -> ... -> 0x01=0x02 -> ...）
//      在真实器件上通常需要**盖住黑靶**、并保证环境光稳定。本模块照官方流程
//      执行，但**没有做任何"校准结果是否合理"的判断**（那要读回更多寄存器）。
//
//-----------------------------------------------------------------------------
// ★★ 明确**不做**的两件事（以及后果）★★
//-----------------------------------------------------------------------------
//   1) SPAD 管理：读 0x92 取 spad_count/spad_type、读 0xB0~0xB5、再写回
//      —— 需要"多字节读改写"，本版跳过。
//      后果：沿用器件 NVM 里的默认 SPAD 配置/参考 SPAD，通常仍能出数，
//            精度可能略降（对"避障"这个用途影响不大）。
//   2) 动态 timing budget 计算：读 0x46/0x51/0x52/0x70/0x71/0x72 算出宏周期，
//      再写回 0x71/0x72 —— 需要乘除法与多字节读，本版跳过，
//      **保留 tuning 表里给的 0x70=0x04 / 0x71=0x01 / 0x72=0xFE**。
//      后果：测量时间/精度用默认值，不是按输入时钟算出来的最优值。
//
//-----------------------------------------------------------------------------
// 结构：只读步骤表 + 微序列器（不是 107 个 localparam + 三套巨型 case）
//-----------------------------------------------------------------------------
//                    ┌──────────── 只读步骤表（case，组合查表）────────────┐
//   step_idx  ──────►│ {st_type, st_reg, st_val}                          │
//                    └────────────────────────────────────────────────────┘
//                                  │
//                    ┌─────────────▼──────────────┐
//                    │ 微序列器 (state == D_SEQ)  │  复用**现有**事务发起器：
//                    │ 一步 = 一笔 I2C 传输       │  t_req/t_pending/i2c_start/
//                    │ 成功才推进 step_idx        │  t_busy/t_done_r
//                    └────────────────────────────┘
//
//   两张表（由 seq_sel 选）：
//     SEQ_INIT : 初始化，**107 步**（104 次写 + 1 次读存 + 2 次轮询），只跑一次
//     SEQ_PRE  : 每次测距前的前导，**7 步**（7 次写），每个采样周期跑一次；
//                跑完接老流程 D_START(0x00=0x01) -> D_POLL -> 读 0x1E/0x1F -> 清中断
//
//   步骤类型（7 种）：
//     ST_WRITE      写 REG = VAL
//     ST_RD_STORE   读 REG，存进变量（用于 stop_variable）
//     ST_RD_OR      读 REG，|MASK 后写回（读-改-写）
//     ST_RD_AND     读 REG，&~MASK 后写回（读-改-写）
//     ST_WR_STORED  把存起来的 stop_variable 写进 REG
//     ST_POLL13     轮询 0x13，bit0~bit2 任意一位置起才继续（没置起就重读）
//     ST_END        序列结束
//
//   状态列表（9 个）：
//     D_BOOT   上电等待（等器件起来）
//     D_SEQ    执行步骤表（seq_sel 决定跑 INIT 还是 PRE）
//     D_WAIT   等采样周期 SAMPLE_MS
//     D_START  写 0x00=0x01 启动单次测距
//     D_POLL   轮询 0x13 bit0（测量用；语义与上一版一样）
//     D_RD_MSB 读 0x1E
//     D_RD_LSB 读 0x1F
//     D_CLEAR  写 0x0B=0x01 清中断
//     D_EVAL   判定 + 输出脉冲
//
//   两层超时（项目铁律：所有等待都要有超时）：
//     * smp_cnt（采样超时）：只管 D_START..D_CLEAR，语义与上一版**完全一样**
//       —— 一次采样的总预算 = TIMEOUT_MS。
//     * seq_cnt（序列超时）：管 D_SEQ，按"**单步无进展**"计时（每推进一步清零）。
//       为什么不能也按总时长：初始化有 107 步、天然几毫秒，用总时长会误杀。
//       按单步算既不会误杀，又能卡住"某一步卡死"（包括轮询一直不就绪）。
//
//-----------------------------------------------------------------------------
// 初始化序列总览（照官方顺序，顺序不能变）
//-----------------------------------------------------------------------------
//   [DataInit]     12 步：0x88=0x00；0x80/0xFF/0x00 三连；读 0x91(存 stop_var)；
//                  0x00/0xFF/0x80 三连；0x60 读改写(|0x12)；0x44=0x00/0x45=0x20；
//                  0x01=0xFF
//   [tuning]       80 步：DefaultTuningSettings()（含多次 0xFF 翻页）
//   [中断/GPIO]     4 步：0x0A=0x04；0x84 读改写(&~0x10)；0x0B=0x01；0x01=0xE8
//   [参考校准]     11 步：0x01=0x01/0x00=0x41 -> 轮询 0x13 -> 0x0B=0x01/0x00=0x00；
//                  0x01=0x02/0x00=0x01 -> 轮询 0x13 -> 0x0B=0x01/0x00=0x00；
//                  0x01=0xE8
//   ---------------------------------------------------------------
//   合计 107 步 = 104 写 + 3 读（含 2 笔读改写的读）+ 2 轮询
//
//   [每次测距前]    7 步：0x80=0x01/0xFF=0x01/0x00=0x00 -> 0x91=stop_var ->
//                  0x00=0x01/0xFF=0x00/0x80=0x00
//                  （这一串以前完全没有，是"读不到数"的关键之一）
//                  之后 D_START 再写 0x00=0x01 启动单次测距。
//
//   ⚠️ 两张表的 (reg,val) 内容与顺序被 tb_tof_driver.v 用
//      "写入总条数 + 32 位校验和"钉住了：改表不改期望值，回归立刻红。
//
//-----------------------------------------------------------------------------
// 时序与寄存器（VL53L0X 数据手册）
//-----------------------------------------------------------------------------
//   0x00 SYSRANGE_START            写 0x01 = 启动一次单次测距
//   0x01 SYSTEM_SEQUENCE_CONFIG    0xFF -> 0xF8 -> 0xE8；校准命令 0x01/0x02
//   0x0A SYSTEM_INTERRUPT_CONFIG   0x04
//   0x0B SYSTEM_INTERRUPT_CLEAR    写 0x01 = 清中断
//   0x13 RESULT_INTERRUPT_STATUS   bit0 = 新测距数据就绪；校准轮询看 bit0~bit2
//   0x1E RESULT_RANGE_STATUS+10    距离高字节（mm）
//   0x1F RESULT_RANGE_STATUS+11    距离低字节（mm）
//   0x44/0x45 信号速率上限          0.25 MCPS -> 0x44=0x00, 0x45=0x20
//   0x60 MSRC_CONFIG               读改写 |= 0x12
//   0x80/0xFF                      使能/翻页（tuning 表大量使用）
//   0x84 GPIO_HV_MUX_ACTIVE_HIGH   读改写 &= ~0x10
//   0x88 I2C 模式                  写 0x00 = 标准模式（官方 SetI2CStandardMode）
//   0x91 STOP_VARIABLE             读出来存着，每次测距前写回
//
//   ⚠️ I2C 地址 0x29 是 VL53L0X 的**出厂默认地址**。两块 TOF 同时挂在总线上
//      会地址冲突（doc/采购清单.md 里提过），必须靠 XSHUT 逐个上电改地址，
//      或者加 TCA9548A 多路复用。本模块只有一个 DEV_ADDR 参数，
//      做双 TOF 时实例化两份、给不同地址（另一块要先改地址）。
//
//-----------------------------------------------------------------------------
// 读数合理性检查（很重要，TOF 在强光/过近/遮挡下会给出垃圾值）
//-----------------------------------------------------------------------------
//   dist == 0        -> 无效（0 是典型的"没测到"返回值，不是"贴着物体"）
//   dist > DIST_MAX_MM(默认 2000) -> 无效（VL53L0X 量程 2m）
//   无效时：给 dist_err 单拍脉冲，dist_mm **保持上一次有效值不变**。
//   为什么保持：main_fsm 侧对 sensor 失效的策略是"限速"（见
//   doc/interface_spec.md 的 B3），把上次的有效距离留在总线上比塞一个
//   0 更安全（0 会被误判成"正前方 0mm 有障碍"→ 急停）。
//
//-----------------------------------------------------------------------------
// 与 main_fsm 的接口
//-----------------------------------------------------------------------------
//   dist_mm / dist_valid / dist_err 都来自寄存器输出，时钟同步，
//   所以 main_fsm 里"下一拍采样"天然满足 interface_spec B6
//   （读数先锁存、下一拍生效），不需要额外锁存器。
//
//   ★ 对外行为与上一版完全一致：参数名、端口、单拍脉冲、无效不刷新 dist_mm、
//     SAMPLE_MS 周期采样，一个都没改（rtl/top.v 在用它）。
//=============================================================================

`timescale 1ns/1ps

module tof_driver #(
    //----------------------------------------------------------------
    // 时钟与采样周期
    //----------------------------------------------------------------
    parameter integer CLK_FREQ_HZ    = 50_000_000,
    parameter integer SAMPLE_MS      = 50,       // 采样周期（ms）

    //----------------------------------------------------------------
    // 超时
    //   一次采样（启动+轮询+读两字节+清中断）超过 TIMEOUT_MS 就放弃。
    //   VL53L0X 默认时序预算约 33ms，所以 100ms 留了足够余量。
    //   ★ 同一个值也用作"序列阶段单步无进展"的超时（见文件头）。
    //----------------------------------------------------------------
    parameter integer TIMEOUT_MS     = 100,

    //----------------------------------------------------------------
    // I2C 速率与单笔传输超时（透传给 i2c_master）
    //----------------------------------------------------------------
    parameter integer I2C_SCL_HZ     = 400_000,
    parameter integer I2C_TIMEOUT_MS = 10,

    //----------------------------------------------------------------
    // 上电等待
    //   VL53L0X 上电/释放 XSHUT 后需要约 1.2ms 才响应 I2C，
    //   这里保守取 10ms；仿真里改成 1 加快验证。
    //----------------------------------------------------------------
    parameter integer BOOT_WAIT_MS   = 10,

    //----------------------------------------------------------------
    // 合理性检查上限与从机地址
    //----------------------------------------------------------------
    parameter integer DIST_MAX_MM    = 2000,     // VL53L0X 量程 ~2m
    parameter [6:0]   DEV_ADDR       = 7'h29,    // VL53L0X 默认 7 位地址

    parameter integer _ASSERT_EN     = 1
) (
    input  wire        clk,
    input  wire        rst_n,          // 低有效

    //---------------- 给 main_fsm / top.v 的结果 ----------------
    output reg  [15:0] dist_mm,        // 最后一次**有效**距离（mm）
    output reg         dist_valid,     // 单拍脉冲：本次读数有效
    output reg         dist_err,       // 单拍脉冲：本次测距失败（无效读数/超时/无应答）

    //---------------- I2C 物理接口（开漏，需外部上拉）----------------
    inout  wire        scl,
    inout  wire        sda,

    //---------------- 共享总线仲裁接口（多器件挂一条 I2C 时用）----------------
    //   单个驱动独占总线时，把 bus_grant 接 1'b1、bus_req/bus_busy 悬空即可。
    //   多器件时接 i2c_bus_arb：bus_req/bus_busy 是"请求/占用"，bus_grant 是"获准"。
    input  wire        bus_grant,      // 1 = 获准使用总线；0 时不会发出 start
    output wire        bus_req,        // 1 = 有排队事务、想用总线
    output wire        bus_busy        // 1 = 一笔事务正在总线上进行（含等 grant 之前）
);

    //================================================================
    // 时间常数
    //
    // ⚠️ 一律先除 1000 再乘，避免 ms * CLK_FREQ_HZ 溢出 32 位。
    //    （100ms * 50MHz = 5e9 > 2^31，是最容易踩的溢出坑）
    //================================================================
    localparam [31:0] MS_CYC = CLK_FREQ_HZ / 1000;

    localparam [31:0] BOOT_CYC    = ((BOOT_WAIT_MS < 1) ? 32'd1 :
                                     BOOT_WAIT_MS) * MS_CYC;
    localparam [31:0] SAMPLE_CYC  = ((SAMPLE_MS    < 1) ? 32'd1 :
                                     SAMPLE_MS)    * MS_CYC;
    localparam [31:0] SMP_TMO_CYC = ((TIMEOUT_MS   < 1) ? 32'd1 :
                                     TIMEOUT_MS)   * MS_CYC;

    //================================================================
    // VL53L0X 寄存器地址（表里只对这几个用了具名常量，其余写字面量+注释）
    //================================================================
    localparam [7:0] REG_SYSRANGE_START    = 8'h00;
    localparam [7:0] REG_SYSTEM_INT_CLEAR  = 8'h0B;
    localparam [7:0] REG_RESULT_INT_STATUS = 8'h13;
    localparam [7:0] REG_RESULT_RANGE_MSB  = 8'h1E;
    localparam [7:0] REG_RESULT_RANGE_LSB  = 8'h1F;
    localparam [7:0] REG_MSRC_CONFIG       = 8'h60;   // 读改写: |= 0x12
    localparam [7:0] REG_GPIO_HV_MUX       = 8'h84;   // 读改写: &= ~0x10
    localparam [7:0] REG_I2C_STD_MODE      = 8'h88;
    localparam [7:0] REG_SIGNAL_RATE_MSB   = 8'h44;
    localparam [7:0] REG_SIGNAL_RATE_LSB   = 8'h45;
    localparam [7:0] REG_STOP_VARIABLE     = 8'h91;
    localparam [7:0] REG_PAGE_SELECT       = 8'hFF;

    //================================================================
    // 步骤类型
    //================================================================
    localparam [2:0]
        ST_WRITE     = 3'd0,   // 写 REG = VAL
        ST_RD_STORE  = 3'd1,   // 读 REG -> 变量
        ST_RD_OR     = 3'd2,   // 读 REG | VAL -> 写回
        ST_RD_AND    = 3'd3,   // 读 REG & ~VAL -> 写回
        ST_WR_STORED = 3'd4,   // 把存起来的 stop_var 写进 REG
        ST_POLL13    = 3'd5,   // 轮询 0x13 bit0~bit2
        ST_END       = 3'd6;   // 序列结束

    //================================================================
    // 序列选择 + 步数（步数与表内容必须一致，见下面自检）
    //================================================================
    localparam SEQ_INIT = 1'b0;    // 初始化（跑一次）
    localparam SEQ_PRE  = 1'b1;    // 每次测距前的前导

    //   INIT_STEPS = 写 + 1(读存 0x91) + 2(轮询 0x13)
    //   （两处读改写各算 1 步，每步 1 次写 + 1 次读，所以写数已含在里面）
    localparam [7:0] INIT_STEPS  = 8'd107;
    localparam [7:0] INIT_WRITES = 8'd104;
    localparam [7:0] PRE_STEPS   = 8'd7;

    //================================================================
    // 状态定义
    //================================================================
    localparam [3:0]
        D_BOOT   = 4'd0,   // 上电等待（器件 boot）
        D_SEQ    = 4'd1,   // 执行步骤表（seq_sel 决定 INIT/PRE）
        D_WAIT   = 4'd2,   // 等下一个采样周期
        D_START  = 4'd3,   // 写 0x00=0x01 启动单次测距
        D_POLL   = 4'd4,   // 读 0x13 轮询 bit0
        D_RD_MSB = 4'd5,   // 读 0x1E
        D_RD_LSB = 4'd6,   // 读 0x1F
        D_CLEAR  = 4'd7,   // 写 0x0B=0x01 清中断
        D_EVAL   = 4'd8;   // 判定 + 输出脉冲

    reg [3:0]  state;
    reg        tx_busy;      // 当前步骤/状态里的那次 I2C 事务是否已发起
    reg [31:0] boot_cnt;
    reg [31:0] wait_cnt;
    reg [31:0] smp_cnt;      // 一次采样内的计数（采样超时用，语义同上一版）
    reg [31:0] seq_cnt;      // 序列阶段"单步无进展"计数
    reg [7:0]  range_msb;
    reg [7:0]  range_lsb;
    reg        init_ok;      // 初始化是否已成功
    reg        smp_err;      // 本次采样是否出错
    reg [7:0]  stop_var;     // 读 0x91 得到的 stop_variable

    //---- 序列器内部 ----
    reg        seq_sel;      // 0 = SEQ_INIT, 1 = SEQ_PRE
    reg [7:0]  step_idx;     // 当前步骤
    reg        seq_phase;    // 读改写用：0 = 刚发起/等待读，1 = 等待写回
    reg [7:0]  rw_val;       // 读改写算好的写回值

    // 拼出的距离读数
    wire [15:0] dist_calc = {range_msb, range_lsb};

    //================================================================
    // 只读步骤表（组合查表：step_idx -> {st_type, st_reg, st_val}）
    //
    //   ★ 全部默认赋值，避免生成 latch（组合 always 必须每条路径都赋值）。
    //   ★ 表用两行标记包住（下面注释里的 SEQ_INIT_TABLE_BEGIN /
    //     SEQ_INIT_TABLE_END 两行，前面各带三个斜杠和箭头），方便外部脚本
    //     把这张表抽出来和官方序列逐条比对、并算出校验和。**别删这两行标记**。
    //================================================================
    reg [2:0] st_type;
    reg [7:0] st_reg;
    reg [7:0] st_val;

    always @(*) begin
        st_type = ST_END;
        st_reg  = 8'h00;
        st_val  = 8'h00;

        case (seq_sel)
        //----------------------------------------------------------------
        SEQ_INIT: begin
            case (step_idx)
            // >>> SEQ_INIT_TABLE_BEGIN
            // ---- DataInit（11 写 + 1 读存 + 1 读改写 = 12 步）----
            8'd0:   begin st_type = ST_WRITE;     st_reg = 8'h88; st_val = 8'h00; end
            8'd1:   begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h01; end
            8'd2:   begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd3:   begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h00; end
            8'd4:   begin st_type = ST_RD_STORE;  st_reg = 8'h91; st_val = 8'h00; end
            8'd5:   begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h01; end
            8'd6:   begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd7:   begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h00; end
            8'd8:   begin st_type = ST_RD_OR;     st_reg = 8'h60; st_val = 8'h12; end
            8'd9:   begin st_type = ST_WRITE;     st_reg = 8'h44; st_val = 8'h00; end
            8'd10:  begin st_type = ST_WRITE;     st_reg = 8'h45; st_val = 8'h20; end
            8'd11:  begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'hFF; end
            // ---- tuning: DefaultTuningSettings()（80 写）----
            8'd12:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd13:  begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h00; end
            8'd14:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd15:  begin st_type = ST_WRITE;     st_reg = 8'h09; st_val = 8'h00; end
            8'd16:  begin st_type = ST_WRITE;     st_reg = 8'h10; st_val = 8'h00; end
            8'd17:  begin st_type = ST_WRITE;     st_reg = 8'h11; st_val = 8'h00; end
            8'd18:  begin st_type = ST_WRITE;     st_reg = 8'h24; st_val = 8'h01; end
            8'd19:  begin st_type = ST_WRITE;     st_reg = 8'h25; st_val = 8'hFF; end
            8'd20:  begin st_type = ST_WRITE;     st_reg = 8'h75; st_val = 8'h00; end
            8'd21:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd22:  begin st_type = ST_WRITE;     st_reg = 8'h4E; st_val = 8'h2C; end
            8'd23:  begin st_type = ST_WRITE;     st_reg = 8'h48; st_val = 8'h00; end
            8'd24:  begin st_type = ST_WRITE;     st_reg = 8'h30; st_val = 8'h20; end
            8'd25:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd26:  begin st_type = ST_WRITE;     st_reg = 8'h30; st_val = 8'h09; end
            8'd27:  begin st_type = ST_WRITE;     st_reg = 8'h54; st_val = 8'h00; end
            8'd28:  begin st_type = ST_WRITE;     st_reg = 8'h31; st_val = 8'h04; end
            8'd29:  begin st_type = ST_WRITE;     st_reg = 8'h32; st_val = 8'h03; end
            8'd30:  begin st_type = ST_WRITE;     st_reg = 8'h40; st_val = 8'h83; end
            8'd31:  begin st_type = ST_WRITE;     st_reg = 8'h46; st_val = 8'h25; end
            8'd32:  begin st_type = ST_WRITE;     st_reg = 8'h60; st_val = 8'h00; end
            8'd33:  begin st_type = ST_WRITE;     st_reg = 8'h27; st_val = 8'h00; end
            8'd34:  begin st_type = ST_WRITE;     st_reg = 8'h50; st_val = 8'h06; end
            8'd35:  begin st_type = ST_WRITE;     st_reg = 8'h51; st_val = 8'h00; end
            8'd36:  begin st_type = ST_WRITE;     st_reg = 8'h52; st_val = 8'h96; end
            8'd37:  begin st_type = ST_WRITE;     st_reg = 8'h56; st_val = 8'h08; end
            8'd38:  begin st_type = ST_WRITE;     st_reg = 8'h57; st_val = 8'h30; end
            8'd39:  begin st_type = ST_WRITE;     st_reg = 8'h61; st_val = 8'h00; end
            8'd40:  begin st_type = ST_WRITE;     st_reg = 8'h62; st_val = 8'h00; end
            8'd41:  begin st_type = ST_WRITE;     st_reg = 8'h64; st_val = 8'h00; end
            8'd42:  begin st_type = ST_WRITE;     st_reg = 8'h65; st_val = 8'h00; end
            8'd43:  begin st_type = ST_WRITE;     st_reg = 8'h66; st_val = 8'hA0; end
            8'd44:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd45:  begin st_type = ST_WRITE;     st_reg = 8'h22; st_val = 8'h32; end
            8'd46:  begin st_type = ST_WRITE;     st_reg = 8'h47; st_val = 8'h14; end
            8'd47:  begin st_type = ST_WRITE;     st_reg = 8'h49; st_val = 8'hFF; end
            8'd48:  begin st_type = ST_WRITE;     st_reg = 8'h4A; st_val = 8'h00; end
            8'd49:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd50:  begin st_type = ST_WRITE;     st_reg = 8'h7A; st_val = 8'h0A; end
            8'd51:  begin st_type = ST_WRITE;     st_reg = 8'h7B; st_val = 8'h00; end
            8'd52:  begin st_type = ST_WRITE;     st_reg = 8'h78; st_val = 8'h21; end
            8'd53:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd54:  begin st_type = ST_WRITE;     st_reg = 8'h23; st_val = 8'h34; end
            8'd55:  begin st_type = ST_WRITE;     st_reg = 8'h42; st_val = 8'h00; end
            8'd56:  begin st_type = ST_WRITE;     st_reg = 8'h44; st_val = 8'hFF; end
            8'd57:  begin st_type = ST_WRITE;     st_reg = 8'h45; st_val = 8'h26; end
            8'd58:  begin st_type = ST_WRITE;     st_reg = 8'h46; st_val = 8'h05; end
            8'd59:  begin st_type = ST_WRITE;     st_reg = 8'h40; st_val = 8'h40; end
            8'd60:  begin st_type = ST_WRITE;     st_reg = 8'h0E; st_val = 8'h06; end
            8'd61:  begin st_type = ST_WRITE;     st_reg = 8'h20; st_val = 8'h1A; end
            8'd62:  begin st_type = ST_WRITE;     st_reg = 8'h43; st_val = 8'h40; end
            8'd63:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd64:  begin st_type = ST_WRITE;     st_reg = 8'h34; st_val = 8'h03; end
            8'd65:  begin st_type = ST_WRITE;     st_reg = 8'h35; st_val = 8'h44; end
            8'd66:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd67:  begin st_type = ST_WRITE;     st_reg = 8'h31; st_val = 8'h04; end
            8'd68:  begin st_type = ST_WRITE;     st_reg = 8'h4B; st_val = 8'h09; end
            8'd69:  begin st_type = ST_WRITE;     st_reg = 8'h4C; st_val = 8'h05; end
            8'd70:  begin st_type = ST_WRITE;     st_reg = 8'h4D; st_val = 8'h04; end
            8'd71:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd72:  begin st_type = ST_WRITE;     st_reg = 8'h44; st_val = 8'h00; end
            8'd73:  begin st_type = ST_WRITE;     st_reg = 8'h45; st_val = 8'h20; end
            8'd74:  begin st_type = ST_WRITE;     st_reg = 8'h47; st_val = 8'h08; end
            8'd75:  begin st_type = ST_WRITE;     st_reg = 8'h48; st_val = 8'h28; end
            8'd76:  begin st_type = ST_WRITE;     st_reg = 8'h67; st_val = 8'h00; end
            8'd77:  begin st_type = ST_WRITE;     st_reg = 8'h70; st_val = 8'h04; end
            8'd78:  begin st_type = ST_WRITE;     st_reg = 8'h71; st_val = 8'h01; end
            8'd79:  begin st_type = ST_WRITE;     st_reg = 8'h72; st_val = 8'hFE; end
            8'd80:  begin st_type = ST_WRITE;     st_reg = 8'h76; st_val = 8'h00; end
            8'd81:  begin st_type = ST_WRITE;     st_reg = 8'h77; st_val = 8'h00; end
            8'd82:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd83:  begin st_type = ST_WRITE;     st_reg = 8'h0D; st_val = 8'h01; end
            8'd84:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd85:  begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h01; end
            8'd86:  begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'hF8; end
            8'd87:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd88:  begin st_type = ST_WRITE;     st_reg = 8'h8E; st_val = 8'h01; end
            8'd89:  begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h01; end
            8'd90:  begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd91:  begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h00; end
            // ---- 中断 / GPIO（3 写 + 1 读改写 = 4 步）----
            8'd92:  begin st_type = ST_WRITE;     st_reg = 8'h0A; st_val = 8'h04; end
            8'd93:  begin st_type = ST_RD_AND;    st_reg = 8'h84; st_val = 8'h10; end
            8'd94:  begin st_type = ST_WRITE;     st_reg = 8'h0B; st_val = 8'h01; end
            8'd95:  begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'hE8; end
            // ---- 参考校准（VHV + phase，各含一次轮询）（9 写 + 2 轮询 = 11 步）----
            8'd96:  begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'h01; end
            8'd97:  begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h41; end
            8'd98:  begin st_type = ST_POLL13;    st_reg = 8'h13; st_val = 8'h00; end
            8'd99:  begin st_type = ST_WRITE;     st_reg = 8'h0B; st_val = 8'h01; end
            8'd100: begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h00; end
            8'd101: begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'h02; end
            8'd102: begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h01; end
            8'd103: begin st_type = ST_POLL13;    st_reg = 8'h13; st_val = 8'h00; end
            8'd104: begin st_type = ST_WRITE;     st_reg = 8'h0B; st_val = 8'h01; end
            8'd105: begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h00; end
            8'd106: begin st_type = ST_WRITE;     st_reg = 8'h01; st_val = 8'hE8; end
            // <<< SEQ_INIT_TABLE_END
            default: st_type = ST_END;
            endcase
        end

        //----------------------------------------------------------------
        // 每次单次测距之前的前导（官方写法；这段以前完全没有）
        //   注意最后一步不是启动测距：启动仍是 D_START 的 0x00=0x01。
        //----------------------------------------------------------------
        default: begin
            case (step_idx)
            // >>> SEQ_PRE_TABLE_BEGIN
            8'd0:   begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h01; end
            8'd1:   begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h01; end
            8'd2:   begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h00; end
            8'd3:   begin st_type = ST_WR_STORED; st_reg = 8'h91; st_val = 8'h00; end
            8'd4:   begin st_type = ST_WRITE;     st_reg = 8'h00; st_val = 8'h01; end
            8'd5:   begin st_type = ST_WRITE;     st_reg = 8'hFF; st_val = 8'h00; end
            8'd6:   begin st_type = ST_WRITE;     st_reg = 8'h80; st_val = 8'h00; end
            // <<< SEQ_PRE_TABLE_END
            default: st_type = ST_END;
            endcase
        end
        endcase
    end

    //================================================================
    // I2C 事务发起器（把一次事务打包好交给 i2c_master）
    //
    //   为什么要有这一层：序列器/状态机里每个步骤只需要写"发什么"，
    //   不用关心 i2c_master 什么时候空闲、done 在哪一拍来。
    //   握手：主 FSM 拉 t_req 一拍 -> 内部 t_pending -> 等 master 空闲后发车
    //         -> master 结束 -> t_done_r 一拍 + t_err_r / t_data_r
    //   ★ 这一层原样保留，序列器复用它（没有再写一套）。
    //================================================================
    reg        t_req;
    reg [6:0]  t_addr;
    reg        t_rd;
    reg        t_reg_en;
    reg [7:0]  t_wr;
    reg [7:0]  t_reg;
    reg        t_clr;        // 丢弃"已请求但还没发出去"的事务（超时收尾用）

    reg        i2c_start;
    wire [7:0] i2c_rd_data;
    wire       i2c_busy;
    wire       i2c_ack_err;
    wire       i2c_done;

    reg        t_busy;
    reg        t_pending;
    reg        t_done_r;
    reg        t_err_r;
    reg [7:0]  t_data_r;

    //---------------- 总线仲裁信号导出 ----------------
    //   bus_req  = 有排队事务（t_pending）；bus_busy = 事务在总线上进行（t_busy）。
    //   仲裁器用 req 来授 grant，用 busy 来"占用总线期间不切换"。
    assign bus_req  = t_pending;
    assign bus_busy = t_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_start <= 1'b0;
            t_busy    <= 1'b0;
            t_pending <= 1'b0;
            t_done_r  <= 1'b0;
            t_err_r   <= 1'b0;
            t_data_r  <= 8'h00;
        end else begin
            i2c_start <= 1'b0;      // 单拍脉冲
            t_done_r  <= 1'b0;      // 单拍脉冲

            if (t_req) t_pending <= 1'b1;

            if (!t_busy) begin
                // ⚠️ 必须等 master 真的空闲（i2c_busy=0）再发车。
                //    否则 start 脉冲会落在 master 的传输中间被直接丢掉，
                //    这一层就会永远等不到 done（表现为每次采样都超时）。
                //    ★ 多器件共享总线时还要 bus_grant=1：没获准就继续等，
                //      不会和另一个驱动抢总线。
                if (t_pending && !i2c_busy && bus_grant) begin
                    i2c_start <= 1'b1;
                    t_busy    <= 1'b1;
                    t_pending <= 1'b0;
                end
            end else if (i2c_done) begin
                t_busy   <= 1'b0;
                t_done_r <= 1'b1;
                t_err_r  <= i2c_ack_err;
                t_data_r <= i2c_rd_data;
            end

            // 超时收尾：把还没发出去的请求丢掉，避免超时之后总线上又冒出一笔交易
            if (t_clr) t_pending <= 1'b0;
        end
    end

    //================================================================
    // 两层超时
    //
    //   smp_* : 一次采样的总预算（D_START..D_CLEAR）—— 语义与上一版完全一样
    //   seq_* : 序列阶段（D_SEQ）的"单步无进展"预算 —— 每推进一步就清零
    //
    //   双保险：i2c_master 内部还有单笔传输超时（I2C_TIMEOUT_MS）。
    //================================================================
    wire in_sample = (state == D_START) || (state == D_POLL) ||
                     (state == D_RD_MSB) || (state == D_RD_LSB) ||
                     (state == D_CLEAR);

    wire smp_timeout = in_sample && (smp_cnt >= SMP_TMO_CYC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smp_cnt <= 32'd0;
        end else if (!in_sample) begin
            smp_cnt <= 32'd0;
        end else begin
            smp_cnt <= smp_cnt + 32'd1;
        end
    end

    // 本拍是否"推进了一步"（成功完成一步；读改写的写回阶段才算完成）
    wire seq_step_done = (state == D_SEQ) && tx_busy && t_done_r && !t_err_r &&
                         ( (st_type == ST_WRITE)     ||
                           (st_type == ST_WR_STORED) ||
                           (st_type == ST_RD_STORE)  ||
                           (((st_type == ST_RD_OR) || (st_type == ST_RD_AND)) &&
                             (seq_phase == 1'b1))    ||
                           ((st_type == ST_POLL13) && (t_data_r[2:0] != 3'b000)) );

    wire seq_timeout = (state == D_SEQ) && (seq_cnt >= SMP_TMO_CYC);
    wire any_timeout = smp_timeout || seq_timeout;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_cnt <= 32'd0;
        end else if ((state != D_SEQ) || seq_step_done) begin
            seq_cnt <= 32'd0;
        end else begin
            seq_cnt <= seq_cnt + 32'd1;
        end
    end

    //================================================================
    // 主状态机
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= D_BOOT;
            tx_busy    <= 1'b0;
            boot_cnt   <= 32'd0;
            wait_cnt   <= 32'd0;
            step_idx   <= 8'd0;
            seq_sel    <= SEQ_INIT;
            seq_phase  <= 1'b0;
            rw_val     <= 8'h00;
            stop_var   <= 8'h00;
            range_msb  <= 8'h00;
            range_lsb  <= 8'h00;
            init_ok    <= 1'b0;
            smp_err    <= 1'b0;
            dist_mm    <= 16'd0;
            dist_valid <= 1'b0;
            dist_err   <= 1'b0;
            t_req      <= 1'b0;
            t_addr     <= DEV_ADDR;
            t_rd       <= 1'b0;
            t_reg_en   <= 1'b0;
            t_wr       <= 8'h00;
            t_reg      <= 8'h00;
            t_clr      <= 1'b0;
        end else begin
            // 两个输出都是单拍脉冲：默认清零
            dist_valid <= 1'b0;
            dist_err   <= 1'b0;
            t_req      <= 1'b0;
            t_clr      <= 1'b0;

            if (any_timeout) begin
                // 超时：置错误，等总线空下来再去 D_EVAL 报错
                smp_err <= 1'b1;
                tx_busy <= 1'b0;
                t_clr   <= 1'b1;
                // 初始化阶段超时 -> 这次初始化没成功，下个周期整段重来
                if (seq_timeout && (seq_sel == SEQ_INIT)) init_ok <= 1'b0;
                if (!i2c_busy && !t_busy) state <= D_EVAL;
            end else begin
                case (state)

                //--------------------------------------------------
                // 上电等待：VL53L0X 起来之前 I2C 是不会应答的
                //--------------------------------------------------
                D_BOOT: begin
                    if (boot_cnt >= BOOT_CYC) begin
                        boot_cnt  <= 32'd0;
                        step_idx  <= 8'd0;
                        seq_sel   <= SEQ_INIT;
                        seq_phase <= 1'b0;
                        tx_busy   <= 1'b0;
                        state     <= D_SEQ;
                    end else begin
                        boot_cnt <= boot_cnt + 32'd1;
                    end
                end

                //--------------------------------------------------
                // D_SEQ：执行只读步骤表（微序列器）
                //
                //   每种步骤类型只做一件事：
                //     WRITE/WR_STORED      -> 一笔写
                //     RD_STORE/RD_OR/RD_AND-> 先一笔读，若是读改写再跟一笔写
                //     POLL13               -> 反复读 0x13 直到 bit0~bit2 有置起
                //   成功（t_done_r 且无 ack_err）才推进 step_idx。
                //--------------------------------------------------
                D_SEQ: begin
                    if (st_type == ST_END) begin
                        //---------------- 序列结束 ----------------
                        step_idx  <= 8'd0;
                        seq_phase <= 1'b0;
                        tx_busy   <= 1'b0;
                        if (seq_sel == SEQ_INIT) begin
                            init_ok  <= 1'b1;
                            wait_cnt <= 32'd0;
                            state    <= D_WAIT;      // 初始化完成 -> 等采样周期
                        end else begin
                            state    <= D_START;     // 前导写完 -> 启动单次测距
                        end
                    end else if (!tx_busy) begin
                        //---------------- 发起本步骤的事务 ----------------
                        t_addr   <= DEV_ADDR;
                        t_reg_en <= 1'b1;
                        t_reg    <= st_reg;
                        if (st_type == ST_POLL13) begin
                            t_rd <= 1'b1;                    // 读 0x13
                            t_wr <= 8'h00;
                        end else if (seq_phase == 1'b1) begin
                            t_rd <= 1'b0;                    // 读改写的第二笔：写回
                            t_wr <= rw_val;
                        end else if ((st_type == ST_RD_STORE) ||
                                     (st_type == ST_RD_OR)    ||
                                     (st_type == ST_RD_AND)) begin
                            t_rd <= 1'b1;                    // 先读
                            t_wr <= 8'h00;
                        end else if (st_type == ST_WR_STORED) begin
                            t_rd <= 1'b0;                    // stop_variable 写回 0x91
                            t_wr <= stop_var;
                        end else begin
                            t_rd <= 1'b0;                    // 普通写
                            t_wr <= st_val;
                        end
                        t_req   <= 1'b1;
                        tx_busy <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin
                            // 器件没应答 / 总线超时
                            smp_err <= 1'b1;
                            if (seq_sel == SEQ_INIT) init_ok <= 1'b0;
                            state   <= D_EVAL;
                        end else begin
                            case (st_type)
                                ST_POLL13: begin
                                    // bit0~bit2 任意一位置起 = 校准就绪；
                                    // 没置起就**保持 step_idx 不变** -> 下一拍再读
                                    if (t_data_r[2:0] != 3'b000)
                                        step_idx <= step_idx + 8'd1;
                                end
                                ST_RD_STORE: begin
                                    stop_var <= t_data_r;
                                    step_idx <= step_idx + 8'd1;
                                end
                                ST_RD_OR, ST_RD_AND: begin
                                    if (seq_phase == 1'b0) begin
                                        rw_val    <= (st_type == ST_RD_OR)
                                                     ? (t_data_r | st_val)
                                                     : (t_data_r & ~st_val);
                                        seq_phase <= 1'b1;
                                    end else begin
                                        seq_phase <= 1'b0;
                                        step_idx  <= step_idx + 8'd1;
                                    end
                                end
                                default: begin
                                    step_idx <= step_idx + 8'd1;
                                end
                            endcase
                        end
                    end
                end

                //--------------------------------------------------
                // 等采样周期（SAMPLE_MS）
                //--------------------------------------------------
                D_WAIT: begin
                    if (wait_cnt >= SAMPLE_CYC) begin
                        wait_cnt  <= 32'd0;
                        tx_busy   <= 1'b0;
                        step_idx  <= 8'd0;
                        seq_phase <= 1'b0;
                        if (!init_ok) begin
                            // 初始化没成功 -> 整段重来（上电顺序/上拉没稳时很常见）
                            seq_sel <= SEQ_INIT;
                            state   <= D_SEQ;
                        end else begin
                            smp_err <= 1'b0;
                            seq_sel <= SEQ_PRE;      // 每次测距前的前导
                            state   <= D_SEQ;
                        end
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                //--------------------------------------------------
                // 启动单次测距：0x00 <- 0x01
                //   （官方序列里这仍然是"前导之后的最后一句"）
                //--------------------------------------------------
                D_START: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b0;
                        t_reg_en <= 1'b1;
                        t_reg    <= REG_SYSRANGE_START;
                        t_wr     <= 8'h01;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin smp_err <= 1'b1; state <= D_EVAL; end
                        else              state <= D_POLL;
                    end
                end

                //--------------------------------------------------
                // 轮询 0x13 bit0 直到数据就绪（测量用，语义同上一版）
                //   没就绪就再来一轮（次数不设死，由采样总超时兜底，
                //   因为不同时序预算下等待时间差别很大）
                //--------------------------------------------------
                D_POLL: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b1;
                        t_reg_en <= 1'b1;
                        t_reg    <= REG_RESULT_INT_STATUS;
                        t_wr     <= 8'h00;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r)            begin smp_err <= 1'b1; state <= D_EVAL; end
                        else if (t_data_r[0])   state <= D_RD_MSB;
                        else                    state <= D_POLL;
                    end
                end

                //--------------------------------------------------
                // 读距离高字节 0x1E
                //--------------------------------------------------
                D_RD_MSB: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b1;
                        t_reg_en <= 1'b1;
                        t_reg    <= REG_RESULT_RANGE_MSB;
                        t_wr     <= 8'h00;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin
                            smp_err <= 1'b1;
                            state   <= D_EVAL;
                        end else begin
                            range_msb <= t_data_r;
                            state     <= D_RD_LSB;
                        end
                    end
                end

                //--------------------------------------------------
                // 读距离低字节 0x1F
                //   ⚠️ 故意分成两笔单字节复合读（0x1E 再 0x1F），而不是
                //      一次连读两字节：i2c_master 只做单字节，保持它简单、
                //      好验证。代价是每帧多一笔 ~50us 的事务，相比
                //      SAMPLE_MS=50ms 完全可以忽略。
                //--------------------------------------------------
                D_RD_LSB: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b1;
                        t_reg_en <= 1'b1;
                        t_reg    <= REG_RESULT_RANGE_LSB;
                        t_wr     <= 8'h00;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin
                            smp_err <= 1'b1;
                            state   <= D_EVAL;
                        end else begin
                            range_lsb <= t_data_r;
                            state     <= D_CLEAR;
                        end
                    end
                end

                //--------------------------------------------------
                // 清中断：0x0B <- 0x01
                //   不清的话 0x13 bit0 一直是 1，下一次采样会立刻"就绪"，
                //   读到的还是上一次的结果（看起来像"距离不刷新"）。
                //--------------------------------------------------
                D_CLEAR: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b0;
                        t_reg_en <= 1'b1;
                        t_reg    <= REG_SYSTEM_INT_CLEAR;
                        t_wr     <= 8'h01;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) smp_err <= 1'b1;
                        state <= D_EVAL;
                    end
                end

                //--------------------------------------------------
                // 判定 + 输出
                //--------------------------------------------------
                D_EVAL: begin
                    tx_busy <= 1'b0;
                    if (smp_err) begin
                        dist_err <= 1'b1;          // 传输失败/超时
                    end else if ((dist_calc == 16'd0) || (dist_calc > DIST_MAX_MM)) begin
                        dist_err <= 1'b1;          // 0 或超量程都算无效
                    end else begin
                        dist_mm    <= dist_calc;
                        dist_valid <= 1'b1;
                    end
                    smp_err  <= 1'b0;
                    wait_cnt <= 32'd0;
                    state    <= D_WAIT;
                end

                default: begin
                    state   <= D_BOOT;
                    tx_busy <= 1'b0;
                end
                endcase
            end
        end
    end

    //================================================================
    // I2C 主机
    //================================================================
    i2c_master #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SCL_HZ      (I2C_SCL_HZ),
        .TIMEOUT_MS  (I2C_TIMEOUT_MS)
    ) u_i2c (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (i2c_start),
        .dev_addr    (t_addr),
        .rd          (t_rd),
        .wr_data     (t_wr),
        .reg_addr_en (t_reg_en),
        .reg_addr    (t_reg),
        .rd_data     (i2c_rd_data),
        .busy        (i2c_busy),
        .ack_err     (i2c_ack_err),
        .done        (i2c_done),
        .scl         (scl),
        .sda         (sda)
    );

    //================================================================
    // 参数 / 步数自检
    //
    //   ★ 这里能查的只有"步数与笔数的关系"：INIT_STEPS 恒等于
    //      INIT_WRITES + 1(读存 0x91) + 2(两次轮询)。
    //      加表时改了 INIT_WRITES 却忘了改 INIT_STEPS，这里立刻报。
    //      （表内容本身由 tb_tof_driver.v 的"写入总条数 + 校验和"钉住。）
    //================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            if (INIT_STEPS !== (INIT_WRITES + 8'd3)) begin
                $display("[tof_driver] ERROR: INIT_STEPS(%0d) != INIT_WRITES(%0d) + 3",
                         INIT_STEPS, INIT_WRITES);
            end
            if (PRE_STEPS < 8'd1) begin
                $display("[tof_driver] ERROR: PRE_STEPS 必须 >= 1");
            end
            if (DIST_MAX_MM < 1) begin
                $display("[tof_driver] ERROR: DIST_MAX_MM 必须 >= 1");
            end
            $display("[tof_driver] SAMPLE=%0dms TIMEOUT=%0dms BOOT=%0dms DIST_MAX=%0dmm ADDR=0x%02h",
                     SAMPLE_MS, TIMEOUT_MS, BOOT_WAIT_MS, DIST_MAX_MM, DEV_ADDR);
            $display("[tof_driver] 初始化序列: %0d 步 / %0d 次写（含 2 处读改写）; 每次测距前导: %0d 步",
                     INIT_STEPS, INIT_WRITES, PRE_STEPS);
        end
    end

endmodule
