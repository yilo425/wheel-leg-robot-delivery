//=============================================================================
// servo_lift -- 升降腿驱动（2 路：前腿组 / 后腿组）
//-----------------------------------------------------------------------------
// 上游（main_fsm）给的是**意图**：lift_cmd[1:0] = {后腿, 前腿}，1=抬起。
// 本模块负责把它变成**舵机能吃的 50Hz PWM**，并做三件 main_fsm 不该管的事：
//
//   1) 舵机 PWM 生成
//   2) 限位开关**消抖 + 极性统一**（原始输入低有效 -> 对外高有效）
//   3) 故障输入消抖
//
//-----------------------------------------------------------------------------
// 为什么极性统一要放在这里
//-----------------------------------------------------------------------------
//   限位开关是**常开(NO)** 触点，一端接 GND，另一端接 FPGA 输入。
//   所以必须 **上拉**：未按下读 1、按下（到位）读 0 —— 即**低有效**。
//
//   ⚠️ 但 main_fsm 里写的是 `if (lift_done)`（高有效）。
//      直接连过去的话，一进 LIFT 状态就会把"未按下"的 1 当成"已到位"，
//      立刻跳到下一阶段 —— 腿根本没抬。
//      这个 bug 在仿真里看不出来（tb 直接驱动的是理想信号），
//      上板才暴露。
//
//   现在统一在本模块内做：原始输入低有效 -> 消抖 -> 输出高有效给 main_fsm。
//   main_fsm 保持 `if (lift_done)` 不变。
//
//-----------------------------------------------------------------------------
// 舵机 PWM 参数
//-----------------------------------------------------------------------------
//   标准航模舵机：周期 20ms(50Hz)，脉宽 0.5ms~2.5ms 对应 0°~180°。
//   本设计只用两个位置：
//       放下 = PWM_MIN_US (0.5ms, 0°)
//       抬起 = PWM_MAX_US (2.5ms, 180°)
//
//   ⚠️ 不同舵机的行程不一样。首次调试时把这两个值改小一点
//      （比如 1000/2000）避免顶到机械限位烧舵机。
//
//-----------------------------------------------------------------------------
// 与机械组的接口约定（见 doc/board_pinout.md）
//-----------------------------------------------------------------------------
//   lift_pwm[0] -> 前腿组舵机信号（前两个舵机信号线并联）
//   lift_pwm[1] -> 后腿组舵机信号（后两个舵机信号线并联）
//   lift_lim_n[0] <- 前腿抬起到位限位开关（常开，另一端接 GND）
//   lift_lim_n[1] <- 后腿抬起到位限位开关
//   lift_fault_n[1:0] <- 舵机故障（低有效；**不接时悬空靠上拉=1=正常**）
//
//   ⚠️ 舵机电源必须独立（5V/5A 以上），只把信号线和 GND 接过来。
//      和 FPGA 共用一路会因堵转压降把 FPGA 拉复位。
//=============================================================================

`timescale 1ns / 1ps

module servo_lift #(
    //----------------------------------------------------------------
    // 时钟
    //----------------------------------------------------------------
    parameter integer CLK_FREQ_HZ     = 50_000_000,

    //----------------------------------------------------------------
    // 舵机 PWM
    //   周期 20ms，脉宽 0.5~2.5ms
    //----------------------------------------------------------------
    parameter integer PWM_PERIOD_US   = 20_000,
    parameter integer PWM_MIN_US      = 500,     // 放下位置
    parameter integer PWM_MAX_US      = 2_500,   // 抬起位置

    //----------------------------------------------------------------
    // 输入消抖
    //   机械限位开关抖动典型 1~5ms，10ms 足够且不拖慢响应
    //----------------------------------------------------------------
    parameter integer DEBOUNCE_MS     = 10,

    //----------------------------------------------------------------
    // 断言用（仿真里校验参数合法）
    //----------------------------------------------------------------
    parameter integer _ASSERT_EN      = 1
) (
    input  wire        clk,
    input  wire        rst_n,          // 低有效

    //---------------- 来自 main_fsm 的意图 ----------------
    input  wire [1:0]  lift_cmd,       // {后腿, 前腿}  1=抬起

    //---------------- ★ 抬腿高度（2026-10-18 新增，CONFIG 帧 0x10）----------------
    //
    //   0   = 完全放下（PWM_MIN_US, 0.5ms）
    //   100 = 完全抬起（PWM_MAX_US, 2.5ms）← **默认值，与旧行为完全一致**
    //
    //   为什么需要连续高度：舵机带着腿往上顶时，如果机械上限比舵机行程小，
    //   会一直顶着堵转，舵机发热/齿箱受损。有了可调高度，可以在**不重新综合**
    //   的情况下把"抬起"限制在机构真正到得了的位置。
    //
    //   实现上没有除法器：`(PWM_MAX_US-PWM_MIN_US)*CYC_PER_US/100` 是**编译期常数**
    //   （50MHz、0.5~2.5ms 时 = 1us = 50 拍/单位），运行时只做一次常数乘。
    input  wire [7:0]  lift_height,    // 0..100（>100 按 100 处理）

    //---------------- 原始硬件输入（低有效）----------------
    input  wire [1:0]  lift_lim_n,     // 限位开关：0=已到位（常开+上拉）
    input  wire [1:0]  lift_fault_n,   // 舵机故障：0=故障（不接=1=正常）

    //---------------- 给舵机的 PWM ----------------
    output wire [1:0]  lift_pwm,

    //---------------- 消抖后的状态（高有效，给 main_fsm）----------------
    output wire [1:0]  lift_done,      // 1=抬起到位
    output wire [1:0]  lift_fault      // 1=故障
);

    //================================================================
    // 参数换算
    //================================================================
    localparam integer CLK_MHZ     = CLK_FREQ_HZ / 1_000_000;
    localparam integer CYC_PER_US  = (CLK_FREQ_HZ >= 1_000_000) ? (CLK_FREQ_HZ / 1_000_000) : 1;

    //----------------------------------------------------------------
    // ⚠️ 下面这几个声明成**定宽 [31:0]**，不是 integer。
    //
    //   原因：Verilog-2001 **不允许对 parameter/localparam 做位选**
    //   （`DEB_CYC` 这种）。ModelSim 能过，但综合器可能拒绝，
    //   而且报错信息很难懂。声明成定宽向量就根本不需要位选。
    //
    //   这是本项目踩过的坑之一，别再写回 integer。
    //----------------------------------------------------------------
    localparam [31:0] PERIOD_CYC  = PWM_PERIOD_US * CYC_PER_US;
    localparam [31:0] PWM_MIN_CYC = PWM_MIN_US    * CYC_PER_US;
    localparam [31:0] PWM_MAX_CYC = PWM_MAX_US    * CYC_PER_US;
    localparam [31:0] DEB_CYC     = (DEBOUNCE_MS * CLK_FREQ_HZ) / 1000;

    //------------------------------------------------------------------
    // ★ 抬腿高度换算（2026-10-18 新增）
    //
    //   STEP = (最大脉宽 - 最小脉宽) / 100  —— **编译期常数**，不需要除法器。
    //   50MHz、0.5~2.5ms 时：STEP = (2500-500)us * 50 / 100 = 1000 拍。
    //
    //   运行时只做一次乘法：cmp = MIN + height * STEP（height<=100）
    //   最坏 50000 + 100*1000 = 150000 拍，32 位绰绰有余。
    //------------------------------------------------------------------
    localparam [31:0] LIFT_STEP_CYC = ((PWM_MAX_US - PWM_MIN_US) * CYC_PER_US) / 100;

    // 高度钳位：>100 按 100 算（防御性，config_rx 也会钳）
    wire [7:0]  h_cl = (lift_height > 8'd100) ? 8'd100 : lift_height;

    wire [31:0] cmp_lift = PWM_MIN_CYC + ({24'd0, h_cl} * LIFT_STEP_CYC);


    // 参数合法性检查（编译期报错比上板调试便宜）
    initial begin
        if (PWM_MIN_CYC >= PWM_MAX_CYC) begin
            $display("[servo_lift] ERROR: PWM_MIN_US(%0d) 必须小于 PWM_MAX_US(%0d)",
                     PWM_MIN_US, PWM_MAX_US);
        end
        if (PWM_MAX_CYC >= PERIOD_CYC) begin
            $display("[servo_lift] ERROR: PWM_MAX_US(%0d) 必须小于 PWM_PERIOD_US(%0d)",
                     PWM_MAX_US, PWM_PERIOD_US);
        end
        if (DEB_CYC < 1) begin
            $display("[servo_lift] ERROR: DEBOUNCE_MS 太小，消抖计数为 0");
        end
    end

    //================================================================
    // 1) 输入消抖 + 极性统一
    //
    //    原始输入低有效。逐位做"连续 N 拍相同才认"的消抖，
    //    然后把极性翻成高有效。
    //
    //    为什么不用边沿检测：这里要的是**电平**（腿是否在抬起位置），
    //    不是事件。main_fsm 里 `if (lift_done)` 是电平判断。
    //================================================================
    genvar gi;
    generate
        for (gi = 0; gi < 2; gi = gi + 1) begin : g_deb

            //---------------- 限位开关 ----------------
            // 原始：0=到位。翻转成 raw_hi：1=到位
            wire raw_hi = ~lift_lim_n[gi];

            reg        st;
            reg [31:0] cnt;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    st  <= 1'b0;
                    cnt <= 32'd0;
                end else if (raw_hi == st) begin
                    cnt <= 32'd0;                 // 和当前状态一致，清计数
                end else begin
                    if (cnt >= DEB_CYC) begin
                        st  <= raw_hi;            // 稳定够久，认了
                        cnt <= 32'd0;
                    end else begin
                        cnt <= cnt + 32'd1;
                    end
                end
            end
            assign lift_done[gi] = st;

            //---------------- 故障 ----------------
            // 原始：0=故障。翻转成 raw_hi：1=故障
            // 不接时靠上拉读到 1 -> raw_hi=0 -> 无故障 ✓
            wire f_raw_hi = ~lift_fault_n[gi];

            reg        f_st;
            reg [31:0] f_cnt;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    f_st  <= 1'b0;
                    f_cnt <= 32'd0;
                end else if (f_raw_hi == f_st) begin
                    f_cnt <= 32'd0;
                end else begin
                    if (f_cnt >= DEB_CYC) begin
                        f_st  <= f_raw_hi;
                        f_cnt <= 32'd0;
                    end else begin
                        f_cnt <= f_cnt + 32'd1;
                    end
                end
            end
            assign lift_fault[gi] = f_st;

            //---------------- 舵机 PWM ----------------
            // 一个自由计数器数一个完整周期，
            // 比较器产生脉宽。两路共用相位（同周期），互相独立控制。
            reg [31:0] phase;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    phase <= 32'd0;
                end else if (phase >= PERIOD_CYC - 32'd1) begin
                    phase <= 32'd0;
                end else begin
                    phase <= phase + 32'd1;
                end
            end

            // 抬起时用"按高度换算出来的脉宽"，放下时用短脉宽。
            //
            //   cmp = MIN + height * STEP
            //     height=100 -> MIN + 100*STEP = PWM_MAX_CYC（和旧行为完全一致）
            //     height=0   -> MIN（= 放下的脉宽）
            //
            //   只用一次常数乘，没有除法器（STEP 是编译期常数）。
            wire [31:0] cmp = lift_cmd[gi] ? cmp_lift : PWM_MIN_CYC;

            assign lift_pwm[gi] = (phase < cmp) ? 1'b1 : 1'b0;
        end
    endgenerate

endmodule
