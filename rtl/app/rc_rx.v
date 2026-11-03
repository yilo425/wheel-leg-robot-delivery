//=============================================================================
// module : rc_rx
// layer  : 应用层（通信）
//-----------------------------------------------------------------------------
// 功能 : 从 0x05 RC_CMD 帧解析"手动遥控"指令，并做 **失效保护**。
//
//   为什么需要它：
//     遥控链路一旦断掉（WiFi 掉线 / K230 死机 / 页面关了），
//     如果 FPGA 还保持最后一帧的速度，机器人会**一直往前冲**。
//     所以这里做 500ms 看门狗：只要超过 TIMEOUT_MS 没收到合法帧，
//     rc_active 立刻归 0，上层据此把电机停掉。
//
//-----------------------------------------------------------------------------
// 载荷格式（3 字节）
//-----------------------------------------------------------------------------
//   +--------+--------+--------+
//   | steer  | speed  |  seq   |
//   | int8   | uint8  | uint8  |
//   +--------+--------+--------+
//
//   steer : -100 .. +100，正 = 右转（和 motion_decision 的 steer 同号约定）
//   speed : 0 .. 100 百分比；映射到 speed_scale：
//             0        -> 0（停）
//             1..50    -> 1（慢）
//             51..100  -> 2（满）
//   seq   : 发送端自增序号，本模块只统计"序号不连续"的次数（丢帧诊断）
//
//-----------------------------------------------------------------------------
// 为什么要 steer << 4
//-----------------------------------------------------------------------------
//   motion_decision 里转向项是 (steer >>> STEER_SHIFT)，STEER_SHIFT = 4。
//   视觉那条路的 steer 是"像素误差 >>> 6"，量级几十~几百。
//   遥控这边是百分比（-100..100），直接送进去转向项只有 ±6，几乎是直线。
//   所以这里乘 16（<<4）：steer=±100 时转向项正好 ±100，
//   满速下打满能压到一侧 0、另一侧限幅 120，也就是原地转。
//
//-----------------------------------------------------------------------------
// 端口
//-----------------------------------------------------------------------------
//   [来自 uart_frame 的接收接口]（和 vision_rx / config_rx 同一套）
//   [输出] rc_steer / rc_speed / rc_active / rc_frames / rc_bad / rc_seq_err
//=============================================================================

`default_nettype none

module rc_rx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer TIMEOUT_MS  = 500,
    parameter [7:0]   TYPE_RC_CMD = 8'h05,
    parameter integer RC_PL_LEN   = 3,

    // steer 放大倍数：<< STEER_GAIN_SHIFT
    parameter integer STEER_GAIN_SHIFT = 4,

    // 速度分档阈值（百分比）
    parameter integer SPEED_SLOW_MAX = 50
) (
    input  wire        clk,
    input  wire        rst_n,

    //---------------- 来自 uart_frame ----------------
    input  wire        rx_valid,
    input  wire [7:0]  rx_type,
    input  wire [15:0] rx_len,
    input  wire [7:0]  rx_pl_data,
    output reg  [7:0]  rx_pl_addr,

    //---------------- 输出 ----------------
    output reg  signed [15:0] rc_steer,
    output reg  [1:0]         rc_speed,
    output reg  [7:0]         rc_speed_pct,   // ★ 原始百分比 0~100（连续调速用）
    output reg                rc_rev,         // ★ 1=后退（speed 字节的 bit7）
    output reg                rc_active,
    output reg  [15:0]        rc_frames,
    output reg                rc_bad,
    output reg  [7:0]         rc_seq_err
);

    //=========================================================================
    // 常量
    //=========================================================================
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_B0   = 2'd1;   // 地址=0，下一拍取 byte0
    localparam [1:0] S_B1   = 2'd2;
    localparam [1:0] S_B2   = 2'd3;

    localparam integer CYC_PER_MS  = CLK_FREQ_HZ / 1000;
    localparam integer TICK_PER_MS = (CYC_PER_MS > 0) ? CYC_PER_MS : 1;

    //=========================================================================
    // 所有寄存器先声明（项目静态检查要求"先声明后使用"）
    //=========================================================================
    // 1ms 节拍
    reg [15:0] tick_cnt;
    reg        ms_tick;

    // 看门狗
    reg [15:0] idle_ms;

    // 载荷读取
    reg [1:0]  st;
    reg [7:0]  b0, b1, b2;
    reg        frame_ok;       // 单拍：本帧三字节解析完成
    reg [15:0] frames_cnt;

    // 序号统计
    reg [7:0]  last_seq;
    reg        have_seq;
    reg [7:0]  seq_err_cnt;

    // 组合
    wire signed [7:0]  steer_i8;
    wire signed [15:0] steer_ext;

    assign steer_i8  = b0;
    assign steer_ext = {{8{steer_i8[7]}}, steer_i8};

    //=========================================================================
    // 1ms 节拍
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 16'd0;
            ms_tick  <= 1'b0;
        end else begin
            if (tick_cnt >= (TICK_PER_MS[15:0] - 16'd1)) begin
                tick_cnt <= 16'd0;
                ms_tick  <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 16'd1;
                ms_tick  <= 1'b0;
            end
        end
    end

    //=========================================================================
    // 看门狗：收到合法帧清零，否则每 ms 加 1（饱和在 TIMEOUT_MS）
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            idle_ms <= TIMEOUT_MS[15:0];   // ★ 复位即"已超时"，rc_active=0
        else if (frame_ok)
            idle_ms <= 16'd0;
        else if (ms_tick && (idle_ms < TIMEOUT_MS[15:0]))
            idle_ms <= idle_ms + 16'd1;
    end

    //=========================================================================
    // 载荷读取状态机
    //   uart_frame 的 rx_pl_data 是组合输出（地址 -> 下一拍数据有效），
    //   所以每个状态先摆地址，下一个状态再取数据。
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            rx_pl_addr <= 8'd0;
            b0         <= 8'd0;
            b1         <= 8'd0;
            b2         <= 8'd0;
            frame_ok   <= 1'b0;
            rc_bad     <= 1'b0;
        end else begin
            frame_ok <= 1'b0;
            rc_bad   <= 1'b0;

            case (st)
                //---------------------------------------------------------
                S_IDLE: begin
                    rx_pl_addr <= 8'd0;
                    if (rx_valid && (rx_type == TYPE_RC_CMD)) begin
                        if (rx_len != RC_PL_LEN[15:0])
                            rc_bad <= 1'b1;      // 长度不对，整帧丢弃
                        else
                            st <= S_B0;
                    end
                end
                //---------------------------------------------------------
                S_B0: begin
                    rx_pl_addr <= 8'd1;
                    b0         <= rx_pl_data;    // byte0 = steer
                    st         <= S_B1;
                end
                //---------------------------------------------------------
                S_B1: begin
                    rx_pl_addr <= 8'd2;
                    b1         <= rx_pl_data;    // byte1 = speed
                    st         <= S_B2;
                end
                //---------------------------------------------------------
                S_B2: begin
                    b2       <= rx_pl_data;      // byte2 = seq
                    frame_ok <= 1'b1;
                    st       <= S_IDLE;
                end
                //---------------------------------------------------------
                default: st <= S_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 指令寄存器（只在 frame_ok 时更新）
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rc_steer   <= 16'sd0;
            rc_speed   <= 2'd0;
            rc_speed_pct <= 8'd0;
            rc_rev     <= 1'b0;
            frames_cnt <= 16'd0;
        end else if (frame_ok) begin
            // steer 饱和到 -100..100，再 << 放大
            if (steer_i8 > 8'sd100)
                rc_steer <= 16'sd100 <<< STEER_GAIN_SHIFT;
            else if (steer_i8 < -8'sd100)
                rc_steer <= -(16'sd100 <<< STEER_GAIN_SHIFT);
            else
                rc_steer <= steer_ext <<< STEER_GAIN_SHIFT;

            //----------------------------------------------------------
            // ★ 2026-10-30：speed 字节扩展
            //     bit7   = 后退标志（0=前进 1=后退）
            //     bit6:0 = 百分比 0~100（连续调速）
            //   兼容老帧：老帧 bit7 恒 0 -> 前进，行为和以前一致
            //----------------------------------------------------------
            rc_rev <= b1[7];
            if (b1[6:0] > 7'd100)
                rc_speed_pct <= 8'd100;
            else
                rc_speed_pct <= {1'b0, b1[6:0]};

            // 档位（保留给自动模式/兼容用）
            if (b1[6:0] == 7'd0)
                rc_speed <= 2'd0;
            else if (b1[6:0] <= SPEED_SLOW_MAX[7:0])
                rc_speed <= 2'd1;
            else
                rc_speed <= 2'd2;

            frames_cnt <= frames_cnt + 16'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rc_frames <= 16'd0;
        else
            rc_frames <= frames_cnt;
    end

    //=========================================================================
    // 序号连续性统计（只做诊断，不影响输出）
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_seq   <= 8'd0;
            have_seq   <= 1'b0;
            seq_err_cnt<= 8'd0;
        end else if (frame_ok) begin
            if (have_seq && (b2 != (last_seq + 8'd1)))
                seq_err_cnt <= seq_err_cnt + 8'd1;
            last_seq <= b2;
            have_seq <= 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rc_seq_err <= 8'd0;
        else
            rc_seq_err <= seq_err_cnt;
    end

    //=========================================================================
    // rc_active：看门狗未超时
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rc_active <= 1'b0;
        else
            rc_active <= (idle_ms < TIMEOUT_MS[15:0]);
    end

endmodule

`default_nettype wire
