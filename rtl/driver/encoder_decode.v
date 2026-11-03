//=============================================================================
// module : encoder_decode
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : 增量式编码器正交解码 + 定窗测速
//
// 设计要点:
//   1) A/B 先过 2 级同步器 (跨时钟域/异步输入必做), 再过可选数字滤波。
//      FILTER_CYCLES>0 时, 只有新电平连续保持该周期数才被采信,
//      用来滤掉机械抖动与线缆串扰引起的毛刺; 设为 0 则直通。
//   2) 4 倍频解码: 用 prev 和当前相位的组合查表得 +1/0/-1。
//      非法跳变(两位同时变) 记 0, 不脏计数; 电机静止时该计数为 0。
//   3) 位置 pos_cnt 为 32 位有符号累计值, 由上层换算成毫米/角度。
//   4) 速度 speed_cnt 每 SPEED_WINDOW_CYCLES 个周期刷新一次,
//      含义是 "本窗口内累计的 4 倍频计数", 符号表示方向。
//      换算物理速度: v = speed_cnt / (4 * PPR) / (WINDOW/CLK_FREQ_HZ)  转/秒
//      例如 PPR=13, 4倍频=52, 窗口=10ms -> v(rps) = speed_cnt / 0.52
//   5) 本模块只做计数与测速, 不做 PID。PID 放在 app/motion_decision.v。
//=============================================================================
`timescale 1ns/1ps

module encoder_decode #(
    parameter integer FILTER_CYCLES       = 8,        // 输入滤波窗口; 0=关闭
    parameter integer SPEED_WINDOW_CYCLES = 1_000_000,// 测速窗口 (clk 周期)
    parameter integer POS_WIDTH           = 32,       // 位置计数位宽
    parameter integer SPEED_WIDTH         = 24        // 速度输出位宽
) (
    input  wire                    clk,
    input  wire                    rst_n,      // 低有效
    input  wire                    enc_a,      // 编码器 A 相 (异步)
    input  wire                    enc_b,      // 编码器 B 相 (异步)
    output reg  signed [POS_WIDTH-1:0]   pos_cnt,   // 有符号累计位置
    output reg  signed [SPEED_WIDTH-1:0] speed_cnt, // 有符号窗口速度
    output reg                     dir,        // 1=正转 0=反转 (最近一次跳变方向)
    output wire                    speed_valid,// 窗口刷新脉冲 (1 clk)
    output reg  [31:0]             window_cnt  // 窗口内计数(调试)
);

    //------------------------------------------------------------------
    // 1) 两级同步器
    //------------------------------------------------------------------
    reg a_s1, a_s2, b_s1, b_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_s1 <= 1'b0; a_s2 <= 1'b0;
            b_s1 <= 1'b0; b_s2 <= 1'b0;
        end else begin
            a_s1 <= enc_a;  a_s2 <= a_s1;
            b_s1 <= enc_b;  b_s2 <= b_s1;
        end
    end

    //------------------------------------------------------------------
    // 2) 可选数字滤波: 新值需连续保持 FILTER_CYCLES 个周期才生效
    //------------------------------------------------------------------
    wire [1:0] raw_ab = {a_s2, b_s2};
    reg  [1:0] ab_filt;
    reg  [1:0] ab_hold;
    reg [15:0] hold_cnt;

    generate
        if (FILTER_CYCLES == 0) begin : g_nofilt
            always @(*) ab_filt = raw_ab;
        end else begin : g_filt
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    ab_filt  <= 2'b00;
                    ab_hold  <= 2'b00;
                    hold_cnt <= 16'd0;
                end else if (raw_ab != ab_hold) begin
                    // 电平变化, 重新计时
                    ab_hold  <= raw_ab;
                    hold_cnt <= 16'd0;
                end else if (hold_cnt >= FILTER_CYCLES[15:0] - 16'd1) begin
                    ab_filt <= ab_hold;      // 稳定足够久, 采信
                end else begin
                    hold_cnt <= hold_cnt + 16'd1;
                end
            end
        end
    endgenerate

    //------------------------------------------------------------------
    // 3) 4 倍频正交解码查表
    //    前向序列: 00 -> 01 -> 11 -> 10 -> 00
    //    idx = {prev[1:0], cur[1:0]}
    //------------------------------------------------------------------
    wire [1:0] cur_ab;
    reg  [1:0] prev_ab;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) prev_ab <= 2'b00;
        else        prev_ab <= cur_ab;
    end

    assign cur_ab = ab_filt;

    reg [1:0] steps;   // 0=无, 1=+1, 2=-1

    always @(*) begin
        case ({prev_ab, cur_ab})
            4'b0001, 4'b0111, 4'b1110, 4'b1000: steps = 2'd1;  // 前向
            4'b0010, 4'b0100, 4'b1101, 4'b1011: steps = 2'd2;  // 后向
            default:                            steps = 2'd0;  // 静止或非法跳变
        endcase
    end

    // 2 位方向编码 + 幅值, 全部无符号, 只在最后回读时按有符号解释:
    //   steps=1 -> 前进(count+1), steps=2 -> 后退(count-1), steps=0 -> 不变。
    // 说明: 不直接把 -1 写进有符号常量, 是为了避开 "拼接后整体当有符号数"
    //       时的扩展坑, 逻辑更直白, 也不会被综合器误推断。
    wire [31:0] step_mag = (steps == 2'd0) ? 32'd0 : 32'd1;
    wire        step_up  = (steps == 2'd1);

    //------------------------------------------------------------------
    // 4) 位置累计 + 方向锁存
    //------------------------------------------------------------------
    wire step_act = (steps != 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_cnt <= {POS_WIDTH{1'b0}};
            dir     <= 1'b0;
        end else if (step_act) begin
            pos_cnt <= step_up ? (pos_cnt + step_mag[POS_WIDTH-1:0])
                               : (pos_cnt - step_mag[POS_WIDTH-1:0]);
            dir     <= step_up;
        end
    end

    //------------------------------------------------------------------
    // 5) 定窗测速
    //    正反向分别累加, 窗口结束时两者相减得到带符号速度,
    //    彻底避免有符号/无符号混用带来的扩展问题。
    //------------------------------------------------------------------
    reg [31:0] fwd_acc;
    reg [31:0] rev_acc;
    reg [31:0] cyc_cnt;

    assign speed_valid = (cyc_cnt == SPEED_WINDOW_CYCLES[31:0] - 32'd1);

    // 帧内净计数 = 正向 - 反向, 结果按有符号解释
    wire signed [31:0] net_cnt = $signed(fwd_acc) - $signed(rev_acc);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cyc_cnt    <= 32'd0;
            fwd_acc    <= 32'd0;
            rev_acc    <= 32'd0;
            speed_cnt  <= {SPEED_WIDTH{1'b0}};
            window_cnt <= 32'd0;
        end else begin
            if (step_act) begin
                if (step_up) fwd_acc <= fwd_acc + 32'd1;
                else         rev_acc <= rev_acc + 32'd1;
            end

            if (speed_valid) begin
                cyc_cnt    <= 32'd0;
                fwd_acc    <= 32'd0;
                rev_acc    <= 32'd0;
                window_cnt <= net_cnt;
                speed_cnt  <= net_cnt[SPEED_WIDTH-1:0];
            end else begin
                cyc_cnt <= cyc_cnt + 32'd1;
            end
        end
    end

endmodule
