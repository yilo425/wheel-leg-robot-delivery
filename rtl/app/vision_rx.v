//=============================================================================
// module : vision_rx
// layer  : 应用层（通信）
//-----------------------------------------------------------------------------
// 功能 : 从 uart_frame 收到的一帧 0x02 VISION_RESULT 里解析出：
//          - 视觉类别  (vision_class)
//          - 转向指令  (steer, 有符号, 正=右转)
//          - 数据有效  (vision_valid)
//
//-----------------------------------------------------------------------------
// 端口
//-----------------------------------------------------------------------------
//   clk / rst_n          系统时钟与复位(低有效)
//
//   [来自 uart_frame 的接收接口]
//   rx_valid             单拍脉冲：收到一个合法帧
//   rx_type              帧类型（与 rx_valid 同拍有效）
//   rx_len               载荷长度（与 rx_valid 同拍有效）
//   rx_pl_data           载荷读出数据（组合）
//   rx_pl_addr           载荷读出地址（本模块输出）
//
//   [给 main_fsm / motion_decision]
//   vision_class[7:0]    选中目标的类别码
//   vision_valid         单拍脉冲：有新的一帧视觉结果
//   steer[15:0]          有符号转向指令，正=右转
//   target_x[15:0]       选中目标的归一化 x 中心（调试/上报用）
//   target_conf[7:0]     选中目标的置信度
//   vision_bad           单拍脉冲：收到视觉帧但格式不对
//   frame_cnt / bad_cnt  累计收到帧数 / 丢弃帧数（可上报）
//
//-----------------------------------------------------------------------------
// VISION_RESULT 载荷格式（见 doc/uart_frame_protocol.md §3.1）
//-----------------------------------------------------------------------------
//   +--------+--------+--------+--------+--------+--------+
//   | 目标数 | 序号   | 类别   | x 中心 | y 中心 | 置信度 |
//   | 1B     | 1B     | 1B     | 2B     | 2B     | 1B     |
//   +--------+--------+--------+--------+--------+--------+
//      每个目标 7 字节，重复「目标数」次；多字节一律**大端**
//      载荷总长 = 1 + 7n
//
//   偏移：  0=目标数 | 1+i*7+0=序号  +1=类别  +2,+3=x(大端)  +4,+5=y  +6=置信度
//
//-----------------------------------------------------------------------------
// 选目标策略
//-----------------------------------------------------------------------------
//   **取置信度最高的那个目标。**
//
//   为什么不用"离中心最近的"：置信度是 ARM 侧对"这个检测有多可信"的量化，
//   选它可以让误检尽可能少地影响决策。载荷里没有距离信息（只有归一化像素
//   坐标），所以"最近"也无从判断。
//
//   要改成别的策略，只改 pick 那一小段（搜索 `最优目标` 注释）。
//
//-----------------------------------------------------------------------------
// 坐标 -> 转向换算
//-----------------------------------------------------------------------------
//   x 中心归一化到 0..65535（与摄像头分辨率解耦），画面正中心 = 32768。
//
//       e = x - 32768                      有符号，范围约 ±32768
//       e' = e - sign(e)*STEER_DEAD        先减死区，保证在死区边界是连续的
//       steer = clamp(e' >>> STEER_SHIFT, ±STEER_MAX)
//
//   * 减死区（而不是"小于死区就归零"）是为了**避免阶跃**：
//     直接归零的话，x 从 32767 变到 30719（刚出死区）时 steer 会从 0
//     突跳到 -32，车会"咯噔"一下。减死区后是平滑的。
//
//   steer 的量纲由 motion_decision 决定：
//       motion_decision 里  bias = steer >>> STEER_SHIFT_M
//                           target = base + bias × STEER_COEFi
//   默认 STEER_SHIFT=6 -> e/64 -> 最大 ±512，
//   配 motion_decision 的 STEER_SHIFT=4 得到 bias ±32
//   （base 满速时是 VREF>>1 = 30），转向作用明显但不至于失控。
//
//   ** 这个系数**必须在实车上调**：把车放地上，让 ARM 发固定 x 的帧，
//      看转向是否过冲。过冲就加大 STEER_SHIFT（输出变小）。
//
//   [!]️ 已知的固有不对称（不是 bug，不要"修"它）：
//      算术右移对负数是**向下取整**，正数是截断：
//          +7952 >>> 6 =  124      （+124.25 截断）
//          -7952 >>> 6 = -125      （-124.25 向下取整，多走一格）
//      所以左右转向的输出绝对值会差 1 个 LSB（约 0.2%）。
//      影响可以忽略：转向是**比例控制**（无积分项），不会累积成航向偏置。
//      tools/verify_vision_rx_model.py 里把这个差值显式钉住了，
//      以后改了换算公式会立刻被测出来。
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) 只认 TYPE_VISION_RESULT；其它类型（ACK/CONFIG 等）直接忽略。
//
// 2) **长度必须严格校验**：载荷长必须正好等于 1 + 7n，且 n <= MAX_TARGETS。
//    不合法就整帧丢弃并报 vision_bad ——
//    **绝不能按错长度去读 rx_buf**，那会读到上一帧的残留数据，
//    产生"看起来合理但完全错误"的视觉结果，比直接报错危险得多。
//
// 3) 目标数 = 0 是**合法**的（"这一帧没看到东西"）。
//    这时仍然输出 vision_valid=1 且 vision_class=0（无目标）——
//    上层需要区分"没看到"和"通信断了"。两者的 fail-safe 行为不同。
//
// 4) 读载荷用的是 uart_frame 的 rx_buf，它会**保持到下一帧**，
//    所以可以慢慢读（一帧最少 86.8us @115200，够几千个时钟）。
//    但仍设了超时保护：万一 rx_buf 被提前改写，不会永远卡住。
//
// 5) 输出保持策略：vision_class/steer/target_x **保持到下一帧**，
//    vision_valid 是**单拍脉冲**。这样 main_fsm 每帧收到一个脉冲，
//    而 motion_decision 始终有稳定的 steer 可用。
//=============================================================================
`timescale 1ns/1ps

module vision_rx #(
    //------------------------------------------------------------------
    // 协议常量
    //------------------------------------------------------------------
    parameter [7:0]   TYPE_VISION_RESULT = 8'h02,
    parameter integer MAX_TARGETS        = 8,        // 协议上限

    //------------------------------------------------------------------
    // 坐标 -> 转向
    //------------------------------------------------------------------
    parameter integer X_CENTER    = 32768,  // 画面中心
    parameter integer STEER_DEAD  = 2048,   // 死区半宽（约 ±3%）
    parameter integer STEER_SHIFT = 6,      // steer = e' >>> STEER_SHIFT
    parameter integer STEER_MAX   = 512,    // 钳位幅值

    //------------------------------------------------------------------
    // 超时保护
    //   [!]️ 任何等待都要有超时, 否则一旦 uart_frame 行为异常就永久卡死
    //------------------------------------------------------------------
    parameter integer SCAN_TIMEOUT_CYCLES = 100_000
) (
    input  wire        clk,
    input  wire        rst_n,

    //---- 来自 uart_frame ----
    input  wire        rx_valid,
    input  wire [7:0]  rx_type,
    input  wire [7:0]  rx_len,
    input  wire [7:0]  rx_pl_data,
    output reg  [7:0]  rx_pl_addr,

    //---- 给上层 ----
    output reg  [7:0]  vision_class,
    output reg         vision_valid,
    output reg  signed [15:0] steer,
    output reg  [15:0] target_x,
    output reg  [7:0]  target_conf,
    output reg         vision_bad,
    output reg  [15:0] frame_cnt,
    output reg  [15:0] bad_cnt
);

    //==================================================================
    // 0) 定宽常量
    //
    //   [!]️ Verilog-2001 **不允许对 parameter 做位选**：
    //         X_CENTER[16:0]        <-- 非法
    //      必须先转成定宽 localparam 再用。
    //      本工程在这个坑上栽过 4 次，rtl_lint.py 里有自动检查。
    //==================================================================
    localparam [7:0]  MAX_TARGETS_8 = MAX_TARGETS;

    localparam signed [16:0] X_CENTER_S  = X_CENTER;
    localparam signed [16:0] STEER_DEAD_S = STEER_DEAD;
    localparam signed [16:0] STEER_MAX_S  = STEER_MAX;

    localparam [31:0] TMO_MAX = SCAN_TIMEOUT_CYCLES;

    // 状态
    localparam [2:0] S_IDLE = 3'd0,
                     S_HDR  = 3'd1,
                     S_SCAN = 3'd2,
                     S_DONE = 3'd3;

    //==================================================================
    // 1) 期望载荷长度查表
    //
    //   合法长度只有 1, 8, 15, 22, 29, 36, 43, 50, 57（= 1 + 7n）。
    //   用查表而不是 n*7，既避免综合出乘法器，也把"合法长度"写得更明白。
    //==================================================================
    function [7:0] expected_len;
        input [7:0] n;
        begin
            case (n)
                8'd0: expected_len = 8'd1;
                8'd1: expected_len = 8'd8;
                8'd2: expected_len = 8'd15;
                8'd3: expected_len = 8'd22;
                8'd4: expected_len = 8'd29;
                8'd5: expected_len = 8'd36;
                8'd6: expected_len = 8'd43;
                8'd7: expected_len = 8'd50;
                8'd8: expected_len = 8'd57;
                default: expected_len = 8'hFF;   // 不可能等于任何合法 len
            endcase
        end
    endfunction

    //==================================================================
    // 2) 状态与寄存器
    //==================================================================
    reg [2:0]  state;
    reg [7:0]  n_targets;
    reg [3:0]  tidx;          // 当前第几个目标
    reg [2:0]  bidx;          // 当前目标内的第几字节 (0..6)
    reg        len_ok;

    // 当前目标
    reg [7:0]  cur_cls;
    reg [15:0] cur_x;
    reg [7:0]  cur_conf;

    // 目前最优
    reg [7:0]  best_cls;
    reg [15:0] best_x;
    reg [7:0]  best_conf;
    reg        has_best;

    reg [31:0] tmo;

    //==================================================================
    // 3) 载荷地址（组合）
    //      目标 i 的 7 字节偏移 = 1 + i*7
    //      i*7 = i*8 - i，用移位减法，避免乘法器
    //==================================================================
    wire [7:0] tidx_x8   = {4'd0, tidx, 3'b000};        // tidx * 8
    wire [7:0] scan_base = 8'd1 + tidx_x8 - {4'd0, tidx}; // 1 + tidx*7

    always @(*) begin
        case (state)
            S_HDR  : rx_pl_addr = 8'd0;                              // 目标数
            S_SCAN : rx_pl_addr = scan_base + {5'd0, bidx};
            default: rx_pl_addr = 8'd0;
        endcase
    end

    //==================================================================
    // 4) 坐标 -> 转向（纯组合，由 best_x 算出）
    //
    //   用组合而不是在时序块里做临时变量：
    //   时序块里用 blocking 赋值当临时变量是能跑，但可读性差、
    //   而且容易和同块里的非阻塞赋值混出微妙的顺序问题。
    //==================================================================
    wire signed [16:0] e_full = $signed({1'b0, best_x}) - X_CENTER_S;

    // 减死区：在死区外才按超出量算，保证死区边界连续
    wire signed [16:0] e_dead =
        (e_full >  STEER_DEAD_S) ? (e_full - STEER_DEAD_S) :
        (e_full < -STEER_DEAD_S) ? (e_full + STEER_DEAD_S) :
                                   17'sd0;

    wire signed [16:0] e_shr = e_dead >>> STEER_SHIFT;

    wire signed [16:0] e_clamp =
        (e_shr >  STEER_MAX_S) ?  STEER_MAX_S :
        (e_shr < -STEER_MAX_S) ? -STEER_MAX_S :
                                  e_shr;

    //==================================================================
    // 5) 主状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            n_targets    <= 8'd0;
            tidx         <= 4'd0;
            bidx         <= 3'd0;
            len_ok       <= 1'b0;
            cur_cls      <= 8'd0;
            cur_x        <= 16'd0;
            cur_conf     <= 8'd0;
            best_cls     <= 8'd0;
            best_x       <= 16'd0;
            best_conf    <= 8'd0;
            has_best     <= 1'b0;
            tmo          <= 32'd0;
            vision_class <= 8'd0;
            vision_valid <= 1'b0;
            steer        <= 16'sd0;
            target_x     <= 16'd0;
            target_conf  <= 8'd0;
            vision_bad   <= 1'b0;
            frame_cnt    <= 16'd0;
            bad_cnt      <= 16'd0;
        end else begin
            // 默认：单拍脉冲，每拍自动落回 0
            vision_valid <= 1'b0;
            vision_bad   <= 1'b0;

            case (state)
            //----------------------------------------------------------
            S_IDLE: begin
                tmo <= 32'd0;
                if (rx_valid && (rx_type == TYPE_VISION_RESULT)) begin
                    frame_cnt <= frame_cnt + 16'd1;
                    state     <= S_HDR;
                end
            end

            //----------------------------------------------------------
            // 读目标数 + 严格校验长度
            //----------------------------------------------------------
            S_HDR: begin
                n_targets <= rx_pl_data;
                len_ok    <= (rx_len == expected_len(rx_pl_data)) &&
                             (rx_pl_data <= MAX_TARGETS_8);
                tidx      <= 4'd0;
                bidx      <= 3'd0;
                has_best  <= 1'b0;
                best_conf <= 8'd0;
                best_x    <= 16'd0;
                best_cls  <= 8'd0;
                state     <= S_SCAN;
            end

            //----------------------------------------------------------
            // 逐个目标读 7 字节
            //----------------------------------------------------------
            S_SCAN: begin
                tmo <= tmo + 32'd1;

                if (tmo >= TMO_MAX) begin
                    // 超时兜底：绝不永久卡死
                    vision_bad <= 1'b1;
                    bad_cnt    <= bad_cnt + 16'd1;
                    state      <= S_IDLE;

                end else if (!len_ok) begin
                    // 长度不合法 -> 整帧丢弃，**不读缓冲**
                    vision_bad <= 1'b1;
                    bad_cnt    <= bad_cnt + 16'd1;
                    state      <= S_IDLE;

                end else if (rx_valid && (rx_type == TYPE_VISION_RESULT)) begin
                    //--------------------------------------------------
                    // 扫描途中又来了新帧。
                    //
                    //   uart_frame 的 rx_buf 是**单缓冲**，新帧一到就把
                    //   旧数据覆盖掉，继续按旧 n_targets 扫描会读到错位数据。
                    //
                    //   理论上不会发生：扫完一帧最多约 7*8+2 = 58 拍，
                    //   而 115200 波特下最短帧间隔是 86.8us = 4340 拍 @50MHz，
                    //   余量 75 倍。但这是"靠余量"而不是"靠设计"，
                    //   所以显式处理：直接用新帧重新开始。
                    //--------------------------------------------------
                    frame_cnt <= frame_cnt + 16'd1;
                    state     <= S_HDR;

                end else if (n_targets == 8'd0) begin
                    // 合法但"无目标"
                    state <= S_DONE;

                end else begin
                    // 按 bidx 收集当前目标的字节
                    case (bidx)
                        3'd0: ;                                  // 序号，不用
                        3'd1: cur_cls     <= rx_pl_data;
                        3'd2: cur_x[15:8] <= rx_pl_data;         // 大端：高字节在前
                        3'd3: cur_x[7:0]  <= rx_pl_data;
                        3'd4: ;                                  // y 高字节，本模块不用
                        3'd5: ;                                  // y 低字节
                        3'd6: cur_conf    <= rx_pl_data;
                        default: ;
                    endcase

                    if (bidx == 3'd6) begin
                        bidx <= 3'd0;
                        if (tidx + 4'd1 >= {4'd0, n_targets}) begin
                            state <= S_DONE;                     // 全部读完
                        end else begin
                            tidx <= tidx + 4'd1;
                        end
                    end else begin
                        bidx <= bidx + 3'd1;
                    end
                end
            end

            //----------------------------------------------------------
            // 输出
            //----------------------------------------------------------
            S_DONE: begin
                if ((n_targets == 8'd0) || !has_best) begin
                    // 合法但无目标
                    vision_class <= 8'h00;
                    target_x     <= 16'd0;
                    target_conf  <= 8'd0;
                    steer        <= 16'sd0;
                end else begin
                    vision_class <= best_cls;
                    target_x     <= best_x;
                    target_conf  <= best_conf;
                    steer        <= e_clamp[15:0];
                end
                vision_valid <= 1'b1;
                state        <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase

            //----------------------------------------------------------
            // 最优目标挑选
            //
            //   [!]️ 用 rx_pl_data（当前正读到的置信度字节）而不是 cur_conf：
            //      cur_conf 要到**下一拍**才更新，那时 bidx 已经回 0 了。
            //
            //   这一块和上面的 case 在同一个 always 里，
            //   它在 bidx==6 那一拍同时写 best_*，而 case 在同拍写 state<=S_DONE，
            //   所以到 S_DONE 时 best_* 已经是最终值。
            //----------------------------------------------------------
            if ((state == S_SCAN) && len_ok &&
                (n_targets != 8'd0) && (bidx == 3'd6)) begin
                if (!has_best || (rx_pl_data > best_conf)) begin
                    best_conf <= rx_pl_data;
                    best_cls  <= cur_cls;
                    best_x    <= cur_x;
                    has_best  <= 1'b1;
                end
            end
        end
    end

endmodule
