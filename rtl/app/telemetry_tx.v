//=============================================================================
// module : telemetry_tx
// layer  : 应用层（通信）
//-----------------------------------------------------------------------------
// 功能 : 周期性把机器人状态组一帧 0x10 TELEMETRY，通过 uart_frame 发给 ARM。
//
//   为什么需要它：
//     ARM 只发视觉结果，对 FPGA 侧的状态一无所知。没有这个回传，
//     联调时"车为什么不动"只能靠猜。有了 TELEMETRY，ARM 能直接看到
//     状态机在哪、四个轮子转多快、前面有没有障碍。
//
//-----------------------------------------------------------------------------
// 端口
//-----------------------------------------------------------------------------
//   [状态输入 —— 都是现成的内部信号]
//   state[2:0]          主状态机状态（main_fsm.state）
//   spd_l / spd_r       左右轮速度（encoder_decode.speed_cnt，有符号 24 位）
//   dist_mm[15:0]       TOF 距离
//   roll/pitch/yaw      姿态角（0.01 度/LSB，有符号）—— IMU 未接入时给 0
//
//   [到 uart_frame 的发送接口]
//   tx_pl_wr / tx_pl_addr / tx_pl_data   写发送缓冲
//   tx_req / tx_type / tx_len            请求发送
//   tx_busy / tx_done                    状态回读
//
//-----------------------------------------------------------------------------
// TELEMETRY 载荷格式（见 doc/uart_frame_protocol.md §3.3）
//-----------------------------------------------------------------------------
//   +--------+--------+--------+--------+--------+--------+--------+
//   | 状态   | 速度L  | 速度R  | 距离   | roll   | pitch  | yaw    |
//   | 1B     | 2B     | 2B     | 2B     | 2B     | 2B     | 2B     |
//   +--------+--------+--------+--------+--------+--------+--------+
//      总长 = 1 + 6*2 = 13 字节
//      多字节一律**大端**；速度与姿态角是**有符号补码**
//
//   [!]️ 载荷长度是 13 不是 17。
//      文档里曾把 7 个字段的字节数算成 17（误把"字段个数"当成了别的）。
//      本模块用 PAYLOAD_LEN 常量 + 自检断言把它钉死。
//
//-----------------------------------------------------------------------------
// 速度的位宽处理
//-----------------------------------------------------------------------------
//   encoder_decode 的 speed_cnt 是 **24 位有符号**，而协议里只有 2 字节。
//   直接截低 16 位会"回绕"：+40000 截成负数，看起来像突然倒车。
//   **必须饱和**（saturate），不能截断：
//       >  32767 ->  32767
//       < -32768 -> -32768
//       否则 -> 原值低 16 位
//
//   24 位 @10ms 窗口能到 ±8388607，实际转速远小于 32767，
//   所以饱和在正常工况下不会触发 —— 但异常（编码器抖动/干扰）时会，
//   那时"读数饱和了"比"读数变成负的"安全得多。
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) **必须先写完缓冲再拉 tx_req**。uart_frame 在收到 tx_req 的那一刻
//    才锁存 tx_type/tx_len 并开始发送；缓冲是随写随存的。
//
// 2) **只有在 !tx_busy 时才发起新帧**。uart_frame 的 T_IDLE 之外的
//    状态不接受 tx_req，但显式判断一下能让"漏帧"变得可见
//    （通过 drop_cnt 计数），而不是静默丢掉。
//
// 3) **等待 tx_done 有超时**。没超时的话，万一 uart_frame 卡住，
//    本模块就永久停在上报循环里 —— 看不见任何现象。
//
// 4) 发送周期用 CLK_FREQ_HZ / TX_RATE_HZ 算，换 PLL 频率时不用改代码，
//    只要把 CLK_FREQ_HZ 同步改掉。**这两个值必须和实际时钟一致**。
//=============================================================================
`timescale 1ns/1ps

module telemetry_tx #(
    //------------------------------------------------------------------
    // 时钟与上报频率
    //------------------------------------------------------------------
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer TX_RATE_HZ   = 10,          // 10Hz = 每 100ms 一帧

    //------------------------------------------------------------------
    // 协议
    //------------------------------------------------------------------
    parameter [7:0]   TYPE_TELEMETRY = 8'h10,
    parameter integer PAYLOAD_LEN    = 13,        // 1 + 6*2

    //------------------------------------------------------------------
    // 超时保护
    //   等 tx_done 最多等这么久。115200 波特下一帧 13 字节约 1.13ms，
    //   给 100ms 余量非常充足。
    //------------------------------------------------------------------
    parameter integer WAIT_TIMEOUT_MS = 100
) (
    input  wire        clk,
    input  wire        rst_n,

    //---- 状态输入 ----
    input  wire [2:0]  state,
    input  wire [3:0]  flags,        // ★ [3]=goal_reached [2]=spd_stale [1]=limited [0]=error
    input  wire signed [23:0] spd_l,
    input  wire signed [23:0] spd_r,
    input  wire [15:0] dist_mm,
    input  wire signed [15:0] roll,
    input  wire signed [15:0] pitch,
    input  wire signed [15:0] yaw,

    //---- 到 uart_frame ----
    output reg         tx_pl_wr,
    output reg  [7:0]  tx_pl_addr,
    output reg  [7:0]  tx_pl_data,
    output reg         tx_req,
    output reg  [7:0]  tx_type,
    output reg  [7:0]  tx_len,
    input  wire        tx_busy,
    input  wire        tx_done,

    //---- 统计（调试/上报用）----
    output reg  [15:0] sent_cnt,
    output reg  [15:0] dropped_cnt,   // 因为 tx_busy 而错过的时间片
    output reg         wait_timeout   // 单拍：等 tx_done 超时了
);

    //==================================================================
    // 0) 定宽常量（Verilog-2001 不允许对 parameter 做位选）
    //==================================================================
    localparam [7:0]  PAYLOAD_LEN_8 = PAYLOAD_LEN;

    localparam integer TICK_CYCLES = CLK_FREQ_HZ / TX_RATE_HZ;   // 50M/10 = 5,000,000
    localparam [31:0] TICK_MAX     = TICK_CYCLES;

    localparam integer MS_CYCLES   = CLK_FREQ_HZ / 1000;
    localparam [31:0] WAIT_MAX     = WAIT_TIMEOUT_MS * MS_CYCLES;

    //------------------------------------------------------------------
    // 自检：载荷长度必须是 1 + 6*2 = 13
    //
    //   文档里曾把 13 误写成 17。这类错误不会导致编译或仿真失败，
    //   只会让 ARM 侧解析失败（或读到错位数据），所以在这里断言一次。
    //   arm/uart_proto.py 的 parse_telemetry 也按 13 校验，两边一致。
    //------------------------------------------------------------------
    initial begin
        if (PAYLOAD_LEN != 13) begin
            $display("[telemetry_tx] ERROR: PAYLOAD_LEN 应为 13 (1+6*2), 实际 %0d",
                     PAYLOAD_LEN);
        end
        if (CLK_FREQ_HZ / TX_RATE_HZ <= 0) begin
            $display("[telemetry_tx] ERROR: TX_RATE_HZ=%0d 导致周期为 0", TX_RATE_HZ);
        end
    end

    // 状态
    localparam [2:0] S_IDLE = 3'd0,   // 等发送时间片
                     S_LOAD = 3'd1,   // 逐字节写缓冲
                     S_REQ  = 3'd2,   // 拉 tx_req
                     S_WAIT = 3'd3;   // 等 tx_done

    //==================================================================
    // 1) 24 位 -> 16 位有符号饱和
    //
    //   不能直接截断：+40000 的低 16 位是 -25536，看起来像突然倒车。
    //   饱和至少保证"方向不翻转"。
    //==================================================================
    function signed [15:0] sat16;
        input signed [23:0] v;
        begin
            if      (v >  24'sd32767) sat16 =  16'sd32767;
            else if (v < -24'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    //==================================================================
    // 2) 发送周期计数
    //==================================================================
    reg [31:0] tick_cnt;
    wire       tick = (tick_cnt >= TICK_MAX - 32'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          tick_cnt <= 32'd0;
        else if (tick)       tick_cnt <= 32'd0;
        else                 tick_cnt <= tick_cnt + 32'd1;
    end

    //==================================================================
    // 3) 饱和后的速度值（每 100ms 重算一次即可，用寄存器省组合路径）
    //==================================================================
    reg signed [15:0] spd_l16, spd_r16;

    // ★ 2026-10-18 补复位：这两个寄存器每拍都被覆盖，理论上漏复位也无害，
    //   但"每拍全覆盖"这种事以后改起来很容易被破坏；补上复位只要 32 个触发器，
    //   换来的是 check_always_reset.py 能保持零误报（有例外就得解释，例外一多检查就废）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spd_l16 <= 16'sd0;
            spd_r16 <= 16'sd0;
        end else begin
            spd_l16 <= sat16(spd_l);
            spd_r16 <= sat16(spd_r);
        end
    end

    //==================================================================
    // 3.5) 状态机寄存器 —— 必须**在 §4 之前**声明
    //
    //   为什么提前：§4 的 pl_byte 组合块要用 ld_idx 来选字节。
    //   如果留到 §5 才声明，vlog 会把 §4 里的 ld_idx 当成**隐式线网**，
    //   然后 §5 再声明一次 -> 直接编译报错：
    //       (vlog-2730) Undefined variable: 'ld_idx'
    //       (vlog-2388) 'ld_idx' already declared in this scope
    //   这个错误真的发生了，跑 compile_all.do 时抓出来的。
    //
    //   （同样的坑在 top.v 里也踩过一次：telemetry_tx 用 state，
    //     而 state 在后面的 main_fsm 段才声明 -> 隐式线网。
    //     那次是 tools/check_decl_order.py 抓的。
    //     两处一起说明：**新增模块时，先列清楚谁用谁，再定声明顺序**。）
    //==================================================================
    reg [2:0]  st;
    reg [7:0]  ld_idx;
    reg [31:0] wait_cnt;
    //==================================================================
    // 4) 载荷字节组装（组合）
    //
    //   索引            内容
    //     0             state
    //     1,2           speed_left  (大端，有符号补码)
    //     3,4           speed_right
    //     5,6           dist_mm
    //     7,8           roll
    //     9,10          pitch
    //    11,12          yaw
    //
    //   [!]️ 这里必须用 **ld_idx**（本拍要写的地址），不能用 tx_pl_addr。
    //      tx_pl_addr 是非阻塞赋值，比 ld_idx **滞后一拍**；
    //      用它选字节会导致"每个字节写到前一个地址上"，
    //      结果载荷整体错位 1 字节 —— 而长度仍然正确，
    //      所以现象是"帧能收到、字段全错位"，极难查。
    //      第一版就是写成了 tx_pl_addr，自审时发现的。
    //==================================================================
    reg [7:0] pl_byte;

    always @(*) begin
        case (ld_idx)
            8'd0 : pl_byte = {1'b0, flags, state};   // 低 3 位 state，高 4 位标志（协议向后兼容）
            8'd1 : pl_byte = spd_l16[15:8];
            8'd2 : pl_byte = spd_l16[7:0];
            8'd3 : pl_byte = spd_r16[15:8];
            8'd4 : pl_byte = spd_r16[7:0];
            8'd5 : pl_byte = dist_mm[15:8];
            8'd6 : pl_byte = dist_mm[7:0];
            8'd7 : pl_byte = roll[15:8];
            8'd8 : pl_byte = roll[7:0];
            8'd9 : pl_byte = pitch[15:8];
            8'd10: pl_byte = pitch[7:0];
            8'd11: pl_byte = yaw[15:8];
            8'd12: pl_byte = yaw[7:0];
            default: pl_byte = 8'h00;
        endcase
    end

    //==================================================================
    // 5) 主状态机
    //   （st / ld_idx / wait_cnt 的声明已提前到 §4 之前 —— 见下面的说明）
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            ld_idx       <= 8'd0;
            wait_cnt     <= 32'd0;
            tx_pl_wr     <= 1'b0;
            tx_pl_addr   <= 8'd0;
            tx_pl_data   <= 8'd0;
            tx_req       <= 1'b0;
            tx_type      <= TYPE_TELEMETRY;
            tx_len       <= PAYLOAD_LEN_8;
            sent_cnt     <= 16'd0;
            dropped_cnt  <= 16'd0;
            wait_timeout <= 1'b0;
        end else begin
            // 默认单拍脉冲
            tx_req       <= 1'b0;
            tx_pl_wr     <= 1'b0;
            wait_timeout <= 1'b0;

            case (st)
            //----------------------------------------------------------
            S_IDLE: begin
                wait_cnt <= 32'd0;
                ld_idx   <= 8'd0;
                if (tick) begin
                    if (tx_busy) begin
                        // 上一次还没发完 —— 记一笔，**不静默丢弃**
                        dropped_cnt <= dropped_cnt + 16'd1;
                    end else begin
                        st <= S_LOAD;
                    end
                end
            end

            //----------------------------------------------------------
            // 逐字节写缓冲：一拍一个字节
            //----------------------------------------------------------
            S_LOAD: begin
                tx_pl_addr <= ld_idx;
                tx_pl_data <= pl_byte;
                tx_pl_wr   <= 1'b1;

                if (ld_idx + 8'd1 >= PAYLOAD_LEN_8) begin
                    // 最后一个字节已经在本拍写入，下一拍可以拉 req
                    ld_idx <= 8'd0;
                    st     <= S_REQ;
                end else begin
                    ld_idx <= ld_idx + 8'd1;
                end
            end

            //----------------------------------------------------------
            // 拉一次 tx_req 启动发送
            //
            //   uart_frame 在 tx_req 那一拍锁存 tx_type/tx_len 并开始发送。
            //   缓冲已经在上一个状态写完了。
            //----------------------------------------------------------
            S_REQ: begin
                tx_pl_wr <= 1'b0;
                tx_req   <= 1'b1;
                st       <= S_WAIT;
            end

            //----------------------------------------------------------
            // 等发送完成（带超时）
            //----------------------------------------------------------
            S_WAIT: begin
                wait_cnt <= wait_cnt + 32'd1;
                if (tx_done) begin
                    sent_cnt <= sent_cnt + 16'd1;
                    st       <= S_IDLE;
                end else if (wait_cnt >= WAIT_MAX) begin
                    // 超时兜底：uart_frame 没回 tx_done
                    wait_timeout <= 1'b1;
                    st           <= S_IDLE;
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
