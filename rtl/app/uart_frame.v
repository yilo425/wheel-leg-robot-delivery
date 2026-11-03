//=============================================================================
// uart_frame -- 帧封装 / 解析 / 超时保护
//-----------------------------------------------------------------------------
// 在链路里的位置：
//
//   ARM ──串口──► uart_rx ──► uart_frame ──► main_fsm（视觉类别/目标位置）
//        ◄──────  uart_tx ◄── uart_frame ◄── 遥测字段（状态/速度/距离/姿态）
//
// 本模块**不含波特率逻辑**（那是 uart_tx/uart_rx 的事），
// 只负责"字节流 ↔ 有结构的帧"这一层。
//
//-----------------------------------------------------------------------------
// 帧格式（与 doc/uart_frame_protocol.md 一致）
//-----------------------------------------------------------------------------
//   +------+------+-----+------+-------+-------+----------+-----+------+------+
//   | 0xAA | 0x55 | VER | TYPE | LEN_H | LEN_L | DATA...  | BCC | 0x55 | 0xAA |
//   +------+------+-----+------+-------+-------+----------+-----+------+------+
//   BCC = XOR(VER, TYPE, LEN_H, LEN_L, DATA[0..N-1])      不含帧头帧尾
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) 【超时保护】任何中间状态超过 FRAME_TIMEOUT 未收到下一字节 -> 回 S_HDR1
//    没有这个保护，丢一个字节就会永久卡在"等长度/等校验"上，
//    现象是"通信莫名其妙死了"，而且不报任何错。这是串口协议的经典陷阱。
//
// 2) 【未知类型不丢同步】收到不认识的 TYPE，标记丢弃，但**仍然按长度收完**，
//    这样帧边界不会错乱。很多实现直接跳回 S_HDR1，结果后面全乱。
//
// 3) 【长度上限】LEN 超过 MAX_PAYLOAD 直接判非法帧，防止写爆缓冲区。
//
// 4) 【收发缓冲分离】tx_buf / rx_buf 各一套。
//    收发同时进行时互不干扰（串口是全双工的）。
//
// 5) 【BCC 而不是 CRC16】FPGA 侧只需一个异或，时序压力为零；
//    UART 自带起始/停止位同步，短帧出错率很低。
//    若后期误帧率高，升级 CRC16-CCITT 并同步升 PROTO_VER。
//=============================================================================
`timescale 1ns/1ps

module uart_frame #(
    parameter integer CLK_FREQ_HZ      = 50_000_000,
    parameter integer BAUD_RATE        = 115_200,
    parameter integer FRAME_TIMEOUT_MS = 20,      // 帧内字节间隔超时
    parameter integer MAX_PAYLOAD      = 64,      // 载荷上限
    parameter [7:0]   PROTO_VER        = 8'h01,

    // 帧定界符（参数化便于换协议）
    parameter [7:0]   HDR1  = 8'hAA,
    parameter [7:0]   HDR2  = 8'h55,
    parameter [7:0]   TAIL1 = 8'h55,
    parameter [7:0]   TAIL2 = 8'hAA
) (
    input  wire        clk,
    input  wire        rst_n,

    //---------------- 物理层（接到 uart_tx / uart_rx）----------------
    input  wire        rx_pin,
    output wire        tx_pin,

    //---------------- 发送：载荷缓冲写入 ----------------
    input  wire        tx_pl_wr,          // 写一个载荷字节
    input  wire [7:0]  tx_pl_addr,        // 0..MAX_PAYLOAD-1
    input  wire [7:0]  tx_pl_data,

    //---------------- 发送：触发 ----------------
    input  wire        tx_req,            // 单拍脉冲
    input  wire [7:0]  tx_type,
    input  wire [7:0]  tx_len,            // 载荷长度
    output reg         tx_busy,
    output reg         tx_done,           // 单拍：一帧发完

    //---------------- 接收：帧结果 ----------------
    output reg  [7:0]  rx_type,
    output reg  [7:0]  rx_len,
    output reg         rx_valid,          // 单拍：收到合法帧
    output reg         rx_bcc_err,        // 单拍：BCC 错
    output reg         rx_timeout,        // 单拍：帧内超时
    output reg         rx_drop,           // 单拍：帧被丢弃（未知类型/超长）

    //---------------- 接收：载荷读出 ----------------
    input  wire [7:0]  rx_pl_addr,
    output wire [7:0]  rx_pl_data,

    //---------------- 统计（调试/上报）----------------
    output reg  [15:0] frame_ok_cnt,
    output reg  [15:0] frame_err_cnt
);

    //==================================================================
    // 常量与推导
    //==================================================================
    localparam integer MS_TICKS     = CLK_FREQ_HZ / 1000;
    localparam integer TMO_TICKS_32 = FRAME_TIMEOUT_MS * MS_TICKS;
    localparam [31:0]  TMO_TICKS    = TMO_TICKS_32;
    localparam [7:0]   MAX_PAYLOAD_8 = MAX_PAYLOAD;

    // 发送状态
    localparam [3:0] T_IDLE  = 4'd0,
                     T_HDR1  = 4'd1,
                     T_HDR2  = 4'd2,
                     T_VER   = 4'd3,
                     T_TYPE  = 4'd4,
                     T_LENH  = 4'd5,
                     T_LENL  = 4'd6,
                     T_DATA  = 4'd7,
                     T_BCC   = 4'd8,
                     T_TAIL1 = 4'd9,
                     T_TAIL2 = 4'd10;

    // 接收状态
    localparam [3:0] S_HDR1  = 4'd0,
                     S_HDR2  = 4'd1,
                     S_VER   = 4'd2,
                     S_TYPE  = 4'd3,
                     S_LENH  = 4'd4,
                     S_LENL  = 4'd5,
                     S_DATA  = 4'd6,
                     S_BCC   = 4'd7,
                     S_TAIL1 = 4'd8,
                     S_TAIL2 = 4'd9;

    //==================================================================
    // 两端字节缓冲
    //   用 reg 数组而不是 RAM 原语：容量小（64B），
    //   综合器会自动推断成分布式 RAM 或寄存器，行为最好预测。
    //==================================================================
    reg [7:0] tx_buf [0:MAX_PAYLOAD-1];
    reg [7:0] rx_buf [0:MAX_PAYLOAD-1];

    // 发送缓冲写入（存储器写口）
    //
    //   为什么不复位：这是对一个 reg 数组的写入端口。给整块 RAM 加复位要生成
    //   一个巨大的 mux（MAX_PAYLOAD 个字节全要清零），代价远大于收益：
    //   读之前上层一定会先把载荷写满（telemetry_tx 是"写完才拉 tx_req"），
    //   所以不存在"读到未初始化字节"的路径。接收缓冲同理，由收帧逻辑写满。
    //
    // check_always_reset: waive -- 存储器写口，不需要复位（理由见上）
    always @(posedge clk) begin
        if (tx_pl_wr && tx_pl_addr < MAX_PAYLOAD_8)
            tx_buf[tx_pl_addr] <= tx_pl_data;
    end

    // 接收缓冲读出（组合）
    assign rx_pl_data = rx_buf[rx_pl_addr];

    //==================================================================
    // 物理层：uart_tx / uart_rx
    //==================================================================
    wire       tx_start;
    wire [7:0] tx_byte;
    wire       tx_busy_phy;
    wire       tx_done_phy;

    wire [7:0] rx_byte;
    wire       rx_done_phy;
    wire       rx_frame_err;
    wire       rx_parity_err;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .DATA_BITS   (8),
        .PARITY_EN   (0)
    ) u_uart_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (tx_start),
        .data_in (tx_byte),
        .tx      (tx_pin),
        .busy    (tx_busy_phy),
        .done    (tx_done_phy)
    );

    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .DATA_BITS   (8),
        .PARITY_EN   (0)
    ) u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (rx_pin),
        .data_out   (rx_byte),
        .done       (rx_done_phy),
        .frame_err  (rx_frame_err),
        .parity_err (rx_parity_err),
        .rx_busy    ()
    );

    //==================================================================
    // 发送状态机
    //
    //   每个状态：给出本字节 -> 拉高 tx_start 一拍 -> 等 tx_done_phy
    //   注意：uart_tx 忙时会**静默忽略** start，所以必须先等 !tx_busy_phy
    //==================================================================
    reg [3:0]  t_state;
    reg [7:0]  t_idx;
    reg [7:0]  t_bcc;
    reg [7:0]  t_type_r;
    reg [7:0]  t_len_r;
    reg        t_start_r;      // 送给 uart_tx.start
    reg [7:0]  t_byte_r;       // 送给 uart_tx.data_in

    assign tx_start = t_start_r;
    assign tx_byte  = t_byte_r;

    // 当前要发的字节（组合，按状态选）
    reg [7:0] t_cur;
    always @(*) begin
        case (t_state)
            T_HDR1 : t_cur = HDR1;
            T_HDR2 : t_cur = HDR2;
            T_VER  : t_cur = PROTO_VER;
            T_TYPE : t_cur = t_type_r;
            T_LENH : t_cur = 8'h00;              // 载荷 <=255，高字节恒 0
            T_LENL : t_cur = t_len_r;
            T_DATA : t_cur = tx_buf[t_idx];
            T_BCC  : t_cur = t_bcc;
            T_TAIL1: t_cur = TAIL1;
            T_TAIL2: t_cur = TAIL2;
            default: t_cur = 8'h00;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state   <= T_IDLE;
            t_idx     <= 8'd0;
            t_bcc     <= 8'd0;
            t_type_r  <= 8'd0;
            t_len_r   <= 8'd0;
            t_start_r <= 1'b0;
            t_byte_r  <= 8'd0;
            tx_busy   <= 1'b0;
            tx_done   <= 1'b0;
        end else begin
            // 默认：start 只拉一拍
            t_start_r <= 1'b0;
            tx_done   <= 1'b0;

            case (t_state)
                //------------------------------------------------------
                T_IDLE: begin
                    tx_busy <= 1'b0;
                    if (tx_req) begin
                        t_type_r <= tx_type;
                        t_len_r  <= (tx_len > MAX_PAYLOAD_8) ? MAX_PAYLOAD_8
                                                             : tx_len;
                        t_bcc    <= 8'd0;
                        t_idx    <= 8'd0;
                        tx_busy  <= 1'b1;
                        t_state  <= T_HDR1;
                    end
                end

                //------------------------------------------------------
                // 通用"发一个字节"流程
                //
                // ⚠️ 两个分支的**顺序至关重要**，写反了会出现优先级遮蔽：
                //
                //     uart_tx 在同一个时钟沿同时做两件事：
                //         busy <= 0;   done <= 1;      (STOP 的 bit_end)
                //     于是**下一拍** tx_busy_phy 已经是 0，
                //     如果"可以发下一字节"（!t_start_r && !tx_busy_phy）
                //     排在前面，它就会把 done 分支**遮住** ——
                //     结果是：状态永远不推进，一直重复发同一个字节。
                //
                //     实测症状很有迷惑性：
                //       * tx 上的字节间隔**完全正常**（10 个比特 = 86.8us）
                //       * tx_busy 一直是 1
                //       * 收到的每个字节都等于当前状态要发的那个（例如一直 0xAA）
                //     因为帧头第一个字节就是 0xAA，看起来像"能发但发错"。
                //
                //     正确顺序：先判"上一字节完成"，再判"可以发下一字节"。
                //------------------------------------------------------
                default: begin
                    if (tx_done_phy) begin
                        // 累加 BCC：VER/TYPE/LENH/LENL/DATA 都算，帧头帧尾不算
                        case (t_state)
                            T_VER, T_TYPE, T_LENH, T_LENL, T_DATA:
                                t_bcc <= t_bcc ^ t_cur;
                            default: ;
                        endcase

                        case (t_state)
                            T_HDR1 : t_state <= T_HDR2;
                            T_HDR2 : t_state <= T_VER;
                            T_VER  : t_state <= T_TYPE;
                            T_TYPE : t_state <= T_LENH;
                            T_LENH : t_state <= T_LENL;
                            T_LENL : t_state <= (t_len_r == 8'd0) ? T_BCC : T_DATA;
                            T_DATA : begin
                                if (t_idx >= t_len_r - 8'd1) t_state <= T_BCC;
                                else begin
                                    t_idx   <= t_idx + 8'd1;
                                    t_state <= T_DATA;
                                end
                            end
                            T_BCC   : t_state <= T_TAIL1;
                            T_TAIL1 : t_state <= T_TAIL2;
                            T_TAIL2 : begin
                                t_state <= T_IDLE;
                                tx_done <= 1'b1;
                                tx_busy <= 1'b0;
                            end
                            default : t_state <= T_IDLE;
                        endcase
                    end else if (!t_start_r && !tx_busy_phy) begin
                        // 上一字节已经完成（或者还没开始）—— 现在才轮到发下一字节。
                        // 这个分支必须放在 done 之后，否则会把状态推进遮住（见上方说明）。
                        t_byte_r  <= t_cur;
                        t_start_r <= 1'b1;
                    end
                end
            endcase
        end
    end

    //==================================================================
    // 接收状态机
    //
    //   超时保护：用 tmo_cnt 计时，只在"非 S_HDR1"状态下累加；
    //   收到任何字节就清零。超过阈值直接回 S_HDR1。
    //==================================================================
    reg [3:0]  r_state;
    reg [7:0]  r_idx;
    reg [7:0]  r_bcc;
    reg [7:0]  r_len;
    reg [7:0]  r_type;
    reg        r_drop;             // 本帧要丢弃（未知类型/超长）
    reg [31:0] tmo_cnt;

    // 已知帧类型（未知类型仍然收完，只是不交给上层）
    function is_known_type;
        input [7:0] t;
        begin
            // ★ 2026-10-29：加上 0x05 RC_CMD（手动遥控）。
            //   漏了这一个数字的后果：遥控帧被当成"未知类型"整帧丢弃，
            //   rx_valid 从不产生 -> rc_rx 永远收不到 -> rc_active 恒 0，
            //   现象就是"CONFIG(0x04) 能收、遥控(0x05) 完全不生效"。
            is_known_type = (t == 8'h01) || (t == 8'h02) ||
                            (t == 8'h03) || (t == 8'h04) ||
                            (t == 8'h05) ||
                            (t == 8'h10);
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state       <= S_HDR1;
            r_idx         <= 8'd0;
            r_bcc         <= 8'd0;
            r_len         <= 8'd0;
            r_type        <= 8'd0;
            r_drop        <= 1'b0;
            tmo_cnt       <= 32'd0;
            rx_type       <= 8'd0;
            rx_len        <= 8'd0;
            rx_valid      <= 1'b0;
            rx_bcc_err    <= 1'b0;
            rx_timeout    <= 1'b0;
            rx_drop       <= 1'b0;
            frame_ok_cnt  <= 16'd0;
            frame_err_cnt <= 16'd0;
        end else begin
            // 默认单拍脉冲
            rx_valid   <= 1'b0;
            rx_bcc_err <= 1'b0;
            rx_timeout <= 1'b0;
            rx_drop    <= 1'b0;

            //----------------------------------------------------------
            // 超时计数：只在帧中途累加
            //----------------------------------------------------------
            if (r_state == S_HDR1)             tmo_cnt <= 32'd0;
            else if (rx_done_phy)              tmo_cnt <= 32'd0;
            else if (tmo_cnt >= TMO_TICKS)     tmo_cnt <= tmo_cnt;
            else                               tmo_cnt <= tmo_cnt + 32'd1;

            if ((r_state != S_HDR1) && (tmo_cnt >= TMO_TICKS)) begin
                // 帧内超时 -> 回帧头搜索，并报错
                r_state       <= S_HDR1;
                tmo_cnt       <= 32'd0;
                rx_timeout    <= 1'b1;
                frame_err_cnt <= frame_err_cnt + 16'd1;
            end
            else if (rx_done_phy) begin
                //------------------------------------------------------
                // 每收到一个字节就推进状态机
                //------------------------------------------------------
                case (r_state)
                    //--------------------------------------------------
                    S_HDR1: begin
                        if (rx_byte == HDR1) r_state <= S_HDR2;
                    end

                    //--------------------------------------------------
                    // 这里有个细节：若收到的又是 HDR1，要**停在本状态**，
                    // 这样 "AA AA 55 ..." 这种（前导噪声）也能正确同步。
                    //--------------------------------------------------
                    S_HDR2: begin
                        if (rx_byte == HDR2) begin
                            r_state <= S_VER;
                            r_bcc   <= 8'd0;
                        end else if (rx_byte == HDR1) begin
                            r_state <= S_HDR2;      // 停在等 HDR2
                        end else begin
                            r_state <= S_HDR1;
                        end
                    end

                    S_VER: begin
                        if (rx_byte == PROTO_VER) begin
                            r_bcc   <= r_bcc ^ rx_byte;
                            r_state <= S_TYPE;
                        end else begin
                            // 版本不匹配：丢弃整帧
                            r_state       <= S_HDR1;
                            rx_drop       <= 1'b1;
                            frame_err_cnt <= frame_err_cnt + 16'd1;
                        end
                    end

                    S_TYPE: begin
                        r_type  <= rx_byte;
                        r_bcc   <= r_bcc ^ rx_byte;
                        r_drop  <= ~is_known_type(rx_byte);
                        r_state <= S_LENH;
                    end

                    S_LENH: begin
                        r_bcc   <= r_bcc ^ rx_byte;
                        // 高字节非 0 -> 长度过大，不可能合法
                        r_drop  <= r_drop | (rx_byte != 8'h00);
                        r_state <= S_LENL;
                    end

                    S_LENL: begin
                        r_bcc   <= r_bcc ^ rx_byte;
                        r_len   <= rx_byte;
                        r_idx   <= 8'd0;
                        if (rx_byte > MAX_PAYLOAD_8) begin
                            // 超长：仍然要收完，保持帧同步
                            r_drop  <= 1'b1;
                            r_state <= (rx_byte == 8'd0)
                                       ? S_BCC : S_DATA;
                        end else begin
                            r_state <= (rx_byte == 8'd0) ? S_BCC : S_DATA;
                        end
                    end

                    S_DATA: begin
                        r_bcc <= r_bcc ^ rx_byte;
                        if (r_idx < MAX_PAYLOAD_8) rx_buf[r_idx] <= rx_byte;
                        if (r_idx >= r_len - 8'd1) r_state <= S_BCC;
                        else begin
                            r_idx   <= r_idx + 8'd1;
                            r_state <= S_DATA;
                        end
                    end

                    S_BCC: begin
                        if (rx_byte == r_bcc) r_state <= S_TAIL1;
                        else begin
                            r_state       <= S_HDR1;
                            rx_bcc_err    <= 1'b1;
                            frame_err_cnt <= frame_err_cnt + 16'd1;
                        end
                    end

                    S_TAIL1: begin
                        if (rx_byte == TAIL1) r_state <= S_TAIL2;
                        else begin
                            r_state       <= S_HDR1;
                            rx_drop       <= 1'b1;
                            frame_err_cnt <= frame_err_cnt + 16'd1;
                        end
                    end

                    //--------------------------------------------------
                    // 帧尾正确 -> 交付上层（除非本帧标记丢弃）
                    //--------------------------------------------------
                    S_TAIL2: begin
                        r_state <= S_HDR1;
                        if (rx_byte == TAIL2) begin
                            if (r_drop) begin
                                rx_drop       <= 1'b1;
                                frame_err_cnt <= frame_err_cnt + 16'd1;
                            end else begin
                                rx_type      <= r_type;
                                rx_len       <= r_len;
                                rx_valid     <= 1'b1;
                                frame_ok_cnt <= frame_ok_cnt + 16'd1;
                            end
                        end else begin
                            rx_drop       <= 1'b1;
                            frame_err_cnt <= frame_err_cnt + 16'd1;
                        end
                    end

                    default: r_state <= S_HDR1;
                endcase
            end
        end
    end

    //==================================================================
    // 参数自检
    //==================================================================
    initial begin
        if (MS_TICKS < 1) begin
            $display("[uart_frame] FATAL: CLK_FREQ_HZ 太低");
        end
        if ((CLK_FREQ_HZ * 1.0 * FRAME_TIMEOUT_MS / 1000.0) >= 2147483647.0) begin
            $display("[uart_frame] WARN: 帧超时换算后超出 32 位计数器范围");
        end
        if (MAX_PAYLOAD < 1 || MAX_PAYLOAD > 255) begin
            $display("[uart_frame] WARN: MAX_PAYLOAD=%0d 超出常用范围 1..255",
                     MAX_PAYLOAD);
        end
        if (HDR1 == HDR2 || TAIL1 == TAIL2 || HDR1 == TAIL1) begin
            $display("[uart_frame] WARN: 定界符有重复，可能影响帧同步");
        end
    end

endmodule
