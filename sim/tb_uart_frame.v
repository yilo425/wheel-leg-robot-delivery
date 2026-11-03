//=============================================================================
// tb : tb_uart_frame
//-----------------------------------------------------------------------------
// 验证 rtl/app/uart_frame.v 的帧封装 / 解析 / 超时保护。
//
//-----------------------------------------------------------------------------
// 方法：在 Testbench 里写一个"软件串口"
//-----------------------------------------------------------------------------
//   接收路径不好测 —— 得往 rx_pin 上**按波特率逐位灌**一个帧。
//   所以这里实现两个任务：
//       put_byte(b)  按 UART 格式把 1 个字节打到 rx_pin 上
//       get_byte(b)  从 tx_pin 上按时序收 1 个字节（软件接收器）
//   这样 RX 侧的所有异常（BCC 错 / 超时 / 未知类型 / 前导噪声）
//   都能精确构造出来，而不用靠"碰运气发错"。
//
//-----------------------------------------------------------------------------
// 覆盖场景
//-----------------------------------------------------------------------------
//   1  复位后空闲、无任何误触发
//   2  RX 正常帧（含载荷还原）
//   3  RX 空载荷帧（len=0）
//   4  RX 长度恰好 = MAX_PAYLOAD（边界）
//   5  RX BCC 错 -> rx_bcc_err，且不交付
//   6  RX 未知类型 -> rx_drop，且**帧同步不丢**（下一帧能收到）
//   7  RX 前导噪声 AA AA AA + 正常帧
//   8  RX 帧内超时 -> rx_timeout，且超时后能立刻恢复
//   9  RX 长度超上限 -> rx_drop
//   10 TX 主动发一帧 -> 在 tx_pin 上收到的字节序列必须符合协议
//   11 统计计数 frame_ok_cnt / frame_err_cnt
//
//-----------------------------------------------------------------------------
// 运行: vsim -c -do "vsim -c -quiet -voptargs=+acc -L work -L gw5a \
//                     work.tb_uart_frame; run -all; quit -f"
// 结果: uart_frame_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_uart_frame;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ      = 100_000_000;
    localparam integer CLK_PERIOD       = 10;        // 10ns
    localparam integer BAUD_RATE        = 115_200;
    localparam integer BIT_CLKS         = CLK_FREQ_HZ / BAUD_RATE;   // 868
    localparam integer HALF_BIT         = BIT_CLKS / 2;              // 434

    localparam integer FRAME_TIMEOUT_MS = 1;         // 缩短以便仿真
    localparam integer MS_TICKS         = CLK_FREQ_HZ / 1000;
    localparam integer TMO_TICKS        = FRAME_TIMEOUT_MS * MS_TICKS;

    localparam integer MAX_PAYLOAD      = 64;
    // ⚠️ Verilog-2001 **不允许对 parameter/localparam 做位选**：
    //       MAX_PAYLOAD[7:0]        <-- 非法
    //    必须像下面这样先转成定宽常量再用。
    //    本工程在这个坑上栽过 4 次（CLK_PER_BIT / DIST_STOP_MM / KEY_IDLE /
    //    MAX_PAYLOAD），现在 rtl_lint.py 里加了自动检查。
    localparam [7:0]   MAX_PAYLOAD_8    = MAX_PAYLOAD;
    localparam [7:0]   PROTO_VER        = 8'h01;
    localparam [7:0]   HDR1 = 8'hAA, HDR2 = 8'h55;
    localparam [7:0]   TAIL1 = 8'h55, TAIL2 = 8'hAA;
    localparam integer FRAME_OVERHEAD   = 9;   // 2头+1版本+1类型+2长度+1BCC+2尾

    // 帧类型
    localparam [7:0]   T_VISION   = 8'h02;
    localparam [7:0]   T_TELEMETRY= 8'h10;
    localparam [7:0]   T_ACK      = 8'h03;

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk = 0;
    reg         rst_n = 0;

    reg         rx_pin = 1'b1;         // 由 tb 灌数据
    wire        tx_pin;                // DUT 输出

    reg         tx_pl_wr = 0;
    reg  [7:0]  tx_pl_addr = 0;
    reg  [7:0]  tx_pl_data = 0;
    reg         tx_req = 0;
    reg  [7:0]  tx_type = 0;
    reg  [7:0]  tx_len = 0;

    wire        tx_busy;
    wire        tx_done_dut;

    wire [7:0]  rx_type;
    wire [7:0]  rx_len;
    wire        rx_valid;
    wire        rx_bcc_err;
    wire        rx_timeout;
    wire        rx_drop;
    wire [15:0] frame_ok_cnt;
    wire [15:0] frame_err_cnt;

    // 接收载荷读出
    reg  [7:0]  rx_pl_addr = 0;
    wire [7:0]  rx_pl_data;

    //------------------------------------------------------------------
    // tb 内部：帧缓冲、载荷缓冲
    //------------------------------------------------------------------
    reg [7:0] frame_buf [0:127];       // 待发送的完整帧
    // frame_pl 要能放下 MAX_PAYLOAD + 余量：
    //   测试 9 要构造一个"长度超上限"的帧，需要 80 字节的载荷，
    //   如果只开 MAX_PAYLOAD(64) 深，越界那部分会读到旧值，
    //   算出来的 BCC 就是错的 -> 超长帧会先报 bcc_err、走不到 rx_drop。
    reg [7:0] frame_pl  [0:127];
    reg [7:0] tx_cap    [0:127];       // 从 tx_pin 收到的字节
    integer   frame_len;

    // TX 完成字节数：如果 TX 状态机卡住，这个数会停在 1 —— 一眼可判
    integer   tx_done_cnt = 0;
    always @(posedge clk) if (tx_done_dut) tx_done_cnt = tx_done_cnt + 1;

    // 脉冲锁存（事件统计）
    reg       seen_valid = 0;
    reg       seen_bcc   = 0;
    reg       seen_tmo   = 0;
    reg       seen_drop  = 0;
    integer   valid_cnt  = 0;

    // 最近一帧的性质（用于断言）
    reg [7:0] last_type = 0;
    reg [7:0] last_len  = 0;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer k;

    initial begin
        log_fd = $fopen("uart_frame_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 uart_frame_result.txt");
    end

    always #(CLK_PERIOD/2) clk = ~clk;

    //==================================================================
    // DUT
    //==================================================================
    uart_frame #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .BAUD_RATE        (BAUD_RATE),
        .FRAME_TIMEOUT_MS (FRAME_TIMEOUT_MS),
        .MAX_PAYLOAD      (MAX_PAYLOAD),
        .PROTO_VER        (PROTO_VER),
        .HDR1  (HDR1),  .HDR2  (HDR2),
        .TAIL1 (TAIL1), .TAIL2 (TAIL2)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .rx_pin       (rx_pin),
        .tx_pin       (tx_pin),
        .tx_pl_wr     (tx_pl_wr),
        .tx_pl_addr   (tx_pl_addr),
        .tx_pl_data   (tx_pl_data),
        .tx_req       (tx_req),
        .tx_type      (tx_type),
        .tx_len       (tx_len),
        .tx_busy      (tx_busy),
        .tx_done      (tx_done_dut),
        .rx_type      (rx_type),
        .rx_len       (rx_len),
        .rx_valid     (rx_valid),
        .rx_bcc_err   (rx_bcc_err),
        .rx_timeout   (rx_timeout),
        .rx_drop      (rx_drop),
        .rx_pl_addr   (rx_pl_addr),
        .rx_pl_data   (rx_pl_data),
        .frame_ok_cnt (frame_ok_cnt),
        .frame_err_cnt(frame_err_cnt)
    );

    //==================================================================
    // 脉冲监视：把单拍脉冲变成可查询的标志
    //==================================================================
    always @(posedge clk) begin
        if (rx_valid)    begin seen_valid <= 1'b1; valid_cnt <= valid_cnt + 1;
                               last_type <= rx_type; last_len <= rx_len; end
        if (rx_bcc_err)  seen_bcc  <= 1'b1;
        if (rx_timeout)  seen_tmo  <= 1'b1;
        if (rx_drop)     seen_drop <= 1'b1;
    end

    task clear_seen;
        begin
            seen_valid = 1'b0;  seen_bcc = 1'b0;
            seen_tmo   = 1'b0;  seen_drop = 1'b0;
        end
    endtask

    //==================================================================
    // 软件串口
    //==================================================================
    // 按 UART 格式（1 起始 + 8 数据 + 1 停止）把 1 字节打到 rx_pin
    task put_byte;
        input [7:0] b;
        integer i2;
        begin
            rx_pin = 1'b0;                        // 起始位
            repeat (BIT_CLKS) @(posedge clk);
            for (i2 = 0; i2 < 8; i2 = i2 + 1) begin
                rx_pin = b[i2];                   // LSB first
                repeat (BIT_CLKS) @(posedge clk);
            end
            rx_pin = 1'b1;                        // 停止位
            repeat (BIT_CLKS) @(posedge clk);
        end
    endtask

    // 从 tx_pin 收 1 字节（在每个比特的中点采样）
    //
    // ⚠️ 等起始位要用**电平判断**，不能用 @(negedge tx_pin)：
    //    uart_tx 是在 posedge 的 NBA 区把 tx 拉低的（t+0），
    //    而这里往往在 #1 之后才执行到，边沿已经过去了。
    //    用 @(negedge) 会一直等**下一个**负沿 —— 也就是下一字节的起始位，
    //    结果是丢掉第一个字节。
    //    本工程在 tb_uart 的"起始位宽度测量"上踩过一模一样的坑。
    //
    // 带 idx 参数是为了把"每个字节的起始时刻"打进日志 ——
    //   正常时相邻字节的起始间隔应该 ≈ 10 比特 = 8680 clk = 86.8us；
    //   如果 TX 卡住或间隔异常，从时间戳一眼就能看出来。
    task get_byte;
        input  integer idx;
        output [7:0] b;
        integer i2;
        begin
            while (tx_pin !== 1'b0) @(posedge clk);   // 等起始位（电平，非边沿）
            $fwrite(log_fd, "    [tx] byte%0d 起始位 @ %0t ns\n", idx, $time);
            repeat (HALF_BIT) @(posedge clk);         // 对齐到起始位中点
            for (i2 = 0; i2 < 8; i2 = i2 + 1) begin
                repeat (BIT_CLKS) @(posedge clk);     // 到第 i2 位中点
                b[i2] = tx_pin;
            end
            repeat (BIT_CLKS) @(posedge clk);         // 收到停止位中点
            $fwrite(log_fd, "    [tx] byte%0d = 0x%02X  结束 @ %0t ns\n",
                    idx, b, $time);
        end
    endtask

    //==================================================================
    // 组帧
    //==================================================================
    // BCC = VER ^ TYPE ^ LEN_H ^ LEN_L ^ DATA[0..N-1]（不含帧头帧尾）
    function [7:0] calc_bcc;
        input [7:0] ftype;
        input [7:0] len;
        integer i3;
        reg [7:0] x;
        begin
            x = PROTO_VER ^ ftype ^ 8'h00 ^ len;
            for (i3 = 0; i3 < len; i3 = i3 + 1) x = x ^ frame_pl[i3];
            calc_bcc = x;
        end
    endfunction

    // 把一帧组装进 frame_buf；bcc_bad=1 时故意写错一个字节
    task build_frame;
        input [7:0] ftype;
        input [7:0] len;
        input       bcc_bad;
        integer i3;
        begin
            frame_buf[0] = HDR1;
            frame_buf[1] = HDR2;
            frame_buf[2] = PROTO_VER;
            frame_buf[3] = ftype;
            frame_buf[4] = 8'h00;
            frame_buf[5] = len;
            for (i3 = 0; i3 < len; i3 = i3 + 1)
                frame_buf[6 + i3] = frame_pl[i3];
            frame_buf[6 + len] = bcc_bad ? (calc_bcc(ftype, len) ^ 8'hFF)
                                         : calc_bcc(ftype, len);
            frame_buf[7 + len] = TAIL1;
            frame_buf[8 + len] = TAIL2;
            frame_len = FRAME_OVERHEAD + len;
        end
    endtask

    // 把 frame_buf[0..frame_len-1] 逐字节灌到 rx_pin
    task send_frame_buf;
        integer i3;
        begin
            for (i3 = 0; i3 < frame_len; i3 = i3 + 1) put_byte(frame_buf[i3]);
        end
    endtask

    //==================================================================
    // 断言
    //==================================================================
    task check;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d\n", name, got, exp);
            end else begin
                $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
            end
        end
    endtask

    //==================================================================
    // 复位
    //==================================================================
    task do_reset;
        begin
            rst_n = 1'b0;
            rx_pin = 1'b1;
            tx_req = 1'b0;
            tx_pl_wr = 1'b0;
            repeat (10) @(posedge clk);
            rst_n = 1'b1;
            repeat (10) @(posedge clk);
            clear_seen;
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer i;

    initial begin
        $fwrite(log_fd, "=== tb_uart_frame ===\n");
        $fwrite(log_fd, "  [info] CLK=%0d BAUD=%0d BIT_CLKS=%0d\n",
                CLK_FREQ_HZ, BAUD_RATE, BIT_CLKS);
        $fwrite(log_fd, "  [info] FRAME_TIMEOUT=%0d ms = %0d clk\n",
                FRAME_TIMEOUT_MS, TMO_TICKS);
        $fwrite(log_fd, "  [info] MAX_PAYLOAD=%0d FRAME_OVERHEAD=%0d\n",
                MAX_PAYLOAD, FRAME_OVERHEAD);

        //==================================================================
        $fwrite(log_fd, "\n--- 1. 复位后空闲 ---\n");
        rst_n = 1'b0;
        rx_pin = 1'b1;
        repeat (10) @(posedge clk);
        check("1.1 复位中 tx_pin 应为高(空闲)", tx_pin, 1);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);
        clear_seen;
        check("1.2 复位释放: 无 rx_valid",   seen_valid, 0);
        check("1.3 复位释放: 无 rx_bcc_err", seen_bcc, 0);
        check("1.4 复位释放: 无 rx_timeout", seen_tmo, 0);
        check("1.5 复位释放: tx_busy=0",     tx_busy, 0);
        check("1.6 frame_ok_cnt=0",          frame_ok_cnt, 0);

        //==================================================================
        $fwrite(log_fd, "\n--- 2. RX 正常帧（8 字节载荷）---\n");
        for (i = 0; i < 8; i = i + 1) frame_pl[i] = 8'h10 + i;   // 10..17
        build_frame(T_VISION, 8'd8, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("2.1 收到 rx_valid",     seen_valid, 1);
        check("2.2 rx_type",           last_type, T_VISION);
        check("2.3 rx_len",            last_len, 8'd8);
        // 逐字节读回载荷
        for (i = 0; i < 8; i = i + 1) begin
            rx_pl_addr = i[7:0];
            #1;
            check_name_pl(i, rx_pl_data, 8'h10 + i);
        end
        check("2.4 无 bcc 错",   seen_bcc, 0);
        check("2.5 无 timeout",  seen_tmo, 0);

        //==================================================================
        $fwrite(log_fd, "\n--- 3. RX 空载荷帧 (len=0) ---\n");
        build_frame(T_ACK, 8'd0, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("3.1 空载荷也能收",  seen_valid, 1);
        check("3.2 rx_len=0",      last_len, 0);
        check("3.3 帧长 = 9",      frame_len, FRAME_OVERHEAD);

        //==================================================================
        $fwrite(log_fd, "\n--- 4. RX 载荷恰好 = MAX_PAYLOAD ---\n");
        for (i = 0; i < MAX_PAYLOAD; i = i + 1) frame_pl[i] = i[7:0];
        build_frame(T_TELEMETRY, MAX_PAYLOAD_8, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("4.1 满载荷能收",   seen_valid, 1);
        check("4.2 rx_len=64",    last_len, MAX_PAYLOAD_8);
        check("4.3 帧长 = 73",    frame_len, FRAME_OVERHEAD + MAX_PAYLOAD);

        //==================================================================
        $fwrite(log_fd, "\n--- 5. RX BCC 错 ---\n");
        for (i = 0; i < 4; i = i + 1) frame_pl[i] = 8'hAA;
        build_frame(T_VISION, 8'd4, 1'b1);      // 故意写错 BCC
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("5.1 检测到 rx_bcc_err", seen_bcc, 1);
        check("5.2 不交付 (无 rx_valid)", seen_valid, 0);

        // BCC 错之后紧接着一帧正常的，必须能收到（同步没丢）
        frame_pl[0] = 8'h5A; frame_pl[1] = 8'hA5;
        build_frame(T_VISION, 8'd2, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("5.3 ★BCC 错后仍能收到下一帧", seen_valid, 1);
        check("5.4 下一帧类型正确", last_type, T_VISION);

        //==================================================================
        $fwrite(log_fd, "\n--- 6. RX 未知类型（必须收完但不交付）---\n");
        for (i = 0; i < 3; i = i + 1) frame_pl[i] = 8'h77;
        build_frame(8'h77, 8'd3, 1'b0);         // 未知类型
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("6.1 未知类型被丢弃",  seen_drop, 1);
        check("6.2 未知类型不交付",  seen_valid, 0);

        frame_pl[0] = 8'h3C;
        build_frame(T_ACK, 8'd1, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("6.3 ★未知类型后仍能收到正常帧", seen_valid, 1);
        check("6.4 类型正确", last_type, T_ACK);

        //==================================================================
        $fwrite(log_fd, "\n--- 7. RX 前导噪声 AA AA AA + 正常帧 ---\n");
        rx_pin = 1'b1;
        put_byte(8'hAA); put_byte(8'hAA); put_byte(8'hAA);
        frame_pl[0] = 8'h11;
        build_frame(T_ACK, 8'd1, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("7.1 ★前导噪声后仍能解出帧", seen_valid, 1);
        check("7.2 类型正确", last_type, T_ACK);

        // 纯噪声字节（不是 AA）也不该误触发
        clear_seen;
        put_byte(8'h11); put_byte(8'h22); put_byte(8'h33);
        repeat (50) @(posedge clk);
        check("7.3 纯噪声不误触发", seen_valid, 0);

        //==================================================================
        $fwrite(log_fd, "\n--- 8. RX 帧内超时 ---\n");
        // 只发前 3 个字节（AA 55 01），然后停住
        put_byte(HDR1); put_byte(HDR2); put_byte(PROTO_VER);
        clear_seen;
        // 等超过超时阈值
        repeat (TMO_TICKS + 5000) @(posedge clk);
        check("8.1 检测到 rx_timeout",  seen_tmo, 1);
        check("8.2 超时未交付残帧",      seen_valid, 0);

        // 超时后必须能立刻恢复
        frame_pl[0] = 8'h42;
        build_frame(T_ACK, 8'd1, 1'b0);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("8.3 ★超时后能立刻恢复接收", seen_valid, 1);

        //==================================================================
        $fwrite(log_fd, "\n--- 9. RX 长度超上限 ---\n");
        // 构造 len=80 的帧（> MAX_PAYLOAD=64），但 **BCC 用正确的值** ——
        // 这样它会先通过 BCC 检查，走到帧尾时才因为 r_drop 被丢弃。
        // 如果 BCC 也写错，会在 S_BCC 就报 bcc_err，测不到 rx_drop。
        for (i = 0; i < 80; i = i + 1) frame_pl[i] = i[7:0];
        build_frame(T_VISION, 8'd80, 1'b0);
        check("9.0 构造的帧长 = 89", frame_len, FRAME_OVERHEAD + 80);
        clear_seen;
        send_frame_buf;
        repeat (50) @(posedge clk);
        check("9.1 超长帧被丢弃",  seen_drop, 1);
        check("9.2 超长帧不交付",  seen_valid, 0);
        check("9.3 超长时 BCC 是通过的", seen_bcc, 0);

        //==================================================================
        $fwrite(log_fd, "\n--- 10. TX：主动发一帧，校验引脚上的字节序列 ---\n");
        // 装载载荷
        for (i = 0; i < 4; i = i + 1) begin
            tx_pl_addr = i[7:0];
            tx_pl_data = 8'hA0 + i[7:0];
            tx_pl_wr   = 1'b1;
            @(posedge clk);
        end
        tx_pl_wr = 1'b0;
        repeat (5) @(posedge clk);

        tx_type = T_TELEMETRY;
        tx_len  = 8'd4;
        tx_req  = 1'b1;
        @(posedge clk);
        tx_req  = 1'b0;

        // 收 9 + 4 = 13 个字节
        //   ⚠️ 这里**不能**先等几拍再收：
        //      tx_req 之后 1~2 拍起始位就拉低了，
        //      如果先 repeat(N) @(posedge clk)，等再进 get_byte 时
        //      起始位已经过去，会丢掉第一个字节。
        for (i = 0; i < 13; i = i + 1) get_byte(i, tx_cap[i]);

        check("10.1 TX 帧头1 = 0xAA", tx_cap[0], 8'hAA);
        check("10.2 TX 帧头2 = 0x55", tx_cap[1], 8'h55);
        check("10.3 TX 版本",         tx_cap[2], PROTO_VER);
        check("10.4 TX 类型",         tx_cap[3], T_TELEMETRY);
        check("10.5 TX 长度高字节",   tx_cap[4], 8'h00);
        check("10.6 TX 长度低字节",   tx_cap[5], 8'd4);
        check("10.7 TX 载荷[0]",      tx_cap[6],  8'hA0);
        check("10.8 TX 载荷[3]",      tx_cap[9],  8'hA3);
        check("10.9 TX BCC = 01^10^00^04^A0^A1^A2^A3",
              tx_cap[10], PROTO_VER ^ T_TELEMETRY ^ 8'h00 ^ 8'd4 ^
                          8'hA0 ^ 8'hA1 ^ 8'hA2 ^ 8'hA3);
        check("10.10 TX 帧尾1 = 0x55", tx_cap[11], 8'h55);
        check("10.11 TX 帧尾2 = 0xAA", tx_cap[12], 8'hAA);

        //------------------------------------------------------------------
        // 等 TX 真正回到空闲再检查 tx_busy。
        //
        // ⚠️ get_byte 是在**停止位中点**返回的，此时最后一字节的停止位
        //    才走了一半 —— uart_tx 的 done/busy 要等停止位**结束**
        //    （再 434 拍 = HALF_BIT）才动作。
        //    所以立刻检查 tx_busy 必然还是 1。
        //
        //    这和 tb_uart 里"接收器报 done 比发送器空闲早半个比特"
        //    是同一个现象（那次导致字节严格交替丢失）。
        //    这里带超时保护，避免真的卡住时仿真挂死。
        //------------------------------------------------------------------
        k = 0;
        while (tx_busy && k < 100000) begin
            @(posedge clk);
            k = k + 1;
        end
        $fwrite(log_fd, "  [info] 等 tx_busy 落用了 %0d 拍 (半个比特 = %0d)\n",
                k, HALF_BIT);
        check("10.12 发完后 tx_busy=0", tx_busy, 0);
        $fwrite(log_fd, "  [info] TX 实际完成字节数 = %0d (期望 13)\n", tx_done_cnt);
        $fwrite(log_fd, "  [info] 发完后 tx_busy=%0d  tx_pin=%0d  tx_done_dut=%0d\n",
                tx_busy, tx_pin, tx_done_dut);

        //==================================================================
        $fwrite(log_fd, "\n--- 11. 统计计数 ---\n");
        $fwrite(log_fd, "  [info] frame_ok_cnt = %0d, frame_err_cnt = %0d\n",
                frame_ok_cnt, frame_err_cnt);
        // 前面成功收到：2(正常) 3(空载荷) 4(满载荷) 5.3(一帧) 6.3(一帧)
        //              7.1(一帧) 8.3(一帧) = 7 帧
        check("11.1 frame_ok_cnt = 7", frame_ok_cnt, 7);
        // 出错：5.1(BCC) 6.1(未知类型) 8.1(超时) 9.1(超长) = 4
        check("11.2 frame_err_cnt = 4", frame_err_cnt, 4);
        check("11.3 累计无 errors", errors, 0);

        //==================================================================
        $fwrite(log_fd, "\n=== tb_uart_frame: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    // 载荷逐字节断言的辅助（把序号编进名字里，便于定位是哪一字节错）
    task check_name_pl;
        input integer idx;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $fwrite(log_fd, "  [FAIL] 2.3.%0d 载荷[%0d] : got=%0d exp=%0d\n",
                        idx, idx, got, exp);
            end else begin
                $fwrite(log_fd, "  [ OK ] 2.3.%0d 载荷[%0d] : %0d\n",
                        idx, idx, got);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 超时保护（不要写成超过 32 位的字面量）
    //------------------------------------------------------------------
    initial begin
        #200_000_000;                    // 200ms
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
