//=============================================================================
// tb : tb_uart
//-----------------------------------------------------------------------------
// 拓扑: uart_tx 的 tx 直接接 uart_rx 的 rx, 做回环自检。
//
// 自检内容:
//   1) 多组数据(含 0x00/0xFF 边界)回环后完全一致
//   2) 每个字节产生且仅产生 1 个 done 脉冲, 无 frame_err / parity_err
//   3) 空闲时 tx 恒为高, 无多余起始位
//   4) 比特宽度 = CLK_FREQ_HZ / BAUD_RATE (容差 3 clk)
//
// 运行: vlog -work work ../rtl/driver/uart_tx.v ../rtl/driver/uart_rx.v tb_uart.v
//       vsim -c -do "run -all; quit -f" work.tb_uart
//=============================================================================
`timescale 1ns/1ps

module tb_uart;

    // 时钟频率的选择很关键，这里连续踩了两次坑：
    //
    // 坑1: 最初用 1MHz —— uart_rx 要 16 倍过采样，需要
    //      CLK_FREQ_HZ/(BAUD_RATE*16) >= 2，1MHz 根本不够。
    //
    // 坑2: 改用 20MHz 后还是失败，因为**整数截断误差太大**：
    //      OS_DIV = CLK/(BAUD*16)，理想值 10.85，截断成 10，误差 8%。
    //      每比特偏短 8%，10 个比特累积偏差 > 80% 比特宽，
    //      中点采样漂到隔壁比特 -> 起始位复核失败 -> 接收器永远收不到。
    //      现象是：tx_busy=0(发完了) 但 rx_done=0, rx_busy=0(一直退回 IDLE)。
    //
    //      对比各频率的截断误差：
    //          20MHz  -> OS_DIV=10 (理想 10.85)  误差 8%    ✗
    //          50MHz  -> OS_DIV=27 (理想 27.13)  误差 0.5%  ✓
    //         100MHz  -> OS_DIV=54 (理想 54.25)  误差 0.46% ✓
    //
    // 结论: 直接用 100MHz —— 与 project_config.vh 里 PLL 之后的系统时钟一致，
    //       既避开误差，又更接近真实工作条件。
    localparam integer CLK_FREQ_HZ = 100_000_000;
    localparam integer BAUD_RATE   = 115_200;
    localparam integer CLK_PERIOD  = 10;       // 100MHz -> 10ns
    localparam integer BIT_CLKS    = CLK_FREQ_HZ / BAUD_RATE;   // 868
    // 16 倍过采样下每个 tick 占几个 clk
    localparam integer OS_DIV      = CLK_FREQ_HZ / (BAUD_RATE * 16);   // = 54

    reg  clk = 0;
    reg  rst_n = 0;
    reg  tx_start = 0;
    reg  [7:0] tx_data = 8'h00;
    wire tx_line;
    wire tx_busy, tx_done;
    wire [7:0] rx_data;
    wire rx_done, rx_frame_err, rx_parity_err, rx_busy;

    integer errors = 0;
    integer checks = 0;


    // ---- 结果同时写文件 ----
    //   ModelSim -c 模式下 $display 走 stdout，管道常抓不到，
    //   用 $fwrite 直接落盘最可靠。结果文件: uart_result.txt
    integer log_fd;
    initial begin
        log_fd = $fopen("uart_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 uart_result.txt");
    end
    always #(CLK_PERIOD/2) clk = ~clk;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .DATA_BITS   (8),
        .PARITY_EN   (0)
    ) u_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (tx_start),
        .data_in (tx_data),
        .tx      (tx_line),
        .busy    (tx_busy),
        .done    (tx_done)
    );

    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE),
        .DATA_BITS   (8),
        .PARITY_EN   (0)
    ) u_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (tx_line),      // 回环
        .data_out   (rx_data),
        .done       (rx_done),
        .frame_err  (rx_frame_err),
        .parity_err (rx_parity_err),
        .rx_busy    (rx_busy)
    );

    //------------------------------------------------------------------
    // 接收监视
    //------------------------------------------------------------------
    integer   done_cnt = 0;
    integer   ferr_cnt = 0;
    integer   perr_cnt = 0;
    reg [7:0] last_rx  = 8'h00;
    event     rx_ev;

    always @(posedge clk) begin
        if (rst_n) begin
            if (rx_done) begin
                done_cnt = done_cnt + 1;
                last_rx  = rx_data;
                -> rx_ev;
            end
            if (rx_frame_err)  ferr_cnt = ferr_cnt + 1;
            if (rx_parity_err) perr_cnt = perr_cnt + 1;
        end
    end

    task check;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  [FAIL] %0s : got=%0d exp=%0d", name, got, exp);
                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d\n", name, got, exp);
            end else begin
                $display("  [ OK ] %0s : %0d", name, got);
                $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 发一个字节并等回环完成
    //------------------------------------------------------------------
    integer d0, guard;

    // 一个字节的等待上限（clk 数）
    //
    // 这个值我连续算错三轮，原因各不相同，最后一次是**整数溢出**：
    //   旧写法 BYTE_NS = (10 * 1000000000) / BAUD_RATE
    //   10 * 1e9 = 1e10 已经超过 32 位整数上限(2.1e9) -> 回绕
    //   实测 BYTE_CLKS 算出来是 13 而不是 87，guard 只有 390 clk，
    //   而发送器还在忙(tx_busy=1) -> 每个字节都超时。
    //
    //   现在改用不会溢出的写法：直接把"每比特 clk 数"乘 10 位再留余量。
    localparam integer BYTE_CLKS  = BIT_CLKS * 10;    // 每比特 clk 数 * 10 位
    localparam integer GUARD_CLKS = BYTE_CLKS * 20;   // 20 倍余量

    task send_byte;
        input [7:0] b;
        begin
            //------------------------------------------------------------------
            // 关键：先等发送器真正空闲（tx_busy=0）再发
            //
            // 踩过的坑（症状很有迷惑性）：
            //   接收器在"停止位中点"就判定完成（约 9.5 比特时间），
            //   而发送器要等"停止位结束"（10 比特时间）才回到 IDLE。
            //   中间有半个比特时间（100MHz 下 434 拍）的窗口。
            //   如果收到 done 后马上发下一个字节，那时 tx 还在 STOP 状态，
            //   而 uart_tx 只在 IDLE 响应 start —— 这个 start 会被**静默丢弃**。
            //
            //   现象：字节严格交替地成功/失败（1,3,5,7 收到，2,4,6 丢失），
            //         因为被丢的那个字节让 tx 空转了一帧时间，下一个又赶得上。
            //
            //   这是 RTL 的正确行为（忙时不接受新请求），
            //   驱动方必须自己查 busy —— 即真实的流控。
            //------------------------------------------------------------------
            while (tx_busy) @(posedge clk);

            @(posedge clk);
            #1;
            d0 = done_cnt;

            tx_data  = b;
            tx_start = 1'b1;
            @(posedge clk);
            #1;
            tx_start = 1'b0;

            guard = 0;
            while (done_cnt == d0 && guard < GUARD_CLKS) begin
                @(posedge clk);
                guard = guard + 1;
            end

            if (done_cnt == d0) begin
                errors = errors + 1;
                // 超时时把内部状态和实际耗时都打出来，否则只看到"超时"没法定位
                $display("  [FAIL] send_byte 0x%02h 超时", b);
                $display("         等待 %0d clk (= %0t), 上限 GUARD_CLKS=%0d, 一字节理论 BYTE_CLKS=%0d",
                         guard, $time, GUARD_CLKS, BYTE_CLKS);
                $display("         rx_done=%0d frame_err=%0d parity_err=%0d rx_busy=%0d",
                         rx_done, rx_frame_err, rx_parity_err, rx_busy);
                $display("         tx_busy=%0d tx_line=%0d rx_data=0x%02h last_rx=0x%02h",
                         tx_busy, tx_line, rx_data, last_rx);
                $fwrite(log_fd, "  [FAIL] send_byte 0x%02h 超时\n", b);
                $fwrite(log_fd, "         等待 %0d clk, 上限 GUARD_CLKS=%0d, 一字节理论 BYTE_CLKS=%0d\n",
                        guard, GUARD_CLKS, BYTE_CLKS);
                $fwrite(log_fd, "         rx_done=%0d frame_err=%0d parity_err=%0d rx_busy=%0d\n",
                        rx_done, rx_frame_err, rx_parity_err, rx_busy);
                $fwrite(log_fd, "         tx_busy=%0d tx_line=%0d rx_data=0x%02h last_rx=0x%02h\n",
                        tx_busy, tx_line, rx_data, last_rx);
            end else begin
                check("loopback data", last_rx, b);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer high_idle;
    integer i;

    initial begin
        $display("=== tb_uart ===");

        rst_n = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (50) @(posedge clk);      // 等所有寄存器稳定

        // --- 1) 空闲: tx 恒为高, 无 done, 不 busy ---
        high_idle = 0;
        for (i = 0; i < 500; i = i + 1) begin
            @(posedge clk);
            #1;
            if (tx_line === 1'b1) high_idle = high_idle + 1;
        end
        check("idle: tx high count", high_idle, 500);
        check("idle: done_cnt",      done_cnt, 0);
        check("idle: rx_busy",       rx_busy,  0);
        check("idle: tx_busy",       tx_busy,  0);

        // --- 2) 回环多组数据 ---
        send_byte(8'h55);
        send_byte(8'hA3);
        send_byte(8'h00);
        send_byte(8'hFF);
        send_byte(8'h01);
        send_byte(8'h80);
        send_byte(8'h7E);

        check("total done_cnt",  done_cnt, 7);
        check("frame_err cnt",   ferr_cnt, 0);
        check("parity_err cnt",  perr_cnt, 0);

        // --- 3) 比特宽度: 起始位下降沿起到线回到高的 clk 数 ---
        //     data=0x0F: bit0..3=1, 所以起始位之后低电平恰好 1 个比特
        //
        //   同样必须先等发送器空闲！否则 start 被忽略，
        //   后面 @(negedge tx_line) 会永远等不到下降沿，一直挂到超时。
        //   （这次实测就是这样：7 个字节全 OK，但最后卡在 RESULT: FAIL (timeout)。）
        @(posedge clk);
        #1;
        while (tx_busy) @(posedge clk);      // ← 关键：等 TX 空闲再发
        tx_data  = 8'h0F;
        tx_start = 1'b1;
        @(posedge clk);
        #1;
        tx_start = 1'b0;

        // 等 tx 真正拉低 —— 必须用【电平判断】，不能等边沿！
        //
        // 踩过的坑（症状：d0 = 3473 而不是 868）：
        //   tx_line 是在 posedge clk 的 NBA 区拉低的（t+0），
        //   而下面的语句在 #1 之后才执行（t+1ns）——边沿早就过去了。
        //   写 @(negedge tx_line) 会一直等【下一个】负沿。
        //   对 0x0F 来说，LSB 序列是 1,1,1,1,0,0,0,0，
        //   下一个负沿就是 bit4 的下降沿，于是数到了 4 个低比特
        //   （4 × 868 = 3472，实测 3473，完全吻合）。
        while (tx_line !== 1'b0) @(posedge clk);   // 等低电平出现

        d0 = 0;
        while (tx_line === 1'b0) begin
            @(posedge clk);
            d0 = d0 + 1;
        end
        // 把实测值写进结果文件（原来只 $display 到 stdout，值看不到，很难查）
        $display("  [info] start bit width = %0d clk (expect ~%0d)", d0, BIT_CLKS);
        $fwrite(log_fd, "  [info] start bit width = %0d clk (BIT_CLKS=%0d)\n",
                d0, BIT_CLKS);
        // 容差 ±5：d0 的精确值取决于"边沿 vs 时钟沿"的对齐，
        //          给 ±5 足够覆盖，同时仍能抓出真正的错误
        //          （比如误数成 4 个比特就是 3472，会被抓出来）。
        check("start bit width",
              (d0 > BIT_CLKS - 5 && d0 < BIT_CLKS + 5) ? 1 : 0, 1);

        // 等这一帧收完（同样要按真实比特时间给足余量）
        guard = 0;
        while (done_cnt < 8 && guard < BYTE_CLKS * 10) begin
            @(posedge clk);
            guard = guard + 1;
        end
        check("final done_cnt",  done_cnt, 8);
        check("final frame_err", ferr_cnt, 0);

        //------------------------------------------------------------------
        $display("=== tb_uart: %0d checks, %0d errors ===", checks, errors);
        $fwrite(log_fd, "=== tb_uart: %0d checks, %0d errors ===\n", checks, errors);
        if (errors == 0) begin
            $display("RESULT: PASS");
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $display("RESULT: FAIL");
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $stop;
    end

    initial begin
        #50000000;
        $display("RESULT: FAIL (timeout)");
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
