//=============================================================================
// tb_telemetry_payload —— 验 telemetry 的 **13 字节载荷内容**（不只是"有没有发"）
//
// 为什么需要这个 tb：
//   `tb_telemetry_check` 只验证 `uart_tx` 有没有在动（发送通路活着）。
//   但 ARM 侧真正依赖的是**载荷里的字段**：状态、左右速度、距离、三个姿态角。
//   字节顺序/大端/饱和任何一处错了，现象都是"帧能收到、数值全是乱的"，
//   而且 FPGA 侧所有既有 tb 都不会红。
//
//   2026-10-18 排查电机时也体会到了：**能收到 ≠ 收对了**。
//
// 这个 tb 直接例化 telemetry_tx，把 tx_pl_wr 写进缓冲的字节抓下来，
// 拼成载荷后逐字段比对：
//
//   字节  内容        验证点
//   0     state       低 3 位
//   1,2   左速度      16 位有符号、**大端**、24->16 饱和
//   3,4   右速度      同上
//   5,6   距离        **大端**
//   7,8   roll        有符号大端
//   9,10  pitch       有符号大端
//   11,12 yaw         有符号大端
//
// 以及 tx_len=13、tx_type=0x10。
//=============================================================================
`timescale 1ns/1ps

module tb_telemetry_payload;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;

    // 上报频率给高一点，缩短仿真
    localparam integer TX_RATE_HZ  = 1000;
    localparam integer PAYLOAD_LEN = 13;

    reg clk = 0;
    reg rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //---- DUT 接口 ----
    reg  [2:0]  state;
    reg  signed [23:0] spd_l, spd_r, roll, pitch, yaw;
    reg  [15:0] dist_mm;

    wire        tx_pl_wr;
    wire [7:0]  tx_pl_addr;
    wire [7:0]  tx_pl_data;
    wire        tx_req;
    wire [7:0]  tx_type;
    wire [7:0]  tx_len;
    reg         tx_busy;
    reg         tx_done;
    wire [15:0] sent_cnt, dropped_cnt;
    wire        wait_timeout;

    telemetry_tx #(
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .TX_RATE_HZ   (TX_RATE_HZ)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .state       (state),
        .flags       (4'b0000),      // 本 TB 只验 state 低 3 位；标志位全 0
        .spd_l       (spd_l),
        .spd_r       (spd_r),
        .dist_mm     (dist_mm),
        .roll        (roll),
        .pitch       (pitch),
        .yaw         (yaw),
        .tx_pl_wr    (tx_pl_wr),
        .tx_pl_addr  (tx_pl_addr),
        .tx_pl_data  (tx_pl_data),
        .tx_req      (tx_req),
        .tx_type     (tx_type),
        .tx_len      (tx_len),
        .tx_busy     (tx_busy),
        .tx_done     (tx_done),
        .sent_cnt    (sent_cnt),
        .dropped_cnt (dropped_cnt),
        .wait_timeout(wait_timeout)
    );

    //==================================================================
    // 抓载荷：把 tx_pl_wr 的每个字节按地址存进 pl_buf
    //==================================================================
    reg [7:0] pl_buf [0:15];        // 只关心前 13 字节
    integer   nwr;               // 本轮写了多少字节
    reg       got_req;           // 是否已经拉过 tx_req
    reg [7:0] got_len;
    reg [7:0] got_type;

    integer k;
    initial begin
        for (k = 0; k < 16; k = k + 1) pl_buf[k] = 8'hEE;   // 显眼的地雷值
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            nwr      <= 0;
            got_req  <= 1'b0;
            got_len  <= 8'h00;
            got_type <= 8'h00;
        end else begin
            if (tx_pl_wr) begin
                if (tx_pl_addr < 16) pl_buf[tx_pl_addr] <= tx_pl_data;
                nwr <= nwr + 1;
            end
            if (tx_req) begin
                got_req  <= 1'b1;
                got_len  <= tx_len;
                got_type <= tx_type;
            end
        end
    end

    //==================================================================
    // 工具
    //==================================================================
    integer checks = 0;
    integer errors = 0;
    integer log_fd;

    task check;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got === exp) begin
                $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
            end else begin
                errors = errors + 1;
                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d\n", name, got, exp);
            end
        end
    endtask

    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task wait_ms;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n * (CLK_FREQ_HZ/1000); i = i + 1) step;
        end
    endtask

    //------------------------------------------------------------------
    // 等一轮完整的"写缓冲 -> 拉 req"，然后把 pl_buf 清成地雷值给下一轮用
    //   tx_busy 全程保持 0，tx_done 在 req 之后给一拍，模拟 uart_frame 发完
    //------------------------------------------------------------------
    task capture_one_frame;
        integer guard;
        begin
            // 等地雷值被覆盖（nwr 增长）
            nwr = 0;
            got_req = 1'b0;
            guard = 0;
            while (!got_req && guard < 5_000_000) begin
                step;
                guard = guard + 1;
                if (tx_req) begin
                    step;
                    tx_done = 1'b1;      // 回一个 done
                    step;
                    tx_done = 1'b0;
                end
            end
        end
    endtask

    task clear_buf;
        integer kk;
        begin
            for (kk = 0; kk < 16; kk = kk + 1) pl_buf[kk] = 8'hEE;
            nwr = 0;
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        log_fd = $fopen("telemetry_payload_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建结果文件");

        $fwrite(log_fd, "=== tb_telemetry_payload ===\n");

        // 初值
        state = 3'd2; spd_l = 24'sd0; spd_r = 24'sd0;
        dist_mm = 16'd0; roll = 24'sd0; pitch = 24'sd0; yaw = 24'sd0;
        tx_busy = 1'b0; tx_done = 1'b0;

        repeat (20) step;
        rst_n = 1'b1;
        repeat (5) step;

        //------------------------------------------------------------------
        // 1. 帧头字段：LEN 必须是 13、TYPE 必须是 0x10
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 帧头字段 ---\n");
        capture_one_frame;
        check("1.1 发过 tx_req",            got_req, 1);
        check("1.2 tx_len = 13",            got_len, PAYLOAD_LEN);
        check("1.3 tx_type = 0x10",         got_type, 8'h10);

        //------------------------------------------------------------------
        // 2. 典型值逐字段比对（正数，容易看）
        //    state=2, spd_l=+1234, spd_r=-1234, dist=1234,
        //    roll=+300(-), pitch=-200, yaw=+450
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 典型值（大端 + 有符号）---\n");
        clear_buf;
        state   = 3'd2;
        spd_l   = 24'sd1234;
        spd_r   = -24'sd1234;
        dist_mm = 16'd1234;
        roll    = 24'sd300;
        pitch   = -24'sd200;
        yaw     = 24'sd450;
        capture_one_frame;

        check("2.1  [0] state=2",        pl_buf[0], 8'd2);
        check("2.2  [1] spd_l 高字节",   pl_buf[1], 8'h04);        // 1234 = 0x04D2
        check("2.3  [2] spd_l 低字节",   pl_buf[2], 8'hD2);
        check("2.4  [3] spd_r 高字节",   pl_buf[3], 8'hFB);        // -1234 = 0xFB2E
        check("2.5  [4] spd_r 低字节",   pl_buf[4], 8'h2E);
        check("2.6  [5] dist 高字节",    pl_buf[5], 8'h04);
        check("2.7  [6] dist 低字节",    pl_buf[6], 8'hD2);
        check("2.8  [7] roll 高字节",    pl_buf[7], 8'h01);        // 300 = 0x012C
        check("2.9  [8] roll 低字节",    pl_buf[8], 8'h2C);
        check("2.10 [9] pitch 高字节",   pl_buf[9], 8'hFF);        // -200 = 0xFF38
        check("2.11 [10] pitch 低字节",  pl_buf[10], 8'h38);
        check("2.12 [11] yaw 高字节",    pl_buf[11], 8'h01);       // 450 = 0x01C2
        check("2.13 [12] yaw 低字节",    pl_buf[12], 8'hC2);

        //------------------------------------------------------------------
        // 3. 24 位 -> 16 位饱和（不能直接截断）
        //    +40000 的低 16 位是 -25536，如果直接截断会"看起来在倒车"
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 速度饱和（24->16 有符号）---\n");
        clear_buf;
        state = 3'd1;
        spd_l = 24'sd40000;         // 超上限 -> 32767 = 0x7FFF
        spd_r = -24'sd40000;        // 超下限 -> -32768 = 0x8000
        capture_one_frame;
        check("3.1  +40000 饱和到 +32767 高字节", pl_buf[1], 8'h7F);
        check("3.2  +40000 饱和到 +32767 低字节", pl_buf[2], 8'hFF);
        check("3.3  -40000 饱和到 -32768 高字节", pl_buf[3], 8'h80);
        check("3.4  -40000 饱和到 -32768 低字节", pl_buf[4], 8'h00);
        check("3.5  状态位只有 3 位（state=1）",   pl_buf[0], 8'd1);

        //------------------------------------------------------------------
        // 4. 边界：dist=0 与 dist=65535 都要原样过去
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 距离边界 ---\n");
        clear_buf;
        dist_mm = 16'd0;
        capture_one_frame;
        check("4.1  dist=0 高字节",  pl_buf[5], 8'h00);
        check("4.2  dist=0 低字节",  pl_buf[6], 8'h00);

        clear_buf;
        dist_mm = 16'hFFFF;
        capture_one_frame;
        check("4.3  dist=65535 高字节", pl_buf[5], 8'hFF);
        check("4.4  dist=65535 低字节", pl_buf[6], 8'hFF);

        //------------------------------------------------------------------
        // 5. 每轮都写满 13 字节（不能少写/多写）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 每轮字节数 ---\n");
        clear_buf;
        capture_one_frame;
        check("5.1 一轮正好写 13 字节", nwr, PAYLOAD_LEN);
        check("5.2 没有写到 13 号地址之外", pl_buf[13], 8'hEE);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_telemetry_payload: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_telemetry_payload: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    // 看门狗：30ms 仿真时间还没结束就报错退出
    initial begin
        #30_000_000;
        $fwrite(log_fd, "=== tb_telemetry_payload: 看门狗超时 ===\nRESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
