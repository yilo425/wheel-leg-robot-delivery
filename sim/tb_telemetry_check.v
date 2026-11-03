//=============================================================================
// tb_telemetry_check —— 只回答一个问题：**uart_tx 到底会不会动**
//
// 为什么需要它：
//   tb_top_integration 声明了 wire uart_tx 却从没解码过它，
//   telemetry 发送路径在上板前**没有任何断言覆盖**。
//   结果：上板实测 uart_tx 一直高电平（2 秒零跳变），仿真却是绿的。
//
// 这里用**和硬件完全一致的参数**（TOF_ENABLE=1 / TOF_IF=1 / IMU_ENABLE=0）
// 例化 top，把 TELEM_RATE_HZ 提到 1000Hz 缩短仿真，监视 uart_tx 的跳变。
//=============================================================================
`timescale 1ns/1ps

module tb_telemetry_check;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;      // 50MHz

    reg  clk   = 0;
    reg  rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // DUT 端口
    wire [7:0] led;
    wire       uart_tx;
    wire       uart_tx_dbg;
    wire       sr04_trig;
    wire [1:0] lift_pwm;
    wire [3:0] m_pwm;
    wire [3:0] m_dir;
    wire       m_en;
    wire [2:0] state_dbg;
    wire [1:0] speed_scale_dbg;
    wire       error_flag;

    top #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .POR_CYCLES      (100),
        .TOF_ENABLE      (1),
        .TOF_IF          (1),          // ★ 和上板一致
        .IMU_ENABLE      (0),
        .TELEM_RATE_HZ   (1000),       // 加速仿真
        .SR04_TMO_MS     (2),
        .SR04_SAMPLE_MS  (3),
        .SR04_BOOT_WAIT_MS(1)
    ) dut (
        .clk_50m         (clk),
        .rst_key_n       (rst_n),
        .key             (2'b11),      // 两个键都不按
        .uart_tx         (uart_tx),
        .uart_rx         (1'b1),
        .i2c_scl         (),
        .i2c_sda         (),
        .sr04_trig       (sr04_trig),
        .sr04_echo       (1'b0),       // 没有回波 -> SR04 一直 err，不影响 TX
        .goal_reached    (1'b1),
        .lift_lim_n      (2'b11),
        .lift_fault_n    (2'b11),
        .led             (led),
        .lift_pwm        (lift_pwm),
        .m_pwm           (m_pwm),
        .m_dir           (m_dir),
        .m_en            (m_en),
        .enc_a           (4'b0000),
        .enc_b           (4'b0000),
        .state_dbg       (state_dbg),
        .speed_scale_dbg (speed_scale_dbg),
        .error_flag      (error_flag),
        .uart_tx_dbg     (uart_tx_dbg)
    );

    //------------------------------------------------------------------
    // 监视 uart_tx 的跳变 + 抓帧头
    //------------------------------------------------------------------
    integer edges   = 0;
    integer frames  = 0;
    integer bytes_rx= 0;
    reg     prev    = 1'b1;
    real    baud_us = 8.68;                 // 115200 8N1 一位约 8.68us

    always @(uart_tx) begin
        edges = edges + 1;
        prev  = uart_tx;
    end

    // 简易 UART 接收：检测起始位 -> 按位采样 -> 数帧头 AA 55
    reg     rx_active = 0;
    integer bit_cnt   = 0;
    integer clk_cnt   = 0;
    reg [9:0] shreg   = 10'h3FF;
    reg [7:0] last_b  = 8'h00;
    reg [7:0] prev_b  = 8'h00;
    localparam integer BIT_CLKS = 434;      // 50MHz / 115200 ≈ 434 拍一位

    always @(posedge clk) begin
        if (!rx_active) begin
            if (!uart_tx) begin
                rx_active = 1;
                bit_cnt   = 0;
                clk_cnt   = BIT_CLKS/2;     // 半个位对齐到起始位中点
                shreg     = 10'h3FF;
            end
        end else begin
            if (clk_cnt >= BIT_CLKS-1) begin
                clk_cnt = 0;
                shreg   = {uart_tx, shreg[9:1]};
                bit_cnt = bit_cnt + 1;
                if (bit_cnt == 9) begin           // 起始1 + 数据8
                    last_b  = shreg[8:1];
                    prev_b  = last_b;
                    if (last_b == 8'h55 && prev_b == 8'h55) frames = frames + 1;
                    bytes_rx = bytes_rx + 1;
                    rx_active = 0;
                end
            end else begin
                clk_cnt = clk_cnt + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // 结果文件（格式与其它 tb 一致，便于 run_sim.do 统一汇总）
    //------------------------------------------------------------------
    integer fd;
    integer nchk = 0;
    integer nerr = 0;

    task chk;
        input [1023:0] name;
        input         cond;
        begin
            nchk = nchk + 1;
            if (cond) begin
                $fwrite(fd, "  [ OK ] %0s\n", name);
            end else begin
                nerr = nerr + 1;
                $fwrite(fd, "  [FAIL] %0s\n", name);
            end
        end
    endtask

    initial begin
        fd = $fopen("telemetry_check_result.txt", "w");
        $display("=== tb_telemetry_check: TOF_ENABLE=1 TOF_IF=1 TELEM=1000Hz ===");
        repeat (10) @(posedge clk);
        rst_n = 1;
        // 让 telemetry 有 3 个上报周期（1000Hz -> 3ms）
        #3_000_000;

        $display("  uart_tx 跳变次数 = %0d", edges);
        $display("  UART 收到字节数  = %0d", bytes_rx);
        $display("  uart_tx_dbg 和 uart_tx 一致 ? %0d", (uart_tx_dbg === uart_tx) ? 1 : 0);

        chk("1.1 uart_tx 有跳变（telemetry 真的在发）", edges > 0);
        chk("1.2 收到了完整 UART 字节",                 bytes_rx > 0);
        chk("1.3 uart_tx_dbg 与 uart_tx 同相",         (uart_tx_dbg === uart_tx));

        $fwrite(fd, "\n=== tb_telemetry_check: %0d checks, %0d errors ===\n", nchk, nerr);
        if (nerr == 0) $fwrite(fd, "RESULT: PASS\n");
        else           $fwrite(fd, "RESULT: FAIL\n");
        $fclose(fd);

        if (nerr == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL");
        $finish;
    end

endmodule
