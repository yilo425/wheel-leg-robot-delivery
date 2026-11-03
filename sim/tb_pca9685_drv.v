//=============================================================================
// tb_pca9685_drv —— 验 PCA9685 驱动的**寄存器序列和数值**
//
// 为什么不用真的 I2C 从机模型：
//   i2c_master 的时序正确性已经由它自己的 tb 覆盖了。
//   这里要验的是"驱动写的寄存器地址对不对、值对不对"，
//   所以用一个**命令记录器**替换 i2c_master：收到 start 就记下
//   (reg_addr, wr_data)，下一拍回 done。这样又快又准。
//
// 验证内容：
//   1. 初始化四条命令：MODE1=0x10 / PRESCALE=121 / MODE2=0x04 / MODE1=0x20
//   2. init_done 在初始化后拉高
//   3. 四条通道写的是 "ON=0, OFF=计数值" 这 4 个寄存器（地址 0x06+4n+0..3）
//   4. 计数换算：count = 102 + height*4
//   5. ★ 前后腿**独立**：lift_cmd=2'b01 时前 2 通道抬起、后 2 通道放下
//   6. height 超 100 被钳位
//   7. 目标没变化时**不重复写**（省 I2C 带宽）
//=============================================================================
`timescale 1ns/1ps

module tb_pca9685_drv;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;          // 50MHz -> 20ns
    localparam integer POWERUP_MS  = 1;           // 缩短，仿真快
    localparam integer OSC_MS      = 1;

    // 期望的计数值
    localparam integer CNT_MIN  = 102;
    localparam integer CNT_STEP = 4;

    reg clk = 0;
    reg rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //---- DUT 接口 ----
    reg  [1:0] lift_cmd = 2'b00;
    reg  [7:0] lift_height = 8'd0;

    wire        start;
    wire [6:0]  dev_addr;
    wire [7:0]  wr_data;
    wire        reg_addr_en;
    wire [7:0]  reg_addr;
    wire        done;
    reg         ack_err = 0;
    reg         busy = 0;      // 假 master：立即完成，busy 恒 0

    wire        init_done;
    wire        err_sticky;

    pca9685_drv #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .PWM_HZ      (50),
        .DEV_ADDR    (7'h40),
        .COUNT_MIN   (CNT_MIN),
        .COUNT_PER_H (CNT_STEP),
        .POWERUP_MS  (POWERUP_MS),
        .OSC_MS      (OSC_MS)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .lift_cmd    (lift_cmd),
        .lift_height (lift_height),
        .stance_front (12'd102),
        .stance_rear  (12'd102),
        .start       (start),
        .dev_addr    (dev_addr),
        .wr_data     (wr_data),
        .reg_addr_en (reg_addr_en),
        .reg_addr    (reg_addr),
        .busy        (busy),
        .done        (done),
        .ack_err     (ack_err),
        .init_done   (init_done),
        .err_sticky  (err_sticky)
    );

    //==================================================================
    // 命令记录器（代替 i2c_master）
    //==================================================================
    reg [7:0] regs [0:255];        // 寄存器镜像
    reg       seen [0:255];        // 该寄存器被写过没有
    integer   nrec = 0;            // 总事务数
    reg       done_r = 0;

    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            regs[i] = 8'h00;
            seen[i] = 1'b0;
        end
    end

    always @(posedge clk) begin
        done_r <= 1'b0;
        if (rst_n && start) begin
            if (reg_addr_en) begin
                regs[reg_addr] = wr_data;
                seen[reg_addr] = 1'b1;
            end
            nrec  <= nrec + 1;
            done_r <= 1'b1;
        end
    end

    assign done = done_r;

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

    // 等一拍数（用一个足够大的上限，避免死等）
    task wait_cycles;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) step;
        end
    endtask

    task wait_ms;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n * (CLK_FREQ_HZ/1000); k = k + 1) step;
        end
    endtask

    // 取某通道的 OFF 计数值
    function integer off_cnt;
        input integer ch;
        begin
            off_cnt = ({28'd0, regs[8'h06 + 4*ch + 3]} << 8) | regs[8'h06 + 4*ch + 2];
        end
    endfunction

    // 等"不再有新事务"（稳定下来）
    task wait_quiet;
        integer last_n;
        integer guard;
        begin
            last_n = -1;
            guard  = 0;
            while ((nrec != last_n) && (guard < 200000)) begin
                last_n = nrec;
                wait_cycles(200);
                guard = guard + 200;
            end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        log_fd = $fopen("pca9685_drv_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建结果文件");

        $fwrite(log_fd, "=== tb_pca9685_drv ===\n");
        $fwrite(log_fd, "  CLK=%0d PRESCALE 应为 %0d\n",
                CLK_FREQ_HZ, (25_000_000/(4096*50)) - 1);

        //------------------------------------------------------------------
        // 1. 复位后：还没初始化
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 上电初始化 ---\n");
        repeat (10) step;
        check("1.1 复位期间 init_done=0",        init_done, 0);

        rst_n = 1'b1;
        wait_ms(POWERUP_MS + 1);

        // 初始化四条命令应该发完了
        check("1.2 PRE_SCALE = 121",             regs[8'hFE], 121);
        check("1.3 MODE1 最终 = 0x20(AI+唤醒)",  regs[8'h00], 8'h20);
        check("1.4 MODE2 = 0x04(推挽)",          regs[8'h01], 8'h04);

        // 等振荡器
        wait_ms(OSC_MS + 1);
        check("1.5 初始化完成 init_done=1",      init_done, 1);
        check("1.6 没有 NACK 错误",              err_sticky, 0);

        //------------------------------------------------------------------
        // 2. 四条通道：ON 寄存器必须是 0
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 通道寄存器布局 ---\n");
        wait_quiet;
        check("2.1 通道0 ON_L=0",  regs[8'h06], 8'h00);
        check("2.2 通道0 ON_H=0",  regs[8'h07], 8'h00);
        check("2.3 通道1 ON_L=0",  regs[8'h0A], 8'h00);
        check("2.4 通道2 ON_L=0",  regs[8'h0E], 8'h00);
        check("2.5 通道3 ON_L=0",  regs[8'h12], 8'h00);

        //------------------------------------------------------------------
        // 3. lift_cmd=00（都放下）-> 4 个通道都是 COUNT_MIN
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 全部放下 ---\n");
        lift_cmd    = 2'b00;
        lift_height = 8'd100;       // 高度给满，但 cmd=0 所以应该放下
        wait_quiet;
        check("3.1 通道0 计数值=102",            off_cnt(0), CNT_MIN);
        check("3.2 通道3 计数值=102",            off_cnt(3), CNT_MIN);

        //------------------------------------------------------------------
        // 4. lift_cmd=11 + height=50 -> 全部 = 102 + 50*4 = 302
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 全部抬起，高度 50 ---\n");
        lift_cmd    = 2'b11;
        lift_height = 8'd50;
        wait_quiet;
        check("4.1 通道0 计数值=302",            off_cnt(0), 302);
        check("4.2 通道1 计数值=302",            off_cnt(1), 302);
        check("4.3 通道2 计数值=302",            off_cnt(2), 302);
        check("4.4 通道3 计数值=302",            off_cnt(3), 302);

        //------------------------------------------------------------------
        // 5. ★ 前后腿独立：只有前腿抬（cmd=01），高度 100
        //      ch0/ch1 = 102+400 = 502 ； ch2/ch3 = 102
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. ★ 只有前腿抬起（验证通道独立）---\n");
        lift_cmd    = 2'b01;
        lift_height = 8'd100;
        wait_quiet;
        check("5.1 通道0(前) 计数值=502",        off_cnt(0), 502);
        check("5.2 通道1(前) 计数值=502",        off_cnt(1), 502);
        check("5.3 通道2(后) 计数值=102",        off_cnt(2), CNT_MIN);
        check("5.4 通道3(后) 计数值=102",        off_cnt(3), CNT_MIN);

        //------------------------------------------------------------------
        // 6. 高度钳位：给 200 应该按 100 算
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 高度超范围钳位 ---\n");
        lift_height = 8'd200;
        wait_quiet;
        check("6.1 height=200 钳到 100 -> 502",  off_cnt(0), 502);

        //------------------------------------------------------------------
        // 7. 目标不变时不重复写（省 I2C 带宽）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 不重复写 ---\n");
        begin : no_rewrite
            integer n_before;
            n_before = nrec;
            wait_ms(2);                 // 等 2ms，足够写完好几轮
            check("7.1 目标没变 -> 事务数不增加", nrec - n_before, 0);
        end

        //------------------------------------------------------------------
        // 8. 只改高度 -> 会重新写
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 改高度会触发重写 ---\n");
        begin : do_rewrite
            integer n_b2;
            n_b2 = nrec;
            lift_height = 8'd60;
            wait_quiet;
            check("8.1 事务数增加 16 条",        nrec - n_b2, 16);
            check("8.2 新计数值 = 102+240=342",  off_cnt(0), 342);
        end

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_pca9685_drv: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_pca9685_drv: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    // 看门狗
    initial begin
        #50_000_000;
        $fwrite(log_fd, "=== tb_pca9685_drv: 看门狗超时 ===\nRESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
