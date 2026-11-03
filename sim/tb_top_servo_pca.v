//=============================================================================
// tb_top_servo_pca —— 验 top.v 里 SERVO_IF=1 的 PCA9685 通路
//
// 为什么单独一个 tb：
//   top 的默认是 SERVO_IF=0（直接 PWM），所以两条通路必须**分别**验证。
//   这个 tb 把 SERVO_IF 传成 1，接上 pca9685_slave_model（0x40），检查：
//     ① 顶层能正常复位、跑起来（generate 分支没错、I2C 引脚直接接到主机）
//     ② PCA9685 初始化序列真的跑起来了，且**寄存器值对**
//        MODE1=0x10(SLEEP) -> PRE_SCALE=121 -> MODE2=0x04 -> MODE1=0x20
//     ③ 总线上有 ACK（不是 NACK），I2C 时序无违规（SCL 高电平期间 SDA 不变）
//     ④ force 抬腿命令 -> 16 次事务（4 通道 × 4 字节），通道 OFF 计数对
//     ⑤ 目标不变 -> 不重复写（省 I2C 带宽）
//     ⑥ 只改高度 -> 再 16 次，计数跟着变
//
// ★ 时序注意：
//   复位后 driver 除了初始化，还会跑一轮"当前目标"的写入（last_cmd/last_h
//   的初值和实际目标不一定相等）。所以**不能用固定 wait 去数事务**，
//   必须等 driver 真正回到 IDLE 再检查 —— 见 wait_pca_idle。
//=============================================================================
`timescale 1ns/1ps

module tb_top_servo_pca;

    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;

    reg clk = 0;
    reg rst_key_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //---- 输入 ----
    reg        uart_rx      = 1'b1;
    reg  [3:0] enc_a        = 4'b0000;
    reg  [3:0] enc_b        = 4'b0000;
    reg  [1:0] key          = 2'b11;     // 低有效，11 = 都没按
    reg        sr04_echo    = 1'b0;
    reg        goal_reached = 1'b0;
    reg  [1:0] lift_lim_n   = 2'b11;
    reg  [1:0] lift_fault_n = 2'b11;

    //---- 输出 ----
    wire [3:0] m_pwm, m_dir, m_dir2, led;
    wire       m_en, uart_tx, sr04_trig;
    wire [1:0] lift_pwm;
    wire [2:0] state_dbg;
    wire [1:0] speed_scale_dbg;
    wire       error_flag, uart_tx_dbg;

    //---- I2C（开漏，必须上拉，否则是 x）----
    wire i2c_scl, i2c_sda;
    pullup (i2c_scl);
    pullup (i2c_sda);

    //---- PCA9685 从机模型（地址 0x40）----
    pca9685_slave_model #(.DEV_ADDR(7'h40)) u_slv (
        .scl (i2c_scl),
        .sda (i2c_sda)
    );

    //---- DUT：SERVO_IF=1 ----
    top #(
        .CLK_FREQ_HZ       (CLK_FREQ_HZ),
        .TOF_ENABLE        (0),        // 用桩距离，避免真实 tof_driver
        .TOF_IF            (1),        // 超声波（SR04 分支，I2C 线归舵机）
        .IMU_ENABLE        (0),        // ★ 必须 0，否则抢 I2C 总线
        .SERVO_IF          (1),        // ★ 走 PCA9685
        .SERVO_I2C_SCL_HZ  (100_000),
        .SERVO_I2C_TIMEOUT_MS (2),     // 缩短，仿真快
        .SERVO_PWM_HZ      (50),
        .AVOID_TIMEOUT_MS  (100),
        .LIFT_TIMEOUT_MS   (20),
        .CROSS_TIMEOUT_MS  (50),
        .LIFT_SETTLE_MS    (5),
        .SERVO_DEBOUNCE_MS (1),
        .TELEM_RATE_HZ     (100),
        .ESTOP_FROM_KEY    (1)
    ) dut (
        .clk_50m        (clk),
        .rst_key_n      (rst_key_n),
        .uart_tx        (uart_tx),
        .uart_rx        (uart_rx),
        .m_pwm          (m_pwm),
        .m_dir          (m_dir),
        .m_dir2         (m_dir2),
        .m_en           (m_en),
        .enc_a          (enc_a),
        .enc_b          (enc_b),
        .key            (key),
        .led            (led),
        .i2c_scl        (i2c_scl),
        .i2c_sda        (i2c_sda),
        .sr04_trig      (sr04_trig),
        .sr04_echo      (sr04_echo),
        .goal_reached   (goal_reached),
        .lift_pwm       (lift_pwm),
        .lift_lim_n     (lift_lim_n),
        .lift_fault_n   (lift_fault_n),
        .state_dbg      (state_dbg),
        .speed_scale_dbg(speed_scale_dbg),
        .error_flag     (error_flag),
        .uart_tx_dbg    (uart_tx_dbg)
    );

    //==================================================================
    // 统计
    //==================================================================
    integer n_start = 0;        // pca_start 脉冲数 = I2C 事务数
    integer n_scl   = 0;        // SCL 边沿数
    reg     scl_d   = 1'b1;
    integer n_at_init = -1;     // init_done 升起那一刻的事务数（应该 = 4）

    always @(posedge clk) begin
        if (dut.pca_start) n_start <= n_start + 1;
        if (i2c_scl !== scl_d) begin
            n_scl <= n_scl + 1;
            scl_d <= i2c_scl;
        end
        if (dut.pca_init_done && (n_at_init < 0)) n_at_init <= n_start;
    end

    //==================================================================
    // 工具
    //==================================================================
    integer checks = 0;
    integer errors = 0;
    integer log_fd;

    task check;
        input [511:0] name;      // 512 位 = 64 字节，中文名字不会被截断
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

    task step; begin @(posedge clk); #1; end endtask

    task wait_ms;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n * (CLK_FREQ_HZ/1000); k = k + 1) step;
        end
    endtask

    // 等 driver 真正空闲（连续 2ms 都在 S_IDLE 且 busy=0）
    task wait_pca_idle;
        input integer max_ms;
        integer k, good;
        begin
            good = 0;
            for (k = 0; k < max_ms; k = k + 1) begin
                wait_ms(1);
                if (dut.g_servo_pca.u_pca9685.st == 4'd3 && !dut.pca_busy)
                    good = good + 1;
                else
                    good = 0;
                if (good >= 2) k = max_ms;   // 提前退出
            end
        end
    endtask

    // 等 init_done 升起（最多 max_ms）
    task wait_init_done;
        input integer max_ms;
        integer k;
        begin
            k = 0;
            while (!dut.pca_init_done && (k < max_ms)) begin
                wait_ms(1);
                k = k + 1;
            end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        log_fd = $fopen("top_servo_pca_result.txt", "w");
        $fwrite(log_fd, "=== tb_top_servo_pca（SERVO_IF=1，挂 PCA9685 从机模型）===\n");

        repeat (20) step;
        check("1.1 复位期间 pca_init_done=0", dut.pca_init_done, 0);
        check("1.2 复位期间 I2C 无事务",      n_start, 0);

        rst_key_n = 1'b1;

        //------------------------------------------------------------------
        // 2. 初始化
        //------------------------------------------------------------------
        wait_init_done(40);
        $fwrite(log_fd, "\n--- 2. 初始化序列 ---\n");
        check("2.1 初始化完成 init_done=1", dut.pca_init_done, 1);
        check("2.2 init_done 那一刻正好 4 次事务", n_at_init, 4);
        check("2.3 SCL 有翻转（总线真的动了）",   (n_scl > 50) ? 1 : 0, 1);
        check("2.4 MODE1 先写过 SLEEP(0x10)",   u_slv.init_mode1_sleep, 1);
        check("2.5 PRE_SCALE=121（50Hz）",      u_slv.prescale_val, 121);
        check("2.6 MODE2=0x04（OUTDRV）",       u_slv.mode2_val, 4);
        check("2.7 MODE1 最后=0x20（AI+唤醒）", u_slv.mode1_val, 32);

        $fwrite(log_fd, "\n--- 3. PCA9685 有应答（ACK）---\n");
        check("3.1 没有 NACK（err_sticky=0）",   dut.pca_err_sticky, 0);
        check("3.2 地址全对（addr_bad_cnt=0）",  u_slv.addr_bad_cnt, 0);
        check("3.3 I2C 时序无违规（hold_err=0）", u_slv.hold_err, 0);

        // 让复位后那一轮"当前目标"的写入跑完
        wait_pca_idle(80);

        //------------------------------------------------------------------
        // 4. force 抬腿 -> 16 次事务
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 抬腿命令触发 16 次写入 ---\n");
        begin : lift
            integer nb1, ob1;
            nb1 = n_start;
            ob1 = u_slv.off_wr_cnt;      // 基线（复位后那轮同步已经写过 4 次）
            force dut.lift_cmd        = 2'b11;
            force dut.cfg_lift_height = 8'd50;
            wait_pca_idle(80);
            check("4.1 新增 16 次 I2C 事务", n_start - nb1, 16);
            check("4.2 本轮 4 个通道都写了 OFF", u_slv.off_wr_cnt - ob1, 4);
            check("4.3 ch0（前左）=102+50*4=302", u_slv.ch_off[0], 302);
            check("4.4 ch1（前右）=302",          u_slv.ch_off[1], 302);
            check("4.5 ch2（后左）=302",          u_slv.ch_off[2], 302);
            check("4.6 ch3（后右）=302",          u_slv.ch_off[3], 302);
        end

        //------------------------------------------------------------------
        // 5. 目标不变 -> 不重复写
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 目标不变不重复写 ---\n");
        begin : no_rewrite
            integer nb2;
            nb2 = n_start;
            wait_ms(10);
            check("5.1 10ms 内事务数不增加", n_start - nb2, 0);
        end

        //------------------------------------------------------------------
        // 6. 只改高度 -> 再 16 次
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 改高度再写 16 次 ---\n");
        begin : height
            integer nb3, ob3;
            nb3 = n_start;
            ob3 = u_slv.off_wr_cnt;
            force dut.cfg_lift_height = 8'd80;
            wait_pca_idle(80);
            check("6.1 新增 16 次 I2C 事务", n_start - nb3, 16);
            check("6.2 本轮又写 4 个通道 OFF", u_slv.off_wr_cnt - ob3, 4);
            check("6.3 ch0=102+80*4=422",    u_slv.ch_off[0], 422);
            check("6.4 ch3=422",             u_slv.ch_off[3], 422);
        end

        //------------------------------------------------------------------
        // 7. 释放 force，确认其它通路正常
        //------------------------------------------------------------------
        release dut.lift_cmd;
        release dut.cfg_lift_height;
        wait_ms(2);
        $fwrite(log_fd, "\n--- 7. 其它通路正常 ---\n");
        check("7.1 state_dbg 不是 X", (^state_dbg === 1'bx) ? 0 : 1, 1);
        check("7.2 m_en 是确定值",    (m_en === 1'bx) ? 0 : 1, 1);
        check("7.3 uart_tx 是确定值", (uart_tx === 1'bx) ? 0 : 1, 1);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_top_servo_pca: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_top_servo_pca: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    initial begin
        #500_000_000;
        $fwrite(log_fd, "=== 看门狗超时 ===\nRESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
