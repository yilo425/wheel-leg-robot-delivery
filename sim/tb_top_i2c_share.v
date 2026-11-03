//=============================================================================
// tb : tb_top_i2c_share
//-----------------------------------------------------------------------------
// 目的: 验证 **TOF(VL53L0X @0x29) 和 IMU(MPU6050 @0x68) 共用同一条 I2C 总线**。
//
//   背景（为什么必须有这条 tb）：
//     tof_driver 和 imu_driver **各自内部**都例化了一份 i2c_master。
//     如果两份 master 直接挂到同一条 scl/sda 上，它们会同时发起传输、
//     互相打断 —— 读回的数据是坏的，而且**不会报错**（只表现为"偶尔读到垃圾"）。
//     top.v §1.6 用 i2c_bus_arb 发 grant 解决：没拿到 grant 的驱动
//     不会发 start。这条 tb 就是证明它真的管用。
//
//-----------------------------------------------------------------------------
// 本 tb 怎么证明"仲裁真的对了"
//-----------------------------------------------------------------------------
//   1) **两个器件都真的被服务了**：TOF 出 dist_mm、IMU 出 roll，值都对得上从机给的数；
//   2) ★ **交叉污染检查**（最有力的一条）：
//        两块从机的寄存器表里选一个"只有对方会写"的地址 ——
//        MPU6050 的 0x19（采样率分频，VL53L0X 的序列永远不碰它），
//        VL53L0X 的 0x4E（官方 tuning 表里 0x4E=0x2C，MPU 的序列永远不碰它）。
//        跑到最后这两个地址在"对方的"模型里必须还是初值 0。
//        如果从机模型错误地回应了不属于自己的事务，这两个值就会被写脏 —— 立刻红。
//        （从机模型已经修成"只有地址命中才 ACK / 才处理后续字节"，
//          见 vl53l0x_slave_model.v 里的说明。）
//   3) **长时间稳定**：连续若干帧里 TOF 的距离和 IMU 的姿态都不许跳变；
//   4) **协议质量**：两个模型的 hold_err 都必须为 0（SCL 高电平期间 SDA 不能变）；
//   5) **授权确实发生过**：两边的 grant 计数都 > 0（真的在轮流用总线）。
//
//   时间尺度：仿真里 SCL=1MHz / SAMPLE_MS=1ms（真实是 400kHz/50ms），
//   比例关系一致，只是快几十倍。
//
// 结果: top_i2c_share_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_top_i2c_share;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;
    localparam integer MS_TICKS    = CLK_FREQ_HZ / 1000;

    localparam integer SAMPLE_MS    = 1;      // 两个驱动都用 1ms，加快仿真
    localparam integer BOOT_WAIT_MS = 1;
    localparam integer TIMEOUT_MS   = 1;
    localparam integer SCL_HZ       = 1_000_000;
    localparam integer I2C_TMO_MS   = 1;

    localparam integer POR_CYCLES    = 16;
    localparam integer SPEED_WIN     = 1000;
    localparam integer DIST_STOP_MM  = 200;
    localparam integer DIST_SLOW_MM  = 500;
    localparam integer DIST_CLEAR_MM = 800;

    localparam [6:0] TOF_ADDR = 7'h29;
    localparam [6:0] IMU_ADDR = 7'h68;

    // 从机给的原始值（和 tb_imu_driver / tb_top_tof_i2c 里一致）
    localparam [15:0] TOF_RANGE = 16'd1234;
    localparam [15:0] IMU_AY    = 16'd8192;      // -> roll = (8192*358)>>10 = 2864
    localparam integer EXP_ROLL = 2864;

    // 交叉污染用的寄存器地址
    localparam [7:0] MPU_ONLY_REG = 8'h19;       // MPU 写 0x09；VL53L0X 不碰
    localparam [7:0] TOF_ONLY_REG = 8'h4E;       // VL53L0X 写 0x2C；MPU 不碰

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk_50m = 0;
    reg         rst_key_n = 0;
    reg  [1:0]  key = 2'b11;
    reg  [3:0]  enc_a = 4'b0000;
    reg  [3:0]  enc_b = 4'b0000;
    reg         uart_rx = 1'b1;
    wire        uart_tx;
    reg         goal_reached = 1;
    reg  [1:0]  lift_lim_n   = 2'b11;
    reg  [1:0]  lift_fault_n = 2'b11;

    wire        i2c_scl;
    wire        i2c_sda;
    wire [3:0]  m_pwm, m_dir;
    wire        m_en;
    wire [3:0]  led;
    wire [1:0]  lift_pwm;
    wire [2:0]  state_dbg;
    wire [1:0]  speed_scale_dbg;
    wire        error_flag;

    always #(CLK_PERIOD/2) clk_50m = ~clk_50m;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;

    //------------------------------------------------------------------
    // ★ 两个驱动都开，两个器件挂**同一条** scl/sda
    //------------------------------------------------------------------
    top #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .POR_CYCLES       (POR_CYCLES),
        // ---- TOF ----
        .TOF_ENABLE       (1),
        // ★ 本 tb 测的是 **I2C 共享总线**（VL53L0X + MPU6050）：
        //   显式 TOF_IF=0。top.v 默认是 1（HC-SR04 超声波），
        //   不写这行 TOF 就不走 I2C，共享总线的用例全废。
        .TOF_IF           (0),
        .TOF_SAMPLE_MS    (SAMPLE_MS),
        .TOF_TIMEOUT_MS   (TIMEOUT_MS),
        .TOF_BOOT_WAIT_MS (BOOT_WAIT_MS),
        .TOF_I2C_SCL_HZ   (SCL_HZ),
        .TOF_I2C_TIMEOUT_MS (I2C_TMO_MS),
        .TOF_DIST_MAX_MM  (2000),
        .TOF_DEV_ADDR     (TOF_ADDR),
        // ---- IMU（这次的重点：打开它）----
        .IMU_ENABLE       (1),
        .IMU_SAMPLE_MS    (SAMPLE_MS),
        .IMU_TIMEOUT_MS   (TIMEOUT_MS),
        .IMU_BOOT_WAIT_MS (BOOT_WAIT_MS),
        .IMU_I2C_SCL_HZ   (SCL_HZ),
        .IMU_I2C_TIMEOUT_MS (I2C_TMO_MS),
        .IMU_DEV_ADDR     (IMU_ADDR),
        // ---- 其它 ----
        .SPEED_WIN        (SPEED_WIN),
        .DIST_STOP_MM     (DIST_STOP_MM),
        .DIST_SLOW_MM     (DIST_SLOW_MM),
        .DIST_CLEAR_MM    (DIST_CLEAR_MM),
        .AVOID_TIMEOUT_MS (100),
        .LIFT_TIMEOUT_MS  (20),
        .CROSS_TIMEOUT_MS (50),
        .CROSS_MIN_MS     (5),
        .LIFT_SETTLE_MS   (5),
        .SERVO_DEBOUNCE_MS (1),
        .TELEM_RATE_HZ    (10)
    ) dut (
        .clk_50m      (clk_50m),
        .rst_key_n    (rst_key_n),
        .m_pwm        (m_pwm),
        .m_dir        (m_dir),
        .m_en         (m_en),
        .enc_a        (enc_a),
        .enc_b        (enc_b),
        .key          (key),
        .led          (led),
        .i2c_scl      (i2c_scl),
        .i2c_sda      (i2c_sda),
        .uart_tx      (uart_tx),
        .uart_rx      (uart_rx),
        .goal_reached (goal_reached),
        .lift_lim_n   (lift_lim_n),
        .lift_fault_n (lift_fault_n),
        .lift_pwm     (lift_pwm),
        .state_dbg    (state_dbg),
        .speed_scale_dbg (speed_scale_dbg),
        .error_flag   (error_flag)
    );

    //------------------------------------------------------------------
    // 两个从机模型 —— 挂在同一条总线上（模型自带 pullup）
    //------------------------------------------------------------------
    vl53l0x_slave_model #(.DEV_ADDR (TOF_ADDR)) u_tof (
        .scl (i2c_scl),
        .sda (i2c_sda)
    );

    mpu6050_slave_model #(.DEV_ADDR (IMU_ADDR)) u_mpu (
        .scl (i2c_scl),
        .sda (i2c_sda)
    );

    //------------------------------------------------------------------
    // 监视
    //------------------------------------------------------------------
    integer tof_valid_cnt = 0;
    integer imu_valid_cnt = 0;
    integer tof_err_cnt   = 0;
    integer imu_err_cnt   = 0;
    integer tof_grant_cnt = 0;
    integer imu_grant_cnt = 0;
    reg signed [15:0] last_roll = 0;
    reg [15:0]        last_dist = 0;

    always @(posedge clk_50m) begin
        if (rst_key_n) begin
            if (dut.tof_grant) tof_grant_cnt = tof_grant_cnt + 1;
            if (dut.imu_grant) imu_grant_cnt = imu_grant_cnt + 1;
            // tof_driver / imu_driver 的输出脉冲（下一拍才被这里看到，值已经稳定）
            if (dut.dist_valid) begin
                tof_valid_cnt = tof_valid_cnt + 1;
                last_dist     = dut.dist_mm;
            end
            if (dut.dist_err) tof_err_cnt = tof_err_cnt + 1;
            if (dut.imu_valid) begin
                imu_valid_cnt = imu_valid_cnt + 1;
                last_roll     = dut.imu_roll;
            end
            if (dut.imu_err_pulse) imu_err_cnt = imu_err_cnt + 1;
        end
    end

    //------------------------------------------------------------------
    // 工具
    //------------------------------------------------------------------
    initial begin
        log_fd = $fopen("top_i2c_share_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 top_i2c_share_result.txt");
    end

    task check;
        input [1023:0] name;
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

    task step;
        begin
            @(posedge clk_50m);
            #1;
        end
    endtask

    task wait_ms;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n * MS_TICKS; k = k + 1) step;
        end
    endtask

    // 等"至少又出了 n 帧有效数据"，两个器件各等各的
    task wait_tof_frames;
        input integer n;
        integer base, k;
        begin
            base = tof_valid_cnt;
            k = 0;
            while (((tof_valid_cnt - base) < n) && (k < 20 * MS_TICKS)) begin
                step; k = k + 1;
            end
        end
    endtask

    task wait_imu_frames;
        input integer n;
        integer base, k;
        begin
            base = imu_valid_cnt;
            k = 0;
            while (((imu_valid_cnt - base) < n) && (k < 20 * MS_TICKS)) begin
                step; k = k + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer i;
    integer tof_stable_bad, imu_stable_bad;

    initial begin
        $fwrite(log_fd, "=== tb_top_i2c_share: TOF + IMU 共享一条 I2C 总线 ===\n");
        $fwrite(log_fd, "  SCL=%0dHz  SAMPLE=%0dms  TOF_ADDR=0x%02h  IMU_ADDR=0x%02h\n",
                SCL_HZ, SAMPLE_MS, TOF_ADDR, IMU_ADDR);
        $fwrite(log_fd, "  从机: TOF 给 %0dmm;  IMU 给 ay=%0d -> 期望 roll=%0d\n",
                TOF_RANGE, IMU_AY, EXP_ROLL);

        // 从机数据源
        u_tof.range_mm = TOF_RANGE;
        u_mpu.ay       = IMU_AY;
        u_mpu.ax       = 16'sd0;
        u_mpu.gz       = 16'sd0;

        //------------------------------------------------------------------
        // 1. 复位
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位 ---\n");
        rst_key_n = 1'b0;
        repeat (10) step;
        check("1.1 复位中 SCL 释放为高", (i2c_scl === 1'b1) ? 1 : 0, 1);
        check("1.2 复位中 SDA 释放为高", (i2c_sda === 1'b1) ? 1 : 0, 1);
        rst_key_n = 1'b1;
        repeat (POR_CYCLES + 30) step;

        //------------------------------------------------------------------
        // 2. 两个器件都真的被服务了
        //    （107 步 TOF 初始化 + MPU 初始化都要跑完，给足时间）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 两个器件都出数 ---\n");
        wait_tof_frames(2);
        wait_imu_frames(2);

        check("2.1 TOF 出了有效帧", (tof_valid_cnt >= 2) ? 1 : 0, 1);
        check("2.2 ★TOF 距离 == 1234", last_dist, TOF_RANGE);
        check("2.3 IMU 出了有效帧", (imu_valid_cnt >= 2) ? 1 : 0, 1);
        check("2.4 ★IMU roll == 2864", last_roll, EXP_ROLL);

        //------------------------------------------------------------------
        // 3. 从机侧证据：各自的初始化真的到达了各自的器件
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 各自的初始化到达各自的器件 ---\n");
        check("3.1 VL53L0X 收到 0x88=0x00",  u_tof.regs[8'h88], 8'h00);
        check("3.3 VL53L0X 收到 0x4E=0x2C",  u_tof.regs[TOF_ONLY_REG], 8'h2C);
        // ⚠️ 这里**不能**断言 regs[0x00]==1：0x00(SYSRANGE_START) 在"测距前导"里
        //    会先写 0x00=0x00、再写 0x00=0x01，紧接着 D_START 又写 0x00=0x01 ——
        //    采样时刻落在中间就会读到 0（本 tb 第一版就是这么误报的）。
        //    改用从机里**单调递增**的计数：它只增不减，不受采样时刻影响。
        check("3.2 VL53L0X 收到过 0x00=0x01（启动过测距）",
              (u_tof.start_meas_cnt > 0) ? 1 : 0, 1);
        check("3.4 MPU6050 收到 0x6B=0x00",  u_mpu.regs[8'h6B], 8'h00);
        check("3.5 MPU6050 收到 0x19=0x09",  u_mpu.regs[MPU_ONLY_REG], 8'h09);
        check("3.6 TOF 模型收到过自己的地址+W", (u_tof.addr_w_cnt > 0) ? 1 : 0, 1);
        check("3.7 IMU 模型收到过自己的地址+W", (u_mpu.addr_w_cnt > 0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 4. ★ 交叉污染检查（本 tb 最有价值的两条）
        //      MPU 只会写 0x19；VL53L0X 的序列不碰它
        //        -> 如果 VL53L0X 模型错误地接受了 MPU 的事务，这里会变成 0x09
        //      VL53L0X 只会写 0x4E=0x2C；MPU 的序列不碰它
        //        -> 如果 MPU 模型错误地接受了 TOF 的事务，这里会变成 0x2C
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. ★ 交叉污染检查 ---\n");
        check("4.1 ★VL53L0X 的 0x19 未被 MPU 事务污染", u_tof.regs[MPU_ONLY_REG], 8'h00);
        check("4.2 ★MPU6050 的 0x4E 未被 TOF 事务污染", u_mpu.regs[TOF_ONLY_REG], 8'h00);

        //------------------------------------------------------------------
        // 5. 协议质量 + 授权确实发生过
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. I2C 协议与授权 ---\n");
        check("5.1 TOF 侧：SCL 高时 SDA 稳定", u_tof.hold_err, 0);
        check("5.2 IMU 侧：SCL 高时 SDA 稳定", u_mpu.hold_err, 0);
        check("5.3 TOF 拿到过 grant", (tof_grant_cnt > 0) ? 1 : 0, 1);
        check("5.4 IMU 拿到过 grant", (imu_grant_cnt > 0) ? 1 : 0, 1);
        check("5.5 两个驱动都用过总线（TOF 有事务）", (u_tof.start_cnt > 0) ? 1 : 0, 1);
        check("5.6 两个驱动都用过总线（IMU 有事务）", (u_mpu.start_cnt > 0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 6. ★ 长时间稳定：连续多帧两边都不能跳变
        //    （仲裁如果偶尔漏掉授权，最典型的表现就是某帧读到错数据）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. ★ 连续多帧稳定 ---\n");
        tof_stable_bad = 0;
        imu_stable_bad = 0;
        for (i = 0; i < 12; i = i + 1) begin
            wait_tof_frames(1);
            wait_imu_frames(1);
            if (last_dist !== TOF_RANGE) tof_stable_bad = tof_stable_bad + 1;
            if (last_roll !== EXP_ROLL)  imu_stable_bad = imu_stable_bad + 1;
        end
        check("6.1 12 轮里 TOF 距离始终正确（跳变次数=0）", tof_stable_bad, 0);
        check("6.2 12 轮里 IMU roll 始终正确（跳变次数=0）", imu_stable_bad, 0);
        check("6.3 期间 TOF 帧数 >= 12", (tof_valid_cnt >= 12) ? 1 : 0, 1);
        check("6.4 期间 IMU 帧数 >= 12", (imu_valid_cnt >= 12) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 7. 共享总线上 TOF 通路仍然能驱动行为（进 AUTO：距离 1234 > CLEAR 800）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. TOF 仍驱动车辆行为 ---\n");
        key[0] = 1'b0; repeat (3) step; key[0] = 1'b1; repeat (3) step;
        step;
        check("7.1 启动后 state=AUTO", state_dbg, 3'd2);
        check("7.2 距离 1234mm -> speed=FULL", speed_scale_dbg, 2'd2);
        check("7.3 编号 3.4 之后 IMU 仍在出数", (imu_valid_cnt > 0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n  [info] TOF 帧=%0d  IMU 帧=%0d  TOF grant 拍=%0d  IMU grant 拍=%0d\n",
                tof_valid_cnt, imu_valid_cnt, tof_grant_cnt, imu_grant_cnt);
        $fwrite(log_fd, "\n=== tb_top_i2c_share: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 超时保护
    //------------------------------------------------------------------
    initial begin
        #50_000_000;                     // 50ms
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
