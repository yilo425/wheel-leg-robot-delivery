//=============================================================================
// tb : tb_top_tof_i2c
//-----------------------------------------------------------------------------
// 目的: 验证 **TOF 真的集成进了 top** —— 而且验证的是真实通路，
//       不是"我给了个电平"、也不是 force 灌进去的常数：
//
//          i2c_scl/i2c_sda ──► tof_driver ──► dist_mm/valid/err ──► main_fsm
//                 ▲                                                     │
//        vl53l0x_slave_model                                          state/m_en
//
//       所以本 tb **一个 force 都没有**。tb 只做两件事：
//         1) 通过层次引用改从机模型的"测量值"（range_mm）；
//         2) 读 top 的 dist_mm / dist_valid / dist_err 和 state / m_en。
//
//-----------------------------------------------------------------------------
// 为什么单独一条 tb，而不是塞进 tb_top_integration
//-----------------------------------------------------------------------------
//   tb_top_integration 是**确定性场景**测试（66 项）：它用 force 直接灌
//   dist_mm，好把 FSM 的每个分支都走一遍 —— 那是它该干的事，而且现在跑的是
//   TOF_ENABLE=0 的桩模式。
//
//   本条 tb 关心的是"接线到底通没通"：
//       I2C 上真的出现了初始化写 → 启动测距 → 轮询 0x13 → 读 0x1E/0x1F
//       → 清中断；距离真的从器件搬到 dist_mm；dist_mm 真的能改变车辆行为
//       （150mm -> AVOID 停车；读数失效 -> 限速不冒进）。
//
//   两条 tb 合起来才完整：一条保证"逻辑对"，一条保证"接线通"。
//
//-----------------------------------------------------------------------------
// 时间尺度
//-----------------------------------------------------------------------------
//   真实配置是 400kHz / SAMPLE=50ms。仿真里改成 1MHz / SAMPLE=1ms：
//   分频、轮询次数、超时与采样的相对关系完全一样，只是快几十倍。
//   绝对时间常数在 top.v 的参数默认值里（TOF_*），这里只是覆写。
//
// 结果: top_tof_i2c_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_top_tof_i2c;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;              // 20ns = 50MHz
    localparam integer MS_TICKS    = CLK_FREQ_HZ / 1000;   // 1ms = 50000 拍

    // 仿真加速用的 TOF 参数（比例关系和真实值一致）
    localparam integer TOF_SAMPLE_MS    = 1;
    localparam integer TOF_TIMEOUT_MS   = 1;
    localparam integer TOF_BOOT_WAIT_MS = 1;
    localparam integer TOF_SCL_HZ       = 1_000_000;
    localparam integer TOF_I2C_TMO_MS   = 1;
    localparam integer TOF_DIST_MAX_MM  = 2000;
    localparam [6:0]  TOF_DEV_ADDR      = 7'h29;

    // 让 FSM 快点反应
    localparam integer POR_CYCLES     = 16;
    localparam integer SPEED_WIN      = 1000;
    localparam integer VREF_FULL      = 69;
    localparam integer DIST_STOP_MM   = 200;
    localparam integer DIST_SLOW_MM   = 500;
    localparam integer DIST_CLEAR_MM  = 800;

    //------------------------------------------------------------------
    // 状态 / 速度档编码（与 main_fsm 一致）
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0,
                     S_AUTO  = 3'd2,
                     S_AVOID = 3'd3;

    localparam [1:0] SPD_STOP = 2'd0,
                     SPD_SLOW = 2'd1,
                     SPD_FULL = 2'd2;

    //------------------------------------------------------------------
    // key 常量：一律用 {key[1], key[0]} 拼接写，别裸写 2'bXY
    //   （2'bXY 里 X 是 bit1 —— 本工程在 tb_top_integration 里为此踩过两次）
    //   key 低有效：0 = 按下
    //------------------------------------------------------------------
    localparam [1:0] KEY_IDLE  = {1'b1, 1'b1};
    localparam [1:0] KEY_START = {1'b1, 1'b0};

    //------------------------------------------------------------------
    // VL53L0X 寄存器地址（断言从机收到了什么）
    //------------------------------------------------------------------
    localparam [7:0] R_START   = 8'h00;
    localparam [7:0] R_CLR_INT = 8'h0B;
    localparam [7:0] R_I2C_MODE= 8'h88;
    localparam [7:0] R_RATE_H  = 8'h44;
    localparam [7:0] R_RATE_L  = 8'h45;

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk_50m = 0;
    reg         rst_key_n = 0;
    reg  [1:0]  key = KEY_IDLE;

    reg  [3:0]  enc_a = 4'b0000;
    reg  [3:0]  enc_b = 4'b0000;

    reg         uart_rx = 1'b1;      // 空闲为高（本 tb 不发帧）
    wire        uart_tx;

    reg         goal_reached = 1;
    reg  [1:0]  lift_lim_n   = 2'b11;   // 低有效：0 = 到位
    reg  [1:0]  lift_fault_n = 2'b11;   // 低有效：0 = 故障

    wire        i2c_scl;
    wire        i2c_sda;

    wire [3:0]  m_pwm;
    wire [3:0]  m_dir;
    wire        m_en;
    wire [3:0]  led;
    wire [1:0]  lift_pwm;
    wire [2:0]  state_dbg;
    wire [1:0]  speed_scale_dbg;
    wire        error_flag;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer k;

    // 距离脉冲监视（dist_valid / dist_err 都是单拍脉冲，轮询不到，只能数）
    integer    valid_cnt = 0;
    integer    err_cnt   = 0;
    reg [15:0] last_dist = 16'd0;

    always #(CLK_PERIOD/2) clk_50m = ~clk_50m;

    //==================================================================
    // DUT：★ TOF_ENABLE = 1（默认，也是上板用的那份配置）
    //==================================================================
    top #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .POR_CYCLES       (POR_CYCLES),
        .TOF_ENABLE       (1),
        // ★ 本 tb 测的是 **I2C 的 VL53L0X** 通路：显式 TOF_IF=0。
        //   top.v 默认值是 1（HC-SR04 超声波），不写这行会走超声波分支，
        //   I2C 上一个事务都不会发（实测会挂 26 项）。
        .TOF_IF           (0),
        // 本 tb 只测 TOF 通路：显式关掉 IMU，免得它的 NACK 事务占用总线时间、
        // 影响这里的时延预算（共享总线的用例在 tb_top_i2c_share.v）
        .IMU_ENABLE       (0),
        .TOF_SAMPLE_MS    (TOF_SAMPLE_MS),
        .TOF_TIMEOUT_MS   (TOF_TIMEOUT_MS),
        .TOF_BOOT_WAIT_MS (TOF_BOOT_WAIT_MS),
        .TOF_I2C_SCL_HZ   (TOF_SCL_HZ),
        .TOF_I2C_TIMEOUT_MS (TOF_I2C_TMO_MS),
        .TOF_DIST_MAX_MM  (TOF_DIST_MAX_MM),
        .TOF_DEV_ADDR     (TOF_DEV_ADDR),
        .SPEED_WIN        (SPEED_WIN),
        .VREF_FULL        (VREF_FULL),
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

    //==================================================================
    // I2C 从机模型（VL53L0X）
    //   模型内部带上拉，所以这里不需要再加 pullup 原语。
    //==================================================================
    vl53l0x_slave_model #(
        .DEV_ADDR (TOF_DEV_ADDR)
    ) u_tof (
        .scl (i2c_scl),
        .sda (i2c_sda)
    );

    //==================================================================
    // 读数监视：dist_valid / dist_err 是单拍脉冲，poll 不到，只能数
    //
    //   ⚠️ 时机的细节（本工程在 tb_tof_driver 里已经趟过一次）：
    //      tof_driver 在 D_EVAL 那一拍用**非阻塞赋值**把 dist_mm/dist_valid
    //      一起更新；tb 这个块在同一拍读到的是**上一拍的值**，
    //      所以它实际是在**下一个** posedge 看到 dist_valid=1 ——
    //      而那时 dist_mm 已经是新值了。这样恰好一次、值也对。
    //==================================================================
    always @(posedge clk_50m) begin
        if (rst_key_n) begin
            if (dut.dist_valid) begin
                valid_cnt = valid_cnt + 1;
                last_dist = dut.dist_mm;
            end
            if (dut.dist_err) err_cnt = err_cnt + 1;
        end
    end

    //==================================================================
    // 工具
    //==================================================================
    initial begin
        log_fd = $fopen("top_tof_i2c_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 top_tof_i2c_result.txt");
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
        integer j;
        begin
            for (j = 0; j < n * MS_TICKS; j = j + 1) step;
        end
    endtask

    // 按一次 key[0]（低有效 -> 产生 start 下降沿脉冲）
    task press_start;
        begin
            key[0] = 1'b0;
            repeat (3) step;
            key[0] = 1'b1;
            repeat (3) step;
        end
    endtask

    // 等 n 个新的读数事件（valid 或 err 都算），带超时。
    //
    //   ★ 一定要盯着**监视块的计数**，而不是直接采 dist_valid 这根线：
    //     脉冲只在下一拍才被计数（见上面监视块的说明），
    //     直接采线的话会在计数之前就返回，紧接着的断言就会看到旧值。
    task wait_frames;
        input integer n;
        input integer max_ms;
        integer base;
        integer j;
        begin
            base = valid_cnt + err_cnt;
            j    = 0;
            while ((((valid_cnt + err_cnt) - base) < n) && (j < max_ms * MS_TICKS)) begin
                step;
                j = j + 1;
            end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer v0, e0, d0;

    initial begin
        $fwrite(log_fd, "=== tb_top_tof_i2c: 顶层 TOF 真实通路（I2C，无 force）===\n");
        $fwrite(log_fd, "  参数: CLK=%0dHz SCL=%0dHz SAMPLE=%0dms BOOT=%0dms DIST_MAX=%0dmm ADDR=0x%02h\n",
                CLK_FREQ_HZ, TOF_SCL_HZ, TOF_SAMPLE_MS, TOF_BOOT_WAIT_MS,
                TOF_DIST_MAX_MM, TOF_DEV_ADDR);
        $fwrite(log_fd, "  阈值: STOP=%0d SLOW=%0d CLEAR=%0d mm\n",
                DIST_STOP_MM, DIST_SLOW_MM, DIST_CLEAR_MM);

        //------------------------------------------------------------------
        // 1. 复位 + 释放
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位与释放 ---\n");
        rst_key_n = 1'b0;
        repeat (10) step;
        check("1.1 复位中 state=IDLE", state_dbg, S_IDLE);
        check("1.2 复位中 m_en=0",     m_en, 0);
        rst_key_n = 1'b1;
        repeat (POR_CYCLES + 30) step;
        check("1.3 复位释放: led[0]=1", led[0], 1);
        check("1.4 未启动: state=IDLE", state_dbg, S_IDLE);

        //------------------------------------------------------------------
        // 2. ★ 第一帧真实测距：I2C 初始化 -> 启动 -> 轮询 -> 读数 -> 清中断
        //
        //    这一段是"TOF 真的接上了"的**主要证据**：
        //    下面这些寄存器只有在 I2C 上真的发生过对应传输时才会被写。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. ★ 真实 I2C 测距（无 force，靠从机模型出数）---\n");
        u_tof.range_mm    = 16'd1234;
        u_tof.ready_after = 2;        // 前两次读 0x13 未就绪 -> 验证轮询真的在轮询
        u_tof.poll_cnt    = 0;

        v0 = valid_cnt;
        wait_frames(1, 10);
        check("2.1 收到 dist_valid（测距成功）", (valid_cnt > v0) ? 1 : 0, 1);
        check("2.2 dist_mm == 1234",            last_dist, 1234);

        check("2.3 init 写 0x88=0x00", u_tof.regs[R_I2C_MODE], 8'h00);
        check("2.4 init 写 0x44=0x00", u_tof.regs[R_RATE_H],   8'h00);
        check("2.5 init 写 0x45=0x20", u_tof.regs[R_RATE_L],   8'h20);
        check("2.6 写 0x00=0x01 启动测距", u_tof.regs[R_START], 8'h01);
        check("2.7 写 0x0B=0x01 清中断",   u_tof.regs[R_CLR_INT], 8'h01);
        check("2.8 轮询过 0x13", (u_tof.ptr_13_cnt >= 2) ? 1 : 0, 1);
        check("2.9 读过 0x1E（距离高字节）", (u_tof.ptr_1e_cnt > 0) ? 1 : 0, 1);
        check("2.10 读过 0x1F（距离低字节）", (u_tof.ptr_1f_cnt > 0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 3. I2C 协议质量
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. I2C 协议 ---\n");
        check("3.1 出现过 START",        (u_tof.start_cnt > 0) ? 1 : 0, 1);
        check("3.2 出现过 STOP",         (u_tof.stop_cnt  > 0) ? 1 : 0, 1);
        check("3.3 收到 地址+W",         (u_tof.addr_w_cnt > 0) ? 1 : 0, 1);
        check("3.4 收到 地址+R",         (u_tof.addr_r_cnt > 0) ? 1 : 0, 1);
        check("3.5 ★复合读用了重复START", (u_tof.rstart_cnt > 0) ? 1 : 0, 1);
        check("3.6 地址没错过",          u_tof.addr_bad_cnt, 0);
        check("3.7 ★SCL 高时 SDA 稳定",  u_tof.hold_err, 0);

        //------------------------------------------------------------------
        // 4. ★ 真实距离驱动车辆行为（这一段才是集成的意义）
        //    以前 dist_mm 是常量桩/force，现在是 I2C 上读回来的东西。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 距离 -> 行为 ---\n");
        press_start;
        step;
        check("4.1 启动: state=AUTO", state_dbg, S_AUTO);
        check("4.2 1234mm(远): speed=FULL", speed_scale_dbg, SPD_FULL);
        check("4.3 1234mm: m_en=1",         m_en, 1);

        // 换一个距离（字节模式不同：950 = 0x03B6），证明距离是逐字节搬过来的
        u_tof.range_mm = 16'd950;
        v0 = valid_cnt;
        wait_frames(1, 5);
        check("4.4 第二帧有效",        (valid_cnt > v0) ? 1 : 0, 1);
        check("4.5 dist_mm == 950",    last_dist, 950);
        step;
        check("4.6 950mm(>CLEAR): speed=FULL", speed_scale_dbg, SPD_FULL);

        // 近距 -> 必须避障停车
        u_tof.range_mm = 16'd150;
        v0 = valid_cnt;
        wait_frames(1, 5);
        check("4.7 150mm: 收到有效读数", (valid_cnt > v0) ? 1 : 0, 1);
        step;
        check("4.8 ★150mm(< STOP): state=AVOID", state_dbg, S_AVOID);
        check("4.9 ★AVOID: m_en=0（停车）",      m_en, 0);
        check("4.10 AVOID: led[2]=1",            led[2], 1);

        //------------------------------------------------------------------
        // 5. ★ 读数失效（超量程）—— fail-safe：限速不冒进，旧读数不刷新
        //
        //    这是 TOF 通路里最容易出事的一段：器件报了垃圾值，
        //    main_fsm 侧不能因此"以为前方通畅"冲出去。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. ★ 超量程读数 -> dist_err + 限速 ---\n");
        u_tof.range_mm = 16'd3000;      // > DIST_MAX_MM = 无效读数
        e0 = err_cnt;
        d0 = last_dist;
        wait_frames(1, 5);
        check("5.1 超量程 -> dist_err", (err_cnt > e0) ? 1 : 0, 1);
        check("5.2 无效读数不刷新 dist_mm", dut.dist_mm, d0);
        step;
        check("5.3 读数失效: 仍在 AVOID",   state_dbg, S_AVOID);
        check("5.4 ★读数失效: speed=SLOW（限速）", speed_scale_dbg, SPD_SLOW);
        check("5.5 ★读数失效: m_en=1（不冒进但也没停死）", m_en, 1);

        //------------------------------------------------------------------
        // 6. 恢复：距离变远 -> 回到 AUTO 满速
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 恢复 ---\n");
        u_tof.range_mm = 16'd1234;
        v0 = valid_cnt;
        wait_frames(1, 5);
        check("6.1 恢复 -> dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        check("6.2 dist_mm == 1234",     last_dist, 1234);
        step;
        check("6.3 1234mm: state=AUTO",  state_dbg, S_AUTO);
        check("6.4 1234mm: speed=FULL",  speed_scale_dbg, SPD_FULL);
        check("6.5 1234mm: m_en=1",      m_en, 1);

        //------------------------------------------------------------------
        // 7. 周期性：采样不是"只出一次"
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 周期采样 ---\n");
        v0 = valid_cnt;
        wait_ms(4);                       // SAMPLE_MS=1，4ms 内至少应有 2 帧
        check("7.1 4ms 内至少两帧", ((valid_cnt - v0) >= 2) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n=== tb_top_tof_i2c: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 超时保护（不要写成超过 32 位的字面量）
    //------------------------------------------------------------------
    initial begin
        #50_000_000;                     // 50ms
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
