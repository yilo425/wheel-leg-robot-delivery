//=============================================================================
// tb : tb_imu_driver
//-----------------------------------------------------------------------------
// 拓扑: imu_driver(DUT) <--I2C--> mpu6050_slave_model（本 tb 里的 MPU6050 从机）
//
// 验证内容：
//   1) 复位后总线释放、姿态输出清零
//   2) 正常一帧：初始化表写进去了（0x6B/0x1C/0x1B/0x19 逐条抽查）、
//      12 字节原始值读回来了、roll/pitch/yaw **和手算的期望值一致**
//   3) imu_valid / imu_err 都是**单拍脉冲**（不会连续两拍高）
//   4) 从机 NACK -> imu_err，且**不刷新**姿态输出
//   5) 数据不合理（加速度三轴全 0）-> imu_err，且不刷新姿态输出
//   6) I2C 协议质量：复合读用了**重复 START**、地址没错过、
//      ★SCL 高电平期间 SDA 没变过（hold_err=0）
//   7) 采样周期正确；复位能清状态
//
//-----------------------------------------------------------------------------
// 时间尺度（为什么参数和真实值不一样）
//-----------------------------------------------------------------------------
//   真实配置是 400kHz / SAMPLE=20ms / TIMEOUT=100ms。
//   仿真里改成 SCL=1MHz / SAMPLE=TIMEOUT=BOOT=1ms：
//   分频、事务数量、超时与采样的相对关系完全一样，只是快几十倍。
//   这里验证的是**逻辑**，绝对时间常数在 imu_driver/top.v 的参数里。
//
//-----------------------------------------------------------------------------
// ★ 期望值是怎么手算的（不借 DUT 的任何中间量）
//-----------------------------------------------------------------------------
//   本 tb 里 SAMPLE_MS = 1，代进 imu_driver 的常量公式：
//
//     加速度：ACC_GAIN = (5729 * 1024) / 16384 = 358      （ACC_SHIFT = 10）
//             roll  = ( ay_raw * 358) >>> 10
//             pitch = (-ax_raw * 358) >>> 10
//
//     陀螺：  GYRO_GAIN = (100 * SAMPLE_MS * 2^16) / (131 * 1000)
//                       = (100 * 1 * 65536) / 131000 = 50
//             Δyaw = ( gz_raw * 50 + 32768 ) >>> 16      （+32768 = 四舍五入）
//
//   各相位的期望值：
//     A) ay=+8192, ax=0, gz=+1310
//          roll  = (8192*358)>>10 = 2864            (28.64°)
//          pitch = 0
//          Δyaw  = (1310*50+32768)>>16 = 98268>>16 = 1  (0.01°)  -> yaw=1
//     B) ay=+8192, ax=-8192, gz=0
//          roll  = 2864（不变）
//          pitch = (8192*358)>>10 = 2864            (28.64°)
//          Δyaw  = (0*50+32768)>>16 = 0             -> yaw 保持 1
//     C) ay=-8192, ax=0, gz=0
//          roll  = -2864（测符号）
//     D) 加速度三轴全 0 -> imu_err，三个输出都保持不变
//     E) 从机 NACK       -> imu_err，三个输出都保持不变
//
// 运行: vsim -c -voptargs=+acc -L work -L gw5a work.tb_imu_driver
// 结果: imu_driver_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_imu_driver;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ  = 50_000_000;
    localparam integer CLK_PERIOD   = 20;                 // 20ns = 50MHz
    localparam integer MS_CYC       = CLK_FREQ_HZ / 1000; // 1ms = 50000 拍

    localparam integer SCL_HZ       = 1_000_000;   // 仿真加速：1MHz
    localparam integer SAMPLE_MS    = 1;
    localparam integer TIMEOUT_MS   = 1;
    localparam integer I2C_TMO_MS   = 1;
    localparam integer BOOT_WAIT_MS = 1;
    localparam [6:0]   DEV_ADDR     = 7'h68;       // MPU6050（AD0=0）

    // MPU6050 寄存器
    localparam [7:0] REG_SMPLRT_DIV   = 8'h19;
    localparam [7:0] REG_GYRO_CONFIG  = 8'h1B;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;

    //------------------------------------------------------------------
    // 手算的期望值（见文件头；改 imu_driver 的换算必须同步改这里）
    //------------------------------------------------------------------
    localparam integer EXP_ROLL_A  =  2864;   // (8192*358)>>10
    localparam integer EXP_YAW_A   =     1;   // (1310*50+32768)>>16
    localparam integer EXP_PITCH_B =  2864;   // (8192*358)>>10
    localparam integer EXP_ROLL_C  = -2864;   // (-8192*358)>>10

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk   = 0;
    reg rst_n = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // I2C 总线
    //   上拉由从机模型内部提供（pullup 原语），这里不再重复加，
    //   否则仿真是弱驱动叠加，虽然不会报错但语义混乱。
    //------------------------------------------------------------------
    wire scl;
    wire sda;

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    wire signed [15:0] roll;
    wire signed [15:0] pitch;
    wire signed [15:0] yaw;
    wire               imu_valid;
    wire               imu_err;

    imu_driver #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .SAMPLE_MS      (SAMPLE_MS),
        .TIMEOUT_MS     (TIMEOUT_MS),
        .I2C_SCL_HZ     (SCL_HZ),
        .I2C_TIMEOUT_MS (I2C_TMO_MS),
        .BOOT_WAIT_MS   (BOOT_WAIT_MS),
        .DEV_ADDR       (DEV_ADDR)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .roll      (roll),
        .pitch     (pitch),
        .yaw       (yaw),
        .imu_valid (imu_valid),
        .imu_err   (imu_err),
        .scl       (scl),
        .sda       (sda),
        // 本 tb 是单驱动独占总线：直接给授权，不接仲裁器
        .bus_grant (1'b1),
        .bus_req   (),
        .bus_busy  ()
    );

    //------------------------------------------------------------------
    // 从机模型（MPU6050）
    //------------------------------------------------------------------
    mpu6050_slave_model #(
        .DEV_ADDR (DEV_ADDR)
    ) u_imu (
        .scl (scl),
        .sda (sda)
    );

    //------------------------------------------------------------------
    // 结果文件 / 计数器
    //------------------------------------------------------------------
    integer log_fd;
    integer errors = 0;
    integer checks = 0;

    initial begin
        log_fd = $fopen("imu_driver_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 imu_driver_result.txt");
    end

    task check;
        // ⚠️ 名字端口必须是 [1023:0]（128 字节）。
        //    写成 [255:0] 只有 32 字节，中文一个字 3 字节，
        //    长一点的检查名会**从前面**被截掉，日志里就看不到 "2.5" 这种编号。
        input [1023:0] name;
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

    //==================================================================
    // 输出脉冲监视（验证"单拍脉冲" + 采样周期 + 抓取本次姿态）
    //
    //   ⚠️ 时机的细节（本项目在 tb_tof_driver 里踩过并写进了注释）：
    //      DUT 在 I_EVAL 那一拍用**非阻塞赋值**把
    //      roll/pitch/yaw/imu_valid 一起更新；本块在同一拍读到的是
    //      **上一拍的值**，所以它实际是在**下一个** posedge 看到
    //      imu_valid=1 —— 那时姿态输出已经是这一次的新值了。
    //      这样恰好一次、值也对。所以断言要盯**计数**，别直接采那根线。
    //==================================================================
    integer    valid_cnt    = 0;
    integer    err_cnt      = 0;
    integer    valid_double = 0;     // imu_valid 连续两拍 = 不是单拍脉冲
    integer    err_double   = 0;
    reg        prev_valid   = 0;
    reg        prev_err     = 0;
    integer    cyc          = 0;
    integer    last_valid_cyc = 0;
    integer    valid_period = 0;     // 两次 imu_valid 之间的拍数

    integer    roll_at_valid  = 0;   // 最近一次有效帧的姿态（供断言用）
    integer    pitch_at_valid = 0;
    integer    yaw_at_valid   = 0;

    always @(posedge clk) begin
        if (rst_n) begin
            cyc = cyc + 1;
            if (imu_valid) begin
                valid_cnt     = valid_cnt + 1;
                roll_at_valid  = $signed(roll);
                pitch_at_valid = $signed(pitch);
                yaw_at_valid   = $signed(yaw);
                if (prev_valid) valid_double = valid_double + 1;
                if (last_valid_cyc != 0) valid_period = cyc - last_valid_cyc;
                last_valid_cyc = cyc;
            end
            if (imu_err) begin
                err_cnt = err_cnt + 1;
                if (prev_err) err_double = err_double + 1;
            end
            prev_valid = imu_valid;
            prev_err   = imu_err;
        end
    end

    //==================================================================
    // 工具任务
    //==================================================================
    // 等下一个 imu_valid（带超时）
    task wait_valid;
        input integer max_cyc;
        integer v0;
        integer k;
        begin
            v0 = valid_cnt;
            k  = 0;
            while ((valid_cnt == v0) && (k < max_cyc)) begin
                @(posedge clk);
                #1;
                k = k + 1;
            end
        end
    endtask

    // 等下一个 imu_err（带超时）
    task wait_err;
        input integer max_cyc;
        integer e0;
        integer k;
        begin
            e0 = err_cnt;
            k  = 0;
            while ((err_cnt == e0) && (k < max_cyc)) begin
                @(posedge clk);
                #1;
                k = k + 1;
            end
        end
    endtask

    // 复位 + 清从机模型
    task do_reset;
        begin
            rst_n = 1'b0;
            u_imu.clear_bus;
            repeat (20) @(posedge clk);
            rst_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer v0;
    integer e0;
    integer held_roll;
    integer held_pitch;
    integer held_yaw;

    initial begin
        $fwrite(log_fd, "=== tb_imu_driver: MPU6050 驱动自检 ===\n");
        $fwrite(log_fd, "  参数: SCL=%0dHz SAMPLE=%0dms TIMEOUT=%0dms BOOT=%0dms ADDR=0x%02h\n",
                SCL_HZ, SAMPLE_MS, TIMEOUT_MS, BOOT_WAIT_MS, DEV_ADDR);
        $fwrite(log_fd, "  说明: 仿真里 SCL 1MHz / 三个 ms 参数 1ms 是为了缩短时间,\n");
        $fwrite(log_fd, "        比例关系与真实配置(400kHz/20ms/100ms)一致。\n");
        $fwrite(log_fd, "  期望值(手算): ACC_GAIN=(5729*1024)/16384=358,  GYRO_GAIN=(100*1*65536)/131000=50\n");

        // 注意：从机模型的 regs[] 由模型自己的 initial 清零，这里不用管。

        //------------------------------------------------------------------
        // 1. 复位状态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位 ---\n");
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        check("1.1 复位后 SCL 释放为高", (scl === 1'b1) ? 1 : 0, 1);
        check("1.2 复位后 SDA 释放为高", (sda === 1'b1) ? 1 : 0, 1);
        check("1.3 复位后 imu_valid=0", imu_valid, 0);
        check("1.4 复位后 imu_err=0",   imu_err,   0);
        check("1.5 复位后 roll=0",      $signed(roll),  0);
        check("1.6 复位后 pitch=0",     $signed(pitch), 0);
        check("1.7 复位后 yaw=0",       $signed(yaw),   0);
        rst_n = 1'b1;

        //------------------------------------------------------------------
        // 2. 正常一帧
        //    boot 1ms -> 写 4 条初始化 -> 等 1ms -> 12 笔单字节复合读 -> 输出
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 正常一帧（初始化 + 读 12 字节 + 换算）---\n");
        u_imu.ax =  16'sd0;
        u_imu.ay =  16'sd8192;      // +0.5g
        u_imu.az =  16'sd16384;     // +1g
        u_imu.gx =  16'sd0;
        u_imu.gy =  16'sd0;
        u_imu.gz =  16'sd1310;      // 10 °/s

        v0 = valid_cnt;
        wait_valid(8 * MS_CYC);

        check("2.1 收到 imu_valid 脉冲",      (valid_cnt > v0) ? 1 : 0, 1);
        check("2.2 ★roll  == 2864",           roll_at_valid,  EXP_ROLL_A);
        check("2.3 ★pitch == 0",              pitch_at_valid, 0);
        check("2.4 ★yaw   == 1 (陀螺积分1帧)", yaw_at_valid,   EXP_YAW_A);

        // 初始化表真的写出去了（逐条抽查，值要和 imu_driver 的 CFG_* 一致）
        check("2.5  init 写 0x6B = 0x00 (唤醒)",    u_imu.regs[REG_PWR_MGMT_1],   8'h00);
        check("2.6  init 写 0x1C = 0x00 (±2g)",     u_imu.regs[REG_ACCEL_CONFIG], 8'h00);
        check("2.7  init 写 0x1B = 0x00 (±250°/s)", u_imu.regs[REG_GYRO_CONFIG],  8'h00);
        check("2.8  init 写 0x19 = 0x09 (100Hz)",   u_imu.regs[REG_SMPLRT_DIV],   8'h09);
        check("2.9  init 一共写了 4 条寄存器",       u_imu.wr_reg_cnt, 4);

        // 真的去读了 12 个字节（6 加速度 + 6 陀螺）
        check("2.10 读过 0x3B(加速度起始)", (u_imu.ptr_3b_cnt > 0) ? 1 : 0, 1);
        check("2.11 读过 0x43(陀螺起始)",   (u_imu.ptr_43_cnt > 0) ? 1 : 0, 1);
        check("2.12 读方向移出 12 个数据字节", u_imu.rd_byte_cnt, 12);

        //------------------------------------------------------------------
        // 3. I2C 协议质量
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. I2C 协议 ---\n");
        check("3.1 出现过 START",         (u_imu.start_cnt  > 0) ? 1 : 0, 1);
        check("3.2 出现过 STOP",          (u_imu.stop_cnt   > 0) ? 1 : 0, 1);
        check("3.3 收到 地址+W",          (u_imu.addr_w_cnt > 0) ? 1 : 0, 1);
        check("3.4 收到 地址+R",          (u_imu.addr_r_cnt > 0) ? 1 : 0, 1);
        check("3.5 ★复合读用了重复START", (u_imu.rstart_cnt >= 12) ? 1 : 0, 1);
        check("3.6 地址没错过",           u_imu.addr_bad_cnt, 0);
        check("3.7 ★SCL 高时 SDA 稳定",   u_imu.hold_err, 0);

        //------------------------------------------------------------------
        // 4. 第二帧：pitch 符号 + yaw 不漂（陀螺给 0）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 第二帧（pitch 反号 + gz=0 时 yaw 不动）---\n");
        u_imu.ax = -16'sd8192;      // -0.5g -> pitch 应为 +2864
        u_imu.gz =  16'sd0;         // 角速度为 0 -> yaw 不推进

        v0 = valid_cnt;
        wait_valid(6 * MS_CYC);

        check("4.1 第二帧有效",          (valid_cnt > v0) ? 1 : 0, 1);
        check("4.2 ★pitch == +2864",     pitch_at_valid, EXP_PITCH_B);
        check("4.3 roll 仍为 2864",      roll_at_valid,  EXP_ROLL_A);
        check("4.4 ★gz=0 -> yaw 保持 1", yaw_at_valid,   EXP_YAW_A);

        //------------------------------------------------------------------
        // 5. 第三帧：roll 负方向（测符号约定）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 第三帧（roll 负方向）---\n");
        u_imu.ax =  16'sd0;
        u_imu.ay = -16'sd8192;      // -0.5g -> roll 应为 -2864

        v0 = valid_cnt;
        wait_valid(6 * MS_CYC);

        check("5.1 第三帧有效",       (valid_cnt > v0) ? 1 : 0, 1);
        check("5.2 ★roll == -2864",   roll_at_valid, EXP_ROLL_C);

        //------------------------------------------------------------------
        // 6. 数据不合理：加速度三轴全 0 -> imu_err，且姿态保持不变
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 数据不合理（加速度全 0）---\n");
        held_roll  = roll_at_valid;
        held_pitch = pitch_at_valid;
        held_yaw   = yaw_at_valid;

        u_imu.ax = 16'sd0;
        u_imu.ay = 16'sd0;
        u_imu.az = 16'sd0;          // 三轴全 0 = 器件没在出数

        e0 = err_cnt;
        wait_err(6 * MS_CYC);
        check("6.1 全 0 加速度 -> imu_err", (err_cnt > e0) ? 1 : 0, 1);
        check("6.2 无效帧不刷新 roll",  $signed(roll),  held_roll);
        check("6.3 无效帧不刷新 pitch", $signed(pitch), held_pitch);
        check("6.4 无效帧不刷新 yaw",   $signed(yaw),   held_yaw);

        //------------------------------------------------------------------
        // 7. 从机 NACK -> imu_err，且不刷新姿态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 从机 NACK ---\n");
        u_imu.az = 16'sd16384;      // 数据恢复正常，但下面从机不应答
        u_imu.no_ack_mode = 1;

        e0 = err_cnt;
        wait_err(6 * MS_CYC);
        check("7.1 NACK -> imu_err",   (err_cnt > e0) ? 1 : 0, 1);
        check("7.2 NACK 不刷新 roll",  $signed(roll),  held_roll);
        check("7.3 NACK 不刷新 pitch", $signed(pitch), held_pitch);
        check("7.4 NACK 不刷新 yaw",   $signed(yaw),   held_yaw);

        //------------------------------------------------------------------
        // 8. 恢复：从机重新应答 -> 又能出有效帧
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 恢复 ---\n");
        u_imu.no_ack_mode = 0;
        u_imu.ay = 16'sd8192;       // 回到相位 A 的数据
        u_imu.ax = 16'sd0;
        u_imu.gz = 16'sd0;          // yaw 不再推进

        v0 = valid_cnt;
        wait_valid(8 * MS_CYC);
        check("8.1 恢复后收到 imu_valid", (valid_cnt > v0) ? 1 : 0, 1);
        check("8.2 恢复后 roll == 2864",  roll_at_valid, EXP_ROLL_A);
        check("8.3 恢复后 pitch == 0",    pitch_at_valid, 0);

        //------------------------------------------------------------------
        // 9. 单拍脉冲 + 采样周期
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 9. 单拍脉冲 / 采样周期 ---\n");
        check("9.1 imu_valid 是单拍脉冲", valid_double, 0);
        check("9.2 imu_err   是单拍脉冲", err_double,   0);

        // ⚠️ 这里不能直接拿当前的 valid_period 断周期：
        //    valid_period 是"相邻两次 imu_valid 的间隔"，而第 7~8 段中间插了
        //    NACK 测试 —— 那段时间**没有**有效帧，所以此刻的 valid_period
        //    还横跨着那段空档（实测 > 4*SAMPLE_MS），拿它断言周期会误报。
        //    正确做法：先连等两帧，让 valid_period 落在稳定窗口里再断言。
        //    （2026-10-15：这条原本 FAIL，根因在这里，不是驱动的问题。）
        v0 = valid_cnt;
        wait_valid(8 * MS_CYC);        // 第一帧：把窗口重新对齐
        v0 = valid_cnt;
        wait_valid(8 * MS_CYC);        // 第二帧：此时 valid_period = 两个相邻帧的间隔
        check("9.3 采样间隔 >= SAMPLE_MS",      (valid_period >= MS_CYC) ? 1 : 0, 1);
        check("9.4 采样间隔 <= 4*SAMPLE_MS",    (valid_period <= (4 * MS_CYC)) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 10. 复位清状态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 10. 复位清状态 ---\n");
        rst_n = 1'b0;
        u_imu.clear_bus;
        repeat (20) @(posedge clk);
        check("10.1 复位期间 imu_valid=0", imu_valid, 0);
        check("10.2 复位期间 imu_err=0",   imu_err,   0);
        check("10.3 复位后 roll=0",        $signed(roll),  0);
        check("10.4 复位后 pitch=0",       $signed(pitch), 0);
        check("10.5 复位后 yaw=0",         $signed(yaw),   0);
        check("10.6 复位后总线释放", ((scl === 1'b1) && (sda === 1'b1)) ? 1 : 0, 1);
        rst_n = 1'b1;

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_imu_driver: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_imu_driver: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    //------------------------------------------------------------------
    // 仿真总超时（防止 DUT 卡住导致仿真永不结束）
    //   ⚠️ 字面量不要超过 32 位（5e9 这种会让 vlog 直接拒绝编译）
    //------------------------------------------------------------------
    initial begin
        #50_000_000;                     // 50ms
        $fwrite(log_fd, "\n[FATAL] 仿真超时（DUT 可能卡死）\n");
        $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
