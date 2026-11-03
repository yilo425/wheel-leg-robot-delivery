//=============================================================================
// tb : tb_motor_guard
//-----------------------------------------------------------------------------
// 目的: 验证"电机护栏"（软启动 + 堵转保护 + 冷却恢复）。
//
//   这个模块夹在 motion_decision 和 pwm_motor 之间，防两个**会烧硬件**的事：
//     1) 堵转：轮子卡住时闭环会一直加大占空比 -> TM6612 持续大电流 -> 烧管子/拖垮电池
//     2) 启动冲击：从 0 猛给满占空比 -> 启动电流是稳态的好几倍
//
//-----------------------------------------------------------------------------
// 时间尺度（为什么参数和真实值不一样）
//-----------------------------------------------------------------------------
//   真实配置：CTRL=1ms / STALL=500ms / COOLDOWN=1000ms。
//   仿真里把 STALL/COOLDOWN 缩到 3ms/5ms（比例关系一样，只是快 100 多倍），
//   否则一条 tb 要跑几秒钟仿真时间。
//
//   CLK_FREQ_HZ 仍然用真实的 50MHz —— 这样 CTRL_CYC 的换算
//   （1ms = 50000 拍）被真实地走了一遍。
//
//-----------------------------------------------------------------------------
// 怎么保证"每 1 个 CTRL 节拍"是可数的
//-----------------------------------------------------------------------------
//   tick 每 CTRL_CYC=50000 拍来一次，而 wait_ms(1) 正好也是 50000 拍。
//   所以先做一次"暖场" wait_ms(1) 对齐相位，之后**每次 wait_ms(1) 里恰好一个 tick**。
//   这样"每拍最多涨 RAMP_STEP"才能被逐拍断言（不是只看终值）。
//
// 结果: motor_guard_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_motor_guard;

    //------------------------------------------------------------------
    // 参数（与 DUT 保持一致）
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ   = 50_000_000;
    localparam integer CLK_PERIOD    = 20;              // 20ns = 50MHz
    localparam integer MS_TICKS      = CLK_FREQ_HZ / 1000;   // 1ms = 50000 拍

    localparam integer CTRL_MS       = 1;
    localparam integer RAMP_STEP     = 16;
    localparam integer STALL_DUTY    = 40;
    localparam integer STALL_SPD_MAX = 2;
    localparam integer STALL_MS      = 3;      // 仿真加速（真实 500）
    localparam integer COOLDOWN_MS   = 5;      // 仿真加速（真实 1000）

    localparam integer TARGET        = 100;    // 常用目标占空比

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg               enable = 1'b0;
    reg  [15:0]       duty0 = 16'd0, duty1 = 16'd0, duty2 = 16'd0, duty3 = 16'd0;
    reg  signed [23:0] spd0 = 24'sd0, spd1 = 24'sd0, spd2 = 24'sd0, spd3 = 24'sd0;
    reg               spd_valid = 1'b0;

    wire              enable_out;
    wire [15:0]       duty_safe0, duty_safe1, duty_safe2, duty_safe3;
    wire [3:0]        stall;
    wire              stall_any;
    wire              ramp_active;

    motor_guard #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .CTRL_MS       (CTRL_MS),
        .RAMP_STEP     (RAMP_STEP),
        .STALL_DUTY    (STALL_DUTY),
        .STALL_SPD_MAX (STALL_SPD_MAX),
        .STALL_MS      (STALL_MS),
        .COOLDOWN_MS   (COOLDOWN_MS)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .enable      (enable),
        .duty0       (duty0),
        .duty1       (duty1),
        .duty2       (duty2),
        .duty3       (duty3),
        .spd0        (spd0),
        .spd1        (spd1),
        .spd2        (spd2),
        .spd3        (spd3),
        .spd_valid   (spd_valid),
        .enable_out  (enable_out),
        .duty_safe0  (duty_safe0),
        .duty_safe1  (duty_safe1),
        .duty_safe2  (duty_safe2),
        .duty_safe3  (duty_safe3),
        .stall       (stall),
        .stall_any   (stall_any),
        .ramp_active (ramp_active)
    );

    //------------------------------------------------------------------
    // 结果文件 / 计数
    //------------------------------------------------------------------
    integer log_fd;
    integer errors = 0;
    integer checks = 0;
    integer prev   = 0;
    integer curr   = 0;

    initial begin
        log_fd = $fopen("motor_guard_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 motor_guard_result.txt");
    end

    //------------------------------------------------------------------
    // check：名字端口必须 [1023:0]（[255:0] 只有 32 字节，会从前面截掉编号）
    //------------------------------------------------------------------
    task check;
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

    //------------------------------------------------------------------
    // 基本时序工具
    //------------------------------------------------------------------
    task step;
        begin
            @(posedge clk);
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

    //------------------------------------------------------------------
    // 喂一次编码器速度（模拟"测速窗口更新"那一拍）
    //   ⚠️ DUT 只在 spd_valid 那拍锁存 spd -> 不 pulse 的话 spd_lat 一直是 0，
    //      会被判成"没转"（这是模块的保守设计，不是 bug）。
    //------------------------------------------------------------------
    task feed_spd;
        input integer s0;
        input integer s1;
        input integer s2;
        input integer s3;
        begin
            spd0 = s0[23:0];
            spd1 = s1[23:0];
            spd2 = s2[23:0];
            spd3 = s3[23:0];
            spd_valid = 1'b1;
            step;
            spd_valid = 1'b0;
        end
    endtask

    task set_all_duty;
        input integer d;
        begin
            duty0 = d[15:0];
            duty1 = d[15:0];
            duty2 = d[15:0];
            duty3 = d[15:0];
        end
    endtask

    //------------------------------------------------------------------
    // 等某个条件成立（带超时）
    //
    //   ⚠️ 为什么不能只靠"等固定毫秒"：
    //      堵转 -> 冷却 -> 重新爬 -> 又堵转 是个循环（周期约 COOLDOWN+STALL），
    //      固定等 9ms 可能正好落在"已经又断电"或"刚好恢复"的两种相位上，
    //      断言就会时好时坏。改成"等到 stall[0] 变 X 为止"，与相位无关。
    //------------------------------------------------------------------
    task wait_stall0;
        input integer want;         // 想等到的值（0 或 1）
        input integer max_ms;
        integer k;
        begin
            k = 0;
            while ((stall[0] !== want[0]) && (k < max_ms * MS_TICKS)) begin
                step;
                k = k + 1;
            end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        $fwrite(log_fd, "=== tb_motor_guard: 电机护栏（软启动 / 堵转 / 冷却）===\n");
        $fwrite(log_fd, "  参数: CTRL=%0dms RAMP_STEP=%0d STALL_DUTY=%0d STALL_SPD_MAX=%0d STALL=%0dms COOLDOWN=%0dms\n",
                CTRL_MS, RAMP_STEP, STALL_DUTY, STALL_SPD_MAX, STALL_MS, COOLDOWN_MS);
        $fwrite(log_fd, "  CLK=%0dHz  1ms=%0d 拍\n", CLK_FREQ_HZ, MS_TICKS);

        //------------------------------------------------------------------
        // 0. 复位
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 0. 复位 ---\n");
        rst_n = 1'b0;
        enable = 1'b0;
        repeat (20) step;
        check("0.1 复位后 duty_safe0=0", duty_safe0, 0);
        check("0.2 复位后 enable_out=0",  enable_out, 0);
        check("0.3 复位后 stall=0",       stall, 0);
        check("0.4 复位后 ramp_active=0", ramp_active, 0);

        rst_n = 1'b1;
        repeat (20) step;
        enable = 1'b1;
        feed_spd(0, 0, 0, 0);
        step;
        check("0.5 enable=1 后 enable_out=1（组合跟随）", enable_out, 1);
        check("0.6 目标为 0 时输出 0", duty_safe0, 0);
        check("0.7 目标为 0 时 ramp_active=0（已在位）", ramp_active, 0);

        //------------------------------------------------------------------
        // 1. 急停 0 拍（enable 组合直通，不等寄存器）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 急停 0 拍 ---\n");
        set_all_duty(TARGET);
        feed_spd(50, 50, 50, 50);        // 四轮都在转（避免触发堵转）
        wait_ms(3);                       // 让 duty 爬到 48 左右
        check("1.1 急停前 duty_safe0 非 0", (duty_safe0 > 0) ? 1 : 0, 1);

        enable = 1'b0;
        #1;                                // ⚠️ 只等组合延迟，**不推进时钟**
        check("1.2 ★enable=0 当拍 duty_safe0=0（0 拍）", duty_safe0, 0);
        check("1.3 ★enable=0 当拍 enable_out=0（0 拍）", enable_out, 0);
        check("1.4 其余三路也当拍为 0", duty_safe1 + duty_safe2 + duty_safe3, 0);

        repeat (3) step;                   // 保持断电几拍：内部斜坡应归零
        check("1.5 断电期间 duty 保持 0", duty_safe0, 0);

        enable = 1'b1;
        #1;
        check("1.6 ★重新使能从 0 开始（软启动，不回跳旧值）", duty_safe0, 0);

        //------------------------------------------------------------------
        // 2. 软启动（上升限速）与下降不限速
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 软启动 ---\n");
        feed_spd(50, 50, 50, 50);
        wait_ms(1);                        // 暖场：对齐 tick 相位
        prev = duty_safe0;
        wait_ms(1);
        curr = duty_safe0;
        check("2.1 ★第 1 拍只涨 RAMP_STEP", curr - prev, RAMP_STEP);
        prev = curr;
        wait_ms(1);
        curr = duty_safe0;
        check("2.2 ★第 2 拍仍只涨 RAMP_STEP", curr - prev, RAMP_STEP);
        check("2.3 ★没有一步跳到目标 100", (curr < TARGET) ? 1 : 0, 1);
        check("2.4 爬升中 ramp_active=1", ramp_active, 1);

        wait_ms(6);                        // 7 拍足够爬完 100/16
        check("2.5 爬到目标 100", duty_safe0, TARGET);
        check("2.6 到位后 ramp_active=0", ramp_active, 0);
        check("2.7 四路同步爬到目标", duty_safe1 + duty_safe2 + duty_safe3, 3 * TARGET);

        set_all_duty(10);
        wait_ms(1);
        check("2.8 ★目标降到 10 -> 1 拍内跟到 10（下降不限速）", duty_safe0, 10);

        //------------------------------------------------------------------
        // 3. 堵转判定（只有卡住的那一路断电，其它不受影响）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 堵转判定 ---\n");
        set_all_duty(TARGET);
        feed_spd(50, 50, 50, 50);
        wait_ms(8);
        check("3.0 四路都到 100", duty_safe0 + duty_safe1, 2 * TARGET);
        check("3.0b 到位后 ramp_active=0", ramp_active, 0);

        // 只有 0 号轮卡住（编码器 0），其余照常转
        feed_spd(0, 50, 50, 50);

        // 先证明"**不到** STALL_MS 不会判"（否则一停就断电，车根本动不了）
        wait_ms(1);
        check("3.1 ★不到 STALL_MS 不判堵转", stall[0], 0);

        // 再等够 STALL_MS
        wait_ms(STALL_MS + 1);
        check("3.2 ★持续 STALL_MS 后判出堵转", stall[0], 1);
        check("3.3 ★堵转轮 duty 立刻为 0", duty_safe0, 0);
        check("3.4 ★其它轮不误报（lane1 转着）", stall[1], 0);
        check("3.5 其它轮 duty 保持 100", duty_safe1, TARGET);
        check("3.6 只有 lane0 的 stall 置起", stall, 4'b0001);
        check("3.7 stall_any=1", stall_any, 1);

        //------------------------------------------------------------------
        // 4. 冷却 + 自动恢复 + 卡阻仍在则再次判堵转
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 冷却恢复 ---\n");
        wait_ms(1);
        check("4.1 冷却期间 duty 恒 0", duty_safe0, 0);
        check("4.2 冷却期间 stall 保持 1", stall[0], 1);

        // 等冷却自动结束（用"等条件"而不是等固定时间，避免和下一次卡阻赛跑）
        wait_stall0(1'b0, COOLDOWN_MS + 6);
        check("4.3 ★冷却结束 stall 自动清 0", stall[0], 0);

        wait_ms(2);
        check("4.4 冷却后重新爬升（duty>0）", (duty_safe0 > 0) ? 1 : 0, 1);

        // 卡阻仍在 -> 应能**再次**判出（断续试探），等它自己再次置起
        wait_stall0(1'b1, STALL_MS + COOLDOWN_MS + 10);
        check("4.5 ★卡阻仍在 -> 再次判堵转（断续试探）", stall[0], 1);
        check("4.6 这期间 lane1 一直没被误判", stall[1], 0);

        //------------------------------------------------------------------
        // 5. 正常转动不误报（含负速度：判据用的是绝对值）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 正常转动不误报 ---\n");
        feed_spd(0, 50, 50, -50);          // lane3 反向转：|−50| 也算转
        wait_ms(STALL_MS + 2);
        check("5.1 ★duty 大且转着 -> 不判堵转（lane1）", stall[1], 0);
        check("5.2 ★负速度也算转 -> 不判堵转（lane3）", stall[3], 0);
        check("5.3 lane1 duty 仍 100", duty_safe1, TARGET);

        //------------------------------------------------------------------
        // 6. duty 小不判堵转（低速小占空比本来就可能不转）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. duty 小不判堵转 ---\n");
        duty2 = 16'd10;                    // < STALL_DUTY(40)
        feed_spd(0, 50, 0, -50);           // lane2 编码器 0，但不该判堵转
        wait_ms(STALL_MS + 3);
        check("6.1 ★duty 小(10<40)+不动 -> 不判堵转", stall[2], 0);
        check("6.2 duty 保持 10", duty_safe2, 10);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n=== tb_motor_guard: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

    //------------------------------------------------------------------
    // 超时保护
    //   估算：整个 tb 约 55~60ms 仿真时间（3M 拍左右）。
    //   这里给 200ms，足够松，只有在真卡死时才会触发。
    //------------------------------------------------------------------
    initial begin
        #200_000_000;
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
