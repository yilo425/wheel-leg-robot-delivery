//=============================================================================
// tb : tb_pwm_motor
//-----------------------------------------------------------------------------
// 自检内容:
//   1) 载波周期是否等于 CLK_FREQ_HZ/PWM_FREQ_HZ
//   2) 高电平宽度 vs (duty * STEP_DIV)
//   3) 方向脚、使能门、零点死区、超量程钳位
//   4) duty_actual 回读是否与指令一致
//
// 测量方法: 在恰好 PERIOD_CNT 个 clk 的固定窗口内统计高电平拍数。
//           不用 "等边沿" 方式, 因此 0% / 100% 占空比也不会死等。
//
// 运行: vlog -work work ../rtl/driver/pwm_motor.v tb_pwm_motor.v
//       vsim -c -do "run -all; quit -f" work.tb_pwm_motor
//=============================================================================
`timescale 1ns/1ps

module tb_pwm_motor;

    // ---- 测试参数 (用小数值让仿真快) ----
    localparam integer CLK_FREQ_HZ = 10_000_000;   // 10MHz
    localparam integer PWM_FREQ_HZ = 100_000;      // 100kHz -> PERIOD_CNT=100
    localparam integer DUTY_GAIN   = 100;          // STEP_DIV = 100/100 = 1
    localparam integer DEADBAND    = 3;

    localparam integer PERIOD_CNT = CLK_FREQ_HZ / PWM_FREQ_HZ;  // 100
    localparam integer HALF_CLK   = 50;                         // 半周期 5us

    reg                   clk = 0;
    reg                   rst_n = 0;
    reg                   enabled = 0;
    reg  signed [15:0]    duty_in = 0;
    reg                   dir_in = 1'b1;
    wire                  pwm_out, dir_out;
    wire [15:0]           pwm_cnt;
    wire signed [15:0]    duty_actual;

    integer errors = 0;
    integer checks = 0;

    always #HALF_CLK clk = ~clk;   // 10MHz

    pwm_motor #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .PWM_FREQ_HZ   (PWM_FREQ_HZ),
        .DUTY_GAIN     (DUTY_GAIN),
        .CNT_WIDTH     (16),
        .ZERO_DEADBAND (DEADBAND)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .enabled     (enabled),
        .duty_in     (duty_in),
        .dir_in      (dir_in),
        .pwm_out     (pwm_out),
        .dir_out     (dir_out),
        .pwm_cnt     (pwm_cnt),
        .duty_actual (duty_actual)
    );

    //------------------------------------------------------------------
    // 结果同时写文件
    //
    // 为什么需要这个：
    //   ModelSim 批处理(-c)模式下, $display 的输出走 stdout,
    //   而 PowerShell 的 `*>` / 管道经常抓不到它（缓冲问题）。
    //   结果就是"仿真跑完了但看不到 PASS/FAIL"。
    //   用 $fwrite 直接写文件就完全不依赖 ModelSim 的输出行为。
    //
    //   汇总文件: pwm_motor_result.txt
    //------------------------------------------------------------------
    integer log_fd;

    initial begin
        log_fd = $fopen("pwm_motor_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 pwm_motor_result.txt");
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
    // 在恰好 PERIOD_CNT 个 clk 内统计高电平拍数
    // 先等 2 个完整周期让 cmp_r 更新生效
    //------------------------------------------------------------------
    integer high_cnt;
    integer i;

    task measure_high;
        begin
            // 对齐到载波周期起点: 等 pwm_cnt 回到 0 之后再多走 2 个周期
            @(posedge clk);
            while (pwm_cnt !== 16'd0) @(posedge clk);
            repeat (PERIOD_CNT * 2) @(posedge clk);

            high_cnt = 0;
            for (i = 0; i < PERIOD_CNT; i = i + 1) begin
                if (pwm_out === 1'b1) high_cnt = high_cnt + 1;
                @(posedge clk);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        $display("=== tb_pwm_motor ===");

        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // --- 1) 未使能: 必须无输出 ---
        enabled = 0;
        duty_in = 16'sd1000;             // 即使给了大指令也不许输出
        repeat (3) @(posedge clk);
        check("disabled: duty_actual", duty_actual, 0);
        measure_high;
        check("disabled: high cnt", high_cnt, 0);

        // --- 2) 50%: duty_in = PERIOD_CNT*GAIN/2 = 50, 方向由 dir_in 决定 ---
        enabled = 1;
        dir_in  = 1'b1;
        duty_in = 16'sd50;
        repeat (3) @(posedge clk);
        check("50%: dir_out=dir_in", dir_out, 1);
        check("50%: duty_actual",    duty_actual, 50);
        measure_high;
        check("50%: high cnt", high_cnt, 50);

        // --- 3) 25% ---
        duty_in = 16'sd25;
        repeat (3) @(posedge clk);
        measure_high;
        check("25%: high cnt", high_cnt, 25);

        // --- 4) 方向由 dir_in 翻转: 占空比不变, dir_out 变 0 ---
        dir_in  = 1'b0;
        duty_in = 16'sd25;
        repeat (3) @(posedge clk);
        check("rev: dir_out=dir_in", dir_out, 0);
        check("rev: duty_actual",    duty_actual, -25);
        measure_high;
        check("rev: high cnt (不变)", high_cnt, 25);

        // --- 5) 负 duty 仍按幅值处理(兼容旧用法), 方向仍看 dir_in ---
        dir_in  = 1'b1;
        duty_in = -16'sd25;
        repeat (3) @(posedge clk);
        check("neg duty: dir_out",   dir_out, 1);
        check("neg duty: duty_actual", duty_actual, 25);
        measure_high;
        check("neg duty: high cnt", high_cnt, 25);

        // --- 6) 零点死区: 死区内不换向 ---
        //   语义要测的是"保持"：先建立一个已知方向，再在死区内给相反方向，
        //   方向必须"保持"原值不变。
        //   踩过的坑：原先直接给 dir_in=0 就期望 dir_out=0，
        //   但那时 dir_r 保持的是上一步的 1（RTL 行为正确，是断言写错了）。
        duty_in = 16'sd25;                // 先离开死区，建立方向
        dir_in  = 1'b0;                   // 期望方向 = 0
        repeat (3) @(posedge clk);
        check("deadband 前置: dir_out", dir_out, 0);

        duty_in = 16'sd2;                 // 进入死区 (<= DEADBAND=3)
        dir_in  = 1'b1;                   // 故意给相反方向
        repeat (3) @(posedge clk);
        measure_high;
        check("deadband: high cnt", high_cnt, 0);
        check("deadband: dir 保持不换向", dir_out, 0);

        // --- 7) 死区边界: duty_in=4 应正常输出, 且方向此时才更新 ---
        duty_in = 16'sd4;
        repeat (3) @(posedge clk);
        measure_high;
        check("deadband edge: high cnt", high_cnt, 4);
        check("deadband edge: dir 更新", dir_out, 1);

        // --- 8) 超量程钳位: duty_in=20000 -> 满量程 100 ---
        //   注意方向：此时 dir_in 仍为上一步的 1（正转），
        //   所以 duty_actual 应为 +100，不是 -100。
        //   （原先期望写成了 -100，是断言错误 —— RTL 行为正确。）
        duty_in = 16'sd20000;
        repeat (3) @(posedge clk);
        check("clamp: dir_out", dir_out, 1);
        check("clamp: duty_actual", duty_actual, 100);
        measure_high;
        check("clamp: high cnt", high_cnt, PERIOD_CNT);

        // --- 9) 归零 ---
        duty_in = 16'sd0;
        repeat (3) @(posedge clk);
        measure_high;
        check("zero: high cnt", high_cnt, 0);

        //------------------------------------------------------------------
        $display("=== tb_pwm_motor: %0d checks, %0d errors ===", checks, errors);
        $fwrite(log_fd, "=== tb_pwm_motor: %0d checks, %0d errors ===\n", checks, errors);
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
        #50000000;                       // 50ms 仿真时间上限
        $display("RESULT: FAIL (timeout)");
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
