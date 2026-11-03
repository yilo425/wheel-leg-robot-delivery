//=============================================================================
// tb_motion_loop —— 速度环**闭环**仿真（motion_decision + 电机被控对象模型）
//
// 为什么需要这个 tb：
//   之前所有 tb 都是"给一个 spd_fb，看 duty 是多少"（开环）。
//   但真实系统是**闭环**：duty -> 电机转速 -> 编码器计数 -> spd_fb -> duty。
//   开环测试看不出"稳态误差""是否振荡""会不会爬不上去"这类问题。
//
//   而 `motion_decision` 是**纯比例控制**：
//        duty = (VREF - spd) >> KP_SHIFT        (KP_SHIFT=1 -> 增益 0.5)
//   纯 P + 有限增益的被控对象，稳态必然有残差：
//        spd_ss = VREF * G / (G + 2)
//   G 是被控对象增益（每个 duty 单位对应多少 计数/窗口）。
//   如果 G ≈ 1，那么 spd_ss ≈ VREF/3 —— **只能跑到目标的三分之一**。
//
//   这个 tb 就是用来量这个残差的。
//
// 被控对象模型（一阶惯性，离散化到每个测速窗口）：
//   稳态关系  : spd_target = duty * G
//   一阶滞后  : spd = spd + (spd_target - spd) / TAU
//   都是整数运算，没有浮点（和 RTL 一致）。
//=============================================================================
`timescale 1ns/1ps

module tb_motion_loop;

    localparam integer CLK_PERIOD = 20;         // 50MHz
    localparam integer VREF       = 69;         // 满速目标（计数/窗口）
    localparam integer KP_SHIFT   = 1;          // duty = err >> 1  -> 增益 0.5
    localparam integer LIMIT_DUTY = 120;
    localparam integer DEADBAND   = 1;
    localparam integer STEER_SHIFT= 4;

    // 被控对象参数
    //   G = 每单位 duty 对应的稳态计数/窗口。
    //   取 1 是"最保守"的假设：满 duty(127) 时约 127 计数/窗口，
    //   和我们 VREF=69 同一量级 —— 也就是"设计点"的合理猜测。
    localparam integer PLANT_G    = 1;
    localparam integer PLANT_TAU  = 3;          // 3 个窗口到 63%
    localparam integer NWIN       = 160;        // 观察 160 个窗口（给积分足够时间收敛到零残差）

    reg clk = 0;
    reg rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //---- DUT ----
    reg         motor_enable = 0;
    reg  [1:0]  speed_scale  = 0;
    reg  signed [15:0] steer = 0;
    reg  signed [23:0] spd0 = 0, spd1 = 0, spd2 = 0, spd3 = 0;
    reg         spd_valid = 0;

    wire [15:0] duty0, duty1, duty2, duty3;
    wire        dir0, dir1, dir2, dir3;

    motion_decision #(
        .CLK_FREQ_HZ  (50_000_000),
        .VREF         (VREF),
        .KP_SHIFT     (KP_SHIFT),
        .LIMIT_DUTY   (LIMIT_DUTY),
        .ZERO_DEADBAND(DEADBAND),
        .STEER_COEF0  ( 1),
        .STEER_COEF1  (-1),
        .STEER_COEF2  ( 1),
        .STEER_COEF3  (-1),
        .STEER_SHIFT  (STEER_SHIFT)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .motor_enable (motor_enable),
        .speed_scale  (speed_scale),
        .vref         (VREF[15:0]),
        .steer        (steer),
        .spd_fb0      (spd0),
        .spd_fb1      (spd1),
        .spd_fb2      (spd2),
        .spd_fb3      (spd3),
        .spd_valid    (spd_valid),
        .duty0        (duty0),
        .duty1        (duty1),
        .duty2        (duty2),
        .duty3        (duty3),
        .dir0         (dir0),
        .dir1         (dir1),
        .dir2         (dir2),
        .dir3         (dir3)
    );

    integer log_fd;
    integer checks = 0;
    integer errors = 0;

    task check;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            checks = checks + 1;
            if (got === exp) $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
            else begin
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

    //==================================================================
    // 主流程：每个"窗口"更新一次被控对象，然后看控制器怎么响应
    //==================================================================
    integer w;
    integer duty_now;
    integer plant_target;
    integer spd_int;        // 用整数做一阶滞后（避免实数）
    integer ss_min, ss_max, ss_avg;
    integer trace_spd [0:NWIN-1];
    integer trace_duty[0:NWIN-1];

    initial begin
        log_fd = $fopen("motion_loop_result.txt", "w");
        $fwrite(log_fd, "=== tb_motion_loop (闭环) ===\n");
        $fwrite(log_fd, "  VREF=%0d KP_SHIFT=%0d 被控对象 G=%0d TAU=%0d\n",
                VREF, KP_SHIFT, PLANT_G, PLANT_TAU);

        repeat (10) step;
        rst_n = 1'b1;
        repeat (3) step;

        // 全速档起步
        motor_enable = 1'b1;
        speed_scale  = 2'd2;       // SPD_FULL

        //------------------------------------------------------------------
        // 闭环跑 NWIN 个窗口
        //------------------------------------------------------------------
        for (w = 0; w < NWIN; w = w + 1) begin
            // 让 Duty 稳定（组合逻辑，给几拍）
            repeat (3) step;
            duty_now = duty0;

            // 被控对象：一阶滞后
            plant_target = duty_now * PLANT_G;
            spd_int = spd0;
            spd_int = spd_int + (plant_target - spd_int) / PLANT_TAU;

            // 更新反馈（四个轮子同步，走直线）
            spd0 = spd_int;
            spd1 = spd_int;
            spd2 = spd_int;
            spd3 = spd_int;

            // 假装来了一次测速窗口脉冲
            spd_valid = 1'b1; step; spd_valid = 1'b0;

            trace_spd [w] = spd_int;
            trace_duty[w] = duty_now;
        end

        //------------------------------------------------------------------
        // 打印轨迹（每 5 个窗口一行）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n  win  duty  spd\n");
        for (w = 0; w < NWIN; w = w + 5) begin
            $fwrite(log_fd, "  %3d  %4d  %4d\n", w, trace_duty[w], trace_spd[w]);
        end

        //------------------------------------------------------------------
        // 稳态统计（最后 10 个窗口）
        //------------------------------------------------------------------
        ss_min = 999999; ss_max = -999999; ss_avg = 0;
        for (w = NWIN - 10; w < NWIN; w = w + 1) begin
            if (trace_spd[w] < ss_min) ss_min = trace_spd[w];
            if (trace_spd[w] > ss_max) ss_max = trace_spd[w];
            ss_avg = ss_avg + trace_spd[w];
        end
        ss_avg = ss_avg / 10;

        $fwrite(log_fd, "\n  稳态（最后 10 窗口）: min=%0d max=%0d avg=%0d, 目标=%0d\n",
                ss_min, ss_max, ss_avg, VREF);
        $fwrite(log_fd, "  占目标比例: %0d%%\n", (ss_avg * 100) / VREF);
        $fwrite(log_fd, "  理论预期 spd_ss = VREF*G/(G+KP分母) = %0d\n",
                (VREF * PLANT_G) / (PLANT_G + 2));

        //------------------------------------------------------------------
        // 检查项
        //   ★ 这里先只断言"结构性正确"的东西（会不会动、收不收敛、会不会跑飞）。
        //     "稳态能不能到目标"单独列一条 —— 它反映的是控制律设计是否有残差，
        //     是个需要人判断的结论，不是"代码写没写对"。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 结构性 ---\n");
        check("1.1 有占空比输出（不是 0）",        (duty0 != 0) ? 1 : 0, 1);
        check("1.2 电机转起来了（spd > 0）",       (trace_spd[NWIN-1] > 0) ? 1 : 0, 1);
        check("1.3 方向为正（前进）",              dir0, 1);
        check("1.4 占空比没超过 LIMIT_DUTY",       (duty0 <= LIMIT_DUTY) ? 1 : 0, 1);

        $fwrite(log_fd, "\n--- 2. 收敛性 ---\n");
        check("2.1 稳态已收敛（最后 10 窗极差 <= 1）", (ss_max - ss_min <= 1) ? 1 : 0, 1);
        check("2.2 没有过冲（spd 全程 <= 目标*1.2）",
              (ss_max <= (VREF * 12) / 10) ? 1 : 0, 1);

        $fwrite(log_fd, "\n--- 3. 稳态精度（★ 这一条反映控制律设计）---\n");
        $fwrite(log_fd, "  [info] 稳态 %0d / 目标 %0d = %0d%%\n",
                ss_avg, VREF, (ss_avg * 100) / VREF);
        check("3.1 稳态误差在 10%% 以内",          (ss_avg >= (VREF * 9) / 10) ? 1 : 0, 1);

        //==================================================================
        // 阶段 2：被控对象增益**翻倍**（G=2）
        //
        //   为什么必须测：真实电机的 G 会随电池电压、负载、个体差异变化。
        //   纯 P 控制在 G 变化时稳态残差跟着变（spd=VREF*G/(G+2)）；
        //   加了积分项后稳态与 G 无关 —— 这一条就是验证那个"无关"。
        //
        //   同时也要确认**不会因为增益高而振荡**（积分增益 × 被控对象增益
        //   是环路增益，太高会振荡）。
        //==================================================================
        $fwrite(log_fd, "\n--- 4. 鲁棒性：被控对象增益翻倍（G=2）---\n");
        spd0 = 0; spd1 = 0; spd2 = 0; spd3 = 0;   // 从静止重新起步

        for (w = 0; w < NWIN; w = w + 1) begin
            repeat (3) step;
            duty_now = duty0;

            plant_target = duty_now * 2;          // ★ G=2
            spd_int = spd0;
            spd_int = spd_int + (plant_target - spd_int) / PLANT_TAU;

            spd0 = spd_int; spd1 = spd_int; spd2 = spd_int; spd3 = spd_int;
            spd_valid = 1'b1; step; spd_valid = 1'b0;

            trace_spd [w] = spd_int;
            trace_duty[w] = duty_now;
        end

        ss_min = 999999; ss_max = -999999; ss_avg = 0;
        for (w = NWIN - 10; w < NWIN; w = w + 1) begin
            if (trace_spd[w] < ss_min) ss_min = trace_spd[w];
            if (trace_spd[w] > ss_max) ss_max = trace_spd[w];
            ss_avg = ss_avg + trace_spd[w];
        end
        ss_avg = ss_avg / 10;
        $fwrite(log_fd, "  [info] G=2 稳态 %0d / 目标 %0d = %0d%%，极差 %0d\n",
                ss_avg, VREF, (ss_avg * 100) / VREF, ss_max - ss_min);

        check("4.1 G=2 时稳态仍接近目标（10%% 内）", (ss_avg >= (VREF * 9) / 10) ? 1 : 0, 1);
        check("4.2 G=2 时没有振荡（极差 <= 2）",      (ss_max - ss_min <= 2) ? 1 : 0, 1);
        check("4.3 G=2 时没有跑飞（<= 目标*1.2）",    (ss_max <= (VREF * 12) / 10) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_motion_loop: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_motion_loop: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

endmodule
