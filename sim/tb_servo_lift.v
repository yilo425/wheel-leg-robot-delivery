//=============================================================================
// tb_servo_lift -- servo_lift 模块自检
//-----------------------------------------------------------------------------
// 验证四件事：
//
//   1) 舵机 PWM 周期正确（默认 20ms）
//   2) 放下/抬起两个位置的脉宽正确（0.5ms / 2.5ms）
//   3) 限位开关**极性转换正确** —— 低有效输入 -> 高有效输出
//      ★ 这条最重要：修的就是"上拉导致一进 LIFT 就误判到位"那个 bug
//   4) 消抖真的起作用（短于消抖窗口的抖动不被采信）
//
//-----------------------------------------------------------------------------
// 时间尺度说明
//-----------------------------------------------------------------------------
//   舵机 PWM 周期是 20ms，而仿真时钟是 100MHz（10ns/拍）——
//   一个周期 200 万拍。等完整周期太慢，所以本 tb 把
//   PWM_PERIOD_US 缩到 100us、PWM_MIN_US=20、PWM_MAX_US=80，
//   比例关系不变，但跑得快几个数量级。
//
//   消抖用 DEBOUNCE_MS=1（1ms = 100000 拍 @100MHz）。
//
//   这里验证的是**逻辑正确性**（计数器比较、极性、消抖），
//   不是绝对时间。真实时间常数在 top.v 的参数里，一眼可查。
//=============================================================================

`timescale 1ns / 1ps

module tb_servo_lift;

    //------------------------------------------------------------------
    // 参数（缩小时间尺度，加快仿真）
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ   = 100_000_000;
    localparam integer CLK_PERIOD    = 10;              // ns
    localparam integer CYC_PER_US    = CLK_FREQ_HZ / 1_000_000;   // 100
    localparam integer PERIOD_US     = 100;
    localparam integer MIN_US        = 20;
    localparam integer MAX_US        = 80;
    localparam integer DEBOUNCE_MS   = 1;

    localparam integer PERIOD_CYC    = PERIOD_US * CYC_PER_US;   // 10000
    localparam integer MIN_CYC       = MIN_US    * CYC_PER_US;   // 2000
    localparam integer MAX_CYC       = MAX_US    * CYC_PER_US;   // 8000
    localparam integer DEB_TICKS     = DEBOUNCE_MS * (CLK_FREQ_HZ / 1000);  // 100000

    //------------------------------------------------------------------
    // DUT 信号
    //------------------------------------------------------------------
    reg         clk = 0;
    reg         rst_n = 0;
    reg  [1:0]  lift_cmd = 2'b00;
    reg  [7:0]  lift_height = 8'd100;   // 抬腿高度（0..100），100 = 旧行为
    reg  [1:0]  lift_lim_n = 2'b11;      // 低有效：默认未按下
    reg  [1:0]  lift_fault_n = 2'b11;    // 低有效：默认无故障

    wire [1:0]  lift_pwm;
    wire [1:0]  lift_done;
    wire [1:0]  lift_fault;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer i;

    initial begin
        log_fd = $fopen("servo_lift_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 servo_lift_result.txt");
    end

    always #(CLK_PERIOD/2) clk = ~clk;

    //==================================================================
    // DUT
    //==================================================================
    servo_lift #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .PWM_PERIOD_US (PERIOD_US),
        .PWM_MIN_US    (MIN_US),
        .PWM_MAX_US    (MAX_US),
        .DEBOUNCE_MS   (DEBOUNCE_MS)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .lift_cmd     (lift_cmd),
        .lift_height  (lift_height),
        .lift_lim_n   (lift_lim_n),
        .lift_fault_n (lift_fault_n),
        .lift_pwm     (lift_pwm),
        .lift_done    (lift_done),
        .lift_fault   (lift_fault)
    );

    //==================================================================
    // 工具
    //==================================================================
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

    // 等 n 个时钟
    task wait_cyc;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) step;
        end
    endtask

    // 等 n 毫秒
    task wait_ms;
        input integer n;
        begin
            wait_cyc(n * (CLK_FREQ_HZ / 1000));
        end
    endtask

    //------------------------------------------------------------------
    // 在 lift_pwm[ch] 上测一个完整周期的高电平拍数
    //
    //   ⚠️ 必须从**高电平的起点**开始测。
    //      如果调用时信号正好处于高电平中段，会测到半截脉冲
    //      （第一次写就踩了这个坑：测出 1899 而不是 2000，
    //        正好差一个"从哪里开始"的量）。
    //
    //   做法：先等一段完整的高+低，回到边沿上再测。
    //------------------------------------------------------------------
    integer meas_hi;
    task measure_pwm_hi;
        input integer ch;
        begin
            // 先等当前高电平结束（如果正在高电平中段）
            while (lift_pwm[ch] === 1'b1) step;
            // 再等下一个高电平开始
            while (lift_pwm[ch] !== 1'b1) step;
            // 现在正好在上升沿上，开始数
            meas_hi = 0;
            while (lift_pwm[ch] === 1'b1) begin
                step; meas_hi = meas_hi + 1;
            end
        end
    endtask

    // 数一个完整周期（从上升沿到下一个上升沿）
    integer per_cnt;
    task count_period;
        input integer ch;
        begin
            while (lift_pwm[ch] === 1'b1) step;
            while (lift_pwm[ch] !== 1'b1) step;   // 上升沿
            per_cnt = 0;
            // 先走过高电平
            while (lift_pwm[ch] === 1'b1) begin step; per_cnt = per_cnt + 1; end
            // 再走过低电平，到下一个上升沿
            while (lift_pwm[ch] !== 1'b1) begin step; per_cnt = per_cnt + 1; end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer hi_low, hi_high;

    initial begin
        $fwrite(log_fd, "=== tb_servo_lift: servo_lift 模块自检 ===\n");
        $fwrite(log_fd, "  参数: PERIOD=%0dus MIN=%0dus MAX=%0dus DEBOUNCE=%0dms\n",
                PERIOD_US, MIN_US, MAX_US, DEBOUNCE_MS);
        $fwrite(log_fd, "  换算: PERIOD=%0d拍 MIN=%0d拍 MAX=%0d拍 消抖=%0d拍\n",
                PERIOD_CYC, MIN_CYC, MAX_CYC, DEB_TICKS);

        //------------------------------------------------------------------
        // 复位
        //------------------------------------------------------------------
        rst_n = 1'b0;
        wait_cyc(20);
        $fwrite(log_fd, "\n--- 1. 复位 ---\n");
        rst_n = 1'b1;
        step;
        check("1.1 复位后 lift_done=0（未到位）",  lift_done, 2'b00);
        check("1.2 复位后 lift_fault=0（无故障）", lift_fault, 2'b00);

        //------------------------------------------------------------------
        // 2. 舵机 PWM —— 放下位置（短脉宽）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 舵机 PWM: 放下位置（短脉宽）---\n");
        lift_cmd = 2'b00;
        wait_cyc(100);
        measure_pwm_hi(0);
        hi_low = meas_hi;
        $fwrite(log_fd, "  [info] 放下时高电平 = %0d 拍（期望 %0d）\n", hi_low, MIN_CYC);
        check("2.1 放下前腿: 脉宽 = PWM_MIN",
              (hi_low >= MIN_CYC - 2 && hi_low <= MIN_CYC + 2) ? 1 : 0, 1);

        // 周期也要对：从上升沿数到下一个上升沿
        count_period(0);
        $fwrite(log_fd, "  [info] 周期 = %0d 拍（期望 %0d）\n", per_cnt, PERIOD_CYC);
        check("2.2 PWM 周期正确",
              (per_cnt >= PERIOD_CYC - 2 && per_cnt <= PERIOD_CYC + 2) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 3. 舵机 PWM —— 抬起位置（长脉宽）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 舵机 PWM: 抬起位置（长脉宽）---\n");
        lift_cmd = 2'b11;
        wait_cyc(100);
        measure_pwm_hi(0);
        hi_high = meas_hi;
        $fwrite(log_fd, "  [info] 抬起时高电平 = %0d 拍（期望 %0d）\n", hi_high, MAX_CYC);
        check("3.1 抬起前腿: 脉宽 = PWM_MAX",
              (hi_high >= MAX_CYC - 2 && hi_high <= MAX_CYC + 2) ? 1 : 0, 1);
        check("3.2 抬起脉宽 > 放下脉宽", (hi_high > hi_low) ? 1 : 0, 1);

        // 两路独立：只抬前腿时，后腿应该还是短脉宽
        lift_cmd = 2'b01;
        wait_cyc(100);
        measure_pwm_hi(1);
        check("3.3 只抬前腿时后腿仍是放下（短脉宽）",
              (meas_hi >= MIN_CYC - 2 && meas_hi <= MIN_CYC + 2) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 4. ★ 限位开关极性转换（修的就是这个 bug）
        //
        //   原始输入是**低有效**（常开触点 + 上拉）：
        //       未按下 -> 引脚读 1  -> 应该输出 done=0
        //       按下   -> 引脚读 0  -> 应该输出 done=1
        //
        //   ⚠️ 如果这里反了，上板就是"腿没抬就以为到位了"。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. ★限位开关极性（低有效 -> 高有效）---\n");
        lift_cmd = 2'b00;
        lift_lim_n = 2'b11;                  // 未按下
        wait_ms(DEBOUNCE_MS + 1);
        check("4.1 未按下(引脚=1): lift_done=0",      lift_done, 2'b00);

        lift_lim_n = 2'b00;                  // 两组都按下
        wait_ms(DEBOUNCE_MS + 1);
        check("4.2 按下(引脚=0): lift_done=2'b11",    lift_done, 2'b11);

        // ⚠️ 位序：bit0 = 前腿，bit1 = 后腿。
        //    "只按前腿" = 前腿引脚拉低 = bit0=0、bit1 保持 1 -> 2'b10
        lift_lim_n = 2'b10;
        wait_ms(DEBOUNCE_MS + 1);
        check("4.3 只按前腿: lift_done=2'b01",        lift_done, 2'b01);

        lift_lim_n = 2'b11;                  // 松开
        wait_ms(DEBOUNCE_MS + 1);
        check("4.4 松开: lift_done 回 0",             lift_done, 2'b00);

        //------------------------------------------------------------------
        // 5. ★ 消抖：短于消抖窗口的抖动不能被采信
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. ★消抖：抖动不被采信 ---\n");
        // 先确认稳定为 0
        lift_lim_n = 2'b11;
        wait_ms(DEBOUNCE_MS + 1);
        check("5.1 稳定未按下: lift_done=0",          lift_done, 2'b00);

        // 抖一下：拉起半个消抖窗口，然后恢复，反复几次
        begin : deb_chk
            integer r;
            for (r = 0; r < 5; r = r + 1) begin
                lift_lim_n = 2'b00;
                wait_cyc(DEB_TICKS / 4);          // 只有消抖窗口的 1/4
                lift_lim_n = 2'b11;
                wait_cyc(DEB_TICKS / 4);
            end
        end
        check("5.2 ★抖动后仍为 0（没被误采信）",     lift_done, 2'b00);

        // 真的按下并且保持够久 -> 应该被采信
        lift_lim_n = 2'b11;
        wait_ms(DEBOUNCE_MS + 1);
        check("5.3 抖完稳定: 仍是 0",                 lift_done, 2'b00);

        lift_lim_n = 2'b00;
        wait_ms(DEBOUNCE_MS + 2);
        check("5.4 持续按下够久: 采信为 1",           lift_done, 2'b11);

        //------------------------------------------------------------------
        // 6. 故障输入极性（同样低有效，不接=悬空靠上拉=1=正常）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 故障输入（低有效，不接=正常）---\n");
        lift_fault_n = 2'b11;                // 不接 / 正常
        wait_ms(DEBOUNCE_MS + 1);
        check("6.1 不接(上拉=1): lift_fault=0（无故障）", lift_fault, 2'b00);

        // 位序同上：bit0=前腿。报后腿故障 = bit1 拉低 = 2'b01
        lift_fault_n = 2'b01;
        wait_ms(DEBOUNCE_MS + 1);
        check("6.2 后腿故障(引脚=0): lift_fault=2'b10",   lift_fault, 2'b10);

        lift_fault_n = 2'b11;
        wait_ms(DEBOUNCE_MS + 1);
        check("6.3 故障消除: lift_fault 回 0",            lift_fault, 2'b00);

        //------------------------------------------------------------------
        // 7. 复位能立刻清掉状态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 复位清状态 ---\n");
        lift_lim_n = 2'b00;
        wait_ms(DEBOUNCE_MS + 2);
        check("7.1 复位前: lift_done=2'b11",          lift_done, 2'b11);
        rst_n = 1'b0;
        step;
        check("7.2 复位后立刻: lift_done=0",          lift_done, 2'b00);
        rst_n = 1'b1;
        step;

        //------------------------------------------------------------------
        // 8. ★ 抬腿高度（CONFIG 帧 0x10，2026-10-18 新增）
        //
        //    脉宽 = MIN + height*(MAX-MIN)/100，所以：
        //      height=100 -> MAX_US   = 80us   （和旧行为完全一致）
        //      height=50  -> 20+30     = 50us
        //      height=0   -> MIN_US    = 20us
        //    ⚠️ 高度只影响"抬起"时的脉宽；放下永远是 MIN_US。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 抬腿高度（0x10）---\n");
        lift_height = 8'd100;
        lift_cmd    = 2'b11;          // 两腿都抬起
        wait_ms(DEBOUNCE_MS + 1);
        measure_pwm_hi(0);
        check("8.1 height=100 -> 脉宽=MAX_US",      meas_hi, MAX_CYC);

        lift_height = 8'd50;
        wait_ms(2);
        measure_pwm_hi(0);
        check("8.2 height=50 -> 脉宽=中值",         meas_hi, MIN_CYC + (MAX_CYC-MIN_CYC)/2);

        lift_height = 8'd0;
        wait_ms(2);
        measure_pwm_hi(0);
        check("8.3 height=0 -> 脉宽=MIN_US",        meas_hi, MIN_CYC);

        // 超范围钳位：>100 按 100 处理（防御性）
        lift_height = 8'd200;
        wait_ms(2);
        measure_pwm_hi(0);
        check("8.4 height=200 钳到 100",            meas_hi, MAX_CYC);

        // 放下时永远是 MIN_US，与高度无关
        lift_height = 8'd50;
        lift_cmd    = 2'b00;
        wait_ms(2);
        measure_pwm_hi(0);
        check("8.5 放下时脉宽与高度无关（=MIN）",   meas_hi, MIN_CYC);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_servo_lift: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_servo_lift: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    // 超时保护
    initial begin
        #(500_000_000);
        $fwrite(log_fd, "\n[FATAL] 仿真超时\n");
        $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
