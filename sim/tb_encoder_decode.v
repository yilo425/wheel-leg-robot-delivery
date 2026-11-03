//=============================================================================
// tb : tb_encoder_decode
//-----------------------------------------------------------------------------
// 自检内容:
//   1) 正转/反转 4 倍频计数是否正确、符号是否正确
//   2) dir 方向锁存
//   3) 定窗测速: speed_cnt 是否等于窗口内累计步数, 符号是否与方向一致
//   4) 输入滤波: 宽度小于 FILTER_CYCLES 的毛刺不得计入
//   5) 静止时速度必须为 0
//
// 采样约定: 所有对 DUT 输出的读取统一走 sample() —— 等时钟沿后 #1 再读,
//           避免与 DUT 内非阻塞赋值产生仿真竞争。
// 滤波约定: STEP_CYCLES 必须 > FILTER_CYCLES, 否则单相脉冲会被滤波吃掉,
//           这是本模块的正常行为(每相必须稳定足够久才算有效跳变)。
//
// 运行: vlog -work work ../rtl/driver/encoder_decode.v tb_encoder_decode.v
//       vsim -c -do "run -all; quit -f" work.tb_encoder_decode
//=============================================================================
`timescale 1ns/1ps

module tb_encoder_decode;

    localparam integer CLK_PERIOD    = 10;       // 10ns -> 100MHz
    localparam integer FILTER_CYCLES = 8;
    localparam integer WINDOW        = 1000;     // 测速窗口 1000 clk (缩短仿真)
    localparam integer POS_WIDTH     = 32;
    localparam integer SPEED_WIDTH   = 24;
    localparam integer STEP_CYCLES   = 20;       // 每相保持 20 clk (> FILTER_CYCLES)

    reg  clk = 0;
    reg  rst_n = 0;
    reg  enc_a = 0, enc_b = 0;
    wire signed [POS_WIDTH-1:0]   pos_cnt;
    wire signed [SPEED_WIDTH-1:0] speed_cnt;
    wire                          dir;
    wire                          speed_valid;
    wire [31:0]                   window_cnt;

    integer errors = 0;
    integer checks = 0;


    // ---- 结果同时写文件 ----
    //   ModelSim -c 模式下 $display 走 stdout，管道常抓不到，
    //   用 $fwrite 直接落盘最可靠。结果文件: encoder_decode_result.txt
    integer log_fd;
    initial begin
        log_fd = $fopen("encoder_decode_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 encoder_decode_result.txt");
    end
    always #(CLK_PERIOD/2) clk = ~clk;

    encoder_decode #(
        .FILTER_CYCLES       (FILTER_CYCLES),
        .SPEED_WINDOW_CYCLES (WINDOW),
        .POS_WIDTH           (POS_WIDTH),
        .SPEED_WIDTH         (SPEED_WIDTH)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .enc_a       (enc_a),
        .enc_b       (enc_b),
        .pos_cnt     (pos_cnt),
        .speed_cnt   (speed_cnt),
        .dir         (dir),
        .speed_valid (speed_valid),
        .window_cnt  (window_cnt)
    );

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
    // 同步采样
    //------------------------------------------------------------------
    integer s_pos, s_spd, s_win, s_dir;

    task sample;
        begin
            @(posedge clk);
            #1;
            s_pos = pos_cnt;
            s_spd = speed_cnt;
            s_win = window_cnt;
            s_dir = dir;
        end
    endtask

    //------------------------------------------------------------------
    // 驱动一个相位并保持 STEP_CYCLES; 只在时钟沿后改电平
    //------------------------------------------------------------------
    task drive_phase;
        input a_val;
        input b_val;
        begin
            @(posedge clk);
            #1;
            enc_a = a_val;
            enc_b = b_val;
            repeat (STEP_CYCLES) @(posedge clk);
        end
    endtask

    // 前向序列  00 -> 01 -> 11 -> 10 -> 00
    task step_fwd;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                drive_phase(1'b0, 1'b1);
                drive_phase(1'b1, 1'b1);
                drive_phase(1'b1, 1'b0);
                drive_phase(1'b0, 1'b0);
            end
        end
    endtask

    // 后向序列  00 -> 10 -> 11 -> 01 -> 00
    task step_rev;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                drive_phase(1'b1, 1'b0);
                drive_phase(1'b1, 1'b1);
                drive_phase(1'b0, 1'b1);
                drive_phase(1'b0, 1'b0);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 等速度窗口刷新后采样
    //
    // 踩过的坑：直接等"第一个" speed_valid，那个窗口可能只累积了一半
    //   （电机刚起转 / 编码器刚接上），量出来的速度偏小
    //   （实测 speed=14，而稳定值应该 ~50）。
    //   所以这里先丢掉 skip 个脉冲，等窗口进入稳态再采。
    //------------------------------------------------------------------
    task get_speed;
        input integer skip;      // 丢弃前 skip 个脉冲
        integer k;
        begin
            for (k = 0; k < skip; k = k + 1) begin
                @(posedge speed_valid);
            end
            @(posedge speed_valid);
            sample;
        end
    endtask

    // 估算一个窗口内应有的计数量（用于合理性检查）
    //   一个正交整周期 = 4 * STEP_CYCLES 个 clk，产生 4 个计数
    //   所以每 clk 约 4/(4*STEP_CYCLES) = 1/STEP_CYCLES 个计数
    localparam integer EXPECT_CNT = WINDOW / STEP_CYCLES;

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer pos_a;

    initial begin
        $display("=== tb_encoder_decode ===");

        rst_n = 0;
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (20) @(posedge clk);

        // --- 1) 静止: 位置为 0, 一个窗口后速度为 0 ---
        get_speed(0);
        check("idle: speed_count", s_spd, 0);
        check("idle: window",      s_win, 0);
        check("idle: pos",         s_pos, 0);

        // --- 2) 正转 10 个整周期 = 40 个 4 倍频计数 ---
        sample;
        pos_a = s_pos;
        step_fwd(10);
        repeat (WINDOW + 50) @(posedge clk);   // 等滤波延迟清空
        sample;
        check("fwd: pos delta", s_pos - pos_a, 40);
        check("fwd: dir",       s_dir, 1);

        // --- 3) 正转期间测速（关键：测量必须和运动同时进行）---
        //
        // 踩过的坑（重点）：
        //   原先写法是 step_fwd(600) 跑完**之后**才 get_speed()，
        //   但那时编码器已经停了，速度窗口里是空的 —— 量出来必然是 0。
        //   这不是参数问题，是测试结构问题：测速必须在编码器运动中采样。
        //
        //   正确做法：用 fork 让"驱动编码器"和"等速度窗口"两个分支同时跑。
        //   get_speed(1) 在 fork 里等窗口，train 分支持续给脉冲。
        //
        //   一个正交整周期 = 4 * STEP_CYCLES clk，产生 4 个计数，
        //   所以窗口内计数 ≈ WINDOW * 4 / (4*STEP_CYCLES) = WINDOW/STEP_CYCLES。
        fork
            step_fwd(600);                // 分支 A: 持续正转
            begin                         // 分支 B: 运动期间采样
                get_speed(1);             // 丢掉第一个脉冲，等窗口进入稳态
                $display("  [info] fwd speed_count=%0d window=%0d (预期约 %0d)",
                         s_spd, s_win, EXPECT_CNT);
                $fwrite(log_fd, "  [info] fwd speed_count=%0d window=%0d (预期约 %0d)\n",
                        s_spd, s_win, EXPECT_CNT);
                check("fwd: speed == window", s_spd, s_win);
                check("fwd: speed positive",  (s_spd > 0) ? 1 : 0, 1);
                // 合理性: 应接近 EXPECT_CNT, 允许 ±40% 的窗口对齐误差
                check("fwd: speed sanity",
                      (s_spd > EXPECT_CNT * 6 / 10 && s_spd < EXPECT_CNT * 14 / 10) ? 1 : 0, 1);
            end
        join

        // 停下来, 等窗口彻底清零
        //
        // 注意 window_cnt 的语义：它是"上一个完整窗口的计数"，会一直**保持**，
        // 直到下一个窗口结束才更新。所以停车后不能只等一个窗口就采样，
        // 必须等足够长时间让那个"空窗口"走完。等 2 个窗口最稳。
        repeat (WINDOW * 2 + 50) @(posedge clk);
        sample;
        check("stop 后: window 清零", s_win, 0);
        check("stop 后: speed 清零",  s_spd, 0);

        // --- 4) 反转 10 个整周期 ---
        sample;
        pos_a = s_pos;
        step_rev(10);
        repeat (WINDOW + 50) @(posedge clk);
        sample;
        check("rev: pos delta", s_pos - pos_a, -40);
        check("rev: dir",       s_dir, 0);

        // --- 5) 反转期间测速（同样要 fork）---
        fork
            step_rev(600);                // 分支 A: 持续反转
            begin                         // 分支 B: 运动期间采样
                get_speed(1);
                $display("  [info] rev speed_count=%0d window=%0d", s_spd, s_win);
                $fwrite(log_fd, "  [info] rev speed_count=%0d window=%0d\n", s_spd, s_win);
                check("rev: speed negative",   (s_spd < 0) ? 1 : 0, 1);
                // window_cnt 本身就是**有符号的净计数**（不是幅值），
                // 所以反转时它和 speed_cnt 应该相等，而不是互为相反数。
                // （原先写成 s_spd == -s_win，是断言错误：实测 -48 vs -48 被判成不等。）
                check("rev: speed == window", s_spd, s_win);
                check("rev: speed sanity",
                      (s_spd < -EXPECT_CNT * 6 / 10 && s_spd > -EXPECT_CNT * 14 / 10) ? 1 : 0, 1);
            end
        join

        repeat (WINDOW + 50) @(posedge clk);

        // --- 6) 毛刺抑制: 宽度 2 clk 的假边沿 (< FILTER_CYCLES=8) 不得计入 ---
        sample;
        pos_a = s_pos;
        enc_a = 1'b1;
        repeat (2) @(posedge clk);
        enc_a = 1'b0;
        repeat (50) @(posedge clk);
        sample;
        check("glitch: pos unchanged", s_pos - pos_a, 0);

        // --- 7) 真边沿仍要能通过滤波 ---
        sample;
        pos_a = s_pos;
        step_fwd(1);
        repeat (WINDOW + 50) @(posedge clk);
        sample;
        check("after glitch: pos", s_pos - pos_a, 4);

        //------------------------------------------------------------------
        $display("=== tb_encoder_decode: %0d checks, %0d errors ===", checks, errors);
        $fwrite(log_fd, "=== tb_encoder_decode: %0d checks, %0d errors ===\n", checks, errors);
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
        #50000000;
        $display("RESULT: FAIL (timeout)");
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
