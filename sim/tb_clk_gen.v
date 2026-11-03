//=============================================================================
// tb : tb_clk_gen
//-----------------------------------------------------------------------------
// 验证 rtl/hal/clk_gen.v 的**旁路模式**（BYPASS=1）。
//
//-----------------------------------------------------------------------------
// 为什么只测旁路模式
//-----------------------------------------------------------------------------
//   BYPASS=0 会例化 Gowin 的 `PLL` 原语 —— 那需要挂 gw5a 仿真库。
//   但现在**还没填 PLL**（见 clk_gen.v 的说明），所以测不了。
//   等 PLL 生成并填好之后，在这里加第二个 DUT 即可：
//
//       clk_gen #(.BYPASS(0), ...) dut_pll (...);
//       // 用 #delay 检查 clk_sys 的周期接近 10ns
//
//   本 tb 现在能确认的是：
//     1) 旁路时 clk_sys 就是 clk_in，一个周期都不差
//     2) 旁路时 locked 恒为 1（否则上层复位不释放，板子什么都不动）
//     3) BYPASS=0 但没填 PLL 时，locked 必须是 0
//        —— 这是刻意的"安全兜底"：宁可板子不动，
//           也不要静默地跑在 50MHz 而以为在跑 100MHz
//
//-----------------------------------------------------------------------------
// 结果: clk_gen_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_clk_gen;

    //------------------------------------------------------------------
    // 时钟周期
    //------------------------------------------------------------------
    localparam integer CLK_PERIOD = 20;      // 50MHz
    localparam integer HALF       = CLK_PERIOD / 2;

    //------------------------------------------------------------------
    // DUT 1：旁路模式（默认）
    //------------------------------------------------------------------
    reg  clk_in = 0;
    reg  arst_n = 0;

    wire clk_bypass;
    wire lock_bypass;

    //------------------------------------------------------------------
    // DUT 2：PLL 模式但未填（安全兜底）
    //------------------------------------------------------------------
    wire clk_unfilled;
    wire lock_unfilled;

    //------------------------------------------------------------------
    // 时钟产生
    //------------------------------------------------------------------
    always #(HALF) clk_in = ~clk_in;

    //------------------------------------------------------------------
    // 例化
    //------------------------------------------------------------------
    clk_gen #(
        .BYPASS     (1),
        .CLK_IN_HZ  (50_000_000),
        .CLK_OUT_HZ (50_000_000)
    ) dut_bypass (
        .clk_in  (clk_in),
        .arst_n  (arst_n),
        .clk_sys (clk_bypass),
        .locked  (lock_bypass)
    );

    clk_gen #(
        .BYPASS     (0),
        .CLK_IN_HZ  (50_000_000),
        .CLK_OUT_HZ (100_000_000)
    ) dut_unfilled (
        .clk_in  (clk_in),
        .arst_n  (arst_n),
        .clk_sys (clk_unfilled),
        .locked  (lock_unfilled)
    );

    //------------------------------------------------------------------
    // 检查与日志
    //------------------------------------------------------------------
    integer checks = 0;
    integer errors = 0;
    integer log_fd;
    integer i;

    initial begin
        log_fd = $fopen("clk_gen_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 clk_gen_result.txt");
    end

    task check;
        input [255:0] name;
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

    //------------------------------------------------------------------
    // 测时钟周期的辅助变量
    //
    // [!]️[!]️ 这里**故意不写成 task** —— 这个坑值得记一笔：
    //
    //   写成
    //       task measure_period;
    //           input clk;
    //           begin
    //               @(posedge clk);      // ← 永远等不到！
    //               ...
    //           end
    //       endtask
    //   会**永久挂起**。原因：Verilog 的 task `input` 参数是**值拷贝**
    //   （一个静态局部变量），不是对被传信号的连续引用。
    //   任务内部 `@(posedge clk)` 等的是那个被冻结的副本，
    //   它永远不变，所以事件永远不触发。
    //
    //   症状：仿真停在任务调用处不动，最后撞到 tb 的超时保护，
    //         结果文件里只有一项 FAIL (timeout)，看不出是哪一行。
    //
    //   正确做法：**直接在 initial 块里对真实信号写 @(posedge xxx)**，
    //             或者在任务外把时钟选好。
    //   （第 116 行那次还叠了一个错：task 端口写成 `input wire clk;`
    //     会让 vlog 报 near "wire" syntax error，且级联到文件末尾。）
    //------------------------------------------------------------------
    integer t0, t1, meas_period;

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        $fwrite(log_fd, "=== tb_clk_gen ===\n");
        $fwrite(log_fd, "  [info] 只测旁路模式；PLL 未填，见 clk_gen.v 说明\n");

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位期间 ---\n");
        arst_n = 1'b0;
        repeat (5) @(posedge clk_in);

        // 旁路模式下 locked 与 arst_n 无关，恒为 1
        // （locked 是"时钟可用"的指示，不是"复位已释放"）
        check("1.1 旁路: 复位期间 locked 仍为 1", lock_bypass, 1);

        arst_n = 1'b1;
        repeat (5) @(posedge clk_in);

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 旁路：clk_sys 必须就是 clk_in ---\n");
        check("2.1 旁路: clk_sys === clk_in (电平 1)", clk_bypass === clk_in, 1);

        // 连续比对 20 拍，确保不是碰巧
        i = 0;
        while (i < 20) begin
            @(posedge clk_in);
            #1;
            if (clk_bypass !== clk_in) i = 999;
            i = i + 1;
        end
        check("2.2 旁路: 连续 20 拍都跟随", i, 20);

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 旁路：周期就是 20ns ---\n");
        // 对**真实信号**直接等上升沿（见上面那段说明，不要包进 task）
        @(posedge clk_bypass); t0 = $time;
        @(posedge clk_bypass); t1 = $time;
        meas_period = t1 - t0;
        $fwrite(log_fd, "  [info] clk_sys 实测周期 = %0d ns (期望 %0d)\n",
                meas_period, CLK_PERIOD);
        check("3.1 clk_sys 周期 = 20ns", meas_period, CLK_PERIOD);

        @(posedge clk_in); t0 = $time;
        @(posedge clk_in); t1 = $time;
        meas_period = t1 - t0;
        check("3.2 clk_in  周期 = 20ns", meas_period, CLK_PERIOD);

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. * 未填 PLL 时的安全兜底 ---\n");
        $fwrite(log_fd, "  [info] BYPASS=0 但 g_pll 分支是空的\n");

        check("4.1 未填 PLL: locked 必须为 0", lock_unfilled, 0);
        $fwrite(log_fd, "  [info] locked=0 -> top 的 reset_sync 不会释放复位\n");
        $fwrite(log_fd, "  [info] 现象：上电后 led[0] 不亮，一眼看出时钟没起来\n");

        // clk_sys 仍然跟随 clk_in —— 这样即使没填，也能用示波器量到波形，
        // 方便区分"PLL 没配"和"板子没供电"
        check("4.2 未填 PLL: clk_sys 仍跟随 clk_in", clk_unfilled === clk_in, 1);

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 旁路模式下长时间运行 ---\n");
        repeat (200) @(posedge clk_in);
        check("5.1 200 拍后 locked 仍为 1", lock_bypass, 1);
        check("5.2 200 拍后 clk_sys 仍跟随", clk_bypass === clk_in, 1);

        //--------------------------------------------------------------
        $fwrite(log_fd, "\n=== tb_clk_gen: %0d checks, %0d errors ===\n",
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
        #10_000_000;                 // 10ms 足够（本 tb 只需几百拍）
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
