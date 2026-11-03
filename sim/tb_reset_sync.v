//=============================================================================
// tb : tb_reset_sync
//-----------------------------------------------------------------------------
// 目的: 验证 hal/reset_sync.v 的行为
//   1) 异步 assert : arst_n 一拉低, rst_n 立即(不等时钟)跟着拉低
//   2) POR 生效    : 上电后额外保持 POR_CYCLES 拍
//   3) 同步 release: 释放后要等 SYNC_STAGES 拍 rst_n 才拉高
//
// 测试写法说明（这里踩过两个坑，都值得记下来）
//------------------------------------------------------------------------------
// 坑1: 一开始把"同步释放延迟"和"POR 计数"混在一起测。
//      结果 POR 把 rst_n 一路拉低，同步链的延迟根本没被观测到。
//
// 坑2: 后来用同一个 DUT 反复测，测得"延迟 = 16 拍"而不是 3 拍。
//      原因是 reset_sync 里的 POR 计数器**每次 arst_n 拉低都会重启**
//      （por_done 被清零）——这是正确且必要的行为（按键复位要重新计时）。
//      所以只要中途再按一次复位，就必须重新等 POR_CYCLES 拍。
//      把这件事和"同步链延迟"混在一次测量里，必然测不准。
//
// 解决办法: 例化**两个** DUT
//      dut_por  : POR_CYCLES=64   -> 专门测 POR 行为
//      dut_sync : POR_CYCLES=0    -> por_done 恒为 1, 纯粹测同步链延迟
//   这样两个特性各自独立、互不干扰，测量结果是确定的。
//
// 运行: vsim -c -do "do run_sim.do tb_reset_sync"
// 结果: reset_sync_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_reset_sync;

    localparam integer CLK_PERIOD  = 10;
    localparam integer SYNC_STAGES = 3;
    localparam integer POR_CYCLES  = 64;

    reg  clk = 0;
    reg  arst_n = 0;

    wire rst_n_por;      // 带 POR 的那个
    wire rst_n_sync;     // POR 关闭，只测同步链

    integer errors = 0;
    integer checks = 0;

    // ---- 结果同时写文件 ----
    integer log_fd;
    initial begin
        log_fd = $fopen("reset_sync_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 reset_sync_result.txt");
    end

    integer cyc;
    integer idx;

    always #(CLK_PERIOD/2) clk = ~clk;

    //---- DUT 1: 带 POR，测 POR 行为 ----
    reset_sync #(
        .SYNC_STAGES (SYNC_STAGES),
        .POR_CYCLES  (POR_CYCLES)
    ) dut_por (
        .clk    (clk),
        .arst_n (arst_n),
        .rst_n  (rst_n_por)
    );

    //---- DUT 2: POR 关闭，纯测同步链 ----
    reset_sync #(
        .SYNC_STAGES (SYNC_STAGES),
        .POR_CYCLES  (0)
    ) dut_sync (
        .clk    (clk),
        .arst_n (arst_n),
        .rst_n  (rst_n_sync)
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

    initial begin
        $display("=== tb_reset_sync ===");
        $fwrite(log_fd, "=== tb_reset_sync ===\n");
        $display("  [info] CLK_PERIOD=%0d SYNC_STAGES=%0d POR_CYCLES=%0d",
                 CLK_PERIOD, SYNC_STAGES, POR_CYCLES);
        $fwrite(log_fd, "  [info] CLK_PERIOD=%0d SYNC_STAGES=%0d POR_CYCLES=%0d\n",
                CLK_PERIOD, SYNC_STAGES, POR_CYCLES);

        //==================================================================
        // A) 异步 assert（两个 DUT 都测）
        //==================================================================
        #25;                              // 落在时钟半周期中间
        arst_n = 1'b1;
        #3;
        check("A1. async assert: rst_n_por 立即为低",  rst_n_por,  0);
        check("A2. async assert: rst_n_sync 立即为低", rst_n_sync, 0);

        //==================================================================
        // B) 同步释放延迟精确测量（用 dut_sync，POR 已关闭）
        //==================================================================
        // 此刻 arst_n=1，dut_sync 的 por_done 恒为 1，同步链正在推进。
        // 从"当前时刻"开始数拍，rst_n_sync 应在 SYNC_STAGES 拍内拉高。
        // 为了从干净的起点测量，先重新按一次复位。
        arst_n = 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check("B1. 按复位: rst_n_sync 为低", rst_n_sync, 0);

        //------------------------------------------------------------------
        // 精确验证同步释放延迟
        //
        // 换成**无歧义的判据**：不再"数到它变高"（那个数法受 NBA/active 区
        // 时序影响，容易差一拍），而是直接验证两个确定的事实：
        //     第 SYNC_STAGES-1 拍：必须仍为低
        //     第 SYNC_STAGES   拍：必须已为高
        // 这样"延迟恰好等于 SYNC_STAGES"被完整锁定，且不受测量技巧影响。
        //
        // 同时把前几拍的实际电平打进日志，便于万一不符合时定位。
        //------------------------------------------------------------------
        arst_n = 1'b1;                    // 释放，从现在开始数时钟沿
        for (idx = 1; idx <= SYNC_STAGES + 2; idx = idx + 1) begin
            @(posedge clk);
            #1;                           // 等 NBA 更新生效再读
            $fwrite(log_fd, "    释放后第 %0d 拍: rst_n_sync=%0d\n", idx, rst_n_sync);
            if (idx == SYNC_STAGES - 1) begin
                check("B2. 第 SYNC_STAGES-1 拍仍为低", rst_n_sync, 0);
            end
            if (idx == SYNC_STAGES) begin
                check("B3. 第 SYNC_STAGES 拍已为高", rst_n_sync, 1);
            end
        end

        //==================================================================
        // C) POR 行为（用 dut_por）
        //    por_done 在 arst_n 拉低时被清零，所以释放后要重新等 POR_CYCLES 拍
        //==================================================================
        // 此刻 arst_n 已释放一段时间，dut_por 的 POR 也该走完了
        repeat (POR_CYCLES + SYNC_STAGES + 8) @(posedge clk);
        #1;
        check("C1. POR 走完: rst_n_por 为高", rst_n_por, 1);

        // 再按一次复位 -> POR 重新计时
        arst_n = 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check("C2. 再按复位: rst_n_por 为低", rst_n_por, 0);

        arst_n = 1'b1;
        // POR 中途应该保持低
        repeat (POR_CYCLES / 2) @(posedge clk);
        #1;
        check("C3. POR 中途: rst_n_por 仍为低", rst_n_por, 0);

        // POR 结束后应该拉高
        repeat (POR_CYCLES / 2 + SYNC_STAGES + 8) @(posedge clk);
        #1;
        check("C4. POR 结束: rst_n_por 为高", rst_n_por, 1);

        //==================================================================
        // D) 复位可重复触发
        //==================================================================
        repeat (5) @(posedge clk);
        arst_n = 1'b0;
        #2;
        check("D1. 复位可重复: rst_n_sync 为低", rst_n_sync, 0);

        arst_n = 1'b1;
        repeat (SYNC_STAGES + 4) @(posedge clk);
        #1;
        check("D2. 再次释放: rst_n_sync 为高", rst_n_sync, 1);

        //------------------------------------------------------------------
        $display("=== tb_reset_sync: %0d checks, %0d errors ===", checks, errors);
        $fwrite(log_fd, "=== tb_reset_sync: %0d checks, %0d errors ===\n", checks, errors);
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
        #1000000;
        $display("RESULT: FAIL (timeout)");
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
