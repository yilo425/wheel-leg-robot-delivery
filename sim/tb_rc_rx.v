//=============================================================================
// tb : tb_rc_rx
//-----------------------------------------------------------------------------
// 验证 rtl/app/rc_rx.v：0x05 RC_CMD 帧解析 + 500ms 看门狗失效保护。
//
//-----------------------------------------------------------------------------
// 设计原则（沿用 tb_config_rx.v 已验证的约定）
//-----------------------------------------------------------------------------
// 1) 结果用 $fwrite 落盘 -> rc_rx_result.txt
// 2) 判据写死在 tb 里，不引用 DUT 内部 localparam
// 3) 不用 dut.xxx 层次引用（避免 -voptargs="+acc" 依赖）
// 4) 载荷缓冲在 tb 里复刻一份，用组合 assign 复刻
//       rx_pl_data = pl_buf[rx_pl_addr]
//    这样"地址 -> 数据"的组合时序能被真实测到（地址接错也测得出）
// 5) 每个等待都有固定拍数上限，不写"等到出现脉冲"
//
//-----------------------------------------------------------------------------
// 覆盖的行为
//-----------------------------------------------------------------------------
//   1) 复位值 .................................. 场景 1
//   2) 正常帧：steer=+50 speed=100 ............. 场景 2
//   3) steer 负值 -50 .......................... 场景 3
//   4) steer 饱和：+120 / -120 -> ±1600 ........ 场景 4
//   5) speed 分档：0 / 50 / 51 -> 0 / 1 / 2 .... 场景 5
//   6) 长度不对 -> rc_bad，且不改输出 .......... 场景 6
//   7) 非 0x05 帧 -> 完全不响应 ................ 场景 7
//   8) 序号不连续 -> rc_seq_err 累加 ........... 场景 8
//   9) 看门狗：超时后 rc_active 归 0 ........... 场景 9
//  10) 超时后再收到帧 -> rc_active 恢复 ........ 场景 10
//
//-----------------------------------------------------------------------------
// 时间
//-----------------------------------------------------------------------------
//   TIMEOUT_MS 在 tb 里覆盖成 2ms（真实值 500ms 仿不起）。
//   CLK 50MHz。总仿真时间约 10ms，跑起来几秒。
//=============================================================================
`timescale 1ns/1ps

module tb_rc_rx;

    localparam integer CLK_PERIOD  = 20;          // 50MHz
    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer TB_TIMEOUT_MS = 2;         // ★ 覆盖成 2ms，否则仿真太慢

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk;
    reg rst_n;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // DUT 端口
    //------------------------------------------------------------------
    reg         rx_valid;
    reg  [7:0]  rx_type;
    reg  [15:0] rx_len;

    wire [7:0]  rx_pl_addr;      // ★ DUT 的输出，tb 只读 -> 必须是 wire

    wire [7:0]  rx_pl_data;      // 由 tb 的组合 assign 驱动

    wire signed [15:0] rc_steer;
    wire [1:0]         rc_speed;
    wire               rc_active;
    wire [15:0]        rc_frames;
    wire               rc_bad;
    wire [7:0]         rc_seq_err;

    //------------------------------------------------------------------
    // 载荷缓冲（复刻 uart_frame 的 assign rx_pl_data = buf[addr]）
    //------------------------------------------------------------------
    reg [7:0] pl_buf [0:7];
    assign rx_pl_data = pl_buf[rx_pl_addr[2:0]];

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    rc_rx #(
        .CLK_FREQ_HZ       (CLK_FREQ_HZ),
        .TIMEOUT_MS        (TB_TIMEOUT_MS),
        .TYPE_RC_CMD       (8'h05),
        .RC_PL_LEN         (3),
        .STEER_GAIN_SHIFT  (4),
        .SPEED_SLOW_MAX    (50)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .rx_valid    (rx_valid),
        .rx_type     (rx_type),
        .rx_len      (rx_len),
        .rx_pl_data  (rx_pl_data),
        .rx_pl_addr  (rx_pl_addr),
        .rc_steer    (rc_steer),
        .rc_speed    (rc_speed),
        .rc_active   (rc_active),
        .rc_frames   (rc_frames),
        .rc_bad      (rc_bad),
        .rc_seq_err  (rc_seq_err)
    );

    //------------------------------------------------------------------
    // 统计 / 日志
    //------------------------------------------------------------------
    integer checks;
    integer errors;
    integer log_fd;

    task check;
        input [1023:0] name;
        input integer  got;
        input integer  exp;
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
    // 发一帧 RC_CMD
    //   s8   : steer (-100..100)
    //   spd  : speed (0..100)
    //   seq  : 序号
    //   len  : 帧长度字段（正常给 3，异常场景故意给错）
    //   type : 帧类型（正常 0x05）
    //------------------------------------------------------------------
    task send_rc;
        input integer s8;
        input integer spd;
        input integer seq;
        input integer len;
        input [7:0]   type;
        begin
            pl_buf[0] = s8[7:0];
            pl_buf[1] = spd[7:0];
            pl_buf[2] = seq[7:0];

            @(posedge clk);
            rx_type  <= type;
            rx_len   <= len[15:0];
            rx_valid <= 1'b1;
            @(posedge clk);
            rx_valid <= 1'b0;
            // 状态机要 3 拍走完 B0->B1->B2，多给几拍余量
            repeat (8) @(posedge clk);
            // rx_bad 是单拍脉冲，这里用过程变量接不住，改为由调用者按需查
        end
    endtask

    //------------------------------------------------------------------
    // 发 "非法长度" 帧，并顺便抓 rc_bad 单拍脉冲
    //------------------------------------------------------------------
    reg bad_seen;
    always @(posedge clk) begin
        if (rc_bad) bad_seen <= 1'b1;
    end

    task do_reset;
        begin
            rst_n    = 1'b0;
            rx_valid = 1'b0;
            rx_type  = 8'd0;
            rx_len   = 16'd0;
            pl_buf[0] = 8'd0; pl_buf[1] = 8'd0; pl_buf[2] = 8'd0;
            pl_buf[3] = 8'd0; pl_buf[4] = 8'd0; pl_buf[5] = 8'd0;
            pl_buf[6] = 8'd0; pl_buf[7] = 8'd0;
            bad_seen = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            repeat (5) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer i;
    initial begin
        checks = 0;
        errors = 0;
        log_fd = $fopen("rc_rx_result.txt", "w");
        $fwrite(log_fd, "=== tb_rc_rx start ===\n");

        //=================================================================
        // 场景 1：复位值
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 1：复位值 --\n");
        do_reset;
        check("reset steer",  rc_steer,  0);
        check("reset speed",  rc_speed,  0);
        check("reset active", rc_active, 0);
        check("reset frames", rc_frames, 0);

        //=================================================================
        // 场景 2：正常帧 steer=+50 speed=100
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 2：正常帧 steer=+50 speed=100 --\n");
        send_rc(50, 100, 1, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("steer=+50 -> 800",  rc_steer,  800);
        check("speed=100 -> 2",    rc_speed,  2);
        check("active=1",          rc_active, 1);
        check("frames=1",          rc_frames, 1);

        //=================================================================
        // 场景 3：steer 负值 -50
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 3：steer=-50 --\n");
        send_rc(-50, 30, 2, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("steer=-50 -> -800", rc_steer, -800);
        check("speed=30 -> 1",     rc_speed,  1);
        check("frames=2",          rc_frames, 2);

        //=================================================================
        // 场景 4：steer 饱和
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 4：steer 饱和 --\n");
        send_rc(120, 80, 3, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("steer=+120 -> +1600", rc_steer,  1600);

        send_rc(-120, 80, 4, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("steer=-120 -> -1600", rc_steer, -1600);

        //=================================================================
        // 场景 5：speed 分档
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 5：speed 分档 --\n");
        send_rc(0, 0, 5, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("speed=0 -> 0",  rc_speed, 0);

        send_rc(0, 50, 6, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("speed=50 -> 1", rc_speed, 1);

        send_rc(0, 51, 7, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("speed=51 -> 2", rc_speed, 2);

        //=================================================================
        // 场景 6：长度不对 -> rc_bad，输出不变
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 6：长度不对 --\n");
        begin : sc6
            integer st_before;
            integer sp_before;
            integer fr_before;
            st_before = rc_steer;
            sp_before = rc_speed;
            fr_before = rc_frames;
            bad_seen  = 1'b0;
            send_rc(99, 99, 8, 2, 8'h05);       // len 故意给 2
            repeat (3) @(posedge clk);
            check("bad pulse seen", bad_seen, 1);
            check("steer unchanged", rc_steer,  st_before);
            check("speed unchanged", rc_speed,  sp_before);
            check("frames unchanged", rc_frames, fr_before);
        end

        //=================================================================
        // 场景 7：非 0x05 帧 -> 完全不响应
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 7：非 RC 帧 --\n");
        begin : sc7
            integer fr7;
            fr7 = rc_frames;
            send_rc(60, 60, 9, 3, 8'h02);       // 假装是视觉帧
            repeat (5) @(posedge clk);
            check("non-RC frames unchanged", rc_frames, fr7);
            check("non-RC addr back to 0",   rx_pl_addr, 0);
        end

        //=================================================================
        // 场景 8：序号不连续 -> rc_seq_err 累加
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 8：序号不连续 --\n");
        begin : sc8
            integer e_before;
            e_before = rc_seq_err;
            send_rc(10, 60, 10, 3, 8'h05);      // 上一帧 seq=7 -> 跳号 +1
            send_rc(10, 60, 11, 3, 8'h05);      // 连续
            send_rc(10, 60, 20, 3, 8'h05);      // ★ 跳号 -> +1
            repeat (3) @(posedge clk);
            check("seq_err increments twice", rc_seq_err - e_before, 2);
        end

        //=================================================================
        // 场景 9：看门狗超时 -> rc_active 归 0
        //   TB_TIMEOUT_MS = 2，等 4ms 足够
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 9：看门狗超时 --\n");
        send_rc(20, 60, 21, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("active before timeout", rc_active, 1);
        // 2ms = 100000 拍；多等一点，确保超时
        repeat (140000) @(posedge clk);
        check("active after timeout", rc_active, 0);
        check("frames hold on timeout", rc_frames, 11);

        //=================================================================
        // 场景 10：超时后再来帧 -> rc_active 恢复
        //=================================================================
        $fwrite(log_fd, "\n-- 场景 10：恢复 --\n");
        send_rc(30, 60, 22, 3, 8'h05);
        repeat (3) @(posedge clk);
        check("active recovered", rc_active, 1);
        check("frames=12",        rc_frames, 12);

        //=================================================================
        // 汇总
        //=================================================================
        $fwrite(log_fd, "\n=== tb_rc_rx: %0d checks, %0d errors ===\n", checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);

        if (errors == 0) $display("=== tb_rc_rx: %0d checks, 0 errors === PASS", checks);
        else             $display("=== tb_rc_rx: %0d checks, %0d errors === FAIL", checks, errors);

        $finish;
    end

    // 总超时保护
    initial begin
        #(50_000_000);      // 50ms 仿真时间上限
        $display("=== tb_rc_rx: TIMEOUT ===");
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $finish;
    end

endmodule
