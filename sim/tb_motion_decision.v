//=============================================================================
// tb : tb_motion_decision
//-----------------------------------------------------------------------------
// 验证 rtl/app/motion_decision.v 的算术与符号约定。
//
// 本模块是**纯组合逻辑**，所以测试很简单：摆好输入 -> #1 -> 采样输出。
// 重点不在时序，而在：
//   * speed_scale -> base 的映射对不对
//   * 比例控制/死区/限幅的边界
//   * **转向符号**（左轮加速才是右转）——这是最容易搞反的地方
//   * 关使能时是否真的强制 0
//
//-----------------------------------------------------------------------------
// 约定（与 RTL 一致）
//-----------------------------------------------------------------------------
//   ★ 轮子编号：0=左前 1=右前 2=左后 3=右后（全项目统一）
//     -> 左轮是 0 和 2（系数 +1），右轮是 1 和 3（系数 -1）
//
//   steer > 0  = 右转 = 左轮加速、右轮减速
//              = 左轮系数 +1、右轮系数 -1
//   duty = clamp( (target - fb) >>> KP_SHIFT ) 的绝对值
//   dir  = 该命令的符号
//
// 运行: vsim -c -do "vsim -c -quiet -voptargs=+acc -L work -L gw5a \
//                     work.tb_motion_decision; run -all; quit -f"
// 结果: motion_decision_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_motion_decision;

    //------------------------------------------------------------------
    // 参数（与 RTL 默认值一致）
    //------------------------------------------------------------------
    localparam integer VREF          = 60;
    localparam integer KP_SHIFT      = 1;
    localparam integer LIMIT_DUTY    = 120;
    localparam integer ZERO_DEADBAND = 1;
    localparam integer STEER_SHIFT   = 4;

    localparam [1:0] SPD_STOP = 2'd0;
    localparam [1:0] SPD_SLOW = 2'd1;
    localparam [1:0] SPD_FULL = 2'd2;

    //------------------------------------------------------------------
    // 信号
    //------------------------------------------------------------------
    reg         clk = 0;
    reg         rst_n = 0;

    reg         motor_enable = 0;
    reg  [1:0]  speed_scale = SPD_STOP;
    reg  signed [15:0] steer = 0;

    reg  signed [23:0] spd_fb0 = 0;
    reg  signed [23:0] spd_fb1 = 0;
    reg  signed [23:0] spd_fb2 = 0;
    reg  signed [23:0] spd_fb3 = 0;
    reg         spd_valid = 0;

    wire [15:0] duty0, duty1, duty2, duty3;
    wire        dir0, dir1, dir2, dir3;
    wire        limited;
    wire signed [15:0] target0, target1, target2, target3;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;

    initial begin
        log_fd = $fopen("motion_decision_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 motion_decision_result.txt");
    end

    always #10 clk = ~clk;      // 50MHz，本模块组合逻辑为主，时钟只为复位

    //==================================================================
    // DUT（用默认参数，保证与 Python 模型一致）
    //==================================================================
    motion_decision #(
        .CLK_FREQ_HZ    (50_000_000),
        .VREF           (VREF),
        .KP_SHIFT       (KP_SHIFT),
        .LIMIT_DUTY     (LIMIT_DUTY),
        .ZERO_DEADBAND  (ZERO_DEADBAND),
        .STEER_COEF0    ( 1),      // 左前 FL -> 左
        .STEER_COEF1    (-1),      // 右前 FR -> 右
        .STEER_COEF2    ( 1),      // 左后 RL -> 左
        .STEER_COEF3    (-1),      // 右后 RR -> 右
        .STEER_SHIFT    (STEER_SHIFT)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .motor_enable (motor_enable),
        .speed_scale  (speed_scale),
        .speed_pct    (8'd100),
        .dir_inv      (1'b0),
        .vref         (VREF[15:0]),   // 运行时配置口：这里给与参数相同的值
        .steer        (steer),
        .spd_fb0      (spd_fb0),
        .spd_fb1      (spd_fb1),
        .spd_fb2      (spd_fb2),
        .spd_fb3      (spd_fb3),
        .spd_valid    (spd_valid),
        .duty0        (duty0),
        .duty1        (duty1),
        .duty2        (duty2),
        .duty3        (duty3),
        .dir0         (dir0),
        .dir1         (dir1),
        .dir2         (dir2),
        .dir3         (dir3),
        .limited      (limited),
        .target0      (target0),
        .target1      (target1),
        .target2      (target2),
        .target3      (target3),
        .spd_stale_o  ()                // 本 TB 不验超时降级，留空
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
            if (got !== exp) begin
                errors = errors + 1;
                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d\n", name, got, exp);
            end else begin
                $fwrite(log_fd, "  [ OK ] %0s : %0d\n", name, got);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 摆好输入 + 等组合逻辑稳定
    //------------------------------------------------------------------
    task apply;
        input               en;
        input [1:0]         spd;
        input signed [15:0] st;
        input signed [23:0] f0;
        input signed [23:0] f1;
        input signed [23:0] f2;
        input signed [23:0] f3;
        begin
            motor_enable = en;
            speed_scale  = spd;
            steer        = st;
            spd_fb0 = f0; spd_fb1 = f1; spd_fb2 = f2; spd_fb3 = f3;
            #1;
        end
    endtask

    task apply_all;
        input               en;
        input [1:0]         spd;
        input signed [15:0] st;
        input signed [23:0] fb;
        begin
            apply(en, spd, st, fb, fb, fb, fb);
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        $fwrite(log_fd, "=== tb_motion_decision ===\n");
        $fwrite(log_fd, "  [info] VREF=%0d KP_SHIFT=%0d LIMIT=%0d DB=%0d STEER_SHIFT=%0d\n",
                VREF, KP_SHIFT, LIMIT_DUTY, ZERO_DEADBAND, STEER_SHIFT);
        $fwrite(log_fd, "  [info] 约定: steer>0 = 右转 = 左轮加速(系数+1) / 右轮减速(系数-1)\n");

        rst_n = 1'b0;
        #25;
        rst_n = 1'b1;
        #1;

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. speed_scale -> base 映射 ---\n");
        apply_all(1'b1, SPD_STOP, 16'sd0, 24'sd0);
        check("1.1 STOP: 四轮 duty 全为 0",
              (duty0|duty1|duty2|duty3), 0);
        apply_all(1'b1, SPD_SLOW, 16'sd0, 24'sd0);
        check("1.2 SLOW: target=30", target0, 30);
        apply_all(1'b1, SPD_FULL, 16'sd0, 24'sd0);
        check("1.3 FULL: target=60", target0, 60);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 比例控制 + 死区 ---\n");
        apply_all(1'b1, SPD_FULL, 16'sd0, 24'sd0);
        check("2.1 静止: duty=30", duty0, 30);
        check("2.2 静止: dir=1", dir0, 1);
        check("2.3 静止: limited=0", limited, 0);

        apply_all(1'b1, SPD_FULL, 16'sd0, 24'sd60);
        check("2.4 达到目标: duty=0 (死区)", duty0, 0);

        apply_all(1'b1, SPD_FULL, 16'sd0, 24'sd61);
        check("2.5 略微超速: duty=1", duty0, 1);
        check("2.6 略微超速: dir=0 (算术右移保号)", dir0, 0);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 限幅（每轮独立）---\n");
        apply_all(1'b1, SPD_FULL, 16'sd0, -24'sd400);
        check("3.1 深超速: duty=120 (饱和)", duty0, 120);
        check("3.2 深超速: limited=1", limited, 1);

        apply(1'b1, SPD_FULL, 16'sd0, -24'sd400, 24'sd0, 24'sd0, 24'sd0);
        check("3.3 只有 0 路饱和: duty0=120", duty0, 120);
        check("3.4 只有 0 路饱和: duty1=30 (不受拖累)", duty1, 30);
        check("3.5 只有 0 路饱和: limited=1", limited, 1);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 关使能 ---\n");
        apply_all(1'b0, SPD_FULL, 16'sd0, -24'sd400);
        check("4.1 关使能: 四轮 duty 全为 0",
              (duty0|duty1|duty2|duty3), 0);
        check("4.2 关使能: limited=0", limited, 0);
        check("4.3 关使能: 方向仍给出", dir0, 1);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 转向符号（最容易搞反的地方）---\n");
        apply_all(1'b1, SPD_FULL, 16'sd160, 24'sd0);
        check("5.1 steer=160: 左轮 target=70", target0, 70);
        check("5.2 steer=160: 右前 target=50", target1, 50);
        check("5.3 ★右转: 左轮比右轮目标高",
              (target0 > target1) ? 1 : 0, 1);
        check("5.4 右转: 左轮 duty=35", duty0, 35);
        check("5.5 右转: 右前 duty=25", duty1, 25);

        apply_all(1'b1, SPD_FULL, -16'sd160, 24'sd0);
        check("5.6 steer=-160: 左轮 target=50", target0, 50);
        check("5.7 steer=-160: 右前 target=70", target1, 70);
        check("5.8 ★左转: 右轮比左轮目标高",
              (target1 > target0) ? 1 : 0, 1);

        apply_all(1'b1, SPD_FULL, 16'sd0, 24'sd0);
        check("5.9 steer=0: 四轮目标相同（走直线）",
              (target0==target1 && target1==target2 && target2==target3) ? 1 : 0, 1);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 转向不附带整体加减速（系数和为 0）---\n");
        apply_all(1'b1, SPD_FULL, 16'sd160, 24'sd0);
        check("6.1 转向后四轮目标之和 = 4*60 = 240",
              target0 + target1 + target2 + target3, 240);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 低速档 + 转向 ---\n");
        apply_all(1'b1, SPD_SLOW, 16'sd160, 24'sd0);
        check("7.1 SLOW+右转: 左轮 target=40", target0, 40);
        check("7.2 SLOW+右转: 右前 target=20", target1, 20);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 反馈为负 / 滑行 ---\n");
        apply_all(1'b1, SPD_FULL, 16'sd0, -24'sd20);
        check("8.1 反馈为负: duty=40 (加速纠正)", duty0, 40);
        check("8.2 反馈为负: dir=1", dir0, 1);

        apply_all(1'b1, SPD_STOP, 16'sd0, 24'sd50);
        check("8.3 STOP 档 + 滑行: duty=0 (不主动刹车)",
              (duty0|duty1|duty2|duty3), 0);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n=== tb_motion_decision: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 超时保护（注意不要写成超过 32 位的字面量）
    //------------------------------------------------------------------
    initial begin
        #1_000_000;                      // 1ms 足够（本模块基本是组合逻辑）
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
