//=============================================================================
// tb : tb_sr04_driver
//-----------------------------------------------------------------------------
// 拓扑: tof_sr04(DUT) --trig--> 本文件里的 HC-SR04 行为模型 --echo--> DUT
//
// 模型做什么：
//   看到 trig 上升沿 -> 延时 SETTLE(0.5ms) -> echo 拉高
//   -> 保持 (2*dist_mm/343 秒，按 CLK_FREQ_HZ 换成拍) -> echo 拉低
//   三种行为可切（model_mode）：
//     MODE_ECHO   正常回波（距离 = model_dist_mm）
//     MODE_NOECHO 无回波（永远不抬 echo）  -> 测"等上升沿超时"
//     MODE_STUCK  echo 抬起后永不落下       -> 测"量宽度超时"
//   模型是**时钟驱动的状态机**，不是"always 里挂一个 #delay"：
//   后者在回波还没发完就来下一个 trig 时会错过事件，而且"echo 一直为高"
//   那种用例会把进程永久卡死，后面的测试全废。
//   状态机写法每个状态只等一拍，任何时候来新 trig 都能正确处理。
//
//-----------------------------------------------------------------------------
// 时间尺度（为什么参数和真实值不一样）
//-----------------------------------------------------------------------------
//   真实配置是 SAMPLE=60ms / TMO=40ms。仿真里改成：
//     SAMPLE_MS    = 30     (trig 到 trig = 30ms)
//     TMO_MS       = 24     (够 3500mm 的往返 20.4ms + 0.5ms 建立时间)
//     MAX_MM       = 3200   (3000mm 仍算有效，3500mm 算超量程)
//     BOOT_WAIT_MS = 2
//   ★ 时钟仍保持 50MHz、TRIG_US 仍是 10us —— 换算系数 450 和脉宽都是
//     "真实值"，不掺假。
//   ★ SAMPLE_MS 必须 > TMO_MS（这里 30 > 24）：否则一次超时之后
//     period_cnt 已经过点，下一次 trig 会紧跟在本拍之后，tb 来不及在
//     trig 之前改好模型参数（会产生"测的是上一组参数"的随机失败）。
//     留 6ms 余量就是为了消除这个竞争。
//   整场仿真约 360ms 仿真时间（12 次测量 x 30ms + 启动），
//   50MHz 下约 1800 万拍。
//
//-----------------------------------------------------------------------------
// 覆盖的 10 条要求（检查名一一对应）
//-----------------------------------------------------------------------------
//   1) 复位状态 / 无脉冲 ........................ 1.x
//   2) 正常 1000mm ±10mm ....................... 2.1~2.3
//   3) 200 / 2000 / 3000mm ±10mm ............... 4.x
//   4) 无回波 -> dist_err 单拍 + dist_mm 保持 .. 5.x
//   5) 回波 < MIN_MM -> dist_err ............... 6.x
//   6) 回波 > MAX_MM -> dist_err ............... 7.x
//   7) trig 脉宽 = TRIG_US ±1us ................ 2.4
//   8) 测量周期 = SAMPLE_MS ±10% ............... 3.1
//   9) dist_valid / dist_err 不同拍同时为 1 .... 9.3（另有 9.1/9.2 单拍）
//  10) 连续 3 次正常 -> ok_cnt +3 .............. 10.x
//
//   ⚠️ 本 tb **没有**跑 ModelSim（沙箱里写不了 .simlib），
//      它是否真的 PASS 需要你在本机跑一次；静态检查只保证语法/编码/
//      例化名这些层面没问题。
//=============================================================================

`timescale 1ns/1ps

module tb_sr04_driver;

    //------------------------------------------------------------------
    // 时钟 / 时间
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ = 50_000_000;
    localparam integer CLK_PERIOD  = 20;                     // 20ns = 50MHz
    localparam integer MS_CYC      = CLK_FREQ_HZ / 1000;     // 1ms = 50000 拍

    //------------------------------------------------------------------
    // DUT 参数（仿真加速版，见文件头）
    //------------------------------------------------------------------
    localparam integer SAMPLE_MS    = 30;
    localparam integer TMO_MS       = 24;
    localparam integer TRIG_US      = 10;
    localparam integer MIN_MM       = 20;
    localparam integer MAX_MM       = 3200;
    localparam integer BOOT_WAIT_MS = 2;

    // 期望值
    localparam integer PERIOD_US_EXP = SAMPLE_MS * 1000;           // 30000us
    localparam integer PERIOD_US_TOL = PERIOD_US_EXP / 10;         // ±10%
    localparam integer DIST_TOL_MM   = 10;                         // ±10mm

    // 等一次测量结果的拍数预算
    //
    //   ★ 原值 45ms **不够**，会误报"没报错"：改完模型参数后要等下一次
    //     trig（最坏 SAMPLE_MS=30ms），再等回波/超时：
    //       最坏 = SAMPLE_MS(30) + TMO_MS(24) = 54ms
    //       3500mm 那次 = 30 + 20.4 = 50.9ms
    //     45ms 的窗口会让检查在结果出来之前就执行 —— 表现为
    //     "7.1 3500mm -> dist_err got=0"（而 7.3/7.4 是假阳性通过）。
    //   wait_event 是"事件到了就提前返回"，所以把这个上限加大
    //   只在**没有事件**的用例上多花仿真时间。
    localparam integer WAIT_CYC      = 65 * MS_CYC;

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk   = 0;
    reg rst_n = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    wire        trig;
    wire        echo;
    wire [15:0] dist_mm;
    wire        dist_valid;
    wire        dist_err;
    wire [15:0] ok_cnt;
    wire [15:0] err_cnt;

    tof_sr04 #(
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .SAMPLE_MS    (SAMPLE_MS),
        .TMO_MS       (TMO_MS),
        .TRIG_US      (TRIG_US),
        .MIN_MM       (MIN_MM),
        .MAX_MM       (MAX_MM),
        .BOOT_WAIT_MS (BOOT_WAIT_MS),
        ._ASSERT_EN   (1)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .trig       (trig),
        .echo       (echo),
        .dist_mm    (dist_mm),
        .dist_valid (dist_valid),
        .dist_err   (dist_err),
        .ok_cnt     (ok_cnt),
        .err_cnt    (err_cnt)
    );

    //==================================================================
    // HC-SR04 行为模型
    //==================================================================
    localparam integer MODE_ECHO   = 0;    // 正常回波
    localparam integer MODE_NOECHO = 1;    // 无回波
    localparam integer MODE_STUCK  = 2;    // 抬起后永不落下

    localparam [1:0]
        M_IDLE   = 2'd0,
        M_SETTLE = 2'd1,   // trig 之后 0.5ms 的"发射 + 建立"时间
        M_ECHO   = 2'd2,   // echo 高电平保持中
        M_STUCK  = 2'd3;   // echo 一直高（故障注入）

    // 0.5ms @50MHz
    localparam [63:0] SETTLE_CYC = 64'd25_000;

    // 由 tb 主流程改（只在 initial 里赋值，避免多驱动）
    integer model_mode    = MODE_ECHO;
    integer model_dist_mm = 1000;

    // 只由模型那个 always 块驱动
    reg        echo_r    = 1'b0;
    reg [1:0]  mst       = M_IDLE;
    reg [63:0] mcnt      = 64'd0;
    reg        model_clr = 1'b0;   // 由主流程置一拍，复位模型状态

    reg        trig_d    = 1'b0;   // trig 打一拍（模型和监视器共用）

    assign echo = echo_r;

    //------------------------------------------------------------------
    // 距离 -> echo 高电平拍数
    //   t = 2*d(m)/343 = 2*d_mm/343000 (s)；拍数 = t * CLK_FREQ_HZ
    //   => cycles = 2*d_mm*CLK_FREQ_HZ / 343000
    //   用 64 位算，d=4000mm 时分子 4e11，32 位早就爆了。
    //   （tb 里用除法没问题；RTL 里才是"不许除法器"。）
    //------------------------------------------------------------------
    function [63:0] cycles_for_mm;
        input integer d_mm;
        begin
            cycles_for_mm = (64'd2 * d_mm * CLK_FREQ_HZ) / 64'd343_000;
        end
    endfunction

    // trig 打一拍（模型/监视器都读它，保证看到的是同一个沿）
    always @(posedge clk) begin
        trig_d <= trig;
    end

    //------------------------------------------------------------------
    // 模型状态机：每个状态最多等一拍，随时能接受新 trig
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (model_clr) begin
            mst    <= M_IDLE;
            mcnt   <= 64'd0;
            echo_r <= 1'b0;
        end else begin
            case (mst)
            M_IDLE: begin
                echo_r <= 1'b0;
                if (trig && !trig_d) begin
                    if (model_mode == MODE_NOECHO) begin
                        mst <= M_IDLE;              // 什么都不发
                    end else begin
                        mcnt <= SETTLE_CYC;
                        mst  <= M_SETTLE;
                    end
                end
            end

            M_SETTLE: begin
                if (mcnt <= 64'd1) begin
                    echo_r <= 1'b1;
                    if (model_mode == MODE_STUCK) begin
                        mst <= M_STUCK;
                    end else begin
                        mcnt <= cycles_for_mm(model_dist_mm);
                        mst  <= M_ECHO;
                    end
                end else begin
                    mcnt <= mcnt - 64'd1;
                end
            end

            M_ECHO: begin
                if (mcnt <= 64'd1) begin
                    echo_r <= 1'b0;
                    mst    <= M_IDLE;
                end else begin
                    mcnt <= mcnt - 64'd1;
                end
            end

            M_STUCK: begin
                echo_r <= 1'b1;
            end

            default: mst <= M_IDLE;
            endcase
        end
    end

    //==================================================================
    // 监视器：脉冲计数 / 单拍 / 互斥 / trig 脉宽与周期
    //==================================================================
    integer    valid_cnt      = 0;
    integer    err_cnt_tb     = 0;      // 名字避开 DUT 的 err_cnt 端口
    integer    valid_double   = 0;      // dist_valid 连续两拍 = 不是单拍
    integer    err_double     = 0;
    integer    both_cnt       = 0;      // valid 与 err 同一拍为 1（必须为 0）
    integer    trig_rise_cnt  = 0;

    reg        prev_valid     = 0;
    reg        prev_err       = 0;
    reg [15:0] last_dist      = 16'd0;  // 最近一次 dist_valid 时的 dist_mm
    time       trig_rise_time;
    time       last_width_ns;           // 最近一个完整 trig 脉冲的宽度
    time       last_period_ns;          // 最近两个 trig 上升沿的间隔
    integer    last_elapsed_us;         // 最近一次 dist_err 距上一次 trig 上升沿多久

    always @(posedge clk) begin
        if (rst_n) begin
            //---- TRIG 脉宽 / 周期 ----
            if (trig && !trig_d) begin
                trig_rise_cnt = trig_rise_cnt + 1;
                if (trig_rise_cnt >= 2) last_period_ns = $time - trig_rise_time;
                trig_rise_time = $time;
            end
            if (!trig && trig_d) begin
                last_width_ns = $time - trig_rise_time;
            end

            //---- 结果脉冲 ----
            if (dist_valid) begin
                valid_cnt = valid_cnt + 1;
                last_dist = dist_mm;
                if (prev_valid) valid_double = valid_double + 1;
            end
            if (dist_err) begin
                err_cnt_tb  = err_cnt_tb + 1;
                last_elapsed_us = (($time - trig_rise_time) / 1000);
                if (prev_err) err_double = err_double + 1;
            end
            if (dist_valid && dist_err) both_cnt = both_cnt + 1;

            prev_valid = dist_valid;
            prev_err   = dist_err;
        end
    end

    //==================================================================
    // 结果文件 / 检查任务
    //==================================================================
    integer log_fd;
    integer errors = 0;
    integer checks = 0;

    task check;
        // 名字端口用 [1023:0]：中文一个字 3 字节，窄了会把编号从前面截掉
        input [1023:0] name;
        input integer  got;
        input integer  exp;
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

    task check_near;
        input [1023:0] name;
        input integer  got;
        input integer  exp;
        input integer  tol;
        integer diff;
        begin
            checks = checks + 1;
            diff = got - exp;
            if (diff < 0) diff = -diff;
            if (diff <= tol) begin
                $fwrite(log_fd, "  [ OK ] %0s : got=%0d exp=%0d(+-%0d)\n",
                        name, got, exp, tol);
            end else begin
                errors = errors + 1;
                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d(+-%0d)\n",
                        name, got, exp, tol);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 工具任务
    //------------------------------------------------------------------
    // 等下一个 dist_valid 或 dist_err（带拍数预算）
    task wait_event;
        input integer max_cyc;
        integer v0;
        integer e0;
        integer k;
        begin
            v0 = valid_cnt;
            e0 = err_cnt_tb;
            k  = 0;
            while ((valid_cnt == v0) && (err_cnt_tb == e0) && (k < max_cyc)) begin
                @(posedge clk);
                #1;
                k = k + 1;
            end
        end
    endtask

    // 复位（tb 侧）+ 清模型
    task do_reset;
        begin
            rst_n = 1'b0;
            model_clr = 1'b1;
            repeat (20) @(posedge clk);
            rst_n = 1'b1;
            model_clr = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer v0;        // 事件前的 valid 计数
    integer e0;        // 事件前的 err 计数
    integer d0;        // 事件前的 dist_mm（"保持上次有效值"用）
    integer ok0;       // DUT ok_cnt 快照
    integer width_us;  // trig 脉宽（us）
    integer period_us; // trig 周期（us）

    initial begin
        // 先开文件（放在主 initial 的第一句，避免"两个 initial 在 0 时刻的
        // 执行顺序不定"导致第一行 $fwrite 落到未初始化的 log_fd 上）
        log_fd = $fopen("sr04_result.txt", "w");
        if (log_fd == 0) begin
            $display("[warn] 无法创建 sr04_result.txt，改用标准输出");
            log_fd = 1;
        end

        $fwrite(log_fd, "=== tb_sr04_driver: HC-SR04 超声波驱动自检 ===\n");
        $fwrite(log_fd, "  参数: CLK=%0dHz SAMPLE=%0dms TMO=%0dms TRIG=%0dus MIN=%0dmm MAX=%0dmm BOOT=%0dms\n",
                CLK_FREQ_HZ, SAMPLE_MS, TMO_MS, TRIG_US, MIN_MM, MAX_MM, BOOT_WAIT_MS);
        $fwrite(log_fd, "  说明: CLK/TRIG_US 用真实值(50MHz/10us)；SAMPLE/TMO/MAX 缩小以缩短仿真,\n");
        $fwrite(log_fd, "        比例关系与真实配置(60ms/40ms/4000mm)一致。\n");
        $fwrite(log_fd, "        模型: 0.5ms 建立 -> echo 宽度 = 2*d/343s (按 50MHz 换成拍)\n");

        //==================================================================
        // 1. 复位状态：trig 低、无脉冲、上电窗口内不测量
        //==================================================================
        $fwrite(log_fd, "\n--- 1. 复位 / 上电等待 ---\n");
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        check("1.1 复位后 trig=0",            trig,        0);
        check("1.2 复位后 dist_valid 无脉冲", valid_cnt,   0);
        check("1.3 复位后 dist_err 无脉冲",   err_cnt_tb,  0);
        check("1.4 复位后 dist_mm=0",         dist_mm,     0);
        check("1.5 复位后 ok_cnt=0",          ok_cnt,      0);
        rst_n = 1'b1;

        // 上电等待窗口（BOOT_WAIT_MS=2ms）内不许有 trig，也不许有结果脉冲。
        // 第一次 trig 在 SAMPLE_MS(30ms) 才来，所以这里等 5ms 观察。
        repeat (5 * MS_CYC) @(posedge clk);
        check("1.6 上电窗口内未发 trig",      trig_rise_cnt, 0);
        check("1.7 上电窗口内无结果脉冲",     valid_cnt + err_cnt_tb, 0);

        //==================================================================
        // 2. 正常测距 1000mm
        //==================================================================
        $fwrite(log_fd, "\n--- 2. 正常测距 1000mm ---\n");
        model_mode    = MODE_ECHO;
        model_dist_mm = 1000;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("2.1 1000mm 收到 dist_valid",   (valid_cnt > v0) ? 1 : 0, 1);
        check_near("2.2 dist_mm=1000+-10",    last_dist, 1000, DIST_TOL_MM);
        check("2.3 1000mm 无 dist_err",       err_cnt_tb, e0);

        // trig 脉宽（同一帧的 trig 早已结束）
        width_us = last_width_ns / 1000;
        $fwrite(log_fd, "  [info] trig 脉宽 = %0dus (期望 %0dus)\n", width_us, TRIG_US);
        check_near("2.4 trig 脉宽=TRIG_US+-1us", width_us, TRIG_US, 1);

        //==================================================================
        // 3+4. 200mm（这一帧同时用来量"trig 到 trig"的周期）
        //==================================================================
        $fwrite(log_fd, "\n--- 3. 测量周期 / 4. 换算系数抽查 ---\n");
        model_dist_mm = 200;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("3.1 第二帧收到 dist_valid",    (valid_cnt > v0) ? 1 : 0, 1);
        period_us = last_period_ns / 1000;
        $fwrite(log_fd, "  [info] trig 周期 = %0dus (期望 %0dus)\n",
                period_us, PERIOD_US_EXP);
        check_near("3.2 测量周期=SAMPLE_MS+-10%", period_us,
                   PERIOD_US_EXP, PERIOD_US_TOL);

        check_near("4.1 200mm -> dist_mm +-10",   last_dist, 200, DIST_TOL_MM);

        //------------------------------------------------------------------
        // 2000mm
        //------------------------------------------------------------------
        model_dist_mm = 2000;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);
        check("4.2 2000mm 收到 dist_valid",   (valid_cnt > v0) ? 1 : 0, 1);
        check_near("4.3 2000mm -> dist_mm +-10",  last_dist, 2000, DIST_TOL_MM);
        check("4.4 2000mm 无 dist_err",       err_cnt_tb, e0);

        //------------------------------------------------------------------
        // 3000mm（量程内最远的一档，验证换算系数在高端的误差）
        //------------------------------------------------------------------
        model_dist_mm = 3000;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);
        check("4.5 3000mm 收到 dist_valid",   (valid_cnt > v0) ? 1 : 0, 1);
        check_near("4.6 3000mm -> dist_mm +-10",  last_dist, 3000, DIST_TOL_MM);
        check("4.7 3000mm 无 dist_err",       err_cnt_tb, e0);

        //==================================================================
        // 5. 无回波 -> dist_err（单拍）+ dist_mm 保持上次有效值
        //==================================================================
        $fwrite(log_fd, "\n--- 5. 无回波（等上升沿超时）---\n");
        d0 = last_dist;                  // 上一次有效值 = 3000mm 那一档
        model_mode = MODE_NOECHO;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("5.1 无回波 -> dist_err",       (err_cnt_tb > e0) ? 1 : 0, 1);
        check("5.2 无回波 -> 本次只有1个err",  err_cnt_tb, e0 + 1);
        check("5.3 无回波 -> 无 dist_valid",  valid_cnt, v0);
        check("5.4 无回波 -> dist_mm 保持",   last_dist, d0);
        check("5.5 无回波 -> DUT dist_mm 保持", dist_mm,  d0);

        //==================================================================
        // 6. 回波太近（< MIN_MM）
        //    10mm 的回波宽度约 58us，远小于超时，所以走的必然是"范围判断"。
        //==================================================================
        $fwrite(log_fd, "\n--- 6. 回波太近（< MIN_MM=%0dmm）---\n", MIN_MM);
        model_mode    = MODE_ECHO;
        model_dist_mm = 10;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("6.1 10mm -> dist_err",         (err_cnt_tb > e0) ? 1 : 0, 1);
        check("6.2 10mm -> 只有1个err脉冲",    err_cnt_tb, e0 + 1);
        check("6.3 10mm -> 无 dist_valid",    valid_cnt, v0);
        check("6.4 10mm -> dist_mm 保持不变", dist_mm,   d0);

        //==================================================================
        // 7. 回波太远（> MAX_MM）
        //    3500mm -> echo 宽约 20.4ms，仍在 TMO_MS(24ms) 内，
        //    所以必须靠"范围判断"报错，而不是靠超时。
        //==================================================================
        $fwrite(log_fd, "\n--- 7. 回波太远（> MAX_MM=%0dmm）---\n", MAX_MM);
        model_dist_mm = 3500;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("7.1 3500mm -> dist_err",       (err_cnt_tb > e0) ? 1 : 0, 1);
        check("7.2 3500mm -> 只有1个err脉冲",  err_cnt_tb, e0 + 1);
        check("7.3 3500mm -> 无 dist_valid",  valid_cnt, v0);
        check("7.4 3500mm -> dist_mm 保持",   dist_mm,   d0);
        // 距离判断的旁证：错误时刻 ≈ trig 后 (0.5ms+20.4ms)=20.9ms，
        // 而超时会等到 24ms。若实现把"超量程"当超时处理，这里会 >=24ms。
        $fwrite(log_fd, "  [info] 超量程报错时刻 = trig 后 %0dus (TMO=%0dus)\n",
                last_elapsed_us, TMO_MS * 1000);
        check("7.5 超量程在回波结束时报错(非超时)",
              (last_elapsed_us < (TMO_MS * 1000)) ? 1 : 0, 1);

        //==================================================================
        // 8. echo 一直为高 -> 量宽度超时
        //==================================================================
        $fwrite(log_fd, "\n--- 8. echo 不掉（量宽度超时）---\n");
        model_mode = MODE_STUCK;
        v0 = valid_cnt;
        e0 = err_cnt_tb;
        wait_event(WAIT_CYC);

        check("8.1 echo 不落 -> dist_err",    (err_cnt_tb > e0) ? 1 : 0, 1);
        check("8.2 echo 不落 -> 只有1个err脉冲", err_cnt_tb, e0 + 1);
        check("8.3 echo 不落 -> 无 dist_valid", valid_cnt, v0);
        check("8.4 echo 不落 -> dist_mm 保持", dist_mm,   d0);

        // 释放模型（把 echo 拉低），否则后面所有测量都会继续超时
        model_mode = MODE_NOECHO;
        model_clr  = 1'b1;
        @(posedge clk);
        #1;
        model_clr  = 1'b0;
        check("8.5 模型释放后 echo=0",        echo, 0);

        //==================================================================
        // 9. 单拍 / 互斥（到这里的统计是累计的，覆盖了前面所有帧）
        //==================================================================
        $fwrite(log_fd, "\n--- 9. 单拍脉冲 / valid-err 互斥 ---\n");
        check("9.1 dist_valid 从不连续两拍",  valid_double, 0);
        check("9.2 dist_err 从不连续两拍",    err_double,   0);
        check("9.3 valid 与 err 从不同拍",    both_cnt,     0);

        //==================================================================
        // 10. 连续 3 次正常 -> ok_cnt 恰好 +3
        //==================================================================
        $fwrite(log_fd, "\n--- 10. 连续 3 次正常测距 -> ok_cnt ---\n");
        model_mode    = MODE_ECHO;
        model_dist_mm = 500;
        ok0 = ok_cnt;
        v0  = valid_cnt;
        wait_event(WAIT_CYC);
        check("10.1 连续正常第1帧 dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        v0  = valid_cnt;
        wait_event(WAIT_CYC);
        check("10.2 连续正常第2帧 dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        v0  = valid_cnt;
        wait_event(WAIT_CYC);
        check("10.3 连续正常第3帧 dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        check("10.4 ok_cnt 恰好增 3",          ok_cnt, ok0 + 3);
        check_near("10.5 第3次 dist_mm=500+-10", last_dist, 500, DIST_TOL_MM);

        //==================================================================
        // 11. 复位清状态 + 复位后能重新工作
        //==================================================================
        $fwrite(log_fd, "\n--- 11. 复位清状态 / 恢复 ---\n");
        do_reset;
        check("11.1 复位后 trig=0",           trig,     0);
        check("11.2 复位清 ok_cnt",           ok_cnt,   0);
        check("11.3 复位清 err_cnt",          err_cnt,  0);
        check("11.4 复位清 dist_mm",          dist_mm,  0);

        model_mode    = MODE_ECHO;
        model_dist_mm = 1500;
        v0 = valid_cnt;
        wait_event(WAIT_CYC + BOOT_WAIT_MS * MS_CYC);
        check("11.5 复位后恢复出数",          (valid_cnt > v0) ? 1 : 0, 1);
        check_near("11.6 恢复后 dist_mm=1500+-10", last_dist, 1500, DIST_TOL_MM);

        //==================================================================
        // 汇总
        //==================================================================
        $fwrite(log_fd, "\n  [info] 累计: trig=%0d 次, dist_valid=%0d 次, dist_err=%0d 次\n",
                trig_rise_cnt, valid_cnt, err_cnt_tb);
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_sr04_driver: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_sr04_driver: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    //------------------------------------------------------------------
    // 仿真总超时（DUT 卡死时不要让仿真永不结束）
    //   正常约 360ms 仿真时间，这里给 1s。
    //------------------------------------------------------------------
    initial begin
        #(1_000_000_000);                 // 1000ms
        $fwrite(log_fd, "\n[FATAL] 仿真超时（DUT 可能卡死）\n");
        $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
