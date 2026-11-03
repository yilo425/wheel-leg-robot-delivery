//=============================================================================
// tb : tb_main_fsm
//-----------------------------------------------------------------------------
// 验证 rtl/app/main_fsm.v 的状态转移与输出。
//
//-----------------------------------------------------------------------------
// 设计原则（沿用本项目已验证的几条约定）
//-----------------------------------------------------------------------------
// 1) 结果用 $fwrite 落盘，不依赖 stdout
//      ModelSim 批处理模式下 $display 走 stdout，管道经常抓不到。
//      结果文件: main_fsm_result.txt
//      而且不只写 PASS/FAIL，**关键测量值也落盘** —— 否则事后无法定位。
//
// 2) 判据要无歧义
//      比如"急停是否 0 拍断使能"，不能只看"最终 motor_enable==0"，
//      必须在 estop 拉起的**同一拍**就检查。
//
// 3) 不使用 dut.xxx 层次引用
//      层次引用要求 vsim 带 -voptargs="+acc"，否则优化器删信号后
//      加载阶段直接报 "Error loading design" 且不给细节。只用端口信号。
//
// 4) TOF 读数是"先锁存、下一拍生效"的，测试要显式考虑这个延迟
//      （这不是缺陷，是避免组合路径直穿传感器输入）
//
//-----------------------------------------------------------------------------
// 参数说明：时钟/超时都按真实语义给，只是把超时缩短以便仿真跑得快
//-----------------------------------------------------------------------------
// 运行: vsim -c -do "vsim -c -quiet -voptargs=+acc -L work -L gw5a \
//                     work.tb_main_fsm; run -all; quit -f"
//=============================================================================
`timescale 1ns/1ps

module tb_main_fsm;

    //------------------------------------------------------------------
    // 参数（全部显式声明；位序相关一律用具名常量）
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ      = 50_000_000;
    localparam integer CLK_PERIOD       = 20;      // 20ns 时钟

    localparam integer DIST_STOP_MM     = 200;
    localparam integer DIST_SLOW_MM     = 500;
    localparam integer DIST_CLEAR_MM    = 800;

    // 缩短超时以便快速仿真（RTL 默认 5000/3000/2000 ms）
    localparam integer AVOID_TIMEOUT_MS = 2;
    localparam integer LIFT_TIMEOUT_MS  = 5;
    // ★ 2026-10-18：抬腿超时改成运行时端口（CONFIG 帧 0x11），这里用 reg 驱动它
    reg [15:0] lift_timeout_ms = 16'd5;
    localparam integer CROSS_TIMEOUT_MS = 1;

    //------------------------------------------------------------------
    // 越障时序（RTL 默认 CROSS_MIN_MS=800 / LIFT_SETTLE_MS=600）
    //
    //   这两个数必须和下面的测试里等待的拍数对得上：
    //     1ms = MS_TICKS = 50000 拍
    //   测试里用 wait_ms 等待，不写裸 repeat(N)。
    //------------------------------------------------------------------
    localparam integer CROSS_MIN_MS     = 1;      // 整车通过最短保持时间
    localparam integer LIFT_SETTLE_MS   = 4;      // 放腿等待（要比 CROSS_MIN 长，测试才能卡在 LP_DOWN 上）

    //------------------------------------------------------------------
    // ★ 越障失败降级（2026-10-19 新增）
    //   锁定时长/清零时长也缩到几毫秒，否则仿真要跑好几秒
    //------------------------------------------------------------------
    localparam integer LIFT_LOCKOUT_MS  = 8;      // 失败后 8ms 内不许再抬腿
    localparam integer LIFT_CLEAR_MS    = 40;     // 干净 40ms 才清零（要大于两次失败之间的间隔 ~15ms）
    localparam integer LIFT_RETRY_MAX   = 3;      // 连续 3 次失败 -> 真停车

    localparam integer MS_TICKS         = CLK_FREQ_HZ / 1000;   // 50000
    localparam integer AVOID_TMO_TICKS  = AVOID_TIMEOUT_MS * MS_TICKS;
    localparam integer LIFT_TMO_TICKS   = LIFT_TIMEOUT_MS  * MS_TICKS;
    localparam integer CROSS_TMO_TICKS  = CROSS_TIMEOUT_MS * MS_TICKS;
    localparam integer CROSS_MIN_TICKS  = CROSS_MIN_MS     * MS_TICKS;
    localparam integer SETTLE_TICKS     = LIFT_SETTLE_MS   * MS_TICKS;

    //------------------------------------------------------------------
    // 状态/子阶段/速度 编码（必须与 RTL 和 interface_spec.md 一致）
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE   = 3'd0,
                     S_MANUAL = 3'd1,
                     S_AUTO   = 3'd2,
                     S_AVOID  = 3'd3,
                     S_LIFT   = 3'd4,
                     S_GOAL   = 3'd5,
                     S_ESTOP  = 3'd6;

    localparam [2:0] LP_IDLE    = 3'd0,
                     LP_UP      = 3'd1,   // 模式0：四腿一起抬
                     LP_CROSS   = 3'd2,   // 模式0：一起慢速通过
                     LP_F_UP    = 3'd3,   // 模式1：前腿抬
                     LP_F_CROSS = 3'd4,   // 模式1：前轮上位
                     LP_R_UP    = 3'd5,   // 模式1：后腿抬（前腿保持）
                     LP_R_CROSS = 3'd6,   // 模式1：后轮上位
                     LP_DOWN    = 3'd7;   // 公共：都放下

    localparam [1:0] SPD_STOP = 2'd0,
                     SPD_SLOW = 2'd1,
                     SPD_FULL = 2'd2;

    localparam [7:0] VIS_LOW_OBST = 8'h04;
    localparam [7:0] VIS_STEP     = 8'h05;   // 台阶 -> 两腿分段
    // 障碍物（可绕行）—— ARM 侧 camera_vision.py 固定发这个类别
    localparam [7:0] VIS_OBST     = 8'h02;
    // 保持窗口：仿真里缩短到 2ms（RTL 默认 300ms）
    localparam integer VIS_HOLD_MS = 2;

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk = 0;
    reg         rst_n = 0;

    reg         start = 0;
    reg         estop = 0;
    reg         manual_sel = 0;

    reg  [15:0] dist_mm = 0;
    reg         dist_valid = 0;
    reg         dist_err = 0;

    reg  [7:0]  vision_class = 0;
    reg         vision_valid = 0;

    reg         goal_reached = 1;
    // 升降腿反馈：2 位 {后腿, 前腿}，**高有效**（main_fsm 的输入侧）
    //   注意：这是 main_fsm 的接口，不是顶层引脚。
    //   顶层是低有效的 lift_lim_n，极性转换在 servo_lift 里做。
    reg  [1:0]  lift_done  = 2'b00;
    reg  [1:0]  lift_fault = 2'b00;

    wire [2:0]  state;
    wire        motor_enable;
    wire [1:0]  speed_scale;
    wire [1:0]  lift_cmd;
    wire [2:0]  lift_phase;
    wire        avoid_active;
    wire        lift_active;
    wire        error_flag;

    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer i;

    initial begin
        log_fd = $fopen("main_fsm_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 main_fsm_result.txt");
    end

    always #(CLK_PERIOD/2) clk = ~clk;

    //==================================================================
    // DUT
    //==================================================================
    main_fsm #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .DIST_STOP_MM     (DIST_STOP_MM),
        .DIST_SLOW_MM     (DIST_SLOW_MM),
        .DIST_CLEAR_MM    (DIST_CLEAR_MM),
        .AVOID_TIMEOUT_MS (AVOID_TIMEOUT_MS),
        .LIFT_TIMEOUT_MS  (LIFT_TIMEOUT_MS),
        .CROSS_TIMEOUT_MS (CROSS_TIMEOUT_MS),
        .CROSS_MIN_MS     (CROSS_MIN_MS),
        .LIFT_SETTLE_MS   (LIFT_SETTLE_MS),
        .VIS_LOW_OBST     (VIS_LOW_OBST),
        .VIS_OBST         (VIS_OBST),
        .VIS_STEP         (VIS_STEP),
        // ★ 越障失败降级用的时间参数，缩到很小以便仿真快
        .LIFT_LOCKOUT_MS  (LIFT_LOCKOUT_MS),
        .LIFT_CLEAR_MS    (LIFT_CLEAR_MS),
        .LIFT_RETRY_MAX   (LIFT_RETRY_MAX),
        .VIS_HOLD_MS      (VIS_HOLD_MS)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start),
        .estop        (estop),
        .manual_sel   (manual_sel),
        .dist_mm      (dist_mm),
        .dist_valid   (dist_valid),
        .dist_err     (dist_err),
        .dist_stop_mm (DIST_STOP_MM[15:0]),   // 运行时配置口：给与参数相同的值
        .dist_slow_mm (DIST_SLOW_MM[15:0]),
        .lift_timeout_ms (lift_timeout_ms),         // 运行时配置口：下面场景 12 会改它
        .vision_class (vision_class),
        .vision_valid (vision_valid),
        .goal_reached (goal_reached),
        .lift_done    (lift_done),
        .lift_fault   (lift_fault),
        .state        (state),
        .motor_enable (motor_enable),
        .speed_scale  (speed_scale),
        .lift_cmd     (lift_cmd),
        .lift_phase   (lift_phase),
        .avoid_active (avoid_active),
        .lift_active  (lift_active),
        .error_flag   (error_flag)
    );

    //==================================================================
    // 工具任务
    //==================================================================
    task check;
        // ⚠️ name 是**字符串**，宽度决定能放多少字节。
        //   原来是 [255:0] = 32 字节。中文一个字 3 字节，一个长一点的
        //   检查名（带括号说明的）就超过 32 字节了 —— 超出的部分会被
        //   **从前面截掉**，结果日志里看不到 "11a.3" 这种编号，
        //   只有后半句，排查时对不上号。
        //   加宽到 [1023:0] = 128 字节，够用。
        input [1023:0] name;
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
    // 推一拍：等时钟沿 + #1，让所有非阻塞更新生效后再采样
    //   输入应在调用前设置好（组合式驱动）
    //------------------------------------------------------------------
    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    //------------------------------------------------------------------
    // 等 n 毫秒
    //
    //   越障状态机里的 CROSS_MIN_MS / LIFT_SETTLE_MS 是**毫秒级**的，
    //   而 step 只走一个时钟。1ms = 50000 拍 ——
    //   写裸 repeat(60) 差三个数量级，状态机根本不会推进。
    //------------------------------------------------------------------
    task wait_ms;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n * MS_TICKS; k = k + 1) step;
        end
    endtask

    // 清除所有单拍脉冲
    task clear_pulses;
        begin
            start        = 1'b0;
            dist_valid   = 1'b0;
            dist_err     = 1'b0;
            vision_valid = 1'b0;
            goal_reached = 1'b1;          // ★ 低有效：1 = 未到达（默认）
            lift_done    = 2'b00;
            lift_fault   = 2'b00;
        end
    endtask

    // 送一次有效测距读数（锁存），不推进拍摄
    task set_dist;
        input [15:0] mm;
        begin
            dist_mm    = mm;
            dist_valid = 1'b1;
        end
    endtask

    //------------------------------------------------------------------
    // 复位
    //------------------------------------------------------------------
    task do_reset;
        begin
            rst_n = 1'b0;
            clear_pulses;
            start = 0; estop = 0; manual_sel = 0;
            dist_mm = 0; vision_class = 0;
            repeat (4) step;
            rst_n = 1'b1;
            repeat (2) step;
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer cyc;
    integer s_avoid, s_lift;
    reg [2:0] cap_state;

    initial begin
        $fwrite(log_fd, "=== tb_main_fsm ===\n");
        $fwrite(log_fd, "  [info] CLK=%0d MS_TICKS=%0d\n", CLK_FREQ_HZ, MS_TICKS);
        $fwrite(log_fd, "  [info] 超时阈值: AVOID=%0d LIFT=%0d CROSS=%0d 拍\n",
                AVOID_TMO_TICKS, LIFT_TMO_TICKS, CROSS_TMO_TICKS);
        $fwrite(log_fd, "  [info] 距离阈值: STOP=%0d SLOW=%0d CLEAR=%0d mm\n",
                DIST_STOP_MM, DIST_SLOW_MM, DIST_CLEAR_MM);

        //==================================================================
        // 场景 1: 上电 + 启动
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 1: 上电 + 启动 ---\n");
        do_reset;
        check("1.1 上电后 state=IDLE",           state, S_IDLE);
        check("1.2 上电后 motor_enable=0",       motor_enable, 0);
        check("1.3 上电后 error_flag=0",         error_flag, 0);

        set_dist(2000); start = 1'b1;
        step;
        clear_pulses;
        check("1.4 start 后 state=AUTO",         state, S_AUTO);
        check("1.5 AUTO: motor_enable=1",        motor_enable, 1);
        check("1.6 AUTO 距离远: speed=FULL",     speed_scale, SPD_FULL);
        check("1.7 AUTO: avoid_active=0",        avoid_active, 0);

        //==================================================================
        // 场景 2: 减速 -> 停车避障
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 2: 减速 -> 停车避障 ---\n");
        set_dist(600); step; clear_pulses;
        check("2.1 距离 600mm: speed=FULL",      speed_scale, SPD_FULL);

        set_dist(400); step; clear_pulses;
        check("2.2 距离 400mm: speed=SLOW",      speed_scale, SPD_SLOW);
        check("2.3 距离 400mm: 仍为 AUTO",       state, S_AUTO);

        set_dist(150); step; clear_pulses;      // 锁存
        step;                                    // 读数生效 -> 状态转移
        check("2.4 距离 150mm: state=AVOID",     state, S_AVOID);
        check("2.5 深入障碍: motor_enable=0",    motor_enable, 0);
        check("2.6 深入障碍: speed=STOP",        speed_scale, SPD_STOP);
        check("2.7 AVOID: avoid_active=1",       avoid_active, 1);

        //==================================================================
        // 场景 3: 路径通畅后恢复导航
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 3: 路径通畅后恢复 ---\n");
        set_dist(900); step; clear_pulses;
        check("3.1 AVOID 中距离变远: 允许走",     motor_enable, 1);
        step;
        check("3.2 距离 900mm: 恢复 AUTO",        state, S_AUTO);
        check("3.3 恢复后: speed=FULL",           speed_scale, SPD_FULL);
        check("3.4 恢复后: avoid_active=0",       avoid_active, 0);

        //==================================================================
        // 场景 4: 低矮障碍 -> 四腿一起抬（模式 0）
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 4: 低矮障碍 -> 四腿一起抬(模式0) ---\n");
        set_dist(1000); step; clear_pulses;

        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step;
        clear_pulses;
        check("4.1 低矮障碍: state=LIFT",         state, S_LIFT);
        check("4.2 抬腿中: lift_phase=LP_UP",     lift_phase, LP_UP);
        check("4.3 抬腿中: lift_cmd=2'b11(前后一起抬)", lift_cmd, 2'b11);
        check("4.4 抬腿中: motor_enable=0",       motor_enable, 0);
        check("4.5 LIFT: lift_active=1",          lift_active, 1);

        // 只给一组到位：模式 0 要求两组都到位才推进
        lift_done = 2'b01; step; clear_pulses;
        check("4.6 只有前腿到位: 仍停在 LP_UP",    lift_phase, LP_UP);

        lift_done = 2'b11; step; clear_pulses;
        check("4.7 两组都到位: lift_phase=LP_CROSS", lift_phase, LP_CROSS);
        check("4.8 通过中: lift_cmd=2'b11(保持抬起)", lift_cmd, 2'b11);
        check("4.9 通过中: motor_enable=1",       motor_enable, 1);
        check("4.10 通过中: speed=SLOW",          speed_scale, SPD_SLOW);

        // 通过完成需要 path_clear **并且** t_cross >= CROSS_MIN_MS(1ms)
        //   只给 path_clear 不够 —— 这正是修掉"提前收腿"的地方
        set_dist(1200); step; clear_pulses;     // 锁存
        step;                                    // 读数生效
        check("4.11 ★前向通畅但时间不够: 仍不收腿", lift_phase, LP_CROSS);

        wait_ms(2);                              // 等过 CROSS_MIN_MS
        check("4.12 时间够了: lift_phase=LP_DOWN", lift_phase, LP_DOWN);
        check("4.13 收腿中: lift_cmd=2'b00",       lift_cmd, 2'b00);
        check("4.14 收腿中: motor_enable=0",      motor_enable, 0);

        // 放腿不需要限位反馈，等 LIFT_SETTLE_MS(1ms) 就回 AUTO
        wait_ms(6);
        check("4.15 收腿完成: state=AUTO",        state, S_AUTO);
        check("4.16 回到 AUTO: lift_cmd=2'b00",    lift_cmd, 2'b00);

        lift_done = 2'b00; step; clear_pulses;

        //==================================================================
        // 场景 4b: 台阶 -> 两腿分段（模式 1）
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 4b: 台阶 -> 两腿分段(模式1) ---\n");
        set_dist(1000); step; clear_pulses;

        vision_class = VIS_STEP; vision_valid = 1'b1;
        step;
        clear_pulses;
        check("4b.1 台阶: state=LIFT",             state, S_LIFT);
        check("4b.2 先抬前腿: lift_phase=LP_F_UP",  lift_phase, LP_F_UP);
        check("4b.3 只抬前腿: lift_cmd=2'b01",      lift_cmd, 2'b01);
        check("4b.4 抬腿中: motor_enable=0",        motor_enable, 0);

        // 模式 1 只看 lift_done[0]（前腿），后腿此时必然是 0
        lift_done = 2'b01; step; clear_pulses;
        check("4b.5 前腿到位: lift_phase=LP_F_CROSS", lift_phase, LP_F_CROSS);
        check("4b.6 前轮上位: lift_cmd=2'b01(保持前腿)", lift_cmd, 2'b01);
        check("4b.7 前轮上位: motor_enable=1",       motor_enable, 1);

        set_dist(1200); step; clear_pulses;
        step;
        wait_ms(2);
        check("4b.8 前轮上位完成: lift_phase=LP_R_UP", lift_phase, LP_R_UP);
        // ★ 关键：抬后腿时前腿必须保持抬起，否则车头会掉下来
        check("4b.9 ★抬后腿时前腿保持: lift_cmd=2'b11", lift_cmd, 2'b11);
        check("4b.10 抬后腿: motor_enable=0",        motor_enable, 0);

        lift_done = 2'b11; step; clear_pulses;
        check("4b.11 后腿到位: lift_phase=LP_R_CROSS", lift_phase, LP_R_CROSS);
        check("4b.12 后轮上位: motor_enable=1",       motor_enable, 1);

        wait_ms(2);
        check("4b.13 后轮上位完成: lift_phase=LP_DOWN", lift_phase, LP_DOWN);
        check("4b.14 都放下: lift_cmd=2'b00",         lift_cmd, 2'b00);

        wait_ms(6);
        check("4b.15 跨台阶完成: state=AUTO",         state, S_AUTO);

        lift_done = 2'b00; step; clear_pulses;

        //==================================================================
        // 场景 5: 急停（0 拍断使能 + 最高优先级 + 不误报异常）
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 5: 急停 ---\n");
        set_dist(2000); step; clear_pulses;
        check("5.1 急停前: motor_enable=1",      motor_enable, 1);

        // 关键：estop 拉起的【同一拍】就要看到使能断掉
        estop = 1'b1; step;
        check("5.2 ★estop 当拍: motor_enable=0", motor_enable, 0);
        check("5.3 estop 当拍: state=ESTOP",      state, S_ESTOP);
        check("5.4 人为急停: error_flag=0",       error_flag, 0);

        estop = 1'b0;
        repeat (5) step;
        check("5.5 急停解除未按 start: 仍 ESTOP",  state, S_ESTOP);
        check("5.6 急停解除未按 start: motor=0",   motor_enable, 0);
        check("5.7 ★急停解除后: error_flag 仍为 0", error_flag, 0);

        start = 1'b1; step; clear_pulses;
        check("5.8 按 start: state=IDLE",         state, S_IDLE);
        set_dist(2000); start = 1'b1; step; clear_pulses;
        check("5.9 再按 start: state=AUTO",       state, S_AUTO);

        //==================================================================
        // 场景 6: 避障超时 -> 安全停车
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 6: 避障超时 -> 安全停车 ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(100); step; clear_pulses;
        step;
        check("6.1 进入 AVOID",                   state, S_AVOID);

        // 保持障碍直到超时
        cyc = 0;
        dist_mm = 16'd100;
        while (state == S_AVOID && cyc < AVOID_TMO_TICKS + 1000) begin
            step;
            cyc = cyc + 1;
        end
        $fwrite(log_fd, "  [info] 避障持续 %0d 拍后离开 AVOID (阈值 %0d)\n",
                cyc, AVOID_TMO_TICKS);
        check("6.2 避障超时: state=ESTOP",        state, S_ESTOP);
        check("6.3 避障超时: error_flag=1",       error_flag, 1);
        check("6.4 避障超时: motor_enable=0",     motor_enable, 0);

        //==================================================================
        // 场景 7: 抬腿超时 -> 安全停车
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 7: 抬腿超时 -> 安全停车 ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("7.1 进入 LIFT/UP",                 state, S_LIFT);
        check("7.2 lift_phase=UP",                lift_phase, LP_UP);

        cyc = 0;
        while (state == S_LIFT && lift_phase == LP_UP &&
               cyc < LIFT_TMO_TICKS + 1000) begin
            step;
            cyc = cyc + 1;
        end
        $fwrite(log_fd, "  [info] 抬腿等待 %0d 拍后离开 LP_UP (阈值 %0d)\n",
                cyc, LIFT_TMO_TICKS);
        // ★ 2026-10-19 行为变了：超时 =「这次没跨过去」→ 收腿绕行（S_AVOID），
        //   不是 ESTOP 停车等人。连续失败 LIFT_RETRY_MAX 次才真停车（见场景 13）。
        check("7.3 抬腿超时: state=AVOID（降级绕行）", state, S_AVOID);
        check("7.4 降级后腿收回（lift_cmd=00）",        lift_cmd, 2'b00);
        check("7.5 降级不算故障（error_flag=0）",       error_flag, 0);

        //------------------------------------------------------------------
        // 场景 7b: 只有前腿故障时也要停车（模式 1 的分组故障通路）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 场景 7b: 模式1 前腿超时 -> 安全停车 ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;
        vision_class = VIS_STEP; vision_valid = 1'b1;
        step; clear_pulses;
        check("7b.1 进入 LIFT/LP_F_UP",           state, S_LIFT);
        check("7b.2 lift_phase=LP_F_UP",          lift_phase, LP_F_UP);

        cyc = 0;
        while (state == S_LIFT && lift_phase == LP_F_UP &&
               cyc < LIFT_TMO_TICKS + 1000) begin
            step;
            cyc = cyc + 1;
        end
        // ★ 模式 1 的前腿超时也走降级（同样是「没跨过去」，不是故障）
        check("7b.3 前腿超时: state=AVOID（降级绕行）", state, S_AVOID);
        check("7b.4 降级后腿收回（lift_cmd=00）",        lift_cmd, 2'b00);

        //==================================================================
        // 场景 7c: ★ 越障失败降级（2026-10-19）
        //
        //   验证三件事：
        //     1) 锁定期内再收到 0x04 **不会**又去抬腿（否则会无限循环）
        //     2) 每次失败都进 AVOID（当普通障碍物绕开），腿收回
        //     3) 连续第 3 次失败 -> 才真的进 ESTOP
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 7c: 连续失败 3 次 -> ESTOP ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;

        // ---- 第 1 次：抬腿 -> 超时 -> AVOID ----
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("7c.1 第1次进 LIFT",                 state, S_LIFT);
        cyc = 0;
        while (state == S_LIFT && cyc < LIFT_TMO_TICKS + 1000) begin
            step; cyc = cyc + 1;
        end
        check("7c.2 第1次超时 -> AVOID",           state, S_AVOID);

        // ---- 锁定期内再发 0x04：必须被门控掉 ----
        //   ⚠️ 这里不能断言"仍停在 AVOID"：AVOID 在 path_clear 时会正常退回 AUTO
        //      （t_avoid 已经不为 0，那与锁定无关）。真正要验的是：
        //      **收到 0x04 也没有进 LIFT**。
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("7c.3 锁定期内发 0x04 进不了 LIFT",  state, S_AUTO);

        // ---- 等锁定过期（但仍未到"干净清零"的时间）-> 第 2 次 ----
        wait_ms(LIFT_LOCKOUT_MS + 2);
        set_dist(2000); step; clear_pulses;
        check("7c.4 锁定过后回 AUTO",              state, S_AUTO);

        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("7c.5 第2次进 LIFT",                 state, S_LIFT);
        cyc = 0;
        while (state == S_LIFT && cyc < LIFT_TMO_TICKS + 1000) begin
            step; cyc = cyc + 1;
        end
        check("7c.6 第2次超时 -> AVOID",           state, S_AVOID);

        // ---- 第 3 次：应该直接 ESTOP ----
        wait_ms(LIFT_LOCKOUT_MS + 2);
        set_dist(2000); step; clear_pulses;
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("7c.7 第3次进 LIFT",                 state, S_LIFT);
        cyc = 0;
        while (state == S_LIFT && cyc < LIFT_TMO_TICKS + 1000) begin
            step; cyc = cyc + 1;
        end
        check("7c.8 第3次失败 -> ESTOP",           state, S_ESTOP);
        check("7c.9 第3次失败 error_flag=1",       error_flag, 1);

        //==================================================================
        // 场景 8: 舵机故障 -> 立即安全停车
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 8: 舵机故障 -> 立即安全停车 ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        lift_fault = 2'b11; step; clear_pulses;
        check("8.1 lift_fault: state=ESTOP",      state, S_ESTOP);
        check("8.2 lift_fault: error_flag=1",     error_flag, 1);

        //------------------------------------------------------------------
        // 场景 8b: 只报一路故障也要停（模式 0 用 |lift_fault）
        //------------------------------------------------------------------
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        lift_fault = 2'b01; step; clear_pulses;   // 只有前腿报故障
        check("8b.1 单路故障也要停: state=ESTOP", state, S_ESTOP);
        check("8b.2 单路故障: error_flag=1",      error_flag, 1);

        //==================================================================
        // 场景 9: 传感器失效 fail-safe（限速而非停车）
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 9: 传感器失效 fail-safe ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        check("9.1 正常: speed=FULL",             speed_scale, SPD_FULL);

        dist_err = 1'b1; step; clear_pulses;
        check("9.2 测距失效: 仍为 AUTO (不卡死)",  state, S_AUTO);
        check("9.3 测距失效: speed=SLOW (fail-safe)", speed_scale, SPD_SLOW);
        check("9.4 测距失效: motor_enable=1",     motor_enable, 1);

        set_dist(1500); step; clear_pulses;
        check("9.5 恢复有效读数: speed=FULL",      speed_scale, SPD_FULL);

        //==================================================================
        // 场景 10: 到达终点 + 手动模式
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 10: 到达终点 + 手动模式 ---\n");
        do_reset;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        // ★ 低有效：0 = 到达终点（开关接地）
        goal_reached = 1'b0; step; clear_pulses;
        check("10.1 goal_reached: state=GOAL",    state, S_GOAL);
        check("10.2 GOAL: motor_enable=0",        motor_enable, 0);
        goal_reached = 1'b1;              // 松开终点开关，再按 start 重新出发
        start = 1'b1; step; clear_pulses;
        check("10.3 GOAL 后按 start: AUTO",       state, S_AUTO);

        do_reset;
        manual_sel = 1'b1; step;
        check("10.4 manual_sel: state=MANUAL",    state, S_MANUAL);
        check("10.5 MANUAL: motor_enable=1",      motor_enable, 1);
        check("10.6 MANUAL: speed=SLOW",          speed_scale, SPD_SLOW);
        manual_sel = 1'b0; step;
        check("10.7 松开 manual_sel: IDLE",       state, S_IDLE);

        //==================================================================
        // 11) 视觉类别表：0x02 障碍物 -> AVOID
        //
        //   背景：ARM 侧 camera_vision.py 固定发 0x02（障碍物）。
        //   本模块以前只实现了 0x04（低矮障碍），导致 ARM 报障碍物时
        //   FPGA 毫无反应 —— 这是联调时「车看到障碍物不躲」的直接原因。
        //
        //   ⚠️ 还验证了「保持窗口」：ARM 是 10Hz 发的，vision_valid 是
        //      单拍脉冲。没有保持的话 AVOID 只持续 1 拍就退出，等于没避障。
        //==================================================================
        $fwrite(log_fd, "\n--- 11. 视觉类别表（0x02 障碍物）---\n");

        // ---- 11a: 0x02 -> AVOID ----
        do_reset;
        set_dist(2000); start = 1'b1;
        step;
        clear_pulses;
        check("11a.1 启动后 AUTO", state, S_AUTO);

        set_dist(2000);                 // TOF 通畅
        step;
        vision_class = VIS_OBST; vision_valid = 1'b1;
        step;
        vision_valid = 1'b0;
        step;
        check("11a.2 收到 0x02 -> AVOID",        state, S_AVOID);
        check("11a.3 视觉触发: motor_enable=1",  motor_enable, 1);
        check("11a.4 视觉触发: speed=SLOW",        speed_scale, SPD_SLOW);

        // ---- 11b: 保持窗口内仍在 AVOID（防 10Hz 横跳）----
        repeat (10) step;
        check("11b.1 保持窗口内: 仍在 AVOID",   state, S_AVOID);

        // ---- 11c: 等过保持窗口(VIS_HOLD_MS=2ms) -> 自动回 AUTO ----
        repeat (VIS_HOLD_MS * MS_TICKS + 100) step;
        check("11c.1 保持窗口过后 -> AUTO",     state, S_AUTO);

        // ---- 11d: 未知类别不应触发任何动作 ----
        vision_class = 8'h07; vision_valid = 1'b1;
        step;
        vision_valid = 1'b0;
        step;
        check("11d.1 未知类别 0x07 -> 保持 AUTO", state, S_AUTO);

        //==================================================================
        // 场景 12: ★ 抬腿超时可运行时配置（CONFIG 帧 0x11，2026-10-18 新增）
        //
        //   原来 LIFT_TIMEOUT_MS 是编译期参数；改成端口后可以按机械实际情况调。
        //   这里验证"改端口真的改变行为"：
        //     - 阈值设成 20ms，走到 10ms 时**不能**超时
        //     - 走到 25ms 时**必须**超时进 ESTOP
        //   如果端口没接对（比如 RTL 还在用编译期参数 5ms），10ms 那条就会挂。
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 12: 抬腿超时运行时可变（0x11）---\n");
        do_reset;
        lift_timeout_ms = 16'd20;
        set_dist(2000); start = 1'b1; step; clear_pulses;
        set_dist(1000); step; clear_pulses;
        vision_class = VIS_LOW_OBST; vision_valid = 1'b1;
        step; clear_pulses;
        check("12.1 进入 LIFT/UP",                     state, S_LIFT);
        check("12.2 lift_phase=UP",                    lift_phase, LP_UP);

        // 走到约 10ms（阈值 20ms 的一半）—— 必须还在等
        cyc = 0;
        while (cyc < 10 * MS_TICKS && state == S_LIFT) begin
            step; cyc = cyc + 1;
        end
        check("12.3 10ms 时仍在 LIFT（没提前超时）",   state, S_LIFT);
        check("12.4 10ms 时 lift_phase 仍是 UP",       lift_phase, LP_UP);

        // 再走到约 25ms —— 必须已经超时
        cyc = 0;
        while (cyc < 15 * MS_TICKS && state == S_LIFT) begin
            step; cyc = cyc + 1;
        end
        // ★ 2026-10-19：超时的结果从 ESTOP 改成 AVOID（降级绕行）。
        //   这一条仍然在验"运行时端口真的改变了超时时刻" ——
        //   10ms 没超时、25ms 超时了 ✓ 只是超时后的去向变了。
        check("12.5 25ms 后已超时: state=AVOID",       state, S_AVOID);
        check("12.6 超时后腿收回（lift_cmd=00）",      lift_cmd, 2'b00);
        check("12.7 超时后不算故障（error_flag=0）",   error_flag, 0);

        lift_timeout_ms = 16'd0;   // 恢复默认（0 = 用编译期默认值）

        // 汇总
        //==================================================================
        $fwrite(log_fd, "\n=== tb_main_fsm: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 仿真上限（超时保护）
    //
    // ⚠️ 这里**不能**写成 #5_000_000_000：
    //    Verilog 的无尺寸字面量默认是 **32 位**，上限 4294967295。
    //    5e9 超了，vlog 直接报
    //        (vlog-13008) Unsized literal value 5000000000 exceeds
    //        default size of literals which is 32-bit
    //    然后**拒绝编译这个文件**。而 compile_all.do 会继续往下走，
    //    于是 tb 加载时找不到 main_fsm，只报一句无任何细节的
    //    "Error loading design"，极难查。
    //
    //    本测试实际只需约 5ms 仿真时间，100ms 有 20 倍余量。
    //    若要更长上限，必须写成带尺寸的：#64'd5_000_000_000
    //------------------------------------------------------------------
    initial begin
        #100_000_000;                    // 100ms 仿真时间上限
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
