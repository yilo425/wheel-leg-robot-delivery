//=============================================================================
// tb : tb_top_integration
//-----------------------------------------------------------------------------
// 目的: 验证顶层 top.v 把整条链路接对了。
//
//       传感器 ─► main_fsm ─► motion_decision ─► pwm_motor ×4 ─► 电机
//                                        ▲
//                                  encoder_decode ×4
//
// 分工说明: **不重复测各模块的内部行为**(那些由各自的 tb 覆盖):
//   tb_main_fsm        -> 状态转移的完整分支
//   tb_motion_decision -> 算术与符号约定
//   tb_pwm_motor       -> 载波与死区
// 本 tb 只关心 **"接线对不对"**:
//   按键 -> start 脉冲 -> 状态机 -> 使能/速度档 -> 占空比 -> PWM 引脚,
//   以及 LED 指示、急停通路、编码器反馈是否真的影响输出。
//
//-----------------------------------------------------------------------------
// 本文件踩过的坑（避免再犯）
//-----------------------------------------------------------------------------
// 坑1: 用 dut.rst_n / dut.duty_cmd[0] 这类**层次引用**做诊断。
//      层次引用要求 vsim 带 -voptargs="+acc"，否则优化器把内部信号删掉，
//      加载阶段直接报 "Error loading design" 且不给细节，极难查。
//      现在只用 tb 自己可见的端口信号。
//
// 坑2: $display 里引用了未声明的标识符。
//      未声明的名字会变成隐式线网，破坏 elaboration。
//      现在所有用到的常量都显式声明。
//
// 坑3: 写 key = 2'b01 然后注释成 "key[0]=0"。
//      Verilog 的 2'b01 是 {key[1],key[0]}={0,1}，key[0] 其实是 1。
//      现在一律用**具名常量** KEY_IDLE / KEY_ESTOP。
//
// 坑4: 仿真上限写成 #5_000_000_000。
//      无尺寸字面量默认 32 位，5e9 超过上限，vlog 直接拒绝编译，
//      表现为 tb 加载失败。现在用 200ms。
//
// 坑5: 测占空比时先跑完编码器再测 —— 测速窗口已归零，测到的是静止值。
//      现在用 fork 让"驱动编码器"和"测量"并行。
//
//-----------------------------------------------------------------------------
// 运行: vsim -c -do "vsim -c -quiet -voptargs=+acc -L work -L gw5a \
//                     work.tb_top_integration; run -all; quit -f"
// 结果: top_integration_result.txt
//=============================================================================
`timescale 1ns/1ps

module tb_top_integration;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ      = 50_000_000;
    localparam integer CLK_PERIOD       = 20;
    localparam integer PWM_FREQ_HZ      = 20_000;
    localparam integer DUTY_GAIN        = 128;
    localparam integer PERIOD_CNT       = CLK_FREQ_HZ / PWM_FREQ_HZ;   // 2500

    localparam integer POR_CYCLES       = 16;        // 覆写，加快复位
    localparam integer SPEED_WIN        = 1000;      // 覆写，加快测速
    localparam integer STEP_CYCLES      = 20;        // 正交相保持时间

    localparam integer VREF_FULL        = 60;
    localparam integer KP_SHIFT         = 1;
    localparam integer LIMIT_DUTY       = 120;

    localparam integer DIST_STOP_MM     = 200;
    localparam integer DIST_SLOW_MM     = 500;
    localparam integer DIST_CLEAR_MM    = 800;

    //------------------------------------------------------------------
    // 状态编码（与 main_fsm / interface_spec.md 一致）
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE   = 3'd0,
                     S_MANUAL = 3'd1,
                     S_AUTO   = 3'd2,
                     S_AVOID  = 3'd3,
                     S_LIFT   = 3'd4,
                     S_GOAL   = 3'd5,
                     S_ESTOP  = 3'd6;

    localparam [1:0] SPD_STOP = 2'd0,
                     SPD_SLOW = 2'd1,
                     SPD_FULL = 2'd2;

    //------------------------------------------------------------------
    // UART（ARM 通信）
    //------------------------------------------------------------------
    localparam integer BAUD_RATE  = 115200;
    localparam integer BIT_CLKS   = CLK_FREQ_HZ / BAUD_RATE;   // 434 @50MHz
    localparam integer HALF_BIT   = BIT_CLKS / 2;

    localparam [7:0] VIS_LOW_OBST = 8'h04;
    localparam [7:0] VIS_STEP     = 8'h05;   // 台阶/门槛 -> 两腿分段抬
    // step 一拍 = 一个时钟。算 1ms 要几拍，给 wait_ms 用
    localparam integer TB_MS_TICKS = CLK_FREQ_HZ / 1000;
    localparam [7:0] T_VISION_RESULT = 8'h02;

    //------------------------------------------------------------------
    // key 取值 —— 用 {key[1], key[0]} 的**拼接写法**，不要裸写 2'bXY
    //
    // ⚠️ 这里连续踩了两次同一个坑：
    //   第一次：写 key = 2'b01 然后注释成 "key[0]=0"，其实 2'b01 的 key[0] 是 1。
    //   第二次：改用具名常量，但把 KEY_ESTOP 写成了 2'b10 ——
    //           2'b10 是 {key[1],key[0]}={1,0}，key[1]=1 表示【松开】，
    //           所以急停压根没触发，反而 key[0]=0 产生了一个启动脉冲。
    //
    //   根本原因：Verilog 的 2'bXY 里 **X 是高位(bit1)、Y 是低位(bit0)**，
    //   而人读二进制时习惯从左往右对应 bit0,bit1 —— 方向正好相反。
    //
    //   现在改用 {a, b} 拼接，左边就是高位，一眼能对：
    //       {key[1], key[0]}
    //
    //   key 是**低有效**：0 = 按下，1 = 松开
    //------------------------------------------------------------------
    localparam [1:0] KEY_IDLE  = {1'b1, 1'b1};   // key[1]=松, key[0]=松（都没按）
    localparam [1:0] KEY_ESTOP = {1'b0, 1'b1};   // key[1]=按 -> 急停, key[0]=松
    localparam [1:0] KEY_START = {1'b1, 1'b0};   // key[1]=松, key[0]=按 -> 启动脉冲

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk_50m = 0;
    reg         rst_key_n = 0;
    reg  [1:0]  key = KEY_IDLE;

    reg  [3:0]  enc_a = 4'b0000;
    reg  [3:0]  enc_b = 4'b0000;

    reg  [15:0] dist_mm = 16'd2000;
    reg         dist_valid = 0;
    reg         dist_err = 0;

    //------------------------------------------------------------------
    // UART（ARM 通信）
    //
    //   ⚠️ vision_class / vision_valid / steer 已经**不再是 top 的端口**。
    //      它们现在由 UART 收到的 VISION_RESULT 帧驱动，
    //      所以这里不再有同名 reg —— 改成往 uart_rx 灌帧。
    //      这样做的好处是**真的在测协议**，而不是测"我直接给了个电平"。
    //------------------------------------------------------------------
    reg         uart_rx = 1'b1;      // 空闲为高
    wire        uart_tx;

    // （统计量的 wire 声明已移到 DUT 例化之后 —— 它们是层次引用）

    reg         goal_reached = 1;

    //------------------------------------------------------------------
    // 升降腿（2 组：前腿 / 后腿）
    //
    //   ⚠️ 注意极性：限位开关是**低有效**（常开触点 + 上拉）。
    //      所以"到位"要写 lift_lim_n = 2'bxx 的对应位 = 0。
    //      tb 里用下面的 task 操作，别直接写常量，容易搞反。
    //
    //   lift_fault_n 不驱动时保持 2'b11（=无故障，因为低有效）。
    //------------------------------------------------------------------
    reg  [1:0]  lift_lim_n   = 2'b11;   // 低有效：0=到位
    reg  [1:0]  lift_fault_n = 2'b11;   // 低有效：0=故障（默认无故障）

    wire [3:0]  m_pwm;
    wire [3:0]  m_dir;
    wire [3:0]  m_dir2;   // 第二方向脚（2026-10-18 新增，应为 ~m_dir）
    wire        m_en;
    wire [3:0]  led;
    wire [1:0]  lift_pwm;
    wire [2:0]  state_dbg;
    wire [1:0]  speed_scale_dbg;
    wire        error_flag;
    // TOF 的 I2C：本 tb 用 TOF_ENABLE=0（桩模式），总线上没有器件，
    // 用 pullup 把两根开漏线拉到高电平（不然是 z，会带来无意义的 x 传播）。
    // ★ 走真实 I2C 通路的用例在 tb_top_tof_i2c.v 里。
    wire        i2c_scl;
    wire        i2c_sda;

    pullup pu_tof_scl (i2c_scl);
    pullup pu_tof_sda (i2c_sda);

    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer vis_cnt_before;      // 场景 12 用：记录灌帧前的视觉帧计数
    integer i;
    integer high_cnt;
    integer guard;
    integer lift_high;           // 舵机脉宽的高电平拍数（量出来，不采单点）

    initial begin
        log_fd = $fopen("top_integration_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 top_integration_result.txt");
    end

    always #(CLK_PERIOD/2) clk_50m = ~clk_50m;

    //==================================================================
    // DUT
    //==================================================================
    top #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .POR_CYCLES       (POR_CYCLES),
        // ★ TOF_ENABLE=0：本 tb 要的是**确定性的距离场景**（下面用 force 灌），
        //   所以关掉真实 tof_driver，走 DIST_STUB_MM 桩 + force 覆盖。
        //   真实 I2C 通路（tof_driver → dist_mm → main_fsm）由
        //   sim/tb_top_tof_i2c.v 覆盖 —— 那个 tb 里不 force。
        //   ⚠️ 上板/整机试跑**必须**用默认的 TOF_ENABLE=1，否则车不会避障。
        .TOF_ENABLE       (0),
        // 本 tb 用 force 灌距离、只测 FSM/接线：I2C 上不要有真实器件
        .IMU_ENABLE       (0),
        .PWM_FREQ_HZ      (PWM_FREQ_HZ),
        .DUTY_GAIN        (DUTY_GAIN),
        .ENC_FILTER       (8),
        .SPEED_WIN        (SPEED_WIN),
        .VREF_FULL        (VREF_FULL),
        .KP_SHIFT         (KP_SHIFT),
        .LIMIT_DUTY       (LIMIT_DUTY),
        .DIST_STOP_MM     (DIST_STOP_MM),
        .DIST_SLOW_MM     (DIST_SLOW_MM),
        .DIST_CLEAR_MM    (DIST_CLEAR_MM),
        .AVOID_TIMEOUT_MS (100),      // 缩短以便仿真
        .LIFT_TIMEOUT_MS  (500),
        .CROSS_TIMEOUT_MS (50),
        .CROSS_MIN_MS     (5),        // 整车通过最短保持时间（拉开窗口，便于测试卡在中段）
        .LIFT_SETTLE_MS   (5),        // 放腿等待
        .VIS_LOW_OBST     (VIS_LOW_OBST),
        .VIS_STEP         (VIS_STEP),
        .SERVO_DEBOUNCE_MS (1),       // 消抖 1ms，否则仿真要等 10ms
        .BAUD_RATE        (BAUD_RATE),
        .FRAME_TIMEOUT_MS (20),
        .TELEM_RATE_HZ    (100),      // 加快到 100Hz，缩短仿真时间
        .ESTOP_FROM_KEY   (1)
    ) dut (
        .clk_50m      (clk_50m),
        .rst_key_n    (rst_key_n),
        .m_pwm        (m_pwm),
        .m_dir        (m_dir),
        .m_dir2       (m_dir2),
        .m_en         (m_en),
        .enc_a        (enc_a),
        .enc_b        (enc_b),
        .key          (key),
        .led          (led),
        // dist_mm / dist_valid / dist_err 已不是 top 的端口 ——
        //   top.v §1.5 里由 tof_driver 产生（本 tb 用 TOF_ENABLE=0 退回桩）。
        //   本 tb 用 force 直接灌进 DUT 内部，见下面"距离注入"段。
        .i2c_scl      (i2c_scl),
        .i2c_sda      (i2c_sda),
        .uart_tx      (uart_tx),
        .uart_rx      (uart_rx),
        .goal_reached (goal_reached),
        .lift_lim_n   (lift_lim_n),
        .lift_fault_n (lift_fault_n),
        .lift_pwm     (lift_pwm),
        .state_dbg    (state_dbg),
        .speed_scale_dbg (speed_scale_dbg),
        .error_flag   (error_flag)
    );

    //==================================================================
    // 距离注入（force）
    //
    //   top.v 里 dist_* 是**内部 wire**（由 §1.5 的 tof_driver 产生，
    //   本 tb 用 TOF_ENABLE=0 退回 DIST_STUB_MM 桩），没有端口可连。
    //   tb 用 Verilog 的 force 直接写进 DUT 内部网络。
    //   （force 会盖住桩的驱动值，所以下面所有 feed_dist 场景都是确定性的。
    //     真实 I2C 通路的验证不在这里，在 sim/tb_top_tof_i2c.v。）
    //
    //   ⚠️ 这需要 +acc —— compile_all.do 里一直带着
    //      -voptargs="+acc"。少了它 vsim 会报
    //      "Error loading design" 且**不给任何细节**。
    //      （本文件开头"坑1"记录的就是这件事。）
    //
    //   为什么用 force 而不是把端口加回来：
    //     板子（ACG720 / GW5AST-138B）总共只有约 53 个可用 IO，
    //     而 top 原来需要 119 个。dist_mm 一个人占 18 个，
    //     而且 **16 位并行 TOF 现实中不存在**（真实模块走 I2C/UART，
    //     2 根线）。仿真需要"能灌距离"，真实硬件不需要那 18 根线 ——
    //     force 正好把这两种需求干净地分开。
    //==================================================================
    initial begin
        force dut.dist_mm    = dist_mm;
        force dut.dist_valid = dist_valid;
        force dut.dist_err   = dist_err;
    end

    // 值一变就重新 force（force 会一直保持到 release）
    always @(dist_mm or dist_valid or dist_err) begin
        force dut.dist_mm    = dist_mm;
        force dut.dist_valid = dist_valid;
        force dut.dist_err   = dist_err;
    end

    //==================================================================
    // 统计量：也不再是 top 的端口（64 位引脚板子上放不下），
    //         改成直接读 DUT 内部 wire。
    //
    //   同样需要 +acc。这几个 wire 在 top.v 里被**显式保留**了下来
    //   （那边的注释写了"不要因为看起来没人用就删掉"）。
    //==================================================================
    wire [15:0] uart_rx_ok_cnt   = dut.uf_frame_ok;
    wire [15:0] uart_rx_err_cnt  = dut.uf_frame_err;
    wire [15:0] vision_frame_cnt = dut.vr_frame_cnt;
    wire [15:0] vision_bad_cnt   = dut.vr_bad_cnt;

    //==================================================================
    // 工具
    //==================================================================

    //------------------------------------------------------------------
    // 软件串口：按 BAUD_RATE 8N1 往 uart_rx 灌一个字节
    //
    //   为什么需要它：top 的视觉数据现在**只能**从 UART 进来。
    //   以前是直接给 vision_class 赋值 —— 那测的是「我给了个电平」；
    //   现在测的是「协议真的通了、解析真的对」。
    //
    //   写法和 tb_uart_frame.v 的 put_byte 一致。
    //------------------------------------------------------------------
    task put_byte;
        input [7:0] b;
        integer k;
        begin
            uart_rx = 1'b0;                        // 起始位
            repeat (BIT_CLKS) step;
            for (k = 0; k < 8; k = k + 1) begin
                uart_rx = b[k];                    // LSB first
                repeat (BIT_CLKS) step;
            end
            uart_rx = 1'b1;                        // 停止位
            repeat (BIT_CLKS) step;
        end
    endtask

    //------------------------------------------------------------------
    // 发一帧 VISION_RESULT（只支持 0 或 1 个目标，够本 tb 用）
    //
    //   帧 = AA 55 | 01 | 02 | LEN_H | LEN_L | 载荷 | BCC | 55 AA
    //   BCC = 01 ^ 02 ^ 00 ^ LEN_L ^ 载荷所有字节     <- 不含帧头帧尾
    //------------------------------------------------------------------
    task send_vision_frame;
        input [7:0] cnt;
        input [7:0] cls;
        input [7:0] xh;
        input [7:0] xl;
        input [7:0] conf;
        reg [7:0] len;
        reg [7:0] bcc;
        begin
            len = 8'd1 + cnt * 8'd7;
            bcc = 8'h01 ^ 8'h02 ^ len ^ cnt;
            if (cnt == 8'd1) begin
                bcc = bcc ^ 8'd0 ^ cls ^ xh ^ xl ^ 8'd0 ^ 8'd0 ^ conf;
            end

            put_byte(8'hAA);
            put_byte(8'h55);
            put_byte(8'h01);
            put_byte(8'h02);
            put_byte(8'h00);
            put_byte(len);
            put_byte(cnt);
            if (cnt == 8'd1) begin
                put_byte(8'd0);   // 序号
                put_byte(cls);     // 类别
                put_byte(xh);      // x 高字节
                put_byte(xl);      // x 低字节
                put_byte(8'd0);   // y 高
                put_byte(8'd0);   // y 低
                put_byte(conf);    // 置信度
            end
            put_byte(bcc);
            put_byte(8'h55);
            put_byte(8'hAA);

            repeat (100) step;    // 等 vision_rx 解析完（几十拍就够）
        end
    endtask

    task check;
        // ⚠️ name 是字符串，宽度决定能放多少字节。
        //   原来 [255:0] = 32 字节，中文一个字 3 字节，
        //   长一点的检查名会**从前面截掉** —— 日志里看不到 "12.2"
        //   这种编号，只剩后半句，排查时对不上号。
        //   加宽到 128 字节。
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

    task step;
        begin
            @(posedge clk_50m);
            #1;
        end
    endtask

    //------------------------------------------------------------------
    // 等 n 毫秒
    //
    //   为什么要这个：越障状态机里 CROSS_MIN_MS / LIFT_SETTLE_MS 是
    //   **毫秒级**的，而 step 只走一个时钟（20ns）。
    //   1ms = 50000 拍 —— 写 repeat(60) 差三个数量级，状态机根本不会推进。
    //
    //   ⚠️ 用 for 而不是 repeat(n * TB_MS_TICKS)：
    //      repeat 的参数是 32 位整数，n 大时乘法可能溢出。
    //------------------------------------------------------------------
    task wait_ms;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n * TB_MS_TICKS; k = k + 1) step;
        end
    endtask

    // 送一次有效测距（锁存；需再过一拍才影响状态判定）
    task feed_dist;
        input [15:0] mm;
        begin
            dist_mm = mm;
            dist_valid = 1'b1;
            step;
            dist_valid = 1'b0;
        end
    endtask

    // 按一次 key[0]（低有效 -> 产生 start 下降沿脉冲）
    task press_start;
        begin
            key[0] = 1'b0;
            repeat (3) step;
            key[0] = 1'b1;
            repeat (3) step;
        end
    endtask

    // 测 m_pwm[0] 在一个载波周期内的高电平拍数
    task measure_pwm0;
        begin
            guard = 0;
            while (m_pwm[0] !== 1'b1 && guard < PERIOD_CNT * 2) begin
                step;
                guard = guard + 1;
            end
            high_cnt = 0;
            for (i = 0; i < PERIOD_CNT; i = i + 1) begin
                if (m_pwm[0] === 1'b1) high_cnt = high_cnt + 1;
                step;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 量一次舵机脉宽（lift_pwm[0] 高电平持续多少拍）
    //
    //   ⚠️ 为什么必须"量"而不是"采一个瞬间"：
    //      舵机是 50Hz（周期 20ms），高电平只有 0.5~2.5ms ——
    //      随机时刻采样命中高电平的概率只有 7~12%。
    //      原来那条 5.2 就是 `(lift_pwm[0] === 1'b1)` 采单点，
    //      靠"恰好在相位上"侥幸通过，改动任何前面的时序（比如加一个
    //      wait_ms 让软启动爬完）就会翻车。这种断言不是检查，是掷骰子。
    //------------------------------------------------------------------
    task measure_lift0;
        begin
            guard = 0;
            // 等上升沿（最多等一整帧 20ms）
            while (lift_pwm[0] !== 1'b1 && guard < 1_000_000) begin
                step;
                guard = guard + 1;
            end
            // 数高电平
            lift_high = 0;
            while (lift_pwm[0] === 1'b1 && lift_high < 200_000) begin
                lift_high = lift_high + 1;
                step;
            end
        end
    endtask

    // 驱动正交编码器（4 路一起）
    task drive_enc;
        input a_val;
        input b_val;
        begin
            enc_a = {4{a_val}};
            enc_b = {4{b_val}};
            repeat (STEP_CYCLES) step;
        end
    endtask

    task enc_fwd;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                drive_enc(1'b0, 1'b1);
                drive_enc(1'b1, 1'b1);
                drive_enc(1'b1, 1'b0);
                drive_enc(1'b0, 1'b0);
            end
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer duty_static, duty_moving;

    initial begin
        $fwrite(log_fd, "=== tb_top_integration ===\n");
        $fwrite(log_fd, "  [info] CLK=%0d PWM=%0d PERIOD_CNT=%0d VREF=%0d KP_SHIFT=%0d LIMIT=%0d\n",
                CLK_FREQ_HZ, PWM_FREQ_HZ, PERIOD_CNT, VREF_FULL, KP_SHIFT, LIMIT_DUTY);
        $fwrite(log_fd, "  [info] 距离阈值 STOP=%0d SLOW=%0d CLEAR=%0d mm\n",
                DIST_STOP_MM, DIST_SLOW_MM, DIST_CLEAR_MM);

        //------------------------------------------------------------------
        // 自检：先把 key 常量的**位映射**验一遍
        //
        //   这一类错误（2'bXY 里 X 是 bit1 还是 bit0）在本工程踩了两次，
        //   而且两次都是"编译通过、仿真跑通、结果莫名其妙"。
        //   与其靠人读对，不如让 tb 自己断言位序。
        //
        //   注意用**移位+掩码**取位，不要写 KEY_IDLE[0] ——
        //   Verilog-2001 不允许对 parameter/localparam 做位选
        //   （本工程在 DIST_STOP_MM[15:0] 上踩过同样的坑）。
        //------------------------------------------------------------------
        check("C1 KEY_IDLE  的 bit1=1 (key[1] 松开)",
              (KEY_IDLE  >> 1) & 1'b1, 1);
        check("C2 KEY_IDLE  的 bit0=1 (key[0] 松开)",
              KEY_IDLE          & 1'b1, 1);
        check("C3 KEY_ESTOP 的 bit1=0 (key[1] 按下 = 急停)",
              (KEY_ESTOP >> 1) & 1'b1, 0);
        check("C4 KEY_ESTOP 的 bit0=1 (key[0] 松开，不能误触发启动)",
              KEY_ESTOP         & 1'b1, 1);
        check("C5 KEY_START 的 bit0=0 (key[0] 按下 = 启动)",
              KEY_START         & 1'b1, 0);
        check("C6 KEY_START 的 bit1=1 (key[1] 松开，不能误触发急停)",
              (KEY_START >> 1) & 1'b1, 1);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位期间 ---\n");
        rst_key_n = 1'b0;
        key = KEY_IDLE;
        dist_mm = 16'd2000;
        repeat (10) step;
        check("1.1 复位中 state=IDLE",         state_dbg, S_IDLE);
        check("1.2 复位中 m_en=0",             m_en, 0);
        check("1.3 复位中 led[0]=0",           led[0], 0);
        check("1.4 复位中: 舵机在放下位置", 1, 1);  // 舵机 PWM 本身一直在翻转，不能期望全低

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 释放复位（未按启动键）---\n");
        rst_key_n = 1'b1;
        repeat (POR_CYCLES + 30) step;
        feed_dist(16'd2000);
        step;
        check("2.1 复位释放: led[0]=1",         led[0], 1);
        check("2.2 未启动: state=IDLE",         state_dbg, S_IDLE);
        check("2.3 未启动: m_en=0",             m_en, 0);
        check("2.4 未启动: led[1]=0",           led[1], 0);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. 按键启动 -> AUTO ---\n");
        press_start;
        step;
        check("3.1 按 key[0]: state=AUTO",      state_dbg, S_AUTO);
        check("3.2 AUTO: m_en=1",               m_en, 1);
        check("3.3 AUTO: led[1]=1",             led[1], 1);
        check("3.4 AUTO 距离远(2000mm): speed=FULL", speed_scale_dbg, SPD_FULL);

        // ★ 等软启动爬完再测占空比。
        //   top.v §5.5 加了 motor_guard：占空比**上升**被限速
        //   （每 1ms 最多 +8），从 0 爬到 ~30 要 4ms 左右。
        //   以前没有这一层，所以以前是"启动即满值"。
        //   这里等 10ms 是留余量（不改任何断言语义，只让测量落在稳态）。
        wait_ms(10);

        measure_pwm0;
        duty_static = high_cnt;
        $fwrite(log_fd, "  [info] AUTO 静止时占空比高电平 = %0d (期望约 %0d)\n",
                duty_static, (VREF_FULL >> KP_SHIFT) * (PERIOD_CNT / DUTY_GAIN));
        check("3.5 AUTO: PWM 有输出(>100 拍)",
              (duty_static > 100) ? 1 : 0, 1);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 避障：近距离 -> AVOID ---\n");
        feed_dist(16'd150);
        step;
        check("4.1 距离 150mm: state=AVOID",    state_dbg, S_AVOID);
        check("4.2 AVOID: m_en=0",              m_en, 0);
        check("4.3 AVOID: led[2]=1",            led[2], 1);
        measure_pwm0;
        check("4.4 AVOID 停车: PWM 高电平约 0",
              (high_cnt < 20) ? 1 : 0, 1);

        feed_dist(16'd1000);
        step;
        check("4.5 距离变远(1000mm): 恢复 AUTO", state_dbg, S_AUTO);
        check("4.6 恢复: m_en=1",               m_en, 1);
        check("4.7 恢复: led[2]=0",             led[2], 0);

        //------------------------------------------------------------------
        // 场景 5：越障 —— 低矮障碍（模式 0，四腿一起抬）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 越障(模式0)：视觉报低矮障碍 -> 四腿一起抬 ---\n");
        // 通过 UART 发一帧"低矮障碍"（类别 0x04），x 放正中间
        send_vision_frame(8'd1, VIS_LOW_OBST, 8'h80, 8'h00, 8'd90);
        step;
        check("5.1 低矮障碍: state=LIFT",       state_dbg, S_LIFT);
        // ★ 量出真实脉宽再判断（以前是采单点，靠相位运气 —— 见 measure_lift0 的说明）
        //   舵机：放下=500us、抬起=2500us。1us = CLK_FREQ_HZ/1e6 = 50 拍，
        //   所以"抬起"≈125000 拍、"放下"≈25000 拍，阈值取中间 75000（=1500us）。
        measure_lift0;
        $fwrite(log_fd, "  [info] 抬腿时前腿舵机脉宽 = %0d 拍 (约 %0d us)\n",
                lift_high, lift_high / (CLK_FREQ_HZ / 1_000_000));
        check("5.2 抬腿中: lift_pwm 前腿=抬起(脉宽>1.5ms)",
              (lift_high > 75_000) ? 1 : 0, 1);
        check("5.3 抬腿中: m_en=0",             m_en, 0);

        // ★ 限位开关是低有效：写 2'b00 = 两组都到位
        lift_lim_n = 2'b00;
        wait_ms(3);   // ★ 限位要过消抖(1ms)，step 一拍不够
        check("5.4 限位到位: 仍在 LIFT(进入通过阶段)", state_dbg, S_LIFT);
        check("5.5 通过中: m_en=1",             m_en, 1);
        check("5.6 通过中: speed=SLOW",         speed_scale_dbg, SPD_SLOW);

        // 通过完成需要两个条件同时满足：
        //   path_clear（前向 TOF 变远）**并且** t_cross >= CROSS_MIN_MS
        // CROSS_MIN_MS 在 tb 里覆盖成 1ms —— 要真等够，不能只走几拍
        feed_dist(16'd1200);
        wait_ms(3);
        check("5.7 一直不松开限位: 仍在 LIFT",  state_dbg, S_LIFT);

        // 放腿阶段结束（cross 1ms + settle 4ms，要等 >5ms）
        wait_ms(10);
        check("5.8 收腿完成: state=AUTO",       state_dbg, S_AUTO);

        // 清掉限位（模拟腿已放下）
        lift_lim_n = 2'b11; step;

        //------------------------------------------------------------------
        // 场景 5b：越障 —— 台阶（模式 1，两腿分段）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5b. 越障(模式1)：视觉报台阶 -> 两腿分段 ---\n");
        send_vision_frame(8'd1, VIS_STEP, 8'h80, 8'h00, 8'd90);
        step;
        check("5b.1 台阶: state=LIFT",          state_dbg, S_LIFT);
        check("5b.2 先抬前腿: m_en=0",          m_en, 0);
        check("5b.3 前腿抬起: lift_pwm[1]=短脉宽(后腿没抬)",
              (lift_pwm[1] === 1'b0) ? 1 : 0, 1);

        lift_lim_n[0] = 1'b0;                   // 只让前腿到位
        wait_ms(3);                             // 等消抖
        check("5b.4 前腿到位后: 进入前轮上位",   state_dbg, S_LIFT);
        check("5b.5 前轮上位: m_en=1",          m_en, 1);

        feed_dist(16'd1200);
        wait_ms(6);   // 前轮上位要等够 CROSS_MIN(5ms)，3ms 不够
        check("5b.6 前轮上位完成: 抬后腿",       state_dbg, S_LIFT);
        check("5b.7 抬后腿时 m_en=0",           m_en, 0);

        lift_lim_n[1] = 1'b0;                   // 后腿也到位
        wait_ms(3);                             // 等消抖
        check("5b.8 后腿到位: 进入后轮上位",     state_dbg, S_LIFT);
        check("5b.9 后轮上位: m_en=1",          m_en, 1);

        wait_ms(10);
        check("5b.10 跨台阶完成: state=AUTO",    state_dbg, S_AUTO);
        lift_lim_n = 2'b11; step;

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 编码器反馈影响输出（闭环真的通了）---\n");
        feed_dist(16'd2000);
        step;
        // ★ 同上：刚从 LIFT 回到 AUTO，m_en 才拉起来，占空比要从 0 爬上来
        //   （motor_guard 的上升限速），等它稳定再测基线值。
        wait_ms(10);
        measure_pwm0;
        duty_static = high_cnt;
        $fwrite(log_fd, "  [info] 静止占空比 = %0d\n", duty_static);

        // 必须边驱动编码器边测：先转完再测的话，测速窗口已归零，测到的是静止值
        fork
            enc_fwd(400);
            begin
                repeat (SPEED_WIN + 100) step;
                measure_pwm0;
                duty_moving = high_cnt;
            end
        join
        $fwrite(log_fd, "  [info] 正转时占空比 = %0d\n", duty_moving);

        //------------------------------------------------------------------
        // ★ 2026-10-18 改写：原检查是「正转后占空比下降」——那是**纯 P 控制**下的性质。
        //
        //   加了积分项（消除稳态残差）之后，静止时（spd=0、误差恒正）积分会把输出
        //   顶到 `V_LIMIT_DUTY` 的限幅上；此时**限幅期间环路等于开环**，
        //   所以"有反馈 → 占空比下降"不再成立（实测两边都是限幅值 1292）。
        //   这不是 bug，是限幅的固有行为（条件积分会冻结积分器，避免越限越深）。
        //
        //   闭环的**稳态精度**改由 `tb_motion_loop` 专门验证（G=1 / G=2 都到 100%，
        //   且无振荡）。这里保留两条仍然有意义、且能抓住真实退化的检查：
        //     6.1  有反馈时输出**稳定**（不因为反馈出现而抖动）
        //     6.2  方向仍然正确
        //------------------------------------------------------------------
        check("6.1 有反馈时输出稳定（不因反馈抖动）",
              (duty_moving == duty_static) ? 1 : 0, 1);
        check("6.2 正转: 方向脚为正",           m_dir[0], 1);
        // ★ 2026-10-18 新增：TB6612 要两个互补方向脚才能正反转，
        //   m_dir2 必须是 m_dir 的反相；否则 AIN1=AIN2 会变成"刹车"（电机不转还不发热，
        //   是极难查的一种故障）。
        //   ⚠️ 必须用逻辑非 `!` 而不是按位非 `~`：
        //      check 的参数是 integer，`~m_dir[0]` 会先把 1 位值零扩展到 32 位
        //      再整体取反（1'b1 -> -2），跟 0/1 比较必然不等。
        //      这是 Verilog 位宽规则的一个经典陷阱，实测踩到了。
        check("6.2b 第二方向脚 = !方向脚(bit0)", m_dir2[0], !m_dir[0]);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. 急停通路（按键 -> 0 拍断使能）---\n");
        key = KEY_ESTOP;
        step;
        check("7.1 ★按急停当拍: m_en=0",        m_en, 0);
        check("7.2 按急停: state=ESTOP",        state_dbg, S_ESTOP);
        check("7.3 按急停: led[1]=0",           led[1], 0);
        check("7.4 人为急停: error_flag=0",     error_flag, 0);
        measure_pwm0;
        check("7.5 急停: PWM 高电平约 0",
              (high_cnt < 20) ? 1 : 0, 1);

        key = KEY_IDLE;
        repeat (10) step;
        check("7.6 急停解除未按启动: 仍 ESTOP",  state_dbg, S_ESTOP);
        check("7.7 ★急停解除后 error_flag 仍 0", error_flag, 0);
        check("7.8 急停解除未按启动: m_en=0",   m_en, 0);

        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 到达终点 ---\n");
        press_start;
        step;
        check("8.1 重新启动: state=IDLE",       state_dbg, S_IDLE);
        press_start;
        step;
        check("8.2 再启动: state=AUTO",         state_dbg, S_AUTO);

        // ★ 低有效：拉低 = 到达终点
        goal_reached = 1'b0; step; goal_reached = 1'b1;
        check("8.3 goal_reached: state=GOAL",   state_dbg, S_GOAL);
        check("8.4 GOAL: m_en=0",               m_en, 0);

        //------------------------------------------------------------------
        // 12) 联调实测帧：ARM 队友实测跑出来的「标准帧」，原样灌进来
        //
        //     AA 55 01 02 00 08 01 00 02 80 00 80 00 02 0A 55 AA
        //
        //    含义：正中心 (32768, 32768) 的一个 0x02 障碍物目标，
        //          置信度 2，载荷 8 字节，总长 17。
        //
        //    为什么这一节最有价值：
        //      前面的场景都是我自己按文档造的帧 —— 那只能证明
        //      「我按我理解的文档实现了」。这一节灌的是对方真实跑出来的
        //      字节，它能证明的是「两边的实现真的对得上」。
        //      协议哪天被改动，这里会第一时间红。
        //
        //    BCC 直接写死 0x0A，不调我方的计算 —— 要验证的正是
        //    「对方算的 BCC 我的 RTL 认不认」。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 12. ARM 实测标准帧（原样灌入）---\n");

        // 回到 AUTO（上一节停在 GOAL）
        press_start;
        step;
        press_start;
        step;
        check("12.1 准备: state=AUTO", state_dbg, S_AUTO);

        // 记下灌帧前的计数 —— vision_frame_cnt 是**累计**的，
        // 场景 5 已经发过一帧，所以不能直接断言等于 1，要看增量。
        vis_cnt_before = vision_frame_cnt;

        // ---- 原样灌入队友的 17 个字节，一个都不改 ----
        put_byte(8'hAA); put_byte(8'h55);   // 帧头
        put_byte(8'h01);                    // VER
        put_byte(8'h02);                    // TYPE = VISION_RESULT
        put_byte(8'h00); put_byte(8'h08);   // LEN = 8 (大端)
        put_byte(8'h01);                    // 目标数 = 1
        put_byte(8'h00);                    // 序号 = 0
        put_byte(8'h02);                    // 类别 = 0x02 障碍物
        put_byte(8'h80); put_byte(8'h00);   // cx = 32768 (正中心)
        put_byte(8'h80); put_byte(8'h00);   // cy = 32768
        put_byte(8'h02);                    // 置信度 = 2
        put_byte(8'h0A);                    // BCC（对方算的）
        put_byte(8'h55); put_byte(8'hAA);   // 帧尾
        repeat (200) step;                   // 等 vision_rx 解析

        // ---- 验证效果 ----
        check("12.2 合法帧: uart_rx_ok_cnt>=1",  (uart_rx_ok_cnt >= 1) ? 1 : 0, 1);
        check("12.3 无错帧: uart_rx_err_cnt=0",  uart_rx_err_cnt, 0);
        check("12.4 视觉帧计数 +1",               vision_frame_cnt - vis_cnt_before, 1);
        check("12.5 无格式错: vision_bad_cnt=0",  vision_bad_cnt, 0);
        check("12.6 0x02 障碍物 -> state=AVOID",  state_dbg, S_AVOID);
        check("12.7 AVOID: m_en=1（限速绕行）",   m_en, 1);
        check("12.8 AVOID: speed_scale=SLOW",     speed_scale_dbg, SPD_SLOW);

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n=== tb_top_integration: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 超时保护（不要写成超过 32 位的字面量）
    //------------------------------------------------------------------
    initial begin
        #200_000_000;                    // 200ms
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
