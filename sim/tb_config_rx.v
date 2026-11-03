//=============================================================================
// tb : tb_config_rx
//-----------------------------------------------------------------------------
// 验证 rtl/app/config_rx.v：0x04 CONFIG 帧解析 + 参数钳位 + 计数饱和。
//
//-----------------------------------------------------------------------------
// 设计原则（沿用本项目已验证的几条约定，见 sim/tb_main_fsm.v）
//-----------------------------------------------------------------------------
// 1) 结果用 $fwrite 落盘，不依赖 stdout
//      结果文件: config_rx_result.txt
//      格式: [ OK ] / [FAIL] 行 + "=== tb_config_rx: N checks, M errors ==="
//            + "RESULT: PASS/FAIL"
//
// 2) 判据**写死在 tb 里**，不引用 DUT 内部的 localparam
//      否则 RTL 的钳位区间改错了，tb 跟着一起错，等于没测。
//
// 3) 不使用 dut.xxx 层次引用
//      层次引用要求 vsim 带 -voptargs="+acc"，否则优化器删信号后
//      加载阶段直接报 "Error loading design" 且不给细节。只用端口信号。
//
// 4) 载荷缓冲在 tb 里复刻一份
//      真实 uart_frame 是 assign rx_pl_data = rx_buf[rx_pl_addr]（组合），
//      这里用 pl_buf[] + 同样的 assign 复刻，这样 rx_pl_addr → rx_pl_data
//      的**组合时序**（同一拍数据有效）能被真实验证到。
//      如果只在 tb 里按状态机节拍直接给数据，地址线接错也测不出来。
//
// 5) 每个等待都有超时
//      帧处理用固定拍数 RESP_CYCLES（含余量）等，不写"等到出现脉冲"，
//      否则 DUT 一旦卡死 tb 跟着永久挂住。
//
//-----------------------------------------------------------------------------
// 覆盖的 9 条行为
//-----------------------------------------------------------------------------
//   1) 复位值 = 默认值 .................. 场景 1
//   2) 正常配置，每个 ID 都能写入 ........ 场景 2（含大端判别值）
//   3) 长度校验非法 -> bad 且值不变 ...... 场景 3
//   4) 未知 ID / 0x03 -> 忽略 ........... 场景 4
//   5) 0x20 手动模式 0/1，>1 非法 ........ 场景 5
//   6) 钳位 ............................. 场景 6
//   7) 非 CONFIG 帧完全不响应 ........... 场景 7（含 rx_pl_addr 恒 0）
//   8) 一帧只处理一个参数（长度多也算非法）场景 3
//   9) 计数饱和 ......................... 场景 8
//
//-----------------------------------------------------------------------------
// 时钟/时间
//-----------------------------------------------------------------------------
//   50MHz（20ns）。正常场景约 60 帧 x 13 拍 ≈ 16us。
//   **场景 8（饱和测试）是耗时大头**：每轮 65700 帧，
//   合计约 92 万拍 ≈ 18.4ms 仿真时间。
//   想快速回归可以把 RUN_SAT_TEST 置 0（只跳过饱和测试）。
//=============================================================================
`timescale 1ns/1ps

module tb_config_rx;

    //------------------------------------------------------------------
    // 时钟
    //------------------------------------------------------------------
    localparam integer CLK_PERIOD  = 20;          // 50MHz
    localparam integer CLK_FREQ_HZ = 50_000_000;

    //------------------------------------------------------------------
    // DUT 参数（必须与 rtl/app/config_rx.v 的 DEF_* 一致）
    //------------------------------------------------------------------
    localparam integer DEF_VREF         = 69;
    localparam integer DEF_DIST_STOP_MM = 200;
    localparam integer DEF_DIST_SLOW_MM = 500;
    localparam integer DEF_LIFT_HEIGHT  = 100;
    localparam integer DEF_LIFT_TMO_MS  = 3000;

    //------------------------------------------------------------------
    // 帧类型 / 参数 ID（协议 doc/uart_frame_protocol.md §2 §3.2）
    //------------------------------------------------------------------
    localparam [7:0] TYPE_VISION = 8'h02;
    localparam [7:0] TYPE_ACK    = 8'h03;
    localparam [7:0] TYPE_CONFIG = 8'h04;

    localparam [7:0] ID_VREF     = 8'h01;
    localparam [7:0] ID_STOP     = 8'h02;
    localparam [7:0] ID_WIN      = 8'h03;   // 测速窗口：只忽略
    localparam [7:0] ID_SLOW     = 8'h04;   // ★ 扩展
    localparam [7:0] ID_LIFT_H   = 8'h10;
    localparam [7:0] ID_LIFT_TMO = 8'h11;
    localparam [7:0] ID_MANUAL   = 8'h20;   // ★ 扩展

    //------------------------------------------------------------------
    // 一帧处理完需要的拍数（实测 5~6 拍，这里给到 12 拍余量）
    //   固定拍数等待 = 自带超时，DUT 卡死也不会把 tb 挂住。
    //------------------------------------------------------------------
    localparam integer RESP_CYCLES = 12;

    //------------------------------------------------------------------
    // 饱和测试
    //   要证明"到 FFFF 就不涨"必须真打满 65536 帧，没有捷径
    //   （不做层次引用 force，那种写法院里跑得通、别的流程会炸）。
    //------------------------------------------------------------------
    localparam integer RUN_SAT_TEST = 1;
    localparam integer SAT_FRAMES   = 65700;    // > 65536，留余量
    // 每帧占用拍数（含 1 拍余量，纯 IDLE 空转不影响正确性）
    localparam integer SAT_CYC_BAD  = 4;
    localparam integer SAT_CYC_OK   = 6;

    //------------------------------------------------------------------
    // DUT 连接
    //------------------------------------------------------------------
    reg         clk   = 0;
    reg         rst_n = 0;

    reg         rx_valid = 0;
    reg  [7:0]  rx_type  = 0;
    reg  [15:0] rx_len   = 0;

    // 复刻 uart_frame 的载荷缓冲（组合读出）
    reg  [7:0]  pl_buf [0:63];
    wire [7:0]  rx_pl_addr;
    wire [7:0]  rx_pl_data;

    // 待发送的载荷（发送前 stage 到 pl_buf）
    reg  [7:0]  pl [0:15];

    wire [15:0] vref;
    wire [15:0] dist_stop_mm;
    wire [15:0] dist_slow_mm;
    wire [7:0]  lift_height;
    wire [15:0] lift_timeout_ms;
    wire        manual_sel;

    wire        cfg_valid;
    wire        cfg_bad;
    wire [15:0] cfg_cnt;
    wire [15:0] cfg_bad_cnt;
    wire [15:0] cfg_ign_cnt;

    // 与 uart_frame 完全同款：地址给上，同一拍数据有效
    assign rx_pl_data = pl_buf[rx_pl_addr];

    //------------------------------------------------------------------
    // 统计/日志
    //------------------------------------------------------------------
    integer errors = 0;
    integer checks = 0;
    integer log_fd;
    integer i;
    integer k;

    // 观测器用的快照
    integer seen_valid = 0;      // cfg_valid 出现过多少拍
    integer seen_bad   = 0;      // cfg_bad   出现过多少拍
    integer both_pulse = 0;      // 两者同拍为高（不该发生）
    integer narrow_err = 0;      // cfg_valid 连续两拍为高（不是单拍）
    integer addr_err   = 0;      // 非 CONFIG 帧时 rx_pl_addr != 0（仲裁违约）
    integer addr_hit   = 0;      // 场景 7 里非 CONFIG 帧期间地址不为 0 的拍数
    reg     cfg_valid_d = 1'b0;
    reg  [7:0] addr_mask = 8'd0; // 一帧内被读过的载荷地址（bit n = 地址 n）
    integer sv, sb;              // 场景前后的脉冲快照

    initial begin
        log_fd = $fopen("config_rx_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 config_rx_result.txt");
    end

    always #(CLK_PERIOD/2) clk = ~clk;

    // 缓冲清零：避免 X 在比较里乱飘（真实 uart_frame 上电也是已知值）
    initial begin
        for (k = 0; k < 64; k = k + 1) pl_buf[k] = 8'd0;
        for (k = 0; k < 16; k = k + 1) pl[k]     = 8'd0;
    end

    //==================================================================
    // DUT
    //==================================================================
    config_rx #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .DEF_VREF         (DEF_VREF),
        .DEF_DIST_STOP_MM (DEF_DIST_STOP_MM),
        .DEF_DIST_SLOW_MM (DEF_DIST_SLOW_MM),
        .DEF_LIFT_HEIGHT  (DEF_LIFT_HEIGHT),
        .DEF_LIFT_TMO_MS  (DEF_LIFT_TMO_MS),
        .TYPE_CONFIG      (TYPE_CONFIG),
        ._ASSERT_EN       (1)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .rx_valid        (rx_valid),
        .rx_type         (rx_type),
        .rx_len          (rx_len),
        .rx_pl_data      (rx_pl_data),
        .rx_pl_addr      (rx_pl_addr),
        .vref            (vref),
        .dist_stop_mm    (dist_stop_mm),
        .dist_slow_mm    (dist_slow_mm),
        .lift_height     (lift_height),
        .lift_timeout_ms (lift_timeout_ms),
        .manual_sel      (manual_sel),
        .cfg_valid       (cfg_valid),
        .cfg_bad         (cfg_bad),
        .cfg_cnt         (cfg_cnt),
        .cfg_bad_cnt     (cfg_bad_cnt),
        .cfg_ign_cnt     (cfg_ign_cnt)
    );

    //==================================================================
    // 观测器（只读端口信号，不碰 DUT 内部）
    //
    //   在时钟沿采样 DUT 的**输出当前值**（沿前那一拍的值），
    //   所以计数与主流程里 #1 采到的值是同一批脉冲，不重不漏。
    //==================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_valid_d = 1'b0;
            addr_mask   = 8'd0;
        end else begin
            // 单拍脉冲宽度
            if (cfg_valid && cfg_valid_d) narrow_err = narrow_err + 1;
            cfg_valid_d = cfg_valid;

            if (cfg_valid) seen_valid = seen_valid + 1;
            if (cfg_bad)   seen_bad   = seen_bad   + 1;
            if (cfg_valid && cfg_bad) both_pulse = both_pulse + 1;

            // ★ 仲裁约定：不是 CONFIG 帧，rx_pl_addr 必须是 0
            if ((rx_type !== TYPE_CONFIG) && (rx_pl_addr !== 8'd0))
                addr_err = addr_err + 1;

            // 一帧 CONFIG 里被读过的地址集合（新帧到来时清零）
            if (rx_valid && (rx_type === TYPE_CONFIG)) addr_mask = 8'd0;
            addr_mask = addr_mask | (8'd1 << rx_pl_addr);
        end
    end

    //==================================================================
    // 工具任务
    //==================================================================

    //------------------------------------------------------------------
    // check : 判据比较 + 落盘
    //   name 用 [1023:0] = 128 字节，中文一个字 3 字节，
    //   [255:0] 放不下带编号的检查名（超出会被从前面截掉）。
    //------------------------------------------------------------------
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

    // 推一拍：等时钟沿 + #1，让所有非阻塞更新生效后再采样
    task step;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    //------------------------------------------------------------------
    // 载荷暂存
    //------------------------------------------------------------------
    task stage2;                       // 2 字节参数：ID + 长度 + 大端值
        input [7:0]  id;
        input [15:0] val;
        input [7:0]  declen;
        begin
            pl[0] = id;
            pl[1] = declen;
            pl[2] = val[15:8];         // 大端：高字节在前
            pl[3] = val[7:0];
        end
    endtask

    task stage1;                       // 1 字节参数
        input [7:0] id;
        input [7:0] val;
        input [7:0] declen;
        begin
            pl[0] = id;
            pl[1] = declen;
            pl[2] = val;
        end
    endtask

    //------------------------------------------------------------------
    // 发一帧：把 pl[0..nb-1] 灌进 pl_buf，给一拍 rx_valid，
    //         然后固定等 RESP_CYCLES 拍（自带超时，不依赖脉冲出现）。
    //------------------------------------------------------------------
    task send_frame;
        input [7:0]  ftype;
        input [15:0] flen;
        input integer nb;
        integer j;
        begin
            for (j = 0; j < nb; j = j + 1) pl_buf[j] = pl[j];
            rx_type  = ftype;
            rx_len   = flen;
            rx_valid = 1'b1;
            step;
            rx_valid = 1'b0;
            repeat (RESP_CYCLES) step;
        end
    endtask

    // 合法 2 字节帧（ID + 长度2 + 值）
    task cfg2;
        input [7:0]  id;
        input [15:0] val;
        begin
            stage2(id, val, 8'd2);
            send_frame(TYPE_CONFIG, 16'd4, 4);
        end
    endtask

    // 合法 1 字节帧（ID + 长度1 + 值）
    task cfg1;
        input [7:0] id;
        input [7:0] val;
        begin
            stage1(id, val, 8'd1);
            send_frame(TYPE_CONFIG, 16'd3, 3);
        end
    endtask

    // 2 字节参数：发帧 + 直接检查对应寄存器
    task chk2;
        input [1023:0] name;
        input [7:0]    id;
        input [15:0]   val;
        input [15:0]   exp;
        reg   [15:0]   got;
        begin
            cfg2(id, val);
            case (id)
                ID_VREF:     got = vref;
                ID_STOP:     got = dist_stop_mm;
                ID_SLOW:     got = dist_slow_mm;
                ID_LIFT_TMO: got = lift_timeout_ms;
                default:     got = 16'h0000;
            endcase
            check(name, got, exp);
        end
    endtask

    // 1 字节参数：发帧 + 直接检查对应寄存器
    task chk1;
        input [1023:0] name;
        input [7:0]    id;
        input [7:0]    val;
        input integer  exp;
        reg   [7:0]    got;
        begin
            cfg1(id, val);
            if (id == ID_LIFT_H) got = lift_height;
            else                 got = {7'd0, manual_sel};
            check(name, got, exp);
        end
    endtask

    //------------------------------------------------------------------
    // 复位
    //------------------------------------------------------------------
    task do_reset;
        begin
            rst_n    = 1'b0;
            rx_valid = 1'b0;
            rx_type  = 8'd0;
            rx_len   = 16'd0;
            repeat (4) step;
            rst_n = 1'b1;
            repeat (2) step;
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        $fwrite(log_fd, "=== tb_config_rx ===\n");
        $fwrite(log_fd, "  [info] CLK=%0dHz 周期=%0dns\n", CLK_FREQ_HZ, CLK_PERIOD);
        $fwrite(log_fd, "  [info] 默认值 VREF=%0d STOP=%0d SLOW=%0d LIFT_H=%0d LIFT_TMO=%0d\n",
                DEF_VREF, DEF_DIST_STOP_MM, DEF_DIST_SLOW_MM,
                DEF_LIFT_HEIGHT, DEF_LIFT_TMO_MS);
        if (RUN_SAT_TEST != 0)
            $fwrite(log_fd, "  [info] 饱和测试: 开启\n");
        else
            $fwrite(log_fd, "  [info] 饱和测试: 关闭\n");

        //==================================================================
        // 场景 1: 复位值 = 默认值
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 1: 复位值 = DEF_* 默认值 ---\n");
        do_reset;
        check("1.1 复位后 vref=69",              vref,            69);
        check("1.2 复位后 dist_stop_mm=200",     dist_stop_mm,    200);
        check("1.3 复位后 dist_slow_mm=500",     dist_slow_mm,    500);
        check("1.4 复位后 lift_height=100",      lift_height,     100);
        check("1.5 复位后 lift_timeout_ms=3000", lift_timeout_ms, 3000);
        check("1.6 复位后 manual_sel=0",         manual_sel,      0);
        check("1.7 复位后 cfg_valid=0",          cfg_valid,       0);
        check("1.8 复位后 cfg_bad=0",            cfg_bad,         0);
        check("1.9 复位后 rx_pl_addr=0",         rx_pl_addr,      0);
        check("1.10 复位后 cfg_cnt=0",           cfg_cnt,         0);
        check("1.11 复位后 cfg_bad_cnt=0",       cfg_bad_cnt,     0);
        check("1.12 复位后 cfg_ign_cnt=0",       cfg_ign_cnt,     0);

        //==================================================================
        // 场景 2: 正常配置 —— 每个 ID 都能写进去
        //
        //   ★ 每个值都**故意选成大端/小端结果不同**的数：
        //     0x01: 120 = 0x0078   （字节反了就成 0x7800=30720 -> 钳到 200）
        //     0x02: 300 = 0x012C   （反了 11265 -> 钳到 2000）
        //     0x04: 1000 = 0x03E8  （反了 59395 -> 钳到 3000）
        //     0x11: 3000 = 0x0BB8  （反了 47107 -> 钳到 30000）
        //     0x11: 4660 = 0x1234  （反了 13330，**还在区间内**，最难查）
        //   字节序接反是这类解析器最常见的 bug，必须让它在 tb 里现形。
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 2: 正常配置（每个 ID + 大端判别）---\n");
        sv = seen_valid; sb = seen_bad;

        cfg2(ID_VREF, 16'd120);
        check("2.1 0x01 最大速度写入 120",       vref, 120);
        check("2.2 0x01 产生 1 次 cfg_valid",    seen_valid - sv, 1);
        check("2.3 0x01 不产生 cfg_bad",         seen_bad - sb, 0);
        check("2.4 帧结束后 cfg_valid 已落回 0", cfg_valid, 0);
        check("2.5 0x01 载荷地址序列 0,1,2,3",   addr_mask, 8'h0F);

        chk2("2.6 0x02 停车距离写入 300",        ID_STOP, 16'd300, 300);
        chk2("2.7 0x04 减速距离写入 1000",       ID_SLOW, 16'd1000, 1000);
        chk1("2.8 0x10 抬升高度写入 60",         ID_LIFT_H, 8'd60, 60);
        check("2.9 0x10 载荷地址序列 0,1,2",     addr_mask, 8'h07);
        chk2("2.10 0x11 超时写入 3000",          ID_LIFT_TMO, 16'd3000, 3000);
        chk2("2.11 ★0x11 超时 4660(字节反了会变 13330)", ID_LIFT_TMO, 16'd4660, 4660);
        chk1("2.12 0x20 手动模式=1",             ID_MANUAL, 8'd1, 1);
        chk1("2.13 0x20 手动模式=0",             ID_MANUAL, 8'd0, 0);

        check("2.14 场景2 共 8 次 cfg_valid",    seen_valid - sv, 8);
        check("2.15 场景2 无 cfg_bad",           seen_bad - sb, 0);
        check("2.16 场景2 后 cfg_cnt=8",         cfg_cnt, 8);
        check("2.17 场景2 不动 dist_stop(300)",  dist_stop_mm, 300);
        check("2.18 场景2 不动 dist_slow(1000)", dist_slow_mm, 1000);

        //==================================================================
        // 场景 3: 长度校验 —— 非法就 bad，**一个寄存器都不写**
        //
        //   两种非法都要测：
        //     (a) 帧头 rx_len != 2 + 值长度
        //     (b) 载荷里的"长度"字节 != 期望长度
        //   还有 (c) 一帧塞两个参数（rx_len 比期望更长）—— 同样判非法。
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 3: 长度校验非法 -> cfg_bad 且值不变 ---\n");
        sv = seen_valid; sb = seen_bad;

        // (a) 帧头长度太长（说 5，实际按 2 字节参数）
        stage2(ID_VREF, 16'd50, 8'd2);
        send_frame(TYPE_CONFIG, 16'd5, 4);
        check("3.1 rx_len 太长 -> vref 不变(120)", vref, 120);
        check("3.2 rx_len 太长 -> cfg_bad +1",     seen_bad - sb, 1);

        // (a) 帧头长度太短
        stage2(ID_VREF, 16'd50, 8'd2);
        send_frame(TYPE_CONFIG, 16'd3, 4);
        check("3.3 rx_len 太短 -> vref 不变",      vref, 120);

        // (b) 载荷长度字节与 ID 不符（0x01 是 2 字节，这里写 3）
        stage2(ID_VREF, 16'd50, 8'd3);
        send_frame(TYPE_CONFIG, 16'd4, 4);
        check("3.4 载荷长度字节不符 -> vref 不变", vref, 120);

        // (c) 一帧两个参数：rx_len=6（2 + 4）
        stage2(ID_LIFT_TMO, 16'd999, 8'd2);
        pl[4] = ID_STOP; pl[5] = 8'd2; pl[6] = 8'h01; pl[7] = 8'h2C;
        send_frame(TYPE_CONFIG, 16'd6, 8);
        check("3.5 一帧两参数(rx_len=6) -> 超时不变(4660)", lift_timeout_ms, 4660);
        check("3.5b 一帧两参数 -> 停车距离不变(300)",       dist_stop_mm, 300);

        // (b) 1 字节参数却声明长度 2
        stage1(ID_LIFT_H, 8'd80, 8'd2);
        send_frame(TYPE_CONFIG, 16'd4, 3);
        check("3.6 1字节参数声明长度2 -> lift_height 不变(60)", lift_height, 60);

        // (a) 1 字节参数但帧头说 3 字节值
        stage1(ID_LIFT_H, 8'd80, 8'd1);
        send_frame(TYPE_CONFIG, 16'd5, 3);
        check("3.7 1字节参数帧头超长 -> lift_height 不变", lift_height, 60);

        check("3.8 场景3 全程 0 次 cfg_valid",   seen_valid - sv, 0);
        check("3.9 场景3 共 6 次 cfg_bad",       seen_bad - sb, 6);
        check("3.10 场景3 后 cfg_bad_cnt=6",     cfg_bad_cnt, 6);
        check("3.11 场景3 后 cfg_cnt 仍是 8",    cfg_cnt, 8);
        check("3.12 非法帧只读了地址 0,1",       addr_mask, 8'h03);

        //==================================================================
        // 场景 4: 未知 ID / 0x03 测速窗口 -> 忽略（计数），不算错
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 4: 未知 ID / 0x03 -> 忽略 ---\n");
        sv = seen_valid; sb = seen_bad;

        // 0x03 测速窗口：本模块不实现（encoder_decode 的编译期参数）
        cfg2(ID_WIN, 16'd500);
        check("4.1 0x03 测速窗口 -> cfg_ign_cnt+1", cfg_ign_cnt, 1);
        check("4.2 0x03 不产生 cfg_valid",          seen_valid - sv, 0);
        check("4.3 0x03 不产生 cfg_bad",            seen_bad - sb, 0);

        // 未知 ID 0x55
        cfg2(8'h55, 16'd700);
        check("4.4 未知ID 0x55 -> cfg_ign_cnt=2",   cfg_ign_cnt, 2);

        // 未知 ID 0x00
        cfg2(8'h00, 16'd1);
        check("4.5 未知ID 0x00 -> cfg_ign_cnt=3",   cfg_ign_cnt, 3);

        // 0x03 而且长度也不对：仍然是"忽略"，不是 bad
        stage2(ID_WIN, 16'd500, 8'd2);
        send_frame(TYPE_CONFIG, 16'd9, 4);
        check("4.6 0x03 长度非法也只算忽略 -> ign=4", cfg_ign_cnt, 4);
        check("4.7 忽略帧不算 bad",                 seen_bad - sb, 0);
        check("4.8 忽略帧只读地址 0,1",             addr_mask, 8'h03);

        // 一个值都不能被这些帧改动
        check("4.9 忽略帧后 vref 仍 120",           vref, 120);
        check("4.10 忽略帧后 stop 仍 300",          dist_stop_mm, 300);
        check("4.11 忽略帧后 slow 仍 1000",         dist_slow_mm, 1000);
        check("4.12 忽略帧后 lift_h 仍 60",         lift_height, 60);
        check("4.13 忽略帧后 超时仍 4660",          lift_timeout_ms, 4660);
        check("4.14 忽略帧后 manual 仍 0",          manual_sel, 0);

        //==================================================================
        // 场景 5: 0x20 手动模式开关（0/1 合法，>1 非法）
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 5: 0x20 手动模式 ---\n");
        sv = seen_valid; sb = seen_bad;

        chk1("5.1 0x20 = 1 -> manual_sel=1",   ID_MANUAL, 8'd1, 1);
        chk1("5.2 0x20 = 0 -> manual_sel=0",   ID_MANUAL, 8'd0, 0);

        cfg1(ID_MANUAL, 8'd2);
        check("5.3 0x20 = 2 -> cfg_bad",         seen_bad - sb, 1);
        check("5.4 0x20 = 2 -> manual_sel 不变(0)", manual_sel, 0);

        sb = seen_bad;
        cfg1(ID_MANUAL, 8'd255);
        check("5.5 0x20 = 255 -> cfg_bad",       seen_bad - sb, 1);
        check("5.6 0x20 = 255 -> manual_sel 不变(0)", manual_sel, 0);

        // 合法值 1，然后长度非法的 0x20 帧不能把它改掉
        cfg1(ID_MANUAL, 8'd1);
        check("5.7 0x20 = 1 -> manual_sel=1",     manual_sel, 1);
        sb = seen_bad;
        stage1(ID_MANUAL, 8'd0, 8'd1);
        send_frame(TYPE_CONFIG, 16'd5, 3);
        check("5.8 0x20 帧长非法 -> cfg_bad",     seen_bad - sb, 1);
        check("5.9 0x20 帧长非法 -> manual_sel 仍 1", manual_sel, 1);

        cfg1(ID_MANUAL, 8'd0);
        check("5.10 0x20 = 0 -> manual_sel=0",    manual_sel, 0);
        check("5.11 场景5 后 cfg_bad_cnt=9",      cfg_bad_cnt, 9);
        check("5.12 场景5 后 cfg_cnt=12",         cfg_cnt, 12);

        //==================================================================
        // 场景 6: 钳位
        //
        //   起手值：vref=120 stop=300 slow=1000 lift_h=60 tmo=4660 manual=0
        //   每个参数都要测：下边界越界、上边界越界、两个边界本身、
        //   以及"钳位算成功（cfg_valid）不算 bad"。
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 6: 钳位 ---\n");
        sv = seen_valid; sb = seen_bad;

        chk2("6.1 vref 0 钳到 1",        ID_VREF, 16'd0,    1);
        chk2("6.2 vref 500 钳到 200",    ID_VREF, 16'd500,  200);
        chk2("6.3 vref 201 钳到 200",    ID_VREF, 16'd201,  200);
        chk2("6.4 vref 200 边界不变",    ID_VREF, 16'd200,  200);
        chk2("6.5 vref 1 边界不变",      ID_VREF, 16'd1,    1);
        chk2("6.6 vref 100 正常",        ID_VREF, 16'd100,  100);

        chk2("6.7 stop 0 钳到 50",       ID_STOP, 16'd0,    50);
        chk2("6.8 stop 49 钳到 50",      ID_STOP, 16'd49,   50);
        chk2("6.9 stop 5000 钳到 2000",  ID_STOP, 16'd5000, 2000);
        chk2("6.10 stop 2000 边界不变",  ID_STOP, 16'd2000, 2000);

        // ★ 不变量 slow > stop 是**双向**维持的（原先只做单向，这里曾钉过"已知缺口"）：
        //   单独写 0x02 把 stop 顶到 2000 时，slow(1000) 会被一起顶到 stop+50=2050。
        check("6.11 ★stop=2000 时 slow 被顶到 2050", dist_slow_mm, 2050);

        chk2("6.12 stop 800 恢复",       ID_STOP, 16'd800,  800);

        chk2("6.13 slow 5000 钳到 3000", ID_SLOW, 16'd5000, 3000);
        chk2("6.14 slow 3000 边界不变",  ID_SLOW, 16'd3000, 3000);
        chk2("6.15 slow 0 钳到 50 后被顶到 stop+50=850", ID_SLOW, 16'd0, 850);
        chk2("6.16 slow 850 (>stop=800) 不动",          ID_SLOW, 16'd850, 850);
        chk2("6.17 slow 800 (==stop) -> 850",           ID_SLOW, 16'd800, 850);
        chk2("6.18 slow 700 (<stop) -> 850",            ID_SLOW, 16'd700, 850);

        chk1("6.19 lift_h 255 钳到 100",  ID_LIFT_H, 8'd255, 100);
        chk1("6.20 lift_h 101 钳到 100",  ID_LIFT_H, 8'd101, 100);
        chk1("6.21 lift_h 0 合法不变",    ID_LIFT_H, 8'd0,   0);
        chk1("6.22 lift_h 50 正常",       ID_LIFT_H, 8'd50,  50);

        chk2("6.23 超时 0 钳到 500",      ID_LIFT_TMO, 16'd0,     500);
        chk2("6.24 超时 499 钳到 500",    ID_LIFT_TMO, 16'd499,   500);
        chk2("6.25 超时 40000 钳到 30000", ID_LIFT_TMO, 16'd40000, 30000);
        chk2("6.26 超时 30000 边界不变",  ID_LIFT_TMO, 16'd30000, 30000);
        chk2("6.27 超时 500 边界不变",    ID_LIFT_TMO, 16'd500,   500);
        chk2("6.28 超时 1000 正常",       ID_LIFT_TMO, 16'd1000,  1000);

        check("6.29 钳位帧全部算成功(27 次 cfg_valid)", seen_valid - sv, 27);
        check("6.30 钳位不算 bad",                      seen_bad - sb, 0);

        //==================================================================
        // 场景 7: 非 CONFIG 帧 -> 完全不响应
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 7: 非 CONFIG 帧完全不响应 ---\n");
        sv = seen_valid; sb = seen_bad;

        // 0x02 视觉帧（8 字节载荷，看起来很像一帧真数据）
        pl[0] = 8'd1; pl[1] = 8'd0; pl[2] = 8'h02; pl[3] = 8'h80; pl[4] = 8'h00;
        pl[5] = 8'h80; pl[6] = 8'h00; pl[7] = 8'd90;
        for (i = 0; i < 8; i = i + 1) pl_buf[i] = pl[i];

        // 打一拍 rx_valid，然后连续 10 拍盯住 rx_pl_addr（必须一直是 0）
        addr_hit = 0;
        rx_type  = TYPE_VISION; rx_len = 16'd8; rx_valid = 1'b1;
        step;
        rx_valid = 1'b0;
        for (i = 0; i < 10; i = i + 1) begin
            step;
            if (rx_pl_addr !== 8'd0) addr_hit = addr_hit + 1;
        end
        check("7.1 0x02 帧期间 rx_pl_addr 恒为 0", addr_hit, 0);
        check("7.2 0x02 帧不产生 cfg_valid",       seen_valid - sv, 0);
        check("7.3 0x02 帧不产生 cfg_bad",         seen_bad - sb, 0);
        check("7.4 0x02 帧不动 cfg_cnt",           cfg_cnt, 39);
        check("7.5 0x02 帧不动 cfg_bad_cnt",       cfg_bad_cnt, 9);
        check("7.6 0x02 帧不动 cfg_ign_cnt",       cfg_ign_cnt, 4);

        // 0x03 ACK 帧
        stage1(8'h00, 8'd0, 8'd1);
        send_frame(TYPE_ACK, 16'd1, 1);
        check("7.7 0x03 ACK 帧不响应: cfg_cnt",    cfg_cnt, 39);
        check("7.8 0x03 ACK 帧不响应: ign_cnt",    cfg_ign_cnt, 4);

        // 0x01 IMAGE_DATA 帧
        send_frame(8'h01, 16'd4, 4);
        check("7.9 0x01 图像帧不响应: cfg_cnt",    cfg_cnt, 39);

        // 值一个都不能变
        check("7.10 非 CONFIG 帧后 vref 仍 100",   vref, 100);
        check("7.11 非 CONFIG 帧后 stop 仍 800",   dist_stop_mm, 800);
        check("7.12 非 CONFIG 帧后 slow 仍 850",   dist_slow_mm, 850);
        check("7.13 非 CONFIG 帧后 lift_h 仍 50",  lift_height, 50);
        check("7.14 非 CONFIG 帧后 超时仍 1000",   lift_timeout_ms, 1000);
        check("7.15 非 CONFIG 帧后 manual 仍 0",   manual_sel, 0);

        //==================================================================
        // 场景 8: 计数饱和（到 16'hFFFF 就不涨）
        //
        //   没有捷径：要证明饱和必须真打满 65536 帧。
        //   本轮占用拍数最多，是 tb 耗时的绝大部分（约 18ms 仿真时间）。
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 8: 计数饱和 ---\n");
        if (RUN_SAT_TEST != 0) begin
            $fwrite(log_fd, "  [info] 饱和测试: 每轮 %0d 帧, 预计 %0d 拍 = %0d us\n",
                    SAT_FRAMES,
                    SAT_FRAMES * (SAT_CYC_BAD + SAT_CYC_OK + SAT_CYC_BAD),
                    (SAT_FRAMES * (SAT_CYC_BAD + SAT_CYC_OK + SAT_CYC_BAD)
                     * CLK_PERIOD) / 1000);

            //---- 8.a cfg_bad_cnt 饱和 ----
            //   0x01 声明长度 9、帧长 4 -> 永久非法
            pl_buf[0] = ID_VREF; pl_buf[1] = 8'd9;
            pl_buf[2] = 8'd0;    pl_buf[3] = 8'd0;
            for (i = 0; i < SAT_FRAMES; i = i + 1) begin
                rx_type = TYPE_CONFIG; rx_len = 16'd4; rx_valid = 1'b1;
                step;
                rx_valid = 1'b0;
                repeat (SAT_CYC_BAD - 1) step;
            end
            check("8.1 cfg_bad_cnt 饱和在 FFFF",       cfg_bad_cnt, 16'hFFFF);
            check("8.2 饱和后 cfg_cnt 没被带偏(39)",   cfg_cnt, 39);
            check("8.3 饱和后 cfg_ign_cnt 没被带偏(4)", cfg_ign_cnt, 4);

            //---- 8.b cfg_cnt 饱和 ----
            //   0x10 合法 1 字节帧（值 60）
            pl_buf[0] = ID_LIFT_H; pl_buf[1] = 8'd1; pl_buf[2] = 8'd60;
            for (i = 0; i < SAT_FRAMES; i = i + 1) begin
                rx_type = TYPE_CONFIG; rx_len = 16'd3; rx_valid = 1'b1;
                step;
                rx_valid = 1'b0;
                repeat (SAT_CYC_OK - 1) step;
            end
            check("8.4 cfg_cnt 饱和在 FFFF",           cfg_cnt, 16'hFFFF);
            check("8.5 饱和后 cfg_bad_cnt 仍是 FFFF",  cfg_bad_cnt, 16'hFFFF);
            check("8.6 满帧后 lift_height 仍 60",      lift_height, 60);

            //---- 8.c cfg_ign_cnt 饱和 ----
            pl_buf[0] = 8'h55; pl_buf[1] = 8'd2;
            pl_buf[2] = 8'h00; pl_buf[3] = 8'h01;
            for (i = 0; i < SAT_FRAMES; i = i + 1) begin
                rx_type = TYPE_CONFIG; rx_len = 16'd4; rx_valid = 1'b1;
                step;
                rx_valid = 1'b0;
                repeat (SAT_CYC_BAD - 1) step;
            end
            check("8.7 cfg_ign_cnt 饱和在 FFFF",       cfg_ign_cnt, 16'hFFFF);
            check("8.8 饱和后 cfg_cnt 仍是 FFFF",      cfg_cnt, 16'hFFFF);
            check("8.9 饱和后 vref 仍 100",            vref, 100);
        end else begin
            $fwrite(log_fd, "  [info] RUN_SAT_TEST=0，跳过饱和测试\n");
        end

        //==================================================================
        // 场景 9: 全程不变量
        //==================================================================
        $fwrite(log_fd, "\n--- 场景 9: 全程不变量 ---\n");
        check("9.1 全程 cfg_valid/cfg_bad 从未同拍",      both_pulse, 0);
        check("9.2 全程 cfg_valid 从未连续两拍为高",       narrow_err, 0);
        check("9.3 全程非 CONFIG 帧 rx_pl_addr 恒为 0",    addr_err, 0);

        // 汇总
        $fwrite(log_fd, "\n=== tb_config_rx: %0d checks, %0d errors ===\n",
                checks, errors);
        if (errors == 0) $fwrite(log_fd, "RESULT: PASS\n");
        else             $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $stop;
    end

    //------------------------------------------------------------------
    // 仿真上限（超时保护）
    //
    //   饱和测试约 18.4ms 仿真时间，100ms 有 5 倍余量。
    //   [!] 不能写成 #5_000_000_000：无尺寸字面量默认 32 位，
    //       超 2^32 会让 vlog 直接拒绝编译本文件。
    //------------------------------------------------------------------
    initial begin
        #100_000_000;
        $fwrite(log_fd, "RESULT: FAIL (timeout)\n");
        $fclose(log_fd);
        $stop;
    end

endmodule
