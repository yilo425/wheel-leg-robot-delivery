//=============================================================================
// tb : tb_tof_driver
//-----------------------------------------------------------------------------
// 拓扑: tof_driver(DUT) <--I2C--> 本文件里的 I2C 从机模型（模拟 VL53L0X）
//
// 验证内容：
//   1) I2C 时序正确：START / 地址+W/R / ACK / 数据 / STOP 都被从机模型识别，
//      并且**SDA 在 SCL 高电平期间没有变化过**（I2C 的硬性约束，
//      violation 计数必须为 0）
//   2) ★ 官方初始化序列真的写下去了：tuning 表代表项、翻页、0x01 的
//      F8->E8、中断配置、以及**写入总条数 104 + 32 位校验和**（见下面 §校验和）
//   3) ★ 两处读-改-写（0x60 |= 0x12、0x84 &= ~0x10）是"读了再改"，
//      不是把常数直接写下去 —— 从机预置初值，两轮初始化用**不同初值**，
//      断言最终写回值随初值变化
//   4) ★ 参考校准真的跑了：0x00=0x41 写了 2 次，且之后 0x13 被轮询过
//   5) ★ 每次单次测距前都写了 stop_variable（0x91 写次数 >= 采样次数）
//   6) 从机返回的 16 位距离被正确拼成 dist_mm（0x1E 高字节 / 0x1F 低字节）
//   7) dist_valid / dist_err 是**单拍脉冲**（不会连续两拍高）
//   8) 从机不应答(NACK) -> dist_err
//   9) 从机拉住 SCL（时钟拉伸）-> 主机超时 -> dist_err，并且**释放总线**
//
// 另外还验证：复合读用了重复 START（不是 STOP+START）、采样周期正确、
//             0 / 超量程读数判为无效且不刷新 dist_mm、复位能清状态。
//
//-----------------------------------------------------------------------------
// ★ 校验和（怎么算的、为什么）
//-----------------------------------------------------------------------------
//   初始化那 104 笔"写数据字节"按**总线上的先后顺序**喂进一个 32 位哈希：
//
//       chk = chk * 31 + {reg[7:0], val[7:0]}        （32 位回绕）
//
//   期望值 CHK32_EXP 是拿 RTL 里的步骤表和官方序列**逐条比对通过**之后，
//   用脚本独立算出来的（脚本不在这份 tb 里，是临时工具）。这样以后任何人
//   改表（改值/改顺序/多一条少一条）都会被立刻发现，而不是等上板。
//
//   窗口怎么界定：从复位到 DUT 的 init_ok 置起为止（初始化结束的那一刻）。
//   之后"每次测距前"的前导会写 0x91=stop_var，那个值是**动态**的，
//   不能进固定常数，所以窗口必须在它之前闭合。
//   ⚠️ 这里用了一次层次引用 dut.init_ok —— 需要 vsim 带 -voptargs="+acc"
//      （run_sim.do / compile_all.do 一直带着，本项目已多次踩过这个坑）。
//   另外还断言"闭窗后的**第一笔**写必须是 0x80=0x01"，用来证明窗口
//   正好关在初始化与前导的交界处。
//
//   读-改-写的两个值不是常数，而是"从机预置值算出来的"：
//       0x60: 预置 0xC0 -> 写回 0xC0|0x12 = 0xD2
//       0x84: 预置 0xFF -> 写回 0xFF&~0x10 = 0xEF
//   这两笔也按算出来的值计入校验和，所以上面那两行预置值**不能随便改**
//   （改了要同步改 CHK32_EXP）。第 8 节会用另一组预置值再验一次 RMW。
//
//-----------------------------------------------------------------------------
// 时间尺度（为什么参数和真实值不一样）
//-----------------------------------------------------------------------------
//   真实配置是 400kHz / SAMPLE=50ms / TIMEOUT=100ms。
//   仿真里改成 SCL=1MHz、SAMPLE/TIMEOUT/BOOT=1ms：
//   比例关系（时钟分频、轮询次数、超时与采样的相对大小）完全一样，
//   但整体快几十倍。这里验证的是**逻辑**，绝对时间常数在 top.v 的参数里。
//
//   ⚠️ 升级成官方初始化序列后，初始化从 3 笔写变成 **104 笔写**：
//      50MHz + SCL 1MHz 下一笔写约 31us，104 笔 ≈ 3.4ms，
//      再加 boot 1ms + 采样周期 1ms，**第一帧要等约 6ms**。
//      所以第 2 节和第 8 节的等待预算从 6*MS_CYC 放到了 12*MS_CYC
//      （这是"等多久算超时"的预算，不是放宽对结果的断言 ——
//       断言仍然是"必须收到 dist_valid 且距离正确"）。
//
//-----------------------------------------------------------------------------
// 关于上拉
//-----------------------------------------------------------------------------
//   ⚠️ SCL/SDA 是开漏，主机/从机都只会"拉低或释放"。仿真里如果没人上拉，
//      释放时线会停在 z 而不是 1，从机模型和 DUT 都会读到 x，
//      表现成"什么都不动"。所以这里用 pullup 原语顶替板子上的 4.7k 上拉。
//=============================================================================

`timescale 1ns/1ps

module tb_tof_driver;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ  = 50_000_000;
    localparam integer CLK_PERIOD   = 20;              // 20ns = 50MHz
    localparam integer MS_CYC       = CLK_FREQ_HZ / 1000;   // 1ms = 50000 拍

    localparam integer SCL_HZ       = 1_000_000;       // 仿真加速：1MHz
    localparam integer SAMPLE_MS    = 1;
    localparam integer TIMEOUT_MS   = 1;
    localparam integer I2C_TMO_MS   = 1;
    localparam integer BOOT_WAIT_MS = 1;
    localparam integer DIST_MAX_MM  = 2000;
    localparam [6:0]   DEV_ADDR     = 7'h29;           // VL53L0X 默认地址

    // VL53L0X 寄存器
    localparam [7:0] REG_START    = 8'h00;
    localparam [7:0] REG_CLR_INT  = 8'h0B;
    localparam [7:0] REG_STATUS   = 8'h13;
    localparam [7:0] REG_RANGE_MSB = 8'h1E;
    localparam [7:0] REG_RANGE_LSB = 8'h1F;
    localparam [7:0] REG_MSRC      = 8'h60;   // 读改写 |= 0x12
    localparam [7:0] REG_GPIO_MUX  = 8'h84;   // 读改写 &= ~0x10
    localparam [7:0] REG_I2C_MODE = 8'h88;
    localparam [7:0] REG_RATE_MSB = 8'h44;
    localparam [7:0] REG_RATE_LSB = 8'h45;
    localparam [7:0] REG_STOP_VAR = 8'h91;    // stop_variable（每次测距前写回）

    //------------------------------------------------------------------
    // ★ 初始化序列的期望值（改 RTL 步骤表就要同步改这两个）
    //
    //   INIT_WRITES_EXP : 初始化阶段"写数据字节"的总笔数
    //   CHK32_EXP       : 那 104 笔按  chk = chk*31 + {reg,val}  累加的结果
    //
    //   校验和的窗口 = 复位 -> DUT 的 init_ok 置起（见文件头说明）。
    localparam integer INIT_WRITES_EXP = 104;
    localparam [31:0]  CHK32_EXP       = 32'h7F80F14A;

    //------------------------------------------------------------------
    // 读-改-写的从机预置初值（第 2 节用；改了要同步改 CHK32_EXP）
    //   0x60: 0xC0 | 0x12 = 0xD2
    //   0x84: 0xFF & ~0x10 = 0xEF
    //------------------------------------------------------------------
    localparam [7:0] PRESET_60_A = 8'hC0;
    localparam [7:0] PRESET_84_A = 8'hFF;
    // 第 8 节用第二组初值再验一次 RMW（证明它真的"读"了）
    localparam [7:0] PRESET_60_B = 8'h01;     // 0x01 | 0x12 = 0x13
    localparam [7:0] PRESET_84_B = 8'h00;     // 0x00 & ~0x10 = 0x00

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk   = 0;
    reg rst_n = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------
    // I2C 总线（开漏 + 上拉）
    //------------------------------------------------------------------
    wire scl;
    wire sda;

    pullup pu_scl (scl);
    pullup pu_sda (sda);

    // 从机侧的开漏驱动
    reg slave_sda_low = 0;
    reg slave_scl_low = 0;

    assign sda = slave_sda_low ? 1'b0 : 1'bz;
    assign scl = slave_scl_low ? 1'b0 : 1'bz;

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    wire [15:0] dist_mm;
    wire        dist_valid;
    wire        dist_err;

    tof_driver #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .SAMPLE_MS      (SAMPLE_MS),
        .TIMEOUT_MS     (TIMEOUT_MS),
        .I2C_SCL_HZ     (SCL_HZ),
        .I2C_TIMEOUT_MS (I2C_TMO_MS),
        .BOOT_WAIT_MS   (BOOT_WAIT_MS),
        .DIST_MAX_MM    (DIST_MAX_MM),
        .DEV_ADDR       (DEV_ADDR)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .dist_mm    (dist_mm),
        .dist_valid (dist_valid),
        .dist_err   (dist_err),
        .scl        (scl),
        .sda        (sda),
        // 本 tb 是单驱动独占总线：直接给授权，不接仲裁器
        //（多器件共享总线的用例在 tb_top_i2c_share.v 里）
        .bus_grant  (1'b1),
        .bus_req    (),
        .bus_busy   ()
    );

    //------------------------------------------------------------------
    // 结果文件 / 计数器
    //------------------------------------------------------------------
    integer log_fd;
    integer errors = 0;
    integer checks = 0;

    initial begin
        log_fd = $fopen("tof_driver_result.txt", "w");
        if (log_fd == 0) $display("[warn] 无法创建 tof_driver_result.txt");
    end

    task check;
        // ⚠️ name 是字符串，宽度决定能放多少字节。
        //   原来 [255:0] = 32 字节，中文一个字 3 字节，
        //   稍长的检查名会**从前面截掉**（日志里看不到 "2.11" 这种编号）。
        //   本工程其他 tb 已经加宽到 [1023:0]，这里同步（纯基础设施，不影响断言）。
        input [1023:0] name;
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

    //==================================================================
    // 输出脉冲监视（验证"单拍脉冲"）
    //==================================================================
    integer    valid_cnt    = 0;
    integer    err_cnt      = 0;
    integer    valid_double = 0;     // dist_valid 连续两拍 = 不是单拍脉冲
    integer    err_double   = 0;
    reg        prev_valid   = 0;
    reg        prev_err     = 0;
    reg [15:0] last_dist    = 16'h0000;
    integer    cyc          = 0;
    integer    last_valid_cyc = 0;
    integer    valid_period = 0;     // 两次 dist_valid 之间的拍数

    always @(posedge clk) begin
        if (rst_n) begin
            cyc = cyc + 1;
            if (dist_valid) begin
                valid_cnt = valid_cnt + 1;
                last_dist = dist_mm;
                if (prev_valid) valid_double = valid_double + 1;
                if (last_valid_cyc != 0) valid_period = cyc - last_valid_cyc;
                last_valid_cyc = cyc;
            end
            if (dist_err) begin
                err_cnt = err_cnt + 1;
                if (prev_err) err_double = err_double + 1;
            end
            prev_valid = dist_valid;
            prev_err   = dist_err;
        end
    end

    //==================================================================
    // I2C 从机模型（VL53L0X）
    //==================================================================
    reg [7:0]  regs [0:255];          // 寄存器模型
    integer    idx;

    // 行为开关（测试里动态改）
    integer    ready_after  = 2;      // 第几次读 0x13 之后 bit0 才置 1
    integer    no_ack_mode  = 0;      // 1 = 不应答（测 NACK -> dist_err）
    integer    stretch_mode = 0;      // 1 = 拉住 SCL 不放（测超时保护）
    reg [15:0] range_mm     = 16'd1234;   // "传感器测量值"
    reg [15:0] meas_range   = 16'd1234;   // 本次测量锁存值（模拟器件在启动时锁存）
    integer    poll_cnt     = 0;

    // 统计 / 协议监视
    integer start_cnt      = 0;       // START 次数（含重复 START）
    integer rstart_cnt     = 0;       // 重复 START 次数
    integer stop_cnt       = 0;
    integer addr_w_cnt     = 0;       // 收到 dev+W
    integer addr_r_cnt     = 0;       // 收到 dev+R
    integer addr_bad_cnt   = 0;       // 地址不匹配
    integer nack_by_master = 0;       // 主机对读字节发 NACK
    integer hold_err       = 0;       // ★SDA 在 SCL 高电平期间变化 = I2C 违规
    integer ptr_13_cnt     = 0;       // 写寄存器指针 0x13 的次数（轮询）
    integer ptr_1e_cnt     = 0;
    integer ptr_1f_cnt     = 0;

    // 从机内部状态
    reg        s_active    = 0;       // START 之后、STOP 之前
    reg        s_mon       = 0;       // 协议监视窗口（START/STOP 期间关掉）
    reg        s_start_pend = 0;
    reg [7:0]  s_shift     = 8'h00;
    reg [3:0]  s_sc        = 4'd0;    // 本字节已过的 SCL 上升沿数
    reg        s_first     = 1'b1;    // 还没收到本次会话的地址字节
    reg        s_rd        = 1'b0;    // 当前方向 1=从机发
    reg        s_match     = 1'b0;    // 地址命中
    reg [7:0]  s_ptr       = 8'h00;   // 寄存器指针（命中后自动递增）
    reg        s_ptr_pend  = 1'b0;    // 下一个写字节是寄存器指针
    reg [7:0]  s_tx        = 8'hFF;   // 读方向移位输出
    reg        s_ack_seen  = 1'b0;    // 主机对读字节的 ACK/NACK
    reg        s_at_rise   = 1'b1;    // SCL 上升沿时的 SDA（用于稳定性检查）

    //------------------------------------------------------------------
    // ★ 初始化序列校验：写数据字节的统计 + 32 位哈希
    //
    //   hash_en  : 1 = 还在初始化阶段（DUT 的 init_ok 置起后关窗）
    //   chk32    : chk = chk*31 + {reg,val}（32 位回绕）
    //   wr_chk_cnt / chk32 都只统计窗口内的字节，断言的是固定常数
    //------------------------------------------------------------------
    reg        hash_en        = 1'b1;
    reg [31:0] chk32          = 32'd0;
    integer    wr_chk_cnt     = 0;    // 窗口内的写数据字节数（期望 104）
    integer    wr_cnt         = 0;    // 整个仿真的写数据字节数（只用于打印）
    integer    wr91_cnt       = 0;    // 写 0x91(stop_variable) 的次数
    integer    cal41_cnt      = 0;    // 写 0x00=0x41（校准命令）的次数
    integer    ptr13_at_cal   = 0;    // 第一次写 0x41 时的 ptr_13_cnt
    reg [7:0]  wr60_first     = 8'h00; // 复位后第一次写 0x60 的值
    integer    wr60_cnt       = 0;
    reg [7:0]  wr84_first     = 8'h00; // 复位后第一次写 0x84 的值
    integer    wr84_cnt       = 0;
    integer    wr51_cnt       = 0;    // 写 0x51=0x00 的次数
    integer    wr01f8_cnt     = 0;    // 写 0x01=0xF8 的次数
    integer    wr0102_cnt     = 0;    // 写 0x01=0x02（相位校准命令）的次数
    integer    wrff01_cnt     = 0;    // 写 0xFF=0x01（翻到页 1）的次数
    // 闭窗之后的**第一笔**写（用来证明窗口关在初始化/前导的交界处）
    reg        after_end_seen = 1'b0;
    reg [7:0]  after_end_reg  = 8'h00;
    reg [7:0]  after_end_val  = 1'b0;

    // 关窗：DUT 的 init_ok 一置起就说明初始化序列跑完了
    //   ⚠️ 层次引用，需要 vsim -voptargs="+acc"（run_sim.do 一直带着）
    always @(posedge clk) begin
        if (dut.init_ok) hash_en = 1'b0;
    end

    //------------------------------------------------------------------
    // 从机辅助任务
    //------------------------------------------------------------------
    // 写寄存器时器件内部的副作用
    task write_hook;
        begin
            // 0x00 = 0x01：启动一次单次测距 -> 锁存测量值、清轮询计数
            if ((s_ptr == REG_START) && (s_shift == 8'h01)) begin
                poll_cnt   = 0;
                meas_range = range_mm;
            end
        end
    endtask

    // 装载读方向的下一字节
    task prep_read_byte;
        begin
            case (s_ptr)
                REG_STATUS: begin
                    poll_cnt = poll_cnt + 1;
                    s_tx     = (poll_cnt >= ready_after) ? 8'h01 : 8'h00;
                end
                REG_RANGE_MSB: s_tx = meas_range[15:8];
                REG_RANGE_LSB: s_tx = meas_range[7:0];
                default:       s_tx = regs[s_ptr];
            endcase
            s_ptr = s_ptr + 8'd1;      // 器件内部指针自动递增
        end
    endtask

    // 一个字节（含 ACK 拍）收完之后的协议处理
    task byte_done;
        begin
            if (s_first) begin
                //---------------- 地址字节 ----------------
                s_first = 1'b0;
                if (s_shift[7:1] !== DEV_ADDR) begin
                    addr_bad_cnt = addr_bad_cnt + 1;
                    s_match      = 1'b0;
                end else begin
                    s_match = 1'b1;
                    if (s_shift[0] === 1'b1) begin
                        addr_r_cnt = addr_r_cnt + 1;
                        s_rd       = 1'b1;             // 接下来从机发数据
                    end else begin
                        addr_w_cnt = addr_w_cnt + 1;
                        s_rd       = 1'b0;
                        s_ptr_pend = 1'b1;             // 下一个字节是寄存器指针
                    end
                end
            end else if (!s_rd) begin
                //---------------- 写方向的后续字节 ----------------
                if (s_ptr_pend) begin
                    s_ptr      = s_shift;
                    s_ptr_pend = 1'b0;
                    if (s_ptr == REG_STATUS)    ptr_13_cnt = ptr_13_cnt + 1;
                    if (s_ptr == REG_RANGE_MSB) ptr_1e_cnt = ptr_1e_cnt + 1;
                    if (s_ptr == REG_RANGE_LSB) ptr_1f_cnt = ptr_1f_cnt + 1;
                end else begin
                    //--------------------------------------------------
                    // ★ 写数据字节：统计 / 校验和 / 各种捕获
                    //   （s_ptr 此刻还是本次写的寄存器地址，写完才自增）
                    //--------------------------------------------------
                    wr_cnt = wr_cnt + 1;

                    if (s_ptr == REG_STOP_VAR) begin
                        wr91_cnt = wr91_cnt + 1;
                        hash_en  = 1'b0;   // 兜底：已经进"每次测距前"阶段了
                    end
                    if (s_ptr == REG_MSRC) begin
                        if (wr60_cnt == 0) wr60_first = s_shift;
                        wr60_cnt = wr60_cnt + 1;
                    end
                    if (s_ptr == REG_GPIO_MUX) begin
                        if (wr84_cnt == 0) wr84_first = s_shift;
                        wr84_cnt = wr84_cnt + 1;
                    end
                    if ((s_ptr == REG_START) && (s_shift == 8'h41)) begin
                        if (cal41_cnt == 0) ptr13_at_cal = ptr_13_cnt;
                        cal41_cnt = cal41_cnt + 1;
                    end
                    if ((s_ptr == 8'h51) && (s_shift == 8'h00)) wr51_cnt  = wr51_cnt + 1;
                    if ((s_ptr == 8'h01) && (s_shift == 8'hF8)) wr01f8_cnt = wr01f8_cnt + 1;
                    if ((s_ptr == 8'h01) && (s_shift == 8'h02)) wr0102_cnt = wr0102_cnt + 1;
                    if ((s_ptr == 8'hFF) && (s_shift == 8'h01)) wrff01_cnt = wrff01_cnt + 1;

                    if (hash_en) begin
                        chk32      = chk32 * 32'd31 + {16'd0, s_ptr, s_shift};
                        wr_chk_cnt = wr_chk_cnt + 1;
                    end else if (!after_end_seen) begin
                        after_end_reg = s_ptr;
                        after_end_val = s_shift;
                        after_end_seen = 1'b1;
                    end

                    regs[s_ptr] = s_shift;         // 写数据（自动递增）
                    write_hook;
                    s_ptr = s_ptr + 8'd1;
                end
            end
            // 读方向不需要在收完时做什么：主机 ACK 了就在下降沿继续发
        end
    endtask

    // 清总线状态（测试里强制复位从机模型用）
    task slave_clear_bus;
        begin
            slave_sda_low = 1'b0;
            slave_scl_low = 1'b0;
            s_active      = 1'b0;
            s_mon         = 1'b0;
            s_sc          = 4'd0;
            s_start_pend  = 1'b0;
        end
    endtask

    //------------------------------------------------------------------
    // SCL 上升沿：写方向采样数据位；第 9 拍采样主机的 ACK/NACK
    //------------------------------------------------------------------
    always @(posedge scl) begin
        s_at_rise = sda;
        if (s_active) begin
            if (s_sc < 4'd8) begin
                s_shift = {s_shift[6:0], sda};
                s_sc    = s_sc + 4'd1;
            end else if (s_sc == 4'd8) begin
                s_ack_seen = sda;
                s_sc       = 4'd9;
                if (s_rd && (sda === 1'b1)) nack_by_master = nack_by_master + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // SCL 下降沿：推进字节；读方向在这里把下一位推上总线
    //------------------------------------------------------------------
    always @(negedge scl) begin
        // ---- 协议监视：SCL 高电平期间 SDA 不该变（START/STOP 除外）----
        if (s_start_pend) begin
            s_start_pend = 1'b0;
            s_mon        = 1'b1;                  // START 之后的第一个下降沿才开始
        end else if (s_mon) begin
            if (sda !== s_at_rise) hold_err = hold_err + 1;
        end

        // ---- 字节推进 ----
        if (!s_active) begin
            s_sc = 4'd0;
        end else if (s_sc == 4'd8) begin
            // 8 位数据发完，准备第 9 拍（ACK）
            if (s_rd) slave_sda_low = 1'b0;                    // 读：等主机 ACK/NACK
            else      slave_sda_low = (no_ack_mode == 0);      // 写：拉低 = ACK
        end else if (s_sc == 4'd9) begin
            // 一个字节（含 ACK 拍）结束
            byte_done;
            s_sc    = 4'd0;
            s_shift = 8'h00;
            if (s_rd) begin
                if (!s_ack_seen && s_match) begin
                    // 主机 ACK 了 -> 继续发下一个寄存器字节
                    prep_read_byte;
                    slave_sda_low = ~s_tx[7];
                    s_tx          = {s_tx[6:0], 1'b1};
                end else begin
                    slave_sda_low = 1'b0;      // NACK：交还总线
                    s_tx          = 8'hFF;
                end
            end else begin
                slave_sda_low = 1'b0;          // 写方向：释放 SDA
            end
        end else begin
            // s_sc = 0..7：读方向把当前位推上总线
            if (s_rd && s_match) begin
                slave_sda_low = ~s_tx[7];
                s_tx          = {s_tx[6:0], 1'b1};
            end
        end
    end

    //------------------------------------------------------------------
    // START / STOP 检测
    //   SCL 高电平期间 SDA 下降 = START，上升 = STOP
    //
    //   ⚠️ 这里用 #1 错开时刻：SDA 的驱动和这个监视块在同一时刻事件里，
    //      不加延迟会读到变化前的值（delta cycle 竞争）。
    //------------------------------------------------------------------
    always @(sda) begin
        #1;
        if (scl === 1'b1) begin
            if (sda === 1'b0) begin
                //---------------- START / 重复 START ----------------
                start_cnt = start_cnt + 1;
                if (s_active) rstart_cnt = rstart_cnt + 1;
                s_active      = 1'b1;
                s_first       = 1'b1;
                s_match       = 1'b0;
                s_rd          = 1'b0;
                s_sc          = 4'd0;
                s_shift       = 8'h00;
                s_ptr_pend    = 1'b0;
                s_start_pend  = 1'b1;
                slave_sda_low = 1'b0;
                if (stretch_mode) slave_scl_low = 1'b1;   // 时钟拉伸
            end else begin
                //---------------- STOP ----------------
                stop_cnt      = stop_cnt + 1;
                s_active      = 1'b0;
                s_mon         = 1'b0;
                s_sc          = 4'd0;
                slave_sda_low = 1'b0;
                if (stretch_mode) slave_scl_low = 1'b0;
            end
        end
    end

    //==================================================================
    // 工具任务
    //==================================================================
    // 等下一个 dist_valid 或 dist_err（带超时）
    task wait_event;
        input integer max_cyc;
        integer v0;
        integer e0;
        integer k;
        begin
            v0 = valid_cnt;
            e0 = err_cnt;
            k  = 0;
            while ((valid_cnt == v0) && (err_cnt == e0) && (k < max_cyc)) begin
                @(posedge clk);
                #1;
                k = k + 1;
            end
        end
    endtask

    // 复位 + 清从机模型
    task do_reset;
        begin
            rst_n = 1'b0;
            slave_clear_bus;
            // 重新捕获"这次复位之后第一次写 0x60 / 0x84"（第 8 节要用）
            wr60_cnt = 0;
            wr84_cnt = 0;
            repeat (20) @(posedge clk);
            rst_n = 1'b1;
            repeat (2) @(posedge clk);
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer v0;
    integer e0;
    integer d0;

    initial begin
        $fwrite(log_fd, "=== tb_tof_driver: VL53L0X 驱动自检 ===\n");
        $fwrite(log_fd, "  参数: SCL=%0dHz SAMPLE=%0dms TIMEOUT=%0dms BOOT=%0dms DIST_MAX=%0dmm\n",
                SCL_HZ, SAMPLE_MS, TIMEOUT_MS, BOOT_WAIT_MS, DIST_MAX_MM);
        $fwrite(log_fd, "  说明: 仿真里 SCL 1MHz / 三个 ms 参数 1ms 是为了缩短时间,\n");
        $fwrite(log_fd, "        比例关系与真实配置(400kHz/50ms/100ms)一致。\n");

        // 从机寄存器模型清零
        for (idx = 0; idx < 256; idx = idx + 1) regs[idx] = 8'h00;

        // ★ 读-改-写的预置初值（第 2 节）
        //   必须**非 0**、且不能刚好等于掩码结果，否则"读了再改"和
        //   "直接写常数"的结果一样，就测不出区别了。
        //   这里 0x60=0xC0 -> 期望写回 0xC0|0x12 = 0xD2
        //        0x84=0xFF -> 期望写回 0xFF&~0x10 = 0xEF
        //   这两个值参与 CHK32_EXP，改这里必须同步改常数。
        regs[REG_MSRC]     = PRESET_60_A;
        regs[REG_GPIO_MUX] = PRESET_84_A;

        // ★ stop_variable 的预置值：初始化时 DUT 会**读** 0x91 存起来，
        //   在每次测距前**写回**同一个值。给一个非 0 的怪值，
        //   就能证明"写回的是读回来的东西"，而不是写了常数 0。
        //   （第 8 节会换成另一个值再验一次）
        regs[REG_STOP_VAR] = 8'h5A;

        //------------------------------------------------------------------
        // 1. 复位状态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 1. 复位 ---\n");
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        check("1.1 复位后 SCL 释放为高", (scl === 1'b1) ? 1 : 0, 1);
        check("1.2 复位后 SDA 释放为高", (sda === 1'b1) ? 1 : 0, 1);
        check("1.3 复位后 dist_valid=0", dist_valid, 0);
        check("1.4 复位后 dist_err=0",   dist_err,   0);
        rst_n = 1'b1;

        //------------------------------------------------------------------
        // 2. 正常测距一帧（★ 官方初始化序列 + 每次测距前导）
        //    boot 1ms -> 初始化 104 笔写(≈3.4ms) -> 等 1ms -> 前导 7 笔
        //    -> 启动 -> 轮询 0x13 -> 读 0x1E/0x1F -> 清中断
        //
        //    ⚠️ 等待预算用 12*MS_CYC（≈12ms）：升级初始化后第一帧要等约 6ms，
        //       原来的 6ms 只剩不到 20% 余量，太紧。
        //       这是"等多久"的预算，断言本身没有放宽。
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 2. 正常测距（官方初始化 + 前导 + 启动 + 轮询 + 读数）---\n");
        range_mm    = 16'd1234;
        ready_after = 2;          // 前两次读 0x13 返回未就绪 -> 验证轮询确实在轮询
        poll_cnt    = 0;
        wait_event(12 * MS_CYC);

        check("2.1 收到 dist_valid 脉冲", (valid_cnt > 0) ? 1 : 0, 1);
        check("2.2 dist_mm==1234", last_dist, 1234);
        check("2.3 init 写 0x88=0x00", regs[REG_I2C_MODE], 8'h00);
        check("2.4 init 写 0x44=0x00", regs[REG_RATE_MSB], 8'h00);
        check("2.5 init 写 0x45=0x20", regs[REG_RATE_LSB], 8'h20);
        check("2.6 写 0x00=0x01 启动", regs[REG_START], 8'h01);
        check("2.7 写 0x0B=0x01 清中断", regs[REG_CLR_INT], 8'h01);
        check("2.8 轮询 0x13 达 2 次", (ptr_13_cnt >= 2) ? 1 : 0, 1);
        check("2.9 读过寄存器 0x1E", (ptr_1e_cnt > 0) ? 1 : 0, 1);
        check("2.10 读过寄存器 0x1F", (ptr_1f_cnt > 0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 2b. ★ 官方初始化序列：总条数 + 校验和 + 代表项 + 读改写 + 校准
        //------------------------------------------------------------------
        $fwrite(log_fd, "  [info] 初始化阶段写入 %0d 笔, 校验和 = 0x%08h (期望 0x%08h)\n",
                wr_chk_cnt, chk32, CHK32_EXP);
        $fwrite(log_fd, "  [info] 闭窗后第一笔写 = 0x%02h=0x%02h (期望 0x80=0x01)\n",
                after_end_reg, after_end_val);

        // 总条数：初始化阶段的**写数据字节**数，漏抄/多抄一条立刻红。
        //   它与步数的关系：107 步 = 104 写 + 1 次读存（0x91）+ 2 次轮询
        check("2.11 init 写入总数=104", wr_chk_cnt, INIT_WRITES_EXP);
        // 校验和：值、顺序、条数一起钉死
        check("2.12 init 序列校验和匹配", (chk32 === CHK32_EXP) ? 1 : 0, 1);
        // 窗口边界：闭窗后第一笔必须是"每次测距前导"的第一句
        check("2.13 闭窗后首笔=0x80=0x01",
              ((after_end_reg == 8'h80) && (after_end_val == 8'h01)) ? 1 : 0, 1);
        // tuning 表代表项（挑"非 0 才有意义"的；0 值的项由校验和+计数覆盖）
        check("2.14 tuning 0x4E=0x2C",      regs[8'h4E], 8'h2C);
        check("2.15 tuning 0x52=0x96",      regs[8'h52], 8'h96);
        check("2.16 tuning 0x51=0x00 写过", wr51_cnt, 1);
        check("2.17 tuning 0x70=0x04",      regs[8'h70], 8'h04);
        check("2.18 tuning 0x71=0x01",      regs[8'h71], 8'h01);
        check("2.19 tuning 0x72=0xFE",      regs[8'h72], 8'hFE);
        // 翻页：写过页 1（0xFF=0x01），最后停在页 0（0xFF=0x00）
        check("2.20 翻页 0xFF=0x01 写过",   (wrff01_cnt >= 1) ? 1 : 0, 1);
        check("2.21 停在页 0 (0xFF=0x00)",  regs[8'hFF], 8'h00);
        // 0x01 = 0xF8 然后 0x01 = 0xE8
        check("2.22 写 0x01=0xF8 一次",     wr01f8_cnt, 1);
        check("2.23 最终 0x01=0xE8",        regs[8'h01], 8'hE8);
        // 中断/GPIO
        check("2.24 写 0x0A=0x04",          regs[8'h0A], 8'h04);
        // 两处读-改-写：预置 0xC0/0xFF -> 写回 0xD2/0xEF
        check("2.25 0x60 读改写 |=0x12",    wr60_first, 8'hD2);
        check("2.26 0x84 读改写 &=~0x10",   wr84_first, 8'hEF);
        check("2.27 0x84 最终值=0xEF",      regs[REG_GPIO_MUX], 8'hEF);
        // 参考校准：官方是"两段"——
        //   ① VHV 校准：0x01=0x01 + 0x00=0x41 + 轮询 + 清中断 + 0x00=0x00
        //   ② 相位校准：0x01=0x02 + 0x00=0x01 + 轮询 + 清中断 + 0x00=0x00
        //   ⚠️ 所以 0x41 只写 **1 次**（第二段用的是 0x00=0x01），别写成 2 次。
        check("2.28 校准VHV命令0x41写1次",  cal41_cnt, 1);
        check("2.29 校准后轮询过 0x13",     (ptr_13_cnt > ptr13_at_cal) ? 1 : 0, 1);
        // stop_variable 的"读出来 -> 测距前写回"闭环（预置 0x5A）
        check("2.30 stop_var 读回并回写=0x5A", regs[REG_STOP_VAR], 8'h5A);
        // 相位校准命令 0x01=0x02（证明第二段也跑了；后面会被 0x01=0xE8 覆盖，
        // 所以只能靠计数器，不能看 regs[]）
        check("2.31 相位校准命令0x01=0x02", wr0102_cnt, 1);

        //------------------------------------------------------------------
        // 3. I2C 时序 / 协议
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 3. I2C 时序与协议 ---\n");
        check("3.1 总线上出现过 START",            (start_cnt > 0) ? 1 : 0, 1);
        check("3.2 总线上出现过 STOP",             (stop_cnt  > 0) ? 1 : 0, 1);
        check("3.3 从机收 地址+W",   (addr_w_cnt > 0) ? 1 : 0, 1);
        check("3.4 从机收 地址+R",   (addr_r_cnt > 0) ? 1 : 0, 1);
        check("3.5 没有出现地址不匹配",            addr_bad_cnt, 0);
        check("3.6 ★复合读用重复START", (rstart_cnt > 0) ? 1 : 0, 1);
        check("3.7 主机末字节发NACK",         (nack_by_master > 0) ? 1 : 0, 1);
        check("3.8 ★SCL高时SDA稳定", hold_err, 0);

        //------------------------------------------------------------------
        // 4. 采样周期 + 单拍脉冲
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 4. 周期采样 / 单拍脉冲 ---\n");
        wait_event(4 * MS_CYC);
        check("4.1 出现第二帧", (valid_cnt >= 2) ? 1 : 0, 1);
        check("4.2 采样间隔 >= SAMPLE_MS",      (valid_period >= MS_CYC) ? 1 : 0, 1);
        check("4.3 采样间隔 <= 3*SAMPLE_MS",    (valid_period <= (3 * MS_CYC)) ? 1 : 0, 1);
        check("4.4 dist_valid 单拍", valid_double, 0);
        check("4.5 dist_err 单拍", err_double,   0);
        // ★ 每次单次测距前都要写 stop_variable（0x91）——
        //   这一串以前**完全没有**，是"读不到数"的关键之一。
        //   断言：0x91 的写次数 >= 成功采样次数（每帧至少一次）。
        check("4.6 每帧前写 0x91=stop_var", (wr91_cnt >= valid_cnt) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 5. 读数合理性：0 与超量程都算无效
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 5. 读数合理性检查 ---\n");

        range_mm = 16'd3000;                 // 超量程
        e0 = err_cnt;
        v0 = valid_cnt;
        d0 = last_dist;
        wait_event(4 * MS_CYC);
        check("5.1 超量程->dist_err", (err_cnt > e0) ? 1 : 0, 1);
        check("5.2 超量程无 valid",    (valid_cnt == v0) ? 1 : 0, 1);
        check("5.3 无效不刷 dist_mm",     last_dist, d0);

        range_mm = 16'd0;                    // 0 = 没测到
        e0 = err_cnt;
        wait_event(4 * MS_CYC);
        check("5.4 读数为 0 -> dist_err", (err_cnt > e0) ? 1 : 0, 1);

        range_mm = 16'd1500;                 // 恢复正常
        v0 = valid_cnt;
        wait_event(4 * MS_CYC);
        check("5.5 恢复->dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        check("5.6 恢复后 dist_mm == 1500",       last_dist, 1500);

        //------------------------------------------------------------------
        // 6. 从机不应答(NACK) -> dist_err
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 6. 从机不应答 ---\n");
        no_ack_mode = 1;
        range_mm    = 16'd700;
        do_reset;                            // 复位后重新初始化 -> 初始化就会失败
        v0 = valid_cnt;
        e0 = err_cnt;
        repeat (3 * MS_CYC) @(posedge clk);
        check("6.1 NACK->dist_err", (err_cnt > e0) ? 1 : 0, 1);
        check("6.2 NACK 无 valid", (valid_cnt == v0) ? 1 : 0, 1);

        //------------------------------------------------------------------
        // 7. 时钟拉伸 / 超时保护
        //    从机在 START 之后把 SCL 拉住不放，主机必须在 I2C_TIMEOUT_MS 后
        //    放弃并释放总线（不能永远等）
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 7. ★时钟拉伸 / 超时保护 ---\n");
        no_ack_mode  = 0;
        stretch_mode = 1;
        do_reset;
        e0 = err_cnt;
        repeat (12 * MS_CYC) @(posedge clk);
        check("7.1 ★拉低SCL->超时", (err_cnt > e0) ? 1 : 0, 1);

        // 检查"超时后总线被释放"时必须让总线安静下来：
        //   如果只是放开 stretch，DUT 可能正好在发下一笔重试交易，
        //   SCL 本来就会一会儿低一会儿高 —— 那样检查结果是随机的。
        //   所以先按住复位（DUT 一定释放总线），再看 SCL/SDA 是不是被上拉回高。
        stretch_mode = 0;
        rst_n        = 1'b0;
        slave_clear_bus;                     // 放开从机侧
        repeat (20) @(posedge clk);
        check("7.2 ★超时后 SCL 释放", (scl === 1'b1) ? 1 : 0, 1);
        check("7.3 ★超时后 SDA 释放", (sda === 1'b1) ? 1 : 0, 1);
        rst_n = 1'b1;

        //------------------------------------------------------------------
        // 8. 恢复：总线放开后能重新正常出数
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 8. 恢复 ---\n");
        stretch_mode = 0;
        slave_clear_bus;
        range_mm    = 16'd800;
        ready_after = 1;
        poll_cnt    = 0;

        // ★ 用**第二组**预置初值再验一次读-改-写：
        //     0x60 = 0x01 -> 写回 0x01|0x12 = 0x13（不是上一轮的 0xD2）
        //     0x84 = 0x00 -> 写回 0x00&~0x10 = 0x00（不是上一轮的 0xEF）
        //   若实现是"直接写常数"而不是"读了再改"，这里必然对不上。
        //   （do_reset 会把"第一次写 0x60/0x84"的捕获重新武装）
        regs[REG_MSRC]     = PRESET_60_B;
        regs[REG_GPIO_MUX] = PRESET_84_B;
        // stop_variable 也换一个值：写回值应随预置值变化（证明是读回来的）
        regs[REG_STOP_VAR] = 8'h3C;

        do_reset;
        v0 = valid_cnt;
        wait_event(12 * MS_CYC);        // 预算同第 2 节：升级初始化后第一帧要 ≈6ms
        check("8.1 恢复->dist_valid", (valid_cnt > v0) ? 1 : 0, 1);
        check("8.2 恢复后 dist_mm == 800",         last_dist, 800);
        check("8.3 再次初始化 0x60 读改写=0x13",   wr60_first, 8'h13);
        check("8.4 再次初始化 0x84 读改写=0x00",   wr84_first, 8'h00);
        check("8.5 再次回写 stop_var=0x3C",        regs[REG_STOP_VAR], 8'h3C);

        //------------------------------------------------------------------
        // 9. 复位清状态
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n--- 9. 复位清状态 ---\n");
        rst_n = 1'b0;
        slave_clear_bus;
        repeat (5) @(posedge clk);
        check("9.1 复位期间 dist_valid=0", dist_valid, 0);
        check("9.2 复位期间 dist_err=0",   dist_err,   0);
        check("9.3 复位总线释放", (scl === 1'b1) ? 1 : 0, 1);
        rst_n = 1'b1;

        //------------------------------------------------------------------
        // 汇总
        //------------------------------------------------------------------
        $fwrite(log_fd, "\n");
        if (errors == 0) begin
            $fwrite(log_fd, "=== tb_tof_driver: %0d checks, 0 errors ===\n", checks);
            $fwrite(log_fd, "RESULT: PASS\n");
        end else begin
            $fwrite(log_fd, "=== tb_tof_driver: %0d checks, %0d errors ===\n", checks, errors);
            $fwrite(log_fd, "RESULT: FAIL\n");
        end
        $fclose(log_fd);
        $finish;
    end

    //------------------------------------------------------------------
    // 仿真总超时（防止 DUT 卡住导致仿真永不结束）
    //------------------------------------------------------------------
    initial begin
        #(60_000_000);                       // 60ms
        $fwrite(log_fd, "\n[FATAL] 仿真超时（DUT 可能卡死）\n");
        $fwrite(log_fd, "RESULT: FAIL\n");
        $fclose(log_fd);
        $finish;
    end

endmodule
