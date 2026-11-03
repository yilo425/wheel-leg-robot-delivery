//=============================================================================
// module : i2c_master
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : 主机模式 I2C 控制器（7 位地址 + 单字节读写 + 复合"先写寄存器地址再读"）。
//        TOF(VL53L0X) 和 IMU(MPU6050) 共用一路 I2C，所以本模块只做"通用搬运工"，
//        读哪个寄存器、多久读一次由上层（tof_driver / imu_driver）决定。
//
//-----------------------------------------------------------------------------
// 支持的四种传输（由 rd / reg_addr_en 选择）
//-----------------------------------------------------------------------------
//   A) 写单字节              start=1, rd=0, reg_addr_en=0
//        START | dev+W | ACK | wr_data | ACK | STOP
//   B) 写寄存器              start=1, rd=0, reg_addr_en=1
//        START | dev+W | ACK | reg_addr | ACK | wr_data | ACK | STOP
//        ★ VL53L0X/MPU6050 的"配置寄存器写"就是这一种
//   C) 复合读（关键）        start=1, rd=1,  reg_addr_en=1
//        START | dev+W | ACK | reg_addr | ACK | 重复START | dev+R | ACK | data | NACK | STOP
//        ★ TOF 读 0x13/0x1E/0x1F、IMU 读数据寄存器都是这一种
//   D) 单纯读                start=1, rd=1,  reg_addr_en=0
//        START | dev+R | ACK | data | NACK | STOP
//
//   ⚠️ 为什么端口比最初设想多了 reg_addr / reg_addr_en：
//      TOF 和 IMU 都要求"先写寄存器地址、再读数据"。如果只给一个 wr_data，
//      上层就得自己拼两次传输，而两次传输之间的 STOP 会让器件把内部寄存器
//      指针复位（不少器件 STOP 后会重置指针），读出来就是错的。
//      必须在**一次**传输里用重复 START（而不是 STOP+START）把方向翻过来。
//      所以复合读必须在本模块内部完成，端口上就得把寄存器地址暴露出来。
//
//-----------------------------------------------------------------------------
// 时序与参数
//-----------------------------------------------------------------------------
//   SCL 半周期 = CLK_FREQ_HZ / (2 * SCL_HZ) 个 clk（四舍五入）。
//   50MHz + 400kHz -> 63 个 clk，实际约 397kHz（-0.8%，从机毫无压力）。
//
//   ⚠️ 时钟拉伸（clock stretching）与超时保护：
//      本模块在"SCL 应当为高"的半周期里会**等 SCL 真的变高**再往下走，
//      所以从机拉低 SCL 拖时间是可以忍的（正常 I2C 行为，VL53L0X 会这么干）。
//      但等待有上限：连续等超过 TIMEOUT_MS 就认定总线被卡死，立刻释放
//      SCL/SDA、置 ack_err、给 done 脉冲退出。
//      另有一个覆盖整个传输的总超时，保证任何状态都不会把人困住。
//      （项目约定：所有等待都必须有超时。）
//
//   ⚠️ SCL/SDA 是开漏，模块只会"拉低"或"释放"，**外部必须有上拉电阻**
//      （板子上 4.7k 到 3.3V）。仿真里若没有上拉，线会是 x，
//      testbench 用 pullup 原语顶替上拉。
//
//-----------------------------------------------------------------------------
// 用法约定（写错的症状是"数据偶尔错一位"，很难查）
//-----------------------------------------------------------------------------
//   1) start 是单拍脉冲；busy=1 期间 dev_addr/rd/wr_data/reg_addr/reg_addr_en
//      **必须保持不变**（本模块在进 S_START 时就锁存了要发的字节）。
//      tof_driver 的做法：先把地址/数据写进寄存器，再拉一拍 t_req。
//   2) done 是单拍脉冲。done=1 时 rd_data / ack_err 才是最终值。
//   3) ack_err=1 的两种原因：从机没应答（NACK）或超时。
//      出错时本模块仍然会发 STOP 把总线释放掉，不会把总线占住。
//   4) SDA 只在 SCL 低电平半周期里换值，而且是等 SCL 确实低了一小段之后
//      （tHD;DAT），保证"SCL 高电平期间 SDA 稳定"这条 I2C 铁律。
//=============================================================================

`timescale 1ns/1ps

module i2c_master #(
    //----------------------------------------------------------------
    // 时钟与速率
    //----------------------------------------------------------------
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SCL_HZ      = 400_000,     // 400kHz 快速模式

    //----------------------------------------------------------------
    // 超时（毫秒）
    //   10ms 对一次 I2C 传输（几百 us）来说非常宽松；
    //   仿真里传 1 可以加快验证。
    //----------------------------------------------------------------
    parameter integer TIMEOUT_MS  = 10,

    //----------------------------------------------------------------
    // 参数自检（只有 initial 里的 $display，综合器忽略）
    //----------------------------------------------------------------
    parameter integer _ASSERT_EN  = 1
) (
    input  wire        clk,
    input  wire        rst_n,          // 低有效

    //---------------- 命令接口（给上层驱动用）----------------
    input  wire        start,          // 单拍脉冲：发起一次传输
    input  wire [6:0]  dev_addr,       // 7 位从机地址（不含读写位）
    input  wire        rd,             // 1=读 0=写
    input  wire [7:0]  wr_data,        // 要写的数据字节
    input  wire        reg_addr_en,    // 1=本次传输带寄存器地址（B/C 类型）
    input  wire [7:0]  reg_addr,       // 寄存器地址

    //---------------- 结果与状态 ----------------
    output reg  [7:0]  rd_data,        // 读到的数据（done=1 时有效）
    output reg         busy,           // 1=正在传输
    output reg         ack_err,        // 从机没应答 / 超时（done=1 时有效）
    output reg         done,           // 单拍脉冲：本次传输结束

    //---------------- I2C 物理接口（开漏，需外部上拉）----------------
    inout  wire        scl,
    inout  wire        sda
);

    //================================================================
    // 参数换算
    //
    // ⚠️ 全部声明成定宽 [31:0]，不做 parameter 位选。
    //    Verilog-2001 不允许对 parameter/localparam 做位选，本工程为此踩过 4 次，
    //    每次都是"vlog 不报、elaboration 才炸、报错还不给名字"。
    //================================================================
    localparam [31:0] QDIV      = (CLK_FREQ_HZ + SCL_HZ) / (SCL_HZ * 2);  // 半个 SCL 周期
    localparam [31:0] QDIV_HALF = (QDIV < 32'd4) ? 32'd1 : (QDIV / 32'd2); // SDA 换值点

    // 超时上限（先除 1000 再乘，避免 TIMEOUT_MS*CLK_FREQ_HZ 溢出 32 位）
    localparam [31:0] TMO_CYC   = TIMEOUT_MS * (CLK_FREQ_HZ / 1000);
    localparam [31:0] LIMIT_CYC = (TMO_CYC < 32'd1) ? 32'd1 : TMO_CYC;

    //================================================================
    // 开漏三态：只"拉低"或"释放"，永不强驱高
    //================================================================
    reg scl_oe;      // 1 = 把 SCL 拉低
    reg sda_oe;      // 1 = 把 SDA 拉低

    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    //================================================================
    // 输入同步（异步总线信号必须打两拍再用）
    //   复位值给 1 = "总线空闲（被上拉）"，否则 rst_n 期间会读成 x
    //================================================================
    reg scl_s1, scl_s2;
    reg sda_s1, sda_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_s1 <= 1'b1;
            scl_s2 <= 1'b1;
            sda_s1 <= 1'b1;
            sda_s2 <= 1'b1;
        end else begin
            scl_s1 <= scl;
            scl_s2 <= scl_s1;
            sda_s1 <= sda;
            sda_s2 <= sda_s1;
        end
    end

    //================================================================
    // 状态定义
    //
    //   一个"字节"= 8 个数据位 + 1 个 ACK 位，用 bit_idx(0..8) 计数；
    //   每个位分两个半周期：bit_low=1 低电平半周期（准备数据），
    //                       bit_low=0 高电平半周期（双方采样）。
    //
    //   START / STOP 各 4 个子步骤，因为"SDA 在 SCL 高电平期间变化"
    //   这两个特例必须显式摆出来（否则就违反了 I2C 的换数据规则）。
    //================================================================
    localparam [3:0]
        S_IDLE  = 4'd0,
        S_START = 4'd1,     // 产生 START（重复 START 也走这里，见 start_rep）
        S_ADDRW = 4'd2,     // 地址 + W，第 9 拍采样 ACK
        S_REGWR = 4'd3,     // 寄存器地址字节
        S_ADDRR = 4'd4,     // 地址 + R
        S_RDATA = 4'd5,     // 读数据字节（主机在 ACK 位发 NACK 结束）
        S_WDATA = 4'd6,     // 写数据字节
        S_STOP  = 4'd7,     // 产生 STOP
        S_DONE  = 4'd8;     // 收尾，给 done 脉冲

    reg [3:0]  state;
    reg [2:0]  step;        // START / STOP 内部子步骤
    reg [3:0]  bit_idx;     // 0..7 数据位，8 = ACK 位
    reg        bit_low;     // 1 = 当前在 SCL 低电平半周期
    reg [31:0] qcnt;        // 半周期内的 clk 计数
    reg [7:0]  tx_shift;    // 发送移位寄存器（MSB 先出）
    reg [7:0]  rd_shift;    // 接收移位寄存器
    reg        start_rep;   // 1 = 这次 S_START 是复合读里的重复 START

    reg [31:0] stretch_cnt; // SCL 被拉住不放的计数
    reg [31:0] total_cnt;   // 整个传输的计数

    // 半周期时间到
    wire q_tick = (qcnt == (QDIV - 32'd1));

    // 当前状态是不是"字节传输"状态
    wire in_byte = (state == S_ADDRW) || (state == S_REGWR) || (state == S_ADDRR) ||
                   (state == S_RDATA) || (state == S_WDATA);

    // 读字节：数据由从机驱动，主机整字节释放 SDA
    wire is_read_byte = (state == S_RDATA);

    // 要发送的下一位（0 才需要拉低）
    wire tx_bit_oe = ~tx_shift[7];

    //================================================================
    // 超时看门狗
    //   scl_need_high  ：本该看到 SCL 高电平的时刻（从机可能在做时钟拉伸）
    //   stretch_timeout：SCL 被拉低超过 TIMEOUT_MS
    //   total_timeout  ：整个传输超过 TIMEOUT_MS
    //================================================================
    wire scl_need_high = ((state == S_START) && (step == 3'd1 || step == 3'd2)) ||
                         ((state == S_STOP)  && (step == 3'd1 || step == 3'd2)) ||
                         (in_byte && (bit_low == 1'b0));

    wire stretch_timeout = busy && scl_need_high && (scl_s2 != 1'b1) &&
                           (stretch_cnt >= LIMIT_CYC);
    wire total_timeout   = busy && (total_cnt >= LIMIT_CYC);
    wire timeout_abort   = (stretch_timeout || total_timeout) &&
                           (state != S_IDLE) && (state != S_DONE);

    //================================================================
    // 主状态机
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            step        <= 3'd0;
            bit_idx     <= 4'd0;
            bit_low     <= 1'b1;
            qcnt        <= 32'd0;
            tx_shift    <= 8'h00;
            rd_shift    <= 8'h00;
            rd_data     <= 8'h00;
            start_rep   <= 1'b0;
            scl_oe      <= 1'b0;
            sda_oe      <= 1'b0;
            busy        <= 1'b0;
            ack_err     <= 1'b0;
            done        <= 1'b0;
            stretch_cnt <= 32'd0;
            total_cnt   <= 32'd0;
        end else begin
            // done 是单拍脉冲：默认清零，只有 S_DONE / 超时那一拍才置起
            done <= 1'b0;

            //---------------- 看门狗 ----------------
            if (!busy) begin
                stretch_cnt <= 32'd0;
                total_cnt   <= 32'd0;
            end else begin
                total_cnt <= total_cnt + 32'd1;
                if (scl_need_high && (scl_s2 != 1'b1)) begin
                    if (stretch_cnt < LIMIT_CYC) stretch_cnt <= stretch_cnt + 32'd1;
                end else begin
                    stretch_cnt <= 32'd0;
                end
            end

            if (timeout_abort) begin
                // 超时：立刻放弃并释放总线。
                // ⚠️ 绝不能停在某个状态里一直拉着 SCL/SDA —— 那会把整条 I2C 总线
                //    （TOF + IMU 共用）一起拖死，而且上电时序里没人来救。
                scl_oe  <= 1'b0;
                sda_oe  <= 1'b0;
                ack_err <= 1'b1;
                busy    <= 1'b0;
                done    <= 1'b1;
                state   <= S_IDLE;
                step    <= 3'd0;
                bit_low <= 1'b1;
                qcnt    <= 32'd0;
            end else begin
                case (state)

                //--------------------------------------------------
                // 空闲：等 start
                //--------------------------------------------------
                S_IDLE: begin
                    scl_oe    <= 1'b0;
                    sda_oe    <= 1'b0;
                    busy      <= 1'b0;
                    qcnt      <= 32'd0;
                    bit_low   <= 1'b1;
                    step      <= 3'd0;
                    start_rep <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        ack_err <= 1'b0;     // 新一次传输，清上次的错误
                        state   <= S_START;
                        step    <= 3'd0;
                        qcnt    <= 32'd0;
                    end
                end

                //--------------------------------------------------
                // START / 重复 START
                //   step0: SCL 低，SDA 释放（保证 SDA 为高）
                //   step1: SCL 释放 -> 高
                //   step2: SCL 高时把 SDA 拉低      <- 这才是 START 条件
                //   step3: SCL 拉低，准备第一个字节
                //--------------------------------------------------
                S_START: begin
                    case (step)
                        3'd0: begin
                            scl_oe <= 1'b1;
                            sda_oe <= 1'b0;
                            if (q_tick) begin step <= 3'd1; qcnt <= 32'd0; end
                            else qcnt <= qcnt + 32'd1;
                        end
                        3'd1: begin
                            scl_oe <= 1'b0;
                            sda_oe <= 1'b0;
                            if (q_tick && scl_s2) begin step <= 3'd2; qcnt <= 32'd0; end
                            else if (!q_tick) qcnt <= qcnt + 32'd1;
                        end
                        3'd2: begin
                            scl_oe <= 1'b0;
                            sda_oe <= 1'b1;      // SCL 高 -> SDA 下降 = START
                            if (q_tick && scl_s2) begin step <= 3'd3; qcnt <= 32'd0; end
                            else if (!q_tick) qcnt <= qcnt + 32'd1;
                        end
                        default: begin
                            scl_oe <= 1'b1;
                            sda_oe <= 1'b1;
                            if (q_tick) begin
                                qcnt    <= 32'd0;
                                bit_idx <= 4'd0;
                                bit_low <= 1'b1;
                                if (start_rep) begin
                                    // 重复 START：直接发"地址 + R"
                                    state    <= S_ADDRR;
                                    tx_shift <= {dev_addr, 1'b1};
                                end else if (!rd) begin
                                    // 写：先发地址 + W
                                    state    <= S_ADDRW;
                                    tx_shift <= {dev_addr, 1'b0};
                                end else if (reg_addr_en) begin
                                    // 复合读：先发地址 + W 去写寄存器地址
                                    state    <= S_ADDRW;
                                    tx_shift <= {dev_addr, 1'b0};
                                end else begin
                                    // 单纯读：直接发地址 + R
                                    state    <= S_ADDRR;
                                    tx_shift <= {dev_addr, 1'b1};
                                end
                            end else begin
                                qcnt <= qcnt + 32'd1;
                            end
                        end
                    endcase
                end

                //--------------------------------------------------
                // 字节传输（地址 / 寄存器地址 / 写数据 / 读数据）
                //
                //   低电平半周期：主机准备数据（读时释放 SDA 交给从机）
                //   高电平半周期：双方采样。本模块在"高电平半周期结束点"
                //                 采样 SDA：
                //                   写/地址字节 -> 这 1 位就是从机的 ACK
                //                   读字节     -> 这 1 位就是数据位
                //--------------------------------------------------
                S_ADDRW, S_REGWR, S_ADDRR, S_RDATA, S_WDATA: begin
                    if (bit_low) begin
                        //-------- SCL 低电平半周期 --------
                        scl_oe <= 1'b1;
                        // ⚠️ 等 SCL 确实低了一小段之后才换 SDA（tHD;DAT 保持时间）。
                        //    若与 SCL 下降沿同一拍换值，从机可能采到变化瞬间，
                        //    症状是"偶尔错一位"，极难定位。
                        if (qcnt >= QDIV_HALF) begin
                            if (is_read_byte || (bit_idx == 4'd8)) begin
                                sda_oe <= 1'b0;          // 读 / ACK 位：释放
                            end else begin
                                sda_oe <= tx_bit_oe;     // 写：驱动当前位
                            end
                        end
                        if (q_tick) begin bit_low <= 1'b0; qcnt <= 32'd0; end
                        else qcnt <= qcnt + 32'd1;
                    end else begin
                        //-------- SCL 高电平半周期 --------
                        scl_oe <= 1'b0;
                        if (is_read_byte || (bit_idx == 4'd8)) sda_oe <= 1'b0;

                        if (q_tick && scl_s2) begin
                            qcnt    <= 32'd0;
                            bit_low <= 1'b1;
                            if (bit_idx == 4'd8) begin
                                //================ 一个字节（含 ACK 位）结束 ================
                                bit_idx <= 4'd0;
                                if (is_read_byte) rd_data <= rd_shift;   // 整字节锁存

                                if (is_read_byte) begin
                                    // 单字节读：ACK 位已由主机发 NACK -> 收 STOP
                                    state <= S_STOP;
                                    step  <= 3'd0;
                                end else if (sda_s2 != 1'b0) begin
                                    // ★ 没等到 ACK：从机不在 / 地址不对 / 总线异常
                                    //   置错误并立刻 STOP 释放总线（不再往下发数据）
                                    ack_err <= 1'b1;
                                    state   <= S_STOP;
                                    step    <= 3'd0;
                                end else begin
                                    //---------------- 收到 ACK，按传输类型推进 ----------------
                                    if (state == S_ADDRW) begin
                                        if (reg_addr_en) begin
                                            state    <= S_REGWR;
                                            tx_shift <= reg_addr;
                                        end else begin
                                            state    <= S_WDATA;
                                            tx_shift <= wr_data;
                                        end
                                    end else if (state == S_REGWR) begin
                                        // ⚠️ 寄存器地址之后去哪，取决于方向：
                                        //   读(复合读) -> 重复 START 翻方向
                                        //   写(写寄存器) -> 直接把数据字节发出去
                                        //   这两种都带 reg_addr_en，仅靠 reg_addr_en
                                        //   区分不出来，必须看 rd。
                                        if (rd) begin
                                            start_rep <= 1'b1;
                                            state     <= S_START;
                                            step      <= 3'd0;
                                        end else begin
                                            state    <= S_WDATA;
                                            tx_shift <= wr_data;
                                        end
                                    end else if (state == S_ADDRR) begin
                                        state    <= S_RDATA;
                                        rd_shift <= 8'h00;
                                    end else begin
                                        // S_WDATA：写完一个字节 -> STOP
                                        state <= S_STOP;
                                        step  <= 3'd0;
                                    end
                                end
                            end else begin
                                // 未到 ACK 位：推进一位
                                bit_idx <= bit_idx + 4'd1;
                                if (is_read_byte) begin
                                    rd_shift <= {rd_shift[6:0], sda_s2};
                                end else begin
                                    tx_shift <= {tx_shift[6:0], 1'b0};
                                end
                            end
                        end else if (!q_tick) begin
                            qcnt <= qcnt + 32'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // STOP
                //   step0: SCL 低，SDA 拉低
                //   step1: SCL 释放 -> 高（SDA 仍低）
                //   step2: SCL 高时释放 SDA -> 上升沿 = STOP
                //   step3: 再等半周期，收尾
                //--------------------------------------------------
                S_STOP: begin
                    case (step)
                        3'd0: begin
                            scl_oe <= 1'b1;
                            if (qcnt >= QDIV_HALF) sda_oe <= 1'b1;
                            if (q_tick) begin step <= 3'd1; qcnt <= 32'd0; end
                            else qcnt <= qcnt + 32'd1;
                        end
                        3'd1: begin
                            scl_oe <= 1'b0;
                            if (q_tick && scl_s2) begin step <= 3'd2; qcnt <= 32'd0; end
                            else if (!q_tick) qcnt <= qcnt + 32'd1;
                        end
                        3'd2: begin
                            scl_oe <= 1'b0;
                            sda_oe <= 1'b0;      // SCL 高 -> SDA 上升 = STOP
                            if (q_tick && scl_s2) begin step <= 3'd3; qcnt <= 32'd0; end
                            else if (!q_tick) qcnt <= qcnt + 32'd1;
                        end
                        default: begin
                            scl_oe <= 1'b0;
                            sda_oe <= 1'b0;
                            if (q_tick) begin state <= S_DONE; qcnt <= 32'd0; step <= 3'd0; end
                            else qcnt <= qcnt + 32'd1;
                        end
                    endcase
                end

                //--------------------------------------------------
                // 收尾：done 一拍，释放总线
                //--------------------------------------------------
                S_DONE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    busy   <= 1'b0;
                    done   <= 1'b1;
                    state  <= S_IDLE;
                    qcnt   <= 32'd0;
                end

                default: begin
                    state  <= S_IDLE;
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    busy   <= 1'b0;
                end
                endcase
            end
        end
    end

    //================================================================
    // 参数自检（综合器忽略 initial，仿真里能立刻看出参数配错）
    //================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            if (QDIV < 32'd8) begin
                $display("[i2c_master] ERROR: SCL_HZ=%0d 相对 CLK_FREQ_HZ=%0d 太高 (QDIV=%0d < 8)",
                         SCL_HZ, CLK_FREQ_HZ, QDIV);
            end
            if (TIMEOUT_MS < 1) begin
                $display("[i2c_master] ERROR: TIMEOUT_MS 必须 >= 1");
            end
            $display("[i2c_master] CLK=%0dHz SCL=%0dHz -> QDIV=%0d (实际 SCL=%0dHz)",
                     CLK_FREQ_HZ, SCL_HZ, QDIV, CLK_FREQ_HZ / (2 * QDIV));
        end
    end

endmodule
