//=============================================================================
// module : config_rx
// layer  : 应用层（通信）
//-----------------------------------------------------------------------------
// 功能
//-----------------------------------------------------------------------------
//   uart_frame 收到一帧 0x04 CONFIG，本模块把载荷
//
//       +--------+--------+--------+--------+
//       | 参数ID | 长度   | 值 ...          |
//       | 1B     | 1B     | 长度 字节        |
//       +--------+--------+--------+--------+
//
//   解析成"改哪个参数、改成多少"，做**范围钳位**后写进配置寄存器。
//   多字节值一律**大端**（高字节在前）。
//
//   输出的配置寄存器**任何时刻都有效**（复位后 = DEF_* 默认值），
//   上层（main_fsm / motion_decision / servo_lift）直接当常量用，
//   不需要判断"配置到底有没有下发过"。
//
//-----------------------------------------------------------------------------
// 支持的参数 ID（协议 §3.2 + 本工程扩展 ★）
//-----------------------------------------------------------------------------
//   ID     含义              长度   区间（写进去之前先钳位）
//   ----   ----------------   ----   -------------------------
//   0x01   最大速度           2B     1 .. 200
//   0x02   避障停车距离       2B     50 .. 2000 mm
//   0x03   测速窗口           2B     —— **只有忽略**，见下
//   0x04★  避障减速距离       2B     50 .. 3000 mm（且必须 > 停车距离）
//   0x10   升降腿抬升高度     1B     0 .. 100
//   0x11   升降腿动作超时     2B     500 .. 30000 ms
//   0x20★  手动模式开关       1B     0 / 1（>1 判非法）
//   其它   未知               ——     忽略（计数），不算错
//
//   为什么 0x03 只忽略：
//       测速窗口是 encoder_decode 的**编译期**参数（决定 CNT 位宽与折算
//       系数），运行期改它需要重算整条数据通路，本模块做不到。收到就
//       记一笔"忽略"——既不报错（否则 ARM 每次上电全量下发参数都会
//       多一条错误日志），也不假装成功。
//
//-----------------------------------------------------------------------------
// 与 vision_rx 的关系：rx_pl_addr 的仲裁（**关键约定**）
//-----------------------------------------------------------------------------
//   uart_frame 只有**一个**载荷读地址口，而 vision_rx 和 config_rx 都要用：
//
//       input  wire [7:0] rx_pl_addr;    // 谁解析谁给地址
//       output wire [7:0] rx_pl_data;    // assign rx_pl_data = rx_buf[rx_pl_addr]
//
//   所以 top.v 里必须用 rx_type 做二选一 mux：
//
//       rx_pl_addr = (rx_type == 8'h02) ? vision_rx 的地址 :
//                    (rx_type == 8'h04) ? config_rx 的地址 : 8'd0
//
//   为了让那个 mux 写得下来，本模块必须保证：
//
//       **rx_type != TYPE_CONFIG 时，rx_pl_addr 恒为 8'd0**
//
//   这就是下面那句 assign 里三元表达式的用途，不是冗余逻辑，别删。
//
//   载荷读出是**组合**的（uart_frame 里是 assign rx_pl_data = rx_buf[addr]），
//   地址给上、同一拍数据就有效 —— 所以每读一个字节正好花一个时钟，
//   不需要任何握手信号。（与 vision_rx.v 的用法完全一致。）
//
//-----------------------------------------------------------------------------
// 读载荷的时序（每帧最多 5 拍）
//-----------------------------------------------------------------------------
//   拍  state      rx_pl_addr   动作
//   --  ---------  -----------  ------------------------------------------
//    1  S_IDLE        0         看到 rx_valid && type==CONFIG：锁存 rx_len
//    2  S_ID         0         读参数ID
//    3  S_LEN        1         读载荷长度字节 -> 判定：继续 / 非法 / 忽略
//    4  S_VAL_HI     2         读值高字节（仅 2 字节参数）
//    5  S_VAL_LO     3         读值低字节 + **当拍**写寄存器（拼大端）
//       S_VAL8       2         1 字节参数：读值 + 当拍写寄存器
//    6  S_OK         0         cfg_valid 单拍，cfg_cnt+1
//
//   写寄存器用的是 S_VAL_LO / S_VAL8 **当拍**的 rx_pl_data（低字节），
//   高字节取上一拍锁存的 val_hi_r —— 少一个状态，逻辑也不绕。
//
//-----------------------------------------------------------------------------
// 长度校验：一帧只放一个参数
//-----------------------------------------------------------------------------
//   合法条件（**两个都要满足**）：
//       rx_len（帧头长度字段） == 2 + 值长度
//       载荷里的"长度"字节     == 值长度
//   任一不满足 -> cfg_bad 单拍脉冲，**一个寄存器都不写**。
//
//   为什么必须严格：
//       不校验就按"以为的"长度去读 rx_buf，会读到上一帧的残留字节，
//       产生"看起来合理但完全错误"的配置（比如把避障停车距离配成 3mm）。
//       宁可报错不改，也不能改错 —— 这是本项目 vision_rx 的同一条原则。
//
//   注意 rx_len == 2 + 值长度 是**相等**而不是"至少"：
//       rx_len 更大（一帧塞两个参数）也判非法。简化实现的代价是
//       ARM 侧必须一帧一个参数；好处是解析器没有循环、没有指针、
//       状态数固定，不可能跑飞。
//
//-----------------------------------------------------------------------------
// 钳位（clamp）：配置得离谱要能自保
//-----------------------------------------------------------------------------
//   直接把 vref 配成 0、把停车距离配成 0mm，车就再也不会动了（或者更糟：
//   一上电就急停/乱冲）。所以每个参数都有硬区间，越界**钳到边界**：
//
//       vref             1 .. 200
//       dist_stop_mm     50 .. 2000
//       dist_slow_mm     50 .. 3000，且必须 > dist_stop_mm
//                        （钳位后仍 <= dist_stop 就改成 dist_stop + 50）
//       lift_height      0 .. 100
//       lift_timeout_ms  500 .. 30000
//
//   ★ 钳位是**成功**：cfg_valid=1，cfg_cnt+1，不是 cfg_bad。
//     理由：值被接受并生效了（只是被夹到安全边界），对 ARM 来说这帧
//     是"收到并处理了"；报 bad 会让它以为链路坏了反复重发。
//
//   "dist_slow > dist_stop" 这条不变量特别重要：
//       两段式避障逻辑（> slow 全速 / stop..slow 减速 / < stop 停车）里，
//       如果 slow <= stop，中间那一段就不存在了，车会从全速直接切停车。
//
//-----------------------------------------------------------------------------
// 计数与饱和
//-----------------------------------------------------------------------------
//   cfg_cnt     成功应用（含钳位）的帧数
//   cfg_bad_cnt 格式/长度非法的帧数
//   cfg_ign_cnt 忽略的帧数（0x03 或未知 ID）
//   三个都到 16'hFFFF 就**停住不再涨**（饱和），避免回绕成 0 后
//   "错误计数看起来是 0"这种最坏情况。
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) 只认 TYPE_CONFIG；别的帧（0x02 视觉 / 0x03 ACK …）完全不响应 ——
//    三个计数都不动，rx_pl_addr 保持 0。
//
// 2) 处理途中来了新帧：uart_frame 的 rx_buf 是**单缓冲**，新帧一到旧数据
//    就被覆盖，继续按旧参数读会读到错位字节。所以一旦在处理中又看到
//    rx_valid，立刻按新帧重来（与 vision_rx 完全一致的处理）。
//
// 3) 超时兜底：处理态累计 TMO_LAST 个时钟还没走完就判 bad 并回 IDLE。
//    理论上永远不会触发（一帧最多 5 拍，而 115200 下最短帧间隔是
//    86.8us = 4340 拍），但"靠余量"不等于"靠设计"。
//
// 4) 不用除法/乘法：钳位只有比较，大端拼装只有位拼接，计数只有 +1。
//=============================================================================
`timescale 1ns/1ps

module config_rx #(
    parameter integer CLK_FREQ_HZ      = 50_000_000,
    // 复位后生效的默认值（与 top.v 的参数保持一致）
    parameter integer DEF_VREF         = 69,
    parameter integer DEF_DIST_STOP_MM = 200,
    parameter integer DEF_DIST_SLOW_MM = 500,
    parameter integer DEF_LIFT_HEIGHT  = 25,   // ★ 2026-11-03：100->25（抬腿 45°）
    // ★ 2026-11-03：绕行直行时长默认值（毫秒），目标相机会按实测宽度发 0x23 覆盖
    parameter integer DEF_RUN_MS     = 1000,
    parameter integer DEF_LIFT_TMO_MS  = 3000,
    parameter [7:0]   TYPE_CONFIG      = 8'h04,
    parameter integer _ASSERT_EN       = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    //------------ 来自 uart_frame 的接收接口（与 vision_rx.v **完全同款**）------------
    input  wire        rx_valid,     // 单拍脉冲：收到一帧合法帧
    input  wire [7:0]  rx_type,      // 与 rx_valid 同拍有效
    input  wire [15:0] rx_len,       // 载荷长度
    input  wire [7:0]  rx_pl_data,   // 载荷读出数据（组合）
    output wire [7:0]  rx_pl_addr,   // 载荷读出地址（本模块输出）

    //------------ 配置寄存器输出（**任何时刻都有效**，复位后 = 默认值）------------
    output reg  [15:0] vref,           // 0x01 最大速度
    output reg  [15:0] dist_stop_mm,   // 0x02 避障停车距离
    output reg  [15:0] dist_slow_mm,   // 0x04 避障减速距离
    output reg  [7:0]  lift_height,    // 0x10 0..100
    output reg  [15:0] lift_timeout_ms,// 0x11
    output reg         manual_sel,     // 0x20
    output reg         stance_low,     // ★ 0x21：0=高位(0状态) 1=低位
    output reg         run_cmd,        // ★ 0x22：0=停止 1=运行（开始）
    output reg  [15:0] run_ms,         // ★ 0x23：绕行直行时长（毫秒）

    //------------ 状态（给 LED / 遥测 / 调试）------------
    output reg         cfg_valid,   // 单拍：本帧成功应用了参数
    output reg         cfg_bad,     // 单拍：CONFIG 帧但格式/长度非法（**值不变**）
    output reg  [15:0] cfg_cnt,     // 累计成功帧数（饱和）
    output reg  [15:0] cfg_bad_cnt, // 累计非法帧数（饱和）
    output reg  [15:0] cfg_ign_cnt  // 累计忽略（未知ID / 0x03）帧数（饱和）
);

    //==================================================================
    // 0) 定宽常量
    //
    //   [!] Verilog-2001 **不允许对 parameter 做位选**（X[7:0] 非法），
    //       所以 DEF_* 一律先转成定宽 localparam 再用。
    //       本工程在这个坑上栽过 4 次，rtl_lint.py 里有自动检查。
    //==================================================================
    localparam [15:0] DEF_VREF_16  = DEF_VREF;
    localparam [15:0] DEF_STOP_16  = DEF_DIST_STOP_MM;
    localparam [15:0] DEF_SLOW_16  = DEF_DIST_SLOW_MM;
    localparam [7:0]  DEF_LIFT_8   = DEF_LIFT_HEIGHT;
    localparam [15:0] DEF_RUN_16   = DEF_RUN_MS;
    localparam [15:0] DEF_TMO_16   = DEF_LIFT_TMO_MS;

    //---- 参数 ID（协议 §3.2 + 扩展）----
    localparam [7:0] ID_VREF      = 8'h01,   // 最大速度
                     ID_DIST_STOP = 8'h02,   // 避障停车距离
                     ID_SPEED_WIN = 8'h03,   // 测速窗口（**只忽略**）
                     ID_DIST_SLOW = 8'h04,   // ★ 扩展：避障减速距离
                     ID_LIFT_H    = 8'h10,   // 升降腿抬升高度
                     ID_LIFT_TMO  = 8'h11,   // 升降腿动作超时
                     ID_MANUAL    = 8'h20,   // ★ 扩展：手动模式开关
                     ID_STANCE    = 8'h21,   // ★ 扩展：站立高度（0=高位 1=低位）
                     ID_RUN       = 8'h22,   // ★ 扩展：运行/停止（0=停止 1=运行）
                     ID_RUN_MS    = 8'h23;   // ★ 扩展：绕行直行时长（毫秒，200..10000）

    //---- 载荷/帧长（一帧只放一个参数）----
    localparam [7:0]  PAY_LEN_1B  = 8'd1,
                      PAY_LEN_2B  = 8'd2;
    localparam [15:0] FRM_LEN_1B  = 16'd3,   // 2 + 1
                      FRM_LEN_2B  = 16'd4;   // 2 + 2

    //---- 钳位区间 ----
    localparam [15:0] VREF_MIN    = 16'd1,
                      VREF_MAX    = 16'd200;
    localparam [15:0] STOP_MIN    = 16'd50,
                      STOP_MAX    = 16'd2000;
    localparam [15:0] SLOW_MIN    = 16'd50,
                      SLOW_MAX    = 16'd3000,
                      SLOW_GAP    = 16'd50;   // slow <= stop 时改成 stop + GAP
    localparam [7:0]  LIFT_H_MIN  = 8'd0,
                      LIFT_H_MAX  = 8'd100;
    localparam [15:0] TMO_MIN     = 16'd500,
                      TMO_MAX     = 16'd30000;
    localparam [7:0]  MANUAL_MAX  = 8'd1;     // 只接受 0/1

    //---- 载荷地址（每一拍只读一个字节）----
    localparam [7:0] ADDR_ID     = 8'd0,   // 参数 ID
                     ADDR_LEN    = 8'd1,   // 载荷里的长度字节
                     ADDR_VAL_HI = 8'd2,   // 值高字节 / 1 字节参数的值
                     ADDR_VAL_LO = 8'd3;   // 值低字节

    //---- 状态编码 ----
    localparam [2:0] S_IDLE   = 3'd0,
                     S_ID     = 3'd1,
                     S_LEN    = 3'd2,
                     S_VAL_HI = 3'd3,
                     S_VAL_LO = 3'd4,
                     S_VAL8   = 3'd5,
                     S_OK     = 3'd6;

    //---- 超时兜底（处理态最多 5 拍，64 拍是 12 倍余量）----
    localparam [15:0] TMO_LAST = 16'd64;

    //==================================================================
    // 1) 饱和加一
    //
    //   到 16'hFFFF 就停住。写成函数是因为三个计数都要用，
    //   而且"回绕成 0"是本工程明确要避免的最坏情况（错误计数看起来是 0）。
    //==================================================================
    function [15:0] sat16;
        input [15:0] v;
        begin
            sat16 = (v == 16'hFFFF) ? v : (v + 16'd1);
        end
    endfunction

    //==================================================================
    // 2) 内部寄存器
    //==================================================================
    reg [2:0]  state;
    reg [15:0] len_rx;      // rx_valid 当拍锁存的帧头长度字段
    reg [7:0]  id_r;        // 锁存的参数 ID
    reg [7:0]  val_hi_r;    // 锁存的值高字节
    reg [15:0] tmo;         // 处理态超时计数
    reg [7:0]  addr_sel;    // 状态 -> 载荷地址

    //==================================================================
    // 3) 大端拼装 + 钳位（纯组合）
    //
    //   val16_w 只在 2 字节参数的 S_VAL_LO 那一拍有意义
    //   （val_hi_r 是上一拍的高字节，rx_pl_data 是当拍的低字节）；
    //   val8_w 只在 S_VAL8 那一拍有意义。其它状态的值是"没意义的组合值"，
    //   但不会被写进任何寄存器，所以不影响功能。
    //==================================================================
    wire [15:0] val16_w = {val_hi_r, rx_pl_data};
    wire [7:0]  val8_w  = rx_pl_data;

    // 只有比较，没有除法/乘法
    wire [15:0] vref_cl = (val16_w < VREF_MIN) ? VREF_MIN :
                          (val16_w > VREF_MAX) ? VREF_MAX : val16_w;

    wire [15:0] stop_cl = (val16_w < STOP_MIN) ? STOP_MIN :
                          (val16_w > STOP_MAX) ? STOP_MAX : val16_w;

    // ★ 2026-11-03：绕行时长钳位 200..10000 ms
    wire [15:0] run_ms_cl = (val16_w < 16'd200)   ? 16'd200 :
                            (val16_w > 16'd10000) ? 16'd10000 : val16_w;

    wire [15:0] slow_lim_cl = (val16_w < SLOW_MIN) ? SLOW_MIN :
                              (val16_w > SLOW_MAX) ? SLOW_MAX : val16_w;

    // ★ 不变量：dist_slow 必须 > dist_stop。
    //   钳到边界之后如果仍然 <= 停车距离，就顶到 stop + 50。
    //   （dist_stop 最大 2000 -> 2050 <= SLOW_MAX， 不会再把 slow 顶出区间）
    wire [15:0] slow_gap_w  = dist_stop_mm + SLOW_GAP;
    wire [15:0] slow_fin_cl = (slow_lim_cl <= dist_stop_mm) ? slow_gap_w
                                                            : slow_lim_cl;

    wire [15:0] tmo_cl = (val16_w < TMO_MIN) ? TMO_MIN :
                         (val16_w > TMO_MAX) ? TMO_MAX : val16_w;

    wire [7:0]  lift_h_cl = (val8_w > LIFT_H_MAX) ? LIFT_H_MAX : val8_w;

    //==================================================================
    // 4) 载荷地址（组合）
    //
    //   每个状态只读一个字节：给上地址，同一拍 rx_pl_data 就有效。
    //   这是**纯组合块，没有时钟，所以不需要复位分支**。
    //==================================================================
    always @(*) begin
        case (state)
            S_LEN    : addr_sel = ADDR_LEN;
            S_VAL_HI : addr_sel = ADDR_VAL_HI;
            S_VAL_LO : addr_sel = ADDR_VAL_LO;
            S_VAL8   : addr_sel = ADDR_VAL_HI;
            default  : addr_sel = ADDR_ID;      // S_IDLE / S_ID
        endcase
    end

    // ★ 仲裁约定：不是 CONFIG 帧就吐 0，让 top.v 能用 rx_type 二选一 mux。
    //   这句不能省：否则 0x02 视觉帧进来时本模块会把地址线抢过去，
    //   vision_rx 读到错位的字节。
    assign rx_pl_addr = (rx_type == TYPE_CONFIG) ? addr_sel : ADDR_ID;

    //==================================================================
    // 5) 主状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            len_rx          <= 16'd0;
            id_r            <= 8'd0;
            val_hi_r        <= 8'd0;
            tmo             <= 16'd0;

            // 复位值 = 默认值（**故意不再钳位**：tb 与 top.v 都要求
            // "复位后等于 DEF_*"，默认值本身是合法的，钳了反而对不上）
            vref            <= DEF_VREF_16;
            dist_stop_mm    <= DEF_STOP_16;
            dist_slow_mm    <= DEF_SLOW_16;
            lift_height     <= DEF_LIFT_8;
            lift_timeout_ms <= DEF_TMO_16;
            manual_sel      <= 1'b0;
            run_cmd         <= 1'b0;
            run_ms          <= DEF_RUN_16;

            cfg_valid       <= 1'b0;
            cfg_bad         <= 1'b0;
            cfg_cnt         <= 16'd0;
            cfg_bad_cnt     <= 16'd0;
            cfg_ign_cnt     <= 16'd0;
        end else begin
            // 默认：单拍脉冲，每拍自动落回 0
            cfg_valid <= 1'b0;
            cfg_bad   <= 1'b0;

            // 超时计数：只在处理态累加，回到 IDLE 立刻清零
            if (state == S_IDLE) tmo <= 16'd0;
            else                 tmo <= tmo + 16'd1;

            if ((state != S_IDLE) && (tmo >= TMO_LAST)) begin
                //------------------------------------------------------
                // 兜底：绝不停在中间态
                //   理论上不会触发（一帧最多 5 拍）。真触发了就按 bad 计，
                //   因为这一帧确实没解析完。
                //------------------------------------------------------
                cfg_bad     <= 1'b1;
                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                state       <= S_IDLE;

            end else if ((state != S_IDLE) && rx_valid) begin
                //------------------------------------------------------
                // 处理途中又来了新帧
                //
                //   uart_frame 的 rx_buf 是**单缓冲**，新帧一到旧数据立刻被
                //   覆盖；再按旧参数往下读就是错位字节，会写进一个
                //   "看起来合理但完全错误"的值。所以直接按新帧重来。
                //
                //   115200 下最短帧间隔 86.8us = 4340 拍，而一帧只要 5 拍，
                //   余量 800 多倍 —— 但这是"靠余量"而不是"靠设计"，
                //   所以显式处理（与 vision_rx 同一套做法）。
                //------------------------------------------------------
                if (rx_type == TYPE_CONFIG) begin
                    len_rx <= rx_len;
                    state  <= S_ID;
                end else begin
                    state  <= S_IDLE;
                end

            end else begin
                case (state)
                //------------------------------------------------------
                // 空闲：只认 CONFIG 帧
                //------------------------------------------------------
                S_IDLE: begin
                    if (rx_valid && (rx_type == TYPE_CONFIG)) begin
                        len_rx <= rx_len;      // rx_len 只在 rx_valid 同拍有效
                        state  <= S_ID;
                    end
                end

                //------------------------------------------------------
                // 读参数 ID
                //------------------------------------------------------
                S_ID: begin
                    id_r  <= rx_pl_data;
                    state <= S_LEN;
                end

                //------------------------------------------------------
                // 读载荷长度字节 + 判定（继续 / 非法 / 忽略）
                //
                //   注意 rx_pl_data 这一拍就是**载荷里的长度字节**，
                //   所以"两个长度都要对"能在同一拍判完：
                //       len_rx  == 2 + 值长度   （帧头说的）
                //       载荷长度 == 值长度       （载荷自己说的）
                //------------------------------------------------------
                S_LEN: begin
                    case (id_r)
                        //---- 2 字节值 ----
                        ID_VREF, ID_DIST_STOP, ID_DIST_SLOW, ID_LIFT_TMO, ID_RUN_MS: begin
                            if ((len_rx == FRM_LEN_2B) && (rx_pl_data == PAY_LEN_2B)) begin
                                state <= S_VAL_HI;
                            end else begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;      // **一个寄存器都不写**
                            end
                        end

                        //---- 1 字节值 ----
                        ID_LIFT_H, ID_MANUAL, ID_STANCE, ID_RUN: begin
                            if ((len_rx == FRM_LEN_1B) && (rx_pl_data == PAY_LEN_1B)) begin
                                state <= S_VAL8;
                            end else begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;
                            end
                        end

                        //---- 0x03 测速窗口：只忽略 ----
                        //   它是 encoder_decode 的编译期参数，运行期改不了。
                        //   不报错（否则 ARM 全量下发参数时每次都会多一条错误），
                        //   也不改任何值 —— 只计数。
                        ID_SPEED_WIN: begin
                            cfg_ign_cnt <= sat16(cfg_ign_cnt);
                            state       <= S_IDLE;
                        end

                        //---- 未知 ID：忽略（计数），不算错 ----
                        default: begin
                            cfg_ign_cnt <= sat16(cfg_ign_cnt);
                            state       <= S_IDLE;
                        end
                    endcase
                end

                //------------------------------------------------------
                // 2 字节参数：先锁高字节
                //------------------------------------------------------
                S_VAL_HI: begin
                    val_hi_r <= rx_pl_data;
                    state    <= S_VAL_LO;
                end

                //------------------------------------------------------
                // 2 字节参数：读低字节，**当拍**拼大端并钳位后写寄存器
                //------------------------------------------------------
                S_VAL_LO: begin
                    case (id_r)
                        ID_VREF:      vref            <= vref_cl;
                        // ★ 不变量 slow > stop 要**双向**维持：
                        //   只写 0x04（减速距离）时下面那条 slow_fin_cl 会处理；
                        //   但只写 0x02 把 stop 顶上去时，slow 原来不会跟 ——
                        //   会出现 slow <= stop（例：slow=1000，再写 stop=2000），
                        //   结果"减速距离比停车距离还近"，状态机会先停后减，逻辑反了。
                        //   所以写 stop 时如果 slow 不够大，一起抬上去。
                        ID_DIST_STOP: begin
                            dist_stop_mm <= stop_cl;
                            if (dist_slow_mm <= stop_cl)
                                dist_slow_mm <= stop_cl + 16'd50;
                        end
                        ID_DIST_SLOW: dist_slow_mm    <= slow_fin_cl;
                        ID_LIFT_TMO:  lift_timeout_ms <= tmo_cl;
                        ID_RUN_MS:    run_ms          <= run_ms_cl;   // ★ 2026-11-03
                        default: ;    // 1 字节参数走不到这里
                    endcase
                    state <= S_OK;
                end

                //------------------------------------------------------
                // 1 字节参数：读值，当拍写寄存器
                //------------------------------------------------------
                S_VAL8: begin
                    case (id_r)
                        ID_LIFT_H: begin
                            lift_height <= lift_h_cl;
                            state       <= S_OK;
                        end

                        ID_STANCE: begin
                            // 0x21 只接受 0/1（0=升高到 0 状态，1=降低）
                            if (val8_w > 8'd1) begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;
                            end else begin
                                stance_low <= val8_w[0];
                                state      <= S_OK;
                            end
                        end

                        ID_RUN: begin
                            // 0x22 只接受 0/1（0=停止 1=运行）
                            if (val8_w > 8'd1) begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;
                            end else begin
                                run_cmd <= val8_w[0];
                                state   <= S_OK;
                            end
                        end

                        ID_MANUAL: begin
                            // 0x20 只接受 0/1：>1 当**非法**（不改值、报 bad）
                            // 而不是钳到 1 —— 因为"2"没有任何合理含义，
                            // 多半是 ARM 侧打包错了，钳位会把这个错误藏起来。
                            if (val8_w > MANUAL_MAX) begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;
                            end else begin
                                manual_sel <= val8_w[0];
                                state      <= S_OK;
                            end
                        end

                        default: begin
                            cfg_bad     <= 1'b1;
                            cfg_bad_cnt <= sat16(cfg_bad_cnt);
                            state       <= S_IDLE;
                        end
                    endcase
                end

                //------------------------------------------------------
                // 成功：单拍脉冲 + 计数
                //------------------------------------------------------
                S_OK: begin
                    cfg_valid <= 1'b1;
                    cfg_cnt   <= sat16(cfg_cnt);
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end

    //==================================================================
    // 6) 参数自检（综合器忽略 initial；仿真里一眼看出参数配错）
    //==================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            $display("[config_rx] TYPE_CONFIG=0x%02h CLK=%0dHz",
                     TYPE_CONFIG, CLK_FREQ_HZ);
            $display("[config_rx] 默认值 VREF=%0d STOP=%0dmm SLOW=%0dmm LIFT_H=%0d LIFT_TMO=%0dms",
                     DEF_VREF, DEF_DIST_STOP_MM, DEF_DIST_SLOW_MM,
                     DEF_LIFT_HEIGHT, DEF_LIFT_TMO_MS);
            $display("[config_rx] 钳位区间 VREF=%0d..%0d STOP=%0d..%0d SLOW=%0d..%0d LIFT_H=%0d..%0d TMO=%0d..%0d",
                     VREF_MIN, VREF_MAX, STOP_MIN, STOP_MAX, SLOW_MIN, SLOW_MAX,
                     LIFT_H_MIN, LIFT_H_MAX, TMO_MIN, TMO_MAX);

            if (DEF_VREF < VREF_MIN || DEF_VREF > VREF_MAX) begin
                $display("[config_rx] ERROR: DEF_VREF=%0d 不在 %0d..%0d（上电默认值必须本来就合法）",
                         DEF_VREF, VREF_MIN, VREF_MAX);
            end
            if (DEF_DIST_STOP_MM < STOP_MIN || DEF_DIST_STOP_MM > STOP_MAX) begin
                $display("[config_rx] ERROR: DEF_DIST_STOP_MM=%0d 不在 %0d..%0d",
                         DEF_DIST_STOP_MM, STOP_MIN, STOP_MAX);
            end
            if (DEF_DIST_SLOW_MM <= DEF_DIST_STOP_MM) begin
                $display("[config_rx] ERROR: 默认 SLOW(%0d) 必须 > STOP(%0d)，否则两段式避障退化成一段",
                         DEF_DIST_SLOW_MM, DEF_DIST_STOP_MM);
            end
            if (DEF_LIFT_TMO_MS < TMO_MIN || DEF_LIFT_TMO_MS > TMO_MAX) begin
                $display("[config_rx] ERROR: DEF_LIFT_TMO_MS=%0d 不在 %0d..%0d",
                         DEF_LIFT_TMO_MS, TMO_MIN, TMO_MAX);
            end
        end
    end

endmodule
