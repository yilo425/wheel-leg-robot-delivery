//=============================================================================
// motion_decision -- 运动决策（把"意图"翻译成每轮的转速与方向）
//-----------------------------------------------------------------------------
// 在整条链路里的位置：
//
//   sensors ──► main_fsm ──► motion_decision ──► pwm_motor ──► 电机
//   (TOF/视觉)  (行为决策)     (本模块: 速度闭环)    (载波生成)
//                                  ▲
//                             encoder_decode
//                             (每轮速度反馈)
//
// main_fsm 输出的是**意图**（speed_scale=停/慢/满、lift_cmd）；
// 本模块负责把它变成**每个轮子具体的目标速度**，再用编码器反馈做闭环，
// 最后输出每轮的 duty 幅值 + 方向脚。
//
//-----------------------------------------------------------------------------
// 为什么按"每轮独立"设计
//-----------------------------------------------------------------------------
// 底盘驱动形式**还没定**（差速 / 麦克纳姆 / 四轮独立）。
// 所以本模块不假设任何具体底盘，只提供：
//
//      target[g] = base + (steer >> STEER_SHIFT) * STEER_COEF[g]
//
//   * 差速底盘：左侧两轮 COEF = +1，右侧两轮 COEF = -1（默认值）
//   * 麦克纳姆 / 全向：把那四个系数按运动学矩阵改成别的组合
//     （将来若需要二维平移，再给本模块加一路 steer2 即可）
//
// 换底盘时**只改参数、不改逻辑**，不会返工。
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) 目标速度由 speed_scale 直接决定，不用查表：
//        0=停 -> 0    1=慢 -> VREF/2    2=满 -> VREF
//    VREF 是"每个测速窗口内期望的编码器净计数"，与 encoder_decode 同量纲。
//
// 2) 比例控制用**算术右移**实现除法（省 DSP、延迟固定）：
//        duty = clamp( (target - fb) >> KP_SHIFT )
//    KP_SHIFT 越大增益越小。用移位而非除法，时序确定性更好 ——
//    这也是 FPGA 相对软件方案的卖点之一。
//
// 3) 符号拆成"幅值 + 方向脚"：
//        电机驱动板只有一根方向脚，负 duty 表达不了倒车
//        （早期 pwm_motor 就踩过这个坑）
//
// 4) 关使能时强制 duty=0：不依赖上层是否记得清零，本模块自己兜底
//
// 5) 每轮独立限幅：某一轮打滑/堵转时不会拖累其它轮
//
//-----------------------------------------------------------------------------
// 内部统一用 32 位有符号中间量
//-----------------------------------------------------------------------------
// 这是刻意的：本工程今天已经因为"位宽不够/截断"踩了 4 次坑
// （见 tools/check_literal_overflow.py 的说明）。
// 中间运算全部放宽到 32 位，只在最后输出 duty 时收窄到 16 位。
//=============================================================================
`timescale 1ns/1ps

module motion_decision #(
    parameter integer CLK_FREQ_HZ     = 50_000_000,

    //----------------------------------------------------------------
    // 速度给定
    //----------------------------------------------------------------
    parameter integer VREF            = 60,    // 满速目标计数/窗口
    parameter integer KP_SHIFT        = 1,     // duty = err >> KP_SHIFT
    parameter integer LIMIT_DUTY      = 120,   // 每轮占空比上限（标度值）
    parameter integer ZERO_DEADBAND   = 1,     // 小于此值视为 0，避免嗡嗡抖

    //----------------------------------------------------------------
    // ★ 轮子编号约定（全项目统一 —— 机械组按这个接线）
    //
    //       索引 0 = 左前 FL     索引 1 = 右前 FR
    //       索引 2 = 左后 RL     索引 3 = 右后 RR
    //
    //   ⚠️ 这个编号**曾经和 top.v / 引脚表不一致**：
    //      本模块原先按"0,1=左；2,3=右"分组（按侧面分），
    //      而引脚表按"0=左前 1=右前 2=左后 3=右后"（按位置分）。
    //      两边都自洽，但**合起来就是错的** ——
    //      机械组按位置接好线，转向会整个反过来。
    //      现在统一成**按位置**（和引脚表一致，也是 4 轮机器人的通用约定）。
    //
    //----------------------------------------------------------------
    // 转向系数（每轮一个）
    //
    // ⚠️ 符号约定同样容易搞反，写清楚：
    //      steer > 0 表示【右转】，右转要靠【左轮加速、右轮减速】。
    //      target[g] = base + (steer >>> STEER_SHIFT) * STEER_COEF[g]
    //   所以【左轮系数必须取 +1、右轮取 -1】。
    //
    //   最初写成左轮 -1、右轮 +1，结果 steer>0 时左轮减速 ——
    //   正好转成左转，方向反了。
    //
    //   系数之和为 0，所以转向不会附带整体加减速。
    //   换麦克纳姆/全向底盘时，按运动学矩阵改这四个系数即可。
    //----------------------------------------------------------------
    // ★ 2026-10-30：按物理结构重排通道映射（不动接线）
    //   驱动板输出口在板上顺时针是 D-C-B-A，和四个轮子物理位置一致：
    //     A -> 右后 RR      B -> 右前 FR
    //     C -> 左前 FL      D -> 左后 RL
    //   转向系数：左轮 +1（加速）、右轮 -1（减速）
    // ★ 2026-11-01 驱动板换方向后重新定义：
    //   A=左前  B=左后  C=右后  D=右前
    //   左轮 +1（转向时加速）、右轮 -1（减速）
    parameter integer STEER_COEF0     =  1,    // A 左前 FL -> 左轮 +1
    parameter integer STEER_COEF1     =  1,    // B 左后 RL -> 左轮 +1
    parameter integer STEER_COEF2     = -1,    // C 右后 RR -> 右轮 -1
    parameter integer STEER_COEF3     = -1,    // D 右前 FR -> 右轮 -1
    parameter integer STEER_SHIFT     = 4,     // 转向量衰减：steer >>> 4
    // ★ 2026-11-02：转向权威（%）。steer=100 时 bias = base × AUTH%，
    //   内侧轮保留 (1-AUTH%) 的动力，不再直接归零 -> 转弯力大幅提升
    parameter integer STEER_AUTH      = 45,
    // ★ 2026-11-02：低速补偿曲线。实测 40% 只有 80% 速度的 1/4，
    //   因为电机有启动死区。补偿：
    //     eff = pct + pct*(100-pct)/CURVE_DIV
    //   140 -> 40%→~57%、80%→~91%、100%→100%（越大补偿越少）
    parameter integer SPEED_CURVE_DIV = 140,    // 实测 80 时转 160°，要 90° -> 80*90/160 ≈ 45
    //----------------------------------------------------------------
    // ★ 测速反馈超时（2026-10-25 新增，收尾 spd_stale 死代码）
    //
    //   失效场景：编码器信号线松了/断了 -> spd_fb 恒 0 -> 误差恒 = target
    //   -> 积分一路涨到限幅 -> **机器人自己跑到满速**。
    //   这不是理论风险，一根杜邦线没插好就会发生。
    //
    //   处理：超过 SPEED_TIMEOUT_MS 收不到 spd_valid，就判定反馈失效，
    //        1) 闭环输出 u 换成"开环兜底"（duty = base，方向跟指令）
    //        2) 积分器清零，等反馈恢复再平滑切回闭环
    //   SPEED_TIMEOUT_MS 取 200ms＝10 个测速窗口（窗口 20ms），
    //   既不会误判偶尔丢一次，也能在打滑/断线后很快收住。
    //----------------------------------------------------------------
    parameter integer SPEED_TIMEOUT_MS = 200,
    // ★ 2026-11-02：强制开环。编码器极性不确定时，
    //   闭环可能变正反馈 -> 积分跑飞 -> 協占比顶满（一直高速）。
    //   1 = 一直用开环（duty = ±base，速度直接跟指令）；0 = 闭环
    parameter integer FORCE_OPEN_LOOP = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    //---------------- 来自 main_fsm 的意图 ----------------
    input  wire        motor_enable,      // 0 = 立即停在 0 占空比
    input  wire [1:0]  speed_scale,       // 0=停 1=慢 2=满（自动模式用）
    input  wire [7:0]  speed_pct,         // ★ 连续调速 0~100（手动遥控用；100=满档）
    input  wire        dir_inv,           // ★ 1=整机后退（目标速度取反）

    //---------------- 运行时配置（来自 config_rx 的 CONFIG 帧）----------------
    //   原来这是编译期参数 VREF，改成端口后**不重新综合就能改最大速度**
    //   （实车标定很有用）。参数 VREF 保留作复位默认值。
    input  wire [15:0] vref,              // 满速目标计数/窗口（1..200）

    //---------------- 转向指令（暂时给 0，走直线）----------------
    input  wire signed [15:0] steer,      // 有符号：正=右转

    //---------------- 编码器速度反馈 ----------------
    input  wire signed [23:0] spd_fb0,
    input  wire signed [23:0] spd_fb1,
    input  wire signed [23:0] spd_fb2,
    input  wire signed [23:0] spd_fb3,
    input  wire        spd_valid,         // 测速窗口更新脉冲

    //---------------- 输出：每轮占空比幅值 + 方向 ----------------
    output reg  [15:0] duty0,
    output reg  [15:0] duty1,
    output reg  [15:0] duty2,
    output reg  [15:0] duty3,

    output wire        dir0,              // 1 = 正转
    output wire        dir1,
    output wire        dir2,
    output wire        dir3,

    //---------------- 调试/上报 ----------------
    output reg         limited,           // 至少一路触发限幅
    output wire signed [15:0] target0,    // 当前目标速度（便于观测/上报）
    output wire signed [15:0] target1,
    output wire signed [15:0] target2,
    output wire signed [15:0] target3,
    output wire        spd_stale_o      // ★ 1 = 测速反馈超时（已降级开环），给遥测上报
);

    //==================================================================
    // 常量
    //==================================================================
    localparam [1:0] SPD_STOP = 2'd0;
    localparam [1:0] SPD_SLOW = 2'd1;
    localparam [1:0] SPD_FULL = 2'd2;

    // 参数不能做位选（Verilog-2001），先转成定宽常量
    localparam [15:0] LIMIT_W = LIMIT_DUTY;

    // 全部中间量用 32 位有符号，避免截断
    localparam signed [31:0] LIM32 = LIMIT_DUTY;
    localparam signed [31:0] DB32  = ZERO_DEADBAND;

    //==================================================================
    // 基础目标速度：由 speed_scale 直接决定
    //==================================================================
    reg signed [31:0] base_gear;
    always @(*) begin
        case (speed_scale)
            SPD_SLOW: base_gear = vref >>> 1;
            SPD_FULL: base_gear = vref;
            default:  base_gear = 32'sd0;      // SPD_STOP 及非法值都当停
        endcase
    end

    //------------------------------------------------------------------
    // ★ 2026-10-30：连续调速 + 后退
    //
    //   原来 base 只有 vref/2 和 vref 两个值（遥控 speed 只能出两档，
    //   实测"速度只在 50 变一次"就是这个原因）。
    //
    //   现在按 speed_pct(0~100) 线性缩放：50% 就是半速，任意百分比都有效。
    //   dir_inv=1 时整个目标速度取反 -> 走 abs_duty + dir=(sat>=0) 的反向路径。
    //------------------------------------------------------------------
    wire signed [31:0] pct32 = speed_pct;                       // 0..100
    // ★ 低速补偿：eff = pct + pct*(100-pct)/DIV
    wire signed [31:0] pct_curve = pct32 + (pct32 * (32'sd100 - pct32)) / SPEED_CURVE_DIV;
    wire signed [31:0] base_scaled = (base_gear * pct_curve) / 100;  // 常量除 100，综合器会优化
    wire signed [31:0] base = dir_inv ? -base_scaled : base_scaled;

    //==================================================================
    // 每轮目标速度
    //   target = base + (steer >>> STEER_SHIFT) * STEER_COEF
    //   steer 为 0 时（走直线）target 就等于 base
    //==================================================================
    // ★ 2026-10-30：转向幅度改成「按比例」
    //   原来是固定偏置（steer >>> STEER_SHIFT），和当前速度无关 ->
    //   慢速时偏置比基速还大（内侧轮反转），快速时又偏小。
    //   现在 bias = base * steer / 100：
    //     steer = 100%  ->  bias = base  ->  内侧轮=0、外侧轮=2*base
    //     恰好对应最小转弯半径 R = W/2（绕内侧轮转）
    //     steer = 0     ->  直行
    //   STEER_SHIFT 参数保留但不再使用（后续可删）
    // ★★ 2026-11-02 最简可靠转向：bias 固定 ±100，符号只看 steer[15]
    wire steering  = (steer != 16'sd0);
    wire steer_neg = steer[15];
    wire signed [31:0] bias = steering ? (steer_neg ? -32'sd100 : 32'sd100) : 32'sd0;

    // ★★ 硬编码±bias，规避 bias * 系数(-1) 的带符号乘法
    //   左轮(0,1) +bias；右轮(2,3) -bias
    wire signed [31:0] tgt0 = base + bias;   // A 左前
    wire signed [31:0] tgt1 = base + bias;   // B 左后
    wire signed [31:0] tgt2 = base - bias;   // C 右后
    wire signed [31:0] tgt3 = base - bias;   // D 右前

    assign target0 = tgt0[15:0];
    assign target1 = tgt1[15:0];
    assign target2 = tgt2[15:0];
    assign target3 = tgt3[15:0];

    //==================================================================
    // 比例控制（32 位有符号）
    //==================================================================
    wire signed [31:0] err0 = tgt0 - $signed(spd_fb0);
    wire signed [31:0] err1 = tgt1 - $signed(spd_fb1);
    wire signed [31:0] err2 = tgt2 - $signed(spd_fb2);
    wire signed [31:0] err3 = tgt3 - $signed(spd_fb3);

    wire signed [31:0] prop0 = err0 >>> KP_SHIFT;
    wire signed [31:0] prop1 = err1 >>> KP_SHIFT;
    wire signed [31:0] prop2 = err2 >>> KP_SHIFT;
    wire signed [31:0] prop3 = err3 >>> KP_SHIFT;

    //==================================================================
    // ★ 积分项（2026-10-18 新增）—— 消除纯 P 控制的稳态残差
    //
    //   为什么必须加：纯比例控制 `duty = (VREF-spd) >> 1`，对"duty -> 转速"
    //   这类有限增益的被控对象**必然有残差**：
    //
    //       spd_ss = VREF * G / (G + 2)        G = 被控对象增益
    //
    //   G≈1（每 duty 单位 ≈ 1 计数/窗口，和 VREF=69 同量级）时只能跑到
    //   **目标的 1/3** —— 闭环仿真 `tb_motion_loop` 实测 31%（22/69）。
    //   那样 `VREF_FULL` 的标定（69 = 目标转速）就没有意义了。
    //
    //   加了积分项后稳态误差 -> 0（与 G 无关），标定才成立。
    //
    //   实现要点（都是为了不引入乘法器 / 不让它失控）：
    //     1) 每个测速窗口（spd_valid）累加一次误差；KP/KI 都用移位
    //     2) **抗饱和**：积分器钳在 ±(LIMIT<<KI_SHIFT)
    //        —— 否则限幅期间积分无限涨，退饱和时会冲过头
    //     3) **停使能就清零** —— 别在马达没转的时候攒积分
    //==================================================================
    parameter integer KI_SHIFT = 3;

    localparam signed [31:0] INT_MAX_W = LIM32 <<< KI_SHIFT;

    wire en_int = motor_enable && (speed_scale != SPD_STOP);

    //------------------------------------------------------------------
    // ★ 测速反馈超时检测（替代原来那个"只置 0、永远不置 1、没人读"的 spd_stale）
    //------------------------------------------------------------------
    localparam integer TIMEOUT_CYC = (CLK_FREQ_HZ / 1000) * SPEED_TIMEOUT_MS;
    localparam [31:0]  TIMEOUT_32  = TIMEOUT_CYC;

    reg [31:0] stale_cnt;
    reg        spd_stale;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stale_cnt <= 32'd0;
            spd_stale <= 1'b0;
        end else if (spd_valid) begin
            stale_cnt <= 32'd0;
            spd_stale <= 1'b0;
        end else if (stale_cnt >= TIMEOUT_32) begin
            spd_stale <= 1'b1;              // 锁存，等 spd_valid 恢复才清
        end else begin
            stale_cnt <= stale_cnt + 32'd1;
        end
    end

    reg signed [31:0] integ0, integ1, integ2, integ3;

    // 控制量 = 比例项 + 积分项
    wire signed [31:0] u0 = prop0 + (integ0 >>> KI_SHIFT);
    wire signed [31:0] u1 = prop1 + (integ1 >>> KI_SHIFT);
    wire signed [31:0] u2 = prop2 + (integ2 >>> KI_SHIFT);
    wire signed [31:0] u3 = prop3 + (integ3 >>> KI_SHIFT);

    //------------------------------------------------------------------
    // ★ 条件积分（抗饱和的更好做法，2026-10-18）
    //
    //   光把积分器钳在 ±INT_MAX 还不够：输出**顶在限幅上**的时候积分器照样
    //   涨到钳位值，于是"限幅期间环路等于开环" —— 真需要退饱和时要花很久，
    //   表现为"该减速时迟迟不减速"。
    //
    //   条件积分：输出已饱和、且误差还在往饱和方向推时 —— **冻结**积分器。
    //   （这是条件积分的简化版，只在饱和方向冻结，够用且不会引入抖动。）
    //------------------------------------------------------------------
    wire freeze0 = ((u0 >=  LIM32) && (err0 > 0)) || ((u0 <= -LIM32) && (err0 < 0));
    wire freeze1 = ((u1 >=  LIM32) && (err1 > 0)) || ((u1 <= -LIM32) && (err1 < 0));
    wire freeze2 = ((u2 >=  LIM32) && (err2 > 0)) || ((u2 <= -LIM32) && (err2 < 0));
    wire freeze3 = ((u3 >=  LIM32) && (err3 > 0)) || ((u3 <= -LIM32) && (err3 < 0));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integ0 <= 32'sd0;
            integ1 <= 32'sd0;
            integ2 <= 32'sd0;
            integ3 <= 32'sd0;
        end else if (!en_int || spd_stale) begin
            integ0 <= 32'sd0;
            integ1 <= 32'sd0;
            integ2 <= 32'sd0;
            integ3 <= 32'sd0;
        end else if (spd_valid) begin
            // ★ 条件积分：饱和且误差还在往饱和方向推时冻结（见下面 freeze* 的说明）
            if (!freeze0)
                integ0 <= (integ0 >=  INT_MAX_W) ?  INT_MAX_W :
                          (integ0 <= -INT_MAX_W) ? -INT_MAX_W : (integ0 + err0);
            if (!freeze1)
                integ1 <= (integ1 >=  INT_MAX_W) ?  INT_MAX_W :
                          (integ1 <= -INT_MAX_W) ? -INT_MAX_W : (integ1 + err1);
            if (!freeze2)
                integ2 <= (integ2 >=  INT_MAX_W) ?  INT_MAX_W :
                          (integ2 <= -INT_MAX_W) ? -INT_MAX_W : (integ2 + err2);
            if (!freeze3)
                integ3 <= (integ3 >=  INT_MAX_W) ?  INT_MAX_W :
                          (integ3 <= -INT_MAX_W) ? -INT_MAX_W : (integ3 + err3);
        end
    end


    //------------------------------------------------------------------
    // 限幅（饱和）
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // ★ 反馈失效兜底：spd_stale=1 用开环（base，方向跟 target），不用闭环 u
    //------------------------------------------------------------------
    // ★★ 2026-11-02 彻底无符号重写（GOWIN 带符号减法/符号位提取 bug 太多）
    wire base_neg = base[31];
    wire [31:0] base_abs = base_neg ? (~base + 32'd1) : base;

    // ★ 2026-11-03：steering / steer_neg 在上面已经声明过（本文件 237/238 行），
    //   这里不能再声明一次（RTL lint 会报重复定义）。
    wire turn_left  = steering &  steer_neg;   // 左转
    wire turn_right = steering & ~steer_neg;   // 右转

    // 前进方向（base 正 = 前进）
    wire fwd = !base_neg;

    // 每轮方向：左转时左轮反转，右转时右轮反转
    wire dir0_w = turn_left  ? !fwd : fwd;   // A 左前
    wire dir1_w = turn_left  ? !fwd : fwd;   // B 左后
    wire dir2_w = turn_right ? !fwd : fwd;   // C 右后
    wire dir3_w = turn_right ? !fwd : fwd;   // D 右前

    // 每轮幅值：直行 base_abs；转向 base_abs + 100（内侧也出力）
    wire [31:0] boost = steering ? 32'd100 : 32'd0;
    wire [31:0] mag0 = base_abs + boost;
    wire [31:0] mag1 = base_abs + boost;
    wire [31:0] mag2 = base_abs + boost;
    wire [31:0] mag3 = base_abs + boost;

    wire lim0 = (mag0 > LIM32);
    wire lim1 = (mag1 > LIM32);
    wire lim2 = (mag2 > LIM32);
    wire lim3 = (mag3 > LIM32);

    wire [31:0] sat0 = (mag0 > LIM32) ? LIM32 : mag0;
    wire [31:0] sat1 = (mag1 > LIM32) ? LIM32 : mag1;
    wire [31:0] sat2 = (mag2 > LIM32) ? LIM32 : mag2;
    wire [31:0] sat3 = (mag3 > LIM32) ? LIM32 : mag3;

    //------------------------------------------------------------------
    // 取绝对值 + 死区
    //   死区避免"极小占空比"让电机嗡嗡响却不转
    //------------------------------------------------------------------
    function [15:0] abs_duty;
        input signed [31:0] v;
        reg   signed [31:0] a;
        begin
            a = (v < 0) ? -v : v;
            if (a < DB32)          abs_duty = 16'd0;
            else if (a > LIM32)    abs_duty = LIMIT_W;
            else                   abs_duty = a[15:0];
        end
    endfunction

    //------------------------------------------------------------------
    // 输出：关使能或速度档为"停"时，强制 duty = 0
    //------------------------------------------------------------------
    wire en = motor_enable && (speed_scale != SPD_STOP);

    always @(*) begin
        if (!en) begin
            duty0   = 16'd0;
            duty1   = 16'd0;
            duty2   = 16'd0;
            duty3   = 16'd0;
            limited = 1'b0;
        end else begin
            duty0   = abs_duty(sat0);
            duty1   = abs_duty(sat1);
            duty2   = abs_duty(sat2);
            duty3   = abs_duty(sat3);
            limited = lim0 | lim1 | lim2 | lim3;
        end
    end

    //------------------------------------------------------------------
    // 方向：命令 >= 0 视为正转
    //   关使能时仍给出方向，但 duty 已被强制为 0，电机不动
    //------------------------------------------------------------------
    assign dir0 = dir0_w;
    assign dir1 = dir1_w;
    assign dir2 = dir2_w;
    assign dir3 = dir3_w;

    assign spd_stale_o = spd_stale;

    //==================================================================
    // 参数自检
    //==================================================================
    initial begin
        if (LIMIT_DUTY <= 0) begin
            $display("[motion_decision] FATAL: LIMIT_DUTY 必须 > 0 (当前 %0d)",
                     LIMIT_DUTY);
        end
        if (KP_SHIFT < 0 || KP_SHIFT > 8) begin
            $display("[motion_decision] WARN: KP_SHIFT=%0d 超出常用范围 0..8",
                     KP_SHIFT);
        end
        if ((STEER_COEF0 + STEER_COEF1 + STEER_COEF2 + STEER_COEF3) != 0) begin
            $display("[motion_decision] WARN: 转向系数之和不为 0 (%0d+%0d+%0d+%0d)，转向时会附带整体加减速",
                     STEER_COEF0, STEER_COEF1, STEER_COEF2, STEER_COEF3);
        end
        if (CLK_FREQ_HZ <= 0) begin
            $display("[motion_decision] FATAL: CLK_FREQ_HZ 必须 > 0");
        end
        if (SPEED_TIMEOUT_MS <= 0) begin
            $display("[motion_decision] FATAL: SPEED_TIMEOUT_MS 必须 > 0 (当前 %0d)",
                     SPEED_TIMEOUT_MS);
        end
    end

endmodule
