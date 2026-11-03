//=============================================================================
// main_fsm -- 主状态机（自主导航 / 避障 / 越障 / 急停）
//-----------------------------------------------------------------------------
// 对应竞赛基础任务 4「自主导航」与任务 3「实时避障」的决策核心。
//
// 职责边界（很重要，别把逻辑堆到这里）：
//   本模块只做 **行为决策**：现在该走、该停、该减速、还是该抬腿。
//   具体"轮子转多快、PID 怎么算"由 motion_decision 负责。
//   本模块输出的是**意图**（speed_scale / lift_cmd / motor_enable），
//   不是最终 PWM。
//
//-----------------------------------------------------------------------------
// 设计要点
//-----------------------------------------------------------------------------
// 1) 急停是**硬件级最高优先级**
//    espot 直接组合进 motor_enable（assign motor_enable = en & ~estop），
//    从 estop 有效到电机断使能 **0 拍延迟**（纯组合），
//    状态机最迟 1 拍进入 ESTOP 状态。
//    这正是"FPGA 硬实时"相对软件方案的核心卖点 —— 软件方案要经过
//    中断响应+任务调度+串口传输，延迟是毫秒级且不确定。
//
// 2) 传感器失效时 **fail-safe（失效安全）**
//    测距读数无效时，**不判定为"无障碍"**，而是按"能见度差"处理：
//    限速前进（不冒进，但也不原地卡死）。
//    这样既安全又不会因为一次读数丢失就永久停机。
//
// 3) 所有等待都有 **超时保护**
//    避障超时 -> 安全停车；抬腿超时 / 舵机故障 -> 安全停车。
//    绝不允许状态机无限期停在某个状态。
//
// 4) 三段式状态机
//    段1: 状态寄存器       (时序)
//    段2: 次态组合逻辑     (组合)
//    段3: 输出组合逻辑     (组合, Moore)
//    全部 case 带 default，不留 latch。
//
// 5) 时间参数用 **ms** 表达，内部换算成 clk 数
//    这样换时钟频率时不用改阈值语义。
//=============================================================================
`timescale 1ns/1ps

module main_fsm #(
    parameter integer CLK_FREQ_HZ      = 50_000_000,   // 系统时钟
    //----------------------------------------------------------------
    // 距离阈值 (mm)
    //----------------------------------------------------------------
    parameter integer DIST_STOP_MM     = 200,   // 低于此值 -> 必须停车避障
    parameter integer DIST_SLOW_MM     = 500,   // 低于此值 -> 减速
    parameter integer DIST_CLEAR_MM    = 800,   // 高于此值 -> 判定路径已通畅
    //----------------------------------------------------------------
    // 超时阈值 (ms)
    //----------------------------------------------------------------
    parameter integer AVOID_TIMEOUT_MS = 5000,  // 避障太久无法脱困 -> 安全停车
    parameter integer LIFT_TIMEOUT_MS  = 3000,  // 抬腿/放腿超时 -> 安全停车
    parameter integer CROSS_TIMEOUT_MS = 2000,  // 跨越阶段超时 -> 收腿

    //----------------------------------------------------------------
    // ★ 越障失败降级（2026-10-19 新增）
    //
    //   原来：抬腿超时 -> 直接 S_ESTOP -> 停车等人按两次 start。
    //   实际比赛里这就是**卡死**：一次没跨过去，整台车停在那儿。
    //
    //   现在：抬腿超时 -> 收腿 + 进 S_AVOID（当普通障碍物绕开），
    //         并在 LIFT_LOCKOUT_MS 内**禁止再抬腿**，
    //         免得"退回来 -> 同一帧视觉又报 0x04 -> 又去抬" 无限循环。
    //
    //   安全兜底：连续失败 LIFT_RETRY_MAX 次 -> 才真的进 S_ESTOP。
    //   连续 LIFT_CLEAR_MS 没失败 -> 失败计数清零（说明刚才是偶发）。
    //
    //   ⚠️ 舵机**故障**（lift_fault）仍然直接进 ESTOP —— 那是硬件坏了，
    //      不是"这次没跨过去"，不该用降级逻辑混过去。
    //----------------------------------------------------------------
    parameter integer LIFT_LOCKOUT_MS  = 3000,  // 失败后禁止再抬腿的时长
    parameter integer LIFT_CLEAR_MS    = 5000,  // 多久没失败就把计数清零
    parameter integer LIFT_RETRY_MAX   = 3,     // 连续失败几次才真停车
    //----------------------------------------------------------------
    // 整车通过判定：最短"保持抬起并前进"的时间（毫秒）
    //
    //   ⚠️ 这个参数是为了修一个真 bug：
    //      原来 LP_CROSS 只靠 path_clear（前向 TOF 距离变远）判定收腿。
    //      但"小石头"的定义是【长度小于整车】——
    //      车头过了石头时前向 TOF 就读到远处，此刻【后轮还没过】，
    //      立刻收腿会让石头磕到车身后部。
    //
    //   现在改成：path_clear **并且** 已经保持抬起走了至少这么久。
    //   这个时间按"车身长度 / 慢速档车速"估：
    //      车身约 200mm，慢速档约 0.15 m/s -> 200/150 ≈ 1.3 秒
    //   取 800ms 是折中（宁可多走一点，也别提前收腿）。
    //   等后向 TOF 接上来之后，这个兜底还会继续作为"最短保证时间"存在。
    //----------------------------------------------------------------
    parameter integer CROSS_MIN_MS     = 800,
    //----------------------------------------------------------------
    // 放腿后的等待时间（毫秒）
    //
    //   舵机是位置控制的，"放下"是确定动作，不需要限位反馈。
    //   给 600ms 让它走完行程，然后回 S_AUTO。
    //----------------------------------------------------------------
    parameter integer LIFT_SETTLE_MS   = 600,
    //----------------------------------------------------------------
    // ARM 视觉类别码（见 doc/uart_frame_protocol.md §3.1）
    //
    //   本模块按这张表动作：
    //       0x01 人员     -> 减速（保持距离）—— 目前等同"需减速"，不做
    //       0x02 障碍物   -> 避障（进 AVOID）
    //       0x03 目标点   -> 向目标行进（靠 steer，本模块不动状态）
    //       0x04 低矮障碍 -> 抬腿跨越（进 LIFT）
    //
    //   ⚠️ 这里曾经只实现了 0x04，导致 ARM 发 0x02（障碍物）时 FPGA
    //      **完全没反应** —— 视觉识别只能产生转向，触发不了避障。
    //      这是联调时"车看到障碍物不躲"的直接原因，已修。
    //----------------------------------------------------------------
    parameter [7:0]   VIS_LOW_OBST     = 8'h04, // 低矮障碍（小石头）-> LIFT 模式 0（四腿一起抬）
    parameter [7:0]   VIS_OBST         = 8'h02, // 障碍物（可绕行）  -> AVOID
    parameter [7:0]   VIS_STEP         = 8'h05, // 台阶/门槛         -> LIFT 模式 1（两腿分段）
    // ★ 2026-11-03 新增
    parameter [7:0]   VIS_BACK         = 8'h06, // 无法越过也无法避开 -> 后退
    parameter [7:0]   VIS_PASS         = 8'h07, // 低矮可直接通过     -> 不减速继续走
    //----------------------------------------------------------------
    // 视觉障碍物的"保持窗口"（毫秒）
    //
    //   ARM 是 10Hz 发的，vision_valid 是单拍脉冲。
    //   不加保持的话 AVOID 只持续 1 拍（详见下面 vis_obst 的注释）。
    //   取 300ms = 帧间隔(100ms) 的 3 倍，容忍偶尔丢 2 帧。
    //
    //   ⚠️ 必须 > 1000/TELEM_RATE_HZ，否则每帧之间会解除一次，等于没保持。
    //      收帧频率是 ARM 侧的 TARGET_HZ（camera_vision.py 里是 10Hz）。
    //----------------------------------------------------------------
    parameter integer VIS_HOLD_MS      = 300,
    //----------------------------------------------------------------
    // ★★ 2026-11-03 新增（视频演示版）
    //
    //   1) 障碍物检测区间 50~80cm：TOF 读数 < DIST_CHECK_MM 就停车。
    //   2) 停车后进 S_CHECK 等摄像头判高度（摄像头读遥测里的 dist_mm
    //      + 自身安装高度 12cm 推算）：
    //        0x04 低矮障碍 -> 抬腿跨过
    //        0x02 高障碍   -> 绕行（转向→跑1s→停→反向转→直行）
    //   3) 抬腿 45°（lift_height=25）、越障全速跑 1s。
    //   4) 绕行转向 45°（沿用手动转向的差速方案，时长 TURN_MS）。
    //----------------------------------------------------------------
    parameter integer DIST_CHECK_MM    = 800,   // 50~80cm 检测区（< 此值就停车检查）
    parameter integer CHECK_TIMEOUT_MS = 1500,  // 等摄像头判高度的超时
    parameter integer TURN_MS          = 600,   // 转向 45° 的时长（待实测标定）
    parameter integer RUN_MS           = 1000,  // 绕行时直行时长（1s）
    parameter integer AV_STOP_MS       = 300,   // 绕行中间停车时长
    parameter integer LIFT_CROSS_MS    = 1000   // 抬腿后全速越障时长（1s）
) (
    input  wire        clk,
    input  wire        rst_n,

    //---------------- 用户指令 ----------------
    input  wire        start,          // 单拍脉冲：启动自主导航
    input  wire        estop,          // 高有效急停（最高优先级）
    input  wire        manual_sel,     // 手动模式选择

    //---------------- TOF 测距 ----------------
    input  wire [15:0] dist_mm,        // 距离读数
    input  wire        dist_valid,     // 单拍脉冲：本次读数有效
    input  wire        dist_err,       // 单拍脉冲：本次测距失败

    //---------------- 运行时配置（来自 config_rx 的 CONFIG 帧）----------------
    //   原来这两个是编译期参数，改成端口后**不重新综合就能改避障距离**
    //   （实车标定非常有用）。参数 DIST_*_MM 保留作复位默认值。
    input  wire [15:0] dist_stop_mm,   // 低于此值必须停车（50..2000）
    input  wire [15:0] dist_slow_mm,   // 低于此值减速（必须 > dist_stop_mm）
    //   ★ 2026-10-18 新增：抬腿动作超时（CONFIG 帧 0x11）
    //     原来是编译期参数 LIFT_TIMEOUT_MS；改成端口后可以按机械实际情况调，
    //     不用为一个"这台机器的腿到底要几秒"重新综合。
    //     0 视为"用默认值"（防御性：config_rx 复位值就是这个默认值，不会真给 0）
    input  wire [15:0] lift_timeout_ms, // 抬腿/放腿超时（500..30000 ms）

    //---------------- ARM 视觉 ----------------
    input  wire [7:0]  vision_class,   // 目标类别
    input  wire        vision_valid,   // 单拍脉冲：本帧结果有效
    input  wire [15:0] vr_target_x,    // ★ 视觉目标 x 中心（用于绕行选转向）
    input  wire [15:0] run_ms,         // ★ 绕行直行时长（CONFIG 0x23，0=用默认）

    //---------------- 导航 / 越障反馈 ----------------
    input  wire        goal_reached,   // 到达终点（**低有效**，接地=到达；未接=上拉=未到达）
    //----------------------------------------------------------------
    // 升降腿反馈（2 位 = {后腿, 前腿}）
    //   高有效。来自 servo_lift 模块（那里做了消抖 + 极性转换）。
    //
    //   ⚠️ 为什么是 2 位而不是 1 位：
    //      "跨台阶"要求【前腿先抬 -> 前进 -> 后腿再抬 -> 再前进】，
    //      前后腿必须能独立控制。1 位做不到这个分段动作。
    //----------------------------------------------------------------
    input  wire [1:0]  lift_done,      // 对应腿组抬起到位（高有效）
    input  wire [1:0]  lift_fault,     // 对应腿组故障（高有效）

    //---------------- 输出（意图，不是最终 PWM）----------------
    output reg  [2:0]  state,          // 当前状态（调试/LED/上报）
    output wire        motor_enable,   // 电机使能（急停时立即为 0）
    output reg  [1:0]  speed_scale,    // 0=停 1=慢 2=正常
    //----------------------------------------------------------------
    // 升降腿指令（2 位 = {后腿, 前腿}，1=抬起）
    //   2'b00 都放下  2'b01 只抬前腿  2'b10 只抬后腿  2'b11 都抬起
    //
    //   两种越障模式：
    //     模式 0（小石头）：2'b11 一起抬 -> 走过去 -> 2'b00 一起放
    //     模式 1（台阶）  ：2'b01 抬前腿 -> 前轮上位
    //                      -> 2'b11 抬后腿（前腿保持）-> 后轮上位
    //                      -> 2'b00 都放
    //----------------------------------------------------------------
    output reg  [1:0]  lift_cmd,
    output reg  [1:0]  turn_dir,       // ★ 0=不转 1=左转 2=右转（给 motion_decision）
    output reg         dir_inv,        // ★ 1=后退（给 motion_decision 的 dir_inv）
    output reg  [2:0]  lift_phase,     // 越障子阶段（8 个）
    output wire        avoid_active,   // 正在避障（给显示/上报）
    output wire        lift_active,    // 正在越障（给显示/上报）
    output reg         error_flag      // 因超时/故障进入过安全停车
);

    //==================================================================
    // 常量
    //==================================================================
    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_MANUAL = 3'd1;
    localparam [2:0] S_AUTO   = 3'd2;
    localparam [2:0] S_AVOID  = 3'd3;
    localparam [2:0] S_LIFT   = 3'd4;
    localparam [2:0] S_GOAL   = 3'd5;
    localparam [2:0] S_ESTOP  = 3'd6;
    localparam [2:0] S_CHECK  = 3'd7;    // ★ 停车检查（等摄像头判高度）

    //------------------------------------------------------------------
    // 越障子阶段（8 个，3 位）
    //
    //   两种模式共用同一套阶段号：
    //     模式 0（小石头，四腿一起抬）走 LP_UP -> LP_CROSS -> LP_DOWN
    //     模式 1（台阶，两腿分段）走 LP_F_UP -> LP_F_CROSS
    //                                -> LP_R_UP -> LP_R_CROSS -> LP_DOWN
    //
    //   lift_mode 在进入 S_LIFT 那一刻锁存（由触发它的类别码决定），
    //   越障过程中不再改变 —— 避免中途视觉又发别的类别把序列打乱。
    //------------------------------------------------------------------
    localparam [2:0] LP_IDLE    = 3'd0;   // 不在越障
    localparam [2:0] LP_UP      = 3'd1;   // 模式0：四腿一起抬
    localparam [2:0] LP_CROSS   = 3'd2;   // 模式0：一起慢速通过
    localparam [2:0] LP_F_UP    = 3'd3;   // 模式1：前腿抬
    localparam [2:0] LP_F_CROSS = 3'd4;   // 模式1：前轮上位（慢速前进）
    localparam [2:0] LP_R_UP    = 3'd5;   // 模式1：后腿抬（前腿保持抬起）
    localparam [2:0] LP_R_CROSS = 3'd6;   // 模式1：后轮上位（慢速前进）
    localparam [2:0] LP_DOWN    = 3'd7;   // 公共：都放下

    // 越障模式
    localparam        LIFT_MODE_BULK = 1'b0;   // 四腿一起抬（小石头）
    localparam        LIFT_MODE_STEP = 1'b1;   // 两腿分段（台阶）

    // ★★ 2026-11-03：绕行子阶段（转向→跑1s→停→反向转）
    localparam [1:0] AP_TURN1 = 2'd0;   // 第一次转向（45°）
    localparam [1:0] AP_RUN   = 2'd1;   // 直行 1s
    localparam [1:0] AP_STOP  = 2'd2;   // 停车
    localparam [1:0] AP_TURN2 = 2'd3;   // 反方向转回（恢复朝向）

    localparam [1:0] SPD_STOP = 2'd0;
    localparam [1:0] SPD_SLOW = 2'd1;
    localparam [1:0] SPD_FULL = 2'd2;

    //------------------------------------------------------------------
    // ms -> clk 换算
    //   用 integer 中间量，避免在参数表达式里溢出
    //------------------------------------------------------------------
    localparam integer MS_TICKS        = CLK_FREQ_HZ / 1000;
    localparam integer AVOID_TMO_TICKS = AVOID_TIMEOUT_MS * MS_TICKS;
    localparam integer CROSS_TMO_TICKS = CROSS_TIMEOUT_MS * MS_TICKS;
    localparam integer CROSS_MIN_TICKS = CROSS_MIN_MS     * MS_TICKS;
    localparam integer SETTLE_TICKS    = LIFT_SETTLE_MS   * MS_TICKS;

    //------------------------------------------------------------------
    // ⚠️ 下面这些"宽度定型"的常量是必须的，不是冗余：
    //   Verilog-2001 **不允许对 parameter 做位选**（`P[15:0]` 非法，
    //   那是 SystemVerilog 的语法）。本模块早先就直接写了
    //   `dist_r < DIST_STOP_MM[15:0]`，在 uart_tx 上也踩过同样的坑。
    //   统一在这里转成定宽 wire，后面只用这些名字。
    //------------------------------------------------------------------
    // ★ 这两个值现在来自 config_rx（运行时可变），参数只作复位默认值。
    wire [15:0] DIST_STOP_W  = dist_stop_mm;
    wire [15:0] DIST_SLOW_W  = dist_slow_mm;

    // ★ 抬腿超时也改成运行时（毫秒）。为了让"次数阈值"不用做乘法，
    //   `t_up` 改成**数毫秒**（每 1ms 加 1），直接和这个毫秒值比较。
    //   0 视为"用默认值"，防止有人接错线把端口拉成 0 -> 一进 LIFT 就超时。
    localparam [15:0] DEF_LIFT_TMO_MS_W = LIFT_TIMEOUT_MS;
    wire [15:0] TMO_MS_W16 = (lift_timeout_ms == 16'd0) ? DEF_LIFT_TMO_MS_W
                                                       : lift_timeout_ms;
    wire [31:0] TMO_MS_W   = {16'd0, TMO_MS_W16};

    localparam [15:0] DIST_CLEAR_W = DIST_CLEAR_MM;
    localparam [31:0] AVOID_TMO_W  = AVOID_TMO_TICKS;
    localparam [31:0] CROSS_TMO_W  = CROSS_TMO_TICKS;
    localparam [31:0] CROSS_MIN_W  = CROSS_MIN_TICKS;
    localparam [31:0] SETTLE_W     = LIFT_SETTLE_MS;   // ★ 2026-11-03：t_down 已改毫秒，这里也跟毫秒
    // ★★ 2026-11-03 新增的毫秒阈值
    localparam [15:0] DIST_CHECK_W    = DIST_CHECK_MM;
    localparam [31:0] CHECK_TMO_W     = CHECK_TIMEOUT_MS;
    localparam [31:0] TURN_W          = TURN_MS;
    localparam [31:0] RUN_W           = RUN_MS;
    localparam [31:0] AV_STOP_W       = AV_STOP_MS;
    localparam [31:0] LIFT_CROSS_W    = LIFT_CROSS_MS;
    // ★ 2026-11-03：绕行直行时长 = 运行时值（相机按实测宽度发）
    //   0 视为未配置，退回默认 RUN_MS
    localparam [15:0] RUN_MS_DEF = RUN_MS;
    wire [31:0] RUN_W_DYN = (run_ms == 16'd0) ? {16'd0, RUN_MS_DEF} : {16'd0, run_ms};
    localparam [31:0] CNT_MAX_W    = 32'h7FFFFFFF;   // 计数器上限（整数形式）
    localparam real   CNT_MAX_R    = 2147483647.0;   // 同一上限（实数形式，供自检比较）

    //==================================================================
    // TOF 读数锁存 + 有效性
    //
    //   保留"最近一次有效读数"和"当前是否可信"两个信息。
    //   测距失败只清有效标志，**不清读数** —— 保留旧值便于诊断，
    //   而判断逻辑只看有效标志（fail-safe）。
    //==================================================================
    reg [15:0] dist_r;
    reg        dist_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dist_r  <= 16'd0;
            dist_ok <= 1'b0;
        end else begin
            if (dist_valid) begin
                dist_r  <= dist_mm;
                dist_ok <= 1'b1;
            end else if (dist_err) begin
                dist_ok <= 1'b0;         // 失效 -> 后续按"限速"处理
            end
        end
    end

    //==================================================================
    // 环境判定（组合）
    //==================================================================
    // 定宽常量（Verilog-2001 不允许对 parameter 做位选）
    localparam [31:0] VIS_HOLD_W = VIS_HOLD_MS * MS_TICKS;

    wire obst_stop  = dist_ok && (dist_r < DIST_STOP_W);
    // 读数不可信时也算"需减速" —— fail-safe，不冒进
    wire obst_slow  = !dist_ok || (dist_r < DIST_SLOW_W);
    wire path_clear = dist_ok && (dist_r > DIST_CLEAR_W);

    //------------------------------------------------------------------
    // 视觉触发
    //
    //   low_obst : 类别 0x04 -> 抬腿跨越（进 LIFT）
    //   vis_obst : 类别 0x02 -> 绕行（进 AVOID）
    //
    //   两者互斥：一帧里 ARM 只发一个类别。
    //
    //------------------------------------------------------------------
    // ⚠️ 为什么 vis_obst 必须"保持"，不能只用单拍脉冲
    //------------------------------------------------------------------
    //   ARM 是 **10Hz** 发的，`vision_valid` 只在收到一帧的那**一拍**为 1，
    //   其余 99.9% 的时间是 0（@100MHz，一帧间隔是 1000 万拍）。
    //
    //   如果 vis_obst 直接用脉冲：
    //       第 N 拍     vision_valid=1 -> 进 AVOID
    //       第 N+1 拍   vision_valid=0 -> vis_obst=0，而 TOF 通畅
    //                   -> path_clear && !vis_obst 成立 -> **立刻退回 AUTO**
    //   结果：AVOID 只持续 1 拍，等于没有避障。
    //
    //   所以这里加一个"保持窗口"：收到障碍物后，把 vis_obst 拉高并保持
    //   VIS_HOLD_MS 毫秒。只要 ARM 还在 10Hz 报（100ms 一次），
    //   保持就会被不断续期，AVOID 稳稳维持住；
    //   ARM 不再报之后，最多 VIS_HOLD_MS 就自动解除。
    //
    //   VIS_HOLD_MS 取 300ms：大于帧间隔(100ms) 的 2 倍，容忍偶尔丢 1~2 帧；
    //   又不至于"障碍物已经过去很久了还赖着不走"。
    //
    //   low_obst 不需要保持：进了 LIFT 之后由 lift_done 和超时驱动，
    //   不再依赖视觉信号，所以一拍脉冲足够。
    //------------------------------------------------------------------
    wire low_obst_pulse = vision_valid && (vision_class == VIS_LOW_OBST);
    wire low_obst       = low_obst_pulse;      // 进 LIFT 只需一拍，不保持

    // 台阶/门槛：类别 0x05 -> LIFT 模式 1（两腿分段抬升）
    //
    //   ⚠️ 这个类别码需要 ARM 侧配合新增。在 B 加上之前，
    //      这里永远收不到，跨台阶功能不会被触发（但也不会误触发）。
    wire step_obst_pulse = vision_valid && (vision_class == VIS_STEP);
    wire step_obst       = step_obst_pulse;    // 同样只需一拍

    //------------------------------------------------------------------
    // ★ 越障失败降级（2026-10-19）
    //
    //   lift_locked = 刚发生过"抬腿超时"，这段时间内**不允许再抬腿**。
    //   被门控掉的 0x04/0x05 不会触发 LIFT，车会当普通障碍物处理
    //   （由 TOF 推着进 S_AVOID 绕行）—— 这就是"跨不过就绕"。
    //------------------------------------------------------------------
    // ★ 这些寄存器必须在下面 lift_locked 之前声明（Verilog-2001 先用后声明会变成隐式 1 位网）
    reg [31:0] lockout_ms;
    reg [31:0] clean_ms;
    reg [2:0]  lift_fail_cnt;    // 3 位：避免 cnt+1 在 3 时回绕

    wire lift_locked = (lockout_ms != 32'd0);

    // 门控后的"真正可以触发抬腿"的信号
    wire low_obst_lift  = low_obst  && !lift_locked;
    wire step_obst_lift = step_obst && !lift_locked;

    reg  [31:0] t_vis;                          // 保持窗口计时
    reg         vis_obst_hold;

    wire vis_obst_pulse = vision_valid && (vision_class == VIS_OBST);
    wire vis_obst       = vis_obst_hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vis_obst_hold <= 1'b0;
            t_vis         <= 32'd0;
        end else if (vis_obst_pulse) begin
            vis_obst_hold <= 1'b1;              // 收到就拉高
            t_vis         <= 32'd0;             // 并重新计时（续期）
        end else if (t_vis >= VIS_HOLD_W) begin
            vis_obst_hold <= 1'b0;              // 窗口到期自动解除
        end else begin
            t_vis <= t_vis + 32'd1;
        end
    end

    //==================================================================
    // 段 1：状态寄存器 + 三个分阶段超时计数器
    //==================================================================
    reg [2:0]  nphase;               // 次态的子阶段（组合）
    reg [2:0]  nstate;               // 次态（组合）

    // 本轮越障用哪种模式（进入 S_LIFT 时锁存）
    reg        lift_mode;            // 0=四腿一起抬  1=两腿分段
    reg        n_lift_mode;          // 它的次态（组合）

    reg [31:0] t_avoid;              // AVOID 停留时间
    reg [31:0] t_up;                 // LIFT 抬腿阶段（毫秒）
    reg [31:0] t_cross;              // LIFT 通过阶段（毫秒）
    reg [31:0] t_down;               // LIFT 放下阶段（毫秒）
    // ★★ 2026-11-03 新增
    reg [31:0] t_check;              // S_CHECK 停车检查耗时（毫秒）
    reg [31:0] t_av;                 // 绕行子阶段耗时（毫秒）
    reg [1:0]  avoid_phase;          // 绕行子阶段
    reg [1:0]  n_avoid_phase;
    reg        turn_1st;             // 第一次转向方向：0=左 1=右（闸存）
    reg        n_turn_1st;
    reg        back_mode;            // ★ 2026-11-03：1=本次避障是「后退」而不是「转向」
    reg        n_back_mode;

    // 各计数器的使能条件（只在该阶段累加，离开即清零）
    wire cnt_avoid = (state == S_AVOID);
    wire cnt_check = (state == S_CHECK);      // ★ 停车检查计时
    wire cnt_av    = (state == S_AVOID);      // ★ 绕行子阶段计时
    // 抬腿阶段：模式 0 是 LP_UP，模式 1 是 LP_F_UP / LP_R_UP，共用一个计时器
    wire cnt_up    = (state == S_LIFT) &&
                     ((lift_phase == LP_UP)    ||
                      (lift_phase == LP_F_UP)  ||
                      (lift_phase == LP_R_UP));
    // 通过阶段：模式 0 是 LP_CROSS，模式 1 是 LP_F_CROSS / LP_R_CROSS
    wire cnt_cross = (state == S_LIFT) &&
                     ((lift_phase == LP_CROSS)   ||
                      (lift_phase == LP_F_CROSS) ||
                      (lift_phase == LP_R_CROSS));
    wire cnt_down  = (state == S_LIFT) && (lift_phase == LP_DOWN);

    //------------------------------------------------------------------
    // ★ 1ms 节拍（2026-10-18 新增，给"抬腿超时"用）
    //
    //   为什么需要：抬腿超时现在来自 CONFIG 帧（运行时可变）。如果 `t_up`
    //   还是数时钟拍，比较时就得把"毫秒"换算成"拍"——那是运行时乘法，
    //   要占一个 16x16 乘法器（几百 LUT），不值得。
    //   改成 **`t_up` 数毫秒** + 一个 1ms 节拍，就只剩一个比较器。
    //
    //   代价：LIFT 超时的分辨率从 20ns 变成 1ms —— 对"等舵机到位"完全够。
    //------------------------------------------------------------------
    reg [31:0] ms_cnt;
    wire       ms_tick = (ms_cnt >= (MS_TICKS[31:0] - 32'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt <= 32'd0;
        end else if (ms_tick) begin
            ms_cnt <= 32'd0;
        end else begin
            ms_cnt <= ms_cnt + 32'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            lift_phase   <= LP_IDLE;
            lift_mode    <= LIFT_MODE_BULK;
            avoid_phase  <= AP_TURN1;
            turn_1st     <= 1'b1;      // 默认右转
            back_mode    <= 1'b0;
            t_avoid      <= 32'd0;
            t_up         <= 32'd0;
            t_cross      <= 32'd0;
            t_down       <= 32'd0;
            t_check      <= 32'd0;
            t_av         <= 32'd0;
        end else begin
            state        <= nstate;
            lift_phase   <= nphase;
            lift_mode    <= n_lift_mode;
            avoid_phase  <= n_avoid_phase;
            turn_1st     <= n_turn_1st;
            back_mode    <= n_back_mode;

            t_avoid <= cnt_avoid ? (t_avoid + 32'd1) : 32'd0;
            t_up    <= !cnt_up ? 32'd0 : (ms_tick ? (t_up + 32'd1) : t_up);
            // ★ t_cross 改成数毫秒（供 LIFT_CROSS_MS 使用）
            t_cross <= !cnt_cross ? 32'd0 : (ms_tick ? (t_cross + 32'd1) : t_cross);
            t_down  <= !cnt_down  ? 32'd0 : (ms_tick ? (t_down  + 32'd1) : t_down);
            t_check <= !cnt_check ? 32'd0 : (ms_tick ? (t_check + 32'd1) : t_check);
            t_av    <= !cnt_av    ? 32'd0 : (ms_tick ? (t_av    + 32'd1) : t_av);
        end
    end

    //==================================================================
    // ★ 越障失败降级：锁定计时 + 失败计数（2026-10-19）
    //
    //   三个寄存器：
    //     lockout_ms   : 还剩多少毫秒"禁止抬腿"（0 = 不锁）
    //     lift_fail_cnt: 连续失败次数（到 LIFT_RETRY_MAX 就真进 ESTOP）
    //     clean_ms     : 距离上次失败已经"干净"了多久（够长就清零失败计数）
    //
    //   用一个独立的 always 块，不去动原来的计数器块 ——
    //   那个块里有 t_up/t_avoid 的时序，混进来容易改坏。
    //==================================================================
    // 组合：本拍是不是"抬腿超时"（只认超时，不认舵机故障）
    wire lift_tmo_now = (state == S_LIFT) && (t_up >= TMO_MS_W);

    localparam [31:0] LOCKOUT_W = LIFT_LOCKOUT_MS;
    localparam [31:0] CLEAR_W   = LIFT_CLEAR_MS;
    localparam [2:0]  RETRY_W   = LIFT_RETRY_MAX;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lockout_ms    <= 32'd0;
            clean_ms      <= 32'd0;
            lift_fail_cnt <= 3'd0;
        end else begin
            //---- 锁定倒计时（每毫秒减 1）----
            if (lift_tmo_now) begin
                lockout_ms <= LOCKOUT_W;              // 触发：重新上锁
            end else if (ms_tick) begin
                if (lockout_ms != 32'd0)
                    lockout_ms <= lockout_ms - 32'd1;
            end

            //---- 失败计数 ----
            //   超时 -> +1；连续干净 CLEAR_W 毫秒 -> **一次性清零**
            //
            //   ⚠️ 第一版写成"clean_ms 满后每毫秒减 1"，那样 3ms 就恢复到 0 了，
            //      等于失败计数形同虚设。必须是"干净够久 -> 清零一次"。
            if (lift_tmo_now) begin
                clean_ms <= 32'd0;
                if (lift_fail_cnt != RETRY_W)
                    lift_fail_cnt <= lift_fail_cnt + 3'd1;
            end else if (ms_tick) begin
                if (clean_ms < CLEAR_W) begin
                    clean_ms <= clean_ms + 32'd1;
                    // 刚好数到 CLEAR_W 的那一拍：清零一次（之后 clean_ms 饱和不再变）
                    if ((clean_ms + 32'd1) >= CLEAR_W)
                        lift_fail_cnt <= 3'd0;
                end
            end
        end
    end

    //==================================================================
    // 段 2：次态组合逻辑
    //
    //   优先级（从高到低）：
    //     急停 > 越障/避障 > 正常导航 > 上层指令
    //==================================================================
    //------------------------------------------------------------------
    // "通过完成"的判定（修了一个真 bug）
    //
    //   原来只写 `if (path_clear)`。但 path_clear 来自【前向 TOF】：
    //
    //       车头越过石头 -> 前向 TOF 读到远处 -> path_clear=1
    //                                    ↑
    //                      此刻【后轮还在石头上】！
    //                                    ↓
    //                       收腿 -> 车身下降 -> 石头磕到车底
    //
    //   现在要求 **前向通畅 并且 已经保持抬起走了 CROSS_MIN_MS**。
    //   最小时间按"车身长度 / 慢速档车速"估，宁可多走一点。
    //------------------------------------------------------------------
    wire cross_blocked = (t_cross >= CROSS_TMO_W);
    wire cross_ok      = (path_clear && (t_cross >= CROSS_MIN_W));

    // 手动位下降沿：单拍脉冲（用于从手动自动启动 AUTO）
    reg  manual_d;
    wire manual_fell = manual_d && !manual_sel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) manual_d <= 1'b0;
        else        manual_d <= manual_sel;
    end

    always @(*) begin
        // 默认保持
        nstate        = state;
        nphase        = lift_phase;
        n_lift_mode   = lift_mode;
        n_avoid_phase = avoid_phase;
        n_turn_1st    = turn_1st;
        n_back_mode   = back_mode;

        if (estop) begin
            //------------------------------------------------------
            // 急停最高优先级：无论当前在做什么，立刻进入 ESTOP
            //------------------------------------------------------
            nstate = S_ESTOP;
        end else if (manual_sel && (state != S_MANUAL) && (state != S_ESTOP)) begin
            //------------------------------------------------------
            // ★ 2026-10-30：手动模式必须能从【任何状态】接管
            //
            //   原来只有 S_IDLE 里检查 manual_sel ->
            //   一旦按过 S0 进了 S_AUTO，遥控的"停车"就被状态机完全忽略，
            //   电机一直转、按停也不停（实测事故）。
            //
            //   现在：只要 manual_sel=1 且不在 S_MANUAL/S_ESTOP，立刻切 S_MANUAL。
            //   （ESTOP 故意排除：急停解除必须人工再按 start，不能靠切模式绕过）
            //------------------------------------------------------
            nstate = S_MANUAL;
        end else begin
            case (state)
                //------------------------------------------------------
                S_IDLE: begin
                    if (manual_sel)      nstate = S_MANUAL;
                    else if (start || manual_fell) begin
                        // 2026-10-30: 从手动切回自动时自动启动，不用再按 S0
                        nstate = S_AUTO;
                        nphase = LP_IDLE;    // 复位子阶段
                    end
                end

                //------------------------------------------------------
                S_MANUAL: begin
                    // 手动模式：只受 manual_sel 控制，不做自主决策
                    if (!manual_sel) nstate = S_IDLE;
                end

                //------------------------------------------------------
                S_AUTO: begin
                    // ★★ 2026-11-03：TOF 检测到 50~80cm 内有障碍物 -> 立即停车检查
                    //   停车后进 S_CHECK，由摄像头判高度：
                    //     低矮 0x04 -> 抬腿跨过；高 0x02 -> 绕行
                    if (dist_ok && (dist_r < DIST_CHECK_W)) begin
                        nstate = S_CHECK;
                    end else if (!goal_reached) begin
                        nstate = S_GOAL;
                    end
                end

                //------------------------------------------------------
                // ★★ 2026-11-03 新增：停车检查（等摄像头判障碍物高度）
                //
                //   停车后 TOF 会再测一次（车还会往前滑一点），
                //   摄像头通过遥测拿到这个距离，再结合自身安装高度 12cm
                //   推算障碍物高度，然后发类别码回来。
                //------------------------------------------------------
                S_CHECK: begin
                    if (low_obst_lift) begin
                        // 低矮障碍（摄像头判定可抬腿跨过）
                        nstate      = S_LIFT;
                        nphase      = LP_UP;
                        n_lift_mode = LIFT_MODE_BULK;
                    end else if (vision_valid && (vision_class == VIS_PASS)) begin
                        // ★ <5cm 低矮：不减速直接过 -> 立刻回 AUTO
                        nstate = S_AUTO;

                    end else if (vision_valid && (vision_class == VIS_BACK)) begin
                        // ★ >8cm 抬腿也过不去、绕也绕不开 -> 后退
                        nstate        = S_AVOID;
                        n_avoid_phase = AP_TURN1;
                        n_back_mode   = 1'b1;

                    end else if (vis_obst) begin
                        // 高障碍（抬腿过不去）-> 绕行
                        //   转向方向由摄像头给的 vr_target_x 决定：
                        //     < 0x8000 -> 左转；>= 0x8000 -> 右转（居中也是右转）
                        nstate        = S_AVOID;
                        n_avoid_phase = AP_TURN1;
                        n_turn_1st    = (vr_target_x >= 16'h8000) ? 1'b1 : 1'b0;
                        n_back_mode   = 1'b0;
                    end else if (t_check >= CHECK_TMO_W) begin
                        // 摄像头超时没回结果 -> 先回到自动导航
                        nstate = S_AUTO;
                    end
                end

                //------------------------------------------------------
                //------------------------------------------------------
                // ★★ 2026-11-03：绕行（转向 -> 跑 1s -> 停 -> 反方向转 -> 回 AUTO）
                //
                //   先斜着走出去（转 45° + 跑 1s），再转回去，
                //   车就横向平移了一段，绕过障碍物后继续直行。
                //------------------------------------------------------
                S_AVOID: begin
                    case (avoid_phase)
                        AP_TURN1: begin
                            if (t_av >= TURN_W) begin
                                if (back_mode) begin
                                    n_back_mode = 1'b0;
                                    nstate      = S_AUTO;   // 后退完直接回自主
                                end else begin
                                    n_avoid_phase = AP_RUN;
                                end
                            end
                        end
                        AP_RUN: begin
                            // ★ 2026-11-03：用运行时时长（宽度/最大速度 + 0.2s）
                            if (t_av >= RUN_W_DYN) n_avoid_phase = AP_STOP;
                        end
                        AP_STOP: begin
                            if (t_av >= AV_STOP_W) n_avoid_phase = AP_TURN2;
                        end
                        AP_TURN2: begin
                            if (t_av >= TURN_W) begin
                                n_avoid_phase = AP_TURN1;
                                nstate        = S_AUTO;   // 绕完回自动导航
                            end
                        end
                        default: n_avoid_phase = AP_TURN1;
                    endcase
                end

                //------------------------------------------------------
                //------------------------------------------------------
                // 越障（两种模式，共用同一套阶段号）
                //
                //   模式 0（小石头，lift_mode=0）：
                //     LP_UP   四腿一起抬          motor 停
                //     LP_CROSS 保持抬起慢速通过    motor 慢速
                //     LP_DOWN 一起放下             motor 停
                //
                //   模式 1（台阶，lift_mode=1）：
                //     LP_F_UP    前腿抬             motor 停
                //     LP_F_CROSS 前轮上位（慢速前进）motor 慢速
                //     LP_R_UP    后腿抬（前腿保持）  motor 停
                //     LP_R_CROSS 后轮上位（慢速前进）motor 慢速
                //     LP_DOWN    都放下              motor 停
                //
                //------------------------------------------------------------------
                // ⚠️ "通过完成"的判定（修了一个真 bug）
                //
                //   原来只写 `if (path_clear)`。但 path_clear 来自【前向 TOF】：
                //
                //       车头越过石头 -> 前向 TOF 读到远处 -> path_clear=1
                //                                    ↑
                //                      此刻【后轮还在石头上】！
                //                                    ↓
                //                       收腿 -> 车身下降 -> 石头磕到车底
                //
                //   现在要求 **前向通畅 并且 已经保持抬起走了 CROSS_MIN_MS**。
                //   最小时间按"车身长度 / 慢速档车速"估，宁可多走一点。
                //
                //   等后向 TOF 接上来之后，还可以再加一路 `path_clear_rear`
                //   一起判（更准），但最小时间这个兜底要保留 —— 它保证
                //   即使 TOF 读数抖动也不会提前收腿。
                //------------------------------------------------------------------

                S_LIFT: begin
                    case (lift_phase)

                        //=========== 模式 0：四腿一起抬 ===========
                        LP_UP: begin
                            // 两组都要到位
                            if (lift_done == 2'b11) begin
                                nphase = LP_CROSS;
                            end else if (|lift_fault) begin
                                // 舵机自报故障 = 硬件坏了，直接安全停车
                                nstate = S_ESTOP;
                            end else if (t_up >= TMO_MS_W) begin
                                // 超时 = 这次没跨过去 -> 收腿绕行；连续  次才真停车
                                nstate = ((lift_fail_cnt + 3'd1) >= RETRY_W) ? S_ESTOP : S_AVOID;
                            end
                        end
                        LP_CROSS: begin
                            // ★ 2026-11-03：改成"抬腿后半速跑 LIFT_CROSS_MS（1s）"
                            if (t_cross >= LIFT_CROSS_W) begin
                                nphase = LP_DOWN;
                            end
                        end

                        //=========== 模式 1：两腿分段 ===========
                        LP_F_UP: begin
                            // 只等前腿到位（后腿此时还没开始抬）
                            //   用 lift_done[0]（bit0 = 前腿）而不是整字比较，
                            //   因为后腿此刻必然是 0，整字永远不可能等于 2'b11。
                            if (lift_done[0]) begin
                                nphase = LP_F_CROSS;
                            end else if (lift_fault[0]) begin
                                // 舵机自报故障 = 硬件坏了，直接安全停车
                                nstate = S_ESTOP;
                            end else if (t_up >= TMO_MS_W) begin
                                // 超时 = 这次没跨过去 -> 收腿绕行；连续  次才真停车
                                nstate = ((lift_fail_cnt + 3'd1) >= RETRY_W) ? S_ESTOP : S_AVOID;
                            end
                        end
                        LP_F_CROSS: begin
                            if (cross_ok || cross_blocked) begin
                                nphase = LP_R_UP;
                            end
                        end
                        LP_R_UP: begin
                            // 后腿也到位（前腿保持抬起，所以看 [1]）
                            if (lift_done[1]) begin
                                nphase = LP_R_CROSS;
                            end else if (lift_fault[1]) begin
                                // 舵机自报故障 = 硬件坏了，直接安全停车
                                nstate = S_ESTOP;
                            end else if (t_up >= TMO_MS_W) begin
                                // 超时 = 这次没跨过去 -> 收腿绕行；连续  次才真停车
                                nstate = ((lift_fail_cnt + 3'd1) >= RETRY_W) ? S_ESTOP : S_AVOID;
                            end
                        end
                        LP_R_CROSS: begin
                            if (cross_ok || cross_blocked) begin
                                nphase = LP_DOWN;
                            end
                        end

                        //=========== 公共：放下 ===========
                        LP_DOWN: begin
                            // 舵机是位置控制的，"放下"不需要限位反馈，
                            // 给固定时间走完行程就回 AUTO。
                            if (t_down >= SETTLE_W) begin
                                nstate = S_AUTO;
                                nphase = LP_IDLE;
                            end
                        end

                        default: nstate = S_ESTOP;
                    endcase
                end

                //------------------------------------------------------
                S_GOAL: begin
                    // 到终点停车；再次按 start 重新出发
                    if (start) begin
                        nstate = S_AUTO;
                        nphase = LP_IDLE;
                    end
                end

                //------------------------------------------------------
                S_ESTOP: begin
                    // 急停解除后必须再按一次 start 才恢复（防意外自启动）
                    if (start) nstate = S_IDLE;
                end

                //------------------------------------------------------
                default: nstate = S_IDLE;
            endcase
        end
    end

    //==================================================================
    // 段 3：输出组合逻辑（Moore，只依赖 state / lift_phase）
    //==================================================================
    reg motor_en_r;

    always @(*) begin
        // 安全默认值：一律先当"停"
        motor_en_r  = 1'b0;
        speed_scale = SPD_STOP;
        lift_cmd    = 2'b00;
        turn_dir    = 2'd0;
        dir_inv     = 1'b0;

        case (state)
            //----------------------------------------------------------
            S_IDLE, S_GOAL, S_ESTOP: begin
                motor_en_r  = 1'b0;
                speed_scale = SPD_STOP;
                lift_cmd    = 2'b00;
            end

            //----------------------------------------------------------
            S_MANUAL: begin
                motor_en_r  = 1'b1;
                speed_scale = SPD_SLOW;      // 手动模式限速，安全起见
                lift_cmd    = 2'b00;
            end

            //----------------------------------------------------------
            S_AUTO: begin
                motor_en_r  = 1'b1;
                speed_scale = obst_slow ? SPD_SLOW : SPD_FULL;
                lift_cmd    = 2'b00;
            end

            //----------------------------------------------------------
            //----------------------------------------------------------
            // ★★ 2026-11-03：绕行四阶段
            //   AP_TURN1 转向 45°（停着原地转，差速由 motion_decision 做）
            //   AP_RUN   直行 1s（全速）
            //   AP_STOP  停车 0.3s
            //   AP_TURN2 反方向转 45°（恢复朝向）
            //----------------------------------------------------------
            S_AVOID: begin
                lift_cmd = 2'b00;
                case (avoid_phase)
                    AP_TURN1: begin
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_SLOW;
                        if (back_mode) begin
                            // ★ 后退：方向取反，不转向
                            dir_inv     = 1'b1;
                            turn_dir    = 2'd0;
                        end else begin
                            dir_inv     = 1'b0;
                            turn_dir    = turn_1st ? 2'd2 : 2'd1;
                        end
                    end
                    AP_RUN: begin
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_FULL;
                        turn_dir    = 2'd0;
                    end
                    AP_STOP: begin
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                        turn_dir    = 2'd0;
                    end
                    AP_TURN2: begin
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_SLOW;
                        turn_dir    = turn_1st ? 2'd1 : 2'd2;   // 反方向
                    end
                    default: begin
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                        turn_dir    = 2'd0;
                    end
                endcase
            end

            //----------------------------------------------------------
            // ★★ 2026-11-03：停车检查（等摄像头判高度）
            //----------------------------------------------------------
            S_CHECK: begin
                motor_en_r  = 1'b0;
                speed_scale = SPD_STOP;
                lift_cmd    = 2'b00;
                turn_dir    = 2'd0;
            end

            //----------------------------------------------------------
            // 越障输出
            //   lift_cmd = {后腿, 前腿}
            //------------------------------------------------------------------
            // 为什么"抬腿阶段"要停电机
            //   腿在动作时如果还在走，轮子会给舵机额外的侧向载荷，
            //   而且车身姿态不稳容易倒。停下来抬更安全。
            //   代码里 LP_UP / LP_F_UP / LP_R_UP 都是 motor_en_r=0。
            //
            // 为什么模式 1 的 LP_R_UP 要抬【两组】
            //   前轮已经站在台阶上了，这时候如果前腿放下，
            //   车头会掉下来卡在台阶边缘。所以抬后腿时前腿必须保持抬起。
            //   所以 lift_cmd = 2'b11（前+后都抬）。
            //------------------------------------------------------------------
            S_LIFT: begin
                case (lift_phase)
                    //===== 模式 0：四腿一起抬（过小石头）=====
                    LP_UP: begin
                        lift_cmd    = 2'b11;  // 前后一起抬
                        motor_en_r  = 1'b0;   // 抬腿时不走，避免拖拽
                        speed_scale = SPD_STOP;
                    end
                    LP_CROSS: begin
                        lift_cmd    = 2'b11;  // 保持抬起
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_FULL;  // ★ 2026-11-03：全速通过（1s）
                    end

                    //===== 模式 1：两腿分段（跨台阶）=====
                    LP_F_UP: begin
                        lift_cmd    = 2'b01;  // 只抬前腿
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                    end
                    LP_F_CROSS: begin
                        lift_cmd    = 2'b01;  // 前腿保持抬起，前进让前轮上位
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_SLOW;
                    end
                    LP_R_UP: begin
                        lift_cmd    = 2'b11;  // 前腿保持 + 抬后腿
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                    end
                    LP_R_CROSS: begin
                        lift_cmd    = 2'b11;  // 都保持，前进让后轮上位
                        motor_en_r  = 1'b1;
                        speed_scale = SPD_SLOW;
                    end

                    //===== 公共：放下 =====
                    LP_DOWN: begin
                        lift_cmd    = 2'b00;  // 都放下
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                    end
                    default: begin
                        lift_cmd    = 2'b00;
                        motor_en_r  = 1'b0;
                        speed_scale = SPD_STOP;
                    end
                endcase
            end

            //----------------------------------------------------------
            default: begin
                motor_en_r  = 1'b0;
                speed_scale = SPD_STOP;
                lift_cmd    = 2'b00;
            end
        endcase
    end

    //------------------------------------------------------------------
    // 急停直接组合进使能：从 estop 有效到电机断使能 0 拍延迟
    //   这是本模块最重要的时序特性，不要改成寄存器输出！
    //------------------------------------------------------------------
    assign motor_enable = motor_en_r & ~estop;

    assign avoid_active = (state == S_AVOID);
    assign lift_active  = (state == S_LIFT);

    //==================================================================
    // error_flag：区分"人为急停"和"异常导致的安全停车"
    //   因超时/故障**进入** ESTOP -> error_flag = 1
    //   因 estop 信号进入        -> error_flag 保持 0
    //   回到 IDLE 清除
    //
    // ⚠️ 必须只在【进入 ESTOP 的那一拍】判定。不能写成：
    //       else if ((nstate == S_ESTOP) && !estop) error_flag <= 1'b1;
    //    因为人为急停松开 estop 之后，状态还停在 ESTOP 而 estop 已变低，
    //    那种写法会把"人为急停"误报成"异常停车"。
    //    加 (state != S_ESTOP) 把它限定为"跳变沿"即可。
    //    （这是 Python 模型跑场景 5 时发现的。）
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_flag <= 1'b0;
        end else if (state == S_IDLE) begin
            error_flag <= 1'b0;
        end else if ((state != S_ESTOP) && (nstate == S_ESTOP) && !estop) begin
            error_flag <= 1'b1;
        end
    end

    //==================================================================
    // 参数自检（elaboration 期提示，便于早点发现配错）
    //
    // ⚠️ 这里的比较必须用**实数**做，不能用 integer 乘法：
    //      CLK_FREQ_HZ * AVOID_TIMEOUT_MS = 50e6 * 5000 = 2.5e11
    //    这在 32 位 integer 里会【溢出回绕】，判断结果完全错误。
    //    （自检代码自己踩了它本该防的坑 —— 用 real 换算即可。）
    //==================================================================
    initial begin
        if (MS_TICKS < 1) begin
            $display("[main_fsm] FATAL: CLK_FREQ_HZ=%0d 太低，MS_TICKS=0",
                     CLK_FREQ_HZ);
        end
        if (!(DIST_STOP_MM < DIST_SLOW_MM && DIST_SLOW_MM < DIST_CLEAR_MM)) begin
            $display("[main_fsm] FATAL: 距离阈值必须满足 STOP(%0d) < SLOW(%0d) < CLEAR(%0d)",
                     DIST_STOP_MM, DIST_SLOW_MM, DIST_CLEAR_MM);
        end
        // 用 real 比较，避免 32 位整数乘法溢出（上限也用具名常量）
        if ((CLK_FREQ_HZ * 1.0 * AVOID_TIMEOUT_MS / 1000.0) >= CNT_MAX_R) begin
            $display("[main_fsm] WARN: AVOID 超时换算后超出 32 位计数器范围");
        end
        if ((CLK_FREQ_HZ * 1.0 * LIFT_TIMEOUT_MS / 1000.0) >= CNT_MAX_R) begin
            $display("[main_fsm] WARN: LIFT 超时换算后超出 32 位计数器范围");
        end
    end

endmodule
