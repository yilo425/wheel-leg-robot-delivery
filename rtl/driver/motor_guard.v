//=============================================================================
// module : motor_guard
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : **四路电机护栏**（软启动 + 堵转保护 + 冷却恢复），夹在
//        motion_decision 和 pwm_motor 之间。
//
//       motion_decision ──duty──► motor_guard ──duty_safe──► pwm_motor
//                                     │
//                                     └── stall[3:0] / stall_any / ramp_active（调试）
//
//-----------------------------------------------------------------------------
// 为什么要这个模块（两个都会烧硬件）
//-----------------------------------------------------------------------------
//   1) **堵转**：轮子卡住（撞墙/卡缝）时编码器反馈≈0，但闭环会一直加大占空比
//      -> 驱动板（TM6612）持续大电流 -> 过热/烧管子，电池也被拖垮。
//   2) **启动冲击**：从 0 猛地给满占空比，启动电流是稳态的好几倍。
//
//-----------------------------------------------------------------------------
// 每路三个状态
//-----------------------------------------------------------------------------
//   NORMAL   : duty_r 每 CTRL_MS 最多 +RAMP_STEP（上升限速），
//              目标变小/变 0 时**立刻跟下去**（下降不限速，要能立刻减速）
//   STALLED  : 判出堵转 -> duty_r 立刻 0，并保持 COOLDOWN_MS（这期间恒 0）
//   冷却结束 : 回 NORMAL，从 0 重新软启动；卡阻还在就**再判一次**
//              -> 形成"断电 -> 试一下 -> 再断电"的断续试探，避免一直硬顶
//
//-----------------------------------------------------------------------------
// 判据（三条同时满足才算"在使劲但没转"）
//-----------------------------------------------------------------------------
//   duty_r >= STALL_DUTY        力度够大（小占空比本来就可能不转，不能算堵转）
//   |spd_lat| <= STALL_SPD_MAX  编码器几乎不动
//   持续 STALL_MS                避免把"刚起步还没转起来"误判成堵转
//
//   ⚠️ 判据用的是 **duty_r（实际给出的力度）**，不是 duty_target：
//      软启动期间目标可能已经很大，但实际只给出去 16 —— 这时不转是正常的。
//
//   ⚠️ spd_lat 只在 spd_valid（测速窗口更新那拍）锁存。
//      所以"编码器一直没回报"也等价于 spd_lat=0 -> 会被判堵转。
//      这是**保守**选择：没有反馈就当作没转，宁可断电试探。
//
//-----------------------------------------------------------------------------
// 为什么夹在中间，而不是改 motion_decision
//-----------------------------------------------------------------------------
//   1) motion_decision 管"把意图变成目标速度"，堵转保护是**硬件保护**，
//      关注点不同；混在一起以后调闭环会互相绊。
//   2) 这一层**逐轮独立**：某一轮卡住断电，其它三轮照常跑
//      （放在 motion_decision 里很容易顺手把四个轮子一起停）。
//   3) 急停仍是 0 拍：enable_out / duty_safe 都是**组合**的，不过任何寄存器。
//
//-----------------------------------------------------------------------------
// 参数怎么调（上板时看这几个）
//-----------------------------------------------------------------------------
//   CTRL_MS       斜坡/判定的节拍（越小越细腻，堵转判定也更频繁）
//   RAMP_STEP     每 CTRL_MS 允许涨多少。满量程 128（DUTY_GAIN=128）时，
//                 RAMP_STEP=8 + CTRL_MS=1 大约 16ms 从 0 到满 —— 够软也不拖沓
//   STALL_DUTY    "算在使劲"的门槛（低于它的占空比本来就可能转不动）
//   STALL_SPD_MAX "算没转"的门槛（每测速窗口净计数；要和 SPEED_WIN/VREF 一起看）
//   STALL_MS      持续多久才判（太小会把"起步瞬间"误判成堵转）
//   COOLDOWN_MS   断电冷却（太小会让驱动板反复承受短路冲击，太大则脱困太慢）
//
//   ⚠️ STALL_MS / COOLDOWN_MS 会**量化到 CTRL_MS 的整数倍**
//      （内部按节拍数计：STALL_MS/CTRL_MS）。别写 STALL_MS < CTRL_MS。
//
//-----------------------------------------------------------------------------
// 写法说明（为什么只有一个 module）
//-----------------------------------------------------------------------------
//   四路逻辑完全同构，但**不能**拆成一个子模块放进同一个文件：
//   本工程的 tools/rtl_lint.py 按**文件**维护声明表，一个文件里放两个 module
//   会把两边同名的参数/端口（clk/rst_n/enable/...）报成"重复声明"。
//   所以这里用一个 module + generate-for 内联出四路：
//   声明写在 generate 块里（每路一份），逻辑只写一遍。
//=============================================================================
`timescale 1ns/1ps

module motor_guard #(
    //------------------------------------------------------------------
    // 时钟与节拍
    //------------------------------------------------------------------
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer CTRL_MS       = 1,        // 斜坡/判定更新周期 (ms)

    //------------------------------------------------------------------
    // 软启动
    //------------------------------------------------------------------
    parameter integer RAMP_STEP     = 8,        // 每个 CTRL_MS 允许的 duty 最大增量

    //------------------------------------------------------------------
    // 堵转判据
    //------------------------------------------------------------------
    parameter integer STALL_DUTY    = 40,       // duty >= 它才算"在使劲"
    parameter integer STALL_SPD_MAX = 2,        // |编码器速度| <= 它算"没转"
    parameter integer STALL_MS      = 500,      // 持续这么久判堵转
    parameter integer COOLDOWN_MS   = 1000,     // 断电后冷却这么久再放开

    //------------------------------------------------------------------
    // ★ 台架测试开关（2026-10-18 新增）
    //
    //   GUARD_ENABLE = 1（默认，产品配置）: 软启动 + 堵转保护都生效
    //   GUARD_ENABLE = 0（台架配置）      : **只旁路堵转判定**，软启动仍然生效
    //
    //   为什么需要：没接编码器时 spd 恒为 0，闭环会把 duty 顶到限幅，
    //   于是必然被判堵转（转半秒就停）。台架测电机要显式关掉它。
    //   ⚠️ 装车/带载前必须改回 1 —— 那时堵转保护是防止烧驱动板的最后一道闸。
    //------------------------------------------------------------------
    parameter integer GUARD_ENABLE  = 1,

    //------------------------------------------------------------------
    // 参数自检
    //------------------------------------------------------------------
    parameter integer _ASSERT_EN    = 1
) (
    input  wire               clk,
    input  wire               rst_n,

    //---------------- 上游：motion_decision ----------------
    input  wire               enable,      // 0 = 立刻断（急停路径，必须 0 拍生效）
    input  wire [15:0]        duty0,
    input  wire [15:0]        duty1,
    input  wire [15:0]        duty2,
    input  wire [15:0]        duty3,

    //---------------- 编码器反馈（判堵转用）----------------
    input  wire signed [23:0] spd0,        // 每测速窗口的净计数
    input  wire signed [23:0] spd1,
    input  wire signed [23:0] spd2,
    input  wire signed [23:0] spd3,
    input  wire               spd_valid,   // 测速窗口更新脉冲（单拍）

    //---------------- 下游：pwm_motor ----------------
    output wire               enable_out,  // 与 enable 组合直通（0 拍）
    output wire [15:0]        duty_safe0,
    output wire [15:0]        duty_safe1,
    output wire [15:0]        duty_safe2,
    output wire [15:0]        duty_safe3,

    //---------------- 状态（给 LED / telemetry / 调试）----------------
    output reg  [3:0]         stall,       // 每位 1 = 该轮处于堵转断电/冷却中
    output wire               stall_any,   // |stall
    output wire               ramp_active  // 1 = 还有轮子的输出没到目标
);

    //==================================================================
    // 参数换算（全部常数折叠，不占逻辑）
    //
    // ⚠️ 一律"**先除 1000 再乘**"：100ms × 50MHz = 5e9 > 2^31，
    //    直接相乘会在 32 位里回绕（本项目踩过这个坑）。
    // ⚠️ 除数为 0 会在 elaboration 阶段就炸，所以用 *_SAFE 兜一层。
    //==================================================================
    localparam integer CTRL_MS_SAFE  = (CTRL_MS     < 1) ? 1 : CTRL_MS;
    localparam integer STALL_MS_SAFE = (STALL_MS    < 1) ? 1 : STALL_MS;
    localparam integer COOL_MS_SAFE  = (COOLDOWN_MS < 1) ? 1 : COOLDOWN_MS;

    localparam [31:0]  CTRL_CYC      = CTRL_MS_SAFE * (CLK_FREQ_HZ / 1000);

    // 毫秒 -> 节拍数（量化到 CTRL_MS 的整数倍）
    localparam [31:0]  STALL_TICKS_RAW    = STALL_MS_SAFE / CTRL_MS_SAFE;
    localparam [31:0]  COOLDOWN_TICKS_RAW = COOL_MS_SAFE  / CTRL_MS_SAFE;

    // 至少 1 拍（防止 STALL_MS < CTRL_MS 时算出 0，行为不可预期）
    localparam [31:0]  CTRL_CYC_W        = (CTRL_CYC           < 32'd1) ? 32'd1 : CTRL_CYC;
    localparam [31:0]  STALL_TICKS_W     = (STALL_TICKS_RAW    < 32'd1) ? 32'd1 : STALL_TICKS_RAW;
    localparam [31:0]  COOLDOWN_TICKS_W  = (COOLDOWN_TICKS_RAW < 32'd1) ? 32'd1 : COOLDOWN_TICKS_RAW;

    // 16 位 duty 满量程（给参数自检用，避免裸写 65535 触发魔法数字提示）
    localparam [31:0]  DUTY_FULL_W  = 32'd65535;

    //==================================================================
    // CTRL_MS 节拍发生器（全模块只用一个计数器，四路共用）
    //
    //   tick 是**单拍脉冲**：每隔 CTRL_CYC 拍出现一次。
    //   四路同时采这一个 tick，所以斜坡/判定严格同步。
    //==================================================================
    reg [31:0] tick_cnt;
    reg        tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 32'd0;
            tick     <= 1'b0;
        end else if (tick_cnt >= (CTRL_CYC_W - 32'd1)) begin
            tick_cnt <= 32'd0;
            tick     <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 32'd1;
            tick     <= 1'b0;
        end
    end

    //==================================================================
    // 四路打包（用 generate 内联，不复制粘贴四遍）
    //
    //   打包成定宽向量，generate 里用**索引部分选择** `[base +: W]`
    //   （Verilog-2001 语法；base 是 genvar 常量，可以综合）。
    //==================================================================
    wire [63:0] duty_pack;      // 每路 16 位：{duty3,duty2,duty1,duty0}
    wire [95:0] spd_pack;       // 每路 24 位：{spd3,spd2,spd1,spd0}
    wire [63:0] safe_pack;      // 每路 16 位（输出）
    wire [3:0]  stall_w;        // 每路 1 位
    wire [3:0]  ramp_w;         // 每路 1 位

    assign duty_pack = {duty3, duty2, duty1, duty0};
    assign spd_pack  = {spd3, spd2, spd1, spd0};

    //==================================================================
    // 每路：软启动 + 堵转 + 冷却（逻辑只写一遍，generate 展开四份）
    //==================================================================
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_lane

            //------------ 本路状态（每份 generate 各一套）------------
            reg [15:0]        duty_r;      // 斜坡后的占空比（实际给出的力度）
            reg [31:0]        stall_cnt;   // "在使劲但没转"连续了多少个节拍
            reg [31:0]        cool_cnt;    // 冷却还剩多少个节拍
            reg               stalled_r;   // 1 = 堵转断电/冷却中
            reg signed [23:0] spd_lat;     // 锁存的速度（spd_valid 那拍更新）

            //------------ 组合量 ------------
            // ⚠️ 这里刻意写成"先声明、再 assign"，而不是
            //    `wire x = expr;`（net 声明直接带赋值）——
            //    后者放在 generate 块里虽然合法，但少一个不确定因素更好。
            wire [23:0] spd_abs;
            wire        stall_cond;
            wire        stall_now;

            // 绝对值：24 位有符号 -> 24 位无符号
            //   只用于和 STALL_SPD_MAX 比；最负数取绝对值会溢出成 2^23，
            //   但那时数值远大于阈值，"没转"的判定不受影响。
            assign spd_abs    = spd_lat[23] ? (~spd_lat + 24'd1) : spd_lat;
            // 三条同时满足才算"在使劲但没转"
            //
            //   ★ GUARD_ENABLE=0 时**旁路堵转判定**（软启动斜坡仍然生效）。
            //     为什么需要：没有接编码器时 spd 恒为 0，而闭环会把 duty 顶到
            //     限幅（>=STALL_DUTY），于是 500ms 后**必然**被判成堵转，
            //     台架测电机时表现为"转半秒就停、反复抽"。
            //     这不是 bug，是"没有反馈就没法区分堵转"的必然结果 ——
            //     所以台架测试必须显式关掉，别去调 STALL_* 阈值糊弄。
            assign stall_cond = (GUARD_ENABLE != 0) &&
                                (duty_r >= STALL_DUTY) && (spd_abs <= STALL_SPD_MAX);
            // 本拍是否正好到"判堵转"的临界
            assign stall_now  = stall_cond && ((stall_cnt + 32'd1) >= STALL_TICKS_W);

            //------------ 本路主逻辑 ------------
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    spd_lat   <= 24'sd0;
                    duty_r    <= 16'd0;
                    stall_cnt <= 32'd0;
                    cool_cnt  <= 32'd0;
                    stalled_r <= 1'b0;
                end else begin
                    //---- 速度锁存（不受 enable 影响，一直在采）----
                    if (spd_valid) spd_lat <= spd_pack [gi*24 +: 24];

                    if (!enable) begin
                        //---- 急停：内部斜坡归零，重新使能时从 0 软启动 ----
                        //   输出侧的 0 拍清零是组合做的（见下面的 assign），
                        //   这里只清"积分器"，免得解除急停瞬间跳回旧值。
                        duty_r    <= 16'd0;
                        stall_cnt <= 32'd0;
                        // ⚠️ 冷却状态**不清**：时间照样流逝，该冷却完就冷却完。
                    end else if (!stalled_r && (duty_pack [gi*16 +: 16] < duty_r)) begin
                        //=============== 下降：**当拍**跟下去，不等 tick ===============
                        //   ★ 这是安全相关的：避障减速 / 闭环收敛都要求立刻响应。
                        //     曾经把这条写在 tick 分支里 -> 减速最多晚 CTRL_MS(1ms)。
                        //     实测抓到：tb 的占空比测量窗口只有 ~22us，1ms 的 tick
                        //     让"正转后占空比下降"完全看不到变化。
                        //     （真车上就是"该刹车了还多给 1ms 的力"，没必要。）
                        duty_r <= duty_pack [gi*16 +: 16];
                    end else if (tick) begin
                        if (stalled_r) begin
                            //=============== 冷却中 ===============
                            duty_r <= 16'd0;            // 恒 0，不许重新爬
                            if (cool_cnt > 32'd0) cool_cnt <= cool_cnt - 32'd1;
                            if (cool_cnt <= 32'd1) begin
                                // 冷却到点：放开，从 0 重新软启动
                                stalled_r <= 1'b0;
                                stall_cnt <= 32'd0;
                                cool_cnt  <= 32'd0;
                            end
                        end else if (stall_now) begin
                            //=============== 判出堵转：立刻断电 ===============
                            stalled_r <= 1'b1;
                            cool_cnt  <= COOLDOWN_TICKS_W;
                            stall_cnt <= 32'd0;
                            duty_r    <= 16'd0;
                        end else begin
                            //=============== 正常 ===============
                            // 累计/清零堵转计数
                            if (stall_cond) stall_cnt <= stall_cnt + 32'd1;
                            else            stall_cnt <= 32'd0;

                            // 斜坡：上升限速，下降不限速
                            if (duty_pack [gi*16 +: 16] <= duty_r) begin
                                // 目标变小（或相等）：立刻跟下去
                                duty_r <= duty_pack [gi*16 +: 16];
                            end else if ((duty_r + RAMP_STEP) >= duty_pack [gi*16 +: 16]) begin
                                // 这一拍就能到目标：直接对齐（避免越过目标来回抖）
                                duty_r <= duty_pack [gi*16 +: 16];
                            end else begin
                                duty_r <= duty_r + RAMP_STEP;
                            end
                        end
                    end
                end
            end

            //------------ 本路输出 ------------
            // enable 必须**组合**生效（急停 0 拍）
            assign safe_pack [gi*16 +: 16] = enable ? duty_r : 16'd0;
            assign stall_w   [gi]          = stalled_r;
            assign ramp_w    [gi]          = (duty_r != duty_pack [gi*16 +: 16]);

        end
    endgenerate

    //==================================================================
    // 对外输出
    //==================================================================
    // 急停 0 拍：这一根就是 enable 的组合直通，没有任何寄存器
    assign enable_out  = enable;

    assign duty_safe0  = safe_pack[15:0];
    assign duty_safe1  = safe_pack[31:16];
    assign duty_safe2  = safe_pack[47:32];
    assign duty_safe3  = safe_pack[63:48];

    assign ramp_active = |ramp_w;
    assign stall_any   = |stall;

    //------------------------------------------------------------------
    // stall 端口是 reg（集成方给的签名如此），从每路的 wire 组合搬一次
    //
    //   为什么不改成 wire：端口类型是集成方定的，保持 reg 他们不用改连线
    //   （reg / wire 对**例化**没有任何区别，只是模块内部的驱动方式不同）。
    //   这是纯组合块，没有时钟，所以不需要复位分支。
    //------------------------------------------------------------------
    always @(*) begin
        stall = stall_w;
    end

    //==================================================================
    // 参数自检（综合器忽略 initial；仿真里一眼看出参数配错）
    //==================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            $display("[motor_guard] CTRL=%0dms RAMP_STEP=%0d STALL_DUTY=%0d STALL_SPD_MAX=%0d STALL=%0dms COOLDOWN=%0dms",
                     CTRL_MS, RAMP_STEP, STALL_DUTY, STALL_SPD_MAX, STALL_MS, COOLDOWN_MS);
            $display("[motor_guard] CTRL_CYC=%0d 拍, STALL_TICKS=%0d, COOLDOWN_TICKS=%0d",
                     CTRL_CYC_W, STALL_TICKS_W, COOLDOWN_TICKS_W);

            if (CTRL_MS < 1) begin
                $display("[motor_guard] ERROR: CTRL_MS 必须 >= 1（=0 等于每拍都更新）");
            end
            if (RAMP_STEP < 1) begin
                $display("[motor_guard] ERROR: RAMP_STEP 必须 >= 1（=0 时占空比永远爬不上去）");
            end
            if (STALL_DUTY < 1) begin
                $display("[motor_guard] ERROR: STALL_DUTY 必须 >= 1（<=0 等于静止就算堵转）");
            end
            if (STALL_SPD_MAX < 0) begin
                $display("[motor_guard] ERROR: STALL_SPD_MAX 不能为负（它和 |速度| 比）");
            end
            if (STALL_MS < 1) begin
                $display("[motor_guard] ERROR: STALL_MS 必须 >= 1");
            end
            if (COOLDOWN_MS < 1) begin
                $display("[motor_guard] ERROR: COOLDOWN_MS 必须 >= 1");
            end
            if (RAMP_STEP >= DUTY_FULL_W) begin
                $display("[motor_guard] ERROR: RAMP_STEP=%0d 超出 16 位 duty 量程", RAMP_STEP);
            end
            if (STALL_DUTY >= DUTY_FULL_W) begin
                $display("[motor_guard] ERROR: STALL_DUTY=%0d 超出 16 位 duty 量程", STALL_DUTY);
            end
            if ((STALL_MS_SAFE / CTRL_MS_SAFE) < 1) begin
                $display("[motor_guard] ERROR: STALL_MS < CTRL_MS，判堵转会被钳到 1 拍");
            end
            if ((COOL_MS_SAFE / CTRL_MS_SAFE) < 1) begin
                $display("[motor_guard] ERROR: COOLDOWN_MS < CTRL_MS，冷却会被钳到 1 拍");
            end
        end
    end

endmodule
