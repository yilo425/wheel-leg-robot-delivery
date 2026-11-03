//=============================================================================
// module : tof_sr04
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : HC-SR04 超声波测距驱动（4 线模式：VCC / TRIG / ECHO / GND）。
//
//   为什么新写一个而不是改 tof_driver.v：
//     实际采购到的是 HC-SR04（超声波），不是原设计的 VL53L0X（I2C 激光）。
//     两者协议毫无共同点（一个是 I2C 寄存器流，一个是"打一个 10us 脉冲、
//     量回波高电平宽度"），没有可复用的部分。
//     但**对外接口语义必须和 tof_driver 完全一致**，这样 main_fsm / top.v
//     不用改一行：
//       dist_mm[15:0]  最后一次**有效**距离（mm）；无效读数**不覆盖**它
//       dist_valid     单拍脉冲：本次测距有效
//       dist_err       单拍脉冲：超时 / 超范围 / 无回波
//
//-----------------------------------------------------------------------------
// HC-SR04 时序（手册）
//-----------------------------------------------------------------------------
//   TRIG 给一个 >=10us 的高脉冲 -> 模块发 8 个 40kHz 脉冲
//   -> ECHO 拉高，高电平宽度 = 声波往返时间 -> 收到回波后拉低
//     距离(mm) = echo_us * 0.1715      （声速 343m/s，往返各走一趟）
//   测距周期建议 >=60ms：上一发的余波没散完就发下一发会"串扰"，
//   读到的是上一次的假回波（甚至一个很近的假距离）。
//
//-----------------------------------------------------------------------------
// 内部结构（6 个状态，全部是"计数到点就走"，没有开放式等待）
//-----------------------------------------------------------------------------
//
//   rst ──► S_BOOT ──(BOOT_WAIT_MS)──► S_WAIT
//                                        │  ▲
//                                        │  └──── S_EVAL（判定 + 输出单拍）
//                                        ▼            ▲
//                                     S_TRIG ──► S_WAIT_RISE ──► S_MEAS
//                                   (TRIG_US)     (等上升沿)     (量宽度)
//                                                      │             │
//                                                      └──超时──┐    └──超时──┐
//                                                               ▼            ▼
//                                                          S_EVAL（dist_err）
//
//   S_WAIT 里的 period_cnt 从**上一次 trig** 起算，到 SAMPLE_CYC 才发下一次。
//   为什么不是"测完再等 SAMPLE_MS"：
//     echo 本身最长 24ms（4m 往返）。若测完再等 60ms，trig 到 trig 就变成
//     84ms —— 周期偏 40%，还会撞上回波串扰。本模块测完只补等剩余时间，
//     所以 trig 间隔恒等于 SAMPLE_MS；测量超时也不会跑飞（period_cnt 已经
//     超过 SAMPLE_CYC，只是紧接着再发一次）。
//
//-----------------------------------------------------------------------------
// 测宽（怎么保证 tick 数就是 echo 的高电平拍数）
//-----------------------------------------------------------------------------
//   echo 是异步输入：先过 2 级同步器（echo_meta/echo_sync），再用 echo_prev
//   做边沿检测。检测到上升沿的那一拍把 echo_ticks 置 1，之后每拍 +1，
//   直到看到 echo_sync=0 就停。
//   因为**上升沿和下降沿经过同一组同步器、延迟一样**，数出来的拍数就等于
//   引脚上的高电平拍数（不多也不少一拍）。1 拍 = 20ns = 0.0034mm，可忽略。
//
//-----------------------------------------------------------------------------
// 换算（不用除法器：只有常数乘 + 移位）
//-----------------------------------------------------------------------------
//   距离(mm) = echo_ticks / CLK_FREQ_HZ * 171500
//   mm = (echo_ticks * DIST_K) >> 17
//
//   DIST_K = round(171500 * 2^17 / CLK_FREQ_HZ)
//          = round(224788480 / (CLK_FREQ_HZ/100))     ← 先除后乘，防 32 位溢出
//
//   50MHz 时 DIST_K = round(224788480/500000) = 450，
//   与需求给的 (ticks*450)>>17 **完全一致**（3000mm 处偏 2mm，0.07%）。
//
//   ★ 与需求文档的一处出入（如实记录，不是笔误）：
//     需求写 "K = 450 * CLK_FREQ_HZ / 50_000_000"，这个方向是反的 ——
//     时钟越慢，1us 里的拍数越少，K 必须**越大**（K 与 CLK 成反比）。
//     按那个式子，25MHz 会算出 K=225，读数只有真实值的一半。
//     本模块按 K ∝ 1/CLK 实现，50MHz 下结果与需求的 450 完全相同，
//     所以默认配置下行为没有任何差别；只有改 CLK_FREQ_HZ 时才有区别。
//
//   溢出检查（乘积上界与时钟无关，改 CLK 不会溢出）：
//     echo_ticks <= TMO_CYC+1，DIST_K ≈ 450*50e6/CLK
//     => ticks*K ≈ (TMO_MS*CLK/1000)*(450*50e6/CLK) = TMO_MS*2.25e7
//     默认 TMO_MS=40 => 9.0e8 < 2^31 ✓
//
//-----------------------------------------------------------------------------
// 无效读数：dist_mm **保持上一次有效值**
//-----------------------------------------------------------------------------
//   与 tof_driver 一致，也符合安全策略：0 会被 main_fsm 当成"正前方 0mm
//   有障碍"直接急停。把上次的好值留着、同时给 dist_err，上层可以走"限速"。
//   （见 doc/interface_spec.md B3。）
//
//-----------------------------------------------------------------------------
// 所有等待都有超时
//-----------------------------------------------------------------------------
//   BOOT      ：计数到 BOOT_CYC 就往下走（本身有界）
//   等上升沿  ：TMO_MS 内没等到 -> dist_err
//   量宽度    ：高电平已持续 TMO_MS 还不落 -> dist_err
//   采样周期  ：计数到 SAMPLE_CYC 就发下一次（本身有界）
//   （4m 往返约 24ms，TMO_MS 默认 40ms，留了 60% 余量。）
//=============================================================================

`timescale 1ns/1ps

module tof_sr04 #(
    //----------------------------------------------------------------
    // 时钟与测量周期
    //----------------------------------------------------------------
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer SAMPLE_MS    = 60,      // 测量周期（HC-SR04 建议 >=60ms，防串扰）

    //----------------------------------------------------------------
    // 超时
    //   同一个 TMO_MS 既管"等 echo 上升沿"，也管"echo 高电平持续多久"。
    //   4m 往返约 24ms，默认 40ms 留 60% 余量。
    //----------------------------------------------------------------
    parameter integer TMO_MS       = 40,

    //----------------------------------------------------------------
    // TRIG 脉冲宽度（手册要求 >=10us）
    //----------------------------------------------------------------
    parameter integer TRIG_US      = 10,

    //----------------------------------------------------------------
    // 量程 / 盲区（mm）
    //   < MIN_MM 或 > MAX_MM 判为无效读数 -> dist_err
    //   ★ 两个参数都必须 >= 0（比较是按无符号做的）
    //----------------------------------------------------------------
    parameter integer MIN_MM       = 20,
    parameter integer MAX_MM       = 4000,

    //----------------------------------------------------------------
    // 上电等待（模块内部 MCU 起来 + 第一次测量稳定）
    //----------------------------------------------------------------
    parameter integer BOOT_WAIT_MS = 50,

    //----------------------------------------------------------------
    // 参数自检开关（只影响 initial 里的 $display，不进逻辑）
    //----------------------------------------------------------------
    parameter integer _ASSERT_EN   = 1
) (
    input  wire        clk,
    input  wire        rst_n,       // 低有效，异步复位

    //---------------- 到模块 ----------------
    output reg         trig,        // TRIG_US 高脉冲（空闲为低）
    input  wire        echo,        // 高电平宽度 = 往返时间（异步输入）

    //---------------- 给上层（语义与 tof_driver 一致）----------------
    output reg  [15:0] dist_mm,     // 最后一次**有效**距离
    output reg         dist_valid,  // 单拍脉冲：本次测距有效
    output reg         dist_err,    // 单拍脉冲：超时/超范围/无回波

    //---------------- 统计（调试用，饱和不回绕）----------------
    output reg  [15:0] ok_cnt,
    output reg  [15:0] err_cnt
);

    //================================================================
    // 1) 时间常数
    //
    //   ⚠️ 一律"先除后乘"，避免 ms * CLK_FREQ_HZ 溢出 32 位：
    //      60ms * 50MHz = 3.0e9 > 2^31（本工程踩过 4 次这个坑）。
    //      MS_CYC 先除出"1ms 几拍"，再乘毫秒数。
    //================================================================
    localparam [31:0] MS_CYC = CLK_FREQ_HZ / 1000;

    // 1us 几拍。CLK < 1MHz 时除法会得 0，兜底成 1（否则 TRIG_CYC=0）
    localparam [31:0] US_CYC = (CLK_FREQ_HZ / 1000000 < 1) ? 32'd1 :
                               (CLK_FREQ_HZ / 1000000);

    localparam [31:0] SAMPLE_CYC = ((SAMPLE_MS    < 1) ? 32'd1 : SAMPLE_MS)    * MS_CYC;
    localparam [31:0] TMO_CYC    = ((TMO_MS       < 1) ? 32'd1 : TMO_MS)       * MS_CYC;
    localparam [31:0] BOOT_CYC   = ((BOOT_WAIT_MS < 1) ? 32'd1 : BOOT_WAIT_MS) * MS_CYC;
    localparam [31:0] TRIG_CYC   = ((TRIG_US      < 1) ? 32'd1 : TRIG_US)      * US_CYC;

    // trig 保持高电平的最后一拍（用 TRIG_CYC 计，不用 TRIG_CYC-1 字面量）
    localparam [31:0] TRIG_LAST  = TRIG_CYC - 32'd1;

    //----------------------------------------------------------------
    // 换算系数 DIST_K（常数表达式，综合时折叠成常数，不是运行时除法）
    //
    //   DIST_K = round(171500 * 2^17 / CLK) = round(224788480 / (CLK/100))
    //   先给分子加半个分母做四舍五入；CLK<100Hz 的荒谬情况兜底成 1，
    //   避免 elaboration 阶段除零。
    //----------------------------------------------------------------
    localparam [31:0] DIST_K_DEN = (CLK_FREQ_HZ >= 400) ? (CLK_FREQ_HZ / 100) :
                                   32'd4;
    localparam [31:0] DIST_K     = (32'd224_788_480 + (DIST_K_DEN / 32'd2)) /
                                   DIST_K_DEN;

    // 声速：343 m/s == 343 mm/ms（自检里算"超时够不够量程"用）
    localparam integer SPEED_MM_PER_MS = 343;

    //================================================================
    // 2) 状态定义
    //================================================================
    localparam [2:0]
        S_BOOT      = 3'd0,   // 上电等待
        S_WAIT      = 3'd1,   // 等采样周期（trig 间隔 = SAMPLE_MS）
        S_TRIG      = 3'd2,   // 拉高 TRIG，保持 TRIG_US
        S_WAIT_RISE = 3'd3,   // 等 ECHO 上升沿（带超时）
        S_MEAS      = 3'd4,   // 量 ECHO 高电平宽度（带超时）
        S_EVAL      = 3'd5;   // 换算 + 范围判断 + 输出单拍脉冲

    //================================================================
    // 3) 寄存器 / 线网（全部先声明后使用，Verilog-2001）
    //================================================================
    reg [2:0]  state;

    reg [31:0] boot_cnt;      // S_BOOT 计数
    reg [31:0] period_cnt;    // 从 trig 起算的周期计数（一直跑）
    reg [31:0] trig_cnt;      // TRIG 高电平计数
    reg [31:0] tmo_cnt;       // 等上升沿的超时计数
    reg [31:0] echo_ticks;    // ECHO 高电平拍数 = 往返时间
    reg        meas_err;      // 本次测量是否已判失败（超时）

    // ECHO 同步器 + 边沿检测
    reg        echo_meta;     // 第 1 级
    reg        echo_sync;     // 第 2 级（当作"干净"的 echo）
    reg        echo_prev;     // 再打一拍，做上升沿检测

    wire       echo_high = echo_sync;
    wire       echo_rise = echo_sync & ~echo_prev;

    // 换算结果（32 位，后面范围检查也在 32 位上做，避免截断后误判）
    wire [31:0] mm_calc = (echo_ticks * DIST_K) >> 17;

    //================================================================
    // 4) ECHO 同步 + 边沿检测
    //    异步输入必须过 2 级同步器再进状态机（否则亚稳态会让边沿检测
    //    抖出毛刺，表现为"莫名其妙多测/少测几个 tick"）。
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            echo_meta <= 1'b0;
            echo_sync <= 1'b0;
            echo_prev <= 1'b0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
            echo_prev <= echo_sync;
        end
    end

    //================================================================
    // 5) 周期计数：从 trig 起算，数到 SAMPLE_CYC 就允许下一次触发
    //
    //   ★ 这里**不**在 S_EVAL 之后清零，而是"真正发 trig 的那一拍"清零，
    //     这样才能保证 trig 到 trig 恰好 SAMPLE_CYC（见文件头说明）。
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt <= 32'd0;
        end else if ((state == S_WAIT) && (period_cnt >= SAMPLE_CYC)) begin
            period_cnt <= 32'd0;
        end else begin
            period_cnt <= period_cnt + 32'd1;
        end
    end

    //================================================================
    // 6) 主状态机
    //
    //   每个状态都是"计数到点就走"；两个等待（等上升沿、量宽度）都用
    //   TMO_CYC 兜底，不会永久卡住。
    //   两个输出单拍脉冲在块首默认清零，只有 S_EVAL 会置起其中一个，
    //   所以**不可能同一拍同时为 1**，也必然是单拍。
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_BOOT;
            boot_cnt   <= 32'd0;
            trig_cnt   <= 32'd0;
            tmo_cnt    <= 32'd0;
            echo_ticks <= 32'd0;
            meas_err   <= 1'b0;
            trig       <= 1'b0;
            dist_mm    <= 16'd0;
            dist_valid <= 1'b0;
            dist_err   <= 1'b0;
            ok_cnt     <= 16'd0;
            err_cnt    <= 16'd0;
        end else begin
            //---- 两个输出是单拍脉冲：默认清零 ----
            dist_valid <= 1'b0;
            dist_err   <= 1'b0;

            case (state)

            //--------------------------------------------------
            // 上电等待：这段时间模块可能还没稳定，不要发 trig
            //--------------------------------------------------
            S_BOOT: begin
                trig <= 1'b0;
                if (boot_cnt >= BOOT_CYC) begin
                    boot_cnt <= 32'd0;
                    state    <= S_WAIT;
                end else begin
                    boot_cnt <= boot_cnt + 32'd1;
                end
            end

            //--------------------------------------------------
            // 等采样周期
            //--------------------------------------------------
            S_WAIT: begin
                trig <= 1'b0;
                if (period_cnt >= SAMPLE_CYC) begin
                    // 新一次测量：清掉上次的中间量
                    trig       <= 1'b1;
                    trig_cnt   <= 32'd0;
                    tmo_cnt    <= 32'd0;
                    echo_ticks <= 32'd0;
                    meas_err   <= 1'b0;
                    state      <= S_TRIG;
                end
            end

            //--------------------------------------------------
            // TRIG 高脉冲：保持 TRIG_CYC 拍（默认 500 拍 = 10us）
            //--------------------------------------------------
            S_TRIG: begin
                trig <= 1'b1;
                if (trig_cnt >= TRIG_LAST) begin
                    trig     <= 1'b0;
                    trig_cnt <= 32'd0;
                    tmo_cnt  <= 32'd0;
                    state    <= S_WAIT_RISE;
                end else begin
                    trig_cnt <= trig_cnt + 32'd1;
                end
            end

            //--------------------------------------------------
            // 等 ECHO 上升沿
            //   注意：如果 echo 在 trig 之前就已经是高（模块坏了 / 线短到地），
            //   这里永远看不到上升沿 -> TMO_MS 后超时，不会卡死。
            //--------------------------------------------------
            S_WAIT_RISE: begin
                if (echo_rise) begin
                    // 检测到上升沿的这一拍就算第 1 拍，保证 tick 数不多不少
                    echo_ticks <= 32'd1;
                    tmo_cnt    <= 32'd0;
                    state      <= S_MEAS;
                end else if (tmo_cnt >= TMO_CYC) begin
                    meas_err <= 1'b1;
                    tmo_cnt  <= 32'd0;
                    state    <= S_EVAL;
                end else begin
                    tmo_cnt <= tmo_cnt + 32'd1;
                end
            end

            //--------------------------------------------------
            // 量 ECHO 高电平宽度
            //   echo 落下来就结束；一直不落就等到 TMO_MS 判超时。
            //--------------------------------------------------
            S_MEAS: begin
                if (!echo_high) begin
                    state <= S_EVAL;
                end else if (echo_ticks >= TMO_CYC) begin
                    meas_err <= 1'b1;
                    state    <= S_EVAL;
                end else begin
                    echo_ticks <= echo_ticks + 32'd1;
                end
            end

            //--------------------------------------------------
            // 判定 + 输出单拍脉冲
            //   无效读数**不写** dist_mm（保持上一次有效值）。
            //--------------------------------------------------
            S_EVAL: begin
                state    <= S_WAIT;
                meas_err <= 1'b0;
                if (meas_err) begin
                    // 等上升沿 / 量宽度超时
                    dist_err <= 1'b1;
                    if (err_cnt != 16'hFFFF) err_cnt <= err_cnt + 16'd1;
                end else if ((mm_calc < MIN_MM) || (mm_calc > MAX_MM)) begin
                    // 盲区内 / 超量程
                    dist_err <= 1'b1;
                    if (err_cnt != 16'hFFFF) err_cnt <= err_cnt + 16'd1;
                end else begin
                    dist_mm    <= mm_calc[15:0];
                    dist_valid <= 1'b1;
                    if (ok_cnt != 16'hFFFF) ok_cnt <= ok_cnt + 16'd1;
                end
            end

            default: begin
                state <= S_BOOT;
                trig  <= 1'b0;
            end
            endcase
        end
    end

    //================================================================
    // 7) 参数自检（只打印，不改逻辑）
    //
    //   这里能查的是"参数之间是否自相矛盾"：
    //     - MAX_MM 必须 > MIN_MM
    //     - TMO_MS 必须够 4m 往返（否则最远距离永远读不出来）
    //     - TRIG_US 必须 >=10us（手册硬要求）
    //   真正的时序由 sim/tb_sr04_driver.v 钉住。
    //================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            $display("[tof_sr04] CLK=%0dHz SAMPLE=%0dms TMO=%0dms TRIG=%0dus MIN=%0dmm MAX=%0dmm BOOT=%0dms",
                     CLK_FREQ_HZ, SAMPLE_MS, TMO_MS, TRIG_US, MIN_MM, MAX_MM,
                     BOOT_WAIT_MS);
            $display("[tof_sr04] 拍数: SAMPLE=%0d TMO=%0d TRIG=%0d ; 换算 DIST_K=%0d (mm = ticks*K>>17)",
                     SAMPLE_CYC, TMO_CYC, TRIG_CYC, DIST_K);

            if (MIN_MM < 0)
                $display("[tof_sr04] ERROR: MIN_MM(%0d) 必须 >= 0", MIN_MM);
            if (MAX_MM <= MIN_MM)
                $display("[tof_sr04] ERROR: MAX_MM(%0d) 必须 > MIN_MM(%0d)",
                         MAX_MM, MIN_MM);
            if (TRIG_US < 10)
                $display("[tof_sr04] WARN: TRIG_US=%0d < 10us（HC-SR04 手册要求 >=10us）",
                         TRIG_US);
            // 往返时间(ms) = 2*MAX_MM/343；要求 TMO_MS >= 它
            if ((TMO_MS * SPEED_MM_PER_MS) < (2 * MAX_MM))
                $display("[tof_sr04] WARN: TMO_MS=%0dms 不够 MAX_MM=%0dmm 的往返(约 %0dms)",
                         TMO_MS, MAX_MM, (2 * MAX_MM) / SPEED_MM_PER_MS);
            if (DIST_K < 1)
                $display("[tof_sr04] ERROR: DIST_K=0（CLK_FREQ_HZ=%0dHz 太慢？）",
                         CLK_FREQ_HZ);
        end
    end

endmodule
