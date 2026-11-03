//=============================================================================
// module : pwm_motor
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : 单路直流电机 PWM 驱动, 带方向输出、使能安全门与零点死区补偿。
//
// 设计要点:
//   1) 载波计数 0..PERIOD_CNT-1, 20kHz @100MHz -> PERIOD_CNT=5000
//   2) duty_in 为有符号归一化占空比, 但**方向由 dir_in 决定**:
//          满幅 = PERIOD_CNT*DUTY_GAIN
//      默认 PERIOD_CNT=5000 / DUTY_GAIN=128 -> duty_in=64 即 50% 占空比,
//      上层 PI 调速全整数运算, 不需要浮点。
//      保留 duty_in 的 signed 类型是为了兼容 "负数也当幅值" 的旧写法,
//      但正负号不再影响 dir_out —— 本模块只出 **1 根方向脚**，
//      第二根（AIN2/BIN2/CIN2/DIN2）由顶层 m_dir2 = ~m_dir 取反生成。
//      （早期用的是"单方向脚 + EN"的驱动板，所以这里只保留一根；
//        现在换 TB6612 四路板，靠顶层补第二根，本模块不用改。）
//   3) 有符号数只出现在组合块的局部 wire 上, 顺序块里一律拆成
//      "幅值 + 方向" 两个无符号寄存器, 避免 Verilog 有符号扩展的坑。
//   4) enabled=0 时强制 duty=0 (滑行停车), 是否短接制动由驱动板 EN/BRK 脚决定。
//   5) ZERO_DEADBAND: 小于该幅值的指令直接输出 0。
//      作用: 抵消驱动板 MOSFET 上升/下降沿不对称导致的 "零指令仍微转";
//      上板后从 0 开始逐步调大, 直到电机完全静止, 该值写回顶层 parameter。
//   6) dir_out 仅在指令幅值非零时更新, duty=0 时保持上次方向,
//      避免电机在 0 附近反复换向打齿。
//=============================================================================
`timescale 1ns/1ps

module pwm_motor #(
    parameter integer CLK_FREQ_HZ   = 100_000_000, // 系统时钟频率
    parameter integer PWM_FREQ_HZ   = 20_000,      // PWM 载波频率
    parameter integer DUTY_GAIN     = 128,         // duty_in 满量程缩放系数
    parameter integer CNT_WIDTH     = 16,          // 载波计数器位宽
    parameter integer ZERO_DEADBAND = 0            // 零点死区(归一化幅值)
) (
    input  wire                 clk,
    input  wire                 rst_n,      // 低有效
    input  wire                 enabled,    // 输出使能, 0=强制停机
    input  wire signed [15:0]   duty_in,    // 有符号归一化占空比
    input  wire                 dir_in,     // 方向指令: 1=正转 0=反转
    output wire                 pwm_out,    // 到驱动板的 PWM 脚
    output wire                 dir_out,    // 方向脚: 1=正转 0=反转
    output wire [CNT_WIDTH-1:0] pwm_cnt,    // 载波计数 (调试/同步观察)
    output wire signed [15:0]   duty_actual // 实际生效的有符号占空比(回读)
);

    //------------------------------------------------------------------
    // 参数推导
    //------------------------------------------------------------------
    localparam integer PERIOD_CNT  = CLK_FREQ_HZ / PWM_FREQ_HZ;  // 载波周期(clk)
    localparam integer STEP_DIV    = PERIOD_CNT / DUTY_GAIN;     // 每单位 duty 的 clk 数
    localparam integer DUTY_FULL   = PERIOD_CNT / STEP_DIV;      // duty_in 满量程

    // 定点常量(显式定宽, 便于在表达式里直接使用)
    localparam [15:0] STEP_DIV_16    = STEP_DIV[15:0];
    localparam [15:0] DUTY_FULL_16   = DUTY_FULL[15:0];
    localparam [15:0] ZERO_DEAD_16   = ZERO_DEADBAND[15:0];

    initial begin
        if (PWM_FREQ_HZ <= 0 || PERIOD_CNT < 4) begin
            $display("[pwm_motor] FATAL: PWM_FREQ_HZ=%0d 相对 CLK_FREQ_HZ=%0d 过高",
                     PWM_FREQ_HZ, CLK_FREQ_HZ);
        end
        // duty_in 为 16 位有符号, 满量程不能超过它
        if (STEP_DIV < 1 || DUTY_FULL > 16'sd32767) begin
            $display("[pwm_motor] FATAL: 参数组合非法 PERIOD_CNT=%0d DUTY_GAIN=%0d",
                     PERIOD_CNT, DUTY_GAIN);
        end
    end

    //------------------------------------------------------------------
    // 载波计数器
    //------------------------------------------------------------------
    reg [CNT_WIDTH-1:0] cnt_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_r <= {CNT_WIDTH{1'b0}};
        end else if (cnt_r >= PERIOD_CNT[CNT_WIDTH-1:0] - 1'b1) begin
            cnt_r <= {CNT_WIDTH{1'b0}};
        end else begin
            cnt_r <= cnt_r + 1'b1;
        end
    end

    assign pwm_cnt = cnt_r;

    //------------------------------------------------------------------
    // 指令解算 (组合): 有符号 duty_in -> 无符号幅值
    //   方向不再由 duty_in 的符号决定, 而是由上层给的 dir_in 决定。
    //   方向不靠 duty_in 的符号表达: 本模块只出 **1 根方向脚**,
    //   第二根由顶层取反（m_dir2 = ~m_dir）—— 占空比符号无法表达"倒车",
    //   若把负 duty 直接当反向 PWM, 电机只会减速不会反转。
    //   因此上层要做的是: 幅值(恒 >=0) 给 duty_in, 方向给 dir_in。
    //   duty_actual 仍按 (dir_in ? + : -) 回读, 便于上层看方向。
    //------------------------------------------------------------------
    reg [15:0] mag_want;

    always @(*) begin
        if (!enabled) begin
            mag_want = 16'd0;
        end else if (duty_in > 0) begin
            mag_want = duty_in;
        end else if (duty_in < 0) begin
            mag_want = (~duty_in) + 16'd1;         // 兼容旧用法: 负值也当幅值用
        end else begin
            mag_want = 16'd0;
        end

        // 幅值钳位
        if (mag_want > DUTY_FULL_16) begin
            mag_want = DUTY_FULL_16;
        end

        // 零点死区
        if (mag_want <= ZERO_DEAD_16) begin
            mag_want = 16'd0;
        end
    end

    reg        dir_r;
    reg [15:0] mag_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_r <= 1'b0;
            mag_r <= 16'd0;
        end else begin
            if (mag_want != 16'd0 || !enabled) begin
                dir_r <= dir_in;                   // 无输出时不换向
            end
            mag_r <= mag_want;
        end
    end

    assign dir_out = dir_r;

    //------------------------------------------------------------------
    // 比较值 = 幅值 * STEP_DIV, 打一拍后供载波比较
    //   1) 中间积用 32 位保存, 避免 mag*STEP_DIV 在 16 位里溢出被静默截断;
    //      只有送进比较器时才取低 CNT_WIDTH 位。
    //   2) 满量程时钳到 PERIOD_CNT, 保证 100% 占空比可达 (否则只能到
    //      DUTY_FULL*STEP_DIV, 例如 5000/39*39=4992, 永远差几点)。
    //------------------------------------------------------------------
    reg [31:0] cmp_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmp_r <= 32'd0;
        end else if (mag_r >= DUTY_FULL_16) begin
            cmp_r <= PERIOD_CNT[31:0];      // 100% 饱和
        end else begin
            cmp_r <= mag_r * STEP_DIV_16;
        end
    end

    //------------------------------------------------------------------
    // PWM 输出: 高有效
    //------------------------------------------------------------------
    assign pwm_out = (cnt_r < cmp_r[CNT_WIDTH-1:0]) ? 1'b1 : 1'b0;

    //------------------------------------------------------------------
    // 回读实际生效占空比: 直接上报生效幅值, 不再反算(避免整除截断误差)
    //------------------------------------------------------------------
    assign duty_actual = dir_r ? $signed(mag_r) : -$signed(mag_r);

endmodule
