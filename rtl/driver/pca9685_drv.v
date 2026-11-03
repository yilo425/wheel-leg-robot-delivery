//=============================================================================
// module : pca9685_drv
// brief  : PCA9685 16 路 PWM 舵机驱动板 —— FPGA 侧驱动
//
//-----------------------------------------------------------------------------
// 为什么要这个模块
//-----------------------------------------------------------------------------
//   原来舵机 PWM 由 servo_lift 直接生成（硬件 PWM，最省事）。
//   但接线时有个实际问题：**一个 FPGA 引脚要带 2 个舵机的信号输入**，
//   而 GPIO0 是单孔母座，插不进两根线 —— 必须加 Y 线或分线板。
//
//   PCA9685 是一块现成的 16 路舵机驱动板（¥10 左右，免焊），
//   4 个舵机直接插、电源走螺丝端子 —— 正好解决接线问题。
//   代价：舵机不再由 FPGA 直接发 PWM，而是 FPGA 通过 I2C 告诉 PCA9685 角度。
//
//-----------------------------------------------------------------------------
// 和 i2c_master 的分工（重要）
//-----------------------------------------------------------------------------
//   本模块**只做 FSM**，**不**例化 i2c_master。这样 testbench 可以用一个
//   "命令记录器"替换 i2c_master，直接验证**寄存器序列和数值**；
//   I2C 的时序正确性已由 i2c_master 自己的 tb 覆盖，不必重复验证。
//
//   上层（top.v）负责把命令信号接到 i2c_master：
//       pca9685_drv.start/.dev_addr/.wr_data/.reg_addr/.reg_addr_en
//                              -> i2c_master 同名端口
//       i2c_master.done/.ack_err -> pca9685_drv.done/.ack_err
//
//-----------------------------------------------------------------------------
// 初始化序列（PCA9685 datasheet 要求）
//-----------------------------------------------------------------------------
//   1) MODE1(0x00)    = 0x10   SLEEP=1（必须先睡才能写 PRESCALE）
//   2) PRE_SCALE(0xFE)= 121
//         内部振荡器 25MHz：prescale = round(25e6/(4096*50)) - 1 = 121
//   3) MODE2(0x01)    = 0x04   OUTDRV=1（推挽输出）
//   4) MODE1(0x00)    = 0x20   AI=1（自动递增）+ 唤醒
//   5) 等 1ms（振荡器起振），然后 init_done=1
//
//-----------------------------------------------------------------------------
// 舵机计数值
//-----------------------------------------------------------------------------
//   50Hz -> 周期 20ms -> 4096 计数 = 20ms -> 1 计数 ≈ 4.88us
//     0.5ms -> 102 计数（放下）      2.5ms -> 512 计数（抬起）
//
//   高度 0..100 用 **count = 102 + height*4**：
//     height=0   -> 102 (0.5ms)   ✓
//     height=100 -> 502 (2.45ms)  —— 比 2.5ms 短 2%（约 1.8°），
//        反而更安全：不会顶着舵机机械极限。
//   ⚠️ 刻意用 ×4（移位）而不是 ×4.1（要除法器）：省一个除法器，
//      换 2% 行程误差，非常划算。
//
//-----------------------------------------------------------------------------
// 4 个通道怎么分
//-----------------------------------------------------------------------------
//   通道 0、1 = 前腿两个舵机  -> 都听 lift_cmd[0]
//   通道 2、3 = 后腿两个舵机  -> 都听 lift_cmd[1]
//   "同组两个舵机同步" 就是两个通道写同一个计数值。
//
//-----------------------------------------------------------------------------
// 为什么"只在变化时写"
//-----------------------------------------------------------------------------
//   PCA9685 会**自己保持**写进去的 PWM 值，不需要周期性刷新。
//   一轮全 4 通道更新 = 16 次单字节 I2C 传输 ≈ 5ms @100kHz。
//   每 20ms 都重写会把 I2C 占满；只在变化时写，开销可忽略。
//=============================================================================
`timescale 1ns/1ps

module pca9685_drv #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer PWM_HZ      = 50,
    parameter [6:0]   DEV_ADDR    = 7'h40,

    // count = COUNT_MIN + height * COUNT_PER_H
    parameter integer COUNT_MIN   = 102,
    parameter integer COUNT_PER_H = 4,

    parameter integer POWERUP_MS  = 2,   // 上电等待
    parameter integer OSC_MS      = 1,   // 振荡器起振等待

    // ★ 2026-11-02：通道 2（后腿左）独立微调（计数，有符号）
    //   10° ≈ 102*10/45 ≈ 22.7 -> 23
    parameter integer REAR_L_TRIM = 0,
    parameter integer _ASSERT_EN  = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    //---------------- 上层意图 ----------------
    input  wire [1:0]  lift_cmd,       // {后腿, 前腿} 1=抬起
    input  wire [7:0]  lift_height,    // 0..100（CONFIG 帧 0x10）
    // ★ 2026-10-30：站立高度（遥控升高/降低），由 stance_ctrl 产生
    //   抬腿动作（lift_cmd != 0）优先，抬腿结束后回到站立高度
    input  wire [11:0] stance_front,   // 通道 0/1 计数值
    input  wire [11:0] stance_rear,    // 通道 2/3 计数值

    //---------------- i2c_master 命令接口 ----------------
    output reg         start,          // 单拍脉冲
    output wire [6:0]  dev_addr,
    output reg  [7:0]  wr_data,
    output reg         reg_addr_en,    // 恒 1
    output reg  [7:0]  reg_addr,
    input  wire        busy,           // 1=i2c_master 正在传输（start 前必须等它落）
    input  wire        done,           // 单拍脉冲
    input  wire        ack_err,        // done=1 时有效

    //---------------- 状态 ----------------
    output reg         init_done,
    output reg         err_sticky      // 曾经 NACK/超时（粘滞）
);

    //==================================================================
    // 常量
    //==================================================================
    localparam [7:0] REG_MODE1    = 8'h00;
    localparam [7:0] REG_MODE2    = 8'h01;
    localparam [7:0] REG_LED0_ON_L= 8'h06;
    localparam [7:0] REG_PRESCALE = 8'hFE;

    localparam [7:0] MODE1_SLEEP  = 8'h10;
    localparam [7:0] MODE1_WAKE   = 8'h20;
    localparam [7:0] MODE2_VAL    = 8'h04;

    localparam integer PRESCALE_I = (25_000_000 / (4096 * PWM_HZ)) - 1;
    localparam [7:0]   PRESCALE   = PRESCALE_I;

    localparam [31:0] MS_CYC      = CLK_FREQ_HZ / 1000;
    localparam [31:0] POWERUP_CYC = POWERUP_MS * MS_CYC;
    localparam [31:0] OSC_CYC     = OSC_MS     * MS_CYC;

    localparam [7:0]  H_MAX = 8'd100;

    // 初始化步骤编号（0..3）
    localparam [1:0] IS_SLEEP  = 2'd0;
    localparam [1:0] IS_PRESC  = 2'd1;
    localparam [1:0] IS_MODE2  = 2'd2;
    localparam [1:0] IS_WAKE   = 2'd3;

    // 状态
    localparam [3:0] S_POWERUP  = 4'd0;
    localparam [3:0] S_INIT     = 4'd1;   // 发初始化命令（由 init_step 选是哪条）
    localparam [3:0] S_OSC_WAIT = 4'd2;
    localparam [3:0] S_IDLE     = 4'd3;
    localparam [3:0] S_ISSUE    = 4'd4;   // 发角度命令
    localparam [3:0] S_WAIT     = 4'd5;   // 等 done

    reg [3:0]  st;
    reg [31:0] wait_cnt;
    reg        in_init;      // 1 = 还在初始化阶段
    reg [1:0]  init_step;

    reg [1:0]  wr_ch;        // 0..3
    reg [1:0]  wr_byte;      // 0..3

    reg [11:0] last_front;
    reg [11:0] last_rear;

    //==================================================================
    // 组合：计数换算
    //==================================================================
    wire [7:0] h_cl = (lift_height > H_MAX) ? H_MAX : lift_height;

    wire [11:0] lift_front = COUNT_MIN + (lift_cmd[0] ? (h_cl * COUNT_PER_H) : 0);
    wire [11:0] lift_rear  = COUNT_MIN + (lift_cmd[1] ? (h_cl * COUNT_PER_H) : 0);

    // ★ 抬腿（视觉触发）优先；没有抬腿时按站立高度走
    wire [11:0] cnt_front = (lift_cmd == 2'b00) ? stance_front : lift_front;
    wire [11:0] cnt_rear  = (lift_cmd == 2'b00) ? stance_rear  : lift_rear;

    // ★ 通道 2（左后腿）单独加微调
    wire signed [12:0] rear_l_s   = $signed({1'b0, cnt_rear}) + REAR_L_TRIM;
    wire        [11:0] cnt_rear_l = rear_l_s[11:0];
    wire [11:0] cnt_cur = (wr_ch == 2'd2) ? cnt_rear_l :
                          (wr_ch[1])      ? cnt_rear   : cnt_front;

    //==================================================================
    // 组合：本轮要写的寄存器地址 / 数据
    //   初始化阶段由 init_step 选，数据阶段由 wr_ch/wr_byte 选
    //==================================================================
    reg [7:0] addr_i;   // 初始化地址
    reg [7:0] data_i;   // 初始化数据
    always @(*) begin
        case (init_step)
            IS_SLEEP: begin addr_i = REG_MODE1;    data_i = MODE1_SLEEP; end
            IS_PRESC: begin addr_i = REG_PRESCALE; data_i = PRESCALE;    end
            IS_MODE2: begin addr_i = REG_MODE2;    data_i = MODE2_VAL;   end
            default:  begin addr_i = REG_MODE1;    data_i = MODE1_WAKE;  end
        endcase
    end

    // 数据阶段：byte0/1 = ON（0），byte2/3 = OFF 低/高
    reg [7:0] data_d;
    always @(*) begin
        case (wr_byte)
            2'd0:    data_d = 8'h00;
            2'd1:    data_d = 8'h00;
            2'd2:    data_d = cnt_cur[7:0];
            default: data_d = {4'h0, cnt_cur[11:8]};
        endcase
    end

    wire [7:0] addr_d = REG_LED0_ON_L + {wr_ch, 2'b00} + {6'd0, wr_byte};

    //==================================================================
    // 目标状态比较
    //==================================================================
    wire [23:0] target_state = {cnt_front, cnt_rear};
    wire [23:0] last_state   = {last_front, last_rear};
    wire        need_update  = (target_state != last_state);

    assign dev_addr = DEV_ADDR;

    //==================================================================
    // 主状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_POWERUP;
            wait_cnt    <= 32'd0;
            in_init     <= 1'b1;
            init_step   <= IS_SLEEP;
            wr_ch       <= 2'd0;
            wr_byte     <= 2'd0;
            last_front  <= 12'd0;
            last_rear   <= 12'd0;
            start       <= 1'b0;
            wr_data     <= 8'd0;
            reg_addr_en <= 1'b1;
            reg_addr    <= 8'd0;
            init_done   <= 1'b0;
            err_sticky  <= 1'b0;
        end else begin
            start <= 1'b0;      // 默认单拍

            case (st)
                //------------------------------------------------------
                S_POWERUP: begin
                    if (wait_cnt >= POWERUP_CYC - 32'd1) begin
                        wait_cnt <= 32'd0;
                        in_init  <= 1'b1;
                        init_step<= IS_SLEEP;
                        st       <= S_INIT;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                //------------------------------------------------------
                // 发一条初始化命令（等 i2c_master 空闲再发）
                //   ⚠️ 不等的话：done 之后 master 还在发 STOP/释放总线，
                //      这时拉的 start 会被忽略 -> FSM 永远等 done -> 卡死
                //------------------------------------------------------
                S_INIT: begin
                    if (!busy) begin
                        reg_addr <= addr_i;
                        wr_data  <= data_i;
                        start    <= 1'b1;
                        st       <= S_WAIT;
                    end
                end

                //------------------------------------------------------
                // 等振荡器
                //------------------------------------------------------
                S_OSC_WAIT: begin
                    if (wait_cnt >= OSC_CYC - 32'd1) begin
                        wait_cnt  <= 32'd0;
                        in_init   <= 1'b0;
                        init_done <= 1'b1;
                        st        <= S_IDLE;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                //------------------------------------------------------
                // 空闲：目标变了才开始写
                //------------------------------------------------------
                S_IDLE: begin
                    if (need_update) begin
                        wr_ch   <= 2'd0;
                        wr_byte <= 2'd0;
                        st      <= S_ISSUE;
                    end
                end

                //------------------------------------------------------
                // 发一条角度命令（同样等 master 空闲）
                //------------------------------------------------------
                S_ISSUE: begin
                    if (!busy) begin
                        reg_addr <= addr_d;
                        wr_data  <= data_d;
                        start    <= 1'b1;
                        st       <= S_WAIT;
                    end
                end

                //------------------------------------------------------
                // 等 done（初始化和数据阶段共用）
                //------------------------------------------------------
                S_WAIT: begin
                    if (done) begin
                        if (ack_err) begin
                            // 从机没应答：记下来但不卡死
                            // （板子没插时也能正常上电，不会把 FSM 挂住）
                            err_sticky <= 1'b1;
                        end

                        if (in_init) begin
                            // --- 初始化：推进步骤 ---
                            if (init_step == IS_WAKE) begin
                                st <= S_OSC_WAIT;
                            end else begin
                                init_step <= init_step + 2'd1;
                                st        <= S_INIT;
                            end
                        end else begin
                            // --- 数据阶段：推进字节/通道 ---
                            if (wr_byte != 2'd3) begin
                                wr_byte <= wr_byte + 2'd1;
                                st      <= S_ISSUE;
                            end else if (wr_ch != 2'd3) begin
                                wr_byte <= 2'd0;
                                wr_ch   <= wr_ch + 2'd1;
                                st      <= S_ISSUE;
                            end else begin
                                // 4 通道全部写完 -> 锁存并回空闲
                                last_front <= cnt_front;
                                last_rear  <= cnt_rear;
                                st       <= S_IDLE;
                            end
                        end
                    end
                end

                default: st <= S_POWERUP;
            endcase
        end
    end

    //==================================================================
    // 参数自检
    //==================================================================
    generate
        if (_ASSERT_EN != 0) begin : g_assert
            initial begin
                if (PRESCALE_I < 3) begin
                    $display("[pca9685_drv] FATAL: PWM_HZ=%0d 太大，PRESCALE=%0d 非法",
                             PWM_HZ, PRESCALE_I);
                end
                if (PRESCALE_I > 255) begin
                    $display("[pca9685_drv] FATAL: PWM_HZ=%0d 太小，PRESCALE=%0d 超 8 位",
                             PWM_HZ, PRESCALE_I);
                end
                if (COUNT_MIN + 100 * COUNT_PER_H > 4095) begin
                    $display("[pca9685_drv] FATAL: 最大计数值 %0d 超过 12 位",
                             COUNT_MIN + 100 * COUNT_PER_H);
                end
                if (MS_CYC < 1) begin
                    $display("[pca9685_drv] FATAL: CLK_FREQ_HZ=%0d 太低", CLK_FREQ_HZ);
                end
            end
        end
    endgenerate

endmodule
