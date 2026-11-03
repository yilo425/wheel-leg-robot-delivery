//=============================================================================
// module : imu_driver
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : MPU6050 六轴姿态驱动（I2C，周期采样，输出 roll/pitch/yaw）。
//
//   顶层用法（集成由 A 来做）：
//       imu_driver #(...) u_imu_driver (
//           .clk (sys_clk), .rst_n (rst_n),
//           .roll (imu_roll), .pitch (imu_pitch), .yaw (imu_yaw),
//           .imu_valid (imu_valid), .imu_err (imu_err),
//           .scl (i2c_scl), .sda (i2c_sda)      // 与 TOF **共用**这条 I2C
//       );
//
//   ⚠️ 和 TOF 共用总线的前提：MPU6050 地址 0x68、VL53L0X 是 0x29，不冲突。
//      两块从机同时挂在一条 I2C 上是允许的（i2c_master 每笔事务自己带地址）。
//      但注意 i2c_master 是**单主机**、不做总线仲裁，两路驱动是顺序访问，
//      不会同时发车（各自有自己的 t_busy 握手），这点已经考虑过。
//
//-----------------------------------------------------------------------------
// ★★ 我实现到了哪一级？★★  （先看这段，免得高估它）
//-----------------------------------------------------------------------------
//   本模块做的是【最小可用姿态输出】：
//       唤醒 + 量程/采样率配置 + 周期读 12 字节原始值 + 换算 roll/pitch/yaw
//
//   **没有**做：
//       * DMP（MPU6050 内置的融合处理器）—— 那需要厂商固件，且是另一套
//         寄存器/中断流程；本模块走"原始数据 + 自己算"这条路
//       * 零偏校准（陀螺静止时也有 bias，yaw 会缓慢漂移，见下面"已知误差"）
//       * 温度补偿（温度寄存器 0x41/0x42 读了但没用）
//       * 磁力计融合（MPU6050 没有磁力计，yaw 天然会漂）
//
//   ⚠️ 上板前必须知道的两件事：
//       1) yaw 是**陀螺积分**出来的，没有绝对参考：
//          静止时若陀螺有 bias，yaw 会以 bias 的速度一直漂；
//          上电后让车静止几秒、把这段时间的陀螺均值当作 bias 减掉，
//          是后面该补的一步（本模块预留了接口位置但没有实现）。
//       2) roll/pitch 用的是**小角度线性近似**（见下面的换算说明），
//          接近 ±90° 时误差会明显变大。
//
//-----------------------------------------------------------------------------
// 姿态换算（这是本模块最需要解释的部分）
//-----------------------------------------------------------------------------
//   输出单位全部是 **0.01 度/LSB**（和 doc/interface_spec.md §3.2 一致）。
//
//   ● roll / pitch —— 加速度计重力分量，**小角度线性近似**：
//
//         真实关系：  roll  = atan2(ay, az)
//                     pitch = atan2(-ax, sqrt(ay² + az²))
//
//         在"接近水平"时（az ≈ 1g）：  θ ≈ sin(θ) = a_横向 / g
//         所以   θ[0.01°] ≈ a_lsb * (5729.58 / 16384) = a_lsb * 0.349706
//         本模块用   (a * ACC_GAIN) >>> ACC_SHIFT，其中
//                    ACC_GAIN = (5729 * 1024) / 16384 = 358
//                    ACC_SHIFT = 10
//                 => 358/1024 = 0.3496094（比理论值低 0.03%，可忽略）
//
//   ⚠️ 为什么不用 atan / 不做除法：
//       ① 项目约束：老器件上 divider 很贵，尽量只用移位；
//       ② 真 atan2 绕不开除法（ay/az），而除数 az 在倾斜时会趋近 0，
//          还得额外处理除零 —— 在这个量级的控制器上不值得。
//       ③ 代价（必须诚实说清）：这是**线性近似**，
//              θ=0°  误差 0
//              θ=30° 近似给 28.6°（差 1.4°）
//              θ=45° 近似给 40.5°（差 4.5°）
//              θ=60° 近似给 52.0°（差 8.0°）
//              θ=90° 近似给 57.3°（完全不可用，所以下面钳位到 ±90°）
//          平地行驶/小坡度（±30° 内）足够；要更准就得换分段近似或除法。
//
//   ● yaw —— 陀螺 Z 轴积分：
//
//         Δθ[0.01°] = gz_lsb * (100 / GYRO_LSB_PER_DPS) * (SAMPLE_MS / 1000)
//                   = gz_lsb * GYRO_GAIN / 2^GYRO_SHIFT
//         其中 GYRO_GAIN = (100 * SAMPLE_MS * 2^16) / (131 * 1000)   ← 常量计算，
//              不占用运行时的除法器（`/` 只出现在 localparam 的常量折叠里）
//         再加上 GYRO_HALF = 2^15 做**四舍五入**：
//              不做四舍五入的话，小角速度会被 SystemVerilog 的算术右移直接抹掉
//              （例如 1°/s、1ms 采样的真值是 0.1 个 LSB，截断恒为 0，
//               误差是 100% 而不是 5% —— 这是实测会发现"yaw 完全不动"的原因）
//
//   ● yaw 的**防溢出**用"环绕"而不是"限幅"：
//         积分器 yaw_acc 是 32 位，但每帧都把它折进 ±180°：
//             > +18000 -> 减 36000
//             < -18000 -> 加 36000
//         好处：① 32 位永远不会溢出；② 角度是圆周量，±180° 等价，
//               转整圈不会突然跳到 +327 度；③ 输出永远落在 ±180.00°。
//         每帧最多只有一次折叠（单帧 |Δθ| 最大 5°，远小于 360°），够用。
//
//-----------------------------------------------------------------------------
// 寄存器（MPU6050 数据手册；只列本模块用到的）
//-----------------------------------------------------------------------------
//   0x19 SMPLRT_DIV      采样率分频：Rate = Gyro_Output_Rate/(1+div)，默认 1kHz
//   0x1B GYRO_CONFIG     陀螺量程：0x00=±250°/s（131 LSB/(°/s)）
//   0x1C ACCEL_CONFIG    加速度量程：0x00=±2g（16384 LSB/g）
//   0x6B PWR_MGMT_1      0x00 = 清 SLEEP（唤醒）
//   0x3B..0x40           ACCEL_XOUT_H ... ACCEL_ZOUT_L（6 字节，大端）
//   0x41..0x42           TEMP_OUT（本模块读了不使用）
//   0x43..0x48           GYRO_XOUT_H ... GYRO_ZOUT_L（6 字节，大端）
//   0x75 WHO_AM_I        固定 0x68（本模块不读；调试时可读来确认器件在）
//
//   ⚠️ i2c_master 只做**单字节**复合读，所以这里是一字节一字节读，
//      一共 12 笔事务。别想着"连读 6 个字节" —— 那要改 i2c_master，
//      而它现在这样是刻意的（简单、好验证，见它的头注释）。
//      代价：12 笔 × 约 50us(400kHz) ≈ 0.6ms，相比 SAMPLE_MS=20ms 可以忽略。
//
//-----------------------------------------------------------------------------
// 数据合理性检查
//-----------------------------------------------------------------------------
//   加速度三轴**全是 0** -> 判为无效（真实器件躺着不动时 az 也应该是 ±1g；
//   三轴全 0 只可能是"器件在睡 / 没在出数 / 读回来的是空的"）。
//
//   ⚠️ 无效时：给 imu_err 单拍脉冲，**不刷新** roll/pitch/yaw
//      （沿用 tof_driver 的做法：把上一次的有效姿态留在总线上，
//       比塞 0 更安全 —— 0 姿态会被上层当成"车是平的、没转"）。
//      注意 yaw 的积分器也**不推进**：失效的这段时间没有可信的角速度，
//      宁可少积一点，也不要拿垃圾数据去积。
//
//-----------------------------------------------------------------------------
// 参数名与 doc/interface_spec.md §3.2 的差异（以代码为准）
//-----------------------------------------------------------------------------
//   文档里写的是 `.SAMPLE_PERIOD (500_000)`（时钟数），本模块用
//   **`.SAMPLE_MS (20)`**（毫秒），并且多了 TIMEOUT_MS / I2C_* / BOOT_WAIT_MS /
//   DEV_ADDR / imu_err。理由和 tof_driver 完全一样：
//   时钟数要跟着 CLK_FREQ_HZ 一起改，换 PLL 时极易漏改；ms 不会。
//   （tof_driver 那边已经踩过一次，见 doc/toolchain_gowin_modelsim.md 的教训。）
//=============================================================================

`timescale 1ns/1ps

module imu_driver #(
    //----------------------------------------------------------------
    // 时钟与采样周期
    //----------------------------------------------------------------
    parameter integer CLK_FREQ_HZ    = 50_000_000,
    parameter integer SAMPLE_MS      = 20,       // 采样周期（ms）

    //----------------------------------------------------------------
    // 超时
    //   一次采样（12 笔单字节读）超过 TIMEOUT_MS 就放弃。
    //   12 笔 @400kHz 约 0.6ms，100ms 留了两个数量级的余量。
    //----------------------------------------------------------------
    parameter integer TIMEOUT_MS     = 100,

    //----------------------------------------------------------------
    // I2C 速率与单笔传输超时（透传给 i2c_master）
    //----------------------------------------------------------------
    parameter integer I2C_SCL_HZ     = 400_000,
    parameter integer I2C_TIMEOUT_MS = 10,

    //----------------------------------------------------------------
    // 上电等待
    //   MPU6050 上电后需要约 30ms 才稳定响应 I2C（手册要求上电后等 100ms
    //   再访问更保险），这里取 50ms；仿真里改成 1 加快验证。
    //----------------------------------------------------------------
    parameter integer BOOT_WAIT_MS   = 50,

    //----------------------------------------------------------------
    // 从机地址
    //   MPU6050 的 7 位地址由 AD0 引脚决定：AD0=0 -> 0x68，AD0=1 -> 0x69。
    //   常见模块（GY-521）AD0 接地，所以默认 0x68。
    //----------------------------------------------------------------
    parameter [6:0]   DEV_ADDR       = 7'h68,

    parameter integer _ASSERT_EN     = 1
) (
    input  wire        clk,
    input  wire        rst_n,          // 低有效

    //---------------- 给 telemetry_tx / 上层的结果 ----------------
    //   单位 0.01 度/LSB（和 interface_spec §3.2 一致）
    output reg  signed [15:0] roll,
    output reg  signed [15:0] pitch,
    output reg  signed [15:0] yaw,     // ±180.00°，会环绕

    output reg         imu_valid,      // 单拍脉冲：本次姿态已更新
    output reg         imu_err,        // 单拍脉冲：本次采样失败/超时/无应答/数据不合理

    //---------------- I2C 物理接口（开漏，需外部上拉）----------------
    inout  wire        scl,
    inout  wire        sda,

    //---------------- 共享总线仲裁接口（多器件挂一条 I2C 时用）----------------
    //   单个驱动独占总线时，把 bus_grant 接 1'b1、bus_req/bus_busy 悬空即可。
    //   多器件时接 i2c_bus_arb：bus_req/bus_busy 是"请求/占用"，bus_grant 是"获准"。
    input  wire        bus_grant,      // 1 = 获准使用总线；0 时不会发出 start
    output wire        bus_req,        // 1 = 有排队事务、想用总线
    output wire        bus_busy        // 1 = 一笔事务正在总线上进行
);

    //================================================================
    // 时间常数
    //
    // ⚠️ 一律先除 1000 再乘，避免 ms * CLK_FREQ_HZ 溢出 32 位。
    //    （100ms * 50MHz = 5e9 > 2^31，是本项目最容易踩的溢出坑）
    //================================================================
    localparam [31:0] MS_CYC = CLK_FREQ_HZ / 1000;

    localparam [31:0] BOOT_CYC    = ((BOOT_WAIT_MS < 1) ? 32'd1 :
                                     BOOT_WAIT_MS) * MS_CYC;
    localparam [31:0] SAMPLE_CYC  = ((SAMPLE_MS    < 1) ? 32'd1 :
                                     SAMPLE_MS)    * MS_CYC;
    localparam [31:0] SMP_TMO_CYC = ((TIMEOUT_MS   < 1) ? 32'd1 :
                                     TIMEOUT_MS)   * MS_CYC;

    //================================================================
    // 姿态换算的常量（全部在编译期算好，运行时只有一个乘法和一次移位）
    //================================================================
    // 量程灵敏度（必须和下面初始化表里写的量程配置一致！）
    localparam integer ACCEL_LSB_PER_G  = 16384;   // ±2g   -> 16384 LSB/g
    localparam integer GYRO_LSB_PER_DPS = 131;     // ±250°/s -> 131 LSB/(°/s)

    // 加速度 -> 角度：0.01° = a_lsb * (5729.58 / 16384)
    localparam [31:0] ACC_SHIFT = 10;
    localparam [31:0] ACC_GAIN  = (5729 * 1024) / ACCEL_LSB_PER_G;   // = 358

    // 陀螺 -> 每帧角度增量（再乘采样周期）
    localparam [31:0] GYRO_SHIFT = 16;
    localparam [31:0] GYRO_HALF  = (1 << (GYRO_SHIFT - 1));          // = 32768
    // ⚠️ 这个常量表达式里 100*SAMPLE_MS*65536 是 32 位运算，
    //    SAMPLE_MS 大约 >327 就会溢出。本工程用 20，留了 16 倍余量。
    //    上限本身也是算出来的（不是拍脑袋的魔法数）：
    //        SAMPLE_MS_MAX = (2^31 - 1) / (100 * 2^GYRO_SHIFT) = 327
    localparam integer SAMPLE_MS_MAX = 2147483647 / (100 * (1 << GYRO_SHIFT));
    localparam [31:0] GYRO_GAIN  = (100 * SAMPLE_MS * (1 << GYRO_SHIFT)) /
                                   (GYRO_LSB_PER_DPS * 1000);

    // 角度钳位与 yaw 环绕
    localparam [31:0] LIMIT_ANGLE = 9000;    // ±90.00°（线性近似的可信上限）
    localparam [31:0] YAW_WRAP    = 18000;   // ±180.00°
    localparam [31:0] YAW_SPAN    = 36000;   // 一圈

    //================================================================
    // MPU6050 寄存器地址
    //================================================================
    localparam [7:0] REG_SMPLRT_DIV   = 8'h19;
    localparam [7:0] REG_GYRO_CONFIG  = 8'h1B;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;   // 0x3B..0x40 加速度 6 字节
    localparam [7:0] REG_GYRO_XOUT_H  = 8'h43;   // 0x43..0x48 陀螺   6 字节
    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;
    localparam [7:0] REG_WHO_AM_I     = 8'h75;

    // 初始化表里的**值**（和上面的量程灵敏度是一对，改一个必须改另一个）
    localparam [7:0] CFG_PWR_MGMT_1   = 8'h00;   // 清 SLEEP（内部 8MHz 时钟）
    localparam [7:0] CFG_ACCEL_CONFIG = 8'h00;   // ±2g
    localparam [7:0] CFG_GYRO_CONFIG  = 8'h00;   // ±250°/s
    localparam [7:0] CFG_SMPLRT_DIV   = 8'h09;   // 1kHz/(1+9) = 100Hz 输出率

    //================================================================
    // 初始化表（可扩展）
    //
    //   每项 = (reg, val) 一次 I2C 写。顺序有讲究：
    //   0x6B（唤醒）必须最先 —— 器件还在 SLEEP 时，其余配置写了也不生效。
    //   要加配置（比如 0x1A DLPF、0x1D 低功耗唤醒动作）就往这个 case 里加，
    //   同时改 INIT_LEN，其它逻辑一行都不用动。
    //================================================================
    localparam [7:0] INIT_LEN = 8'd4;

    reg [7:0] init_idx;
    reg [7:0] init_reg_sel;
    reg [7:0] init_val_sel;

    always @(*) begin
        case (init_idx)
            // 唤醒：清 SLEEP 位（写 0 还能顺带把时钟源选成内部 8MHz）
            8'd0: begin init_reg_sel = REG_PWR_MGMT_1;   init_val_sel = CFG_PWR_MGMT_1;   end
            // 加速度量程 ±2g
            8'd1: begin init_reg_sel = REG_ACCEL_CONFIG; init_val_sel = CFG_ACCEL_CONFIG; end
            // 陀螺量程 ±250°/s
            8'd2: begin init_reg_sel = REG_GYRO_CONFIG;  init_val_sel = CFG_GYRO_CONFIG;  end
            // 采样率分频：100Hz（比本模块的 SAMPLE_MS=20ms -> 50Hz 快，够用）
            8'd3: begin init_reg_sel = REG_SMPLRT_DIV;   init_val_sel = CFG_SMPLRT_DIV;   end
            default: begin init_reg_sel = 8'h00;         init_val_sel = 8'h00;            end
        endcase
    end

    //================================================================
    // 状态定义
    //================================================================
    localparam [2:0]
        I_BOOT = 3'd0,   // 上电等待（器件起来之前 I2C 不会应答）
        I_INIT = 3'd1,   // 逐条写初始化表
        I_WAIT = 3'd2,   // 等下一个采样周期
        I_RD   = 3'd3,   // 逐字节复合读（12 笔，靠 rd_idx 计数）
        I_EVAL = 3'd4;   // 判定 + 换算 + 输出脉冲

    reg [2:0]  state;
    reg        tx_busy;      // 当前那笔 I2C 事务是否已发起
    reg [31:0] boot_cnt;
    reg [31:0] wait_cnt;
    reg [31:0] smp_cnt;      // 一次采样内的计数（采样超时用）
    reg [3:0]  rd_idx;       // 0..11：本次采样读到第几个字节
    reg        init_ok;      // 初始化是否已成功
    reg        smp_err;      // 本次采样是否出错

    // 原始数据（大端拼好后就是有符号 16 位原始值）
    reg signed [15:0] ax_r, ay_r, az_r;
    reg signed [15:0] gx_r, gy_r, gz_r;

    // yaw 积分器（32 位，每帧折进 ±180°，所以永远不会溢出）
    reg signed [31:0] yaw_acc;

    //================================================================
    // 本次要读哪个寄存器（0..5 -> 0x3B..0x40，6..11 -> 0x43..0x48）
    //================================================================
    reg [7:0] rd_reg;

    always @(*) begin
        if (rd_idx < 4'd6) rd_reg = REG_ACCEL_XOUT_H + {4'd0, rd_idx};
        else               rd_reg = REG_GYRO_XOUT_H + {4'd0, rd_idx} - 8'd6;
    end

    //================================================================
    // 采样超时
    //   in_sample = "一次采样正在进行"
    //   双保险：i2c_master 内部有单笔传输超时，这里再管整个采样的总时长
    //================================================================
    wire in_sample  = (state == I_RD);
    wire smp_timeout = in_sample && (smp_cnt >= SMP_TMO_CYC);

    //================================================================
    // 姿态换算（纯组合；只用乘法 + 移位，**没有除法**）
    //
    //   ⚠️ 符号陷阱：Verilog 里"只要有一个操作数无符号，整个表达式就按无符号算"。
    //      所以这里所有参与乘法/比较的量都显式 $signed()，
    //      否则负的加速度会被当成很大的正数，roll 直接反号。
    //================================================================
    wire signed [31:0] roll_mul  = $signed(ay_r) * $signed(ACC_GAIN);
    wire signed [31:0] roll_raw  = roll_mul  >>> ACC_SHIFT;

    wire signed [31:0] pitch_mul = -($signed(ax_r) * $signed(ACC_GAIN));
    wire signed [31:0] pitch_raw = pitch_mul >>> ACC_SHIFT;

    // 钳位到 ±LIMIT_ANGLE（线性近似超过 ±60° 之后就没意义了）
    wire signed [31:0] roll_next  = (roll_raw >  $signed(LIMIT_ANGLE)) ?  $signed(LIMIT_ANGLE) :
                                    (roll_raw < -$signed(LIMIT_ANGLE)) ? -$signed(LIMIT_ANGLE) :
                                                                          roll_raw;
    wire signed [31:0] pitch_next = (pitch_raw >  $signed(LIMIT_ANGLE)) ?  $signed(LIMIT_ANGLE) :
                                    (pitch_raw < -$signed(LIMIT_ANGLE)) ? -$signed(LIMIT_ANGLE) :
                                                                           pitch_raw;

    // 陀螺积分：先乘增益、加半个 LSB 做四舍五入，再算术右移
    wire signed [47:0] gyro_mul  = $signed(gz_r) * $signed(GYRO_GAIN);
    wire signed [47:0] gyro_rnd  = gyro_mul + $signed(GYRO_HALF);
    wire signed [31:0] yaw_delta = gyro_rnd >>> GYRO_SHIFT;

    // 环绕（每帧最多折一次；单帧 |Δθ| 最大约 5°，不可能一次折过头）
    wire signed [31:0] yaw_sum   = yaw_acc + yaw_delta;
    wire signed [31:0] yaw_next  = (yaw_sum >  $signed(YAW_WRAP)) ? (yaw_sum - $signed(YAW_SPAN)) :
                                   (yaw_sum < -$signed(YAW_WRAP)) ? (yaw_sum + $signed(YAW_SPAN)) :
                                                                     yaw_sum;

    // 数据合理性：加速度三轴全 0 = 器件没在出数
    wire acc_all_zero = (ax_r == 16'sd0) && (ay_r == 16'sd0) && (az_r == 16'sd0);

    //================================================================
    // I2C 事务发起器（把一次事务打包好交给 i2c_master）
    //
    //   为什么要有这一层（这段是从 tof_driver 抄过来的，理由完全一样）：
    //     状态机里每个状态只需要写"发什么"，不用关心 i2c_master 什么时候空闲、
    //     done 在哪一拍来。
    //     握手：主 FSM 拉 t_req 一拍 -> 内部 t_pending -> 等 master 空闲后发车
    //          -> master 结束 -> t_done_r 一拍 + t_err_r / t_data_r
    //
    //   ⚠️ 必须等 master 真的空闲（i2c_busy=0）再发车，否则 start 脉冲会落在
    //      master 的传输中间被直接丢掉，这一层就永远等不到 done
    //      （表现为每次采样都超时）。
    //================================================================
    reg        t_req;
    reg [6:0]  t_addr;
    reg        t_rd;
    reg        t_reg_en;
    reg [7:0]  t_wr;
    reg [7:0]  t_reg;
    reg        t_clr;        // 丢弃"已请求但还没发出去"的事务（超时收尾用）

    reg        i2c_start;
    wire [7:0] i2c_rd_data;
    wire       i2c_busy;
    wire       i2c_ack_err;
    wire       i2c_done;

    reg        t_busy;
    reg        t_pending;
    reg        t_done_r;
    reg        t_err_r;
    reg [7:0]  t_data_r;

    //---------------- 总线仲裁信号导出 ----------------
    assign bus_req  = t_pending;
    assign bus_busy = t_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_start <= 1'b0;
            t_busy    <= 1'b0;
            t_pending <= 1'b0;
            t_done_r  <= 1'b0;
            t_err_r   <= 1'b0;
            t_data_r  <= 8'h00;
        end else begin
            i2c_start <= 1'b0;      // 单拍脉冲
            t_done_r  <= 1'b0;      // 单拍脉冲

            if (t_req) t_pending <= 1'b1;

            if (!t_busy) begin
                // ★ 多器件共享总线时还要 bus_grant=1：没获准就继续等，不和另一个驱动抢总线
                if (t_pending && !i2c_busy && bus_grant) begin
                    i2c_start <= 1'b1;
                    t_busy    <= 1'b1;
                    t_pending <= 1'b0;
                end
            end else if (i2c_done) begin
                t_busy   <= 1'b0;
                t_done_r <= 1'b1;
                t_err_r  <= i2c_ack_err;
                t_data_r <= i2c_rd_data;
            end

            // 超时收尾：把还没发出去的请求丢掉，避免超时之后总线上又冒出一笔交易
            if (t_clr) t_pending <= 1'b0;
        end
    end

    //================================================================
    // 采样超时计数
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smp_cnt <= 32'd0;
        end else if (!in_sample) begin
            smp_cnt <= 32'd0;
        end else begin
            smp_cnt <= smp_cnt + 32'd1;
        end
    end

    //================================================================
    // 主状态机
    //================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= I_BOOT;
            tx_busy   <= 1'b0;
            boot_cnt  <= 32'd0;
            wait_cnt  <= 32'd0;
            init_idx  <= 8'd0;
            rd_idx    <= 4'd0;
            init_ok   <= 1'b0;
            smp_err   <= 1'b0;
            ax_r      <= 16'sd0;
            ay_r      <= 16'sd0;
            az_r      <= 16'sd0;
            gx_r      <= 16'sd0;
            gy_r      <= 16'sd0;
            gz_r      <= 16'sd0;
            yaw_acc   <= 32'sd0;
            roll      <= 16'sd0;
            pitch     <= 16'sd0;
            yaw       <= 16'sd0;
            imu_valid <= 1'b0;
            imu_err   <= 1'b0;
            t_req     <= 1'b0;
            t_addr    <= DEV_ADDR;
            t_rd      <= 1'b0;
            t_reg_en  <= 1'b0;
            t_wr      <= 8'h00;
            t_reg     <= 8'h00;
            t_clr     <= 1'b0;
        end else begin
            // 两个输出都是单拍脉冲：默认清零
            imu_valid <= 1'b0;
            imu_err   <= 1'b0;
            t_req     <= 1'b0;
            t_clr     <= 1'b0;

            if (smp_timeout) begin
                // 采样超时：置错误，等总线空下来再去 I_EVAL 报错
                smp_err <= 1'b1;
                tx_busy <= 1'b0;
                t_clr   <= 1'b1;
                if (!i2c_busy && !t_busy) state <= I_EVAL;
            end else begin
                case (state)

                //--------------------------------------------------
                // 上电等待：MPU6050 起来之前 I2C 是不会应答的
                //--------------------------------------------------
                I_BOOT: begin
                    if (boot_cnt >= BOOT_CYC) begin
                        boot_cnt <= 32'd0;
                        init_idx <= 8'd0;
                        tx_busy  <= 1'b0;
                        state    <= I_INIT;
                    end else begin
                        boot_cnt <= boot_cnt + 32'd1;
                    end
                end

                //--------------------------------------------------
                // 初始化表：逐条写寄存器（START | dev+W | reg | val | STOP）
                //
                //   这一步的等待有超时兜底吗？有 —— 每笔写都由 i2c_master 自己
                //   的 TIMEOUT_MS 限时；写失败(NACK/超时)就报一次 imu_err、
                //   回 I_WAIT，下一个采样周期**重新初始化**。
                //   （上电顺序/上拉没稳时很常见，重试比一次定生死合理。）
                //--------------------------------------------------
                I_INIT: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b0;
                        t_reg_en <= 1'b1;
                        t_reg    <= init_reg_sel;
                        t_wr     <= init_val_sel;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin
                            init_ok  <= 1'b0;
                            imu_err  <= 1'b1;
                            init_idx <= 8'd0;
                            wait_cnt <= 32'd0;
                            state    <= I_WAIT;
                        end else if (init_idx >= (INIT_LEN - 8'd1)) begin
                            init_ok  <= 1'b1;
                            wait_cnt <= 32'd0;
                            state    <= I_WAIT;
                        end else begin
                            init_idx <= init_idx + 8'd1;
                        end
                    end
                end

                //--------------------------------------------------
                // 等采样周期（SAMPLE_MS）
                //   到点后：没初始化成功就回去初始化，否则开始读一帧
                //--------------------------------------------------
                I_WAIT: begin
                    if (wait_cnt >= SAMPLE_CYC) begin
                        wait_cnt <= 32'd0;
                        tx_busy  <= 1'b0;
                        if (!init_ok) begin
                            init_idx <= 8'd0;
                            state    <= I_INIT;
                        end else begin
                            smp_err  <= 1'b0;
                            rd_idx   <= 4'd0;
                            state    <= I_RD;
                        end
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                //--------------------------------------------------
                // 逐字节复合读：rd=1 + reg_addr_en=1（i2c_master 的 C 型传输）
                //   12 个字节读完 -> I_EVAL
                //--------------------------------------------------
                I_RD: begin
                    if (!tx_busy) begin
                        t_addr   <= DEV_ADDR;
                        t_rd     <= 1'b1;
                        t_reg_en <= 1'b1;
                        t_reg    <= rd_reg;
                        t_wr     <= 8'h00;
                        t_req    <= 1'b1;
                        tx_busy  <= 1'b1;
                    end else if (t_done_r) begin
                        tx_busy <= 1'b0;
                        if (t_err_r) begin
                            // 有一笔没成功：整帧作废（不要拿半帧数据算姿态）
                            smp_err <= 1'b1;
                            state   <= I_EVAL;
                        end else begin
                            case (rd_idx)
                                4'd0:  ax_r[15:8] <= t_data_r;
                                4'd1:  ax_r[7:0]  <= t_data_r;
                                4'd2:  ay_r[15:8] <= t_data_r;
                                4'd3:  ay_r[7:0]  <= t_data_r;
                                4'd4:  az_r[15:8] <= t_data_r;
                                4'd5:  az_r[7:0]  <= t_data_r;
                                // ⚠️ rd_idx 6 直接就是陀螺 0x43：本模块**跳过**了
                                //    0x41/0x42（温度，用不上），不做"读掉就算"，
                                //    因为多读两字节就多两笔 I2C 事务，没有意义。
                                //    所以 rd_reg 在 idx>=6 时是 0x43+(idx-6)。
                                4'd6:  gx_r[15:8] <= t_data_r;
                                4'd7:  gx_r[7:0]  <= t_data_r;
                                4'd8:  gy_r[15:8] <= t_data_r;
                                4'd9:  gy_r[7:0]  <= t_data_r;
                                4'd10: gz_r[15:8] <= t_data_r;
                                4'd11: gz_r[7:0]  <= t_data_r;
                                default: ;   // 不会发生
                            endcase

                            if (rd_idx == 4'd11) begin
                                state <= I_EVAL;
                            end else begin
                                rd_idx <= rd_idx + 4'd1;
                            end
                        end
                    end
                end

                //--------------------------------------------------
                // 判定 + 换算 + 输出
                //--------------------------------------------------
                I_EVAL: begin
                    tx_busy <= 1'b0;
                    if (smp_err) begin
                        imu_err <= 1'b1;            // 传输失败/超时/无应答
                    end else if (acc_all_zero) begin
                        imu_err <= 1'b1;            // 数据不合理：器件没在出数
                    end else begin
                        roll      <= roll_next[15:0];
                        pitch     <= pitch_next[15:0];
                        yaw       <= yaw_next[15:0];
                        yaw_acc   <= yaw_next;      // 积分器只在**有效帧**推进
                        imu_valid <= 1'b1;
                    end
                    smp_err  <= 1'b0;
                    wait_cnt <= 32'd0;
                    state    <= I_WAIT;
                end

                default: begin
                    state   <= I_BOOT;
                    tx_busy <= 1'b0;
                end
                endcase
            end
        end
    end

    //================================================================
    // I2C 主机（与 tof_driver 共用同一个模块，配置一样）
    //================================================================
    i2c_master #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .SCL_HZ      (I2C_SCL_HZ),
        .TIMEOUT_MS  (I2C_TIMEOUT_MS)
    ) u_i2c (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (i2c_start),
        .dev_addr    (t_addr),
        .rd          (t_rd),
        .wr_data     (t_wr),
        .reg_addr_en (t_reg_en),
        .reg_addr    (t_reg),
        .rd_data     (i2c_rd_data),
        .busy        (i2c_busy),
        .ack_err     (i2c_ack_err),
        .done        (i2c_done),
        .scl         (scl),
        .sda         (sda)
    );

    //================================================================
    // 参数自检（综合器忽略 initial，仿真里能立刻看出参数配错）
    //================================================================
    initial begin
        if (_ASSERT_EN == 1) begin
            if (INIT_LEN < 8'd1) begin
                $display("[imu_driver] ERROR: INIT_LEN 必须 >= 1");
            end
            if (ACC_GAIN < 32'd1) begin
                $display("[imu_driver] ERROR: ACC_GAIN=0 —— 检查 ACCEL_LSB_PER_G");
            end
            if (GYRO_GAIN < 32'd1) begin
                $display("[imu_driver] ERROR: GYRO_GAIN=0 —— SAMPLE_MS 太小或 GYRO_LSB_PER_DPS 太大");
            end
            // 常量表达式的溢出风险提示（100*SAMPLE_MS*2^GYRO_SHIFT）
            if (SAMPLE_MS > SAMPLE_MS_MAX) begin
                $display("[imu_driver] ERROR: SAMPLE_MS=%0d 太大（上限 %0d），GYRO_GAIN 的常量计算会溢出 32 位",
                         SAMPLE_MS, SAMPLE_MS_MAX);
            end
            // 姿态换算的标定关系（换量程时这两行会立刻变得可疑）
            $display("[imu_driver] SAMPLE=%0dms TIMEOUT=%0dms BOOT=%0dms ADDR=0x%02h",
                     SAMPLE_MS, TIMEOUT_MS, BOOT_WAIT_MS, DEV_ADDR);
            $display("[imu_driver] ACC: %0d LSB/g -> GAIN=%0d >>%0d  |  GYRO: %0d LSB/(d/s) -> GAIN=%0d >>%0d",
                     ACCEL_LSB_PER_G, ACC_GAIN, ACC_SHIFT,
                     GYRO_LSB_PER_DPS, GYRO_GAIN, GYRO_SHIFT);
            $display("[imu_driver] 姿态单位 0.01 度/LSB；yaw 环绕 ±%0d (=±180.00 度)", YAW_WRAP);
        end
    end

endmodule
