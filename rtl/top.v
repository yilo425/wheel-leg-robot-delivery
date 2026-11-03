//=============================================================================
// module : top
// layer  : 顶层
//-----------------------------------------------------------------------------
// 说明 : 本层只做例化与连线, 不写复杂逻辑。
//
// 本版本把整条决策链路接通:
//
//   ┌─────────────┐   ┌──────────┐   ┌─────────────────┐   ┌───────────┐
//   │ 传感器输入   │──►│ main_fsm │──►│ motion_decision │──►│ pwm_motor │──► 电机×4
//   │ TOF/视觉/舵机│   │ 行为决策  │   │   速度闭环       │   │  载波生成  │
//   └─────────────┘   └──────────┘   └─────────────────┘   └───────────┘
//                                            ▲
//                                     ┌──────┴──────┐
//                                     │encoder_decode│ ×4
//                                     └─────────────┘
//
// 每层的职责边界(不要越界, 否则后面很难改):
//   main_fsm        : 只决定"走/停/减速/抬腿", 输出 speed_scale 这种**意图**
//   motion_decision : 把意图变成每轮目标速度, 做比例闭环
//   pwm_motor       : 只管把"占空比幅值+方向"变成载波
//
//-----------------------------------------------------------------------------
// 已接入 : clk_gen, reset_sync, encoder_decode ×4, i2c_master + tof_driver,
//          servo_lift, uart_frame + vision_rx + telemetry_tx,
//          main_fsm, motion_decision, pwm_motor ×4
// 待接入 : imu_driver(复用 i2c_master), display_ctrl, 后向 TOF
//          这些模块的**输入端口已经在 top 上引出**, 接上去即可, 不用再改本文件。
//
//-----------------------------------------------------------------------------
// 两个重要约定
//-----------------------------------------------------------------------------
// 1) sys_clk 目前**直连板载 50MHz**。
//    等 clk_gen.v(gowin_pll) 就绪后, 把 sys_clk 换成 PLL 的 100MHz 输出,
//    并同步改 CLK_FREQ_HZ —— **这两处必须一致**, 否则 PWM 载波频率、
//    测速窗口换算、main_fsm 的 ms 计时会同时偏一倍。
//
// 2) 传感器现在**有真有假**:
//      前向 TOF 是真的 —— tof_driver 走 I2C, 见 §1.5 (TOF_ENABLE=1, 默认);
//      IMU 还没接 —— telemetry_tx 收到的姿态角是常量 0。
//    上板调试想先绕开 TOF 时, 把 TOF_ENABLE 置 0, dist_mm 退回
//    DIST_STUB_MM 常量 (链路能一段段打通, 不需要等传感器全到齐)。
//=============================================================================
`timescale 1ns/1ps

module top #(
    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    parameter integer CLK_FREQ_HZ   = 50_000_000,   // 当前=板载晶振; 换 PLL 后改 100_000_000
    parameter integer POR_CYCLES    = 1024,

    //------------------------------------------------------------------
    // PLL 旁路开关
    //   1 = 直通板载 50MHz（默认）—— 现在就能仿真/上板，不需要 PLL
    //   0 = 用 clk_gen 里的 PLL 出 100MHz —— 需先按 clk_gen.v 的说明生成
    //
    //   ⚠️ 本参数和 CLK_FREQ_HZ **必须配对**：
    //         PLL_BYPASS=1 -> CLK_FREQ_HZ =  50_000_000
    //         PLL_BYPASS=0 -> CLK_FREQ_HZ = 100_000_000
    //      只改一个的话，PWM 载波频率、测速窗口换算、main_fsm 的毫秒计时
    //      会**同时偏一倍**，而且不会有任何报错。
    //      文件末尾有一条 initial 断言专门盯这个配对。
    //------------------------------------------------------------------
    parameter integer PLL_BYPASS    = 1,

    //------------------------------------------------------------------
    // ★ 前向 TOF 测距（VL53L0X，I2C）
    //
    //   TOF_ENABLE = 1（默认）: 例化真实 tof_driver，dist_mm 来自 I2C 总线上的
    //                           VL53L0X，对外只占 i2c_scl / i2c_sda 两根引脚。
    //   TOF_ENABLE = 0        : 退回"桩"模式，dist_mm 恒等于 DIST_STUB_MM。
    //                           没有 TOF 模块时用它先把 FSM/电机/舵机链路打通。
    //
    //   ⚠️ 桩模式**只用于分段联调**。整机试跑时开着它，等于告诉车
    //      "前方永远 2m 通畅"，它**不会避障** —— 而且不会有任何报错。
    //
    //   下面这几个参数是"接口参数"，真正的时间常数在 tof_driver 里：
    //     SAMPLE_MS  采样周期（注意参数名是 SAMPLE_MS，不是 SAMPLE_PERIOD）
    //     BOOT_WAIT_MS 上电/释放 XSHUT 后等器件起来的时间
    //     DIST_MAX_MM  超过这个读数判为无效（VL53L0X 量程 ~2m）
    //------------------------------------------------------------------
    //   ★ 2026-10-18 **电机台架测试期间置 0**：超声波（HC-SR04）已拆下。
    //     置 0 时 dist_mm 恒为 DIST_STUB_MM(2000) 且"永远有效" → 状态机走全速档。
    //     若保持 1 而模块不在：dist_ok=0 → 一直按"减速"跑（半速），
    //     测电机时凭空多一个变量。
    //     **接回超声波后必须改回 1**（否则车以为前方永远畅通，不会避障）。
    //   （"1" 的历史原因：2026-10-17 曾为 VL53L0X 未到货置 0，后来接了 I2C/超声波。）
    parameter integer TOF_ENABLE        = 0,

    parameter integer DIST_STUB_MM      = 2000,     // 仅 TOF_ENABLE=0 时使用
    parameter integer TOF_SAMPLE_MS     = 50,       // 采样周期 (ms)
    parameter integer TOF_TIMEOUT_MS    = 100,      // 单次采样超时 (ms)
    parameter integer TOF_BOOT_WAIT_MS  = 10,       // 上电等待 (ms)
    //   ★ 2026-10-17 I2C 挪到 GPIO0 扩展口（原板载 I2C 排针针序无法确认，
    //     没有万用表不敢乱插 —— 详见 constraints/pin_table.csv 区段 8）。
    //     代价：GPIO0 没有板载 4.7k 上拉，只能靠 VL53L0X 模块自带的 10k。
    //     10k 上拉在 400kHz 下边沿太慢（>300ns 的 fast-mode 要求），
    //     所以**从 400kHz 降到 100kHz**。若读不到器件，补两个 4.7k 到 3V3。
    parameter integer TOF_I2C_SCL_HZ    = 100_000,  // I2C 速率
    parameter integer TOF_I2C_TIMEOUT_MS= 10,       // 单笔 I2C 传输超时 (ms)
    parameter integer TOF_DIST_MAX_MM   = 2000,     // 读数合理性上限 (mm)
    parameter [6:0]   TOF_DEV_ADDR      = 7'h29,    // VL53L0X 出厂 7 位地址

    //------------------------------------------------------------------
    // ★ 测距接口选择（2026-10-17 新增）
    //
    //   实际买到的不是 VL53L0X，而是 **HC-SR04 超声波模块（4 线模式）**：
    //   引脚是 VCC/TRIG/ECHO/GND，靠"触发脉冲 + 回波脉宽"测距，
    //   和 I2C 的 VL53L0X 完全是两套协议，所以多一个驱动。
    //
    //     TOF_IF = 0 : I2C VL53L0X（原设计，用 i2c_scl/i2c_sda）
    //     TOF_IF = 1 : HC-SR04 超声波（用 sr04_trig/sr04_echo）
    //
    //   两者输出**同一组** dist_mm/dist_valid/dist_err，上层 main_fsm 不用改。
    //
    //   ⚠️ 超声波的 ECHO 电平**跟随供电电压**：
    //      模块按 5V 供电时 ECHO 输出 5V → **会打死 3.3V 的 FPGA IO**。
    //      所以 sr04_echo 这根必须确保 ≤3.3V：要么模块按 3.3V 供电，
    //      要么在 ECHO 上串 1k+2k 分压。宁可欠压不工作，也不能过压。
    //------------------------------------------------------------------
    parameter integer TOF_IF            = 1,        // 0=I2C VL53L0X, 1=HC-SR04

    //------------------------------------------------------------------
    // ★ 舵机驱动方式（2026-10-19 加参数，2026-10-26 接进 top）
    //
    //   SERVO_IF = 0 : servo_lift 直接产生 PWM，用 lift_pwm[1:0] 两根引脚
    //   SERVO_IF = 1 : 通过 I2C 驱动 PCA9685 舵机板，用 i2c_scl / i2c_sda
    //                  4 个舵机直接插板 —— 不用 Y 线、不用分线板
    //
    //   ⚠️ SERVO_IF=1 的前提：TOF_IF=1（超声波）且 IMU_ENABLE=0，
    //      否则同一条 I2C 总线会被两个主机抢（编译期 assert 会报）。
    //   ⚠️ SERVO_IF=1 时 servo_lift **仍然要例化** ——
    //      它提供 lift_done / lift_fault（限位开关和故障信号），
    //      只是 lift_pwm 的两根引脚不再接舵机（空着）。
    //------------------------------------------------------------------
    parameter integer SERVO_IF             = 1,       // 0=直接PWM, 1=PCA9685（2026-10-29 切到控制板）
    parameter integer SERVO_I2C_SCL_HZ     = 100_000, // 舵机 I2C 速率
    parameter integer SERVO_I2C_TIMEOUT_MS = 10,      // 单次 I2C 超时
    parameter integer SERVO_PWM_HZ         = 50,      // 舵机 50Hz
    parameter [6:0]   SERVO_PCA_ADDR       = 7'h40,   // PCA9685 默认地址
    parameter integer SERVO_COUNT_MIN      = 102,     // 0.5ms 对应计数
    parameter integer SERVO_COUNT_PER_H    = 4,       // 每单位高度 4 计数
    parameter integer SR04_TRIG_US      = 10,       // 触发脉冲宽度 (us)
    parameter integer SR04_SAMPLE_MS    = 60,       // 测量周期 (ms)，HC-SR04 建议 >=60
    parameter integer SR04_TMO_MS       = 40,       // 等回波超时 (ms)，4m 往返约 24ms
    parameter integer SR04_MIN_MM       = 20,       // 盲区
    //   量程上限取 4100 而不是 4000：换算系数是**向上取整**的（+0.09%），
    //   4000mm 会读成 4003mm，若上限写 4000 会把标称量程的最后 3mm 判成超量程。
    //   留 100mm 余量即可，不影响"超量程保护"的作用。
    parameter integer SR04_MAX_MM       = 4100,     // 量程上限
    parameter integer SR04_BOOT_WAIT_MS = 50,       // 上电稳定等待

    //------------------------------------------------------------------
    // ★ IMU 姿态（MPU6050，和 TOF 共用同一条 I2C）
    //
    //   IMU_ENABLE = 1（**默认**，已集成）: 例化 imu_driver，并启用 I2C 总线仲裁
    //                              （i2c_bus_arb），roll/pitch/yaw 进 telemetry。
    //   IMU_ENABLE = 0           : 不例化 imu_driver，telemetry 的姿态恒为常量 0；
    //                              TOF 直接拿常授权（等于独占总线），
    //                              行为和"只有 TOF"时完全一样。
    //
    //   ⚠️ IMU 还没焊上车 / 只想先跑通 TOF 时，把这里置 0：
    //      挂在一个不存在的 0x68 上只会一直 NACK（虽然不会撞车，但白占总线时间），
    //      telemetry 里会看到 imu_valid 一直是 0。
    //      接好 IMU 后保持 1 即可（也就是默认值）。
    //
    //   ⚠️ MPU6050 地址 0x68、VL53L0X 0x29，**不冲突**，可以并挂。
    //      但两块 VL53L0X 会冲突（都是 0x29），后向 TOF 得靠 XSHUT 改地址。
    //------------------------------------------------------------------
    //   ★ 2026-10-17 上板阶段也置 0：MPU6050 还没接。挂着不存在的 0x68
    //     只会一直 NACK（不撞车，但白占总线时间）。**IMU 接好后改回 1**。
    parameter integer IMU_ENABLE        = 0,
    parameter integer IMU_SAMPLE_MS     = 20,       // 采样周期 (ms)
    parameter integer IMU_TIMEOUT_MS    = 100,      // 单次采样超时 (ms)
    parameter integer IMU_BOOT_WAIT_MS  = 50,       // 上电等待 (ms)
    parameter integer IMU_I2C_SCL_HZ    = 100_000,  // I2C 速率（和 TOF 同总线，同上原因降到 100k）
    parameter integer IMU_I2C_TIMEOUT_MS= 10,       // 单笔 I2C 传输超时 (ms)
    parameter [6:0]   IMU_DEV_ADDR      = 7'h68,    // MPU6050 出厂 7 位地址

    //------------------------------------------------------------------
    // ★ 电机护栏（motor_guard）：软启动 + 堵转保护
    //
    //   两个会**烧硬件**的问题，闭环本身管不了：
    //     1) 堵转：轮子被卡住时编码器反馈≈0，但闭环会一直加大占空比
    //        → 驱动板（TM6612）持续大电流 → 过热/烧管子，电池被拖垮；
    //     2) 启动冲击：从 0 猛给满占空比，启动电流是稳态的好几倍。
    //
    //   参数怎么定（都要实车调）：
    //     GUARD_RAMP_STEP  : 每 GUARD_CTRL_MS 允许增加多少 duty。
    //                        默认 8/ms → 从 0 到满速（69）约 9ms，够快又不冲。
    //     GUARD_STALL_DUTY : duty 超过它才算"在使劲"。默认 40 ——
    //                        注意要比"正常低速巡航"的 duty 高，
    //                        否则慢速爬行会被误判成堵转。
    //     GUARD_STALL_SPD_MAX : |编码器净计数| <= 它算"没转"（默认 2）。
    //     GUARD_STALL_MS   : 持续这么久判堵转（默认 500ms）。
    //     GUARD_COOLDOWN_MS: 断电后冷却多久再放开重试（默认 1000ms）。
    //                       卡阻还在就会"断续试探"，不会一直顶死。
    //
    //   ⚠️ 判据只对"有编码器的轮子"成立：编码器没接/坏了，会一直被当成堵转 ——
    //      上板第一步先确认编码器真的在出数（否则电机一动不动，见 LED3/telemetry）。
    //------------------------------------------------------------------
    parameter integer GUARD_CTRL_MS       = 1,
    parameter integer GUARD_RAMP_STEP     = 8,
    parameter integer GUARD_STALL_DUTY    = 40,
    parameter integer GUARD_STALL_SPD_MAX = 2,
    parameter integer GUARD_STALL_MS      = 500,
    parameter integer GUARD_COOLDOWN_MS   = 1000,

    //------------------------------------------------------------------
    // ★ 堵转保护总开关（2026-10-18 新增）
    //
    //   1 = 开（**产品/带载配置**）：软启动 + 堵转保护都生效
    //   0 = 关（*仅台架测试*）：只旁路堵转判定，软启动仍然生效
    //
    //   为什么需要这个开关：**没有编码器时 spd_fb 恒为 0**，而速度闭环会把
    //   duty 顶到限幅（>=STALL_DUTY 40），于是 500ms 后**必然**被判堵转 ——
    //   上台架的表现就是"电机转半秒就停，反复抽"，很容易误判成接线坏了。
    //   这不是 bug：没有反馈就无法区分"堵转"和"没接编码器"。
    //
    //   ⚠️ 装车/带载前必须改回 1 —— 那是防止烧驱动板的最后一道闸。
    //------------------------------------------------------------------
    parameter integer GUARD_ENABLE        = 0,

    //------------------------------------------------------------------
    // ★★★ 电机自测模式（2026-10-18 新增）
    //
    //   0 = 正常（走状态机 -> 速度环 -> 护栏 -> PWM）
    //   1 = **直接接管 4 路电机输出**，彻底绕开状态机/护栏/速度环：
    //       每 2 秒给 1.3 秒的 40% 占空比，然后停 0.7 秒，四个轮子同步
    //
    //   为什么需要它：上板排查"电机为什么不转"时，最怕变量太多
    //   （状态机没进 AUTO？堵转保护误判？速度环不收敛？）。
    //   这个模式把链路压缩成 **FPGA 引脚 -> 驱动板 -> 电机**，只看这一段。
    //   2026-10-18 那次排查就是靠同样的思路（当时是独立工程）定位到
    //   "驱动板输出级烧了"的 —— 现在把它做成参数，不用再单独烧诊断固件。
    //
    //   ⚠️ 只在台架用。MOTOR_TEST_MODE=1 时电机**不受急停控制**
    //      （m_en 被强制拉高），轮子必须离地。
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // ★ 2026-11-03 自主版：遥控链路（0x05 RC_CMD）已彻底删除。
    //   以下超时参数保留仅作历史参考，当前无任何模块使用。
    //------------------------------------------------------------------
    parameter integer RC_TIMEOUT_MS       = 500,
    // speed 分档阈值：0=停，1..50=慢，51..100=满
    parameter integer RC_SPEED_SLOW_MAX   = 50,

    parameter integer MOTOR_TEST_MODE     = 0,
    // ★ 2026-11-01：后轮与前轮镜像安装，方向要取反
    //   bit0=A 左前  bit1=B 左后  bit2=C 右后  bit3=D 右前
    parameter integer FORCE_OPEN_LOOP     = 0,   // ★ 2026-11-03 自主版：改 0 = 编码器闭环
    // ★ ★ 2026-11-02：伺服电机休眠开关
    //   1 = 把舵机计数写 0（PCA9685 输出 0 脉宽→舵机无保持扭矩）
    //        用来手动扳腿、校零位；此时舵机不会发热也不会抗拒
    //   0 = 正常工作
    // ★ 左后腿单独微调（计数，有符号，通道 2）
    parameter integer REAR_L_TRIM        = 23,
    parameter integer SERVO_OFF           = 0,     // 0=正常；1=舵机松力（手动扳腿时用）
    // ★ 2026-11-03：这个值是【实测出来】的，不是推导的 —— 不要凭注释改。
    //   电机是镜像安装：物理上 A/C/D 三个轮子的转向相反，
    //   实测「取反 A(bit0) + D(bit3)」之后四个轮子方向就一致了。
    //   ⚠️ 开闭环(FORCE_OPEN_LOOP=0)后方向反了就是正反馈（越跑越快），
    //      上电务必【轮子离地、低速】逐个确认：给前进指令四轮同向、
    //      手轻挡能回落。某轮抖/加速 -> 翻对应位或临时 FORCE_OPEN_LOOP=1。
    parameter integer DIR_INV_MASK        = 4'b1001,   // 取反 A(左前) + D(右前)【实测值】

    //------------------------------------------------------------------
    // PWM 与编码器
    //------------------------------------------------------------------
    parameter integer PWM_FREQ_HZ   = 20_000,
    parameter integer DUTY_GAIN     = 128,
    parameter integer ENC_FILTER    = 8,
    parameter integer SPEED_WIN     = 500_000,      // 测速窗口, 10ms @50MHz

    //------------------------------------------------------------------
    // 运动决策
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // 满速参考值：每个测速窗口内期望的编码器净计数
    //
    //   ★ 按实购电机（Wheeltec MG310，1:20，13 线）重新标定：
    //
    //       每轮一圈的计数 = PPR × 4倍频 × 减速比
    //                      = 13 × 4 × 20 = 1040
    //
    //       窗口 = SPEED_WIN / CLK_FREQ_HZ = 500000/50e6 = 10ms
    //
    //       VREF_FULL = (额定RPM/60) × 1040 × 0.01
    //                 = (400/60) × 10.4
    //                 = 69.3  ->  取 69
    //
    //   额定 400±13% RPM -> 69 对应约 398 RPM，正好是额定点。
    //   原来是 60（对应 346 RPM，只跑到 86%），换电机后重算。
    //
    //   ⚠️ 换电机时这个数必须跟着改，否则"满速档"的物理速度会变。
    //      公式：VREF_FULL = (目标RPM/60) × (PPR×4×减速比) × 窗口秒数
    //------------------------------------------------------------------
    parameter integer VREF_FULL     = 69,     // 满速目标计数/窗口（MG310 1:20 13线）
    parameter integer KP_SHIFT      = 1,
    parameter integer LIMIT_DUTY    = 120,
    parameter integer DEADBAND      = 1,
    // ★ 2026-10-30：4 -> 6。原来 steer 偏置 ±100 比 VREF_FULL(69) 还大，
    //   一打方向内侧轮目标就变成负数（反转）-> 实际是原地打转，手感很怪。
    //   改成 >>>6 = ±25，约 ±36% 差速，正常转弯。
    parameter integer STEER_SHIFT   = 6,

    // ★ 2026-10-30：站立高度（遥控升高/降低）
    //   STEP_MS 越大越慢（限制舵机电流）：102 步 × 20ms ≈ 2 秒走完 45°
    parameter integer STANCE_STEP_MS   = 20,
    parameter integer STANCE_CNT_MID   = 307,   // 1.5ms 中立位（0 状态=高位）
    parameter integer STANCE_CNT_SPAN  = 102,   // 45° ≈ 0.5ms
    // ★ 旋转方向：0=原方向 1=反向（舵机翻转安装后，力臂更小、腿与前缘平齐）
    parameter integer STANCE_INVERT    = 1,


    //------------------------------------------------------------------
    // ★★★ 电机电压限制（2026-10-18 新增）
    //
    //   问题：电池是 **12V**，电机 MG310 额定 **7.4V**。
    //   直接 12V 满占空比 = 过压 62% → 电流大、电机和驱动板一起发烫
    //   （实测已经触发过 TB6612 的过热保护）。
    //
    //   做法：**PWM 限压**（店家说的"靠板子限压"就是这个）——
    //   串激/永磁有刷电机的**转速和平均电压成正比**，所以只要把占空比
    //   压住，等效电压就压住了：
    //
    //       电机平均电压 = 电池电压 × (duty / 128)
    //       要求 ≤ 7.4V  ->  duty ≤ 128 × 7.4 / Vbat
    //
    //   取值按**电池满电**算（12V 铅酸充满 13.8V / 3S 锂电 12.6V），
    //   取最坏 13.8V：duty ≤ 128 × 7.4 / 13.8 = **68**。
    //   68/128 = 53%，13.8×0.53 = 7.3V ✓  12.0×0.53 = 6.4V ✓
    //
    //   ⚠️ 三个必须知道的限制：
    //   1) **PWM 限压 ≠ 降压模块**。电机看到的仍是 12V 方波，只是平均电压低，
    //      电流纹波和发热比"真·7.4V 直流"略大。长时间大负载建议上 DC-DC 降压。
    //   2) 限占空比会**同时压低最高转速和加速能力** —— 这是应该的。
    //   3) 换电池/换电机时**必须重算**这个数，别照抄。
    //
    //   0 或 >=128 = 不限制（仅调试用，别配 12V 时这么干）。
    //------------------------------------------------------------------
    parameter integer V_LIMIT_DUTY  = 68,     // 最终占空比上限（≈53%，对应 12V→6.4V）


    //------------------------------------------------------------------
    // 转向系数（每轮一个）—— ★ 轮子编号 0=左前 1=右前 2=左后 3=右后
    //
    //   ⚠️ 这一组必须和 rtl/app/motion_decision.v 的默认值保持一致！
    //      曾经这里是 (1,1,-1,-1)（按左右分组），而引脚表是按位置分组的
    //      (0=左前 1=右前 2=左后 3=右后) —— 两边各自自洽但合起来是错的，
    //      机械组按引脚表接好线，转向会整个反过来。
    //      2026-10-13 统一成按位置编号：左轮(0,2)=+1，右轮(1,3)=-1。
    //
    //   steer>0 表示右转，右转要靠左轮加速 -> 左轮系数必须为正。
    //------------------------------------------------------------------
    // ★ 2026-11-01 上层旋转 180° 后的通道映射：
    //   A=左前 FL   B=左后 RL   C=右后 RR   D=右前 FR
    parameter integer STEER_COEF0   =  1,     // A 左前 -> 左轮 +1
    parameter integer STEER_COEF1   =  1,     // B 左后 -> 左轮 +1
    parameter integer STEER_COEF2   = -1,     // C 右后 -> 右轮 -1
    parameter integer STEER_COEF3   = -1,     // D 右前 -> 右轮 -1

    //------------------------------------------------------------------
    // main_fsm 阈值
    //------------------------------------------------------------------
    parameter integer DIST_STOP_MM     = 200,
    parameter integer DIST_SLOW_MM     = 500,
    parameter integer DIST_CLEAR_MM    = 800,
    parameter integer AVOID_TIMEOUT_MS = 5000,
    parameter integer LIFT_TIMEOUT_MS  = 3000,
    parameter integer CROSS_TIMEOUT_MS = 2000,
    // 整车通过的最短保持时间（见 main_fsm.v 里的详细说明）
    //   防止前向 TOF 一报通畅就收腿，而后轮还没过障碍
    parameter integer CROSS_MIN_MS     = 800,
    // 放腿后等待舵机走完行程的时间
    parameter integer LIFT_SETTLE_MS   = 600,
    parameter [7:0]   VIS_LOW_OBST     = 8'h04,   // 低矮障碍(小石头) -> 四腿一起抬
    parameter [7:0]   VIS_OBST         = 8'h02,   // 障碍物           -> 绕行
    parameter [7:0]   VIS_STEP         = 8'h05,   // 台阶/门槛        -> 两腿分段抬

    //------------------------------------------------------------------
    // 升降腿舵机（servo_lift）
    //
    //   标准航模舵机：周期 20ms(50Hz)，脉宽 0.5~2.5ms 对应 0~180°。
    //   本设计只用两个位置：放下 = PWM_MIN_US，抬起 = PWM_MAX_US。
    //
    //   ⚠️ 不同舵机行程不一样。首次调试时如果听到舵机顶到机械限位
    //      发出持续嗡嗡声，把这两个值改小（比如 1000 / 2000）再试。
    //------------------------------------------------------------------
    parameter integer SERVO_PERIOD_US   = 20_000,   // 50Hz
    parameter integer SERVO_MIN_US      = 500,      // 放下（0°）
    parameter integer SERVO_MAX_US      = 2_500,    // 抬起（180°）
    parameter integer SERVO_DEBOUNCE_MS = 10,       // 限位开关消抖

    //------------------------------------------------------------------
    // UART (ARM 通信)
    //------------------------------------------------------------------
    parameter integer BAUD_RATE        = 115200,
    parameter integer FRAME_TIMEOUT_MS = 20,   // 帧内字节间隔超时(见 uart_frame)
    parameter integer TELEM_RATE_HZ    = 10,   // TELEMETRY 上报频率

    //------------------------------------------------------------------
    // 视觉坐标 -> 转向换算
    //   ★ 这几个值**必须在实车上调**：把车放地上，让 ARM 发固定 x 的帧，
    //     看转向是否过冲。过冲就加大 VIS_STEER_SHIFT（输出变小）。
    //   详见 rtl/app/vision_rx.v 的头注释。
    //------------------------------------------------------------------
    parameter integer VIS_X_CENTER    = 32768, // 归一化坐标的画面正中心
    parameter integer VIS_STEER_DEAD  = 2048,  // 死区半宽（约 ±3% 画面宽度）
    parameter integer VIS_STEER_SHIFT = 6,     // steer = e' >>> SHIFT
    parameter integer VIS_STEER_MAX   = 512,   // 钳位幅值

    //------------------------------------------------------------------
    // 调试开关
    //------------------------------------------------------------------
    // 1 = 用 key[1] 作急停(低有效); 0 = 忽略按键急停。
    //   调试初期如果板上没焊 key[1], 读到的电平不确定, 会把状态机一直顶在
    //   ESTOP 上, 看起来像"什么都没反应"。这时把本参数置 0 即可绕过。
    parameter integer ESTOP_FROM_KEY = 1
) (
    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    input  wire        clk_50m,      // 板载 50MHz 晶振
    input  wire        rst_key_n,    // 复位按键, 低有效

    //------------------------------------------------------------------
    // ★ ARM 通信 (UART, 115200 8N1, 3.3V TTL)
    //
    //   接线只有 3 根:
    //       FPGA uart_tx ──► ARM RX
    //       FPGA uart_rx ◄── ARM TX
    //       GND ──────────── GND      ← 必须共地, 否则通信全乱
    //------------------------------------------------------------------
    output wire        uart_tx,      // 到 ARM 的 RX
    input  wire        uart_rx,      // 来自 ARM 的 TX

    //------------------------------------------------------------------
    // 电机驱动接口 (到驱动板逻辑侧)
    //------------------------------------------------------------------
    output wire [3:0]  m_pwm,        // 4 路 PWM
    output wire [3:0]  m_dir,        // 4 路方向（接 TB6612 的 AIN1/BIN1/CIN1/DIN1）
    //------------------------------------------------------------------
    // ★ 第二方向脚（2026-10-18 新增，接 AIN2/BIN2/CIN2/DIN2）
    //
    //   为什么需要：TB6612 每一个通道要 **两个互补的方向脚**才能正反转
    //     AIN1=1, AIN2=0 -> 正转
    //     AIN1=0, AIN2=1 -> 反转
    //     AIN1=AIN2      -> 刹车/滑行（**不转**）
    //   而我们原来每个电机只给 1 根方向线 —— 如果 AIN2 接地，
    //   `dir=0`（倒车）就变成 AIN1=0/AIN2=0 = 滑行，**永远倒不了车**。
    //
    //   所以这里把 `m_dir2[i] = ~m_dir[i]` 引出来：
    //     · 接上这 4 根  -> 完整正反转 ✓
    //     · 不接、AIN2 仍接地 -> 行为退化成"只前进"，和以前一样（安全）
    //
    //   引脚够用：GPIO0 上还有 F19/F18/E19/D20 四个空闲脚。
    //------------------------------------------------------------------
    output wire [3:0]  m_dir2,       // 4 路第二方向（= ~m_dir，接 AIN2/BIN2/...）
    output wire        m_en,         // 驱动器使能 (高有效)

    //------------------------------------------------------------------
    // 编码器接口 (A/B 相, 硬件侧已有上拉)
    //------------------------------------------------------------------
    input  wire [3:0]  enc_a,
    input  wire [3:0]  enc_b,

    //------------------------------------------------------------------
    // 人机接口
    //------------------------------------------------------------------
    input  wire [1:0]  key,          // 低有效: key[0]=启动/停止  key[1]=急停
    output wire [3:0]  led,

    //==================================================================
    // 以下为"待接入模块"的接口 —— 已引出, 接上即可
    //==================================================================
    //---------------- ★ TOF 测距：2 根 I2C 引脚（2026-10-14 已接真实驱动）----------------
    //   为什么 dist_mm[15:0]+valid+err **不做**端口（这一步没变，结论是对的）：
    //
    //   1) 板子（ACG720 / GW5AST-138B）总共有约 53 个可用 IO，
    //      而 top 原来需要 119 个。其中 dist_mm[15:0]+valid+err
    //      一个人就占 18 个。
    //
    //   2) 更根本的是：**16 位并行 TOF 现实中不存在**。
    //      真实模块要么走 I2C（VL53L0X 等，2 根线），
    //      要么走 UART（TF-Mini 等，2 根线），要么是
    //      HC-SR04 那种 2 根线（trig/echo）需要自己计时。
    //      没有哪个模块是把距离直接摆成 16 根并行线的。
    //
    //   3) 所以正确做法是：加一个 tof_driver，
    //      它对外只占 2 根引脚，内部把距离组织成 dist_mm。
    //      dist_mm / dist_valid / dist_err 是**内部信号**，
    //      接线方式和 main_fsm 的接口都不用改。
    //
    //   ★ 现在 tof_driver 已经接上了（§1.5），dist_mm 不再是常量桩。
    //     引脚表里 i2c_scl=M13 / i2c_sda=M16，就是板上那排 I2C 排针，
    //     VL53L0X 模块直接插上去即可。
    //     两根线都是**开漏**，靠板上 4.7k 上拉；FPGA 只会拉低或释放。
    //
    //   ⚠️ 仿真里 tb_top_integration 仍然用 force 直接灌真实距离来做确定性
    //      场景测试（那条 tb 跑 TOF_ENABLE=0）；走真实 I2C 通路的用例在
    //      tb_top_tof_i2c.v 里（VL53L0X 从机模型 + 上拉，不 force）。
    inout  wire        i2c_scl,      // TOF 时钟（开漏，需外部上拉）
    inout  wire        i2c_sda,      // TOF 数据（开漏，需外部上拉）

    //---------------- HC-SR04 超声波（TOF_IF=1 时使用）----------------
    //   trig 是输出（10us 高脉冲），echo 是输入（高电平宽度 = 往返时间）。
    //   ⚠️ echo 必须确保 ≤3.3V（模块按 5V 供电时它输出 5V，会打死 IO）。
    output wire        sr04_trig,
    input  wire        sr04_echo,

    //---------------- 导航 ----------------
    //   ⚠️ vision_class / vision_valid / steer 已经**不再是端口** ——
    //      它们现在由 UART 收到的 VISION_RESULT 帧驱动（见本文件 §3.5）。
    //      ARM 是唯一的视觉来源，再留一份外部引脚只会造成"到底谁在驱动"
    //      的困惑。
    //
    //   goal_reached 暂时**保留为端口**：协议里还没有"到达终点"的字段，
    //   目前留给人接终点限位开关，或等 ARM 侧加一帧 CONFIG 再改成内部信号。
    input  wire        goal_reached,

    //---------------- 升降腿（2 组：前腿组 / 后腿组）----------------
    //------------------------------------------------------------------
    // ⚠️ 这里和以前不一样了，改动原因要记住：
    //
    //   以前是 1 位的 lift_cmd（前后腿连在一起），做不了"跨台阶"——
    //   跨台阶要求【前腿先抬 -> 前进 -> 后腿再抬 -> 再前进】。
    //
    //   现在是 2 位，前后腿可以独立控制：
    //     {后腿, 前腿}   00=都放下  01=只抬前腿  10=只抬后腿  11=都抬起
    //
    //   并且舵机 PWM 由片内的 servo_lift 模块生成 ——
    //   所以对外输出的直接就是**舵机能吃的 50Hz PWM**，
    //   不需要外挂舵机驱动板。
    //
    //   限位开关 / 故障输入都是**低有效**（常开触点 + 上拉）：
    //     未按下读 1、按下读 0
    //   servo_lift 内部做消抖 + 极性转换，输出高有效给 main_fsm。
    //   ★ 这个极性以前是错的（.cst 配了上拉，但 main_fsm 按高有效判），
    //     一上板就会把"未按下"当成"已到位"，腿根本没抬就跳下一步。
    //------------------------------------------------------------------
    output wire [1:0]  lift_pwm,            // 舵机 PWM：{后腿, 前腿}
    input  wire [1:0]  lift_lim_n,          // 限位开关（低有效，常开+上拉）
    input  wire [1:0]  lift_fault_n,        // 舵机故障（低有效，不接=1=正常）

    //---------------- 状态观测 (给 LED / 显示 / ARM 上报) ----------------
    output wire [2:0]  state_dbg,
    output wire [1:0]  speed_scale_dbg,
    output wire        error_flag,

    //---------------- 调试串口（只发，接板载 USB-UART）----------------
    //   和 uart_tx(C19) 输出的是**同一路数据**，只是多接一个引脚：
    //     C19  -> 给 ARM（板外接线）
    //     M15  -> 板上 USB 转串口芯片 -> 电脑的 COM 口
    //   为什么需要它：ARM 不在手上时，没有它就只能靠 4 个 LED 猜状态；
    //   有了它，插根 USB 就能在电脑上看到 10Hz telemetry（含 dist_mm）。
    //   ⚠️ M15/J21 是板上原生的串口引脚，只有小焊盘（没有排针）。
    output wire        uart_tx_dbg

    //---------------- ★ UART / 视觉统计：已不再是端口 ----------------
    //   原来有 4 个 [15:0] 统计端口，共 **64 位** —— 全部 64 个引脚
    //   板子上根本放不下（总预算才 53）。
    //
    //   而且它们**只在仿真里有意义**：板上跑的时候你不会去数
    //   "收了多少帧"，出问题看 TELEMETRY 就够了（state / 速度 / 距离
    //   都在里面，10Hz 自动回传）。
    //
    //   现在它们是 top 内部的 wire，tb_top_integration 通过层次引用读
    //   （compile_all.do 一直带 -voptargs="+acc"，读得到）。
);

    //==================================================================
    // ★ PCA9685 舵机路径的命令线（SERVO_IF=1 时用）
    //
    //   ⚠️ i2c_master 的 scl/sda **直接接到顶层 i2c_scl / i2c_sda 引脚**，
    //      不再经过内部线 —— 否则引脚上的上拉回不到主机侧，
    //      主机会永远读到自己那根线是 z，卡在 START（见 §1.5 分支 A 的注释）。
    //
    //   为什么这些线声明在这里：§1.5 的 generate 要用它们，
    //   而 i2c_master / pca9685_drv 的例化要等到 §4.6
    //   （那里 lift_cmd / cfg_lift_height 才声明）。
    //==================================================================
    wire        pca_start;
    wire [6:0]  pca_dev_addr;
    wire [7:0]  pca_wr_data;
    wire        pca_reg_addr_en;
    wire [7:0]  pca_reg_addr;
    wire        pca_done;
    wire        pca_busy;
    wire        pca_ack_err;
    wire        pca_init_done;
    wire        pca_err_sticky;

    //==================================================================
    // TOF 的 dist_mm / dist_valid / dist_err
    //
    //   ★ 它们**不再在这里定义** —— 现在是 tof_driver 的输出，
    //     在 §1.5 例化并连出来（因为 tof_driver 要用 sys_clk / rst_n，
    //      而这两根线在 §0/§1 才声明；Verilog 不允许先用后声明，
    //      否则会生成隐式 1 位线网，工具只给 warning）。
    //
    //   定宽常量：Verilog-2001 **不允许对 parameter 做位选**，
    //   不能写 DIST_STUB_MM[15:0]，必须先 localparam 定宽。
    //   （这个坑本项目踩过 5 次以上）—— 桩模式在 §1.5 的 generate 里。
    //==================================================================

    //==================================================================
    // 0) 时钟：clk_gen 产生系统时钟与锁定指示
    //
    //    clk_gen 默认是**旁路**的（BYPASS=1）：clk_sys 就是板载 50MHz，
    //    locked 恒 1。这样没有 PLL 也能仿真、能上板跑 LED/按键/编码器/
    //    PWM/UART 的联调（UART 115200 @50MHz 的 OS_DIV=27，够用）。
    //
    //    要上 100MHz 时按 clk_gen.v 开头的说明用 Gowin IP Core Generator
    //    生成 PLL 填进去，再把 PLL_BYPASS 改 0、CLK_FREQ_HZ 改 100MHz。
    //==================================================================
    wire sys_clk;
    wire locked;

    clk_gen #(
        .BYPASS     (PLL_BYPASS),
        .CLK_IN_HZ  (50_000_000),
        .CLK_OUT_HZ (PLL_BYPASS ? 50_000_000 : 100_000_000)
    ) u_clk_gen (
        .clk_in  (clk_50m),
        .arst_n  (rst_key_n),
        .clk_sys (sys_clk),
        .locked  (locked)
    );

    //------------------------------------------------------------------
    // 参数配对自检
    //
    //   PLL_BYPASS 和 CLK_FREQ_HZ 不一致时，设计**不会报错**，
    //   只会让 PWM 载波、测速窗口、毫秒计时同时偏一倍 ——
    //   这种"静默的错误频率"是最难查的一类问题。
    //   所以在这里断言一次，仿真一开始就能看到。
    //------------------------------------------------------------------
    initial begin
        if ((PLL_BYPASS == 1) && (CLK_FREQ_HZ != 50_000_000)) begin
            $display("[top] ERROR: PLL_BYPASS=1 时 CLK_FREQ_HZ 必须 = 50000000, 实际 = %0d",
                     CLK_FREQ_HZ);
            $display("      (旁路模式下系统时钟就是板载 50MHz)");
        end
        if ((PLL_BYPASS == 0) && (CLK_FREQ_HZ != 100_000_000)) begin
            $display("[top] ERROR: PLL_BYPASS=0 时 CLK_FREQ_HZ 必须 = 100000000, 实际 = %0d",
                     CLK_FREQ_HZ);
            $display("      (PLL 模式下系统时钟是 100MHz)");
        end
    end

    //==================================================================
    // 1) 复位: 异步 assert / 同步 release
    //==================================================================
    wire rst_n;

    reset_sync #(
        .SYNC_STAGES (3),
        .POR_CYCLES  (POR_CYCLES)
    ) u_reset_sync (
        .clk    (sys_clk),
        .arst_n (rst_key_n & locked),
        .rst_n  (rst_n)
    );

    //==================================================================
    // 1.5) ★ 前向 TOF 测距：tof_driver（VL53L0X / I2C）
    //
    //   这一节把"原来那三根常量桩"换成真实驱动：
    //       tof_driver ──► dist_mm / dist_valid / dist_err ──► main_fsm
    //                 └──► i2c_master ──► i2c_scl / i2c_sda（板上 I2C 排针）
    //
    //   为什么在 §1 之后例化：
    //       tof_driver 要用 sys_clk（§0）和 rst_n（§1）。
    //       Verilog 不允许"先用后声明" —— 那样会生成**隐式 1 位线网**，
    //       多数工具只给个 warning，然后行为完全不对。
    //       本工程就是靠 tools/check_decl_order.py 抓出过这种错误。
    //
    //   为什么保留一个 TOF_ENABLE=0 的桩分支：
    //       硬件没到、或者只想验证 FSM/电机/舵机时，把它置 0 就能让
    //       dist_mm 恒等于 DIST_STUB_MM，不用等 TOF 模块。
    //       ★ 但整机试跑时必须 TOF_ENABLE=1，否则车**不会避障**。
    //
    //   ⚠️ i2c_scl / i2c_sda 是**开漏**：本模块只拉低或释放（三态），
    //      必须靠板上 4.7k 上拉才有高电平。仿真里没有上拉线就是 x，
    //      所以 tb 用 pullup 原语顶替（见 tb_top_tof_i2c.v）。
    //==================================================================
    wire [15:0] dist_mm;        // 最近一次**有效**距离（mm）
    wire        dist_valid;     // 单拍脉冲：本次读数有效
    wire        dist_err;       // 单拍脉冲：本次测距失败

    //---------------- 共享 I2C 总线的仲裁信号（见 §1.6）----------------
    wire        tof_grant, tof_req, tof_busy;
    wire        imu_grant, imu_req, imu_busy;

    //---------------- IMU 姿态输出（telemetry 用，见 §3.5）----------------
    wire signed [15:0] imu_roll, imu_pitch, imu_yaw;
    wire               imu_valid, imu_err_pulse;

    generate
        //-------- 分支 A：没有测距器件（桩数据）--------
        if (TOF_ENABLE == 0) begin : g_tof_stub
            // 定宽常量：Verilog-2001 不允许对 parameter 做位选
            localparam [15:0] DIST_STUB_W = DIST_STUB_MM;

            assign dist_mm    = DIST_STUB_W;
            assign dist_valid = 1'b1;      // 桩数据永远"有效"
            assign dist_err   = 1'b0;      // 桩数据永远"不报错"

            // 桩模式：总线上没有 TOF —— 通知仲裁器"永远不请求、不占用"
            assign tof_req    = 1'b0;
            assign tof_busy   = 1'b0;

            // ⚠️ 这里**不要**再 assign i2c_scl/i2c_sda！
            //    舵机版 i2c_master（§4.6）和 IMU（§1.6）都是**直接接到引脚**上的，
            //    它们自己的开漏输出就是这根线的驱动源。
            //    以前这里写过 `assign i2c_scl = pca_scl_i;` —— 那是把"内部线"
            //    单向映射到引脚，引脚上的上拉**回不来**，主机读自己那根线永远是 z，
            //    于是 SCL 等不到高电平，卡在 START 直到超时（仿真里 8ms 才退）。
            // 没用到超声波：触发脚保持低（模块只在上升沿后发波）
            assign sr04_trig  = 1'b0;

        //-------- 分支 B：I2C 的 VL53L0X（原设计）--------
        end else if (TOF_IF == 0) begin : g_tof_i2c
            tof_driver #(
                .CLK_FREQ_HZ     (CLK_FREQ_HZ),
                .SAMPLE_MS       (TOF_SAMPLE_MS),
                .TIMEOUT_MS      (TOF_TIMEOUT_MS),
                .I2C_SCL_HZ      (TOF_I2C_SCL_HZ),
                .I2C_TIMEOUT_MS  (TOF_I2C_TIMEOUT_MS),
                .BOOT_WAIT_MS    (TOF_BOOT_WAIT_MS),
                .DIST_MAX_MM     (TOF_DIST_MAX_MM),
                .DEV_ADDR        (TOF_DEV_ADDR)
            ) u_tof_driver (
                .clk        (sys_clk),
                .rst_n      (rst_n),
                .dist_mm    (dist_mm),
                .dist_valid (dist_valid),
                .dist_err   (dist_err),
                .scl        (i2c_scl),
                .sda        (i2c_sda),
                .bus_grant  (tof_grant),
                .bus_req    (tof_req),
                .bus_busy   (tof_busy)
            );
            // 没用到超声波
            assign sr04_trig  = 1'b0;

        //-------- 分支 C：HC-SR04 超声波（4 线：VCC/TRIG/ECHO/GND）--------
        end else begin : g_tof_sr04
            tof_sr04 #(
                .CLK_FREQ_HZ  (CLK_FREQ_HZ),
                .SAMPLE_MS    (SR04_SAMPLE_MS),
                .TMO_MS       (SR04_TMO_MS),
                .TRIG_US      (SR04_TRIG_US),
                .MIN_MM       (SR04_MIN_MM),
                .MAX_MM       (SR04_MAX_MM),
                .BOOT_WAIT_MS (SR04_BOOT_WAIT_MS)
            ) u_tof_sr04 (
                .clk        (sys_clk),
                .rst_n      (rst_n),
                .trig       (sr04_trig),
                .echo       (sr04_echo),
                .dist_mm    (dist_mm),
                .dist_valid (dist_valid),
                .dist_err   (dist_err),
                .ok_cnt     (),
                .err_cnt    ()
            );

            // 不走 I2C：仲裁器侧固定"不请求/不占用"
            assign tof_req    = 1'b0;
            assign tof_busy   = 1'b0;
            // ⚠️ 别在这里 assign i2c_scl/i2c_sda（原因见分支 A 的注释）
        end
    endgenerate

    //==================================================================
    // 1.6) ★ 共享 I2C 总线的仲裁 + IMU（MPU6050）
    //
    //   TOF 和 IMU 都在同一条 i2c_scl/i2c_sda 上：
    //       I2C 是**开漏**总线，两个驱动的 scl/sda 直接并上去就行（线与），
    //       电气上不需要任何 mux —— 缺的只是"同一时刻只能有一个 master 说话"。
    //   i2c_bus_arb 就只做这件事：发 grant；驱动侧用 `t_pending && !i2c_busy && bus_grant`
    //   门控 start，没拿到 grant 就一直等（不会抢总线）。
    //
    //   ⚠️ 为什么不在片内做"三态 mux"：
    //      FPGA 内部没有三态资源，内部三态综合会出问题。
    //      开漏天然支持线与，所以直接并 + 授权即可。
    //
    //   IMU_ENABLE=0（默认）时不例化仲裁器和 IMU，TOF 直接拿常授权，
    //   行为和"TOF 独占总线"完全一样（原有 tb 不受影响）。
    //==================================================================
    generate
        if (IMU_ENABLE != 0) begin : g_imu_real
            i2c_bus_arb u_i2c_bus_arb (
                .clk    (sys_clk),
                .rst_n  (rst_n),
                .req0   (tof_req),
                .busy0  (tof_busy),
                .grant0 (tof_grant),
                .req1   (imu_req),
                .busy1  (imu_busy),
                .grant1 (imu_grant)
            );

            imu_driver #(
                .CLK_FREQ_HZ     (CLK_FREQ_HZ),
                .SAMPLE_MS       (IMU_SAMPLE_MS),
                .TIMEOUT_MS      (IMU_TIMEOUT_MS),
                .I2C_SCL_HZ      (IMU_I2C_SCL_HZ),
                .I2C_TIMEOUT_MS  (IMU_I2C_TIMEOUT_MS),
                .BOOT_WAIT_MS    (IMU_BOOT_WAIT_MS),
                .DEV_ADDR        (IMU_DEV_ADDR)
            ) u_imu_driver (
                .clk        (sys_clk),
                .rst_n      (rst_n),
                .roll       (imu_roll),
                .pitch      (imu_pitch),
                .yaw        (imu_yaw),
                .imu_valid  (imu_valid),
                .imu_err    (imu_err_pulse),
                .scl        (i2c_scl),
                .sda        (i2c_sda),
                .bus_grant  (imu_grant),
                .bus_req    (imu_req),
                .bus_busy   (imu_busy)
            );
        end else begin : g_imu_off
            // 关掉 IMU：TOF 拿常授权（独占总线），IMU 侧全部固定为"不请求/不占用"
            assign tof_grant      = 1'b1;
            assign imu_grant      = 1'b0;
            assign imu_req        = 1'b0;
            assign imu_busy       = 1'b0;
            assign imu_roll       = 16'sd0;
            assign imu_pitch      = 16'sd0;
            assign imu_yaw        = 16'sd0;
            assign imu_valid      = 1'b0;
            assign imu_err_pulse  = 1'b0;
        end
    endgenerate

    //==================================================================
    // 2) 编码器正交解码 + 测速 (4 路轮子)
    //==================================================================
    wire signed [31:0] pos_cnt   [0:3];
    wire signed [23:0] speed_cnt [0:3];
    wire               enc_dir   [0:3];
    wire               spd_valid [0:3];

    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_enc
            encoder_decode #(
                .FILTER_CYCLES       (ENC_FILTER),
                .SPEED_WINDOW_CYCLES (SPEED_WIN),
                .POS_WIDTH           (32),
                .SPEED_WIDTH         (24)
            ) u_encoder_decode (
                .clk         (sys_clk),
                .rst_n       (rst_n),
                .enc_a       (enc_a[gi]),
                .enc_b       (enc_b[gi]),
                .pos_cnt     (pos_cnt[gi]),
                .speed_cnt   (speed_cnt[gi]),
                .dir         (enc_dir[gi]),
                .speed_valid (spd_valid[gi]),
                .window_cnt  ()
            );
        end
    endgenerate

    wire any_spd_valid = spd_valid[0] | spd_valid[1] | spd_valid[2] | spd_valid[3];

    //==================================================================
    // 3) 按键边沿检测与信号整形
    //
    //    key[0] (低有效) -> 按下的**下降沿**产生一个 start 脉冲。
    //       必须做边沿检测: main_fsm 收到的是"单拍脉冲",
    //       如果直接给电平, 状态机会每拍都看到 start, 行为完全不对。
    //
    //    key[1] (低有效) -> 取反后作为 estop 电平。
    //       急停是**电平**语义, 不是脉冲 —— 按住就一直是急停。
    //==================================================================
    reg key0_d;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) key0_d <= 1'b1;          // 释放态为高
        else        key0_d <= key[0];
    end

    // ★ 2026-11-03：start/estop 现在 = 物理按键 + 网页命令（CONFIG 0x22）
    //   具体赋值放在 config_rx 例化之后 —— 那里 cfg_run_cmd 才声明
    wire start_pulse;
    wire estop_sig;
    // 手动模式：现在由 CONFIG 帧（ID 0x20）运行时控制，不用重新综合。
    //   以前硬接 1'b0，手动模式永远进不去。
    //   ⚠️ 赋值放在 config_rx 例化**之后**（`cfg_manual_sel` 要先声明再用，
    //      否则 Verilog-2001 会生成隐式 1 位网 —— check_decl_order.py 会报）。
    wire manual_sel;

    //==================================================================
    // 3.5) ★ ARM 通信链路
    //
    //    uart_frame ──► vision_rx ──► vision_class / vision_valid / steer
    //         ▲                              │
    //         │                              ▼
    //    telemetry_tx ◄── state/spd/dist   main_fsm / motion_decision
    //
    //   为什么单独拆出 vision_rx / telemetry_tx 两个模块，
    //   而不把解析逻辑写在 top 里：
    //      顶层只做例化与连线。载荷解析（字节偏移、大端拼装、最优目标挑选、
    //      定点换算）是有状态的逻辑，放在顶层会让这个文件再也读不动。
    //
    //   ⚠️ 这一段的 wire 声明必须在 main_fsm 例化**之前** ——
    //      Verilog 里用到未声明的标识符会生成**隐式 1 位线网**，
    //      多数工具只给个 warning，然后行为完全不对。
    //      tools/find_undeclared.py 会扫这类问题。
    //==================================================================
    //---- uart_frame 的接收侧 ----
    wire [7:0]  uf_rx_type;
    wire [7:0]  uf_rx_len;
    wire        uf_rx_valid;
    wire        uf_rx_bcc_err;
    wire        uf_rx_timeout;
    wire        uf_rx_drop;
    wire [7:0]  uf_rx_pl_data;
    wire [7:0]  vr_rx_pl_addr;
    wire [7:0]  cfg_rx_pl_addr;   // config_rx 的地址（和 vision 做二选一 mux）
    wire [7:0]  uf_rx_pl_addr;    // 真正送到 uart_frame 的地址
    wire [15:0] uf_frame_ok;
    wire [15:0] uf_frame_err;

    //---- config_rx 的输出（CONFIG 帧运行时配置）----
    wire [15:0] cfg_vref;
    wire [15:0] cfg_dist_stop_mm;
    wire [15:0] cfg_dist_slow_mm;
    wire [7:0]  cfg_lift_height;
    wire [15:0] cfg_lift_timeout_ms;
    wire        cfg_manual_sel;
    wire        cfg_run_cmd;        // ★ 2026-11-03：CONFIG 0x22 运行/停止
    wire [15:0] cfg_run_ms;         // ★ 2026-11-03：CONFIG 0x23 绕行直行时长（毫秒）
    wire [1:0]  turn_dir_fsm;       // ★ 2026-11-03：main_fsm 绕行转向
    wire        dir_inv_fsm;        // ★ 2026-11-03：main_fsm 后退标志
    wire        cfg_stance_low;      // ★ 0x21：0=高位 1=低位
    wire        cfg_valid_w;
    wire        cfg_bad_w;
    wire [15:0] cfg_cnt_w;
    wire [15:0] cfg_bad_cnt_w;
    wire [15:0] cfg_ign_cnt_w;

    //---- uart_frame 的发送侧（由 telemetry_tx 驱动）----
    wire        tt_pl_wr;
    wire [7:0]  tt_pl_addr;
    wire [7:0]  tt_pl_data;
    wire        tt_req;
    wire [7:0]  tt_type;
    wire [7:0]  tt_len;
    wire        uf_tx_busy;
    wire        uf_tx_done;

    //---- vision_rx 的输出（给 main_fsm / motion_decision）----
    wire [7:0]  vision_class_w;
    wire        vision_valid_w;
    wire signed [15:0] steer_w;
    wire [15:0] vr_target_x;
    wire [7:0]  vr_target_conf;
    wire        vr_bad;
    wire [15:0] vr_frame_cnt;
    wire [15:0] vr_bad_cnt;

    //---- main_fsm 的输出 ----
    //   state 由后面的 main_fsm 产生，但本节的 telemetry_tx 要用它，
    //   所以必须在这里**提前声明**。
    //   如果留到 main_fsm 那节才声明，本节就是「先用后声明」——
    //   Verilog 会生成**隐式 1 位线网**，多数工具只给 warning，
    //   结果是 TELEMETRY 里的状态字段恒为 0/1，完全查不出来。
    //   这个错误真的发生了，是 tools/check_decl_order.py 抓出来的。
    wire [2:0]  state;
    //---- telemetry_tx 的统计 ----
    wire [15:0] tt_sent_cnt;
    wire [15:0] tt_dropped_cnt;
    wire        tt_wait_timeout;

    //------------------------------------------------------------------
    // uart_frame : 帧协议（收发缓冲 + BCC + 超时 + 重同步）
    //   已验证：sim/tb_uart_frame.v 58 项检查全过
    //------------------------------------------------------------------
    uart_frame #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .BAUD_RATE        (BAUD_RATE),
        .FRAME_TIMEOUT_MS (FRAME_TIMEOUT_MS),
        .MAX_PAYLOAD      (64),
        .PROTO_VER        (8'h01),
        .HDR1             (8'hAA),
        .HDR2             (8'h55),
        .TAIL1            (8'h55),
        .TAIL2            (8'hAA)
    ) u_uart_frame (
        .clk           (sys_clk),
        .rst_n         (rst_n),
        .rx_pin        (uart_rx),
        .tx_pin        (uart_tx),
        // 发送（来自 telemetry_tx）
        .tx_pl_wr      (tt_pl_wr),
        .tx_pl_addr    (tt_pl_addr),
        .tx_pl_data    (tt_pl_data),
        .tx_req        (tt_req),
        .tx_type       (tt_type),
        .tx_len        (tt_len),
        .tx_busy       (uf_tx_busy),
        .tx_done       (uf_tx_done),
        // 接收
        .rx_type       (uf_rx_type),
        .rx_len        (uf_rx_len),
        .rx_valid      (uf_rx_valid),
        .rx_bcc_err    (uf_rx_bcc_err),
        .rx_timeout    (uf_rx_timeout),
        .rx_drop       (uf_rx_drop),
        // ★ 载荷读地址：vision_rx 和 config_rx **共用** uart_frame 这一个地址口，
        //   按帧类型二选一。`rx_type` 在 rx_valid 之后一直保持到下一帧，
        //   所以整个"走地址读载荷"的窗口里 mux 选择是稳定的。
        .rx_pl_addr    (uf_rx_pl_addr),
        .rx_pl_data    (uf_rx_pl_data),
        .frame_ok_cnt  (uf_frame_ok),
        .frame_err_cnt (uf_frame_err)
    );

    // ★ 2026-11-03 自主版：遥控链路已彻底移除（不再有 RC_CMD 解析）

    // ★ 2026-11-03 自主版：0x05 RC_CMD 已移除，只保留视觉(0x02)和配置(0x04)
    assign uf_rx_pl_addr = (uf_rx_type == 8'h02) ? vr_rx_pl_addr  :
                           (uf_rx_type == 8'h04) ? cfg_rx_pl_addr : 8'd0;

    // 调试串口：和发给 ARM 的是同一路数据，只是多引一个引脚到板载 USB-UART
    assign uart_tx_dbg = uart_tx;

    //------------------------------------------------------------------
    // vision_rx : 从 0x02 VISION_RESULT 帧解析出类别与转向
    //   策略：取置信度最高的目标（见 vision_rx.v 头注释）
    //   模型已验证：tools/verify_vision_rx_model.py 61 项
    //------------------------------------------------------------------
    vision_rx #(
        .TYPE_VISION_RESULT (8'h02),
        .MAX_TARGETS        (8),
        .X_CENTER           (VIS_X_CENTER),
        .STEER_DEAD         (VIS_STEER_DEAD),
        .STEER_SHIFT        (VIS_STEER_SHIFT),
        .STEER_MAX          (VIS_STEER_MAX)
    ) u_vision_rx (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .rx_valid     (uf_rx_valid),
        .rx_type      (uf_rx_type),
        .rx_len       (uf_rx_len),
        .rx_pl_data   (uf_rx_pl_data),
        .rx_pl_addr   (vr_rx_pl_addr),
        .vision_class (vision_class_w),
        .vision_valid (vision_valid_w),
        .steer        (steer_w),
        .target_x     (vr_target_x),
        .target_conf  (vr_target_conf),
        .vision_bad   (vr_bad),
        .frame_cnt    (vr_frame_cnt),
        .bad_cnt      (vr_bad_cnt)
    );

    //------------------------------------------------------------------
    // config_rx : 从 0x04 CONFIG 帧解析运行时可配参数
    //
    //   为什么要它：以前"避障距离/最大速度"都是编译期 parameter，
    //   实车标定时改一个数就要重新综合+烧录（还掉电就没了）。
    //   有了它，ARM 发一帧就能改，**不用重烧**。
    //
    //   ⚠️ 一帧只放一个参数：[ID][LEN][VALUE...]，LEN 必须 = 2 + 值长度。
    //      协议见 doc/uart_frame_protocol.md §3.2。
    //------------------------------------------------------------------
    config_rx #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .DEF_VREF         (VREF_FULL),
        .DEF_DIST_STOP_MM (DIST_STOP_MM),
        .DEF_DIST_SLOW_MM (DIST_SLOW_MM),
        .DEF_LIFT_HEIGHT  (8'd100),
        .DEF_LIFT_TMO_MS  (LIFT_TIMEOUT_MS),
        .TYPE_CONFIG      (8'h04)
    ) u_config_rx (
        .clk             (sys_clk),
        .rst_n           (rst_n),
        .rx_valid        (uf_rx_valid),
        .rx_type         (uf_rx_type),
        // uart_frame 的 rx_len 是 8 位、这里端口是 16 位：显式零扩展，
        // 别写成 uf_rx_len[15:8]（那位恒 0，所有 2 字节参数都会被判长度非法）
        .rx_len          ({8'd0, uf_rx_len}),
        .rx_pl_data      (uf_rx_pl_data),
        .rx_pl_addr      (cfg_rx_pl_addr),
        .vref            (cfg_vref),
        .dist_stop_mm    (cfg_dist_stop_mm),
        .dist_slow_mm    (cfg_dist_slow_mm),
        .lift_height     (cfg_lift_height),
        .lift_timeout_ms (cfg_lift_timeout_ms),
        .manual_sel      (cfg_manual_sel),
        .stance_low      (cfg_stance_low),
        .run_cmd         (cfg_run_cmd),
        .run_ms          (cfg_run_ms),
        .cfg_valid       (cfg_valid_w),
        .cfg_bad         (cfg_bad_w),
        .cfg_cnt         (cfg_cnt_w),
        .cfg_bad_cnt     (cfg_bad_cnt_w),
        .cfg_ign_cnt     (cfg_ign_cnt_w)
    );

    // 手动模式：由 CONFIG 帧的 0x20 参数控制（板上没有物理手动开关）
    // ★ 2026-11-03 自主版：手动模式取消，强制自动（CONFIG 0x20 保留但不再生效）
    assign manual_sel = 1'b0;

    //==================================================================
    // ★★ 2026-11-03：网页【开始 / 停止】命令（CONFIG 0x22）
    //
    //   演示版把遥控面板简化成只有开始/停止，全程自动跑：
    //     开始(run_cmd=1)：解除 estop，并且每 500ms 发一个 start 脉冲。
    //       第一次脉冲把状态机从 ESTOP 拉回 IDLE，
    //       第二次脉冲把 IDLE 拉进 AUTO —— 不用按两次按钮。
    //     停止(run_cmd=0)：保持 estop 电平，车停在原地。
    //
    //   物理按键仍然有效（和网页命令是"或"的关系），不影响原有调试习惯。
    //==================================================================
    // ★ 2026-11-03 修：原来每 500ms 发一个 start 脉冲，会导致
    //   ① 按急停后 0.5~1s 自动回 AUTO（违反"急停必须人工再按"）
    //   ② 到终点后 0.5s 又自己出发
    //   现在改成：【run_cmd 上升沿】连发 2 个脉冲（ESTOP->IDLE->AUTO），
    //   之后不再发。要重启必须重新点【开始】。
    reg        run_d;              // run_cmd 的历史值（边沿检测）
    reg  [31:0] run_tmo;
    reg  [1:0] pulse_left;         // 还剩几个 start 脉冲要发
    reg         start_cfg;

    wire run_rise = cfg_run_cmd & ~run_d;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            run_d      <= 1'b0;
            run_tmo    <= 32'd0;
            pulse_left <= 2'd0;
            start_cfg  <= 1'b0;
        end else begin
            run_d <= cfg_run_cmd;

            if (run_rise) begin
                // 刚点【开始】：立刻发第 1 个脉冲，并预约第 2 个
                pulse_left <= 2'd1;
                run_tmo    <= 32'd0;
                start_cfg  <= 1'b1;
            end else if ((pulse_left != 2'd0) && (run_tmo >= 32'd9_999_999)) begin
                // 200ms 后发第 2 个脉冲
                run_tmo    <= 32'd0;
                pulse_left <= pulse_left - 2'd1;
                start_cfg  <= 1'b1;
            end else begin
                start_cfg <= 1'b0;
                if (pulse_left == 2'd0) run_tmo <= 32'd0;
                else                    run_tmo <= run_tmo + 32'd1;
            end
        end
    end

    assign start_pulse = (key0_d & ~key[0]) | start_cfg;
    assign estop_sig   = (ESTOP_FROM_KEY ? ~key[1] : 1'b0) | ~cfg_run_cmd;

    //==================================================================
    // ★ 手动遥控（0x05 RC_CMD）
    //
    //   为什么加在 top 层而不是改 motion_decision：
    //     motion_decision 的输入本来就是 motor_enable / speed_scale / steer
    //     三个"意图"信号，手动和自动只是**谁来给这三个信号**的区别。
    //     在这里做一层选择器，motion_decision / motor_guard 一行都不用改，
    //     也就不会破坏它们已经验证过的行为。
    //
    //   ★ 2026-11-03 自主版：本节描述的手动遥控接管已删除。
    //     现在电机使能只由 main_fsm 决定（急停仍可立即断使能）。
    //==================================================================

    //------------------------------------------------------------------
    // telemetry_tx : 10Hz 上报状态给 ARM
    //
    //   姿态角暂时给 0 —— IMU 还没接入。等 imu_driver 做好后
    //   把这三个 0 换成真实信号即可，不用改 telemetry_tx。
    //
    //   速度：轮子编号 0=左前 1=右前 2=左后 3=右后
    //         → 左侧 = (左前+左后)/2，右侧 = (右前+右后)/2
    //
    //   ⚠️ 原来直接送 speed_cnt[0]/speed_cnt[1]（左前/右前）。
    //      在"单轮代表一侧"的意义上不算错，但：
    //        * 原注释写"代表左右侧"，容易被理解成侧平均；
    //        * 差速底盘两侧各有两轮，**侧速度取平均才是正解**，
    //          否则某一路打滑时上报值会跳。
    //      改成平均。算术右移 >>> 1 保号（负数不会变成大正数）。
    //------------------------------------------------------------------
    wire signed [23:0] spd_l_avg = (speed_cnt[0] + speed_cnt[2]) >>> 1;
    wire signed [23:0] spd_r_avg = (speed_cnt[1] + speed_cnt[3]) >>> 1;

    // ★ 遥测标志位（先声明，赋值在 motion_decision 之后 —— 那里才有 limited/spd_stale）
    wire        mot_spd_stale;
    wire [3:0]  telem_flags;

    telemetry_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .TX_RATE_HZ  (TELEM_RATE_HZ)
    ) u_telemetry_tx (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .state        (state),
        .flags        (telem_flags),
        .spd_l        (spd_l_avg),
        .spd_r        (spd_r_avg),
        .dist_mm      (dist_mm),
        .roll         (imu_roll),      // IMU_ENABLE=0 时是常量 0（见 §1.6）
        .pitch        (imu_pitch),
        .yaw          (imu_yaw),
        .tx_pl_wr     (tt_pl_wr),
        .tx_pl_addr   (tt_pl_addr),
        .tx_pl_data   (tt_pl_data),
        .tx_req       (tt_req),
        .tx_type      (tt_type),
        .tx_len       (tt_len),
        .tx_busy      (uf_tx_busy),
        .tx_done      (uf_tx_done),
        .sent_cnt     (tt_sent_cnt),
        .dropped_cnt  (tt_dropped_cnt),
        .wait_timeout (tt_wait_timeout)
    );

    //------------------------------------------------------------------
    // UART 统计：不再是顶层端口（64 位引脚板子上放不下），
    // 保留为**内部 wire**，供仿真用层次引用读取。
    //
    //   ⚠️ 不要"因为看起来没人用"就把它们删掉 ——
    //      tb_top_integration 场景 12 会通过 dut.uf_frame_ok 读它们。
    //      仿真时 compile_all.do 带 -voptargs="+acc"，优化器不会删。
    //------------------------------------------------------------------
    wire [15:0] uart_rx_ok_cnt   = uf_frame_ok;
    wire [15:0] uart_rx_err_cnt  = uf_frame_err;
    wire [15:0] vision_frame_cnt = vr_frame_cnt;
    wire [15:0] vision_bad_cnt   = vr_bad_cnt;

    //==================================================================
    // 4) 主状态机: 行为决策
    //==================================================================
    // state 的声明已提前到 §3.5（telemetry_tx 要用）
    wire       motor_enable_fsm;
    wire [1:0] speed_scale;

    //------------------------------------------------------------------
    // ★ 2026-11-03 自主版：没有手动模式，三个选择器固定走自动分支。
    //   steer / speed_scale / motor_enable 全部由 main_fsm + vision_rx 决定。
    //   放在这里是因为 motor_enable_fsm / speed_scale 在上面才声明。
    //------------------------------------------------------------------
    wire signed [15:0] steer_sel;
    wire [1:0]         speed_scale_sel;
    wire               motor_enable_sel;

    // ★ 2026-11-03：自动模式下绕行阶段用 main_fsm 的 turn_dir 覆盖视觉转向
    //   turn_dir: 0=不转 1=左转 2=右转  ->  steer: 负=左 正=右
    wire signed [15:0] steer_auto = (turn_dir_fsm == 2'd1) ? -16'sd1600 :
                                    (turn_dir_fsm == 2'd2) ?  16'sd1600 : steer_w;

    // ★ 2026-11-03 自主版：没有手动模式，全部由状态机决定
    assign steer_sel        = steer_auto;
    assign speed_scale_sel  = speed_scale;
    assign motor_enable_sel = motor_enable_fsm;

    // ★ 2026-11-03 自主版：固定 100% + 前进（速度档由 main_fsm 的 speed_scale 管）
    wire [7:0] speed_pct_sel;
    wire       dir_inv_sel;
    assign speed_pct_sel = 8'd100;
    assign dir_inv_sel   = dir_inv_fsm;   // ★ 2026-11-03：自主模式下的后退由 main_fsm 给
    wire [2:0] lift_phase;
    wire [1:0] lift_cmd;          // main_fsm -> servo_lift（意图）
    wire [1:0] lift_done;         // servo_lift -> main_fsm（消抖后的到位，高有效）
    wire [1:0] lift_fault;        // servo_lift -> main_fsm（消抖后的故障，高有效）
    wire       avoid_active;
    wire       lift_active;
    // main_fsm 自己的故障位。它和"电机堵转"（motor_guard 的 stall_any）
    // 一起或成对外的 error_flag，见 §7 状态输出。
    //   ⚠️ 必须**显式声明**：漏了就是隐式 1 位网，工具只给个 warning，行为不对。
    wire       fsm_error_flag;

    main_fsm #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .DIST_STOP_MM     (DIST_STOP_MM),
        .DIST_SLOW_MM     (DIST_SLOW_MM),
        .DIST_CLEAR_MM    (DIST_CLEAR_MM),
        .AVOID_TIMEOUT_MS (AVOID_TIMEOUT_MS),
        .LIFT_TIMEOUT_MS  (LIFT_TIMEOUT_MS),
        .CROSS_TIMEOUT_MS (CROSS_TIMEOUT_MS),
        .CROSS_MIN_MS     (CROSS_MIN_MS),
        .LIFT_SETTLE_MS   (LIFT_SETTLE_MS),
        .VIS_LOW_OBST     (VIS_LOW_OBST),
        .VIS_OBST         (VIS_OBST),
        .VIS_STEP         (VIS_STEP)
    ) u_main_fsm (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .start        (start_pulse),
        .estop        (estop_sig),
        .manual_sel   (manual_sel),
        .dist_mm      (dist_mm),
        .dist_valid   (dist_valid),
        .dist_err     (dist_err),
        // 运行时配置（来自 config_rx，CONFIG 帧发一帧就能改，不用重烧）
        .dist_stop_mm (cfg_dist_stop_mm),
        .dist_slow_mm (cfg_dist_slow_mm),
        // 抬腿超时（CONFIG 帧 0x11，毫秒；0 = 用默认值）
        .lift_timeout_ms (cfg_lift_timeout_ms),
        .vision_class (vision_class_w),
        .vision_valid (vision_valid_w),
        .vr_target_x  (vr_target_x),     // ★ 2026-11-03：绕行选转向用
        .run_ms       (cfg_run_ms),      // ★ 2026-11-03：绕行直行时长
        .turn_dir     (turn_dir_fsm),    // ★ 2026-11-03：绕行转向输出
        .dir_inv      (dir_inv_fsm),     // ★ 2026-11-03：后退
        .goal_reached (goal_reached),
        .lift_done    (lift_done),
        .lift_fault   (lift_fault),
        .state        (state),
        .motor_enable (motor_enable_fsm),
        .speed_scale  (speed_scale),
        .lift_cmd     (lift_cmd),
        .lift_phase   (lift_phase),
        .avoid_active (avoid_active),
        .lift_active  (lift_active),
        .error_flag   (fsm_error_flag)
    );

    //==================================================================
    // 4.5) 升降腿舵机驱动
    //
    //   把 main_fsm 的"意图"（lift_cmd[1:0]）翻译成舵机能吃的 50Hz PWM，
    //   同时做两件 main_fsm 不该管的事：
    //     * 限位开关消抖（机械触点抖动 1~5ms）
    //     * 极性转换（原始低有效 -> 高有效）
    //
    //   输出 lift_pwm 直接接舵机信号线 —— **不需要外挂舵机驱动板**。
    //
    //   ⚠️ 舵机电源必须独立（5V/≥5A），只把信号线和 GND 接过来。
    //      和 FPGA 共用一路会因堵转压降把 FPGA 拉复位。
    //==================================================================
    servo_lift #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .PWM_PERIOD_US (SERVO_PERIOD_US),
        .PWM_MIN_US    (SERVO_MIN_US),
        .PWM_MAX_US    (SERVO_MAX_US),
        .DEBOUNCE_MS   (SERVO_DEBOUNCE_MS)
    ) u_servo_lift (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .lift_cmd     (lift_cmd),
        // ★ 抬腿高度（CONFIG 帧 0x10，0..100；100 = 与旧行为完全一致）
        .lift_height  (cfg_lift_height),
        .lift_lim_n   (lift_lim_n),
        .lift_fault_n (lift_fault_n),
        .lift_pwm     (lift_pwm),
        .lift_done    (lift_done),
        .lift_fault   (lift_fault)
    );

    //==================================================================
    // 4.6) ★ 舵机走 PCA9685（SERVO_IF=1 时例化，2026-10-26）
    //
    //   为什么放这里，不放到 §1.5：
    //     本块要用 lift_cmd（main_fsm 的输出）和 cfg_lift_height
    //     （config_rx 的输出），那两个在上面才声明。
    //     Verilog-2001 先用后声明会生成隐式 1 位网 —— 很难查。
    //
    //   接线（PCA9685 侧，全免焊）：
    //     VCC → 3V3（★ 绝不能接 5V，否则 I2C 电平变 5V 打死 FPGA）
    //     GND → GND（顶排 6）
    //     SDA → D19（顶排 3）      SCL → C20（顶排 4）
    //     V+  → 5~6V 舵机电源（螺丝端子），V− 再并一根细线到 FPGA GND
    //     舵机插通道：0/1 = 前腿，2/3 = 后腿
    //
    //   通道和 main_fsm 的对应：
    //     lift_cmd[0] → 通道 0、1（前腿）
    //     lift_cmd[1] → 通道 2、3（后腿）
    //==================================================================
    //==================================================================
    // ★ 2026-10-30：站立高度控制（遥控"升高/降低"）
    //   0 状态 = 高位 = 舵机 1.5ms 中立位（装配中立）
    //   降低 = 前腿 +45°（顺时针）、后腿 -45°（逆时针），2 秒缓慢走完
    //==================================================================
    wire [11:0] stance_front;
    wire [11:0] stance_rear;
    wire [7:0]  stance_pct;
    wire        stance_at_low;
    wire        stance_at_high;

    stance_ctrl #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .STEP_MS     (STANCE_STEP_MS),
        .CNT_MID     (STANCE_CNT_MID),
        .CNT_SPAN    (STANCE_CNT_SPAN),
        .INVERT      (STANCE_INVERT)
    ) u_stance (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .target_low (cfg_stance_low),
        .cnt_front  (stance_front),
        .cnt_rear   (stance_rear),
        .pos_pct    (stance_pct),
        .at_low     (stance_at_low),
        .at_high    (stance_at_high)
    );

    generate
        if (SERVO_IF != 0) begin : g_servo_pca

            // 舵机版 I2C 主机（100kHz 够用，10k 上拉也行）
            i2c_master #(
                .CLK_FREQ_HZ (CLK_FREQ_HZ),
                .SCL_HZ      (SERVO_I2C_SCL_HZ),
                .TIMEOUT_MS  (SERVO_I2C_TIMEOUT_MS)
            ) u_servo_i2c (
                .clk         (sys_clk),
                .rst_n       (rst_n),
                .start       (pca_start),
                .dev_addr    (pca_dev_addr),
                .rd          (1'b0),              // 只写不读
                .wr_data     (pca_wr_data),
                .reg_addr_en (pca_reg_addr_en),
                .reg_addr    (pca_reg_addr),
                .rd_data     (),                  // 不用
                .busy        (pca_busy),           // 驱动靠它判断"能不能发下一条"
                .ack_err     (pca_ack_err),
                .done        (pca_done),
                .scl         (i2c_scl),
                .sda         (i2c_sda)
            );

            // PCA9685 驱动 FSM（初始化 + 4 通道写入，只在变化时写）
            pca9685_drv #(
                .CLK_FREQ_HZ (CLK_FREQ_HZ),
                .PWM_HZ      (SERVO_PWM_HZ),
                .DEV_ADDR    (SERVO_PCA_ADDR),
                .COUNT_MIN   (SERVO_COUNT_MIN),
                .COUNT_PER_H (SERVO_COUNT_PER_H),
                .POWERUP_MS  (2),
                .OSC_MS      (1),
                .REAR_L_TRIM (REAR_L_TRIM)
            ) u_pca9685 (
                .clk         (sys_clk),
                .rst_n       (rst_n),
                .lift_cmd    (lift_cmd),
                .lift_height (cfg_lift_height),
                .stance_front(SERVO_OFF != 0 ? 12'd0 : stance_front),
                .stance_rear (SERVO_OFF != 0 ? 12'd0 : stance_rear),
                .start       (pca_start),
                .dev_addr    (pca_dev_addr),
                .wr_data     (pca_wr_data),
                .reg_addr_en (pca_reg_addr_en),
                .reg_addr    (pca_reg_addr),
                .busy        (pca_busy),           // ★ 别漏：驱动靠它等 master 空闲
                .done        (pca_done),
                .ack_err     (pca_ack_err),
                .init_done   (pca_init_done),
                .err_sticky  (pca_err_sticky)
            );
        end
    endgenerate

    //==================================================================
    // 5) 运动决策: 意图 -> 每轮目标速度 (比例闭环)
    //==================================================================
    wire [15:0] duty_cmd [0:3];
    wire        dir_cmd  [0:3];
    wire        limited;
    wire signed [15:0] tgt0, tgt1, tgt2, tgt3;

    motion_decision #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .VREF           (VREF_FULL),
        .KP_SHIFT       (KP_SHIFT),
        .LIMIT_DUTY     (LIMIT_DUTY),
        .ZERO_DEADBAND  (DEADBAND),
        .STEER_COEF0    (STEER_COEF0),
        .STEER_COEF1    (STEER_COEF1),
        .STEER_COEF2    (STEER_COEF2),
        .STEER_COEF3    (STEER_COEF3),
        .STEER_SHIFT    (STEER_SHIFT),
        .FORCE_OPEN_LOOP (FORCE_OPEN_LOOP)
    ) u_motion_decision (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .motor_enable (motor_enable_sel),   // ★ 手动时由遥控接管
        .speed_scale  (speed_scale_sel),    // ★
        .speed_pct    (speed_pct_sel),      // ★ 连续调速 0~100
        .dir_inv      (dir_inv_sel),        // ★ 1=后退

        // 运行时配置：最大速度（CONFIG 帧 ID 0x01）
        .vref         (cfg_vref),
        .steer        (steer_sel),          // ★ 手动时用遥控转向
        .spd_fb0      (speed_cnt[0]),
        .spd_fb1      (speed_cnt[1]),
        .spd_fb2      (speed_cnt[2]),
        .spd_fb3      (speed_cnt[3]),
        .spd_valid    (any_spd_valid),
        .duty0        (duty_cmd[0]),
        .duty1        (duty_cmd[1]),
        .duty2        (duty_cmd[2]),
        .duty3        (duty_cmd[3]),
        .dir0         (dir_cmd[0]),
        .dir1         (dir_cmd[1]),
        .dir2         (dir_cmd[2]),
        .dir3         (dir_cmd[3]),
        .limited      (limited),
        .target0      (tgt0),
        .target1      (tgt1),
        .target2      (tgt2),
        .target3      (tgt3),
        .spd_stale_o  (mot_spd_stale)
    );

    // ★ 遥测标志位：打包进 payload[0] 的高 4 位（帧长仍是 13，协议向后兼容）
    //   [3]=goal_reached  [2]=spd_stale  [1]=limited  [0]=error_flag
    assign telem_flags = {goal_reached, mot_spd_stale, limited, error_flag};

    //==================================================================
    // 5.5) ★ 电机护栏：软启动 + 堵转保护（见参数区的说明）
    //
    //   夹在 motion_decision 和 pwm_motor 之间：
    //     * 软启动：占空比**上升**限速（每 CTRL_MS 最多加 RAMP_STEP），
    //               **下降不限速**（要能立刻减速）；
    //     * 堵转保护：duty 大 + 编码器不动 持续 STALL_MS → 该轮立刻断电，
    //               冷却 COOLDOWN_MS 后放开重试（卡阻还在就断续试探）。
    //     * enable **组合直通**：急停路径一拍都不能慢。
    //==================================================================
    wire [15:0] duty_safe [0:3];
    wire        guard_enable;
    wire [3:0]  stall_flags;
    wire        stall_any;
    wire        ramp_active;

    //------------------------------------------------------------------
    // ★ 电压限制：最终占空比限幅（放在 motor_guard 之前）
    //
    //   夹在 motion_decision 之后、motor_guard 之前，语义是
    //   "**无论控制器想要多大，都不许超过 V_LIMIT_DUTY**"。
    //   放在 motor_guard 之前的好处：软启动斜坡仍然作用在已限幅的
    //   目标上，不会出现"爬到 120 再被砍回 68"的来回抖。
    //------------------------------------------------------------------
    localparam [15:0] V_LIMIT_W = V_LIMIT_DUTY;

    wire [15:0] duty_lim [0:3];

    genvar gi_v;
    generate
        for (gi_v = 0; gi_v < 4; gi_v = gi_v + 1) begin : g_vlim
            assign duty_lim[gi_v] = (duty_cmd[gi_v] > V_LIMIT_W)
                                    ? V_LIMIT_W : duty_cmd[gi_v];
        end
    endgenerate

    motor_guard #(
        .CLK_FREQ_HZ   (CLK_FREQ_HZ),
        .CTRL_MS       (GUARD_CTRL_MS),
        .RAMP_STEP     (GUARD_RAMP_STEP),
        .STALL_DUTY    (GUARD_STALL_DUTY),
        .STALL_SPD_MAX (GUARD_STALL_SPD_MAX),
        .STALL_MS      (GUARD_STALL_MS),
        .COOLDOWN_MS   (GUARD_COOLDOWN_MS),
        // ★ 台架测试开关（见 GUARD_ENABLE 参数的定义处）
        .GUARD_ENABLE  (GUARD_ENABLE)
    ) u_motor_guard (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .enable      (motor_enable_sel),    // ★ 手动时由遥控接管
        .duty0       (duty_lim[0]),
        .duty1       (duty_lim[1]),
        .duty2       (duty_lim[2]),
        .duty3       (duty_lim[3]),
        .spd0        (speed_cnt[0]),
        .spd1        (speed_cnt[1]),
        .spd2        (speed_cnt[2]),
        .spd3        (speed_cnt[3]),
        .spd_valid   (any_spd_valid),
        .enable_out  (guard_enable),
        .duty_safe0  (duty_safe[0]),
        .duty_safe1  (duty_safe[1]),
        .duty_safe2  (duty_safe[2]),
        .duty_safe3  (duty_safe[3]),
        .stall       (stall_flags),
        .stall_any   (stall_any),
        .ramp_active (ramp_active)
    );

    //==================================================================
    // 6) PWM 生成 (4 路)
    //
    //    duty_cmd 是**幅值**, dir_cmd 是方向 —— pwm_motor 只出 1 根方向脚，
    //    第二根（AIN2/BIN2/CIN2/DIN2）由 m_dir2 = ~m_dir 取反生成（TB6612 四路板），
    //    负 duty 表达不了倒车, 所以符号在 motion_decision 里已经拆开。
    //==================================================================
    genvar gj;

    // ★ 电机自测波形（generate 里驱动；先声明再用，避免隐式线网）
    wire test_pwm;
    wire [1:0] test_ch;       // 当前测试的通道 0=A 1=B 2=C 3=D

    generate
        for (gj = 0; gj < 4; gj = gj + 1) begin : g_pwm
            wire pwm_raw;
            wire dir_raw;

            pwm_motor #(
                .CLK_FREQ_HZ (CLK_FREQ_HZ),
                .PWM_FREQ_HZ (PWM_FREQ_HZ),
                .DUTY_GAIN   (DUTY_GAIN),
                .CNT_WIDTH   (16)          // 注意参数名是 CNT_WIDTH
                                           //（不是 DUTY_WIDTH —— 写错时 vlog 不报，
                                           //  要到 vopt/vsim elaboration 才失败，
                                           //  且 vsim 只给一句 "Error loading design"）
            ) u_pwm_motor (
                .clk         (sys_clk),
                .rst_n       (rst_n),
                .enabled     (guard_enable),   // 经护栏（软启动/堵转断电）
                .duty_in     (duty_safe[gj]),  // 护栏限速后的占空比
                .dir_in      (dir_cmd[gj]),
                .pwm_out     (pwm_raw),
                .dir_out     (dir_raw),
                .pwm_cnt     (),
                .duty_actual ()
            );

            // ★ 电机自测模式（MOTOR_TEST_MODE=1）：直接接管输出脚，
            //   彻底绕开状态机/护栏/速度环 —— 只验证"FPGA -> 驱动板 -> 电机"。
            // ★ 逐通道模式：只有当前测试通道有输出
            // ★ 四轮同时测：不再按 test_ch 分时
            // ★ 四轮同时输出
            assign m_pwm[gj] = MOTOR_TEST_MODE ? test_pwm : pwm_raw;
            // ★ DIR_INV_MASK[gj]=1 的通道方向取反（前后轮镜像）
            assign m_dir[gj] = MOTOR_TEST_MODE ? (DIR_INV_MASK[gj] ? 1'b0 : 1'b1)
                                               : (DIR_INV_MASK[gj] ? ~dir_raw : dir_raw);
        end
    endgenerate

    //------------------------------------------------------------------
    // 电机自测波形（只在 MOTOR_TEST_MODE=1 时才综合进去）
    //
    //   2 秒周期：前 1.3 秒输出 40% 占空比的 24.4kHz PWM，后 0.7 秒停。
    //   为什么要"脉冲"而不是一直转：上板时人眼要能分辨"它在动 / 它停了"，
    //   一直转反而看不出是命令生效还是卡住了。
    //
    //   为什么是 40%：12V 电池 x 40% = 4.8V，低于 MG310 额定的 7.4V，
    //   电流小、不发热。真要用满占空比测，改 TEST_DUTY 即可。
    //------------------------------------------------------------------
    generate
        if (MOTOR_TEST_MODE != 0) begin : g_mtest
            reg [26:0] mt_t;
            reg [10:0] mt_p;

            // ★ 2026-10-30：四通道依次轮流，每路 1.5 秒。
            //   上电后 A 转 -> B -> C -> D -> 循环
            //   LED4/LED5 显示当前通道号，直接对上"现在在测哪一路"
            reg [1:0] mt_ch;

            always @(posedge sys_clk or negedge rst_n) begin
                if (!rst_n) begin
                    mt_t  <= 27'd0;
                    mt_p  <= 11'd0;
                    mt_ch <= 2'd0;
                end else begin
                    // ★ A->B->C->D 逐通道，每路 1.5 秒
                    if (mt_t >= 27'd399_999_999) begin
                        mt_t  <= 27'd0;
                        mt_ch <= mt_ch + 2'd1;
                    end else begin
                        mt_t <= mt_t + 27'd1;
                    end
                    // PWM 载波：11 位 = 2048 级 -> 50MHz/2048 = 24.4kHz
                    mt_p <= mt_p + 11'd1;
                end
            end

            localparam [10:0] TEST_DUTY = 11'd1331;           // 1331/2048 = 65% (~7.8V)

            // ★ 连续输出
            assign test_pwm = (mt_p < TEST_DUTY);
            assign test_ch  = mt_ch;
        end else begin : g_mtest_off
            assign test_pwm = 1'b0;
            assign test_ch  = 2'd0;
        end
    endgenerate

    //------------------------------------------------------------------
    // 电机使能: 直接来自 main_fsm（**故意不经过电机护栏**）。
    //   main_fsm 内部已经把 estop 组合进去(0 拍断使能),
    //   这里**不要再做一次**与 estop 的与运算 —— 重复不必要,
    //   而且会让"是谁断的使能"变得难查。
    //   ★ 也正因为它直连，急停的"0 拍"保证不依赖 motor_guard 的实现。
    //     护栏只通过 pwm_motor 的 enabled/duty_in 起作用（见 §5.5）。
    //   ★ MOTOR_TEST_MODE=1 时强制拉高（否则状态机在 IDLE，电机不会动）。
    //------------------------------------------------------------------
    // ★ 2026-10-29：原来接的是 motor_enable_fsm（状态机使能），
    //   ★ 2026-11-03 自主版：m_en 恒跟 motor_enable_sel（= main_fsm 的使能），
    //   急停时 0 拍断使能；不再有任何遥控超时逻辑。
    assign m_en = MOTOR_TEST_MODE ? 1'b1 : motor_enable_sel;

    //------------------------------------------------------------------
    // 第二方向脚：和 m_dir 互补
    //
    //   TB6612 要两个反相的输入才能正反转（见 §6 的端口注释）。
    //   ⚠️ MOTOR_TEST_MODE=1 时 m_dir 被强制为 1，所以 m_dir2 也随之变 0
    //      —— 正好是"正转"的组合 ✓ 自测模式和正常模式都自洽。
    //------------------------------------------------------------------
    assign m_dir2 = ~m_dir;


    //==================================================================
    // 7) 状态输出
    //==================================================================
    // 测试模式下 LED4/LED5 显示当前通道号（0=A 1=B 2=C 3=D）
    assign state_dbg       = MOTOR_TEST_MODE ? {1'b0, test_ch} : state;
    assign speed_scale_dbg = speed_scale;

    //==================================================================
    // 8) LED 指示 (上板调试按这个顺序点亮)
    //    led[0] = 复位释放, 板子活着
    //    led[1] = 电机使能中
    //    led[2] = 正在避障或越障
    //    led[3] = 故障 (FSM 安全停车超时/舵机故障 **或 电机堵转**)
    //
    //   ⚠️ 2026-10-18 排查时曾临时改成 `tt_sent_cnt[0]`（telemetry 心跳），
    //      已改回 rst_n。那个诊断本身很有用（"telemetry 到底发没发"一眼看出），
    //      需要时把下面这行换掉即可。
    //==================================================================
    assign led[0] = rst_n;
    assign led[1] = motor_enable_fsm;
    assign led[2] = avoid_active | lift_active;
    generate
        if (SERVO_IF != 0) begin : g_led3_pca
            assign led[3] = pca_init_done;
        end else begin : g_led3_err
            assign led[3] = error_flag;
        end
    endgenerate   // ★★ motion_decision 的 dir2（亮=正转 灭=反转）   // ★★ C 通道 IN1 方向脚（亮=正转 灭=反转）   // ★★ tgt2 符号位（亮=负 灭=正）   // ★★ C 方向（验证修复：右转时应变灭）   // ★★ 调试：亮=转向≈1600 灭=转向≈100/0   // ★★ 调试：C 通道方向（亮=正转 灭=反转）
    // 故障指示：FSM 的故障位 + 电机护栏的堵转位。
    //   ⚠️ 这里是**或**，所以不影响 tb_top_integration 里"急停时 error_flag=0"那些检查
    //      （那些场景里 stall_any 恒 0，结果与以前一致）。
    assign error_flag = fsm_error_flag | stall_any;
    // ★ 2026-10-29 调试：SERVO_IF=1 时 led[3] 显示 PCA9685 初始化是否成功
    //   D3 亮 = I2C 通了、PCA9685 初始化完成（能出 PWM）
    //   D3 灭 = I2C 没通 / 地址不对 / 上拉缺失
    // led[3] 改用作 steer 调试

    //==================================================================
    // 9) 待接入模块的显式 TODO (全部"看得见", 避免假集成)
    //==================================================================
    // ✅ uart_frame + vision_rx + telemetry_tx : 已接入（见 §3.5）
    // ✅ telemetry 标志位  : goal_reached / spd_stale / limited / error
    //                       打包进 payload[0] 高 4 位（帧长仍 13，向后兼容）
    // ✅ tof_driver        : 已接入（见 §1.5；TOF_ENABLE=0 时退回 DIST_STUB_MM 桩）
    // ✅ imu_driver        : 已接入（见 §1.6；IMU_ENABLE=0 时关闭，姿态恒 0）
    // ✅ i2c_bus_arb       : TOF/IMU 共享总线的授权仲裁（IMU_ENABLE=1 时才例化）
    // ✅ servo_lift        : 已接入（lift_cmd/lift_phase -> 舵机 PWM，
    //                       lift_done/lift_fault 消抖后回 main_fsm）
    // ✅ spd_stale         : motion_decision 内实现"测速超时 -> 降级开环"，并上报遥测
    // ✅ clk_gen           : 已接入（默认旁路 50MHz；填好 PLL 后改 PLL_BYPASS=0）
    // ⬜ TODO(imu_attitude): roll/pitch/yaw 仍是常量 0
    //                       （imu_driver 只出原始加减速/角速度，缺姿态解算）
    // ⬜ TODO(display_ctrl): 7 段数码管驱动 —— **缺数码管引脚表**，暂做不了
    // ⬜ TODO(manual_sel)  : 遥控/手动通道选择（可走 CONFIG 帧，协议里还没有）
    //
    // 说明: limited / target0..3 / pos_cnt / enc_dir / spd_valid / lift_phase
    //       目前只连到内部或暂时没人消费。仿真阶段**不需要**做任何抑制 ——
    //       早先这里放过一句
    //           wire _unused = &{1'b0, limited, ...};
    //       虽然是常见写法，但它在这个文件里引入了不必要的 elaboration 风险，
    //       去掉了。综合时的 unused 警告无害，不用为了消除警告增加风险。

endmodule
