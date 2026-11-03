//=============================================================================
// module : mpu6050_slave_model
// layer  : 仿真模型（**不可综合**，只给 testbench 用）
//-----------------------------------------------------------------------------
// 用途 : MPU6050 驱动的单模块验证（sim/tb_imu_driver.v 使用）。
//
//   它把一个 MPU6050 的 I2C 从机行为建模出来，让
//
//       tb ──► imu_driver ──► i2c_master ──► scl / sda
//                                                  │
//                                  mpu6050_slave_model（本文件）
//
//   这条链路在仿真里真的跑起来：寄存器指针、复合读、ACK/NACK、指针自增，
//   都是按 I2C 时序一位一位走出来的，不是"函数调用"。
//
//-----------------------------------------------------------------------------
// 支持的传输（和 i2c_master.v 头注释里的 A~D 一一对应）
//-----------------------------------------------------------------------------
//   B) 写寄存器     START | dev+W | reg | data | STOP        （初始化表）
//   C) 复合读       START | dev+W | reg | 重复START | dev+R | data | NACK | STOP
//                                                          （读 0x3B..0x48）
//   A/D 本项目没有用到，模型里也走同一条路径（不会出错）。
//
//   ⚠️ 复合读必须靠**重复 START**（不是 STOP+START）——
//      真实器件在 STOP 之后会把内部寄存器指针复位，读出来就是错的。
//      本模型用 rstart_cnt 把它记下来，tb 直接断言 > 0。
//
//-----------------------------------------------------------------------------
// 与 vl53l0x_slave_model.v 的关系
//-----------------------------------------------------------------------------
//   本文件是照着 sim/vl53l0x_slave_model.v 改的（那个已被 tb_top_tof_i2c.v
//   验证可用）。I2C 从机那套状态机（s_active / s_sc / s_shift / s_ptr …）
//   原样保留，只换了两处：
//     1) 寄存器地址表 + 数据来源（加速度/陀螺 6 轴而不是单个距离）；
//     2) 多了 no_ack_mode 开关（tb 要测"从机不应答 -> imu_err"）。
//   这样做的好处：I2C 那部分逻辑已经在 TOF 那边验证过，这里的风险只在
//   "寄存器映射写错"，而那是 tb 能直接查出来的。
//
//-----------------------------------------------------------------------------
// tb 怎么用它（层次引用需要 vsim -voptargs="+acc"，工程一直带着）
//-----------------------------------------------------------------------------
//   1) 改"传感器读数"（有符号 16 位原始值，和真实器件一样）：
//          u_imu.ax = -16'sd8192;   // -0.5g
//          u_imu.gz =  16'sd1310;   // 10 °/s
//   2) 控故障注入：
//          u_imu.no_ack_mode = 1;   // 从机不应答 -> 主机 ack_err -> imu_err
//   3) 断言这些统计量：
//          regs[0x6B]/[0x1C]/[0x1B]/[0x19]   初始化表真的写进来了
//          ptr_3b_cnt / ptr_43_cnt            真的去读了加速度/陀螺
//          rd_byte_cnt                        读方向真的移出过数据字节
//          rstart_cnt > 0                     复合读用了重复 START
//          addr_bad_cnt == 0                  地址没错过
//          hold_err == 0                      ★SCL 高电平期间 SDA 没变过
//
//   ⚠️ 本模型内部**自带上拉**（pullup 原语）。仿真里没有上拉时，开漏线
//      释放后停在 z，两边都会读到 x，表现成"什么都不动"。
//=============================================================================
`timescale 1ns/1ps

module mpu6050_slave_model #(
    //----------------------------------------------------------------
    // 7 位从机地址（MPU6050 的 AD0=0 时是 0x68）
    //----------------------------------------------------------------
    parameter [6:0] DEV_ADDR = 7'h68
) (
    inout wire scl,
    inout wire sda
);

    //================================================================
    // 上拉：板子上是 4.7k 到 3.3V
    //================================================================
    pullup pu_scl (scl);
    pullup pu_sda (sda);

    //================================================================
    // 从机侧开漏驱动：只会"拉低"或"释放"，永不强驱高
    //================================================================
    reg slave_sda_low = 1'b0;

    assign sda = slave_sda_low ? 1'b0 : 1'bz;

    //================================================================
    // MPU6050 寄存器地址（只列本模型用到的）
    //================================================================
    localparam [7:0] REG_SMPLRT_DIV   = 8'h19;
    localparam [7:0] REG_GYRO_CONFIG  = 8'h1B;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;   // 0x3B..0x40
    localparam [7:0] REG_GYRO_XOUT_H  = 8'h43;   // 0x43..0x48
    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;
    localparam [7:0] REG_WHO_AM_I     = 8'h75;

    //================================================================
    // 器件寄存器模型（配置寄存器就是一块 RAM；数据寄存器见下面）
    //================================================================
    reg [7:0]  regs [0:255];
    integer    idx;

    initial begin
        for (idx = 0; idx < 256; idx = idx + 1) regs[idx] = 8'h00;
        regs[REG_WHO_AM_I] = 8'h68;      // MPU6050 的 WHO_AM_I 固定读回 0x68
    end

    //================================================================
    // 传感器读数（tb 用层次引用改；有符号 16 位，和真实器件一致）
    //   默认给一个"平放静止"的合理值：az = 1g，其余为 0
    //================================================================
    reg signed [15:0] ax = 16'sd0;
    reg signed [15:0] ay = 16'sd0;
    reg signed [15:0] az = 16'sd16384;   // 1g
    reg signed [15:0] gx = 16'sd0;
    reg signed [15:0] gy = 16'sd0;
    reg signed [15:0] gz = 16'sd0;

    //================================================================
    // 行为开关（tb 用层次引用改）
    //================================================================
    integer no_ack_mode = 0;      // 1 = 从机不应答（测 NACK -> imu_err）

    //================================================================
    // 统计量（tb 断言用）
    //================================================================
    integer start_cnt    = 0;    // 总线上出现的 START（含重复 START）
    integer rstart_cnt   = 0;    // 其中的重复 START 次数
    integer stop_cnt     = 0;
    integer addr_w_cnt   = 0;    // 收到 地址+W
    integer addr_r_cnt   = 0;    // 收到 地址+R
    integer addr_bad_cnt = 0;    // 地址不匹配（不该出现）
    integer hold_err     = 0;    // ★SCL 高电平期间 SDA 变化 = I2C 违规
    integer ptr_3b_cnt   = 0;    // 寄存器指针写过 0x3B 的次数（读加速度）
    integer ptr_43_cnt   = 0;    // 寄存器指针写过 0x43 的次数（读陀螺）
    integer rd_byte_cnt  = 0;    // 读方向真正移出去的**数据字节**数
    integer wr_reg_cnt   = 0;    // 写方向的配置寄存器写次数

    //================================================================
    // 从机内部状态
    //================================================================
    reg        s_active     = 1'b0;   // START 之后、STOP 之前
    reg        s_mon        = 1'b0;   // 协议监视窗口（START/STOP 期间关掉）
    reg        s_start_pend = 1'b0;
    reg [7:0]  s_shift      = 8'h00;
    reg [3:0]  s_sc         = 4'd0;   // 本字节已过的 SCL 上升沿数
    reg        s_first      = 1'b1;   // 还没收到本次会话的地址字节
    reg        s_rd         = 1'b0;   // 当前方向 1 = 从机发
    reg        s_match      = 1'b0;   // 地址命中
    reg [7:0]  s_ptr        = 8'h00;  // 寄存器指针
    reg        s_ptr_pend   = 1'b0;   // 下一个写字节是寄存器指针
    reg [7:0]  s_tx         = 8'hFF;  // 读方向移位输出
    reg        s_ack_seen   = 1'b0;   // 主机对读字节的 ACK/NACK
    reg        s_at_rise    = 1'b1;   // SCL 上升沿时的 SDA（稳定性检查用）

    //================================================================
    // 辅助任务
    //================================================================
    // 写寄存器时器件内部的副作用
    //   MPU6050 这边没有"写完某寄存器要额外做什么"的必要
    //   （唤醒/量程都是写进去就生效），所以这里只计数，
    //   留给 tb 判断"初始化表到底写了几条"。
    task write_hook;
        begin
            wr_reg_cnt = wr_reg_cnt + 1;
        end
    endtask

    // 装载读方向的下一字节（器件内部指针自动递增）
    //
    //   数据寄存器 0x3B..0x40 / 0x43..0x48 全部由 ax/ay/az/gx/gy/gz 现拼，
    //   这样 tb 改一个 16 位值就等于"传感器读到了新数据"。
    //   0x41/0x42 是温度：给个固定 0，驱动器不读它。
    task prep_read_byte;
        begin
            case (s_ptr)
                8'h3B: s_tx = ax[15:8];
                8'h3C: s_tx = ax[7:0];
                8'h3D: s_tx = ay[15:8];
                8'h3E: s_tx = ay[7:0];
                8'h3F: s_tx = az[15:8];
                8'h40: s_tx = az[7:0];
                8'h41: s_tx = 8'h00;          // 温度高字节（未建模）
                8'h42: s_tx = 8'h00;          // 温度低字节（未建模）
                8'h43: s_tx = gx[15:8];
                8'h44: s_tx = gx[7:0];
                8'h45: s_tx = gy[15:8];
                8'h46: s_tx = gy[7:0];
                8'h47: s_tx = gz[15:8];
                8'h48: s_tx = gz[7:0];
                default: s_tx = regs[s_ptr];
            endcase
            rd_byte_cnt = rd_byte_cnt + 1;
            s_ptr = s_ptr + 8'd1;      // 器件内部指针自动递增
        end
    endtask

    // 一个字节（含 ACK 拍）收完之后的协议处理
    task byte_done;
        begin
            if (s_first) begin
                //---------------- 地址字节 ----------------
                s_first = 1'b0;
                if (s_shift[7:1] !== DEV_ADDR) begin
                    addr_bad_cnt = addr_bad_cnt + 1;
                    s_match      = 1'b0;
                end else begin
                    s_match = 1'b1;
                    if (s_shift[0] === 1'b1) begin
                        addr_r_cnt = addr_r_cnt + 1;
                        s_rd       = 1'b1;             // 接下来从机发数据
                    end else begin
                        addr_w_cnt = addr_w_cnt + 1;
                        s_rd       = 1'b0;
                        s_ptr_pend = 1'b1;             // 下一个字节是寄存器指针
                    end
                end
            end else if (!s_rd) begin
                //---------------- 写方向的后续字节 ----------------
                if (s_ptr_pend) begin
                    s_ptr      = s_shift;
                    s_ptr_pend = 1'b0;
                    if (s_ptr == REG_ACCEL_XOUT_H) ptr_3b_cnt = ptr_3b_cnt + 1;
                    if (s_ptr == REG_GYRO_XOUT_H)  ptr_43_cnt = ptr_43_cnt + 1;
                end else begin
                    regs[s_ptr] = s_shift;         // 写数据（指针自动递增）
                    write_hook;
                    s_ptr = s_ptr + 8'd1;
                end
            end
            // 读方向这里不用做什么：主机 ACK 了就在下降沿继续发下一位
        end
    endtask

    // tb 里强制把从机模型放回空闲（复位/卡死之后收尾用）
    task clear_bus;
        begin
            slave_sda_low = 1'b0;
            s_active      = 1'b0;
            s_mon         = 1'b0;
            s_sc          = 4'd0;
            s_start_pend  = 1'b0;
        end
    endtask

    //================================================================
    // SCL 上升沿：采样数据位；第 9 拍采样主机的 ACK/NACK
    //================================================================
    always @(posedge scl) begin
        s_at_rise = sda;
        if (s_active) begin
            if (s_sc < 4'd8) begin
                s_shift = {s_shift[6:0], sda};
                s_sc    = s_sc + 4'd1;
            end else if (s_sc == 4'd8) begin
                s_ack_seen = sda;
                s_sc       = 4'd9;
            end
        end
    end

    //================================================================
    // SCL 下降沿：推进字节；读方向在这里把下一位推上总线
    //================================================================
    always @(negedge scl) begin
        // ---- 协议监视：SCL 高电平期间 SDA 不该变（START/STOP 除外）----
        if (s_start_pend) begin
            s_start_pend = 1'b0;
            s_mon        = 1'b1;                  // START 之后的第一个下降沿才开始
        end else if (s_mon) begin
            if (sda !== s_at_rise) hold_err = hold_err + 1;
        end

        // ---- 字节推进 ----
        if (!s_active) begin
            s_sc = 4'd0;
        end else if (s_sc == 4'd8) begin
            // 8 位数据发完，准备第 9 拍（ACK）
            if (s_rd) slave_sda_low = 1'b0;              // 读：等主机 ACK/NACK
            // ★ 写方向：**只有地址命中的从机才拉低 SDA**（共享总线的前提，
            //   见 vl53l0x_slave_model.v 同位置的说明）。
            //   地址字节的 ACK 发生在 s_match 更新之前，所以要直接比较 s_shift。
            else if (no_ack_mode != 0) slave_sda_low = 1'b0;   // 故意 NACK（测试用）
            else if (s_first) slave_sda_low = (s_shift[7:1] === DEV_ADDR) ? 1'b1 : 1'b0;
            else              slave_sda_low = s_match ? 1'b1 : 1'b0;
        end else if (s_sc == 4'd9) begin
            // 一个字节（含 ACK 拍）结束
            byte_done;
            s_sc    = 4'd0;
            s_shift = 8'h00;
            if (s_rd) begin
                if (!s_ack_seen && s_match) begin
                    // 主机 ACK 了 -> 继续发下一个寄存器字节
                    prep_read_byte;
                    slave_sda_low = ~s_tx[7];
                    s_tx          = {s_tx[6:0], 1'b1};
                end else begin
                    slave_sda_low = 1'b0;          // NACK：交还总线
                    s_tx          = 8'hFF;
                end
            end else begin
                slave_sda_low = 1'b0;              // 写方向：释放 SDA
            end
        end else begin
            // s_sc = 0..7：读方向把当前位推上总线
            if (s_rd && s_match) begin
                slave_sda_low = ~s_tx[7];
                s_tx          = {s_tx[6:0], 1'b1};
            end
        end
    end

    //================================================================
    // START / STOP 检测
    //   SCL 高电平期间 SDA 下降 = START，上升 = STOP
    //
    //   ⚠️ #1 是用来错开时刻的：SDA 的驱动和这个监视块在同一时刻事件里，
    //      不加延迟会读到变化前的值（delta cycle 竞争）。
    //================================================================
    always @(sda) begin
        #1;
        if (scl === 1'b1) begin
            if (sda === 1'b0) begin
                //---------------- START / 重复 START ----------------
                start_cnt = start_cnt + 1;
                if (s_active) rstart_cnt = rstart_cnt + 1;
                s_active      = 1'b1;
                s_first       = 1'b1;
                s_match       = 1'b0;
                s_rd          = 1'b0;
                s_sc          = 4'd0;
                s_shift       = 8'h00;
                s_ptr_pend    = 1'b0;
                s_start_pend  = 1'b1;
                slave_sda_low = 1'b0;
            end else begin
                //---------------- STOP ----------------
                stop_cnt      = stop_cnt + 1;
                s_active      = 1'b0;
                s_mon         = 1'b0;
                s_sc          = 4'd0;
                slave_sda_low = 1'b0;
            end
        end
    end

endmodule
