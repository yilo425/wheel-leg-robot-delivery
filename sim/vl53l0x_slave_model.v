//=============================================================================
// module : vl53l0x_slave_model
// layer  : 仿真模型（**不可综合**，只给 testbench 用）
//-----------------------------------------------------------------------------
// 用途 : 顶层的 TOF 通路验证（sim/tb_top_tof_i2c.v 使用）。
//
//   它把一个 VL53L0X 的 I2C 从机行为建模出来，让
//
//       top §1.5 ──► tof_driver ──► i2c_master ──► i2c_scl / i2c_sda
//                                                            │
//                                            vl53l0x_slave_model（本文件）
//
//   这条链路在仿真里**真的**跑起来，而不是靠 force 灌一个常数距离。
//   这是"TOF 集成进 top.v"唯一能自证的方式：
//   驱动有没有被例化、引脚有没有连对、dist_mm 有没有真的接到 main_fsm。
//
//-----------------------------------------------------------------------------
// 支持的传输（和 i2c_master.v 头注释里的 A~D 一一对应）
//-----------------------------------------------------------------------------
//   B) 写寄存器     START | dev+W | reg | data | STOP        （初始化表 / 启动 / 清中断）
//   C) 复合读       START | dev+W | reg | 重复START | dev+R | data | NACK | STOP
//                                                          （读 0x13 / 0x1E / 0x1F）
//   A/D 本项目没有用到，模型里也走同一条路径（不会出错）。
//
//   ⚠️ 复合读必须靠**重复 START**（不是 STOP+START）——
//      真实器件 STO P 之后会把内部寄存器指针复位，读出来就是错的。
//      本模型用 rstart_cnt 把它记下来，tb 直接断言 > 0。
//
//-----------------------------------------------------------------------------
// 与 tb_tof_driver.v 里那个从机模型的关系
//-----------------------------------------------------------------------------
//   tb_tof_driver.v 里的从机模型是**为单模块验证定制的**：带时钟拉伸、
//   NACK 注入、SDA 保持时间违规统计等一堆开关，而且和那个 tb 的检查项
//   紧耦合。本模型只保留"顶层集成"需要的最小行为，两者互不影响 ——
//   改这边不会动摇已经跑过的 43 项驱动自检，也不会被那边的改动带崩。
//
//-----------------------------------------------------------------------------
// tb 怎么用它
//-----------------------------------------------------------------------------
//   1) 例化后 tb 通过层次引用改这些量（需要 vsim -voptargs="+acc"，
//      compile_all.do 里一直带着）：
//          u_tof.range_mm    = 16'd150;    // 下一次"测到"的距离
//          u_tof.ready_after = 1;          // 第几次读 0x13 之后才就绪
//   2) 断言这些统计量：
//          regs[8'h88] / regs[8'h44] / regs[8'h45]  初始化表真的写进来了
//          regs[8'h00]==8'h01                       启动过单次测距
//          ptr_13_cnt / ptr_1e_cnt / ptr_1f_cnt     真的去读了状态和距离
//          rstart_cnt > 0                           复合读用了重复 START
//          hold_err == 0                            SCL 高电平期间 SDA 没变过
//
//   ⚠️ 本模型内部**自带上拉**（pullup 原语）。仿真里没有上拉时，开漏线
//      释放后停在 z，两边都会读到 x，表现成"什么都不动"。
//=============================================================================
`timescale 1ns/1ps

module vl53l0x_slave_model #(
    //----------------------------------------------------------------
    // 7 位从机地址（VL53L0X 出厂默认 0x29）
    //----------------------------------------------------------------
    parameter [6:0] DEV_ADDR = 7'h29
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
    // VL53L0X 寄存器地址
    //================================================================
    localparam [7:0] REG_START     = 8'h00;   // 写 0x01 = 启动单次测距
    localparam [7:0] REG_CLR_INT   = 8'h0B;   // 写 0x01 = 清中断
    localparam [7:0] REG_STATUS    = 8'h13;   // bit0 = 数据就绪
    localparam [7:0] REG_RANGE_MSB = 8'h1E;   // 距离高字节
    localparam [7:0] REG_RANGE_LSB = 8'h1F;   // 距离低字节

    //================================================================
    // 器件寄存器模型（普通寄存器就是一块 RAM）
    //================================================================
    reg [7:0]  regs [0:255];
    integer    idx;

    initial begin
        for (idx = 0; idx < 256; idx = idx + 1) regs[idx] = 8'h00;
    end

    //================================================================
    // 行为参数（tb 用层次引用改）
    //================================================================
    reg [15:0] range_mm    = 16'd1234;   // 下一次"测到"的距离
    reg [15:0] meas_range  = 16'd1234;   // 启动测距时锁存的测量值
    integer    ready_after = 1;          // 第几次读 0x13 之后 bit0 才置 1
    integer    poll_cnt    = 0;

    //================================================================
    // 统计量（tb 断言用）
    //================================================================
    integer start_cnt      = 0;    // 总线上出现的 START（含重复 START）
    integer rstart_cnt     = 0;    // 其中的重复 START 次数
    integer stop_cnt       = 0;
    integer addr_w_cnt     = 0;    // 收到 地址+W
    integer addr_r_cnt     = 0;    // 收到 地址+R
    integer addr_bad_cnt   = 0;    // 地址不匹配（不该出现）
    integer hold_err       = 0;    // ★SCL 高电平期间 SDA 变化 = I2C 违规
    integer ptr_13_cnt     = 0;    // 写过寄存器指针 0x13 的次数（轮询次数）
    integer ptr_1e_cnt     = 0;
    integer ptr_1f_cnt     = 0;
    integer start_meas_cnt = 0;    // 收到过几次"启动单次测距"

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
    task write_hook;
        begin
            // 0x00 = 0x01：启动一次单次测距 -> 锁存测量值、清轮询计数
            if ((s_ptr == REG_START) && (s_shift == 8'h01)) begin
                poll_cnt       = 0;
                meas_range     = range_mm;
                start_meas_cnt = start_meas_cnt + 1;
            end
        end
    endtask

    // 装载读方向的下一字节（器件内部指针自动递增）
    task prep_read_byte;
        begin
            case (s_ptr)
                REG_STATUS: begin
                    poll_cnt = poll_cnt + 1;
                    s_tx     = (poll_cnt >= ready_after) ? 8'h01 : 8'h00;
                end
                REG_RANGE_MSB: s_tx = meas_range[15:8];
                REG_RANGE_LSB: s_tx = meas_range[7:0];
                default:       s_tx = regs[s_ptr];
            endcase
            s_ptr = s_ptr + 8'd1;
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
                    if (s_ptr == REG_STATUS)    ptr_13_cnt = ptr_13_cnt + 1;
                    if (s_ptr == REG_RANGE_MSB) ptr_1e_cnt = ptr_1e_cnt + 1;
                    if (s_ptr == REG_RANGE_LSB) ptr_1f_cnt = ptr_1f_cnt + 1;
                end else begin
                    regs[s_ptr] = s_shift;         // 写数据（指针自动递增）
                    write_hook;
                    s_ptr = s_ptr + 8'd1;
                end
            end
            // 读方向这里不用做什么：主机 ACK 了就在下降沿继续发下一位
        end
    endtask

    // tb 里强制把从机模型放回空闲（例如测"总线被卡住"之后复位用）
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
            if (s_rd) slave_sda_low = 1'b0;        // 读：等主机 ACK/NACK
            // ★ 写方向：**只有地址命中的从机才拉低 SDA**。
            //   一条总线上挂多块器件时（TOF + MPU6050），各自只回应自己的地址，
            //   别人的事务必须完全不理会 —— 否则两块从机会同时应答、同时被写寄存器，
            //   读回的数据就是错乱的。单器件时这条恒为真，行为和以前一样。
            //   注意地址字节的 ACK 发生在 s_match 更新（byte_done）之前，
            //   所以第一字节要直接比较 s_shift。
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
