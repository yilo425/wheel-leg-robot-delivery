//=============================================================================
// module : pca9685_slave_model
// layer  : 仿真模型（**不可综合**，只给 testbench 用）
//-----------------------------------------------------------------------------
// 用途 : 验证 top.v 里 SERVO_IF=1 的 PCA9685 通路（sim/tb_top_servo_pca.v）。
//
//   它把一块 PCA9685（I2C 从机，默认地址 0x40）的行为建模出来，让
//
//       top §4.6 ──► pca9685_drv ──► i2c_master ──► i2c_scl / i2c_sda
//                                                          │
//                                        pca9685_slave_model（本文件）
//
//   这条链路在仿真里**真的**跑起来：
//   驱动有没有被例化、I2C 有没有接对、寄存器序列和数值对不对，全都自证。
//
//-----------------------------------------------------------------------------
// 支持的行为
//-----------------------------------------------------------------------------
//   写寄存器（本项目只用到这一种）：
//       START | dev+W | ACK | reg | ACK | data | ACK | STOP
//   读寄存器（驱动不用，但实现完整，方便以后做回读验证）：
//       START | dev+W | ACK | reg | ACK | 重复START | dev+R | ACK | data | NACK | STOP
//
//   寄存器用一块 256 字节 RAM 兜底，另外把关键寄存器单独抓出来：
//       MODE1(0x00) / MODE2(0x01) / PRE_SCALE(0xFE)
//       LEDn_OFF_L/H (0x08/0x09 + 4n) -> 合成 12 位 ch_off[0..3]
//
//-----------------------------------------------------------------------------
// 关键寄存器（和 pca9685_drv.v 对应）
//-----------------------------------------------------------------------------
//   初始化四步：MODE1=0x10 -> PRE_SCALE=121 -> MODE2=0x04 -> MODE1=0x20
//   角度写入  ：每通道 4 字节 LEDn_ON_L=0, ON_H=0, OFF_L=count[7:0], OFF_H=count[11:8]
//   50Hz 下 count = 102 + height*4（0.5ms .. 2.45ms）
//
//-----------------------------------------------------------------------------
// tb 怎么用它
//-----------------------------------------------------------------------------
//   pca9685_slave_model #(.DEV_ADDR(7'h40)) u_slv (.scl(i2c_scl), .sda(i2c_sda));
//
//   tb 直接断言这些量：
//       init_mode1_sleep  MODE1 被写过 0x10（SLEEP）
//       prescale_val      PRE_SCALE 最终值（50Hz -> 121）
//       mode2_val         MODE2 最终值（0x04 = OUTDRV）
//       mode1_val         MODE1 最终值（0x20 = AI + 唤醒）
//       ch_off[0..3]      4 个通道的 12 位 OFF 计数
//       off_wr_cnt        通道 OFF_H 写入次数（每次更新 = 4）
//       addr_bad_cnt      不该出现（地址不匹配）
//       hold_err          SCL 高电平期间 SDA 变化（I2C 违规）
//
//   ⚠️ 本模型内部**自带上拉**（pullup 原语）。没有上拉时开漏线会停在 z，
//      两边都读成 x，表现成"什么都不动"。
//=============================================================================
`timescale 1ns/1ps

module pca9685_slave_model #(
    // 7 位从机地址（PCA9685 出厂默认 0x40；A0~A5 全接地）
    parameter [6:0] DEV_ADDR = 7'h40
) (
    inout wire scl,
    inout wire sda
);

    //================================================================
    // 上拉（板子上是 4.7k 到 3.3V ~ 10k 也行）
    //================================================================
    pullup pu_scl (scl);
    pullup pu_sda (sda);

    //================================================================
    // 从机侧开漏：只拉低或释放
    //================================================================
    reg slave_sda_low = 1'b0;
    assign sda = slave_sda_low ? 1'b0 : 1'bz;

    //================================================================
    // PCA9685 寄存器地址
    //================================================================
    localparam [7:0] REG_MODE1     = 8'h00;
    localparam [7:0] REG_MODE2     = 8'h01;
    localparam [7:0] REG_LED0_ON_L = 8'h06;
    localparam [7:0] REG_PRESCALE  = 8'hFE;

    //================================================================
    // 寄存器 RAM
    //================================================================
    reg [7:0] regs [0:255];
    integer   idx;

    initial begin
        for (idx = 0; idx < 256; idx = idx + 1) regs[idx] = 8'h00;
    end

    //================================================================
    // 抓出来的关键寄存器（tb 断言用）
    //================================================================
    reg [7:0]  mode1_val     = 8'h00;
    reg [7:0]  mode2_val     = 8'h00;
    reg [7:0]  prescale_val  = 8'h00;
    reg [11:0] ch_off [0:3];          // 4 个通道的 12 位 OFF 计数
    integer    i;

    initial begin
        for (i = 0; i < 4; i = i + 1) ch_off[i] = 12'h000;
    end

    //================================================================
    // 统计量
    //================================================================
    integer start_cnt     = 0;
    integer rstart_cnt    = 0;
    integer stop_cnt      = 0;
    integer addr_w_cnt    = 0;
    integer addr_r_cnt    = 0;
    integer addr_bad_cnt  = 0;   // 地址不匹配（不该出现）
    integer hold_err      = 0;   // ★ SCL 高电平期间 SDA 变化 = I2C 违规
    integer wr_cnt        = 0;   // 写数据字节数（不含寄存器地址字节）
    integer mode1_wr_cnt  = 0;
    integer mode2_wr_cnt  = 0;
    integer prescale_wr_cnt = 0;
    integer off_wr_cnt    = 0;   // LEDn_OFF_H 写入次数（每次 4 通 = 4）
    reg     init_mode1_sleep = 1'b0;   // 见过 MODE1=0x10 吗
    reg     init_mode1_wake  = 1'b0;   // 见过 MODE1=0x20 吗

    //================================================================
    // 从机内部状态
    //================================================================
    reg        s_active     = 1'b0;
    reg        s_mon        = 1'b0;
    reg        s_start_pend = 1'b0;
    reg [7:0]  s_shift      = 8'h00;
    reg [3:0]  s_sc         = 4'd0;
    reg        s_first      = 1'b1;
    reg        s_rd         = 1'b0;
    reg        s_match      = 1'b0;
    reg [7:0]  s_ptr        = 8'h00;
    reg        s_ptr_pend   = 1'b0;
    reg [7:0]  s_tx         = 8'hFF;
    reg        s_ack_seen   = 1'b0;
    reg        s_at_rise    = 1'b1;

    //================================================================
    // 写寄存器时器件的副作用
    //================================================================
    task write_hook;
        begin
            case (s_ptr)
                REG_MODE1: begin
                    mode1_val   = s_shift;
                    mode1_wr_cnt = mode1_wr_cnt + 1;
                    if (s_shift == 8'h10) init_mode1_sleep = 1'b1;
                    if (s_shift == 8'h20) init_mode1_wake  = 1'b1;
                end
                REG_MODE2: begin
                    mode2_val    = s_shift;
                    mode2_wr_cnt = mode2_wr_cnt + 1;
                end
                REG_PRESCALE: begin
                    prescale_val      = s_shift;
                    prescale_wr_cnt   = prescale_wr_cnt + 1;
                end
                // LEDn_OFF_H：0x09 / 0x0D / 0x11 / 0x15
                8'h09: begin ch_off[0] = {s_shift[3:0], regs[8'h08]}; off_wr_cnt = off_wr_cnt + 1; end
                8'h0D: begin ch_off[1] = {s_shift[3:0], regs[8'h0C]}; off_wr_cnt = off_wr_cnt + 1; end
                8'h11: begin ch_off[2] = {s_shift[3:0], regs[8'h10]}; off_wr_cnt = off_wr_cnt + 1; end
                8'h15: begin ch_off[3] = {s_shift[3:0], regs[8'h14]}; off_wr_cnt = off_wr_cnt + 1; end
                default: ;
            endcase
        end
    endtask

    // 读方向的下一字节（指针自动递增）
    task prep_read_byte;
        begin
            s_tx  = regs[s_ptr];
            s_ptr = s_ptr + 8'd1;
        end
    endtask

    // 一个字节（含 ACK 拍）收完
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
                        s_rd       = 1'b1;
                    end else begin
                        addr_w_cnt = addr_w_cnt + 1;
                        s_rd       = 1'b0;
                        s_ptr_pend = 1'b1;      // 下一个字节是寄存器指针
                    end
                end
            end else if (!s_rd) begin
                //---------------- 写方向 ----------------
                if (s_ptr_pend) begin
                    s_ptr      = s_shift;
                    s_ptr_pend = 1'b0;
                end else begin
                    regs[s_ptr] = s_shift;     // 先存
                    write_hook;                // 再做副作用
                    wr_cnt = wr_cnt + 1;
                    s_ptr  = s_ptr + 8'd1;
                end
            end
        end
    endtask

    // tb 强制从机回空闲
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
    // SCL 上升沿：采样数据位 / 主机的 ACK
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
    // SCL 下降沿：协议监视 + 字节推进 + 读方向推位
    //================================================================
    always @(negedge scl) begin
        // SCL 高电平期间 SDA 不该变（START/STOP 除外）
        if (s_start_pend) begin
            s_start_pend = 1'b0;
            s_mon        = 1'b1;
        end else if (s_mon) begin
            if (sda !== s_at_rise) hold_err = hold_err + 1;
        end

        if (!s_active) begin
            s_sc = 4'd0;
        end else if (s_sc == 4'd8) begin
            if (s_rd) slave_sda_low = 1'b0;
            else if (s_first) slave_sda_low = (s_shift[7:1] === DEV_ADDR) ? 1'b1 : 1'b0;
            else              slave_sda_low = s_match ? 1'b1 : 1'b0;
        end else if (s_sc == 4'd9) begin
            byte_done;
            s_sc    = 4'd0;
            s_shift = 8'h00;
            if (s_rd) begin
                if (!s_ack_seen && s_match) begin
                    prep_read_byte;
                    slave_sda_low = ~s_tx[7];
                    s_tx          = {s_tx[6:0], 1'b1};
                end else begin
                    slave_sda_low = 1'b0;
                    s_tx          = 8'hFF;
                end
            end else begin
                slave_sda_low = 1'b0;
            end
        end else begin
            if (s_rd && s_match) begin
                slave_sda_low = ~s_tx[7];
                s_tx          = {s_tx[6:0], 1'b1};
            end
        end
    end

    //================================================================
    // START / STOP 检测（SCL 高电平期间 SDA 下降 = START，上升 = STOP）
    //   #1 用来避免和 SDA 驱动同一时刻的 delta 竞争
    //================================================================
    always @(sda) begin
        #1;
        if (scl === 1'b1) begin
            if (sda === 1'b0) begin
                // START / 重复 START
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
                // STOP
                stop_cnt      = stop_cnt + 1;
                s_active      = 1'b0;
                s_mon         = 1'b0;
                s_sc          = 4'd0;
                slave_sda_low = 1'b0;
            end
        end
    end

endmodule
