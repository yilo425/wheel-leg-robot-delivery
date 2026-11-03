//=============================================================================
// module : uart_rx
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : UART 接收, 8N1 / 可选校验位, LSB first, 16 倍过采样中点判决。
//
//------------------------------------------------------------------------------
// 重大坑修复记录（与 uart_tx 同源，务必保留）
//------------------------------------------------------------------------------
// 旧版本用"过采样累加器"，而且公式本身是错的：
//     OS_INC = CLK_FREQ_HZ / (BAUD_RATE*16)      // = 每 54 个 clk 一个 tick
//     os_acc += OS_INC;  溢出 2^16 才算一个 tick
//   这里把"每 tick 多少 clk"（T）直接当成"每 clk 累加多少"用了。
//   正确关系是 inc = M / T（M=2^16 时 inc≈1208），写成 54 就慢了 16 倍。
//   而且同样存在大数乘法溢出风险。
//
// 新版本改用**整数分频计数器**，既没有大数乘法，公式也一目了然：
//     OS_DIV = CLK_FREQ_HZ / (BAUD_RATE * 16)    // 每个过采样 tick 占几个 clk
//     一个比特 = 16 个 tick，在中点（第 8 个 tick）采样
//
// 精度：OS_DIV 是整数除法，100MHz 下 54（理论 54.25），单比特误差 0.47%，
//   一帧 10 比特累计 < 5% 比特宽，远小于 50% 容限。
//   如需零误差，选 CLK_FREQ_HZ 能被 BAUD_RATE*16 整除的组合。
//=============================================================================
`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115_200,
    parameter integer DATA_BITS   = 8,
    parameter integer PARITY_EN   = 0      // 0=无校验 1=偶校验 2=奇校验
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 rx,
    output reg  [DATA_BITS-1:0] data_out,
    output reg                  done,        // 收到完整一字节, 单周期
    output reg                  frame_err,
    output reg                  parity_err,
    output reg                  rx_busy
);

    //------------------------------------------------------------------
    // 过采样参数
    //
    // 精度说明（重要，这里踩过坑）：
    //   OS_DIV 是整数除法，会有截断误差。误差大小取决于 OS_DIV 的值：
    //       OS_DIV 越大，相对误差越小。
    //   实测对比（BAUD_RATE=115200）：
    //       CLK= 20MHz -> OS_DIV=10 (理想 10.85)  误差 7.8%  -> 接收失败
    //       CLK= 50MHz -> OS_DIV=27 (理想 27.13)  误差 0.5%  -> 可用
    //       CLK=100MHz -> OS_DIV=54 (理想 54.25)  误差 0.46% -> 可用
    //   原因：每比特偏短 8%，10 个比特累积偏差超过 80% 比特宽，
    //         中点采样会漂到相邻比特，起始位复核必然失败
    //         （现象：发送正常，但接收端 rx_busy 一直回 0、rx_done 永不置起）。
    //
    //   结论：要求 OS_DIV >= 16（对应误差 < 约 3%）；
    //         即 CLK_FREQ_HZ >= BAUD_RATE * 16 * 16 = BAUD_RATE * 256。
    //         115200 时约需 29.5MHz，50MHz/100MHz 系统时钟都满足。
    //------------------------------------------------------------------
    localparam integer OS_RATE = 16;                              // 每比特 16 个 tick
    localparam integer HALF    = OS_RATE / 2;                     // 8, 比特中点
    localparam integer OS_DIV  = CLK_FREQ_HZ / (BAUD_RATE * OS_RATE);

    // 可靠过采样所需的最小分频比（OS_DIV 不小于它时截断误差 < 约 3%）
    localparam integer OS_DIV_MIN = 16;

    initial begin
        if (OS_DIV < 2) begin
            $display("[uart_rx] FATAL: CLK_FREQ_HZ=%0d 相对 BAUD_RATE=%0d 过低 (OS_DIV=%0d < 2)",
                     CLK_FREQ_HZ, BAUD_RATE, OS_DIV);
        end else if (OS_DIV < OS_DIV_MIN) begin
            // 这是"能编译但不可靠"的情况，必须显式警告，否则症状很难查
            $display("[uart_rx] WARNING: OS_DIV=%0d 偏小（要求 >= %0d）,",
                     OS_DIV, OS_DIV_MIN);
            $display("           过采样截断误差过大会导致接收失败。");
            $display("           建议 CLK_FREQ_HZ >= BAUD_RATE * %0d = %0d",
                     OS_RATE * OS_DIV_MIN, BAUD_RATE * OS_RATE * OS_DIV_MIN);
        end
    end

    //------------------------------------------------------------------
    // 输入同步（异步输入必须打两拍）
    //------------------------------------------------------------------
    reg rx_s1, rx_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s1 <= 1'b1;
            rx_s2 <= 1'b1;
        end else begin
            rx_s1 <= rx;
            rx_s2 <= rx_s1;
        end
    end

    //------------------------------------------------------------------
    // 状态与计数器
    //------------------------------------------------------------------
    localparam integer S_IDLE  = 3'd0;
    localparam integer S_RECHK = 3'd1;   // 起始位复核（起点后第 8 个 tick）
    localparam integer S_DATA  = 3'd2;   // 数据位 d0..dN-1
    localparam integer S_PAR   = 3'd3;   // 校验位
    localparam integer S_STOP  = 3'd4;   // 停止位

    reg [2:0]  state;
    reg [15:0] div_cnt;       // 过采样分频计数 0..OS_DIV-1
    reg [4:0]  tick_cnt;      // 当前比特内已过的 tick 数 0..15
    reg [7:0]  data_idx;      // 已采样数据位数
    reg [7:0]  frame_data;    // 数据移位寄存器

    // 分频回零 = 一个过采样 tick
    wire os_tick = (div_cnt == 0);
    // 当前比特的中点
    wire mid_bit = (tick_cnt == HALF - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            div_cnt    <= 16'd0;
            tick_cnt   <= 5'd0;
            data_idx   <= 8'd0;
            frame_data <= 8'd0;
            data_out   <= {DATA_BITS{1'b0}};
            done       <= 1'b0;
            frame_err  <= 1'b0;
            parity_err <= 1'b0;
            rx_busy    <= 1'b0;
        end else begin
            done <= 1'b0;

            //---------------- 过采样分频 ----------------
            if (os_tick) begin
                div_cnt  <= OS_DIV[15:0] - 16'd1;
                // tick_cnt 在 0..OS_RATE-1 循环
                tick_cnt <= (tick_cnt >= OS_RATE - 1) ? 5'd0 : (tick_cnt + 5'd1);
            end else begin
                div_cnt <= div_cnt - 16'd1;
            end

            case (state)
                //------------------------------------------------------
                // 空闲：只在**下降沿**启动
                //
                // ⚠️ 这里踩过一个从最初版本就存在的致命 bug：
                //     写成了  if (rx_s2 == 1'b0 && rx_s1 == 1'b1)
                //   这是 **上升沿** 的判据（2 拍前低、1 拍前高）。
                //   结果接收器把它当起始位，可那时上一个比特已经结束了，
                //   起始位复核必然读到高电平 -> 判假起始 -> 退回 IDLE，
                //   永远收不到任何字节（现象：tx_busy=0 但 rx_done 永远为 0）。
                //
                //   正确判据：2 拍前为高、1 拍前为低 —— 也就是 s2==1 && s1==0。
                //   注意因为过了两级同步器，检测到边沿时已经比真实边沿晚 1~2 拍，
                //   这是同步器的固有延迟，中点采样有足够余量吸收它。
                //------------------------------------------------------
                S_IDLE: begin
                    rx_busy  <= 1'b0;
                    tick_cnt <= 5'd0;
                    if (rx_s2 == 1'b1 && rx_s1 == 1'b0) begin   // 下降沿
                        state      <= S_RECHK;
                        rx_busy    <= 1'b1;
                        data_idx   <= 8'd0;
                        frame_data <= 8'd0;
                        parity_err <= 1'b0;
                        frame_err  <= 1'b0;
                    end
                end

                //------------------------------------------------------
                // 起始位复核：过了 8 个 tick（约半比特）仍为低才算真起始位
                //------------------------------------------------------
                S_RECHK: begin
                    if (os_tick && mid_bit) begin
                        if (rx_s2 == 1'b1) begin
                            state   <= S_IDLE;     // 假起始位，丢弃
                            rx_busy <= 1'b0;
                        end else begin
                            state    <= S_DATA;
                            data_idx <= 8'd0;
                        end
                    end
                end

                //------------------------------------------------------
                // 数据位：中点采样，LSB first
                //------------------------------------------------------
                S_DATA: begin
                    if (os_tick && mid_bit) begin
                        frame_data[data_idx[2:0]] <= rx_s2;
                        if (data_idx >= DATA_BITS - 1) begin
                            state <= (PARITY_EN != 0) ? S_PAR : S_STOP;
                        end else begin
                            data_idx <= data_idx + 8'd1;
                        end
                    end
                end

                //------------------------------------------------------
                // 校验位（可选）
                //   相位说明：上一个采样点之后 tick_cnt 停在 HALF-1，
                //   再过 OS_RATE 个 tick 正好是下一比特的中点，无需重置。
                //------------------------------------------------------
                S_PAR: begin
                    if (os_tick && mid_bit) begin
                        parity_err <= (rx_s2 != (PARITY_EN == 1
                                                 ? ^frame_data
                                                 : ~^frame_data));
                        state <= S_STOP;
                    end
                end

                //------------------------------------------------------
                // 停止位
                //------------------------------------------------------
                S_STOP: begin
                    if (os_tick && mid_bit) begin
                        if (rx_s2 == 1'b1) begin
                            data_out <= frame_data[DATA_BITS-1:0];
                            done     <= 1'b1;
                        end else begin
                            frame_err <= 1'b1;
                        end
                        state   <= S_IDLE;
                        rx_busy <= 1'b0;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
