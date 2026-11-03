//=============================================================================
// module : uart_tx
// layer  : driver
//-----------------------------------------------------------------------------
// 功能 : UART 发送, 8N1 / 可选校验位, LSB first, 波特率由 parameter 推导。
//
//------------------------------------------------------------------------------
// 重大坑修复记录（务必保留这段说明）
//------------------------------------------------------------------------------
// 旧版本用"分数分频累加器"：
//     BAUD_INC = (BAUD_RATE * (1<<ACC_WIDTH) + CLK/2) / CLK
//     acc += BAUD_INC;  acc 溢出 2^ACC_WIDTH 即一个 baud_tick
//
// 这个公式在数学上是对的（tick 频率 = BAUD_RATE/CLK），但**在 Verilog 里溢出**：
//     BAUD_RATE * 2^24 = 115200 * 16777216 = 1.93e12
//     32 位有符号整数上限只有 2.15e9  ->  乘积回绕
//   实测结果：
//     CLK=100MHz  正确 INC=19327   实际算出 0
//     CLK= 50MHz  正确 INC=38655   实际算出 0
//    INC=0 意味着 acc 永远不增长、baud_tick 永不触发 -> 一个字节都发不出去。
//   （tb_uart 报 "tx_busy 一直为 1" 就是这个原因。）
//
// 新版本改用**整数分频计数器**，没有任何大数乘法，彻底避开溢出：
//     CLK_PER_BIT = CLK_FREQ_HZ / BAUD_RATE      e.g. 100MHz/115200 = 868
//     计数器在 0..CLK_PER_BIT-1 之间循环，回零即产生一个 baud_tick
//
// 精度代价：CLK_PER_BIT 是整数除法（100MHz 下 868 而理论 868.071），
//   单比特误差 0.008%，一帧 10 比特累计 < 0.1%，远小于 UART 的 2% 容限。
//   这比"公式对但算出来是 0"要好得多。
//
// 选参数建议：CLK_FREQ_HZ 最好是 BAUD_RATE 的整数倍，误差为 0。
//   例：10MHz / 115200 = 86.8（有误差）；10MHz/100000=100（整除，零误差）
//      100MHz/115200 = 868.07（误差 0.008%，完全可用）
//=============================================================================
`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115_200,
    parameter integer DATA_BITS   = 8,
    parameter integer PARITY_EN   = 0      // 0=无校验 1=偶校验 2=奇校验
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,     // 单周期发送请求
    input  wire [DATA_BITS-1:0]  data_in,
    output reg                   tx,
    output reg                   busy,
    output reg                   done       // 发送完成单周期脉冲
);

    //------------------------------------------------------------------
    // 参数推导：每比特占多少个 clk（整数分频）
    //------------------------------------------------------------------
    localparam integer CLK_PER_BIT   = CLK_FREQ_HZ / BAUD_RATE;
    // 独立的常量，避免对 parameter 做位选（Verilog-2001 不支持）
    localparam [31:0]  CLK_PER_BIT_M1 = CLK_PER_BIT - 1;

    localparam integer IDLE  = 2'd0;
    localparam integer START = 2'd1;
    localparam integer DATA  = 2'd2;
    localparam integer STOP  = 2'd3;

    //------------------------------------------------------------------
    // 寄存器声明必须在组合逻辑之前（Verilog 要求先声明后使用）
    //------------------------------------------------------------------
    reg [1:0]         state;
    reg [31:0]        div_cnt;      // 比特内分频计数
    reg [15:0]        bit_idx;      // 数据位序号
    reg [DATA_BITS:0] shift;        // {校验位/1, 数据}

    //------------------------------------------------------------------
    // 比特结束标志
    //
    // ⚠️ 这里踩过一个致命坑，务必看清：
    //   最初写的是 wire baud_tick = (div_cnt == 0);
    //   又让 IDLE 状态把 div_cnt 强制清零 —— 两者组合后：
    //       进入 START 时 div_cnt 正好是 0 -> baud_tick 立刻为真
    //       -> 起始位只持续 1 个 clk 就跳到数据位
    //       整个字节 11 个 clk 就"发完"了，而正常需要 8680 个 clk
    //   现象：tx_line 永远不拉低（起始位宽度 = 0），接收端什么都收不到。
    //
    //   正确写法：把 tick 放在"计满一个比特"的位置（div_cnt == CLK_PER_BIT-1），
    //             而不是放在"计数刚开始"的位置。
    //------------------------------------------------------------------
    wire bit_end = (div_cnt == CLK_PER_BIT_M1);

    // 组帧：LSB first，(可选)校验位放在数据之后
    wire [DATA_BITS:0] frame = PARITY_EN
                             ? {(PARITY_EN == 1 ? ^data_in : ~^data_in), data_in}
                             : {1'b1, data_in};

    initial begin
        if (CLK_PER_BIT < 2) begin
            $display("[uart_tx] FATAL: CLK_FREQ_HZ=%0d 相对 BAUD_RATE=%0d 过低",
                     CLK_FREQ_HZ, BAUD_RATE);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            div_cnt <= 32'd0;
            bit_idx <= 16'd0;
            shift   <= {(DATA_BITS+1){1'b1}};
            tx      <= 1'b1;          // 空闲为高
            busy    <= 1'b0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;

            //------------------------------------------------------------------
            // 分频计数：0,1,2,...,CLK_PER_BIT-1, 回零
            //   IDLE 时停在 0，这样 start 到来后会走满 CLK_PER_BIT 个 clk。
            //   注意配合上面的 bit_end —— 不能再把它当成 tick 用。
            //------------------------------------------------------------------
            if (state == IDLE) begin
                div_cnt <= 32'd0;
            end else if (bit_end) begin
                div_cnt <= 32'd0;
            end else begin
                div_cnt <= div_cnt + 32'd1;
            end

            case (state)
                //------------------------------------------------------
                IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        shift   <= frame;
                        bit_idx <= 16'd0;
                        tx      <= 1'b0;      // 起始位开始
                        busy    <= 1'b1;
                        state   <= START;
                    end
                end

                //------------------------------------------------------
                // 起始位持续 CLK_PER_BIT 个 clk，结束后发 bit0
                //------------------------------------------------------
                START: begin
                    if (bit_end) begin
                        tx      <= shift[0];
                        shift   <= {1'b1, shift[DATA_BITS:1]};
                        bit_idx <= 16'd0;
                        state   <= DATA;
                    end
                end

                //------------------------------------------------------
                // DATA 状态共移出 DATA_BITS + PARITY_EN 位
                //   bit_idx 从 0 走到 DATA_BITS+PARITY_EN-1
                //------------------------------------------------------
                DATA: begin
                    if (bit_end) begin
                        tx    <= shift[0];
                        shift <= {1'b1, shift[DATA_BITS:1]};
                        if (bit_idx >= (DATA_BITS + PARITY_EN - 1)) begin
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 16'd1;
                        end
                    end
                end

                //------------------------------------------------------
                // 停止位：拉高一个比特，然后回 IDLE
                //------------------------------------------------------
                STOP: begin
                    if (bit_end) begin
                        tx    <= 1'b1;
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= IDLE;
                    end
                end

                //------------------------------------------------------
                default: state <= IDLE;
            endcase
        end
    end

endmodule
