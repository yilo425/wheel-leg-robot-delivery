//=============================================================================
// beacon.v -- 引脚连通性信标（诊断用，不是产品固件）
//
// 目的：确定性地回答一个问题 ——
//     "ARM 的排针 13 到底连没连在 FPGA 的 C19 上？"
//
// 做法：在 C19 上输出一个约 3Hz 的方波（led 同步闪，便于人眼确认板子在跑）。
//   如果 ARM 把排针 13 当输入、数 2 秒的电平跳变：
//      数到跳变  -> 这根线确实接着 C19，问题在别处
//      一个都没有 -> 线没接在 C19 上（或断了）
//
// 为什么需要这个：telemetry 在仿真里正常（sim/tb_telemetry_check.v 收到 26 字节），
// 但上板后 ARM 那个引脚 2 秒零跳变，且一直是高电平 —— 只能靠"确定性地驱动一根线"
// 来区分"RTL 没发" 和 "线没接对"。继续猜没有意义。
//=============================================================================
module beacon (
    input  wire clk_50m,
    output wire beacon_out,   // -> C19（和正式固件的 uart_tx 同一根）
    output wire led,          // -> D0/M22，和 beacon_out 同相，人眼可见
    output wire [2:0] led2    // -> D1~D3，做一个缓慢流水，证明时钟在跑
);

    // 50MHz / 2^22 = 11.9Hz -> 方波约 6Hz；再分一级到 ~3Hz
    reg [22:0] cnt;
    always @(posedge clk_50m) begin
        cnt <= cnt + 23'd1;
    end

    // cnt[22] : 每 2^22 拍翻转一次 -> 周期 2*8388608/50e6 = 0.336s -> 2.98Hz
    assign beacon_out = ~cnt[22];   // 上电先高（UART 空闲电平），再开始振
    assign led        = ~cnt[22];

    // 慢速流水（约 0.3Hz 换一个灯），证明设计真的在跑
    reg [24:0] slow;
    always @(posedge clk_50m) begin
        slow <= slow + 25'd1;
    end
    // 用 slow 的高 3 位做一个 3 位滚动
    reg [2:0] walk;
    always @(posedge clk_50m) begin
        if (slow[24:23] == 2'b00)      walk <= 3'b001;
        else if (slow[24:23] == 2'b01) walk <= 3'b010;
        else if (slow[24:23] == 2'b10) walk <= 3'b100;
        else                           walk <= 3'b001;
    end
    assign led2 = walk;

endmodule
