//=============================================================================
// module : reset_sync
// layer  : hal
//-----------------------------------------------------------------------------
// 功能 : 异步复位、同步释放。
//        异步 assert 保证复位立即生效(不依赖时钟);
//        synchronizer 链保证 de-assert 与 clk 同步, 避免复位释放时刻的
//        亚稳态与不同触发器复位释放时间不一致导致的 state 分裂。
//-----------------------------------------------------------------------------
// 用法 : 所有业务模块统一使用本模块输出的 rst_n, 模块内部不再自己同步。
//        多时钟域时每个域各例化一份。
//=============================================================================
`timescale 1ns/1ps

module reset_sync #(
    parameter integer SYNC_STAGES = 3,          // 同步链级数, 2~3 即可
    parameter integer POR_CYCLES  = 256         // 上电后强制拉长复位(可选), 0=关闭
) (
    input  wire clk,        // 目标时钟域
    input  wire arst_n,     // 异步复位输入, 低有效(按键/上电复位)
    output wire rst_n       // 同步释放后的复位, 低有效
);

    //------------------------------------------------------------------
    // POR 计数器: 上电后 rst_n 额外保持 POR_CYCLES 个周期
    // 目的: PLL 锁定前 clk 可能不存在或抖动, 用计数器把复位撑住
    //------------------------------------------------------------------
    reg [31:0] por_cnt;
    reg        por_done;

    generate
        if (POR_CYCLES > 0) begin : g_por
            always @(posedge clk or negedge arst_n) begin
                if (!arst_n) begin
                    por_cnt  <= 32'd0;
                    por_done <= 1'b0;
                end else if (!por_done) begin
                    if (por_cnt >= POR_CYCLES[31:0] - 32'd1) begin
                        por_done <= 1'b1;
                    end else begin
                        por_cnt <= por_cnt + 32'd1;
                    end
                end
            end
        end else begin : g_nopor
            // POR 关闭：不需要计数器
            //
            // ⚠️ 这里踩过一个会让电路**死锁**的坑：
            //   原来写的是  always @(*) por_done = 1'b1;
            //   但 body 里没有读任何信号 -> 敏感列表为空 -> 该块只在 time 0
            //   执行一次；而 por_done 的声明初值 `= 1'b0` 也在 time 0 赋值，
            //   两者竞争、执行顺序不确定。
            //   ModelSim 实测：初值赢了 -> por_done 恒为 0
            //   -> arst_n_eff 恒为 0 -> 复位永远不释放，整个电路锁死。
            //
            //   现在改成：这里只给一个明确的初值，por_done 是否被用到
            //   完全由下面的常量选择器决定，不依赖任何求值顺序。
            initial begin
                por_cnt  = 32'd0;
                por_done = 1'b0;
            end
        end
    endgenerate

    // 关键：POR_CYCLES=0 时用**常量**直接放行，绕开 por_done，
    //       彻底消除"组合块写寄存器"带来的不确定性。
    wire por_ok = (POR_CYCLES == 0) ? 1'b1 : por_done;

    wire arst_n_eff = arst_n & por_ok;

    //------------------------------------------------------------------
    // 异步 assert / 同步 release
    //------------------------------------------------------------------
    reg [SYNC_STAGES-1:0] sync_chain;

    always @(posedge clk or negedge arst_n_eff) begin
        if (!arst_n_eff) begin
            sync_chain <= {SYNC_STAGES{1'b0}};
        end else begin
            sync_chain <= {sync_chain[SYNC_STAGES-2:0], 1'b1};
        end
    end

    assign rst_n = sync_chain[SYNC_STAGES-1];

endmodule
