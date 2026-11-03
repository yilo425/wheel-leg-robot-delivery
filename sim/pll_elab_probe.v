//=============================================================================
// module : pll_elab_probe
// layer  : 仿真探针（**不可综合**，只用来验"PLL 原语能不能被解析"）
//-----------------------------------------------------------------------------
// 它解决的是一个很具体的失败：
//
//     Instantiation of 'PLL' failed. The design unit was not found
//
// 这是"原语库没挂上 / 库不对 / 路径不对"时**唯一**的报错形式，
// 而它只在**真的例化 PLL** 时才会出现 —— 工程现在 clk_gen 走 BYPASS=1
// （纯连线），所以整条回归**碰不到 PLL**，也就永远发现不了这个错。
//
//-----------------------------------------------------------------------------
// 为什么单独做"只 elaborate、不跑时间"
//-----------------------------------------------------------------------------
//   GW5A 的 PLL 仿真模型是按参数**实时算出时钟周期**的：
//       vco_period   = clkin_period * IDIV_SEL / FBDIV_SEL / (MDIV_SEL + MDIV_frac/8)
//       clkout0_period = vco_period * ODIV0_SEL
//   参数配错时可能出现 0 周期 -> `#0` 死循环 -> **整个仿真挂住**。
//   而"PLL 单元是否存在"在 **elaborate 阶段**就已经确定了，根本不需要跑时间。
//
//   ★ 判据用 **vopt**（本项目铁律：vsim 只会给一句没细节的
//     "Error loading design"，**真实错误只有 vopt 会报**，
//     见 doc/踩坑与经验.md 1.3）：
//
//       # 正向：挂了库，应该 Errors: 0、退出码 0
//       vopt -work work pll_elab_probe -o pll_opt -L gw5a
//
//       # 反向对照：故意不挂库，应该报 PLL 找不到
//       vopt -work work pll_elab_probe -o pll_opt2
//
//   实测（2026-10-15，库 = Gowin V1.9.12 的 prim_sim.v 编译）：
//       正向 -> Errors: 0, Warnings: 0          退出码 0
//       反向 -> ** Error: pll_elab_probe.v(81): Module 'PLL' is not defined.
//                                              Errors: 1   退出码 2
//   一正一反，说明"库挂上就能解析 PLL、没挂就一定报错"。
//
//   ⚠️ 下面 initial 里的 $display 在 `vsim -c` 批处理下**不会**出现在
//      stdout（试过 `run 0` 也一样，只有 transcript 里可能也没有），
//      所以**判据别靠它**，靠上面 vopt 的 Errors / 退出码。
//
//   ⚠️ 真正的 PLL（50MHz→100MHz、分频比、LOCK 行为）**不在这里验** ——
//      那必须用 Gowin IP Core Generator 生成参数后再单独做。
//      本文件里的参数只是为了"把端口接全"，**不是可用的配置**，不要拿去上板。
//
//-----------------------------------------------------------------------------
// ⚠️ 这个文件**不在** run_sim.do 的 tb 列表里（它没有 tb_ 前缀，也不该被 run -all
//    去跑时间）。它只被 compile_all.do 编译（见 emit_dofiles.ps1 的 -SimFiles），
//    然后由上面那条 vopt 命令手动验证。
//=============================================================================
`timescale 1ns/1ps

module pll_elab_probe;

    wire clk_in;
    wire clk_out;
    wire lock;
    wire clkfb;
    wire gw_gnd = 1'b0;

    // 给一个时钟源（只 elaborate，不 run，所以它不会真的翻转）
    reg clk_src = 1'b0;
    always #10 clk_src = ~clk_src;
    assign clk_in = clk_src;
    assign clkfb  = gw_gnd;      // INTERNAL 反馈模式下按生成文件接

    //------------------------------------------------------------------
    // 例化 GW5A 的 PLL 原语（端口按 V1.9.12 prim_sim.v 的定义逐个接全）
    //   端口宽度都是从 prim_sim.v 抄的，别凭记忆改：
    //     FBDSEL/IDSEL[5:0]  MDSEL[6:0]  MDSEL_FRAC[2:0]
    //     ODSEL0..6[6:0]     ODSEL0_FRAC[2:0]
    //     DT0..3[3:0]        PSSEL[2:0]  ICPSEL[5:0]  LPFRES[2:0]  LPFCAP[1:0]
    //     SSCMDSEL[6:0]      SSCMDSEL_FRAC[2:0]
    //------------------------------------------------------------------
    PLL #(
        .FCLKIN        ("50.0"),
        .IDIV_SEL      (1),
        .FBDIV_SEL     (8),
        .ODIV0_SEL     (4),
        .MDIV_SEL      (8)
    ) u_pll (
        .CLKOUT0        (clk_out),
        .CLKOUT1        (),
        .CLKOUT2        (),
        .CLKOUT3        (),
        .CLKOUT4        (),
        .CLKOUT5        (),
        .CLKOUT6        (),
        .CLKFBOUT       (),
        .LOCK           (lock),
        .CLKIN          (clk_in),
        .CLKFB          (clkfb),
        .ENCLK0         (1'b1),
        .ENCLK1         (1'b0),
        .ENCLK2         (1'b0),
        .ENCLK3         (1'b0),
        .ENCLK4         (1'b0),
        .ENCLK5         (1'b0),
        .ENCLK6         (1'b0),
        .FBDSEL         (6'd0),
        .IDSEL          (6'd0),
        .MDSEL          (7'd0),
        .MDSEL_FRAC     (3'd0),
        .ODSEL0         (7'd0),
        .ODSEL0_FRAC    (3'd0),
        .ODSEL1         (7'd0),
        .ODSEL2         (7'd0),
        .ODSEL3         (7'd0),
        .ODSEL4         (7'd0),
        .ODSEL5         (7'd0),
        .ODSEL6         (7'd0),
        .DT0            (4'd0),
        .DT1            (4'd0),
        .DT2            (4'd0),
        .DT3            (4'd0),
        .PSSEL          (3'd0),
        .PSDIR          (1'b0),
        .PSPULSE        (1'b0),
        .SSCPOL         (1'b0),
        .SSCON          (1'b0),
        .SSCMDSEL       (7'd0),
        .SSCMDSEL_FRAC  (3'd0),
        .ICPSEL         (6'd0),
        .LPFRES         (3'd0),
        .LPFCAP         (2'd0),
        .RESET          (1'b0),
        .PLLPWD         (1'b0),
        .RESET_I        (1'b0),
        .RESET_O        (1'b0)
    );

    initial begin
        $display("[pll_elab_probe] PLL 例化成功 —— 原语库里的 PLL 能被 elaborate");
        $display("[pll_elab_probe] (注意: 这里只验证'单元存在', 不验证分频比/LOCK)");
        $finish;
    end

endmodule
