//=============================================================================
// module : clk_gen
// layer  : 硬件抽象层 (HAL)
//-----------------------------------------------------------------------------
// 功能 : 把板载 50MHz 变成系统时钟 100MHz，并给出 PLL 锁定信号。
//
//-----------------------------------------------------------------------------
// * 为什么这个模块长得像"半成品" —— 请先读这段
//-----------------------------------------------------------------------------
//   GW5A 的 PLL **不能用纯 RTL 描述**，必须例化厂商原语。
//   而 GW5A 的原语是 `PLL`（不是 `rPLL`）：
//
//       module PLL (CLKOUT0..CLKOUT6, CLKFBOUT, LOCK, CLKIN, CLKFB,
//                   ENCLK0..ENCLK6, FBDSEL, IDSEL, MDSEL, MDSEL_FRAC,
//                   ODSEL0..ODSEL6, DT0..DT3, PSSEL, PSDIR, PSPULSE,
//                   SSCPOL, SSCON, SSCMDSEL, SSCMDSEL_FRAC, ICPSEL,
//                   LPFRES, LPFCAP, RESET, PLLPWD, RESET_I, RESET_O);
//
//   **45 个端口**，其中 FBDSEL / IDSEL / MDSEL / ODSEL0-6 都是
//   "把分频比按某种编码写成常量位"的配置端口。
//
//   [!]️ 这个编码没有公开的简单公式 —— 它由 Gowin 的 IP Core Generator
//      根据「输入频率 + 输出频率 + 器件型号」算出来。
//      **手写必错**，而且错了之后现象是"时钟频率不对"，
//      整个设计的行为都会偏（PWM 频率、波特率、毫秒计时全错），
//      但**不会有任何报错**。
//
//   （对比：GW1N/GW2A 用的是 `rPLL`，配置走 `defparam`，可以手写。
//     本工程的目标器件 **GW5AT-138** **没有** `rPLL`，已核实：
//     在 Gowin V1.9.12 的
//     D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64\IDE\simlib\gw5a\prim_sim.v
//     里搜不到 rPLL。⚠️ 注意是 **GW5AT-138**，不是 GW5AST-138 —— 两个器件
//     ID 差一位、封装相同，选错器件生成的 PLL 参数是错的，而且不报错。
//     器件型号的定案过程见 doc/board_pinout.md 第 2 节。）
//
//-----------------------------------------------------------------------------
// 因此本模块的设计是「可旁路」：
//-----------------------------------------------------------------------------
//   BYPASS = 1（默认）
//       clk_sys = clk_in（50MHz），locked = 1
//       -> **现在就能仿真、能上板**，不需要先配好 PLL
//       -> 代价是没有 100MHz。对 LED/按键/编码器/UART 联调完全够用
//          （UART 115200 @50MHz 的 OS_DIV = 27，满足 >=16 的要求）
//
//   BYPASS = 0
//       使用真正的 PLL。**但必须先把 g_pll 分支里的代码填上**。
//       没填的话，clk_sys 仍然跟随 clk_in，而 locked 故意输出 0 ——
//       上层复位永远不释放，板子"什么都不动"，现象极其明显，
//       不会出现"静默跑在错频率上"这种最难查的情况。
//
//-----------------------------------------------------------------------------
// * 怎么生成 PLL（约 3 分钟）
//-----------------------------------------------------------------------------
//   1) 打开 Gowin IDE，新建或打开工程，器件选
//      **GW5AT-138**（`GW5AT-LV138PG484AC1/I0`）
//
//      ⚠️ 三件事都别搞错：
//        * 是 **GW5AT**，不是 GW5AST —— 差一个 "S" 就是另一个器件
//          （ID 0x0011081B vs 0x0001081B），生成的 PLL 参数会不对；
//        * 必须用 **Gowin V1.9.12 或更新** —— V1.9.11 教育版的器件表里
//          根本没有 GW5AT-138，综合能过但**烧录会报 ID code mismatch**；
//        * ModelSim 里挂的 Gowin 原语库，也必须是**同一个 V1.9.12** 的
//          prim_sim.v 编译出来的（查法见下面"仿真注意事项"）。
//
//   2) 菜单 Tools -> IP Core Generator
//
//   3) 左侧选 **PLL**（在 "Soft IP Core / Clock" 分类下；
//      不要选 rPLL —— GW5A 没有）
//
//   4) 参数：
//        Input Clock   : 50 MHz
//        Output Clock0 : 100 MHz
//        其余保持默认（Phase = 0，Duty = 50%，Internal Feedback）
//
//   5) 点 Generate，文件名填 **clk_gen_pll**（不要叫 clk_gen，
//      否则会和本模块重名）
//
//   6) 生成后会有 clk_gen_pll.v（或 .vp）。把 **整个文件内容**
//      原样粘贴到下面 g_pll 分支的 "=== 粘贴处 ==== " 之间，
//      然后把 clk_sys / locked 接上：
//          assign clk_sys = <生成模块的输出时钟端口>;
//          assign locked  = <生成模块的 LOCK 端口>;
//
//   7) 在 Gowin 工程里把 clk_gen_pll.v 加入文件列表，
//      并把 top.v 的 CLK_FREQ_HZ 改成 100_000_000、
//      PLL_BYPASS 改成 0
//
//   [!]️ **第 7 步的两个值必须同时改**：
//         PLL_BYPASS=0  ->  CLK_FREQ_HZ = 100_000_000
//         PLL_BYPASS=1  ->  CLK_FREQ_HZ =  50_000_000
//      只改一个的话，PWM 载波频率、测速窗口换算、main_fsm 的毫秒计时
//      会**同时偏一倍**，而且不会报错。
//      top.v 里有一条 initial 断言专门盯这个配对。
//
//-----------------------------------------------------------------------------
// 端口
//-----------------------------------------------------------------------------
//   clk_in    板载晶振输入（50MHz）
//   arst_n    异步复位（低有效），一般接按键；用于复位 PLL
//   clk_sys   系统时钟输出
//   locked    PLL 锁定指示。**必须**参与复位：见 top.v 的
//                 reset_sync(.arst_n(rst_key_n & locked))
//             PLL 未锁定时时钟不稳定，这时绝不能让逻辑跑起来。
//
//-----------------------------------------------------------------------------
// 仿真注意事项
//-----------------------------------------------------------------------------
//   BYPASS=1 时本模块是纯连线，没有任何时序逻辑 ——
//   ModelSim 里不需要 Gowin 原语库就能跑。
//   BYPASS=0 时（即例化真正的 PLL）**必须**挂上 gw5a 仿真库，
//   否则 vsim 会报
//       Instantiation of 'PLL' failed. The design unit was not found
//   生成库的方法见 doc/toolchain_gowin_modelsim.md。
//
//   ⚠️ 库的**版本**也必须对（2026-10-15 踩过）：
//       ModelSim 里那个 gw5a 库，必须是由**和综合器同一个 Gowin 版本**的
//       prim_sim.v 编译出来的，否则仿真模型和综合器不是一个版本。
//       一眼判断（Gowin V1.9.12 把 `DQS` 从 prim_sim.v 移进了 DQS_*.vp，
//       V1.9.11 的 prim_sim.v 里有 DQS）：
//           vdir -lib D:/modeltech64_2020.4/gw5a | findstr "MODULE DQS"
//           有输出 = V1.9.11 建的（过期）；无输出 = V1.9.12（模块总数 132）
//       回归里的 **tb_gowin_prim** 就是专门盯这条链的：它例化真实原语，
//       库缺失/版本不对/路径不对都会当场红。
//=============================================================================
`timescale 1ns/1ps

module clk_gen #(
    //------------------------------------------------------------------
    // 1 = 直通（默认）：clk_sys = clk_in，不需要 PLL
    // 0 = 使用 PLL（需先按上面第 6 步填入生成的代码）
    //------------------------------------------------------------------
    parameter integer BYPASS = 1,

    //------------------------------------------------------------------
    // 频率（仅用于自检，不参与综合）
    //------------------------------------------------------------------
    parameter integer CLK_IN_HZ  = 50_000_000,
    parameter integer CLK_OUT_HZ = 100_000_000
) (
    input  wire clk_in,
    input  wire arst_n,
    output wire clk_sys,
    output wire locked
);

    //==================================================================
    // 自检
    //   把"参数填错"变成一条开机就能看到的报错，
    //   而不是"板子行为诡异但查不出来"。
    //==================================================================
    initial begin
        if (BYPASS !== 0 && BYPASS !== 1) begin
            $display("[clk_gen] ERROR: BYPASS 只能是 0 或 1, 实际 %0d", BYPASS);
        end
        if (BYPASS == 1 && CLK_OUT_HZ != CLK_IN_HZ) begin
            $display("[clk_gen] WARNING: BYPASS=1 时 clk_sys 等于 clk_in (%0d Hz),",
                     CLK_IN_HZ);
            $display("          但 CLK_OUT_HZ 写成 %0d Hz —— 两者不一致。", CLK_OUT_HZ);
            $display("          如果上面有模块按 CLK_OUT_HZ 换算时间, 会偏 %0d 倍。",
                     CLK_OUT_HZ / CLK_IN_HZ);
        end
        if (BYPASS == 0 && CLK_OUT_HZ == CLK_IN_HZ) begin
            $display("[clk_gen] WARNING: BYPASS=0 但输入输出频率相同 (%0d Hz),",
                     CLK_IN_HZ);
            $display("          PLL 生成的时候是不是填错了?");
        end
    end

    //==================================================================
    // 主体
    //==================================================================
    generate
    if (BYPASS != 0) begin : g_bypass
        //--------------------------------------------------------------
        // 直通模式：板上没有 PLL，系统时钟就是晶振
        //
        //   适用：仿真、上板初调（LED/按键/编码器/PWM/UART）。
        //   UART 115200 @50MHz -> uart_rx 的 OS_DIV = 50e6/(115200*16) = 27，
        //   大于要求的 16，可以正常工作。
        //--------------------------------------------------------------
        assign clk_sys = clk_in;
        assign locked  = 1'b1;

    end else begin : g_pll
        //--------------------------------------------------------------
        // PLL 模式
        //
        // *** 这里还没有填 ***
        //
        //   现在的行为是"故意让板子不动"：
        //       clk_sys 跟随 clk_in（不会真的变成 100MHz）
        //       locked  = 0  ->  上层复位永远不释放
        //   这样"忘了填 PLL 就上板"会表现为"上电后 LED[0] 不亮"，
        //   一眼就知道是时钟没起来，而不是"跑在 50MHz 但以为是 100MHz"。
        //
        //   填法见本文件开头的「怎么生成 PLL」第 6 步：
        //     把 Gowin IP Core Generator 生成的内容粘到下面，
        //     再把 clk_sys / locked 接出来。
        //
        // === 粘贴处（把生成的 clk_gen_pll 模块内容放这里）==============
        //
        //   例（仅示意结构，**不要照抄参数** —— 参数必须由工具生成）：
        //
        //   wire pll_clk;
        //   wire pll_lock;
        //   wire gw_gnd = 1'b0;
        //
        //   PLL pll_inst (
        //       .CLKOUT0 (pll_clk),
        //       .LOCK    (pll_lock),
        //       .CLKIN   (clk_in),
        //       .CLKFB   (gw_gnd),
        //       .RESET   (~arst_n),
        //       .FBDSEL  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        //       .IDSEL   ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        //       .MDSEL   ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        //       .ODSEL0  ({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        //       .ODSEL0_FRAC (gw_gnd),
        //       .ENCLK0  (1'b1),
        //       .PLLPWD  (gw_gnd),
        //       .RESET_I (gw_gnd),
        //       ... 其余端口按生成文件补全 ...
        //   );
        //
        //   assign clk_sys = pll_clk;
        //   assign locked  = pll_lock;
        //
        // === 粘贴处结束 ================================================

        // ---- 未填 PLL 时的安全兜底（填好后删掉这两行）----
        assign clk_sys = clk_in;
        assign locked  = 1'b0;      // 故意 0：让上层复位不释放，现象明显

        initial begin
            $display("[clk_gen] ERROR: BYPASS=0 但 g_pll 分支是空的！");
            $display("          系统会一直停在复位（locked=0）。");
            $display("          请按本文件开头的说明生成 PLL 并填入。");
        end
    end
    endgenerate

endmodule
