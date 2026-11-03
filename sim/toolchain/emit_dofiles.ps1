<#
=============================================================================
 emit_dofiles.ps1
-----------------------------------------------------------------------------
 只负责生成 ModelSim 的 compile_all.do 与 run_sim.do，不做编译。

 为什么单独拆出来：
   1. 改了 RTL 文件列表后，可以只重跑这个脚本，不用重新编译 Gowin 库；
   2. setup_gowin_modelsim.ps1 内部也调用它，逻辑只有一份。

 用法：
   powershell -ExecutionPolicy Bypass -File emit_dofiles.ps1 `
       -LibName prim_gw5a -LibPath 'C:\gowin_lib\prim_gw5a' `
       -WorkDir 'C:\gowin_lib\work' -OutDir '..' `
       -RtlFiles @('hal/reset_sync.v','driver/pwm_motor.v')

 参数说明：
   -LibName   Gowin 原语库名（要和 vmap 的名字一致）
   -LibPath   原语库所在目录
   -WorkDir   work 库放哪（必须是纯 ASCII 路径）
   -OutDir    .do 文件输出目录（相对或绝对）
   -RtlFiles  RTL 源文件列表，相对于 OutDir
   -Tbs       Testbench 文件名列表（相对于 OutDir）
   -TopName   顶层模块名（只写进注释，便于人看）

 注意：.do 文件里的注释一律用英文。
       因为 Tcl 读脚本按系统 ANSI 代码页解码，中文注释会变成乱码，
       某些情况下还会让 ModelSim 编辑器报编码错。纯 ASCII 最稳。
=============================================================================
#>

[CmdletBinding()]
param(
    [string]$LibName = 'gw5a',
    [string]$LibPath = '',
    [string]$WorkDir = '',
    [string]$OutDir = '',
    [string[]]$RtlFiles = @(
        # 注意：不要把 .vh 头文件放进来当独立单元编译。
        # 头文件只装 `define 宏，不产生 design unit，
        # 单独丢给 vlog 属于"编译一个空文件"，某些 ModelSim 版本会报
        # "no design unit found"。头文件通过 -IncludeDir 用 +incdir+ 提供给需要它的文件。
        '../rtl/hal/reset_sync.v',
        '../rtl/hal/clk_gen.v',
        '../rtl/driver/pwm_motor.v',
        '../rtl/driver/encoder_decode.v',
        '../rtl/driver/uart_tx.v',
        '../rtl/driver/uart_rx.v',
        '../rtl/app/main_fsm.v',
        '../rtl/app/motion_decision.v',
        '../rtl/app/uart_frame.v',
        '../rtl/app/vision_rx.v',
        '../rtl/app/rc_rx.v',
        '../rtl/app/stance_ctrl.v',
        # CONFIG 帧（0x04）解析：运行时可配参数（避障距离/最大速度/手动模式）
        '../rtl/app/config_rx.v',
        '../rtl/app/telemetry_tx.v',
        '../rtl/driver/servo_lift.v',
        # ★ PCA9685 舵机板驱动（SERVO_IF=1 时用它代替直接 PWM）
        #   注：已通过 tb_pca9685_drv 验证，但 top.v 里的接续还未接通，
        #       详见 doc/PCA9685舵机板接入说明.md
        '../rtl/driver/pca9685_drv.v',
        '../rtl/driver/i2c_master.v',
        # I2C 总线仲裁：TOF/IMU 共享一条总线时的授权器（top.v §1.6）
        '../rtl/driver/i2c_bus_arb.v',
        '../rtl/driver/tof_driver.v',
        # HC-SR04 超声波（4 线 TRIG/ECHO）：TOF_IF=1 时用它代替 I2C 的 VL53L0X
        '../rtl/driver/tof_sr04.v',
        # IMU（MPU6050）：已集成进 top.v §1.6（IMU_ENABLE 默认 1）
        '../rtl/driver/imu_driver.v',
        # 电机护栏：软启动 + 堵转保护（top.v §5.5，夹在 motion_decision 与 pwm_motor 之间）
        '../rtl/driver/motor_guard.v',
        '../rtl/top.v'
    ),
    [string[]]$SimFiles = @(
        # 仿真专用模型（**不可综合**），由 testbench 例化。
        #
        # ⚠️ 千万别把这些塞进 $Tbs：
        #    $Tbs 会进 run_sim.do 的 TB_LIST，被当成 testbench 去 vsim 加载。
        #    模型里没有 initial/$stop，加载会一直挂着，看起来像"仿真卡死"。
        'vl53l0x_slave_model.v',
        # PLL elaborate 探针：只验"PLL 原语能不能被解析"，**不能进 TB_LIST**
        #   （run_sim.do 会 run -all 跑时间，而 GW5A 的 PLL 模型在参数没配好时
        #    可能算出 0 周期 -> #0 死循环，把整场仿真挂住）。
        #   用法是手动一条 vsim 加载命令，见 doc/toolchain_gowin_modelsim.md。
        'pll_elab_probe.v',
        # MPU6050 从机模型（imu_driver 的单模块 tb 用）
        'mpu6050_slave_model.v',
        # PCA9685 从机模型（0x40）：tb_top_servo_pca 用，验 SERVO_IF=1 的顶层通路
        'pca9685_slave_model.v'
    ),
    [string[]]$Tbs = @(
        'tb_reset_sync.v',
        'tb_clk_gen.v',
        'tb_pwm_motor.v',
        'tb_encoder_decode.v',
        'tb_uart.v',
        'tb_uart_frame.v',
        'tb_main_fsm.v',
        'tb_motion_decision.v',
        'tb_servo_lift.v',
        'tb_tof_driver.v',
        'tb_top_integration.v',
        'tb_top_tof_i2c.v',
        # ★ TOF + IMU 共享一条 I2C 总线（开了 IMU_ENABLE=1）：
        #   含"交叉污染"检查 —— 两块从机都不能被对方的事务写脏
        'tb_top_i2c_share.v',
        # 工具链冒烟：例化真实 Gowin 原语，专门盯"原语库有没有挂上 / 版本对不对"。
        # 其他 tb 全是纯 RTL，一个原语都不碰 —— 所以库坏了它们也不会红。
        'tb_gowin_prim.v',
        # IMU（MPU6050）：单模块验证，走真实 I2C 从机模型
        #   （集成进 top.v 的用例是上面的 tb_top_i2c_share）
        'tb_imu_driver.v',
        # 电机护栏：软启动 + 堵转保护（单模块，41 项）
        'tb_motor_guard.v',
        # CONFIG 帧（0x04）解析：参数 ID / 长度校验 / 钳位 / 手动模式
        'tb_config_rx.v',
        'tb_rc_rx.v',
        # HC-SR04 超声波：触发脉冲 / 回波测宽 / 距离换算 / 超时与超范围
        'tb_sr04_driver.v',
        # ★ telemetry 发送通路：用**和硬件一致的参数**（TOF_ENABLE=1/TOF_IF=1）
        #   例化 top，只看 uart_tx 到底动不动。
        #   加它的原因：原来所有 tb 只声明 wire uart_tx 却从不解码，
        #   telemetry 发送路径**一直没有断言覆盖** —— 上板才发现它没在发。
        'tb_telemetry_check.v',
        # ★ telemetry 载荷内容：13 字节逐字段（大端 / 有符号 / 24->16 饱和）+ 帧头。
        #   补的是真缺口 —— 之前只验了发送通路活不活，没验载荷字段对不对；
        #   而 ARM 侧真正依赖的正是这些字段。
        'tb_telemetry_payload.v',
        # 速度环**闭环**仿真：motion_decision + 电机一阶模型，看稳态精度/收敛/振荡。
        #   加它的原因：以前全是开环（给 spd_fb 看 duty），看不出稳态残差 ——
        #   而纯 P 控制在 G 约 1 时只能跑到目标的 1/3（实测 31%），
        #   加了积分项后到 100%，且对被控对象增益变化鲁棒（G=1 / G=2 都测）。
        'tb_motion_loop.v',
        # PCA9685 舵机板驱动：用命令记录器验证寄存器序列（25 项）
        'tb_pca9685_drv.v',
        # ★ 顶层 SERVO_IF=1 通路：PCA9685 从机模型接上之后才能进回归
        #   验：初始化 4 次事务 + 寄存器值（MODE1/PRE_SCALE/MODE2）+ 抬腿 16 次写 + 通道 OFF 计数
        'tb_top_servo_pca.v'
        #     （i2c_master 在没有从机时会挂到超时，不能这样测）
    ),
    [string]$TopName = 'top',
    [string]$IncludeDir = '../rtl',
    [string]$ModelsimIni = ''
)

#-----------------------------------------------------------------------------
# 输出目录：默认 = 本脚本所在目录的**上一级**（也就是 sim\）
#
#   ⚠️ 绝对不能默认成 '.'（= 当前目录）。文档教大家
#      "cd sim\toolchain && ... emit_dofiles.ps1"，
#      那样 .do 会被生成到 sim\toolchain\ 里；而 .do 内部的源文件路径是
#      **相对 OutDir** 写的（"tb_xxx.v"、"../rtl/..."），
#      于是回到 sim\ 编译时**一个文件都编不到**，而且不会报"路径错"，
#      只会报一堆找不到模块/文件。
#      这个坑 2026-10-15 真踩了一次：不带参跑完后 sim\ 里的 .do 其实没变，
#      "看起来一致"是假象（因为改动写到了 sim\toolchain\）。
#-----------------------------------------------------------------------------
if (-not $OutDir) { $OutDir = Split-Path $PSScriptRoot -Parent }
$OutDir = [System.IO.Path]::GetFullPath($OutDir)

# 防呆：正确的 OutDir 里应该有一堆 tb_*.v（.do 就是给它们用的）
if (-not (Get-ChildItem -Path $OutDir -Filter 'tb_*.v' -ErrorAction SilentlyContinue)) {
    Write-Host "  [!] 警告: $OutDir 里没有 tb_*.v —— 这看起来不是工程的 sim 目录。" -ForegroundColor Yellow
    Write-Host "      .do 里的源文件路径是相对该目录写的，放错地方会'一个文件都编不到'。" -ForegroundColor Yellow
    Write-Host "      请用 -OutDir 指定工程 sim 目录（默认取本脚本的上一级）。" -ForegroundColor Yellow
}

#-----------------------------------------------------------------------------
# 读本机专属配置（toolchain.local.ps1）—— 与 run_tb.ps1 用同一份配置
#
#   这样队友只需建一次配置文件，之后两个脚本都不用传参。
#
#   ⚠️ 不用 dot-source（`. $localCfg`）：
#      PowerShell 默认执行策略会拒绝加载未签名的 .ps1，
#      而且 `-ExecutionPolicy Bypass -File` 的策略**不会传递**给
#      内部 dot-source 的文件。实测会报 "is not digitally signed"。
#      改为读文本 + 正则抽路径，不执行任何代码。
#-----------------------------------------------------------------------------
$localCfg = Join-Path $PSScriptRoot 'toolchain.local.ps1'
if (Test-Path $localCfg) {
    $cfgText = Get-Content -LiteralPath $localCfg -Raw
    function Get-CfgVal([string]$key) {
        $mm = [regex]::Match($cfgText, ('(?m)^\s*' + $key + '\s*=\s*[''""]([^''""]+)[''""]'))
        if ($mm.Success) { return $mm.Groups[1].Value }
        return $null
    }
    if (-not $LibName) { $tmp = Get-CfgVal 'GowinSimLib'; if (-not $tmp) { $tmp = 'gw5a' }; $LibName = Split-Path $tmp -Leaf }
    if (-not $LibPath) { $LibPath = Get-CfgVal 'GowinSimLib' }
    if (-not $WorkDir) { $WorkDir = Get-CfgVal 'WorkDir' }
    if (-not $ModelsimIni) { $ModelsimIni = Get-CfgVal 'ModelsimIni' }
    Write-Host "  已加载本机配置: toolchain.local.ps1"
} else {
    Write-Host "  提示: 未找到 toolchain.local.ps1"
    Write-Host "        建议先执行: Copy-Item toolchain.local.ps1.template toolchain.local.ps1"
    Write-Host "        然后编辑里面的路径，之后两个脚本都不用再传参。"
}

# 兜底默认值（A 的机器）
if (-not $LibPath)     { $LibPath = 'D:\modeltech64_2020.4\gw5a' }
if (-not $WorkDir)     { $WorkDir = 'D:\gowin_lib\work' }
if (-not $ModelsimIni) { $ModelsimIni = 'C:\modeltech64_2020.4\modelsim.ini' }
if (-not $LibName)     { $LibName = 'gw5a' }

# 早失败：库路径不对就别往下走
if (-not (Test-Path $LibPath)) {
    Write-Host ""
    Write-Host "  x 找不到 Gowin 仿真库: $LibPath" -ForegroundColor Red
    Write-Host "    如果还没生成过，先跑一次：" -ForegroundColor Yellow
    Write-Host "        powershell -ExecutionPolicy Bypass -File setup_gowin_modelsim.ps1" -ForegroundColor Gray
    exit 1
}

Write-Host "  LibName=$LibName"
Write-Host "  LibPath=$LibPath"
Write-Host "  WorkDir=$WorkDir"

$ErrorActionPreference = 'Stop'

#-----------------------------------------------------------------------------
# Tcl 里路径必须用正斜杠：反斜杠是转义字符，
# "C:\gowin_lib" 会被吃成 "C:gowin_lib"，路径直接失效。
#-----------------------------------------------------------------------------
$LibPathTcl = $LibPath -replace '\\', '/'
$WorkDirTcl = $WorkDir -replace '\\', '/'

if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Force -Path $OutDir | Out-Null }

#-----------------------------------------------------------------------------
# 确保工程目录里有一份【完整可用】的 modelsim.ini
#
# 踩过的坑：modelsim.ini 不能凭手工新建。
#   vmap 需要往它的 [Library] 段里写映射；如果文件里没有 [Library] 段，
#   vmap 会报
#       ** Error: (vmap-20) Cannot access for writing file "modelsim.ini".
#          No such file or directory. (errno = ENOENT)
#   而且缺了 [Library] 段，ModelSim 也找不到 std / ieee 这些基础库。
#
# 正确做法：从 ModelSim 安装目录复制一份官方模板过来（那份自带完整 [Library] 段），
#           再去掉只读属性，最后由 vmap 自己往里写。
#-----------------------------------------------------------------------------
$iniPath = Join-Path $OutDir 'modelsim.ini'
$iniOk = $false

if (Test-Path $iniPath) {
    $existing = Get-Content -LiteralPath $iniPath -Raw
    if ($existing -match '\[Library\]') { $iniOk = $true }
    else { Write-Host "  现有 modelsim.ini 缺少 [Library] 段，将重新生成" }
}

if (-not $iniOk -and $ModelsimIni) {
    if (Test-Path $ModelsimIni) {
        Copy-Item -Path $ModelsimIni -Destination $iniPath -Force
        # 安装目录那份常带只读属性，不清掉后面写不进去
        try { Set-ItemProperty -Path $iniPath -Name IsReadOnly -Value $false -ErrorAction Stop } catch { }
        Write-Host "  已从 $ModelsimIni 复制完整的 modelsim.ini"
        $iniOk = $true
    } else {
        Write-Host "  [WARN] 找不到模板 $ModelsimIni，将退回只含 [Library] 段的简易版本" -ForegroundColor Yellow
    }
}

if (-not $iniOk) {
    # 兜底：造一个最小可用版本（至少保证 vmap 有地方可写）
    "[Library]`r`n" | Set-Content -LiteralPath $iniPath -Encoding ASCII
    Write-Host "  已生成最小 modelsim.ini（仅含 [Library] 段）"
}

#-----------------------------------------------------------------------------
# 拼 vlog 行
#   +incdir+ 是给 `include 用的搜索路径。project_config.vh 现在还没被 include，
#   但迟早会 include —— 没有这个参数到时就会报 "Cannot open include file"。
#-----------------------------------------------------------------------------
$incDir   = $IncludeDir -replace '\\', '/'
$incArg   = if ($IncludeDir) { "+incdir+""$incDir"" " } else { "" }
$rtlLines = ($RtlFiles | ForEach-Object { "vlog -work work $incArg`"$_`"" }) -join "`r`n"
$simLines = ($SimFiles | ForEach-Object { "vlog -work work $incArg`"$_`"" }) -join "`r`n"
$tbLines  = ($Tbs      | ForEach-Object { "vlog -work work $incArg`"$_`"" }) -join "`r`n"

$timestamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'

#-----------------------------------------------------------------------------
# compile_all.do  （注释全部英文，保证纯 ASCII）
#-----------------------------------------------------------------------------
$compileDo = @"
#=============================================================================
# compile_all.do   --  AUTO-GENERATED by emit_dofiles.ps1, DO NOT EDIT
# generated at : $timestamp
# top module   : $TopName
#-----------------------------------------------------------------------------
# Usage (run in the directory containing this file):
#     vsim -c -do compile_all.do
#
# This script is what actually connects Gowin and ModelSim:
#   1) The work library lives on a pure-ASCII path, which works around the
#      ModelSim 2020.4 bug where it opens the library database with an ANSI
#      path. Any non-ASCII character (e.g. a Chinese Windows user name)
#      makes sqlite fail with "unable to open database file".
#   2) vmap attaches the pre-compiled Gowin primitive library, so that
#      PLL / OSC / GSR and friends can be resolved during elaboration.
#   3) Sources are referenced in place -- no more copying .v files around.
#=============================================================================

# ---- Gowin device primitive library (built by setup_gowin_modelsim.ps1) ----
set GOWIN_LIB "$LibPathTcl"
set GOWIN_LIB_NAME $LibName

if {![file exists `${GOWIN_LIB}]} {
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "!! Gowin primitive library NOT FOUND: `${GOWIN_LIB}"
    echo "!! Please run setup_gowin_modelsim.ps1 first."
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    quit -f
}

# ---- modelsim.ini must exist and contain a [Library] section ----
#      Otherwise vmap fails with:
#        (vmap-20) Cannot access for writing file "modelsim.ini".
#      Fix: copy <ModelSim install>\modelsim.ini into this directory.
if {![file exists modelsim.ini]} {
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    echo "!! Missing modelsim.ini in the current directory."
    echo "!! Fix:  copy <ModelSim install>\\modelsim.ini here, then rerun."
    echo "!!   e.g. copy D:\\modeltech64_2020.4\\modelsim.ini ."
    echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    quit -f
}

# ---- work library location (MUST be a pure-ASCII path) ----
set WORK_DIR "$WorkDirTcl"
if {[file exists `${WORK_DIR}]} { file delete -force `${WORK_DIR} }
vlib `${WORK_DIR}
vmap work `${WORK_DIR}

# ---- attach the Gowin primitive library (the key step) ----
vmap `${GOWIN_LIB_NAME} `${GOWIN_LIB}

# ---- compile RTL ----
$rtlLines

# ---- compile simulation-only models (NOT synthesizable) ----
$simLines

# ---- compile testbenches ----
$tbLines

echo ""
echo "########## COMPILE DONE ##########"
echo "If there is no Error above, run run_sim.do next."
echo "Gowin primitive lib: `${GOWIN_LIB}"
"@

#-----------------------------------------------------------------------------
# run_sim.do
#-----------------------------------------------------------------------------
$tbListTcl = '{' + (($Tbs | ForEach-Object { $_ -replace '\.v$', '' }) -join ' ') + '}'

$runDo = @"
#=============================================================================
# run_sim.do   --  AUTO-GENERATED by emit_dofiles.ps1, DO NOT EDIT
# generated at : $timestamp
#-----------------------------------------------------------------------------
# Usage:
#     vsim -c -do run_sim.do                  batch: run all, print results
#     vsim -c -do "do run_sim.do tb_uart"     batch: run just one
#     vsim    -do run_sim.do                  GUI  : load one TB and keep it
#                                                    open so you can add waves
#
# GUI tips (after the window opens and the transcript settles):
#     add wave -r /*            dump every signal of the loaded TB
#     run -all                  run to the end
#     run 1us                   run a bit at a time
#
# Batch results are ALSO written to <tb>_result.txt so that a long log
# cannot push the PASS/FAIL lines out of view.
#=============================================================================

set GOWIN_LIB_NAME $LibName

set TB_LIST $tbListTcl

#-----------------------------------------------------------------------------
# Select which testbench to run.
#
# NEVER pass the tb name through `$argv !
#
#   Pitfall hit twice in this project (the second time by re-writing it):
#       vsim -c -do "do run_sim.do tb_top_integration"
#   Inside a -do script, `$argv is NOT {tb_top_integration}; it is
#       {-c tb_top_integration}
#   -- vsim leaks its OWN command-line switches into argv.
#   So [lindex `$TB_LIST 0] becomes "-c", the script tries to load work.-c,
#   and you get a detail-free "Error loading design" that is very hard to trace.
#
#   Correct way: the caller sets the Tcl variable TB_SEL first:
#       vsim -c -do "set TB_SEL tb_top_integration; do run_sim.do"
#
#   `$argv is only a last-resort fallback, and words starting with '-'
#   are explicitly filtered out.
#
# NOTE: keep every comment in the EMITTED .do files ASCII-only.
#       Tcl reads scripts through the ANSI code page, so non-ASCII bytes in a
#       .do file can corrupt parsing. This PowerShell generator may carry
#       Chinese comments, but the text it writes into .do must stay ASCII.
#-----------------------------------------------------------------------------
if {[info exists TB_SEL] && `$TB_SEL ne ""} {
    set TB_LIST `$TB_SEL
} else {
    set _argv_clean {}
    foreach _a `$argv {
        if {[string index `$_a 0] ne "-" && [string match "tb_*" `$_a]} { lappend _argv_clean `$_a }
    }
    if {[llength `$_argv_clean] > 0} { set TB_LIST `$_argv_clean }
}

# ---- detect GUI vs batch: in GUI mode 'noweyen' style commands work ----
#   In -c mode, [batch_mode] is 1. In GUI mode it is 0.
set IS_BATCH [batch_mode]

set FAILED {}

if {!`$IS_BATCH} {
    #------------------------------------------------------------------
    # GUI MODE
    #   -voptargs="+acc" is MANDATORY here.
    #   Without it ModelSim's optimizer deletes internal signals, so the
    #   "sim" pane only shows tasks/processes and the DUT instance is
    #   nowhere to be found -- you cannot add any wave.
    #   "+acc" means: keep full accessibility of every signal.
    #
    #   Wave scope is /<tb>/* rather than /* : the bare /* pattern matches
    #   nothing at the root and yields
    #       (vish-4014) No objects found matching '/*'.
    #------------------------------------------------------------------
    set tb [lindex `${TB_LIST} 0]
    echo "############################################################"
    echo "# GUI MODE -- loading `${tb} and staying open"
    echo "############################################################"
    vsim -voptargs="+acc" -L work -L `${GOWIN_LIB_NAME} work.`${tb}
    add wave -r /`${tb}/*
    echo ""
    echo "== Design loaded, signals kept, waves added. =="
    echo "   run -all      run to the end"
    echo "   run 1us       run a slice"
    echo "   restart -f    rewind and run again"
    echo "   zoom full     fit the whole waveform"
    # NOTE: deliberately no 'run' and no 'quit' here -- the user drives it.
} else {
    #------------------------------------------------------------------
    # BATCH MODE: run each TB, then tear down.
    #   The whole transcript is also teed into sim_result.txt by the caller
    #   (see run_sim.bat / the command in the runbook). Here we simply make
    #   sure every TB prints its RESULT line and we re-echo a summary.
    #------------------------------------------------------------------
    foreach tb `${TB_LIST} {
        echo ""
        echo "############################################################"
        echo "# RUN `${tb}"
        echo "############################################################"
        #  -L work             : resolve design units from the work library
        #  -L <primitive lib>  : WITHOUT THIS, PLL/OSC/GSR cannot be resolved
        #  -voptargs="+acc"    : MANDATORY when the testbench uses hierarchical
        #                        references such as dut.some_signal. Without it
        #                        the optimizer strips those signals and vsim
        #                        aborts with "Error loading design".
        #                        (tb_top_integration hits exactly this.)
        if {[catch {
            vsim -c -quiet -voptargs="+acc" -onfinish stop -L work -L `${GOWIN_LIB_NAME} work.`${tb}
            run -all
            quit -sim
        } err]} {
            echo "!! `${tb} FAILED to run: `${err}"
            lappend FAILED `${tb}
        }
    }

    echo ""
    echo "########## SIMULATION FINISHED ##########"
    if {[llength `${FAILED}] > 0} {
        echo "!! these testbenches errored: `${FAILED}"
    } else {
        echo "All testbenches ran. Look for 'RESULT: PASS' above."
    }
    echo "Tip: pipe the output to a file so results are not scrolled away:"
    echo "     vsim -c -do run_sim.do > sim_result.txt"
    quit -f
}
"@

#-----------------------------------------------------------------------------
# wave.do —— 一键加载某个 tb 并把波形全加好
#
# 背景（踩过的两个坑，都写进脚本里避免复现）：
#   1) vsim 默认会做 vopt 优化，把内部信号删掉，导致 sim 窗口里
#      只看到 task / process，找不到 DUT 实例，加不了波形。
#      必须加 -voptargs="+acc" 才能保留全部信号。
#   2) add wave 的作用域要写 /<tb>/*，不能写 /*，
#      后者会报 (vish-4014) No objects found matching '/*'。
#-----------------------------------------------------------------------------
$waveDo = @"
#=============================================================================
# wave.do   --  AUTO-GENERATED by emit_dofiles.ps1, DO NOT EDIT
# generated at : $timestamp
#-----------------------------------------------------------------------------
# Usage (GUI):
#     vsim -do wave.do                  load tb_pwm_motor (default)
#     vsim -do "do wave.do tb_uart"     load a specific testbench
#
# IMPORTANT: you MUST set the licence variables before launching the GUI,
#            otherwise vsim exits with
#                Unable to checkout a viewer license
#            (that is an ENV problem, not a missing licence feature --
#             LICENSE.TXT does contain INCREMENT vsimviewer_c)
#     `$env:LM_LICENSE_FILE   = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
#     `$env:MGLS_LICENSE_FILE = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
#
#-----------------------------------------------------------------------------
# Commands to type in the Transcript AFTER this file has loaded
#-----------------------------------------------------------------------------
#   restart -f            rewind to time 0
#   run 60us              run a slice (run -all = to the end)
#
#   ZOOM -- the command name is `wave zoom`, NOT `zoom`!
#     typing a bare `zoom full` in the Transcript gives
#         invalid command name "zoom"
#     because `zoom` is a WAVE-WINDOW command, not a main-interpreter one.
#
#   wave zoom full                     fit the whole simulation time
#   wave zoom range 30us 60us          show an exact window  <-- most useful
#   NEVER use a bare number: wave zoom range 42000 52000 is read as
#   **42 ns .. 52 ns**, not 42 us .. 52 us -- bare numbers are interpreted in
#   the simulator PRECISION unit (100 ps here), so they come out 1000x too small.
#   Always attach a unit: us / ns / ps.
#   wave zoom in / wave zoom out       step zoom (default factor 2.0)
#   wave zoom last                     undo the last zoom
#   wave seetime -time 45us -at 50     put 45us at the middle of the display
#
#   GUI equivalents: right-click in the waveform area -> Zoom -> ...
#                    or drag-select a region -> Zoom -> Zoom Selected
#=============================================================================

set GOWIN_LIB_NAME $LibName

set TB tb_pwm_motor
if {`$argc > 0} { set TB `$argv }
# guard: never accept a switch like "-c" as a testbench name
if {[string match "-*" `${TB}]} { set TB tb_pwm_motor }

echo "############################################################"
echo "# Loading `${TB} with full signal access"
echo "############################################################"

# -voptargs="+acc"  : keep every signal (without it the DUT is invisible)
# -L work           : design units from the work library
# -L <primitives>   : Gowin primitives (PLL / OSC / GSR)
vsim -voptargs="+acc" -L work -L `${GOWIN_LIB_NAME} work.`${TB}

# scope MUST be /<tb>/* ; a bare /* matches nothing and errors out
add wave -r /`${TB}/*

# make the window usable straight away
configure wave -timelineunits ns
echo ""
echo "== Ready =="
echo "   restart -f                          -> rewind"
echo "   run 60us                            -> run a slice"
echo "   wave zoom full                      -> NOTE the 'wave' prefix!"
echo "   wave zoom range 30us 60us           -> show an exact window"
"@

Set-Content -Path (Join-Path $OutDir 'wave.do') -Value $waveDo -Encoding ASCII
Write-Host "  已生成: $(Join-Path $OutDir 'wave.do')"

#-----------------------------------------------------------------------------
# vcd.do —— 不需要 GUI 许可的看波形方案
#
# 背景：
#   更正：GUI 实测【可用】。之前报 viewer license 错是 LM_LICENSE_FILE
#   漏设所致，不是许可缺 feature（LICENSE.TXT 里有 INCREMENT vsimviewer_c）。
#   VCD 方案作为备选保留（不开 GUI、或需要存档波形时用）。
#   原本的（已被证伪的）说明：许可签不出 "viewer license"，GUI（不带 -c 的 vsim）会报
#       Unable to checkout a viewer license ... Vsim is closing.
#   但**批处理模式（vsim -c）完全正常**。
#   于是改用：批处理跑仿真 -> 导出 VCD -> 用免费的 GTKWave 看波形。
#   VCD 是纯文本标准格式，GTKWave / 任何波形工具都能读，不需要任何许可。
#
# 用法（推荐用 Tcl 变量传参）：
#     vsim -c -do "set TB tb_pwm_motor; do vcd.do"
#     vsim -c -do "set TB tb_pwm_motor; set MAX 3000; do vcd.do"   限制 3000ns
#
# 为什么不从 $argv 读 testbench 名（重要的坑）：
#   ModelSim 的 -do 会把 vsim **自己的命令行开关**漏进 $argv。
#   实测 vsim -c -do "do vcd.do tb_reset_sync" 时：
#       $argv 变成 {-c 0}  ->  $TB = "-c"  ->  VCD 文件叫 "-c.vcd"
#       ->  加载 work.-c 失败, 报 "Error loading design"
#   所以这里优先用显式设置的 $TB，完全避开这个行为。
#-----------------------------------------------------------------------------
$vcdDo = @"
#=============================================================================
# vcd.do   --  AUTO-GENERATED by emit_dofiles.ps1, DO NOT EDIT
# generated at : $timestamp
#-----------------------------------------------------------------------------
# Batch-mode waveform dump. Works WITHOUT a viewer license.
#
# Usage (RECOMMENDED -- pass via Tcl variable):
#     vsim -c -do "set TB tb_pwm_motor; do vcd.do"
#     vsim -c -do "set TB tb_pwm_motor; set MAX 3000; do vcd.do"
#
# Then open the .vcd with GTKWave (free, no license):
#     gtkwave tb_pwm_motor.vcd
#
# NOTE: we deliberately do NOT rely on `$argv here. ModelSim's -do leaks
#       vsim's own switches (-c etc.) into `$argv, which made `$TB parse as
#       "-c" and broke loading. An explicitly set variable is immune to that.
#=============================================================================

set GOWIN_LIB_NAME $LibName

# ---- pick the testbench: prefer the variable, fall back to `$argv ----
if {![info exists TB]} {
    if {`$argc > 0} { set TB [lindex `${argv} 0] }
}
if {![info exists TB] || `${TB} eq "" || [string match "-*" `${TB}]} {
    echo "ERROR: no valid testbench name."
    echo "  Use: vsim -c -do {set TB <tbname>; do vcd.do}"
    quit -f
}
if {![info exists MAX]} { set MAX 0 }
set VCD "`${TB}.vcd"

echo "############################################################"
echo "# VCD dump: `${TB}   (max time: `${MAX} ns, 0 = run to end)"
echo "############################################################"

# -voptargs="+acc" : keep every signal. Without it the optimizer strips
#                    internal nets and the VCD only holds top-level ports.
vsim -c -quiet -voptargs="+acc" -L work -L `${GOWIN_LIB_NAME} work.`${TB}

vcd file `${VCD}
vcd add -r /`${TB}/*

if {`$MAX > 0} {
    run `${MAX} ns
} else {
    run -all
}
vcd flush
quit -f
"@

Set-Content -Path (Join-Path $OutDir 'vcd.do') -Value $vcdDo -Encoding ASCII
Write-Host "  已生成: $(Join-Path $OutDir 'vcd.do')"

#-----------------------------------------------------------------------------
# 落盘：ASCII 编码（Tcl 按 ANSI 代码页读脚本，纯 ASCII 最稳）
#-----------------------------------------------------------------------------
$compilePath = Join-Path $OutDir 'compile_all.do'
$runPath     = Join-Path $OutDir 'run_sim.do'
Set-Content -Path $compilePath -Value $compileDo -Encoding ASCII
Set-Content -Path $runPath     -Value $runDo     -Encoding ASCII

Write-Host "  已生成: $compilePath"
Write-Host "  已生成: $runPath"
Write-Host "  Gowin 原语库: $LibPathTcl"
Write-Host "  work 库     : $WorkDirTcl"
