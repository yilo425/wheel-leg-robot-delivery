<#
=============================================================================
 run_all.ps1 -- 一键：编译 + 跑测试 + 导出波形 + 汇总结果
-----------------------------------------------------------------------------
 凌晨两点的产物：把"要记一堆命令"变成"跑一个脚本"。

 设计前提（本机实测结论）：
   * ModelSim 的批处理模式（vsim -c）工作正常
   * ModelSim GUI 可用，但必须先设好 LM_LICENSE_FILE；漏设会报
        Unable to checkout a viewer license ... Vsim is closing.
     所以本脚本完全不碰 GUI，走 "批处理 + VCD" 路线。
   * VCD 用免费的 GTKWave 打开即可看波形

 用法：
   cd D:\robot_trial\sim
   powershell -ExecutionPolicy Bypass -File run_all.ps1

   只想跑测试、不要波形（更快）：
   ... -NoVcd

   只跑某一个 tb：
   ... -Only tb_pwm_motor

 输出：
   sim_result.txt      所有测试的完整日志
   summary.txt         只有 RESULT / FAIL 行的精简汇总  <-- 先看这个
   <tb>.vcd            每个 tb 的波形文件（GTKWave 打开）
=============================================================================
#>

[CmdletBinding()]
param(
    [string]$ModelsimHome = '',
    [string]$WorkDir      = 'D:\gowin_lib\work',
    [string]$GowinLib     = 'gw5a',
    [string]$SimDir       = '',
    [string[]]$Tbs = @('tb_reset_sync','tb_pwm_motor','tb_encoder_decode','tb_uart','tb_top_integration'),
    [switch]$NoVcd,
    [string]$Only = '',
    [switch]$SkipCompile
)

$ErrorActionPreference = 'Stop'

#-----------------------------------------------------------------------------
# 调用外部程序（vsim/vlog）
#
# 两个坑，都在这里解决：
#
# 坑1（已修）：$ErrorActionPreference='Stop' + 2>&1 捕获输出时，
#   只要 vsim 往 stderr 写了任何东西（哪怕是正常的 "Error loading design"
#   或者进度提示），PowerShell 就当成终止性错误，把整个脚本掐掉，
#   导致 summary.txt 根本没机会生成。
#   -> 调用外部程序时用 'Continue'，改由我们自己检查退出码和文本。
#
# 坑2（已修）：vsim 编译 Gowin 原语库很慢（700KB / 200+ 模块）。
#   如果把它输出重定向到内存，用户会看到屏幕一动不动，以为死机了。
#   -> 加 -Live 开关：需要长时间等待的调用直接继承控制台，
#      实时回显 vsim 的进度；同时把输出重定向到临时文件以便事后分析。
#-----------------------------------------------------------------------------
function Invoke-Tool {
    param(
        [string]$Exe,
        [string[]]$Arguments,
        [string]$WorkDir = '',
        [string]$LogFile = '',
        [switch]$Live
    )
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $oldLoc = $null
    try {
        if ($WorkDir) { $oldLoc = Get-Location; Set-Location $WorkDir }

        if ($Live) {
            # 实时模式：逐行回显子进程输出。
            # 不用 Tee-Object —— 它在"两个管道串联"时仍可能攒到最后才刷出来，
            # 而逐行 Write-Host 是立刻可见的，用户不会误以为死机。
            $lines = New-Object System.Collections.Generic.List[string]
            & $Exe @Arguments 2>&1 | ForEach-Object {
                $line = [string]$_
                $lines.Add($line)
                Write-Host "    $line"
            }
            $code = $LASTEXITCODE
            $txt = ($lines -join "`r`n")
            if ($LogFile) { Set-Content -Path $LogFile -Value $txt -Encoding UTF8 }
            return [pscustomobject]@{ Text = $txt; ExitCode = $code; Log = $LogFile }
        }

        $output = & $Exe @Arguments 2>&1
        $code = $LASTEXITCODE
        $txt = ($output | Out-String)
        if ($LogFile) { Set-Content -Path $LogFile -Value $txt -Encoding UTF8 }
        return [pscustomobject]@{ Text = $txt; ExitCode = $code; Log = $LogFile }
    } finally {
        if ($oldLoc) { Set-Location $oldLoc }
        $ErrorActionPreference = $old
    }
}

function Say-Step { param($m) Write-Host "`n=== $m ===" -ForegroundColor Cyan }
function Say-OK   { param($m) Write-Host "  [ OK ] $m" -ForegroundColor Green }
function Say-Warn { param($m) Write-Host "  [WARN] $m" -ForegroundColor Yellow }
function Say-Err  { param($m) Write-Host "  [FAIL] $m" -ForegroundColor Red }

#-----------------------------------------------------------------------------
# 定位 ModelSim
#-----------------------------------------------------------------------------
if (-not $ModelsimHome) {
    foreach ($c in @('D:\modeltech64_2020.4', 'C:\modeltech64_2020.4')) {
        if (Test-Path (Join-Path $c 'win64\vsim.exe')) { $ModelsimHome = $c; break }
    }
    if (-not $ModelsimHome) {
        $cmd = Get-Command vsim.exe -ErrorAction SilentlyContinue
        if ($cmd) { $ModelsimHome = Split-Path (Split-Path $cmd.Source -Parent) -Parent }
    }
}
if (-not $ModelsimHome) { Say-Err "找不到 ModelSim，请用 -ModelsimHome 指定"; exit 1 }

$VSIM = Join-Path $ModelsimHome 'win64\vsim.exe'
Say-Step "环境"
Say-OK "ModelSim : $ModelsimHome"
Say-OK "work 库  : $WorkDir"
Say-OK "原语库   : $GowinLib"

#-----------------------------------------------------------------------------
# 定位 sim 目录
#
# 坑（已修）：本脚本放在 sim\toolchain\ 里，$PSScriptRoot 指向的是
#   toolchain 目录而不是 sim 目录，导致 Set-Location 到 toolchain 后
#   找不到 compile_all.do，报 "找不到 compile_all.do"。
#   正确做法：若脚本所在目录名叫 toolchain，就往上一级取。
#
# 手动指定： -SimDir 'D:\robot_trial\sim'
#-----------------------------------------------------------------------------
if ($SimDir) {
    $simDir = $SimDir
} elseif ($PSScriptRoot) {
    $simDir = $PSScriptRoot
    if ((Split-Path $simDir -Leaf) -eq 'toolchain') {
        $simDir = Split-Path $simDir -Parent
    }
} else {
    $simDir = (Get-Location).Path
}

if (-not (Test-Path $simDir)) { Say-Err "sim 目录不存在: $simDir"; exit 1 }
Set-Location $simDir
Say-OK "工作目录 : $simDir"

if ($Only) { $Tbs = @($Only) }

#-----------------------------------------------------------------------------
# 1) 编译
#-----------------------------------------------------------------------------
if (-not $SkipCompile) {
    Say-Step "1) 编译"
    if (-not (Test-Path 'compile_all.do')) {
        Say-Err "找不到 compile_all.do，请先跑 emit_dofiles.ps1 生成"
        exit 1
    }
    Write-Host "  注意：编译 Gowin 原语库（700KB / 200+ 模块）可能需要几分钟，" -ForegroundColor Yellow
    Write-Host "        下面是 vsim 的实时输出，请不要以为卡住了。" -ForegroundColor Yellow
    Write-Host ""

    # -Live：实时回显 vsim 输出，避免看起来像死机
    $cRes = Invoke-Tool -Exe $VSIM -Arguments @('-c','-do','compile_all.do') `
                        -LogFile 'compile_result.txt' -Live
    $cTxt = $cRes.Text

    Write-Host ""
    $errCount = ([regex]::Matches($cTxt, '\*\* Error')).Count
    if ($errCount -gt 0) {
        Say-Err "编译有 $errCount 个 Error，详见 compile_result.txt"
        $cTxt -split "`n" | Where-Object { $_ -match '\*\* Error' } |
            Select-Object -First 20 | ForEach-Object { Write-Host "    $_" -ForegroundColor Red }
        exit 1
    }
    Say-OK "编译通过（0 Error），日志见 compile_result.txt"
} else {
    Say-Step "1) 编译（已跳过）"
}

#-----------------------------------------------------------------------------
# 2) 跑测试 + 导出 VCD
#-----------------------------------------------------------------------------
Say-Step "2) 跑测试"

$allText = New-Object System.Text.StringBuilder
[void]$allText.AppendLine("=== run_all.ps1  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ===")

foreach ($tb in $Tbs) {
    Write-Host ""
    Write-Host "------------------------------------------------------------"
    Write-Host "  $tb" -ForegroundColor White
    Write-Host "------------------------------------------------------------"

    if ($NoVcd) {
        # 不用 vcd.do：直接一条 Tcl 命令串，最直接
        $res = Invoke-Tool -Exe $VSIM -Arguments @(
            '-c','-do',
            "vsim -c -quiet -voptargs=+acc -L work -L $GowinLib work.$tb; run -all; quit -f"
        )
    } else {
        # 用 Tcl 变量传 testbench 名，不要用 `do vcd.do <name>` 的形式：
        # ModelSim 的 -do 会把 vsim 自己的开关(-c 等)漏进 $argv，
        # 导致 vcd.do 把 -c 当成 tb 名，去加载 work.-c 而报
        # "Error loading design"。
        $res = Invoke-Tool -Exe $VSIM -Arguments @(
            '-c','-do', "set TB $tb; do vcd.do"
        )
    }
    $txt = $res.Text
    [void]$allText.AppendLine("---------- $tb ----------")
    [void]$allText.AppendLine($txt)

    # 即时打印关键行，边跑边看
    foreach ($line in ($txt -split "`n")) {
        if ($line -match 'RESULT:|checks,|\[FAIL\]') {
            $color = if ($line -match 'FAIL') { 'Red' } elseif ($line -match 'PASS') { 'Green' } else { 'Gray' }
            Write-Host "    $($line.Trim())" -ForegroundColor $color
        }
    }

    if (-not $NoVcd) {
        $vcd = Join-Path $simDir "$tb.vcd"
        if (Test-Path $vcd) {
            $mb = [math]::Round((Get-Item $vcd).Length / 1MB, 2)
            Say-OK "$tb.vcd  已生成 ($mb MB)"
        } else {
            Say-Warn "$tb.vcd 没生成（可能仿真没跑到 vcd flush）"
        }
    }
}

Set-Content -Path 'sim_result.txt' -Value $allText.ToString() -Encoding UTF8

#-----------------------------------------------------------------------------
# 3) 汇总
#-----------------------------------------------------------------------------
Say-Step "3) 汇总"

$summary = New-Object System.Text.StringBuilder
[void]$summary.AppendLine("=========== 测试汇总  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ===========")
$pass = 0; $fail = 0; $unknown = 0
foreach ($tb in $Tbs) {
    $vcdNote = ''
    if (-not $NoVcd -and (Test-Path (Join-Path $simDir "$tb.vcd"))) { $vcdNote = '   [VCD 已生成]' }
    # 从完整日志里找这个 tb 的 RESULT
    $section = ($allText.ToString() -split "---------- $tb ----------")[1]
    if ($section) {
        if ($section -match 'RESULT:\s*PASS') {
            $pass++; [void]$summary.AppendLine("  PASS   $tb$vcdNote")
        } elseif ($section -match 'RESULT:\s*FAIL') {
            $fail++; [void]$summary.AppendLine("  FAIL   $tb$vcdNote")
            foreach ($line in ($section -split "`n")) {
                if ($line -match '\[FAIL\]') { [void]$summary.AppendLine("           $($line.Trim())") }
            }
        } else {
            # 没找到 RESULT 行 = 根本没跑起来（加载失败/编译缺失）
            # 这是"失败"，绝不能当成功处理。
            $unknown++
            [void]$summary.AppendLine("  ERROR  $tb  <== 没有 RESULT 行，仿真没跑起来$vcdNote")
            foreach ($line in ($section -split "`n")) {
                if ($line -match 'Error loading design|Error:|error while|unable to') {
                    [void]$summary.AppendLine("           $($line.Trim())")
                }
            }
        }
    } else {
        $unknown++
        [void]$summary.AppendLine("  ERROR  $tb  <== 日志里没有这个 tb 的记录")
    }
}
[void]$summary.AppendLine("")
[void]$summary.AppendLine("  PASS=$pass  FAIL=$fail  ERROR=$unknown")

Set-Content -Path 'summary.txt' -Value $summary.ToString() -Encoding UTF8

$allOk = ($fail -eq 0 -and $unknown -eq 0)

Write-Host ""
Write-Host $summary.ToString() -ForegroundColor $(if ($allOk) { 'Green' } else { 'Red' })

#-----------------------------------------------------------------------------
# 4) 下一步提示
#-----------------------------------------------------------------------------
Say-Step "下一步"
if ($allOk) {
    Write-Host "  全部通过！" -ForegroundColor Green
} elseif ($unknown -gt 0) {
    Write-Host "  有 $unknown 个 tb 根本没跑起来（看上面 ERROR 行的原因）。" -ForegroundColor Red
    Write-Host "  常见原因：work 库没编译、原语库没挂上(-L gw5a)、modelsim.ini 有问题。" -ForegroundColor Yellow
} else {
    Write-Host "  有 $fail 个 tb 未通过。把 summary.txt 的内容发给 AI 即可。" -ForegroundColor Yellow
}
if (-not $NoVcd) {
    Write-Host ""
    Write-Host "  看波形（GTKWave，免费、不需要任何许可）：" -ForegroundColor Cyan
    Write-Host "    现在还没装 GTKWave 的话，先跑这个看有没有装："
    Write-Host "      Get-Command gtkwave.exe -ErrorAction SilentlyContinue"
    Write-Host "    装了的话直接打开："
    foreach ($tb in $Tbs) {
        if (Test-Path (Join-Path $simDir "$tb.vcd")) {
            Write-Host "      gtkwave `"$simDir\$tb.vcd`""
        }
    }
}
Write-Host ""
Write-Host "  文件清单：" -ForegroundColor Cyan
foreach ($f in @('summary.txt','sim_result.txt','compile_result.txt')) {
    if (Test-Path (Join-Path $simDir $f)) { Write-Host "    $f" }
}

#-----------------------------------------------------------------------------
# 5) 明确的退出码
#   坑（已修）：最后调用过 vsim，$LASTEXITCODE 会残留它的值（可能非 0），
#   PowerShell 进程就带着那个码退出了，外部看起来像"脚本失败"。
#   这里显式给一个我们自己的退出码。
#-----------------------------------------------------------------------------
if ($allOk) { exit 0 } else { exit 2 }
