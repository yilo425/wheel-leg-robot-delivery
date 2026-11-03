#=============================================================================
# run_checks.ps1 -- 一键跑完所有"不需要 ModelSim、不需要硬件"的检查
#-----------------------------------------------------------------------------
# 用法：右键 -> 使用 PowerShell 运行
#   或在 PowerShell 里：
#       powershell -ExecutionPolicy Bypass -File tools\run_checks.ps1
#
# 为什么要有这个脚本：
#   1) 工程里的 python 命令**经常是坏的** ——
#      Windows 自带一个 0 字节的 Store 占位程序，命令叫 python，
#      敲 `python --version` 不报错但也不输出（退出码 9009）。
#      新手会以为"Python 装了"，然后所有命令都静默失败。
#      本脚本会**实际跑一次 --version 验证**，而不是只看命令存不存在。
#
#   2) 新队友第一次拿到代码，需要一条命令确认"我的环境是对的"。
#      这就是那条命令。
#
# 它跑什么：
#   - 5 个静态检查（编码/lint/例化/溢出/声明顺序）
#   - 4 个 Python 行为模型（主状态机/运动决策/UART帧/UART）
#   - ARM 侧协议库自测 + 联调工具 dry-run
#
# 全是秒级的，**不需要 ModelSim，不需要任何硬件**。
#=============================================================================

$ErrorActionPreference = 'Continue'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

function Line { Write-Host ('-' * 66) -ForegroundColor DarkGray }
function Head { param($t) Write-Host ""; Line; Write-Host "  $t" -ForegroundColor Cyan; Line }

$root = Split-Path $PSScriptRoot -Parent     # ...\robot_project
Set-Location -LiteralPath $root

Write-Host ""
Write-Host ('=' * 66) -ForegroundColor Cyan
Write-Host "  环境检查（不需要 ModelSim / 不需要硬件）" -ForegroundColor Cyan
Write-Host ('=' * 66) -ForegroundColor Cyan

#=============================================================================
# 1) 找一个**真正能跑**的 python
#-----------------------------------------------------------------------------
#   ⚠️ 不能只用 Get-Command 判断存在 —— Windows 有个 0 字节的
#      WindowsApps\python.exe 占位程序，存在但跑不起来。
#      必须实际执行 --version 并检查退出码和输出。
#=============================================================================
Head "[1/3] 找 Python"

$candidates = @()
foreach ($c in 'python', 'python3', 'py') {
    $g = Get-Command $c -ErrorAction SilentlyContinue
    if ($g) { $candidates += $g.Source }
}
# 常见安装位置
foreach ($p in @(
    "$env:LOCALAPPDATA\Programs\Python\Python313\python.exe",
    "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe",
    "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe",
    "$env:LOCALAPPDATA\Programs\Python\Python310\python.exe",
    "C:\Python313\python.exe", "C:\Python312\python.exe",
    "C:\Python311\python.exe", "C:\Python310\python.exe",
    "$env:ProgramFiles\Python313\python.exe",
    "$env:ProgramFiles\Python312\python.exe",
    # DSH 自带的（本机验证过可用）
    "$env:USERPROFILE\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe"
)) { if (Test-Path $p) { $candidates += $p } }

$py = $null
$tried = @()
foreach ($c in ($candidates | Select-Object -Unique)) {
    $tried += $c
    $out = & $c --version 2>&1
    $code = $LASTEXITCODE
    if ($code -eq 0 -and "$out" -match 'Python\s+3') {
        $py = $c
        Write-Host "  [OK]   $c" -ForegroundColor Green
        Write-Host "         $out"
        break
    } else {
        Write-Host "  [跳过] $c" -ForegroundColor DarkGray
        $sz = (Get-Item $c -ErrorAction SilentlyContinue).Length
        if ($sz -eq 0) {
            Write-Host "         0 字节 —— 这是 Windows Store 的占位程序，不是真 Python" -ForegroundColor DarkYellow
        } else {
            Write-Host "         退出码 $code，输出 '$out'" -ForegroundColor DarkGray
        }
    }
}

if (-not $py) {
    Write-Host ""
    Write-Host "  [错误] 找不到能用的 Python 3" -ForegroundColor Red
    Write-Host ""
    Write-Host "  检查过这些位置：" -ForegroundColor Yellow
    $tried | Select-Object -Unique | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkGray }
    Write-Host ""
    Write-Host "  怎么解决：" -ForegroundColor Yellow
    Write-Host "    1. 去 https://www.python.org/downloads/ 下载 Python 3.10+" -ForegroundColor Gray
    Write-Host "    2. 安装时**务必勾选** 'Add python.exe to PATH'" -ForegroundColor Gray
    Write-Host "    3. 装完关掉所有 PowerShell 窗口，重新打开再跑本脚本" -ForegroundColor Gray
    Write-Host ""
    Write-Host "  ★ 如果你之前敲 python 看着没报错但也没输出 ——" -ForegroundColor Yellow
    Write-Host "    那多半是 Windows 自带的 Store 占位程序。" -ForegroundColor Gray
    Write-Host "    打开 设置 -> 应用 -> 高级应用设置 -> 应用执行别名，" -ForegroundColor Gray
    Write-Host "    把 python.exe / python3.exe 两个开关**关掉**。" -ForegroundColor Gray
    Write-Host ""
    Read-Host "按回车退出"
    exit 1
}

#=============================================================================
# 2) 静态检查 + 行为模型
#=============================================================================
Head "[2/3] 静态检查 + 行为模型"

$checks = @(
    # ---- 静态检查：从【工程根】跑，路径参数相对根目录 ----
    @{ Name = '编码 / BOM / 纯ASCII';  Cwd = '.';   Script = 'tools\check_encoding.py';         ArgList = @('.') },
    @{ Name = 'RTL lint';              Cwd = '.';   Script = 'tools\rtl_lint.py';               ArgList = @('rtl', 'sim') },
    @{ Name = '例化参数/端口名';       Cwd = '.';   Script = 'tools\check_instantiations.py';   ArgList = @('rtl', 'sim') },
    @{ Name = '数值溢出';              Cwd = '.';   Script = 'tools\check_literal_overflow.py'; ArgList = @('rtl', 'sim') },
    @{ Name = '先用后声明';            Cwd = '.';   Script = 'tools\check_decl_order.py';       ArgList = @() },

    # ---- Python 行为模型：从工程根跑 ----
    @{ Name = '主状态机模型';          Cwd = '.';   Script = 'tools\verify_main_fsm_model.py';        ArgList = @() },
    @{ Name = '运动决策模型';          Cwd = '.';   Script = 'tools\verify_motion_decision_model.py'; ArgList = @() },
    @{ Name = 'UART 帧协议模型';       Cwd = '.';   Script = 'tools\verify_uart_frame_model.py';      ArgList = @() },
    @{ Name = 'UART 收发模型';         Cwd = '.';   Script = 'tools\verify_uart_model.py';            ArgList = @() },

    # ---- ARM 侧：必须在 arm\ 下跑（脚本 import 同目录的 uart_proto）----
    @{ Name = 'ARM 协议库自测';        Cwd = 'arm'; Script = 'uart_proto.py';        ArgList = @() },
    @{ Name = 'ARM 联调工具 dry-run';  Cwd = 'arm'; Script = 'fpga_link.py';         ArgList = @('--dry-run') },
    @{ Name = 'ARM 帧结构演示';        Cwd = 'arm'; Script = 'detect_from_image.py'; ArgList = @('--demo-frame') }
)

$pass = 0; $fail = 0; $skip = 0
$failedNames = @()

foreach ($c in $checks) {
    $dir = Join-Path $root $c.Cwd
    $scriptPath = Join-Path $dir $c.Script
    if (-not (Test-Path $scriptPath)) {
        Write-Host ("  [跳过] {0,-24} (文件不存在: {1})" -f $c.Name, $c.Script) -ForegroundColor DarkGray
        $skip++
        continue
    }

    Push-Location $dir
    try {
        # ⚠️ 必须用 @cmdArgs（数组 splat）把元素**分别**作为参数传给外部程序。
        #    写成 `& $py @($a) @($b)` 是错的 —— PowerShell 会把字符串
        #    '--dry-run' 按字符拆开发出去，实测报
        #        unrecognized arguments: - - d r y - r u n
        $cmdArgs = @($c.Script) + @($c.ArgList)
        $out = & $py @cmdArgs 2>&1
        $code = $LASTEXITCODE
        $joined = ($out | Out-String)

        # 判定：优先看 RESULT 行，再看 errors=，最后看退出码
        $isPass = $false
        if     ($joined -match 'RESULT:\s*PASS') { $isPass = $true }
        elseif ($joined -match 'RESULT:\s*FAIL') { $isPass = $false }
        elseif ($joined -match 'errors=(\d+)')   { $isPass = ($Matches[1] -eq '0') }
        else                                     { $isPass = ($code -eq 0) }

        if ($isPass) {
            $summary = ''
            if     ($joined -match 'RESULT:\s*PASS\s*\((\d+)\s*项') { $summary = " ($($Matches[1]) 项)" }
            elseif ($joined -match 'errors=(\d+)\s+warnings=(\d+)') { $summary = " (errors=$($Matches[1]))" }
            elseif ($joined -match '(\d+)\s*项检查全部通过')        { $summary = " ($($Matches[1]) 项)" }
            Write-Host ("  [PASS] {0,-24}{1}" -f $c.Name, $summary) -ForegroundColor Green
            $pass++
        } else {
            Write-Host ("  [FAIL] {0,-24}" -f $c.Name) -ForegroundColor Red
            $out | Select-Object -Last 8 | ForEach-Object { Write-Host "         $_" -ForegroundColor DarkGray }
            $fail++
            $failedNames += $c.Name
        }
    } finally { Pop-Location }
}

#=============================================================================
# 3) 汇总
#=============================================================================
Head "[3/3] 汇总"

Write-Host "  Python : $py"
Write-Host "  通过   : $pass" -ForegroundColor Green
if ($fail -gt 0) { Write-Host "  失败   : $fail" -ForegroundColor Red }
if ($skip -gt 0) { Write-Host "  跳过   : $skip" -ForegroundColor DarkGray }
Write-Host ""

if ($fail -eq 0) {
    Write-Host "  ✅ 全部通过 —— 你的环境没问题，可以开工了。" -ForegroundColor Green
    Write-Host ""
    Write-Host "  下一步：" -ForegroundColor White
    Write-Host "    队友 B  -> 读 handoff\B_arm.md，把提示词发给你的 AI" -ForegroundColor Gray
    Write-Host "    队友 C  -> 读 handoff\C_mech.md，先填接口表" -ForegroundColor Gray
    Write-Host "    队友 A  -> 读 handoff\A_fpga.md" -ForegroundColor Gray
    Write-Host ""
    Write-Host "    要跑 Verilog 仿真，还需要装 ModelSim ——" -ForegroundColor DarkGray
    Write-Host "    见 doc\runbook_verify_toolchain.md" -ForegroundColor DarkGray
} else {
    Write-Host "  ❌ 有 $fail 项失败：" -ForegroundColor Red
    $failedNames | ForEach-Object { Write-Host "       $_" -ForegroundColor Red }
    Write-Host ""
    Write-Host "  这是**代码或环境的问题**，不是你的操作问题。" -ForegroundColor Yellow
    Write-Host "  把上面的完整输出（含失败项）发给队友 A 或你的 AI。" -ForegroundColor Yellow
}

Write-Host ""
Read-Host "按回车关闭"
