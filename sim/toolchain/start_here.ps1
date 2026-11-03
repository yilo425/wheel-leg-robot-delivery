<#
=============================================================================
 start_here.ps1 -- 新终端第一件事：一键把环境配好
-----------------------------------------------------------------------------
 为什么需要它：
   你每次开新终端都要重敲 $VSIM / $env:LM_LICENSE_FILE / cd 一大堆，
   忘一个就报 "can't read VSIM" 或 viewer license 错。
   这个脚本把全部预设条目一次性设好，并做一轮预检。

 用法（新终端里就这一条）：
     powershell -ExecutionPolicy Bypass -File D:\robot_trial\sim\toolchain\start_here.ps1

   然后它会留在同一个进程里不行（子进程改了环境变量不影响父进程），
   所以请改用"点源"方式运行，环境变量才会留在你当前终端：

     . D:\robot_trial\sim\toolchain\start_here.ps1

   （注意前面那个点是必须的，代表在当前作用域里执行）

 跑完你就有这些变量：
     $VSIM  ModelSim 可执行文件
     $PY    Python 解释器（跑 tools/ 下的检查脚本）
     $PRJ   工程根目录
     $SIM   sim 目录（.do 所在处）
=============================================================================
#>

[CmdletBinding()]
param(
    [string]$ProjectRoot = 'D:\robot_trial',
    [string]$ModelsimHome = '',
    [string]$GowinLibDir  = 'D:\gowin_lib',
    [string]$GowinHome    = ''
)

function Say-Step { param($m) Write-Host "`n=== $m ===" -ForegroundColor Cyan }
function Say-OK   { param($m) Write-Host "  [ OK ] $m" -ForegroundColor Green }
function Say-Warn { param($m) Write-Host "  [WARN] $m" -ForegroundColor Yellow }
function Say-Err  { param($m) Write-Host "  [FAIL] $m" -ForegroundColor Red }

$problems = @()

#-----------------------------------------------------------------------------
# 1) 定位 ModelSim
#-----------------------------------------------------------------------------
Say-Step "1) ModelSim"

if (-not $ModelsimHome) {
    foreach ($c in @('D:\modeltech64_2020.4', 'C:\modeltech64_2020.4')) {
        if (Test-Path (Join-Path $c 'win64\vsim.exe')) { $ModelsimHome = $c; break }
    }
}

if ($ModelsimHome -and (Test-Path (Join-Path $ModelsimHome 'win64\vsim.exe'))) {
    $global:VSIM = Join-Path $ModelsimHome 'win64\vsim.exe'
    Say-OK "VSIM = $global:VSIM"
} else {
    Say-Err "找不到 ModelSim（试过 D:\ 和 C:\ 下的 modeltech64_2020.4）"
    $problems += 'ModelSim 未找到'
}

#-----------------------------------------------------------------------------
# 2) 许可环境变量
#    本机实测：批处理(vsim -c) 与 GUI 都正常。GUI 之前报 viewer license
#    错，根因是 LM_LICENSE_FILE 漏设，不是许可缺 feature。
#    设好这两个变量能让许可检查走文件模式，少一类报错。
#-----------------------------------------------------------------------------
Say-Step "2) 许可环境变量"

$licCandidates = @()
if ($ModelsimHome) { $licCandidates += (Join-Path $ModelsimHome 'win64\LICENSE.TXT') }
$licCandidates += 'D:\modeltech64_2020.4\win64\LICENSE.TXT'

$lic = $licCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if ($lic) {
    $env:LM_LICENSE_FILE   = $lic
    $env:MGLS_LICENSE_FILE = $lic
    Say-OK "LM_LICENSE_FILE   = $lic"
    Say-OK "MGLS_LICENSE_FILE = $lic"

    # 顺手核对许可文件有效性
    $txt = Get-Content $lic -Raw
    $incCount = ([regex]::Matches($txt, '(?m)^\s*INCREMENT\s')).Count
    $hasViewer = $txt -match 'INCREMENT\s+vsimviewer'
    $hostIdOk  = $txt -match [regex]::Escape($env:COMPUTERNAME)
    Say-OK "INCREMENT 条目数 = $incCount"
    if ($hasViewer) { Say-OK "含 vsimviewer_c（GUI 许可条目在）" }
    else { Say-Warn "没有 vsimviewer_c —— GUI 一定会失败，只能用批处理" }
    if ($hostIdOk) { Say-OK "主机名匹配（$env:COMPUTERNAME）" }
    else { Say-Warn "许可文件里找不到本机名 $env:COMPUTERNAME" }
} else {
    Say-Err "找不到 LICENSE.TXT"
    $problems += '许可文件未找到'
}

#-----------------------------------------------------------------------------
# 3) 工程目录
#-----------------------------------------------------------------------------
Say-Step "3) 工程目录"

if (Test-Path $ProjectRoot) {
    $global:PRJ = (Resolve-Path $ProjectRoot).Path
    Say-OK "PRJ = $global:PRJ"
} else {
    Say-Err "工程目录不存在: $ProjectRoot"
    Say-Warn "如果放别处，用 -ProjectRoot '路径' 指定"
    $problems += '工程目录不存在'
}

if ($global:PRJ) {
    $global:SIM = Join-Path $global:PRJ 'sim'
    if (Test-Path $global:SIM) { Say-OK "SIM = $global:SIM" }
    else { Say-Err "sim 目录不存在: $global:SIM"; $problems += 'sim 目录不存在' }
}

#-----------------------------------------------------------------------------
# 4) Gowin 原语库
#-----------------------------------------------------------------------------
Say-Step "4) Gowin 原语库"

$gLib = Join-Path $GowinLibDir 'work'
if (Test-Path $gLib) { Say-OK "work 库目录存在: $gLib" }
else { Say-Warn "work 库目录还不存在（第一次编译会自动建）: $gLib" }

# 本机已有一个编译好的 gw5a 原语库，直接复用，不用重编
# 注意：PowerShell 里若结果只有一项，$x[0] 会退化成"取字符串第0个字符"，
#       所以先用 @() 强制成数组再取下标。
$prebuilt = @(
    @('D:\modeltech64_2020.4\gw5a', 'C:\modeltech64_2020.4\gw5a') |
    Where-Object { Test-Path (Join-Path $_ '_info') }
)
if ($prebuilt.Count -gt 0) {
    $firstLib = @($prebuilt)[0]
    Say-OK "已编译好的 Gowin 原语库: $firstLib"
    Say-OK "  -> .do 里用的库名是 gw5a，指向的就是它"
} else {
    Say-Warn "没找到编译好的 gw5a 原语库，需要先跑 setup_gowin_modelsim.ps1"
}

#-----------------------------------------------------------------------------
# 5) Python（跑 tools/ 下的静态检查）
#-----------------------------------------------------------------------------
Say-Step "5) Python"

$pyCands = @(
    'C:\Users\忆洛\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'
)
$py = $pyCands | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $py) { $py = (Get-Command python.exe -ErrorAction SilentlyContinue).Source }

if ($py) {
    $global:PY = $py
    Say-OK "PY = $global:PY"
} else {
    Say-Warn "没找到 python，tools/ 下的静态检查脚本会跑不了（不影响仿真）"
}

#-----------------------------------------------------------------------------
# 6) 检查关键 .do 文件
#-----------------------------------------------------------------------------
Say-Step "6) 关键 .do 文件"

if ($global:SIM) {
    foreach ($f in @('compile_all.do','run_sim.do','wave.do','vcd.do')) {
        $p = Join-Path $global:SIM $f
        if (Test-Path $p) { Say-OK $f }
        else { Say-Warn "$f 缺失 —— 跑 toolchain\emit_dofiles.ps1 生成" }
    }
    $ini = Join-Path $global:SIM 'modelsim.ini'
    if (Test-Path $ini) { Say-OK 'modelsim.ini' }
    else { Say-Warn 'modelsim.ini 缺失 —— vmap 会报 (vmap-20)' }
}

#-----------------------------------------------------------------------------
# 6.5) 检查 Testbench 是否带"结果写文件"能力
#
# 为什么重要：
#   ModelSim -c 模式下 $display 走 stdout，PowerShell 的管道经常抓不到，
#   结果就是"跑完了但看不到 PASS/FAIL"。
#   带 $fwrite 的 tb 会把结果直接写进 <xxx>_result.txt，不依赖控制台。
#-----------------------------------------------------------------------------
Say-Step "6.5) Testbench 结果文件支持"

if ($global:SIM) {
    $tbFiles = @(Get-ChildItem -Path $global:SIM -Filter 'tb_*.v' -ErrorAction SilentlyContinue)
    if ($tbFiles.Count -eq 0) {
        Say-Warn "sim 目录下没有 tb_*.v"
    } else {
        $withLog = 0
        foreach ($tb in $tbFiles) {
            if (Select-String -Path $tb.FullName -Pattern 'integer log_fd' -Quiet) { $withLog++ }
        }
        if ($withLog -eq $tbFiles.Count) {
            Say-OK "全部 $withLog 个 tb 都支持结果写文件"
            Say-OK "  -> 跑完会产生 *_result.txt，不受控制台抓不到输出的影响"
        } else {
            Say-Warn "$withLog / $($tbFiles.Count) 个 tb 支持结果写文件"
            Say-Warn "  未支持的可以跑 tools\add_result_logging.py 打补丁"
        }
    }
}

#-----------------------------------------------------------------------------
# 7) GTKWave（可选，用来看 VCD 波形）
#-----------------------------------------------------------------------------
Say-Step "7) GTKWave（可选）"

$gtk = Get-Command gtkwave.exe -ErrorAction SilentlyContinue
if ($gtk) { Say-OK "gtkwave = $($gtk.Source)" }
else { Say-Warn "没装 GTKWave —— 想看波形再装（免费，不需要许可）" }

#-----------------------------------------------------------------------------
# 汇总
#-----------------------------------------------------------------------------
Say-Step "预检结果"

if ($problems.Count -eq 0) {
    Write-Host "  环境就绪。" -ForegroundColor Green
} else {
    Write-Host "  有 $($problems.Count) 个问题：" -ForegroundColor Yellow
    $problems | ForEach-Object { Write-Host "    - $_" -ForegroundColor Yellow }
}

Write-Host ""
Write-Host "接下来最省事的做法（只拿测试结果）：" -ForegroundColor Cyan
if ($global:SIM) { Write-Host "  cd `"$global:SIM`"" }
Write-Host "  powershell -ExecutionPolicy Bypass -File toolchain\run_all.ps1 -NoVcd"
Write-Host "  Get-Content summary.txt"
Write-Host ""
Write-Host "如果想看波形（导出 VCD 给 GTKWave）：" -ForegroundColor Cyan
Write-Host "  powershell -ExecutionPolicy Bypass -File toolchain\run_all.ps1"
Write-Host ""
Write-Host "手动单跑一个 tb（把下面路径直接用，不用再拼变量）：" -ForegroundColor Cyan
if ($global:VSIM) {
    Write-Host "  & `"$global:VSIM`" -c -do compile_all.do"
    Write-Host "  & `"$global:VSIM`" -c -do run_sim.do"
    Write-Host "  & `"$global:VSIM`" -c -do `"do vcd.do tb_pwm_motor 3000`""
} else {
    Write-Host "  (VSIM 未设置，先解决上面的问题)"
}
Write-Host ""
Write-Host "如果执行策略又拦你，跑一次这条永久解决：" -ForegroundColor Gray
Write-Host "  Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned -Force" -ForegroundColor Gray
Write-Host ""
