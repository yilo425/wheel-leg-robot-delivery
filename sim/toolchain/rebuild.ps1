<#
=============================================================================
 rebuild.ps1 -- 强制全量重编译（排除"旧产物干扰"）
-----------------------------------------------------------------------------
 什么时候要用：
   改了 tb 或 RTL 后，结果看起来"半新半旧" —— 说明可能跑的是旧的编译产物。
   本脚本会把 work 库整个删掉重建，保证编译的是当前源码。

 用法：
   cd D:\robot_trial\sim
   powershell -ExecutionPolicy Bypass -File toolchain\rebuild.ps1
=============================================================================
#>
[CmdletBinding()]
param(
    [string]$ModelsimHome = 'D:\modeltech64_2020.4',
    [string]$WorkDir = 'D:\gowin_lib\work'
)

$ErrorActionPreference = 'Continue'
function Say-Step { param($m) Write-Host "`n=== $m ===" -ForegroundColor Cyan }
function Say-OK   { param($m) Write-Host "  [ OK ] $m" -ForegroundColor Green }
function Say-Warn { param($m) Write-Host "  [WARN] $m" -ForegroundColor Yellow }
function Say-Err  { param($m) Write-Host "  [FAIL] $m" -ForegroundColor Red }

$simDir = $PSScriptRoot
if ((Split-Path $simDir -Leaf) -eq 'toolchain') { $simDir = Split-Path $simDir -Parent }
Set-Location $simDir

$VSIM = Join-Path $ModelsimHome 'win64\vsim.exe'
if (-not (Test-Path $VSIM)) {
    foreach ($c in @('D:\modeltech64_2020.4','C:\modeltech64_2020.4')) {
        $v = Join-Path $c 'win64\vsim.exe'
        if (Test-Path $v) { $VSIM = $v; break }
    }
}
if (-not (Test-Path $VSIM)) { Say-Err "找不到 vsim.exe"; exit 1 }
Say-OK "vsim = $VSIM"
Say-OK "sim  = $simDir"

# ---- 1) 删掉 work 库和旧的 .do 缓存 ----
Say-Step "1) 清理旧的编译产物"
if (Test-Path $WorkDir) {
    Write-Host "  删除 $WorkDir"
    Remove-Item -Recurse -Force $WorkDir -ErrorAction SilentlyContinue
}
foreach ($f in @('transcript','vsim.wlf','modelsim.ini.bak')) {
    Remove-Item -Force (Join-Path $simDir $f) -ErrorAction SilentlyContinue
}
# 删掉旧的结果文件，避免把上次的结果误当本次
Get-ChildItem $simDir -Filter '*_result.txt' -ErrorAction SilentlyContinue |
    ForEach-Object { Write-Host "  删除旧结果 $($_.Name)"; Remove-Item -Force $_.FullName }
Say-OK "清理完成"

# ---- 2) 全量编译（实时回显） ----
Say-Step "2) 全量编译（输出实时显示）"

# 坑（已修）：ModelSim 找不到 .do 文件时只打印
#   "invalid command name ..." 然后正常退出（exit 0），
#   只看退出码会误判成"编译通过"。所以必须先确认文件存在，
#   再用输出里的 COMPILE DONE 标记判断真的编完了。
foreach ($need in @('compile_all.do','run_sim.do','modelsim.ini')) {
    if (-not (Test-Path (Join-Path $simDir $need))) {
        Say-Err "缺少 $need —— 先跑 toolchain\emit_dofiles.ps1 生成"
        exit 1
    }
}
Say-OK "compile_all.do / run_sim.do / modelsim.ini 都在"

$lines = New-Object System.Collections.Generic.List[string]
& $VSIM -c -do compile_all.do 2>&1 | ForEach-Object {
    $line = [string]$_
    $lines.Add($line)
    Write-Host "    $line"
}
$txt = ($lines -join "`r`n")
Set-Content -Path (Join-Path $simDir 'compile_result.txt') -Value $txt -Encoding UTF8

$errCount   = ([regex]::Matches($txt, '\*\* Error')).Count
$didCompile = $txt -match 'COMPILE DONE'
$badCmd     = $txt -match 'invalid command name'

if ($errCount -gt 0) {
    Say-Err "编译有 $errCount 个 Error（详见 compile_result.txt）"
    exit 1
}
if ($badCmd -or -not $didCompile) {
    Say-Err "编译没有真正执行完（没看到 COMPILE DONE 标记）"
    Write-Host "  可能原因：compile_all.do 内容损坏，或工作目录不对" -ForegroundColor Yellow
    Write-Host "  当前目录: $(Get-Location)" -ForegroundColor Yellow
    exit 1
}
Say-OK "编译通过，0 Error，且确认完成（COMPILE DONE）"

# ---- 3) 跑测试（逐行实时回显，不要过滤！）----
#
# 重要教训（我在这里错了好几轮）：
#   曾经写成
#       & $VSIM ... | ForEach-Object { if ($_ -match 'RESULT:|checks,') { ... } }
#   想"只显示关键行"。但 tb 的 $display 输出**根本不经过这个管道**
#   （ModelSim 批处理模式下 stdout 抓不到），所以过滤器永远不匹配，
#   屏幕一片空白 —— 用户看到的就是"卡死了"，而实际上测试可能在正常跑。
#
#   现在改成：**不过滤**，vsim 说什么就显示什么。
#   真正的结果读 *_result.txt（tb 用 $fwrite 写的，不依赖 stdout）。
Say-Step "3) 跑测试（输出实时回显，结果稍后从 *_result.txt 读取）"
Write-Host "  注意：100MHz 下一个字节 8680 clk，8 个字节约 0.7ms 仿真时间，" -ForegroundColor Yellow
Write-Host "        最坏情况（全部超时）约 14ms。屏幕长时间不动是正常的。" -ForegroundColor Yellow
Write-Host ""

& $VSIM -c -do "do run_sim.do" 2>&1 | ForEach-Object { Write-Host "  $_" }

# ---- 4) 汇总结果文件 ----
Say-Step "4) 结果汇总（读 *_result.txt）"
$found = @(Get-ChildItem $simDir -Filter '*_result.txt' -ErrorAction SilentlyContinue)
if ($found.Count -eq 0) {
    Say-Err "没有生成任何 *_result.txt —— 说明 tb 没跑到结尾"
    Write-Host "  排查建议：" -ForegroundColor Yellow
    Write-Host "    1) 单独跑一个 tb 看是否报错：" -ForegroundColor Yellow
    Write-Host "       & '$VSIM' -c -do `"vsim -c -quiet -voptargs=+acc -L work -L gw5a work.tb_uart; run -all; quit -f`"" -ForegroundColor Yellow
    Write-Host "    2) 检查上面 vsim 的输出里有没有 Error loading design" -ForegroundColor Yellow
    exit 2
}
$allPass = $true
foreach ($f in $found) {
    Write-Host ""
    Write-Host "----- $($f.Name) -----" -ForegroundColor White
    $content = Get-Content $f.FullName
    $content | ForEach-Object {
        $c = if ($_ -match 'FAIL') { 'Red' } elseif ($_ -match 'OK') { 'Green' } else { 'Gray' }
        Write-Host "  $_" -ForegroundColor $c
    }
    if ($content -match 'RESULT: FAIL') { $allPass = $false }
}
Say-Step "结论"
if ($allPass) { Write-Host "  全部 PASS" -ForegroundColor Green; exit 0 }
else { Write-Host "  有 FAIL，请把上面内容发给 AI" -ForegroundColor Yellow; exit 2 }
