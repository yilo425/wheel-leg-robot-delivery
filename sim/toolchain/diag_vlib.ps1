<#
=============================================================================
 diag_vlib.ps1
-----------------------------------------------------------------------------
 只做一件事：把 ModelSim 建库失败的真实报错抓出来。

 为什么单独写这个：
   setup 脚本里的探针之前把 vlib 的输出吞掉了，导致只看到"建库失败"
   却看不到原因，没法对症下药。这个脚本会把原始输出、退出码、
   环境变量、目录权限一次全打出来。

 用法：
   powershell -ExecutionPolicy Bypass -File diag_vlib.ps1
   powershell -ExecutionPolicy Bypass -File diag_vlib.ps1 -LibDir 'E:\gowin_lib'
=============================================================================
#>

[CmdletBinding()]
param(
    [string[]]$Candidates = @('D:\gowin_lib', 'E:\gowin_lib', 'D:\eda\gowin_lib'),
    [string]$LibDir
)

function Line { param($m) Write-Host $m }

#-----------------------------------------------------------------------------
# 0) ModelSim 位置
#-----------------------------------------------------------------------------
Line "===== 0) 找 ModelSim ====="
$msHomes = @()
foreach ($c in @('C:\modeltech64_2020.4', 'D:\modeltech64_2020.4')) {
    if (Test-Path (Join-Path $c 'win64\vlib.exe')) { $msHomes += $c }
}
$cmd = Get-Command vlib.exe -ErrorAction SilentlyContinue
if ($cmd) { $msHomes += (Split-Path (Split-Path $cmd.Source -Parent) -Parent) }
$msHomes = $msHomes | Select-Object -Unique

if (-not $msHomes) { Line "  找不到 vlib.exe"; exit 1 }
foreach ($h in $msHomes) { Line "  发现: $h" }
$MS = $msHomes[0]
$VLIB = Join-Path $MS 'win64\vlib.exe'
Line "  使用: $VLIB"

#-----------------------------------------------------------------------------
# 1) 环境变量
#-----------------------------------------------------------------------------
Line ""
Line "===== 1) 相关环境变量 ====="
foreach ($v in @('LM_LICENSE_FILE', 'MGLS_LICENSE_FILE', 'MODELSIM', 'MODELSIM_TCL', 'PATH')) {
    $val = [Environment]::GetEnvironmentVariable($v)
    if ($v -eq 'PATH') {
        $msInPath = ($val -split ';') | Where-Object { $_ -match 'modeltech|questa' }
        if ($msInPath) { Line ("  PATH 中的 ModelSim 项: {0}" -f ($msInPath -join ' | ')) }
        else           { Line "  PATH 中的 ModelSim 项: (无)" }
    } else {
        if ($val) { Line ("  {0} = {1}" -f $v, $val) }
        else      { Line ("  {0} = (未设置)" -f $v) }
    }
}

#-----------------------------------------------------------------------------
# 2) 许可文件是否真实存在
#-----------------------------------------------------------------------------
Line ""
Line "===== 2) 许可文件检查 ====="
foreach ($v in @('LM_LICENSE_FILE', 'MGLS_LICENSE_FILE')) {
    $val = [Environment]::GetEnvironmentVariable($v)
    if ($val) {
        $exists = Test-Path $val
        Line ("  {0} -> {1}  存在={2}" -f $v, $val, $exists)
        if ($exists) {
            Get-Content -LiteralPath $val -TotalCount 4 | ForEach-Object { Line "      | $_" }
        }
    }
}

#-----------------------------------------------------------------------------
# 3) 逐个候选目录试建库
#-----------------------------------------------------------------------------
Line ""
Line "===== 3) 试建库 ====="

$targets = @()
if ($LibDir) { $targets += $LibDir } else { $targets += $Candidates }
$targets += $env:TEMP

foreach ($T in $targets) {
    Line ""
    Line "--- 测试目录: $T ---"

    # 3.1 路径是否纯 ASCII
    $nonAscii = @()
    foreach ($ch in $T.ToCharArray()) { if ([int][char]$ch -gt 127) { $nonAscii += $ch } }
    if ($nonAscii.Count -gt 0) {
        Line ("  [!] 路径含非 ASCII 字符: {0}" -f ($nonAscii -join ''))
    } else {
        Line "  [ok] 路径是纯 ASCII"
    }

    # 3.2 PowerShell 能否写
    if (-not (Test-Path $T)) {
        try { New-Item -ItemType Directory -Force -Path $T -ErrorAction Stop | Out-Null; Line "  [ok] 目录已创建" }
        catch { Line "  [X] 目录无法创建: $($_.Exception.Message)"; continue }
    }
    try {
        $pf = Join-Path $T "_ps_probe_$PID.txt"
        Set-Content -LiteralPath $pf -Value 'x' -ErrorAction Stop
        Remove-Item -LiteralPath $pf -Force
        Line "  [ok] PowerShell 可写"
    } catch {
        Line "  [X] PowerShell 不可写: $($_.Exception.Message)"
        continue
    }

    # 3.3 真跑 vlib
    $libName = "diag_$PID"
    $libPath = Join-Path $T $libName
    Remove-Item -Recurse -Force $libPath -ErrorAction SilentlyContinue

    Push-Location $T
    try {
        Line "  > vlib $libName   (cwd=$T)"
        $out = & $VLIB $libName 2>&1
        $code = $LASTEXITCODE
        if (($out | Out-String).Trim()) {
            $out | ForEach-Object { Line "    | $_" }
        } else {
            Line "    (无输出)"
        }
        Line "    退出码 = $code"

        $infoF = Join-Path $libPath '_info'
        $qdbF  = Join-Path $libPath '_lib.qdb'
        if ((Test-Path $infoF) -and $code -eq 0) {
            Line "  [OK] 建库成功（_info 已生成）"
            Line ("       _info 大小 = {0} 字节" -f (Get-Item $infoF).Length)
            if (Test-Path $qdbF) {
                Line "       _lib.qdb 也已生成（说明该库被 vlog 编译过）"
            } else {
                Line "       _lib.qdb 尚未生成 —— 正常，要等 vlog 编译设计单元后才出现"
            }
            Line "       >>> 这个目录就是可以用的库目录"
        } else {
            Line "  [X] 建库失败（连 _info 都没写出来）"
        }
    } finally {
        Pop-Location
        Remove-Item -Recurse -Force $libPath -ErrorAction SilentlyContinue
    }
}

Line ""
Line "===== 诊断结束 ====="
Line "请把从 '===== 0)' 到这里的完整输出发给我。"
