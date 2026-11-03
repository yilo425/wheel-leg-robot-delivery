#=============================================================================
# k230_push.ps1 -- 把一个文件推到 K230 的卡里（不用读卡器、不用 CanMV IDE）
#
# 原理：K230 的 CanMV 固件自带 REPL 和 ubinascii，所以可以用 base64
#       把文件分块写进去 —— 每块 ~120 字符，避免 REPL 行缓冲溢出。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tools\k230_push.ps1 `
#       -File arm\wifi_rc.py -Dest /sdcard/wifi_rc.py
#
#   # 只试连，不推文件
#   powershell -ExecutionPolicy Bypass -File tools\k230_push.ps1 -Probe
#
# 参数：
#   -Port  串口（默认 COM9）
#   -File  要推的本地文件（相对仓库根目录或绝对路径）
#   -Dest  板子上的目标路径（默认 /sdcard/<文件名>）
#   -Chunk base64 每块字符数（默认 120，别再大）
#=============================================================================
param(
    [string]$Port  = 'COM9',
    [string]$File  = '',
    [string]$Dest  = '',
    [int]$Chunk    = 120,
    [switch]$Probe
)

$ErrorActionPreference = 'Continue'

function Open-Port([string]$p) {
    $sp = New-Object System.IO.Ports.SerialPort $p, 115200,
          ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
    $sp.ReadTimeout  = 500
    $sp.WriteTimeout = 2000
    # ★ K230 的 USB-CDC 必须拉 DTR/RTS，否则一个字节都收不到
    $sp.DtrEnable = $true
    $sp.RtsEnable = $true
    $sp.Open()
    return $sp
}

function Send-Line($sp, [string]$line) {
    $sp.Write($line + "`r`n")
}

function Drain($sp, [int]$ms) {
    Start-Sleep -Milliseconds $ms
    try { return $sp.ReadExisting() } catch { return '' }
}

try {
    $sp = Open-Port $Port
} catch {
    Write-Host "[错误] 打开 $Port 失败: $($_.Exception.Message)"
    Write-Host "       确认 K230 已插在电脑上，且端口号对（设备管理器 -> 端口）"
    exit 1
}

Write-Host "[信息] 已打开 $Port @ 115200"

# Ctrl-C 两次，打断板上可能正在跑的程序，回到 REPL
$sp.Write([string][char]3); Start-Sleep -Milliseconds 150
$sp.Write([string][char]3); Start-Sleep -Milliseconds 300
$sp.DiscardInBuffer()

Send-Line $sp "print('PUSH_READY')"
$r = Drain $sp 800
if ($r -notmatch 'PUSH_READY') {
    Write-Host "[警告] 没看到 REPL 回应，收到的内容："
    Write-Host $r
    Write-Host "       还是要继续试？"
}

Send-Line $sp "import os, ubinascii; print('MPY_OK', len(dir(os)))"
$r = Drain $sp 1000
if ($r -notmatch 'MPY_OK') {
    Write-Host "[错误] ubinascii/os 不可用，放弃："
    Write-Host $r
    $sp.Close(); exit 2
}
Write-Host "[信息] REPL 正常"

if ($Probe) {
    Send-Line $sp "print(os.listdir('/sdcard')[:8])"
    Write-Host (Drain $sp 1200)
    $sp.Close()
    exit 0
}

if (-not $File) {
    Write-Host "[错误] 没给 -File"
    $sp.Close(); exit 1
}

$path = Resolve-Path $File -ErrorAction SilentlyContinue
if (-not $path) {
    Write-Host "[错误] 找不到文件: $File"
    $sp.Close(); exit 1
}
if (-not $Dest) {
    $Dest = '/sdcard/' + (Split-Path $path -Leaf)
}

$bytes = [System.IO.File]::ReadAllBytes($path)
$b64   = [Convert]::ToBase64String($bytes)
Write-Host ("[信息] {0}  {1} 字节 -> base64 {2} 字符" -f $path, $bytes.Length, $b64.Length)
Write-Host "[信息] 目标: $Dest"

# 打开文件
Send-Line $sp ("f=open('" + $Dest + "','wb')")
$null = Drain $sp 300

# 分块写
$total = [math]::Ceiling($b64.Length / $Chunk)
$n = 0
for ($i = 0; $i -lt $b64.Length; $i += $Chunk) {
    $len = [Math]::Min($Chunk, $b64.Length - $i)
    $piece = $b64.Substring($i, $len)
    Send-Line $sp ("f.write(ubinascii.a2b_base64('" + $piece + "'))")
    $n++
    if ($n % 20 -eq 0 -or $n -eq $total) {
        Write-Host ("   进度 {0}/{1}" -f $n, $total)
        $null = Drain $sp 120
    }
    Start-Sleep -Milliseconds 25
}

Send-Line $sp "f.close()"
$null = Drain $sp 400

# 校验大小
Send-Line $sp ("print('PUSHED', os.stat('" + $Dest + "')[6])")
$r = Drain $sp 1200
Write-Host $r

$sp.Close()

if ($r -match 'PUSHED\s+(\d+)') {
    $got = [int]$Matches[1]
    if ($got -eq $bytes.Length) {
        Write-Host ("[结果] 成功 ✓ 大小一致（{0} 字节）" -f $got)
        exit 0
    } else {
        Write-Host ("[结果] 大小不一致！本地 {0} / 板上 {1}" -f $bytes.Length, $got)
        exit 3
    }
} else {
    Write-Host "[结果] 没读到校验结果，手动确认："
    Write-Host ("       import os; print(os.stat('{0}'))" -f $Dest)
    exit 4
}
