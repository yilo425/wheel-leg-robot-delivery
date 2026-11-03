#=============================================================================
# k230_repl.ps1 -- 不用 CanMV IDE，直接用串口和 K230 的 MicroPython REPL 对话
#
# 为什么写这个：本机没装 CanMV IDE，但 CanMV 固件自带 REPL（USB 虚拟串口），
# 任何串口工具都能进 —— 这里用 .NET 的 SerialPort，不依赖 pyserial。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tools\k230_repl.ps1
#   powershell -ExecutionPolicy Bypass -File tools\k230_repl.ps1 -Cmd "print(1+1)"
#   powershell -ExecutionPolicy Bypass -File tools\k230_repl.ps1 -Port COM7 -Cmd "os.listdir('/')"
#
# 参数：
#   -Port  串口名（默认 COM9，K230 的 USB 虚拟串口）
#   -Baud  波特率（USB-CDC 时无意义，给 115200）
#   -Cmd   要发到 REPL 的一行 Python（默认只做探活）
#   -Wait  发完读多少秒（默认 3）
#=============================================================================
param(
    [string]$Port = 'COM9',
    [int]$Baud = 115200,
    [string]$Cmd = "print('REPL OK')",
    [int]$Wait = 3
)

$ErrorActionPreference = 'Continue'

try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud,
          ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
} catch {
    Write-Host "[错误] 打开 $Port 失败: $($_.Exception.Message)"
    exit 1
}

$sp.ReadTimeout  = 500
$sp.WriteTimeout = 1000

# ★ 关键：K230 的 USB-CDC 必须拉 DTR/RTS 才会收发，
#   不拉的话 Open() 成功但一个字节都收不到（2026-10-27 实测）。
$sp.DtrEnable = $true
$sp.RtsEnable = $true

try {
    $sp.Open()
} catch {
    Write-Host "[错误] $Port 打开异常: $($_.Exception.Message)"
    exit 1
}

Write-Host "[信息] 已打开 $Port @ $Baud"
Start-Sleep -Milliseconds 300
$sp.DiscardInBuffer()

# 连发两次 Ctrl-C：打断卡里可能正在自动运行的程序，回到 REPL
$sp.Write([string][char]3)
Start-Sleep -Milliseconds 150
$sp.Write([string][char]3)
Start-Sleep -Milliseconds 400

$sp.WriteLine($Cmd)
Write-Host "[信息] 已发送: $Cmd"
Write-Host "-------- 输出 --------"

$deadline = (Get-Date).AddSeconds($Wait)
$acc = ''
while ((Get-Date) -lt $deadline) {
    $chunk = $sp.ReadExisting()
    if ($chunk) { $acc += $chunk }
    Start-Sleep -Milliseconds 80
}

$sp.Close()
Write-Host $acc
Write-Host "----------------------"
if (-not $acc.Trim()) {
    Write-Host "[结果] 什么都没收到 —— 这个口可能不是 K230 的 REPL，换一个口试"
    exit 2
}
if ($acc -match 'REPL OK') {
    Write-Host "[结果] REPL 可用 ✓"
} else {
    Write-Host "[结果] 收到数据（见上），自己判断是不是 MicroPython 的 >> 提示符"
}
