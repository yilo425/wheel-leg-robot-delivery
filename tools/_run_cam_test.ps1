$ErrorActionPreference = 'Continue'
$port = 'COM9'
$src  = 'D:\dsh-ws\robot_project\arm\_cam_test.py'

$code = (Get-Content $src -Raw) -replace "`r`n", "`n"

function Read-For($sp, $ms) {
    $t = ''
    $end = (Get-Date).AddMilliseconds($ms)
    while ((Get-Date) -lt $end) {
        try { $c = $sp.ReadExisting() } catch { $c = '' }
        if ($c) { $t += $c }
        Start-Sleep -Milliseconds 60
    }
    return $t
}

$sp = [System.IO.Ports.SerialPort]::new($port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 500
$sp.WriteTimeout = 5000
try { $sp.Open() } catch { Write-Host ("打开 " + $port + " 失败: " + $_.Exception.Message); exit 1 }
$sp.DtrEnable = $true
$sp.RtsEnable = $true

# 进 REPL
$sp.Write([byte[]](0x03, 0x03), 0, 2)
Start-Sleep -Milliseconds 300
$sp.Write([byte[]](0x0D, 0x0A), 0, 2)
Start-Sleep -Milliseconds 300
$sp.DiscardInBuffer()

# 粘贴模式：Ctrl-E -> 代码 -> Ctrl-D
$sp.Write([byte[]](0x05), 0, 1)
Start-Sleep -Milliseconds 200
$bytes = [System.Text.Encoding]::ASCII.GetBytes($code + "`n")
Write-Host ("发送 " + $bytes.Length + " 字节代码...")
for ($i = 0; $i -lt $bytes.Length; $i += 512) {
    $n = [Math]::Min(512, $bytes.Length - $i)
    $sp.Write($bytes, $i, $n)
    Start-Sleep -Milliseconds 25
}
Start-Sleep -Milliseconds 400
$sp.Write([byte[]](0x04), 0, 1)

Write-Host '--- 板子输出 ---'
$out = Read-For $sp 25000
Write-Host $out
Write-Host '--- 结束 ---'

$sp.Close()
