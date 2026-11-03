#=============================================================================
# dl_gowin.ps1 -- 带断点续传 + 自动重试的下载脚本
#-----------------------------------------------------------------------------
# 用途：高云官网下载 Gowin EDA 安装包时，浏览器经常慢/断。
#       本脚本用 curl 的断点续传(-C -)，断了自动接着下，反复重试。
#
# 用法：
#     # 简单用法（把浏览器拿到的下载链接贴进来）
#     powershell -ExecutionPolicy Bypass -File dl_gowin.ps1 `
#         -Url "https://cdn.gowinsemi.com.cn/xxxxx.zip" `
#         -Out "D:\Gowin\Gowin_V1.9.12_Standard_x64.zip"
#
#     # 换个镜像/重试更多次
#     powershell -ExecutionPolicy Bypass -File dl_gowin.ps1 `
#         -Url "..." -Out "..." -MaxRetry 50
#
#     # 只看当前下到哪了
#     powershell -ExecutionPolicy Bypass -File dl_gowin.ps1 `
#         -Url "..." -Out "..." -Info
#
#-----------------------------------------------------------------------------
# 怎么拿到下载链接
#-----------------------------------------------------------------------------
#   1) 浏览器里登录高云账号，点到下载页
#   2) 按 F12 打开开发者工具 -> Network(网络) 标签
#   3) 点"下载"按钮，Network 里会出现一个大的请求
#   4) 右键那条 -> Copy -> Copy link address
#   5) 把链接贴到 -Url
#
#   ⚠️ 这类链接通常**带会话令牌、有时间限制**。如果脚本报 403/401，
#      说明过期了，重新复制一次。
#
#-----------------------------------------------------------------------------
# 为什么不用浏览器的"继续下载"
#-----------------------------------------------------------------------------
#   浏览器续传经常失败的原因：服务器不返回 Accept-Ranges，
#   或者浏览器没保存好临时文件。curl 的 -C - 会：
#     * 先探测服务器支不支持 Range
#     * 支持就从现有字节数 +1 开始请求
#     * 不支持就明确报错，而不是悄悄从头来
#
#-----------------------------------------------------------------------------
# 中文路径注意
#-----------------------------------------------------------------------------
#   **不要下到含中文的路径**（比如 C:\Users\<中文名>\Downloads\）。
#   一是历史经验里带中文的路径容易出问题，
#   二是以后安装 Gowin 也必须在纯 ASCII 路径。
#   建议直接下到 D:\Gowin\ 下面。
#=============================================================================

[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Url,
    [Parameter(Mandatory=$true)][string]$Out,
    [int]$MaxRetry = 30,          # 最多重试次数
    [int]$RetryDelaySec = 10,     # 每次重试前等几秒
    [switch]$Info                 # 只看当前进度，不下载
)

$ErrorActionPreference = 'Continue'

function Show-Progress {
    param([string]$Path)
    if (Test-Path $Path) {
        $f = Get-Item $Path
        $mb = [math]::Round($f.Length / 1MB, 1)
        Write-Host ("    已下载 {0} MB   (最后更新 {1})" -f $mb, $f.LastWriteTime.ToString('HH:mm:ss'))
    } else {
        Write-Host "    还没有文件"
    }
}

Write-Host "=============================================="
Write-Host " URL : $Url"
Write-Host " 保存到: $Out"
Write-Host "=============================================="

# ---- 只查进度 ----
if ($Info) {
    Show-Progress -Path $Out
    exit 0
}

# ---- 目标目录必须是纯 ASCII ----
$dir = Split-Path $Out -Parent
if ($dir -match '[^\x00-\x7F]') {
    Write-Host "[!] 警告：目标路径含非 ASCII 字符，容易出问题。"
    Write-Host "    建议改用纯 ASCII 路径，例如 D:\Gowin\ 下面。"
    Write-Host ""
}
if ($dir -and -not (Test-Path $dir)) {
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    Write-Host "  已创建目录: $dir"
}

# ---- 检查 curl ----
$curl = (Get-Command curl.exe -ErrorAction SilentlyContinue).Source
if (-not $curl) { $curl = 'curl.exe' }
Write-Host "  使用下载器: $curl"
Write-Host ""

# ---- 主循环：断了就续 ----
$attempt = 0
$lastSize = -1
$stuck = 0

while ($attempt -lt $MaxRetry) {
    $attempt++
    Write-Host ("--- 第 {0}/{1} 次尝试 ---" -f $attempt, $MaxRetry)

    # -C -        断点续传
    # --retry     传输层错误自动重试
    # --retry-delay / --retry-all-errors  连接被掐也重试
    # -L          跟随跳转
    # --connect-timeout 连不上别干等
    & $curl -L -C - `
        --retry 5 --retry-delay 5 --retry-all-errors `
        --connect-timeout 30 `
        --output $Out `
        $Url
    $code = $LASTEXITCODE

    if ($code -eq 0) {
        Write-Host ""
        Write-Host "[OK] 下载完成（退出码 0）"
        break
    }

    # curl 退出码含义（常见的几个）
    $reason = switch ($code) {
        6   { 'DNS 解析失败 —— 检查网络/网址' }
        7   { '连不上服务器 —— 可能被墙/服务器挂了' }
        18  { '传输中断 —— 服务器把连接掐了（常见）' }
        22  { 'HTTP 错误 —— 多半是链接过期（403/404），重新复制一次' }
        28  { '超时' }
        33  { '服务器不支持断点续传' }
        35  { 'SSL 握手失败 —— 试试关掉 VPN/代理' }
        default { "curl 退出码 $code" }
    }
    Write-Host ""
    Write-Host ("[!] 这次没成: {0}" -f $reason)
    Show-Progress -Path $Out

    # 判断是不是完全卡住不动了
    $cur = if (Test-Path $Out) { (Get-Item $Out).Length } else { 0 }
    if ($cur -eq $lastSize) {
        $stuck++
        Write-Host ("    连续 {0} 次没有增长" -f $stuck)
    } else {
        $stuck = 0
    }
    $lastSize = $cur

    if ($stuck -ge 5) {
        Write-Host ""
        Write-Host "[X] 连续 5 次一个字节都没下下来 —— 先停下来排查："
        Write-Host "    1) 链接是不是过期了？回浏览器重新复制一次"
        Write-Host "    2) 试试换个网络（手机热点）"
        Write-Host "    3) 干脆去要个百度网盘链接，比硬啃官网强"
        break
    }

    Write-Host ("    等 {0} 秒后重试..." -f $RetryDelaySec)
    Start-Sleep -Seconds $RetryDelaySec
}

Write-Host ""
Write-Host "=============================================="
if (Test-Path $Out) {
    $f = Get-Item $Out
    Write-Host (" 最终文件: {0}" -f $f.FullName)
    Write-Host (" 大小    : {0} MB" -f [math]::Round($f.Length / 1MB, 1))
    Write-Host "=============================================="
    Write-Host ""
    Write-Host " 如果下载完成，接着解压/安装。"
    Write-Host " ⚠️ 安装路径必须纯 ASCII，例如："
    Write-Host "      D:\Gowin\Gowin_V1.9.12_Standard_x64"
    Write-Host "    不要装到 C:\Users\<中文名>\... 下面。"
} else {
    Write-Host " [X] 没有下载到文件"
    Write-Host "=============================================="
    exit 1
}
