#=============================================================================
# setup_git.ps1 -- Git 一键配置
#-----------------------------------------------------------------------------
# 用法：右键本文件 -> 使用 PowerShell 运行
#   或者在这里敲：
#       powershell -ExecutionPolicy Bypass -File setup_git.ps1
#
# 它会问你 3 个问题（每个都能直接回车跳过），剩下的自动做完：
#   1. 你的名字      -> 写进提交记录
#   2. 你的邮箱      -> 写进提交记录
#   3. Gitee 仓库地址 -> 绑定并推送
#
# 全程不会删你的代码。放心一路回车。
#=============================================================================

$ErrorActionPreference = 'Continue'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

function Line { param($ch = '-') Write-Host ($ch * 62) -ForegroundColor DarkGray }
function Title { param($t) Write-Host ""; Line '='; Write-Host "  $t" -ForegroundColor Cyan; Line '=' }
function OK   { param($t) Write-Host "  [OK]   $t" -ForegroundColor Green }
function Warn { param($t) Write-Host "  [注意] $t" -ForegroundColor Yellow }
function Bad  { param($t) Write-Host "  [错误] $t" -ForegroundColor Red }
function Info { param($t) Write-Host "         $t" -ForegroundColor DarkGray }

#-----------------------------------------------------------------------------
# 判断一个值是不是"占位文本"（不是真人输入的东西）
#
#   为什么需要：如果脚本被**非交互式**调用（管道喂输入、重定向、
#   或被别的脚本调用），Read-Host 会返回不可预期的字符串。
#   实测踩到：管道调用时它把提示语里的"你的名字""你的Gitee邮箱"
#   当成用户输入写了进去，把 git 配置搞成占位值。
#-----------------------------------------------------------------------------
function Test-Placeholder([string]$s) {
    if ([string]::IsNullOrWhiteSpace($s)) { return $true }
    return ($s -match '你的|请填|请输入|Gitee邮箱|xxx|XXX|TODO|placeholder|<|>')
}

#-----------------------------------------------------------------------------
# 必须是真正的交互式控制台
#-----------------------------------------------------------------------------
if ([Console]::IsInputRedirected) {
    Write-Host ""
    Bad "本脚本需要交互式输入"
    Info "检测到标准输入被重定向（管道 / 重定向 / 被脚本调用），"
    Info "这种情况下 Read-Host 的返回值不可预期，可能写坏 git 配置。"
    Write-Host ""
    Info "请这样运行："
    Info "  ① 在 PowerShell 窗口里直接敲本脚本路径，或"
    Info "  ② 在文件资源管理器里右键本文件 -> 使用 PowerShell 运行"
    Write-Host ""
    exit 1
}

#-----------------------------------------------------------------------------
# 找到工程根目录（本脚本所在目录）
#-----------------------------------------------------------------------------
$root = $PSScriptRoot
if (-not $root) { $root = (Get-Location).Path }
Set-Location -LiteralPath $root

Title "Git 一键配置"

#=============================================================================
# 1) 检查 git
#=============================================================================
Write-Host ""
Write-Host "[1/6] 检查 Git ..." -ForegroundColor White

$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $git) {
    Bad "没有找到 git"
    Info "请先安装：https://git-scm.com/download/win"
    Info "一路默认安装，装完关掉这个窗口重新运行本脚本。"
    Write-Host ""
    Read-Host "按回车退出"
    exit 1
}
OK (git --version)

#=============================================================================
# 2) 确认这是不是一个 git 仓库
#=============================================================================
Write-Host ""
Write-Host "[2/6] 检查仓库 ..." -ForegroundColor White

git rev-parse --git-dir *> $null
if ($LASTEXITCODE -ne 0) {
    Warn "这个目录还不是 git 仓库"
    $a = Read-Host "  要现在初始化吗？(Y/n，直接回车=是)"
    if ($a -eq '' -or $a -match '^[Yy]') {
        git init -b main | Out-Null
        OK "已初始化"
    } else {
        Bad "已取消"
        Read-Host "按回车退出"
        exit 1
    }
} else {
    $branch = git branch --show-current
    OK "仓库正常，当前分支：$branch"
}

#=============================================================================
# 3) 名字
#=============================================================================
Write-Host ""
Write-Host "[3/6] 你的名字" -ForegroundColor White
Info "提交记录里会显示这个名字，队友靠它知道是谁改的"

$curNameOk = $false
$curName = git config --local user.name
if ($curName -and -not (Test-Placeholder $curName)) { $curNameOk = $true }

$hostName = $env:USERNAME
if (Test-Placeholder $hostName) { $hostName = $null }

Write-Host ""
if ($curNameOk) { Write-Host "      当前：$curName   （直接回车 = 不改）" -ForegroundColor Gray }
elseif ($hostName) { Write-Host "      直接回车 = 用 $hostName" -ForegroundColor Gray }
else { Write-Host "      请输入你的名字" -ForegroundColor Gray }

$name = $null
while (-not $name) {
    Write-Host -NoNewline "      > " -ForegroundColor Cyan
    $input1 = Read-Host
    if ([string]::IsNullOrWhiteSpace($input1)) {
        if ($curNameOk) { $name = $curName; OK "名字保持：$name" }
        elseif ($hostName) { $name = $hostName; OK "名字用：$name" }
        else { Warn "没输入，请再试一次（不能为空）" }
    } elseif (Test-Placeholder $input1) {
        Warn "「$input1」看起来是提示文字而不是真名字，请重新输入"
    } else {
        $name = $input1
    }
}

git config --local user.name $name
$verifyName = git config --local user.name
if ($verifyName -eq $name) { OK "名字已设为：$name" }
else { Bad "写入后读回不一致：'$verifyName' —— 请手动跑 git config --local user.name 你的名字" }

#=============================================================================
# 4) 邮箱
#=============================================================================
Write-Host ""
Write-Host "[4/6] 你的邮箱" -ForegroundColor White
Info "建议用你 Gitee 账号的邮箱，这样贡献统计里能看到你"
Info "不填也能用，只是提交记录里的邮箱是占位值"

$curMailOk = $false
$curMail = git config --local user.email
if ($curMail -and -not (Test-Placeholder $curMail) -and $curMail -match '@') { $curMailOk = $true }

Write-Host ""
if ($curMailOk) { Write-Host "      当前：$curMail   （直接回车 = 不改）" -ForegroundColor Gray }
else { Write-Host "      直接回车 = 跳过" -ForegroundColor Gray }

Write-Host -NoNewline "      > " -ForegroundColor Cyan
$mail = Read-Host

if ([string]::IsNullOrWhiteSpace($mail)) {
    if ($curMailOk) { OK "邮箱保持：$curMail" }
    else { Warn "跳过了邮箱（不影响使用，以后可以再跑一次本脚本填）" }
} elseif ($mail -notmatch '@') {
    Warn "「$mail」不像邮箱（没有 @），已跳过"
    Info "以后想改：git config --local user.email 你的邮箱"
} elseif (Test-Placeholder $mail) {
    Warn "看起来是提示文字，已跳过"
} else {
    git config --local user.email $mail
    $verifyMail = git config --local user.email
    if ($verifyMail -eq $mail) { OK "邮箱已设为：$mail" }
    else { Bad "写入后读回不一致，请手动设置" }
}

#-----------------------------------------------------------------------------
# 把已有提交的作者也改掉（如果原来是占位值）
#-----------------------------------------------------------------------------
if ($curMail -match 'localhost' -or -not $curMail) {
    $n = (git rev-list --count HEAD 2>$null)
    if ($LASTEXITCODE -eq 0 -and [int]$n -gt 0) {
        Write-Host ""
        Warn "已有 $n 次提交，作者是占位值"
        $a = Read-Host "      要把它们也改成你的名字/邮箱吗？(Y/n，直接回车=是)"
        if ($a -eq '' -or $a -match '^[Yy]') {
            Write-Host "      正在改写（可能要几秒）..." -ForegroundColor DarkGray
            git rebase --root --exec "git commit --amend --no-edit --reset-author" 2>&1 | Out-Null
            if ($LASTEXITCODE -eq 0) {
                OK "已改写"
                git log -1 --format='         最新提交作者 = %an <%ae>'
            } else {
                Warn "改写没成功，但不影响使用（可以跳过）"
                git rebase --abort 2>&1 | Out-Null
            }
        }
    }
}

#=============================================================================
# 5) 远程仓库
#=============================================================================
Write-Host ""
Write-Host "[5/6] Gitee 仓库" -ForegroundColor White

$remote = git remote get-url origin 2>$null
if ($remote) {
    OK "已绑定远程仓库：$remote"
} else {
    Info "现在你的仓库只在本地，队友拿不到。"
    Info "需要先去 https://gitee.com 建一个【私有】仓库（不要勾初始化）"
    Info "然后把地址粘贴进来（形如 git@gitee.com:你的账号/robot-fpga.git）"
    Write-Host ""
    Write-Host -NoNewline "      粘贴地址后回车（直接回车=先跳过，以后再说）" -ForegroundColor White
    Write-Host ""
    Write-Host -NoNewline "      > " -ForegroundColor Cyan
    $url = Read-Host

    if ($url -eq '') {
        Warn "跳过了远程仓库"
        Info "以后想推的时候，回到这个目录重新运行本脚本即可"
    } else {
        git remote add origin $url 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            OK "已绑定：$url"

            #---- 顺便检查 SSH 密钥 ----
            Write-Host ""
            Write-Host "      检查 SSH 密钥 ..." -ForegroundColor White
            $keyFile = Join-Path $env:USERPROFILE '.ssh\id_ed25519.pub'
            if (-not (Test-Path $keyFile)) {
                Warn "还没有 SSH 密钥"
                $a = Read-Host "      要现在生成吗？(Y/n，直接回车=是)"
                if ($a -eq '' -or $a -match '^[Yy]') {
                    $mailFor = if ($mail) { $mail } else { 'me@example.com' }
                    ssh-keygen -t ed25519 -C $mailFor -f (Join-Path $env:USERPROFILE '.ssh\id_ed25519') -N '""' 2>&1 | Out-Null
                    if (Test-Path $keyFile) { OK "已生成密钥" } else { Warn "生成失败，请手动跑 ssh-keygen" }
                }
            }

            if (Test-Path $keyFile) {
                Write-Host ""
                Line
                Write-Host "  ★ 把下面这一整行复制，粘到 Gitee：" -ForegroundColor Yellow
                Write-Host "     设置 -> SSH 公钥 -> 新建" -ForegroundColor Yellow
                Line
                Write-Host ""
                Get-Content $keyFile -Encoding UTF8 | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
                Write-Host ""
                Line
                Read-Host "  粘好了就按回车继续"
            }
        } else {
            Bad "绑定失败，地址可能写错了"
        }
    }
}

#=============================================================================
# 6) 推送
#=============================================================================
Write-Host ""
Write-Host "[6/6] 推送到远程" -ForegroundColor White

$remote = git remote get-url origin 2>$null
if (-not $remote) {
    Warn "没有远程仓库，跳过推送"
} else {
    $st = git status --short
    if ($st) {
        Write-Host "      有未提交的改动，先提交：" -ForegroundColor DarkGray
        git add -A
        git commit -m "配置：更新" 2>&1 | Out-Null
        OK "已提交"
    }

    Write-Host "      正在推送..." -ForegroundColor DarkGray
    $pushOut = git push -u origin HEAD 2>&1
    if ($LASTEXITCODE -eq 0) {
        OK "推送成功"
        # 建三个分支
        foreach ($b in 'fpga', 'vision', 'mech') {
            git branch $b 2>&1 | Out-Null
        }
        git push origin fpga vision mech 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) { OK "已建分支：fpga / vision / mech" }
    } else {
        Bad "推送失败"
        $pushOut | Select-Object -Last 6 | ForEach-Object { "         $_" }
        Write-Host ""
        Info "最常见原因：SSH 公钥还没粘到 Gitee"
        Info "或者仓库地址写错了"
    }
}

#=============================================================================
# 完成
#=============================================================================
Write-Host ""
Write-Host ""
Line '='
Write-Host "  配置完成！" -ForegroundColor Green
Line '='
Write-Host ""
Write-Host "  你的身份：" -ForegroundColor White
Write-Host "    名字 = $(git config --local user.name)"
Write-Host "    邮箱 = $(git config --local user.email)"
$remote = git remote get-url origin 2>$null
Write-Host "    远程 = $(if ($remote) { $remote } else { '（未设置）' })"
Write-Host ""
Write-Host "  以后每天就这 5 条命令：" -ForegroundColor White
Write-Host ""
Write-Host "    git pull                          拿队友的最新改动" -ForegroundColor Gray
Write-Host "    git status                        看自己改了哪些" -ForegroundColor Gray
Write-Host "    git add -A                        把改动加入本次提交" -ForegroundColor Gray
Write-Host "    git commit -m ""说明""              记录这次改动" -ForegroundColor Gray
Write-Host "    git push                          推给队友" -ForegroundColor Gray
Write-Host ""
Write-Host "  更详细的说明看： doc\git_basics.md" -ForegroundColor DarkGray
Write-Host ""
Read-Host "按回车关闭"
