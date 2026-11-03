#=============================================================================
# run_tb.ps1 -- 编译并运行【指定】Testbench（全清重建）
#
# 为什么写这个：
#   1) "Error loading design"（无细节）在本工程出现过多次，根因之一是
#      **库里有旧版本模块** —— 每次运行前把 work 库整个删掉重建可根治。
#
#   2) ⚠️ 但本脚本早先版本**把真正的错误信息过滤掉了**：
#      只 Select-String 'RUN |FAILED|Error loading'，
#      而 ModelSim 的详细报错行是 `# ** Error: ...`，正好不在过滤词里。
#      结果屏幕上只剩一句没头没尾的 "Error loading design"，极难查。
#      现在改为：**完整输出 tee 到日志文件**，不做任何过滤。
#
# 用法:
#   powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1 tb_top_integration
#   powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1            # 跑全部
#   powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1 -SkipClean # 不清库
#
# 路径配置（★ 队友注意）:
#   下面的默认值是 A 的机器路径，**你的机器上多半不一样**。
#   不要改这个脚本，改为在本目录建一个 toolchain.local.ps1：
#       Copy-Item toolchain.local.ps1.template toolchain.local.ps1
#   然后编辑里面的路径。该文件已被 .gitignore 排除，不会入库、不会冲突。
#   本脚本会**优先读它**，找不到才用下面的默认值。
#=============================================================================
param(
    [string]$Tb = '',
    [string]$Vsim = '',
    [string]$WorkDir = '',
    [switch]$SkipClean
)

$ErrorActionPreference = 'Continue'

#-----------------------------------------------------------------------------
# 读本机专属配置（如果存在）
#   优先级：显式传参 > toolchain.local.ps1 > 脚本内置默认值
#
#   ⚠️ 故意**不用 dot-source**（`. $localCfg`）：
#      PowerShell 默认执行策略（Restricted / AllSigned）会拒绝加载
#      **未签名**的 .ps1，dot-source 会直接抛错。实测报错：
#          "toolchain.local.ps1 is not digitally signed. You cannot run
#           this script on the current system."
#      本脚本自己是用 `-ExecutionPolicy Bypass -File` 跑的，
#      但**该策略不会传递给内部 dot-source 的文件**。
#
#   所以改为：读**文本** + 正则抽取路径。不执行任何代码，
#   天然免疫执行策略问题。
#-----------------------------------------------------------------------------
$localCfg = Join-Path $PSScriptRoot 'toolchain.local.ps1'
if (Test-Path $localCfg) {
    $cfgText = Get-Content -LiteralPath $localCfg -Raw
    $cfgHome = $null; $cfgWork = $null
    $m = [regex]::Match($cfgText, "(?m)^\s*ModelsimHome\s*=\s*['""]([^'""]+)['""]")
    if ($m.Success) { $cfgHome = $m.Groups[1].Value }
    $m = [regex]::Match($cfgText, "(?m)^\s*WorkDir\s*=\s*['""]([^'""]+)['""]")
    if ($m.Success) { $cfgWork = $m.Groups[1].Value }

    if (-not $Vsim -and $cfgHome) {
        # ⚠️ 不用 Join-Path：盘符不存在时（如 'Z:\...'）它会抛
        #    DriveNotFoundException，导致 $Vsim 保持为空 ->
        #    下面的早失败检查形同虚设 -> 脚本继续往下跑到报一堆无关的错。
        #    实测过：配置写错盘符时退出码是 0，看不出任何问题。
        $Vsim = "$cfgHome\win64\vsim.exe"
    }
    if (-not $WorkDir -and $cfgWork) { $WorkDir = $cfgWork }
    Write-Host "  已加载本机配置: toolchain.local.ps1" -ForegroundColor DarkGray
} else {
    Write-Host "  提示: 未找到 toolchain.local.ps1，使用脚本内置默认路径" -ForegroundColor DarkGray
    Write-Host "        如需自定义: Copy-Item toolchain.local.ps1.template toolchain.local.ps1" -ForegroundColor DarkGray
}

# 兜底默认值（A 的机器）
if (-not $Vsim)    { $Vsim    = 'D:\modeltech64_2020.4\win64\vsim.exe' }
if (-not $WorkDir) { $WorkDir = 'D:\gowin_lib\work' }

#-----------------------------------------------------------------------------
# 早失败：vsim 不存在就别往下走了，否则后面会报一堆看不懂的错
#-----------------------------------------------------------------------------
if (-not (Test-Path $Vsim)) {
    Write-Host ""
    Write-Host "  x 找不到 vsim: $Vsim" -ForegroundColor Red
    Write-Host "    请在本目录建 toolchain.local.ps1 并设置 ModelsimHome，例如：" -ForegroundColor Yellow
    Write-Host "        @{ ModelsimHome = 'C:\modeltech64_2020.4' }" -ForegroundColor Gray
    Write-Host "    模板见 toolchain.local.ps1.template" -ForegroundColor Gray
    exit 1
}

$sim = $PSScriptRoot | Split-Path -Parent      # ...\sim
Set-Location -LiteralPath $sim

$compileLog = Join-Path $sim 'compile.log'
$simLog     = Join-Path $sim 'sim.log'
$prj        = $sim | Split-Path -Parent      # ...\robot_project

#=============================================================================
# 0) 预检：参数名/端口名是否正确
#
#   ⚠️ 这一步是为了拦住本工程最"阴"的一类错误：
#      vlog **不检查**参数覆盖名和端口连接名是否存在于目标模块 ——
#      写错时 vlog 报 Errors: 0，直到 elaboration 才失败，
#      而 vsim 只给一句 "Error loading design"，没有行号没有名字。
#      真实踩到：top.v 给 pwm_motor 传了不存在的 .DUTY_WIDTH（应为 CNT_WIDTH）。
#      这类问题只有 vopt 报得清楚，本脚本用静态检查提前拦住。
#=============================================================================
Write-Host "===== 0) 预检：例化参数名/端口名 =====" -ForegroundColor Cyan
$chk = Join-Path $prj 'tools\check_instantiations.py'
if (Test-Path $chk) {
    # 找 python：PATH 里的 python/python3/py，再试 DSH 自带的那份
    $py = $null
    foreach ($cand in @('python', 'python3', 'py')) {
        $c = Get-Command $cand -ErrorAction SilentlyContinue
        if ($c -and $c.Source) { $py = $c.Source; break }
    }
    if (-not $py) {
        $guess = Join-Path $env:USERPROFILE '.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'
        if (Test-Path $guess) { $py = $guess }
    }

    if ($py) {
        # 先把输出和退出码都取到，再做判断 ——
        #   不要在管道里读 $LASTEXITCODE：管道里的 cmdlet 会让它变得不可靠，
        #   之前就因为这样报过一次"假警告"（检查器明明 PASS，脚本却说有问题）。
        $pyOut  = & $py $chk (Join-Path $prj 'rtl') (Join-Path $prj 'sim') 2>&1
        $pyCode = $LASTEXITCODE
        $pyOut | ForEach-Object { "  $_" }

        if ($null -eq $pyOut -or ($pyOut | Measure-Object).Count -eq 0) {
            Write-Host "  (预检无输出，跳过判断)" -ForegroundColor DarkGray
        } elseif ($pyCode -ne 0) {
            Write-Host "  ⚠️ 预检发现问题 —— 继续编译通常也会失败（见上面的 [ERROR] 行）" -ForegroundColor Yellow
        } else {
            Write-Host "  预检通过 ✓" -ForegroundColor Green
        }
    } else {
        Write-Host "  (未找到 python，跳过预检)" -ForegroundColor DarkGray
    }
} else {
    Write-Host "  (未找到 tools\check_instantiations.py，跳过)"
}

#=============================================================================
# 0.5) 预检：请求的 tb 是否在 compile_all.do 的文件列表里
#
#   ⚠️ 这一步是为了拦住一个**很容易忽略**的失败：
#      新增了一个 Testbench，把它加进了 emit_dofiles.ps1 的列表，
#      但**忘了重新生成 .do** —— 于是 compile_all.do 里没有它、
#      从来没编译过，而 run_sim.do 因为 TB_SEL 指定了它，
#      去加载 work.<tb> 时只报一句 "Error loading design"（无细节）。
#
#   真实踩到：tb_uart_frame 首次运行就是这样失败的。
#   而 vopt 单独跑却完全成功（0 errors）—— 因为 vopt 用的是库里的模块，
#   跟 compile_all.do 的列表无关，所以这个检查不能靠 vopt 发现。
#=============================================================================
if ($Tb) {
    $caDo = Join-Path $sim 'compile_all.do'
    if (Test-Path $caDo) {
        if (-not (Select-String -Path $caDo -Pattern ([regex]::Escape($Tb)) -Quiet)) {
            Write-Host ""
            Write-Host "  ✗✗ compile_all.do 里没有 '$Tb'！" -ForegroundColor Red
            Write-Host "     说明它从来没被编译过 -> vsim 只会报一句 Error loading design" -ForegroundColor Yellow
            Write-Host "     解决：先重新生成 .do ——" -ForegroundColor Yellow
            Write-Host "       powershell -ExecutionPolicy Bypass -File toolchain\emit_dofiles.ps1" -ForegroundColor Gray
            Write-Host "     （不需要传参；路径自动从 toolchain.local.ps1 读）" -ForegroundColor Gray
            Write-Host "     （或者把 $Tb 加进 emit_dofiles.ps1 的 `$Tbs 列表后再生成）" -ForegroundColor Yellow
            Write-Host ""
        } else {
            Write-Host "  预检: compile_all.do 里包含 $Tb ✓" -ForegroundColor Green
        }
    }
}

#=============================================================================
# 1) 清除旧库
#=============================================================================
Write-Host "===== 1) 清除旧库 =====" -ForegroundColor Cyan
if (-not $SkipClean) {
    foreach ($d in @($WorkDir, (Join-Path $sim 'work'))) {
        if (Test-Path $d) {
            Remove-Item -LiteralPath $d -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "  已删除 $d"
        }
    }
    Remove-Item -LiteralPath (Join-Path $sim 'transcript') -Force -ErrorAction SilentlyContinue
} else {
    Write-Host "  (跳过)"
}

#=============================================================================
# 2) 编译 —— 完整输出落盘，屏幕只高亮 Error
#=============================================================================
Write-Host "`n===== 2) 编译 =====" -ForegroundColor Cyan
& $Vsim -c -do "do compile_all.do; quit -f" *> $compileLog
$compileAll = Get-Content $compileLog -Raw
$errLines = Select-String -Path $compileLog -Pattern '\*\* Error|^# Errors: [1-9]|vlog failed'
if ($errLines) {
    Write-Host "  发现编译错误:" -ForegroundColor Red
    $errLines | Select-Object -First 20 | ForEach-Object { "    $($_.Line)" }
} else {
    Write-Host "  编译无 Error"
}
Write-Host "  完整编译日志: $compileLog"
if ($compileAll -notmatch 'COMPILE DONE') {
    Write-Host "  ⚠️ 没看到 'COMPILE DONE' 标记，编译可能不完整" -ForegroundColor Yellow
}

#=============================================================================
# 3) 运行 —— **完整输出落盘，不做任何过滤**
#=============================================================================
Write-Host "`n===== 3) 运行 =====" -ForegroundColor Cyan

#-----------------------------------------------------------------------------
# ⚠️ 关键：tb 名字必须用 **Tcl 变量** 传给 run_sim.do，不能走 `$argv！
#
#   踩过的坑（本工程出现过两次，第二次是我自己又写回去的）：
#       vsim -c -do "do run_sim.do tb_top_integration"
#   脚本里 `$argv 并不是 {tb_top_integration}，而是 {-c tb_top_integration}
#   —— **vsim 会把它自己的命令行开关也塞进 argv**。
#   于是 [lindex `$TB_LIST 0] 取到 "-c"，去加载 work.-c，
#   报一句没有任何细节的 "Error loading design"，查了很久才发现。
#
#   现在改成先 set TB_SEL 再 source 脚本，argv 里不传任何东西。
#-----------------------------------------------------------------------------
if ($Tb) {
    $do = "set TB_SEL $Tb; do run_sim.do"
} else {
    $do = "do run_sim.do"
}
Write-Host "  vsim -c -do `"$do`""
& $Vsim -c -do $do *> $simLog

Write-Host "  --- 运行日志（完整）---"
Get-Content $simLog | ForEach-Object { "  $_" }
Write-Host "  --- 运行日志结束 ---"
Write-Host "  完整运行日志: $simLog"

$runErr = Select-String -Path $simLog -Pattern '\*\* Error|Error loading|FAILED to run'
if ($runErr) {
    Write-Host "`n  ⚠️ 运行期错误:" -ForegroundColor Yellow
    $runErr | ForEach-Object { "    $($_.Line)" }
}

#=============================================================================
# 4) 结果文件
#=============================================================================
Write-Host "`n===== 4) 结果 =====" -ForegroundColor Cyan
Get-ChildItem -Filter *_result.txt | ForEach-Object {
    Write-Host "===== $($_.Name) ====="
    Get-Content $_.FullName | Select-String 'RESULT|checks,|\[FAIL\]'
}

#=============================================================================
# 5) 生成【可提交的】验证报告
#-----------------------------------------------------------------------------
# 为什么需要这个：
#   *_result.txt 每次跑都变，必须 gitignore（否则三个人各自跑一次
#   就产生"冲突"，冲突内容毫无意义，还会淹没真正的代码改动）。
#
#   但"三个人怎么知道对方验证过没有"这个需求是**真实**的，Git 本身解决不了。
#
# 做法：
#   把结果写进 verify/ 目录，文件名里带 【用户 + 机器 + commit】。
#   因为文件名唯一，三个人各生成各的文件，
#   git pull 之后只会"多一个文件" —— **永远不会冲突**。
#
# 这样 git log verify/ 就能回答：
#   "什么时候、谁、在哪台机器上、对哪个 commit、跑出了什么结果"
#=============================================================================
$repoRoot = (Resolve-Path (Join-Path $sim '..')).Path
$verifyDir = Join-Path $repoRoot 'verify'
if (-not (Test-Path $verifyDir)) {
    New-Item -ItemType Directory -Path $verifyDir -Force | Out-Null
}

$who     = if ($env:USERNAME)     { $env:USERNAME }     else { 'unknown' }
$machine = if ($env:COMPUTERNAME) { $env:COMPUTERNAME } else { 'unknown' }
$commit  = 'nogit'
$branch  = '-'
$dirty   = 'unknown'
Push-Location $repoRoot
try {
    $c = & git rev-parse --short HEAD 2>$null
    if ($LASTEXITCODE -eq 0 -and $c) { $commit = "$c".Trim() }
    $b = & git rev-parse --abbrev-ref HEAD 2>$null
    if ($LASTEXITCODE -eq 0 -and $b) { $branch = "$b".Trim() }
    $st = & git status --porcelain 2>$null
    $dirty = if ($st) { 'dirty（有未提交改动）' } else { 'clean' }
} finally { Pop-Location }

$safeName = ("{0}_{1}_{2}" -f $who, $machine, $commit) -replace '[^\w\-]', '_'
if ($dirty -ne 'clean') { $safeName = "$safeName`_dirty" }
$reportPath = Join-Path $verifyDir "$safeName.md"

$rows = @()
$totChecks = 0; $totErrors = 0; $fails = 0; $tbCount = 0
Push-Location $sim
try {
    Get-ChildItem -Filter *_result.txt | Sort-Object Name | ForEach-Object {
        $txt = Get-Content $_.FullName -Raw -EA SilentlyContinue
        if (-not $txt) { return }
        $m = [regex]::Match($txt, ':\s*(\d+)\s+checks,\s*(\d+)\s+errors')
        $ck = if ($m.Success) { [int]$m.Groups[1].Value } else { -1 }
        $er = if ($m.Success) { [int]$m.Groups[2].Value } else { -1 }
        $res = if ($txt -match 'RESULT:\s*PASS') { 'PASS' } else { 'FAIL' }
        $tb = $_.BaseName -replace '_result$', ''
        if ($ck -ge 0) { $totChecks += $ck; $totErrors += $er }
        $tbCount++
        if ($res -ne 'PASS') { $fails++ }
        $rows += ("| {0} | {1} | {2} | {3} |" -f $tb, $(if ($ck -ge 0) { $ck } else { '?' }), $(if ($er -ge 0) { $er } else { '?' }), $res)
    }
} finally { Pop-Location }

$overall = if ($fails -eq 0 -and $tbCount -gt 0) { '**PASS**' } else { '**FAIL**' }

#-----------------------------------------------------------------------------
# 守卫：一个 tb 都没跑到（比如 tb 名字写错、库没建起来）就不要生成报告。
#
#   实测过：用 `run_tb.ps1 __nope__` 会生成一份 "0 个 tb / 结论 FAIL" 的报告，
#   这种报告没有任何信息量，却会被提交上去污染 verify/，
#   让队友以为"这个 commit 验证失败了"—— 实际上根本没跑。
#-----------------------------------------------------------------------------
if ($tbCount -eq 0) {
    Write-Host ""
    Write-Host "  ! 没有解析到任何 *_result.txt —— 不生成验证报告" -ForegroundColor Yellow
    Write-Host "    可能原因：tb 名字写错、库没建起来、或仿真在早期就失败了" -ForegroundColor Yellow
    Write-Host "    请检查上面的 3) 运行日志" -ForegroundColor Yellow
    return
}

$out = @()
$out += "# 验证报告"
$out += ""
$out += "> 本文件由 `sim\toolchain\run_tb.ps1` 自动生成，**可以提交**。"
$out += "> 文件名带【用户 + 机器 + commit】，所以三个人各生成各的，**永远不会冲突**。"
$out += "> 想看谁验证过什么：``git log --oneline verify/``"
$out += ""
$out += "| 项 | 值 |"
$out += "|---|---|"
$out += "| 时间 | $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') |"
$out += "| 机器 | $machine |"
$out += "| 用户 | $who |"
$out += "| 分支 | $branch |"
$out += "| commit | ``$commit`` ($dirty) |"
$out += "| 仿真工具 | $Vsim |"
$out += "| 工作库 | $WorkDir |"
$out += "| **总计** | **$tbCount 个 tb / $totChecks 项检查 / $totErrors 个错误** |"
$out += "| **结论** | $overall |"
$out += ""
$out += "## 各 Testbench"
$out += ""
$out += "| Testbench | checks | errors | 结果 |"
$out += "|---|---|---|---|"
$out += $rows
$out += ""
$out += "---"
$out += ""
$out += "**队友注意**：如果你在 ``main`` 上看到某个 tb 是 FAIL，说明有人提交了没跑通的代码 ——"
$out += "在群里问一下，别直接在上面继续改。"

Set-Content -Path $reportPath -Value $out -Encoding UTF8

Write-Host "`n===== 5) 验证报告 =====" -ForegroundColor Cyan
Write-Host "  已生成（可提交）: verify\$safeName.md"
Write-Host "  结论: $overall  ($tbCount 个 tb / $totChecks 项 / $totErrors 个错误)" -ForegroundColor $(if ($fails -eq 0) { 'Green' } else { 'Red' })
Write-Host ""
Write-Host "  提交命令:" -ForegroundColor Gray
Write-Host "     git add verify/ ; git commit -m \"验证: $commit $overall\"" -ForegroundColor Gray
Write-Host "     想看谁验证过什么: git log --oneline verify/" -ForegroundColor Gray

