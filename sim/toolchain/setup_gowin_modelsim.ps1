<#
=============================================================================
 setup_gowin_modelsim.ps1
-----------------------------------------------------------------------------
 作用：把高云 Gowin 和 ModelSim 真正"联起来"。

 解决的问题（按出现频率排序）：
   1. ModelSim 里找不到 PLL / OSC / GSR 等原语
      原因：Gowin 的仿真库从来没有编译进 ModelSim。
      修复：把 <Gowin>\IDE\simlib\gw5a\prim_sim.v 编译成名为 prim_gw5a 的库，
            并在工程自己的 modelsim.ini 里建立映射。

   2. 报错 init_dbinfo() DATABASE ERROR: (sqlite3_open ...) unable to open database file
      原因：ModelSim 2020.4 的库管理器用 ANSI 编码打开路径，
            路径里有中文/非 ASCII 字符时必然失败（例如用户名是中文）。
      修复：把 *work 库* 放到纯 ASCII 路径。源码可以留在中文路径下，
            只要库目录和目标文件是 ASCII 就行。

   3. 手工把 .v 文件拷来拷去
      原因：没有统一的编译脚本，两边各维护一份文件列表。
      修复：本脚本生成的 compile_all.do 直接从 Gowin 工程目录读取源码。

 用法（普通权限即可，不需要管理员）：
     powershell -ExecutionPolicy Bypass -File setup_gowin_modelsim.ps1
     powershell -ExecutionPolicy Bypass -File setup_gowin_modelsim.ps1 -Check

 参数：
     -GowinHome   指定 Gowin 安装根目录（默认自动探测）
     -ModelsimHome 指定 ModelSim 安装根目录（默认自动探测）
     -LibDir      编译好的 Gowin 库存放位置（默认 <ModelsimHome>\..\gowin_lib）
     -ProjectDir  工程 sim 目录（默认脚本所在目录）
     -Check       只做环境诊断，不改动任何文件
=============================================================================
#>

[CmdletBinding()]
param(
    [string]$GowinHome,
    [string]$ModelsimHome,
    [string]$LibDir,
    [string]$ProjectDir,
    [switch]$Check
)

$ErrorActionPreference = 'Stop'

#-----------------------------------------------------------------------------
# 小小的输出helper
#-----------------------------------------------------------------------------
function Say-OK   { param($m) Write-Host "  [ OK ] $m" -ForegroundColor Green }
function Say-Warn { param($m) Write-Host "  [WARN] $m" -ForegroundColor Yellow }
function Say-Err  { param($m) Write-Host "  [FAIL] $m" -ForegroundColor Red }
function Say-Step { param($m) Write-Host "`n=== $m ===" -ForegroundColor Cyan }

#-----------------------------------------------------------------------------
# 判断路径是否纯 ASCII（ModelSim 2020.4 的硬性要求）
#-----------------------------------------------------------------------------
function Test-AsciiPath {
    param([string]$Path)
    foreach ($ch in $Path.ToCharArray()) {
        if ([int][char]$ch -gt 127) { return $false }
    }
    return $true
}

#-----------------------------------------------------------------------------
# 真的试着往目录里写一个临时文件。只看 ACL 是不够的：
#   - 有些目录 ACL 看着能写，实际会被 UAC/权限虚拟化拦住
#   - C:\ 根目录就是典型：非管理员写不进去，但 ACL 上写着 Authenticated Users 可改
# 所以必须实际写一次才算数。
#-----------------------------------------------------------------------------
function Test-WritableDir {
    param([string]$Path)
    if (-not $Path) { return $false }
    if (-not (Test-Path $Path)) { return $false }
    $probe = Join-Path $Path (".dsh_write_probe_{0}" -f $PID)
    try {
        Set-Content -LiteralPath $probe -Value 'probe' -ErrorAction Stop
        Remove-Item -LiteralPath $probe -Force -ErrorAction SilentlyContinue
        return $true
    } catch {
        return $false
    }
}

#-----------------------------------------------------------------------------
# 找工程 sim 目录：从脚本位置向上找 "包含 tb_*.v 的目录"
#   这样从 sim\toolchain\ 里运行也能正确定位到 sim\
#-----------------------------------------------------------------------------
function Find-ProjectDir {
    param([string]$StartDir)
    $d = $StartDir
    for ($i = 0; $i -lt 4 -and $d; $i++) {
        if (Get-ChildItem -Path $d -Filter 'tb_*.v' -ErrorAction SilentlyContinue) { return $d }
        $parent = Split-Path $d -Parent
        if (-not $parent -or $parent -eq $d) { break }
        $d = $parent
    }
    return (Split-Path $StartDir -Parent)
}

#-----------------------------------------------------------------------------
# 0) 定位工具
#-----------------------------------------------------------------------------
#-----------------------------------------------------------------------------
# 读本机专属配置（toolchain.local.ps1）—— 与 emit_dofiles.ps1 / run_tb.ps1 同一份
#
#   ⚠️ 为什么必须读它（真实踩过的坑，2026-10-15）：
#     本脚本原来**无视**工程实际用的库位置，一律建成
#         <ModelSim 同级>\gowin_lib\prim_gw5a
#     而工程实际用的是
#         <ModelSim>\gw5a            （emit_dofiles.ps1 生成 .do 时写死的名字）
#     于是"照文档重跑一次 setup"的后果是：
#         * 库名从 gw5a 变成 prim_gw5a、路径也变了 -> .do 被重新指向新库，
#           和原来那份库**分家**，而且**不报任何错**；
#         * 更糟的是它会从**旧版本 Gowin** 重新编译原语（见下面 Find-Gowin）。
#     现在：配置里给了 GowinSimLib 就按它来（路径最后一段 = 库名），
#           没给才退回原来的默认行为（队友没建配置文件时行为不变）。
#-----------------------------------------------------------------------------
$script:cfgGowinSimLib = $null

$localCfg = Join-Path $PSScriptRoot 'toolchain.local.ps1'
if (Test-Path $localCfg) {
    $cfgText = Get-Content -LiteralPath $localCfg -Raw
    function Get-LocalCfg([string]$key) {
        $m = [regex]::Match($cfgText, ('(?m)^\s*' + $key + '\s*=\s*[''""]([^''""]+)[''""]'))
        if ($m.Success) { return $m.Groups[1].Value }
        return $null
    }
    $script:cfgGowinSimLib = Get-LocalCfg 'GowinSimLib'
    $script:cfgWorkDir     = Get-LocalCfg 'WorkDir'
    if (-not $GowinHome)    { $GowinHome    = Get-LocalCfg 'GowinHome' }
    if (-not $ModelsimHome) { $ModelsimHome = Get-LocalCfg 'ModelsimHome' }
    Write-Host "  已加载本机配置: toolchain.local.ps1" -ForegroundColor DarkGray
    if ($script:cfgGowinSimLib) {
        Write-Host "  GowinSimLib (来自配置): $($script:cfgGowinSimLib)" -ForegroundColor DarkGray
    }
} else {
    Write-Host "  提示: 未找到 toolchain.local.ps1，将使用脚本内置默认位置" -ForegroundColor DarkGray
    Write-Host "        建议先建一份: Copy-Item toolchain.local.ps1.template toolchain.local.ps1" -ForegroundColor DarkGray
}

Say-Step "0) 定位 Gowin 与 ModelSim"

#-----------------------------------------------------------------------------
# 从目录名里抽版本号，用于"装了多个 Gowin 时取最新那个"
#   'Gowin_V1.9.12_x64' -> 1.9.12
#   'Gowin_V1.9.11.03_Education_x64' -> 1.9.11.3
#   抽不出来就返回 0.0（排最后，但不会抛异常）
#-----------------------------------------------------------------------------
function ConvertTo-GowinVersion {
    param([string]$Name)
    $m = [regex]::Match($Name, '(\d+(?:\.\d+)+)')
    if ($m.Success) {
        try { return [version]$m.Groups[1].Value } catch { }
    }
    return [version]'0.0'
}

function Find-Gowin {
    param([string]$Hint)
    #---- 1) 显式指定 / 环境变量：优先级最高，原样采用 ----
    $explicit = @()
    if ($Hint)           { $explicit += $Hint }
    if ($env:GOWIN_HOME) { $explicit += $env:GOWIN_HOME }
    if ($env:GOWIN_ROOT) { $explicit += $env:GOWIN_ROOT }
    foreach ($c in $explicit) {
        if ($c -and (Test-Path (Join-Path $c 'IDE\simlib'))) { return $c }
    }

    #---- 2) 自动探测：★ 装了多个版本时**取版本号最大的那个** ----
    #   ⚠️ 不能"取第一个找到的"：目录枚举顺序不保证，本机同时装着
    #      V1.9.11.03_Education 和 V1.9.12，取错就会用**旧版本**的
    #      prim_sim.v 建库 —— 仿真模型和综合器不是一个版本，而且不报错。
    #      （这正是本工程 gw5a 库曾经是 V1.9.11 建的原因。）
    $found = @()
    foreach ($root in @('D:\Gowin', 'C:\Gowin', 'E:\Gowin', 'D:\', 'C:\', 'E:\')) {
        if (Test-Path $root) {
            $found += Get-ChildItem -Path $root -Directory -ErrorAction SilentlyContinue |
                      Where-Object { $_.Name -like 'Gowin_V*' } |
                      Select-Object -ExpandProperty FullName
        }
    }
    $found = @($found | Select-Object -Unique)
    if ($found.Count -gt 1) {
        Write-Host ("  发现 $($found.Count) 个 Gowin 安装，按版本号取最新：") -ForegroundColor DarkGray
        $found | ForEach-Object { Write-Host ("      $_") -ForegroundColor DarkGray }
    }
    $sorted = @($found | Sort-Object -Property @{ Expression = { ConvertTo-GowinVersion (Split-Path $_ -Leaf) } } -Descending)
    foreach ($c in $sorted) {
        if ($c -and (Test-Path (Join-Path $c 'IDE\simlib'))) { return $c }
    }
    return $null
}

function Find-ModelSim {
    param([string]$Hint)
    $cands = @()
    if ($Hint) { $cands += $Hint }
    if ($env:MODELSIM_HOME) { $cands += $env:MODELSIM_HOME }
    # 从 PATH 里找 vsim.exe，再回退两级
    $vsimCmd = Get-Command vsim.exe -ErrorAction SilentlyContinue
    if ($vsimCmd) { $cands += (Split-Path (Split-Path $vsimCmd.Source -Parent) -Parent) }
    foreach ($root in @('D:\', 'C:\', 'E:\')) {
        if (Test-Path $root) {
            $cands += Get-ChildItem -Path $root -Directory -ErrorAction SilentlyContinue |
                      Where-Object { $_.Name -like 'modeltech*' -or $_.Name -like 'questa*' } |
                      Select-Object -ExpandProperty FullName
        }
    }
    foreach ($c in $cands) {
        if ($c -and (Test-Path (Join-Path $c 'win64\vsim.exe'))) { return $c }
        if ($c -and (Test-Path (Join-Path $c 'win32\vsim.exe'))) { return $c }
        if ($c -and (Test-Path (Join-Path $c 'win64\vlog.exe'))) { return $c }
    }
    return $null
}

$GowinHome    = Find-Gowin    -Hint $GowinHome
$ModelsimHome = Find-ModelSim -Hint $ModelsimHome

if (-not $GowinHome)    { Say-Err "没找到 Gowin 安装目录（找 IDE\simlib 这一级）。请用 -GowinHome 指定。" }
else                    { Say-OK  "Gowin    : $GowinHome" }
if (-not $ModelsimHome) { Say-Err "没找到 ModelSim 安装目录（找 win64\vsim.exe 这一级）。请用 -ModelsimHome 指定。" }
else                    { Say-OK  "ModelSim : $ModelsimHome" }

if (-not $GowinHome -or -not $ModelsimHome) {
    Write-Host "`n定位失败，先确认安装路径再重跑。"
    exit 1
}

#-----------------------------------------------------------------------------
# 1) 找器件族的原语文件
#-----------------------------------------------------------------------------
Say-Step "1) 检查 Gowin 仿真库文件"

$famCandidates = @('gw5a','gw5ast','gw2a','gw1n')
$PrimFile = $null
$Family   = $null
foreach ($fam in $famCandidates) {
    $p = Join-Path $GowinHome "IDE\simlib\$fam\prim_sim.v"
    if (Test-Path $p) {
        # gw5a 优先；但别把 gw5ast 当成 gw5a
        $PrimFile = $p
        $Family   = $fam
        break
    }
}
if (-not $PrimFile) {
    Say-Err "没找到任何 prim_sim.v。请确认 Gowin 安装完整。"
    exit 1
}
Say-OK "器件族 : $Family"
Say-OK "原语文件: $PrimFile"
Say-OK ("文件大小: {0:N0} 字节" -f (Get-Item $PrimFile).Length)

# 顺带确认关键原语真的在里面
$primText = Get-Content $PrimFile -Raw
foreach ($need in @('module PLL', 'module OSC', 'module GSR')) {
    if ($primText -match [regex]::Escape($need)) { Say-OK "包含 $need" }
    else { Say-Warn "没找到 '$need'，该器件族可能不用它" }
}

$vlib = Join-Path $ModelsimHome 'win64\vlib.exe'
$vlog = Join-Path $ModelsimHome 'win64\vlog.exe'
$vmap = Join-Path $ModelsimHome 'win64\vmap.exe'
foreach ($tool in @($vlib, $vlog, $vmap)) {
    if (-not (Test-Path $tool)) { Say-Err "缺少 $tool"; exit 1 }
}
Say-OK "vlib / vlog / vmap 都在"

#-----------------------------------------------------------------------------
# 2) 选一个"纯 ASCII 且真的可写"的库存放位置
#    两个条件缺一不可：非 ASCII 会让 ModelSim 建库失败（ANSI 路径 bug），
#    不可写会让 vlib 直接报 Permission denied。
#-----------------------------------------------------------------------------
Say-Step "2) 选择 Gowin 库的存放位置（必须纯 ASCII 且可写）"

if ($LibDir) {
    if (-not (Test-AsciiPath $LibDir)) {
        Say-Err "指定的 -LibDir 含非 ASCII 字符：$LibDir"
        exit 1
    }
    if (-not (Test-WritableDir $LibDir)) {
        Say-Err "指定的 -LibDir 不可写：$LibDir"
        exit 1
    }
    Say-OK "库路径 : $LibDir  (指定，纯 ASCII 且可写)"
} else {
    # 候选按优先级排列：
    #   0. ★ toolchain.local.ps1 里 GowinSimLib 的父目录 —— **本工程实际用的位置**
    #      （不把它排第一的话，重跑本脚本会把库建到别处，.do 再被重新指向新库，
    #        和现有那份库分家且不报错）
    #   1. ModelSim 同级的 gowin_lib（同盘、访问快）
    #   2. ModelSim 目录内的 gowin_lib
    #   3. D:\gowin_lib / E:\gowin_lib（非系统盘通常不需要管理员权限）
    $msParent = Split-Path $ModelsimHome -Parent
    $cands = @()
    if ($script:cfgGowinSimLib) { $cands += (Split-Path $script:cfgGowinSimLib -Parent) }
    $cands += (Join-Path $msParent 'gowin_lib')
    $cands += (Join-Path $ModelsimHome 'gowin_lib')
    foreach ($drv in @('D:\', 'E:\')) {
        if (Test-Path $drv) { $cands += (Join-Path $drv 'gowin_lib') }
    }

    $picked = $null
    foreach ($c in $cands) {
        $ascii = Test-AsciiPath $c
        if (-not $ascii) {
            Write-Host ("    跳过（含非 ASCII）: {0}" -f $c) -ForegroundColor DarkGray
            continue
        }
        # 目录可能还不存在，先尝试创建
        if (-not (Test-Path $c)) {
            try { New-Item -ItemType Directory -Force -Path $c -ErrorAction Stop | Out-Null }
            catch {
                Write-Host ("    跳过（无法创建）: {0}" -f $c) -ForegroundColor DarkGray
                continue
            }
        }
        if (Test-WritableDir $c) {
            $picked = $c
            Write-Host ("    选中: {0}" -f $c) -ForegroundColor DarkGray
            break
        } else {
            Write-Host ("    跳过（不可写）: {0}" -f $c) -ForegroundColor DarkGray
        }
    }

    if (-not $picked) {
        Say-Err "没能自动找到既纯 ASCII 又可写的目录。"
        Write-Host "       请手工指定一个，例如： -LibDir 'D:\gowin_lib'" -ForegroundColor Yellow
        Write-Host "       （注意别用 C:\ 根目录或 Program Files，非管理员写不进去）" -ForegroundColor Yellow
        exit 1
    }
    $LibDir = $picked
    Say-OK "库路径 : $LibDir  (自动选择，纯 ASCII 且可写)"
}

$asciiMs = Test-AsciiPath $ModelsimHome
if ($asciiMs) { Say-OK "ModelSim 路径纯 ASCII，OK" }
else { Say-Warn "ModelSim 安装在非 ASCII 路径下，某些功能可能异常：$ModelsimHome" }

#-----------------------------------------------------------------------------
# 2.5) 关键验证：ModelSim 到底能不能在这个目录建库
#
# 前面 Test-WritableDir 只证明了"PowerShell 能写"。
# 但 ModelSim 是通过 sqlite 建 _lib.qdb 的，能不能成还得真让它试一次。
# 这一步就是整个问题的分水岭：
#   建库成功 -> 中文路径问题已规避，后面都能通
#   建库失败 -> 库路径仍在非 ASCII 路径下，必须先换路径
# 所以这一步在 -Check 模式下也照样执行（只建一个临时小库再删掉）。
#-----------------------------------------------------------------------------
Say-Step "2.5) 验证 ModelSim 能否在该目录建库（关键）"

$probeName = "dsh_probe_$PID"
$probePath = Join-Path $LibDir $probeName
$null = New-Item -ItemType Directory -Force -Path $probePath -ErrorAction SilentlyContinue

Push-Location $LibDir
try {
    Write-Host "  执行: vlib $probeName   (工作目录 $LibDir)" -ForegroundColor DarkGray
    $probeOut = & $vlib $probeName 2>&1
    $probeExit = $LASTEXITCODE
    $probeText = ($probeOut | Out-String)

    # ---- 无论成败都完整打印 vlib 的输出 ----
    #   这是诊断里最有价值的信息，绝不能被吞掉。
    if ($probeText.Trim()) {
        Write-Host "  --- vlib 输出开始 ---" -ForegroundColor DarkGray
        $probeOut | ForEach-Object { Write-Host "  | $_" }
        Write-Host "  --- vlib 输出结束 (exit=$probeExit) ---" -ForegroundColor DarkGray
    } else {
        Write-Host "  (vlib 没有输出内容, exit=$probeExit)" -ForegroundColor DarkGray
    }

    # ---- 判据修正（重要）----
    #   vlib 建库成功时最先写出的标志文件是 _info；
    #   _lib.qdb 要等 vlog 真正编译过设计单元之后才会出现。
    #   所以必须查 _info —— 查 _lib.qdb 会把"成功"误判成"失败"。
    $infoOk     = Test-Path (Join-Path $probePath '_info')
    $qdbOk      = Test-Path (Join-Path $probePath '_lib.qdb')
    $hasDbError = $probeText -match 'DATABASE ERROR|unable to open database'
    $hasPermErr = $probeText -match 'Permission denied|EACCES|access is denied|拒绝访问'
    $hasLicErr  = $probeText -match 'license|License|LICENSE'

    Write-Host "  检测: _info=$infoOk  _lib.qdb=$qdbOk  DATABASE错误=$hasDbError  权限错误=$hasPermErr  许可错误=$hasLicErr" -ForegroundColor DarkGray

    if ($infoOk -and -not $hasDbError) {
        Say-OK "建库成功（_info 已生成）—— 路径问题已规避"
        if (-not $qdbOk) {
            Write-Host "       (_lib.qdb 尚未出现属正常：要等 vlog 编译过设计单元才生成)" -ForegroundColor DarkGray
        }
    } else {
        Say-Err "ModelSim 建库失败！"
        Write-Host ""
        # ---- 给出针对性的下一步，而不是笼统建议 ----
        if ($hasPermErr) {
            Write-Host "  诊断: 目录权限不足（不是中文路径问题）。" -ForegroundColor Yellow
            Write-Host "  对策:" -ForegroundColor Yellow
            Write-Host "    1) 换一个你有写权限的位置，例如 -LibDir 'E:\gowin_lib'" -ForegroundColor Yellow
            Write-Host "    2) 或者用【管理员身份】重新打开 PowerShell 再跑" -ForegroundColor Yellow
            Write-Host "    3) 确认目录不是只读: attrib `"$LibDir`"" -ForegroundColor Yellow
        } elseif ($hasDbError) {
            Write-Host "  诊断: sqlite 打不开库数据库 —— 典型的中文/非 ASCII 路径问题。" -ForegroundColor Yellow
            Write-Host "  对策: 把 -LibDir 换成纯 ASCII 且不含任何中文的路径。" -ForegroundColor Yellow
        } elseif ($hasLicErr) {
            Write-Host "  诊断: 疑似 ModelSim 许可问题。" -ForegroundColor Yellow
            Write-Host "  对策: 检查环境变量 LM_LICENSE_FILE / MGLS_LICENSE_FILE 指向的许可文件是否存在。" -ForegroundColor Yellow
        } else {
            Write-Host "  诊断: 原因不明确，请看上面 vlib 的原始输出。" -ForegroundColor Yellow
        }
        $probeFail = $true
    }
} finally {
    Pop-Location
    Remove-Item -Recurse -Force $probePath -ErrorAction SilentlyContinue
}

if ($probeFail) { exit 1 }

if ($Check) {
    Say-Step "诊断完成（-Check 模式，只建了一个临时库并已删除）"
    Write-Host "上面没有 FAIL，就可以去掉 -Check 正式执行。"
    Write-Host "注意：正式执行还会真正编译 709KB 的 prim_sim.v，" -ForegroundColor Gray
    Write-Host "      那一步是否成功只有跑起来才知道（见文档 §8 已知风险）。" -ForegroundColor Gray
    exit 0
}

#-----------------------------------------------------------------------------
# 3) 工程目录与 modelsim.ini
#-----------------------------------------------------------------------------
Say-Step "3) 检查工程目录"

if (-not $ProjectDir) { $ProjectDir = Find-ProjectDir -StartDir $PSScriptRoot }
if (-not (Test-Path $ProjectDir)) { Say-Err "工程目录不存在: $ProjectDir"; exit 1 }
Say-OK "工程目录: $ProjectDir"

$projAscii = Test-AsciiPath $ProjectDir
if ($projAscii) {
    Say-OK "工程目录纯 ASCII : $ProjectDir"
} else {
    Say-Warn "工程目录含非 ASCII 字符： $ProjectDir"
    Write-Host "       源码放这里没问题，但 ModelSim 的 work 库不能建在这里。" -ForegroundColor Yellow
    $altWork = Join-Path $LibDir 'work'
    Write-Host "       -> compile_all.do 会把 work 库建到: $altWork" -ForegroundColor Yellow
    $script:AltWorkDir = $altWork
}

#-----------------------------------------------------------------------------
# 4) 编译 Gowin 原语库
#-----------------------------------------------------------------------------
Say-Step "4) 编译 Gowin 原语库"

#-----------------------------------------------------------------------------
# 库名
#   配置里给了 GowinSimLib，且这次选中的 LibDir 正好是它的父目录
#     -> 用它的最后一段当库名（本工程 = gw5a，和 compile_all.do 里的 -L 一致）
#   否则退回历史默认 prim_gw5a
#
#   ⚠️ 这个判断不能省：名字不一致会让 .do 指向一个"新库"，
#      而现有那份库还在原地，两边分家且不报错。
#-----------------------------------------------------------------------------
$LibName = "prim_$Family"
if ($script:cfgGowinSimLib) {
    $cfgParent = [System.IO.Path]::GetFullPath((Split-Path $script:cfgGowinSimLib -Parent))
    $picked    = [System.IO.Path]::GetFullPath($LibDir)
    if ($picked -eq $cfgParent) {
        $LibName = Split-Path $script:cfgGowinSimLib -Leaf
        Say-OK "库名取自 toolchain.local.ps1: $LibName"
    } else {
        Say-Warn "本次库目录($picked)与配置里的 GowinSimLib 父目录($cfgParent)不一致"
        Say-Warn "   -> 仍用默认库名 $LibName；若这不是你想要的，请检查 -LibDir / 配置文件"
    }
}
$LibPath = Join-Path $LibDir $LibName

$env:MODELSIM = Join-Path $ModelsimHome 'modelsim.ini'
Say-OK "MODELSIM = $env:MODELSIM"

Push-Location $LibDir
try {
    # 已有旧库先删掉，避免残留导致"改了文件却没生效"
    if (Test-Path $LibPath) {
        Write-Host "  清理旧库 $LibPath"
        Remove-Item -Recurse -Force $LibPath -ErrorAction SilentlyContinue
    }

    Write-Host "  vlib $LibName"
    & $vlib $LibName 2>&1 | ForEach-Object { "    $_" }
    # 判据同 2.5 节：vlib 成功会写出 _info（_lib.qdb 要等 vlog 之后才有）
    if (-not (Test-Path (Join-Path $LibPath '_info'))) {
        Say-Err "vlib 失败，库没建出来（连 _info 都没有）"
        exit 1
    }

    Write-Host "  vlog -work $LibName <prim_sim.v>"
    $out = & $vlog -work $LibName $PrimFile 2>&1
    $out | Select-Object -Last 15 | ForEach-Object { "    $_" }

    if ($LASTEXITCODE -ne 0) {
        Say-Err "vlog 编译原语库失败（exit=$LASTEXITCODE）"
        Write-Host "常见原因：库路径不是 ASCII，或 ModelSim 许可失效。" -ForegroundColor Yellow
        exit 1
    }
    Say-OK "原语库编译完成: $LibPath"
} finally {
    Pop-Location
}

#-----------------------------------------------------------------------------
# 5) 在工程 modelsim.ini 里建立映射
#-----------------------------------------------------------------------------
Say-Step "5) 建立库映射（写进工程的 modelsim.ini，不动安装目录）"

$projIni = Join-Path $ProjectDir 'modelsim.ini'
$installIni = Join-Path $ModelsimHome 'modelsim.ini'

if (-not (Test-Path $projIni)) {
    Copy-Item $installIni $projIni -Force
    # 复制过来的 ini 里可能有只读属性
    Set-ItemProperty -Path $projIni -Name IsReadOnly -Value $false -ErrorAction SilentlyContinue
    Say-OK "已从安装目录复制一份 modelsim.ini 到工程"
}

# 把自己加到 [Library] 段
#   路径必须写绝对路径：modelsim.ini 的相对路径是相对"当前工作目录"解析的，
#   而用户可能从任意目录启动 vsim，相对路径会随机失效。
#   统一转正斜杠，Windows 下 ModelSim 同样接受，且不会被 Tcl 转义吃掉。
$iniText = Get-Content $projIni -Raw
$libAbs  = [System.IO.Path]::GetFullPath($LibPath).Replace('\', '/')
$mapLine = "$LibName = $libAbs"
if ($iniText -match "(?m)^\s*$([regex]::Escape($LibName))\s*=") {
    $iniText = [regex]::Replace($iniText, "(?m)^\s*$([regex]::Escape($LibName))\s*=.*$", $mapLine)
    Say-OK "更新已有映射: $mapLine"
} else {
    # 找到 [Library] 段插入
    if ($iniText -match '(?m)^\[Library\]\s*$') {
        $iniText = [regex]::Replace($iniText, '(?m)^(\[Library\]\s*)$', "`$1`r`n$mapLine")
        Say-OK "新增映射: $mapLine"
    } else {
        $iniText += "`r`n[Library]`r`n$mapLine`r`n"
        Say-Warn "[Library] 段不存在，已追加"
    }
}
Set-Content -Path $projIni -Value $iniText -NoNewline
Say-OK "modelsim.ini 已更新: $projIni"

#-----------------------------------------------------------------------------
# 6) 生成编译与仿真脚本
#-----------------------------------------------------------------------------
Say-Step "6) 生成 compile_all.do / run_sim.do"

#---- work 库位置：优先级 = 非 ASCII 工程的被迫选择 > 本机配置 > 工程内默认 ----
#   ⚠️ 不能无条件用 $ProjectDir\work：那样每次跑本脚本都会把 .do 里的
#      WORK_DIR 改到 sim\work，和 toolchain.local.ps1 / 文档里的 .simlib\work
#      变成两个地方 —— 库一分家，"换机器就找不到"这类问题就来了。
$workDir = if ($script:AltWorkDir) { $script:AltWorkDir }
           elseif ($script:cfgWorkDir) { $script:cfgWorkDir }
           else { Join-Path $ProjectDir 'work' }
Write-Host "  work 库: $workDir" -ForegroundColor DarkGray

# ---- .do 的生成逻辑只有一份，放在 emit_dofiles.ps1 ----
# 这样改了 RTL 文件列表后，可以只重跑 emit_dofiles.ps1，不用重新编译 Gowin 库。
$emitScript = Join-Path $PSScriptRoot 'emit_dofiles.ps1'
if (-not (Test-Path $emitScript)) {
    Say-Err "找不到 $emitScript（.do 生成器）"
    exit 1
}

& $emitScript -LibName $LibName -LibPath $LibPath -WorkDir $workDir `
              -OutDir $ProjectDir -TopName 'top'

if ($LASTEXITCODE -ne 0) {
    Say-Err "生成 .do 文件失败"
    exit 1
}
#-----------------------------------------------------------------------------
# 7) 收尾提示
#-----------------------------------------------------------------------------
Say-Step "完成"
Write-Host @"
接下来这样用（在 $ProjectDir 目录下）：

  # 1. 编译（含挂载 Gowin 原语库）
  vsim -c -do compile_all.do

  # 2. 跑回归
  vsim -c -do run_sim.do

  # 3. 看波形（GUI）
  vsim -do run_sim.do

如果 ModelSim 不在 PATH 里，用绝对路径，例如：
  & '$vsim' -c -do compile_all.do

Gowin 原语库位置 : $LibPath
work 库位置      : $workDir
工程 modelsim.ini: $projIni

注意：生成的 .do 里路径已自动转成正斜杠（Tcl 会把反斜杠当转义符）。
"@ -ForegroundColor Gray
