#=============================================================================
# toolchain.local.ps1 —— 本机专属路径配置（A 的机器 / 本工作区）
#-----------------------------------------------------------------------------
# 本文件**已被 .gitignore 排除，不入库** —— 队友各自的配置互不影响。
# 模板见同目录 toolchain.local.ps1.template。
#
# 为什么必须存在这个文件（不写会怎样）：
#   所有 toolchain 脚本的优先级是  显式传参 > 本文件 > 脚本内置默认值，
#   而**内置默认值不是本机的**：
#       WorkDir     默认 'D:\gowin_lib\work'   ->  work 库会跑到仓库外面去
#       GowinSimLib 默认 'D:\modeltech64_2020.4\gw5a'
#   也就是说：不带参数直接跑 emit_dofiles.ps1，生成的 .do 会把工作库
#   指到 D:\gowin_lib\work，而不是本工程一直用的 .simlib\work ——
#   编译能过但库在两处，换机器/换目录就找不到，且**不报错**。
#   本文件就是把这件事钉死，让"不带参数直接跑"也是对的。
#
# ⚠️ 关于 Gowin 版本（2026-10-15 修）：
#   GowinSimLib 里的库是由**哪个 Gowin 版本的 prim_sim.v** 编译出来的，
#   必须和用来综合的版本一致，否则仿真模型和综合器不是一个版本。
#   本机综合用的是 **V1.9.12**，所以 gw5a 库也是 V1.9.12 编译的。
#
#   怎么一眼看出库是哪个版本建的：
#       Gowin V1.9.12 把 `DQS` 从 prim_sim.v 里移走了（挪进 DQS_*.vp），
#       V1.9.11 的 prim_sim.v 里**有** DQS。所以：
#           vdir -lib D:/modeltech64_2020.4/gw5a | findstr "MODULE DQS"
#           有输出 -> V1.9.11 建的（过期）
#           没输出 -> V1.9.12 建的（正确；模块总数 132）
#   旧库已备份在 D:\modeltech64_2020.4\gw5a_v1911_backup_20261015（可回退）。
#
#   回归里有一条 tb_gowin_prim 专门盯这条链（例化真实原语），
#   库缺失/版本不对/路径不对都会当场红 —— 不用靠人记得检查。
#=============================================================================

$LocalPaths = @{

    #-------------------------------------------------------------------------
    # ModelSim 安装根目录（里面有 win64\ 子目录）
    #   本机装了两份（C:\modeltech64_2020.4 和 D:\modeltech64_2020.4），
    #   工程统一用 **D 盘那份**（LICENSE.TXT、gw5a 库都在它下面）。
    #       验证: Test-Path 'D:\modeltech64_2020.4\win64\vsim.exe'
    #-------------------------------------------------------------------------
    ModelsimHome = 'D:\modeltech64_2020.4'

    #-------------------------------------------------------------------------
    # ModelSim 的 modelsim.ini（安装目录里那份，**不要**拷进 sim\）
    #-------------------------------------------------------------------------
    ModelsimIni = 'D:\modeltech64_2020.4\modelsim.ini'

    #-------------------------------------------------------------------------
    # Gowin 器件原语仿真库（已编译好；由 setup_gowin_modelsim.ps1 生成）
    #   库名 = 路径最后一段 = `gw5a`，compile_all.do 里就是 -L gw5a。
    #       验证: Test-Path 'D:\modeltech64_2020.4\gw5a\_info'
    #-------------------------------------------------------------------------
    GowinSimLib = 'D:\modeltech64_2020.4\gw5a'

    #-------------------------------------------------------------------------
    # ★ work 库放哪（这里填的是**库目录本身**，不是父目录）
    #   必须纯 ASCII：ModelSim 2020.4 在含中文的路径下建不了库
    #   （本机用户目录是 C:\Users\忆洛\...，必挂）。
    #   放在仓库内 .simlib\ 下 —— 迁移仓库时跟着走，不会再和源码分家。
    #-------------------------------------------------------------------------
    WorkDir = 'D:\dsh-ws\robot_project\.simlib\work'

    #-------------------------------------------------------------------------
    # Gowin IDE 安装根目录（综合用；查 simlib / 手册时也用它）
    #   注意是**两层同名目录**：...\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64
    #   本机还留着旧的 V1.9.11.03_Education（器件表不全，不能用来综合 GW5AT-138）
    #-------------------------------------------------------------------------
    GowinHome = 'D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64'
}


#=============================================================================
# 加载时自检：哪条路径不对立刻说清楚
#   配环境时最有用的一段 —— 否则错的是"路径"，报的却是别的错。
#=============================================================================
Write-Host '--- toolchain.local.ps1 路径检查 ---' -ForegroundColor Cyan
$checks = @(
    @{ Name = 'ModelSim vsim.exe'; Path = (Join-Path $LocalPaths.ModelsimHome 'win64\vsim.exe') },
    @{ Name = 'modelsim.ini';      Path = $LocalPaths.ModelsimIni },
    @{ Name = 'Gowin 仿真库';      Path = (Join-Path $LocalPaths.GowinSimLib '_info') },
    @{ Name = 'work 库目录';       Path = (Split-Path $LocalPaths.WorkDir -Parent) },
    @{ Name = 'Gowin IDE';         Path = (Join-Path $LocalPaths.GowinHome 'IDE\simlib\gw5a\prim_sim.v') }
)
$allOk = $true
foreach ($c in $checks) {
    if (Test-Path $c.Path) {
        Write-Host "  [OK]   $($c.Name): $($c.Path)" -ForegroundColor Green
    } else {
        Write-Host "  [缺失] $($c.Name): $($c.Path)" -ForegroundColor Red
        $allOk = $false
    }
}
if ($LocalPaths.WorkDir -match '[^\x00-\x7F]') {
    Write-Host "  [错误] WorkDir 含非 ASCII 字符，ModelSim 会建库失败: $($LocalPaths.WorkDir)" -ForegroundColor Red
    $allOk = $false
}
if ($allOk) { Write-Host '  全部通过' -ForegroundColor Green }
else        { Write-Host '  请修正上面标红项后再跑脚本' -ForegroundColor Yellow }
Write-Host ''
