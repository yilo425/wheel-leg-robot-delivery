# build_leg_stl.ps1 —— 把 mech/leg_70mm.scad 渲染成 STL（二进制，干净几何）
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tools\build_leg_stl.ps1
#
# 前提：OpenSCAD 在 D:\dsh-ws\tools\openscad-2021.01\openscad.exe
#       （绿色版，免安装；换机器就改下面 $OSC 的路径）
#
# 改尺寸的流程：
#   1) 编辑 mech/leg_70mm.scad 顶部的参数
#   2) 跑这个脚本
#   3) mech/leg_70mm_clean.stl 更新（给 3D 打印）
#   4) 顺手跑 tools\gen_leg_model.py 更新预览图，方便核对尺寸

$ErrorActionPreference = 'Stop'
$OSC = 'D:\dsh-ws\tools\openscad-2021.01\openscad.exe'

$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$scad = Join-Path $root 'mech\leg_70mm.scad'
$stl  = Join-Path $root 'mech\leg_70mm_clean.stl'

if (-not (Test-Path $OSC)) { throw "找不到 OpenSCAD: $OSC" }
if (-not (Test-Path $scad)) { throw "找不到模型: $scad" }

# 无窗口渲染（offscreen 平台插件）
$env:QT_QPA_PLATFORM = 'offscreen'

Write-Host "渲染 $scad ..."
$sw = [Diagnostics.Stopwatch]::StartNew()
& $OSC --export-format binstl -o $stl $scad 2>&1 | Select-String 'Volumes|Facets|WARNING|ERROR' | ForEach-Object { "  " + $_.Line.Trim() }
$sw.Stop()

if (Test-Path $stl) {
    $f = Get-Item $stl
    Write-Host ("OK: {0}  {1:N0} KB  用时 {2:N1}s" -f $f.Name, ($f.Length/1KB), $sw.Elapsed.TotalSeconds)
} else {
    throw "渲染失败，没有生成 STL"
}
