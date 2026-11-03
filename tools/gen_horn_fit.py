#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_horn_fit.py —— 生成"六芒星试配板"（5 个凹槽 × 5 档过盈量）

为什么要试配板：
  打印件会收缩（PETG 0.2~0.4%），32mm 的凹槽就缩 0.06~0.13mm ✗
  而我们的过盈量只有 0.15mm —— 收缩量跟过盈量是同一量级，
  所以"算"不出来，必须**打一块试配板实测** ✓

用法：
  1) 打印 mech/horn_fit.stl（3mm 薄板，5 分钟）
  2) 把六芒星挨个往 1~5 号凹槽里压
  3) 报告："N 号最合适"（压得进去、有齿感、不掉）
  4) 我把 STAR_INTER 改对，重新出腿的 STL

编号规则：从左到右 1→5，凹槽下方的小凹点数量 = 编号
过盈量（负 = 凹槽比实物大）：
  1 号  +0.30  最紧（凹槽最小）
  2 号  +0.15  现在的设定
  3 号   0.00  等尺寸
  4 号  -0.15  大 0.15
  5 号  -0.30  最松（凹槽最大）
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MECH = os.path.join(ROOT, 'mech')
os.makedirs(MECH, exist_ok=True)

# ---- 和 leg_70mm.scad 完全一致的舵机盘参数 ----
STAR_RO, STAR_RI = 16.0, 8.0
ARM_W, ARM_W_TIP = 7.0, 5.0
HUB_R, ARM_R0 = 3.4, 4.8
STAR_DEP = 2.4          # 试配板上的槽深（比 2.1 深一点，方便手压）
SHINKS = [0.30, 0.15, 0.00, -0.15, -0.30]
PLATE_T = 3.0
SPACING = 36.0
PLATE_W = SPACING * 5 + 16       # 196
PLATE_H = 40.0

scad = f'''//=============================================================================
// horn_fit.scad —— 六芒星试配板（5 档过盈量），2026-10-23
//   从左到右 1..5，凹槽下方凹点数 = 编号
//   打出来把舵机盘逐个压进去，报"几号最合适"
//=============================================================================
STAR_RO = {STAR_RO};
STAR_RI = {STAR_RI};
ARM_W = {ARM_W};
ARM_W_TIP = {ARM_W_TIP};
HUB_R = {HUB_R};
ARM_R0 = {ARM_R0};
STAR_DEP = {STAR_DEP};
PLATE_T = {PLATE_T};
$fn = 64;

// 舵机盘外形（和腿零件里的完全一样）
module horn2d(ro, ri, w_base, w_tip, n, shrink) {{
    ro2 = ro - shrink;
    ri2 = ri - shrink;
    wb  = w_base - 2*shrink;
    wt  = w_tip  - 2*shrink;
    union() {{
        circle(r = HUB_R - shrink);
        for (i = [0 : n-1])
            rotate([0, 0, i * 360/n])
                hull() {{
                    translate([ARM_R0 - shrink, 0]) circle(d = wb);
                    translate([ro2 - wt/2, 0]) circle(d = wt);
                }}
    }}
}}

module pocket(shrink) {{
    // 从顶面往下挖 STAR_DEP
    translate([0, 0, PLATE_T - STAR_DEP])
        linear_extrude(height = STAR_DEP + 0.01)
            horn2d(STAR_RO, STAR_RI, ARM_W, ARM_W_TIP, 6, shrink);
}}

module dimples(cnt) {{
    // 编号用凹点：cnt 个小圆坑
    for (i = [0 : cnt-1])
        translate([(i - (cnt-1)/2) * 4.2, -(STAR_RO + 7), PLATE_T - 1.2])
            cylinder(d = 2.2, h = 2.0);
}}

module plate() {{
    difference() {{
        // 底板
        translate([-{PLATE_W/2}, -{PLATE_H/2}, 0])
            cube([{PLATE_W}, {PLATE_H}, PLATE_T]);
        // 5 个凹槽 + 编号凹点
'''

for i, sh in enumerate(SHINKS):
    x = (i - 2) * SPACING
    scad += f'''        translate([{x}, 0, 0]) pocket({sh});
        translate([{x}, 0, 0]) dimples({i+1});
'''

scad += '''    }
}
plate();
'''

p = os.path.join(MECH, 'horn_fit.scad')
with open(p, 'w', encoding='utf-8', newline='\n') as f:
    f.write(scad)
print('已生成: mech/horn_fit.scad')

# ---- 调用 OpenSCAD 出 STL ----
OSC = r'D:\dsh-ws\tools\openscad-2021.01\openscad.exe'
stl = os.path.join(MECH, 'horn_fit.stl')
if os.path.exists(OSC):
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    r = subprocess.run([OSC, '--export-format', 'binstl', '-o', stl, p],
                       capture_output=True, text=True, timeout=600)
    for ln in (r.stdout + r.stderr).splitlines():
        if any(k in ln for k in ('ERROR', 'WARNING', 'Volumes', 'Facets')):
            print('  ' + ln.strip())
    if os.path.exists(stl):
        print('已生成: mech/horn_fit.stl  (%.0f KB)' % (os.path.getsize(stl)/1024))
    else:
        print('!! OpenSCAD 没出 STL')
else:
    print('!! 找不到 OpenSCAD:', OSC)

print()
print('=== 试配板对照表 ===')
for i, sh in enumerate(SHINKS):
    d = '紧' if sh > 0 else ('松' if sh < 0 else '等尺寸')
    print('  %d 号：过盈 %+.2f mm（%s）  凹槽外形尺寸 = 实物 %+.2f mm'
          % (i+1, sh, d, -sh))
print('  报告格式："3 号最合适" 或 "2 号压不进，3 号刚好" 都可以')
