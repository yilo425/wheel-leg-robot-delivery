#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_leg_model.py —— 生成腿零件（舵机驱动轮腿）的参数化模型 + 预览图

v2（2026-10-22）：按讨论加了三样东西
  ① 根部局部加厚到 11mm（否则"沉孔 4mm + 六芒星凹槽 2.6mm"中间只剩 1.4mm 会裂）
  ② 顶面 φ8 × 4mm 沉孔（把 φ8 垫片 + M3 螺丝头藏进去）
  ③ 挡件平台（R=25 腿根处，上表面在【工作角 45°】时水平）

⚠️ 必须实测的 5 个数（模型里是占位默认值）：
  1) STAR_RO / STAR_RI  舵机盘外接/内接圆半径
  2) STAR_DEP           舵机盘厚度 + 0.1
  3) MOT_BOLT_SQ        MG310 4 个 M3 的方阵边长（对角孔距 ÷ 1.414）
  4) MOT_BOSS_D         MG310 凸台直径
"""
import os
import math

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MECH = os.path.join(ROOT, 'mech')
os.makedirs(MECH, exist_ok=True)

# ============================================================ 参数
L          = 70.0    # 舵机轴中心 -> 电机轴中心
TH         = 6.0     # 腿厚（一般段）—— 打印件实心强度远超预期，减 2mm
ROOT_TH    = 9.0     # ★ 根部局部厚度（沉孔 4 + 凹槽 2.1 + 壁 2.9）
ROOT_R     = 19.5    # 根部圆盘半径（六芒星 R10 + 壁 3.5）
ROOT_SEG   = 14.0    # 加厚段延伸到 x=20，之后过渡到 8mm

# 六芒星凹槽（★ 必须量）
STAR_PTS   = 6
STAR_RO    = 16.0
STAR_RI    = 8.0
STAR_DEP   = 2.1
STAR_INTER = -0.15     # 负数 = 凹槽比实物【大】0.15mm（宁松勿紧，松了滴 502）
ARM_W      = 7.0      # 舵机盘臂【根部】宽（实测 7）
ARM_W_TIP  = 5.0      # 舵机盘臂【端部】宽（实测 5，锥形渐变）
HUB_R      = 3.4      # 中心小盘半径（只填中心，谷底靠臂侧自然形成尖 V）
ARM_R0     = 4.8      # 臂根部所在半径（d7 圆的圆心）

CENTER_D   = 3.6     # 中心 M3 过孔（放 0.1 余量）
CSNK_D     = 8.5     # ★ 沉孔直径（φ8 垫片 + 0.5 余量，打印会缩）
CSNK_DEP   = 4.0     # ★ 沉孔深度

# 电机安装（★ 必须量）
MOT_BOLT_SQ = 12.0
MOT_BOLT_D  = 3.4     # 电机 M3 孔（放 0.2，打印会缩）
CBORE_DEP   = 2.0     # 沉槽深度（实测头厚仅 1mm，2mm 够；余量留给垫片）
CBORE_D     = 5.8     # 沉槽直径 = 实测头径 φ5 + 0.8 余量（M3x12 304 盘头）
MOT_BOSS_D  = 9.2     # 电机凸台让位（9.3 -> 9.2，余量过大）

# 挡件平台
WORK_ANG   = 45.0    # 工作角（腿相对水平向下）
PAD_R      = 25.0    # 平台中心沿腿的半径（腿根，按用户画的）
PAD_L      = 14.0    # 接触面长度（削平面，太大会削太深）
PAD_H      = 5.0     # 凸起高度
PAD_SURF_Y = 12.3    # 平台处腿半高（x=25 附近）
PAD_FLARE  = 8.0
PAD_EN     = 0        # 1=要凸台平台  0=不要（腿边缘直接压橡胶挡件）    # 底部外扩（做成整块凸台，不是薄片）

CTRL = [(0.0, ROOT_R), (18.0, 15.0), (42.0, 13.0), (60.0, 13.0), (72.0, 13.0)]

# ---------------- 扎带孔（走电机线用，队友要求）----------------
#   2 个 φ2.2 通孔，沿 Z 打穿；扎带穿过去把电机线压在腿面上 ✓
#   位置在腿中段（远离根部受力区和尖端电机座）
TIE_D    = 2.2
TIE_X1   = 30.0
TIE_X2   = 44.0

# ============================================================ 1) 写 OpenSCAD
scad = f'''//=============================================================================
// leg_70mm.scad —— 舵机驱动轮腿（参数化 v2，2026-10-22）
//
// 用法：
//   1) 装 OpenSCAD（免费 20MB）：https://openscad.org/downloads.html
//   2) 改最上面【必改：实测参数】
//   3) 按 F6 → File → Export → Export as STL
//
// 打印：PETG / 填充 60~80% / 4 圈壁 / **平躺**（8mm 厚朝 Z）
//=============================================================================

//---------------- ★ 必改：实测参数 ----------------
STAR_RO     = {STAR_RO};     // 量：舵机盘尖到圆心
STAR_RI     = {STAR_RI};     // 量：舵机盘凹处到圆心
STAR_DEP    = {STAR_DEP};    // 量：舵机盘厚度 + 0.1
MOT_BOLT_SQ = {MOT_BOLT_SQ}; // 量：MG310 对角孔距 / 1.414
MOT_BOSS_D  = {MOT_BOSS_D};  // 量：MG310 凸台直径

//---------------- 结构参数 ----------------
L          = {L};
TH         = {TH};
ROOT_TH    = {ROOT_TH};
ROOT_R     = {ROOT_R};
ROOT_SEG   = {ROOT_SEG};
STAR_INTER = {STAR_INTER};
ARM_W      = {ARM_W};
ARM_W_TIP  = {ARM_W_TIP};
HUB_R      = {HUB_R};
ARM_R0     = {ARM_R0};
CENTER_D   = {CENTER_D};
CSNK_D     = {CSNK_D};
CSNK_DEP   = {CSNK_DEP};
MOT_BOLT_D = {MOT_BOLT_D};
TIE_D      = {TIE_D};
TIE_X1     = {TIE_X1};
TIE_X2     = {TIE_X2};
CBORE_DEP  = {CBORE_DEP};
CBORE_D    = {CBORE_D};

WORK_ANG   = {WORK_ANG};
PAD_R      = {PAD_R};
PAD_L      = {PAD_L};
PAD_H      = {PAD_H};
PAD_FLARE  = {PAD_FLARE};
PAD_EN     = {PAD_EN};
PAD_SURF_Y = {PAD_SURF_Y};

$fn = 64;

//---------------- 舵机盘（六芒星）凹槽 ----------------
//   按实物重建：**6 根圆头粗臂 + 中心盘**，不是细尖星。
//     臂宽 ARM_W，端头半圆，谷底半径 = ri，臂尖 = ro
//   过盈通过 shrink 实现（臂宽/半径各缩小一点）
module horn2d(ro, ri, w_base, w_tip, n, shrink) {{
    ro2 = ro - shrink;
    ri2 = ri - shrink;
    wb  = w_base - 2*shrink;
    wt  = w_tip  - 2*shrink;
    union() {{
        circle(r = HUB_R - shrink);   // 只填中心，让 6 个谷底成为【尖 V】
        for (i = [0 : n-1])
            rotate([0, 0, i * 360/n])
                hull() {{
                    translate([ARM_R0 - shrink, 0]) circle(d = wb);
                    translate([ro2 - wt/2, 0]) circle(d = wt);
                }}
    }}
}}

//---------------- 腿的外形 ----------------
module profile_2d() {{
    hull() {{
        circle(r = {CTRL[0][1]});
        translate([{CTRL[1][0]}, 0]) circle(r = {CTRL[1][1]});
        translate([{CTRL[2][0]}, 0]) circle(r = {CTRL[2][1]});
        translate([{CTRL[3][0]}, 0]) circle(r = {CTRL[3][1]});
        translate([{CTRL[4][0]}, 0]) circle(r = {CTRL[4][1]});
    }}
}}

//---------------- ★ 根部加厚（8 -> 11mm）----------------
module root_boss() {{
    hull() {{
        translate([0, 0, TH]) linear_extrude(height = ROOT_TH - TH)
            circle(r = ROOT_R);
        translate([ROOT_SEG, 0, TH]) linear_extrude(height = ROOT_TH - TH)
            circle(r = 12.0);
    }}
}}

//---------------- ★ 挡件平台 ----------------
//   局部坐标里绕 Z 转 WORK_ANG：它的 +Y 面在腿转到 -WORK_ANG（工作位）时
//   正好变成"水平朝上"，挡件才能水平压住它。
//   方案 A（保留凸台，端头做钝）：为什么不能"削平"见 doc 说明 ——
//   腿的上边缘几乎平行于腿轴，而接触面要求与腿轴成 45°，
//   两者夹角 45° => 任何"削平"都会切掉一大块楔形，反而更难看/更弱。
//   所以改成"堆一个小凸台 + 外侧倒圆"，接触面仍是 45°。
module stop_pad() {{
    translate([PAD_R, PAD_SURF_Y - 3, 0])
        rotate([0, 0, WORK_ANG])
            linear_extrude(height = TH)
                // 梯形（底部外扩）+ 顶端倒角，避免刀刃状尖角
                polygon([[ -PAD_L/2 - PAD_FLARE, 0 ],
                         [  PAD_L/2 + PAD_FLARE, 0 ],
                         [  PAD_L/2 - 2,         PAD_H + 3 ],
                         [ -PAD_L/2 + 2,         PAD_H + 3 ]]);
}}

//---------------- 主体 ----------------
difference() {{
    union() {{
        linear_extrude(height = TH) profile_2d();   // 主承力体
        root_boss();                                // 根部加厚
        if (PAD_EN != 0) stop_pad();                                 // 挡件平台
    }}

    // 六芒星凹槽（从底面 z=0 往上挖）
    translate([0, 0, -1])
        linear_extrude(height = STAR_DEP + 1)
            horn2d(STAR_RO, STAR_RI, ARM_W, ARM_W_TIP, {STAR_PTS}, STAR_INTER/2);

    // 中心 M3 通孔
    translate([0, 0, -1])
        cylinder(d = CENTER_D, h = ROOT_TH + 2);

    // ★ 顶面沉孔（φ8 垫片 + 螺丝头）
    translate([0, 0, ROOT_TH - CSNK_DEP])
        cylinder(d = CSNK_D, h = CSNK_DEP + 1);

    // 电机凸台让位孔
    translate([L, 0, -1])
        cylinder(d = MOT_BOSS_D, h = TH + 2);

    // 4 个电机安装孔
    for (sx = [-1, 1], sy = [-1, 1])
        translate([L + sx * MOT_BOLT_SQ/2, sy * MOT_BOLT_SQ/2, -1])
            cylinder(d = MOT_BOLT_D, h = TH + 2);

    // ★ 扎带孔 ×2（走电机线）
    for (tx = [TIE_X1, TIE_X2])
        translate([tx, 0, -1])
            cylinder(d = TIE_D, h = TH + 2);

    // 5) ★ 电机螺丝沉槽：顶面往下 CBORE_DEP，直径 = 孔径 + CBORE_D
    //    让原装螺丝的头藏进腿里（照片里那种小盘头螺丝）
    for (sx = [-1, 1], sy = [-1, 1])
        translate([L + sx * MOT_BOLT_SQ/2, sy * MOT_BOLT_SQ/2, TH - CBORE_DEP])
            cylinder(d = CBORE_D, h = CBORE_DEP + 1);
}}
'''

with open(os.path.join(MECH, 'leg_70mm.scad'), 'w', encoding='utf-8', newline='\n') as f:
    f.write(scad)
print('已生成: mech/leg_70mm.scad')

# ============================================================ 2) 体积/重量
def half_h(x):
    for i in range(len(CTRL) - 1):
        x0, r0 = CTRL[i]; x1, r1 = CTRL[i + 1]
        if x <= x1 or i == len(CTRL) - 2:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            t = max(0.0, min(1.0, t))
            return r0 + (r1 - r0) * t
    return CTRL[-1][1]

N = 2000
xa, xb = -ROOT_R, CTRL[-1][0] + CTRL[-1][1]
dx = (xb - xa) / N
area = 0.0
for i in range(N):
    x = xa + (i + 0.5) * dx
    h = math.sqrt(max(0.0, ROOT_R**2 - x*x)) if x < 0 else half_h(x)
    area += 2 * h * dx

star_area = 0.0
for i in range(2 * STAR_PTS):
    a_i, a_j = i * 180 / STAR_PTS, ((i + 1) % (2 * STAR_PTS)) * 180 / STAR_PTS
    r_i = (STAR_RO if i % 2 == 0 else STAR_RI) - STAR_INTER / 2
    r_j = (STAR_RO if (i + 1) % 2 == 0 else STAR_RI) - STAR_INTER / 2
    star_area += 0.5 * r_i * r_j * math.sin(math.radians(a_j - a_i))

v = (area * TH
     + math.pi * ROOT_R**2 * (ROOT_TH - TH) * 0.75          # 根部加厚（近似）
     + (PAD_L * PAD_H + 0.5 * 6 * PAD_H) * TH               # 平台（梯形近似）
     - abs(star_area) * STAR_DEP                            # 六芒星凹槽
     - math.pi * (CENTER_D/2)**2 * ROOT_TH                  # 中心孔
     - math.pi * (CSNK_D/2)**2 * CSNK_DEP                   # 沉孔
     - math.pi * (MOT_BOSS_D/2)**2 * TH                     # 电机凸台孔
     - 4 * math.pi * (MOT_BOLT_D/2)**2 * TH)                # 4 个电机孔
mass = v * 1.27e-3
print('体积 %.0f mm3  单条 %.1f g  4 条 %.1f g（60%%填充约 %.1f g）'
      % (v, mass, mass*4, mass*4*0.75))

# ============================================================ 3) 预览图
from PIL import Image, ImageDraw, ImageFont

SC, PADPX = 7.0, 90
W = int((xb - xa) * SC + PADPX * 2)
H = int(70 * SC + PADPX * 2)
img = Image.new('RGB', (W, H), 'white')
dr = ImageDraw.Draw(img)

def X(x): return int((x - xa) * SC + PADPX)
def Y(y): return int(H/2 - y * SC)

try:
    font = ImageFont.truetype("consola.ttf", 15)
    font_b = ImageFont.truetype("consola.ttf", 19)
except Exception:
    font = font_b = ImageFont.load_default()

pts_top, pts_bot = [], []
for i in range(301):
    x = xa + (xb - xa) * i / 300
    h = math.sqrt(max(0.0, ROOT_R**2 - x*x)) if x < 0 else half_h(x)
    pts_top.append((X(x), Y(h))); pts_bot.append((X(x), Y(-h)))
poly = pts_top + pts_bot[::-1]
dr.polygon(poly, fill=(228, 235, 245), outline=(40, 70, 120))
dr.line(poly + [poly[0]], fill=(40, 70, 120), width=3)

# 根部加厚区提示
dr.line([X(-ROOT_R), Y(12.6), X(ROOT_SEG + 12), Y(12.6)], fill=(90, 150, 90), width=2)
dr.line([X(-ROOT_R), Y(-12.6), X(ROOT_SEG + 12), Y(-12.6)], fill=(90, 150, 90), width=2)
dr.text((X(-ROOT_R) + 4, Y(12.6) - 22), "root zone %.0fmm thick" % ROOT_TH,
        fill=(60, 120, 60), font=font)

# 六芒星凹槽
star = []
for i in range(2 * STAR_PTS):
    a = i * 180 / STAR_PTS
    r = (STAR_RO if i % 2 == 0 else STAR_RI) - STAR_INTER / 2
    star.append((X(r*math.cos(math.radians(a))), Y(r*math.sin(math.radians(a)))))
dr.polygon(star, fill=(255, 205, 120), outline=(180, 110, 0))
dr.line(star + [star[0]], fill=(180, 110, 0), width=2)

# 沉孔
r = CSNK_D / 2
dr.ellipse([X(-r), Y(r), X(r), Y(-r)], fill=(208, 222, 242), outline=(160, 40, 40), width=3)
dr.text((X(r) + 5, Y(r) - 18), "csnk phi%.0f x %.1f" % (CSNK_D, CSNK_DEP),
        fill=(160, 40, 40), font=font)

# 中心孔
r = CENTER_D / 2
dr.ellipse([X(-r), Y(r), X(r), Y(-r)], fill='white', outline=(180, 0, 0), width=2)

# 挡件平台（局部坐标里转了 45° -> 俯视是斜矩形）
a = math.radians(WORK_ANG)
def pad_pt(u, vv):
    px = PAD_R + u * math.cos(a) - vv * math.sin(a)
    py = (PAD_SURF_Y - 3) + u * math.sin(a) + vv * math.cos(a)
    return (X(px), Y(py))
quad = [pad_pt(-PAD_L/2, 0), pad_pt(PAD_L/2, 0),
        pad_pt(PAD_L/2, PAD_H + 2), pad_pt(-PAD_L/2, PAD_H + 2)]
dr.polygon(quad, fill=(255, 228, 175), outline=(200, 120, 0), width=3)
dr.text((quad[1][0] + 5, quad[1][1] - 20),
        "stop pad (face horizontal @ %.0f deg)" % WORK_ANG,
        fill=(200, 120, 0), font=font)

# 电机孔
r = MOT_BOSS_D / 2
dr.ellipse([X(L - r), Y(r), X(L + r), Y(-r)], fill='white', outline=(0, 130, 0), width=2)
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = L + sx * MOT_BOLT_SQ/2, sy * MOT_BOLT_SQ/2
        rr = MOT_BOLT_D / 2
        dr.ellipse([X(cx - rr), Y(cy + rr), X(cx + rr), Y(cy - rr)],
                   fill='white', outline=(0, 130, 0), width=2)

# 中心线 + 腿长
dr.line([X(xa - 3), Y(0), X(xb + 3), Y(0)], fill=(150, 150, 150))
yy = Y(-28)
dr.line([X(0), yy, X(L), yy], fill=(200, 0, 0), width=2)
dr.line([X(0), yy-6, X(0), yy+6], fill=(200, 0, 0), width=2)
dr.line([X(L), yy-6, X(L), yy+6], fill=(200, 0, 0), width=2)
dr.text((int((X(0)+X(L))/2) - 56, yy - 24), "L = %.0f mm" % L, fill=(200, 0, 0), font=font_b)

dr.text((PADPX, 12), "LEG 70mm v2  -  top view (X-Y)", fill=(0, 0, 0), font=font_b)
dr.text((PADPX, 36), "blue=leg  orange=star pocket  red=countersink  pale=stop pad",
        fill=(90, 90, 90), font=font)

# 参数表
tx, ty = W - 360, H - 240
dr.rectangle([tx, ty, W - 10, H - 10], outline=(150, 150, 150))
rows = [("L servo->motor", "%.0f mm" % L),
        ("thickness gen/root", "%.0f / %.0f mm" % (TH, ROOT_TH)),
        ("root dia", "%.0f mm" % (ROOT_R*2)),
        ("star pocket depth", "%.1f mm" % STAR_DEP),
        ("countersink", "phi%.0f x %.1f" % (CSNK_D, CSNK_DEP)),
        ("wall under csnk", "%.1f mm" % (ROOT_TH - CSNK_DEP - STAR_DEP)),
        ("stop pad", "R%.0f  %.0fx%.0f" % (PAD_R, PAD_L, PAD_H)),
        ("motor bolts", "%.1f mm sq" % MOT_BOLT_SQ),
        ("volume", "%.0f mm3" % v),
        ("mass solid PETG", "%.1f g" % mass)]
dr.text((tx + 8, ty + 4), "KEY PARAMS", fill=(0, 0, 0), font=font_b)
yy2 = ty + 30
for k, val in rows:
    dr.text((tx + 8, yy2), k, fill=(50, 50, 50), font=font)
    dr.text((tx + 215, yy2), val, fill=(180, 0, 0), font=font)
    yy2 += 19

img.save(os.path.join(MECH, 'leg_70mm_preview.png'))
print('已生成: mech/leg_70mm_preview.png (%dx%d)' % (W, H))
