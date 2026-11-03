#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_leg_assembly.py —— 画腿+舵机盘的装配示意图（3D 爆炸图 + 剖面图）

目的：回答"六芒星怎么卡进去、螺丝怎么从外面拧"
  A 图：等轴测爆炸图（4 层分开摆，箭头指装配方向）
  B 图：中心剖面图（看清螺丝的穿行路径：螺丝头 -> 垫片 -> 腿 -> 盘 -> 花键）
"""
import math
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'mech', 'leg_assembly.png')

# ---------------- 关键尺寸（和 leg_70mm.scad 一致）----------------
ROOT_R   = 13.5      # 根部半径
TH       = 8.0       # 腿厚
STAR_RO  = 10.0      # 六芒星外接圆
STAR_RI  = 6.5       # 六芒星内接圆
STAR_DEP = 2.6       # 凹槽深
HORN_T   = 2.5       # 舵机盘厚
CENTER_D = 3.5
WASHER_D = 8.0
WASHER_T = 1.5
SCREW_HEAD_D = 5.5
SCREW_HEAD_T = 2.5
SPLINE_R = 6.0       # 花键/舵机输出凸台半径
SERVO_W  = 20.0      # 舵机本体宽（画个方块示意）
SERVO_H  = 18.0

# ---------------- 画布 ----------------
W, H = 1500, 1000
img = Image.new('RGB', (W, H), (250, 250, 252))
dr = ImageDraw.Draw(img)

def load_font(size, bold=False):
    cands = [r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
             r"C:\Windows\Fonts\msyh.ttc",
             r"C:\Windows\Fonts\simhei.ttf",
             "consola.ttf"]
    for c in cands:
        try:
            return ImageFont.truetype(c, size)
        except Exception:
            continue
    return ImageFont.load_default()

F  = load_font(17)
FB = load_font(22, True)
FT = load_font(30, True)

# ---------------- 等轴测投影 ----------------
def iso(x, y, z, s, ox, oy):
    """x,y,z(mm) -> 屏幕像素；z 向上"""
    u = (x - y) * 0.866
    v = (x + y) * 0.5 - z
    return (ox + u * s, oy + v * s)

def leg_outline(n=60):
    """腿的 2D 轮廓（俯视），返回闭合点列"""
    ctrl = [(0.0, ROOT_R), (25.0, 9.0), (45.0, 8.0), (62.0, 10.5), (72.0, 10.5)]
    def hh(x):
        for i in range(len(ctrl) - 1):
            x0, r0 = ctrl[i]; x1, r1 = ctrl[i + 1]
            if x <= x1 or i == len(ctrl) - 2:
                t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
                t = max(0.0, min(1.0, t))
                return r0 + (r1 - r0) * t
        return ctrl[-1][1]
    top, bot = [], []
    xs = [-ROOT_R + (ctrl[-1][0] + ctrl[-1][1] + ROOT_R) * i / n for i in range(n + 1)]
    for x in xs:
        h = math.sqrt(max(0.0, ROOT_R**2 - x * x)) if x < 0 else hh(x)
        top.append((x, h)); bot.append((x, -h))
    return top + bot[::-1]

def star_pts(ro, ri, n=6, phase=0.0):
    pts = []
    for i in range(2 * n):
        a = math.radians(i * 180 / n + phase)
        r = ro if i % 2 == 0 else ri
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts

def draw_prism(pts2d, z0, z1, s, ox, oy,
               face=(190, 205, 225), top=(225, 235, 248), side=(150, 170, 200), outline=(40, 70, 120)):
    """把一个 2D 多边形沿 z 挤出成棱柱（画顶面+两个侧面，简易 3D）"""
    n = len(pts2d)
    # 侧面（按屏幕坐标排序，只画"朝我们"的那部分：取点列顺序的下半 + 上半）
    # 简化：画整个轮廓的两层 + 连接线
    low = [iso(x, y, z0, s, ox, oy) for (x, y) in pts2d]
    high = [iso(x, y, z1, s, ox, oy) for (x, y) in pts2d]
    # 侧面（轮廓外圈）
    for i in range(n):
        j = (i + 1) % n
        quad = [low[i], low[j], high[j], high[i]]
        dr.polygon(quad, fill=side, outline=None)
    # 顶面
    dr.polygon(high, fill=top, outline=outline)
    dr.line(high + [high[0]], fill=outline, width=2)
    # 底面轮廓（只画外缘提示）
    dr.line(low + [low[0]], fill=outline, width=1)

# ================================================================
# A 图：等轴测爆炸图（左）
# ================================================================
S_A = 2.85
OX_A, OY_A = 200, 610

dr.text((40, 24), "A. 装配爆炸图（从下往上装）", fill=(0, 0, 0), font=FT)

outline = leg_outline(72)
star6   = star_pts(STAR_RO, STAR_RI, 6, 0)
star6_in= star_pts(STAR_RO - 0.15, STAR_RI - 0.15, 6, 0)

# 层 z 位置（爆炸状态，层间留 14mm 间隙）
z_servo   = -SERVO_H
z_spline  = -4.0
z_horn    = 30.0                 # 爆炸：舵机盘抬起来
z_leg     = z_horn + HORN_T + 46.0
z_washer  = z_leg + TH + 26.0
z_head    = z_washer + WASHER_T

# ---- 舵机本体（盒子）
def draw_box(x0, x1, y0, y1, z0, z1, s, ox, oy, fill=(120, 125, 135), top=(150, 155, 165)):
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    draw_prism(pts, z0, z1, s, ox, oy, face=fill, top=top, side=(95, 100, 110))

draw_box(-SERVO_W/2, SERVO_W/2, -SERVO_H/2, SERVO_H/2,
         z_servo, z_spline, S_A, OX_A, OY_A)
# 花键
spl = [(SPLINE_R*math.cos(math.radians(a)), SPLINE_R*math.sin(math.radians(a)))
       for a in range(0, 360, 15)]
draw_prism(spl, z_spline, 0.0, S_A, OX_A, OY_A,
           face=(210, 210, 215), top=(235, 235, 240), side=(180, 180, 190))

# ---- 六芒星（舵机盘）
draw_prism(star6, z_horn, z_horn + HORN_T, S_A, OX_A, OY_A,
           face=(255, 200, 110), top=(255, 218, 140), side=(228, 165, 60),
           outline=(170, 100, 0))
# 盘中孔
draw_prism([(CENTER_D/2*math.cos(math.radians(a)), CENTER_D/2*math.sin(math.radians(a)))
            for a in range(0, 360, 30)],
           z_horn - 0.1, z_horn + HORN_T + 0.1, S_A, OX_A, OY_A,
           face=(250, 250, 252), top=(250, 250, 252), side=(250, 250, 252),
           outline=(170, 100, 0))

# ---- 腿（底面朝下，带六芒星凹槽）
draw_prism(outline, z_leg, z_leg + TH, S_A, OX_A, OY_A)
# 凹槽（画在底面，用深色表示凹进去）
draw_prism(star6_in, z_leg - 0.2, z_leg + 0.6, S_A, OX_A, OY_A,
           face=(255, 200, 110), top=(255, 218, 140), side=(228, 165, 60),
           outline=(170, 100, 0))
# 中心孔
draw_prism([(CENTER_D/2*math.cos(math.radians(a)), CENTER_D/2*math.sin(math.radians(a)))
            for a in range(0, 360, 30)],
           z_leg - 0.3, z_leg + TH + 0.3, S_A, OX_A, OY_A,
           face=(250, 250, 252), top=(250, 250, 252), side=(250, 250, 252),
           outline=(180, 0, 0))

# ---- 垫片 + 螺丝
draw_prism([(WASHER_D/2*math.cos(math.radians(a)), WASHER_D/2*math.sin(math.radians(a)))
            for a in range(0, 360, 30)],
           z_washer, z_washer + WASHER_T, S_A, OX_A, OY_A,
           face=(170, 175, 185), top=(200, 205, 215), side=(140, 145, 155),
           outline=(90, 95, 105))
# 螺丝头
draw_prism([(SCREW_HEAD_D/2*math.cos(math.radians(a)), SCREW_HEAD_D/2*math.sin(math.radians(a)))
            for a in range(0, 360, 30)],
           z_head, z_head + SCREW_HEAD_T, S_A, OX_A, OY_A,
           face=(140, 145, 155), top=(175, 180, 190), side=(110, 115, 125),
           outline=(70, 75, 85))

# ---- 装配箭头（沿 z 向下）
def arrow_down(x, y, z_from, z_to, s, ox, oy, color=(200, 0, 0)):
    x0, y0 = iso(x, y, z_from, s, ox, oy)
    x1, y1 = iso(x, y, z_to, s, ox, oy)
    dr.line([x0, y0, x1, y1], fill=color, width=4)
    # 箭头尖
    ang = math.atan2(y1 - y0, x1 - x0)
    for d in (0.45, -0.45):
        dr.line([x1, y1, x1 - 16*math.cos(ang + d), y1 - 16*math.sin(ang + d)],
                fill=color, width=4)

arrow_down(6, 6, z_leg + TH + 22, z_leg + TH + 4, S_A, OX_A, OY_A)
arrow_down(6, 6, z_horn + 40, z_horn + 6, S_A, OX_A, OY_A)
arrow_down(6, 6, z_leg - 40, z_leg - 6, S_A, OX_A, OY_A)

# ---- A 图文字标注
def label(txt, xy, color=(30, 30, 30), font=F, anchor='la'):
    dr.text(xy, txt, fill=color, font=font, anchor=anchor)

lx = 40
ly = 790
for t, c in [("① 舵机（花键朝上）", (90, 95, 105)),
             ("② 六芒星舵机盘 —— 套在花键上", (170, 100, 0)),
             ("③ 腿 —— 六芒星凹槽朝下，压进盘里", (40, 70, 120)),
             ("④ φ8 垫片 + M3 螺丝 —— 从最外面（上面）拧下来", (200, 0, 0))]:
    dr.text((lx, ly), t, fill=c, font=FB)
    ly += 34

dr.text((lx, ly + 6),
        "关键：一颗 M3 同时穿「腿 → 盘 → 花键」，\n"
        "      把三层压成一个整体 —— 不用碰舵机盘上的小孔。",
        fill=(120, 0, 0), font=FB, spacing=8)

# ================================================================
# B 图：中心剖面图（右）
# ================================================================
BX, BY = 880, 300        # 剖面图原点（模型 x=0, z=0 处）
S_B = 6.2

def bx(x):  return int(BX + x * S_B)
def bz(z):  return int(BY - z * S_B)

def rect(xy, **kw):
    # z 向上 -> 屏幕 y 变小，所以坐标要自动排序，否则 PIL 报错
    x0, y0, x1, y1 = xy
    dr.rectangle([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)], **kw)

dr.text((BX - 60, 24), "B. 中心剖面（看清螺丝路径）", fill=(0, 0, 0), font=FT)

# 舵机本体
rect([bx(-SERVO_W/2), bz(z_servo), bx(SERVO_W/2), bz(-4)], fill=(120, 125, 135), outline=(80, 85, 95))
# 花键
rect([bx(-SPLINE_R), bz(-4), bx(SPLINE_R), bz(0)], fill=(210, 210, 215), outline=(150, 150, 160))

# 六芒星（剖面：y=0 处穿过星尖，所以从 -STAR_RO..STAR_RO）
rect([bx(-STAR_RO), bz(0), bx(STAR_RO), bz(HORN_T)], fill=(255, 205, 120), outline=(170, 100, 0), width=2)
# 盘中心孔
rect([bx(-CENTER_D/2), bz(0), bx(CENTER_D/2), bz(HORN_T)], fill='white', outline=(170, 100, 0))

# 腿（z 从 HORN_T 到 HORN_T+TH；凹槽从底下挖 STAR_DEP）
z_leg0 = HORN_T
z_leg1 = HORN_T + TH
rect([bx(-ROOT_R), bz(z_leg1), bx(ROOT_R), bz(z_leg0)], fill=(210, 222, 240), outline=(40, 70, 120), width=2)
# 腿向右延伸（画一段示意 + 省略号）
dr.polygon([[bx(ROOT_R), bz(z_leg1)], [bx(ROOT_R + 34), bz(z_leg1)],
            [bx(ROOT_R + 34), bz(z_leg0 + 0.8)], [bx(ROOT_R), bz(z_leg0)]],
           fill=(210, 222, 240), outline=(40, 70, 120))
# 凹槽（从底面往上挖）：两侧留下，中间挖掉到 ±STAR_RO，但中心孔处保留
pocket_z0 = z_leg0
pocket_z1 = z_leg0 + STAR_DEP
rect([bx(-STAR_RO), bz(pocket_z1), bx(-CENTER_D/2), bz(pocket_z0)], fill='white', outline=(170, 100, 0))
rect([bx(CENTER_D/2), bz(pocket_z1), bx(STAR_RO), bz(pocket_z0)], fill='white', outline=(170, 100, 0))
# 腿的中心孔（贯穿）
rect([bx(-CENTER_D/2), bz(z_leg1), bx(CENTER_D/2), bz(pocket_z1)], fill='white', outline=(180, 0, 0))
rect([bx(-CENTER_D/2), bz(pocket_z0), bx(CENTER_D/2), bz(0)], fill='white', outline=(180, 0, 0))

# 垫片 + 螺丝
rect([bx(-WASHER_D/2), bz(z_leg1 + WASHER_T), bx(WASHER_D/2), bz(z_leg1)], fill=(175, 180, 190), outline=(90, 95, 105), width=2)
rect([bx(-SCREW_HEAD_D/2), bz(z_leg1 + WASHER_T + SCREW_HEAD_T), bx(SCREW_HEAD_D/2), bz(z_leg1 + WASHER_T)],
             fill=(140, 145, 155), outline=(70, 75, 85), width=2)
# 螺杆（M3，从螺丝头下方一直穿到花键里）
rect([bx(-1.5), bz(z_leg1 + WASHER_T), bx(1.5), bz(-3.0)], fill=(105, 110, 120), outline=(60, 65, 75))

# 标注：螺丝从外面拧
ay = bz(z_leg1 + WASHER_T + SCREW_HEAD_T) - 70
dr.line([bx(0), ay + 40, bx(0), ay + 8], fill=(200, 0, 0), width=4)
dr.polygon([[bx(0), ay + 8], [bx(0) - 9, ay + 24], [bx(0) + 9, ay + 24]], fill=(200, 0, 0))
dr.text((bx(0) + 16, ay + 2), "从外面（上方）拧 M3", fill=(200, 0, 0), font=FB)

# 标注：各层
# 剖面各层 -> 打上数字圆标，图例放在右侧（不要引线，避免交叉）
_cal = [
    (z_leg1 + WASHER_T/2, "1", (90, 95, 105)),
    (z_leg0 + TH/2,       "2", (40, 70, 120)),
    (z_leg0 + STAR_DEP/2, "3", (170, 100, 0)),
    (HORN_T/2,            "4", (170, 100, 0)),
    (-1.5,                "5", (120, 120, 130)),
]
_cx = bx(STAR_RO + 16)
for (zz, num, col) in _cal:
    yy = bz(zz)
    dr.ellipse([_cx - 12, yy - 12, _cx + 12, yy + 12], fill='white', outline=col, width=2)
    dr.text((_cx, yy), num, fill=col, font=FB, anchor="mm")

# 图例（右上空白处，竖排）
LGX, LGY = W - 470, 150
dr.text((LGX, LGY - 34), "剖面图例", fill=(0, 0, 0), font=FB)
_legend = [
    ("1", "φ8 钢垫片（必须有）", (90, 95, 105)),
    ("2", "腿（厚 8mm，中心 φ3.5 通孔）", (40, 70, 120)),
    ("3", "六芒星凹槽（深 2.6mm，过盈 0.15）", (170, 100, 0)),
    ("4", "六芒星舵机盘（厚 2.5mm）", (170, 100, 0)),
    ("5", "舵机输出花键（25 齿）", (120, 120, 130)),
]
for i, (num, txt, col) in enumerate(_legend):
    yy = LGY + i * 32
    dr.text((LGX, yy), num + ".", fill=col, font=FB)
    dr.text((LGX + 26, yy + 2), txt, fill=col, font=F)
# 结论框
rect([BX - 70, 780, W - 20, 960], outline=(150, 0, 0), width=2)
dr.text((BX - 56, 792), "为什么这样能「卡住 + 不打滑」：", fill=(150, 0, 0), font=FB)
for i, t in enumerate([
    "① 六芒星嵌进凹槽 → 6 个面挡住转动（形状配合，不靠摩擦）",
    "② M3 + 垫片把「腿 + 盘 + 花键」压成一体 → 不会掉",
    "③ 垫片分散压力 → PETG 不会被螺丝头压出坑",
]):
    dr.text((BX - 46, 826 + i * 30), t, fill=(60, 60, 60), font=F)

dr.text((40, H - 40), "LEG 70mm  /  thickness 8mm  /  star pocket depth 2.6mm  /  interference 0.15mm",
        fill=(120, 120, 130), font=F)

img.save(OUT)
print('已生成:', OUT, '(%dx%d)' % (W, H))
