#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
body_v5_layout.py —— body_v5 下层俯视布局 + 接线规则标注

坐标与 mech/body_v5.scad 一致：x=前后(+x 前)，y=左右，原点=托盘中心。
只画示意图（不追求精确像素），用来核对"模块在哪 + 线怎么走"。
"""
import sys, io, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_layout.png'

# ---- 尺寸（与 scad 一致）----
L, W, WALL = 248, 150, 3
IN_X, IN_Y = L/2 - WALL, W/2 - WALL
SV_X, SV_HW, SV_HH = 85, 42, 21
SV_DEPTH = 42.9

# 模块: (name, cx, cy, X, Y, color, ink)
MODS = [
    ("电池 88x61",     -71.5,   0,  88, 61, (214, 96, 80),  (255,255,255)),
    ("12->5 降压",     -40,    49,  43, 21, (90, 190, 205), (20,20,20)),
    ("舵机板 62.3x25.5",  20,    45,  62.3, 25.5, (70, 165, 110), (255,255,255)),
    ("电机板 50x50",     10,   -30,  50, 50, (128, 92, 175), (255,255,255)),
    ("端子排 76x8",      15,   -67,  76,  8, (60, 105, 175),  (255,255,255)),
    ("保险丝+开关",       0,    15,  30, 18, (225, 190, 60), (20,20,20)),
    ("12->9 降压",      85,     0,  43, 21, (90, 190, 205), (20,20,20)),
]
ULTRA = ("超声波 HC-SR04", L/2 - 4, 0, 4, 45, (240, 150, 50), (20,20,20))

PART_X = [-23, 60]

SC = 6.0
PAD = 90
CW = int(L*SC) + PAD*2
CH = int(W*SC) + PAD*2 + 120

img = Image.new('RGB', (CW, CH), (250, 250, 252))
d = ImageDraw.Draw(img)

def font(sz, bold=False):
    paths = ([r"C:\Windows\Fonts\msyhbd.ttc"] if bold else []) + [r"C:\Windows\Fonts\msyh.ttc"]
    for p in paths:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()

F   = font(15)
FB  = font(17, True)
FT  = font(26, True)
FX  = font(13)

def X(x): return PAD + (x + L/2) * SC
def Y(y): return PAD + (W/2 - y) * SC

# ---- 车身 ----
def rr(box, r, **kw):
    d.rounded_rectangle(box, radius=r, **kw)

x0, y0 = X(-L/2), Y(W/2)
x1, y1 = X(L/2), Y(-W/2)
rr([x0, y0, x1, y1], 8*SC, outline=(45, 55, 80), width=4)
rr([X(-IN_X), Y(IN_Y), X(IN_X), Y(-IN_Y)], 6*SC, outline=(150, 155, 170), width=2)

def box(x0m, x1m, y0m, y1m):
    """模型坐标 → PIL 矩形（自动归一化，保证 y0<=y1）"""
    px0, px1 = X(min(x0m, x1m)), X(max(x0m, x1m))
    py0, py1 = Y(max(y0m, y1m)), Y(min(y0m, y1m))
    return [px0, py0, px1, py1]

# ---- 舵机穿墙区（4 块）----
for sx in (-1, 1):
    for sy in (-1, 1):
        bx0, bx1 = sx*SV_X - SV_HW/2, sx*SV_X + SV_HW/2
        by0, by1 = sy*(W/2 - SV_DEPTH), sy*(W/2 + 6)
        d.rectangle(box(bx0, bx1, by0, by1), fill=(238, 228, 228), outline=(185, 125, 125), width=2)
        d.text((X(min(bx0,bx1))+6, Y(max(by0,by1))+4), "舵机", fill=(165, 95, 95), font=FX)

# ---- 分区墙 ----
for px in PART_X:
    d.rectangle([X(px-1.25), Y(30), X(px+1.25), Y(-30)], fill=(70, 90, 150))

# ---- 模块 ----
def draw_mod(m):
    name, cx, cy, mx, my, col, ink = m
    d.rectangle([X(cx-mx/2), Y(cy+my/2), X(cx+mx/2), Y(cy-my/2)], fill=col, outline=(40,40,55), width=2)
    lines = name.split("\n")
    ty = Y(cy) - len(lines)*9
    for ln in lines:
        tw = d.textlength(ln, font=F)
        d.text((X(cx)-tw/2, ty), ln, fill=ink, font=F)
        ty += 20

for m in MODS: draw_mod(m)
draw_mod(ULTRA)

# ---- 四角穿孔柱（φ2.6 穿 φ3.1/3.2 孔）----
PINS = [
    (10, -30, 44, 44),    # 电机板
    (20, 45, 56, 19),     # 舵机板
    (-40, 49, 30, 15),    # 12->5
    (85, 0, 30, 15),      # 12->9
]
for (pcx, pcy, pdx, pdy) in PINS:
    for sx2 in (-1, 1):
        for sy2 in (-1, 1):
            pxx = X(pcx + sx2*pdx/2); pyy = Y(pcy + sy2*pdy/2)
            d.ellipse([pxx-6, pyy-6, pxx+6, pyy+6], outline=(255, 255, 255), width=3)
            d.ellipse([pxx-2, pyy-2, pxx+2, pyy+2], fill=(255, 255, 255))

# ---- 接线规则 ----
RED   = (205, 45, 45)
BLACK = (35, 35, 35)
ORG   = (215, 125, 20)
BLUE  = (40, 90, 200)
GRAY  = (110, 110, 125)

def arrow(p, q, col, w=4, dash=False):
    if dash:
        n = max(2, int(((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5 / 14))
        for i in range(n):
            t0, t1 = i/n, (i+0.55)/n
            d.line([(p[0]+(q[0]-p[0])*t0, p[1]+(q[1]-p[1])*t0),
                    (p[0]+(q[0]-p[0])*t1, p[1]+(q[1]-p[1])*t1)], fill=col, width=w)
    else:
        d.line([p, q], fill=col, width=w)
    # 箭头
    import math
    ang = math.atan2(q[1]-p[1], q[0]-p[0])
    for s in (-0.42, 0.42):
        d.line([q, (q[0]-18*math.cos(ang+s), q[1]-18*math.sin(ang+s))], fill=col, width=w)

# 电池 + → 保险丝 → 端子排（12V）
bat_out = (X(-76.5)+44*SC, Y(0)+8*SC)
arrow(bat_out, (X(-12)+15*SC, Y(21)+2*SC), RED, 5)
arrow((X(-12)+15*SC, Y(21)-2*SC), (X(15)-14*SC, Y(-67)+2*SC), RED, 5)
# 电池 - → 端子排（GND）
arrow((X(-76.5)+44*SC, Y(0)-8*SC), (X(15)-10*SC, Y(-67)-2*SC), BLACK, 5)

# 端子排 → 电机板 Vin
arrow((X(15)-2*SC, Y(-67)+5*SC), (X(10)+18*SC, Y(-30)-25*SC), RED, 3, dash=True)
arrow((X(15)-6*SC, Y(-67)+5*SC), (X(10)+22*SC, Y(-30)-25*SC), BLACK, 3, dash=True)
# 端子排 → 12->9
arrow((X(15)+28*SC, Y(-67)+5*SC), (X(85)-20*SC, Y(0)-14*SC), RED, 3, dash=True)
# 端子排 → 12->5
arrow((X(15)-20*SC, Y(-67)+5*SC), (X(-40)+20*SC, Y(49)-14*SC), RED, 3, dash=True)

# 12->9 输出 → FPGA（上层中部 DC 孔 x≈37,y≈-20）
arrow((X(85)-20*SC, Y(0)), (X(44), Y(-20)), BLUE, 4)
d.text((X(46), Y(-30)), "9V → 上层 FPGA DC 孔", fill=BLUE, font=FX)

# 12->5 输出 → 舵机板 V+
arrow((X(-40)+20*SC, Y(49)+14*SC), (X(20)-31*SC, Y(45)+12.5*SC), ORG, 4)
d.text((X(-16), Y(62)), "6V → 舵机", fill=ORG, font=FX)

# ---- 图例 ----
lx, ly = PAD, CH - 96
items = [
    (RED,   "12V 正（电池 → 保险丝 → 端子排 → 各路）"),
    (BLACK, "GND 负极（共地星形）"),
    (ORG,   "6V（UBEC/12→5 → 舵机分配板）"),
    (BLUE,  "9V（12→9 → FPGA DC 圆口）"),
    (GRAY,  "分区矮墙（12 高，线从上方跨过）"),
]
for i, (c, t) in enumerate(items):
    yy = ly + i*17
    d.rectangle([lx, yy+4, lx+22, yy+12], fill=c)
    d.text((lx+30, yy), t, fill=(60, 60, 70), font=FX)

d.text((PAD, 18), "body_v5 下层（高 45）— 按手绘布局重排 + 接线规则", fill=(25, 25, 35), font=FT)
d.text((X(L/2) - 130, Y(W/2) - 34), "前 ▲（超声波端）", fill=(200, 40, 40), font=FB)
d.text((X(-L/2) + 12, Y(-W/2) + 12), "后（电池端）", fill=(90, 90, 100), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
