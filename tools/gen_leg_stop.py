#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_leg_stop.py —— 画"挡件 + 腿上的平台"（工作角侧视图）

关键点：平台的上表面在【工作角】时必须是【水平】的，挡件从上面压住它，
把"电机+轮子"的静态重量卸到车身结构上，舵机只在调整高度时出力。
"""
import math, os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'mech', 'leg_stop_pad.png')

# ---------------- 参数 ----------------
L        = 70.0     # 舵机轴 -> 电机轴
WORK_ANG = 45.0     # 工作角（腿相对水平向下的角度）
PAD_R    = 35.0     # 平台中心沿腿的半径
PAD_L    = 12.0     # 平台接触面长度（沿斜面的坡长）
PAD_H    = 4.0      # 平台凸起高度
PAD_W    = 8.0      # 平台宽度（= 腿厚）
STOP_H   = 10.0     # 挡件高度
STOP_W   = 16.0     # 挡件宽度
STOP_D   = 12.0     # 挡件深度
MOTOR_R  = 18.0     # 电机+轮 的半径（示意）
LEG_HALF = 9.0      # 腿半高（示意）

# ---------------- 画布 ----------------
W, H = 1150, 780
img = Image.new('RGB', (W, H), (250, 250, 252))
dr = ImageDraw.Draw(img)

def load_font(size, bold=False):
    for c in ([r"C:\Windows\Fonts\msyhbd.ttc"] if bold else []) + [r"C:\Windows\Fonts\msyh.ttc", "consola.ttf"]:
        try:
            return ImageFont.truetype(c, size)
        except Exception:
            pass
    return ImageFont.load_default()

F  = load_font(16)
FB = load_font(20, True)
FT = load_font(26, True)

SC = 4.4
OX, OY = 130, 300        # 世界原点（舵机轴）在屏幕上的位置

def wx(x, y):
    return (OX + x * SC, OY - y * SC)

# ---------------- 腿（旋转 -WORK_ANG）----------------
th = math.radians(-WORK_ANG)
ax = (math.cos(th), math.sin(th))      # 腿轴方向
up = (-math.sin(th), math.cos(th))     # 腿的"上方"法线（旋转后的 +Y）
# 注意：腿的上方 = 轴方向逆时针转 90°
up = (math.cos(th + math.pi/2), math.sin(th + math.pi/2))

def leg_half(t):
    """沿腿 t(mm) 处的半高"""
    ctrl = [(0, 13.5), (25, 9), (45, 8), (62, 10.5), (72, 10.5)]
    for i in range(len(ctrl) - 1):
        x0, r0 = ctrl[i]; x1, r1 = ctrl[i + 1]
        if t <= x1 or i == len(ctrl) - 2:
            k = 0.0 if x1 == x0 else max(0.0, min(1.0, (t - x0) / (x1 - x0)))
            return r0 + (r1 - r0) * k
    return ctrl[-1][1]

# 轮廓点
top_pts, bot_pts = [], []
N = 60
for i in range(N + 1):
    t = -13.5 + (L + 20 + 13.5) * i / N
    if t < 0:
        h = math.sqrt(max(0.0, 13.5**2 - t**2))
    else:
        h = leg_half(t)
    for (pts, sign) in ((top_pts, 1), (bot_pts, -1)):
        px = ax[0] * t + up[0] * sign * h
        py = ax[1] * t + up[1] * sign * h
        pts.append(wx(px, py))

poly = top_pts + bot_pts[::-1]
dr.polygon(poly, fill=(210, 222, 240), outline=(40, 70, 120))
dr.line(poly + [poly[0]], fill=(40, 70, 120), width=3)

# ---------------- 电机 + 轮（腿尖）----------------
cx, cy = ax[0] * L, ax[1] * L
p = wx(cx, cy)
r = MOTOR_R * SC
dr.ellipse([p[0]-r, p[1]-r, p[0]+r, p[1]+r], fill=(170, 175, 185), outline=(90, 95, 105), width=3)
dr.ellipse([p[0]-r*0.35, p[1]-r*0.35, p[0]+r*0.35, p[1]+r*0.35], fill=(120, 125, 135), outline=(70, 75, 85))
dr.text((p[0] + r + 8, p[1] - 10), "电机+轮\n(108g)", fill=(90, 95, 105), font=F, spacing=4)

# ---------------- 平台（上表面水平）----------------
# 平台中心位置（沿腿 PAD_R，往"上"偏 半高 + 凸起）
h_at = leg_half(PAD_R)
base_x = ax[0] * PAD_R + up[0] * (h_at - 1.0)
base_y = ax[1] * PAD_R + up[1] * (h_at - 1.0)
top_y  = base_y + PAD_H          # 上表面高度（世界坐标）

# 平台画成一个矩形：从左下到右，顶面水平
# 平台 = 水平矩形（因为上表面必须是水平的）
pad_pts = [
    wx(base_x - PAD_L/2, base_y),
    wx(base_x + PAD_L/2, base_y),
    wx(base_x + PAD_L/2, top_y),
    wx(base_x - PAD_L/2, top_y),
]
dr.polygon(pad_pts, fill=(255, 205, 120), outline=(170, 100, 0))
dr.line(pad_pts + [pad_pts[0]], fill=(170, 100, 0), width=2)
# 上表面高亮
dr.line([wx(base_x - PAD_L/2, top_y), wx(base_x + PAD_L/2, top_y)], fill=(200, 90, 0), width=5)
# ---------------- 挡件（金属块，在车身上）----------------
# 挡件底面 = 平台顶面（接触），挡件向上
sx0 = base_x - STOP_W/2
sx1 = base_x + STOP_W/2
stop_pts = [
    wx(sx0, top_y),
    wx(sx1, top_y),
    wx(sx1, top_y + STOP_H),
    wx(sx0, top_y + STOP_H),
]
dr.polygon(stop_pts, fill=(150, 155, 165), outline=(70, 75, 85))
dr.line(stop_pts + [stop_pts[0]], fill=(70, 75, 85), width=3)
dr.text((wx(sx1, top_y)[0] + 10, wx(sx1, top_y)[1] + 14),
        "挡件（金属）", fill=(70, 75, 85), font=FB)
# ---------------- 车身（示意）----------------
bx0 = base_x - 40
bx1 = base_x + 40
body_pts = [
    wx(bx0, top_y + STOP_H + 10),
    wx(bx1, top_y + STOP_H + 10),
    wx(bx1, top_y + STOP_H + 22),
    wx(bx0, top_y + STOP_H + 22),
]
dr.polygon(body_pts, fill=(235, 235, 240), outline=(150, 150, 160))
dr.text((wx(bx0, top_y + STOP_H + 46)[0], wx(bx0, top_y + STOP_H + 26)[1]),
        "车身底板", fill=(120, 120, 130), font=F)

# ---------------- 力箭头 ----------------
# 电机+轮 的重量通过腿 -> 平台 -> 挡件（竖直向下压在挡件上）
ax_p = wx(base_x, top_y + 8)
ax_e = wx(base_x, top_y + 30)
dr.line([ax_p, ax_e], fill=(200, 0, 0), width=4)
dr.polygon([[ax_p[0], ax_p[1]], [ax_p[0]-8, ax_p[1]-14], [ax_p[0]+8, ax_p[1]-14]], fill=(200, 0, 0))
dr.text((ax_p[0] + 14, ax_p[1] - 30), "负载传到挡件", fill=(200, 0, 0), font=FB)
dr.text((ax_p[0] + 14, ax_p[1] - 8), "舵机此时不出力", fill=(200, 0, 0), font=F)

# ---------------- 尺寸标注 ----------------
# 工作角
arc_r = 90
dr.arc([OX - arc_r, OY - arc_r, OX + arc_r, OY + arc_r], -WORK_ANG, 0, fill=(120, 0, 120), width=3)
dr.text((OX + arc_r * 0.55, OY - arc_r * 0.55 + 6), "工作角 %.0f°" % WORK_ANG,
        fill=(120, 0, 120), font=FB)
# 水平参考线
dr.line([OX, OY, OX + 190, OY], fill=(180, 180, 190), width=1)
dr.text((OX + 150, OY + 4), "水平", fill=(150, 150, 160), font=F)

# PAD_R
mp = wx(ax[0] * PAD_R * 0.5, ax[1] * PAD_R * 0.5)
dr.text((mp[0] - 20, mp[1] + 26), "R=%.0f" % PAD_R, fill=(170, 100, 0), font=FB)

# ---------------- 说明框 ----------------
dr.rectangle([600, 60, W - 20, 300], outline=(150, 90, 0), width=2)
dr.text((616, 72), "挡件 + 平台 的三条硬要求", fill=(150, 90, 0), font=FB)
for i, t in enumerate([
    "1. 平台的上表面，在【工作角】时必须水平",
    "2. 挡件底面也水平 → 面和面接触（≥100 mm²）",
    "3. 加垫片调高度 = 调工作角（0.1mm ≈ 0.16°）",
]):
    dr.text((616, 106 + i * 32), t, fill=(60, 60, 60), font=F)
for i, t in enumerate([
    "",
    "为什么：腿末端挂着电机+轮（108g），",
    "静态力矩 0.76 kg·cm 会让舵机长期发热。",
    "平台顶住挡件后 → 重量走结构，舵机零负载。",
]):
    dr.text((616, 210 + i * 26), t, fill=(120, 0, 0), font=F)

img.save(OUT)
print('已生成:', OUT, '(%dx%d)' % (W, H))
