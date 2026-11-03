#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""body_v5 上层平面图（X-Y），标注 FPGA / K230 / 摄像头 / DC弯头 / 走线。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_upper_plan.png'

L, W, WALL = 236, 150, 3
IN_X, IN_Y = L/2 - WALL, W/2 - WALL
FPG = (-40, 0, 150, 109)
K3 = (84, 0, 56, 85)
KP = 4.4                      # K230 限位柱偏移
POST_X, POST_Y = L/2 - 7, 68
DCJ = (35, 49, -30, 16)       # x0,x1,cy,w
CAM = (112, 115, 26)          # x0,x1,宽
SLOTS = [(-6, -28, 14, 16, "9V 过线孔"),
         (-110, 50, 70, 16, "侧边走线"),
         (-110, -66, 70, 16, "侧边走线")]
ZIP = [                       # 层间扎带/固定孔（示意）
        ]

CW, CH = 1780, 1120
img = Image.new('RGB', (CW, CH), (252, 252, 254))
d = ImageDraw.Draw(img)

def font(sz, b=False):
    for p in ([r"C:\Windows\Fonts\msyhbd.ttc"] if b else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F, FB, FT, FX = font(16), font(19, True), font(26, True), font(14)

SC = 5.4
OX, OY = 120, 150
def X(x): return OX + (x + L/2) * SC
def Y(y): return OY + (W/2 - y) * SC
def B(xa, xb, ya, yb):
    return [X(min(xa, xb)), Y(max(ya, yb)), X(max(xa, xb)), Y(min(ya, yb))]

# 车壳
d.rounded_rectangle(B(-L/2, L/2, -W/2, W/2), radius=8 * SC,
                    fill=(238, 242, 248), outline=(40, 50, 70), width=4)
d.rounded_rectangle(B(-IN_X, IN_X, -IN_Y, IN_Y), radius=5 * SC,
                    outline=(150, 160, 180), width=2)

# 层间走线孔
for x0, y0, w, h, lab in SLOTS:
    d.rectangle(B(x0, x0 + w, y0, y0 + h), fill=(255, 255, 255), outline=(140, 140, 150), width=2)
    d.text((X(x0) + 4, Y(y0) - 22), lab, fill=(120, 120, 135), font=FX)

# FPGA
f0, f1, fw, fh = FPG
d.rectangle(B(f0 - fw/2, f0 + fw/2, f1 + fh/2, f1 - fh/2),
            fill=(64, 96, 165), outline=(30, 45, 90), width=3)
d.text((X(f0) - 120, Y(0) - 34), "FPGA  150 × 109", fill=(255, 255, 255), font=FT)
d.text((X(f0) - 120, Y(0) + 2), "自带升高支架 → 不做支撑", fill=(215, 225, 245), font=F)
d.text((X(f0) - 120, Y(0) + 28), f"x {-115} … {35}    y ±{54.5}", fill=(215, 225, 245), font=FX)

# K230
k0, k1, kw, kh = K3
d.rectangle(B(k0 - kw/2, k0 + kw/2, k1 + kh/2, k1 - kh/2),
            fill=(198, 86, 98), outline=(130, 40, 52), width=3)
d.text((X(k0) - 58, Y(0) - 30), "K230", fill=(255, 255, 255), font=FB)
d.text((X(k0) - 58, Y(0) - 4), "56 × 85", fill=(255, 255, 255), font=F)
for sx in (-1, 1):
    for sy in (-1, 1):
        px, py = k0 + sx*(kw/2 + KP), k1 + sy*(kh/2 + KP)
        d.rectangle(B(px - 2, px + 2, py + 2, py - 2), fill=(70, 95, 160), outline=(35, 50, 95))
d.text((X(k0) - 30, Y(-kh/2) + 64), "四角 4mm 限位柱", fill=(70, 95, 160), font=FX)

# 摄像头（抬高 z=33，在 K230 上方 —— 平面图重叠是正常的）
d.rectangle(B(CAM[0], CAM[1], 13, -13), fill=(240, 165, 70), outline=(150, 90, 20), width=3)
d.text((X(L/2) + 12, Y(0) - 10), "摄像头", fill=(150, 90, 20), font=FB)
d.text((X(L/2) + 12, Y(0) + 14), "26×26", fill=(150, 90, 20), font=FX)
d.text((X(L/2) + 12, Y(0) + 34), "抬高 z=33", fill=(150, 90, 20), font=FX)

# DC 弯头
x0, x1, cy, w = DCJ
d.rectangle(B(x0, x1, cy + w/2, cy - w/2), fill=(120, 195, 155), outline=(50, 120, 85), width=3)
d.text((X(x0) - 30, Y(cy - w/2) + 16), "DC 弯头  X 占 14mm（朝上）", fill=(20, 100, 65), font=FX)

# 充电宝 Type-C 软线（前上方 -> K230 的 -X 长边）
d.line([(X(118), Y(64)), (X(64), Y(64)), (X(64), Y(0)), (X(56), Y(0))],
       fill=(110, 110, 130), width=4, joint='curve')
d.text((X(20), Y(68) - 18), "Type-C 软线（从前上方翻过前壁，沿 +Y 走到 K230 的 −X 长边）",
       fill=(110, 110, 130), font=FX)

# 固定桩孔
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy2 = sx*POST_X, sy*POST_Y
        d.ellipse([X(cx) - 7, Y(cy2) - 7, X(cx) + 7, Y(cy2) + 7],
                  fill=(255, 255, 255), outline=(90, 90, 110), width=2)

# 尺寸线
d.line([(X(-L/2), Y(-W/2) + 52), (X(L/2), Y(-W/2) + 52)], fill=(200, 60, 60), width=2)
d.text((X(-18), Y(-W/2) + 58), f"L = {L}", fill=(200, 60, 60), font=F)
d.line([(X(-L/2) - 44, Y(-W/2)), (X(-L/2) - 44, Y(W/2))], fill=(200, 60, 60), width=2)
d.text((X(-L/2) - 108, Y(0)), f"W\n{W}", fill=(200, 60, 60), font=F)

d.text((40, 44), "body_v5 第二层平面图（俯视，+X = 前 / 摄像头端，+Y = 左）",
       fill=(25, 25, 35), font=FT)
d.text((40, 84), f"车身 {L}×{W}，层高 50；内腔 {IN_X*2:.0f}×{IN_Y*2:.0f}",
       fill=(80, 80, 95), font=F)

d.text((40, CH - 44),
       "白圆=下层固定桩过孔  白块=层间走线孔  蓝=FPGA  暗红=K230  橙=摄像头(抬高)  绿=DC弯头",
       fill=(70, 70, 85), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
