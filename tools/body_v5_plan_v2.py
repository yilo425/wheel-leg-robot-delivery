#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""body_v5 第二层方案图：充电宝内置（79 沿车长 / 35 跨车宽）+ K230 偏 Y。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_plan_v2.png'

L, W, WALL = 236, 150, 3
IX, IY = L/2 - WALL, W/2 - WALL          # 115, 72
FPG = (-115, 35, -54.5, 54.5)            # x0,x1,y0,y1
K3 = (56, 112, -60, 25)                  # 56(X) x 85(Y)
PB = (36, 115, 30, 65)                   # 79(X) x 35(Y)
DCJ = (35, 49, -40, -24)                 # 弯头占位
CAM = (112, 115, -13, 13)

CW, CH = 1800, 1240
img = Image.new('RGB', (CW, CH), (252, 252, 254))
d = ImageDraw.Draw(img)

def font(sz, b=False):
    for p in ([r"C:\Windows\Fonts\msyhbd.ttc"] if b else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F, FB, FT, FX = font(16), font(19, True), font(27, True), font(14)

SC = 4.2
OX, OY = 330, 170
def X(x): return OX + (x + L/2) * SC
def Y(y): return OY + (W/2 - y) * SC
def B(x0, x1, y0, y1):
    return [X(min(x0, x1)), Y(max(y0, y1)), X(max(x0, x1)), Y(min(y0, y1))]

# 车壳
d.rounded_rectangle(B(-L/2, L/2, -W/2, W/2), radius=8*SC, fill=(238, 242, 248),
                    outline=(40, 50, 70), width=4)
d.rounded_rectangle(B(-IX, IX, -IY, IY), radius=5*SC, outline=(160, 170, 190), width=2)

# 方向标
d.text((X(0) - 6, OY - 112), "▲ 前 (+X，摄像头端)", fill=(200, 60, 60), font=FB)
d.text((X(-L/2) - 96, Y(0)), "后\n(-X)\nFPGA 端", fill=(120, 120, 135), font=FX)
d.text((X(0) - 150, Y(W/2) - 46), "← +Y（车左）", fill=(120, 120, 135), font=FX)
d.text((X(0) - 150, Y(-W/2) + 26), "← −Y（车右）", fill=(120, 120, 135), font=FX)

# --- FPGA ---
d.rectangle(B(*FPG), fill=(64, 96, 165), outline=(30, 45, 90), width=3)
d.text((X(-60) - 100, Y(12) - 30), "FPGA", fill=(255, 255, 255), font=FT)
d.text((X(-60) - 100, Y(12) + 6), "150（X） × 109（Y）", fill=(220, 230, 250), font=F)
d.text((X(-60) - 100, Y(12) + 30), "自带升高支架 → 不做支撑", fill=(200, 215, 240), font=FX)

# --- 充电宝 ---
d.rectangle(B(*PB), fill=(232, 232, 240), outline=(110, 110, 130), width=3)
d.text((X(60), Y(56) - 10), "充电宝", fill=(40, 40, 60), font=FB)
d.text((X(60), Y(56) + 14), "79（X）× 35（Y）", fill=(80, 80, 100), font=FX)
d.text((X(60), Y(56) + 34), "Type-C 在长边中间", fill=(80, 80, 100), font=FX)

# --- K230 ---
d.rectangle(B(*K3), fill=(198, 86, 98), outline=(130, 40, 52), width=3)
d.text((X(84) - 60, Y(-14) - 12), "K230", fill=(255, 255, 255), font=FB)
d.text((X(84) - 60, Y(-14) + 12), "56（X）× 85（Y）", fill=(255, 235, 235), font=FX)
for sx in (-1, 1):
    for sy in (-1, 1):
        px = 84 + sx*(28 + 4.4); py = -17.5 + sy*(42.5 + 4.4)
        d.rectangle(B(px-2, px+2, py-2, py+2), fill=(70, 95, 160), outline=(35, 50, 95))

# --- 摄像头 ---
d.rectangle(B(*CAM), fill=(240, 165, 70), outline=(150, 90, 20), width=3)

# --- DC 弯头 ---
d.rectangle(B(*DCJ), fill=(120, 195, 155), outline=(40, 115, 80), width=3)
d.text((X(DCJ[0]) - 6, Y(DCJ[2]) + 34), "DC 弯头 14mm", fill=(20, 100, 65), font=FX)

# --- Type-C 软线 ---
d.line([(X(56), Y(18)), (X(66), Y(24)), (X(70), Y(30))], fill=(90, 90, 115), width=5)
d.text((X(-46), 118), "Type-C 软线：K230 侧口 → 充电宝长边口（S 弯 ~22mm）",
       fill=(90, 90, 115), font=FX)

# --- 固定桩孔 ---
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = sx*(L/2 - 7), sy*68
        d.ellipse([X(cx)-8, Y(cy)-8, X(cx)+8, Y(cy)+8], fill=(255,255,255),
                  outline=(90, 90, 110), width=2)

# 尺寸
d.line([(X(-L/2), Y(-W/2)+64), (X(L/2), Y(-W/2)+64)], fill=(200, 60, 60), width=2)
d.text((X(-16), Y(-W/2)+70), f"L = {L}", fill=(200, 60, 60), font=FB)
d.line([(X(-L/2)-52, Y(-W/2)), (X(-L/2)-52, Y(W/2))], fill=(200, 60, 60), width=2)
d.text((X(-L/2)-160, Y(0)-10), f"W = {W}", fill=(200, 60, 60), font=FB)

# 标题
d.text((40, 44), "body_v5 第二层方案 v2 —— 充电宝内置（长边沿车长）", fill=(25, 25, 35), font=FT)
d.text((40, 86), "关键：充电宝 35 跨车宽 + K230 85 跨车宽 = 120 ≤ 144（内腔），所以不用加宽也不用加长",
       fill=(60, 60, 80), font=F)

# 右侧要点
tx = X(L/2) + 60
d.text((tx, 180), "尺寸核对", fill=(25, 25, 35), font=FB)
lines = [
    ("内腔", "230 × 144"),
    ("FPGA", "150 × 109  → x −115…35"),
    ("DC 弯头", "x 35…49（14）"),
    ("充电宝", "x 36…115（79）  y +30…+65"),
    ("K230", "x 56…112（56）  y −60…+25"),
    ("X 合计", "150 + 79 = 229 ≤ 230 ✓"),
    ("Y 合计", "35 + 85 = 120 ≤ 144 ✓"),
    ("L", f"{L}（不变）"),
]
for i, (k, v) in enumerate(lines):
    yy = 220 + i*38
    d.text((tx, yy), k, fill=(90, 90, 110), font=FX)
    d.text((tx + 96, yy), v, fill=(30, 30, 40), font=FX)

d.text((tx, 560), "要你确认的两点", fill=(25, 25, 35), font=FB)
d.text((tx, 596), "① FPGA 的 DC 孔在哪一侧？", fill=(200, 60, 60), font=F)
d.text((tx, 620), "   本图假定在 −Y 侧（图下方），", fill=(80, 80, 95), font=FX)
d.text((tx, 640), "   这样和充电宝不在同一 Y 带，", fill=(80, 80, 95), font=FX)
d.text((tx, 660), "   弯头不撞充电宝。", fill=(80, 80, 95), font=FX)
d.text((tx, 692), "② K230 的 Type-C 靠哪个角？", fill=(200, 60, 60), font=F)
d.text((tx, 716), "   本图假定在 −X 长边、近 +Y 端", fill=(80, 80, 95), font=FX)
d.text((tx, 736), "   （就是靠近充电宝那个角）。", fill=(80, 80, 95), font=FX)
d.text((tx, 772), "如果实际是镜像的，整体左右翻转即可，", fill=(80, 80, 95), font=FX)
d.text((tx, 792), "尺寸完全不变。", fill=(80, 80, 95), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
