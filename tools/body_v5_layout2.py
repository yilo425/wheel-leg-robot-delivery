#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""body_v5 第二层俯视布局（FPGA + K230 + 摄像头 + 层间走线孔）。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_layout2.png'

L, W, WALL = 248, 150, 3
IN_X, IN_Y = L/2 - WALL, W/2 - WALL
POST_X, POST_Y = 118, 68

FPG = (-51, 0, 140, 92)
K3  = (89, 0, 56, 85)
PB  = (38.5, 25, 35, 79)             # 充电宝 79x35x27
CAM = (L/2 - WALL, 0, 26, 26)          # 贴前壁内
SLOTS = [                              # 层间走线孔 (x0,y0,w,h,label)
    (-3, -28, 14, 16, "9V → FPGA DC"),
    (-110, 50, 70, 16, "电机/编码器/舵机"),
    (-110, -66, 70, 16, "电机/编码器/舵机"),
]

SC = 6.0
PAD = 90
CW = int(L*SC) + PAD*2
CH = int(W*SC) + PAD*2 + 130

img = Image.new('RGB', (CW, CH), (250, 250, 252))
d = ImageDraw.Draw(img)

def font(sz, b=False):
    for p in ([r"C:\Windows\Fonts\msyhbd.ttc"] if b else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F, FB, FT, FX = font(15), font(17, True), font(25, True), font(13)

def X(x): return PAD + (x + L/2)*SC
def Y(y): return PAD + (W/2 - y)*SC
def box(x0, x1, y0, y1):
    return [X(min(x0,x1)), Y(max(y0,y1)), X(max(x0,x1)), Y(min(y0,y1))]

# 外形
d.rounded_rectangle([X(-L/2), Y(W/2), X(L/2), Y(-W/2)], radius=8*SC, outline=(45,55,80), width=4)
d.rounded_rectangle([X(-IN_X), Y(IN_Y), X(IN_X), Y(-IN_Y)], radius=6*SC, outline=(150,155,170), width=2)

def posts(cx, cy, bx, by, s=8, gap=0.4, col=(70,95,160)):
    for sx in (-1, 1):
        for sy in (-1, 1):
            px = cx + sx*(bx/2 + gap + s/2); py = cy + sy*(by/2 + gap + s/2)
            d.rectangle(box(px-s/2, px+s/2, py-s/2, py+s/2), fill=col, outline=(40,55,100), width=2)

# 走线孔
for (x0, y0, w, h, lbl) in SLOTS:
    d.rectangle(box(x0, x0+w, y0, y0+h), fill=(255, 255, 255), outline=(150, 150, 165), width=2)

# FPGA（自带升高支架，不画限位柱）
f0, f1, fw, fh = FPG
d.rectangle(box(f0-fw/2, f0+fw/2, f1-fh/2, f1+fh/2), fill=(64, 96, 165), outline=(30,45,90), width=3)
d.text((X(f0)-46, Y(f1)-22), "FPGA 140x92x34（自带支架）", fill=(255,255,255), font=FB)

# K230
k0, k1, kw, kh = K3
d.rectangle(box(k0-kw/2, k0+kw/2, k1-kh/2, k1+kh/2), fill=(196, 84, 96), outline=(120,40,50), width=3)
d.text((X(k0)-30, Y(k1)-20), "K230 56x85", fill=(255,255,255), font=F)
posts(k0, k1, kw, kh)

# 充电宝兜（3 面挡边，朝 -Y 开口）
p0, p1, pw, pd = PB
d.rectangle(box(p0-pw/2, p0+pw/2, p1-pd/2, p1+pd/2), fill=(228, 228, 232), outline=(120,120,130), width=2)
d.rectangle(box(p0-pw/2-2, p0-pw/2, p1-pd/2, p1+pd/2), fill=(70,95,160))       # -X 挡边
d.rectangle(box(p0-pw/2-2, p0+pw/2, p1+pd/2, p1+pd/2+2), fill=(70,95,160))     # +Y 挡边
d.rectangle(box(p0-pw/2-2, p0+pw/2, p1-pd/2-2, p1-pd/2), fill=(70,95,160))     # -Y 挡边
d.text((X(p0)-56, Y(p1)-20), "充电宝 35(X)x79(Y)x27", fill=(70,70,80), font=F)
# Type-C 朝 +X 插进 K230 的长边
d.rectangle(box(p0+pw/2, p0+pw/2+8, p1-4.5, p1+4.5), fill=(120,120,130), outline=(80,80,90))
d.text((X(p0+pw/2)+11, Y(p1)-6), "Type-C", fill=(120,120,130), font=FX)

# FPGA DC 免焊公头（5.5×2.1，总长 36mm，卡在 FPGA 前边缘与充电宝之间）
d.rectangle(box(-7, 29, -28, -12), fill=(160, 160, 165), outline=(90, 90, 95), width=2)
d.text((X(-7)+2, Y(-12)+2), "DC 公头 36mm", fill=(80,80,85), font=FX)

# 摄像头（贴前壁，抬高到 z=33，在 K230 正上方 —— 平面图上会和 K230 重叠，属正常）
c0, c1, cw, ch = CAM
d.rectangle(box(c0-8, c0, c1-ch/2, c1+ch/2), fill=(238, 158, 60), outline=(150,90,20), width=3)
d.text((X(c0)-150, Y(c1)+4), "摄像头（抬高 z=33，在K230上方）", fill=(150,90,20), font=FX)

# 下层固定桩过孔
for sx in (-1, 1):
    for sy in (-1, 1):
        px, py = sx*POST_X, sy*POST_Y
        d.ellipse([X(px)-5, Y(py)-5, X(px)+5, Y(py)+5], fill=(255,255,255), outline=(90,90,110))

# 标题/图例
d.text((PAD, 16), "body_v5 第二层（高 50）— FPGA 后 / K230 前 / 摄像头前壁 / 层间走线", fill=(25,25,35), font=FT)
d.text((X(L/2)-150, Y(W/2)-34), "前 ▲（摄像头端）", fill=(200,40,40), font=FB)
d.text((X(-L/2)+12, Y(-W/2)+12), "后（FPGA 端）", fill=(90,90,100), font=FX)

ly = CH - 40
d.text((PAD, ly), "白块=层间走线孔    深蓝方块=四角限位柱    白色小圆=下层 φ3.4 固定桩过孔", fill=(70,70,80), font=FX)
img.save(OUT)
print('saved', OUT, img.size)
