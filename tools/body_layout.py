#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
body_layout.py —— 车身两层平面布局图（俯视）

不追求精确到 0.5mm，目的是把"所有件放哪、怎么固定"画出来核对。
改下面的坐标就能重排。
"""
import os
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_layout.png'

# ---------------- 车身 ----------------
L, W = 225, 125          # 外形（缩小后）
WALL, H1, H2 = 3, 45, 50
IN_X = L/2 - WALL        # 内腔半长 = 109.5
IN_Y = W/2 - WALL        # 内腔半宽 = 59.5
SV_X = 85                # 舵机前后位置
SV_Y0, SV_Y1 = 32, 72    # 舵机本体在内腔占的 y 范围（穿墙式）

# ---------------- 模块清单（中心 x, y, 长X, 宽Y, 名称, 颜色, 层）----------------
MODS_LOW = [
    (-55,   0, 88, 61, "电池 88x61",            (255, 210, 150), 'bat'),
    ( 10, -30, 50, 50, "电机驱动板 50x50",       (200, 180, 240), 'drv'),
    ( 62,  20, 40, 20, "5.5V降压 (舵机)",        (180, 230, 180), 'buck'),
    (-13,  35, 31, 12, "端子排 6位 GND",         (170, 200, 230), 'term'),
    (-13,  20, 26, 12, "端子排 5位 舵机5V",      (170, 200, 230), 'term'),
    (-13,   5, 21, 12, "端子排 4位 12V",         (170, 200, 230), 'term'),
    (-95, -40, 14, 12, "挡件",                   (140, 140, 150), 'stop'),
    (-95,  40, 14, 12, "挡件",                   (140, 140, 150), 'stop'),
    ( 95, -40, 14, 12, "挡件",                   (140, 140, 150), 'stop'),
    ( 95,  40, 14, 12, "挡件",                   (140, 140, 150), 'stop'),
]
MODS_UP = [
    (-34,   0, 140, 92, "FPGA 140x92",           (200, 220, 245), 'fpga'),
    ( 78, -12, 56, 85, "K230 56x85(转90°)",      (255, 200, 200), 'k230'),
    (-15,  47, 40, 18, "9V降压 (FPGA)",          (180, 230, 180), 'buck'),
    ( 45, -47, 26, 16, "I2C/UART 线束区",        (230, 230, 230), 'wire'),
]

SC = 5.2
PADX, PADY = 60, 60
PANEL_W = int(L * SC) + PADX * 2
PANEL_H = int(W * SC) + PADY * 2 + 60
CANVAS = Image.new('RGB', (PANEL_W * 2 + 40, PANEL_H), (252, 252, 254))

def font(sz, bold=False):
    for c in ([r"C:\Windows\Fonts\msyhbd.ttc"] if bold else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try:
            return ImageFont.truetype(c, sz)
        except Exception:
            pass
    return ImageFont.load_default()

F  = font(13)
FB = font(16, True)
FT = font(21, True)

def draw_layer(canvas, ox, title, mods, hcolor):
    dr = ImageDraw.Draw(canvas)
    def X(x): return int(ox + PADX + (x + L/2) * SC)
    def Y(y): return int(PADY + 46 + (W/2 - y) * SC)
    # 车身外形
    dr.rectangle([X(-L/2), Y(W/2), X(L/2), Y(-W/2)], outline=(60, 60, 70), width=3)
    dr.rectangle([X(-IN_X), Y(IN_Y), X(IN_X), Y(-IN_Y)], outline=(150, 150, 160), width=1)
    # 舵机占位（四处）
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sx*SV_X - 20.4, sx*SV_X + 20.4
            y0, y1 = (sy*SV_Y0 if sy > 0 else -SV_Y1), (sy*SV_Y1 if sy > 0 else -SV_Y0)
            dr.rectangle([X(x0), Y(y1), X(x1), Y(y0)], fill=(235, 225, 225), outline=(190, 120, 120))
            dr.text((X((x0+x1)/2)-18, Y((y0+y1)/2)-6), "舵机", fill=(170, 90, 90), font=F)
    # 模块
    for (cx, cy, w, h, name, col, tag) in mods:
        x0, x1 = cx - w/2, cx + w/2
        y0, y1 = cy - h/2, cy + h/2
        dr.rectangle([X(x0), Y(y1), X(x1), Y(y0)], fill=col, outline=(70, 70, 90), width=2)
        dr.text((X(x0)+4, Y(y1)+4), name, fill=(30, 30, 40), font=F)
    dr.text((ox + PADX, 14), title, fill=(0, 0, 0), font=FT)
    dr.text((ox + PADX, PADY + 24), "前 ▲", fill=(200, 0, 0), font=FB)

draw_layer(CANVAS, 0, "下层（高 45）", MODS_LOW, None)
draw_layer(CANVAS, PANEL_W + 40, "上层（高 50）", MODS_UP, None)

# 右下角备注
dr = ImageDraw.Draw(CANVAS)
ty = CANVAS.size[1] - 52
dr.text((20, ty), "蓝色=端子排  绿色=降压  紫色=驱动板  粉色=K230  灰色=挡件", fill=(90, 90, 90), font=F)
dr.text((20, ty + 22), f"外形 {L}×{W}mm  舵机 ±{SV_X}mm 穿墙  内腔 {IN_X*2:.0f}×{IN_Y*2:.0f}", fill=(90, 90, 90), font=F)

CANVAS.save(OUT)
print('已生成:', OUT, CANVAS.size)
