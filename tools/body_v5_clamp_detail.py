#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""边夹剖面示意：说明 lift=2mm 的作用（底部焊点/排针过得去）。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_clamp.png'

FLOOR, T, GAP, LIFT = 3.0, 1.6, 0.3, 2.0
POST    = LIFT + T + GAP + T        # 5.5  -> 竖柱高
CLAW_Z  = FLOOR + LIFT + T + GAP    # 6.9  -> 爪底（全局 z）

CW, CH = 1240, 730
img = Image.new('RGB', (CW, CH), (250, 250, 252))
d = ImageDraw.Draw(img)

def font(sz, b=False):
    for p in ([r"C:\Windows\Fonts\msyhbd.ttc"] if b else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F, FB, FT, FX = font(15), font(16, True), font(23, True), font(13)

ZS = 500.0 / 10.0     # z 轴：0..10mm -> 500px

def make_xf(ox):
    def X(x): return ox + 92 + x*13.0
    def SZ(z): return 615 - z*ZS
    return X, SZ

PANELS = [
    ("情况 A：板底有 2mm 焊点（被抬起）", 5.0, True),
    ("情况 B：板底是平的（不抬空）",       3.0, False),
]

for pi, (title, bz, raised) in enumerate(PANELS):
    ox = 30 + pi*600
    X, SZ = make_xf(ox)

    # 底板 z=0..3
    d.rectangle([X(0), SZ(FLOOR), X(34), SZ(0)], fill=(122, 142, 192), outline=(60,70,100), width=2)
    d.text((X(0.6), SZ(1.3)), "底板 3mm", fill=(255,255,255), font=FX)

    # 边夹：竖柱（x 0..4，z 3..8.5）
    d.rectangle([X(0), SZ(FLOOR+POST), X(4), SZ(FLOOR)], fill=(72, 96, 162), outline=(38,52,96), width=2)
    # 边夹：爪（x 0..9，z 6.9..8.5）
    d.rectangle([X(0), SZ(FLOOR+POST), X(9), SZ(CLAW_Z)], fill=(72, 96, 162), outline=(38,52,96), width=2)

    # 模块 PCB
    d.rectangle([X(4), SZ(bz+T), X(31), SZ(bz)], fill=(112, 190, 142), outline=(48,108,78), width=2)
    d.text((X(8), SZ(bz+T/2)-9), "模块 PCB 1.6", fill=(18,52,34), font=FX)

    # 底部焊点
    if raised:
        for px in (7, 11, 15, 19, 23, 27):
            d.ellipse([X(px)-6, SZ(5.0)-5, X(px)+6, SZ(4.4)+5], fill=(214, 142, 42), outline=(150,90,20))
        d.text((X(6), SZ(4.1)+2), "底部焊点 ≈2mm", fill=(158,94,14), font=FX)
        # lift 标注
        d.line([(X(33.6), SZ(bz)), (X(33.6), SZ(FLOOR))], fill=(150, 60, 170), width=3)
        for zz in (bz, FLOOR):
            d.line([(X(33.2), SZ(zz)), (X(34.0), SZ(zz))], fill=(150,60,170), width=2)
        d.text((X(30.0), SZ((bz+FLOOR)/2)-8), "lift 2", fill=(150,60,170), font=FX)

    # 爪底与板顶的间隙
    gd = bz + T
    d.line([(X(10), SZ(gd)), (X(26), SZ(gd))], fill=(205, 42, 42), width=2)
    d.text((X(26.6), SZ(gd)-7), f"间隙 {CLAW_Z-gd:.1f}", fill=(205,42,42), font=FX)

    # 高度标尺
    for zz, lbl in [(FLOOR, "底面 z=3"), (CLAW_Z, "爪底 6.9"), (FLOOR+POST, "爪顶 8.5")]:
        d.line([(X(0)-30, SZ(zz)), (X(0)-16, SZ(zz))], fill=(95,95,105), width=2)
        d.text((X(0)-125, SZ(zz)-8), lbl, fill=(95,95,105), font=FX)

    d.text((ox+92, 46), title, fill=(25,25,35), font=FB)

d.text((30, 8), "边夹剖面：lift=2mm 给「板底焊点/排针」留的高度", fill=(25,25,35), font=FT)
d.text((30, CH-74),
       "爪底固定在 z=6.9（底板 z=3 起算）：板被 2mm 焊点抬起 → 板顶 6.6 → 爪正好压住（间隙 0.3）；",
       fill=(68,68,78), font=FX)
d.text((30, CH-52),
       "板平放 → 板顶 4.6 → 爪高 2.3 压不到，但竖柱仍挡横向；想压紧平放板就把 lift 改小（如 0.3）。",
       fill=(68,68,78), font=FX)
d.text((30, CH-30),
       "参数在 mech/body_v5.scad 的 board_clamp(lift=2.0)。", fill=(68,68,78), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
