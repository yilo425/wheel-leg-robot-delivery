#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""body_v5 第二层方案 v3：充电宝在右侧、K230 转 180°，DC 孔在左侧不被遮挡。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_plan_v3.png'

L, W, WALL = 242, 150, 3
IX, IY = L/2 - WALL, W/2 - WALL          # 115, 72
FPG = (-118, 32, -54.5, 54.5)
JACK = (32, 46, 37, 53)                  # FPGA DC 座 + 弯头（+Y 侧）
PB = (34, 113, -68, -33)                 # 79(X) x 35(Y)，右侧
K3 = (59, 115, -25, 60)                  # 56(X) x 85(Y)，转 180°
CAM = (115, 118, -13, 13)
TC_K = (59, -18)                         # K230 的 Type-C（-X 长边，近 -Y 端）
TC_P = (73.5, -33)                       # 充电宝的 Type-C（+Y 长边中间）

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
OX, OY = 330, 175
def X(x): return OX + (x + L/2) * SC
def Y(y): return OY + (W/2 - y) * SC
def B(x0, x1, y0, y1):
    return [X(min(x0, x1)), Y(max(y0, y1)), X(max(x0, x1)), Y(min(y0, y1))]

d.rounded_rectangle(B(-L/2, L/2, -W/2, W/2), radius=8*SC, fill=(238, 242, 248),
                    outline=(40, 50, 70), width=4)
d.rounded_rectangle(B(-IX, IX, -IY, IY), radius=5*SC, outline=(160, 170, 190), width=2)

d.text((X(0) + 210, 118), "▲ 前 (+X)，摄像头端", fill=(200, 60, 60), font=FB)
d.text((X(-L/2) - 66, Y(-W/2) + 30), "后 (−X)，FPGA 端", fill=(120, 120, 135), font=FX)
d.text((X(0) - 130, Y(W/2) - 46), "← +Y（车左）", fill=(120, 120, 135), font=FX)
d.text((X(0) - 130, Y(-W/2) + 26), "← −Y（车右）", fill=(120, 120, 135), font=FX)

# FPGA
d.rectangle(B(*FPG), fill=(64, 96, 165), outline=(30, 45, 90), width=3)
d.text((X(-60) - 100, Y(-6) - 26), "FPGA", fill=(255, 255, 255), font=FT)
d.text((X(-60) - 100, Y(-6) + 8), "150（X） × 109（Y）", fill=(220, 230, 250), font=F)
d.text((X(-60) - 100, Y(-6) + 32), "自带升高支架 → 不做支撑", fill=(200, 215, 240), font=FX)
d.text((X(-60) - 100, Y(-6) + 56), "DC 座在前边靠 +Y（上）", fill=(255, 220, 150), font=FX)

# FPGA DC 座 + 弯头
d.rectangle(B(*JACK), fill=(215, 80, 70), outline=(140, 40, 35), width=3)
d.text((X(JACK[0]) - 116, Y(JACK[3]) - 46), "FPGA\nDC 座", fill=(180, 50, 40), font=FX, spacing=2)

# 充电宝
d.rectangle(B(*PB), fill=(232, 232, 240), outline=(110, 110, 130), width=3)
d.text((X(60), Y(-47) - 12), "充电宝", fill=(40, 40, 60), font=FB)
d.text((X(60), Y(-47) + 12), "79（X）× 35（Y）", fill=(80, 80, 100), font=FX)
d.text((X(60), Y(-47) + 32), "长边贴前壁，口朝 +X", fill=(80, 80, 100), font=FX)

# K230
d.rectangle(B(*K3), fill=(198, 86, 98), outline=(130, 40, 52), width=3)
d.text((X(84) - 56, Y(20) - 14), "K230（转 180°）", fill=(255, 255, 255), font=FB)
d.text((X(84) - 56, Y(20) + 12), "56（X）× 85（Y）", fill=(255, 235, 235), font=FX)
for sy in (-1, 1):
    for sx in (-1, 1):
        px = 84 + sx*(28 + 4.4); py = 12.5 + sy*(42.5 + 4.4)
        d.rectangle(B(px-2, px+2, py-2, py+2), fill=(70, 95, 160), outline=(35, 50, 95))

# 摄像头
d.rectangle(B(*CAM), fill=(240, 165, 70), outline=(150, 90, 20), width=3)

# Type-C 软线
d.line([(X(TC_K[0]), Y(TC_K[1])), (X(62), Y(-30)), (X(TC_P[0]), Y(TC_P[1]))],
       fill=(90, 90, 115), width=6)
d.ellipse([X(TC_K[0])-7, Y(TC_K[1])-7, X(TC_K[0])+7, Y(TC_K[1])+7], fill=(40, 40, 60))
d.ellipse([X(TC_P[0])-7, Y(TC_P[1])-7, X(TC_P[0])+7, Y(TC_P[1])+7], fill=(40, 40, 60))
d.text((X(-104), Y(-64)), "Type-C：K230 侧口 → 充电宝口，直线距离 ≈ 23mm", fill=(80, 80, 105), font=FX)

# 固定桩
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = sx*(L/2 - 7), sy*68
        d.ellipse([X(cx)-8, Y(cy)-8, X(cx)+8, Y(cy)+8], fill=(255,255,255),
                  outline=(90, 90, 110), width=2)

# 尺寸
d.line([(X(-L/2), Y(-W/2)+66), (X(L/2), Y(-W/2)+66)], fill=(200, 60, 60), width=2)
d.text((X(-16), Y(-W/2)+72), f"L = {L}", fill=(200, 60, 60), font=FB)
d.line([(X(-L/2)-54, Y(-W/2)), (X(-L/2)-54, Y(W/2))], fill=(200, 60, 60), width=2)
d.text((X(-L/2)-150, Y(0)-10), f"W = {W}", fill=(200, 60, 60), font=FB)

d.text((40, 44), "body_v5 第二层方案 v3 —— 充电宝放右侧，不挡 FPGA 的 DC 座", fill=(25, 25, 35), font=FT)
d.text((40, 86), "充电宝 79 沿车长 / 35 跨车宽；K230 转 180° 让 Type-C 朝充电宝", fill=(60, 60, 80), font=F)

tx = X(L/2) + 56
d.text((tx, 176), "尺寸核对", fill=(25, 25, 35), font=FB)
rows = [
    ("内腔", "236 × 144"),
    ("FPGA", "150 × 109，x −115…35"),
    ("DC 座+弯头", "x 35…49，y +37…+53（+Y 侧）"),
    ("充电宝", "x 36…115（79），y −72…−37（−Y 侧）"),
    ("K230", "x 56…112（56），y −30…+55"),
    ("X 合计", "150 + 79 = 229 ≤ 236 ✓"),
    ("Y 合计", "35 + 85 = 120 ≤ 144 ✓"),
    ("L / W", f"{L} / {W}（都不变）"),
]
for i, (k, v) in enumerate(rows):
    yy = 216 + i*40
    d.text((tx, yy), k, fill=(90, 90, 110), font=FX)
    d.text((tx + 110, yy), v, fill=(30, 30, 40), font=FX)

d.text((tx, 566), "为什么这样排", fill=(25, 25, 35), font=FB)
notes = [
    "① FPGA 的 DC 座和 K230 的 Type-C",
    "   都在 +Y 侧（照片左侧）。",
    "② 充电宝放到 −Y 侧，就不挡 DC 座。",
    "③ K230 转 180°，Type-C 从 +Y 角",
    "   换到 −Y 角，正好对着充电宝。",
    "④ 两口距离 ≈ 23mm，短线够用。",
    "⑤ X/Y 都没超，L=236 不变。",
]
for i, t in enumerate(notes):
    d.text((tx, 602 + i*26), t, fill=(60, 60, 80), font=FX)

d.text((tx, 812), "要确认", fill=(25, 25, 35), font=FB)
d.text((tx, 846), "K230 转 180° 后，摄像头排线", fill=(200, 60, 60), font=FX)
d.text((tx, 866), "和 HDMI/USB 会不会被挡住？", fill=(200, 60, 60), font=FX)
d.text((tx, 890), "（K230 的排线在它的一条短边上）", fill=(90, 90, 110), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
