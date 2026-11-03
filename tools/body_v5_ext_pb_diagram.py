#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""body_v5 外挂充电宝示意：左=侧视图(X-Z)，右=第二层俯视(X-Y)。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
from PIL import Image, ImageDraw, ImageFont

OUT = r'D:\dsh-ws\robot_project\mech\body_v5_ext_pb.png'

L, W, H1, H2 = 236, 150, 45, 50
HT = H1 + H2
EXT = 30           # 托兜伸出
PB_T, PB_W, PB_H = 27, 79, 35
PB_Z0 = 50         # 充电宝底面高
FPG = (-40, 0, 150, 109)
K3 = (84, 0, 56, 85)
CAM_D = 8

CW, CH = 1760, 900
img = Image.new('RGB', (CW, CH), (252, 252, 254))
d = ImageDraw.Draw(img)

def font(sz, b=False):
    for p in ([r"C:\Windows\Fonts\msyhbd.ttc"] if b else []) + [r"C:\Windows\Fonts\msyh.ttc"]:
        try: return ImageFont.truetype(p, sz)
        except Exception: pass
    return ImageFont.load_default()
F, FB, FT, FX = font(15), font(17, True), font(24, True), font(13)

# ================= 面板 1：侧视图 =================
P1X, P1Y, SC1 = 60, 170, 2.5
def SX(x): return P1X + (x + L/2 + EXT + 4) * SC1
def SZ(z): return P1Y + (HT - z) * SC1
def SB(xa, xb, za, zb):
    return [SX(min(xa, xb)), SZ(max(za, zb)), SX(max(xa, xb)), SZ(min(za, zb))]

d.rectangle(SB(-L/2, L/2, 0, HT), fill=(226, 232, 240), outline=(40, 50, 70), width=3)
d.line([(SX(-L/2), SZ(H1)), (SX(L/2), SZ(H1))], fill=(130, 140, 160), width=2)
d.text((SX(-L/2) + 8, SZ(H1) - 22), "下层 45", fill=(90, 95, 110), font=FX)
d.text((SX(-L/2) + 8, SZ(HT) + 6), "上层 50", fill=(90, 95, 110), font=FX)

# 前壁外侧：传感器孔位
d.line([(SX(L/2), SZ(18.5)), (SX(L/2) + 16, SZ(18.5))], fill=(60, 140, 70), width=4)
d.text((SX(L/2) + 20, SZ(18.5) - 9), "超声波孔", fill=(60, 140, 70), font=FX)
d.line([(SX(L/2), SZ(33)), (SX(L/2) + 16, SZ(33))], fill=(205, 125, 40), width=4)
d.text((SX(L/2) + 20, SZ(33) - 22), "摄像孔", fill=(205, 125, 40), font=FX)

# 托兜（前壁上方）
d.rectangle(SB(L/2, L/2 + EXT, PB_Z0 - 3, PB_Z0), fill=(96, 110, 140), outline=(50, 60, 85))
d.rectangle(SB(L/2 + EXT - 2, L/2 + EXT, PB_Z0 - 3, PB_Z0 + PB_H + 3), fill=(96, 110, 140), outline=(50, 60, 85))
d.rectangle(SB(L/2, L/2 + 3, PB_Z0, PB_Z0 + PB_H), fill=(96, 110, 140), outline=(50, 60, 85))
# 充电宝
d.rectangle(SB(L/2 + 1, L/2 + 1 + PB_T, PB_Z0, PB_Z0 + PB_H),
            fill=(235, 235, 242), outline=(120, 120, 135), width=3)
d.text((SX(L/2 + 34), SZ(PB_Z0 + PB_H) + 8), "充电宝 27×79×35", fill=(70, 70, 85), font=FB)
d.text((SX(L/2 + 34), SZ(PB_Z0 + PB_H) + 32), "（前壁上方 z 50~85）", fill=(90, 90, 105), font=FX)

# 电池（后排内侧）
d.rectangle(SB(-L/2 + 3, -L/2 + 3 + 91, 0, 25), fill=(214, 96, 80), outline=(150, 60, 50), width=2)
d.text((SX(-L/2) + 20, SZ(12) - 9), "锂电池 88×61（后排）", fill=(255, 255, 255), font=F)

# 尺寸
d.line([(SX(-L/2), SZ(-8)), (SX(L/2), SZ(-8))], fill=(200, 60, 60), width=2)
d.text((SX(-20), SZ(-8) - 22), f"车身 {L}", fill=(200, 60, 60), font=FX)
d.line([(SX(L/2), SZ(92)), (SX(L/2 + EXT), SZ(92))], fill=(200, 60, 60), width=2)
d.text((SX(L/2 + 4), SZ(92) - 22), "伸出 30", fill=(200, 60, 60), font=FX)

d.text((P1X, 40), "① 侧视图（沿 Y 看）", fill=(25, 25, 35), font=FT)
d.text((P1X, 76), "锂电池在后排、充电宝在前上方 —— 对边放，重心回到中间",
       fill=(25, 25, 35), font=F)
d.text((P1X, 100), f"车身 {L}×150×{HT}mm   伸出后总长 {L + EXT}mm（打印件仍是 {L}）",
       fill=(80, 80, 95), font=FX)

# ================= 面板 2：第二层俯视 =================
P2X, P2Y, SC2 = 960, 200, 2.75
def TX(x): return P2X + (x + L/2) * SC2
def TY(y): return P2Y + (W/2 - y) * SC2
def TB(xa, xb, ya, yb):
    return [TX(min(xa, xb)), TY(max(ya, yb)), TX(max(xa, xb)), TY(min(ya, yb))]

d.rounded_rectangle([TX(-L/2), TY(W/2), TX(L/2), TY(-W/2)], radius=8 * SC2,
                    fill=(236, 240, 246), outline=(40, 50, 70), width=3)

# 外挂充电宝（前壁外，示意）
d.rectangle(TB(L/2, L/2 + EXT, PB_W/2, -PB_W/2), fill=(228, 228, 236), outline=(120, 120, 140), width=2)
d.text((TX(L/2) + 8, TY(0) - 8), "充电宝", fill=(90, 90, 110), font=FX)
d.text((TX(L/2) + 8, TY(0) + 12), "（前壁外）", fill=(90, 90, 110), font=FX)

# FPGA
f0, f1, fw, fh = FPG
d.rectangle(TB(f0 - fw/2, f0 + fw/2, f1 + fh/2, f1 - fh/2), fill=(64, 96, 165), outline=(30, 45, 90), width=2)
d.text((TX(f0) - 62, TY(f1) - 12), "FPGA 150×109", fill=(255, 255, 255), font=FB)
d.text((TX(f0) - 62, TY(f1) + 10), "（自带升高支架）", fill=(210, 220, 240), font=FX)

# K230
k0, k1, kw, kh = K3
d.rectangle(TB(k0 - kw/2, k0 + kw/2, k1 + kh/2, k1 - kh/2), fill=(196, 84, 96), outline=(120, 40, 50), width=2)
d.text((TX(k0) - 34, TY(k1) - 6), "K230 56×85", fill=(255, 255, 255), font=F)
for sx in (-1, 1):
    for sy in (-1, 1):
        px, py = k0 + sx*(kw/2 + 4.4), k1 + sy*(kh/2 + 4.4)
        d.rectangle([TX(px) - 4, TY(py) - 4, TX(px) + 4, TY(py) + 4], fill=(70, 95, 160))

# 板间：DC 弯头 + Type-C 走线
d.rectangle(TB(35, 49, -22, -38), fill=(120, 190, 150), outline=(60, 120, 90), width=2)
d.text((TX(35) - 40, TY(-38) + 26), "DC弯头 14mm", fill=(20, 90, 60), font=FX)
d.line([(TX(-118), TY(62)), (TX(56), TY(62))], fill=(120, 120, 135), width=3)
d.line([(TX(56), TY(62)), (TX(56), TY(0))], fill=(120, 120, 135), width=3)
d.text((TX(-60), TY(66) - 18), "Type-C 软线（沿 +Y 侧走）", fill=(120, 120, 135), font=FX)

# 摄像头（抬高，在 K230 上方）
d.rectangle(TB(L/2 - 3 - CAM_D, L/2 - 3, 13, -13), fill=(238, 158, 60), outline=(150, 90, 20), width=2)

d.text((P2X - 250, 40), "② 第二层俯视", fill=(25, 25, 35), font=FT)
d.text((P2X - 250, 76), "FPGA / K230 / 摄像头，充电宝已挪到车外", fill=(80, 80, 95), font=FX)

d.text((60, CH - 36),
       "灰蓝=外挂托兜（单独打，2×M3 拧前壁）   白=充电宝   红=锂电池   蓝=FPGA   暗红=K230   橙=摄像头",
       fill=(70, 70, 85), font=FX)

img.save(OUT)
print('saved', OUT, img.size)
