#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_stl.py —— 把 STL 渲成 PNG（正交投影 + 平面着色 + z-buffer）

用途：不用装任何 CAD/查看器，直接看零件的 3D 样子。
输出 mech/leg_3d.png（三视图拼一张：等轴测 / 俯视 / 侧视）
"""
import os
import struct
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL = os.path.join(ROOT, 'mech', 'leg_70mm.stl')
OUT = os.path.join(ROOT, 'mech', 'leg_3d.png')

# ---------------- 读二进制 STL ----------------
with open(STL, 'rb') as f:
    f.read(80)
    n = struct.unpack('<I', f.read(4))[0]
    d = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
tri = d[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)
print('三角形', n)

# ---------------- 光照 & 着色 ----------------
def normals(t):
    e1 = t[:, 1] - t[:, 0]
    e2 = t[:, 2] - t[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    return nrm / np.where(ln < 1e-12, 1, ln)

def render(R, W=520, Hh=400, bg=(252, 252, 254), margin=18):
    """R: 3x3 旋转矩阵（世界 -> 相机）"""
    t = tri.reshape(-1, 3)
    p = t @ R.T                      # 旋转
    p = p.reshape(-1, 3, 3)
    # 正交投影：屏幕 x = cam.x, y = -cam.y（图像 y 向下）；深度 = cam.z
    us = p[:, :, 0]
    vs = -p[:, :, 1]
    zs = p[:, :, 2]
    umin, umax = us.min(), us.max()
    vmin, vmax = vs.min(), vs.max()
    sc = min((W - 2*margin) / (umax - umin + 1e-9),
             (Hh - 2*margin) / (vmax - vmin + 1e-9))
    ox = margin - umin * sc + ((W - 2*margin) - (umax-umin)*sc) / 2
    oy = margin - vmin * sc + ((Hh - 2*margin) - (vmax-vmin)*sc) / 2
    sx = us * sc + ox
    sy = vs * sc + oy

    img = np.full((Hh, W, 3), bg, dtype=np.float64)
    zbuf = np.full((Hh, W), -1e18)

    nrm = normals(tri.reshape(-1, 3, 3))
    # 相机空间法线
    nrm_c = nrm @ R.T
    light = np.array([0.35, 0.45, 0.82])
    light = light / np.linalg.norm(light)
    lam = np.clip(nrm_c @ light, 0, 1)
    base = np.array([120, 165, 225]) / 255.0
    shade = 0.35 + 0.65 * lam

    for i in range(len(p)):
        x0, x1 = int(np.floor(sx[i].min())), int(np.ceil(sx[i].max()))
        y0, y1 = int(np.floor(sy[i].min())), int(np.ceil(sy[i].max()))
        x0 = max(x0, 0); y0 = max(y0, 0)
        x1 = min(x1, W - 1); y1 = min(y1, Hh - 1)
        if x1 < x0 or y1 < y0:
            continue
        xs_, ys_ = np.meshgrid(np.arange(x0, x1+1), np.arange(y0, y1+1))
        ax, ay = sx[i, 0], sy[i, 0]
        bx, by = sx[i, 1], sy[i, 1]
        cx, cy = sx[i, 2], sy[i, 2]
        den = (by - cy)*(ax - cx) + (cx - bx)*(ay - cy)
        if abs(den) < 1e-12:
            continue
        w0 = ((by - cy)*(xs_ - cx) + (cx - bx)*(ys_ - cy)) / den
        w1 = ((cy - ay)*(xs_ - cx) + (ax - cx)*(ys_ - cy)) / den
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
        if not m.any():
            continue
        z = w0*zs[i, 0] + w1*zs[i, 1] + w2*zs[i, 2]
        sub_z = zbuf[y0:y1+1, x0:x1+1]
        better = m & (z > sub_z)
        if not better.any():
            continue
        sub_z[better] = z[better]
        col = np.clip(base * shade[i], 0, 1) * 255
        sub_img = img[y0:y1+1, x0:x1+1]
        sub_img[better] = col
    return Image.fromarray(img.astype(np.uint8))

# ---------------- 三个视角 ----------------
def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == 'x': return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    if axis == 'y': return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

views = []
# ① 等轴测（从斜上前方看）
R1 = rot('x', -62) @ rot('z', -38)
views.append((R1, u"1) isometric  (from front-upper-left)"))
# ② 俯视（看六芒星凹槽 / 电机孔）
R2 = np.eye(3)
views.append((R2, u"2) top view  (star pocket + motor holes)"))
# ③ 侧视（看根部 11mm 台阶 + 挡件平台 45°）
R3 = np.array([[1,0,0],[0,1,0],[0,0,-1]])   # 沿 Z 镜像 = 从底面看
views.append((R3, u"3) bottom view  (servo horn pocket - 6 round arms)"))

imgs = [render(R) for (R, _) in views]

W, Hh = imgs[0].size
PAD = 26
canvas = Image.new('RGB', (W * 3 + PAD * 4, Hh + PAD * 2 + 34), (255, 255, 255))
try:
    font = ImageFont.truetype(r"C:\Windows\Fonts\consola.ttf", 16)
except Exception:
    font = ImageFont.load_default()
dr = ImageDraw.Draw(canvas)
for i, im in enumerate(imgs):
    x = PAD + i * (W + PAD)
    canvas.paste(im, (x, PAD + 26))
    dr.rectangle([x-1, PAD+25, x+W, PAD+26+Hh], outline=(200, 200, 205))
    dr.text((x, 6), views[i][1], fill=(30, 30, 30), font=font)

canvas.save(OUT)
print('已生成:', OUT, canvas.size)
