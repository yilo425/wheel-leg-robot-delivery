#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_leg_stl.py —— 直接生成腿零件的 STL（不需要 OpenSCAD）

体素化 0.3mm -> 只输出"实心/空"交界面上的四边形 -> 写二进制 STL。
天然水密；曲面有 0.3mm 台阶（FDM 0.2mm 层高，实际看不出来）。

⚠️ 六芒星凹槽用**标称尺寸**（不做 0.15 过盈，体素量化会吃掉它）：
   松了 -> 滴 502；紧了 -> 砂纸磨。
"""
import os
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MECH = os.path.join(ROOT, 'mech')
os.makedirs(MECH, exist_ok=True)

L, TH, ROOT_TH = 70.0, 8.0, 11.0
ROOT_R, ROOT_SEG = 19.5, 14.0
STAR_RO, STAR_RI, STAR_DEP = 16.0, 8.0, 2.1
CENTER_D, CSNK_D, CSNK_DEP = 3.5, 8.0, 4.0
MOT_BOLT_SQ, MOT_BOLT_D, MOT_BOSS_D = 12.0, 3.2, 9.0
WORK_ANG, PAD_R, PAD_L, PAD_H, PAD_SURF_Y = 45.0, 25.0, 12.0, 4.0, 12.3
CTRL = [(0.0, ROOT_R), (22.0, 12.5), (45.0, 10.0), (62.0, 10.5), (72.0, 10.5)]
VOX = 0.3

xs = np.arange(-22.0, 86.0, VOX)
ys = np.arange(-22.0, 22.0, VOX)
zs = np.arange(0.0, 12.0, VOX)
X, Y = np.meshgrid(xs, ys, indexing='ij')

def half_h(x):
    if x < 0:
        return float(np.sqrt(max(0.0, ROOT_R**2 - x*x)))
    if x > CTRL[-1][0]:
        d = x - CTRL[-1][0]
        return float(np.sqrt(max(0.0, CTRL[-1][1]**2 - d*d)))
    for i in range(len(CTRL) - 1):
        x0, r0 = CTRL[i]; x1, r1 = CTRL[i + 1]
        if x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return r0 + (r1 - r0) * t
    return CTRL[-1][1]

def boss_h(x):
    if x < 0:
        return float(np.sqrt(max(0.0, ROOT_R**2 - x*x)))
    if x > ROOT_SEG:
        return 0.0
    return ROOT_R + (12.0 - ROOT_R) * (x / ROOT_SEG)

H  = np.vectorize(half_h)(xs)[:, None]
HB = np.vectorize(boss_h)(xs)[:, None]
in_body = np.abs(Y) <= H
in_boss = np.abs(Y) <= HB

a = np.radians(WORK_ANG)
dx = X - PAD_R
dy = Y - (PAD_SURF_Y - 2.0)
u =  dx * np.cos(a) + dy * np.sin(a)
v = -dx * np.sin(a) + dy * np.cos(a)
vc = np.clip(v, 0, PAD_H + 2)
half_w = (PAD_L/2 + 3.0) + ((PAD_L/2) - (PAD_L/2 + 3.0)) * vc / (PAD_H + 2)
in_pad = (v >= 0) & (v <= PAD_H + 2) & (np.abs(u) <= half_w)

ZZ = zs[None, None, :]
occ = (in_body[:, :, None] & (ZZ < TH)) | \
      (in_boss[:, :, None] & (ZZ < ROOT_TH)) | \
      (in_pad[:, :, None] & (ZZ < TH))

# 六芒星（12 边形射线法）
star = []
for i in range(12):
    ang = np.radians(i * 30.0)
    r = STAR_RO if i % 2 == 0 else STAR_RI
    star.append((r*np.cos(ang), r*np.sin(ang)))
inside = np.zeros(X.shape, dtype=bool)
j = 11
for i in range(12):
    xi, yi = star[i]; xj, yj = star[j]
    cond = ((yi > Y) != (yj > Y)) & (X < (xj - xi) * (Y - yi) / (yj - yi + 1e-12) + xi)
    inside ^= cond
    j = i
occ &= ~(inside[:, :, None] & (ZZ < STAR_DEP))

R2 = X**2 + Y**2
occ &= ~((R2 <= (CENTER_D/2)**2)[:, :, None])
occ &= ~((R2 <= (CSNK_D/2)**2)[:, :, None] & (ZZ >= ROOT_TH - CSNK_DEP))
occ &= ~((((X-L)**2 + Y**2) <= (MOT_BOSS_D/2)**2)[:, :, None])
for sx in (-1, 1):
    for sy in (-1, 1):
        cx, cy = L + sx*MOT_BOLT_SQ/2, sy*MOT_BOLT_SQ/2
        occ &= ~((((X-cx)**2 + (Y-cy)**2) <= (MOT_BOLT_D/2)**2)[:, :, None])

print('网格 %s  占据 %d' % (str(occ.shape), occ.sum()))
vol = float(occ.sum()) * VOX**3
print('体积 %.0f mm3   单条 %.1f g' % (vol, vol*1.27e-3))

# ---------------- 提取表面 ----------------
tris = []

def emit(axis, plane, i, j, k, sign):
    x0, y0, z0 = xs[i], ys[j], zs[k]
    if axis == 0:
        c = [(plane, y0, z0), (plane, y0+VOX, z0), (plane, y0+VOX, z0+VOX), (plane, y0, z0+VOX)]
    elif axis == 1:
        c = [(x0, plane, z0), (x0+VOX, plane, z0), (x0+VOX, plane, z0+VOX), (x0, plane, z0+VOX)]
    else:
        c = [(x0, y0, plane), (x0+VOX, y0, plane), (x0+VOX, y0+VOX, plane), (x0, y0+VOX, plane)]
    if (sign < 0) ^ (axis == 1):
        c = c[::-1]
    tris.append((c[0], c[1], c[2]))
    tris.append((c[0], c[2], c[3]))

nx, ny, nz = len(xs), len(ys), len(zs)

# +X / -X
for i in range(nx - 1):
    d = occ[i] & ~occ[i+1]
    for (j, k) in np.argwhere(d):
        emit(0, xs[i+1], i, int(j), int(k), +1)
    d = occ[i+1] & ~occ[i]
    for (j, k) in np.argwhere(d):
        emit(0, xs[i+1], i, int(j), int(k), -1)
# +Y / -Y
for j in range(ny - 1):
    d = occ[:, j] & ~occ[:, j+1]
    for (i, k) in np.argwhere(d):
        emit(1, ys[j+1], int(i), j, int(k), +1)
    d = occ[:, j+1] & ~occ[:, j]
    for (i, k) in np.argwhere(d):
        emit(1, ys[j+1], int(i), j, int(k), -1)
# +Z / -Z
for k in range(nz - 1):
    d = occ[:, :, k] & ~occ[:, :, k+1]
    for (i, j) in np.argwhere(d):
        emit(2, zs[k+1], int(i), int(j), k, +1)
    d = occ[:, :, k+1] & ~occ[:, :, k]
    for (i, j) in np.argwhere(d):
        emit(2, zs[k+1], int(i), int(j), k, -1)

print('三角形 %d' % len(tris))

out = os.path.join(MECH, 'leg_70mm.stl')
with open(out, 'wb') as f:
    f.write(b'\0' * 80)
    f.write(np.uint32(len(tris)).tobytes())
    for (p0, p1, p2) in tris:
        e1 = np.subtract(p1, p0); e2 = np.subtract(p2, p0)
        n = np.cross(e1, e2)
        ln = np.linalg.norm(n)
        n = n/ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])
        f.write(np.array(n, dtype=np.float32).tobytes())
        for p in (p0, p1, p2):
            f.write(np.array(p, dtype=np.float32).tobytes())
        f.write(b'\0\0')

print('已生成: mech/leg_70mm.stl  (%.1f MB)' % (os.path.getsize(out)/1e6))
