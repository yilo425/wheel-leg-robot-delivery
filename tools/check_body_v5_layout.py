#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 body_v5 下层模块布局：两两重叠 + 是否落在内腔 + 是否撞舵机穿墙区。"""
import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

L, W, WALL = 242, 150, 3
IX = L/2 - WALL      # 122
IY = W/2 - WALL      # 72
SV_X, SV_HW = 85, 42
SV_Y0 = W/2 - 42.9   # 32.1 碰撞开口内侧

# (name, cx, cy, X, Y)
MODS = [
    ("电池",      -68.5,  0,   88,  61),
    ("12->5降压", -40,   49,   43,  21),
    ("舵机板",     20,   48, 62.3, 25.5),
    ("电机板",     21,   -2,   50,  50),
    ("端子排",     15,  -67,   76,   8),
    ("保险丝",     20,  -45,   30,  18),
    ("12->9降压",  83,    0,   43,  21),
]
# 四角柱位置（应与 body_v5.scad 一致）
PINS = [
    ("电机板柱", 10, -30, 44, 44),
    ("舵机板柱", 20,  45, 56, 19),
    ("12-5柱",  -40,  49, 30, 15),
    ("12-9柱",   85,   0, 30, 15),
]
PART = []
P_T, P_Y = 2.5, 30

def rect(m):
    _, cx, cy, sx, sy = m
    return (cx-sx/2, cx+sx/2, cy-sy/2, cy+sy/2)

def ov(a, b, tol=0.0):
    ax0, ax1, ay0, ay1 = a; bx0, bx1, by0, by1 = b
    dx = min(ax1, bx1) - max(ax0, bx0)
    dy = min(ay1, by1) - max(ay0, by0)
    return dx > tol and dy > tol, dx, dy

print("=== 模块两两重叠 ===")
bad = 0
for i in range(len(MODS)):
    for j in range(i+1, len(MODS)):
        hit, dx, dy = ov(rect(MODS[i]), rect(MODS[j]))
        if hit:
            print(f"  !! {MODS[i][0]} x {MODS[j][0]}  重叠 {dx:.1f} x {dy:.1f} mm")
            bad += 1
if not bad: print("  OK 无重叠")

print("=== 是否在内腔 ===")
for m in MODS:
    x0, x1, y0, y1 = rect(m)
    ok = x0 >= -IX and x1 <= IX and y0 >= -IY and y1 <= IY
    print(f"  {'OK ' if ok else '!! '}{m[0]:10s} x[{x0:6.1f},{x1:6.1f}] y[{y0:6.1f},{y1:6.1f}]")
    if not ok: bad += 1

print("=== 是否撞舵机穿墙区 (x in [64,106] or [-106,-64]) 且 y 越过 32.1 ===")
for m in MODS:
    x0, x1, y0, y1 = rect(m)
    for s in (-1, 1):
        sx0, sx1 = (s*SV_X - SV_HW/2), (s*SV_X + SV_HW/2)
        hit = (min(x1, max(sx0, sx1)) - max(x0, min(sx0, sx1)) > 0) if s == 1 else \
              (min(x1, -64) - max(x0, -106) > 0)
        if s == 1:
            hit = x1 > 64 and x0 < 106
        else:
            hit = x1 > -106 and x0 < -64
        if hit and (y1 > 32.1 or y0 < -32.1):
            # 精确判断是否同时落在 y 的舵机带
            yin = (y1 > 32.1) or (y0 < -32.1)
            if yin:
                print(f"  !! {m[0]:10s} 与舵机区 s={s} 相交  x[{x0:.1f},{x1:.1f}] y[{y0:.1f},{y1:.1f}]")
                bad += 1

print("=== 分区墙 vs 模块 ===")
for pn, px in PART:
    pr = (px-P_T/2, px+P_T/2, -P_Y, P_Y)
    for m in MODS:
        hit, dx, dy = ov(pr, rect(m))
        if hit:
            print(f"  !! {pn}(x={px}) 撞 {m[0]}  重叠 {dx:.2f} x {dy:.2f}")
            bad += 1
print(f"\n结果：{'全部通过 OK' if bad == 0 else f'{bad} 处问题 !!'}")
