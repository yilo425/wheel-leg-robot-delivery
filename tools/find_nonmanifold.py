#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""找 STL 里非流形边（被 !=2 个三角面共用）的坐标，用来定位几何错误。"""
import sys, io, struct, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else 'mech/body_v5_lower.stl'
with open(path, 'rb') as f:
    head = f.read(84)
    n = struct.unpack('<I', head[80:84])[0]
    data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
tris = data[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)

# 量化到 1e-3，避免浮点误差
q = np.round(tris * 1000.0).astype(np.int64)
edges = collections.Counter()
for t in q:
    for a, b in ((0, 1), (1, 2), (2, 0)):
        e = (tuple(t[a]), tuple(t[b]))
        e = tuple(sorted(e))
        edges[e] += 1
bad = [(e, c) for e, c in edges.items() if c != 2]
print(f"三角形 {n}，边 {len(edges)}，非流形边 {len(bad)}")
if bad:
    pts = []
    for (a, b), c in bad[:400]:
        pts.append(a); pts.append(b)
    pts = np.array(pts) / 1000.0
    print("  非流形边坐标范围:")
    print(f"    x {pts[:,0].min():8.2f} .. {pts[:,0].max():8.2f}")
    print(f"    y {pts[:,1].min():8.2f} .. {pts[:,1].max():8.2f}")
    print(f"    z {pts[:,2].min():8.2f} .. {pts[:,2].max():8.2f}")
    # 聚类：按 20mm 网格统计热点
    cell = collections.Counter()
    for p in pts:
        cell[(int(p[0]//20)*20, int(p[1]//20)*20, int(p[2]//20)*20)] += 1
    print("  热点（按 20mm 格）：")
    for k, c in cell.most_common(8):
        print(f"    x={k[0]:5d}..{k[0]+20:5d}  y={k[1]:5d}..{k[1]+20:5d}  z={k[2]:4d}..{k[2]+20:4d}   边={c}")
