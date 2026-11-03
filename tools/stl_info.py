#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stl_info.py —— 读二进制 STL，打印包围盒 / 三角面数 / 非流形边数。

用法: python tools/stl_info.py mech/body_v5_print.stl [...]
"""
import sys, io, struct, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import numpy as np


def info(path):
    with open(path, 'rb') as f:
        head = f.read(84)
        n = struct.unpack('<I', head[80:84])[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tris = data[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    q = np.round(tris * 1000.0).astype(np.int64)
    edges = collections.Counter()
    for t in q:
        for a, b in ((0, 1), (1, 2), (2, 0)):
            e = tuple(sorted((tuple(t[a]), tuple(t[b]))))
            edges[e] += 1
    bad = sum(1 for c in edges.values() if c != 2)
    print(f"{path}")
    print(f"  三角面 {n}   包围盒 X {hi[0]-lo[0]:7.1f}  Y {hi[1]-lo[1]:7.1f}  Z {hi[2]-lo[2]:7.1f} mm")
    print(f"  坐标范围 x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}] z[{lo[2]:.1f},{hi[2]:.1f}]")
    print(f"  非流形边 {bad}  {'OK' if bad == 0 else '!! 需要修'}")
    return bad


if __name__ == '__main__':
    args = sys.argv[1:]
    if not args:
        print("用法: python tools/stl_info.py <a.stl> [b.stl ...]")
        sys.exit(1)
    total = sum(info(p) for p in args)
    sys.exit(0 if total == 0 else 2)
