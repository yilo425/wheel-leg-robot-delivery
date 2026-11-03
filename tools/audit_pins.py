#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audit_pins.py -- 引脚分配全量审计

查四件事：
  1) 同一个球号被分配给了两个信号   （重复引脚 -> 硬件冲突）
  2) 同一个信号名出现两次           （重复定义）
  3) .cst 和 pin_table.csv 是否一致  （生成的约束和源头不同步）
  4) 是否有引脚被分配到"不可达"的位置（不在 GPIO0 排针、也不是板载固定器件）
     —— 这一条最有价值：分到不可达的球号 = 线根本接不上去

用法：python tools/audit_pins.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIN_CSV = os.path.join(ROOT, 'constraints', 'pin_table.csv')
CST = os.path.join(ROOT, 'constraints', 'top.cst')

# ---------------------------------------------------------------- GPIO0 排针
# 用户拍的实物照片（每个孔旁边印着球号）
# 顶排：F20 那一排；底排：A13 那一排
GPIO0_TOP = ['F20', 'E18', 'D19', 'C20', 'E17', 'GND', 'C19', 'B18',
             'D16', 'C17', 'B16', 'C15', 'D15', 'B13', 'GND', 'E14',
             'F14', 'A19', 'A16', 'A14']
GPIO0_BOT = ['F19', 'F18', 'E19', 'D20', 'F16', '3V3', 'C18', 'B17',
             'E16', 'D17', 'B15', 'C14', 'D14', 'C13', '5V', 'E13',
             'F13', 'A18', 'A15', 'A13']

# 板上固定器件（不在 GPIO0 排针上，是焊在板上的）
BOARD_FIXED = {
    'Y18': '板载 50MHz 晶振',
    'B21': 'S4 复位键',
    'F15': 'S0 启动键',
    'A20': 'S1 急停键',
    'M22': 'LED0', 'N22': 'LED1', 'L21': 'LED2', 'K21': 'LED3',
    'K22': 'LED4', 'J22': 'LED5', 'H22': 'LED6',
    'M21': 'LED7', 'N19': 'LED8（与 BEEP 同球，待确认）',
    'M15': '板载 UART TX 焊盘',
}


def load_pin_table():
    rows = []
    with open(PIN_CSV, encoding='utf-8') as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('signal,'):
                continue
            p = [x.strip() for x in line.split(',')]
            if len(p) < 2:
                continue
            rows.append({'sig': p[0], 'pin': p[1], 'io': p[2] if len(p) > 2 else '',
                         'pull': p[3] if len(p) > 3 else '',
                         'note': ','.join(p[5:]) if len(p) > 5 else '', 'ln': ln})
    return rows


def load_cst():
    m = {}
    if not os.path.exists(CST):
        return m
    with open(CST, encoding='utf-8', errors='replace') as f:
        for line in f:
            mm = re.match(r'\s*IO_LOC\s+"([^"]+)"\s+([A-Z]\d+)\s*;', line)
            if mm:
                m[mm.group(1)] = mm.group(2)
    return m


def hole_of(ball):
    """返回该球号在 GPIO0 排针上的物理位置，不在则 None"""
    if ball in GPIO0_TOP:
        return '顶排第 %d 孔' % (GPIO0_TOP.index(ball) + 1)
    if ball in GPIO0_BOT:
        return '底排第 %d 孔' % (GPIO0_BOT.index(ball) + 1)
    return None


def main():
    rows = load_pin_table()
    cst = load_cst()
    problems = []

    print('=== 引脚审计 ===')
    print('pin_table.csv: %d 行' % len(rows))
    print('top.cst      : %d 个 IO_LOC' % len(cst))
    print()

    # ---- 1) 重复球号 ----
    by_pin = {}
    for r in rows:
        by_pin.setdefault(r['pin'], []).append(r)
    dup_pin = {k: v for k, v in by_pin.items() if len(v) > 1}
    print('[1] 重复球号（同一引脚分配两次）')
    if dup_pin:
        for pin, rs in sorted(dup_pin.items()):
            print('  !! %s 被分配给了: %s' % (pin, ', '.join(r['sig'] for r in rs)))
            problems.append('duplicate pin %s' % pin)
    else:
        print('    OK 无重复 OK')

    # ---- 2) 重复信号名 ----
    by_sig = {}
    for r in rows:
        by_sig.setdefault(r['sig'], []).append(r)
    dup_sig = {k: v for k, v in by_sig.items() if len(v) > 1}
    print('\n[2] 重复信号名')
    if dup_sig:
        for sig, rs in sorted(dup_sig.items()):
            print('  !! %s 出现 %d 次: %s' % (sig, len(rs), ', '.join(r['pin'] for r in rs)))
            problems.append('duplicate signal %s' % sig)
    else:
        print('    OK 无重复 OK')

    # ---- 3) .cst 与 csv 一致性 ----
    print('\n[3] top.cst 与 pin_table.csv 一致性')
    csv_map = {r['sig']: r['pin'] for r in rows}
    only_csv = set(csv_map) - set(cst)
    only_cst = set(cst) - set(csv_map)
    diff = {s for s in set(csv_map) & set(cst) if csv_map[s] != cst[s]}
    if only_csv:
        print('  !! 只在 csv 里: %s' % ', '.join(sorted(only_csv)))
        problems.append('csv-only %d' % len(only_csv))
    if only_cst:
        print('  !! 只在 cst 里: %s' % ', '.join(sorted(only_cst)))
        problems.append('cst-only %d' % len(only_cst))
    if diff:
        for s in sorted(diff):
            print('  !! %s: csv=%s  cst=%s' % (s, csv_map[s], cst[s]))
        problems.append('mismatch %d' % len(diff))
    if not (only_csv or only_cst or diff):
        print('    OK 完全一致 OK')

    # ---- 4) 可达性 ----
    print('\n[4] 引脚可达性（能不能真的接上线）')
    unreachable = []
    for r in rows:
        if hole_of(r['pin']):
            continue
        if r['pin'] in BOARD_FIXED:
            continue
        unreachable.append(r)
    if unreachable:
        for r in unreachable:
            print('  !! %s -> %s：既不在 GPIO0 排针，也不是板载器件' % (r['sig'], r['pin']))
            problems.append('unreachable %s' % r['pin'])
    else:
        print('    OK 全部可达 OK（要么在 GPIO0 排针上，要么是板载器件）')

    # ---- 5) GPIO0 占用统计 ----
    used = {}
    for r in rows:
        h = hole_of(r['pin'])
        if h:
            used[r['pin']] = r['sig']
    signal_holes = [b for b in GPIO0_TOP + GPIO0_BOT if b not in ('GND', '3V3', '5V')]
    print('\n[5] GPIO0 排针占用  %d / %d' % (len(used), len(signal_holes)))
    free = [b for b in signal_holes if b not in used]
    print('    空闲: %s' % (', '.join('%s(%s)' % (b, hole_of(b)) for b in free) if free else '无'))

    # ---- 汇总 ----
    print('\n=== 结果 ===')
    if problems:
        print('发现 %d 个问题' % len(problems))
        for p in problems:
            print('  - ' + p)
        return 1
    print('全部通过：无重复引脚、cst 同步、全部可达')
    return 0


if __name__ == '__main__':
    sys.exit(main())
