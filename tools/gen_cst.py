#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_cst.py -- 从 pin_table.csv 生成 constraints/top.cst

为什么需要这个脚本
=============================================================================
top.cst 里每个信号要写**两条**约束（IO_LOC + IO_PORT），
约 70 个引脚 = 140 行，而且 IO_PORT 的电气属性（PULL_MODE / DRIVE）
每个信号都不一样。手抄的典型后果：

    * 抄漏一行 -> 综合报 "port has no location constraint"，
      或者更糟：**静默地**把引脚放到别的地方去
    * PULL_MODE 抄错 -> 不报错，但编码器读不到高电平 / 上电电机乱转
    * 两个信号填了同一个球号 -> Gowin 会报错，但如果手抄时
      把 m_pwm[2] 写成 m_pwm[1] 的球号，**它不会报错**，
      结果是两个轮子一起动

这个脚本把这些都变成自动检查。

用法
=============================================================================
    python tools/gen_cst.py                # 生成 + 检查
    python tools/gen_cst.py --check        # 只检查，不写文件
    python tools/gen_cst.py --dry-run      # 打印将要生成的内容

输入：constraints/pin_table.csv
输出：constraints/top.cst
"""

import argparse
import csv
import io
import os
import re
import sys

# ---------------------------------------------------------------------------
# 中文 Windows 控制台编码兜底
#
#   控制台默认代码页是 GBK(CP936)，脚本里会打印 [OK] / [X] / -> 这类符号。
#   不设这个的话会抛
#       UnicodeEncodeError: 'gbk' codec can't encode character ...
#   注意是 errors='replace' 而**不是** encoding='utf-8' ——
#   强制 UTF-8 会让控制台把它当 GBK 解，中文全变乱码（比崩还糟）。
# ---------------------------------------------------------------------------
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors='replace')
    except (AttributeError, ValueError):
        pass


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CSV_PATH = os.path.join(ROOT, 'constraints', 'pin_table.csv')
CST_PATH = os.path.join(ROOT, 'constraints', 'top.cst')

# ---------------------------------------------------------------------------
# 高云球号格式：字母 + 数字，例如 T5 / R3 / P14 / A1 / AB12
#
#   高云 BGA 封装的行用字母、列用数字。这里只做**格式**检查，
#   不做"这个球存不存在"的检查 —— 后者要器件数据库，
#   而且真填错了 Gowin 的布局布线会报出来。
#
#   之所以还要检查格式，是因为最常见的错误是：
#       * 把封装脚位序号（1, 2, 3...）当成球号填进来
#       * 填成 PIN_xx（模板没改）
#       * 填了带空格的 "T 5"
# ---------------------------------------------------------------------------
BALL_RE = re.compile(r'^[A-Za-z]{1,2}[0-9]{1,3}$')

# 电气属性白名单（高云 IO_PORT 的合法取值）
VALID_IO_TYPE = {'LVCMOS33', 'LVCMOS25', 'LVCMOS18', 'LVCMOS15', 'LVCMOS12',
                 'SSTL15', 'SSTL135', 'LVTTL33', 'PCI33'}
VALID_PULL = {'NONE', 'UP', 'DOWN', 'KEEPER'}
VALID_DRIVE = {'', '4', '8', '12', '16', '24'}


class Row(object):
    def __init__(self, signal, pin, io_type, pull, drive, note, lineno):
        self.signal = signal.strip()
        self.pin = pin.strip()
        self.io_type = io_type.strip().upper() or 'LVCMOS33'
        self.pull = pull.strip().upper() or 'NONE'
        self.drive = drive.strip()
        self.note = note.strip()
        self.lineno = lineno


def parse_csv(path):
    """读 pin_table.csv，跳过 # 开头的注释行和空行。

    为什么要手工跳注释：csv 模块会把 'signal,pin,...' 这样的表头识别出来，
    但 '#' 开头的整行注释它不是标准 CSV 特性，得自己处理。
    """
    rows = []
    skipped_comment = 0
    with io.open(path, 'r', encoding='utf-8') as f:
        lines = []
        for i, raw in enumerate(f, start=1):
            s = raw.strip()
            if not s or s.startswith('#'):
                skipped_comment += 1
                continue
            lines.append((i, raw))

    reader = csv.reader([l for _, l in lines])
    first = True
    for (lineno, _), fields in zip(lines, reader):
        if first:
            first = False
            if fields and fields[0].strip().lower() == 'signal':
                continue        # 表头
        if len(fields) < 2:
            continue
        while len(fields) < 6:
            fields.append('')
        rows.append(Row(fields[0], fields[1], fields[2], fields[3],
                        fields[4], fields[5], lineno))
    return rows, skipped_comment


def validate(rows):
    """返回 (errors, warnings, filled, empty)"""
    errors = []
    warnings = []
    filled = []
    empty = []
    seen_pin = {}       # 球号 -> 第一个用它的信号

    for r in rows:
        if not r.signal:
            errors.append('第 %d 行：信号名为空' % r.lineno)
            continue

        if not r.pin:
            empty.append(r)
            continue

        # ---- 引脚名格式 ----
        if r.pin.startswith('PIN_'):
            errors.append('第 %d 行 %s：引脚还是占位值 %s'
                          % (r.lineno, r.signal, r.pin))
            continue
        if not BALL_RE.match(r.pin):
            errors.append('第 %d 行 %s：引脚 %r 不像高云球号（应形如 T5 / R3 / P14）'
                          % (r.lineno, r.signal, r.pin))
            continue

        # ---- 重复分配 ----
        key = r.pin.upper()
        if key in seen_pin:
            errors.append('第 %d 行 %s：球号 %s 已经被 %s 占用 —— '
                          '两个信号抢同一个引脚，布局布线会失败'
                          % (r.lineno, r.signal, r.pin, seen_pin[key]))
            continue
        seen_pin[key] = r.signal

        # ---- 电气属性 ----
        if r.io_type not in VALID_IO_TYPE:
            errors.append('第 %d 行 %s：io_type %r 不是高云合法取值'
                          % (r.lineno, r.signal, r.io_type))
        if r.pull not in VALID_PULL:
            errors.append('第 %d 行 %s：pull %r 不是 NONE/UP/DOWN/KEEPER'
                          % (r.lineno, r.signal, r.pull))
        if r.drive not in VALID_DRIVE:
            errors.append('第 %d 行 %s：drive %r 不是 4/8/12/16/24'
                          % (r.lineno, r.signal, r.drive))

        # ---- 提醒类（不阻断） ----
        if r.signal.startswith('enc_') and r.pull != 'UP':
            warnings.append('%s 是编码器输入但 PULL_MODE=%s —— '
                            '若编码器是开漏输出，可能读不到高电平'
                            % (r.signal, r.pull))
        if r.signal == 'm_en' and r.pull == 'NONE':
            warnings.append('m_en 建议 PULL_MODE=DOWN —— '
                            '上电默认停车更安全')

        filled.append(r)

    return errors, warnings, filled, empty


def gen_cst(filled, header_note):
    out = []
    w = out.append

    w('//=============================================================================')
    w('// top.cst -- 高云 (Gowin) 物理约束文件')
    w('//-----------------------------------------------------------------------------')
    w('// !!! 本文件由 tools/gen_cst.py 从 constraints/pin_table.csv 自动生成 !!!')
    w('//')
    w('//   不要直接编辑本文件 —— 下次生成会被覆盖。')
    w('//   要改引脚请改 pin_table.csv，然后重跑：')
    w('//       python tools/gen_cst.py')
    w('//')
    w('//   生成时间来源：%s' % header_note)
    w('//-----------------------------------------------------------------------------')
    w('// 电气属性说明')
    w('//   IO_TYPE  : LVCMOS33 = 3.3V')
    w('//   PULL_MODE: 编码器/按键 UP；m_en DOWN；其余 NONE')
    w('//   DRIVE    : 电机 PWM/方向 8mA；其余默认')
    w('//=============================================================================')
    w('')

    for r in filled:
        w('// %s  %s' % (r.signal, r.note) if r.note else '// %s' % r.signal)
        w('IO_LOC  "%s"%s%s;' % (r.signal, ' ' * max(1, 20 - len(r.signal)), r.pin))
        attrs = 'IO_TYPE=%s PULL_MODE=%s' % (r.io_type, r.pull)
        if r.drive:
            attrs += ' DRIVE=%s' % r.drive
        w('IO_PORT "%s"%s%s;' % (r.signal, ' ' * max(1, 20 - len(r.signal)), attrs))
        w('')

    w('//-----------------------------------------------------------------------------')
    w('// 时序约束 (SDC 风格，写在 top.sdc 里，本文件只放物理约束)')
    w('//   50MHz 周期 = 20ns;  换 PLL 100MHz 后改成 10ns')
    w('//   create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {clk_50m}]')
    w('//-----------------------------------------------------------------------------')
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser(description='从 pin_table.csv 生成 top.cst')
    ap.add_argument('--check', action='store_true', help='只检查，不写文件')
    ap.add_argument('--dry-run', action='store_true', help='打印生成内容')
    ap.add_argument('--csv', default=CSV_PATH)
    ap.add_argument('--out', default=CST_PATH)
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print('[X] 找不到 %s' % args.csv)
        return 1

    rows, n_comment = parse_csv(args.csv)
    errors, warnings, filled, empty = validate(rows)

    print('=== gen_cst: 读 %s ===' % os.path.basename(args.csv))
    print('  数据行 %d 行（另有 %d 行注释/空行被跳过）' % (len(rows), n_comment))
    print('  已填引脚 %d 个，未填 %d 个' % (len(filled), len(empty)))
    print()

    if empty:
        print('  未填的信号：')
        for r in empty:
            print('    - %-20s (第 %d 行) %s' % (r.signal, r.lineno, r.note))
        print()

    if warnings:
        print('  提醒（不阻断生成）：')
        for m in warnings:
            print('    [!] %s' % m)
        print()

    if errors:
        print('  错误：')
        for m in errors:
            print('    [X] %s' % m)
        print()
        print('RESULT: FAIL  (%d 个错误)' % len(errors))
        return 1

    if not filled:
        print('  [!] 一个引脚都没填 —— 请先按小梅哥 ACG720 的原理图/引脚表填写')
        print('RESULT: FAIL  (没有可生成的引脚)')
        return 1

    # ---- 覆盖检查：现有 top.cst 里有哪些没进 CSV ----
    if os.path.exists(args.out):
        try:
            with io.open(args.out, 'r', encoding='utf-8', errors='replace') as f:
                old = f.read()
            old_sigs = set(re.findall(r'IO_LOC\s+"([^"]+)"', old))
            new_sigs = set(r.signal for r in filled)
            lost = old_sigs - new_sigs
            if lost and 'gen_cst.py' not in old:
                print('  [!] 现有 top.cst 里有 %d 个信号不在 CSV 中，会被删掉：'
                      % len(lost))
                for s in sorted(lost):
                    print('        %s' % s)
                print('      （如果你确实想删，忽略这条；否则把它们补进 pin_table.csv）')
                print()
        except Exception:
            pass

    text = gen_cst(filled, os.path.basename(args.csv))

    if args.dry_run:
        print('=== 将要生成的内容 ===')
        print(text)
        print('RESULT: PASS  (%d 个引脚，dry-run 未写文件)' % len(filled))
        return 0

    if args.check:
        print('RESULT: PASS  (%d 个引脚，--check 未写文件)' % len(filled))
        return 0

    with io.open(args.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print('  已生成: %s  (%d 个引脚)' % (args.out, len(filled)))
    print()
    print('  下一步：')
    print('    1) 用 Gowin IDE 打开工程，跑一次 Synthesize')
    print('    2) 看 Pin Planner / IO Port 视图，确认引脚和预期一致')
    print('    3) 先烧一个最简 LED 程序，确认板子活着')
    print('RESULT: PASS  (%d 个引脚)' % len(filled))
    return 0


if __name__ == '__main__':
    sys.exit(main())
