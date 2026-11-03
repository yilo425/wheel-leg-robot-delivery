#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_literal_overflow.py -- 扫描 Verilog 里的 32 位溢出风险

为什么写这个：
   同一天里，"32 位溢出"这个 bug 类在工程里出现了 **4 次**：

     1. uart_tx/uart_rx : BAUD_RATE * 2^24 = 1.93e12  -> BAUD_INC 回绕成 0
                          => 硬件上一个字节都发不出去（致命）
     2. tb_uart         : 10 * 1000000000 = 1e10       -> guard 上限算错
                          => 测试超时，误判为 RTL 有问题
     3. main_fsm.v      : CLK_FREQ_HZ * AVOID_TIMEOUT_MS = 2.5e11
                          => 参数自检的判断结果完全错误
     4. tb_main_fsm.v   : #5_000_000_000 （无尺寸字面量超 32 位）
                          => vlog 拒绝编译，表现为"tb 加载失败"，
                             且报错无细节，极难定位

   这四条都属于"看代码时很难注意到、但后果严重"的类型，
   值得用一个自动化检查兜住。

检查内容：
   A) 无尺寸十进制字面量 > 2^31-1（除非显式带位宽，如 64'd...）
   B) `localparam/parameter integer NAME = A * B;` 形式中 A*B 超出 32 位
   C) `#delay` 里的无尺寸字面量超 32 位

用法：
   python check_literal_overflow.py                 # 扫描默认目录
   python check_literal_overflow.py ../rtl ../sim
退出码：0 = 无问题，1 = 发现问题
"""

import os
import re
import sys

INT32_MAX = 2**31 - 1
UINT32_MAX = 2**32 - 1

# 无尺寸十进制字面量：一串数字（可带下划线分隔），
#   前面不能紧跟 '  （排除 64'd123 这类带位宽的）
#   后面不能紧跟 '  （排除 123'd45 这类带位宽的）
LITERAL_RE = re.compile(r"(?<![0-9A-Za-z_'])(\d[\d_]*)(?![0-9A-Za-z_'])")

# localparam/parameter 里的常量乘法： NAME = A * B ;
MUL_RE = re.compile(
    r"\b(?:localparam|parameter)\b[^=;]*?\b(\w+)\s*=\s*"
    r"(\d[\d_]*)\s*\*\s*(\d[\d_]*)\s*;"
)

# 延迟控制：#123 / #1_000
DELAY_RE = re.compile(r"#\s*(\d[\d_]*)")


def strip_comment(line):
    """去掉 // 之后的注释（不做完整词法分析，够用即可）"""
    idx = line.find("//")
    if idx >= 0:
        return line[:idx]
    return line


def num(tok):
    return int(tok.replace("_", ""))


def check_file(path):
    """返回问题列表 [(行号, 类别, 说明)]"""
    problems = []
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError as exc:
        return [(0, "READ", "无法读取: %s" % exc)]

    for lineno, raw in enumerate(lines, 1):
        line = strip_comment(raw)

        # ---- A) 无尺寸字面量超 32 位 ----
        for m in LITERAL_RE.finditer(line):
            tok = m.group(1)
            val = num(tok)
            if val > UINT32_MAX:
                problems.append((lineno, "A-无尺寸字面量",
                                 "%s = %d 超 32 位（上限 %d）"
                                 % (tok, val, UINT32_MAX)))

        # ---- B) 常量乘法溢出 ----
        for m in MUL_RE.finditer(line):
            name, a_tok, b_tok = m.group(1), m.group(2), m.group(3)
            prod = num(a_tok) * num(b_tok)
            if prod > INT32_MAX:
                problems.append((lineno, "B-常量乘法溢出",
                                 "%s = %s * %s = %d 超 32 位有符号上限 %d"
                                 % (name, a_tok, b_tok, prod, INT32_MAX)))

        # ---- C) 延迟值超 32 位 ----
        for m in DELAY_RE.finditer(line):
            tok = m.group(1)
            val = num(tok)
            if val > UINT32_MAX:
                problems.append((lineno, "C-延迟值超 32 位",
                                 "#%s = %d" % (tok, val)))

    return problems


def main():
    targets = sys.argv[1:] or ["rtl", "sim"]
    here = os.path.dirname(os.path.abspath(__file__))       # .../robot_project/tools
    prj = os.path.dirname(here)                              # .../robot_project

    roots = []
    for t in targets:
        if os.path.isabs(t):
            roots.append(os.path.normpath(t))
            continue
        # 依次尝试：当前工作目录 -> 工程根目录 -> 脚本所在目录
        for base in (os.getcwd(), prj, here):
            cand = os.path.normpath(os.path.join(base, t))
            if os.path.exists(cand):
                roots.append(cand)
                break
        else:
            print("  [WARN ] 找不到路径: %s（跳过）" % t)

    files = []
    for root in roots:
        if os.path.isfile(root) and (root.endswith(".v") or root.endswith(".vh")):
            files.append(root)
        elif os.path.isdir(root):
            for dirpath, _dirnames, filenames in os.walk(root):
                for fn in filenames:
                    if fn.endswith(".v") or fn.endswith(".vh"):
                        files.append(os.path.join(dirpath, fn))

    files = sorted(set(files))
    total = 0
    for path in files:
        problems = check_file(path)
        if not problems:
            continue
        try:
            rel = os.path.relpath(path, os.getcwd())
        except ValueError:
            rel = path
        for lineno, kind, msg in problems:
            print("  [WARN ] %s:%d: %s | %s" % (rel, lineno, kind, msg))
            total += 1

    print()
    print("=== check_literal_overflow: 扫描 %d 个文件, %d 处风险 ==="
          % (len(files), total))
    if total == 0:
        print("RESULT: PASS")
        return 0
    print("RESULT: FAIL")
    print()
    print("说明：32 位溢出是本工程反复踩到的坑（已出现 4 次）。")
    print("      修法三选一：")
    print("        * 减小数值，使其落在 32 位内")
    print("        * 显式指定位宽：64'd5000000000")
    print("        * 用实数比较，或先做除法再乘（避免中间量过大）")
    return 1


if __name__ == "__main__":
    sys.exit(main())
