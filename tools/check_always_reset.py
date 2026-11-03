#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_always_reset.py -- 检查"时序 always 缺少复位分支"

为什么需要：
    项目规则 2.6 写的是"**每个 `always` 都要有复位分支**"。
    但规则写在文档里，靠人记 —— 而这个错误**编译器不会报**：

        always @(posedge clk) begin
            cnt <= cnt + 1;          // 忘了复位
        end

    上电后 cnt 是 X，然后一路 X 传播下去。仿真里会看到一堆 x，
    但如果是"偶尔跑到某个分支才用 cnt"，可能一直是绿的；
    上板表现是"偶尔启动后不工作、重上电又好了" —— 极难查。

    这个脚本把规则变成机器检查，和 check_decl_order / rtl_lint 一起跑。

判定规则：
    - 只看**时序** always：敏感列表里出现 `posedge` 或 `negedge`
      （组合 `always @(*)` / `always @(a or b)` 不需要复位，跳过）
    - 时序 always 的**块内**必须出现复位判断，可接受的写法：
        if (!rst_n)  /  if (rst_n == 1'b0)  /  if (rst_n == 0)
        if (~rst_n)  /  if (rst_n) begin ... end else
      （最后一种"先判复位"的写法也认，因为它是 `if (rst_n)` 开头，
        块内没有别的条件先出现 —— 见下面的简化说明）
    - 判定窗口：always 块内的**前 8 行**

简化说明（宁可漏报，不要误报成灾）：
    脚本做的是"块内前几行有没有 rst"的文本判定，不做完整语法分析。
    所以：
      * `if (!rst_n)` 在前 8 行内 -> 通过
      * 完全没有 rst 字样         -> 报错
      * 有 rst 但写法古怪         -> 通过（漏报，可接受）
    误报比漏报烦人得多 —— 一个天天误报的检查会被关掉。

用法：
    python check_always_reset.py            # 检查 rtl/
    python check_always_reset.py <路径>...
"""

import os
import re
import sys

# 时序 always 的关键字
EDGE_RE = re.compile(r'\b(posedge|negedge)\b')

# 可接受的复位判断写法（在块内前 N 行里找）
#   ⚠️ 必须容忍前缀/后缀：本项目里既用 `rst_n`，也用 `arst_n`（reset_sync）。
#      第一版只写了 `!\s*rst`，结果 `!arst_n` 被判成"没有复位" -> 误报 2 处。
RESET_PATTERNS = [
    re.compile(r'if\s*\(\s*!\s*\w*rst\w*', re.I),              # if (!rst_n) / if (!arst_n)
    re.compile(r'if\s*\(\s*~\s*\w*rst\w*', re.I),              # if (~rst_n)
    re.compile(r'if\s*\(\s*\w*rst\w*\s*==\s*1?\'b?0', re.I),   # if (rst_n == 1'b0)
    re.compile(r'if\s*\(\s*\w*rst\w*\s*\)', re.I),             # if (rst_n) ... else
]

# 白名单标记：出现在 always 行或它上面 2 行的**注释**里就跳过
#   用于"确实不需要复位"的少数情况，例如：
#       * 存储器写口（给整块 RAM 加复位要一个巨大的 mux，代价远大于收益）
#       * 每拍都被无条件覆盖的流水寄存器
WAIVE_MARK = 'check_always_reset: waive'

WINDOW = 8      # 在 always 关键字之后的这几行内找复位


def strip_comments(text: str) -> str:
    """把注释和字符串替换成空格（保留换行，行号不变）。"""
    out = []
    i, n = 0, len(text)
    in_line = in_block = in_str = False
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ''
        if in_line:
            out.append('\n' if ch == '\n' else ' ')
            if ch == '\n':
                in_line = False
            i += 1
            continue
        if in_block:
            if ch == '*' and nxt == '/':
                out.append('  ')
                i += 2
                in_block = False
                continue
            out.append('\n' if ch == '\n' else ' ')
            i += 1
            continue
        if in_str:
            out.append(' ')
            if ch == '\\' and i + 1 < n:
                out.append(' ')
                i += 2
                continue
            if ch == '"':
                in_str = False
            i += 1
            continue
        if ch == '/' and nxt == '/':
            in_line = True
            out.append('  ')
            i += 2
            continue
        if ch == '/' and nxt == '*':
            in_block = True
            out.append('  ')
            i += 2
            continue
        if ch == '"':
            in_str = True
            out.append(' ')
            i += 1
            continue
        out.append(ch)
        i += 1
    return ''.join(out)


def check_file(path: str, results: list):
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        raw = f.read()
    raw_lines = raw.split('\n')
    text = strip_comments(raw)
    lines = text.split('\n')

    for idx, line in enumerate(lines):
        m = re.search(r'\balways\b', line)
        if not m:
            continue

        # 白名单：always 行或上面 2 行的注释里有没有 waive 标记
        ctx = '\n'.join(raw_lines[max(0, idx - 2): idx + 1])
        if WAIVE_MARK in ctx:
            continue

        # 收集这个 always 的敏感列表（可能跨行）
        header = line[m.start():]
        j = idx
        while '@' in header and ')' not in header.split('@', 1)[1] and j + 1 < len(lines) and j - idx < 4:
            j += 1
            header += ' ' + lines[j]

        if not EDGE_RE.search(header):
            continue        # 组合 always，不需要复位

        # 在 always 之后的 WINDOW 行内找复位判断
        window = '\n'.join(lines[idx: idx + WINDOW + 1])
        if any(p.search(window) for p in RESET_PATTERNS):
            continue        # 有复位，通过

        results.append((path, idx + 1, header.strip()[:70]))


def iter_verilog(paths):
    for p in paths:
        if os.path.isfile(p):
            if p.endswith('.v'):
                yield p
            continue
        for root, _dirs, files in os.walk(p):
            for fn in sorted(files):
                if fn.endswith('.v'):
                    yield os.path.join(root, fn)


def main(argv):
    args = argv[1:]
    if args:
        targets = args
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        targets = [os.path.join(os.path.dirname(here), 'rtl')]

    files = [f for f in iter_verilog(targets)
             if not os.path.basename(f).startswith('tb_')]

    results = []
    for f in files:
        check_file(f, results)

    print(f"=== check_always_reset: 检查 {len(files)} 个 RTL 文件 ===")
    if not results:
        print("所有时序 always 都有复位分支。")
        print("RESULT: PASS")
        return 0

    for path, ln, hdr in results:
        print(f"  [ERROR] {os.path.relpath(path)}:{ln}: 时序 always 前 {WINDOW} 行没有复位")
        print(f"          {hdr}")
    print(f"=== 共 {len(results)} 处缺少复位 ===")
    print("RESULT: FAIL")
    return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
