#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_decl_order.py -- 检查 Verilog 里"先用后声明"的问题

为什么需要：
    ModelSim 的 vlog 报
        (vlog-2730) Undefined variable: 'acc'
        (vlog-2388) 'acc' already declared in this scope
    这类错时，一轮只报一个文件，改完再跑又冒出下一个，很浪费时间。
    这个脚本一次性把所有文件的"先用后声明"全找出来。

规则（Verilog-2001 要求先声明后使用）：
    在模块作用域里，某个标识符第一次出现在"表达式"中，
    必须早于它的声明行。

用法：
    python check_decl_order.py            # 检查 ../rtl
    python check_decl_order.py <路径>...
"""

import os
import re
import sys

DECL_KW = ('input', 'output', 'inout', 'wire', 'reg', 'integer',
           'parameter', 'localparam', 'genvar', 'real', 'time')


def strip_comments(text):
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
                in_block = False
                out.append('  ')
                i += 2
                continue
            out.append('\n' if ch == '\n' else ' ')
            i += 1
            continue
        if in_str:
            out.append(ch)
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
            out.append(ch)
            i += 1
            continue
        out.append(ch)
        i += 1
    return ''.join(out)


def check_file(path, results):
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        raw = fh.read()
    code = strip_comments(raw)
    lines = code.split('\n')

    # 找模块边界
    in_module = False
    decl_line = {}        # 名字 -> 首次声明行
    first_use = {}        # 名字 -> 首次使用行
    order = []            # (行号, 类型, 名字)

    for ln, text in enumerate(lines, 1):
        t = text.strip()
        if not t:
            continue

        if re.match(r'^module\b', t):
            in_module = True
            decl_line = {}
            first_use = {}
            continue
        if re.match(r'^endmodule\b', t):
            # ---- 模块结束，核对顺序 ----
            for name, use_ln in first_use.items():
                d = decl_line.get(name)
                if d is not None and use_ln < d:
                    results.append((path, use_ln, d, name))
            in_module = False
            continue
        if not in_module:
            continue
        # task/function 内部单独算，跳过以免误报
        if re.match(r'^(task|function)\b', t):
            in_module = False
            continue
        if re.match(r'^(endtask|endfunction)\b', t):
            in_module = True
            continue

        # --- 记录声明 ---
        m = re.match(r'^(' + '|'.join(DECL_KW) + r')\b(.*)$', t)
        if m:
            rest = m.group(2)
            rest = re.sub(r'\b(signed|unsigned)\b', ' ', rest)
            rest = re.sub(r'\[[^\]]*\]', ' ', rest)
            rest = rest.split('=')[0]
            for name in re.findall(r'[A-Za-z_][A-Za-z0-9_$]*', rest):
                if name in ('signed', 'unsigned'):
                    continue
                decl_line.setdefault(name, ln)
            # 声明行右侧的初值表达式也算"使用"
            if '=' in t:
                rhs = t.split('=', 1)[1]
                for name in re.findall(r'[A-Za-z_][A-Za-z0-9_$]*', rhs):
                    if name in DECL_KW or name in ('signed', 'unsigned'):
                        continue
                    if name not in first_use:
                        first_use[name] = ln
            continue

        # --- 记录使用（非声明行） ---
        for name in re.findall(r'[A-Za-z_][A-Za-z0-9_$]*', t):
            if name in DECL_KW:
                continue
            if name not in first_use and not re.match(r'^\s*(assign|always|initial|generate|endgenerate|case|endcase|begin|end|if|else|for|while|repeat)\b', t):
                # 只关心看起来像信号的标识符
                if not name.startswith('$'):
                    first_use.setdefault(name, ln)


def main():
    args = sys.argv[1:]
    if args:
        files = []
        for a in args:
            if os.path.isdir(a):
                for root, _, names in os.walk(a):
                    for n in names:
                        if n.endswith('.v'):
                            files.append(os.path.join(root, n))
            else:
                files.append(a)
    else:
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rtl')
        files = []
        for root, _, names in os.walk(base):
            for n in names:
                if n.endswith('.v'):
                    files.append(os.path.join(root, n))

    results = []
    for f in sorted(files):
        if os.path.basename(f).startswith('tb_'):
            continue
        check_file(f, results)

    print(f"=== check_decl_order: 检查 {len(files)} 个文件 ===")
    if not results:
        print("未发现『先用后声明』问题。")
        print("RESULT: PASS")
        return 0

    real = []
    for path, use_ln, decl_ln, name in results:
        # 过滤掉明显的误报：名字在模块端口列表里(ANSI 端口在 module 行已声明)
        real.append((path, use_ln, decl_ln, name))

    for path, use_ln, decl_ln, name in real:
        print(f"  [ERROR] {os.path.relpath(path)}: '{name}' 在第 {use_ln} 行被使用, "
              f"但第 {decl_ln} 行才声明")
    print(f"=== 共 {len(real)} 处 ===")
    print("RESULT: FAIL")
    return 1


if __name__ == '__main__':
    sys.exit(main())
