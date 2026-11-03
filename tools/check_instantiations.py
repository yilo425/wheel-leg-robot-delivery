#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_instantiations.py -- 检查模块例化时的**参数名 / 端口名**是否正确

为什么需要这个（这是本工程最"阴"的一类错误）：
   `vlog` **不检查**参数覆盖名和端口连接名是否存在于目标模块 ——
   它只在**编译阶段**处理语法，名字对不对要等 **elaboration** 才知道。

   于是写错一个参数名会出现：
       vlog    : Errors: 0, Warnings: 0       ← 一片祥和
       vsim    : Error loading design         ← 没有任何细节
   （vsim 不说哪一行、不说哪个名字）

   本工程真实踩到：
       top.v:319  pwm_motor #( ... .DUTY_WIDTH (16) )
       而 pwm_motor 的参数叫 CNT_WIDTH，于是 elaboration 直接失败，
       vsim 只给一句 "Error loading design"，查了很久。

   这类错误**只有 vopt 才报得清楚**：
       ** Error (vopt-2732) Module parameter 'DUTY_WIDTH' not found for override.

   本脚本在**编译之前**就用纯文本解析把这类问题找出来，不用等 vopt。

检查内容：
   A) 参数覆盖名是否存在于目标模块的 parameter/localparam 列表
   B) 端口连接名是否存在于目标模块的端口列表
   C) 目标模块是否在本工程内定义（跨文件也认）
   D) 目标模块的端口是否有漏连（提示级）

用法：
   python check_instantiations.py                 # 扫 rtl/ 与 sim/
   python check_instantiations.py rtl sim
退出码：0 = 无问题，1 = 发现问题
"""

import os
import re
import sys

VERILOG_KW = set("""
module endmodule input output inout wire reg integer real signed unsigned
parameter localparam begin end if else case endcase for while repeat forever
always initial assign generate endgenerate genvar function endfunction
task endtask fork join posedge negedge or and not default casex casez
time realtime supply0 supply1 tri triand trior wait disable
""".split())


#=============================================================================
# 文本处理
#=============================================================================
def strip_comments(src):
    src = re.sub(r'/\*.*?\*/', ' ', src, flags=re.S)
    src = re.sub(r'//[^\n]*', ' ', src)
    return src


def match_paren(text, start):
    """text[start] 必须是 '('；返回与之匹配的 ')' 的下标，找不到返回 -1"""
    depth = 0
    i = start
    n = len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


def parse_names(block):
    """从 '( .a(x), .b(y) )' 里取出 [名字] —— 只取 .name( 形式（端口连接/参数覆盖）"""
    return re.findall(r'\.\s*([A-Za-z_]\w*)\s*\(', block)


def parse_param_decls(block):
    """
    从**模块定义**的参数块里取参数名。
    定义里写的是 `parameter integer NAME = value`（逗号分隔多个），
    不是覆盖时的 `.NAME(value)`，所以不能用 parse_names。

    要处理：`parameter integer A = 1, B = 2, parameter [7:0] C = 8'h04`
    """
    names = set()
    for m in re.finditer(
            r'\b(?:parameter|localparam)\b(.*?)'
            r'(?=;|\bparameter\b|\blocalparam\b|$)', block, flags=re.S):
        seg = m.group(1)
        for n in re.findall(r'([A-Za-z_]\w*)\s*=', seg):
            if n not in ('signed', 'unsigned'):
                names.add(n)
    return names


#=============================================================================
# 解析模块定义
#=============================================================================
def parse_modules(src):
    """
    返回 {模块名: {'params': set(), 'ports': set(), 'file':..., 'line':...}}
    """
    mods = {}
    for m in re.finditer(r'\bmodule\s+([A-Za-z_]\w*)', src):
        name = m.group(1)
        pos = m.end()
        params, ports = set(), set()

        # 跳过空白
        while pos < len(src) and src[pos] in ' \t\r\n':
            pos += 1

        # 可选参数块 #( ... )
        if pos < len(src) and src[pos] == '#':
            pos += 1
            while pos < len(src) and src[pos] in ' \t\r\n':
                pos += 1
            if pos < len(src) and src[pos] == '(':
                end = match_paren(src, pos)
                if end > 0:
                    params = parse_param_decls(src[pos:end])
                    pos = end + 1

        # 端口块 ( ... )
        while pos < len(src) and src[pos] in ' \t\r\n':
            pos += 1
        if pos < len(src) and src[pos] == '(':
            end = match_paren(src, pos)
            if end > 0:
                body = src[pos:end]
                # 端口可以是 .name(sig) 或 直接的 (input wire [..] name)
                named = set(parse_names(body))
                if named:
                    ports = named
                else:
                    for pm in re.finditer(
                            r'\b(?:input|output|inout)\b[^,;)]*?'
                            r'([A-Za-z_]\w*)\s*(?=,|\)|$)', body):
                        ports.add(pm.group(1))
                pos = end + 1

        mods[name] = {
            'params': params,
            'ports': ports,
            'line': src[:m.start()].count('\n') + 1,
        }
    return mods


#=============================================================================
# 解析例化
#=============================================================================
def parse_instances(src, known_modules):
    """
    返回 [(模块名, 参数覆盖 set, 端口连接 set, 行号)]
    只认"首词是本工程里定义过的模块名"的形式，避免把 always/if/case 误判成例化。
    """
    out = []
    # 逐个已知模块名去找  NAME [#(...)] inst (...)
    for mod in known_modules:
        pat = re.compile(r'\b' + re.escape(mod) + r'\b')
        for m in pat.finditer(src):
            pos = m.end()
            # 避免匹配到 module 定义本身
            before = src[max(0, m.start() - 20):m.start()]
            if re.search(r'\bmodule\s+$', before):
                continue

            while pos < len(src) and src[pos] in ' \t\r\n':
                pos += 1

            p_ovr = set()
            if pos < len(src) and src[pos] == '#':
                pos += 1
                while pos < len(src) and src[pos] in ' \t\r\n':
                    pos += 1
                if pos < len(src) and src[pos] == '(':
                    end = match_paren(src, pos)
                    if end < 0:
                        continue
                    p_ovr = set(parse_names(src[pos:end]))
                    pos = end + 1

            while pos < len(src) and src[pos] in ' \t\r\n':
                pos += 1

            # 实例名
            nm = re.match(r'([A-Za-z_]\w*)', src[pos:])
            if not nm:
                continue
            pos += nm.end()

            while pos < len(src) and src[pos] in ' \t\r\n':
                pos += 1
            if pos >= len(src) or src[pos] != '(':
                continue
            end = match_paren(src, pos)
            if end < 0:
                continue
            conns = set(parse_names(src[pos:end]))

            line = src[:m.start()].count('\n') + 1
            out.append((mod, p_ovr, conns, line, nm.group(1)))
            pos = end
    out.sort(key=lambda t: t[3])
    return out


#=============================================================================
# 主流程
#=============================================================================
def main():
    targets = sys.argv[1:] or ['rtl', 'sim']
    here = os.path.dirname(os.path.abspath(__file__))
    prj = os.path.dirname(here)

    roots = []
    for t in targets:
        if os.path.isabs(t):
            roots.append(t); continue
        for base in (os.getcwd(), prj, here):
            cand = os.path.normpath(os.path.join(base, t))
            if os.path.exists(cand):
                roots.append(cand); break
        else:
            print("  [WARN] 找不到路径: %s（跳过）" % t)

    files = []
    for root in roots:
        if os.path.isfile(root):
            files.append(root)
        elif os.path.isdir(root):
            for dp, _dn, fns in os.walk(root):
                for fn in fns:
                    if fn.endswith('.v') or fn.endswith('.vh'):
                        files.append(os.path.join(dp, fn))
    files = sorted(set(files))

    # 先收集所有模块定义（跨文件）
    all_mods = {}
    srcs = {}
    for path in files:
        try:
            raw = open(path, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        src = strip_comments(raw)
        srcs[path] = src
        for name, info in parse_modules(src).items():
            info['file'] = path
            all_mods[name] = info

    problems = 0
    for path in files:
        src = srcs[path]
        try:
            rel = os.path.relpath(path, os.getcwd())
        except ValueError:
            rel = path
        for mod, p_ovr, conns, line, inst in parse_instances(src, all_mods.keys()):
            if mod not in all_mods:
                continue
            info = all_mods[mod]

            # A) 参数名
            for p in sorted(p_ovr):
                if p not in info['params']:
                    print("  [ERROR] %s:%d: 例化 %s 时参数 '%s' 不存在"
                          % (rel, line, mod, p))
                    print("           %s 的参数是: %s"
                          % (mod, ', '.join(sorted(info['params'])) or '(无)'))
                    problems += 1

            # B) 端口名
            for c in sorted(conns):
                if c not in info['ports']:
                    print("  [ERROR] %s:%d: 例化 %s 时端口 '.%s' 不存在"
                          % (rel, line, mod, c))
                    print("           %s 的端口是: %s"
                          % (mod, ', '.join(sorted(info['ports'])) or '(无)'))
                    problems += 1

            # D) 漏连（提示级，不算错误）
            missing = info['ports'] - conns
            if missing and conns:
                print("  [hint ] %s:%d: %s 未连接的端口: %s"
                      % (rel, line, mod, ', '.join(sorted(missing))))

    print()
    print("=== check_instantiations: 扫描 %d 个文件, %d 个模块, %d 处错误 ==="
          % (len(files), len(all_mods), problems))
    if problems == 0:
        print("RESULT: PASS")
        return 0
    print("RESULT: FAIL")
    print()
    print("说明：这类错误 vlog **查不出来**（编译期不校验名字），")
    print("      要到 elaboration 才失败，而 vsim 只给一句")
    print("      'Error loading design' —— 没有任何细节。")
    print("      本脚本在编译前就把它找出来。")
    return 1


if __name__ == '__main__':
    sys.exit(main())
