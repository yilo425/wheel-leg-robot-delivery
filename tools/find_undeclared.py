#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
find_undeclared.py -- 找出 Verilog 文件里"用了但没声明"的标识符

为什么需要这个：
   未声明的标识符在 Verilog 里会变成**隐式 1 位线网**，不会报"未定义"，
   但会让 elaboration 出问题，表现为一句没有细节的 "Error loading design"。
   本工程已经踩过一次（tb_top_integration 里引用了未声明的 TEST_VREF）。

用法：
   python find_undeclared.py <文件1> [文件2 ...]
"""

import os
import re
import sys

VERILOG_KW = set("""
module endmodule input output inout wire reg integer real signed unsigned
parameter localparam begin end if else case endcase for while repeat forever
always initial assign generate endgenerate genvar function endfunction
task endtask fork join join_any join_none posedge negedge or and not
default casex casez time realtime supply0 supply1 tri triand trior
wait disable force release deassign event
""".split())


def strip_comments_and_strings(src):
    src = re.sub(r'/\*.*?\*/', ' ', src, flags=re.S)
    src = re.sub(r'//[^\n]*', ' ', src)
    src = re.sub(r'"[^"]*"', '""', src)
    return src


def collect_decls(src):
    names = set()

    # input/output/inout 端口
    for m in re.finditer(
            r'\b(?:input|output|inout)\s+(?:wire|reg)?\s*(?:signed\s+)?'
            r'(?:\[[^\]]*\]\s*)?([A-Za-z_]\w*)', src):
        names.add(m.group(1))

    # wire/reg/integer/... 单声明
    #   必须**显式消费** signed/unsigned 和位宽，不能用懒惰的 [^;=()]*?：
    #   后者会先匹配到 `signed` 就停，然后 finditer 跳过了整个 `[` 之后的部分，
    #   真正的变量名 `steer` 反而被漏掉（产生误报）。
    for m in re.finditer(
            r'\b(?:wire|reg|integer|real|genvar)\b'
            r'(?:\s+(?:signed|unsigned))?'
            r'(?:\s*\[[^\]]*\])?'
            r'\s+([A-Za-z_]\w*)', src):
        names.add(m.group(1))

    # 逗号分隔的多重声明
    for m in re.finditer(r'\b(?:wire|reg|integer)\b([^;]*);', src):
        for n in re.findall(r'([A-Za-z_]\w*)\s*,', m.group(1)):
            if n not in ('signed', 'unsigned'):
                names.add(n)
        tail = re.findall(r'([A-Za-z_]\w*)\s*$', m.group(1))
        names.update(n for n in tail if n not in ('signed', 'unsigned'))

    # parameter / localparam （可能多个，用逗号分隔）
    for m in re.finditer(r'\b(?:parameter|localparam)\b([^;]*);', src):
        for n in re.findall(r'([A-Za-z_]\w*)\s*=', m.group(1)):
            names.add(n)

    # task / function 名
    for m in re.finditer(r'\b(?:function|task)\b[^;]*?\b([A-Za-z_]\w*)\s*(?:;|\()', src):
        names.add(m.group(1))

    # 实例名： module_name #(...) inst_name (  或  module_name inst_name (
    for m in re.finditer(r'\b([A-Za-z_]\w*)\s*(?:#\s*\([^;]*?\))?\s*([A-Za-z_]\w*)\s*\(', src):
        names.add(m.group(2))

    return names


def find(path, extra_known=()):
    try:
        src = open(path, encoding='utf-8', errors='replace').read()
    except OSError as exc:
        return {}, "无法读取: %s" % exc

    s = strip_comments_and_strings(src)
    decls = collect_decls(s) | set(extra_known)

    used = {}
    for lineno, line in enumerate(s.splitlines(), 1):
        for m in re.finditer(r'\b([A-Za-z_]\w*)\b', line):
            n = m.group(1)
            if n in VERILOG_KW or n in decls or n.startswith('$'):
                continue
            used.setdefault(n, lineno)
    return used, None


def main():
    files = sys.argv[1:]
    here = os.path.dirname(os.path.abspath(__file__))
    prj = os.path.dirname(here)

    if not files:
        files = [os.path.join(prj, 'rtl', 'top.v'),
                 os.path.join(prj, 'sim', 'tb_top_integration.v')]

    total = 0
    for f in files:
        path = f if os.path.isabs(f) else f
        if not os.path.exists(path):
            cand = os.path.join(prj, f)
            path = cand if os.path.exists(cand) else path
        used, err = find(path)
        print("===== %s =====" % os.path.relpath(path, os.getcwd())
              if os.path.exists(path) else "===== %s =====" % f)
        if err:
            print("  %s" % err)
            continue
        if not used:
            print("  [OK] 没发现未声明标识符")
        else:
            for n, ln in sorted(used.items(), key=lambda kv: kv[1]):
                print("  [??] 行%d: '%s'" % (ln, n))
                total += 1
        print()

    print("=== 共 %d 处可疑 ===" % total)
    if total:
        print()
        print("说明：这里列出的名字**可能**是误报（比如模块名、跨文件引用）。")
        print("      但只要有真正的未声明标识符，就会变成隐式线网，")
        print("      导致 elaboration 失败，报出无细节的 Error loading design。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
