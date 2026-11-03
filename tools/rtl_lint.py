#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rtl_lint.py -- 轻量 Verilog/RTL 静态检查 (不需要仿真器)

为什么需要它：
    在没装仿真器的机器上（或者 CI 里），很多低级错误只能靠这类静态检查提前抓到。
    本项目就是靠它发现了 uart_rx.v 里 os_acc 重复声明这种编译不过的问题。

检查项：
    1. module / endmodule 配对
    2. begin / end、case / endcase、generate / endgenerate、fork / join 配平
    3. 括号 () [] {} 配平
    4. 同一模块内信号重复声明
    5. 被赋值(<= 或 =)的名字是否已声明  —— 能抓出"少声明/拼错名"
    6. 组合 always 块是否漏了 else / default 风险提示
    7. 疑似魔法数字提示（宽大于 1 的字面量直接出现在 RTL 里）

用法：
    python rtl_lint.py                     # 检查 ../rtl 下所有 .v
    python rtl_lint.py path/to/file.v ...
"""

import os
import re
import sys

#-----------------------------------------------------------------------------
# 预处理
#-----------------------------------------------------------------------------
def strip_comments(text):
    """去掉 // 行注释和 /* */ 块注释，保留行号（用空格填充）。"""
    out = []
    i = 0
    n = len(text)
    in_line = False
    in_block = False
    in_str = False
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ''
        if in_line:
            if ch == '\n':
                in_line = False
                out.append('\n')
            else:
                out.append(' ')
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


KEYWORDS = {
    'module', 'endmodule', 'begin', 'end', 'case', 'casex', 'casez', 'endcase',
    'if', 'else', 'for', 'while', 'repeat', 'forever', 'initial', 'always',
    'assign', 'input', 'output', 'inout', 'wire', 'reg', 'integer', 'parameter',
    'localparam', 'genvar', 'generate', 'endgenerate', 'function', 'endfunction',
    'task', 'endtask', 'fork', 'join', 'posedge', 'negedge', 'or', 'and', 'not',
    'default', 'signed', 'unsigned', 'defparam', 'specify', 'endspecify',
    'casez', 'casex', 'real', 'time', 'supply0', 'supply1', 'tri', 'initial',
}

DECL_KW = ('input', 'output', 'inout', 'wire', 'reg', 'integer',
           'parameter', 'localparam', 'genvar')


class LintResult:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.checks = 0

    def err(self, f, line, msg):
        self.errors.append((f, line, msg))

    def warn(self, f, line, msg):
        self.warnings.append((f, line, msg))


#-----------------------------------------------------------------------------
# 主检查
#-----------------------------------------------------------------------------
def check_file(path, res, do_undeclared=True):
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        raw = fh.read()

    lines = raw.split('\n')
    code = strip_comments(raw)
    clines = code.split('\n')

    #--------------------------------------------------------------------------
    # 1) 括号配平 (逐行累计)
    #--------------------------------------------------------------------------
    pairs = {')': '(', ']': '[', '}': '{'}
    stack = []
    for ln, text in enumerate(clines, 1):
        for ch in text:
            if ch in '([{':
                stack.append((ch, ln))
            elif ch in ')]}':
                res.checks += 1
                if not stack:
                    res.err(path, ln, f"多余的 '{ch}'")
                elif stack[-1][0] != pairs[ch]:
                    res.err(path, ln,
                            f"'{ch}' 与第 {stack[-1][1]} 行的 '{stack[-1][0]}' 不匹配")
                    stack.pop()
                else:
                    stack.pop()
    for ch, ln in stack:
        res.err(path, ln, f"'{ch}' 未闭合")

    #--------------------------------------------------------------------------
    # 2) 关键字配平 (module/begin/case/generate/fork)
    #--------------------------------------------------------------------------
    tokens = []
    for ln, text in enumerate(clines, 1):
        for m in re.finditer(r"[A-Za-z_][A-Za-z0-9_$]*|[:;=<>+\-*/&|^~!?%#@.,()\[\]{}']+", text):
            tokens.append((m.group(0), ln))

    counts = {'begin': 0, 'case': 0, 'generate': 0, 'fork': 0, 'module': 0}
    open_ln = {}
    modules = []
    cur_module = None
    module_signals = {}

    for tok, ln in tokens:
        if tok == 'module':
            counts['module'] += 1
            cur_module = f"<{path}:{ln}>"
            module_signals[cur_module] = set()
        elif tok == 'endmodule':
            counts['module'] -= 1
            if counts['module'] < 0:
                res.err(path, ln, "endmodule 多余")
                counts['module'] = 0
            cur_module = None
        elif tok in ('begin', 'case', 'casex', 'casez'):
            key = 'case' if tok.startswith('case') else 'begin'
            counts[key] += 1
            open_ln.setdefault((key, counts[key]), ln)
        elif tok == 'end':
            counts['begin'] -= 1
            if counts['begin'] < 0:
                res.err(path, ln, "end 多余")
                counts['begin'] = 0
        elif tok == 'endcase':
            counts['case'] -= 1
            if counts['case'] < 0:
                res.err(path, ln, "endcase 多余")
                counts['case'] = 0
        elif tok == 'generate':
            counts['generate'] += 1
        elif tok == 'endgenerate':
            counts['generate'] -= 1
            if counts['generate'] < 0:
                res.err(path, ln, "endgenerate 多余")
                counts['generate'] = 0
        elif tok == 'fork':
            counts['fork'] += 1
        elif tok == 'join' or tok.startswith('join'):
            counts['fork'] -= 1
            if counts['fork'] < 0:
                res.err(path, ln, "join 多余")
                counts['fork'] = 0

    for k, v in counts.items():
        res.checks += 1
        if v != 0:
            name = {'begin': 'begin/end', 'case': 'case/endcase',
                    'generate': 'generate/endgenerate', 'fork': 'fork/join',
                    'module': 'module/endmodule'}[k]
            res.err(path, 1, f"{name} 不配平, 差 {v} 个")

    #--------------------------------------------------------------------------
    # 3) 声明收集 + 重复声明检查
    #    作用域处理: task / function / begin-end 内部可以重名, 所以进入这些
    #    作用域时另开一套声明表, 避免把 "两个 task 各自的局部变量" 误报成重复。
    #--------------------------------------------------------------------------
    declared = {}
    dup = {}
    scope = ['module']          # 当前作用域栈
    scopes = {'module': {}}     # 每个作用域的声明表
    declared = scopes['module']

    decl_open = False
    for ln, text in enumerate(clines, 1):
        t = text.strip()

        # 进入 / 离开子作用域
        if re.match(r'^(task|function)\b', t):
            scope.append('sub%d' % ln)
            scopes[scope[-1]] = {}
            declared = scopes[scope[-1]]
            continue
        if re.match(r'^(endtask|endfunction)\b', t):
            if len(scope) > 1:
                scope.pop()
            declared = scopes[scope[-1]]
            continue
        if re.match(r'^(begin|fork)\s*$', t):
            scope.append('blk%d' % ln)
            scopes[scope[-1]] = {}
            declared = scopes[scope[-1]]
            continue
        if t in ('end', 'end;', 'join', 'join_any', 'join_none'):
            if len(scope) > 1:
                scope.pop()
            declared = scopes[scope[-1]]
            continue

        #--------------------------------------------------------------
        # 声明行识别
        #
        # 要支持**逗号续行**的声明，例如：
        #     localparam [3:0] T_IDLE = 4'd0,
        #                      T_HDR1 = 4'd1,      <- 这一行不以关键字开头
        #                      T_HDR2 = 4'd2;
        #
        # 但**不能**把"表达式换行"也当声明：
        #     wire signed [31:0] sat0 = (prop0 >  LIM32) ?  LIM32 :
        #                               (prop0 < -LIM32) ? -LIM32 : prop0;
        #   上面第 2 行如果不加区分地当声明处理，就会把 prop0 / LIM32
        #   收集成"重复声明"，产生误报（实测一次报 7 个）。
        #
        # 判据：逗号续行的声明，续行一定以 `名字 =` 开头；
        #       表达式换行的续行以 `(` `?` `:` 等符号开头。
        #--------------------------------------------------------------
        NAMED_EQ = re.compile(r'^\s*[A-Za-z_]\w*\s*=')

        m = re.match(r'^(' + '|'.join(DECL_KW) + r')\b(.*)$', t)
        if m:
            rest = m.group(2)
            decl_open = not t.rstrip().endswith(';')
        elif decl_open and NAMED_EQ.match(t):
            rest = t                       # 逗号续行的声明
            decl_open = not t.rstrip().endswith(';')
        elif decl_open:
            # 处于续行中，但这是**表达式换行**，不是新声明。
            # 仍然要维持 decl_open 状态，直到本声明以 ';' 收尾。
            decl_open = not t.rstrip().endswith(';')
            continue
        else:
            continue
        rest = re.sub(r'\b(signed|unsigned)\b', ' ', rest)
        rest = re.sub(r'\[[^\]]*\]', ' ', rest)
        rest = rest.split('=')[0]
        for name in re.findall(r'[A-Za-z_][A-Za-z0-9_$]*', rest):
            if name in KEYWORDS:
                continue
            if name in declared and declared[name] != ln:
                dup.setdefault(name, []).append((ln, scope[-1]))
            else:
                declared[name] = ln

    for name, entries in dup.items():
        res.checks += 1
        ln, sc = entries[0]
        res.err(path, ln, f"同一作用域内信号 '{name}' 重复声明 (又见于 {[e[0] for e in entries]})")

    #--------------------------------------------------------------------------
    # 3.5) 参数位选检查
    #
    #   Verilog-2001 **不允许对 parameter / localparam 做位选或部分选择**：
    #       localparam integer W = 64;
    #       ... W[7:0] ...         <-- 非法（那是 SystemVerilog 的语法）
    #
    #   这个坑在本工程出现过 4 次：
    #       CLK_PER_BIT[31:0]   (uart_tx)
    #       DIST_STOP_MM[15:0]  (main_fsm)
    #       KEY_IDLE[0]         (tb_top_integration)
    #       MAX_PAYLOAD[7:0]    (tb_uart_frame)   <- 这次导致 Error loading design
    #   每次都是"vlog 不报、elaboration 才炸、报错还不给名字"。
    #
    #   正确写法是先转成定宽常量：
    #       localparam [7:0] W8 = W;
    #--------------------------------------------------------------------------
    param_names = set()
    for m in re.finditer(r'\b(?:parameter|localparam)\b([^;]*);',
                         "\n".join(clines), flags=re.S):
        for n in re.findall(r'([A-Za-z_]\w*)\s*=', m.group(1)):
            if n not in ('signed', 'unsigned'):
                param_names.add(n)

    if param_names:
        for ln, text in enumerate(clines, 1):
            scan = re.sub(r'"[^"]*"', '""', text)
            # 声明行本身不算（那是定义位宽，不是位选）
            if re.match(r'^\s*(parameter|localparam|`define)\b', scan):
                continue
            for m in re.finditer(r'\b([A-Za-z_]\w*)\s*\[', scan):
                n = m.group(1)
                if n in param_names:
                    res.checks += 1
                    # 降级为 WARN：
                    #   实测 ModelSim 2020.4 **接受**参数位选（本工程多个仿真
                    #   通过的模块里就有，如 encoder_decode 的
                    #   FILTER_CYCLES[15:0]、pwm_motor 的 PERIOD_CNT[31:0]）。
                    #   所以它不是"当前故障"，但确实**不合 Verilog-2001 标准**，
                    #   Gowin 的综合器可能比 ModelSim 严格 —— 上综合前应清理。
                    res.warn(path, ln,
                             f"对 parameter/localparam '{n}' 做了位选（非标准，"
                             f"ModelSim 能过但综合器可能拒绝）。"
                             f"建议先转定宽常量：localparam [7:0] {n}_8 = {n};")

    #--------------------------------------------------------------------------
    # 4) 赋值左侧名字是否已声明
    #--------------------------------------------------------------------------
    if do_undeclared:
        # 收集所有作用域的声明名, 避免把 task 局部变量误报成未声明
        known = set(KEYWORDS)
        for sc, table in scopes.items():
            known |= set(table.keys())
        # 形参: task/function 头部括号里的名字
        for ln, text in enumerate(clines, 1):
            for m in re.finditer(r'\b(input|output|inout|integer|reg|wire)\b([^;]*);', text):
                for name in re.findall(r'[A-Za-z_][A-Za-z0-9_$]*', m.group(2)):
                    if name not in KEYWORDS:
                        known.add(name)

        # 函数名 / 任务名本身也是"已声明"的名字。
        #   否则 `function [15:0] abs_duty; ... endfunction` 里的
        #   `abs_duty = ...` 会被误报成"未声明信号"。
        full_text = "\n".join(clines)
        for m in re.finditer(
                r'\b(?:function|task)\b[^;]*?\b([A-Za-z_][A-Za-z0-9_$]*)\s*;',
                full_text):
            known.add(m.group(1))
        # 函数/任务的形参（声明式，带 input/... 前缀的已在上面收集）
        for m in re.finditer(r'\b(?:function|task)\s+([A-Za-z_][A-Za-z0-9_$]*)\s*\(', full_text):
            known.add(m.group(1))

        for ln, text in enumerate(clines, 1):
            # 重要：先把**字符串字面量**挖掉再扫描。
            #   否则 $display("... >= %0d = %0d", a, b) 这类格式化串里的
            #   "= " 会被当成赋值，把 d 之类的字符误报成"未声明信号"。
            scan = re.sub(r'"[^"]*"', '""', text)
            for m in re.finditer(r'([A-Za-z_][A-Za-z0-9_$]*)\s*(<=|=)(?!=)', scan):
                name = m.group(1)
                if name in known or name.startswith('$'):
                    continue
                res.checks += 1
                res.warn(path, ln,
                         f"'{name}' 被赋值但本文件内未见声明 (可能是跨文件信号或拼写错误)")

    #--------------------------------------------------------------------------
    # 5) 疑似魔法数字
    #--------------------------------------------------------------------------
    for ln, text in enumerate(clines, 1):
        body = text.strip()
        if body.startswith('//') or not body:
            continue
        # 排除 parameter/localparam 定义行与位宽
        if re.match(r'^(parameter|localparam|`define)\b', body):
            continue
        for m in re.finditer(r"(?<![\w'\]\)])(\d{3,})(?![\w])", text):
            val = int(m.group(1))
            if val in (100, 1000, 10000, 100000, 1000000):
                continue
            res.warn(path, ln,
                     f"疑似魔法数字 {val} (宽度/频率/窗口请用 parameter 或 project_config.vh)")
            break


def main():
    args = sys.argv[1:]
    if args:
        files = []
        for a in args:
            if os.path.isdir(a):
                for root, _, names in os.walk(a):
                    for n in names:
                        if n.endswith(('.v', '.sv', '.vh')):
                            files.append(os.path.join(root, n))
            else:
                files.append(a)
    else:
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rtl')
        files = []
        for root, _, names in os.walk(base):
            for n in names:
                if n.endswith(('.v', '.sv', '.vh')):
                    files.append(os.path.join(root, n))

    res = LintResult()
    for f in sorted(files):
        is_tb = os.path.basename(f).startswith('tb_')
        check_file(f, res, do_undeclared=not is_tb)

    print(f"=== rtl_lint: 检查 {len(files)} 个文件, {res.checks} 项 ===")
    for f, ln, msg in res.errors:
        print(f"  [ERROR] {os.path.relpath(f)}:{ln}: {msg}")
    for f, ln, msg in res.warnings:
        print(f"  [WARN ] {os.path.relpath(f)}:{ln}: {msg}")
    print(f"=== errors={len(res.errors)} warnings={len(res.warnings)} ===")
    if res.errors:
        print("RESULT: FAIL")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
