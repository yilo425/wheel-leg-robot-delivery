#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
add_result_logging.py -- 给所有 Testbench 加上"结果写文件"能力

为什么需要：
    ModelSim 批处理(-c)模式下 $display 的输出走 stdout，
    PowerShell 的 `*>` / 管道经常抓不到（缓冲问题），
    导致"仿真跑完了但看不到 PASS/FAIL"。

    $fwrite 直接写文件不经过 ModelSim 的输出系统，最可靠。
    这样即使控制台什么都看不到，结果文件里也有完整记录。

本脚本做的事（对每个 tb_*.v）：
    1) 在 `integer checks = 0;` 后面插入 log_fd 声明 + 打开文件的 initial
    2) 在 task check 里补 $fwrite 两份输出（OK / FAIL）
    3) 把结尾的 RESULT 块改成同时 $display 和 $fwrite

用法：
    python add_result_logging.py          # 就地修改 ../sim/tb_*.v
    python add_result_logging.py --dry    # 只看会改什么，不写文件
"""

import os
import re
import sys

DRY = '--dry' in sys.argv


def patch(path):
    with open(path, 'r', encoding='utf-8') as fh:
        text = fh.read()

    base = os.path.basename(path)[:-2]          # tb_xxx
    logname = base.replace('tb_', '') + '_result.txt'
    changed = []

    #------------------------------------------------------------------
    # 1) log_fd 声明 + 打开文件
    #------------------------------------------------------------------
    if 'log_fd' not in text:
        m = re.search(r'^(\s*)integer\s+checks\s*=\s*0\s*;\s*$', text, re.M)
        if not m:
            return False, [f'找不到 "integer checks = 0;"，跳过']
        indent = m.group(1)
        insert = (
            f"\n\n{indent}// ---- 结果同时写文件 ----\n"
            f"{indent}//   ModelSim -c 模式下 $display 走 stdout，管道常抓不到，\n"
            f"{indent}//   用 $fwrite 直接落盘最可靠。结果文件: {logname}\n"
            f"{indent}integer log_fd;\n"
            f"{indent}initial begin\n"
            f"{indent}    log_fd = $fopen(\"{logname}\", \"w\");\n"
            f"{indent}    if (log_fd == 0) $display(\"[warn] 无法创建 {logname}\");\n"
            f"{indent}end"
        )
        text = text[:m.end()] + insert + text[m.end():]
        changed.append('插入 log_fd 声明')

    #------------------------------------------------------------------
    # 2) task check 里补 $fwrite
    #------------------------------------------------------------------
    # OK 分支
    if '$fwrite(log_fd, "  [ OK ]' not in text:
        pat_ok = re.compile(
            r'(\$display\(\s*"  \[ OK \] %0s : %0d"\s*,\s*name\s*,\s*got\s*\)\s*;)')
        if pat_ok.search(text):
            text = pat_ok.sub(
                lambda m: m.group(1) +
                f'\n                $fwrite(log_fd, "  [ OK ] %0s : %0d\\n", name, got);',
                text, count=1)
            changed.append('check: 补 OK 的 $fwrite')

    # FAIL 分支
    if '$fwrite(log_fd, "  [FAIL]' not in text:
        pat_fail = re.compile(
            r'(\$display\(\s*"  \[FAIL\] %0s : got=%0d exp=%0d"\s*,\s*name\s*,\s*got\s*,\s*exp\s*\)\s*;)')
        if pat_fail.search(text):
            text = pat_fail.sub(
                lambda m: m.group(1) +
                f'\n                $fwrite(log_fd, "  [FAIL] %0s : got=%0d exp=%0d\\n", name, got, exp);',
                text, count=1)
            changed.append('check: 补 FAIL 的 $fwrite')

    #------------------------------------------------------------------
    # 3) 结尾 RESULT 块
    #------------------------------------------------------------------
    if '$fwrite(log_fd, "RESULT: PASS' not in text:
        # 匹配三行: $display 汇总 / if PASS / else FAIL
        pat_res = re.compile(
            r'(\$display\("=== ' + re.escape(base) + r': %0d checks, %0d errors ===", checks, errors\);\s*\n)'
            r'\s*if \(errors == 0\) \$display\("RESULT: PASS"\);\s*\n'
            r'\s*else\s+\$display\("RESULT: FAIL"\);')
        def repl(m):
            ind = ' ' * 8
            return (m.group(1) +
                    f'{ind}$fwrite(log_fd, "=== ' + base + ': %0d checks, %0d errors ===\\n", checks, errors);\n'
                    f'{ind}if (errors == 0) begin\n'
                    f'{ind}    $display("RESULT: PASS");\n'
                    f'{ind}    $fwrite(log_fd, "RESULT: PASS\\n");\n'
                    f'{ind}end else begin\n'
                    f'{ind}    $display("RESULT: FAIL");\n'
                    f'{ind}    $fwrite(log_fd, "RESULT: FAIL\\n");\n'
                    f'{ind}end\n'
                    f'{ind}$fclose(log_fd);')
        text, n = pat_res.subn(repl, text, count=1)
        if n:
            changed.append('补 RESULT 块的 $fwrite / $fclose')

    #------------------------------------------------------------------
    # 4) timeout 分支
    #------------------------------------------------------------------
    if '$fwrite(log_fd, "RESULT: FAIL (timeout)' not in text:
        pat_to = re.compile(r'(\$display\("RESULT: FAIL \(timeout\)"\);\s*\n)')
        def repl_to(m):
            return (m.group(1) +
                    '        $fwrite(log_fd, "RESULT: FAIL (timeout)\\n");\n'
                    '        $fclose(log_fd);\n')
        text, n = pat_to.subn(repl_to, text, count=1)
        if n:
            changed.append('补 timeout 分支的 $fwrite / $fclose')

    if not changed:
        return False, ['无需修改']

    if not DRY:
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
    return True, changed


def main():
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim')
    files = sorted(
        os.path.join(base, n) for n in os.listdir(base)
        if n.startswith('tb_') and n.endswith('.v')
    )

    if not files:
        print(f'在 {base} 下没找到 tb_*.v')
        return 1

    print(f'=== add_result_logging {"(DRY RUN)" if DRY else ""} ===')
    total = 0
    for f in files:
        ok, notes = patch(f)
        flag = 'MODIFIED' if ok else 'skipped '
        print(f'  [{flag}] {os.path.basename(f)}')
        for n in notes:
            print(f'             - {n}')
        if ok:
            total += 1

    print(f'=== 共处理 {len(files)} 个文件, 修改 {total} 个 ===')
    if not DRY:
        print('请检查改动后再重新编译。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
