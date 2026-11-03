#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
check_encoding.py -- 文件编码 / BOM / 换行符合规检查

=============================================================================
为什么需要这个
=============================================================================
本工程踩过三类"编码导致的怪问题"，共同点是**报错信息完全看不出真正原因**：

1) **含中文的 .ps1 没有 UTF-8 BOM**
   PowerShell 5.x 默认按系统 ANSI 代码页（中文 Windows 是 GBK）读 .ps1。
   UTF-8 的中文被当 GBK 解码 -> 乱码 -> 语法错误。
   实测症状：一个语法完全正确的脚本报出 12 个 "缺少右 }"，
   而报错位置指向的是**中文注释行**，完全误导。

   根因往往是：某个编辑器/工具写文件时**丢掉了 BOM**（本工程的 edit 工具
   就会丢），文件本身内容没变，但 PowerShell 再也读不对了。

2) **.do 文件含非 ASCII 字符**
   ModelSim 的 Tcl 按 ANSI 代码页读脚本。中文会被解成乱码；
   更糟的是如果某个字节被解成行尾反斜杠，会导致**行继续**，
   把下一行当命令执行。
   本工程的 .do 全部由 emit_dofiles.ps1 生成，生成时强制 ASCII。

3) **换行符不统一**
   没有 .gitattributes 时，不同机器的 core.autocrlf 设置不同，
   会让每次提交都显示"整个文件都改了"，git log 完全没法看。

=============================================================================
检查规则
=============================================================================
  *.ps1   含中文 -> 必须有 UTF-8 BOM
  *.ps1   纯 ASCII -> BOM 可有可无（不报错）
  *.do    必须纯 ASCII（不含任何 >127 的字节）
  所有文本文件 -> 必须是合法 UTF-8（或纯 ASCII）

用法:
  python check_encoding.py <目录> [<目录> ...]
  python check_encoding.py .                    # 检查当前目录
  python check_encoding.py ..\rtl ..\sim ..\doc ..\arm ..\handoff

退出码: 0 = 全部合规, 1 = 发现问题
"""

import os
import sys

# 检查这些扩展名
TEXT_EXT = {'.ps1', '.py', '.v', '.vh', '.md', '.cst', '.sdc', '.do',
            '.bat', '.cmd', '.json', '.yml', '.yaml', '.txt', '.tcl'}
# .do 必须是纯 ASCII
MUST_ASCII_EXT = {'.do'}
# 含中文就必须有 BOM 的扩展名
NEED_BOM_EXT = {'.ps1', '.bat', '.cmd'}

SKIP_DIRS = {'.git', 'work', '__pycache__', 'node_modules', '.vscode',
             'impl', 'pnr', 'tmp', '.fse'}

# 生成产物（不是源文件）—— 按**本机编码**写出，天然不是 UTF-8。
#
#   典型：ModelSim 把检查名里的中文按 GBK 写进 *_result.txt，
#   于是这些文件不是合法 UTF-8，检查它们只会产生误报
#   （比如"仿真全过，但编码检查 FAIL"，很容易让人误以为代码有问题）。
#
#   注意：真源文件（.v/.py/.md/.do/.cst）**不跳过** —— 那些才是要管的。
#   这里跳过的全是"跑一次就会重新生成的"东西。
SKIP_FILE_SUFFIX = ('_result.txt', '.log', '.vcd')
SKIP_FILE_EXACT  = ('transcript', 'vsim.wlf', 'modelsim.ini')


def has_bom(raw: bytes) -> bool:
    return len(raw) >= 3 and raw[0] == 0xEF and raw[1] == 0xBB and raw[2] == 0xBF


def has_cjk(text: str) -> bool:
    return any('\u4e00' <= ch <= '\u9fff' for ch in text)


def scan(root: str):
    problems = []
    checked = 0

    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if fn in SKIP_FILE_EXACT or fn.endswith(SKIP_FILE_SUFFIX):
                continue
            ext = os.path.splitext(fn)[1].lower()
            if ext not in TEXT_EXT:
                continue
            path = os.path.join(dirpath, fn)
            rel = os.path.relpath(path, root).replace('\\', '/')
            checked += 1

            with open(path, 'rb') as f:
                raw = f.read()

            # ---- 1) 纯 ASCII 检查（.do）----
            if ext in MUST_ASCII_EXT:
                bad = sum(1 for b in raw if b > 127)
                if bad:
                    # 找出第一个出问题的行
                    line = 1
                    for b in raw:
                        if b == 0x0A:
                            line += 1
                        elif b > 127:
                            break
                    problems.append(
                        (rel, f'含 {bad} 个非 ASCII 字节（.do 必须纯 ASCII）'
                              f'，首个出现在第 {line} 行附近'))
                continue

            # ---- 2) 必须是合法 UTF-8 ----
            try:
                text = raw.decode('utf-8')
            except UnicodeDecodeError as e:
                problems.append((rel, f'不是合法 UTF-8（字节偏移 {e.start}）'
                                      f' —— 可能是 GBK 存的？'))
                continue

            # ---- 3) 含中文的 .ps1 必须有 BOM ----
            if ext in NEED_BOM_EXT and has_cjk(text) and not has_bom(raw):
                problems.append(
                    (rel, '含中文但没有 UTF-8 BOM —— PowerShell 5.x 会按 GBK '
                          '误读，报出莫名其妙的语法错误。'
                          '修复：用「UTF-8 带 BOM」重新保存'))

    return checked, problems


def main():
    roots = sys.argv[1:] or ['.']

    print('=== check_encoding: 编码 / BOM / ASCII 合规检查 ===')
    total_checked = 0
    all_problems = []

    for r in roots:
        if not os.path.isdir(r):
            print(f'  [跳过] 不是目录: {r}')
            continue
        n, probs = scan(r)
        total_checked += n
        for rel, msg in probs:
            all_problems.append((r, rel, msg))

    print(f'  扫描 {len(roots)} 个目录, 共 {total_checked} 个文本文件')

    if not all_problems:
        print()
        print('  全部合规')
        print('    - .do 均为纯 ASCII')
        print('    - 含中文的 .ps1 均有 UTF-8 BOM')
        print('    - 所有文本文件均为合法 UTF-8')
        print()
        print('RESULT: PASS')
        return 0

    print()
    for root, rel, msg in all_problems:
        print(f'  [ERROR] {root}/{rel}')
        print(f'          {msg}')
    print()
    print(f'共 {len(all_problems)} 处问题')
    print('RESULT: FAIL')
    return 1


if __name__ == '__main__':
    sys.exit(main())
