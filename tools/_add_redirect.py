# -*- coding: utf-8 -*-
"""点完 /h 的按钮后 302 跳回 /h，不再显示空白文字页。"""
import re

CR, LF = chr(13), chr(10)

p = r'arm/wifi_rc.py'
s = open(p, encoding='utf-8').read()

# 1) /h 页面所有 /cmd 链接加 ref=1
def add_ref(m):
    href = m.group(1)
    if 'ref=' in href:
        return m.group(0)
    sep = '&amp;' if '&amp;' in href else '&'
    return 'href="' + href + sep + 'ref=1"'

s = re.sub(r'href="(/cmd\?[^"]+)"', add_ref, s)

# 2) /cmd 收到 ref=1 -> 302
old = "                    body = 'st=%d sp=%d' % (steer, speed)"
new = (
    "                    if q.get('ref'):" + LF +
    "                        # 从 /h 页面点过来的：直接跳回控制台，不显示空白页" + LF +
    "                        hdr = ('HTTP/1.1 302 Found" + CR + LF + "Location: /h" + CR + LF + "'" + LF +
    "                               'Cache-Control: no-store" + CR + LF + "'" + LF +
    "                               'Content-Length: 0" + CR + LF + "Connection: close" + CR + LF + CR + LF + "')" + LF +
    "                        try:" + LF +
    "                            cli.send(hdr.encode())" + LF +
    "                        except Exception:" + LF +
    "                            pass" + LF +
    "                        body = ''" + LF +
    "                    else:" + LF +
    "                        body = 'st=%d sp=%d' % (steer, speed)"
)
assert old in s, 'body anchor'
s = s.replace(old, new, 1)

open(p, 'w', encoding='utf-8').write(s)

import ast
ast.parse(open(p, encoding='utf-8').read())
print('302 redirect ok, size =', len(s.encode('utf-8')), 'ref=1 count =', s.count('ref=1'))
