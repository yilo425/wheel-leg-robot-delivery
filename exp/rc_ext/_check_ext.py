# -*- coding: utf-8 -*-
"""
_check_ext.py -- 交付自检（PC 上跑，CPython，不连板子、不连串口）

两段：
  A. 静态检查：README 第四节/第五节的清单 + 产物硬性要求
  B. 行为仿真：把 wifi_rc_ext 当普通模块 import 进来，用**假时钟 + 假串口**
     驱动它的状态机 / HTTP 处理，验证"自动模式不发 RC 帧""手动 20Hz"
     "转弯 360ms 自动停""/cmd 无论如何都回响应""页面零 JS 且 & 全转义"
     "Content-Length 等于字节数""_send_all 不会被截断"。

用法：
    python _check_ext.py
退出码 0 = 全绿。
"""

import ast
import os
import re
import sys
import hashlib

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, 'wifi_rc_ext.py')
BASELINE = os.path.join(HERE, 'wifi_rc_baseline.py')
PROTO = os.path.join(HERE, 'uart_proto.py')
ARM_RC = os.path.join(HERE, '..', '..', 'arm', 'wifi_rc.py')

RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok), detail))
    return ok


#=============================================================================
# 0. 假时钟（MicroPython 的 time.ticks_* 在 CPython 里没有）
#=============================================================================
import time as _time

CLOCK = [0]


def _ticks_ms():
    return CLOCK[0]


def _ticks_diff(a, b):
    return a - b


def _sleep_ms(ms):
    CLOCK[0] += int(ms)


_time.ticks_ms = _ticks_ms
_time.ticks_diff = _ticks_diff
_time.sleep_ms = _sleep_ms


#=============================================================================
# 1. 假串口 / 假 socket
#=============================================================================
class FakeUart(object):
    def __init__(self):
        self.writes = []

    def write(self, b):
        self.writes.append(bytes(b))
        return len(b)

    def frames(self, ftype):
        return [f for f in self.writes if len(f) >= 4 and f[3] == ftype]

    def clear(self):
        self.writes = []


class FakeCli(object):
    """假客户端。chunk>0 时模拟"一次 send 只发一部分"，用来验证 _send_all。"""

    def __init__(self, req=b'', chunk=0, fail=False):
        self.req = req
        self.chunk = chunk
        self.fail = fail
        self.sent = b''
        self.closed = False

    def settimeout(self, t):
        pass

    def recv(self, n):
        r = self.req
        self.req = b''
        return r

    def send(self, b):
        if self.fail:
            raise OSError('boom')
        if self.chunk:
            b = bytes(b[:self.chunk])
        self.sent += b
        return len(b)

    def close(self):
        self.closed = True


class FakeSrv(object):
    def __init__(self, cli=None):
        self.cli = cli

    def accept(self):
        if self.cli is None:
            raise OSError('EAGAIN')
        c = self.cli
        self.cli = None
        return c, ('192.168.169.2', 50000)


#=============================================================================
# 2. 帧解析小工具
#=============================================================================
def parse_http(resp):
    """拆出 (状态行, 头部字典, 体 bytes)。"""
    head, _, body = resp.partition(b'\r\n\r\n')
    lines = head.split(b'\r\n')
    status = lines[0].decode('utf-8', 'ignore') if lines else ''
    hdr = {}
    for ln in lines[1:]:
        if b':' in ln:
            k, v = ln.split(b':', 1)
            hdr[k.strip().lower().decode('utf-8', 'ignore')] = v.strip().decode('utf-8', 'ignore')
    return status, hdr, body


def rc_of(f):
    """从 RC 帧里取 (steer, speed_byte, seq)。"""
    steer = f[6] - 256 if f[6] > 127 else f[6]
    return steer, f[7], f[8]


def cfg_of(f):
    """从 CONFIG 帧里取 (param_id, value)。"""
    return f[6], f[8]


#=============================================================================
# A. 静态检查
#=============================================================================
raw = open(TARGET, 'rb').read()
src = raw.decode('utf-8')

# A1 ast.parse
try:
    tree = ast.parse(src)
    check('ast.parse 通过', True, '%d 个顶层语句' % len(tree.body))
except SyntaxError as e:
    check('ast.parse 通过', False, '%r' % (e,))
    tree = None

# A2 换行符
crlf = raw.count(b'\r\n')
cr = raw.count(b'\r')
check('CRLF 数量 == 0', crlf == 0, 'CRLF=%d' % crlf)
check('完全不含 \\r', cr == 0, 'CR=%d' % cr)

# A3 不含 memoryview
mv = raw.count(b'memoryview')
check("不含 'memoryview'", mv == 0, '出现 %d 次' % mv)

# A4 含 _send_all
check("含 '_send_all'", b'_send_all' in raw, '出现 %d 次' % raw.count(b'_send_all'))

# A5 /h 的 <a href> 里没有裸 &
BARE_AMP = re.compile(rb'&(?!(?:amp|lt|gt|quot|apos|#\d+|#x[0-9a-fA-F]+);)')
SRC_HREF = re.findall(rb'<a [^>]*href="([^"]*)"', raw)
bad_href_src = [h for h in SRC_HREF if BARE_AMP.search(h)]
check('源码里 <a href> 无裸 &（应为 &amp;）', not bad_href_src,
      '检查了 %d 个字面 href' % len(SRC_HREF))
# /h 的绝大部分 href 是 _btn() 生成的，扫它的第 2 个参数（href）
BTN_HREFS = re.findall(rb"_btn\([^,]+,\s*'([^']*)'", raw)
bad_btn = [h.decode('utf-8', 'ignore') for h in BTN_HREFS if BARE_AMP.search(h)]
check('源码里 _btn() 的 href 无裸 &', not bad_btn,
      '检查了 %d 个 _btn href，坏的 %d 个 %r'
      % (len(BTN_HREFS), len(bad_btn), bad_btn[:2]))

# A6 含 gc.collect
check("含 'gc.collect'", b'gc.collect' in raw, '')

# A7 含 TURN_MS 且 == 360
m = re.search(rb'^TURN_MS\s*=\s*(\d+)', raw, re.M)
check("含 'TURN_MS' 且 == 360", bool(m) and int(m.group(1)) == 360,
      'TURN_MS=%s' % (m.group(1).decode() if m else 'missing'))

# A8 文件大小 30000~45000
n = len(raw)
check('文件大小 30000~45000 字节', 30000 <= n <= 45000, '%d 字节' % n)

# A9 DEBUG_HTTP = False
check('DEBUG_HTTP = False', b'DEBUG_HTTP = False' in raw, '')

# A10 禁用模块
for bad in (b'import asyncio', b'import typing', b'from typing', b'import dataclasses',
            b'import pathlib', b'import threading'):
    check('不含 %r' % bad.decode(), bad not in raw, '')

# A11 基线与协议库未被改动（大小 + sha256）
def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()[:16]


for p, want in ((BASELINE, 36220), (PROTO, 23963)):
    if os.path.exists(p):
        sz = os.path.getsize(p)
        check('未被改动: %s (%d 字节)' % (os.path.basename(p), want), sz == want,
              '实际 %d, sha256=%s' % (sz, sha(p)))
if os.path.exists(ARM_RC):
    # 本任务绝不写生产文件。这里不断言字节数（它可能被别人合法修改），
    # 只断言本实验的代码没有混进去。
    _arm = open(ARM_RC, 'rb').read()
    check('arm/wifi_rc.py 未混入本实验代码',
          b'wifi_rc_ext' not in _arm and b'TURN_MS' not in _arm
          and b'wifi_rc_baseline' not in _arm,
          '%d 字节, sha256=%s（本任务未写入）' % (len(_arm), sha(ARM_RC)))

# A12 正常 LF 文本文件（能被行数统计，没有 NUL）
check('无 NUL 字节', b'\x00' not in raw, '行数 %d' % raw.count(b'\n'))


#=============================================================================
# B. 行为仿真
#=============================================================================
sys.path.insert(0, HERE)
import wifi_rc_ext as mod
import uart_proto as up

check('模块可 import（无硬件依赖泄漏）', True, 'VERSION=%s' % mod.VERSION)
check('VERSION 与基线不同', mod.VERSION != '2026-10-30-r9', mod.VERSION)


def fresh():
    """新建一份状态 + 空串口，时钟归零。"""
    CLOCK[0] = 0
    u = FakeUart()
    st = mod._new_state()
    return st, u


def run_ticks(st, u, ms, step=1):
    """按 step 毫秒推进时钟，每步跑一次 _tick。返回发出的 RC 帧数。"""
    sent = 0
    t_end = CLOCK[0] + ms
    while CLOCK[0] < t_end:
        CLOCK[0] += step
        sent += mod._tick(st, u, up, CLOCK[0])
    return sent


#---- B1 解析查询串（无白名单）----
q = mod._parse_query('/cmd?m=1&a=1&foo=9&bar=abc&ref=1')
check('_parse_query 无键名白名单',
      q.get('foo') == 9 and q.get('bar') == 'abc' and q.get('m') == 1 and q.get('ref') == 1,
      repr(q))

#---- B2 自动模式：一帧 RC 都不发 ----
st, u = fresh()
check('默认模式 = 自动', st['mode'] == mod.MODE_AUTO, 'mode=%d' % st['mode'])
n_auto = run_ticks(st, u, 1000)
rc_auto = len(u.frames(up.TYPE_RC_CMD))
check('自动模式 1 秒内 RC 帧 == 0', rc_auto == 0, 'RC=%d, _tick 返回 %d' % (rc_auto, n_auto))

#---- B3 自动模式下动作被拒 ----
mod._apply_cmd({'a': 1}, st, u, up, CLOCK[0])
check('自动模式拒绝动作指令',
      st['action'] == mod.ACT_STOP and '手动' in st['msg'], 'action=%s msg=%s' % (st['action'], st['msg']))

#---- B4 切手动：发 CONFIG 0x20=1 ----
mod._apply_cmd({'m': 1}, st, u, up, CLOCK[0])
modes = [cfg_of(f) for f in u.frames(up.TYPE_CONFIG) if cfg_of(f)[0] == 0x20]
check('切手动 -> CONFIG 0x20=1', any(v == 1 for _, v in modes) and st['mode'] == mod.MODE_MAN,
      'CONFIG 0x20 序列 %r' % (modes,))

#---- B5 手动 20Hz ----
u.clear()
CLOCK[0] += 10
n20 = run_ticks(st, u, 1000)
rc20 = u.frames(up.TYPE_RC_CMD)
check('手动模式 1 秒 ≈ 20 帧（18~22）', 18 <= len(rc20) <= 22, '实际 %d 帧' % len(rc20))

#---- B6 前进：保持 + steer/speed 正确 ----
st['action'] = mod.ACT_STOP
u.clear()
mod._apply_cmd({'a': 1}, st, u, up, CLOCK[0])
run_ticks(st, u, 500)
fwd = [rc_of(f) for f in u.frames(up.TYPE_RC_CMD)]
check('前进：steer=0 speed=40 且一直保持',
      st['action'] == mod.ACT_FWD and fwd and all(s == 0 and sp == 40 for s, sp, _ in fwd),
      '动作=%s, %d 帧, 样例 %r' % (st['action'], len(fwd), fwd[:3]))

#---- B7 后退：bit7 保留（0x80|40 = 0xA8）----
u.clear()
mod._apply_cmd({'a': 2}, st, u, up, CLOCK[0])
run_ticks(st, u, 500)
back = [rc_of(f) for f in u.frames(up.TYPE_RC_CMD)]
check('后退：speed 字节 = 0x80|40 = 168',
      back and all(sp == 168 for _, sp, _ in back),
      '%d 帧, 样例 %r' % (len(back), back[:3]))

#---- B8 左转：steer=-100 speed=40，360ms 后自动停 ----
u.clear()
t0 = CLOCK[0]
mod._apply_cmd({'a': 3}, st, u, up, t0)
turning = [rc_of(f) for f in u.frames(up.TYPE_RC_CMD)]
steer_ok = turning and turning[0][0] == -100 and turning[0][1] == 40
# 推进到自动停
while st['action'] == mod.ACT_LEFT and CLOCK[0] - t0 < 5000:
    CLOCK[0] += 1
    mod._tick(st, u, up, CLOCK[0])
elapsed = CLOCK[0] - t0
check('左转：steer=-100 speed=40', steer_ok, '首帧 %r' % (turning[:1],))
check('左转：360ms 后自动停（360~370）', 360 <= elapsed <= 370,
      '实际 %d ms' % elapsed)
last = rc_of(u.frames(up.TYPE_RC_CMD)[-1])
check('转停瞬间立刻补了停车帧（steer=0 speed=0）',
      st['action'] == mod.ACT_STOP and last[0] == 0 and last[1] == 0,
      '最后一帧 %r, 动作=%s' % (last, st['action']))

#---- B9 右转 steer=+100 ----
u.clear()
mod._apply_cmd({'a': 4}, st, u, up, CLOCK[0])
r0 = rc_of(u.frames(up.TYPE_RC_CMD)[0])
check('右转：steer=+100 speed=40', r0[0] == 100 and r0[1] == 40, repr(r0))
while st['action'] == mod.ACT_RIGHT:
    CLOCK[0] += 1
    mod._tick(st, u, up, CLOCK[0])

#---- B10 停止 ----
u.clear()
mod._apply_cmd({'a': 0}, st, u, up, CLOCK[0])
s0 = rc_of(u.frames(up.TYPE_RC_CMD)[0])
check('停止：立刻发 steer=0 speed=0', s0[0] == 0 and s0[1] == 0 and st['action'] == mod.ACT_STOP,
      repr(s0))

#---- B11 CONFIG 每 2 秒重发（模式 + 站立高度）----
u.clear()
CLOCK[0] += 10
run_ticks(st, u, 4600)
cfgs = [cfg_of(f) for f in u.frames(up.TYPE_CONFIG)]
p20 = [v for p, v in cfgs if p == 0x20]
p21 = [v for p, v in cfgs if p == 0x21]
check('4.6 秒内 0x20 与 0x21 各重发 ≥ 2 次',
      len(p20) >= 2 and len(p21) >= 2, '0x20 值 %r, 0x21 值 %r' % (p20, p21))
# 顺便：手动全程仍在发 RC
check('重发期间 RC 帧仍在发', len(u.frames(up.TYPE_RC_CMD)) >= 80,
      '%d 帧' % len(u.frames(up.TYPE_RC_CMD)))

#---- B12 升降 ----
u.clear()
mod._apply_cmd({'ht': 1}, st, u, up, CLOCK[0])
h1 = [cfg_of(f) for f in u.frames(up.TYPE_CONFIG) if cfg_of(f)[0] == 0x21]
mod._apply_cmd({'ht': 0}, st, u, up, CLOCK[0])
check('降低 -> CONFIG 0x21=1、升高 -> 0', any(v == 1 for _, v in h1) and st['stance'] == 0,
      '0x21 序列 %r, stance=%d' % ([v for _, v in h1], st['stance']))

#---- B13 切自动：先补停车帧，再发 0x20=0，之后停发 RC ----
u.clear()
mod._apply_cmd({'m': 0}, st, u, up, CLOCK[0])
seq = u.writes
idx_cfg0 = None
idx_last_rc = None
for i, f in enumerate(seq):
    if f[3] == up.TYPE_CONFIG and f[6] == 0x20 and f[8] == 0:
        if idx_cfg0 is None:
            idx_cfg0 = i
    if f[3] == up.TYPE_RC_CMD:
        idx_last_rc = i
check('切自动：停车帧在 0x20=0 之前发出',
      idx_cfg0 is not None and idx_last_rc is not None and idx_last_rc < idx_cfg0,
      'RC@{0} -> CFG0@{1}'.format(idx_last_rc, idx_cfg0))
check('切自动：动作被清零', st['action'] == mod.ACT_STOP and st['mode'] == mod.MODE_AUTO,
      'action=%s mode=%d' % (st['action'], st['mode']))

u.clear()
run_ticks(st, u, 3000)
check('自动模式 3 秒内 RC 帧 == 0（不抢 FPGA 控制）',
      len(u.frames(up.TYPE_RC_CMD)) == 0,
      '实际 %d 帧' % len(u.frames(up.TYPE_RC_CMD)))

#---- B14 软超时默认关闭 ----
check('CMD_TIMEOUT_MS 默认 0（点一下保持，不靠 600ms 超时）',
      mod.CMD_TIMEOUT_MS == 0, 'CMD_TIMEOUT_MS=%d' % mod.CMD_TIMEOUT_MS)

#---- B15 /h 页面：零 JS、& 全转义、服务端状态、高亮 ----
page_bad = []
pages = {}
for mode in (mod.MODE_AUTO, mod.MODE_MAN):
    for act in (mod.ACT_STOP, mod.ACT_FWD, mod.ACT_BACK, mod.ACT_LEFT, mod.ACT_RIGHT):
        st['mode'] = mode
        st['action'] = act
        for stance in (0, 1):
            st['stance'] = stance
            html = mod._page_h(st)
            pages[(mode, act, stance)] = html
            if '<script' in html.lower():
                page_bad.append('有 script 标签')
            if 'onclick' in html.lower() or 'javascript:' in html.lower():
                page_bad.append('有内联事件/伪协议')
            if '<form' in html.lower():
                page_bad.append('有 form')
            hrefs = re.findall(r'<a [^>]*href="([^"]*)"', html)
            for h in hrefs:
                if BARE_AMP.search(h.encode('utf-8')):
                    page_bad.append('裸 & : ' + h)
            # 裸 & 出现在正文里也不行
            if BARE_AMP.search(html.encode('utf-8')):
                page_bad.append('正文有裸 &')
check('/h 零 JS：20 种状态都无 script/onclick/form', not page_bad,
      '; '.join(sorted(set(page_bad))[:4]))

h_man = pages[(mod.MODE_MAN, mod.ACT_FWD, 0)]
h_auto = pages[(mod.MODE_AUTO, mod.ACT_STOP, 1)]
check('状态栏服务端渲染：手动/前进/高位',
      '手动' in h_man and '前进' in h_man and '高位' in h_man and '20Hz' in h_man,
      '长度 %d 字节' % len(h_man.encode('utf-8')))
check('状态栏服务端渲染：自动/停止/低位/停发',
      '自动' in h_auto and '低位' in h_auto and '停发' in h_auto,
      '长度 %d 字节' % len(h_auto.encode('utf-8')))
check('高亮激活按钮（class 含 on）',
      'href="/cmd?a=1&amp;ref=1"' in h_man and
      re.search(r'class="b green on" href="/cmd\?a=1&amp;ref=1"', h_man) is not None,
      '')
check('自动模式下运动按钮置灰（dim）',
      re.search(r'class="b green dim" href="/cmd\?a=1&amp;ref=1"', h_auto) is not None, '')

#---- B16 HTTP: /h 的 Content-Length == 字节数 ----
st['mode'] = mod.MODE_MAN
st['action'] = mod.ACT_FWD
cli = FakeCli(b'GET /h HTTP/1.1\r\nHost: x\r\n\r\n')
mod._http_one(FakeSrv(cli), st, u, up)
status, hdr, body = parse_http(cli.sent)
clen = int(hdr.get('content-length', -1))
check('/h: Content-Length == body 字节数', clen == len(body) and clen > 0,
      '声明 %d, 实际 %d, 状态 %s' % (clen, len(body), status))
check('/h 返回 200 + no-store', '200' in status and hdr.get('cache-control') == 'no-store', status)

#---- B17 /cmd 302 且回响应 ----
cli = FakeCli(b'GET /cmd?m=0&ref=1 HTTP/1.1\r\nHost: x\r\n\r\n')
mod._http_one(FakeSrv(cli), st, u, up)
status, hdr, _ = parse_http(cli.sent)
check('/cmd 回 302 + Location: /h', '302' in status and hdr.get('location') == '/h', status)
check('/cmd 处理完会关连接', cli.closed, '')

#---- B18 /cmd 参数全是垃圾也回响应、不抛异常 ----
for bad_path in (b'/cmd?m=xxx&a=yyy&ht=zzz&sp=qqq',
                 b'/cmd', b'/cmd?', b'/cmd?a=999&sp=-5&ht=abc',
                 b'/cmd?m=&a=&ht=&sp=&ref='):
    cli = FakeCli(b'GET ' + bad_path + b' HTTP/1.1\r\nHost: x\r\n\r\n')
    ok = True
    try:
        mod._http_one(FakeSrv(cli), st, u, up)
    except Exception as e:
        ok = False
    got = cli.sent.startswith(b'HTTP/1.1 302') or cli.sent.startswith(b'HTTP/1.1 200')
    check('/cmd 垃圾参数仍回响应: %s' % bad_path.decode(), ok and got,
          cli.sent.split(b'\r\n')[0].decode('utf-8', 'ignore'))

#---- B19 _apply_cmd 内部抛异常时也回响应 ----
_orig_apply = mod._apply_cmd
try:
    def _boom(*a, **k):
        raise RuntimeError('故意炸')
    mod._apply_cmd = _boom
    st['msg'] = ''
    cli = FakeCli(b'GET /cmd?a=1 HTTP/1.1\r\n\r\n')
    mod._http_one(FakeSrv(cli), st, u, up)
    check('/cmd 内部异常仍回 302（连接不掐断）',
          cli.sent.startswith(b'HTTP/1.1 302'), cli.sent.split(b'\r\n')[0].decode('utf-8', 'ignore'))
finally:
    mod._apply_cmd = _orig_apply

#---- B20 302 发不出去时退回 200 ----
cli = FakeCli(b'GET /cmd?a=0 HTTP/1.1\r\n\r\n')
_orig_send_all = mod._send_all
try:
    calls = [0]

    def _half(sock, data):
        # 第一次调用（302 头）失败，后面正常
        calls[0] += 1
        if calls[0] == 1:
            raise OSError('send failed')
        return _orig_send_all(sock, data)
    mod._send_all = _half
    mod._http_one(FakeSrv(cli), st, u, up)
    check('302 发送失败时回 200 保底页',
          cli.sent.startswith(b'HTTP/1.1 200'), cli.sent.split(b'\r\n')[0].decode('utf-8', 'ignore'))
finally:
    mod._send_all = _orig_send_all

#---- B21 空请求：直接关，不回页面（README 第 12 条）----
cli = FakeCli(b'')
mod._http_one(FakeSrv(cli), st, u, up)
check('空请求不回页面', cli.sent == b'' and cli.closed, 'sent=%d 字节' % len(cli.sent))

#---- B22 _send_all 不截断（每次只发 7 字节）----
body = mod._page_h(st).encode('utf-8')
cli = FakeCli(b'GET /h HTTP/1.1\r\n\r\n', chunk=7)
total = mod._send_all(cli, body)
check('_send_all 分片发送不丢字节', total == len(body) and cli.sent == body,
      '发出 %d / %d' % (len(cli.sent), len(body)))
check('_send_all 接受 str 入参', mod._send_all(FakeCli(), 'abc') == 3, '')

#---- B23 /ver 诊断 ----
cli = FakeCli(b'GET /ver HTTP/1.1\r\n\r\n')
mod._http_one(FakeSrv(cli), st, u, up)
status, hdr, body = parse_http(cli.sent)
txt = body.decode('utf-8', 'ignore')
check('/ver 返回版本与 TURN_MS',
      'VER=' in txt and 'TURN_MS=360' in txt and int(hdr.get('content-length', -1)) == len(body),
      txt.replace('\n', ' | ')[:90])

#---- B24 / 跳 /h，/t 之类 404 ----
cli = FakeCli(b'GET / HTTP/1.1\r\n\r\n')
mod._http_one(FakeSrv(cli), st, u, up)
check('/ 302 跳 /h', cli.sent.startswith(b'HTTP/1.1 302'), cli.sent.split(b'\r\n')[0].decode())
cli = FakeCli(b'GET /favicon.ico HTTP/1.1\r\n\r\n')
mod._http_one(FakeSrv(cli), st, u, up)
check('未知路径回 404', cli.sent.startswith(b'HTTP/1.1 404'), cli.sent.split(b'\r\n')[0].decode())

#---- B25 参数别名 / 数值边界 ----
check('_mode_val 认 man/manual/auto/0/1',
      mod._mode_val('man') == mod.MODE_MAN and mod._mode_val('auto') == mod.MODE_AUTO
      and mod._mode_val(1) == mod.MODE_MAN and mod._mode_val(0) == mod.MODE_AUTO, '')
check('_act_val 认 stop/fwd/back/left/right/0..4',
      mod._act_val('stop') == mod.ACT_STOP and mod._act_val('fwd') == mod.ACT_FWD
      and mod._act_val('back') == mod.ACT_BACK and mod._act_val(3) == mod.ACT_LEFT
      and mod._act_val(4) == mod.ACT_RIGHT and mod._act_val('zzz') is None, '')
st2, u2 = fresh()
st2['mode'] = mod.MODE_MAN
mod._apply_cmd({'sp': 999}, st2, u2, up, 0)
hi = st2['sp']
mod._apply_cmd({'sp': -5}, st2, u2, up, 0)
lo = st2['sp']
check('sp 被钳到 MIN_SPEED..SPEED_CAP', hi == mod.SPEED_CAP and lo == mod.MIN_SPEED,
      'sp=999 -> %d, sp=-5 -> %d' % (hi, lo))

#---- B26 dry_run 不炸且无异常 ----
try:
    import io
    _stdout = sys.stdout
    sys.stdout = io.StringIO()
    try:
        badn = mod.dry_run()
        out = sys.stdout.getvalue()
    finally:
        sys.stdout = _stdout
    check('dry_run 通过（帧结构 + 后退 bit7）', badn == 0,
          [l.strip() for l in out.splitlines() if 'OK' in l or 'FAIL' in l][-1:] or '')
except Exception as e:
    check('dry_run 通过（帧结构 + 后退 bit7）', False, repr(e))

#---- B27 所有写入串口的帧都是合法帧（帧头/帧尾/BCC）----
bad_frames = []
for f in (u.writes + u2.writes):
    if len(f) < 12 and not (len(f) == 11):
        pass
    if not (f[:2] == b'\xAA\x55' and f[-2:] == b'\x55\xAA'):
        bad_frames.append(f.hex())
        continue
    body = f[2:-3]
    x = 0
    for b in body:
        x ^= b
    if (x & 0xFF) != f[-3]:
        bad_frames.append('BCC ' + f.hex())
check('所有发出的帧帧头/帧尾/BCC 正确', not bad_frames,
      '%d 帧被检查, 坏的: %r' % (len(u.writes + u2.writes), bad_frames[:3]))


#=============================================================================
# 输出
#=============================================================================
def main():
    width = max(len(n) for n, _, _ in RESULTS)
    fails = 0
    print('=' * 78)
    print('wifi_rc_ext.py 自检  (%s)' % TARGET)
    print('=' * 78)
    for name, ok, detail in RESULTS:
        if not ok:
            fails += 1
        print('  [%s] %-*s  %s' % ('PASS' if ok else 'FAIL', width, name, detail))
    print('-' * 78)
    print('  共 %d 项，PASS %d，FAIL %d' % (len(RESULTS), len(RESULTS) - fails, fails))
    print('  文件 %d 字节，CRLF=%d' % (len(raw), raw.count(b'\r\n')))
    print('=' * 78)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
