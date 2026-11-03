#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_uart_frame_model.py -- uart_frame 的 Python 模型

为什么写这个：
   帧协议最容易错的地方是**边界与异常**，而不是正常路径：
     * BCC 算不算帧头帧尾？
     * 收到未知 TYPE 时会不会丢帧同步？
     * 帧内丢一个字节会不会永久卡死？
     * 前导噪声 `AA AA 55 ...` 能不能正确同步？
     * 长度超上限会不会写爆缓冲区？

   这些用 Python 复现一遍是分钟级的事，跑 ModelSim 是十几分钟一轮。
   所以先在模型里把协议逻辑验对，再去写真 RTL 的 Testbench。

本模型镜像 rtl/app/uart_frame.v 的行为，并可直接与
doc/uart_frame_protocol.md 里的 ARM 侧参考实现对拍。

用法：
   python verify_uart_frame_model.py
"""

import sys

#=============================================================================
# 协议常量（与 RTL 默认值一致）
#=============================================================================
HDR1, HDR2 = 0xAA, 0x55
TAIL1, TAIL2 = 0x55, 0xAA
PROTO_VER = 0x01
MAX_PAYLOAD = 64

# 帧固定开销（不含载荷）：
#   2 帧头 + 1 版本 + 1 类型 + 2 长度 + 1 BCC + 2 帧尾 = 9
# 一开始我按 6+4=10 算，测试期望值就全错了 —— 这类"off by one"
# 在协议实现里非常常见，写成具名常量比散落各处的数字可靠。
FRAME_OVERHEAD = 2 + 1 + 1 + 2 + 1 + 2   # = 9

KNOWN_TYPES = {0x01, 0x02, 0x03, 0x04, 0x10}


#=============================================================================
# 打包 / 解包
#=============================================================================
def bcc(ver, ftype, payload):
    """校验范围：VER + TYPE + LEN_H + LEN_L + DATA，**不含帧头帧尾**"""
    x = ver ^ ftype ^ 0x00 ^ (len(payload) & 0xFF)
    for b in payload:
        x ^= b
    return x & 0xFF


def pack(ftype, payload, ver=PROTO_VER):
    n = len(payload)
    frame = [HDR1, HDR2, ver, ftype, 0x00, n] + list(payload)
    frame.append(bcc(ver, ftype, payload))
    frame += [TAIL1, TAIL2]
    return frame


#=============================================================================
# 接收状态机（镜像 RTL）
#=============================================================================
S_HDR1, S_HDR2, S_VER, S_TYPE = 0, 1, 2, 3
S_LENH, S_LENL, S_DATA, S_BCC, S_TAIL1, S_TAIL2 = 4, 5, 6, 7, 8, 9


class FrameRx:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = S_HDR1
        self.r_len = 0
        self.r_type = 0
        self.r_bcc = 0
        self.r_idx = 0
        self.r_drop = False
        self.buf = []
        self.events = []          # ('ok', type, payload) / ('bcc',) / ...

    #-------------------------------------------------------------------------
    def feed(self, byte):
        """送一个字节进去，返回本拍是否产生了事件"""
        produced = []

        if self.state == S_HDR1:
            if byte == HDR1:
                self.state = S_HDR2

        elif self.state == S_HDR2:
            if byte == HDR2:
                self.state = S_VER
                self.r_bcc = 0
            elif byte == HDR1:
                self.state = S_HDR2          # 停在等 HDR2（前导噪声）
            else:
                self.state = S_HDR1

        elif self.state == S_VER:
            if byte == PROTO_VER:
                self.r_bcc ^= byte
                self.state = S_TYPE
            else:
                self.state = S_HDR1
                produced.append(('drop', 'bad_ver'))

        elif self.state == S_TYPE:
            self.r_type = byte
            self.r_bcc ^= byte
            self.r_drop = byte not in KNOWN_TYPES
            self.state = S_LENH

        elif self.state == S_LENH:
            self.r_bcc ^= byte
            if byte != 0:
                self.r_drop = True
            self.state = S_LENL

        elif self.state == S_LENL:
            self.r_bcc ^= byte
            self.r_len = byte
            self.r_idx = 0
            self.buf = []
            if byte > MAX_PAYLOAD:
                self.r_drop = True
            self.state = S_BCC if byte == 0 else S_DATA

        elif self.state == S_DATA:
            self.r_bcc ^= byte
            if self.r_idx < MAX_PAYLOAD:
                self.buf.append(byte)
            if self.r_idx >= self.r_len - 1:
                self.state = S_BCC
            else:
                self.r_idx += 1

        elif self.state == S_BCC:
            if byte == self.r_bcc:
                self.state = S_TAIL1
            else:
                self.state = S_HDR1
                produced.append(('bcc_err',))

        elif self.state == S_TAIL1:
            if byte == TAIL1:
                self.state = S_TAIL2
            else:
                self.state = S_HDR1
                produced.append(('drop', 'bad_tail'))

        elif self.state == S_TAIL2:
            self.state = S_HDR1
            if byte == TAIL2:
                if self.r_drop:
                    produced.append(('drop', 'flagged'))
                else:
                    produced.append(('ok', self.r_type, list(self.buf)))
            else:
                produced.append(('drop', 'bad_tail'))

        self.events.extend(produced)
        return produced

    #-------------------------------------------------------------------------
    def feed_all(self, bytes_):
        out = []
        for b in bytes_:
            out.extend(self.feed(b))
        return out


#=============================================================================
# 超时模型：帧内超过 N 个字节时间没收到下一字节就复位
#=============================================================================
def feed_with_timeout(rx, bytes_, gap_after=None, timeout=3):
    """
    gap_after: 在这个下标之后插入一次"超时"（等价于 RTL 里 tmo 计满）
    timeout  : 类似 RTL 的 FRAME_TIMEOUT，这里用"字节数"简化表示
    """
    out = []
    for i, b in enumerate(bytes_):
        out.extend(rx.feed(b))
        if gap_after is not None and i == gap_after:
            # 模拟超时：状态机被重置回 S_HDR1
            if rx.state != S_HDR1:
                rx.state = S_HDR1
                out.append(('timeout',))
    return out


#=============================================================================
# 检查器
#=============================================================================
class Checker:
    def __init__(self):
        self.checks = 0
        self.errors = 0

    def check(self, name, got, exp):
        self.checks += 1
        if got != exp:
            self.errors += 1
            print("  [FAIL] %-52s got=%s exp=%s" % (name, got, exp))
        else:
            print("  [ OK ] %-52s %s" % (name, got))


def run():
    ck = Checker()
    print("=== uart_frame 模型验证 ===")
    print("  VER=0x%02X MAX_PAYLOAD=%d 已知类型=%s"
          % (PROTO_VER, MAX_PAYLOAD, sorted(KNOWN_TYPES)))
    print()

    #-------------------------------------------------------------------------
    print("--- 1. 打包 / 解包 基本正确 ---")
    payload = [0x02, 0x00, 0x04, 0x12, 0x34, 0x56, 0x78, 0x64]
    raw = pack(0x02, payload)
    ck.check("帧长度 = FRAME_OVERHEAD(9) + 载荷", len(raw), FRAME_OVERHEAD + len(payload))
    ck.check("帧头 AA 55", raw[0:2], [0xAA, 0x55])
    ck.check("版本", raw[2], PROTO_VER)
    ck.check("类型", raw[3], 0x02)
    ck.check("长度高字节=0", raw[4], 0x00)
    ck.check("长度低字节", raw[5], len(payload))
    ck.check("帧尾 55 AA", raw[-2:], [0x55, 0xAA])

    rx = FrameRx()
    ev = rx.feed_all(raw)
    ck.check("解出 1 个事件", len(ev), 1)
    ck.check("事件是 ok", ev[0][0], 'ok')
    ck.check("类型还原", ev[0][1], 0x02)
    ck.check("载荷还原", ev[0][2], payload)

    #-------------------------------------------------------------------------
    print("\n--- 2. BCC 计算范围（不含帧头帧尾）---")
    p = [0x11, 0x22, 0x33]
    expect_bcc = PROTO_VER ^ 0x04 ^ 0x00 ^ 0x03 ^ 0x11 ^ 0x22 ^ 0x33
    ck.check("BCC 值", bcc(PROTO_VER, 0x04, p), expect_bcc)
    raw = pack(0x04, p)
    ck.check("帧里的 BCC 字节位置正确", raw[6 + len(p)], expect_bcc)

    #-------------------------------------------------------------------------
    print("\n--- 3. 空载荷（len=0）---")
    raw = pack(0x03, [])
    ck.check("空载荷帧长 = 9", len(raw), FRAME_OVERHEAD)
    rx = FrameRx()
    ev = rx.feed_all(raw)
    ck.check("空载荷也能解出", ev[0][0] if ev else None, 'ok')
    ck.check("空载荷内容为空", ev[0][2] if ev else None, [])

    #-------------------------------------------------------------------------
    print("\n--- 4. BCC 错误 -> 报错且不交付 ---")
    raw = pack(0x02, [1, 2, 3, 4])
    raw[7] ^= 0xFF                     # 篡改一个载荷字节
    rx = FrameRx()
    ev = rx.feed_all(raw)
    kinds = [e[0] for e in ev]
    ck.check("检测到 bcc_err", 'bcc_err' in kinds, True)
    ck.check("没有交付 ok",    'ok' in kinds, False)
    # BCC 错后帧尾的 0x55 0xAA 仍会被送进状态机，其中的 AA 会被当成新帧头 ——
    # 这是**正确行为**（不必跳过帧尾）。重点是后续正常帧还能收到。
    ev_next = rx.feed_all(pack(0x02, [0x77]))
    ck.check("★BCC 错后仍能收到下一帧",
             (ev_next[0][0] if ev_next else None), 'ok')

    #-------------------------------------------------------------------------
    print("\n--- 5. 未知类型：收完但不交付（保持帧同步）---")
    raw = pack(0x77, [9, 9, 9])        # 0x77 不在已知类型里
    rx = FrameRx()
    ev = rx.feed_all(raw)
    kinds = [e[0] for e in ev]
    ck.check("未知类型被丢弃", 'drop' in kinds, True)
    ck.check("未知类型不交付", 'ok' in kinds, False)
    ck.check("★丢弃后仍回到帧头（同步未丢）", rx.state, S_HDR1)

    # 紧接着一帧正常帧，应该能收到
    ev2 = rx.feed_all(pack(0x02, [0xAA]))
    ck.check("★未知类型之后能立刻收到正常帧",
             ev2[0][0] if ev2 else None, 'ok')

    #-------------------------------------------------------------------------
    print("\n--- 6. 前导噪声 `AA AA 55 ...` ---")
    good = pack(0x02, [0x42])
    noisy = [0x11, 0x22, 0xAA] + good   # 先来两个无关字节，再是正常帧
    rx = FrameRx()
    ev = rx.feed_all(noisy)
    kinds = [e[0] for e in ev]
    ck.check("噪声后仍能解出帧", 'ok' in kinds, True)

    # 纯噪声（全是 AA）然后接正常帧
    rx = FrameRx()
    ev = rx.feed_all([0xAA] * 8 + good)
    ck.check("连续 8 个 AA 后能解出帧",
             'ok' in [e[0] for e in ev], True)

    #-------------------------------------------------------------------------
    print("\n--- 7. 长度超上限 ---")
    raw = pack(0x02, list(range(80)))   # 80 > MAX_PAYLOAD(64)
    ck.check("构造的载荷长度", len(raw) - FRAME_OVERHEAD, 80)
    rx = FrameRx()
    ev = rx.feed_all(raw)
    kinds = [e[0] for e in ev]
    ck.check("超长帧被丢弃", 'drop' in kinds, True)
    ck.check("超长帧不交付", 'ok' in kinds, False)
    ck.check("缓冲区没被写爆", len(rx.buf) <= MAX_PAYLOAD, True)
    ck.check("超长后回到帧头", rx.state, S_HDR1)

    #-------------------------------------------------------------------------
    print("\n--- 8. 帧内超时（丢字节）---")
    raw = pack(0x02, [1, 2, 3, 4, 5])
    rx = FrameRx()
    ev = feed_with_timeout(rx, raw, gap_after=3)
    kinds = [e[0] for e in ev]
    ck.check("检测到超时", 'timeout' in kinds, True)
    ck.check("超时后没有交付残帧", 'ok' in kinds, False)

    # 超时后再来一帧正常的，必须能收到
    ev2 = rx.feed_all(pack(0x02, [0x5A]))
    ck.check("★超时后能立刻恢复接收",
             ev2[0][0] if ev2 else None, 'ok')

    #-------------------------------------------------------------------------
    print("\n--- 9. 连发多帧不丢 ---")
    rx = FrameRx()
    stream = pack(0x02, [0x01]) + pack(0x10, [0x02, 0x03]) + pack(0x03, [])
    ev = rx.feed_all(stream)
    ok = [e for e in ev if e[0] == 'ok']
    ck.check("连发 3 帧收到 3 帧", len(ok), 3)
    ck.check("第 1 帧类型", ok[0][1] if len(ok) > 0 else None, 0x02)
    ck.check("第 2 帧类型", ok[1][1] if len(ok) > 1 else None, 0x10)
    ck.check("第 3 帧类型", ok[2][1] if len(ok) > 2 else None, 0x03)

    #-------------------------------------------------------------------------
    print("\n--- 10. 边界：长度恰好 = MAX_PAYLOAD ---")
    raw = pack(0x02, list(range(MAX_PAYLOAD)))
    rx = FrameRx()
    ev = rx.feed_all(raw)
    ck.check("恰好 %d 字节能收" % MAX_PAYLOAD,
             'ok' in [e[0] for e in ev], True)

    #-------------------------------------------------------------------------
    print("\n--- 11. BCC 边界：空载荷的 BCC ---")
    ck.check("空载荷 BCC = VER^TYPE^0^0",
             bcc(PROTO_VER, 0x03, []),
             PROTO_VER ^ 0x03 ^ 0x00 ^ 0x00)

    #-------------------------------------------------------------------------
    print()
    if ck.errors == 0:
        print("RESULT: PASS  (%d 项检查全部通过)" % ck.checks)
        print()
        print("说明：模型验证的是**协议逻辑**（打包/解析/异常/同步）。")
        print("      与真实 uart_tx/uart_rx 的字节级时序仍需 ModelSim 验证。")
        return 0
    print("RESULT: FAIL  (%d 项检查, %d 个错误)" % (ck.checks, ck.errors))
    return 1


if __name__ == '__main__':
    sys.exit(run())
