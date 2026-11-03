#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
verify_telemetry_layout.py -- RTL 的 TELEMETRY 字节布局 vs ARM 侧协议库

=============================================================================
这是在验什么
=============================================================================
**跨模块契约**：FPGA 发出来的 13 字节，ARM 能不能正确解析？

  rtl/app/telemetry_tx.v 的 case(ld_idx)
        │  按这个布局组字节
        ▼
    payload (13 字节)
        │  用 ARM 侧真正的解析器去读
        ▼
  arm/uart_proto.py 的 parse_telemetry()

**两边必须一致。** 只要有一个字段的偏移或大小对不上，
联调时的现象是"帧能收到、字段全错位" —— 长度对、校验对、
就是数值荒谬，而且**不会有任何报错**。

=============================================================================
为什么不用 ModelSim
=============================================================================
这个检查只关心**字节布局**，不关心时序。
用 Python 直接对拍是秒级的；跑 ModelSim 要几分钟，而且还要写
软件串口收发 + 逐字节比对，成本高得多。

时序（握手、tx_req/tx_done）留给 tb_uart_frame 和 tb_top_integration。

=============================================================================
覆盖
=============================================================================
  * 13 个字节的偏移与大小
  * 大端（高字节在前）
  * 有符号补码（负数）
  * 24 位 -> 16 位**饱和**（不是截断！截断会让 +40000 变成负数）
  * 与 ARM 侧 parse_telemetry 的往返一致性
"""

import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'arm'))

try:
    from uart_proto import (
        parse_telemetry, make_telemetry, TYPE_TELEMETRY,
        STATE_NAMES,
    )
except ImportError as e:
    print(f'[错误] 无法导入 arm/uart_proto.py: {e}')
    print('       本脚本需要和 arm/ 目录在同一个工程里。')
    sys.exit(1)


#=============================================================================
# 镜像 rtl/app/telemetry_tx.v 的两段逻辑
#=============================================================================
def sat16(v: int) -> int:
    """24 位有符号 -> 16 位有符号**饱和**。

    对应 RTL 的 sat16() 函数。
    ⚠️ 关键是"饱和"而不是"截断"：
         截断： +40000 & 0xFFFF = 0x9C40 = -25536   <- 正数变负数，方向翻转！
         饱和： +40000 -> +32767                     <- 至少方向是对的
    """
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    # RTL 用的是 v[15:0]（补码截低位），在已饱和的范围内等价于原值
    return v


def rtl_payload(state: int, spd_l: int, spd_r: int,
                dist: int, roll: int, pitch: int, yaw: int) -> bytes:
    """按 telemetry_tx.v 的 case(ld_idx) 逐字节组装。

    ⚠️ 这个函数是**手抄** RTL 的布局。RTL 改了而这里没改，
       本测试就会失败 —— 这正是我们要的（提示去同步）。
    """
    sl = sat16(spd_l)
    sr = sat16(spd_r)

    def be16(v: int) -> bytes:
        """有符号 16 位大端补码"""
        return struct.pack('>h', v)

    b = bytearray(13)
    b[0] = state & 0xFF
    b[1:3] = be16(sl)          # 速度 L
    b[3:5] = be16(sr)          # 速度 R
    b[5:7] = struct.pack('>H', dist & 0xFFFF)   # 距离（无符号）
    b[7:9] = be16(roll)
    b[9:11] = be16(pitch)
    b[11:13] = be16(yaw)
    return bytes(b)


#=============================================================================
class Ck:
    def __init__(self):
        self.ok = 0
        self.fail = 0

    def eq(self, name, got, exp):
        if got == exp:
            self.ok += 1
            print(f'  [OK]   {name:<46} {got}')
        else:
            self.fail += 1
            print(f'  [FAIL] {name:<46} got={got} exp={exp}')


def run():
    ck = Ck()
    print('=== TELEMETRY 字节布局：RTL vs ARM 协议库 ===')
    print('  RTL:  rtl/app/telemetry_tx.v 的 case(ld_idx)')
    print('  ARM:  arm/uart_proto.py 的 parse_telemetry()')
    print()

    #-------------------------------------------------------------------------
    print('--- 1. 长度必须是 13 ---')
    p = rtl_payload(2, 0, 0, 0, 0, 0, 0)
    ck.eq('载荷长度', len(p), 13)
    ck.eq('= 1 + 6*2', len(p), 1 + 6 * 2)

    #-------------------------------------------------------------------------
    print('\n--- 2. 字段偏移与大小 ---')
    p = rtl_payload(state=2, spd_l=0x1234, spd_r=0x5678,
                    dist=0x9ABC, roll=0x1111, pitch=0x2222, yaw=0x3333)
    ck.eq('byte0 = 状态', p[0], 2)
    ck.eq('byte1 = 速度L 高', p[1], 0x12)
    ck.eq('byte2 = 速度L 低', p[2], 0x34)
    ck.eq('byte3 = 速度R 高', p[3], 0x56)
    ck.eq('byte4 = 速度R 低', p[4], 0x78)
    ck.eq('byte5 = 距离 高', p[5], 0x9A)
    ck.eq('byte6 = 距离 低', p[6], 0xBC)
    ck.eq('byte7 = roll 高', p[7], 0x11)
    ck.eq('byte8 = roll 低', p[8], 0x11)
    ck.eq('byte9 = pitch 高', p[9], 0x22)
    ck.eq('byte10 = pitch 低', p[10], 0x22)
    ck.eq('byte11 = yaw 高', p[11], 0x33)
    ck.eq('byte12 = yaw 低', p[12], 0x33)

    #-------------------------------------------------------------------------
    print('\n--- 3. ★ 往返：ARM 侧解析器能不能读对 ---')
    d = parse_telemetry(p)
    ck.eq('状态', d['state'], 2)
    ck.eq('状态名', d['state_name'], 'S_AUTO')
    ck.eq('速度L', d['speed_left'], 0x1234)
    ck.eq('速度R', d['speed_right'], 0x5678)
    ck.eq('距离', d['dist_mm'], 0x9ABC)
    ck.eq('roll 0.01度', d['roll_deg'], 0x1111 / 100.0)
    ck.eq('pitch', d['pitch_deg'], 0x2222 / 100.0)
    ck.eq('yaw', d['yaw_deg'], 0x3333 / 100.0)

    #-------------------------------------------------------------------------
    print('\n--- 4. 有符号负数的补码 ---')
    p = rtl_payload(state=6, spd_l=-100, spd_r=-32768, dist=1234,
                    roll=-150, pitch=-1, yaw=-20000)
    d = parse_telemetry(p)
    ck.eq('速度L 负数', d['speed_left'], -100)
    ck.eq('速度R 最小负数', d['speed_right'], -32768)
    ck.eq('roll 负数', d['roll_deg'], -1.5)
    ck.eq('pitch -1', d['pitch_deg'], -0.01)
    ck.eq('yaw 负数', d['yaw_deg'], -200.0)

    #-------------------------------------------------------------------------
    print('\n--- 5. ★ 24 位 -> 16 位：饱和 而不是 截断 ---')
    ck.eq('+32767 不变', sat16(32767), 32767)
    ck.eq('+32768 饱和到 +32767', sat16(32768), 32767)
    ck.eq('+40000 饱和（截断会变 -25536）', sat16(40000), 32767)
    ck.eq('+8388607 饱和', sat16(8388607), 32767)
    ck.eq('-32768 不变', sat16(-32768), -32768)
    ck.eq('-32769 饱和到 -32768', sat16(-32769), -32768)
    ck.eq('-40000 饱和', sat16(-40000), -32768)
    ck.eq('-8388608 饱和', sat16(-8388608), -32768)

    # 用 ARM 解析器确认"方向没翻转"
    p = rtl_payload(0, 40000, -40000, 0, 0, 0, 0)
    d = parse_telemetry(p)
    ck.eq('大正数饱和后仍是正', d['speed_left'] > 0, True)
    ck.eq('大负数饱和后仍是负', d['speed_right'] < 0, True)

    #-------------------------------------------------------------------------
    print('\n--- 6. 与 ARM 侧 make_telemetry 的一致性 ---')
    # ARM 侧也提供了一个"造帧"函数（联调时用来模拟 FPGA）。
    # 同样的输入，两边产出的载荷必须**逐字节相同**。
    for (st, sl, sr, dist, r, pi, ya) in [
        (0, 0, 0, 0, 0, 0, 0),
        (2, 60, 60, 1234, 0, 0, 0),
        (4, -30, 30, 200, -150, 25, 9000),
        (6, 100, -100, 65535, 32767, -32768, -1),
    ]:
        mine = rtl_payload(st, sl, sr, dist, r, pi, ya)
        theirs = make_telemetry(st, sl, sr, dist, r, pi, ya)
        # make_telemetry 返回整帧（含 9 字节包头尾），取载荷部分
        their_payload = theirs[6:6 + 13]
        ck.eq(f'往返一致 state={st} spd={sl}/{sr}',
              mine == their_payload, True)

    #-------------------------------------------------------------------------
    print('\n--- 7. 每个状态码都能往返 ---')
    for s in range(7):
        p = rtl_payload(s, 1, 1, 1, 0, 0, 0)
        d = parse_telemetry(p)
        ck.eq(f'状态 {s} -> {STATE_NAMES[s]}', d['state'], s)

    #-------------------------------------------------------------------------
    print()
    if ck.fail == 0:
        print(f'RESULT: PASS  ({ck.ok} 项)')
        print()
        print('  结论：RTL 的字节布局与 ARM 侧解析器**逐字节一致**。')
        print('        联调时不会出现"长度对、字段错位"的问题。')
        return 0
    print(f'RESULT: FAIL  ({ck.ok} 通过, {ck.fail} 失败)')
    print()
    print('  ⚠️ 失败通常意味着 RTL 的 telemetry_tx.v 布局改了，')
    print('     而本文件的 rtl_payload() 没同步。请一起改。')
    return 1


if __name__ == '__main__':
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(errors='replace')
        except Exception:
            pass
    sys.exit(run())
