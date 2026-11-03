#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
verify_vision_rx_model.py -- vision_rx 的 Python 行为模型

=============================================================================
为什么先写这个
=============================================================================
vision_rx 要处理的是**字节偏移 + 大端拼装 + 最优目标挑选 + 定点换算**，
这四样都是"写错了不报错、只是结果不对"的类型：

  * 偏移算错一字节 -> 读到的"类别"其实是"序号"，值看起来完全合理
  * 大端小端搞反   -> x 中心变成一个乱七八糟的数，仍然在 0..65535 范围内
  * 死区处理写错   -> 直行时轻微抖动，或出死区时阶跃
  * 算术右移负数   -> 左转和右转不对称（Verilog 的 >>> 是向下取整）

这些在 ModelSim 里要几分钟一轮，**在 Python 里是秒级**。
所以先把逻辑验对，再写 Testbench。

=============================================================================
模型镜像的 RTL 逻辑
=============================================================================
    载荷: [n] [序号 类别 xH xL yH yL 置信度] × n
    长度必须是 1 + 7n 且 n <= MAX_TARGETS，否则整帧丢弃
    选**置信度最高**的目标
    e = x - 32768
    e' = e - sign(e)*DEAD        （死区外才减，保证边界连续）
    steer = clamp(e' >>> SHIFT, ±MAX)
"""

import sys

#=============================================================================
# 与 RTL 默认参数一致
#=============================================================================
TYPE_VISION_RESULT = 0x02
MAX_TARGETS        = 8
X_CENTER           = 32768
STEER_DEAD         = 2048
STEER_SHIFT        = 6
STEER_MAX          = 512

# 类别码
CLS_NONE, CLS_PERSON, CLS_OBSTACLE, CLS_GOAL, CLS_LOW_OBSTACLE = 0, 1, 2, 3, 4

TARGET_SIZE = 7


#=============================================================================
# 期望长度查表（与 RTL 的 expected_len() 一致）
#=============================================================================
def expected_len(n: int) -> int:
    return 1 + 7 * n if 0 <= n <= MAX_TARGETS else 0xFF


#=============================================================================
# 坐标 -> 转向（与 RTL 的组合逻辑逐条对应）
#=============================================================================
def steer_from_x(x: int, dead: int = STEER_DEAD, shift: int = STEER_SHIFT,
                 vmax: int = STEER_MAX, center: int = X_CENTER) -> int:
    """注意：Python 的 >> 对负数是向下取整，和 Verilog 的 >>> 语义一致。

    参数化而不是改全局变量 —— 后者要求 global 声明在所有使用之前，
    容易踩 SyntaxError（本文件第一版就踩了）。
    """
    e = x - center
    if e > dead:
        e2 = e - dead
    elif e < -dead:
        e2 = e + dead
    else:
        e2 = 0
    v = e2 >> shift                # 算术右移 = 向下取整
    if v > vmax:
        v = vmax
    if v < -vmax:
        v = -vmax
    return v


#=============================================================================
# 解析一帧（镜像 vision_rx 的状态机）
#=============================================================================
def parse_vision_frame(ftype: int, payload: bytes):
    """返回 dict：
         ok        : 这帧是否被接受
         bad       : 是否报了 vision_bad
         n         : 目标数
         cls/x/conf: 选中目标（置信度最高）
         steer     : 换算出的转向
    """
    r = {'ok': False, 'bad': False, 'n': 0,
         'cls': 0, 'x': 0, 'conf': 0, 'steer': 0}

    if ftype != TYPE_VISION_RESULT:
        return r                        # 不是视觉帧，直接忽略

    if len(payload) < 1:
        r['bad'] = True
        return r

    n = payload[0]
    if len(payload) != expected_len(n) or n > MAX_TARGETS:
        r['bad'] = True                 # 长度不合法 -> 整帧丢弃
        return r

    r['n'] = n
    r['ok'] = True

    if n == 0:
        r['cls'] = CLS_NONE             # 合法但无目标
        return r

    best = None
    for i in range(n):
        off = 1 + i * TARGET_SIZE
        idx  = payload[off + 0]
        cls  = payload[off + 1]
        x    = (payload[off + 2] << 8) | payload[off + 3]   # 大端
        y    = (payload[off + 4] << 8) | payload[off + 5]
        conf = payload[off + 6]
        if best is None or conf > best[3]:
            best = (idx, cls, x, conf)

    _, cls, x, conf = best
    r['cls']   = cls
    r['x']     = x
    r['conf']  = conf
    r['steer'] = steer_from_x(x)
    return r


#=============================================================================
# 造帧
#=============================================================================
def make_payload(targets):
    p = bytes([len(targets)])
    for i, (cls, x, conf) in enumerate(targets):
        p += bytes([i, cls, (x >> 8) & 0xFF, x & 0xFF, 0, 0, conf])
    return p


#=============================================================================
# 检查器
#=============================================================================
class Ck:
    def __init__(self):
        self.ok = 0
        self.fail = 0

    def eq(self, name, got, exp):
        if got == exp:
            self.ok += 1
            print(f'  [OK]   {name:<44} {got}')
        else:
            self.fail += 1
            print(f'  [FAIL] {name:<44} got={got} exp={exp}')


def run():
    ck = Ck()
    print('=== vision_rx 行为模型验证 ===')
    print(f'  X_CENTER={X_CENTER} STEER_DEAD={STEER_DEAD} '
          f'STEER_SHIFT={STEER_SHIFT} STEER_MAX={STEER_MAX}')
    print()

    #-------------------------------------------------------------------------
    print('--- 1. 期望长度查表 ---')
    for n in range(9):
        ck.eq(f'n={n} -> len', expected_len(n), 1 + 7 * n)
    ck.eq('n=9 -> 非法(0xFF)', expected_len(9), 0xFF)

    #-------------------------------------------------------------------------
    print('\n--- 2. 字节偏移与大端拼装 ---')
    # 造两个目标，x 用能区分的值
    p = make_payload([(CLS_PERSON, 0x1234, 50), (CLS_GOAL, 0xABCD, 99)])
    ck.eq('目标数', p[0], 2)
    ck.eq('载荷长', len(p), 15)
    ck.eq('目标0 类别', p[2], CLS_PERSON)
    ck.eq('目标0 x 高字节', p[3], 0x12)
    ck.eq('目标0 x 低字节', p[4], 0x34)
    ck.eq('目标0 置信度', p[7], 50)
    ck.eq('目标1 类别', p[9], CLS_GOAL)
    ck.eq('目标1 x 高字节', p[10], 0xAB)
    ck.eq('目标1 x 低字节', p[11], 0xCD)

    r = parse_vision_frame(TYPE_VISION_RESULT, p)
    ck.eq('接受该帧', r['ok'], True)
    ck.eq('选中置信度最高的(目标1)', r['conf'], 99)
    ck.eq('选中类别', r['cls'], CLS_GOAL)
    ck.eq('选中 x（大端拼装）', r['x'], 0xABCD)
    ck.eq('x 拼错会得到 0xCDAB', r['x'] != 0xCDAB, True)

    #-------------------------------------------------------------------------
    print('\n--- 3. 选最优目标（置信度最高，不是第一个/最后一个）---')
    p = make_payload([(CLS_PERSON, 1000, 10),
                      (CLS_LOW_OBSTACLE, 2000, 95),
                      (CLS_OBSTACLE, 3000, 40)])
    r = parse_vision_frame(TYPE_VISION_RESULT, p)
    ck.eq('选中中间那个', r['cls'], CLS_LOW_OBSTACLE)
    ck.eq('置信度', r['conf'], 95)
    ck.eq('x', r['x'], 2000)

    # 相同置信度时保持先遇到的（严格大于才替换）
    p = make_payload([(CLS_PERSON, 1000, 50), (CLS_GOAL, 2000, 50)])
    r = parse_vision_frame(TYPE_VISION_RESULT, p)
    ck.eq('同置信度取先遇到的', r['cls'], CLS_PERSON)

    #-------------------------------------------------------------------------
    print('\n--- 4. 无目标 (n=0) 是合法的 ---')
    r = parse_vision_frame(TYPE_VISION_RESULT, bytes([0]))
    ck.eq('接受 n=0', r['ok'], True)
    ck.eq('没报 bad', r['bad'], False)
    ck.eq('类别=无', r['cls'], CLS_NONE)
    ck.eq('steer=0', r['steer'], 0)

    #-------------------------------------------------------------------------
    print('\n--- 5. 长度校验（不合法就整帧丢弃）---')
    # 声明 2 个目标但只给了 1 个
    bad = bytes([2]) + bytes(7)
    r = parse_vision_frame(TYPE_VISION_RESULT, bad)
    ck.eq('长度不符 -> bad', r['bad'], True)
    ck.eq('长度不符 -> 不接受', r['ok'], False)

    # 声明 1 个目标但给了 2 个
    bad = bytes([1]) + bytes(14)
    r = parse_vision_frame(TYPE_VISION_RESULT, bad)
    ck.eq('长度多出来 -> bad', r['bad'], True)

    # 目标数超上限
    bad = bytes([9]) + bytes(63)
    r = parse_vision_frame(TYPE_VISION_RESULT, bad)
    ck.eq('n=9 超上限 -> bad', r['bad'], True)

    # 空载荷
    r = parse_vision_frame(TYPE_VISION_RESULT, b'')
    ck.eq('空载荷 -> bad', r['bad'], True)

    #-------------------------------------------------------------------------
    print('\n--- 6. 不是视觉帧 -> 忽略（不报 bad）---')
    r = parse_vision_frame(0x03, bytes([0x02]))       # ACK
    ck.eq('ACK 不报 bad', r['bad'], False)
    ck.eq('ACK 不接受', r['ok'], False)
    r = parse_vision_frame(0x10, bytes(13))           # TELEMETRY
    ck.eq('TELEMETRY 不报 bad', r['bad'], False)

    #-------------------------------------------------------------------------
    print('\n--- 7. 坐标 -> 转向（含死区与钳位）---')
    ck.eq('正中心 -> 直行', steer_from_x(32768), 0)
    ck.eq('死区内(左) -> 直行', steer_from_x(32768 - 2048), 0)
    ck.eq('死区内(右) -> 直行', steer_from_x(32768 + 2048), 0)
    ck.eq('刚出死区 -> 仍为 0（右移后）', steer_from_x(32768 + 2048 + 1), 0)
    # 死区减完 64 时才出 1
    ck.eq('出死区后第一个 1', steer_from_x(32768 + 2048 + 64), 1)
    ck.eq('出死区后第一个 -1', steer_from_x(32768 - 2048 - 64), -1)

    ck.eq('最左 x=0', steer_from_x(0), -480)
    ck.eq('最右 x=65535', steer_from_x(65535), 479)
    ck.eq('左转是负', steer_from_x(0) < 0, True)
    ck.eq('右转是正', steer_from_x(65535) > 0, True)

    # 钳位：把死区调到 0、位移调到 2，让输出必然超限
    ck.eq('钳位到 +MAX', steer_from_x(65535, dead=0, shift=2, vmax=100), 100)
    ck.eq('钳位到 -MAX', steer_from_x(0,     dead=0, shift=2, vmax=100), -100)

    # 左右对称性
    #
    #   ⚠️ 实测**不是完全对称**，左边比右边绝对值大 1：
    #        +7952 >> 6 = +124.25 ->  124   （正数截断）
    #        -7952 >> 6 = -124.25 -> -125   （负数 floor，往负方向多走一格）
    #      这是 Verilog / Python 算术右移的固有语义，不是 bug。
    #
    #      影响：1/512 ≈ 0.2%，对转向无实际影响；
    #      而且转向是**比例控制**（无积分项），不会累积成航向偏置。
    #      所以选择"接受 + 记录"，而不是加取整逻辑去消掉它。
    #
    #      下面把这个不对称**显式钉住** —— 以后谁改了换算公式，
    #      这里的测试会立刻发现。
    sr = steer_from_x(32768 + 10000)
    sl = steer_from_x(32768 - 10000)
    ck.eq('右转 +10000', sr, 124)
    ck.eq('左转 -10000', sl, -125)
    ck.eq('左右差异不超过 1 LSB', abs(abs(sl) - abs(sr)) <= 1, True)

    # 中心附近完全对称（死区内 / 位移后为 0 的区间）
    ck.eq('死区内左右都是 0',
           (steer_from_x(32768 + 2048) == steer_from_x(32768 - 2048)), True)
    ck.eq('最左/最右近似对称', abs(abs(steer_from_x(0)) - abs(steer_from_x(65535))) <= 1, True)

    #-------------------------------------------------------------------------
    print('\n--- 8. 最大目标数 (n=8) ---')
    tg = [(i % 5, 1000 * (i + 1), 10 * (i + 1)) for i in range(8)]
    p = make_payload(tg)
    ck.eq('载荷长 = 1+7*8', len(p), 57)
    r = parse_vision_frame(TYPE_VISION_RESULT, p)
    ck.eq('n=8 被接受', r['ok'], True)
    ck.eq('选中置信度最高的(第 8 个)', r['conf'], 80)
    ck.eq('它的 x', r['x'], 8000)

    #-------------------------------------------------------------------------
    print('\n--- 9. 联调实测帧（ARM 队友提供的"标准帧"）---')
    print('    帧: AA 55 01 02 00 08 01 00 02 80 00 80 00 02 0A 55 AA')
    print('    来源: ARM 侧实测，正中心(32768,32768)的 0x02 障碍物')
    #
    #   这一节的意义：把**对方实测跑出来的字节**原样喂进我们的解析逻辑。
    #   它验证的不是"我按文档实现了"，而是"两边真的对得上"。
    #   协议哪天被改动，这里会第一时间红。
    #
    #   复算 BCC 时刻意不调用我方任何函数 —— 纯手工异或，
    #   这样才是在验证"对方的帧"，而不是"我方自洽"。
    #-------------------------------------------------------------------------
    raw = bytes.fromhex('AA 55 01 02 00 08 01 00 02 80 00 80 00 02 0A 55 AA')

    calc = 0
    for b in raw[2:-3]:
        calc ^= b
    ck.eq('对方帧的 BCC 独立复算一致', calc, raw[-3])
    ck.eq('对方帧总长 = 9+8 = 17', len(raw), 17)
    ck.eq('对方帧帧头 AA 55', raw[0:2], b'\xAA\x55')
    ck.eq('对方帧帧尾 55 AA', raw[-2:], b'\x55\xAA')
    ck.eq('VER/TYPE = 01/02', (raw[2], raw[3]), (0x01, 0x02))
    ck.eq('LEN 大端 = 8', (raw[4] << 8) | raw[5], 8)

    # 剥掉 帧头(2) + VER/TYPE/LEN(4) + BCC(1) + 帧尾(2) -> 载荷
    payload = raw[6:-3]
    ck.eq('剥出的载荷长 = 8', len(payload), 8)

    r = parse_vision_frame(TYPE_VISION_RESULT, payload)
    ck.eq('解析被接受', r['ok'], True)
    ck.eq('目标数 = 1', r['n'], 1)
    ck.eq('类别 = 0x02 障碍物', r['cls'], 0x02)
    ck.eq('cx = 32768（正中心）', r['x'], 32768)
    ck.eq('置信度 = 2', r['conf'], 2)
    ck.eq('正中心 -> steer 落在死区内 = 0', r['steer'], 0)

    # 正中心应该走直线：转向为 0
    ck.eq('正中心不转向', steer_from_x(32768), 0)

    #-------------------------------------------------------------------------
    print()
    if ck.fail == 0:
        print(f'RESULT: PASS  ({ck.ok} 项)')
        print()
        print('说明：模型验证的是**解析算法与定点换算**。')
        print('      与真实 uart_frame 的握手时序仍需 ModelSim 验证。')
        return 0
    print(f'RESULT: FAIL  ({ck.ok} 通过, {ck.fail} 失败)')
    return 1


if __name__ == '__main__':
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(errors='replace')
        except Exception:
            pass
    sys.exit(run())
