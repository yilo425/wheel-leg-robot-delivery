#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_proto.py -- FPGA <-> ARM 通信协议（ARM 侧参考实现）

=============================================================================
这是什么
=============================================================================
这是 ARM 侧**可以直接 import 使用**的协议库，不是文档里的代码片段。

    from uart_proto import pack_frame, FrameParser, TYPE_VISION_RESULT
    from uart_proto import make_vision_result, parse_telemetry

    ser.write(make_vision_result([(0x01, 19660, 32767, 88)]))

    parser = FrameParser()
    for ftype, payload in parser.feed(ser.read(256)):
        if ftype == TYPE_TELEMETRY:
            print(parse_telemetry(payload))

=============================================================================
协议速查（完整定义见 doc/uart_frame_protocol.md）
=============================================================================
    帧 = AA 55 | VER | TYPE | LEN(2B大端) | DATA | BCC | 55 AA
         └──────── BCC 计算范围 ────────┘

    帧总长 = 9 + N
    BCC    = VER ^ TYPE ^ LEN_H ^ LEN_L ^ DATA[0..N-1]     ← 不含帧头帧尾

    [!] 帧开销是 9 不是 10 —— 我们自测时算错过一次。

=============================================================================
设计原则
=============================================================================
1) **字节流解析，不假设一次 recv 拿到完整帧**
   串口数据会任意切分。FrameParser 是增量式的，喂多少都行。

2) **解析行为与 FPGA 侧 RTL 完全一致**
   FPGA 的 uart_frame.v 已通过 58 项仿真检查，覆盖：
     BCC 错 / 帧内超时 / 未知类型 / 前导噪声 / 长度超上限
   本库的异常处理逐条对齐，这样两边不会"行为不一致"。

3) **出错不丢同步**
   BCC 错时只跳过 2 字节帧头继续找，不会把后续正常帧一起丢掉。
   （FPGA 侧同样如此，已实测：BCC 错后紧接着的正常帧能立刻收到。）

=============================================================================
本库已自测：见 test_uart_proto.py
"""

import struct
import sys as _sys

#-----------------------------------------------------------------------------
# Windows 控制台编码兜底
#
#   中文 Windows 的控制台默认代码页是 GBK(CP936)，而脚本里会打印
#   [OK] [X] → ★ 这类符号 —— GBK 里没有，直接抛
#       UnicodeEncodeError: 'gbk' codec can't encode character '\u2713'
#   实测：不设 PYTHONIOENCODING=utf-8 时，fpga_link.py --dry-run 和
#         detect_from_image.py --demo-frame 都会崩在第一个 [OK] 上。
#
#   这里主动把 stdout/stderr 切到 UTF-8，让脚本自己不依赖外部环境变量。
#   （Python 3.7+ 支持 reconfigure；更老的版本忽略即可，不会报错）
#-----------------------------------------------------------------------------
#   两类问题要分开处理：
#     ① 崩溃：打印 GBK 表示不了的字符会抛 UnicodeEncodeError
#        -> 用 errors='replace' 兜底，最差显示成 '?'，但**不会崩**
#     ② 乱码：如果强行把 stdout 切成 UTF-8，而控制台仍按 GBK 解码，
#        中文会全部变乱码 —— 比崩溃更糟，因为看起来"程序正常但输出是垃圾"
#        所以**保持控制台原生编码**，中文才能正确显示；
#        代价是非 GBK 符号（[OK] [X] [!] <-> > <）要换成 ASCII。
#        本文件的 print 输出里已经全部换掉了。
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(errors='replace')
    except Exception:
        pass

__all__ = [
    'HDR', 'TAIL', 'PROTO_VER', 'MAX_PAYLOAD', 'FRAME_OVERHEAD',
    'TYPE_IMAGE_DATA', 'TYPE_VISION_RESULT', 'TYPE_ACK', 'TYPE_CONFIG',
    'TYPE_TELEMETRY', 'KNOWN_TYPES',
    'CLS_NONE', 'CLS_PERSON', 'CLS_OBSTACLE', 'CLS_GOAL', 'CLS_LOW_OBSTACLE',
    'STATE_NAMES',
    'bcc', 'pack_frame', 'FrameParser', 'ParseError',
    'make_vision_result', 'parse_vision_result',
    'make_telemetry', 'parse_telemetry',
    'make_ack', 'make_config', 'parse_config',
    'norm_to_u16', 'u16_to_norm', 'x_zone',
]

#=============================================================================
# 协议常量
#=============================================================================
HDR = b'\xAA\x55'
TAIL = b'\x55\xAA'
PROTO_VER = 0x01
MAX_PAYLOAD = 64

# 帧固定开销（不含数据段）：
#   2 帧头 + 1 版本 + 1 类型 + 2 长度 + 1 BCC + 2 帧尾 = 9
FRAME_OVERHEAD = 2 + 1 + 1 + 2 + 1 + 2   # = 9

# 帧类型
TYPE_IMAGE_DATA    = 0x01
TYPE_VISION_RESULT = 0x02
TYPE_ACK           = 0x03
TYPE_CONFIG        = 0x04
TYPE_TELEMETRY     = 0x10

KNOWN_TYPES = frozenset({
    TYPE_IMAGE_DATA, TYPE_VISION_RESULT, TYPE_ACK, TYPE_CONFIG, TYPE_TELEMETRY,
})

# 目标类别（VISION_RESULT 用）
CLS_NONE         = 0x00
CLS_PERSON       = 0x01   # 人员   -> FPGA 减速 + 保持距离
CLS_OBSTACLE     = 0x02   # 障碍物 -> FPGA 绕行
CLS_GOAL         = 0x03   # 目标点 -> FPGA 向目标行进
CLS_LOW_OBSTACLE = 0x04   # 低矮障碍 -> FPGA 触发升降腿越障

# 主状态机编码（TELEMETRY 的"状态"字段）
STATE_NAMES = {
    0: 'S_IDLE', 1: 'S_MANUAL', 2: 'S_AUTO', 3: 'S_AVOID',
    4: 'S_LIFT', 5: 'S_GOAL',   6: 'S_ESTOP',
}

# 每个目标在 VISION_RESULT 里占 7 字节
_TARGET_SIZE = 7


#=============================================================================
# 校验
#=============================================================================
def bcc(body: bytes) -> int:
    """BCC = 逐字节异或。

    body 必须是「版本 + 类型 + 长度(2B) + 数据」——
    **不含帧头帧尾**。这是最容易搞错的地方。
    """
    x = 0
    for b in body:
        x ^= b
    return x & 0xFF


#=============================================================================
# 打包
#=============================================================================
def pack_frame(ftype: int, payload: bytes = b'', ver: int = PROTO_VER) -> bytes:
    """把一帧打成完整字节串。

    >>> pack_frame(TYPE_ACK, bytes([TYPE_VISION_RESULT])).hex(' ')
    'aa 55 01 03 00 01 02 01 55 aa'

    （BCC = 01^03^00^01^02 = 0x01。
      ⚠️ 这里原先把示例写成了 '...02 00 55 aa'，**示例本身是错的**
         —— 函数行为一直是对的（BCC 算出来就是 0x01），只是文档抄错了。
         是队友在 camera_vision.py 里逐位比对时发现的，特此更正。
         教训：docstring 里的示例值也要跑一遍 doctest 才对，不能手写。）
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(
            f'载荷 {len(payload)} 字节超过上限 {MAX_PAYLOAD}；'
            f'FPGA 侧会整帧丢弃（设计如此）')
    body = struct.pack('>BBH', ver, ftype, len(payload)) + bytes(payload)
    return HDR + body + bytes([bcc(body)]) + TAIL


def make_ack(acked_ftype: int) -> bytes:
    """确认帧：载荷 1 字节 = 被确认的帧类型。"""
    return pack_frame(TYPE_ACK, bytes([acked_ftype & 0xFF]))


def make_config(param_id: int, value: bytes) -> bytes:
    """参数配置帧：载荷 = 参数ID(1B) + 长度(1B) + 值。"""
    return pack_frame(TYPE_CONFIG, bytes([param_id & 0xFF, len(value)]) + bytes(value))


def parse_config(payload: bytes):
    """解析 CONFIG 载荷 -> (param_id, value_bytes)。

    注意：FPGA 侧的 uart_frame 目前只做帧级收发，
    参数**语义**的上层处理还没实现（见 doc/interface_delivery.md §8）。
    """
    if len(payload) < 2:
        raise ParseError(f'CONFIG 载荷至少 2 字节，实际 {len(payload)}')
    pid, n = payload[0], payload[1]
    if len(payload) < 2 + n:
        raise ParseError(f'CONFIG 声明 {n} 字节值，实际只有 {len(payload) - 2}')
    return pid, payload[2:2 + n]


#=============================================================================
# 坐标换算
#=============================================================================
def norm_to_u16(px: float, total: int) -> int:
    """像素坐标 -> 归一化 0..65535（与分辨率解耦，换摄像头不用改 FPGA）。"""
    if total <= 0:
        raise ValueError('total 必须 > 0')
    v = int(round(px / total * 65535))
    return max(0, min(65535, v))


def u16_to_norm(v: int, total: int) -> float:
    """归一化 0..65535 -> 像素坐标。"""
    return v / 65535.0 * total


def x_zone(cx: int) -> str:
    """FPGA 的转向判决门限（三档）。

    | x 中心范围        | 判定     |
    |------------------|---------|
    | < 21845          | 偏左     |
    | 21845 .. 43690   | 正前方   |
    | > 43690          | 偏右     |
    """
    if cx < 21845:
        return 'left'
    if cx > 43690:
        return 'right'
    return 'center'


#=============================================================================
# VISION_RESULT
#=============================================================================
def make_vision_result(targets) -> bytes:
    """组一个 VISION_RESULT 帧。

    targets: [(类别, cx归一化, cy归一化, 置信度0..100), ...]，最多 8 个。

    >>> make_vision_result([(CLS_PERSON, 19660, 32767, 88)])[:6].hex(' ')
    'aa 55 01 02 00 08'
    """
    targets = list(targets)
    if len(targets) > 8:
        raise ValueError(f'目标数 {len(targets)} 超过上限 8')

    p = bytes([len(targets)])
    for idx, t in enumerate(targets):
        cls, cx, cy, conf = t
        if not (0 <= cls <= 0xFF):
            raise ValueError(f'类别越界: {cls}')
        if not (0 <= cx <= 65535 and 0 <= cy <= 65535):
            raise ValueError(f'坐标必须归一化到 0..65535: cx={cx} cy={cy}')
        if not (0 <= conf <= 100):
            raise ValueError(f'置信度必须在 0..100: {conf}')
        p += struct.pack('>BBHHB', idx & 0xFF, cls, cx, cy, conf & 0xFF)
    return pack_frame(TYPE_VISION_RESULT, p)


def parse_vision_result(payload: bytes):
    """解析 VISION_RESULT 载荷 -> [(序号, 类别, cx, cy, 置信度), ...]。"""
    if len(payload) < 1:
        raise ParseError('VISION_RESULT 载荷至少 1 字节（目标数）')
    n = payload[0]
    need = 1 + n * _TARGET_SIZE
    if len(payload) != need:
        raise ParseError(
            f'声明 {n} 个目标 -> 应为 {need} 字节，实际 {len(payload)}')
    out = []
    for i in range(n):
        off = 1 + i * _TARGET_SIZE
        out.append(struct.unpack_from('>BBHHB', payload, off))
    return out


#=============================================================================
# TELEMETRY（FPGA -> ARM）
#=============================================================================
# 载荷 = 状态(1) + 速度L(2) + 速度R(2) + 距离(2) + roll(2) + pitch(2) + yaw(2)
_TELEMETRY_FMT = '>BhhHhhh'
_TELEMETRY_LEN = struct.calcsize(_TELEMETRY_FMT)   # = 13


def make_telemetry(state: int, spd_l: int, spd_r: int, dist_mm: int,
                   roll: int = 0, pitch: int = 0, yaw: int = 0) -> bytes:
    """组 TELEMETRY 帧（FPGA 侧用；ARM 侧一般只解析，但也提供以免联调时缺工具）。"""
    p = struct.pack(_TELEMETRY_FMT, state & 0xFF, spd_l, spd_r,
                    dist_mm & 0xFFFF, roll, pitch, yaw)
    return pack_frame(TYPE_TELEMETRY, p)


def parse_telemetry(payload: bytes) -> dict:
    """解析 TELEMETRY 载荷 -> dict。

    速度是**有符号补码**；姿态角单位 **0.01 度**。
    """
    if len(payload) != _TELEMETRY_LEN:
        raise ParseError(
            f'TELEMETRY 载荷应为 {_TELEMETRY_LEN} 字节，实际 {len(payload)}')
    state, spd_l, spd_r, dist, roll, pitch, yaw = struct.unpack(
        _TELEMETRY_FMT, payload)
    return {
        'state': state,
        'state_name': STATE_NAMES.get(state, f'UNKNOWN({state})'),
        'speed_left': spd_l,
        'speed_right': spd_r,
        'dist_mm': dist,
        'roll_deg': roll / 100.0,
        'pitch_deg': pitch / 100.0,
        'yaw_deg': yaw / 100.0,
    }


#=============================================================================
# 流式解析器
#=============================================================================
class ParseError(Exception):
    """载荷格式错误（不是帧级错误 —— 帧级错误由统计计数器记录）。"""


class FrameParser:
    """增量式帧解析器 —— 与 FPGA 侧 uart_frame.v 的状态机行为逐条对齐。

    用法::

        p = FrameParser()
        while True:
            for ftype, payload in p.feed(ser.read(ser.in_waiting or 1)):
                handle(ftype, payload)
            # 需要时可以看统计
            if p.stats['bcc_err'] > 10:
                print('误帧率偏高，考虑升级 CRC16')

    异常处理（与 RTL 一致）::

        BCC 错       -> 跳过这 2 字节帧头，继续找下一帧（sync 不丢）
        帧尾错       -> 同上
        未知类型     -> **完整收完**后丢弃（保持字节同步）
        长度超上限   -> 标记丢弃，仍按长度收完
        前导噪声     -> 任意垃圾字节都能自动跳过

    注意：**没有**实现帧内超时复位。
    超时是"字节间隔"概念，需要时间戳；RTL 侧有 20ms 超时保护，
    但 ARM 侧是流式处理，一个半截帧最多占住最多 MAX_PAYLOAD 字节就会因
    长度/校验不符而被丢弃，不会永久卡死。若要严格对齐，可在外层加超时。
    """

    _S_HDR1, _S_HDR2, _S_VER, _S_TYPE = 0, 1, 2, 3
    _S_LENH, _S_LENL, _S_DATA, _S_BCC = 4, 5, 6, 7
    _S_TAIL1, _S_TAIL2 = 8, 9

    def __init__(self, ver: int = PROTO_VER, strict_types: bool = False):
        """
        ver           : 只接受这个版本的帧（默认 0x01）
        strict_types  : True 时未知类型也交付（默认 False，与 FPGA 一致：丢弃）
        """
        self.ver = ver
        self.strict_types = strict_types
        self.stats = {
            'bytes': 0, 'frames_ok': 0, 'bcc_err': 0,
            'tail_err': 0, 'bad_ver': 0, 'dropped': 0, 'noise_bytes': 0,
        }
        self.reset()

    def reset(self):
        self._st = self._S_HDR1
        self._len = 0
        self._type = 0
        self._bcc = 0
        self._idx = 0
        self._drop = False
        self._buf = bytearray()

    #-------------------------------------------------------------------------
    def feed(self, data) -> list:
        """喂任意长度的字节，返回本次解析出的 [(ftype, payload), ...]。"""
        out = []
        for b in data:
            self.stats['bytes'] += 1
            out.extend(self._feed_byte(b))
        return out

    #-------------------------------------------------------------------------
    def _feed_byte(self, byte: int):
        st = self._st

        # ---- 等帧头 ----
        if st == self._S_HDR1:
            if byte == 0xAA:
                self._st = self._S_HDR2
            else:
                self.stats['noise_bytes'] += 1
            return ()

        if st == self._S_HDR2:
            if byte == 0x55:
                self._st = self._S_VER
                self._bcc = 0
            elif byte == 0xAA:
                pass                      # 停在本状态（连续 AA 不丢同步）
            else:
                self.stats['noise_bytes'] += 1
                self._st = self._S_HDR1
            return ()

        # ---- 版本 ----
        if st == self._S_VER:
            if byte == self.ver:
                self._bcc ^= byte
                self._st = self._S_TYPE
            else:
                self.stats['bad_ver'] += 1
                self._st = self._S_HDR1
            return ()

        # ---- 类型 ----
        if st == self._S_TYPE:
            self._type = byte
            self._bcc ^= byte
            self._drop = (byte not in KNOWN_TYPES) and (not self.strict_types)
            self._st = self._S_LENH
            # 未知类型在这里就报出来，但因为还要收完，不能立即交付
            if byte not in KNOWN_TYPES:
                self.stats['dropped'] += 1
            return ()

        # ---- 长度 ----
        if st == self._S_LENH:
            self._bcc ^= byte
            if byte != 0:
                self._drop = True         # 长度高字节必须为 0（载荷 <= 255）
            self._st = self._S_LENL
            return ()

        if st == self._S_LENL:
            self._bcc ^= byte
            self._len = byte
            self._idx = 0
            self._buf = bytearray()
            if byte > MAX_PAYLOAD:
                self._drop = True
            self._st = self._S_BCC if byte == 0 else self._S_DATA
            return ()

        # ---- 数据 ----
        if st == self._S_DATA:
            self._bcc ^= byte
            if self._idx < MAX_PAYLOAD:
                self._buf.append(byte)
            if self._idx >= self._len - 1:
                self._st = self._S_BCC
            else:
                self._idx += 1
            return ()

        # ---- 校验 ----
        if st == self._S_BCC:
            if byte == (self._bcc & 0xFF):
                self._st = self._S_TAIL1
            else:
                self.stats['bcc_err'] += 1
                self._st = self._S_HDR1
                self.reset_partial_keep_stats()
            return ()

        # ---- 帧尾 ----
        if st == self._S_TAIL1:
            if byte == 0x55:
                self._st = self._S_TAIL2
            else:
                self.stats['tail_err'] += 1
                self._st = self._S_HDR1
            return ()

        if st == self._S_TAIL2:
            self._st = self._S_HDR1
            if byte == 0xAA:
                if self._drop:
                    return ()
                self.stats['frames_ok'] += 1
                return ((self._type, bytes(self._buf)),)
            self.stats['tail_err'] += 1
            return ()

        # 不应到达
        self._st = self._S_HDR1
        return ()

    #-------------------------------------------------------------------------
    def reset_partial_keep_stats(self):
        """出错回帧头时清掉半截状态，但**不动统计计数**。"""
        self._len = 0
        self._idx = 0
        self._buf = bytearray()
        self._drop = False

    @property
    def error_rate(self) -> float:
        """误帧率 = (bcc_err + tail_err) / (frames_ok + bcc_err + tail_err)。"""
        bad = self.stats['bcc_err'] + self.stats['tail_err']
        good = self.stats['frames_ok']
        total = good + bad
        return 0.0 if total == 0 else bad / total


#=============================================================================
# 自测
#=============================================================================
def _selftest():
    print('=== uart_proto 自测 ===')
    ok = 0
    fail = 0

    def chk(name, got, exp):
        nonlocal ok, fail
        if got == exp:
            ok += 1
            print(f'  [ OK ] {name}: {got}')
        else:
            fail += 1
            print(f'  [FAIL] {name}: got={got!r} exp={exp!r}')

    # 帧长 = 9 + N
    f = pack_frame(TYPE_VISION_RESULT, b'\x00')
    chk('空目标帧长 = 9+1', len(f), FRAME_OVERHEAD + 1)
    chk('帧头', f[:2], HDR)
    chk('帧尾', f[-2:], TAIL)

    # 回环
    p = FrameParser()
    got = p.feed(f)
    chk('回环 1 帧', len(got), 1)
    chk('回环类型', got[0][0], TYPE_VISION_RESULT)
    chk('回环载荷', got[0][1], b'\x00')

    # VISION_RESULT 往返
    vr = make_vision_result([(CLS_PERSON, 19660, 32767, 88),
                             (CLS_LOW_OBSTACLE, 40000, 100, 55)])
    p = FrameParser()
    got = p.feed(vr)
    chk('两目标帧长', len(vr), FRAME_OVERHEAD + 1 + 2 * 7)
    chk('解析目标数', len(parse_vision_result(got[0][1])), 2)
    chk('第一个目标类别', parse_vision_result(got[0][1])[0][1], CLS_PERSON)

    # TELEMETRY
    tm = make_telemetry(2, -100, 250, 1234, -150, 25, 9000)
    p = FrameParser()
    got = p.feed(tm)
    chk('TELEMETRY 载荷长 = 13', len(got[0][1]), 13)
    d = parse_telemetry(got[0][1])
    chk('状态名', d['state_name'], 'S_AUTO')
    chk('左速度(负)', d['speed_left'], -100)
    chk('距离', d['dist_mm'], 1234)
    chk('roll 0.01度', d['roll_deg'], -1.5)

    # BCC 错 -> 跳过继续
    bad = bytearray(pack_frame(TYPE_ACK, b'\x02'))
    bad[6] ^= 0xFF
    p = FrameParser()
    got = p.feed(bytes(bad) + pack_frame(TYPE_ACK, b'\x03'))
    chk('BCC 错后仍收到下一帧', len(got), 1)
    chk('收到的是第二帧', got[0][1], b'\x03')
    chk('bcc_err 计数', p.stats['bcc_err'], 1)

    # 未知类型 -> 丢弃但不丢同步
    p = FrameParser()
    got = p.feed(pack_frame(0x77, b'\x01\x02') + pack_frame(TYPE_ACK, b'\x05'))
    chk('未知类型不交付', len(got), 1)
    chk('未知类型后收到正常帧', got[0][1], b'\x05')

    # 前导噪声
    p = FrameParser()
    got = p.feed(b'\x11\x22\xAA\xAA\xAA' + pack_frame(TYPE_ACK, b'\x07'))
    chk('前导噪声后收到帧', len(got), 1)

    # 逐字节喂（模拟串口任意切分）
    p = FrameParser()
    frame = make_vision_result([(CLS_GOAL, 32767, 32767, 99)])
    got = []
    for i in range(len(frame)):
        got.extend(p.feed(frame[i:i + 1]))
    chk('逐字节喂也能解出', len(got), 1)

    # 长度超上限
    p = FrameParser()
    body = struct.pack('>BBH', PROTO_VER, TYPE_VISION_RESULT, 80) + bytes(80)
    oversized = HDR + body + bytes([bcc(body)]) + TAIL
    got = p.feed(oversized + pack_frame(TYPE_ACK, b'\x09'))
    chk('超长帧被丢弃', len(got), 1)
    chk('超长后恢复', got[0][1], b'\x09')

    # 坐标换算
    #   注意 320/640 * 65535 = 32767.5，round 到最近的偶数 = 32768。
    #   32767 和 32768 都是"正中心"，FPGA 侧的三档判决都落在 'center'。
    chk('norm_to_u16(0.5, 640)', norm_to_u16(320, 640), 32768)
    chk('norm_to_u16 边界 0', norm_to_u16(0, 640), 0)
    chk('norm_to_u16 边界 满', norm_to_u16(640, 640), 65535)
    chk('norm_to_u16 越界钳位', norm_to_u16(9999, 640), 65535)
    chk('两种中心都在 center 档', x_zone(32767) == x_zone(32768), True)
    chk('x_zone 左', x_zone(1000), 'left')
    chk('x_zone 中', x_zone(32767), 'center')
    chk('x_zone 右', x_zone(60000), 'right')

    print()
    if fail == 0:
        print(f'RESULT: PASS  ({ok} 项)')
        return 0
    print(f'RESULT: FAIL  ({ok} 通过, {fail} 失败)')
    return 1


if __name__ == '__main__':
    import sys
if __name__ == '__main__':
    import sys
    sys.exit(_selftest())