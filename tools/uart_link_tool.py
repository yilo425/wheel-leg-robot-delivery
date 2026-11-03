#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_link_tool.py -- FPGA <-> ARM 串口链路联调工具（PC 侧）

为什么要有它：
  协议细节（帧头/长度/BCC/大端/有符号）靠人眼核对必然出错，
  而"校验错了"和"线没接"在现象上一模一样。这个工具：
    * 用的是 doc/uart_frame_protocol.md §5 的**同一套** pack_frame，
      生成的帧一定和文档自洽（不手算校验和）；
    * 能把 FPGA 发回来的 TELEMETRY 直接翻译成人话（状态名/物理量）；
    * 能当"假 ARM"先验证 FPGA 的接收通路，再换成真的 ARM。

用法：
    # 1) 打印现成的测试帧（不需要任何依赖，直接复制到串口助手也行）
    python tools/uart_link_tool.py frames

    # 2) 解析一帧（十六进制字符串，空格可有可无）
    python tools/uart_link_tool.py decode "AA 55 01 10 00 0D ..."

    # 3) 实时监视（需要 pyserial：pip install pyserial）
    python tools/uart_link_tool.py monitor COM8

    # 4) 当假 ARM，往 FPGA 发一帧（需要 pyserial）
    python tools/uart_link_tool.py send COM8 --class 2        # 障碍物 -> AVOID
    python tools/uart_link_tool.py send COM8 --class 4        # 低矮障碍 -> LIFT
    python tools/uart_link_tool.py send COM8 --clear          # 清空目标 -> 回 AUTO

协议（与 doc/uart_frame_protocol.md 一致）：
    +------+------+-----+------+-------+-------+---------+-----+------+------+
    | 0xAA | 0x55 | VER | TYPE | LEN_H | LEN_L | DATA... | BCC | 0x55 | 0xAA |
    +------+------+-----+------+-------+-------+---------+-----+------+------+
    BCC = XOR(VER, TYPE, LEN_H, LEN_L, DATA...)      不含帧头帧尾
"""

import struct
import sys

HDR  = b'\xAA\x55'
TAIL = b'\x55\xAA'
VER  = 0x01

TYPE_IMAGE     = 0x01
TYPE_VISION    = 0x02
TYPE_ACK       = 0x03
TYPE_CONFIG    = 0x04
TYPE_TELEMETRY = 0x10

# main_fsm 的状态编码（和 rtl/app/main_fsm.v 的 localparam 一一对应）
STATE_NAME = {
    0: 'IDLE', 1: 'MANUAL', 2: 'AUTO',
    3: 'AVOID', 4: 'LIFT', 5: 'GOAL', 6: 'ESTOP',
}

# 视觉类别码（doc/uart_frame_protocol.md §3.1）
CLASS_NAME = {
    0x00: '无',
    0x01: '人员',
    0x02: '障碍物',
    0x03: '目标点/终点',
    0x04: '低矮障碍',
}


# ---------------------------------------------------------------------------
# 帧的封装 / 解析
# ---------------------------------------------------------------------------
def bcc(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def pack_frame(ftype: int, payload: bytes) -> bytes:
    """与文档 §5 完全一致：BCC 覆盖 VER..DATA（不含帧头帧尾）。"""
    body = struct.pack('>BBH', VER, ftype, len(payload)) + payload
    return HDR + body + bytes([bcc(body)]) + TAIL


def vision_result(targets) -> bytes:
    """targets: [(class, cx, cy, conf), ...]；cx/cy 归一化 0..65535。"""
    p = bytes([len(targets) & 0xFF])
    for idx, (cls, cx, cy, conf) in enumerate(targets):
        p += struct.pack('>BBHHB', idx, cls, cx & 0xFFFF, cy & 0xFFFF, conf & 0xFF)
    return pack_frame(TYPE_VISION, p)


def parse_stream(buf: bytearray):
    """从缓冲区里尽量多地解析出完整帧，返回 (帧列表, 剩余缓冲)。"""
    frames = []
    while True:
        i = buf.find(HDR)
        if i < 0:
            del buf[:-1]                      # 只留最后一个字节（可能是 0xAA）
            break
        if i > 0:
            del buf[:i]
        if len(buf) < 6:
            break
        ver, ftype, n = struct.unpack_from('>BBH', buf, 2)
        total = 2 + 1 + 1 + 2 + n + 1 + 2
        if len(buf) < total:
            break
        body     = bytes(buf[2:2 + 1 + 1 + 2 + n])
        got_bcc  = buf[2 + 1 + 1 + 2 + n]
        tail     = bytes(buf[total - 2:total])
        if bcc(body) != got_bcc or tail != TAIL:
            del buf[:2]                       # 坏帧：跳过这个帧头继续找
            continue
        frames.append((ver, ftype, bytes(buf[6:6 + n])))
        del buf[:total]
    return frames, buf


# ---------------------------------------------------------------------------
# 解包成"人话"
# ---------------------------------------------------------------------------
def decode_telemetry(pl: bytes):
    """13 字节：state(1) spdL(2) spdR(2) dist(2) roll(2) pitch(2) yaw(2) 大端/补码。"""
    if len(pl) != 13:
        return 'TELEMETRY 长度 %d != 13（协议是 13，收到别的是固件/协议版本不对）' % len(pl)
    state                  = pl[0]
    spd_l, spd_r, dist     = struct.unpack_from('>hhH', pl, 1)
    roll, pitch, yaw       = struct.unpack_from('>hhh', pl, 7)
    return ('state=%s(%d)  spdL=%6d spdR=%6d  dist=%5dmm  '
            'roll=%7.2f° pitch=%7.2f° yaw=%7.2f°'
            % (STATE_NAME.get(state, '?'), state,
               spd_l, spd_r, dist, roll / 100.0, pitch / 100.0, yaw / 100.0))


def decode_vision(pl: bytes):
    if len(pl) < 1:
        return 'VISION 空载荷'
    n = pl[0]
    if len(pl) != 1 + 7 * n:
        return 'VISION 长度 %d != 1+7*%d（格式不对）' % (len(pl), n)
    if n == 0:
        return 'VISION 目标数=0（清空，FSM 应回 AUTO）'
    out = ['VISION 目标数=%d' % n]
    for i in range(n):
        seq, cls, cx, cy, conf = struct.unpack_from('>BBHHB', pl, 1 + 7 * i)
        out.append('  [%d] seq=%d 类别=0x%02X(%s) x=%d y=%d conf=%d'
                   % (i, seq, cls, CLASS_NAME.get(cls, '未知'), cx, cy, conf))
    return '\n'.join(out)


def describe(ftype, pl):
    if ftype == TYPE_TELEMETRY:
        return decode_telemetry(pl)
    if ftype == TYPE_VISION:
        return decode_vision(pl)
    if ftype == TYPE_ACK:
        return 'ACK 被确认的类型=0x%02X' % (pl[0] if pl else 0)
    return 'TYPE=0x%02X len=%d payload=%s' % (ftype, len(pl), pl.hex(' '))


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------
def cmd_frames(_args):
    # x 归一化：画面正中心 = 32768（0x8000）
    cases = [
        ('障碍物（正前方，触发 AVOID）',     vision_result([(0x02, 32768, 32768, 88)])),
        ('低矮障碍（正前方，触发 LIFT）',     vision_result([(0x04, 32768, 32768, 90)])),
        ('人员（偏左 0.30，减速+保持距离）',  vision_result([(0x01, 19660, 32768, 88)])),
        ('目标点（偏右 0.70，向目标行进）',   vision_result([(0x03, 45875, 32768, 75)])),
        ('清空目标（目标数=0，回 AUTO）',    vision_result([])),
    ]
    print('可直接粘进串口助手（HEX 模式发送）：\n')
    for name, f in cases:
        print('  %s' % name)
        print('    长度 %d 字节' % len(f))
        print('    ' + f.hex(' ').upper())
        print('')
    print('提示：类别码 0x01 人员 / 0x02 障碍物 / 0x03 目标点 / 0x04 低矮障碍')
    print('      x=0x8000 是画面正中心；改 x 就相当于让车"看到目标偏左/偏右"')
    return 0


def cmd_decode(args):
    raw = ''.join(args.hex).replace(',', ' ').replace('0x', ' ')
    data = bytearray(bytes.fromhex(raw.replace(' ', '')))
    frames, rest = parse_stream(data)
    if not frames:
        print('没有解析出完整帧（帧头 0xAA55 / 长度 / BCC / 帧尾任一不匹配都会被丢）')
        print('剩余未消费字节：%s' % bytes(rest).hex(' '))
        return 1
    for ver, ftype, pl in frames:
        print('[VER=0x%02X] %s' % (ver, describe(ftype, pl)))
    if rest:
        print('剩余未消费字节：%s' % bytes(rest).hex(' '))
    return 0


def _open_serial(port, baud):
    try:
        import serial
    except ImportError:
        print('需要 pyserial：pip install pyserial', file=sys.stderr)
        return None
    try:
        return serial.Serial(port, baud, timeout=0.2)
    except Exception as exc:                    # noqa: BLE001（打印清楚就够了）
        print('打开 %s 失败：%s' % (port, exc), file=sys.stderr)
        return None


def cmd_monitor(args):
    ser = _open_serial(args.port, args.baud)
    if ser is None:
        return 1
    print('监视 %s @%d ... （Ctrl-C 退出）' % (args.port, args.baud))
    buf = bytearray()
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
                frames, buf = parse_stream(buf)
                for ver, ftype, pl in frames:
                    print('[VER=0x%02X] %s' % (ver, describe(ftype, pl)))
            elif args.timeout_s and ser.in_waiting == 0:
                pass
    except KeyboardInterrupt:
        print('\n退出')
    finally:
        ser.close()
    return 0


def cmd_send(args):
    if args.clear:
        frame = vision_result([])
        desc  = '清空目标'
    else:
        cx = args.x
        frame = vision_result([(args.cls, cx, args.y, args.conf)])
        desc  = '类别=0x%02X(%s) x=%d y=%d conf=%d' % (
            args.cls, CLASS_NAME.get(args.cls, '未知'), cx, args.y, args.conf)
    print('发送 %d 字节：%s' % (len(frame), frame.hex(' ').upper()))
    print('内容：%s' % desc)
    ser = _open_serial(args.port, args.baud)
    if ser is None:
        return 1
    try:
        ser.write(frame)
        ser.flush()
    finally:
        ser.close()
    print('已发送')
    return 0


def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description='FPGA <-> ARM 串口链路联调工具')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('frames', help='打印现成的测试帧（无需依赖）')
    p.set_defaults(func=cmd_frames)

    p = sub.add_parser('decode', help='解析十六进制帧')
    p.add_argument('hex', nargs='+', help='如 "AA 55 01 10 00 0D ..."')
    p.set_defaults(func=cmd_decode)

    p = sub.add_parser('monitor', help='实时解码（需要 pyserial）')
    p.add_argument('port', help='如 COM8 或 /dev/ttyUSB0')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--timeout-s', type=int, default=0)
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser('send', help='发送一帧 VISION_RESULT（需要 pyserial）')
    p.add_argument('port', help='如 COM8 或 /dev/ttyUSB0')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--class', dest='cls', type=lambda s: int(s, 0), default=2,
                   help='类别码：1 人员 / 2 障碍物 / 3 目标点 / 4 低矮障碍')
    p.add_argument('--x', type=lambda s: int(s, 0), default=32768,
                   help='x 中心，归一化 0..65535（32768=正中心）')
    p.add_argument('--y', type=lambda s: int(s, 0), default=32768)
    p.add_argument('--conf', type=lambda s: int(s, 0), default=88, help='置信度 0..255')
    p.add_argument('--clear', action='store_true', help='发"目标数=0"清空，让 FSM 回 AUTO')
    p.set_defaults(func=cmd_send)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
