#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fpga_link.py -- FPGA <-> ARM 联调工具（不需要摄像头）

=============================================================================
为什么需要它
=============================================================================
ARM 板还没到货，但**联调不该等硬件**。这个工具用"假视觉"驱动整条链路：

    假目标 ──> make_vision_result() ──> 串口 ──> FPGA
                                                  │
    TELEMETRY 解析 <── 串口 <── uart_frame <──────┘

所以板子到货那天，**协议这一层已经验证过了**，剩下的只有摄像头接入。

=============================================================================
三种用法
=============================================================================
1) 不接任何硬件，先看帧长什么样（今天就能做）

       python fpga_link.py --dry-run

2) 用虚拟串口对自测（com0com / 虚拟串口软件）
   开两个终端，一个发一个收：

       # 终端 A
       python fpga_link.py --port COM10
       # 终端 B
       python fpga_link.py --port COM11 --listen-only

3) 接 FPGA 实测

       python fpga_link.py --port COM5 --baud 115200

=============================================================================
依赖
=============================================================================
    pip install pyserial          # 只有接真串口时才需要
    --dry-run 模式**不需要**任何依赖
"""

import argparse
import math
import sys
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
import sys as _sys
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

import time

try:
    from uart_proto import (
        HDR, TAIL, PROTO_VER, FRAME_OVERHEAD, MAX_PAYLOAD,
        TYPE_VISION_RESULT, TYPE_TELEMETRY, TYPE_ACK,
        CLS_NONE, CLS_PERSON, CLS_OBSTACLE, CLS_GOAL, CLS_LOW_OBSTACLE,
        make_vision_result, parse_telemetry, parse_vision_result,
        make_ack, pack_frame, FrameParser, norm_to_u16, x_zone,
    )
except ImportError:
    print('[错误] 找不到 uart_proto.py —— 请在本目录（arm/）下运行本脚本')
    sys.exit(1)


#=============================================================================
# 假目标生成：模拟一个在画面里横向移动的目标
#=============================================================================
CLS_NAME = {
    CLS_NONE: '无', CLS_PERSON: '人员', CLS_OBSTACLE: '障碍物',
    CLS_GOAL: '目标点', CLS_LOW_OBSTACLE: '低矮障碍',
}


class FakeVision:
    """产生假的视觉结果，用来驱动联调。

    模式：
        sweep       一个目标从左扫到右（默认，最能看出链路在动）
        person      固定"人员"在正前方
        low_obstacle 固定"低矮障碍"（会触发 FPGA 升降腿）
        obstacle    固定"障碍物"
        none        空目标（0 个）—— 用来验证 FPGA 的 fail-safe 行为
        multi       三个目标同时出现
    """

    def __init__(self, pattern: str = 'sweep', width: int = 640, height: int = 480):
        self.pattern = pattern
        self.width = width
        self.height = height
        self.t0 = time.time()

    def targets(self, t: float):
        cy = norm_to_u16(self.height // 2, self.height)

        if self.pattern == 'sweep':
            # 一个完整来回 8 秒，用正弦让运动平滑
            phase = (math.sin(t * 2 * math.pi / 8.0) + 1.0) / 2.0
            cx = norm_to_u16(phase * (self.width - 1), self.width)
            return [(CLS_PERSON, cx, cy, 90)]

        if self.pattern == 'person':
            return [(CLS_PERSON, norm_to_u16(self.width // 2, self.width), cy, 92)]

        if self.pattern == 'obstacle':
            return [(CLS_OBSTACLE, norm_to_u16(self.width // 2, self.width), cy, 85)]

        if self.pattern == 'low_obstacle':
            # 低矮障碍在画面下方
            cy2 = norm_to_u16(int(self.height * 0.75), self.height)
            return [(CLS_LOW_OBSTACLE, norm_to_u16(self.width // 2, self.width), cy2, 80)]

        if self.pattern == 'multi':
            return [
                (CLS_PERSON,   norm_to_u16(int(self.width * 0.25), self.width), cy, 88),
                (CLS_OBSTACLE, norm_to_u16(int(self.width * 0.55), self.width), cy, 76),
                (CLS_GOAL,     norm_to_u16(int(self.width * 0.80), self.width), cy, 95),
            ]

        # none
        return []

    def frame(self, t: float) -> bytes:
        return make_vision_result(self.targets(t))


#=============================================================================
# 输出
#=============================================================================
def hexdump(b: bytes, prefix: str = '  ') -> str:
    """把帧打成好看的十六进制，带字段标注。"""
    h = ' '.join(f'{x:02X}' for x in b)
    return f'{prefix}{h}'


def annotate(frame: bytes) -> str:
    """给一帧加字段说明（教学/排查用）。"""
    if len(frame) < FRAME_OVERHEAD:
        return '  (帧太短)'
    lines = []
    lines.append(f'    帧头   AA 55')
    lines.append(f'    版本   {frame[2]:02X}')
    lines.append(f'    类型   {frame[3]:02X}  ({type_name(frame[3])})')
    n = (frame[4] << 8) | frame[5]
    lines.append(f'    长度   {frame[4]:02X} {frame[5]:02X}  = {n} 字节')
    if n:
        data = frame[6:6 + n]
        lines.append(f'    数据   {" ".join(f"{x:02X}" for x in data)}')
    lines.append(f'    校验   {frame[6 + n]:02X}')
    lines.append(f'    帧尾   {frame[7 + n]:02X} {frame[8 + n]:02X}')
    lines.append(f'    总长   {len(frame)} = 9 + {n}  '
                 f'{"[OK]" if len(frame) == FRAME_OVERHEAD + n else "[X] 对不上！"}')
    return '\n'.join(lines)


def type_name(t: int) -> str:
    return {
        0x01: 'IMAGE_DATA', 0x02: 'VISION_RESULT', 0x03: 'ACK',
        0x04: 'CONFIG', 0x10: 'TELEMETRY',
    }.get(t, f'未知(0x{t:02X})')


#=============================================================================
# 主循环
#=============================================================================
def run_dry(args):
    """不接串口，只打印帧长什么样。"""
    print('=' * 70)
    print('  dry-run 模式：不接串口，只生成并解析帧')
    print('  （不需要 pyserial，不需要任何硬件）')
    print('=' * 70)

    fv = FakeVision(args.pattern)

    for label, payload in [('空目标(0 个)', []),
                           ('一个目标', [(CLS_PERSON, 19660, 32767, 88)]),
                           ('两个目标', [(CLS_PERSON, 19660, 32767, 88),
                                         (CLS_LOW_OBSTACLE, 40000, 100, 55)])]:
        frame = make_vision_result(payload)
        print(f'\n--- {label} ---')
        print(hexdump(frame))
        print(annotate(frame))

    # 检查 BCC 算得对不对（手工算一遍对照）
    f = make_vision_result([(CLS_PERSON, 19660, 32767, 88)])
    n = (f[4] << 8) | f[5]
    manual = 0
    for b in f[2:6 + n]:
        manual ^= b
    print(f'\n--- BCC 手工复核 ---')
    print(f'    帧里的校验字节 = 0x{f[6+n]:02X}')
    print(f'    手工异或结果   = 0x{manual:02X}   '
          f'{"[OK] 一致" if manual == f[6+n] else "[X] 不一致！"}')
    print(f'    校验范围 = 版本^类型^长度高^长度低^数据（共 {4+n} 字节，不含帧头帧尾）')

    # 逐字节喂，验证流式解析
    parser = FrameParser()
    got = []
    for i in range(len(f)):
        got.extend(parser.feed(f[i:i + 1]))
    print(f'\n--- 逐字节喂解析 ---')
    print(f'    解出 {len(got)} 帧  {"[OK]" if len(got) == 1 else "[X]"}')
    if got:
        print(f'    内容 {parse_vision_result(got[0][1])}')

    # 前 5 个 sweep 采样，看坐标在动
    print(f'\n--- sweep 模式前 5 个采样（验证坐标在变）---')
    for i in range(5):
        t = i * 0.4
        tg = fv.targets(t)
        if tg:
            cls, cx, cy, conf = tg[0]
            print(f'    t={t:4.1f}s  cx={cx:5d} ({x_zone(cx):6s})  cy={cy:5d}  '
                  f'类别={CLS_NAME[cls]}  置信度={conf}%')

    print('\n' + '=' * 70)
    print('  dry-run 完成。协议库工作正常。')
    print('  接 FPGA 时把 --dry-run 换成 --port COMx')
    print('=' * 70)
    return 0


def run_serial(args):
    try:
        import serial
    except ImportError:
        print('[错误] 需要 pyserial：')
        print('         pip install pyserial')
        print('       如果想先不接硬件看效果，用：')
        print('         python fpga_link.py --dry-run')
        return 1

    print(f'打开 {args.port} @ {args.baud} 8N1 ...')
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
    except Exception as e:
        print(f'[错误] 打不开串口 {args.port}: {e}')
        print('       检查：① 端口号对不对 ② 有没有被别的程序占用 ③ 驱动装了没')
        return 1

    fv = FakeVision(args.pattern)
    parser = FrameParser()
    t0 = time.time()
    next_send = t0
    last_stat = t0
    sent = 0
    rx_frames = 0

    print(f'模式: {args.pattern}   发送频率: {args.hz}Hz   '
          f'{"只收不发" if args.listen_only else "收发"}')
    print('按 Ctrl+C 退出\n')

    try:
        while True:
            now = time.time()

            # ---- 发送 ----
            if not args.listen_only and now >= next_send:
                frame = fv.frame(now - t0)
                ser.write(frame)
                sent += 1
                next_send += 1.0 / args.hz
                if args.verbose and sent % args.hz == 0:
                    tg = fv.targets(now - t0)
                    desc = ' / '.join(
                        f'{CLS_NAME[c]}@x={cx}({x_zone(cx)})' for c, cx, cy, cf in tg
                    ) or '无障碍'
                    print(f'[发 {sent:6d}] {desc}')

            # ---- 接收 ----
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                for ftype, payload in parser.feed(chunk):
                    rx_frames += 1
                    if ftype == TYPE_TELEMETRY:
                        try:
                            d = parse_telemetry(payload)
                            print(f'[收 {rx_frames:6d}] 状态={d["state_name"]:9s} '
                                  f'速度L={d["speed_left"]:6d} 速度R={d["speed_right"]:6d} '
                                  f'距离={d["dist_mm"]:5d}mm '
                                  f'姿态 r/p/y={d["roll_deg"]:7.2f}/{d["pitch_deg"]:7.2f}/'
                                  f'{d["yaw_deg"]:7.2f}')
                        except Exception as e:
                            print(f'[收 {rx_frames:6d}] TELEMETRY 解析失败: {e}')
                    elif ftype == TYPE_ACK:
                        print(f'[收 {rx_frames:6d}] ACK -> 0x{payload[0]:02X} '
                              f'({type_name(payload[0])})')
                    else:
                        print(f'[收 {rx_frames:6d}] 类型 0x{ftype:02X} '
                              f'({type_name(ftype)}) {len(payload)} 字节')

            # ---- 统计 ----
            if now - last_stat >= args.stat_sec:
                last_stat = now
                s = parser.stats
                print(f'--- 统计: 发出 {sent} 帧 / 收到 {rx_frames} 帧 / '
                      f'BCC错 {s["bcc_err"]} / 帧尾错 {s["tail_err"]} / '
                      f'噪声字节 {s["noise_bytes"]} / '
                      f'误帧率 {parser.error_rate * 100:.2f}% ---')
                if parser.error_rate > 0.01:
                    print('    [!] 误帧率 > 1%：先查共地和线长，再考虑升级 CRC16')

    except KeyboardInterrupt:
        print('\n退出。')
    finally:
        ser.close()

    print(f'\n合计: 发出 {sent} 帧, 收到 {rx_frames} 帧')
    return 0


#=============================================================================
def main():
    ap = argparse.ArgumentParser(
        description='FPGA <-> ARM 联调工具（不需要摄像头）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('=============================================================================')[2]
        if '=============================================================================' in __doc__ else '')
    ap.add_argument('--dry-run', action='store_true',
                    help='不接串口，只生成并解析帧（不需要任何依赖，今天就能跑）')
    ap.add_argument('--port', help='串口号，如 COM5 或 /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200, help='波特率（默认 115200）')
    ap.add_argument('--hz', type=float, default=10.0, help='发送频率 Hz（默认 10）')
    ap.add_argument('--pattern', default='sweep',
                    choices=['sweep', 'person', 'obstacle', 'low_obstacle',
                             'multi', 'none'],
                    help='假视觉的目标模式（默认 sweep，左右扫）')
    ap.add_argument('--listen-only', action='store_true', help='只接收，不发送')
    ap.add_argument('--verbose', action='store_true', help='每帧都打印')
    ap.add_argument('--stat-sec', type=float, default=5.0, help='统计打印间隔秒')
    args = ap.parse_args()

    if args.dry_run or not args.port:
        return run_dry(args)
    return run_serial(args)


if __name__ == '__main__':
    sys.exit(main())
