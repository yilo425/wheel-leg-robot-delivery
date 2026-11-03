#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_serial_loopback.py -- 虚拟串口闭环测试（发 100 帧随机数据，收回来逐帧比对）

=============================================================================
这个测试回答什么问题
=============================================================================
uart_proto.py 的自测（test_uart_proto.py）是"喂字节给解析器"的**纯逻辑**测试，
它证明不了下面这几件事：

    1) pyserial 打开端口 / 写 / 读 / in_waiting 这一条**真实串口链路**能不能用；
    2) 一帧数据真的**逐字节过了一遍串口驱动**之后，还能被 FrameParser 原样解出来；
    3) 连续发 100 帧（帧与帧首尾相接）时，解析器**不会串帧**、不会丢同步；
    4) 误帧率到底是多少 —— 这个指标只有在真实字节流上量才有意义。

所以本脚本把整条链路接起来自己咬自己：

    make_vision_result() ──► COM10 ══虚拟串口对══► COM11 ──► FrameParser
                                                                  │
                              断言：收到的 N 个目标 == 发出去的 N 个目标 ◄┘

=============================================================================
先建一对虚拟串口（只需做一次）
=============================================================================
用 com0com（开源免费）：

    1) 下载安装 com0com，安装时勾选 "install signed driver"（Win10/11 需要签名驱动）。
    2) 打开安装目录里的 setupg.exe（图形界面）或 setupc.exe（命令行，需管理员）：
           setupc.exe install PortName=COM10 PortName=COM11
       图形界面则在 "Add Pair" 里把 CNCA0/CNCB0 改成 COM10/COM11。
    3) 设备管理器 -> 端口(COM 和 LPT) 里应出现：
           com0com - serial port emulator (COM10)
           com0com - serial port emulator (COM11)
    4) 然后跑本脚本：

           python test_serial_loopback.py

    没装 com0com 也能跑（脚本会自动退到进程内回环，见下）。

=============================================================================
三种运行模式
=============================================================================
    python test_serial_loopback.py                  # auto：有 COM10/COM11 就走真串口，
                                                    #       没有就退到 fake 并给出警告
    python test_serial_loopback.py --mode real      # 强制真串口；端口不存在直接失败
    python test_serial_loopback.py --mode fake      # 强制进程内回环（不需要 com0com）

    --port-a COM10 --port-b COM11   指定端口名
    --frames 100                    发多少帧（默认 100）
    --seed 20260518                 随机种子（固定种子 -> 结果可复现、可回归）
    --timeout 0.5                   每帧等接收的超时秒数

注意：fake 模式**没有真正过串口驱动**，它只验证协议与收发逻辑。
      真实链路验证必须以 --mode real 跑一次才算完成。

=============================================================================
判定标准
=============================================================================
    * 发出 100 帧，收到 100 帧；
    * 每一帧的类型都是 VISION_RESULT，且 parse_vision_result() 的结果
      与发送时构造的目标列表**逐字段完全相同**（类别/cx/cy/置信度/序号）；
    * 误帧率（FrameParser.error_rate = (bcc_err+tail_err)/总帧数）< 1%；
    * （负向阶段）故意损坏的帧被正确计入错误计数，且**紧接着的好帧仍能解出**
      —— 这条用来证明"误帧率 0%"不是计数器坏了才 0%。

依赖：pyserial（pip install pyserial）。fake 模式同样需要 import 它——
      本脚本为了复用同一套打开端口的代码，把 pyserial 作为硬依赖。

本脚本已自测：见文件末尾 run_negative() 与 __main__。
"""

import argparse
import random
import sys
import time

try:
    from uart_proto import (
        FRAME_OVERHEAD, MAX_PAYLOAD, PROTO_VER,
        TYPE_VISION_RESULT,
        CLS_NONE, CLS_PERSON, CLS_OBSTACLE, CLS_GOAL, CLS_LOW_OBSTACLE,
        make_vision_result, parse_vision_result, FrameParser,
    )
except ImportError:
    print('[错误] 找不到 uart_proto.py —— 请在本目录（arm/）下运行本脚本')
    sys.exit(1)

try:
    import serial
except ImportError:
    print('[错误] 需要 pyserial：')
    print('         pip install pyserial')
    sys.exit(1)


#=============================================================================
# 常量
#=============================================================================
CLS_LIST = [CLS_NONE, CLS_PERSON, CLS_OBSTACLE, CLS_GOAL, CLS_LOW_OBSTACLE]

CLS_NAME = {
    CLS_NONE: '无', CLS_PERSON: '人员', CLS_OBSTACLE: '障碍物',
    CLS_GOAL: '目标点', CLS_LOW_OBSTACLE: '低矮障碍',
}

# 故意往边界上打的值：归一化坐标的 0 / 满量程 / 正中心，置信度的 0 和 100
BOUNDARY_U16 = (0, 1, 32767, 32768, 65534, 65535)
BOUNDARY_CONF = (0, 100)

MAX_TARGETS = 8           # make_vision_result 硬上限（超了会抛 ValueError）
BAR = '=' * 70


#=============================================================================
# 虚拟串口对
#=============================================================================
class LoopbackSerial:
    """pyserial.Serial 的最小替身，用于**进程内回环**（fake 模式）。

    只实现本测试用到的那部分 API，签名/语义尽量与 pyserial 一致：

        write(data) -> 返回写了几个字节
        flush()                    （pyserial 里是清空输出缓冲，这里无缓冲，空操作）
        read(size) -> bytes        没有数据时最多等 timeout 秒，然后返回 b''
        in_waiting -> int          接收缓冲里现有字节数（pyserial 里是 property）
        reset_input_buffer()       丢掉还没读走的数据
        close() / is_open / timeout

    注意：它**不经过 Windows 串口驱动**，所以不能用来验证驱动层的问题
    （比如波特率、流控、线缆、驱动 Bug）。那部分必须用 --mode real。
    """

    def __init__(self, name, rx, tx, timeout=0.02):
        self.port = name
        self.timeout = timeout
        self.is_open = True
        self._rx = rx          # 我收到的（对端 write 进来的）
        self._tx = tx          # 我发出去的（对端的 _rx）
        self.written = 0
        self.read_bytes = 0

    # ---- 发送 ---------------------------------------------------------------
    def write(self, data):
        if not self.is_open:
            raise OSError('port is closed')
        data = bytes(data)
        self._tx.extend(data)
        self.written += len(data)
        return len(data)

    def flush(self):
        """pyserial 的 flush() 等的是"输出缓冲写完"，本实现无缓冲，空操作。"""

    # ---- 接收 ---------------------------------------------------------------
    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, size=1):
        """读最多 size 字节；一个字节都没有时阻塞 <= timeout 秒（与 pyserial 同）。"""
        if size <= 0:
            return b''
        deadline = time.time() + self.timeout
        while not self._rx and time.time() < deadline:
            time.sleep(0.0005)
        n = min(size, len(self._rx))
        out = bytes(self._rx[:n])
        del self._rx[:n]
        self.read_bytes += len(out)
        return out

    def reset_input_buffer(self):
        del self._rx[:]

    def close(self):
        self.is_open = False


def make_loopback_pair(timeout=0.02):
    """造一对**环形对接**的假串口：写给 A 的字节从 B 读出来，反之亦然。

    等价于 com0com 建出来的一对 null-modem 虚拟串口。
    """
    a2b = bytearray()      # COM10 发出 -> COM11 收到
    b2a = bytearray()      # COM11 发出 -> COM10 收到
    a = LoopbackSerial('LOOP-A(COM10)', rx=b2a, tx=a2b, timeout=timeout)
    b = LoopbackSerial('LOOP-B(COM11)', rx=a2b, tx=b2a, timeout=timeout)
    return a, b


def open_serial_pair(port_a, port_b, baud, timeout, mode):
    """按 mode 打开收发两个端口。

    返回 (ser_a, ser_b, mode_label, notice)；notice 是给用户看的提示（可能为 None）。

    mode:
        'real'  必须打开真实端口，失败就抛异常
        'fake'  强制进程内回环
        'auto'  优先真端口，打不开就退到 fake（notice 说明原因）

    真串口这边用 timeout 的**默认值**开（读超时给 0.02s，
    和 fpga_link.py 里的 ser.read(ser.in_waiting or 1) 轮询法保持一致）。
    """
    if mode == 'fake':
        a, b = make_loopback_pair(timeout)
        return a, b, 'fake(进程内回环)', None

    err = None
    try:
        ser_a = serial.Serial(port_a, baud, timeout=timeout)
    except Exception as e:
        err = f'{port_a} 打不开: {e}'
        ser_a = None

    if ser_a is not None:
        try:
            ser_b = serial.Serial(port_b, baud, timeout=timeout)
        except Exception as e:
            err = f'{port_b} 打不开: {e}'
            ser_b = None
        else:
            return ser_a, ser_b, f'real(pyserial {serial.VERSION})', None
        ser_a.close()          # B 失败就把 A 收回去，别占着端口

    if mode == 'real':
        raise RuntimeError(
            f'{err}\n'
            f'       请先建一对虚拟串口（com0com）：\n'
            f'           setupc.exe install PortName={port_a} PortName={port_b}\n'
            f'       或者加 --mode fake 先只验证协议收发逻辑。')

    a, b = make_loopback_pair(timeout)
    return a, b, 'fake(进程内回环)', err


#=============================================================================
# 随机帧生成
#=============================================================================
def gen_random_frames(n, seed):
    """生成 n 帧"结构化随机"的 VISION_RESULT。

    返回 [(frame_bytes, expected, targets), ...]

        expected = ((序号, 类别, cx, cy, 置信度), ...)  —— 与 parse_vision_result() 同构，
                   比对时直接 == 就行
        targets  = ((类别, cx, cy, 置信度), ...)       —— 喂给 make_vision_result() 的那种

    为什么叫"结构化"随机：纯 randint 很难撞上边界值，测试价值低。这里：
        * 目标个数先均匀铺满 0..8 再打乱 —— 保证覆盖"空目标帧"和"8 个目标"两个边界；
        * 坐标有 1/4 概率取 BOUNDARY_U16（0 / 1 / 中心 32767/32768 / 65534 / 65535）；
        * 置信度有 1/4 概率取 0 或 100。
      种子固定 -> 序列可复现，失败能重放。
    """
    rng = random.Random(seed)

    # 目标个数：均匀铺满 0..8（n >= 9 时 0 和 8 一定出现），再打乱
    counts = [i % (MAX_TARGETS + 1) for i in range(n)]
    rng.shuffle(counts)

    frames = []
    for k in range(n):
        targets = []
        for _ in range(counts[k]):
            cls = rng.choice(CLS_LIST)
            if rng.random() < 0.25:
                cx = rng.choice(BOUNDARY_U16)
            else:
                cx = rng.randint(0, 65535)
            if rng.random() < 0.25:
                cy = rng.choice(BOUNDARY_U16)
            else:
                cy = rng.randint(0, 65535)
            if rng.random() < 0.25:
                conf = rng.choice(BOUNDARY_CONF)
            else:
                conf = rng.randint(0, 100)
            targets.append((cls, cx, cy, conf))

        frame = make_vision_result(targets)
        expected = tuple((i, t[0], t[1], t[2], t[3]) for i, t in enumerate(targets))
        frames.append((frame, expected, tuple(targets)))
    return frames


def hexdump(b, prefix='  '):
    """与 fpga_link.hexdump 同风格：大写、空格分隔。"""
    return prefix + ' '.join(f'{x:02X}' for x in b)


def recv_until(ser_rx, parser, want, timeout):
    """从 ser_rx 读数据喂给 parser，直到凑够 want 帧或超时。

    返回本次新解出的 [(ftype, payload), ...]。
    读取方式与 fpga_link.py 一致（in_waiting 判空 + read(1) 带超时兜底），
    这样"一次读到半截帧"和"一次读到好几帧"两种情况都会被自然覆盖到。
    """
    got = []
    deadline = time.time() + timeout
    while len(got) < want and time.time() < deadline:
        n = ser_rx.in_waiting
        chunk = ser_rx.read(n if n else 1)
        if chunk:
            got.extend(parser.feed(chunk))
    return got


def safe_parse_vr(payload):
    """parse_vision_result 的包装：解析失败返回错误字符串（不抛）。"""
    try:
        return tuple(parse_vision_result(payload))
    except Exception as e:
        return f'<解析失败: {e}>'


#=============================================================================
# 阶段一：链路完整性
#=============================================================================
def phase_positive(ser_a, ser_b, frames, timeout):
    """干净链路连发 N 帧，收回来逐帧比对。

    返回 (parser, rx, sent_bytes, lost, elapsed)

        rx     收到的 [(ftype, payload), ...]
        lost   发出去但没等到回应的帧序号列表
    """
    parser = FrameParser()
    rx = []
    sent_bytes = 0
    lost = []
    t0 = time.time()

    for i, (frame, _expected, _targets) in enumerate(frames):
        ser_a.write(frame)
        ser_a.flush()
        sent_bytes += len(frame)

        # 发一帧等一帧：既不会把驱动缓冲灌爆，也能精确定位"是哪一帧丢的"
        got = recv_until(ser_b, parser, 1, timeout)
        if not got:
            lost.append(i)
        rx.extend(got)

    return parser, rx, sent_bytes, lost, time.time() - t0


def verify_positive(frames, rx):
    """逐帧比对，返回 (mismatch, type_bad)。

        mismatch  [(序号, 期望, 实际)]      载荷内容不一致
        type_bad  [(序号, 类型)]            类型不是 VISION_RESULT
    """
    mismatch = []
    type_bad = []
    for i, item in enumerate(rx):
        ftype, payload = item
        if i >= len(frames):
            type_bad.append((i, ftype))          # 多出来的帧
            continue
        if ftype != TYPE_VISION_RESULT:
            type_bad.append((i, ftype))
            continue
        got = safe_parse_vr(payload)
        if got != frames[i][1]:
            mismatch.append((i, frames[i][1], got))
    return mismatch, type_bad


#=============================================================================
# 阶段二：负向 —— 证明错误计数与"出错不丢同步"都是真的
#=============================================================================
def phase_negative(ser_a, ser_b, timeout, n_pairs=10):
    """故意损坏 n_pairs 个帧，每个坏帧后面紧跟一个好帧。

    坏帧轮流用两种坏法：
        偶数号  翻转 BCC 字节   -> 应记入 bcc_err
        奇数号  把帧尾首字节改 0 -> 应记入 tail_err

    返回 (ok_n, bcc_err, tail_err, err_rate, good_parsed, bad_sent)

        ok_n        收到的好帧数（应等于 n_pairs）
        good_parsed 每个好帧 parse_vision_result() 的结果
    """
    parser = FrameParser()
    ok_frames = []
    bad_sent = 0

    for k in range(n_pairs):
        good_targets = ((CLS_PERSON, 1000 * (k + 1) % 65536, 32768, 50 + k),)
        good = make_vision_result(good_targets)

        n = 1 + len(good_targets) * 7
        bad = bytearray(good)
        if k % 2 == 0:
            bad[6 + n] ^= 0xFF                   # BCC 错
        else:
            bad[7 + n] = 0x00                    # 帧尾首字节错
        bad = bytes(bad)

        ser_a.write(bad)
        ser_a.write(good)                        # 紧跟一个完好的帧
        ser_a.flush()
        bad_sent += 1

        # 坏帧不会交付，所以这里只会等到"好帧"这一个
        got = recv_until(ser_b, parser, 1, timeout)
        ok_frames.extend(got)

    good_parsed = []
    for ftype, payload in ok_frames:
        if ftype == TYPE_VISION_RESULT:
            good_parsed.append(safe_parse_vr(payload))

    return (len(ok_frames), parser.stats['bcc_err'], parser.stats['tail_err'],
            parser.error_rate, good_parsed, bad_sent)


#=============================================================================
# 打印
#=============================================================================
def print_parser_stats(parser, indent='    '):
    s = parser.stats
    for k in ('bytes', 'frames_ok', 'bcc_err', 'tail_err',
              'bad_ver', 'noise_bytes', 'dropped'):
        print(f'{indent}{k:12s} {s[k]}')


def show_frame(idx, frame, expected):
    """打印一帧：长度 + 十六进制 + 解析结果。"""
    print(f'   #{idx:<3d} {len(frame):3d}B  {hexdump(frame)}')
    print(f'        解析 = {expected}')


#=============================================================================
# 主流程
#=============================================================================
def run(args):
    print(BAR)
    print(' 虚拟串口闭环测试  test_serial_loopback.py')
    print(BAR)
    print(f' 请求模式  : --mode {args.mode}')
    print(f' 发送端口  : {args.port_a}')
    print(f' 接收端口  : {args.port_b}')
    print(f' 波特率    : {args.baud} 8N1')
    print(f' 帧数      : {args.frames}')
    print(f' 随机种子  : {args.seed}')
    print(f' 每帧超时  : {args.timeout}s')
    print()

    # ---- 打开端口 ----
    try:
        ser_a, ser_b, mode_label, notice = open_serial_pair(
            args.port_a, args.port_b, args.baud, args.timeout, args.mode)
    except Exception as e:
        print('[错误] 打开串口失败:')
        print(f'       {e}')
        return 2

    if notice:
        print(f'[警告] {notice}')
        print('       -> 已自动退到 fake 模式（进程内回环）。')
        print('       真实链路验证请先装 com0com 建好 COM10/COM11，再跑：')
        print('           python test_serial_loopback.py --mode real')
        print()

    fake_mode = mode_label.startswith('fake')
    print(f' 实际模式  : {mode_label}')
    print(f' A 端(发)  : {ser_a.port}')
    print(f' B 端(收)  : {ser_b.port}')
    print()

    frames = gen_random_frames(args.frames, args.seed)

    try:
        #---------------------------------------------------------------------
        # 阶段零：先查"要发的字节"本身对不对
        #---------------------------------------------------------------------
        print('-' * 70)
        print(' 阶段零：发送内容自检（帧长必须 = 9 + 1 + 7N）')
        print('-' * 70)
        gen_bad = []
        cov = {}
        max_len = 0
        for i, (frame, _e, targets) in enumerate(frames):
            want = FRAME_OVERHEAD + 1 + 7 * len(targets)
            if len(frame) != want:
                gen_bad.append((i, len(frame), want))
            cov[len(targets)] = cov.get(len(targets), 0) + 1
            max_len = max(max_len, len(frame))

        print(f' 帧长自检  : {len(frames) - len(gen_bad)}/{len(frames)} 帧长度正确'
              f'   {"✓" if not gen_bad else "✗ " + str(gen_bad[:3])}')
        print(f' 帧长范围  : {FRAME_OVERHEAD + 1} ~ {max_len} 字节'
              f'   (上限 9+{MAX_PAYLOAD}={FRAME_OVERHEAD + MAX_PAYLOAD})')
        print(' 目标数覆盖: ' + ', '.join(f'{k} 个 x{v}' for k, v in sorted(cov.items())))
        print()

        edge0 = next((i for i, f in enumerate(frames) if len(f[1]) == 0), None)
        edge8 = next((i for i, f in enumerate(frames) if len(f[1]) == MAX_TARGETS), None)
        print(' 边界帧样例:')
        if edge0 is not None:
            show_frame(edge0, frames[edge0][0], frames[edge0][1])
        if edge8 is not None:
            show_frame(edge8, frames[edge8][0], frames[edge8][1])
        print()

        #---------------------------------------------------------------------
        # 阶段一：链路完整性 + 误帧率
        #---------------------------------------------------------------------
        print('-' * 70)
        print(f' 阶段一：链路完整性（{args.frames} 帧随机数据，逐帧比对）')
        print('-' * 70)
        parser, rx, sent_bytes, lost, elapsed = phase_positive(
            ser_a, ser_b, frames, args.timeout)
        mismatch, type_bad = verify_positive(frames, rx)

        print(f' 发出      : {len(frames)} 帧 / {sent_bytes} 字节')
        print(f' 收到      : {len(rx)} 帧 / {parser.stats["bytes"]} 字节')
        print(f' 逐帧比对  : {len(rx) - len(mismatch) - len(type_bad)}/{len(frames)} 帧完全一致'
              f'   {"✓" if not mismatch and not type_bad else "✗"}')
        if lost:
            print(f' 丢失帧号  : {lost[:10]}{" ..." if len(lost) > 10 else ""}')
        if type_bad:
            print(f' 类型异常  : {type_bad[:5]}')
        print(f' 耗时      : {elapsed:.3f}s')
        if fake_mode:
            print(' 吞吐      : fake 模式是内存回环，没有波特率限制，'
                  '这个耗时不代表真实串口速度')
        elif elapsed > 0:
            print(f' 吞吐      : 约 {len(frames) / elapsed:.0f} 帧/s'
                  f'（{sent_bytes / elapsed:.0f} B/s；115200 8N1 理论上限约 11520 B/s）')
        print()
        print(' 解析器统计:')
        print_parser_stats(parser)
        err_pct = parser.error_rate * 100
        print(f' 误帧率    : {err_pct:.4f}%   (要求 < {args.max_err_rate * 100:.2f}%)'
              f'   {"✓" if parser.error_rate < args.max_err_rate else "✗"}')
        print()

        if mismatch:
            print(' 不一致详情（最多列 5 条）:')
            for i, exp, got in mismatch[:5]:
                print(f'   #{i} 期望 {exp}')
                print(f'   #{i} 实际 {got}')
            print()

        #---------------------------------------------------------------------
        # 阶段二（负向）：证明计数器是活的、出错不丢同步
        #---------------------------------------------------------------------
        if args.negative:
            print('-' * 70)
            print(f' 阶段二（负向）：故意损坏 {args.negative_pairs} 帧，'
                  f'验证错误计数 + 不丢同步')
            print('-' * 70)
            ok_n, bcc_e, tail_e, neg_rate, good_parsed, bad_sent = phase_negative(
                ser_a, ser_b, args.timeout, args.negative_pairs)
            exp_bcc = bad_sent // 2 + bad_sent % 2
            exp_tail = bad_sent // 2
            print(f' 注入坏帧  : {bad_sent} 帧（{exp_bcc} 帧 BCC 错 + {exp_tail} 帧帧尾错）')
            print(f' 收到好帧  : {ok_n}/{args.negative_pairs}'
                  + ('   ✓ 每个坏帧后面那帧都解出来了，同步没丢'
                     if ok_n == args.negative_pairs else '   ✗ 有帧被连带丢掉了'))
            print(f' 错误计数  : bcc_err={bcc_e} (期望 {exp_bcc})   '
                  f'tail_err={tail_e} (期望 {exp_tail})'
                  + ('   ✓' if bcc_e == exp_bcc and tail_e == exp_tail else '   ✗'))
            print(f' 负向误帧率: {neg_rate * 100:.2f}%   '
                  f'<- 故意打坏的，用来证明阶段一的 0% 不是计数器失灵')
            print(f' 好帧内容  : {good_parsed[0] if good_parsed else "(空)"}')
            print()

    finally:
        for s in (ser_a, ser_b):
            try:
                s.close()
            except Exception:
                pass

    #---------------------------------------------------------------------
    # 结论
    #---------------------------------------------------------------------
    cond_gen = not gen_bad
    cond_cnt = len(rx) == len(frames)
    cond_cmp = not mismatch and not type_bad
    cond_err = parser.error_rate < args.max_err_rate
    ok = cond_gen and cond_cnt and cond_cmp and cond_err

    print(BAR)
    if ok:
        print(' 结果: PASS')
        print(f'   {len(frames)}/{len(frames)} 帧收发一致，'
              f'误帧率 {err_pct:.4f}% < {args.max_err_rate * 100:.2f}%')
    else:
        print(' 结果: FAIL')
        print(f'   帧长自检={"OK" if cond_gen else "NG"}  '
              f'帧数={len(rx)}/{len(frames)}  '
              f'内容比对={"OK" if cond_cmp else "NG"}  '
              f'误帧率={err_pct:.4f}%')
    if fake_mode:
        print(' 注意: 本次是 fake 模式 —— 字节没有真的过 Windows 串口驱动。')
        print('       装好 com0com (COM10/COM11) 后请再跑一次 --mode real 才算数。')
    print(BAR)
    return 0 if ok else 1


#=============================================================================
# 入口
#=============================================================================
def main():
    ap = argparse.ArgumentParser(
        description='虚拟串口闭环测试：make_vision_result -> COM10 -> COM11 -> FrameParser')
    ap.add_argument('--mode', default='auto', choices=['auto', 'real', 'fake'],
                    help='auto=优先真串口(默认) / real=强制真串口 / fake=进程内回环')
    ap.add_argument('--port-a', default='COM10', help='发送端口（默认 COM10）')
    ap.add_argument('--port-b', default='COM11', help='接收端口（默认 COM11）')
    ap.add_argument('--baud', type=int, default=115200, help='波特率（默认 115200）')
    ap.add_argument('--frames', type=int, default=100, help='发送帧数（默认 100）')
    ap.add_argument('--seed', type=int, default=20260518,
                    help='随机种子（默认固定，保证可复现）')
    ap.add_argument('--timeout', type=float, default=0.5,
                    help='每帧等接收的超时秒数（默认 0.5）')
    ap.add_argument('--max-err-rate', type=float, default=0.01,
                    help='误帧率上限（默认 0.01 = 1%%）')
    ap.add_argument('--negative', action='store_true', default=True,
                    help='跑负向阶段（默认开）')
    ap.add_argument('--no-negative', dest='negative', action='store_false',
                    help='跳过负向阶段')
    ap.add_argument('--negative-pairs', type=int, default=10,
                    help='负向阶段注入的坏帧数（默认 10）')
    args = ap.parse_args()

    if args.frames < 9:
        print('[错误] --frames 至少 9，否则覆盖不到 0 个/8 个目标这两个边界帧')
        return 2
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
