#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_camera_offline.py -- 在 PC 上验证 camera_vision.py（不需要 K230、不需要摄像头）

=============================================================================
为什么需要它
=============================================================================
camera_vision.py 只能在 CanMV MicroPython 上跑，但它里面 90% 是纯逻辑：
组帧、BCC、归一化、遥测解析、规则分类、主循环控制流。这些在 PC 上就能验证，
而且能挡住最容易犯的错：

    * 帧长写成 8+N / BCC 把帧头算进去 / 多字节写成小端；
    * 归一化坐标和 uart_proto.py 对不上（PC 参考实现和板子行为不一致）；
    * 规则分类把 0x04 低矮障碍错判成 0x02（该抬腿时不抬）；
    * 引脚又改回 GPIO11/12（实机接线是 GPIO5/6）；
    * 忘了 MediaManager.init()（真机上表现为一直取不到图）。

=============================================================================
怎么跑
=============================================================================
    cd arm
    python test_camera_offline.py

    -> 末尾打印 "RESULT: PASS (N 项)" 即全部通过；任何一项失败退出码为 1。

=============================================================================
实现方式
=============================================================================
用 sys.modules 注入假的 media.sensor / media.display / media.media / machine，
让 camera_vision 能在 CPython 里 import；再用假的 Sensor / Image / Blob / UART
把 main() 完整跑 1~3 帧，最后用**真正的 uart_proto.FrameParser** 解回来比对。
"""

import os
import struct
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


#=============================================================================
# ① 假硬件：只实现 camera_vision 用到的那部分 API
#=============================================================================
class FakeSensor:
    """假摄像头。snapshot() 永远返回同一张 FakeImage。"""
    RGB565 = 1

    def __init__(self, img=None, id=2, width=640, height=480):
        self.img = img
        self.id = id
        self.width = width
        self.height = height
        self.events = []
        self.run_count = 0
        self.stop_count = 0
        self.reset_count = 0
        self.framesize = None
        self.pixformat = None
        self.hmirror = False
        self.vflip = False

    def reset(self):
        self.reset_count += 1

    def set_framesize(self, **kwargs):
        self.framesize = (kwargs.get('width'), kwargs.get('height'))

    def set_pixformat(self, fmt):
        self.pixformat = fmt

    def set_hmirror(self, v):
        self.hmirror = bool(v)

    def set_vflip(self, v):
        self.vflip = bool(v)

    def run(self):
        self.run_count += 1
        EVENT_LOG.append('sensor.run')

    def stop(self):
        self.stop_count += 1

    def snapshot(self, chn=None):
        return self.img


class FakeDisplay:
    VIRT = 1
    ST7701 = 2
    LT9611 = 3
    init_count = 0
    deinit_count = 0
    show_count = 0
    last_init = None

    @classmethod
    def init(cls, *args, **kwargs):
        cls.init_count += 1
        cls.last_init = (args, kwargs)
        EVENT_LOG.append('display.init')

    @classmethod
    def deinit(cls):
        cls.deinit_count += 1

    @classmethod
    def show_image(cls, img, *args, **kwargs):
        cls.show_count += 1


class FakeMediaManager:
    init_count = 0
    deinit_count = 0

    @classmethod
    def init(cls):
        cls.init_count += 1
        EVENT_LOG.append('media.init')

    @classmethod
    def deinit(cls):
        cls.deinit_count += 1


EVENT_LOG = []


class FakeUART:
    """假串口：write 只记录，read 不阻塞。"""
    UART2 = 2
    EIGHTBITS = 8
    PARITY_NONE = 0
    STOPBITS_ONE = 1

    def __init__(self):
        self.tx = b''
        self.rx = b''
        self.write_calls = 0
        self.deinit_count = 0

    def any(self):
        return len(self.rx)

    def read(self, n=None):
        if not self.rx:
            return None
        if n is None or n >= len(self.rx):
            d, self.rx = self.rx, b''
            return d
        d, self.rx = self.rx[:n], self.rx[n:]
        return d

    def write(self, data):
        self.write_calls += 1
        self.tx += bytes(data)
        return len(data)

    def deinit(self):
        self.deinit_count += 1


class FakeUARTNoAny(FakeUART):
    """模拟没有 any() 的 K230 固件。"""
    any = None

    def __getattribute__(self, name):
        if name == 'any':
            raise AttributeError('any')
        return object.__getattribute__(self, name)


class FakeFPIOA:
    UART2_TXD = 1
    UART2_RXD = 2

    def __init__(self):
        self.calls = []

    def set_function(self, pin, func):
        self.calls.append((pin, func))


class FakeBlob:
    """假色块。cx/cy 是质心，rect 是外接框。"""
    def __init__(self, cx, cy, w, h, pixels=None):
        self._cx = cx
        self._cy = cy
        self._w = w
        self._h = h
        self._pixels = w * h if pixels is None else pixels

    def pixels(self):
        return self._pixels

    def area(self):
        return self._w * self._h

    def cx(self):
        return self._cx

    def cy(self):
        return self._cy

    def w(self):
        return self._w

    def h(self):
        return self._h

    def rect(self):
        return (self._cx - self._w // 2, self._cy - self._h // 2, self._w, self._h)


class FakeImage:
    def __init__(self, blobs=None, width=640, height=480):
        self.blobs = list(blobs or [])
        self._w = width
        self._h = height
        self.find_calls = 0
        self.rect_count = 0
        self.cross_count = 0

    def width(self):
        return self._w

    def height(self):
        return self._h

    def find_blobs(self, thresholds, **kwargs):
        self.find_calls += 1
        return list(self.blobs)

    def draw_rectangle(self, rect, *args, **kwargs):
        self.rect_count += 1

    def draw_cross(self, x, y, *args, **kwargs):
        self.cross_count += 1

    def get_pixel(self, x, y):
        return (1, 2, 3)


def _install_stubs():
    """把 media.* / machine 塞进 sys.modules，让 camera_vision 可以 import。"""
    media = types.ModuleType('media')
    media.__path__ = []

    sensor_mod = types.ModuleType('media.sensor')
    sensor_mod.Sensor = FakeSensor
    sensor_mod.ALIGN_UP = lambda x, a=16: ((x + a - 1) // a) * a
    sensor_mod.CAM_CHN_ID_0 = 0

    display_mod = types.ModuleType('media.display')
    display_mod.Display = FakeDisplay

    media_mod = types.ModuleType('media.media')
    media_mod.MediaManager = FakeMediaManager

    media.sensor = sensor_mod
    media.display = display_mod
    media.media = media_mod

    machine = types.ModuleType('machine')
    machine.UART = FakeUART
    machine.FPIOA = FakeFPIOA

    sys.modules['media'] = media
    sys.modules['media.sensor'] = sensor_mod
    sys.modules['media.display'] = display_mod
    sys.modules['media.media'] = media_mod
    sys.modules['machine'] = machine


_install_stubs()

import camera_vision as cv          # noqa: E402  （必须在装好假模块之后 import）
import uart_proto as up             # noqa: E402


#=============================================================================
# ② 极简测试框架
#=============================================================================
_TOTAL = 0
_FAILED = 0


def check(name, cond, detail=''):
    global _TOTAL, _FAILED
    _TOTAL += 1
    if cond:
        print('  [ OK ] %s' % name)
    else:
        _FAILED += 1
        print('  [FAIL] %s' % name)
        if detail:
            print('         %s' % detail)


def check_eq(name, got, want):
    check(name, got == want, 'got=%r want=%r' % (got, want))


def hexs(b):
    return ' '.join('%02X' % x for x in b)


#=============================================================================
# ③ 协议一致性
#=============================================================================
def test_protocol():
    print('---- 协议：与 uart_proto.py 逐字节一致 ----')

    # 引脚回归：实机是 GPIO5/6，不是 GPIO11/12
    check_eq('UART TX 引脚 = GPIO5', cv.UART_TX_PIN, 5)
    check_eq('UART RX 引脚 = GPIO6', cv.UART_RX_PIN, 6)
    check_eq('摄像头 sensor id = 2 (CSI2)', cv.SENSOR_ID, 2)
    check_eq('帧固定开销 = 9', up.FRAME_OVERHEAD, 9)

    # pack_frame：长度 0/1/7/8/64，类型 1/2/3/4/0x10
    ok = True
    detail = ''
    for ftype in (0x01, 0x02, 0x03, 0x04, 0x10):
        for n in (0, 1, 7, 8, 64):
            payload = bytes([(i * 37 + ftype) & 0xFF for i in range(n)])
            a = cv.pack_frame(ftype, payload)
            b = up.pack_frame(ftype, payload)
            if a != b:
                ok = False
                detail = 'ftype=0x%02X n=%d\ncv=%s\nup=%s' % (ftype, n, hexs(a), hexs(b))
                break
    check('pack_frame 与 uart_proto 逐字节一致（25 组）', ok, detail)

    # make_vision_result：空 / 1 个 / 8 个 / 随机多目标
    cases = [
        [],
        [(up.CLS_OBSTACLE, 32768, 32768, 88)],
        [(i, (i * 8191) % 65536, (i * 4093) % 65536, i * 10) for i in range(8)],
        [(up.CLS_PERSON, 0, 0, 0), (up.CLS_LOW_OBSTACLE, 65535, 65535, 100)],
        [(up.CLS_STEP, 12345, 54321, 66)],
    ]
    ok = True
    detail = ''
    for targets in cases:
        a = cv.make_vision_result(targets)
        b = up.make_vision_result(targets)
        if a != b:
            ok = False
            detail = 'targets=%r\ncv=%s\nup=%s' % (targets, hexs(a), hexs(b))
            break
    check('make_vision_result 与 uart_proto 逐字节一致（5 组）', ok, detail)

    # 手册 §4 的三条实测向量
    manual = [
        ([(up.CLS_OBSTACLE, 0x8000, 0x8000, 88)],
         'AA 55 01 02 00 08 01 00 02 80 00 80 00 58 50 55 AA'),
        ([(up.CLS_LOW_OBSTACLE, 0x8000, 0x8000, 90)],
         'AA 55 01 02 00 08 01 00 04 80 00 80 00 5A 54 55 AA'),
        ([], 'AA 55 01 02 00 01 00 02 55 AA'),
    ]
    for targets, want in manual:
        got = hexs(cv.make_vision_result(targets))
        check('手册向量 %s' % want[:11], got == want, 'got=%s' % got)

    # 低矮障碍 0x05：FPGA main_fsm 已实现，ARM 侧必须能组出来
    got = cv.make_vision_result([(cv.CLS_STEP, 0x8000, 0x8000, 90)])
    check('类别 0x05 台阶可组帧', got[8] == 0x05 and got[3] == 0x02, hexs(got))


def test_norm():
    print('---- 归一化：与 uart_proto.norm_to_u16 逐点一致 ----')
    ok = True
    bad = None
    for total in (640, 480):
        for px in range(0, total + 1):
            a = cv.norm_to_u16(px, total)
            b = up.norm_to_u16(px, total)
            if a != b:
                ok = False
                bad = 'px=%d total=%d cv=%d up=%d' % (px, total, a, b)
                break
        if not ok:
            break
    check('0..640 / 0..480 全部像素点一致', ok, bad or '')
    check_eq('640 宽正中心 = 32768', cv.norm_to_u16(320, 640), 32768)
    check_eq('越界钳到 65535', cv.norm_to_u16(9999, 640), 65535)
    check_eq('负数钳到 0', cv.norm_to_u16(-5, 640), 0)
    check_eq('total<=0 返回 0（不抛异常）', cv.norm_to_u16(10, 0), 0)


def test_telemetry():
    print('---- 遥测：解析与流式重同步 ----')
    payload = struct.pack('>BhhHhhh', 3, -100, 200, 1234, -500, 600, -700)
    frame = up.pack_frame(up.TYPE_TELEMETRY, payload)

    d_cv = cv.parse_telemetry(payload)
    d_up = up.parse_telemetry(payload)
    keys = ('state', 'state_name', 'speed_left', 'speed_right', 'dist_mm',
            'roll_deg', 'pitch_deg', 'yaw_deg')
    check('parse_telemetry 8 个字段与 uart_proto 一致',
          all(d_cv[k] == d_up[k] for k in keys),
          'cv=%r up=%r' % (d_cv, d_up))

    # 逐字节喂（最恶劣的切分），仍要解出完整帧
    rx = cv.TelemetryRx()
    out = []
    for i in range(len(frame)):
        out.extend(rx.feed(frame[i:i + 1]))
    check_eq('逐字节喂也能解出 1 帧', len(out), 1)
    check('解析出的类型 = 0x10', out and out[0][0] == cv.TYPE_TELEMETRY)
    check('解析出的载荷 = 原载荷', out and out[0][1] == payload)

    # BCC 错后紧跟好帧：只退 2 字节，不能把好帧一起吞掉
    bad = bytearray(frame)
    bad[-3] ^= 0xFF
    data = bytes(bad) + frame
    rx2 = cv.TelemetryRx()
    got = rx2.feed(data)
    check_eq('坏帧被丢、后面的好帧仍收到', len(got), 1)
    check_eq('坏帧计入 BCC 错', rx2.stats['bcc_err'], 1)

    # 前导噪声 + 半截帧
    rx3 = cv.TelemetryRx()
    got = rx3.feed(b'\x00\xAA\x13\xAA\x55' + frame[:7])
    got += rx3.feed(frame[7:])
    check_eq('前导噪声 + 半截帧仍能解出', len(got), 1)


#=============================================================================
# ④ 识别逻辑
#=============================================================================
def test_detect():
    print('---- 识别：取最大色块 / 规则分类 / 置信度 ----')

    big = FakeBlob(320, 240, 80, 80, 6400)
    small = FakeBlob(100, 100, 20, 20, 400)
    img = FakeImage([small, big])
    check('find_red_blob 返回像素最多的色块', cv.find_red_blob(img) is big)

    # 规则：低矮障碍要求"画面下 40% + 宽高比 >= 1.2"，否则按普通障碍物
    rules = [
        ('红色低矮障碍', cv.CLS_LOW_OBSTACLE, [(0, 100, 30, 127, -10, 70)],
         0.60, 1.0, 1.2, None),
        ('红色障碍物', cv.CLS_OBSTACLE, [(0, 100, 30, 127, -10, 70)],
         None, None, None, None),
    ]

    low = FakeBlob(320, 400, 120, 40, 4800)       # 贴地、扁宽 -> 0x04
    hit = cv.classify(FakeImage([low]), rules)
    check('贴地扁宽红块 -> 0x04 低矮障碍', hit and hit[0] == cv.CLS_LOW_OBSTACLE,
          'hit=%r' % (hit,))

    normal = FakeBlob(320, 100, 60, 60, 3600)     # 画面中上部、方形 -> 0x02
    hit = cv.classify(FakeImage([normal]), rules)
    check('画面中上部方块 -> 0x02 普通障碍', hit and hit[0] == cv.CLS_OBSTACLE,
          'hit=%r' % (hit,))

    tiny = FakeBlob(320, 240, 10, 10, 100)        # 低于像素门限 -> 忽略
    check('低于像素门限 -> None', cv.classify(FakeImage([tiny]), rules) is None)

    # 置信度：80x80=6400px 占 640x480 的 2.08% -> 100 分封顶
    target, (px, py) = cv.blob_to_target(FakeImage([]), big, cv.CLS_OBSTACLE)
    check_eq('大色块置信度封顶 100', target[3], 100)
    check_eq('质心像素回传', (px, py), (320, 240))
    check_eq('中心归一化 x = 32768', target[1], 32768)
    check_eq('中心归一化 y = 32768', target[2], 32768)

    # 20x20=400px 占 0.13% -> 6.5 分 -> 7
    small_img = FakeImage([])
    target2, _ = cv.blob_to_target(small_img, small, cv.CLS_OBSTACLE)
    check_eq('小色块置信度 = 7', target2[3], 7)


def test_read_available():
    print('---- 串口接收：兼容没有 any() 的固件 ----')
    u = FakeUART()
    u.rx = b'\x01\x02'
    check_eq('有 any() 时按数量读', cv._read_available(u), b'\x01\x02')
    check('读空后返回 None', cv._read_available(u) is None)

    u2 = FakeUARTNoAny()
    u2.rx = b'\x03\x04'
    check_eq('没有 any() 时退化为 read()', cv._read_available(u2), b'\x03\x04')
    check('没有 any() 且无数据时返回 None', cv._read_available(u2) is None)


#=============================================================================
# ⑤ 主循环端到端（假硬件，但走完整的 main() 控制流）
#=============================================================================
def _patch_runtime():
    """把 main() 里用到的板端函数/模块换成假实现。"""
    cv.os = types.SimpleNamespace(
        exitpoint=lambda *a, **k: None,
        EXITPOINT_ENABLE=1,
        EXITPOINT_ENABLE_SLEEP=2,
    )
    cv.time = types.SimpleNamespace(
        ticks_ms=lambda: 0,
        ticks_diff=lambda a, b: 0,
        sleep_ms=lambda ms: None,
        sleep=lambda s: None,
    )
    cv.sys = types.SimpleNamespace(print_exception=lambda e: None)
    cv.keep_10hz = lambda t0: None
    cv.DEBUG_PRINT = False
    cv.USE_RX_TELEMETRY = False
    cv.FORCE_CLASS = None


def test_main_end_to_end():
    print('---- 主循环端到端：识别 -> 组帧 -> 写串口 ----')
    _patch_runtime()

    blob = FakeBlob(320, 240, 80, 80, 6400)
    img = FakeImage([blob])
    sensor = FakeSensor(img)
    uart = FakeUART()

    cv.setup_uart = lambda: uart
    cv.setup_camera = lambda: sensor
    cv.USE_DISPLAY = False
    cv.RUN_FRAMES = 3
    cv.FPS_PRINT_EVERY = 0

    cv.main()

    check_eq('主循环跑了 3 帧（写 3 次串口）', uart.write_calls, 3)
    check_eq('sensor.run 调用 1 次', sensor.run_count, 1)
    check_eq('sensor.stop 调用 1 次', sensor.stop_count, 1)
    check_eq('MediaManager.init 调用 1 次', FakeMediaManager.init_count, 1)
    check_eq('MediaManager.deinit 调用 1 次', FakeMediaManager.deinit_count, 1)

    parser = up.FrameParser()
    frames = parser.feed(uart.tx)
    vision = [(t, p) for t, p in frames if t == up.TYPE_VISION_RESULT]
    check_eq('解出 3 帧 VISION_RESULT', len(vision), 3)
    check_eq('接收侧 0 错误', parser.stats['bcc_err'] + parser.stats['tail_err'], 0)

    ok_targets = True
    for _t, payload in vision:
        targets = up.parse_vision_result(payload)
        if len(targets) != 1:
            ok_targets = False
            break
        idx, cls, cx, cy, conf = targets[0]
        if (cls, cx, cy, conf) != (up.CLS_OBSTACLE, 32768, 32768, 100):
            ok_targets = False
            break
    check('每帧都是 1 个 0x02 中心目标且置信度 100', ok_targets)

    # 无目标时：必须发"空目标帧"（目标数=0），不能不发
    img2 = FakeImage([])
    sensor2 = FakeSensor(img2)
    uart2 = FakeUART()
    cv.setup_camera = lambda: sensor2
    cv.setup_uart = lambda: uart2
    cv.RUN_FRAMES = 2
    cv.main()

    parser2 = up.FrameParser()
    frames2 = [p for t, p in parser2.feed(uart2.tx) if t == up.TYPE_VISION_RESULT]
    check_eq('无目标时发 2 帧', len(frames2), 2)
    check('空目标帧载荷 = 0x00',
          all(p == b'\x00' for p in frames2), 'payloads=%r' % (frames2,))


def test_init_order():
    print('---- 初始化顺序：Display -> MediaManager -> sensor.run ----')
    _patch_runtime()
    del EVENT_LOG[:]

    img = FakeImage([FakeBlob(320, 240, 40, 40, 1600)])
    sensor = FakeSensor(img)
    uart = FakeUART()

    cv.setup_uart = lambda: uart
    cv.setup_camera = lambda: sensor
    cv.USE_DISPLAY = True
    cv.RUN_FRAMES = 1
    cv.FPS_PRINT_EVERY = 0

    cv.main()

    check_eq('顺序 = display.init -> media.init -> sensor.run',
             EVENT_LOG, ['display.init', 'media.init', 'sensor.run'])
    check('Display 只 init/deinit 各 1 次',
          FakeDisplay.init_count == 1 and FakeDisplay.deinit_count == 1,
          'init=%d deinit=%d' % (FakeDisplay.init_count, FakeDisplay.deinit_count))


#=============================================================================
# 入口
#=============================================================================
def main():
    print('=' * 70)
    print(' camera_vision.py 离线回归（假硬件 + 真 uart_proto 解析器）')
    print('=' * 70)
    test_protocol()
    test_norm()
    test_telemetry()
    test_detect()
    test_read_available()
    test_main_end_to_end()
    test_init_order()
    print('-' * 70)
    if _FAILED == 0:
        print('RESULT: PASS  (%d 项)' % _TOTAL)
        return 0
    print('RESULT: FAIL  (%d/%d 项失败)' % (_FAILED, _TOTAL))
    return 1


if __name__ == '__main__':
    sys.exit(main())
