# -*- coding: utf-8 -*-
"""
board_bringup.py -- K230 板端"不接摄像头"的最小自检：先证明串口链路对了，再上视觉

=============================================================================
为什么需要它
=============================================================================
camera_vision.py 一旦跑起来，出问题时分不清是"摄像头没出图"还是"串口没通"。
本脚本把变量拆开：不开摄像头、不初始化 Display，只做三件事——

    1) listen   收 3 秒 FPGA 的 TELEMETRY(0x10)，看帧头/BCC/状态对不对；
    2) send     依次发"障碍物 / 低矮障碍 / 空目标"三帧（手册 §4 的实测向量），
                人眼观察 FPGA 状态灯/舵机是否按预期动作；
    3) loopback 拔掉 FPGA 两根线、把 K230 排针 11(TX) 和 13(RX) 短接，
                发什么收什么 —— 专门验证 GPIO5/6 复用和波特率。

=============================================================================
接线（和 camera_vision.py 完全一致）
=============================================================================
    K230 GPIO05（排针 11）--TX--> FPGA F16 (uart_rx)
    K230 GPIO06（排针 13）--RX<-- FPGA C19 (uart_tx)
    GND 共地

    ★ GPIO5/6 是芯片 GPIO 号；排针是 11/13。别写成 GPIO11/12。

=============================================================================
用法（CanMV IDE 的"运行"里跑，或在 REPL 里 exec）
=============================================================================
    import board_bringup
    board_bringup.run(['listen', '3'])     # 收 3 秒遥测
    board_bringup.run(['send'])            # 发三帧测试向量
    board_bringup.run(['loopback'])        # 先短接 11-13 再跑
    board_bringup.run(['pins'])            # 只打印接线表

    也可以直接把本文件当脚本跑（CanMV IDE 里"运行"）：
        默认 = listen 3 秒
        sys.argv 里带 send / loopback / pins 则走对应模式。

=============================================================================
注意
=============================================================================
* send 模式要求 FPGA 已经按过 S0 进入 AUTO(2)，否则视觉帧不会触发状态迁移。
* loopback 模式必须**拔掉**接 FPGA 的两根杜邦线，否则 FPGA 会一起收到乱数据。
* 本脚本复用 camera_vision.py 里的协议函数（pack_frame / TelemetryRx），
  所以它同时也是"camera_vision 能不能 import"的冒烟测试。
"""

import sys
import time

import camera_vision as cv


#=============================================================================
# 手册 §4 的三条实测向量（2026-10-17 实机验证过）
#=============================================================================
# 注意：低矮障碍那条手册里置信度是 0x5A=90，不是 88；这里按 90 生成。
MANUAL_VECTORS = [
    ('障碍物 0x02（期望 AUTO -> AVOID）',
     cv.make_vision_result([(cv.CLS_OBSTACLE, 0x8000, 0x8000, 88)]),
     'AA 55 01 02 00 08 01 00 02 80 00 80 00 58 50 55 AA'),

    ('低矮障碍 0x04（期望触发 LIFT/抬腿）',
     cv.make_vision_result([(cv.CLS_LOW_OBSTACLE, 0x8000, 0x8000, 90)]),
     'AA 55 01 02 00 08 01 00 04 80 00 80 00 5A 54 55 AA'),

    ('空目标（期望回到 AUTO）',
     cv.make_vision_result([]),
     'AA 55 01 02 00 01 00 02 55 AA'),
]


def _sleep_ms(ms):
    if hasattr(time, 'sleep_ms'):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def _ticks_ms():
    if hasattr(time, 'ticks_ms'):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(a, b):
    if hasattr(time, 'ticks_diff'):
        return time.ticks_diff(a, b)
    return a - b


def print_pins():
    print('=' * 70)
    print('K230 <-> FPGA 接线（3 根，3.3V 直连）')
    print('  K230 GPIO05（排针 11） --TX-->  FPGA F16 (uart_rx)')
    print('  K230 GPIO06（排针 13） --RX<--  FPGA C19 (uart_tx)')
    print('  K230 GND（排针 6/9/14/20/25/30/34/39 任一） -- GND')
    print('  115200 8N1；代码里填 GPIO 号 5/6，不是排针号 11/13')
    print('=' * 70)


def check_vectors():
    """先确认本机生成的帧和手册向量逐字节一致（协议冒烟）。"""
    print('---- 协议向量自检 ----')
    all_ok = True
    for name, frame, expect_hex in MANUAL_VECTORS:
        got = cv.hexdump(frame)
        ok = (got == expect_hex)
        all_ok = all_ok and ok
        print('  [%s] %s' % ('OK' if ok else 'FAIL', name))
        if not ok:
            print('        期望: %s' % expect_hex)
            print('        实际: %s' % got)
    return all_ok


def listen(uart, seconds=3):
    """收 N 秒遥测，打印每一条，最后给出统计。"""
    print('---- 监听 TELEMETRY %d 秒 ----' % seconds)
    rx = cv.TelemetryRx()
    memo = {}
    n_ok = 0
    t0 = _ticks_ms()

    while _ticks_diff(_ticks_ms(), t0) < seconds * 1000:
        chunk = cv._read_available(uart)
        if chunk:
            for ftype, payload in rx.feed(chunk):
                if ftype != cv.TYPE_TELEMETRY:
                    continue
                try:
                    d = cv.parse_telemetry(payload)
                except Exception as e:
                    print('  [RX] 载荷错误: %s' % e)
                    continue
                n_ok += 1
                print('  [RX] %s 速度L=%d 速度R=%d 距离=%dmm r/p/y=%.2f/%.2f/%.2f'
                      % (d['state_name'], d['speed_left'], d['speed_right'],
                         d['dist_mm'], d['roll_deg'], d['pitch_deg'], d['yaw_deg']))
        _sleep_ms(10)

    st = rx.stats
    print('  收到 TELEMETRY %d 帧；好帧%d BCC错%d 帧尾错%d 版本错%d 噪声%dB'
          % (n_ok, st['frames_ok'], st['bcc_err'], st['tail_err'],
             st['bad_ver'], st['noise_bytes']))
    if n_ok == 0:
        print('  ⚠ 一帧都没收到：先查 GND、TX/RX 是否交叉、FPGA 是否在跑、')
        print('    以及代码里是不是 GPIO5/6（不是 GPIO11/12）。')
        return False
    print('  ✓ 链路通：FPGA -> K230 方向正常')
    return True


def send_vectors(uart, then_listen=5):
    """依次发三条手册向量，每帧间隔 1.5 秒，留时间看 FPGA 反应。"""
    print('---- 发送三条手册向量 ----')
    print('  前置条件：FPGA 已按 S0 进入 AUTO(2)')
    for name, frame, _expect in MANUAL_VECTORS:
        print('  >> %s' % name)
        print('     %s' % cv.hexdump(frame))
        n = uart.write(frame)
        print('     已写 %s 字节' % n)
        _sleep_ms(1500)

    if then_listen > 0:
        listen(uart, then_listen)


def loopback(uart, rounds=3):
    """TX/RX 短接自测：发什么收什么，逐字节比对。"""
    print('---- 回环自测（排针 11 <-> 13 必须短接，且拔掉 FPGA 两根线）----')
    _sleep_ms(100)
    # 清掉上电噪声
    for _i in range(20):
        if not cv._read_available(uart):
            break

    ok_all = True
    for name, frame, _expect in MANUAL_VECTORS:
        for k in range(rounds):
            # 发之前先清空，避免把上一轮的残留算进来
            for _i in range(10):
                if not cv._read_available(uart):
                    break
            uart.write(frame)

            buf = b''
            t0 = _ticks_ms()
            while _ticks_diff(_ticks_ms(), t0) < 500:
                chunk = cv._read_available(uart)
                if chunk:
                    buf += bytes(chunk)
                    if len(buf) >= len(frame):
                        break
                _sleep_ms(5)

            ok = (buf == frame)
            ok_all = ok_all and ok
            print('  [%s] %s 第%d次  发%dB 收%dB'
                  % ('OK' if ok else 'FAIL', name, k + 1, len(frame), len(buf)))
            if not ok:
                print('        发: %s' % cv.hexdump(frame))
                print('        收: %s' % cv.hexdump(buf))
            _sleep_ms(150)

    if ok_all:
        print('  ✓ 回环全对：GPIO5/6 复用、波特率、收发都正常。')
        print('    现在可以拔掉短接线，接回 FPGA 跑 listen / send。')
    else:
        print('  ⚠ 回环不一致：先确认短接的是排针 11 和 13（不是别的针），')
        print('    再确认没有别的程序占用 UART2，最后怀疑杜邦线/接触。')
    return ok_all


def run(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    mode = argv[0] if argv else 'listen'
    print_pins()

    if mode == 'pins':
        return True

    vectors_ok = check_vectors()
    if not vectors_ok:
        print('⚠ 协议向量不一致：camera_vision.py 的组帧逻辑被改坏了，先别上板。')
        return False

    uart = cv.setup_uart()

    try:
        if mode == 'listen':
            seconds = 3
            if len(argv) > 1:
                try:
                    seconds = int(argv[1])
                except Exception:
                    pass
            return listen(uart, seconds)

        if mode == 'send':
            send_vectors(uart)
            return True

        if mode == 'loopback':
            rounds = 3
            if len(argv) > 1:
                try:
                    rounds = int(argv[1])
                except Exception:
                    pass
            return loopback(uart, rounds)

        print('未知模式: %s（可用: listen / send / loopback / pins）' % mode)
        return False
    finally:
        try:
            uart.deinit()
        except Exception:
            pass


if __name__ == '__main__':
    ok = run(sys.argv[1:])
    print('RESULT: %s' % ('PASS' if ok else 'FAIL'))
