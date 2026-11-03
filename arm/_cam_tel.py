from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
print('UART2 OK')


def pack(cls, cx, cy, conf):
    p = bytes([1, 0, cls, (cx >> 8) & 0xFF, cx & 0xFF,
               (cy >> 8) & 0xFF, cy & 0xFF, conf & 0xFF])
    b = bytes([1, 2, 0, len(p)]) + p
    k = 0
    for x in b:
        k = k ^ x
    return b'\xaa\x55' + b + bytes([k]) + b'\x55\xaa'


def try_read():
    try:
        a = uart.any()
    except Exception:
        a = None
    if a is None:
        try:
            return uart.read()
        except Exception:
            return None
    if not a:
        return None
    try:
        return uart.read(a)
    except Exception:
        return None


buf = b''


def feed(d):
    global buf
    if d:
        buf = buf + d
    if len(buf) > 600:
        buf = buf[-256:]
    out = []
    while True:
        i = buf.find(b'\xaa\x55')
        if i < 0:
            buf = b''
            break
        buf = buf[i:]
        if len(buf) < 9:
            break
        ln = (buf[4] << 8) | buf[5]
        tot = 9 + ln
        if len(buf) < tot:
            break
        f = buf[:tot]
        if f[tot - 2] == 0x55 and f[tot - 1] == 0xAA:
            bcc = 0
            for x in f[2:6 + ln]:
                bcc = bcc ^ x
            if bcc == f[6 + ln]:
                out.append((f[3], f[6:6 + ln]))
        buf = buf[1:]
    return out


sensor = Sensor(id=2, width=640, height=480)
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB565)
try:
    Display.init(Display.VIRT, width=640, height=480, fps=30, to_ide=False)
except Exception as e:
    print('Display fail:', e)
MediaManager.init()
sensor.run()
print('camera up')

RED = [(0, 100, 20, 127, -20, 80)]
tel_ok = 0
red = 0
t0 = time.ticks_ms()
last_state = -1
last_dist = -1
for i in range(120):
    img = sensor.snapshot()
    blobs = img.find_blobs(RED, pixels_threshold=200, area_threshold=200,
                           merge=True, margin=15)
    if blobs:
        b = max(blobs, key=lambda z: z.pixels())
        cx = (b.cx() * 65535) // 640
        cy = (b.cy() * 65535) // 480
        uart.write(pack(2, cx, cy, 88))
        red = red + 1
    else:
        uart.write(pack(0, 32768, 32768, 0))

    for t, pl in feed(try_read()):
        if t == 0x10 and len(pl) >= 7:
            tel_ok = tel_ok + 1
            last_state = pl[0]
            last_dist = (pl[5] << 8) | pl[6]

    if i % 10 == 0:
        print('  %3d: red=%3d tel=%3d state=%d dist=%dmm'
              % (i, red, tel_ok, last_state, last_dist))
    time.sleep_ms(20)

dt = time.ticks_diff(time.ticks_ms(), t0)
print('DONE red=%d/%d telemetry=%d state=%d dist=%dmm elapsed=%dms'
      % (red, 120, tel_ok, last_state, last_dist, dt))

try:
    sensor.stop()
except Exception:
    pass
try:
    Display.deinit()
except Exception:
    pass
try:
    MediaManager.deinit()
except Exception:
    pass
print('cleanup done')
