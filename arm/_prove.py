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
time.sleep_ms(200)
print('PRV start')


def frame(ftype, payload):
    b = bytes([1, ftype, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF]) + payload
    k = 0
    for x in b:
        k = k ^ x
    return b'\xaa\x55' + b + bytes([k]) + b'\x55\xaa'


def send_config(pid, val):
    uart.write(frame(0x04, bytes([pid, 2, (val >> 8) & 0xFF, val & 0xFF])))
    print('CFG id=0x%02X val=%d' % (pid, val))


def send_vision(cls, cx, cy, conf):
    uart.write(frame(0x02, bytes([1, 0, cls, (cx >> 8) & 0xFF, cx & 0xFF,
                                  (cy >> 8) & 0xFF, cy & 0xFF, conf & 0xFF])))


buf = b''


def feed(d):
    global buf
    if d:
        buf = buf + d
    if len(buf) > 2000:
        buf = buf[-1000:]
    out = []
    while True:
        i = buf.find(b'\xaa\x55')
        if i < 0:
            if len(buf) > 1:
                buf = buf[-1:]
            break
        buf = buf[i:]
        if len(buf) < 9:
            break
        ln = (buf[4] << 8) | buf[5]
        if ln > 64:
            buf = buf[1:]
            continue
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
                buf = buf[tot:]
                continue
        buf = buf[1:]
    return out


def read_tel():
    st = -1
    ds = -1
    try:
        d = uart.read()
    except Exception:
        d = None
    for t, pl in feed(d):
        if t == 0x10 and len(pl) >= 7:
            st = pl[0]
            ds = (pl[5] << 8) | pl[6]
    return st, ds


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
g_state = -1
g_dist = -1


def watch(name, ms, send_vf):
    global g_state, g_dist
    t0 = time.ticks_ms()
    nxt = 0
    red_n = 0
    nfr = 0
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        img = sensor.snapshot()
        blobs = img.find_blobs(RED, pixels_threshold=200, area_threshold=200,
                               merge=True, margin=15)
        if blobs:
            red_n = red_n + 1
            b = max(blobs, key=lambda z: z.pixels())
            if send_vf:
                send_vision(2, (b.cx() * 65535) // 640, (b.cy() * 65535) // 480, 88)
        else:
            if send_vf:
                send_vision(0, 32768, 32768, 0)
        nfr = nfr + 1
        st, ds = read_tel()
        if st >= 0:
            g_state = st
            g_dist = ds
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= nxt:
            print('  %-10s t=%4dms red=%d state=%d dist=%dmm'
                  % (name, el, 1 if blobs else 0, g_state, g_dist))
            nxt = nxt + 500
        time.sleep_ms(15)
    print('PHASE %-10s frames=%d red=%d -> state=%d dist=%dmm'
          % (name, nfr, red_n, g_state, g_dist))


# ---------- 1) 先把停车距离降到 50mm、减速距离 100mm ----------
send_config(0x02, 50)
time.sleep_ms(100)
send_config(0x04, 100)
time.sleep_ms(400)

# ---------- 2) 等状态变成 AUTO(2)：CONFIG 生效则 51>50 -> 不会一按就进 AVOID ----------
print('WAIT_AUTO start (press S0 to leave ESTOP / enter AUTO)')
tw = time.ticks_ms()
nxt = 0
while time.ticks_diff(time.ticks_ms(), tw) < 30000:
    st, ds = read_tel()
    if st >= 0:
        g_state = st
        g_dist = ds
    el = time.ticks_diff(time.ticks_ms(), tw)
    if el >= nxt:
        print('  %-10s t=%4dms state=%d dist=%dmm'
              % ('WAIT_AUTO', el, g_state, g_dist))
        nxt = nxt + 1000
    if g_state == 2:
        break
    time.sleep_ms(30)
print('WAIT_AUTO done state=%d dist=%dmm after %dms'
      % (g_state, g_dist, time.ticks_diff(time.ticks_ms(), tw)))

# ---------- 3) 发视觉帧（红色 -> 0x02）：应 2 -> 3 ----------
watch('B_vision', 6000, True)

# ---------- 4) 改回默认 200mm ----------
send_config(0x02, 200)
time.sleep_ms(100)
send_config(0x04, 500)
print('PRV SUMMARY state=%d dist=%dmm' % (g_state, g_dist))

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
print('PRV done')
