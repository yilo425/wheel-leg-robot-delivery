from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
import time

# ---- UART2: GPIO5 -> 排针11 (FPGA F16), GPIO6 <- 排针13 (FPGA C19)
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

# ---- camera: GC2093 on CSI2
sensor = Sensor(id=2, width=640, height=480)
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB565)
try:
    Display.init(Display.VIRT, width=640, height=480, fps=30, to_ide=False)
    print('Display VIRT OK')
except Exception as e:
    print('Display init fail (continue):', e)
MediaManager.init()
sensor.run()
print('camera running, CSI2 640x480 RGB565')

RED = [(0, 100, 20, 127, -20, 80)]
sent = 0
found = 0
t0 = time.ticks_ms()
for i in range(120):
    img = sensor.snapshot()
    blobs = img.find_blobs(RED, pixels_threshold=200, area_threshold=200,
                           merge=True, margin=15)
    if blobs:
        b = max(blobs, key=lambda z: z.pixels())
        cx = (b.cx() * 65535) // 640
        cy = (b.cy() * 65535) // 480
        uart.write(pack(2, cx, cy, 88))
        found = found + 1
        if i % 10 == 0:
            print('  %d: RED px=%d cx=%d -> 0x02' % (i, b.pixels(), cx))
    else:
        uart.write(pack(0, 32768, 32768, 0))
        if i % 10 == 0:
            print('  %d: no blob' % i)
    sent = sent + 1
    time.sleep_ms(20)

dt = time.ticks_diff(time.ticks_ms(), t0)
print('DONE frames=%d red=%d elapsed_ms=%d' % (sent, found, dt))

try:
    sensor.stop()
except Exception as e:
    print('sensor.stop fail:', e)
try:
    Display.deinit()
except Exception:
    pass
try:
    MediaManager.deinit()
except Exception:
    pass
print('cleanup done')
