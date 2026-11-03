from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
time.sleep_ms(200)
print('SETCFG start')


def frame(ftype, payload):
    b = bytes([1, ftype, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF]) + payload
    k = 0
    for x in b:
        k = k ^ x
    return b'\xaa\x55' + b + bytes([k]) + b'\x55\xaa'


def send_config(pid, val):
    uart.write(frame(0x04, bytes([pid, 2, (val >> 8) & 0xFF, val & 0xFF])))
    print('SETCFG id=0x%02X val=%d' % (pid, val))
    time.sleep_ms(120)


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


# 停车距离 200 -> 50（让 5cm 的物体不再触发 AVOID）
send_config(0x02, 50)
send_config(0x04, 100)

st = -1
ds = -1
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 2000:
    try:
        d = uart.read()
    except Exception:
        d = None
    for t, pl in feed(d):
        if t == 0x10 and len(pl) >= 7:
            st = pl[0]
            ds = (pl[5] << 8) | pl[6]
    time.sleep_ms(20)

print('SETCFG done state=%d dist=%dmm' % (st, ds))
