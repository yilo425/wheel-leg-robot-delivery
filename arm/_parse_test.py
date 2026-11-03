from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
time.sleep_ms(200)
print('PARSE start')

buf = b''


def feed(d):
    """增量解析。找不到帧头时**只丢掉已确认无用的字节**，绝不清空整个缓冲。"""
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


n_bytes = 0
n_frames = 0
n_tel = 0
last_state = -1
last_dist = -1
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 3000:
    try:
        d = uart.read()
    except Exception:
        d = None
    if d:
        n_bytes = n_bytes + len(d)
        for t, pl in feed(d):
            n_frames = n_frames + 1
            if t == 0x10 and len(pl) >= 7:
                n_tel = n_tel + 1
                last_state = pl[0]
                last_dist = (pl[5] << 8) | pl[6]

print('PARSE bytes=%d frames=%d tel=%d state=%d dist=%dmm'
      % (n_bytes, n_frames, n_tel, last_state, last_dist))
print('PARSE done')
