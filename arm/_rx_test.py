from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
print('RX start')

has_any = hasattr(uart, 'any')
print('has_any=%d' % (1 if has_any else 0))

n_bytes = 0
n_reads = 0
first = None
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 3000:
    d = None
    if has_any:
        try:
            a = uart.any()
        except Exception:
            a = 0
        if a:
            try:
                d = uart.read(a)
            except Exception:
                d = None
    else:
        try:
            d = uart.read()
        except Exception:
            d = None
    if d:
        n_reads = n_reads + 1
        n_bytes = n_bytes + len(d)
        if first is None:
            first = d

print('RX bytes=%d reads=%d' % (n_bytes, n_reads))
if first:
    s = ''
    k = 0
    for b in first:
        if k >= 24:
            break
        s = s + '%02X ' % b
        k = k + 1
    print('RX first: %s' % s)
print('RX done')
