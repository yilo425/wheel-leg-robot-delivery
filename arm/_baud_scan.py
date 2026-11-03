from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)

BAUDS = (115200, 57600, 38400, 19200, 9600, 230400)

for baud in BAUDS:
    # deinit() 会清掉 FPIOA 映射，所以每次建 UART 前都要重设
    try:
        fp.set_function(5, FPIOA.UART2_TXD)
        fp.set_function(6, FPIOA.UART2_RXD)
    except Exception as e:
        print('BAUD %d fpioa fail: %s' % (baud, e))
    time.sleep_ms(30)
    try:
        uart = UART(UART.UART2, baudrate=baud, bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    except Exception as e:
        print('BAUD %d open fail: %s' % (baud, e))
        continue
    time.sleep_ms(80)
    buf = b''
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 1200:
        n = 0
        try:
            n = uart.any()
        except Exception:
            n = 0
        if n:
            try:
                d = uart.read(n)
            except Exception:
                d = None
            if d:
                buf = buf + d
    hdr = buf.count(b'\xaa\x55')
    tail = buf.count(b'\x55\xaa')
    print('BAUD %6d bytes=%5d hdr_AA55=%d tail_55AA=%d' % (baud, len(buf), hdr, tail))
    try:
        uart.deinit()
    except Exception:
        pass
    time.sleep_ms(50)

print('BAUDSCAN done')
