from machine import UART, FPIOA
import time

fp = FPIOA()
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
time.sleep_ms(300)
print('PROBE start')

for i in range(12):
    try:
        a = uart.any()
    except Exception as e:
        print('PROBE %d any raised: %s' % (i, e))
        a = -1
    line = 'PROBE i=%d any=%d' % (i, a)
    if a > 0:
        try:
            d = uart.read(a)
        except Exception as e:
            d = None
            line = line + ' read_raised'
        if d:
            line = line + ' len=%d' % len(d)
            s = ''
            k = 0
            for b in d:
                if k >= 12:
                    break
                s = s + '%02X ' % b
                k = k + 1
            line = line + ' hex=' + s
        else:
            line = line + ' read_none'
    print(line)
    time.sleep_ms(100)

# 再试一次不带参数的 read()
try:
    d = uart.read()
    print('PROBE plain_read len=%d' % (len(d) if d else -1))
except Exception as e:
    print('PROBE plain_read raised: %s' % e)

print('PROBE done')
