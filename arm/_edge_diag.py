from machine import FPIOA, Pin
import time

fp = FPIOA()
fp.set_function(6, FPIOA.GPIO6)
p = Pin(6, Pin.IN)
time.sleep_ms(100)

n_edges = 0
n_high = 0
n_low = 0
prev = p.value()
t0 = time.ticks_ms()
t_end = t0 + 2000
while time.ticks_ms() < t_end:
    v = p.value()
    if v != prev:
        n_edges = n_edges + 1
        prev = v
    if v:
        n_high = n_high + 1
    else:
        n_low = n_low + 1
dt = time.ticks_diff(time.ticks_ms(), t0)

print('EDGE dt=%dms edges=%d high=%d low=%d' % (dt, n_edges, n_high, n_low))
if n_edges == 0:
    print('EDGE verdict=IDLE_NO_DATA')
elif n_low < 10:
    print('EDGE verdict=WEAK')
else:
    print('EDGE verdict=UART_ACTIVITY')

fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
print('EDGE done')
