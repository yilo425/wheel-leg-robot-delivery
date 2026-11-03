from machine import FPIOA, Pin
import time

fp = FPIOA()

# GPIO6 = 排针 13 = ARM 的 RXD，应该接 FPGA C19（FPGA 的 TX）
# UART 空闲时 TX 线是**高电平**。所以：
#   读到 1  -> FPGA 在驱动这根线（空闲高），线是通的
#   读到 0  -> 被拉住或没接（FPGA 没驱动它）
#   跟随上拉/下拉变化 -> **悬空**，线没接到 FPGA
print('--- GPIO6 (排针13, ARM RXD, 应接 FPGA C19) ---')
for name, pull in (('无上拉', None), ('下拉', Pin.PULL_DOWN), ('上拉', Pin.PULL_UP)):
    fp.set_function(6, FPIOA.GPIO6)
    if pull is None:
        p = Pin(6, Pin.IN)
    else:
        p = Pin(6, Pin.IN, pull=pull)
    time.sleep_ms(50)
    v = p.value()
    print('  %s: GPIO6 = %d' % (name, v))
    time.sleep_ms(50)

print('--- GPIO5 (排针11, ARM TXD, 应接 FPGA F16) ---')
# 这根是 ARM 输出，改成输入读它能看到 FPGA 输入脚的高阻状态
for name, pull in (('无上拉', None), ('下拉', Pin.PULL_DOWN), ('上拉', Pin.PULL_UP)):
    fp.set_function(5, FPIOA.GPIO5)
    if pull is None:
        p = Pin(5, Pin.IN)
    else:
        p = Pin(5, Pin.IN, pull=pull)
    time.sleep_ms(50)
    v = p.value()
    print('  %s: GPIO5 = %d' % (name, v))
    time.sleep_ms(50)

# 恢复 UART2 功能
fp.set_function(5, FPIOA.UART2_TXD)
fp.set_function(6, FPIOA.UART2_RXD)
print('已恢复 UART2 功能')
print('DIAG DONE')
