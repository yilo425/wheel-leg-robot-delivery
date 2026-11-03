#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_uart_model.py -- 用 Python 周期精确地模拟 uart_tx / uart_rx，验证回环

为什么写这个：
   在 ModelSim 里跑一次要用户切窗口、等编译、看文件，成本很高。
   而这套 UART 逻辑是纯时序状态机，完全可以用 Python 逐周期复现。
   改完 RTL 先在这里验证逻辑对了，再让用户去跑 ModelSim，
   避免"改一次 RTL -> 让用户跑一次 -> 又错"的来回。

本脚本严格镜像以下两个文件的时序行为：
   rtl/driver/uart_tx.v
   rtl/driver/uart_rx.v
改了 RTL 后，这个脚本也要同步改（两边行为必须一致）。

用法：
   python verify_uart_model.py
   python verify_uart_model.py --clk 50000000
   python verify_uart_model.py --clk 20000000     # 演示过采样精度不足
"""

import argparse
import sys


#=============================================================================
# uart_tx 模型
#=============================================================================
class UartTx:
    def __init__(self, clk_hz, baud, data_bits=8, parity_en=0):
        self.CLK_PER_BIT = clk_hz // baud
        self.DATA_BITS = data_bits
        self.PARITY_EN = parity_en
        self.reset()

    def reset(self):
        self.state = 'IDLE'
        self.div_cnt = 0
        self.bit_idx = 0
        self.shift = (1 << (self.DATA_BITS + 1)) - 1
        self.tx = 1
        self.busy = 0
        self.done = 0

    def _frame(self, data_in):
        if self.PARITY_EN:
            p = bin(data_in).count('1') & 1
            par = p if self.PARITY_EN == 1 else (p ^ 1)   # 1=偶 2=奇
            return (par << self.DATA_BITS) | data_in
        return (1 << self.DATA_BITS) | data_in

    def cycle(self, start, data_in):
        """推进一个时钟周期（镜像 uart_tx.v 的 always 块）"""
        s = self.state
        div = self.div_cnt
        bit_end = (div == self.CLK_PER_BIT - 1)

        # ---- 组合: bit_end ----
        # ---- 分频计数 ----
        if s == 'IDLE':
            new_div = 0
        elif bit_end:
            new_div = 0
        else:
            new_div = div + 1

        # ---- 状态机（全部用"旧值"计算右式，模拟非阻塞赋值）----
        new_state, new_tx, new_busy, new_done = s, self.tx, self.busy, 0
        new_shift, new_bit_idx = self.shift, self.bit_idx

        if s == 'IDLE':
            new_tx = 1
            new_busy = 0
            if start:
                new_shift = self._frame(data_in)
                new_bit_idx = 0
                new_tx = 0                 # 起始位
                new_busy = 1
                new_state = 'START'
        elif s == 'START':
            if bit_end:
                new_tx = self.shift & 1
                new_shift = (1 << self.DATA_BITS) | (self.shift >> 1)
                new_bit_idx = 0
                new_state = 'DATA'
        elif s == 'DATA':
            if bit_end:
                new_tx = self.shift & 1
                new_shift = (1 << self.DATA_BITS) | (self.shift >> 1)
                if self.bit_idx >= (self.DATA_BITS + self.PARITY_EN - 1):
                    new_state = 'STOP'
                else:
                    new_bit_idx = self.bit_idx + 1
        elif s == 'STOP':
            if bit_end:
                new_tx = 1
                new_busy = 0
                new_done = 1
                new_state = 'IDLE'

        self.state, self.tx, self.busy, self.done = new_state, new_tx, new_busy, new_done
        self.div_cnt, self.shift, self.bit_idx = new_div, new_shift, new_bit_idx


#=============================================================================
# uart_rx 模型
#=============================================================================
class UartRx:
    def __init__(self, clk_hz, baud, data_bits=8, parity_en=0):
        self.OS_RATE = 16
        self.HALF = self.OS_RATE // 2
        self.OS_DIV = clk_hz // (baud * self.OS_RATE)
        self.DATA_BITS = data_bits
        self.PARITY_EN = parity_en
        self.reset()

    def reset(self):
        self.state = 'IDLE'
        self.div_cnt = 0
        self.tick_cnt = 0
        self.data_idx = 0
        self.frame_data = 0
        self.data_out = 0
        self.done = 0
        self.frame_err = 0
        self.parity_err = 0
        self.rx_busy = 0
        # 同步器
        self.s1 = 1
        self.s2 = 1

    def cycle(self, rx):
        """推进一个时钟周期（镜像 uart_rx.v）"""
        # ---- 输入同步（非阻塞，用旧值）----
        new_s1 = rx
        new_s2 = self.s1

        os_tick = (self.div_cnt == 0)
        mid_bit = (self.tick_cnt == self.HALF - 1)

        # ---- 分频 / tick_cnt ----
        if os_tick:
            new_div = self.OS_DIV - 1
            new_tick_cnt = 0 if self.tick_cnt >= self.OS_RATE - 1 else self.tick_cnt + 1
        else:
            new_div = self.div_cnt - 1
            new_tick_cnt = self.tick_cnt

        s = self.state
        new_state = s
        new_done = 0
        new_frame_err = self.frame_err
        new_parity_err = self.parity_err
        new_rx_busy = self.rx_busy
        new_data_idx = self.data_idx
        new_frame_data = self.frame_data
        new_data_out = self.data_out

        if s == 'IDLE':
            new_rx_busy = 0
            new_tick_cnt = 0
            # 下降沿判据：2 拍前高、1 拍前低
            # （曾经写成 s2==0 and s1==1，那是上升沿，导致永远收不到数据）
            if self.s2 == 1 and self.s1 == 0:
                new_state = 'RECHK'
                new_rx_busy = 1
                new_data_idx = 0
                new_frame_data = 0
                new_parity_err = 0
                new_frame_err = 0
        elif s == 'RECHK':
            if os_tick and mid_bit:
                if self.s2 == 1:
                    new_state = 'IDLE'
                    new_rx_busy = 0
                else:
                    new_state = 'DATA'
                    new_data_idx = 0
        elif s == 'DATA':
            if os_tick and mid_bit:
                bit = self.data_idx
                new_frame_data = (self.frame_data & ~(1 << bit)) | (self.s2 << bit)
                if self.data_idx >= self.DATA_BITS - 1:
                    new_state = 'PAR' if self.PARITY_EN else 'STOP'
                else:
                    new_data_idx = self.data_idx + 1
        elif s == 'PAR':
            if os_tick and mid_bit:
                p = bin(self.frame_data).count('1') & 1
                exp = p if self.PARITY_EN == 1 else (p ^ 1)
                new_parity_err = 1 if self.s2 != exp else 0
                new_state = 'STOP'
        elif s == 'STOP':
            if os_tick and mid_bit:
                if self.s2 == 1:
                    new_data_out = self.frame_data & ((1 << self.DATA_BITS) - 1)
                    new_done = 1
                else:
                    new_frame_err = 1
                new_state = 'IDLE'
                new_rx_busy = 0

        self.s1, self.s2 = new_s1, new_s2
        self.state, self.div_cnt, self.tick_cnt = new_state, new_div, new_tick_cnt
        self.data_idx, self.frame_data = new_data_idx, new_frame_data
        self.done, self.frame_err, self.parity_err, self.rx_busy = \
            new_done, new_frame_err, new_parity_err, new_rx_busy
        self.data_out = new_data_out


#=============================================================================
# 回环测试
#=============================================================================
def loopback_send(clk_hz, baud, byte, verbose=False, max_cycles=None):
    """单字节回环（每次都用全新的 tx/rx）"""
    tx = UartTx(clk_hz, baud)
    rx = UartRx(clk_hz, baud)
    bit_clks = clk_hz // baud
    byte_clks = bit_clks * 10
    if max_cycles is None:
        max_cycles = byte_clks * 5

    for cyc in range(max_cycles):
        start = (cyc == 2)                    # 第 2 拍给一个 start 脉冲
        tx.cycle(start, byte)
        rx.cycle(tx.tx)
        if rx.done:
            return True, rx.data_out, cyc
    return False, rx.data_out, max_cycles


def loopback_stream(clk_hz, baud, bytes_in):
    """
    背靠背连续发送多个字节，验证**流控**是否正确。

    为什么要单独测这个：
      uart_tx 只在 IDLE 状态响应 start，忙时(start 到来)会**静默忽略**。
      而接收器在"停止位中点"就报 done，发送器那时还在发停止位 ——
      中间有半个比特时间的窗口。
      如果发送方拿到 done 就立刻发下一个（不做流控），
      那个 start 会被丢掉，表现为"字节严格交替地成功/失败"。

      正确做法：发送前先等 tx.busy == 0。

    本函数模拟"正确流控"：一旦 tx 空闲就立刻发下一个字节。
    如果全部字节都能正确回环，说明流控方案可行、RTL 时序也正确。
    """
    tx = UartTx(clk_hz, baud)
    rx = UartRx(clk_hz, baud)
    recv = []
    idx = 0
    last_start = -100
    bit_clks = clk_hz // baud
    byte_clks = bit_clks * 10

    for cyc in range(len(bytes_in) * byte_clks * 20):
        start = False
        if idx < len(bytes_in) and (not tx.busy) and (cyc - last_start > 5):
            start = True

        tx.cycle(start, bytes_in[idx] if idx < len(bytes_in) else 0)
        rx.cycle(tx.tx)

        if start:
            last_start = cyc
            idx += 1
        if rx.done:
            recv.append(rx.data_out)

        if idx >= len(bytes_in) and len(recv) >= len(bytes_in):
            break

    return recv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clk', type=int, default=100_000_000)
    ap.add_argument('--baud', type=int, default=115_200)
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    clk, baud = args.clk, args.baud
    bit_clks = clk // baud
    os_div = clk // (baud * 16)
    ideal_osdiv = clk / (baud * 16)

    print(f"=== UART 回环模型验证 ===")
    print(f"  CLK_FREQ_HZ = {clk}")
    print(f"  BAUD_RATE   = {baud}")
    print(f"  CLK_PER_BIT = {bit_clks}")
    print(f"  OS_DIV      = {os_div}  (理想 {ideal_osdiv:.2f}, "
          f"截断误差 {100*(1-os_div/ideal_osdiv):.2f}%)")
    print()

    fails = 0
    print("--- 单字节回环（每次全新 tx/rx）---")
    for b in (0x55, 0xA3, 0x00, 0xFF, 0x01, 0x80, 0x7E):
        ok, got, cyc = loopback_send(clk, baud, b, args.verbose)
        flag = 'OK  ' if (ok and got == b) else 'FAIL'
        if not (ok and got == b):
            fails += 1
        print(f"  [{flag}] 0x{b:02X} -> 0x{got:02X}  ({cyc} cycles)")

    #------------------------------------------------------------------
    # 背靠背流控测试
    #   这一项专门验证"发送前必须查 tx_busy"。
    #   不做流控的话会出现字节交替丢失（ModelSim 实测就是这样）。
    #------------------------------------------------------------------
    print()
    print("--- 背靠背连续发送（验证流控）---")
    seq = [0x55, 0xA3, 0x00, 0xFF, 0x01, 0x80, 0x7E]
    recv = loopback_stream(clk, baud, seq)
    got_str = ' '.join(f'{x:02X}' for x in recv)
    exp_str = ' '.join(f'{x:02X}' for x in seq)
    print(f"  发送: {exp_str}")
    print(f"  接收: {got_str}")
    if recv == seq:
        print("  [OK  ] 全部正确")
    else:
        fails += 1
        print("  [FAIL] 不匹配")

    print()
    if fails == 0:
        print("RESULT: PASS  (模型层面回环全部正确)")
        print()
        print("说明：这只证明*逻辑*是对的。真正的时序/综合问题仍需 ModelSim + 上板验证。")
        return 0
    print(f"RESULT: FAIL  ({fails} 个字节回环失败)")
    return 1


if __name__ == '__main__':
    sys.exit(main())
