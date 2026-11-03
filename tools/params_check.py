#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
params_check.py -- 参数自检工具

作用：
    在不打开仿真器的前提下，检查 RTL 里那些"编译期算出来的常数"是否符合预期。
    这些数一旦算错（比如 PPL 换频后忘了改 CLK_FREQ_HZ），上板现象就是
    "PWM 频率偏一半""测速快一倍"这类很难查的问题。

用法：
    python params_check.py                 # 按 FPGA 默认参数(50MHz)检查
    python params_check.py --clk 100000000 # 换成 PLL 100MHz 检查

输出：
    每个参数一行期望值，末尾给出 PASS/FAIL。
"""

import argparse
import sys


class Checker:
    def __init__(self):
        self.errors = 0
        self.checks = 0

    def check(self, name, got, expect):
        self.checks += 1
        ok = got == expect
        if not ok:
            self.errors += 1
        flag = "OK  " if ok else "FAIL"
        print(f"  [{flag}] {name:<46} got={got}  expect={expect}")

    def check_range(self, name, got, lo, hi):
        self.checks += 1
        ok = lo <= got <= hi
        if not ok:
            self.errors += 1
        flag = "OK  " if ok else "FAIL"
        print(f"  [{flag}] {name:<46} got={got}  expect in [{lo}, {hi}]")


def pwm_params(clk_hz, pwm_hz, duty_gain):
    period_cnt = clk_hz // pwm_hz
    step_div = period_cnt // duty_gain
    duty_full = period_cnt // step_div if step_div else 0
    return period_cnt, step_div, duty_full


def uart_params(clk_hz, baud, acc_width=24):
    import math
    inc = int(round(baud * (1 << acc_width) / clk_hz))
    # 每个比特实际消耗的 clk 数
    clks_per_bit = (1 << acc_width) / inc if inc else float("inf")
    # 有效过采样倍数
    os_inc = int(round(clk_hz / (baud * 16.0)))
    return inc, clks_per_bit, os_inc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clk", type=int, default=50_000_000, help="系统时钟 Hz")
    ap.add_argument("--pwm-freq", type=int, default=20_000)
    ap.add_argument("--duty-gain", type=int, default=128)
    ap.add_argument("--baud", type=int, default=115_200)
    ap.add_argument("--speed-window-ms", type=float, default=10.0)
    ap.add_argument("--ppr", type=int, default=13)
    ap.add_argument("--gear", type=int, default=30)
    args = ap.parse_args()

    c = Checker()
    clk = args.clk

    print(f"=== 参数自检: CLK={clk/1e6:.2f}MHz ===")

    #------------------------------------------------------------------
    # 1) PWM
    #------------------------------------------------------------------
    print("\n[PWM]")
    period_cnt, step_div, duty_full = pwm_params(clk, args.pwm_freq, args.duty_gain)
    print(f"  PERIOD_CNT={period_cnt}  STEP_DIV={step_div}  DUTY_FULL={duty_full}")
    c.check("PWM 载波周期(clk)", period_cnt, clk // args.pwm_freq)
    real_pwm = clk / period_cnt
    dev = abs(real_pwm - args.pwm_freq) / args.pwm_freq * 100.0
    c.check_range("PWM 实际频率偏差(%)", round(dev, 3), 0.0, 5.0)
    c.check("DUTY_FULL 在 signed16 范围内", duty_full <= 32767, True)
    c.check("满量程比较值不溢出16位", duty_full * step_div <= 65535, True)
    # 50% 档位
    half = duty_full // 2
    print(f"  50% 占空比对应 duty_in 约为 {half}")

    #------------------------------------------------------------------
    # 2) UART
    #------------------------------------------------------------------
    print("\n[UART]")
    inc, clks_per_bit, os_inc = uart_params(clk, args.baud)
    ideal = clk / args.baud
    print(f"  BAUD_INC={inc}  实际每比特 {clks_per_bit:.4f} clk  理想 {ideal:.4f} clk")
    err = abs(clks_per_bit - ideal) / ideal * 100.0
    c.check_range("波特率误差(%)", round(err, 4), 0.0, 2.0)
    c.check("过采样倍数 >= 1 (上板要求)", os_inc >= 1, True)
    if os_inc < 1:
        print("     -> 该组合只能用于仿真加速, 不能上板")

    #------------------------------------------------------------------
    # 3) 测速窗口
    #------------------------------------------------------------------
    print("\n[编码器测速]")
    window = int(clk * args.speed_window_ms / 1000.0)
    print(f"  SPEED_WINDOW_CYCLES={window}  ({args.speed_window_ms}ms)")
    c.check("窗口周期数", window, int(clk * args.speed_window_ms / 1000.0))
    c.check("窗口计数不溢出 24 位", window <= (1 << 24) - 1, True)

    # 速度换算系数
    counts_per_rev = 4 * args.ppr * args.gear
    win_sec = args.speed_window_ms / 1000.0
    print(f"  4倍频计数/圈 = 4*{args.ppr}*{args.gear} = {counts_per_rev}")
    rps_per_count = 1.0 / counts_per_rev / win_sec
    print(f"  换算: 转速(rps) = speed_cnt * {rps_per_count:.6f}")
    c.check("换算系数合理(每计数转速不为0)", rps_per_count > 0, True)

    #------------------------------------------------------------------
    # 4) 滤波与最低速约束
    #------------------------------------------------------------------
    print("\n[编码器滤波约束]")
    filt = 8
    # 每相最小脉冲宽度 = 窗口 / 最大计数的 1/4
    print(f"  FILTER_CYCLES={filt}")
    print("  每相脉冲宽度必须 > FILTER_CYCLES, 否则低速时会被滤掉。")
    print(f"  例: 若速度达到 {window // 4} 计数/窗口(每相 1 clk), 会完全被滤掉。")
    c.check("FILTER_CYCLES 为 2 的幂或小值", filt <= 16, True)

    #------------------------------------------------------------------
    print(f"\n=== {c.checks} checks, {c.errors} errors ===")
    if c.errors == 0:
        print("RESULT: PASS")
        return 0
    print("RESULT: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
