#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_motion_decision_model.py -- motion_decision 的 Python 模型

为什么写这个：
   这个模块本身是**纯组合算术**，看起来简单，但"符号/位宽"是最容易出错的地方：
     * steer 为正到底是左转还是右转？（系数符号）
     * 负的 duty 怎么拆成"幅值 + 方向脚"？
     * 右移是不是算术右移（负数要保留符号）？
     * 中间量位宽够不够？
   用 Python 复现一遍，秒级就能把这些验掉，不用去跑 ModelSim。

本脚本严格镜像 rtl/app/motion_decision.v。改了 RTL 要同步改这里。

用法：
   python verify_motion_decision_model.py
"""

import sys


#=============================================================================
# 辅助
#=============================================================================
def s32(v):
    """按 32 位有符号回绕（Verilog 的 integer 语义）"""
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


SPD_STOP, SPD_SLOW, SPD_FULL = 0, 1, 2
SPD_NAME = {SPD_STOP: 'STOP', SPD_SLOW: 'SLOW', SPD_FULL: 'FULL'}


#=============================================================================
# 模型
#=============================================================================
class MotionDecision:
    """默认参数与 RTL 一致"""

    def __init__(self, VREF=60, KP_SHIFT=1, LIMIT_DUTY=120, ZERO_DEADBAND=1,
                 STEER_COEF=(1, -1, 1, -1), STEER_SHIFT=4):
        self.VREF = VREF
        self.KP_SHIFT = KP_SHIFT
        self.LIMIT = LIMIT_DUTY
        self.DB = ZERO_DEADBAND
        self.COEF = list(STEER_COEF)
        self.STEER_SHIFT = STEER_SHIFT

    #-------------------------------------------------------------------------
    def evaluate(self, motor_enable=1, speed_scale=SPD_FULL, steer=0,
                 spd_fb=(0, 0, 0, 0)):
        # ---- 基础速度 ----
        if speed_scale == SPD_SLOW:
            base = self.VREF >> 1
        elif speed_scale == SPD_FULL:
            base = self.VREF
        else:
            base = 0

        # ---- 转向偏置（算术右移，负数保留符号）----
        bias = s32(steer) >> self.STEER_SHIFT

        # ---- 每轮目标 ----
        tgt = [s32(base + bias * c) for c in self.COEF]

        # ---- 误差 + 比例 ----
        err = [s32(tgt[g] - s32(spd_fb[g])) for g in range(4)]
        prop = [s32(e >> self.KP_SHIFT) for e in err]

        # ---- 限幅 ----
        lim = [(p > self.LIMIT or p < -self.LIMIT) for p in prop]
        sat = [max(-self.LIMIT, min(self.LIMIT, p)) for p in prop]

        # ---- 输出 ----
        en = bool(motor_enable) and (speed_scale != SPD_STOP)
        if not en:
            duty = [0, 0, 0, 0]
            limited = False
        else:
            duty = [self._abs_duty(v) for v in sat]
            limited = any(lim)

        dirs = [1 if v >= 0 else 0 for v in sat]

        return {
            'base': base,
            'bias': bias,
            'target': [s16(t) for t in tgt],
            'err': err,
            'prop': prop,
            'sat': sat,
            'duty': duty,
            'dir': dirs,
            'limited': 1 if limited else 0,
        }

    def _abs_duty(self, v):
        a = -v if v < 0 else v
        if a < self.DB:
            return 0
        if a > self.LIMIT:
            return self.LIMIT
        return a


#=============================================================================
# 检查器
#=============================================================================
class Checker:
    def __init__(self):
        self.checks = 0
        self.errors = 0

    def check(self, name, got, exp):
        self.checks += 1
        if got != exp:
            self.errors += 1
            print("  [FAIL] %-50s got=%s exp=%s" % (name, got, exp))
        else:
            print("  [ OK ] %-50s %s" % (name, got))


def run():
    ck = Checker()
    print("=== motion_decision 模型验证 ===")

    # 注意：系数用 RTL 里的值。轮子编号：0=左前 1=右前 2=左后 3=右后
      #   左轮 = 索引 0,2 (+1)；右轮 = 索引 1,3 (-1)
    d = MotionDecision(STEER_COEF=(1, -1, 1, -1))

    #-------------------------------------------------------------------------
    print("\n--- 1. speed_scale -> base 映射 ---")
    r = d.evaluate(speed_scale=SPD_STOP)
    ck.check("STOP: base=0", r['base'], 0)
    ck.check("STOP: duty 全为 0", max(r['duty']), 0)

    r = d.evaluate(speed_scale=SPD_SLOW)
    ck.check("SLOW: base=30 (VREF/2)", r['base'], 30)

    r = d.evaluate(speed_scale=SPD_FULL)
    ck.check("FULL: base=60 (VREF)", r['base'], 60)

    #-------------------------------------------------------------------------
    print("\n--- 2. 比例控制 + 死区 ---")
    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(0, 0, 0, 0))
    # err=60, prop=30
    ck.check("静止: prop=30", r['prop'][0], 30)
    ck.check("静止: duty=30", r['duty'][0], 30)
    ck.check("静止: dir=1 (正转)", r['dir'][0], 1)
    ck.check("静止: 未限幅", r['limited'], 0)

    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(60, 60, 60, 60))
    ck.check("达到目标: prop=0", r['prop'][0], 0)
    ck.check("达到目标: duty=0 (死区)", r['duty'][0], 0)

    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(61, 61, 61, 61))
    # err=-1, prop=-1 >>> 1 = -1（算术右移，-1>>1 仍是 -1）
    ck.check("略微超速: prop=-1 (算术右移保号)", r['prop'][0], -1)
    ck.check("略微超速: duty=1", r['duty'][0], 1)
    ck.check("略微超速: dir=0 (反转)", r['dir'][0], 0)

    #-------------------------------------------------------------------------
    print("\n--- 3. 限幅（每轮独立）---")
    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(-400, -400, -400, -400))
    # err=460, prop=230 -> 超过 120
    ck.check("深超速: duty 被限到 120", r['duty'][0], 120)
    ck.check("深超速: limited=1", r['limited'], 1)
    ck.check("深超速: dir=1", r['dir'][0], 1)

    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(-400, 0, 0, 0))
    ck.check("只有 0 路饱和: duty0=120", r['duty'][0], 120)
    ck.check("只有 0 路饱和: duty1=30 (不受影响)", r['duty'][1], 30)
    ck.check("只有 0 路饱和: limited=1", r['limited'], 1)

    #-------------------------------------------------------------------------
    print("\n--- 4. 关使能 ---")
    r = d.evaluate(motor_enable=0, speed_scale=SPD_FULL,
                   spd_fb=(-400, -400, -400, -400))
    ck.check("关使能: duty 全为 0", max(r['duty']), 0)
    ck.check("关使能: limited=0", r['limited'], 0)
    ck.check("关使能: 方向仍给出", r['dir'][0], 1)

    #-------------------------------------------------------------------------
    print("\n--- 5. ★转向符号（最容易搞反的地方）---")
    # 约定：steer > 0 = 右转 = 左轮加速、右轮减速
    steer = 160          # 160 >>> 4 = 10
    r = d.evaluate(speed_scale=SPD_FULL, steer=steer, spd_fb=(0, 0, 0, 0))
    ck.check("steer=160: bias=10", r['bias'], 10)
    ck.check("★右转: 左轮目标 = 60+10 = 70", r['target'][0], 70)
    ck.check("★右转: 右轮目标 = 60-10 = 50", r['target'][1], 50)
    ck.check("★右转: 左轮比右轮快（才是右转）",
             r['target'][0] > r['target'][1], True)
    # 左轮 target 70 -> err 70 -> prop 35；右轮 50 -> err 50 -> prop 25
    ck.check("右转: 左轮 duty=35", r['duty'][0], 35)
    ck.check("右转: 右轮 duty=25", r['duty'][1], 25)

    steer = -160         # 负 = 左转
    r = d.evaluate(speed_scale=SPD_FULL, steer=steer, spd_fb=(0, 0, 0, 0))
    ck.check("steer=-160: bias=-10", r['bias'], -10)
    ck.check("★左转: 左轮目标 = 50", r['target'][0], 50)
    ck.check("★左转: 右轮目标 = 70", r['target'][1], 70)
    ck.check("★左转: 右轮比左轮快（才是左转）",
             r['target'][1] > r['target'][0], True)

    r = d.evaluate(speed_scale=SPD_FULL, steer=0, spd_fb=(0, 0, 0, 0))
    ck.check("steer=0: 四轮目标相同（走直线）",
             len(set(r['target'])), 1)

    #-------------------------------------------------------------------------
    print("\n--- 6. 转向系数之和为 0（转不附带加减速）---")
    rs = d.evaluate(speed_scale=SPD_FULL, steer=160, spd_fb=(0, 0, 0, 0))
    total_target = sum(rs['target'])
    ck.check("转向后四轮目标之和 = 4*base", total_target, 4 * 60)

    #-------------------------------------------------------------------------
    print("\n--- 7. 低速档 + 转向 ---")
    r = d.evaluate(speed_scale=SPD_SLOW, steer=160, spd_fb=(0, 0, 0, 0))
    ck.check("SLOW+右转: 左轮目标 = 30+10 = 40", r['target'][0], 40)
    ck.check("SLOW+右转: 右轮目标 = 30-10 = 20", r['target'][1], 20)

    #-------------------------------------------------------------------------
    print("\n--- 8. 反馈为负（倒转）---")
    r = d.evaluate(speed_scale=SPD_FULL, spd_fb=(-20, -20, -20, -20))
    # err = 60-(-20) = 80, prop = 40
    ck.check("反馈为负: prop=40 (加速纠正)", r['prop'][0], 40)
    ck.check("反馈为负: dir=1", r['dir'][0], 1)

    r = d.evaluate(speed_scale=SPD_STOP, spd_fb=(50, 50, 50, 50))
    ck.check("STOP 档 + 还在滑行: duty=0 (不主动刹车)", max(r['duty']), 0)

    #-------------------------------------------------------------------------
    print()
    if ck.errors == 0:
        print("RESULT: PASS  (%d 项检查全部通过)" % ck.checks)
        print()
        print("说明：这只验证**算术与符号约定**。")
        print("      与 encoder_decode / pwm_motor 的接口时序仍需 ModelSim 验证。")
        return 0
    print("RESULT: FAIL  (%d 项检查, %d 个错误)" % (ck.checks, ck.errors))
    return 1


if __name__ == '__main__':
    sys.exit(run())
