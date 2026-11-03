#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_main_fsm_model.py -- main_fsm 的周期精确 Python 模型

为什么写这个：
   main_fsm 是纯状态机，行为完全可以用 Python 逐拍复现。
   改完 RTL 先在这里跑一遍（秒级），确认状态转移和输出都对，
   再去跑 ModelSim（分钟级）。这样能避免"改一次、跑一次"的来回。

本脚本严格镜像 rtl/app/main_fsm.v 的行为。
**改了 RTL 必须同步改这里**，两边行为要一致。

-----------------------------------------------------------------------------
本模型已经帮忙抓到的两个问题（都值得记住）
-----------------------------------------------------------------------------
1) 输出语义：输出是**组合逻辑**，时钟沿之后就该反映**新状态**。
   最初写成"先算输出、再更新状态"，结果状态对、输出差一拍。

2) RTL 真实缺陷：error_flag 原写成
       else if ((nstate == S_ESTOP) && !estop) error_flag <= 1'b1;
   人为急停松开 estop 后，状态还停在 ESTOP 而 estop 已变低，
   于是"人为急停"被误报成"异常停车"。
   修法：只在**跳变沿**判定 —— 加 (state != S_ESTOP) 限定。

-----------------------------------------------------------------------------
另一个真实的硬件时序特性（测试必须考虑）
-----------------------------------------------------------------------------
TOF 读数是**先锁存、下一拍才参与状态判定**的。
所以送一次 dist_valid 之后，状态不会立刻变化，要再等一拍。
这不是缺陷，是必要的（否则组合路径直穿传感器输入，会有毛刺）。

用法：
   python verify_main_fsm_model.py
   python verify_main_fsm_model.py -v
"""

import argparse
import sys

#=============================================================================
# 参数（与 RTL 默认值一致；时钟/超时缩小以便快速仿真）
#=============================================================================
CLK_FREQ_HZ      = 100_000      # 缩小版：MS_TICKS = 100
DIST_STOP_MM     = 200
DIST_SLOW_MM     = 500
DIST_CLEAR_MM    = 800
AVOID_TIMEOUT_MS = 50           # 缩小版（RTL 默认 5000）
LIFT_TIMEOUT_MS  = 50
CROSS_TIMEOUT_MS = 30
VIS_LOW_OBST     = 0x04
VIS_OBST         = 0x02      # 障碍物 -> 避障（与 RTL 的 VIS_OBST 对应）

# 视觉障碍物的"保持窗口"
#   ARM 是 10Hz 发的，vision_valid 是单拍脉冲。没有保持的话 AVOID 只持续
#   1 拍就退出，等于没避障。RTL 里也是这个逻辑（main_fsm.v 的 VIS_HOLD_MS）。
#   这里取 3ms（RTL 默认 300ms），够测"保持中"和"保持过期"两个分支。
VIS_HOLD_MS      = 3

MS_TICKS        = CLK_FREQ_HZ // 1000
AVOID_TMO_TICKS = AVOID_TIMEOUT_MS * MS_TICKS
VIS_HOLD_TICKS  = VIS_HOLD_MS * MS_TICKS
LIFT_TMO_TICKS  = LIFT_TIMEOUT_MS * MS_TICKS
CROSS_TMO_TICKS = CROSS_TIMEOUT_MS * MS_TICKS

# 状态编码（与 rtl/app/main_fsm.v 和 doc/interface_spec.md 一致）
S_IDLE, S_MANUAL, S_AUTO, S_AVOID, S_LIFT, S_GOAL, S_ESTOP = range(7)
STATE_NAME = {
    S_IDLE: 'IDLE', S_MANUAL: 'MANUAL', S_AUTO: 'AUTO',
    S_AVOID: 'AVOID', S_LIFT: 'LIFT', S_GOAL: 'GOAL', S_ESTOP: 'ESTOP',
}
LP_UP, LP_CROSS, LP_DOWN = 0, 1, 2
PHASE_NAME = {LP_UP: 'UP', LP_CROSS: 'CROSS', LP_DOWN: 'DOWN'}
SPD_STOP, SPD_SLOW, SPD_FULL = 0, 1, 2
SPD_NAME = {SPD_STOP: 'STOP', SPD_SLOW: 'SLOW', SPD_FULL: 'FULL'}


#=============================================================================
# 模型
#=============================================================================
class MainFsm:
    def __init__(self):
        self.reset()

    def reset(self):
        # --- 段1 状态寄存器 ---
        self.state = S_IDLE
        self.lift_phase = LP_UP
        self.t_avoid = self.t_up = self.t_cross = self.t_down = 0
        self.vis_obst_hold = 0      # 视觉障碍物的保持标志（与 RTL 一致）
        self.t_vis = 0              # 保持窗口计时
        # --- TOF 锁存 ---
        self.dist_r = 0
        self.dist_ok = False
        # --- error_flag ---
        self.error_flag = 0
        # --- 输出（组合）---
        self.motor_enable = 0
        self.speed_scale = SPD_STOP
        self.lift_cmd = 0

    #-------------------------------------------------------------------------
    # 段2：次态组合逻辑
    #-------------------------------------------------------------------------
    def _next_state(self, estop, manual_sel, start, goal_reached,
                    lift_done, lift_fault, obst_stop, low_obst, vis_obst, path_clear):
        nstate = self.state
        nphase = self.lift_phase

        if estop:
            return S_ESTOP, nphase          # 急停最高优先级

        s = self.state
        if s == S_IDLE:
            if manual_sel:
                nstate = S_MANUAL
            elif start:
                nstate, nphase = S_AUTO, LP_UP

        elif s == S_MANUAL:
            if not manual_sel:
                nstate = S_IDLE

        elif s == S_AUTO:
            if obst_stop or vis_obst:
                nstate = S_AVOID                    # TOF 太近 或 视觉报 0x02
            elif low_obst:
                nstate, nphase = S_LIFT, LP_UP
            elif goal_reached:
                nstate = S_GOAL

        elif s == S_AVOID:
            if low_obst:
                nstate, nphase = S_LIFT, LP_UP
            elif path_clear and not vis_obst:       # 通畅且视觉已清
                nstate = S_AUTO
            elif self.t_avoid >= AVOID_TMO_TICKS:
                nstate = S_ESTOP

        elif s == S_LIFT:
            if self.lift_phase == LP_UP:
                if lift_done:
                    nphase = LP_CROSS
                elif lift_fault or self.t_up >= LIFT_TMO_TICKS:
                    nstate = S_ESTOP
            elif self.lift_phase == LP_CROSS:
                if path_clear or self.t_cross >= CROSS_TMO_TICKS:
                    nphase = LP_DOWN
            elif self.lift_phase == LP_DOWN:
                if lift_done:
                    nstate = S_AUTO
                elif lift_fault or self.t_down >= LIFT_TMO_TICKS:
                    nstate = S_ESTOP
            else:
                nstate = S_ESTOP

        elif s == S_GOAL:
            if start:
                nstate, nphase = S_AUTO, LP_UP

        elif s == S_ESTOP:
            if start:
                nstate = S_IDLE

        else:
            nstate = S_IDLE

        return nstate, nphase

    #-------------------------------------------------------------------------
    # 段3：输出组合逻辑（Moore）
    #-------------------------------------------------------------------------
    def _outputs(self, state, lift_phase, obst_stop, obst_slow, estop):
        motor_en_r = 0
        speed = SPD_STOP
        lift_cmd = 0

        if state in (S_IDLE, S_GOAL, S_ESTOP):
            motor_en_r, speed, lift_cmd = 0, SPD_STOP, 0
        elif state == S_MANUAL:
            motor_en_r, speed, lift_cmd = 1, SPD_SLOW, 0
        elif state == S_AUTO:
            motor_en_r, speed, lift_cmd = 1, (SPD_SLOW if obst_slow else SPD_FULL), 0
        elif state == S_AVOID:
            motor_en_r = 0 if obst_stop else 1
            speed = SPD_STOP if obst_stop else SPD_SLOW
            lift_cmd = 0
        elif state == S_LIFT:
            if lift_phase == LP_UP:
                motor_en_r, speed, lift_cmd = 0, SPD_STOP, 1
            elif lift_phase == LP_CROSS:
                motor_en_r, speed, lift_cmd = 1, SPD_SLOW, 1
            else:                                  # LP_DOWN / 非法
                motor_en_r, speed, lift_cmd = 0, SPD_STOP, 0
        else:
            motor_en_r, speed, lift_cmd = 0, SPD_STOP, 0

        # 急停直接组合进使能：0 拍延迟
        motor_enable = motor_en_r & (0 if estop else 1)
        return motor_enable, speed, lift_cmd

    #-------------------------------------------------------------------------
    # 推进一拍
    #
    # 顺序必须严格按硬件语义：
    #   1) 用【当前】寄存器值算环境判定和次态   (组合)
    #   2) 更新所有寄存器                       (时钟沿)
    #   3) 用【更新后】的状态和锁存值算输出     (组合)
    #-------------------------------------------------------------------------
    def cycle(self, start=0, estop=0, manual_sel=0,
              dist_mm=0, dist_valid=0, dist_err=0,
              vision_class=0, vision_valid=0,
              goal_reached=0, lift_done=0, lift_fault=0):
        # ---- 1) 环境判定（用当前锁存值）----
        obst_stop  = self.dist_ok and self.dist_r < DIST_STOP_MM
        obst_slow  = (not self.dist_ok) or self.dist_r < DIST_SLOW_MM
        path_clear = self.dist_ok and self.dist_r > DIST_CLEAR_MM
        # low_obst（0x04 -> LIFT）：一拍脉冲就够 —— 进了 LIFT 之后由
        #   lift_done 和超时驱动，不再依赖视觉。
        low_obst = bool(vision_valid) and (vision_class == VIS_LOW_OBST)
        # vis_obst（0x02 -> AVOID）：必须用**保持标志**，不能用当拍脉冲。
        #   否则 vision_valid=0 的那一拍 vis_obst 就变 0，
        #   若 TOF 通畅则 `path_clear and not vis_obst` 成立 -> 立刻退出 AVOID。
        #   与 RTL 的 vis_obst_hold 逐位对齐（含那一拍的寄存器延迟）。
        vis_obst = bool(self.vis_obst_hold)
        vis_obst_pulse = bool(vision_valid) and (vision_class == VIS_OBST)

        # ---- 1) 次态 ----
        nstate, nphase = self._next_state(
            estop, manual_sel, start, goal_reached,
            lift_done, lift_fault, obst_stop, low_obst, vis_obst, path_clear)

        # ---- 2) 更新寄存器（非阻塞语义）----
        cnt_avoid = (self.state == S_AVOID)
        cnt_up    = (self.state == S_LIFT) and (self.lift_phase == LP_UP)
        cnt_cross = (self.state == S_LIFT) and (self.lift_phase == LP_CROSS)
        cnt_down  = (self.state == S_LIFT) and (self.lift_phase == LP_DOWN)

        n_t_avoid = self.t_avoid + 1 if cnt_avoid else 0
        n_t_up    = self.t_up    + 1 if cnt_up    else 0
        n_t_cross = self.t_cross + 1 if cnt_cross else 0
        n_t_down  = self.t_down  + 1 if cnt_down  else 0

        # 保持逻辑（与 RTL 的 always 块逐行对应）
        if vis_obst_pulse:
            n_vis_hold, n_t_vis = 1, 0          # 收到就拉高 + 续期
        elif self.t_vis >= VIS_HOLD_TICKS:
            n_vis_hold, n_t_vis = 0, self.t_vis  # 到期解除（计时停住）
        else:
            n_vis_hold, n_t_vis = self.vis_obst_hold, self.t_vis + 1

        n_dist_r, n_dist_ok = self.dist_r, self.dist_ok
        if dist_valid:
            n_dist_r, n_dist_ok = dist_mm, True
        elif dist_err:
            n_dist_ok = False

        # error_flag：只在【进入 ESTOP 的那一拍】判定，且不是人为急停
        n_err = self.error_flag
        if self.state == S_IDLE:
            n_err = 0
        elif (self.state != S_ESTOP) and (nstate == S_ESTOP) and (not estop):
            n_err = 1

        self.state      = nstate
        self.lift_phase = nphase
        self.t_avoid, self.t_up = n_t_avoid, n_t_up
        self.t_cross, self.t_down = n_t_cross, n_t_down
        self.dist_r, self.dist_ok = n_dist_r, n_dist_ok
        self.vis_obst_hold, self.t_vis = n_vis_hold, n_t_vis
        self.error_flag = n_err

        # ---- 3) 输出（用更新后的状态和锁存值）----
        obst_stop2 = self.dist_ok and self.dist_r < DIST_STOP_MM
        obst_slow2 = (not self.dist_ok) or self.dist_r < DIST_SLOW_MM
        self.motor_enable, self.speed_scale, self.lift_cmd = self._outputs(
            self.state, self.lift_phase, obst_stop2, obst_slow2, estop)

    #-------------------------------------------------------------------------
    @property
    def name(self):
        if self.state == S_LIFT:
            return 'LIFT/%s' % PHASE_NAME.get(self.lift_phase, '?')
        return STATE_NAME.get(self.state, '?')

    @property
    def spd(self):
        return SPD_NAME[self.speed_scale]


#=============================================================================
# 测试辅助
#=============================================================================
def tick(f, n=1, **kw):
    """空推 n 拍"""
    for _ in range(n):
        f.cycle(**kw)


def feed(f, mm, **kw):
    """送一次有效测距读数（锁存）+ 推进一拍。

    注意：这一拍里状态判定用的仍是**上一次**的锁存值，
    读数要再过一拍才生效。所以后面通常需要跟一次 settle()。
    """
    f.cycle(dist_mm=mm, dist_valid=1, **kw)


def settle(f, **kw):
    """再推一拍，让刚锁存的读数真正参与状态判定"""
    f.cycle(**kw)


class Checker:
    def __init__(self):
        self.checks = 0
        self.errors = 0

    def check(self, name, got, exp):
        self.checks += 1
        if got != exp:
            self.errors += 1
            print("  [FAIL] %-46s got=%s exp=%s" % (name, got, exp))
        else:
            print("  [ OK ] %-46s %s" % (name, got))


#=============================================================================
# 场景
#=============================================================================
def run(verbose):
    ck = Checker()
    print("=== main_fsm 模型验证 ===")
    print("  CLK=%d MS_TICKS=%d AVOID_TMO=%d LIFT_TMO=%d CROSS_TMO=%d"
          % (CLK_FREQ_HZ, MS_TICKS, AVOID_TMO_TICKS,
             LIFT_TMO_TICKS, CROSS_TMO_TICKS))

    #-------------------------------------------------------------------------
    print("\n--- 场景 1: 上电 + 启动 ---")
    f = MainFsm()
    tick(f)
    ck.check("上电后: IDLE", f.name, 'IDLE')
    ck.check("上电后: motor_enable=0", f.motor_enable, 0)

    feed(f, 2000, start=1)
    ck.check("start 后: AUTO", f.name, 'AUTO')
    ck.check("AUTO: motor_enable=1", f.motor_enable, 1)
    ck.check("AUTO 距离远: speed=FULL", f.spd, 'FULL')

    #-------------------------------------------------------------------------
    print("\n--- 场景 2: 减速 -> 停车避障 ---")
    feed(f, 600)
    ck.check("距离 600mm: speed=FULL (未低于 SLOW)", f.spd, 'FULL')

    feed(f, 400)
    ck.check("距离 400mm: speed=SLOW", f.spd, 'SLOW')
    ck.check("距离 400mm: 仍为 AUTO", f.name, 'AUTO')

    feed(f, 150)
    settle(f)
    ck.check("距离 150mm: 进入 AVOID", f.name, 'AVOID')
    ck.check("深入障碍: motor_enable=0", f.motor_enable, 0)
    ck.check("深入障碍: speed=STOP", f.spd, 'STOP')

    #-------------------------------------------------------------------------
    print("\n--- 场景 3: 路径通畅后恢复导航 ---")
    feed(f, 900)
    ck.check("AVOID 中距离变远: 允许走(motor=1)", f.motor_enable, 1)
    settle(f)
    ck.check("距离 900mm: 恢复 AUTO", f.name, 'AUTO')
    ck.check("恢复后: speed=FULL", f.spd, 'FULL')

    #-------------------------------------------------------------------------
    print("\n--- 场景 4: 低矮障碍 -> 抬腿越障全流程 ---")
    feed(f, 1000)
    f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST, vision_valid=1)
    ck.check("低矮障碍: LIFT/UP", f.name, 'LIFT/UP')
    ck.check("抬腿中: lift_cmd=1", f.lift_cmd, 1)
    ck.check("抬腿中: motor_enable=0", f.motor_enable, 0)

    f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST, lift_done=1)
    ck.check("lift_done: LIFT/CROSS", f.name, 'LIFT/CROSS')
    ck.check("通过中: lift_cmd=1 (保持抬起)", f.lift_cmd, 1)
    ck.check("通过中: motor_enable=1", f.motor_enable, 1)
    ck.check("通过中: speed=SLOW", f.spd, 'SLOW')

    feed(f, 1200)
    settle(f)
    ck.check("走过去了: LIFT/DOWN", f.name, 'LIFT/DOWN')
    ck.check("收腿中: lift_cmd=0", f.lift_cmd, 0)
    ck.check("收腿中: motor_enable=0", f.motor_enable, 0)

    f.cycle(dist_mm=1200, lift_done=1)
    ck.check("收腿完成: AUTO", f.name, 'AUTO')
    ck.check("回到 AUTO: lift_cmd=0", f.lift_cmd, 0)

    #-------------------------------------------------------------------------
    print("\n--- 场景 5: 急停（0 拍断使能 + 最高优先级 + 不误报异常）---")
    f.cycle(dist_mm=2000, dist_valid=1)
    ck.check("急停前: motor_enable=1", f.motor_enable, 1)

    f.cycle(estop=1)
    ck.check("estop 当拍: motor_enable=0 (组合立即)", f.motor_enable, 0)
    ck.check("estop 当拍: 已进入 ESTOP", f.name, 'ESTOP')
    ck.check("人为急停: error_flag=0", f.error_flag, 0)

    tick(f, 5, dist_mm=2000)
    ck.check("急停解除未按 start: 仍在 ESTOP", f.name, 'ESTOP')
    ck.check("急停解除未按 start: motor_enable=0", f.motor_enable, 0)
    ck.check("★ 急停解除后: error_flag 仍为 0 (不误报)", f.error_flag, 0)

    f.cycle(start=1, dist_mm=2000, dist_valid=1)
    ck.check("按 start: 回到 IDLE", f.name, 'IDLE')
    f.cycle(start=1, dist_mm=2000, dist_valid=1)
    ck.check("再按 start: 进入 AUTO", f.name, 'AUTO')

    #-------------------------------------------------------------------------
    print("\n--- 场景 6: 避障超时 -> 安全停车 ---")
    f = MainFsm()
    feed(f, 2000, start=1)
    feed(f, 100)
    settle(f)
    ck.check("进入 AVOID", f.name, 'AVOID')
    steps = 0
    while f.state == S_AVOID and steps < AVOID_TMO_TICKS + 100:
        f.cycle(dist_mm=100, dist_valid=1)
        steps += 1
    ck.check("避障超时: ESTOP", f.name, 'ESTOP')
    ck.check("避障超时: error_flag=1", f.error_flag, 1)
    ck.check("避障超时: motor_enable=0", f.motor_enable, 0)
    print("         (用了 %d 拍, 阈值 %d)" % (steps, AVOID_TMO_TICKS))

    #-------------------------------------------------------------------------
    print("\n--- 场景 7: 抬腿超时 -> 安全停车 ---")
    f = MainFsm()
    feed(f, 2000, start=1)
    f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST, vision_valid=1)
    ck.check("进入 LIFT/UP", f.name, 'LIFT/UP')
    steps = 0
    while f.state == S_LIFT and f.lift_phase == LP_UP and steps < LIFT_TMO_TICKS + 100:
        f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST)
        steps += 1
    ck.check("抬腿超时: ESTOP", f.name, 'ESTOP')
    ck.check("抬腿超时: error_flag=1", f.error_flag, 1)
    print("         (用了 %d 拍, 阈值 %d)" % (steps, LIFT_TMO_TICKS))

    #-------------------------------------------------------------------------
    print("\n--- 场景 8: 舵机故障 -> 立即安全停车 ---")
    f = MainFsm()
    feed(f, 2000, start=1)
    f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST, vision_valid=1)
    f.cycle(dist_mm=1000, vision_class=VIS_LOW_OBST, lift_fault=1)
    ck.check("lift_fault: ESTOP", f.name, 'ESTOP')
    ck.check("lift_fault: error_flag=1", f.error_flag, 1)

    #-------------------------------------------------------------------------
    print("\n--- 场景 9: 传感器失效 fail-safe（限速而非停车）---")
    f = MainFsm()
    feed(f, 2000, start=1)
    ck.check("正常: speed=FULL", f.spd, 'FULL')
    f.cycle(dist_err=1)
    ck.check("测距失效: 仍为 AUTO (不原地卡死)", f.name, 'AUTO')
    ck.check("测距失效: speed=SLOW (fail-safe)", f.spd, 'SLOW')
    ck.check("测距失效: motor_enable=1", f.motor_enable, 1)
    feed(f, 1500)
    ck.check("恢复有效读数: speed=FULL", f.spd, 'FULL')

    #-------------------------------------------------------------------------
    print("\n--- 场景 10: 到达终点 + 手动模式 ---")
    f = MainFsm()
    feed(f, 2000, start=1)
    f.cycle(dist_mm=2000, goal_reached=1)
    ck.check("goal_reached: GOAL", f.name, 'GOAL')
    ck.check("GOAL: motor_enable=0", f.motor_enable, 0)
    f.cycle(start=1, dist_mm=2000, dist_valid=1)
    ck.check("GOAL 后按 start: AUTO", f.name, 'AUTO')

    f = MainFsm()
    f.cycle(manual_sel=1)
    ck.check("manual_sel: MANUAL", f.name, 'MANUAL')
    ck.check("MANUAL: motor_enable=1", f.motor_enable, 1)
    ck.check("MANUAL: speed=SLOW", f.spd, 'SLOW')
    f.cycle()
    ck.check("松开 manual_sel: IDLE", f.name, 'IDLE')

    #-------------------------------------------------------------------------
    print("\n--- 场景 11: 视觉类别表（0x02 障碍物 / 0x04 低矮障碍）---")
    print("    背景：ARM 侧 camera_vision.py 固定发 0x02（障碍物）。")
    print("    本模块以前只实现了 0x04，导致 ARM 报障碍物时 FPGA 毫无反应。")
    print("    并验证「保持窗口」：vision_valid 是单拍脉冲，没有保持的话")
    print("    AVOID 只持续 1 拍就退出，等于没避障。")

    # ---- 11a: 0x02 障碍物 -> 进 AVOID ----
    f = MainFsm()
    feed(f, 2000, start=1)
    settle(f)
    ck.check("11a.1 启动 -> AUTO", f.name, 'AUTO')

    feed(f, 2000, vision_class=VIS_OBST, vision_valid=1)
    settle(f, vision_class=VIS_OBST, vision_valid=1)
    ck.check("11a.2 收到 0x02 -> AVOID", f.name, 'AVOID')
    # ★ 语义差别：
    #   由 TOF 触发的 AVOID  -> motor_enable=0（停车，确实快撞上了）
    #   由视觉触发的 AVOID  -> motor_enable=1 + SLOW（限速绕行）
    #   能"绕行"的前提是还能动；而且视觉报障碍物时 TOF 往往通畅
    #   （障碍物在侧面、或低于/超出 TOF 量程），停车就永远绕不过去。
    #   真正的"停下"由 TOF 的 obst_stop 负责 —— 那是最后一道防线。
    ck.check("11a.3 视觉触发: motor_enable=1（还能动才能绕）", f.motor_enable, 1)
    ck.check("11a.4 视觉触发: speed=SLOW（限速绕行）", f.spd, 'SLOW')

    # ---- 11b: 保持窗口内仍在 AVOID（防 10Hz 横跳）----
    tick(f, 2)
    ck.check("11b.1 保持窗口内: 仍在 AVOID", f.name, 'AVOID')

    # ---- 11c: 等过保持窗口 -> 自动回 AUTO ----
    tick(f, VIS_HOLD_TICKS + 5)
    ck.check("11c.1 保持窗口过后 -> AUTO", f.name, 'AUTO')

    # ---- 11d: 0x04 低矮障碍 -> LIFT（原有行为，回归确认）----
    f = MainFsm()
    feed(f, 2000, start=1)
    settle(f)
    feed(f, 2000, vision_class=VIS_LOW_OBST, vision_valid=1)
    settle(f, vision_class=VIS_LOW_OBST, vision_valid=1)
    ck.check("11d.1 收到 0x04 低矮障碍 -> LIFT", f.name, 'LIFT/UP')
    ck.check("11d.2 LIFT/UP: lift_cmd=1", f.lift_cmd, 1)

    # ---- 11e: 未知类别（0x07）不应触发任何动作 ----
    f = MainFsm()
    feed(f, 2000, start=1)
    settle(f)
    feed(f, 2000, vision_class=0x07, vision_valid=1)
    settle(f, vision_class=0x07, vision_valid=1)
    ck.check("11e.1 未知类别 0x07 -> 保持 AUTO", f.name, 'AUTO')
    ck.check("11e.2 未知类别不抬腿", f.lift_cmd, 0)

    # ---- 11f: 0x01 人员 / 0x03 目标点目前不改状态 ----
    #     协议表里写的是"减速 / 向目标行进"—— 那两条靠 speed_scale 与
    #     steer 实现，不由本状态机做。这里固化"不该进 AVOID/LIFT"，
    #     免得以后误改。
    for cls in (0x01, 0x03):
        f = MainFsm()
        feed(f, 2000, start=1)
        settle(f)
        feed(f, 2000, vision_class=cls, vision_valid=1)
        settle(f, vision_class=cls, vision_valid=1)
        ck.check("11f.1 类别 0x%02X 不改状态" % cls, f.name, 'AUTO')

    #-------------------------------------------------------------------------
    print()
    if ck.errors == 0:
        print("RESULT: PASS  (%d 项检查全部通过)" % ck.checks)
        print()
        print("说明：这只证明**状态转移逻辑**是对的。")
        print("      复位时序、与其它模块的接口仍需 ModelSim 验证。")
        return 0
    print("RESULT: FAIL  (%d 项检查, %d 个错误)" % (ck.checks, ck.errors))
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()
    return run(args.verbose)


if __name__ == '__main__':
    sys.exit(main())
