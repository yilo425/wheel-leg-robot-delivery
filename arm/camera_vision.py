# -*- coding: utf-8 -*-
"""
camera_vision.py -- 庐山派 K230（CanMV MicroPython）色块识别 -> UART2 发 VISION_RESULT 帧

=============================================================================
这是什么
=============================================================================
detect_from_image.py 那套（OpenCV + uart_proto）只能跑在 PC / Linux 的 CPython 上；
本文件是它的**上机版**：跑在 K230 的 CanMV MicroPython 固件里，用 sensor + image
两个片上库完成"取流 -> 找色块 -> 求质心 -> 归一化 -> 组帧 -> UART2 发出"。

    摄像头(GC2093, CSI2) ──► Sensor.snapshot() ──► find_blobs(按规则) ──► 质心
                                                              │
                     10Hz ◄── UART2 115200 ◄── VISION_RESULT 帧 ◄┘
                                                              ▲
    调试用(非阻塞) ◄──────────── UART2 115200 ◄── TELEMETRY 帧 ◄─┘  FPGA 上报

两条链路挂在同一个 UART2 上，各走各的方向，互不干扰：

    TX 方向：本文件 --VISION_RESULT(0x02)--> FPGA    10Hz 固定节奏发出
    RX 方向：本文件 <--TELEMETRY(0x10)----- FPGA     每拍"顺手收一下"，
                                                     收不到立刻返回，绝不等数据

=============================================================================
硬件接线（庐山派 K230，2026-10-17 实机确认）
=============================================================================
    K230 GPIO05（排针 11）--(TX)-->  FPGA 的 F16（FPGA uart_rx）
    K230 GPIO06（排针 13）--(RX)<--  FPGA 的 C19（FPGA uart_tx）
    GND（排针 6/9/14/20/25/30/34/39 任一）与 FPGA 共地（必须！）
    波特率 115200 8N1，3.3V 直连

    ★ 代码里填的是 **GPIO 号**，不是排针号：
        GPIO05 -> 排针 11
        GPIO06 -> 排针 13
      官网示例里的 set_function(11/12, ...) 是 GPIO11/12，不是排针 11/13。
      照抄会"代码跑通但排针没波形"——本项目上一版就踩过这个坑。

=============================================================================
怎么运行
=============================================================================
    1) 把 camera_vision.py 拷到 /sdcard/（CanMV IDE："文件 -> 保存到设备"）
    2) IDE 里直接运行本文件；或让 /sdcard/main.py 调 main()
    3) 现场调色：IDE 里能实时看到预览画面 + 色块框（USE_DISPLAY = True 时）

=============================================================================
★ 与 PC 版的关键差异（踩坑点）★
=============================================================================
1) **没有 cv2 / numpy**。找色块只能用 image 库的 img.find_blobs()。
2) **阈值是 LAB 不是 HSV**。格式：
        (L_min, L_max, A_min, A_max, B_min, B_max)
   L = 亮度 0..100，A = 红/绿轴 -128..127，B = 黄/蓝轴 -128..127。
   红色表现为 A 明显为正。PC 版那套 HSV 两段阈值在这里**不能直接用**。
3) 没有 uart_proto.py 可 import，所以 norm_to_u16() / 组帧逻辑**内联**在本文件里，
   并且与 uart_proto.py **逐位一致**（已由 test_camera_offline.py 逐字节比对）：
     * 帧长 9+N、BCC 不含帧头帧尾、长度字段大端；
     * 640 宽正中心 320 -> 32768；
     * 连 CPython round() 的"四舍六入五取偶"都手工复刻了（_round_half_even）。
4) **接收侧也要自己写**。PC 版用 uart_proto.FrameParser 是"字节流喂进来"的增量解析器；
   板上没有它，所以本文件里内联了一个等价的 TelemetryRx。
   ⚠️ 接收**绝对不能阻塞**：主循环必须锁在 10Hz，所以只用 uart.any() 判空
   （固件没有 any() 时退化为 timeout=0 的 uart.read()），一个字节都不等。
5) **MediaManager 必须初始化**。CanMV K230 的媒体缓冲由 MediaManager 管理，
   只调 sensor.run() 不调 MediaManager.init() 在部分固件上会直接取不到图。
   本文件按官方顺序：Display.init() -> MediaManager.init() -> sensor.run()；
   退出时反序：sensor.stop() -> Display.deinit() -> MediaManager.deinit()。

=============================================================================
本版（v2）相对上一版改了什么
=============================================================================
  [修] UART 引脚 GPIO11/12 -> GPIO5/6（与 arm_bringup.md 实机接线一致）
  [修] 补 MediaManager.init()/deinit()（不补可能 Sensor.snapshot() 一直失败）
  [修] 遥测接收不再强依赖 uart.any()（K230 固件不一定实现它）
  [增] DETECT_RULES 规则表：一个颜色 -> 一个类别，可加"只在画面下部"
       "宽高比"等几何条件，从而支持 0x04 低矮障碍 / 0x05 台阶 / 0x01 人员
  [增] 实测帧率统计：每 50 帧打印一次真实 Hz，低于 8Hz 会告警
  [增] 发送失败计数、RUN_FRAMES 调试用"跑 N 帧就退出"
  [改] 置信度口径：色块面积占画面比例，2% 画面 = 100 分（FPGA 目前不用它，
       只用于人眼判断识别质量；数值太小会让人误以为"没识别到"）

⚠️ 类别码注意：FPGA 的 main_fsm.v 目前只认
      0x02 障碍物 -> AVOID
      0x04 低矮障碍 -> LIFT（四腿一起抬）
      0x05 台阶 -> LIFT（两腿分段）
   它**没有**实现 0x03 目标点 -> GOAL（GOAL 只由 goal_reached 引脚触发）。
   所以本文件默认不会发 0x03；需要终点导航时先和 FPGA 侧确认。
"""

import time
import os
import sys
import gc

# --- CanMV K230 专属模块（CPython 上没有，所以本文件只能在板子上跑）----------
# ★ 2026-11-03：媒体库改成【懒加载】（在函数内 import）。
#   原因：wifi_rc.py 要 import 本模块复用 classify/make_vision_result 等函数，
#   如果模块级就 from media.xxx import *，在不初始化摄像头的场景会卡住
#   （实测 rc_boot.txt 停在 starting run()，无任何异常）。
#   改成懒加载后，import camera_vision 是安全的；真正用到摄像头时再 import。
from machine import UART, FPIOA  # UART2 + 引脚复用（machine 很轻，可模块级）


#=============================================================================
# ① 现场调参区 —— 比赛现场通常只改这一块
#=============================================================================

# ---- 摄像头 --------------------------------------------------------------
# 庐山派默认摄像头接口是 CSI2；不写 id 时固件默认也是 2，这里显式写出来。
# 如果换了 CSI0/CSI1 的摄像头，把这里改成 0 或 1。
SENSOR_ID = 1              # 当前实际用的 CSI 口（探测成功后会被改写）
SENSOR_TRY = (1, 2, 0)     # ★ 2026-11-03：依次尝试 CSI1 -> CSI2 -> CSI0，第一个能用的就用
                           #   摄像头换过排线口，写死会报 "Can not found sensor on N" 

FRAME_W = 640
FRAME_H = 480

# ---- UART2（★ GPIO 号不是排针号）------------------------------------------
UART_BAUD   = 115200
UART_TX_PIN = 5           # GPIO05 -> 排针 11 -> FPGA F16 (uart_rx)
UART_RX_PIN = 6           # GPIO06 -> 排针 13 <- FPGA C19 (uart_tx)

# ---- 上报类别：默认红色块 = 障碍物 0x02 ------------------------------------
CLS_NONE         = 0x00
CLS_PERSON       = 0x01   # 人员（FPGA 目前只是普通目标，未单独处理）
CLS_OBSTACLE     = 0x02   # 障碍物 -> FPGA AVOID
CLS_GOAL         = 0x03   # 目标点 -> FPGA 暂未实现（见文件头警告）
CLS_LOW_OBSTACLE = 0x04   # 低矮障碍 -> FPGA LIFT（四腿一起抬）
CLS_STEP         = 0x05   # 台阶/门槛 -> FPGA LIFT（两腿分段）
CLS_BACK         = 0x06   # ★ 无法越过也无法避开 -> FPGA 后退
CLS_PASS         = 0x07   # ★ 低矮(<5cm)可直接通过 -> FPGA 不减速继续走

# 调试用：不为 None 时，所有识别结果都强制成这个类别（例如填 0x04 去测抬腿）。
# 比赛/正常跑必须保持 None。
FORCE_CLASS = None

# ---- ★ 识别规则表：每行 = 一种颜色/区域 -> 一个类别 ------------------------
#
# 格式（7 元组，顺序不能错）：
#   (名称,
#    类别码,
#    LAB 阈值列表,        # find_blobs 的 thresholds，可写多条取并集
#    y 中心下限,          # 0.0=画面顶部, 1.0=画面底部；None=不限制
#    y 中心上限,
#    宽高比下限,          # 宽/高；None=不限制
#    宽高比上限)
#
# 匹配顺序：所有规则都会跑一遍，最后取**像素数最多**的那个色块；
# 像素数相同时取规则表里靠前的。所以把"更具体"的规则放前面。
#
# ⚠️ 一帧只发一个目标：FPGA 的 vision_rx 会在多目标里挑置信度最高的一个，
#    而 main_fsm 只用一个类别。多发目标不会让 FPGA 同时做两件事。
#
# 现场怎么调（两条路，任选一条）：
#   A. CanMV IDE：工具 -> 机器视觉 -> 阈值编辑器(Threshold Editor)，
#      用鼠标框住目标，拖动滑块到只剩目标被选中，
#      把界面右上角生成的元组抄回下面。
#   B. 手改数字：
#        色块被切碎/断续          -> 放宽 A/B 区间，并确认 MERGE = True
#        偏橙、砖红的红被漏掉      -> 调小 A_min（例如 30 -> 15）
#        暗红/阴影里的红漏检       -> 确认 L_min 已为 0，再调小 A_min
#        粉红/白墙/肤色误检        -> 调大 A_min（例如 30 -> 45）
#        紫色/品红误检             -> 调大 B_min 或调小 B_max
#
# 参考锚点：纯正红约 A≈+60..+90、B≈0；纯白 A≈0、B≈0；纯绿 A≈-60。
# ⚠️ 本项目的红色瓶身偏暗，实测 A_min=30 会把暗红部分切掉、只剩中间一条
#    （17k 像素 -> 框不到整只瓶子）；A_min=20 能稳定框住整只（40k+ 像素）。
#    如果现场出现肤色/木纹误检，再把 A_min 往上调（20 -> 25/30），或收紧 B。
#
# ---- 默认规则：红色块 -> 障碍物 -------------------------------------------
# ★ 2026-11-03：不能只认红色。障碍物可能是任意颜色，
#   这里按 LAB 给出几组常见色的区间，谁的面积大就取谁（classify 已经是这个策略）。
#   ⚠️ LAB 现场必须重新标定：光照一变阈值就飘，用 IDE 的阈值编辑器抓一下再改。
#   格式：(L_min,L_max, A_min,A_max, B_min,B_max)
#     红色  A 明显为正   | 蓝色  B 明显为负   | 绿色  A 明显为负
#     黄色  B 明显为正   | 深色  L 很小（注意会误检阴影，慎用）
DETECT_RULES = [
    ('红色障碍物', CLS_OBSTACLE, [(0, 100,  20, 127, -20,  80)], None, None, None, None),
    ('蓝色障碍物', CLS_OBSTACLE, [(0, 100, -128, -20, -128, 127)], None, None, None, None),
    ('绿色障碍物', CLS_OBSTACLE, [(0, 100, -128, -20,  -20,  80)], None, None, None, None),
    ('黄色障碍物', CLS_OBSTACLE, [(0, 100,  -20,  20,   20, 127)], None, None, None, None),
    # ('深色障碍物', CLS_OBSTACLE, [(0, 25, -128, 127, -128, 127)], None, None, None, None),
]

# ---- 需要"低矮障碍 -> 抬腿"时，取消下面这一行的注释，并放在上面那条**前面** --
# 含义：红色的、中心在画面下 40%、且明显偏宽（宽/高 >= 1.2）的块才算低矮障碍。
# 为什么加几何条件：低矮障碍通常贴地、在画面下方、横向比纵向宽；
# 只按颜色会和普通障碍物抢同一块红色，误触发抬腿很危险。
#
# DETECT_RULES = [
#     ('红色低矮障碍', CLS_LOW_OBSTACLE, [(0, 100, 20, 127, -20, 80)],
#      0.60, 1.0, 1.2, None),
# ] + DETECT_RULES
#
# ---- 台阶（0x05）示例：蓝色门槛 -------------------------------------------
# DETECT_RULES = [
#     ('蓝色台阶', CLS_STEP, [(0, 100, -128, -20, -128, 127)],
#      None, None, None, None),
# ] + DETECT_RULES

# ---- 色块过滤（换分辨率后要一起调）------------------------------------------
PIXELS_THRESHOLD = 200    # 色块最少像素数，小于它的当噪点丢掉
AREA_THRESHOLD   = 200    # 色块外接框最少面积，同上
MERGE            = True   # 相邻/重叠的小色块合并成一个（被遮挡时很关键）
MERGE_MARGIN     = 15     # 合并时外接框外扩的像素数；能跨过标签/反光造成的小缝
MIN_AREA_RATIO   = 0.0005 # 面积占整幅画面比例门限（<0.05% 当噪声），与 PC 版一致

# 置信度口径：色块面积占画面 = 这个比例时记 100 分。
# 2% 画面 ≈ 640x480 里的 80x80 色块。FPGA 目前不消费置信度，只用于现场观察。
CONF_FULL_RATIO = 0.02

# ---- 循环频率 --------------------------------------------------------------
TARGET_HZ = 10
PERIOD_MS = 1000 // TARGET_HZ          # = 100ms，即 10Hz

# ---- 预览（现场调色开，比赛建议关；VIRT 走 USB，开太高会拖慢主循环）---------
USE_DISPLAY      = False  # ★ 2026-11-03：脱机录像必须 False（没有 IDE，开了会失败/拖慢 10Hz）
USE_IDE_OUTPUT   = False  # ★ 2026-11-03：同上   # True=预览同时输出到 CanMV IDE 帧缓冲窗口
DISPLAY_FPS      = 10     # 预览帧率；跟主循环同频即可，调高只会更占 USB
USE_MIRROR       = False  # True=画面左右镜像+上下翻转（摄像头装反了才开）

# ---- 打印 / 调试 -----------------------------------------------------------
DEBUG_PRINT       = True   # True=打印每帧识别结果+十六进制帧
PROBE_CENTER_PIXEL = False # True=额外打印画面中心像素 RGB，方便肉眼估色
FPS_PRINT_EVERY   = 50     # 每多少帧打印一次真实帧率（10Hz 下 = 每 5 秒）
RUN_FRAMES        = 0      # >0 时只跑这么多帧就退出（自动化测试/离线回归用）

# ---- 接收 FPGA 遥测 TELEMETRY(0x10)【非阻塞，绝不影响 10Hz】----------------
USE_RX_TELEMETRY = True   # True=每拍顺手收一下 FPGA 的遥测；False=完全只发不收
RX_PRINT         = True   # True=遥测有变化就打印；False=收下来但不打印（更安静）
RX_BUF_MAX       = 512    # 接收累积缓冲上限（字节），超了丢旧数据，防噪声灌爆内存


#=============================================================================
# ② 协议内联（从 uart_proto.py 抄出来的核心逻辑；板子上没有 uart_proto.py）
#=============================================================================
# 帧 = AA 55 | VER | TYPE | LEN(2B大端) | DATA | BCC | 55 AA
#          └────────── BCC 计算范围 ──────────┘
#   帧总长 = 9 + N
#   BCC    = VER ^ TYPE ^ LEN_H ^ LEN_L ^ DATA[0..N-1]   ← **不含帧头帧尾**
#
# ⚠️ 帧开销是 9 不是 10 —— 我们自测时算错过一次。
HDR  = b'\xAA\x55'
TAIL = b'\x55\xAA'
PROTO_VER          = 0x01
TYPE_VISION_RESULT = 0x02
TYPE_TELEMETRY     = 0x10


def bcc(body):
    """BCC = 逐字节异或。body 必须是「版本+类型+长度(2B)+数据」，不含帧头帧尾。"""
    x = 0
    for b in body:
        x ^= b
    return x & 0xFF


def pack_frame(ftype, payload=b'', ver=PROTO_VER):
    """打一帧完整字节串：帧头 + 版本 + 类型 + 长度(2B) + 数据 + BCC + 帧尾。

    >>> hexdump(pack_frame(0x03, b'\x02'))
    'AA 55 01 03 00 01 02 01 55 AA'

    （BCC = 01^03^00^01^02 = 0x01。本内联版与 uart_proto.pack_frame 的输出
    已由 test_camera_offline.py 逐位比对，完全一致。）
    """
    payload = bytes(payload)
    body = bytes([ver, ftype & 0xFF,
                  (len(payload) >> 8) & 0xFF, len(payload) & 0xFF]) + payload
    return HDR + body + bytes([bcc(body)]) + TAIL


def make_vision_result(targets):
    """组 VISION_RESULT 帧。

    targets = [(类别, cx归一化, cy归一化, 置信度0..100), ...]，最多 8 个。
    载荷 = 目标数(1B) + 每个目标 7B：
            序号(1) + 类别(1) + cx(2,大端) + cy(2,大端) + 置信度(1)
    """
    targets = list(targets)
    if len(targets) > 8:
        targets = targets[:8]            # 板上不抛异常，超了就截断

    p = bytes([len(targets)])
    for idx, t in enumerate(targets):
        cls, cx, cy, conf = t
        p += bytes([idx & 0xFF,
                    cls & 0xFF,
                    (cx >> 8) & 0xFF, cx & 0xFF,
                    (cy >> 8) & 0xFF, cy & 0xFF,
                    int(conf) & 0xFF])
    return pack_frame(TYPE_VISION_RESULT, p)


def _round_half_even(x):
    """复刻 CPython round() 的"四舍六入五取偶"（banker's rounding）。

    为什么不用 round() 本身：uart_proto.py 跑在 CPython 上，用的是"取偶"规则；
    而 MicroPython 各固件的 round() 可能是"五入"。为了 PC 参考实现和板子**逐位一致**
    （这是本项目"两边行为完全一致"的设计原则），这里手工复刻。
    只在 x >= 0 时使用。
    """
    i = int(x)                 # x >= 0 时即向下取整
    f = x - i
    if f > 0.5:
        return i + 1
    if f < 0.5:
        return i
    return i if (i & 1) == 0 else i + 1   # 正好 .5：取偶数


def norm_to_u16(px, total):
    """像素坐标 -> 归一化 0..65535（与 uart_proto.norm_to_u16 **逐位一致**）。

    与 uart_proto.py 的唯一差异：
      * total <= 0 时返回 0，不抛 ValueError（板子上宁可发个 0 也不能崩）。
    另外：像素坐标先钳到 [0, total]，等价于 uart_proto 最后的 max/min 钳位，
    这样越界/负数输入两边结果也一模一样。
    """
    if total <= 0:
        return 0
    if px < 0:
        px = 0
    elif px > total:
        px = total
    return _round_half_even(px / total * 65535.0)


def x_zone(cx):
    """FPGA 侧的三档转向门限（只用于调试打印，便于和 FPGA 对账）。"""
    if cx < 21845:
        return 'left'
    if cx > 43690:
        return 'right'
    return 'center'


def hexdump(frame):
    """与 fpga_link.hexdump 同风格：大写、空格分隔。"""
    return ' '.join('%02X' % b for b in frame)


#=============================================================================
# ②b 接收侧协议（TELEMETRY，FPGA -> ARM）—— 与 uart_proto.py 逐字段对齐
#=============================================================================
# 帧格式和发送侧完全一样（AA 55 | VER | TYPE | LEN | DATA | BCC | 55 AA），
# 只是方向反过来；所以 bcc() / HDR / TAIL / PROTO_VER / pack_frame() 直接复用。
MAX_PAYLOAD = 64        # 载荷上限（uart_proto.MAX_PAYLOAD），超了按噪声处理

# 本文件认识的帧类型（未知类型完整收完后丢弃，保持字节同步 —— 与 uart_proto 一致）
KNOWN_TYPES = (0x01, 0x02, 0x03, 0x04, 0x10)

# TELEMETRY 载荷（13 字节）：
#   状态(1) + 速度L(2,有符号) + 速度R(2,有符号) + 距离(2,无符号)
#   + roll(2,有符号) + pitch(2,有符号) + yaw(2,有符号)      ← 姿态角单位 0.01 度
# 对应 uart_proto._TELEMETRY_FMT = '>BhhHhhh'，偏移分别是 0/1/3/5/7/9/11。
TELEMETRY_LEN = 13

# 主状态机编码（uart_proto.STATE_NAMES）
STATE_NAMES = {
    0: 'S_IDLE', 1: 'S_MANUAL', 2: 'S_AUTO', 3: 'S_AVOID',
    4: 'S_LIFT', 5: 'S_GOAL',   6: 'S_ESTOP', 7: 'S_CHECK',
}

# -----------------------------------------------------------------------------
# ★★ 2026-11-03：障碍物高度估计（演示版）
#
#   原理：单目针孔模型
#       实物高度 ≈ 障碍物在画面里的像素高度 × 实际距离 ÷ 焦距
#
#   · 距离用【停车后复测】的超声波值（车停下还会往前滑一点，
#     行进中直接测会偏短，所以必须用 FC 停车后的读数）。
#   · 摄像头安装高度取低位 12cm（只关心低位）。
#   · 焦距按【官方镜头规格】算，不是估的：
#       庐山派标准版附送镜头 = 立创 GC2093-200W 小镜头版
#       EFL 4.3mm，视场角 D73°/H65°/V40°，感光 1/2.9"
#       满分辨率 1920x1080：水平 (1920/2)/tan(32.5°)=1507px，
#                          垂直 (1080/2)/tan(20°) =1484px（两者一致 ✓ 说明规格可信）
#       输出 640x480 = 4:3 裁切(1440x1080)后等比缩放(0.4444x)
#       -> (1080/2)/tan(20°) * 480/1080 ≈ 659px
#     现场若换了镜头，改这一个数就行（或用标定板反推）。
#
#   判高度只在 FPGA 处于 S_CHECK（停车检查）时做 —— 其它时候仍按颜色分类，
#   这样不影响正常巡线。
# -----------------------------------------------------------------------------
CAM_HEIGHT_MM    = 120    # 摄像头安装高度（低位 12cm）
CAM_FOCAL_PX     = 659    # 焦距（像素，按官方 H65°/V40° 规格在 640x480 下换算）
OBST_PASS_MAX_MM = 50     # ★ < 5cm：低位不减速直接过
OBST_LIFT_MAX_MM = 80     # ★ 5~8cm：抬腿跨过
                          #    > 8cm：抬腿也过不去 -> 后退（CLS_BACK）
DIST_JUDGE_MIN   = 300    # 只在 30~90cm 区间做高度判断
DIST_JUDGE_MAX   = 900
ST_CHECK         = 7      # FPGA main_fsm 的 S_CHECK 编码


def estimate_obst_height_mm(blob, dist_mm):
    """单目几何估算障碍物高度（毫米）。参数非法时返回 None。"""
    try:
        if dist_mm <= 0:
            return None
        hpx = blob.h()
        if hpx <= 0:
            return None
        return int(hpx * dist_mm / CAM_FOCAL_PX)
    except Exception:
        return None

# -----------------------------------------------------------------------------
# ★★ 2026-11-03：绕行直行时长 = 障碍物实测宽度 / 最大速度 + 0.2s 余量
#
#   为什么不再用固定 1s：
#     障碍物可能很大，固定 1s 走不完就会蹭上去；太小又浪费时间。
#     按"宽度 / 最大速度"算，正好是走完这段距离所需时间，
#     再 +0.2s 覆盖起步加速和收尾减速。
#
#   最大速度 V_MAX_MMPS：满速档实测约 1257 mm/s（v ≈ 12.57 mm/s 每 1%）。
#   0.2s 余量：起步要 0.3~0.5s 才到满速，减速停车也要几十毫秒，取 200ms 折中。
# -----------------------------------------------------------------------------
V_MAX_MMPS    = 1257     # 满速（mm/s）
RUN_MARGIN_MS = 200      # 起步 + 减速余量（ms）
RUN_MS_MIN    = 200
RUN_MS_MAX    = 10000

# CONFIG 帧类型（ARM -> FPGA 参数下发）
TYPE_CONFIG   = 0x04


def make_config(param_id, value_bytes):
    """参数配置帧：载荷 = 参数ID(1) + 长度(1) + 值（与 uart_proto.make_config 一致）"""
    value_bytes = bytes(value_bytes)
    return pack_frame(TYPE_CONFIG, bytes([param_id & 0xFF, len(value_bytes)]) + value_bytes)


def estimate_obst_width_mm(blob, dist_mm):
    """单目几何估算障碍物宽度（毫米）—— 和高度同一套针孔模型。"""
    try:
        if dist_mm <= 0:
            return 0
        wpx = blob.w()
        if wpx <= 0:
            return 0
        return int(wpx * dist_mm / CAM_FOCAL_PX)
    except Exception:
        return 0


def width_to_run_ms(width_mm):
    """绕行直行时长（毫秒）= 宽度 / 最大速度 + 0.2s 余量。"""
    try:
        t = int(width_mm * 1000.0 / V_MAX_MMPS) + RUN_MARGIN_MS
        if t < RUN_MS_MIN:
            t = RUN_MS_MIN
        if t > RUN_MS_MAX:
            t = RUN_MS_MAX
        return t
    except Exception:
        return 1000


def _u16(b, i):
    """大端读一个无符号 16 位。"""
    return (b[i] << 8) | b[i + 1]


def _s16(b, i):
    """大端读一个有符号 16 位（补码）—— FPGA 报的速度/姿态角都是补码。"""
    v = (b[i] << 8) | b[i + 1]
    return v - 0x10000 if v & 0x8000 else v


def parse_telemetry(payload):
    """解析 TELEMETRY 载荷 -> dict（与 uart_proto.parse_telemetry 逐字段一致）。

    返回:
        {
            'state'      : 原始状态码 0..6,
            'state_name' : 'S_AUTO' 之类；不认识的状态显示 'UNKNOWN(n)',
            'speed_left' : 左轮速度（有符号，正=前进）,
            'speed_right': 右轮速度（有符号）,
            'dist_mm'    : 前方距离（毫米，无符号）,
            'roll_deg'   : 横滚角（度，已从 0.01 度换算）,
            'pitch_deg'  : 俯仰角（度）,
            'yaw_deg'    : 偏航角（度）,
        }

    不用 struct：板上没这个必要，手写大端解析更省内存，
    也顺手避免了 MicroPython 各固件 struct 对齐行为的差异。
    载荷长度不对时抛 ValueError —— 调用方（rx_telemetry）会兜住。
    """
    if len(payload) != TELEMETRY_LEN:
        raise ValueError('TELEMETRY 载荷应为 %d 字节，实际 %d'
                         % (TELEMETRY_LEN, len(payload)))

    # ★ 2026-11-03：byte0 = {1'b0, flags[3:0], state[2:0]}，必须掩码取低 3 位
    state = payload[0] & 0x07
    return {
        'state':       state,
        'state_name':  STATE_NAMES.get(state, 'UNKNOWN(%d)' % state),
        'speed_left':  _s16(payload, 1),
        'speed_right': _s16(payload, 3),
        'dist_mm':     _u16(payload, 5),
        'roll_deg':    _s16(payload, 7)  / 100.0,
        'pitch_deg':   _s16(payload, 9)  / 100.0,
        'yaw_deg':     _s16(payload, 11) / 100.0,
    }


class TelemetryRx:
    """增量式接收帧解析器 —— 喂任意切分的字节流，吐出校验通过的完整帧。

    ★ 为什么必须"增量式"：串口数据没有边界，一次 read 可能只拿到半截帧，
      也可能一次拿到好几帧。所以没凑齐的字节要存下来等下一拍。
      行为与 uart_proto.FrameParser / FPGA 的 uart_frame.v 对齐：

        * 先找帧头 AA 55，帧头前面的字节当噪声丢掉；
        * BCC 错 / 帧尾错 -> **只退 2 字节**重新找帧头，
          所以后面紧跟的好帧照样能解出来；
        * 版本错 -> 同上；
        * 未知类型 -> 这一帧丢掉，但字节位置是准的，不影响后续帧；
        * 长度字段超上限（> MAX_PAYLOAD）-> 视为噪声，跳过帧头重找。
          在 ARM 侧没有帧内超时，选"跳过重找"更安全：
          绝不会被一个假长度字段卡死。这是本实现的**唯一**有意差异。

    用法:
        rx = TelemetryRx()
        for ftype, payload in rx.feed(uart.read()):
            ...
    """

    def __init__(self):
        self.buf = b''
        self.stats = {
            'bytes': 0, 'frames_ok': 0, 'bcc_err': 0, 'tail_err': 0,
            'bad_ver': 0, 'dropped': 0, 'noise_bytes': 0, 'overflow': 0,
        }

    def feed(self, data):
        """喂任意长度的字节，返回本次解析出的 [(类型, 载荷), ...]。"""
        if not data:
            return []
        self.stats['bytes'] += len(data)
        self.buf += bytes(data)

        out = []
        while True:
            # ---- 1. 找帧头 AA 55 ----
            i = self.buf.find(HDR)
            if i < 0:
                # 没有帧头。结尾单独一个 AA 要留着 —— 它可能是下一帧帧头的前半
                if self.buf[-1:] == b'\xAA':
                    self.stats['noise_bytes'] += len(self.buf) - 1
                    self.buf = self.buf[-1:]
                else:
                    self.stats['noise_bytes'] += len(self.buf)
                    self.buf = b''
                break
            if i:
                self.stats['noise_bytes'] += i
                self.buf = self.buf[i:]              # 掐掉帧头前面的噪声

            # ---- 2. 帧头后面至少要凑齐 版本+类型+长度(2B)，共 6 字节 ----
            if len(self.buf) < 6:
                break                                # 半截帧，留着等下一拍

            ver   = self.buf[2]
            ftype = self.buf[3]
            n     = (self.buf[4] << 8) | self.buf[5]

            if ver != PROTO_VER:
                self.stats['bad_ver'] += 1
                self.buf = self.buf[2:]              # 跳过 2 字节重找帧头
                continue
            if n > MAX_PAYLOAD:
                self.stats['dropped'] += 1           # 长度字段离谱，当噪声
                self.buf = self.buf[2:]
                continue

            total = 9 + n                            # 帧总长 = 9 + N
            if len(self.buf) < total:
                break                                # 载荷还没到齐

            # ---- 3. 校验 BCC + 帧尾 ----
            if bcc(self.buf[2:6 + n]) != self.buf[6 + n]:
                self.stats['bcc_err'] += 1
                self.buf = self.buf[2:]              # ★ 只退 2 字节，不连累后面的好帧
                continue
            if self.buf[7 + n] != 0x55 or self.buf[8 + n] != 0xAA:
                self.stats['tail_err'] += 1
                self.buf = self.buf[2:]
                continue

            # ---- 4. 校验过了：认识就交付，不认识就丢 ----
            if ftype not in KNOWN_TYPES:
                self.stats['dropped'] += 1
            else:
                self.stats['frames_ok'] += 1
                out.append((ftype, self.buf[6:6 + n]))
            self.buf = self.buf[total:]

        # 兜底：极端噪声下别让缓冲无限长（正常情况根本碰不到）
        if len(self.buf) > RX_BUF_MAX:
            self.stats['overflow'] += 1
            self.buf = self.buf[-64:]

        return out


#=============================================================================
# ③ 10Hz 节奏控制（用 MicroPython 标准 time 模块的 ticks_ms）
#=============================================================================
_HAVE_TICKS = hasattr(time, 'ticks_ms')


def keep_10hz(t0):
    """把每轮循环的总耗时补到 PERIOD_MS，让整环稳定在 TARGET_HZ。

    极老固件没有 ticks_ms 时退化为"固定 sleep"，此时频率 ≤10Hz（不会超）。
    """
    if _HAVE_TICKS:
        rest = PERIOD_MS - time.ticks_diff(time.ticks_ms(), t0)
    else:
        rest = PERIOD_MS
    if rest > 0:
        time.sleep_ms(rest)


def _print_fps_if_due(frame_no, win_t0, win_n):
    """每 FPS_PRINT_EVERY 帧打印一次真实帧率；返回新的窗口起点和计数。"""
    if (not _HAVE_TICKS) or FPS_PRINT_EVERY <= 0 or win_n < FPS_PRINT_EVERY:
        return win_t0, win_n
    dt = time.ticks_diff(time.ticks_ms(), win_t0)
    hz = (win_n * 1000.0 / dt) if dt > 0 else 0.0
    print('  [性能] 最近 %d 帧 %.1f Hz（目标 %d Hz）'
          % (win_n, hz, TARGET_HZ))
    if hz < TARGET_HZ * 0.8:
        print('  [性能] ⚠ 低于目标的 80%%：先关 USE_DISPLAY / 降低分辨率 / '
              '减少 DETECT_RULES 条数')
    return time.ticks_ms(), 0


#=============================================================================
# ④ UART2 初始化（GPIO5/GPIO6 通过 FPIOA 复用成 UART2）
#=============================================================================
def setup_uart():
    """把 GPIO5/6 复用成 UART2 的 TXD/RXD，然后按 115200 8N1 打开 UART2。

    ★ GPIO 号不是排针号：GPIO5 = 排针 11，GPIO6 = 排针 13。
    timeout=0 表示读不阻塞：主循环每拍只"顺手收一下"，绝不等数据。
    """
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)   # GPIO5 -> 排针 11
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)   # GPIO6 -> 排针 13

    uart = UART(UART.UART2, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE, timeout=0)
    print('UART2 就绪: TX=GPIO%d(排针11), RX=GPIO%d(排针13), %d 8N1'
          % (UART_TX_PIN, UART_RX_PIN, UART_BAUD))
    return uart


#=============================================================================
# ⑤ 摄像头初始化（GC2093 走 CSI2）
#=============================================================================
def setup_camera():
    """建 Sensor 对象 + 设帧率分辨率 + 设 RGB565 输出。

    庐山派默认 CSI2，所以 SENSOR_ID = 2；换成 CSI0/CSI1 时改常量即可。
    """
    from media.sensor import Sensor
    global SENSOR_ID

    last_err = None
    for sid in SENSOR_TRY:
        try:
            sensor = Sensor(id=sid, width=FRAME_W, height=FRAME_H)
            sensor.reset()
            sensor.set_framesize(width=FRAME_W, height=FRAME_H)
            sensor.set_pixformat(Sensor.RGB565)          # find_blobs 需要彩色图
            if USE_MIRROR:
                sensor.set_hmirror(True)
                sensor.set_vflip(True)
            SENSOR_ID = sid
            print('[cam] sensor OK on CSI%d' % sid)
            return sensor
        except Exception as e:
            last_err = e
            print('[cam] CSI%d failed: %s' % (sid, e))

    if last_err is not None:
        raise last_err
    raise RuntimeError('no sensor port available')


def start_camera(use_display=None):
    """一次性把摄像头跑起来（Sensor + MediaManager），返回 sensor。

    ★ 2026-11-03 新增：给 wifi_rc.py 合并版用。
      按官方顺序：Display.init()（可选）-> MediaManager.init() -> sensor.run()
    """
    from media.media import MediaManager

    if use_display is None:
        use_display = USE_DISPLAY

    sensor = setup_camera()
    if use_display:
        from media.display import Display
        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H,
                     fps=DISPLAY_FPS, to_ide=USE_IDE_OUTPUT)
    MediaManager.init()
    sensor.run()
    return sensor


def stop_camera(sensor=None):
    """反序停掉（sensor.stop -> MediaManager.deinit）。失败不抛。"""
    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass
    try:
        from media.media import MediaManager
        MediaManager.deinit()
    except Exception:
        pass


#=============================================================================
# ⑥ 识别：按 DETECT_RULES 找色块 -> (类别, blob, 规则名)
#=============================================================================
def find_red_blob(img):
    """在老的单规则场景下，返回第一条规则里面积最大的色块；没有则 None。

    保留这个函数是为了兼容旧调试脚本；主循环现在走 classify()。
    """
    return _find_best_blob(img, DETECT_RULES[0][2])


def _find_best_blob(img, thresholds):
    """按一组 LAB 阈值找 blobs，返回像素数最多的那个；没有则 None。"""
    blobs = img.find_blobs(thresholds,
                           pixels_threshold=PIXELS_THRESHOLD,
                           area_threshold=AREA_THRESHOLD,
                           merge=MERGE,
                           margin=MERGE_MARGIN)
    if not blobs:
        return None
    return max(blobs, key=lambda b: b.pixels())


def classify(img, rules=None):
    """按规则表识别，返回 (类别码, blob, 规则名)；没有命中返回 None。

    每条规则依次调用 find_blobs（阈值不同、可选几何过滤），
    最后在**所有命中项**里取像素数最大的一个。
    这样"红色低矮障碍"和"红色普通障碍"可以共存：谁的面积大听谁的。
    """
    if rules is None:
        rules = DETECT_RULES

    w = img.width()
    h = img.height()
    total_px = float(w * h)

    best = None          # (pixels, cls, blob, rule_name)
    for rule in rules:
        name, cls, thresholds, y_min, y_max, ar_min, ar_max = rule

        blobs = img.find_blobs(thresholds,
                               pixels_threshold=PIXELS_THRESHOLD,
                               area_threshold=AREA_THRESHOLD,
                               merge=MERGE,
                               margin=MERGE_MARGIN)
        if not blobs:
            continue

        for b in blobs:
            px = b.pixels()
            if px < PIXELS_THRESHOLD:
                continue

            ratio = px / total_px
            if ratio < MIN_AREA_RATIO:
                continue

            # ---- 可选的"位置"条件：只认画面下半部分的目标 ----
            if (y_min is not None) or (y_max is not None):
                ny = b.cy() / float(h)
                if (y_min is not None) and (ny < y_min):
                    continue
                if (y_max is not None) and (ny > y_max):
                    continue

            # ---- 可选的"形状"条件：宽高比 ----
            if (ar_min is not None) or (ar_max is not None):
                bh = b.h()
                if bh <= 0:
                    continue
                ar = b.w() / float(bh)
                if (ar_min is not None) and (ar < ar_min):
                    continue
                if (ar_max is not None) and (ar > ar_max):
                    continue

            if (best is None) or (px > best[0]):
                best = (px, cls, b, name)

    if best is None:
        return None
    return (best[1], best[2], best[3])


def blob_to_target(img, blob, cls):
    """色块 -> uart_proto 目标 (类别, cx, cy, 置信度)，并回传质心像素坐标。

    - cx/cy 用 norm_to_u16() 归一化到 0..65535（绝不发像素坐标，换分辨率不用改 FPGA）
    - 质心取 blob.cx()/blob.cy()（色块质心，比外接框中心稳）
    - 置信度 = 色块面积占画面比例 / CONF_FULL_RATIO，折算到 0..100
    """
    w = img.width()
    h = img.height()

    px = blob.cx()
    py = blob.cy()
    cx = norm_to_u16(px, w)
    cy = norm_to_u16(py, h)

    ratio = blob.pixels() / float(w * h)
    conf = int(ratio / CONF_FULL_RATIO * 100.0 + 0.5)   # 四舍五入
    if conf < 0:
        conf = 0
    elif conf > 100:
        conf = 100

    return (cls, cx, cy, conf), (px, py)


#=============================================================================
# ⑥b 收 FPGA 遥测（TELEMETRY）—— 非阻塞，绝不拖慢 10Hz
#=============================================================================
def _read_available(uart):
    """读走串口缓冲区里"已经到的那点数据"，一个字节都不等。

    ★★ 2026-10-27 实测 Bug（庐山派 Lite / CanMV v1.8）★★
        uart.any() **不可靠**：缓冲区里明明有 1320 字节，它返回的是 1。
        旧实现照 any() 的数量去 read(avail)，等于**每拍只读 1 字节**，
        而 FPGA 以 220 B/s 在发、这边每 10ms 才取 1 字节（≈100 B/s）
        -> 接收缓冲区被灌满、大量丢字节 -> 帧永远拼不完整。

        表现（坑了很久）：
            "收到 TELEMETRY 0 帧；噪声 299B"
            299 ≈ 3 秒 × 每 10ms 1 字节 —— 和这个 bug 完全吻合。

    现在的做法：**根本不问 any()**，直接 read()。
    UART 构造时 timeout=0，无数据时立即返回 None/b''，依然"一个字节都不等"。
    """
    try:
        return uart.read()
    except Exception:
        return None


def rx_telemetry(uart, rx, memo):
    """顺手收一下 UART 上的遥测，解析出来喂给 memo 去重打印。

    ★★ 本文件接收逻辑的第一原则：**一个字节都不等**。★★
        所以不管 FPGA 有没有在发、发得多快，
        这一拍最多就是"多读一次缓冲区"，10Hz 发送节奏丝毫不受影响。

    读不到数据时返回 0，本拍什么都不做 —— 宁可漏一条遥测，也绝不能卡住发帧。

    memo  记"上次打印过的遥测内容"的 dict（见 _print_telemetry_if_changed）
    返回  本拍收到的 TELEMETRY 帧数
    """
    chunk = _read_available(uart)
    if not chunk:
        return 0

    n_frames = 0
    for ftype, payload in rx.feed(chunk):
        if ftype != TYPE_TELEMETRY:
            continue                       # 别的东西（比如 ACK）先忽略
        n_frames += 1
        try:
            d = parse_telemetry(payload)
        except Exception as e:
            # 载荷长度不对之类：同样的错误只报一次，别刷屏
            if memo.get('bad_payload') != str(e):
                memo['bad_payload'] = str(e)
                print('  [RX] TELEMETRY 载荷解析失败: %s' % e)
            continue
        memo['last'] = d          # ★ 2026-11-03：主循环要读 state / dist_mm
        _print_telemetry_if_changed(d, memo)
    return n_frames


def _print_telemetry_if_changed(d, memo):
    """遥测和上一条**完全相同**就不打印 —— FPGA 待机时遥测几乎不变，否则刷屏。

    只有内容变了才打印；顺带把"上一条重复了多少帧"带上，
    这样既不刷屏，又能看出 FPGA 一直在稳定上报（而不是掉线了）。
    返回 True 表示这次打印了。
    """
    key = (d['state'], d['speed_left'], d['speed_right'], d['dist_mm'],
           d['roll_deg'], d['pitch_deg'], d['yaw_deg'])

    if memo.get('key') == key:
        memo['same'] = memo.get('same', 0) + 1
        return False

    repeat = memo.get('same', 0)           # 上一条连续重复了多少拍
    memo['key'] = key
    memo['same'] = 0

    if RX_PRINT:
        tail = '   (上一条重复了 %d 帧)' % repeat if repeat else ''
        print('  [RX] 状态=%-9s 速度L=%6d 速度R=%6d 距离=%5dmm '
              '姿态 r/p/y=%7.2f/%7.2f/%7.2f%s'
              % (d['state_name'], d['speed_left'], d['speed_right'], d['dist_mm'],
                 d['roll_deg'], d['pitch_deg'], d['yaw_deg'], tail))
    return True


def _print_rx_stats_if_changed(rx, memo):
    """接收出错计数**只在变化时**报一条（免得每拍刷屏）。

    只盯错误类计数：好帧数是正常增长的，拿它当触发条件会直接刷屏。
    """
    s = rx.stats
    key = (s['bcc_err'], s['tail_err'], s['bad_ver'], s['dropped'], s['overflow'])
    if memo.get('err_key') == key:
        return
    memo['err_key'] = key
    if key != (0, 0, 0, 0, 0):
        print('  [RX] ⚠ 接收出错计数: 好帧%d BCC错%d 帧尾错%d 版本错%d 丢弃%d '
              '溢出%d 噪声%dB'
              % (s['frames_ok'], s['bcc_err'], s['tail_err'], s['bad_ver'],
                 s['dropped'], s['overflow'], s['noise_bytes']))


#=============================================================================
# ⑦ 发送（失败不让主循环崩）
#=============================================================================
def send_frame(uart, frame):
    """发一帧；成功返回 True。失败只打印，不抛异常（视觉节点不能因串口挂掉）。"""
    try:
        n = uart.write(frame)
        if (n is not None) and (n != len(frame)):
            print('  [TX] 串口只写入 %d/%d 字节' % (n, len(frame)))
            return False
        return True
    except Exception as e:
        print('  [TX] 写串口失败: %s' % e)
        return False


#=============================================================================
# ⑧ 主循环：取流 -> 识别 -> 组帧 -> 发 UART2，整环锁在 10Hz
#=============================================================================
def main():
    uart = setup_uart()

    sensor = None
    display_ready = False
    media_ready = False
    frame_no = 0
    tx_ok = 0
    tx_err = 0

    # ---- 接收侧状态（TELEMETRY）----
    rx_parser = TelemetryRx()   # 增量式帧解析器：半截帧留到下一拍接着拼
    rx_memo   = {}              # 去重用：记住上次打印过的遥测 + 上次的错误计数
    rx_ok     = 0               # 累计收到的 TELEMETRY 帧数

    # ---- 帧率统计窗口 ----
    if _HAVE_TICKS:
        win_t0 = time.ticks_ms()
    else:
        win_t0 = 0
    win_n = 0

    try:
        sensor = setup_camera()

        if USE_DISPLAY:
            # 现场调色建议开 IDE 预览（Display.VIRT）；
            # 接板载/HDMI 屏时把 VIRT 换成 Display.ST7701 / Display.LT9611。
            Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H,
                         fps=DISPLAY_FPS, to_ide=USE_IDE_OUTPUT)
            display_ready = True

        # ★ 按官方顺序初始化媒体缓冲：
        #   Display.init() -> MediaManager.init() -> sensor.run()
        MediaManager.init()
        media_ready = True

        sensor.run()
        print('camera_vision 启动: sensor%d %dx%d, 规则%d条, 频率%dHz'
              % (SENSOR_ID, FRAME_W, FRAME_H, len(DETECT_RULES), TARGET_HZ))
        print('  发送 VISION_RESULT(0x%02X) @ GPIO%-2d 10Hz 固定节奏'
              % (TYPE_VISION_RESULT, UART_TX_PIN))
        if USE_RX_TELEMETRY:
            print('  接收 TELEMETRY(0x%02X)     @ GPIO%-2d 非阻塞，收到就解析'
                  % (TYPE_TELEMETRY, UART_RX_PIN))
        else:
            print('  接收 TELEMETRY           关闭（USE_RX_TELEMETRY = False）')
        if FORCE_CLASS is not None:
            print('  ⚠ FORCE_CLASS=0x%02X：所有识别结果都强制成这个类别'
                  % FORCE_CLASS)

        while True:
            os.exitpoint()                      # 让 CanMV IDE 的"停止"按钮能中断
            t0 = time.ticks_ms() if _HAVE_TICKS else 0

            # ---- 1. 取一帧 ----
            img = sensor.snapshot()

            # ---- 2. 按规则识别 ----
            hit = classify(img)

            # ---- 3. 折算 + 组帧 + 发送 ----
            if hit is None:
                # 没找到目标：发"空目标帧"（目标数=0），
                # 让 FPGA 侧走 fail-safe，而不是收到一帧卡在旧数据上。
                frame = make_vision_result([])
                ok = send_frame(uart, frame)
                if DEBUG_PRINT:
                    print('#%-6d 无目标 -> 空目标帧 (%dB) %s'
                          % (frame_no, len(frame), hexdump(frame)))
            else:
                cls, blob, rule_name = hit
                if FORCE_CLASS is not None:
                    cls = FORCE_CLASS

                # ★★ 2026-11-03：FPGA 停车检查（S_CHECK）时，用遥测距离判障碍物高度
                #      <= 9cm -> 0x04 抬腿跨过； > 9cm -> 0x02 绕行
                _tel = rx_memo.get('last') or {}
                if _tel.get('state') == ST_CHECK:
                    _d = _tel.get('dist_mm', 0)
                    if DIST_JUDGE_MIN <= _d <= DIST_JUDGE_MAX:
                        _h = estimate_obst_height_mm(blob, _d)
                        if _h is not None:
                            if _h <= OBST_LIFT_MAX_MM:
                                # 低矮 -> 抬腿跨过（越障时长用 FPGA 的固定 1s）
                                cls = CLS_LOW_OBSTACLE
                                if DEBUG_PRINT:
                                    print('  [高度] 距离=%dmm 像素高=%d 估算=%dmm -> 抬腿'
                                          % (_d, blob.h(), _h))
                            else:
                                # 高 -> 绕行；顺带按实测宽度算出直行时长发给 FPGA
                                cls = CLS_OBSTACLE
                                _w  = estimate_obst_width_mm(blob, _d)
                                _rt = width_to_run_ms(_w)
                                try:
                                    send_frame(uart, make_config(
                                        0x23, bytes([(_rt >> 8) & 0xFF, _rt & 0xFF])))
                                except Exception as _e:
                                    if DEBUG_PRINT:
                                        print('  [绕行] 发 0x23 失败: %s' % _e)
                                if DEBUG_PRINT:
                                    print('  [高度] 距离=%dmm 估算=%dmm -> 绕行'
                                          ' 宽=%dmm 时长=%dms'
                                          % (_d, _h, _w, _rt))

                target, (px, py) = blob_to_target(img, blob, cls)
                frame = make_vision_result([target])
                ok = send_frame(uart, frame)    # ← 真正发出去的就是这一行

                # 在预览画面上画出结果，现场一眼就能看出跟踪对不对
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())

                if DEBUG_PRINT:
                    t_cls, cx, cy, conf = target
                    ratio = blob.pixels() / float(img.width() * img.height())
                    print('#%-6d %s 类别0x%02X 中心(%.0f,%.0f) 归一化(%d,%d) '
                          'x档=%-6s 占比%.2f%% 置信度%d%% 帧%dB'
                          % (frame_no, rule_name, t_cls, px, py, cx, cy,
                             x_zone(cx), ratio * 100.0, conf, len(frame)))
                    print('       帧: %s' % hexdump(frame))

            if ok:
                tx_ok += 1
            else:
                tx_err += 1

            # ---- 4. 顺手收 FPGA 的遥测 ----
            # 只用非阻塞读；所以这一拍最多多读一次缓冲区，10Hz 节奏完全不受影响。
            if USE_RX_TELEMETRY:
                rx_ok += rx_telemetry(uart, rx_parser, rx_memo)
                if DEBUG_PRINT:
                    _print_rx_stats_if_changed(rx_parser, rx_memo)

            # ---- 5. 可选：打印画面中心像素，帮忙估色 ----
            if PROBE_CENTER_PIXEL and (frame_no % TARGET_HZ == 0):
                try:
                    print('       中心像素 RGB = %s'
                          % (img.get_pixel(img.width() // 2, img.height() // 2)[:3],))
                except Exception as e:
                    print('       取像素失败: %s' % e)

            # ---- 6. 送预览 + 回收 + 卡节奏 ----
            if display_ready:
                Display.show_image(img)

            gc.collect()
            frame_no += 1
            win_n += 1
            win_t0, win_n = _print_fps_if_due(frame_no, win_t0, win_n)

            if (RUN_FRAMES > 0) and (frame_no >= RUN_FRAMES):
                print('RUN_FRAMES=%d 已跑满，主动退出' % RUN_FRAMES)
                break

            keep_10hz(t0)

    except KeyboardInterrupt:
        print('用户停止')
    except BaseException as e:
        # 板上不要抛一大串 traceback，打印出来方便现场看
        print('异常退出: %s' % e)
        sys.print_exception(e)
    finally:
        # 收尾顺序（和官方例程一致）：sensor -> Display -> MediaManager
        if sensor is not None:
            try:
                sensor.stop()
            except Exception as e:
                print('sensor.stop 失败: %s' % e)
        if display_ready:
            try:
                Display.deinit()
            except Exception as e:
                print('Display.deinit 失败: %s' % e)
        if media_ready:
            try:
                MediaManager.deinit()
            except Exception as e:
                print('MediaManager.deinit 失败: %s' % e)

        # 收尾统计：这一趟发了多少帧、收回多少遥测、接收侧有没有出错
        st = rx_parser.stats
        print('合计: 发出 VISION_RESULT %d 帧（失败 %d）/ 收到 TELEMETRY %d 帧'
              % (tx_ok, tx_err, rx_ok))
        print('接收侧: 好帧%d BCC错%d 帧尾错%d 版本错%d 丢弃%d 溢出%d 噪声%dB'
              % (st['frames_ok'], st['bcc_err'], st['tail_err'], st['bad_ver'],
                 st['dropped'], st['overflow'], st['noise_bytes']))

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        print('camera_vision 已退出')


#=============================================================================
# 入口
#=============================================================================
if __name__ == '__main__':
    os.exitpoint(os.EXITPOINT_ENABLE)   # 允许 IDE 打断（CanMV 惯例）
    main()
