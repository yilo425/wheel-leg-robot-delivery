#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_from_image.py -- 从一张图片做颜色识别，输出 VISION_RESULT 帧

=============================================================================
这是干什么的
=============================================================================
板子还没到货时，用**本地图片**先把"识别 → 转成协议帧"这一段跑通。

    test.jpg ──> OpenCV 颜色识别 ──> 目标列表 ──> make_vision_result() ──> 帧字节
    
有了它，等摄像头到货时，只要把"读图片"换成"读摄像头帧"就行了。

=============================================================================
用法
=============================================================================
    # 生成一张测试图（不需要摄像头，也不需要 OpenCV）
    python detect_from_image.py --make-test-image

    # 识别图片并打印协议帧
    python detect_from_image.py --image test.jpg

    # 输出带标注的调试图（现场调阈值时用）
    python detect_from_image.py --image test.jpg --debug out.jpg

    # 指定只找某种颜色
    python detect_from_image.py --image test.jpg --color red

=============================================================================
调阈值（★ 比赛现场最重要的事）
=============================================================================
    HSV 阈值写在文件顶部的 COLOR_RANGES 里。
    现场光照和实验室完全不同，**必须在比赛场地重新调一遍**。

    调法：先用 --debug 输出标注图，看框得准不准；
    不准就改 COLOR_RANGES 里的 (H_low, S_low, V_low) 和 (H_high, S_high, V_high)。

    H（色相）: 0~179（OpenCV 的范围，不是 0~360）
    S（饱和）: 0~255，太低会把灰白也算进来
    V（亮度）: 0~255，太低会把阴影也算进来

    经验值：先固定 S_low=100, V_low=100，只调 H 的范围。

=============================================================================
依赖
=============================================================================
    pip install opencv-python numpy

    没装 OpenCV 也能跑 --make-test-image 和 --demo-frame（纯 Python 写图 / 造帧）
"""

import argparse
import os
import sys
#-----------------------------------------------------------------------------
# Windows 控制台编码兜底
#
#   中文 Windows 的控制台默认代码页是 GBK(CP936)，而脚本里会打印
#   [OK] [X] → ★ 这类符号 —— GBK 里没有，直接抛
#       UnicodeEncodeError: 'gbk' codec can't encode character '\u2713'
#   实测：不设 PYTHONIOENCODING=utf-8 时，fpga_link.py --dry-run 和
#         detect_from_image.py --demo-frame 都会崩在第一个 [OK] 上。
#
#   这里主动把 stdout/stderr 切到 UTF-8，让脚本自己不依赖外部环境变量。
#   （Python 3.7+ 支持 reconfigure；更老的版本忽略即可，不会报错）
#-----------------------------------------------------------------------------
import sys as _sys
#   两类问题要分开处理：
#     ① 崩溃：打印 GBK 表示不了的字符会抛 UnicodeEncodeError
#        -> 用 errors='replace' 兜底，最差显示成 '?'，但**不会崩**
#     ② 乱码：如果强行把 stdout 切成 UTF-8，而控制台仍按 GBK 解码，
#        中文会全部变乱码 —— 比崩溃更糟，因为看起来"程序正常但输出是垃圾"
#        所以**保持控制台原生编码**，中文才能正确显示；
#        代价是非 GBK 符号（[OK] [X] [!] <-> > <）要换成 ASCII。
#        本文件的 print 输出里已经全部换掉了。
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(errors='replace')
    except Exception:
        pass
# 协议库在本目录
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uart_proto import (
    make_vision_result, norm_to_u16, x_zone,
    CLS_PERSON, CLS_OBSTACLE, CLS_GOAL, CLS_LOW_OBSTACLE,
)


#=============================================================================
# ★ 颜色阈值 —— 比赛现场必须重新调
#=============================================================================
# OpenCV 的 HSV：H 0~179, S 0~255, V 0~255
# 每种颜色可以有多个区间（红色在 HSV 里跨越 0，所以要两段）
COLOR_RANGES = {
    'red': [
        ((0,   100, 100), (10,  255, 255)),
        ((170, 100, 100), (180, 255, 255)),
    ],
    'green': [
        ((40,  80,  80),  (85,  255, 255)),
    ],
    'blue': [
        ((95,  100, 100), (130, 255, 255)),
    ],
    'yellow': [
        ((20,  100, 100), (35,  255, 255)),
    ],
}

# 颜色 -> 协议类别码
#   FPGA 侧 main_fsm 只认 0x00~0x04，映射关系要和 A 商量好
CLASS_FOR_COLOR = {
    'red':    CLS_LOW_OBSTACLE,   # 红色视为"低矮障碍" -> 触发升降腿
    'green':  CLS_GOAL,           # 绿色视为"目标点"     -> 向目标行进
    'blue':   CLS_OBSTACLE,       # 蓝色视为"障碍物"     -> 绕行
    'yellow': CLS_PERSON,         # 黄色视为"人员"       -> 减速
}

# 最小轮廓面积（像素）。太小的斑点直接忽略，避免噪声。
#   现场调参：如果误识别多就调大，如果远处目标识别不到就调小。
MIN_CONTOUR_AREA = 300

# 置信度折算：用"轮廓面积 / 图像面积"映射到 0~100
#   面积占比 CONF_AREA_FULL 时算满分
CONF_AREA_FULL = 0.10


#=============================================================================
# 生成测试图（不需要 OpenCV）
#=============================================================================
def make_test_image(path: str = 'test.jpg', w: int = 640, h: int = 480) -> int:
    """造一张带红/绿/蓝三个色块的测试图。

    优先用 Pillow（如果有）；没有就写一个**未压缩的 BMP**（纯 Python 就能写）。
    """
    # 色块： (颜色名, 中心 x 比例, 中心 y 比例, 半径比例)
    blobs = [
        ('red',    0.20, 0.50, 0.08),
        ('green',  0.50, 0.40, 0.10),
        ('blue',   0.80, 0.60, 0.07),
    ]
    rgb = {'red': (200, 30, 30), 'green': (30, 200, 30), 'blue': (30, 30, 200)}
    bg = (240, 240, 240)

    try:
        from PIL import Image, ImageDraw
        img = Image.new('RGB', (w, h), bg)
        d = ImageDraw.Draw(img)
        for name, fx, fy, fr in blobs:
            cx, cy, r = int(fx * w), int(fy * h), int(fr * min(w, h))
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=rgb[name])
        img.save(path)
        print(f'已生成测试图: {path}  ({w}x{h}, Pillow)')
        return 0
    except ImportError:
        pass

    # 纯 Python 写 24 位 BMP（不依赖任何库）
    row_pad = (4 - (w * 3) % 4) % 4
    row_size = w * 3 + row_pad
    data_size = row_size * h
    header = bytearray()
    header += b'BM'
    header += (14 + 40 + data_size).to_bytes(4, 'little')
    header += (0).to_bytes(4, 'little')
    header += (54).to_bytes(4, 'little')
    header += (40).to_bytes(4, 'little')
    header += w.to_bytes(4, 'little')
    header += h.to_bytes(4, 'little')
    header += (1).to_bytes(2, 'little')
    header += (24).to_bytes(2, 'little')
    header += (0).to_bytes(4, 'little')
    header += data_size.to_bytes(4, 'little')
    header += (2835).to_bytes(4, 'little')
    header += (2835).to_bytes(4, 'little')
    header += (0).to_bytes(4, 'little')
    header += (0).to_bytes(4, 'little')

    body = bytearray()
    for y in range(h - 1, -1, -1):          # BMP 是自下而上
        row = bytearray()
        for x in range(w):
            c = bg
            for name, fx, fy, fr in blobs:
                cx, cy, r = fx * w, fy * h, fr * min(w, h)
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    c = rgb[name]
                    break
            row += bytes([c[2], c[1], c[0]])   # BGR
        row += b'\x00' * row_pad
        body += row

    out = path.rsplit('.', 1)[0] + '.bmp'
    with open(out, 'wb') as f:
        f.write(header)
        f.write(body)
    print(f'已生成测试图: {out}  ({w}x{h}, 纯 Python BMP)')
    print('  （没装 Pillow，所以存的是 BMP —— 识别时用它即可）')
    return 0


#=============================================================================
# 颜色识别
#=============================================================================
def detect_colors(image_path: str, colors=None, debug_out: str = None) -> list:
    """返回 [(类别, cx归一化, cy归一化, 置信度), ...]"""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print('[错误] 需要 OpenCV 和 numpy：')
        print('         pip install opencv-python numpy')
        print('')
        print('       如果暂时不想装，可以先跑这两个不需要依赖的模式：')
        print('         python detect_from_image.py --make-test-image')
        print('         python detect_from_image.py --demo-frame')
        return None

    img = cv2.imread(image_path)
    if img is None:
        print(f'[错误] 读不到图片: {image_path}')
        print('       支持 jpg / png / bmp。路径含中文一般没问题，')
        print('       但如果是网络路径或路径太长可能失败。')
        return None

    h, w = img.shape[:2]
    print(f'图片: {image_path}  {w}x{h}')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    targets = []
    names = colors if colors else list(COLOR_RANGES.keys())

    for name in names:
        ranges = COLOR_RANGES.get(name)
        if not ranges:
            print(f'  [跳过] 未定义的颜色: {name}')
            continue

        # 多段区间合并
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # 形态学去噪：先腐蚀掉小噪点，再膨胀补回形状
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue

        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(c)
        cx_px, cy_px = x + bw // 2, y + bh // 2
        conf = min(100, int(area / (w * h) / CONF_AREA_FULL * 100))

        cls = CLASS_FOR_COLOR.get(name, CLS_OBSTACLE)
        targets.append((cls, norm_to_u16(cx_px, w), norm_to_u16(cy_px, h), conf))
        print(f'  {name:8s} 像素=({cx_px:4d},{cy_px:4d})  '
              f'归一化=({targets[-1][1]:5d},{targets[-1][2]:5d})  '
              f'档位={x_zone(targets[-1][1]):6s}  面积={int(area):6d}  置信度={conf}%')

        if debug_out:
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
            cv2.circle(img, (cx_px, cy_px), 4, (0, 0, 255), -1)
            cv2.putText(img, f'{name} {conf}%', (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if not targets:
        print('  没有识别到任何色块')
        print('  → 调 COLOR_RANGES 的阈值；现场光照和实验室不同，必须重调')

    if debug_out:
        cv2.imwrite(debug_out, img)
        print(f'  标注图已存: {debug_out}')

    return targets


#=============================================================================
# 演示：不需要图片、不需要 OpenCV
#=============================================================================
def demo_frame():
    """演示"识别结果 -> 协议帧"这一段（纯 Python，不依赖任何库）。"""
    print('=' * 68)
    print('  演示：目标列表 -> VISION_RESULT 帧')
    print('  （这一步和"图从哪来"无关，摄像头到货后完全复用）')
    print('=' * 68)

    cases = [
        ('识别到 1 个绿色目标点', [(CLS_GOAL, 32768, 20000, 85)]),
        ('识别到 1 个红色低矮障碍（触发升降腿）',
         [(CLS_LOW_OBSTACLE, 30000, 35000, 72)]),
        ('识别到障碍物偏右（FPGA 应左转绕行）',
         [(CLS_OBSTACLE, 55000, 24000, 90)]),
        ('没识别到东西（合法，FPGA 会限速直行）', []),
    ]

    for desc, tg in cases:
        frame = make_vision_result(tg)
        n = (frame[4] << 8) | frame[5]
        print(f'\n--- {desc} ---')
        print(f'    帧长 {len(frame)} = 9 + {n}   '
              f'{"[OK]" if len(frame) == 9 + n else "[X]"}')
        print(f'    {" ".join(f"{b:02X}" for b in frame)}')
        if tg:
            cls, cx, cy, conf = tg[0]
            print(f'    FPGA 侧会判为: x 档位 = {x_zone(cx)}')

    print('\n下一步（板子到货后）:')
    print('  把 detect_colors() 里的 cv2.imread(图片) 换成摄像头取帧，')
    print('  剩下的 make_vision_result() 一行都不用改。')
    return 0


#=============================================================================
def main():
    ap = argparse.ArgumentParser(
        description='从图片做颜色识别，输出 FPGA 能收的 VISION_RESULT 帧',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', help='要识别的图片路径')
    ap.add_argument('--color', action='append',
                    choices=list(COLOR_RANGES.keys()),
                    help='只找指定颜色（可重复），默认全找')
    ap.add_argument('--debug', metavar='OUT', help='输出带标注的调试图')
    ap.add_argument('--make-test-image', action='store_true',
                    help='生成一张测试图（不需要 OpenCV）')
    ap.add_argument('--demo-frame', action='store_true',
                    help='演示"目标 -> 帧"（不需要图片和 OpenCV）')
    ap.add_argument('--send-hex', action='store_true',
                    help='把帧以十六进制打印，方便手工灌给 FPGA')
    args = ap.parse_args()

    if args.make_test_image:
        return make_test_image()
    if args.demo_frame or not args.image:
        return demo_frame()

    targets = detect_colors(args.image, args.color, args.debug)
    if targets is None:
        return 1

    frame = make_vision_result(targets)
    n = (frame[4] << 8) | frame[5]
    print(f'\n帧: {len(frame)} 字节 = 9 + {n} 载荷')
    print(f'    {" ".join(f"{b:02X}" for b in frame)}')

    if args.send_hex:
        print('\n可直接灌给 FPGA（例如用串口助手按 HEX 发送）：')
        print('   ' + ''.join(f'{b:02X}' for b in frame))

    return 0


if __name__ == '__main__':
    sys.exit(main())
