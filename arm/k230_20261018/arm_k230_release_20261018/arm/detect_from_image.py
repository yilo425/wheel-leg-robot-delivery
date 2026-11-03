#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_from_image.py -- 本地图片颜色识别 -> uart_proto VISION_RESULT 帧

=============================================================================
为什么需要它
=============================================================================
ARM 板还没到货，先用一张本地图片把"识别 -> 组帧"这段在 PC 上跑通，
板子到货那天剩下的就只有摄像头取流这一环。

    图片 ──► OpenCV 颜色识别 ──► 归一化坐标 ──► make_vision_result() ──► 十六进制帧

=============================================================================
用法
=============================================================================
    python detect_from_image.py 图片.jpg            # 识别红块，打印帧
    python detect_from_image.py 图片.jpg --debug    # 额外画框并存图

=============================================================================
依赖
=============================================================================
    pip install opencv-python numpy
    （没装的话本脚本会给出明确提示，不会抛一堆 traceback）

=============================================================================
现场调参
=============================================================================
    颜色阈值（HSV 上下限）都放在文件顶部，比赛现场直接改常量即可。
    注意 OpenCV 的 H 范围是 0..179（不是 0..360）；红色跨 0 度，故分两段。
"""

import argparse
import os
import sys

#=============================================================================
# ① 现场可调常量：红色 HSV 阈值（OpenCV: H 0..179, S 0..255, V 0..255）
#=============================================================================
# 红色在色环上跨 0 度，需要"低段 + 高段"两段阈值再取并集。
# 现场偏暗 -> 调低 V 下限；偏橙 -> 收紧 H 上限；偏粉 -> 抬高 S 下限。
RED_HSV_LOW1  = (0,   100,  80)     # 红(低段) 下限 H,S,V
RED_HSV_HIGH1 = (10,  255, 255)     # 红(低段) 上限
RED_HSV_LOW2  = (170, 100,  80)     # 红(高段) 下限
RED_HSV_HIGH2 = (179, 255, 255)     # 红(高段) 上限

MIN_AREA_RATIO = 0.0005   # 面积占比 < 0.05% 的轮廓当噪声丢弃（可现场调）
KERNEL_SIZE    = 5        # 形态学去噪核大小（奇数；越大去噪越狠、边缘越糙）


#=============================================================================
# ② 依赖检查：没装 OpenCV / numpy 时给明确提示，绝不打印 traceback
#=============================================================================
try:
    import cv2
    import numpy as np
except ImportError as e:
    print('[错误] 缺少 OpenCV / numpy，无法做图像识别：')
    print(f'        实际错误: {e}')
    print('        安装命令: pip install opencv-python numpy')
    print('        国内镜像: pip install opencv-python numpy '
          '-i https://pypi.tuna.tsinghua.edu.cn/simple')
    sys.exit(1)

# 协议库（对外接口与 fpga_link.py 保持一致；本脚本不修改 uart_proto.py）
try:
    from uart_proto import (
        CLS_OBSTACLE, norm_to_u16, x_zone,
        make_vision_result, parse_vision_result, FrameParser,
    )
except ImportError:
    print('[错误] 找不到 uart_proto.py —— 请在本目录（arm/）下运行本脚本')
    sys.exit(1)

TARGET_CLASS = CLS_OBSTACLE           # 类别固定 = 0x02 障碍物
CLS_NAME = {CLS_OBSTACLE: '障碍物'}


#=============================================================================
# 颜色识别
#=============================================================================
def build_mask(image):
    """BGR 图 -> 红色二值掩码（形态学去噪后）。"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, RED_HSV_LOW1, RED_HSV_HIGH1)
    m2 = cv2.inRange(hsv, RED_HSV_LOW2, RED_HSV_HIGH2)
    mask = cv2.bitwise_or(m1, m2)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNEL_SIZE, KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # 去小噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # 补内部空洞
    return mask


#=============================================================================
# 找最大轮廓 / 算中心
#=============================================================================
def largest_contour(mask):
    """返回 (最大轮廓, 面积)；没有轮廓时返回 (None, 0.0)。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0
    c = max(contours, key=cv2.contourArea)
    return c, float(cv2.contourArea(c))


def contour_center(contour):
    """轮廓质心（像素）。不规则形状用矩比外接框中心更稳。"""
    M = cv2.moments(contour)
    if M['m00'] > 0:
        return M['m10'] / M['m00'], M['m01'] / M['m00']
    x, y, w, h = cv2.boundingRect(contour)   # 退化情况兜底
    return x + w / 2.0, y + h / 2.0


#=============================================================================
# 识别结果 -> uart_proto 目标
#=============================================================================
def image_to_target(image, contour, area):
    """把最大轮廓折算成一个 uart_proto 目标 (类别, cx, cy, 置信度)。

    - cx/cy 用 norm_to_u16() 归一化到 0..65535（绝不发像素坐标）
    - 置信度 = 轮廓面积占整幅画面的比例，折算到 0..100
    """
    h, w = image.shape[:2]
    px, py = contour_center(contour)

    cx = norm_to_u16(px, w)
    cy = norm_to_u16(py, h)

    conf = int(round(area / float(w * h) * 100.0))   # 面积占比 -> 0..100
    conf = max(0, min(100, conf))

    return (TARGET_CLASS, cx, cy, conf), (px, py)


#=============================================================================
# 调试输出：画框存图
#=============================================================================
def save_debug(image, contour, px, py, conf, out_path):
    """在图上画出识别结果（绿框 + 十字中心 + 文本）并保存。"""
    vis = image.copy()
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.drawMarker(vis, (int(round(px)), int(round(py))), (0, 255, 0),
                   cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis, f'{CLS_NAME[TARGET_CLASS]} conf={conf}%',
                (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2)

    cv2.imwrite(out_path, vis)
    return out_path


def hexdump(frame):
    """与 fpga_link.hexdump 同风格：大写、空格分隔。"""
    return ' '.join(f'{b:02X}' for b in frame)


#=============================================================================
# 主流程
#=============================================================================
def main():
    ap = argparse.ArgumentParser(
        description='本地图片颜色识别 -> uart_proto VISION_RESULT 帧（不需要 ARM 板）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', help='要识别的本地图片路径')
    ap.add_argument('--debug', action='store_true',
                    help='把识别结果画框并保存成图片')
    ap.add_argument('--out', help='--debug 输出路径（默认 debug_<图名>.png）')
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        print(f'[错误] 找不到图片: {args.image}')
        return 1

    image = cv2.imread(args.image)
    if image is None:
        print(f'[错误] 读图失败（格式不支持或文件损坏）: {args.image}')
        return 1

    h, w = image.shape[:2]
    print(f'图片 {args.image}   分辨率 {w}x{h}')

    mask = build_mask(image)
    contour, area = largest_contour(mask)

    # --- 没识别到目标：发一帧空目标，顺带验证 FPGA 的 fail-safe ---
    if contour is None:
        print('未识别到红色目标 -> 发送空目标帧（0 个）')
        frame = make_vision_result([])
        print(f'帧({len(frame)}B): {hexdump(frame)}')
        return 0

    ratio = area / float(w * h)
    if ratio < MIN_AREA_RATIO:
        print(f'最大轮廓面积占比 {ratio*100:.3f}% < 门限 '
              f'{MIN_AREA_RATIO*100:.3f}%，当噪声丢弃 -> 发送空目标帧')
        frame = make_vision_result([])
        print(f'帧({len(frame)}B): {hexdump(frame)}')
        return 0

    (cls, cx, cy, conf), (px, py) = image_to_target(image, contour, area)

    print()
    print('--- 识别结果 ---')
    print(f'    类别     0x{cls:02X}  ({CLS_NAME.get(cls, "?")})')
    print(f'    中心像素 ({px:.1f}, {py:.1f})')
    print(f'    归一化   cx={cx:5d}  cy={cy:5d}   (0..65535)')
    print(f'    x 档位   {x_zone(cx)}')
    print(f'    面积占比 {ratio*100:.2f}%  ->  置信度 {conf}%')

    frame = make_vision_result([(cls, cx, cy, conf)])
    print()
    print(f'--- VISION_RESULT 帧（{len(frame)} 字节 = 9 开销 + 1 目标数 + 7 目标）---')
    print(f'    {hexdump(frame)}')

    # 回环自检：确认解析出的坐标与发出的完全一致（与 FPGA 行为对齐）
    parser = FrameParser()
    got = parser.feed(frame)
    if got:
        print(f'    回环解析 {parse_vision_result(got[0][1])}')

    if args.debug:
        out_path = args.out or os.path.join(
            os.path.dirname(os.path.abspath(args.image)) or '.',
            f'debug_{os.path.splitext(os.path.basename(args.image))[0]}.png')
        save_debug(image, contour, px, py, conf, out_path)
        print()
        print(f'[debug] 已保存标注图: {out_path}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
