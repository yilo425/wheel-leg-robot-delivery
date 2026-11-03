#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
k230_csi_test.py —— K230（立创·天猛星 / CanMV）CSI 摄像头自检

用法：
  1. 用 CanMV IDE 或 VS Code 连上 K230
  2. 把这个文件的内容整段粘进 CanMV IDE 的编辑区
  3. 点运行（Run）
  4. 把下面那几行 "=== 结果 ===" 的打印全部贴回来

它做什么：
  ① 先试 CSI1（你现在插的口），失败自动退回 CSI0
  ② 打印固件信息、实际分辨率
  ③ 拍一帧，算平均亮度 —— **判断"有没有画面"**（黑的 = 没出图）
  ④ 存一张 /sd/csi_test.jpg 方便你肉眼看
  ⑤ 最后打印 PASS / FAIL
"""

import sys
import time

# ---------------- 尽量兼容不同固件版本的导入 ----------------
try:
    from media.sensor import *
    from media.display import *
    from media.media import *
    HAVE_MEDIA = True
except Exception as e:
    HAVE_MEDIA = False
    print("[警告] media 模块导入失败:", e)


def try_port(kw_name, port, w=640, h=480):
    """按不同的参数名尝试打开 CSI 口（id= / csi= / port=）"""
    kwargs = {kw_name: port, "width": w, "height": h}
    try:
        s = Sensor(**kwargs)
        return s, None
    except Exception as e:
        return None, e


def probe():
    print("=== 环境 ===")
    print("MicroPython:", sys.implementation)
    try:
        import os
        print("SD 卡挂载:", os.listdir("/sd")[:6] if "sd" in os.listdir("/") else "无 /sd")
    except Exception:
        pass

    # ① 试三种参数名 × 两个口
    combos = []
    for kw in ("id", "csi", "port"):
        for port in (1, 0):
            combos.append((kw, port))

    sensor = None
    used = None
    errs = []
    for (kw, port) in combos:
        s, e = try_port(kw, port)
        if s is not None:
            sensor = s
            used = (kw, port)
            print("=== 打开成功: Sensor(%s=%d) ===" % (kw, port))
            break
        else:
            errs.append("%s=%d -> %s" % (kw, port, e))

    if sensor is None:
        print("=== 结果: FAIL —— 所有参数组合都打不开 ===")
        for e in errs:
            print("   ", e)
        return

    # ② 初始化媒体 + 拍一帧
    try:
        sensor.reset()
        sensor.set_framesize(width=640, height=480)
        try:
            sensor.set_pixformat(Sensor.RGB565)
        except Exception:
            pass
        if HAVE_MEDIA:
            media.init()
        sensor.run()
        time.sleep(0.6)
        img = sensor.snapshot()

        print("=== 结果: 打开成功 ===")
        print("   参数:", used)
        print("   分辨率:", img.width(), "x", img.height())

        # ③ 平均亮度（黑的 = 没出图 / 镜头盖没开 / 没对焦）
        try:
            st = img.get_statistics()
            lum = st.l_mean()
            print("   平均亮度: %.1f  (0~255)" % lum)
            if lum < 3:
                print("   >>> 画面全黑：可能是镜头盖没开 / 排线没插好 / 口不对")
            else:
                print("   >>> 有画面 ✓")
        except Exception as e:
            print("   亮度统计失败:", e)

        # ④ 存图（方便肉眼看）
        try:
            img.save("/sdcard/csi_test.jpg")
            print("   已存 /sdcard/csi_test.jpg（拔卡插电脑上看）")
        except Exception as e:
            print("   存图失败:", e)

        sensor.stop()
        print("=== 结论: PASS（CSI%s 能用）===" % used[1])

    except Exception as e:
        print("=== 结果: FAIL —— 能打开但出图失败 ===")
        print("   错误:", e)
    finally:
        try:
            if HAVE_MEDIA:
                media.deinit()
        except Exception:
            pass


probe()
