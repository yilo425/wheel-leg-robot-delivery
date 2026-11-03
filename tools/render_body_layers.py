#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 body_v4 的 lower / upper 两层 PNG 并排合成一张，加标题。"""
import os
from PIL import Image, ImageDraw, ImageFont

os.chdir(r'D:\dsh-ws\robot_project\mech')

lo = Image.open('_t_lower.png').convert('RGB')
up = Image.open('_t_upper.png').convert('RGB')

h = max(lo.height, up.height) + 70
w = lo.width + up.width + 40
c = Image.new('RGB', (w, h), (250, 250, 252))
c.paste(lo, (0, 70))
c.paste(up, (lo.width + 40, 70))

d = ImageDraw.Draw(c)
try:
    fb = ImageFont.truetype(r'C:\Windows\Fonts\msyhbd.ttc', 26)
except Exception:
    fb = ImageFont.load_default()
d.text((20, 16), u'下层 lower（电池区 | 驱动区 | 降压/端子区 + 固定桩×4 + 舵机穿墙）', fill=(20, 20, 30), font=fb)
d.text((lo.width + 60, 16), u'上层 upper（FPGA + K230 四角边夹）', fill=(20, 20, 30), font=fb)

c.save('body_v4_layers.png')
print('saved body_v4_layers.png', c.size)
