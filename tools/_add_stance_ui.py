# -*- coding: utf-8 -*-
"""遥控加"升高/降低"：
  1) wifi_rc.py：/cmd 支持 ht=0/1；每 2 秒重发 CONFIG 0x21（和手动模式一起）
  2) 页面：加两个按钮（升高/降低），点击发 /cmd?ht=..
"""
p = r'arm/wifi_rc.py'
s = open(p, encoding='utf-8').read()

#---------------------------------------------------------------------------
# 1) 状态变量（放在 steer/speed 附近）
#---------------------------------------------------------------------------
old = "    steer       = 0            # -100..100"
assert old in s, 'steer state anchor'
new = """    steer       = 0            # -100..100
    stance      = 0            # ★ 0=高位(0状态) 1=低位（CONFIG 0x21）"""
s = s.replace(old, new, 1)

#---------------------------------------------------------------------------
# 2) /cmd 里解析 ht
#---------------------------------------------------------------------------
old2 = """                    if q['sp'] is not None:
                        speed = max(0, min(SPEED_CAP, q['sp']))"""
assert old2 in s, 'sp anchor'
new2 = """                    if q['ht'] is not None:
                        # ★ 站立高度：0=升高到 0 状态，1=降低
                        stance = 1 if q['ht'] else 0
                        try:
                            uart.write(up.make_config(0x21, bytes([stance])))
                        except Exception as e:
                            _log('  [cmd] stance write failed: %r' % e)
                    if q['sp'] is not None:
                        speed = max(0, min(SPEED_CAP, q['sp']))"""
s = s.replace(old2, new2, 1)

#---------------------------------------------------------------------------
# 3) 每 2 秒重发 CONFIG：0x20（手动）+ 0x21（站立高度）
#---------------------------------------------------------------------------
old3 = """            if time.ticks_diff(time.ticks_ms(), t_manual) >= 2000:
                t_manual = time.ticks_ms()
                _send_manual_on(uart)"""
assert old3 in s, 'manual resend anchor'
new3 = """            if time.ticks_diff(time.ticks_ms(), t_manual) >= 2000:
                t_manual = time.ticks_ms()
                _send_manual_on(uart)
                # ★ 站立高度也要定期重发：FPGA 复位后要能恢复现场
                try:
                    uart.write(up.make_config(0x21, bytes([stance])))
                except Exception:
                    pass"""
s = s.replace(old3, new3, 1)

#---------------------------------------------------------------------------
# 4) 页面：加两个按钮 + JS
#---------------------------------------------------------------------------
old4 = """<div class="row"><span>速度</span>"""
assert old4 in s, 'page row anchor'
new4 = """<div class="row" style="margin-top:10px">
  <button id="hup" class="st">升高</button>
  <button id="hdn" class="st">降低</button>
</div>

<div class="row"><span>速度</span>"""
s = s.replace(old4, new4, 1)

# 样式：st 按钮矮一点
old5 = """ button.stop{background:#c0392b}"""
new5 = """ button.stop{background:#c0392b}
 button.st{height:52px;font-size:17px;background:#2f8f4f;width:110px}"""
s = s.replace(old5, new5, 1)

# JS：绑定
old6 = """elSp.oninput = function () { sp = +elSp.value; elSpv.textContent = sp; };"""
assert old6 in s, 'js anchor'
new6 = """elSp.oninput = function () { sp = +elSp.value; elSpv.textContent = sp; };

// ★ 站立高度：点一次发一帧 CONFIG 0x21（0=高位 1=低位）
//   固件内部靠 pos 饱和做限位：已在高位再点升高无效，反之亦然。
var ht = 0;
function sendHt() {
  fetch('/cmd?ht=' + ht + '&sq=' + (seq++ & 255), {cache: 'no-store'})
    .then(function (r) { return r.text(); })
    .then(function (t) {
      elSt.className = '';
      elSt.textContent = '站立高度 ' + (ht ? '低位' : '高位') + '  ' + t;
    })
    .catch(function () { elSt.className = 'bad'; elSt.textContent = '断开'; });
}
document.getElementById('hup').addEventListener('click', function () { ht = 0; sendHt(); });
document.getElementById('hdn').addEventListener('click', function () { ht = 1; sendHt(); });"""
s = s.replace(old6, new6, 1)

open(p, 'w', encoding='utf-8').write(s)

import ast
ast.parse(open(p, encoding='utf-8').read())
print('wifi_rc.py patched: ht 按钮 + CONFIG 0x21 定时重发')
print('PAGE 字节数:', len(s.split('PAGE = """')[1].split('"""')[0].encode('utf-8')))
