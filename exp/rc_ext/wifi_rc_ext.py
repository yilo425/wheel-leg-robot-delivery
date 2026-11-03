# -*- coding: utf-8 -*-
"""
wifi_rc_ext.py -- K230 遥控网关【扩展版】
                 手动/自动 + 前进/后退 + 左右转 90° + 停止 + 升降 + 服务端状态栏

=============================================================================
和基线 wifi_rc_baseline.py 的关系
=============================================================================
基线（2026-11-01 实机验证通过，36220 字节）只做了"按住走、松手停"的瞬时遥控。
本文件在**不动硬件、不动协议、不动接线**的前提下，把控制方式改成"点一下保持"，
并加上模式切换、定时转弯、升降和状态显示。

    手机浏览器  --WiFi-->  K230 wifi_rc_ext  --串口 3 根-->  FPGA rc_rx
    /h 纯链接页            手动 20Hz RC 帧 (0x05)            500ms 看门狗
    点一下=一次普通         2s 重发 CONFIG  (0x04)
    页面跳转

=============================================================================
功能（对应 README 第三节的 6 条）
=============================================================================
  1 手动 / 自动   手动 = CONFIG 0x20=1（本机按状态 20Hz 发 RC 帧）
                  自动 = CONFIG 0x20=0（FPGA 接管，本机**一帧 RC 都不发**）
  2 前进 / 后退   手动下点一次就一直走，直到点【停止】（状态存在服务端）
  3 左右转 90°    steer=±100 + speed=40%，定时 TURN_MS=360ms 后自动停
  4 停止          steer=0 speed=0，同时清掉保持状态
  5 升降          CONFIG 0x21：0=高位（0 状态） 1=低位
  6 状态栏        服务端拼字符串渲染：模式 / 动作 / 站立 / 速度 / 发帧

=============================================================================
★ 为什么遥控状态放在【服务端】（基线是放在 JS 里的）
=============================================================================
"点一下就一直走"没法用基线的"按住轮询"实现：手机锁屏/切后台 -> JS 定时器
被节流甚至冻结 -> 帧断了 -> 车停了（用户看到的是"点一下只走了一小段"）；
页面一刷新，JS 里的状态也没了。所以扩展版把"想让它干什么"变成服务端的一块
状态：/cmd 只改状态并回 302，主循环按状态 20Hz 发帧。页面因此能**零 JS**
（README 第 6 条），也顺手绕开"手机浏览器不执行内联 script"那个坑；
状态栏显示什么 = 板子真正在干什么，不存在两套状态打架。

=============================================================================
★ 安全设计（任何一层失效都会停车）
=============================================================================
  1) FPGA 侧 rc_rx.v：500ms 收不到 RC 帧 -> rc_active=0 -> 停车
  2) 本脚本：切模式 / 停车都会**立刻补发一帧 steer=0 speed=0**，不指望"下一帧自然停"
  3) 自动模式下本机完全不发 RC 帧，不跟 FPGA 状态机抢控制
  4) 主循环崩掉 -> 没有帧 -> FPGA 的 500ms 看门狗兜底停车

  ⚠️ 代价（有意为之，见 CHANGELOG）：为满足"点一下就保持"，基线那层"600ms
     没收到 /cmd 就本地停车"的软保护**默认关闭**（CMD_TIMEOUT_MS = 0）。手机
     掉线时车会保持当前动作继续走，要停只能重连点【停止】或复位板子；
     想恢复旧行为把 CMD_TIMEOUT_MS 改成 600，其余逻辑一行都不用动。

=============================================================================
接线 / 怎么跑
=============================================================================
    K230 GPIO05（排针 11）--TX--> FPGA F16 (uart_rx)
    K230 GPIO06（排针 13）--RX<-- FPGA C19 (uart_tx)   本脚本不收，可只接 2 根
    GND 共地

    import wifi_rc_ext
    wifi_rc_ext.run()              # 开热点 + HTTP 服务 + 开始发帧
    wifi_rc_ext.dry_run()          # 不接硬件，只看帧长什么样（PC 上也能跑）

    手机连热点 RobotRC / 12345678 -> 浏览器打开 http://<板子IP>:8080/h

=============================================================================
上板前必读：README 第一节 12 条，本文件的对应位置（逐项自检见 CHANGELOG）
=============================================================================
   1 LF 换行 / 2 _send_all 纯切片   文件级；_send_all 与基线逐字一致
   3 整段 try / 4 必回响应          _handle_cmd + _apply_cmd；302 失败退 200
   5 查询串无白名单                 _parse_query 任意键都收
   6 零 JS / 7 & 写成 &amp;          全是 <a href>，统一由 _btn() 生成
   8 DEBUG_HTTP=False               热路径不写 SD 卡
   9 run() 先 gc.collect            函数体第一件事
  10 不自测 / 11 绑定与端口         只用手机验；SO_REUSEADDR + 8080 + listen(8)
  12 空闲预连接                    空请求立刻 close + return
"""

import time

#-----------------------------------------------------------------------------
# 底层发送：必须循环发到全部发完
#
#   MicroPython 的 sock.send() 可能只发一部分就返回（lwip 发送窗口）。
#   不循环 -> HTML 被截断 -> 手机上表现为"只有几个按钮，下面一片黑"。
#   这段和基线**逐字一致**，实机验证过，别动；用的是纯 bytes 切片
#   （data[total:]），不是零拷贝视图对象，也不是 bytearray 缓冲区。
#-----------------------------------------------------------------------------
def _send_all(sock, data):
    """循环发送直到全部发完（MicroPython 的 send 可能只发一部分）。"""
    if isinstance(data, str):
        data = data.encode('utf-8')
    total = 0
    n = len(data)
    while total < n:
        try:
            k = sock.send(data[total:])
        except Exception:
            break
        if not k:
            break
        total += k
    return total


def _log(msg):
    """拔了 USB 就没有控制台，所以关键步骤写文件（开机自查看 /sdcard/rc_boot.txt）。"""
    for path in ('/sdcard/rc_boot.txt', '/tmp/rc_boot.txt'):
        try:
            with open(path, 'a') as f:
                f.write(str(msg) + '\n')
            return
        except Exception:
            continue


#-----------------------------------------------------------------------------
# HTTP 热路径的日志开关（必须 False）
#
#   往 /sdcard 写一次日志 10~50ms（SD 卡随机写很慢），一个请求写 2~4 次 ->
#   单请求 100~200ms -> 浏览器轮询必然超时 -> 页面闪烁。
#   只在排查 HTTP 问题时临时打开。
#-----------------------------------------------------------------------------
DEBUG_HTTP = False


def _loghttp(msg):
    if DEBUG_HTTP:
        _log(msg)


#=============================================================================
# 一、可调参数
#=============================================================================
# 版本号：每次改动加一。启动时写进 rc_boot.txt，用来确认板上跑的是哪一版
# （以前靠猜"到底烧的是哪一版"，浪费过一晚上）
VERSION     = '2026-11-01-r10-ext'

WIFI_SSID   = 'RobotRC'
WIFI_PASS   = '12345678'      # 至少 8 位；改成 '' 就是开放热点
HTTP_PORT   = 8080            # ★ 80 端口非 root 绑不上（实测 OSError EPERM）
AP_IP       = '192.168.4.1'   # ⚠️ 只是兜底猜测；实测真实值是 192.168.169.1，
                              #    代码里一律用 ap.ifconfig()[0] 取实际 IP

SEND_HZ     = 20              # 发帧频率（FPGA 看门狗 500ms，20Hz 允许连丢 9 帧）
SPEED_CAP   = 100             # 遥控速度上限（%）
MIN_SPEED   = 10              # 速度下限：0% 的"前进"没有意义，容易误判成毛病
DEFAULT_SPEED = 40            # 开机默认速度（%），和转弯速度一致

#-----------------------------------------------------------------------------
# ★ 定时转弯 90° 的 360ms 是怎么来的（README 给了结论，这里记推导）
#
#   轮距 W = 230mm -> 原地转向半径 R = W/2 = 115mm；40% 油门实测 v = 502.7mm/s
#   转 90° 轮子要走的弧长 = (pi/2)*R = 1.5708*115 = 180.6mm
#   时间 t = 180.6 / 502.7 = 0.3593 s -> 取 360ms
#
#   ⚠️ 精度：帧率只有 20Hz（50ms 一帧），主循环还要顺带处理 HTTP，所以实际
#      转角有 -0 ~ +50ms 的量化误差（约 0~13°）。要更准就把 SEND_HZ 提到 50。
#-----------------------------------------------------------------------------
TURN_MS     = 360             # 定时转弯持续时间（ms）
TURN_SPEED  = 40              # 转弯固定速度（%），README 指定 40%

RESEND_MS   = 2000            # CONFIG（模式 + 站立高度）重发周期

#-----------------------------------------------------------------------------
# ★ 软超时：默认关闭（0 = 不启用）
#
#   README 第三节要求"点一下就一直走，直到点停止"，所以不能用基线的 600ms
#   无指令自动停。想让车在手机掉线时自己停，把它改成 600 即可。
#-----------------------------------------------------------------------------
CMD_TIMEOUT_MS = 0

# 开机默认模式：0 = 自动（最安全 —— 上电就跟 FPGA 的默认状态一致，
# 本机一帧 RC 都不发，绝不跟 FPGA 状态机抢控制）。
# 想让开机就直接能遥控，改成 1（手动）。
BOOT_MODE   = 0

MODE_AUTO   = 0               # CONFIG 0x20 = 0
MODE_MAN    = 1               # CONFIG 0x20 = 1

CFG_MODE    = 0x20            # 手动/自动
CFG_STANCE  = 0x21            # 站立高度：0=高位 1=低位

# 动作常量（用字符串而不是魔数，状态栏直接查表显示）
ACT_STOP    = 'stop'
ACT_FWD     = 'fwd'
ACT_BACK    = 'back'
ACT_LEFT    = 'left'
ACT_RIGHT   = 'right'

ACT_TEXT = {
    ACT_STOP:  '停止',
    ACT_FWD:   '前进',
    ACT_BACK:  '后退',
    ACT_LEFT:  '左转 90°',
    ACT_RIGHT: '右转 90°',
}

# /cmd?a=0..4 的整数映射（页面链接用整数，比中文参数短、也不怕编码）
ACT_BY_INT = {
    0: ACT_STOP,
    1: ACT_FWD,
    2: ACT_BACK,
    3: ACT_LEFT,
    4: ACT_RIGHT,
}


#=============================================================================
# 二、网页（服务端渲染，零 JS）
#
#   README 第 6 条：手机浏览器（夸克/一加）可能根本不执行页面里的内联 script，
#   所以 /h 是纯 HTML：按钮全是 <a href="/cmd?...">，状态全部由 Python 拼进去。
#   点一下 = 一次普通页面跳转，没有第二个状态源。
#   README 第 7 条：裸的 & 会被当成实体开头，必须写 &amp; —— 本文件的链接全部
#   由 _btn() 生成，href 里只有 &amp;（CHANGELOG 自检第 5 项会扫一遍）。
#=============================================================================
PAGE_HEAD = """<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>轮足机器人 · 遥控</title>
<style>
 body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:12px;
      -webkit-tap-highlight-color:transparent}
 h3{margin:2px 0 10px;font-weight:400;font-size:17px;text-align:center;color:#ddd}
 .bar{background:#1b1b1b;border:1px solid #333;border-radius:10px;padding:6px 10px;
      margin-bottom:8px}
 .r{display:flex;justify-content:space-between;font-size:15px;line-height:1.75}
 .r .k{color:#888}
 .r .v{font-family:monospace;color:#8f8;font-weight:bold}
 .r .v.warn{color:#fc6}
 .msg{font-size:12px;color:#fc6;min-height:16px;margin:0 2px 8px;line-height:1.5}
 .sec{font-size:12px;color:#777;margin:12px 2px 6px}
 .g{display:grid;grid-gap:8px;margin:0 0 4px}
 .g2{grid-template-columns:1fr 1fr}
 .g3{grid-template-columns:1fr 1fr 1fr}
 a.b{display:block;text-align:center;text-decoration:none;border-radius:12px;
     background:#2a6df4;color:#fff;font-size:18px;padding:15px 0;
     border:2px solid transparent}
 a.b:active{filter:brightness(1.35)}
 a.b.on{border-color:#ffd23f}
 a.b.red{background:#c0392b}
 a.b.green{background:#2f8f4f}
 a.b.gray{background:#3a3a3a}
 a.b.dim{background:#242a2e;color:#7c8a90}
 a.b.sm{font-size:16px;padding:11px 0}
 .h{color:#888;font-size:12px;line-height:1.8;margin-top:14px}
 .h a{color:#6cf}
</style></head><body>
"""


def _esc(s):
    """极简 HTML 转义（状态文字是我们自己拼的，但别留裸 & 和尖括号）。"""
    try:
        s = str(s)
    except Exception:
        return ''
    s = s.replace('&', '&amp;')
    s = s.replace('<', '&lt;')
    s = s.replace('>', '&gt;')
    return s


def _row(k, v, extra=''):
    """状态栏一行（服务端渲染）。"""
    cls = 'v' + ((' ' + extra) if extra else '')
    return '<div class="r"><span class="k">%s</span><span class="%s">%s</span></div>\n' % (k, cls, v)


def _cls(on, extra=''):
    """拼按钮 class：'b' + 颜色/尺寸 + 激活高亮（只改边框色，不动背景色）。"""
    c = 'b'
    if extra:
        c = c + ' ' + extra
    if on:
        c = c + ' on'
    return c


def _btn(label, href, cls):
    """生成一个按钮链接。href 里的 & 必须由调用方写成 &amp;。"""
    return '<a class="%s" href="%s">%s</a>\n' % (cls, href, label)


def _page_h(st):
    """把当前状态渲染成 /h 页面的完整 HTML（无 JS、无外部资源）。"""
    mode = st['mode']
    act = st['action']
    stance = st['stance']
    sp = st['sp']

    manual = (mode == MODE_MAN)
    # 自动模式下运动按钮画灰（但还是能点），点了会在状态栏看到提示
    dim = '' if manual else ' dim'

    p = [PAGE_HEAD]
    p.append('<h3>轮足机器人 · 遥控</h3>\n')

    #---- 状态栏（服务端渲染）----
    p.append('<div class="bar">\n')
    p.append(_row('模式', '手动' if manual else '自动', '' if manual else 'warn'))
    p.append(_row('动作', ACT_TEXT.get(act, '停止'), ''))
    p.append(_row('站立', '低位' if stance else '高位', ''))
    p.append(_row('速度', '%d%%' % sp, ''))
    p.append(_row('发帧', ('%dHz 遥控帧' % SEND_HZ) if manual else '停发', '' if manual else 'warn'))
    p.append('</div>\n')
    p.append('<div class="msg">%s</div>\n' % _esc(st['msg']))

    #---- 模式 ----
    p.append('<div class="sec">模式</div><div class="g g2">\n')
    p.append(_btn('手动', '/cmd?m=1&amp;ref=1', _cls(manual)))
    p.append(_btn('自动', '/cmd?m=0&amp;ref=1', _cls(not manual)))
    p.append('</div>\n')

    #---- 动作 ----
    p.append('<div class="sec">动作（手动模式有效；点一次保持）</div><div class="g g3">\n')
    p.append(_btn('前进', '/cmd?a=1&amp;ref=1', _cls(act == ACT_FWD, 'green' + dim)))
    p.append(_btn('停止', '/cmd?a=0&amp;ref=1', _cls(act == ACT_STOP, 'red')))
    p.append(_btn('后退', '/cmd?a=2&amp;ref=1', _cls(act == ACT_BACK, 'green' + dim)))
    p.append('</div>\n<div class="g g2">\n')
    p.append(_btn('左转 90°', '/cmd?a=3&amp;ref=1', _cls(act == ACT_LEFT, 'gray' + dim)))
    p.append(_btn('右转 90°', '/cmd?a=4&amp;ref=1', _cls(act == ACT_RIGHT, 'gray' + dim)))
    p.append('</div>\n')

    #---- 站立高度 ----
    p.append('<div class="sec">站立高度</div><div class="g g2">\n')
    p.append(_btn('升高（高位）', '/cmd?ht=0&amp;ref=1', _cls(stance == 0, 'green')))
    p.append(_btn('降低（低位）', '/cmd?ht=1&amp;ref=1', _cls(stance == 1, 'green')))
    p.append('</div>\n')

    #---- 速度 ----
    p.append('<div class="sec">速度（前进/后退用；转弯固定 %d%%）</div><div class="g g3">\n' % TURN_SPEED)
    for v in (40, 60, 80):
        p.append(_btn('%d%%' % v, '/cmd?sp=%d&amp;ref=1' % v, _cls(sp == v, 'gray sm')))
    p.append('</div>\n')

    #---- 说明 ----
    turn_note = ('定时 %d ms 后自动停（轮距 230mm，R=115mm；40%% 时 v≈502.7mm/s，'
                 '90° 弧长 180.6mm，t≈359ms）' % TURN_MS)
    p.append('<div class="h">\n')
    p.append('  ● 前进 / 后退：点一次会一直走，必须点【停止】才停。<br>\n')
    p.append('  ● 左转 / 右转：' + turn_note + '。<br>\n')
    p.append('  ● 自动模式下本机不发 RC 帧，运动按钮无效（站立 / 速度照常可设）。<br>\n')
    p.append('  ● 本页零 JS，点完自动跳回这里刷新状态。<br>\n')
    p.append('  <a href="/h">刷新状态</a> · <a href="/ver">版本 / 诊断</a>\n')
    p.append('</div>\n</body></html>')
    return ''.join(p)


#=============================================================================
# 三、UART
#=============================================================================
def _setup_uart():
    """自己把 GPIO5/6 复用成 UART2 @115200（不依赖 camera_vision）。

    ★ 自包含的原因：camera_vision 在 import 阶段就会拉 media.sensor /
      media.display，不初始化摄像头时可能直接卡住（实测 rc_boot.txt 停在
      starting run() 且无异常）。遥控不需要摄像头。
    """
    _log('  [uart] begin')
    try:
        from machine import UART, FPIOA
    except Exception as e:
        _log('  [uart] import machine FAILED: %r' % e)
        print('[错误] 没有 machine 模块: %s' % e)
        return None

    try:
        fpioa = FPIOA()
        fpioa.set_function(5, FPIOA.UART2_TXD)   # GPIO5 -> 排针 11
        fpioa.set_function(6, FPIOA.UART2_RXD)   # GPIO6 -> 排针 13
        _log('  [uart] fpioa OK')
        uart = UART(UART.UART2, baudrate=115200,
                    bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                    stop=UART.STOPBITS_ONE, timeout=0)
        _log('  [uart] UART2 就绪 @115200')
        print('[信息] UART2 就绪: GPIO5(TX)/GPIO6(RX) @115200')
        return uart
    except Exception as e:
        _log('  [uart] init EXCEPTION: %r' % e)
        print('[错误] UART 初始化失败: %s' % e)
        return None


#=============================================================================
# 四、WiFi 热点
#=============================================================================
def _setup_ap():
    try:
        import network
    except Exception:
        print('[错误] 这个固件没有 network 模块（没开 WiFi）')
        return None
    try:
        ap = network.WLAN(network.AP_IF)
        try:
            ap.active(True)
        except Exception:
            pass

        #---------------------------------------------------------
        # ★ 幂等：热点已经起来（有 IP）就沿用，不重新 config。
        #   重新 config 会让 AP 重启 -> 已连着的手机被踢下线。
        #---------------------------------------------------------
        try:
            cur_ip = ap.ifconfig()[0]
        except Exception:
            cur_ip = ''
        if cur_ip and cur_ip != '0.0.0.0':
            _log('  [ap] already up, reuse ip=%s' % cur_ip)
            print('[信息] 热点已在运行，沿用现有配置: %s' % cur_ip)
            return ap

        #---------------------------------------------------------
        # ★ 开机自启时驱动可能还没好：等它给出 IP（最多 15 秒）
        #---------------------------------------------------------
        for _i in range(30):
            try:
                cur_ip = ap.ifconfig()[0]
            except Exception:
                cur_ip = ''
            if cur_ip and cur_ip != '0.0.0.0':
                _log('  [ap] driver ready after %d ms, ip=%s' % (_i * 500, cur_ip))
                break
            time.sleep_ms(500)
        else:
            _log('  [ap] WARN: driver gave no ip in 15s, will try config anyway')

        # 不同固件参数名略有差异，逐个兜底
        for _try in range(3):
            try:
                ap.config(ssid=WIFI_SSID, key=WIFI_PASS)
            except Exception:
                try:
                    ap.config(essid=WIFI_SSID, password=WIFI_PASS)
                except Exception as e:
                    _log('  [ap] config try %d failed: %r' % (_try + 1, e))
            time.sleep_ms(800)
            try:
                ip_now = ap.ifconfig()[0]
            except Exception:
                ip_now = ''
            _log('  [ap] after config try %d: ip=%r' % (_try + 1, ip_now))
            if ip_now and ip_now != '0.0.0.0':
                break

        for _ in range(30):
            if ap.active():
                break
            time.sleep_ms(100)
        _log('  [ap] active, ssid=%s' % WIFI_SSID)
        print('[信息] 热点: %s  密码: %s' % (WIFI_SSID, WIFI_PASS or '(无)'))
        try:
            print('[信息] 板子 IP: %s' % ap.ifconfig()[0])
        except Exception:
            print('[信息] 板子 IP: %s（猜的）' % AP_IP)
        return ap
    except Exception as e:
        print('[错误] 开热点失败: %s' % e)
        return None


#=============================================================================
# 五、查询串解析
#=============================================================================
def _parse_query(path):
    """从 '/cmd?m=1&a=1&ref=1' 里取参数，返回 dict。

    ★ 没有键名白名单（README 第 5 条）！基线之前写成
      out = {'st':None,'sp':None,'sq':None}，后来加的 ht/ref 被静默丢弃 ->
      q.get('ref') 永远 None -> 302 从未生效 -> 点完停在白底小字页。
      现在任意键名都保留，能转整数就转，不行就原样存字符串。
    """
    out = {}
    if '?' not in path:
        return out
    q = path.split('?', 1)[1]
    for kv in q.split('&'):
        if '=' not in kv:
            continue
        k, v = kv.split('=', 1)
        try:
            out[k] = int(v)
        except Exception:
            out[k] = v
    return out


#=============================================================================
# 六、协议帧：RC（0x05）与 CONFIG（0x04）
#=============================================================================
#-----------------------------------------------------------------------------
# ★★ 关键坑：不能把带 bit7 的字节直接塞给 up.make_rc_cmd()
#
#   README 表格写的是 make_rc_cmd(steer, speed_byte, seq)，但 uart_proto 里有
#   `if sp > 100: sp = 100` 的钳位，speed_byte = 0x80|40 = 168 会被钳成 100，
#   bit7 直接消失。实测（CPython + 本目录 uart_proto.py）：
#       make_rc_cmd(0, 0x80|40, 7)          -> 载荷 00 64 07   ← bit7 没了
#       pack_frame(0x05, bytes([0,0xA8,7])) -> 载荷 00 A8 07   ← 正确
#   后果：按【后退】会变成 100% 满速**前进**。所以这里自己组 3 字节载荷，再用
#   uart_proto.pack_frame 打包（帧头/BCC/帧尾仍全部由 uart_proto 生成，协议
#   实现只有一份）。pack_frame 不可用时才退回 make_rc_cmd（丢后退标志但不崩）。
#-----------------------------------------------------------------------------
def _rc_frame(up, steer, speed_byte, seq):
    """组一帧 RC_CMD。speed_byte 的 bit7 = 后退标志，bit6:0 = 速度百分比。"""
    try:
        si = int(steer)
    except Exception:
        si = 0
    if si > 100:
        si = 100
    if si < -100:
        si = -100
    try:
        sb = int(speed_byte) & 0xFF
    except Exception:
        sb = 0
    try:
        sq = int(seq) & 0xFF
    except Exception:
        sq = 0
    try:
        # int8 用 & 0xFF 折成补码：-100 -> 0x9C，和 struct '>b' 完全一致
        return up.pack_frame(up.TYPE_RC_CMD, bytes([si & 0xFF, sb, sq]))
    except Exception as e:
        _log('  [rc] pack_frame FAILED: %r，退回 make_rc_cmd' % e)
        return up.make_rc_cmd(si, sb & 0x7F, sq)


def _send_cfg(uart, up, pid, val):
    """发一帧 CONFIG（0x20 模式 / 0x21 站立高度）。失败只记日志，不抛。"""
    try:
        uart.write(up.make_config(pid, bytes([int(val) & 0xFF])))
        return 1
    except Exception as e:
        _log('  [cfg] write FAILED pid=0x%02X %r' % (pid, e))
        return 0


def _rc_values(st):
    """由当前状态算出 (steer, speed_byte)。这是"状态 -> 帧"的唯一出口。

    后退不是靠负速度，而是靠 speed 字节的 bit7（README 第二节）；
    0x80|mag 里 mag 保持 0..100，FPGA 侧再拆开。
    """
    a = st['action']
    sp = st['sp']
    if a == ACT_FWD:
        return 0, sp & 0x7F
    if a == ACT_BACK:
        return 0, 0x80 | (sp & 0x7F)
    if a == ACT_LEFT:
        return -100, TURN_SPEED & 0x7F
    if a == ACT_RIGHT:
        return 100, TURN_SPEED & 0x7F
    return 0, 0


def _write_rc(st, uart, up, now):
    """立刻发一帧当前状态的 RC 帧（发失败也只记日志）。"""
    steer, spb = _rc_values(st)
    st['seq'] = (st['seq'] + 1) & 0xFF
    try:
        uart.write(_rc_frame(up, steer, spb, st['seq']))
    except Exception as e:
        # ★ 以前这里只 print，拔了 USB 根本看不到；卡里的 uart_proto 缺
        #   make_rc_cmd 时每帧都静默失败 -> 一帧都没出去。
        _log('  [rc] uart write FAILED: %r' % e)
        print('[警告] 串口写失败: %s' % e)
    return steer, spb


#=============================================================================
# 七、状态机（服务端保持状态）
#=============================================================================
def _new_state():
    """初始状态：自动模式 + 停止 + 高位 + 40%。"""
    now = time.ticks_ms()
    return {
        'mode': BOOT_MODE,          # 0 自动 / 1 手动
        'action': ACT_STOP,         # 当前动作（点一下保持的就是它）
        'stance': 0,                # 0 高位 / 1 低位
        'sp': DEFAULT_SPEED,        # 速度百分比
        'seq': 0,                   # RC 帧序号（丢帧诊断用）
        'turn_t0': 0,               # 定时转弯起点（ticks_ms）
        't_send': now,              # 上次发 RC 帧的时刻
        't_cfg': now,               # 上次重发 CONFIG 的时刻
        't_cmd': now,               # 上次收到 /cmd 的时刻
        'period': max(1, 1000 // SEND_HZ),
        'msg': '就绪：默认自动模式，点【手动】接管',
    }


def _mode_val(v):
    """把 m= 参数换成 MODE_*；认不出来返回 None（不抛异常）。"""
    try:
        iv = int(v)
        return MODE_MAN if iv else MODE_AUTO
    except Exception:
        pass
    try:
        s = str(v).strip().lower()
    except Exception:
        return None
    if s in ('man', 'manual', 'm', 'hand', '手动'):
        return MODE_MAN
    if s in ('auto', 'automatic', 'a', '自动'):
        return MODE_AUTO
    return None


def _act_val(v):
    """把 a= 参数换成动作常量；认不出来返回 None（不抛异常）。"""
    try:
        iv = int(v)
    except Exception:
        iv = -1
    if iv >= 0:
        return ACT_BY_INT.get(iv)
    try:
        s = str(v).strip().lower()
    except Exception:
        return None
    if s in ('stop', 's', '停', '停止'):
        return ACT_STOP
    if s in ('fwd', 'forward', 'f', '前进'):
        return ACT_FWD
    if s in ('back', 'b', 'rev', 'backward', '后退'):
        return ACT_BACK
    if s in ('left', 'l', '左转'):
        return ACT_LEFT
    if s in ('right', 'r', '右转'):
        return ACT_RIGHT
    return None


def _set_action(st, avi, now, uart, up):
    """设置动作。★ 设完立刻发一帧 —— 点【停止】不能等下一个 50ms tick。"""
    st['action'] = avi
    if avi == ACT_LEFT or avi == ACT_RIGHT:
        st['turn_t0'] = now        # 定时起点，主循环到点自动停
        st['msg'] = '%s 已下发（%dms 后自动停）' % (ACT_TEXT.get(avi, ''), TURN_MS)
    elif avi == ACT_STOP:
        st['turn_t0'] = 0
        st['msg'] = '已停止（steer=0 speed=0）'
    else:
        st['turn_t0'] = 0
        st['msg'] = '%s中 —— 点【停止】才停' % ACT_TEXT.get(avi, '')
    _write_rc(st, uart, up, now)
    st['t_send'] = now


def _set_mode(st, uart, up, mvi, now):
    """切模式。★ 切之前一定先停车，别把上一个动作带进新模式。"""
    was = st['action']
    st['action'] = ACT_STOP
    st['turn_t0'] = 0
    if mvi == MODE_MAN:
        # 连发两帧：单帧丢了就等于没切，用户会以为"点了没反应"
        _send_cfg(uart, up, CFG_MODE, 1)
        _send_cfg(uart, up, CFG_MODE, 1)
        st['mode'] = MODE_MAN
        st['msg'] = '已切到手动：遥控生效（%dHz 发帧）' % SEND_HZ
    else:
        # 切自动之前先把"停"发下去，否则 FPGA 接管那一瞬间车还在按上一帧走
        _write_rc(st, uart, up, now)
        _send_cfg(uart, up, CFG_MODE, 0)
        _send_cfg(uart, up, CFG_MODE, 0)
        st['mode'] = MODE_AUTO
        st['msg'] = '已切到自动：本机停发 RC 帧，FPGA 接管'
    st['t_send'] = now
    st['t_cfg'] = now
    _log('  [mode] -> %d (prev action=%s)' % (mvi, was))
    return st['mode']


def _apply_cmd(q, st, uart, up, now):
    """把一条 /cmd 的参数应用到状态上，返回状态栏提示文字。

    每个参数单独 try/except 转 int，写错一个不能连累整条；键名无白名单。
    """
    #---- 站立高度：ht=0 高位 / ht=1 低位（CONFIG 0x21）----
    hv = q.get('ht')
    if hv is not None:
        try:
            hvi = 1 if int(hv) else 0
        except Exception:
            hvi = st['stance']
        st['stance'] = hvi
        _send_cfg(uart, up, CFG_STANCE, hvi)
        st['msg'] = '站立 -> %s' % ('低位' if hvi else '高位')

    #---- 速度：sp=MIN_SPEED..SPEED_CAP ----
    sv = q.get('sp')
    if sv is not None:
        try:
            svi = int(sv)
        except Exception:
            svi = st['sp']
        if svi < MIN_SPEED:
            svi = MIN_SPEED
        if svi > SPEED_CAP:
            svi = SPEED_CAP
        st['sp'] = svi
        st['msg'] = '速度 -> %d%%' % svi

    #---- 模式：m=1 手动 / m=0 自动 ----
    mv = q.get('m')
    if mv is not None:
        mvi = _mode_val(mv)
        if mvi is None:
            st['msg'] = '模式参数无效: %s' % mv
        elif mvi != st['mode']:
            _set_mode(st, uart, up, mvi, now)
        else:
            # 重复点同一个模式：重发一次 CONFIG 防丢帧，但不改状态
            _send_cfg(uart, up, CFG_MODE, mvi)
            st['msg'] = '模式不变：%s' % ('手动' if mvi == MODE_MAN else '自动')

    #---- 动作：a=0 停 1 前进 2 后退 3 左转 4 右转 ----
    av = q.get('a')
    if av is not None:
        avi = _act_val(av)
        if avi is None:
            st['msg'] = '动作参数无效: %s' % av
        elif st['mode'] != MODE_MAN:
            # ★ 自动模式下本机不发 RC 帧，所以运动指令一律不接。
            #   不静默忽略 —— 状态栏要说明白，否则用户以为坏了。
            st['msg'] = '自动模式：动作无效，请先点【手动】'
        else:
            _set_action(st, avi, now, uart, up)

    st['t_cmd'] = now
    return st['msg']


def _tick(st, uart, up, now):
    """主循环的周期性工作（不含 HTTP）。返回 1 表示这一轮发了 RC 帧。

    顺序有讲究：软超时 -> 定时转弯到点（立刻补停帧）-> 每 2s 重发 CONFIG
    -> 最后才按需发 20Hz RC 帧（**只在手动模式**）。
    """
    #---- 1) 软超时（CMD_TIMEOUT_MS = 0 时不启用）----
    if CMD_TIMEOUT_MS > 0:
        if time.ticks_diff(now, st['t_cmd']) > CMD_TIMEOUT_MS:
            if st['action'] != ACT_STOP:
                st['action'] = ACT_STOP
                st['turn_t0'] = 0
                st['msg'] = '超时未收到指令，已停车'
                _write_rc(st, uart, up, now)
                st['t_send'] = now

    #---- 2) 定时转弯到点 -> 自动停 ----
    if st['action'] == ACT_LEFT or st['action'] == ACT_RIGHT:
        if time.ticks_diff(now, st['turn_t0']) >= TURN_MS:
            _done = ACT_TEXT.get(st['action'], '转弯')
            st['action'] = ACT_STOP
            st['turn_t0'] = 0
            st['msg'] = '%s 到点（%dms），已自动停' % (_done, TURN_MS)
            # ★ 立刻补一帧停：别等下一个 50ms tick，否则会多转 1~2°
            _write_rc(st, uart, up, now)
            st['t_send'] = now

    #---- 3) 每 2 秒重发模式 + 站立高度（防丢帧 / 防 FPGA 复位丢现场）----
    if time.ticks_diff(now, st['t_cfg']) >= RESEND_MS:
        st['t_cfg'] = now
        _send_cfg(uart, up, CFG_MODE, st['mode'])
        _send_cfg(uart, up, CFG_STANCE, st['stance'])

    #---- 4) 20Hz RC 帧：★ 只在手动模式发 ----
    if st['mode'] == MODE_MAN:
        if time.ticks_diff(now, st['t_send']) >= st['period']:
            st['t_send'] = now
            _write_rc(st, uart, up, now)
            return 1
    return 0


#=============================================================================
# 八、HTTP
#=============================================================================
def _send_302(cli):
    """回 302 跳回 /h（体验好：点完就回到控制台，状态栏已是新状态）。"""
    try:
        hdr = ('HTTP/1.1 302 Found\r\nLocation: /h\r\n'
               'Cache-Control: no-store\r\n'
               'Content-Length: 0\r\nConnection: close\r\n\r\n')
        _send_all(cli, hdr.encode())
        return 1
    except Exception as e:
        print('[cmd] 302 failed', e)
        return 0


def _send_resp(cli, ctype, body):
    """回一段带 Content-Length 的响应（头部只在这一处拼，别的地方都走它）。"""
    hdr = ('HTTP/1.1 200 OK\r\nContent-Type: %s\r\n'
           'Cache-Control: no-store\r\n'
           'Content-Length: %d\r\nConnection: close\r\n\r\n' % (ctype, len(body)))
    try:
        _send_all(cli, hdr.encode())
        _send_all(cli, body)
    except Exception as e:
        _loghttp('  [http] resp send FAILED: %r' % e)


def _send_page(cli, html):
    """回一个 HTML 页面。

    ★ Content-Length 必须是**字节数**，不是字符数。页面里有中文（一个汉字
      UTF-8 占 3 字节），用字符数会少报 -> 浏览器按声明长度截断 -> 页面残缺。
    """
    try:
        body = html.encode('utf-8')
    except Exception:
        body = b'<html><body>encode error</body></html>'
    _send_resp(cli, 'text/html; charset=utf-8', body)


def _handle_cmd(cli, path, st, uart, up):
    """处理 /cmd：整段 try/except，且**无论如何都要回响应**（README 第 3/4 条）。

    绝不能让它抛异常 —— 一抛异常连接就被掐断，浏览器显示"网页无法打开"。
    """
    try:
        q = _parse_query(path)
        _apply_cmd(q, st, uart, up, time.ticks_ms())
    except Exception as e:
        print('[cmd] ERROR', e)
        _log('  [cmd] ERROR %r' % e)
        try:
            st['msg'] = '参数错误: %s' % e
        except Exception:
            pass

    # 成功：302 跳回 /h
    if _send_302(cli):
        return
    # 保底：302 都没发出去，也得回点什么，不能让浏览器空等
    try:
        body = b'<a href="/h">OK</a>'
        hdr = ('HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n'
               'Cache-Control: no-store\r\nContent-Length: %d\r\n'
               'Connection: close\r\n\r\n' % len(body))
        _send_all(cli, hdr.encode())
        _send_all(cli, body)
    except Exception:
        pass


def _send_ver(cli, st):
    """/ver 探针：确认板上跑的是哪一版、当前状态是什么（纯文本）。"""
    try:
        _p = __file__
    except Exception:
        _p = '?'
    try:
        nbytes = len(_page_h(st).encode('utf-8'))
    except Exception:
        nbytes = -1
    txt = ('VER=%s\nPAGE_BYTES=%d\nFILE=%s\n'
           'MODE=%d\nACTION=%s\nSTANCE=%d\nSP=%d\n'
           'TURN_MS=%d\nSEND_HZ=%d\nCMD_TIMEOUT_MS=%d\nRESEND_MS=%d\n'
           % (VERSION, nbytes, _p, st['mode'], st['action'], st['stance'],
              st['sp'], TURN_MS, SEND_HZ, CMD_TIMEOUT_MS, RESEND_MS))
    _send_resp(cli, 'text/plain; charset=utf-8', txt.encode('utf-8'))


def _http_one(srv, st, uart, up):
    """accept 一条连接、处理一个请求。无论出什么事都不往主循环抛异常。"""
    try:
        cli, addr = srv.accept()
    except Exception:
        return 0

    try:
        # ★ 浏览器会开 4~6 条**只连接不发数据**的空闲预连接（README 第 12 条）。
        #   超时压到 20ms —— 请求行是连上就发，20ms 足够；
        #   超时越短，越不容易被空闲连接堵住（那会让 /cmd 看起来"点了没反应"）。
        cli.settimeout(0.02)
        req = b''
        try:
            req = cli.recv(512)
        except Exception:
            req = b''

        if not req:
            # ★ 空请求立刻关掉，绝不能给它回页面（回一次就堵死一条轮询）
            _loghttp('  [http] empty request -> drop')
            return 0

        line = req.split(b'\r\n', 1)[0].decode('utf-8', 'ignore')
        # "GET /cmd?m=1&a=1&ref=1 HTTP/1.1"
        path = ''
        parts = line.split(' ')
        if len(parts) >= 2:
            path = parts[1]

        # 去掉查询串再判断路径，这样 /h?x=1 这种缓存穿透写法也能出页面
        path_only = path.split('?', 1)[0]
        _loghttp('  [http] path=%r' % path)

        if path_only == '/cmd':
            _handle_cmd(cli, path, st, uart, up)
        elif path_only in ('/h', '/html'):
            _send_page(cli, _page_h(st))
        elif path_only == '/ver':
            _send_ver(cli, st)
        elif path_only in ('/', '', '/index.html'):
            # ★ 基线在这里回的是"按住走"的 JS 页面。扩展版**故意**改成跳 /h：
            #   两套控制方式会互相打架（JS 页松手就发 sp=0，会立刻打断服务端
            #   刚设好的"一直走"）。只留一个入口，状态才不会有两份。
            _send_302(cli)
        else:
            # /favicon.ico 之类：回 404，别再回整页
            try:
                _send_all(cli, b'HTTP/1.1 404 Not Found\r\n'
                              b'Content-Length: 0\r\nConnection: close\r\n\r\n')
            except Exception:
                pass
        return 1
    except Exception as e:
        print('[警告] HTTP 处理异常: %s' % e)
        _log('  [http] EXCEPTION %r' % e)
        return 0
    finally:
        try:
            cli.close()
        except Exception:
            pass


#=============================================================================
# 九、主程序
#=============================================================================
def run(port=HTTP_PORT):
    #--------------------------------------------------------------
    # ★ 函数体第一件事：强制 GC
    #   上一次 run() 被 Ctrl-C 打断时，8080 的 socket 可能还没被回收
    #   （MicroPython 不会立刻 close），再 bind 就 OSError EPERM，
    #   而且这个状态有时要物理断电才清得掉。先 collect 收掉僵尸 socket。
    #--------------------------------------------------------------
    try:
        import gc
        gc.collect()
        _free = gc.mem_free()
    except Exception:
        _free = -1
    _log('  [run] enter VERSION=%s free=%d' % (VERSION, _free))
    print('[信息] wifi_rc_ext 版本 %s' % VERSION)

    # 开机自启时 main.py 跑得很早，WiFi 驱动可能还没就绪（此时 config 会静默
    # 失败，热点根本不广播）。REPL 里手动调没事，所以留 3 秒给驱动初始化。
    _log('  [run] wait 3s for wifi driver')
    time.sleep_ms(3000)

    uart = _setup_uart()
    if uart is None:
        _log('  [run] abort: uart is None')
        return
    _log('  [run] uart OK')

    ap = _setup_ap()
    if ap is None:
        _log('  [run] abort: ap is None')
        return
    _log('  [run] ap OK')

    try:
        import socket
        _log('  [run] socket OK')
    except Exception as e:
        _log('  [run] socket FAILED: %r' % e)
        print('[错误] 没有 socket 模块')
        return

    _log('  [run] import uart_proto begin')
    import uart_proto as up
    _log('  [run] uart_proto OK')

    _log('  [run] bind begin')
    srv = socket.socket()
    try:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    try:
        srv.bind(('0.0.0.0', port))
    except Exception as e:
        _log('  [run] bind FAILED on :%d -> %r' % (port, e))
        print('[错误] 绑定端口 %d 失败: %s（80 需要 root，改用 8080）' % (port, e))
        return
    srv.listen(8)                # README 第 11 条：listen(8)（浏览器会开多条预连接）
    srv.settimeout(0.005)        # 5ms：日志已挪出热路径，轮询越短越及时

    #--------------------------------------------------------------
    # 上电把模式 + 站立高度各发两遍，让 FPGA 和本机状态对齐。
    # 默认 BOOT_MODE=0（自动）= 上电就是安全状态，一帧 RC 都不发。
    #--------------------------------------------------------------
    st = _new_state()
    _send_cfg(uart, up, CFG_MODE, st['mode'])
    _send_cfg(uart, up, CFG_MODE, st['mode'])
    _send_cfg(uart, up, CFG_STANCE, st['stance'])
    _send_cfg(uart, up, CFG_STANCE, st['stance'])
    st['t_cfg'] = time.ticks_ms()
    print('[信息] 上电状态: 模式=%s 站立=%s 速度=%d%%'
          % ('手动' if st['mode'] == MODE_MAN else '自动',
             '低位' if st['stance'] else '高位', st['sp']))

    # 用真实 IP（不同固件默认网段不同，实测是 192.168.169.1）
    try:
        real_ip = ap.ifconfig()[0]
    except Exception:
        real_ip = AP_IP
    _log('  [run] HTTP listening on :%d ip=%s' % (port, real_ip))
    print('[信息] HTTP 监听 :%d，手机打开 http://%s:%d/h' % (port, real_ip, port))
    print('[信息] 发帧 %dHz（仅手动），转弯 %dms，CONFIG 每 %dms 重发'
          % (SEND_HZ, TURN_MS, RESEND_MS))

    _log('  [run] ENTER MAIN LOOP (热点应该已经可见了)')
    while True:
        #---- 1) 收 HTTP（非阻塞，5ms 一轮）----
        try:
            _http_one(srv, st, uart, up)
        except Exception as e:
            # 双保险：_http_one 自己已经兜住了，这里是最后一道门
            print('[警告] http 外层异常: %s' % e)

        #---- 2) 状态机 + 发帧 ----
        #   ★ 转弯定时、CONFIG 重发、20Hz RC 帧都在 _tick 里，
        #     和 HTTP 解耦，方便在 PC 上用假时钟做行为自检。
        try:
            _tick(st, uart, up, time.ticks_ms())
        except Exception as e:
            _log('  [tick] EXCEPTION %r' % e)
            print('[警告] tick 异常: %s' % e)
            time.sleep_ms(5)


#=============================================================================
# 十、dry-run：不接硬件，只打印帧（PC 上也能跑）
#=============================================================================
def dry_run():
    """打印每种动作对应的帧，并校验后退标志（bit7）没有被钳掉。"""
    import uart_proto as up
    print('=== wifi_rc_ext dry-run ===')
    print('VERSION = %s' % VERSION)

    st = _new_state()
    st['sp'] = DEFAULT_SPEED
    bad = 0
    for name in (ACT_STOP, ACT_FWD, ACT_BACK, ACT_LEFT, ACT_RIGHT):
        st['action'] = name
        steer, spb = _rc_values(st)
        f = _rc_frame(up, steer, spb, 1)
        hexs = ' '.join('%02X' % b for b in f)
        print('  %-6s steer=%4d speed_byte=0x%02X len=%d -> %s'
              % (name, steer, spb, len(f), hexs))
        if len(f) != 12:
            print('    [警告] 帧长应为 12，实际 %d' % len(f))
            bad = bad + 1
        if spb != (f[7] if len(f) > 7 else -1):
            print('    [警告] speed 字节被改了：期望 0x%02X，帧里是 0x%02X'
                  % (spb, f[7]))
            bad = bad + 1

    print('  CONFIG 0x20=1 -> %s' % ' '.join('%02X' % b for b in up.make_config(CFG_MODE, bytes([1]))))
    print('  CONFIG 0x20=0 -> %s' % ' '.join('%02X' % b for b in up.make_config(CFG_MODE, bytes([0]))))
    print('  CONFIG 0x21=0 -> %s' % ' '.join('%02X' % b for b in up.make_config(CFG_STANCE, bytes([0]))))
    print('  CONFIG 0x21=1 -> %s' % ' '.join('%02X' % b for b in up.make_config(CFG_STANCE, bytes([1]))))
    print('  转弯 TURN_MS=%d ms @ %d%%（20Hz 下发 -> 实际 -0 ~ +50ms）'
          % (TURN_MS, TURN_SPEED))
    print('  帧长应恒为 12 字节（9 固定开销 + 3 载荷）')
    if bad == 0:
        print('  [OK] 所有动作帧结构正确，后退 bit7 保留')
    else:
        print('  [FAIL] %d 处异常' % bad)
    return bad


if __name__ == '__main__':
    dry_run()
