# -*- coding: utf-8 -*-
"""
wifi_rc.py -- K230 当"遥控网关"：手机连 K230 的热点 -> 网页摇杆 -> UART 发给 FPGA

=============================================================================
为什么用这个方案
=============================================================================
FPGA 只剩 1 个空闲引脚（E17），装不下硬件 RC 接收机（2~3 路 PWM）。
而 K230 本来就要开机（跑视觉），它又有 WiFi —— 让它顺手当遥控网关，
**不占任何额外 FPGA 引脚**，指令走已有的那 3 根串口线。

       手机浏览器                K230                      FPGA
    ┌──────────────┐   WiFi   ┌──────────┐   串口 3 根    ┌──────┐
    │ 网页摇杆      ├─────────►│ wifi_rc  ├───────────────►│rc_rx │
    │ (20Hz 轮询)   │          │ 20Hz 发帧 │  0x05 RC_CMD   │ 500ms│
    └──────────────┘          └──────────┘                │看门狗 │
                                                          └──────┘

=============================================================================
接线（和 camera_vision.py 完全一致）
=============================================================================
    K230 GPIO05（排针 11）--TX--> FPGA F16 (uart_rx)
    K230 GPIO06（排针 13）--RX<-- FPGA C19 (uart_tx)   ← 本脚本不收，可以只接 2 根
    GND 共地

=============================================================================
怎么跑
=============================================================================
    # 板子上（CanMV IDE 的"运行"里）
    import wifi_rc
    wifi_rc.run()                 # 开热点 + 开网页 + 开始发帧

    # 不接硬件，先看帧长什么样（PC 上也能跑）
    import wifi_rc
    wifi_rc.dry_run()

=============================================================================
⚠️ 安全设计（三重保护，任何一层失效都会停车）
=============================================================================
    1) FPGA 侧 rc_rx.v：500ms 收不到帧 -> rc_active=0 -> 停车
    2) 本脚本：600ms 没收到 /cmd -> 本地 speed 归 0（手机页面一关就停）
    3) 手动模式必须被 CONFIG 帧 0x20 打开；默认是自动模式，遥控不生效
    另外：speed 上限建议别给满（见 SPEED_CAP）

=============================================================================
⚠️ 本文件尚未在真机跑过（写于 2026-10-28 深夜，板子在实验室）
   第一次跑请先 dry_run() 看帧，再 run() 看手机能不能连上热点。
=============================================================================
"""

import time

#-----------------------------------------------------------------------------
# 日志：拔了 USB 就没有控制台，所以每一步都写文件
#
#   为什么不用 print：print 走 stdout，USB 一拔就什么都看不到。
#   这里统一写 /sdcard/rc_boot.txt（和 main.py 用的是同一个文件），
#   插回电脑当 U 盘打开就能看走到哪一步。
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
    for path in ('/sdcard/rc_boot.txt', '/tmp/rc_boot.txt'):
        try:
            with open(path, 'a') as f:
                f.write(str(msg) + '\n')
            return
        except Exception:
            continue


#-----------------------------------------------------------------------------
# ★ 2026-10-30：HTTP 热路径的日志开关
#
#   为什么默认关：往 /sdcard 写一次日志要 10~50ms（SD 卡随机写很慢），
#   而每个 HTTP 请求原来要写 2~4 次 -> 单请求 100~200ms。
#   手机是 10Hz 轮询（100ms 一次），服务端一慢，fetch 就超时，
#   页面就显示"断开"并闪烁 —— 昨天看到的现象就是这个。
#
#   排查 HTTP 问题时才打开（会明显变慢，但能看清每一步）。
#-----------------------------------------------------------------------------
DEBUG_HTTP = False


def _loghttp(msg):
    if DEBUG_HTTP:
        _log(msg)

#-----------------------------------------------------------------------------
# 可调参数
#-----------------------------------------------------------------------------
# 版本号：每次改动加一。启动时写进 rc_boot.txt，
# 用来确认板上跑的到底是哪一版（避免再靠猜）
VERSION     = '2026-10-30-r9'

WIFI_SSID   = 'RobotRC'
WIFI_PASS   = '12345678'      # 至少 8 位；改成 '' 就是开放热点
HTTP_PORT   = 8080            # ★ 80 端口非 root 绑不上（实测 OSError EPERM），用 8080
                              #   手机访问地址：http://<板子IP>:8080/
AP_IP       = '192.168.4.1'   # ⚠️ 只是兜底猜测；真实值实测是 192.168.169.1，
                              #    代码里一律用 ap.ifconfig()[0] 取得的实际 IP

SEND_HZ     = 20              # 发帧频率（FPGA 看门狗 500ms，20Hz 允许连续丢 9 帧）
SPEED_CAP   = 100              # ★ 遥控速度上限（%）；自动模式满速是 100，遥控建议压到 60
CMD_TIMEOUT_MS = 600          # 这么久没收到手机指令就本地停车

#-----------------------------------------------------------------------------
# 网页（手机打开 http://192.168.4.1/ ）
#-----------------------------------------------------------------------------
PAGE = """<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>轮足机器人 · 遥控</title>
<style>
 body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:14px;
      display:flex;flex-direction:column;align-items:center;user-select:none;
      -webkit-user-select:none;-webkit-tap-highlight-color:transparent}
 h3{margin:4px 0 10px;font-weight:400}
 .pad{display:grid;grid-template-columns:repeat(3,84px);grid-gap:8px}
 button{height:84px;font-size:24px;border-radius:14px;border:0;
        background:#2a6df4;color:#fff;touch-action:none;padding:0}
 button.stop{background:#c0392b}
 button.st{height:52px;font-size:17px;background:#2f8f4f;width:110px}
 button:active{transform:scale(.94);filter:brightness(1.3)}
 .row{display:flex;gap:10px;margin-top:14px;align-items:center}
 input[type=range]{width:180px}
 #st{font-family:monospace;font-size:13px;margin-top:10px;color:#8f8;min-height:18px}
 #st.bad{color:#f66}
 .hint{color:#888;font-size:12px;margin-top:8px;text-align:center;line-height:1.6}
</style></head><body>

<h3>轮足机器人 · 遥控</h3>

<div class="pad">
  <div></div><button id="f">▲</button><div></div>
  <button id="l">◀</button><button id="s" class="stop">停</button><button id="r">▶</button>
  <div></div><button id="b">▼</button><div></div>
</div>

<div class="row" style="margin-top:10px">
  <button id="hup" class="st">升高</button>
  <button id="hdn" class="st">降低</button>
</div>

<div class="row"><span>速度</span>
  <input id="sp" type="range" min="0" max="100" value="40">
  <span id="spv">40</span></div>

<div id="st">JS 已启动，等待连接…</div>
<div class="hint">按住方向键 = 连续动作，松手 = 停<br>
  速度上限 60%（代码里 SPEED_CAP）</div>

<script>
// ★ 第一件事：把 JS 错误显示到页面上（手机上没法开控制台）
window.onerror = function (msg, src, line) {
  try {
    var e = document.getElementById('st');
    if (e) { e.className = 'bad'; e.textContent = 'JS错误: ' + msg + ' @行' + line; }
  } catch (x) {}
};

//=============================================================================
// 状态
//   st : 0=停/直行 1=右转 -1=左转 180=后退   （和服务器 /cmd 的约定一致）
//   sp : 0..100 速度百分比
//   seq: 自增序号，用于丢帧诊断
//=============================================================================
var st = 0, sp = 40, seq = 0, lastResp = 0, pressed = 0;
var elSt = document.getElementById('st');
var elSp = document.getElementById('sp');
var elSpv = document.getElementById('spv');

elSt.textContent = 'JS 已启动，正在连接…';

function setSt(v) { st = v; pressed = 1; }

// 按住 = 给指令，松手 = 归 0（FPGA 侧还有 500ms 看门狗兜底）
function bind(id, val) {
  var b = document.getElementById(id);
  function down(e) { e.preventDefault(); setSt(val); }
  function up(e)   { e.preventDefault(); st = 0; pressed = 0; }
  b.addEventListener('mousedown', down);
  b.addEventListener('touchstart', down, {passive:false});
  b.addEventListener('mouseup', up);
  b.addEventListener('mouseleave', up);
  b.addEventListener('touchend', up);
  b.addEventListener('touchcancel', up);
}

bind('f', 0);      // 前进（直行）
bind('b', 180);    // 后退（注意：FPGA 侧 motion_decision 暂不支持负速度 -> 目前等同停车）
bind('l', -1);     // 左转
bind('r', 1);      // 右转

document.getElementById('s').addEventListener('click', function () {
  st = 0;
  pressed = 0;
  sp = 0;
  elSp.value = 0;
  elSpv.textContent = '0';
});
document.getElementById('s').addEventListener('touchstart', function (e) {
  e.preventDefault();
  st = 0; pressed = 0; sp = 0; elSp.value = 0; elSpv.textContent = '0';
}, {passive:false});

elSp.oninput = function () { sp = +elSp.value; elSpv.textContent = sp; };

// 站立高度：点一次发一帧 CONFIG 0x21（0=高位 1=低位）
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
document.getElementById('hdn').addEventListener('click', function () { ht = 1; sendHt(); });

//=============================================================================
// 10Hz 轮询（FPGA 看门狗 500ms，10Hz 允许连丢 4 帧）
//=============================================================================
setInterval(function () {
  // ★ 只有按住方向键才给速度和方向；松手一律发 sp=0（停）
  var sp_out = pressed ? sp : 0;
  var st_out = pressed ? st : 0;
  var url = '/cmd?st=' + st_out + '&sp=' + sp_out + '&sq=' + (seq++ & 255);
  fetch(url, {cache: 'no-store'})
    .then(function (r) { return r.text(); })
    .then(function (t) {
      lastResp = Date.now();
      elSt.className = '';
      elSt.textContent = '已连接  ' + t;
    })
    .catch(function () {
      elSt.className = 'bad';
      elSt.textContent = '断开！松手会自动停';
    });
}, 100);

// 2 秒没收到任何响应就提示（防止“看起来连上了其实是缓存”）
setInterval(function () {
  if (Date.now() - lastResp > 2000) {
    elSt.className = 'bad';
    elSt.textContent = '2 秒无响应 —— 检查是不是连的 RobotRC 热点';
  }
}, 1000);
</script></body></html>
"""

#-----------------------------------------------------------------------------
# 无 JS 调试页（/t）：纯 <a> 链接，点一下发一条指令
#
# 为什么要这个：带 JS 的页面在手机上有太多变量（fetch 策略、缓存、Content-Length
# 截断……），一旦不通很难判断是"网页问题"还是"服务器问题"。
# 这个页面**一行 JS 都没有**，点链接就是一次普通的页面跳转，
# 服务器回的 st=.. sp=.. 会直接显示在屏幕上 —— 通不通一眼就能看出来。
#-----------------------------------------------------------------------------
PAGE_H = """<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>轮足机器人 · 无JS控制台</title>
<style>
 body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:12px}
 h3{margin:6px 0 12px;font-weight:400;font-size:17px}
 .g{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-bottom:14px}
 a{display:block;padding:18px 0;text-align:center;text-decoration:none;
   background:#2a6df4;color:#fff;border-radius:12px;font-size:19px}
 a.green{background:#2f8f4f}
 a.red{background:#c0392b}
 a.gray{background:#444}
 .t{color:#8f8;font-size:13px;margin:10px 0;font-family:monospace}
 .h{color:#888;font-size:12px;line-height:1.7;margin-top:14px}
</style></head><body>

<h3>轮足机器人 · 无JS控制台</h3>

<div class="t">点一下 = 发一条指令（运动 0.6 秒后自动停）</div>

<div class="g">
  <a class="green" href="/cmd?ht=0&ref=1">升高</a>
  <a class="green" href="/cmd?ht=1&ref=1">降低</a>
</div>

<div class="g">
  <a href="/cmd?st=0&amp;sp=40&amp;sq=1&amp;ref=1">前进 40%</a>
  <a href="/cmd?st=0&amp;sp=80&amp;sq=2&amp;ref=1">前进 80%</a>
</div>

<div class="g">
  <a href="/cmd?st=1&amp;sp=40&amp;sq=3&amp;ref=1">右转</a>
  <a href="/cmd?st=-1&amp;sp=40&amp;sq=4&amp;ref=1">左转</a>
</div>

<div class="g">
  <a class="gray" href="/cmd?st=180&amp;sp=40&amp;sq=5&amp;ref=1">后退</a>
  <a class="red" href="/cmd?st=0&amp;sp=0&amp;sq=6&amp;ref=1">停</a>
</div>

<div class="h">
  点完会跳到一个显示 st/sp 的文字页，
  按手机的返回键回到本页继续点。<br>
  想连续运动用 / 页面（需要浏览器支持 JS）。
</div>

</body></html>"""


PAGE_T = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RC test</title>
<style>
 body{background:#111;color:#eee;font-family:sans-serif;padding:16px}
 a{display:block;margin:10px 0;padding:18px;background:#2a6df4;color:#fff;
   text-decoration:none;border-radius:12px;text-align:center;font-size:20px}
 a.stop{background:#c0392b}
 p{color:#8f8;font-family:monospace}
</style></head><body>
<h3>无 JS 测试页</h3>
<a href="/cmd?st=0&amp;sp=40&amp;sq=1&amp;ref=1">直行 40%</a>
<a href="/cmd?st=1&amp;sp=40&amp;sq=2&amp;ref=1">右转</a>
<a href="/cmd?st=-1&amp;sp=40&amp;sq=3&amp;ref=1">左转</a>
<a href="/cmd?st=0&amp;sp=100&amp;sq=4&amp;ref=1">直行 100%</a>
<a href="/cmd?st=180&amp;sp=40&amp;sq=5&amp;ref=1">后退</a>
<a class="stop" href="/cmd?st=0&amp;sp=0&amp;sq=6&amp;ref=1">停</a>
<p>点完链接后，屏幕上应该显示 st=.. sp=..<br>
   如果显示这个页面本身，说明服务器还在发旧页面（看 /sdcard 里的 wifi_rc.py 版本）</p>
</body></html>"""



#=============================================================================
# 一、UART
#=============================================================================
def _setup_uart():
    """自己把 GPIO5/6 复用成 UART2 @115200（不依赖 camera_vision）。

    ★ 为什么改成自包含：
      原来这里 import camera_vision，而它在 import 阶段就会拉
      `from media.sensor import *` / `media.display` / `media.media` ——
      在"没初始化摄像头"的场景下这一步可能直接卡住（实测 rc_boot.txt
      停在 starting run() 就不动了，且没有任何异常）。
      遥控根本不需要摄像头，所以这里只用 machine.UART + FPIOA，最干净。
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
# 二、WiFi 热点
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
        # ★ 幂等：如果热点已经起来了（有 IP），就**直接沿用，不重新 config**。
        #   重新 config 会让 AP 重启一次 -> 已连着的手机被踢下线 ✗
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
        ok = False
        for _try in range(3):
            try:
                ap.config(ssid=WIFI_SSID, key=WIFI_PASS)
                ok = True
            except Exception:
                try:
                    ap.config(essid=WIFI_SSID, password=WIFI_PASS)
                    ok = True
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
# 三、HTTP 服务 + 发帧主循环
#=============================================================================
def _parse_query(path):
    """从 '/cmd?st=-1&sp=40&sq=3' 里取参数，返回 dict。

    ★ 2026-10-30：不再用白名单！
      之前写成 out = {'st':None,'sp':None,'sq':None}，
      后面加的 'ht' / 'ref' 都被静默丢弃 ->
      q.get('ref') 永远是 None -> 302 跳转从未生效 ->
      点完按钮停在白底小字页（用户反馈的"白屏"）。

    现在：任意键名都保留，能转整数就转，不行就原样存字符串。
      取值一律用 q.get('xxx')，缺省就是 None。
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


def run(port=HTTP_PORT):
    _log('  [run] enter  VERSION=%s' % VERSION)
    print('[信息] wifi_rc 版本 %s' % VERSION)

    #--------------------------------------------------------------
    # ★ 开机自启时 main.py 跑得很早，WiFi 驱动可能还没就绪 ——
    #   此时 WLAN(AP_IF).config() 会静默失败，热点根本不会广播。
    #   在 REPL 里手动调就没事（系统早热了），所以这个坑只在开机自启时出现。
    #   这里先等 3 秒，给驱动留出初始化时间。
    #--------------------------------------------------------------
    #--------------------------------------------------------------
    # ★ 强制 GC：上一次 run() 被 Ctrl-C 打断时，8080 的 socket 可能
    #   还没被回收（MicroPython 不会立刻 close），再 bind 就 EPERM。
    #   这里先 collect 一次，把僵尸 socket 收掉。
    #--------------------------------------------------------------
    try:
        import gc
        gc.collect()
        _log('  [run] gc.collect ok, free=%d' % gc.mem_free())
    except Exception:
        pass

    _log('  [run] wait 3s for wifi driver')
    time.sleep_ms(3000)
    uart = _setup_uart()
    if uart is None:
        _log('  [run] abort: uart is None')
        return
    _log('  [run] uart OK')
    _log('  [run] setup_ap begin')
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
        print('[错误] 换端口：wifi_rc.run(8080)')
        return
    srv.listen(2)
    srv.settimeout(0.005)        # 5ms；越短轮询越及时（日志已挪出热路径）

    #--------------------------------------------------------------
    # ★ 先打开手动模式（CONFIG 帧 0x20 = 1）
    #   不开这一步，FPGA 一直在自动模式，遥控的 0x05 帧会被完全忽略。
    #   每 2 秒重发一次：万一 CONFIG 帧丢了/FPGA 复位了也能自己恢复。
    #--------------------------------------------------------------
    def _send_manual_on():
        try:
            uart.write(up.make_config(0x20, bytes([1])))
        except Exception as e:
            print('[警告] 发手动模式命令失败: %s' % e)

    _send_manual_on()
    _send_manual_on()
    print('[信息] 已发送 CONFIG 0x20=1（手动模式）')

    # ★ 用真实的 IP（不同固件默认网段不同，实测是 192.168.169.1，
    #   而不是常见的 192.168.4.1）
    try:
        real_ip = ap.ifconfig()[0]
    except Exception:
        real_ip = AP_IP
    _log('  [run] HTTP listening on :%d  ip=%s' % (port, real_ip))
    print('[信息] HTTP 监听 :%d，浏览器打开 http://%s:%d/' % (port, real_ip, port))
    print('[信息] 发帧 %dHz，速度上限 %d%%，%dms 无指令自动停'
          % (SEND_HZ, SPEED_CAP, CMD_TIMEOUT_MS))

    steer = 0
    speed = 0
    seq = 0
    stance = 0        # 站立高度：0=高位(0状态) 1=低位
    last_cmd = time.ticks_ms()
    t_send = time.ticks_ms()
    t_manual = time.ticks_ms()

    period_ms = max(1, 1000 // SEND_HZ)

    _log('  [run] ENTER MAIN LOOP (热点应该已经可见了)')
    while True:
        #---- 1) 收 HTTP（非阻塞）----
        try:
            cli, addr = srv.accept()
        except Exception:
            cli = None

        if cli:
            try:
                #---------------------------------------------------------
                # ★ 2026-10-29 实机踩到：浏览器会**预先建立一条只连接、
                #   不发送任何数据的空闲 TCP 连接**。原来 settimeout(0.3)
                #   加单次 recv 会在这条空连接上白等 300ms，而且拿到空请求后
                #   走 else 分支**把整个网页回给了一个空请求**，
                #   真正的 /cmd 轮询就全被堵死 ——
                #   现象正是"页面能打开，但底部永远显示未连接"。
                #
                #   现在：超时压到 120ms（请求行是连上就发，够用）；
                #   空请求直接关掉继续循环，绝不给它回页面。
                #---------------------------------------------------------
                cli.settimeout(0.02)     # 请求行是连上就发，20ms 足够（浏览器会开 4~6 个预连接，超时越短越不容易堵）
                req = b''
                try:
                    req = cli.recv(512)
                except Exception:
                    req = b''

                if not req:
                    _loghttp('  [http] empty request -> drop')
                    try:
                        cli.close()
                    except Exception:
                        pass
                    cli = None
                    continue        # ★ 必须 continue，别拿 None 去 send

                line = req.split(b'\r\n', 1)[0].decode('utf-8', 'ignore')
                # "GET /cmd?st=1&sp=40&sq=7 HTTP/1.1"
                path = ''
                parts = line.split(' ')
                if len(parts) >= 2:
                    path = parts[1]

                # 去掉查询串再做路径判断，这样 /?v=2 这种**缓存穿透**写法也能出页面
                path_only = path.split('?', 1)[0]

                _loghttp('  [http] path=%r' % path)
                if path_only == '/cmd':
                    # ★ 全段 try/except：任何参数都不能把服务搞挂
                    try:
                        q = _parse_query(path)
                        st = q.get('st')
                        spv = q.get('sp')
                        ht = q.get('ht')
                        try:
                            st = None if st is None else int(st)
                        except Exception:
                            st = None
                        try:
                            spv = None if spv is None else int(spv)
                        except Exception:
                            spv = None
                        try:
                            ht = None if ht is None else int(ht)
                        except Exception:
                            ht = 0

                        if st is not None:
                            if st == 180:
                                steer = 0
                            elif st == 1:
                                steer = 100
                            elif st == -1:
                                steer = -100
                            elif st == 0:
                                steer = 0
                            elif st >= 2 or st <= -2:
                                steer = max(-100, min(100, st))

                        if ht is not None:
                            stance = 1 if ht else 0
                            try:
                                uart.write(up.make_config(0x21, bytes([stance])))
                            except Exception as e:
                                print('[cmd] stance write failed', e)

                        if spv is not None:
                            speed = max(0, min(SPEED_CAP, spv))
                            if st == 180 and speed > 0:
                                speed = -speed

                        last_cmd = time.ticks_ms()
                    except Exception as e:
                        print('[cmd] ERROR', e)

                    # 回 302 跳回控制台（失败就回 200 小页）
                    try:
                        hdr = ('HTTP/1.1 302 Found\r\nLocation: /h\r\n'
                               'Cache-Control: no-store\r\n'
                               'Content-Length: 0\r\nConnection: close\r\n\r\n')
                        _send_all(cli, hdr.encode())
                    except Exception as e:
                        print('[cmd] 302 failed', e)
                        try:
                            _send_all(cli, b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n'
                                           b'Cache-Control: no-store\r\nConnection: close\r\n'
                                           b'Content-Length: 18\r\n\r\n<a href="/h">OK</a>')
                        except Exception:
                            pass

                elif path_only == '/ver':
                    # ★ 探针：返回版本号 + PAGE 长度 + 当前文件路径
                    #   手机打开 /ver 就能确认服务端跑的是哪一版
                    try:
                        _p = __file__
                    except Exception:
                        _p = '?'
                    txt = ('VER=%s\nPAGE_BYTES=%d\nFILE=%s\n'
                           % (VERSION, len(PAGE.encode('utf-8')), _p))
                    _b = txt.encode('utf-8')
                    _h = ('HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\n'
                          'Cache-Control: no-store\r\n'
                          'Content-Length: %d\r\nConnection: close\r\n\r\n' % len(_b))
                    try:
                        _send_all(cli, _h.encode())
                        _send_all(cli, _b)
                    except Exception:
                        pass

                elif path_only in ('/h', '/html'):
                    # ★ 纯 HTML、零 JS 的控制台：手机浏览器不执行 script 时用
                    body_h = PAGE_H.encode('utf-8')
                    hdr_h = ('HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n'
                             'Cache-Control: no-store\r\n'
                             'Content-Length: %d\r\nConnection: close\r\n\r\n'
                             % len(body_h))
                    try:
                        _send_all(cli, hdr_h.encode())
                        _send_all(cli, body_h)
                    except Exception:
                        pass

                elif path_only == '/t':
                    # 无 JS 调试页
                    body_t = PAGE_T.encode('utf-8')
                    hdr = ('HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n'
                           'Cache-Control: no-store\r\n'
                           'Content-Length: %d\r\nConnection: close\r\n\r\n'
                           % len(body_t))
                    try:
                        n1 = _send_all(cli, hdr.encode())
                        n2 = _send_all(cli, body_t)
                        _loghttp('  [http] /t sent hdr=%s body=%s (total %d)'
                             % (n1, n2, len(body_t)))
                    except Exception as e:
                        _loghttp('  [http] /t send FAILED: %r' % e)
                elif path_only in ('/', '', '/index.html'):
                    # 只对真正的 GET / 回页面（空 path 已经在上面 drop 掉了，
                    # 这里保留判断是双保险）
                    # ★ 关键：Content-Length 必须是**字节数**，不是字符数。
                    #   PAGE 里有中文（一个汉字 UTF-8 占 3 字节），
                    #   用 len(PAGE) 会少报 75 字节 -> 浏览器按声明长度截断
                    #   -> 网页末尾的 <script> 被切掉 -> 按钮能显示但 JS 从不执行
                    #   -> 页面永远"未连接"（2026-10-29 实机踩到）
                    hdr = ('HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n'
                           'Cache-Control: no-store\r\n'
                           'Content-Length: %d\r\nConnection: close\r\n\r\n'
                           % len(PAGE.encode('utf-8')))
                    try:
                        n1 = _send_all(cli, hdr.encode())
                        n2 = _send_all(cli, PAGE.encode())
                        _loghttp('  [http] / sent hdr=%s body=%s' % (n1, n2))
                    except Exception as e:
                        _loghttp('  [http] / send FAILED: %r' % e)
                else:
                    # /favicon.ico 之类：回 404，别再回整页
                    try:
                        _send_all(cli, b'HTTP/1.1 404 Not Found\r\n'
                                 b'Content-Length: 0\r\nConnection: close\r\n\r\n')
                    except Exception:
                        pass
            except Exception as e:
                print('[警告] HTTP 处理异常: %s' % e)
            finally:
                try:
                    cli.close()
                except Exception:
                    pass

        #---- 2) 手机 600ms 没动静 -> 本地停车（第二层保护）----
        if time.ticks_diff(time.ticks_ms(), last_cmd) > CMD_TIMEOUT_MS:
            speed = 0
            steer = 0

        #---- 2.5) 每 2 秒补发一次手动模式（防丢帧/防 FPGA 复位）----
        if time.ticks_diff(time.ticks_ms(), t_manual) > 2000:
            t_manual = time.ticks_ms()
            _send_manual_on()
            # 站立高度也要定期重发：FPGA 复位后要能恢复现场
            try:
                uart.write(up.make_config(0x21, bytes([stance])))
            except Exception:
                pass

        #---- 3) 到点发一帧 ----
        if time.ticks_diff(time.ticks_ms(), t_send) >= period_ms:
            t_send = time.ticks_ms()
            seq = (seq + 1) & 0xFF

            # ★ 2026-10-30：后退已支持（FPGA 侧 motion_decision 加了 dir_inv）
            #   speed 字节：bit7 = 后退标志，bit6:0 = 百分比 0~100
            mag = abs(speed)
            if mag > 100:
                mag = 100
            rev = 1 if speed < 0 else 0
            try:
                # ★ 2026-11-01 修复：不能用 up.make_rc_cmd！
                #   它内部有 if sp>100: sp=100 钳位，
                #   后退时 speed_byte = 0x80|mag > 100 -> 被钳成 100
                #   -> bit7 丢失 -> 变成【100% 前进】。
                #   改成自己组载荷 + pack_frame（帧头/BCC/帧尾仍由库生成）
                _pl = bytes([steer & 0xFF, ((rev << 7) | mag) & 0xFF, seq & 0xFF])
                uart.write(up.pack_frame(up.TYPE_RC_CMD, _pl))
            except Exception as e:
                # ★ 以前这里只 print（拔了 USB 根本看不到），
                #   卡里的 uart_proto 缺 make_rc_cmd 时每次都静默失败 -> 一帧都没发出去。
                _log('  [rc] uart write FAILED: %r' % e)
                print('[警告] 串口写失败: %s' % e)


#=============================================================================
# 四、dry-run：不接硬件，只打印帧
#=============================================================================
def dry_run():
    """在 PC 或板子上不接串口，直接打印几帧看看格式对不对。"""
    import uart_proto as up
    print('=== wifi_rc dry-run ===')
    cases = [(0, 0, 0), (0, 40, 1), (100, 40, 2), (-100, 40, 3), (0, 100, 4)]
    for st, sp, sq in cases:
        f = up.make_rc_cmd(st, sp, sq)
        hexs = ' '.join('%02X' % b for b in f)
        print('  steer=%4d speed=%3d seq=%d -> %s' % (st, sp, sq, hexs))
    print('总长应为 12 字节（9 固定 + 3 载荷）')


if __name__ == '__main__':
    dry_run()
