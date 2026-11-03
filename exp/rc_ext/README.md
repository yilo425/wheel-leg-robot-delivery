# 遥控扩展功能试验（rc_ext）

> 目标：在**已验证能用**的手动版基础上，扩展出完整遥控功能。
> 基线：`wifi_rc_baseline.py`（36220 字节，2026-11-01 实机验证通过）
> 硬件：K230 庐山派 Lite + FPGA GW5AT-138C + PCA9685 + 4 舵机 + 4 电机

---

## 一、★ 为什么这一版能成功（血泪经验，必须遵守）

这一版是在踩了十几次坑之后才跑通的。**任何一条被破坏，就会出现"页面打不开/打不全/点了没反应"**。

### 1. 换行符必须是 LF（最致命）

```
CRLF (\r\n) -> MicroPython 在【跨行表达式】处报 SyntaxError
           -> import 失败 -> run() 崩溃 -> HTTP 服务根本没起来
           -> 表现：所有网址一片空白
```

**写文件时必须**：
```python
data = open(p,'rb').read().replace(b'\r\n', b'\n')
open(p,'wb').write(data)
```
**校验**：`data.count(b'\r\n') == 0`

### 2. `_send_all` 必须用纯 bytes 切片，不能用 memoryview

```python
# ✓ 正确（实机验证）
def _send_all(sock, data):
    if isinstance(data, str):
        data = data.encode('utf-8')
    total = 0
    n = len(data)
    while total < n:
        k = sock.send(data[total:])
        if not k:
            break
        total += k
    return total

# ✗ 错误（曾导致点按钮后页面无法打开）
mv = memoryview(data)
while len(mv) > 0:
    k = sock.send(mv)
    ...
```

**MicroPython 的 `sock.send()` 可能只发一部分就返回**，不循环就会截断 → 网页只显示上半部分（曾表现为"只有 6 个按钮，下面一片黑，划不动"）。

### 3. `/cmd` 处理器必须整段 try/except + int() 容错

```python
if path_only == '/cmd':
    try:
        st = int(st) if st is not None else None   # 每个参数都要容错
        ...
    except Exception as e:
        print('[cmd] ERROR', e)
    # 无论如何都要回响应
```

**绝不能让它抛异常** —— 一抛异常连接就被掐断，浏览器显示"网页无法打开"。

### 4. `/cmd` 一定要回响应

```python
# 成功：302 跳回 /h（体验好）
hdr = 'HTTP/1.1 302 Found\r\nLocation: /h\r\nCache-Control: no-store\r\nContent-Length: 0\r\nConnection: close\r\n\r\n'
# 失败：回 200 小页面（保底，别让浏览器空等）
```

### 5. `_parse_query` 不能有键名白名单

```python
# ✓ 任意键都收，能转 int 就转，不行就存字符串
# ✗ 曾写成 out={'st':None,'sp':None,'sq':None} -> 后来加的 ht/ref 被静默丢弃
#    -> q.get('ref') 永远 None -> 302 从不生效 -> 点完停在白底小字页
```

### 6. `/h` 页面必须是**纯 HTML、零 JS**

手机浏览器（夸克/一加）可能**不执行页面内联 `<script>`** ✗
所有按钮必须是 `<a href="/cmd?...">` 链接 ✓
状态展示用**服务端渲染**（Python 拼字符串），不靠 JS ✓

### 7. HTML 里的 `&` 要写成 `&amp;`

```html
<a href="/cmd?st=0&amp;sp=40&amp;ref=1">前进</a>
```

### 8. `DEBUG_HTTP` 必须关闭

SD 卡写日志每次 10~50ms，一个请求写 2~4 次 → 单请求 100~200ms
而浏览器轮询间隔只有 100ms → **必然超时** → "页面一直断开闪烁"

### 9. `run()` 开头必须 `gc.collect()`

上一次 `run()` 被 Ctrl-C 打断时，8080 的 socket 没被回收 → 再 bind 报 `OSError: EPERM`
**而且这个状态只有物理断电 10 秒才能清掉** ✗（软复位都不一定行）

### 10. 板子**不能自己请求自己**

```
127.0.0.1        -> OSError EHOSTUNREACH
192.168.169.1    -> 连接超时（lwip 不支持 AP 自连）
```
**所以不要写"自测"脚本**，只能用手机验证 ✓

### 11. 服务端绑定与端口

```python
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', 8080))
srv.listen(8)
srv.settimeout(0.005)
```

### 12. 浏览器会开 4~6 条**空闲预连接**

`cli.settimeout(0.02)`；收到空请求立刻 `cli.close()` + `continue`，**绝不能给它回页面** ✗

---

## 二、硬件与协议（不要改）

### 串口

```python
from machine import FPIOA, UART
fpioa = FPIOA()
fpioa.set_function(5, FPIOA.UART2_TXD)   # K230 排针 11 -> FPGA F16
fpioa.set_function(6, FPIOA.UART2_RXD)   # K230 排针 13 <- FPGA C19
uart = UART(UART.UART2, 115200)
```

### 帧格式

```
AA 55 | VER | TYPE | LEN(2B, 大端) | DATA | BCC | 55 AA
BCC = XOR(VER, TYPE, LEN_H, LEN_L, DATA...)
```

### 关键帧（全部用 `uart_proto.py` 里的函数生成）

| 函数 | 用途 |
|---|---|
| `up.make_rc_cmd(steer, speed_byte, seq)` | RC 遥控帧（20Hz 持续发）|
| `up.make_config(0x20, bytes([v]))` | 手动/自动：0=自动 1=手动 |
| `up.make_config(0x21, bytes([v]))` | 站立高度：0=高位(0状态) 1=低位 |

### RC 帧字段

```
steer : int8  -100..100（正 = 右转）
speed : uint8  bit7 = 后退标志, bit6:0 = 0..100%（速度百分比）
seq   : uint8  自增
```

### ⚠️ 两个必须持续发送

| 东西 | 原因 |
|---|---|
| **RC 帧 20Hz** | FPGA 有 500ms 看门狗，超时就停车 |
| **CONFIG 每 2 秒重发** | 防丢帧 / 防 FPGA 复位丢现场 |

---

## 三、要实现的功能

在 `wifi_rc_baseline.py` 基础上扩出：

| # | 功能 | 说明 |
|---|---|---|
| 1 | **手动 / 自动** 两个按钮 | 手动 = `CONFIG 0x20=1`；自动 = `0x20=0`（FPGA 状态机接管，ARM 停止发 RC 帧）|
| 2 | **前进 / 后退** | 手动模式下：点一下**就一直走**，直到点【停止】（服务端保持状态，不靠 600ms 超时）|
| 3 | **左转 90° / 右转 90°** | 定时转弯：`steer=±100` + `speed=40%`，持续 `TURN_MS` 后自动停。**`TURN_MS = 360ms`**（轮距 W=230mm，R=W/2，40% 时 v=502.7mm/s）|
| 4 | **停止** | `steer=0, speed=0` |
| 5 | **升高 / 降低** | `CONFIG 0x21` 0/1 |
| 6 | **状态显示** | 服务端渲染：模式（手动/自动）、当前动作（停止/前进/后退/左转/右转）、站立（高位/低位）|

### 交互要求

- 页面按钮用 `<a href>` 链接
- 点完 302 跳回控制台，状态栏显示新状态
- 高亮当前激活的按钮（服务端拼 class）

---

## 四、交付要求

1. 输出文件：`exp/rc_ext/wifi_rc_ext.py`
2. **保持 LF**，校验 `CRLF == 0`
3. 必须通过 `ast.parse()` 语法检查
4. 写一份 `exp/rc_ext/CHANGELOG.md`：列出相对基线的每一处改动和原因
5. **不要修改** `arm/wifi_rc.py`（那是实机在用的稳定版）
6. 不要修改 `exp/rc_ext/wifi_rc_baseline.py` 和 `uart_proto.py`

---

## 五、验证清单（写进交付说明）

上板前自查：

```
□ 文件是 LF（CRLF=0）
□ ast.parse 通过
□ 没有 memoryview
□ /cmd 整段包在 try/except 里
□ /cmd 无论如何都回响应
□ /h 页面零 JS，全 <a href>
□ & 写成 &amp;
□ DEBUG_HTTP = False
□ run() 开头有 gc.collect()
□ 模式与站立高度每 2 秒重发
□ 手动模式下 RC 帧 20Hz 持续发
□ 自动模式停止发 RC 帧（避免和 FPGA 状态机抢控制）
□ TURN_MS = 360
```

---

## 13. ★【2026-11-01 子代理发现】make_rc_cmd 会把"后退"吃掉

`uart_proto.make_rc_cmd()` 内部有钳位：

```python
if sp > 100:
    sp = 100
```

而调用方按协议传的是 `(rev << 7) | mag` —— 后退时 = `0x80 | 40 = 168 > 100`
→ **被钳成 100** → bit7（后退标志）丢失
→ FPGA 收到的是 **【100% 前进】** ✗（实测载荷 `00 64 07`，期望 `00 a8 07`）

### 正确写法

```python
_pl = bytes([steer & 0xFF, ((rev << 7) | mag) & 0xFF, seq & 0xFF])
uart.write(up.pack_frame(up.TYPE_RC_CMD, _pl))
```

**不要改 `make_rc_cmd`（协议库改动风险大）**，用 `pack_frame` 自己组 3 字节载荷 ✓
帧头/长度/BCC/帧尾仍由 `pack_frame` 生成，协议实现只有一份 ✓

**校验**：`make_rc_cmd(0, 0xA8, 7)` 的载荷必须是 `00 a8 07`，不是 `00 64 07`
