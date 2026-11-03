# K230（庐山派 Lite-K230D）接入与联调手册

> 面向：接手 ARM 侧代码的同学
> 状态：**板子已调通，双向链路已实测通过**（2026-10-17）
> 对应 FPGA 固件：`robot.fs`（串口在 `C19`/`F16`，见 §2）

---

## 0. 一句话现状

板子里已经有一张烧好 **CanMV v1.8** 的 TF 卡，插上电就启动；
USB 连电脑后枚举成 **COM 口**，能进 MicroPython REPL；
和 FPGA 的串口**双向都通了**（实测 10Hz telemetry + 视觉帧能驱动 FPGA 状态机）。

**你不需要再烧固件**（除非卡坏了）。

---

## 1. 硬件与启动

| 项 | 内容 |
|---|---|
| 型号 | 立创·庐山派 **Lite-K230D-128M**（40P 排针是**树莓派兼容排布**）|
| 系统 | **CanMV v1.8**（MicroPython）|
| 镜像 | `CanMV_K230D_Lushanpi_Lite_micropython_v1.8-0-gc2d1f5c_nncase_v2.11.0.img`（620 MB）|
| 镜像 MD5 | `a1bf6fddfa28470f801ed20997d1d319`（官方与实测一致）|
| 启动介质 | **TF 卡**（microSD，板子**背面**，靠 MIPI DSI 接口和右上角安装孔）|

### ⚠️ 三条铁律

1. **TF 卡是系统盘** —— 拔了、格了、换张空白卡，板子就起不来
2. **出现 `K230 USB Boot Device`（设备管理器带感叹号）= 没从卡启动成功**，
   **不是驱动问题**。先断电重上电；还不行就重烧镜像
3. 重烧**不需要读卡器**：用 **K230BurningTool**（`kendryte/k230_burning_tool`），
   **目标介质选 `SD Card`**，它会把镜像直接写进插在板子上的 TF 卡

### 连电脑

一根 USB Type-C（**同一根线既供电又是串口**）→ 设备管理器出现 **`USB 串行设备 (COMx)`**。
用 [CanMV Web IDE](https://wiki.lckfb.com/zh-hans/web-tool/canmv-web-ide.html)（浏览器版，不用装 IDE）
或任意串口终端连上，**115200 8N1**。

> ★ **PC 侧用 pyserial 时必须 `ser.dtr = True`**
> 这个 CDC 口不置 DTR 就**一个字节都收不到**（实测踩过，非常容易误判成"线没接好"）。

---

## 2. 与 FPGA 的接线（**3 根**）

| ARM 排针 | 芯片 GPIO | 方向 | FPGA 引脚 |
|---|---|---|---|
| **11** | GPIO05（`UART2_TXD`）| → | **`F16`**（FPGA 的 `uart_rx`）|
| **13** | GPIO06（`UART2_RXD`）| ← | **`C19`**（FPGA 的 `uart_tx`）|
| **GND**（6/9/14/20/25/30/34/39 任一）| — | ↔ | **`GND`** |

**交叉接**（ARM 的 TX 接 FPGA 的 RX）。电平都是 3.3V，直连。

### ⚠️ 两个必须知道的点

1. **代码里填的是 GPIO 号，不是排针号**
   ```python
   fpioa.set_function(5, FPIOA.UART2_TXD)   # GPIO5  -> 排针 11
   fpioa.set_function(6, FPIOA.UART2_RXD)   # GPIO6  -> 排针 13
   ```
   官网示例里的 `set_function(11/12, ...)` 是 **GPIO11/12**，不是排针 11/13 —— 照抄会发现"代码跑通但排针没波形"。

2. **FPGA 那侧的串口不是标准的 `M15`/`J21`**
   板上原本的串口只引到两个小焊盘上（插不了杜邦线），已改到 GPIO0 扩展口的
   `C19`（= GPIO0_25，tx）/ `F16`（= GPIO0_26，rx）。

---

## 3. 协议摘要

完整文档见 **`doc/uart_frame_protocol.md`**，这里只列最容易写错的部分。

```
+------+------+-----+------+-------+-------+---------+-----+------+------+
| 0xAA | 0x55 | VER | TYPE | LEN_H | LEN_L | DATA... | BCC | 0x55 | 0xAA |
+------+------+-----+------+-------+-------+---------+-----+------+------+
VER = 0x01
BCC = XOR(VER, TYPE, LEN_H, LEN_L, DATA...)      ← 不含帧头帧尾
多字节一律大端；有符号量用补码
```

| TYPE | 名称 | 方向 |
|---|---|---|
| `0x02` | `VISION_RESULT` | **ARM → FPGA** |
| `0x10` | `TELEMETRY` | FPGA → ARM（10Hz）|

### 3.1 `VISION_RESULT` 载荷

```
目标数(1B) + 每个目标 7B：序号(1B) 类别(1B) x中心(2B) y中心(2B) 置信度(1B)
载荷长度 = 1 + 7n
x/y 归一化到 0..65535，画面正中心 = 32768（0x8000）
```

类别码：

| 码 | 含义 | FPGA 行为 |
|---|---|---|
| `0x00` | 无 | 回 AUTO |
| `0x01` | 人员 | 减速 + 保持距离 |
| `0x02` | 障碍物 | **绕行（AVOID）** |
| `0x03` | 目标点/终点 | 向目标行进（GOAL）|
| `0x04` | **低矮障碍** | **触发升降腿（LIFT）** |
| `0x05` | 台阶 | 触发升降腿（另一种模式）|

### 3.2 `TELEMETRY` 载荷（**13 字节**）

```
状态(1B) + spdL(2B) + spdR(2B) + dist(2B) + roll(2B) + pitch(2B) + yaw(2B)
全部大端；速度与姿态角是有符号补码；姿态角单位 0.01°
```

状态编码（与 `rtl/app/main_fsm.v` 一致）：

| 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| IDLE | MANUAL | **AUTO** | **AVOID** | **LIFT** | GOAL | ESTOP |

---

## 4. 已验证的测试向量（可直接当单元测试）

以下帧都**实测可用**（BCC 用项目文档里的 `pack_frame` 算的，不是手算）：

| 用途 | 帧（HEX）| 期望 |
|---|---|---|
| 障碍物（正前方，conf=88）| `AA 55 01 02 00 08 01 00 02 80 00 80 00 58 50 55 AA` | `AUTO(2)` → **`AVOID(3)`** |
| 低矮障碍（触发抬腿）| `AA 55 01 02 00 08 01 00 04 80 00 80 00 5A 54 55 AA` | **`LIFT(4)`** |
| 清空目标（回 AUTO）| `AA 55 01 02 00 01 00 02 55 AA` | 回 **`AUTO(2)`** |

> 发帧前 FPGA 必须已经在 `AUTO`（板上 **S0** 启动键按一下）。
> 早期固件有个坑：`goal_reached`（`E16`）上拉 + 高有效逻辑 → 未接线时被判成"已到终点"，
> 按 S0 会直接进 `GOAL`。**2026-10-17 已把逻辑改成低有效**，现在 `E16` 悬空即可正常工作。

---

## 5. 参考实现（MicroPython，可直接跑）

```python
from machine import UART, FPIOA
import time

f = FPIOA()
f.set_function(5, FPIOA.UART2_TXD)      # GPIO5 -> 排针11
f.set_function(6, FPIOA.UART2_RXD)      # GPIO6 -> 排针13
u = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
         parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

def pack(ftype, payload):                # -> bytes
    body = bytes([0x01, ftype, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF]) + payload
    bcc = 0
    for b in body:
        bcc = bcc ^ b
    return b'\xaa\x55' + body + bytes([bcc]) + b'\x55\xaa'

def vision(cls, cx=32768, cy=32768, conf=88):
    pl = bytes([1, 0, cls, cx >> 8, cx & 0xFF, cy >> 8, cy & 0xFF, conf])
    return pack(0x02, pl)

def parse(buf):
    out = []
    i = 0
    while True:
        i = buf.find(b'\xaa\x55', i)
        if i < 0 or len(buf) < i + 9:
            break
        n = (buf[i+4] << 8) | buf[i+5]
        total = 9 + n                    # ★ 2+1+1+2+n+1+2 = 9+n（写 8+n 会全部解析失败）
        if len(buf) < i + total:
            break
        bcc = 0
        for b in buf[i+2:i+6+n]:
            bcc = bcc ^ b
        if bcc == buf[i+6+n] and buf[i+total-2] == 0x55 and buf[i+total-1] == 0xAA:
            out.append((buf[i+3], buf[i+6:i+6+n]))
            i += total
        else:
            i += 2
    return out

# 1) 收 1 秒 telemetry
buf = b''
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 1000:
    d = u.read(128)
    if d:
        buf += d
for t, p in parse(buf):
    if t == 0x10 and len(p) == 13:
        print('state', p[0])

# 2) 发一帧障碍物
u.write(vision(0x02))
```

---

## 6. 实测记录（2026-10-17，实机）

| 验证项 | 结果 |
|---|---|
| 从 TF 卡启动 | ✅ USB 枚举 `VID_1209/PID_ABD1` → COM9 |
| MicroPython REPL | ✅ `print(1+1)` → `2` |
| FPGA → ARM | ✅ 10Hz TELEMETRY，帧头/长度/**BCC 全对** |
| ARM → FPGA | ✅ 发障碍物帧 → 状态 `AUTO(2)` → **`AVOID(3)`** |
| 稳态帧率 | ✅ 15 帧 / 1.5s = **10Hz**（设计值）|

---

## 7. 已知问题与坑

| 坑 | 说明 / 对策 |
|---|---|
| **`goal_reached` 极性**（**已修，2026-10-17**）| 原来引脚 `PULL_MODE=UP`（未接=1）但 `main_fsm` 把 1 当"到达" → 一按启动就进 `GOAL` 停车。**已改成低有效**（接地=到达，和 `lift_lim_n` 一致），所以 **`E16` 现在不用再接 GND 了**。⚠️ 旧固件仍需接地 |
| **DTR 不置起收不到数据** | PC 侧：`ser.dtr = True` |
| **GPIO 号 ≠ 排针号** | 见 §2 |
| **`K230 USB Boot Device`** | 卡没启动成功，不是驱动问题 |
| **动线时别碰 FPGA 的 USB** | FPGA 的比特流在 SRAM，掉电即失，要重烧（30 秒）|
| 帧总长写错 | `9 + payload_len`，不是 `8 + payload_len` |
| 帧率测量 | 读之前串口里有**积压数据**，第一窗口会偏高；看**稳态**（第二个窗口）|

---

## 8. 待硬件到齐后再测的部分

| 项 | 依赖 |
|---|---|
| TOF 距离进 telemetry | VL53L0X（未到货）→ 现在 `dist` 恒 0 |
| 姿态角（roll/pitch/yaw）| MPU6050（未接）→ 现在恒 0 |
| 轮速 `spdL/spdR` | 编码器（未接）→ 现在恒 0 |

> 所以在 ARM 侧看到 `dist=0 / spd=0 / roll=0` 是**正常的**，不是通信问题。
