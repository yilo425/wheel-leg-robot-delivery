# arm/ —— ARM 侧协议代码（可以直接用）

> 这是给 **B（ARM 视觉与通信）** 准备的**可运行代码**，不是文档片段。
> **板子还没到也能跑** —— 先用 `--dry-run`，不依赖任何硬件。

---

## 文件

| 文件 | 作用 | 状态 |
|---|---|---|
| **`uart_proto.py`** | 协议库：打包/解析/异常处理 | **已自测 31/31 PASS** |
| **`fpga_link.py`** | 联调工具：假视觉 + 收发统计 | **dry-run 已验证** |
| **`detect_from_image.py`** | 颜色识别骨架：图片 → 目标 → 协议帧 | **已验证**（`--demo-frame` 不需要 OpenCV） |
| `README.md` | 本文件 | — |

---

## 快速开始（今天就能做，不需要买任何东西）

```bash
cd arm

# ① 跑协议库自测
python uart_proto.py
#   -> RESULT: PASS  (31 项)

# ② 看帧长什么样（不需要 pyserial）
python fpga_link.py --dry-run

# ③ 演示"目标 -> 协议帧"（不需要图片，也不需要 OpenCV）
python detect_from_image.py --demo-frame

# ④ 造一张测试图并识别（需要 Pillow 或 OpenCV）
python detect_from_image.py --make-test-image
python detect_from_image.py --image test.jpg --debug out.jpg
```

**`--dry-run` 会打印**：

```
--- 一个目标 ---
  AA 55 01 02 00 08 01 00 01 4C CC 7F FF 58 53 55 AA
    帧头   AA 55
    版本   01
    类型   02  (VISION_RESULT)
    长度   00 08  = 8 字节
    数据   01 00 01 4C CC 7F FF 58
    校验   53
    帧尾   55 AA
    总长   17 = 9 + 8  ✓
```

**这一步就能确认**：帧格式、BCC 范围、长度计算**全部正确**。

---

## 在代码里用

```python
from uart_proto import (
    make_vision_result, FrameParser, parse_telemetry,
    CLS_PERSON, CLS_LOW_OBSTACLE, norm_to_u16,
)

# ---- 发送 ----
targets = [
    (CLS_PERSON, norm_to_u16(cx_px, 640), norm_to_u16(cy_px, 480), 88),
]
ser.write(make_vision_result(targets))

# ---- 接收 ----
parser = FrameParser()
while True:
    for ftype, payload in parser.feed(ser.read(ser.in_waiting or 1)):
        if ftype == TYPE_TELEMETRY:
            d = parse_telemetry(payload)
            print(d['state_name'], d['speed_left'], d['dist_mm'])
```

---

## 接 FPGA 联调

```bash
pip install pyserial
python fpga_link.py --port COM5 --baud 115200
```

**接线只有 3 根**：

```
ARM TX   ──────►  FPGA uart_rx
ARM RX   ◄──────  FPGA uart_tx
ARM GND  ───────  FPGA GND      ← ★ 必须共地，否则通信全乱
```

**⚠️ 电平**：树莓派 / K230 / Luckfox 的 GPIO UART 都是 **3.3V**，可以直接接。
如果你用 5V 的 USB-TTL 调试，**必须加电平转换**，否则会打死 FPGA 引脚。

### 不想等 FPGA？用虚拟串口对自测

装 com0com（Windows）或 `socat`（Linux），然后开两个终端：

```bash
# 终端 A（模拟 ARM）
python fpga_link.py --port COM10

# 终端 B（模拟"回显对端"）
python fpga_link.py --port COM11 --listen-only
```

---

## `--pattern` 假目标模式

| 模式 | 效果 | 用来测什么 |
|---|---|---|
| `sweep` | 一个目标左右扫（默认） | 看链路在动、坐标在变 |
| `person` | 固定"人员"在正前方 | FPGA 应减速 |
| `obstacle` | 固定"障碍物" | FPGA 应绕行 |
| `low_obstacle` | 固定"低矮障碍"（画面下方） | **FPGA 应触发升降腿** |
| `multi` | 三个目标同时出现 | 多目标负载 |
| `none` | 空目标（0 个） | **验证 FPGA 的 fail-safe**（应限速直行，不停车） |

```bash
python fpga_link.py --dry-run --pattern low_obstacle
python fpga_link.py --port COM5 --pattern multi --verbose
```

---

## 协议速查

```
帧 = AA 55 | VER | TYPE | LEN(2B大端) | DATA | BCC | 55 AA
     └──────── BCC 计算范围 ────────┘

帧总长 = 9 + N          ← ⚠️ 是 9 不是 10
BCC    = VER ^ TYPE ^ LEN_H ^ LEN_L ^ DATA[0..N-1]     ← 不含帧头帧尾
```

| 类型 | 值 | 方向 | 载荷长 |
|---|---|---|---|
| `VISION_RESULT` | `0x02` | ARM→FPGA | 1 + 7n |
| `TELEMETRY` | `0x10` | FPGA→ARM | **13** |
| `ACK` | `0x03` | 双向 | 1 |
| `CONFIG` | `0x04` | 双向 | 变长 |

**类别编码**（`VISION_RESULT` 里用）：

| 值 | 含义 | FPGA 动作 |
|---|---|---|
| `0x00` | 无 | — |
| `0x01` | 人员 | 减速 + 保持距离 |
| `0x02` | 障碍物 | 绕行 |
| `0x03` | 目标点/终点 | 向目标行进 |
| **`0x04`** | **低矮障碍** | **触发升降腿越障** |

**坐标必须归一化到 0..65535**（`norm_to_u16()` 帮你算），**不要发像素坐标** ——
这样换摄像头分辨率时 FPGA 侧代码不用改。

---

## 异常处理（已和 FPGA 侧逐条对齐）

`FrameParser` 的行为**刻意与 FPGA 的 `uart_frame.v` 一致**（该模块 58 项仿真检查全过）：

| 情况 | 行为 |
|---|---|
| BCC 错 | 跳过这 2 字节帧头继续找 —— **后续正常帧照样收到** |
| 帧尾错 | 同上 |
| 未知类型 | 完整收完后丢弃（**保持字节同步**） |
| 长度超上限 | 标记丢弃，仍按长度收完 |
| 前导噪声 | 任意垃圾字节（含连续 `AA`）自动跳过 |

**唯一的差别**：RTL 有 20ms 帧内超时复位，Python 侧是流式处理、没有时间戳概念。
实际影响很小（半截帧最多占住 `MAX_PAYLOAD` 字节就会因长度/校验不符被丢弃）。

---

## 统计与误帧率

```python
parser.stats        # {'frames_ok':…, 'bcc_err':…, 'tail_err':…, 'noise_bytes':…}
parser.error_rate   # 0.0 ~ 1.0
```

`fpga_link.py` 每 5 秒打印一次，**误帧率 > 1% 会提示**。

> 按联调清单：连续跑 10 分钟统计误帧率，**若 > 1%，升级到 CRC16 并把版本升到 `0x02`**。

---

## 下一步（板子到货后）

`fpga_link.py` 的 `FakeVision` 类就是**要替换的那一块**：

```python
# 现在：假目标
class FakeVision:
    def targets(self, t): ...

# 板子到后：换成真摄像头
class CameraVision:
    def targets(self, t):
        frame = camera.capture()
        blobs = detect_color(frame)          # OpenCV / CanMV
        return [(CLS_PERSON, norm_to_u16(b.cx, W), norm_to_u16(b.cy, H), b.conf)
                for b in blobs]
```

**只要 `targets()` 返回的格式一样，主循环一行都不用改。**

---

## 相关文档

- [`doc/uart_frame_protocol.md`](../doc/uart_frame_protocol.md) —— 协议完整定义
- [`doc/interface_delivery.md`](../doc/interface_delivery.md) —— 接口全貌 + 你的职责
- [`handoff/B_arm.md`](../handoff/B_arm.md) —— **你的任务清单和 DSH 提示词**
