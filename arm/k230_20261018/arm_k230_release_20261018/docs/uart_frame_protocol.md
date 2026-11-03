# FPGA ↔ ARM 通信帧协议

> 用途：摄像头视觉结果（ARM → FPGA）与调试/状态回传（FPGA → ARM）
> 物理层：UART，115200 8N1，3.3V TTL，**必须共地**
> 版本：v1.0

---

## 1. 帧结构

```
+--------+--------+--------+--------+--------+---------+--------+--------+
| 帧头   | 版本   | 帧类型 | 长度   | 数据   | 校验    | 帧尾   |        |
| 2B     | 1B     | 1B     | 2B     | NB     | 1B      | 2B     |        |
| 0xAA55 | 0x01   | 见下表 | N      | N 字节 | BCC     | 0x55AA |        |
+--------+--------+--------+--------+--------+---------+--------+--------+
```

| 字段 | 长度 | 说明 |
|---|---|---|
| 帧头 | 2B | 固定 `0xAA 0x55`（高字节在前） |
| 版本 | 1B | 协议版本，当前 `0x01` |
| 帧类型 | 1B | 见 §2 |
| 长度 | 2B | **数据段**字节数 N（不含头/尾/校验），大端 |
| 数据 | NB | 载荷 |
| 校验 | 1B | BCC：从版本到数据最后一个字节的逐字节异或 |
| 帧尾 | 2B | 固定 `0x55 0xAA` |

### 字节序
- 多字节字段一律**大端**（高字节在前），与网络序一致，ARM 侧 Python 用 `struct.pack('>H', n)`。
- 图像/二进制载荷本身是字节流，不涉及字节序。

### 校验算法（BCC）

```c
uint8_t bcc(const uint8_t *buf, int n) {   // buf 从"版本"字节开始
    uint8_t x = 0;
    for (int i = 0; i < n; i++) x ^= buf[i];
    return x;
}
```

```python
def bcc(payload_after_header: bytes) -> int:
    x = 0
    for b in payload_after_header:
        x ^= b
    return x
```

> 校验范围是 **版本 + 帧类型 + 长度 + 数据**，不含帧头帧尾。
> 选 BCC 而不是 CRC16 的原因：FPGA 侧实现只要一个异或，时序压力为零；
> UART 本身有起始/停止位同步，短帧出错率很低。若后期发现误帧率高，
> 再升级 CRC16-CCITT，并同时升版本号到 0x02。

---

## 2. 帧类型定义

| 值 | 名称 | 方向 | 载荷 | 说明 |
|---|---|---|---|---|
| 0x01 | `IMAGE_DATA` | FPGA → ARM | 见 §3 | 图像行/块数据（预留） |
| 0x02 | `VISION_RESULT` | ARM → FPGA | 见 §3 | 目标检测结果 |
| 0x03 | `ACK` | 双向 | 1B 被确认的帧类型 | 确认收到 |
| 0x04 | `CONFIG` | 双向 | 见 §3 | 参数配置 |
| 0x10 | `TELEMETRY` | FPGA → ARM | 见 §3 | 状态回传（调试用） |

> 0x01 图像数据帧在本方案里基本不会用到：图像处理放在 ARM 侧本地完成，
> 不需要把整帧图像送进 FPGA。保留该类型是为了兼容"FPGA 侧做图像预处理"
> 的后备路线（例如 FPGA 先做灰度/二值化再送 ARM）。

---

## 3. 载荷格式

### 3.1 `0x02 VISION_RESULT`（ARM → FPGA）

```
+--------+--------+--------+--------+--------+--------+
| 目标数 | 序号   | 类别   | x 中心 | y 中心 | 置信度 |
| 1B     | 1B     | 1B     | 2B     | 2B     | 1B     |
+--------+--------+--------+--------+--------+--------+
   每个目标 7 字节，重复 "目标数" 次
```

| 字段 | 范围 | 说明 |
|---|---|---|
| 目标数 | 0..8 | 0 表示本帧无目标 |
| 序号 | 0..7 | 目标索引 |
| 类别 | 见下表 | 目标类别编码 |
| x 中心 | 0..65535 | 目标框中心 x，归一化到 0..65535（与分辨率解耦） |
| y 中心 | 0..65535 | 目标框中心 y |
| 置信度 | 0..100 | 百分比，0..100 |

类别编码：

| 值 | 含义 | FPGA 侧对应动作 |
|---|---|---|
| 0x00 | 无 | — |
| 0x01 | 人员 | 减速 + 保持距离 |
| 0x02 | 障碍物 | 绕行 |
| 0x03 | 目标点/终点 | 向目标点行进 |
| 0x04 | 低矮障碍 | 触发升降腿越障 |

> **归一化坐标的意义**：ARM 侧换摄像头分辨率时，FPGA 侧代码不用改。
> FPGA 只需要 `x 中心 < 21845` 判左、`> 43690` 判右、中间判直行。

### 3.2 `0x04 CONFIG`（双向）

```
+--------+--------+--------+--------+
| 参数ID | 长度   | 值 ...          |
| 1B     | 1B     | 长度 字节       |
+--------+--------+--------+--------+
```

| 参数 ID | 含义 | 长度 | 单位 |
|---|---|---|---|
| 0x01 | 最大速度 | 2B | 编码器计数/窗口 |
| 0x02 | 避障停车距离 | 2B | mm |
| 0x03 | 测速窗口 | 2B | ms |
| 0x10 | 升降腿抬升高度 | 1B | 0..100 |
| 0x11 | 升降腿动作超时 | 2B | ms |

### 3.3 `0x10 TELEMETRY`（FPGA → ARM，10Hz）

```
+--------+--------+--------+--------+--------+--------+--------+
| 状态   | 速度L  | 速度R  | 距离   | roll   | pitch  | yaw    |
| 1B     | 2B     | 2B     | 2B     | 2B     | 2B     | 2B     |
+--------+--------+--------+--------+--------+--------+--------+
```

| 字段 | 说明 |
|---|---|
| 状态 | 主状态机状态编码（见 `interface_spec.md` §3.4） |
| 速度L/R | 左/右轮 `speed_cnt`，**有符号**，补码大端 |
| 距离 | TOF 距离，mm |
| roll/pitch/yaw | 姿态角，0.01 度/LSB，有符号补码 |

---

## 4. 收发状态机（FPGA 侧实现要点）

接收侧用字节流状态机，**绝不允许在 IDLE 之外丢弃字节后不同步复位**：

```
S_HDR1  : 等 0xAA
S_HDR2  : 等 0x55, 否则回 S_HDR1(若收到 0xAA 则停在本状态)
S_VER   : 收版本, 不匹配则回 S_HDR1
S_TYPE  : 收帧类型, 未知类型标记 "丢弃" 但仍按长度收完(保持同步)
S_LEN_H : 收长度高字节
S_LEN_L : 收长度低字节 -> 校验 N <= MAX_PAYLOAD(建议 64)
S_DATA  : 收 N 字节
S_BCC   : 收校验, 与累加值比对
S_TAIL1 : 等 0x55
S_TAIL2 : 等 0xAA -> 帧完成, 交给上层; 然后回 S_HDR1
```

**超时保护**：任何中间状态超过 `FRAME_TIMEOUT(建议 20ms @115200)` 未收到下一字节，
一律回 `S_HDR1` 并置 `frame_timeout` 标志。没有这个保护，一旦丢一个字节，
状态机可能永久卡在等长度/等校验上，现象是"通信莫名其妙死了"。

---

## 5. ARM 侧 Python 参考实现

```python
import struct

HDR = b'\xAA\x55'
TAIL = b'\x55\xAA'
VER = 0x01

def bcc(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x

def pack_frame(ftype: int, payload: bytes) -> bytes:
    body = struct.pack('>BBH', VER, ftype, len(payload)) + payload
    return HDR + body + bytes([bcc(body)]) + TAIL

def parse_frames(buf: bytearray):
    """从缓冲区里尽量多地解析出完整帧, 返回 (帧列表, 剩余缓冲)"""
    frames = []
    while True:
        i = buf.find(HDR)
        if i < 0:
            # 没有帧头, 只保留最后一个字节(可能是 0xAA)
            del buf[:-1]
            break
        if i > 0:
            del buf[:i]                    # 丢掉帧头之前的垃圾
        if len(buf) < 2 + 1 + 1 + 2:
            break                          # 头都还没收全
        ver, ftype, n = struct.unpack_from('>BBH', buf, 2)
        total = 2 + 1 + 1 + 2 + n + 1 + 2
        if len(buf) < total:
            break
        body = bytes(buf[2:2 + 1 + 1 + 2 + n])
        got_bcc = buf[2 + 1 + 1 + 2 + n]
        tail = bytes(buf[total - 2:total])
        if bcc(body) != got_bcc or tail != TAIL:
            del buf[:2]                    # 这帧坏了, 跳过这个帧头继续找
            continue
        frames.append((ftype, bytes(buf[2 + 1 + 1 + 2:2 + 1 + 1 + 2 + n])))
        del buf[:total]
    return frames, buf

# 组一个视觉结果帧: 1 个目标, 类别=人员, 中心 (0.3, 0.5), 置信度 88
def vision_result(targets):
    p = bytes([len(targets)])
    for idx, (cls, cx, cy, conf) in enumerate(targets):
        p += struct.pack('>BBHHB', idx, cls, cx, cy, conf)
    return pack_frame(0x02, p)

frame = vision_result([(0x01, int(0.30*65535), int(0.50*65535), 88)])
print(frame.hex(' '))
```

---

## 6. 联调检查清单

- [ ] 先跑**回环测试**：TX 短接 RX，Python 发什么收什么（验证 PC 侧和线序）
- [ ] FPGA 侧只发 `TELEMETRY`，Python 侧打印，确认帧头/长度/校验都对
- [ ] Python 侧发 `VISION_RESULT`，FPGA 侧用逻辑分析仪抓 UART 波形核对
- [ ] 故意改坏一个字节的校验，确认 FPGA 侧**丢弃**且不卡死（超时保护生效）
- [ ] 连续跑 10 分钟，统计误帧率；若 > 1%，升级到 CRC16
