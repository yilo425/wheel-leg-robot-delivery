# ARM 侧现状与交接（2026-10-17 更新）

> 对应 FPGA 交付：`robot_fpga_release_20261017`（`ffbab92`）
> 对应 ARM 板：庐山派 **Lite-K230D-128M**，CanMV **v1.8**，摄像头 GC2093（CSI2）
> 本文档是本轮改动的说明 + 上板步骤 + 已发现的接口问题。

---

## 0. 一句话现状

串口链路 FPGA 侧已实测双向通（`arm_bringup.md` §6），**但 ARM 侧代码还写着 GPIO11/12**，
与实机接线（GPIO5/6）不一致 —— 这是本轮修掉的第一个、也是最关键的问题。
修完后 PC 侧 47 项离线回归全过；**摄像头 + 10Hz 还需要上板实测**。

---

## 1. 本轮改了什么

| 文件 | 改动 | 为什么 |
|---|---|---|
| **`camera_vision.py`** | UART 引脚 `11/12` → **`5/6`** | 实机是 GPIO5(排针11)/GPIO6(排针13)；旧代码跑通也不会有波形 |
| | 补 `MediaManager.init()/deinit()` | K230 官方例程必须初始化媒体缓冲，否则 `snapshot()` 可能一直失败 |
| | 遥测接收不再强依赖 `uart.any()` | K230 UART API 文档里没有 `any()`；没有时退化为 `timeout=0` 的 `read()` |
| | 新增 `DETECT_RULES` 规则表 | 一个颜色/区域 → 一个类别，可加"画面下部 / 宽高比"条件，支持 0x04、0x05 |
| | 置信度改为"2% 画面 = 100 分" | 旧口径下发 40×40 红块置信度只有 0~1，现场看起来像坏了（FPGA 目前不用它） |
| | 新增实测帧率统计 | 每 50 帧打印真实 Hz，低于 8Hz 告警；可量化 Display 对 10Hz 的影响 |
| | 新增 `RUN_FRAMES`、发送失败计数 | 离线回归能完整跑 `main()`；串口故障不再拖崩视觉节点 |
| **`uart_proto.py`** | 新增 `CLS_STEP = 0x05` | FPGA `main_fsm.v` 已实现 `VIS_STEP`，旧协议文档只写到 0x04 |
| **`fpga_link.py`** | 增加 `--pattern step` | PC 侧可直接发 0x05 测 FPGA 两腿分段抬升 |
| **`board_bringup.py`** | **新增** | 不接摄像头的板端自检：listen / send / loopback / pins |
| **`test_camera_offline.py`** | **新增** | PC 离线回归 47 项（假硬件 + 真 `uart_proto` 解析器） |
| 本文档 | **新增** | 交接说明 |

---

## 2. ⚠️ 三个必须知道的接口事实

### 2.1 `0x03 目标点` FPGA **没有实现**

`doc/uart_frame_protocol.md` 和 `arm_bringup.md` 都写"0x03 → GOAL"，但
`rtl/app/main_fsm.v` 里只有 `VIS_OBST(0x02)`、`VIS_LOW_OBST(0x04)`、`VIS_STEP(0x05)`，
**没有** `VIS_GOAL`。GOAL 状态只由 `goal_reached` 引脚（`E16`，低有效）触发。

→ **ARM 侧现在不要发 0x03 指望导航**。要么 FPGA 侧补 `VIS_GOAL`，要么继续用终点限位。
这是文档与 RTL 不一致，不是 ARM 侧能单方面修的。

### 2.2 `0x05 台阶` FPGA 已实现，但文档没写

`main_fsm.v` / `top.v` 已有 `VIS_STEP = 8'h05`，触发 LIFT 模式 1（两腿分段）。
`uart_proto.py` 本轮已补 `CLS_STEP`；`camera_vision.py` 的规则表可以映射到它。

### 2.3 手册 §4 的低矮障碍向量置信度是 90，不是 88

```
AA 55 01 02 00 08 01 00 04 80 00 80 00 5A 54 55 AA
                                            ^^ conf=0x5A=90
```
用 88 生成会得到 `... 58 56 ...`，不是手册那条。两条都合法，但**别拿 88 去对 90 的那条**。
`board_bringup.py` 已按 90 生成并逐字节校验。

---

## 3. 文件清单

| 文件 | 跑在哪 | 作用 |
|---|---|---|
| `camera_vision.py` | K230 | 主程序：取流 → 规则识别 → VISION_RESULT 10Hz + 收 TELEMETRY |
| `board_bringup.py` | K230 | 不接摄像头的串口自检（先跑它，再跑主程序） |
| `uart_proto.py` | PC | 协议参考实现（FPGA 侧已按它对齐，**对外接口不要改**） |
| `fpga_link.py` | PC | 假视觉联调工具（`--dry-run` / `--pattern step`） |
| `detect_from_image.py` | PC | 本地图片 HSV 识别 → 帧（现场阈值预演） |
| `test_camera_offline.py` | PC | `camera_vision.py` 离线回归（47 项） |
| `test_serial_loopback.py` | PC | 虚拟串口/真实串口闭环（需 pyserial） |

---

## 4. 上板步骤（按顺序，别跳）

### ① 先证明串口和引脚对（不接摄像头）

把 `camera_vision.py`、`board_bringup.py` 传到 `/sdcard/`，在 CanMV IDE 里：

```python
import board_bringup
board_bringup.run(['pins'])       # 看接线表
```

**回环自测**（拔掉接 FPGA 的 TX/RX 两根线，用杜邦线短接排针 **11 ↔ 13**）：

```python
board_bringup.run(['loopback'])
```

期望：三条手册向量都 `[OK]`，最后打印 `RESULT: PASS`。
失败优先查：短接的是不是 11/13、代码里是不是 GPIO5/6、UART2 是否被别的程序占用。

### ② 接回 FPGA，收遥测

```python
board_bringup.run(['listen', '3'])
```

期望：`10Hz` 左右收到 `TELEMETRY`，状态在 `S_IDLE/S_AUTO`。
一帧都没有 → 查 GND、TX/RX 是否交叉、FPGA 是否在跑。

### ③ 发三条视觉向量，看 FPGA 动作

先按 FPGA 的 **S0** 进 AUTO，然后：

```python
board_bringup.run(['send'])
```

| 发出 | 期望 FPGA |
|---|---|
| `0x02` 障碍物 | AUTO(2) → **AVOID(3)** |
| `0x04` 低矮障碍 | **LIFT(4)**，舵机动作 |
| 空目标 | 回 **AUTO(2)** |

### ④ 再上摄像头

```python
import camera_vision
camera_vision.main()
```

正常应看到：
- `UART2 就绪: TX=GPIO5(排针11), RX=GPIO6(排针13)`
- 每帧一行 `#N 红色障碍物 类别0x02 ... 帧17B`
- 每 5 秒一行 `[性能] 最近 50 帧 X.X Hz`，应 ≥ 9Hz
- 有遥测时穿插 `[RX] 状态=...`

### ⑤ 比赛前关预览

`USE_DISPLAY = False`，再确认帧率。VIRT 预览走 USB，640×480 下会明显吃 CPU/USB。

---

## 5. 现场调阈值 / 加类别

### 5.1 调颜色

改 `camera_vision.py` 顶部 `DETECT_RULES` 里的 LAB 六元组：

```python
DETECT_RULES = [
    ('红色障碍物', CLS_OBSTACLE, [(0, 100, 30, 127, -10, 70)],
     None, None, None, None),
]
```

最省事的办法：CanMV IDE → 工具 → 机器视觉 → **阈值编辑器**，框住目标后抄右上角的元组。
LAB 不是 HSV，PC 版 `detect_from_image.py` 的 HSV 阈值**不能直接搬**。

### 5.2 加"低矮障碍 → 抬腿"（0x04）

低矮障碍通常**贴地、在画面下方、横向比纵向宽**。只按颜色会和普通障碍抢同一块红色，
误触发抬腿很危险，所以建议加几何条件，并放在普通红色规则**前面**：

```python
DETECT_RULES = [
    ('红色低矮障碍', CLS_LOW_OBSTACLE, [(0, 100, 30, 127, -10, 70)],
     0.60, 1.0, 1.2, None),          # y≥60%, 宽/高≥1.2
] + DETECT_RULES
```

`0.60` / `1.2` 必须用比赛场地实拍图调；先开 `DEBUG_PRINT` 看打印的
`占比 / 中心 / 规则名`，再决定阈值。

### 5.3 加"台阶 → 0x05"

```python
DETECT_RULES = [
    ('蓝色台阶', CLS_STEP, [(0, 100, -128, -20, -128, 127)],
     None, None, None, None),
] + DETECT_RULES
```

阈值换成台阶的实际颜色。FPGA 侧会走 LIFT 模式 1（两腿分段）。

### 5.4 只想测抬腿、不想等识别

临时设 `FORCE_CLASS = 0x04`，所有识别结果都会变成低矮障碍；测完务必改回 `None`。

---

## 6. 已验证（PC 离线 + K230 实机）

### 6.1 PC 离线

```
python test_camera_offline.py     -> RESULT: PASS  (47 项)
python uart_proto.py              -> RESULT: PASS  (31 项)
python test_serial_loopback.py --mode fake --frames 30  -> PASS (30/30, 误帧率 0%)
python fpga_link.py --dry-run     -> 协议库工作正常
```

`test_camera_offline.py` 覆盖：

- 引脚回归（必须是 GPIO5/6，防止有人改回 11/12）
- `pack_frame` / `make_vision_result` 与 `uart_proto` **逐字节**一致（25 + 5 组）
- 手册 §4 三条实测向量逐字节一致
- `norm_to_u16` 在 0..640 / 0..480 **每个像素点**都与 PC 版一致
- 遥测解析逐字段一致；逐字节喂、坏帧后重同步、前导噪声都通过
- 规则分类：贴地扁宽红块 → 0x04，画面中上部方块 → 0x02，低于门限 → 丢弃
- 完整 `main()` 端到端（假摄像头 + 假串口）：3 帧 VISION_RESULT、空目标帧、初始化顺序
- `Display.init → MediaManager.init → sensor.run` 顺序

### 6.2 K230 实机（2026-10-17，COM5）

- `find sensor gc2093_csi2, type 25, output 1280x960@90`，CSI2 通；
- 640×480 取流、JPEG 编码正常；实拍图能拉回 PC；
- 红色目标（红瓶）10/10 帧识别为 `0x02`，稳定发 17 字节 VISION_RESULT；
- 稳态 **10.0Hz**（关预览）；前 5 帧约 2.2Hz 是 sensor 启动开销；
- **阈值修正**：原 `A_min=30` 会把暗红瓶身切掉，只框到 17k 像素；
  改成 `A_min=20`、`B=-20..80`、`merge margin=15` 后，整瓶 39k~48k 像素，
  中心抖动约 ±10px，能稳定框住目标；
- UART RX 仍是 0 帧 TELEMETRY —— FPGA 侧未在发（与 ARM 无关）。

**仍未验证**：真实赛道光照下的阈值、低矮障碍 0x04、舵机/电机联动、
连续 10 分钟误帧率、`USE_DISPLAY=True` 时的实际帧率。

---

## 7. 待办 / 风险

| 项 | 状态 | 说明 |
|---|---|---|
| 真机 10Hz | ✅ 关预览时 | 实测 10.0Hz；开 `USE_DISPLAY` 时还要复测 |
| 红色阈值 | ✅ 红瓶 | `A_min=20/B=-20..80/margin=15`；赛道光照还要复调 |
| 低矮障碍阈值 | ⬜ | 需要场地实拍；`0.60 / 1.2` 只是起点 |
| 0x03 GOAL | ❌ FPGA 未实现 | 需和 FPGA 侧确认：补 `VIS_GOAL` 还是继续用 `goal_reached` |
| 连续 10 分钟误帧率 | ⬜ | 目标 < 1%；`camera_vision` 退出时会打印接收统计 |
| TOF / IMU / 编码器 | ⬜ 等硬件 | telemetry 里 `dist/spd/roll/pitch/yaw = 0` 是正常的 |
| 镜头方向 | ⬜ | 装反了把 `USE_MIRROR = True` |

---

## 8. 给 FPGA 侧同学的两条反馈

1. `main_fsm.v` 没有 `VIS_GOAL`，但协议文档/交接书写了 `0x03 → GOAL`。
   要么补逻辑，要么把文档改成"0x03 预留，GOAL 由 `goal_reached` 触发"。
2. `uart_frame_protocol.md` 的类别表只写到 `0x04`，RTL 已有 `0x05 VIS_STEP`；
   建议补一行，避免下一个人以为 0x05 会被静默丢弃。
