# ARM 核（K230）代码与交接包

> 打包日期：**2026-10-18**
> 对应 FPGA 交付：`robot_fpga_release_20261017`（`ffbab92`）
> 板卡：庐山派 **Lite-K230D-128M**，CanMV **v1.8**，摄像头 **GC2093（CSI2）**
> 详细交接说明见 **`docs/ARM_STATUS.md`**，本文件只是索引和最快上手。

---

## 1. 包里有什么

```
arm_k230_release_20261018/
├─ arm/                        # 全部 ARM 侧 Python 代码（放同一目录，import 才通）
│  ├─ camera_vision.py         # K230 主程序：取流 -> 识别 -> UART 发 VISION_RESULT
│  ├─ board_bringup.py         # 板端自检：pins / listen / send / loopback
│  ├─ uart_proto.py            # PC 协议参考实现（FPGA 侧按它对齐，接口不要改）
│  ├─ fpga_link.py             # PC 假视觉联调工具（--dry-run / --pattern step）
│  ├─ detect_from_image.py     # PC 图片 HSV 识别 demo（需要 OpenCV）
│  ├─ test_camera_offline.py   # camera_vision 的 PC 离线回归（47 项）
│  ├─ test_serial_loopback.py  # 虚拟/真实串口闭环测试（需要 pyserial）
│  ├─ _test_nocv.py            # 可选：验证没装 OpenCV 时的提示
│  └─ assets/                  # test_red.png / debug_test_red.png 测试图
├─ docs/
│  ├─ ARM_STATUS.md            # ★ ARM 侧交接主文档（现状/上板步骤/接口问题/待办）
│  ├─ arm_bringup.md           # FPGA 侧给的 K230 接线与联调手册
│  ├─ uart_frame_protocol.md   # UART 帧协议全文
│  └─ evidence/                # cam_snapshot.jpg（原图）/ cam_detect.jpg（识别框）
├─ requirements-pc.txt         # PC 侧依赖（pyserial / OpenCV）
├─ SHA256SUMS.txt              # 全包文件校验清单
└─ README.md                   # 本文件
```

**没有包含**：`backup_before_20261017/`（旧版本备份，留在原 `D:\ARM`）、
`__pycache__/`、TF 卡固件镜像。

---

## 2. 最快上手（PC，不需要硬件）

```bash
cd arm

python uart_proto.py                 # 协议库自测 -> PASS (31 项)
python test_camera_offline.py        # camera_vision 离线回归 -> PASS (47 项)
python fpga_link.py --dry-run        # 看帧长什么样
python fpga_link.py --dry-run --pattern step   # 0x05 台阶帧
```

需要 pyserial 的脚本（`fpga_link.py --port`、`test_serial_loopback.py`）：

```bash
pip install -r requirements-pc.txt
```

---

## 3. 上 K230 板子

把 `arm/camera_vision.py` 和 `arm/board_bringup.py` 传到 `/sdcard/`，
在 CanMV IDE 或 REPL 里：

```python
import board_bringup
board_bringup.run(['pins'])       # 看接线
# 拔掉 FPGA 两根线，短接排针 11 <-> 13：
board_bringup.run(['loopback'])   # 应 RESULT: PASS
# 接回 FPGA：
board_bringup.run(['listen', '3'])# 应收到 10Hz TELEMETRY
# FPGA 按 S0 进 AUTO 后：
board_bringup.run(['send'])       # 障碍->AVOID，低矮->LIFT，空目标->AUTO
```

主程序：

```python
import camera_vision
camera_vision.main()
```

本包**没有** `/sdcard/main.py`，所以不会开机自启。需要脱机自启时新建：

```python
# /sdcard/main.py
import camera_vision
camera_vision.main()
```

---

## 4. 接线（3 根，3.3V 直连）

| K230 | 方向 | FPGA |
|---|---|---|
| GPIO05（排针 **11**） | TX → | **F16**（`uart_rx`） |
| GPIO06（排针 **13**） | RX ← | **C19**（`uart_tx`） |
| GND（排针 6/9/14/20/25/30/34/39 任一） | ↔ | GND |

⚠️ 代码里填的是 **GPIO 号 5/6**，不是排针号 11/13。
⚠️ 必须共地；波特率 115200 8N1。

---

## 5. 已验证（2026-10-17 ~ 10-18）

| 项 | 结果 |
|---|---|
| PC 离线回归 | `test_camera_offline.py` **47/47 PASS** |
| 协议库自测 | `uart_proto.py` **31/31 PASS** |
| 帧格式 | 与 `uart_proto` 逐字节一致，手册三条向量全对 |
| K230 摄像头 | `gc2093_csi2` 识别正常，640×480 出图 |
| 红色目标 | 10/10 帧识别为 `0x02`，整瓶 39k~48k 像素 |
| 帧率 | 关预览稳态 **10.0Hz**；前 5 帧 2.2Hz 是启动开销 |
| 串口 | UART2 GPIO5/6 初始化正常；**FPGA→ARM 遥测仍为 0 帧**（FPGA 侧未发） |

阈值修正：红瓶实测 `A_min=30` 会切掉暗红部分，已改为
`(0,100,20,127,-20,80)` + `merge margin=15`。

---

## 6. 三个必须知道的接口事实

1. **`0x03 目标点` FPGA 没实现**：`main_fsm.v` 只认 `0x02/0x04/0x05`，
   GOAL 由 `goal_reached`（`E16`，低有效）触发。ARM 不要发 0x03 指望导航。
2. **`0x05 台阶` FPGA 已实现**（`VIS_STEP` -> LIFT 模式 1），
   但旧协议文档只写到 0x04；本包 `uart_proto.py` 已补 `CLS_STEP`。
3. **手册 §4 低矮障碍向量置信度是 90（0x5A）不是 88**，
   用 88 生成会得到另一条合法帧，但对不上手册。

---

## 7. 文件校验

```powershell
Get-FileHash .\arm\camera_vision.py -Algorithm SHA256
# 与 SHA256SUMS.txt 对照
```

包内所有文件的 SHA256 在 `SHA256SUMS.txt`，格式为
`<sha256>  <相对路径>`。
