# 交付版说明（2026-11-03）

> 本目录是**演示/交付版**，与 `robot_project/`（生产版）相互独立。
> 生产版代码**未做任何修改**，后续真实改动仍在生产版上进行。
> 本版按视频演示效果整理，改动方案与原代码保持一致风格，不虚构架构。

---

## 一、这版改了什么（5 条）

| # | 改动 | 涉及文件 |
|---|---|---|
| 1 | 遥控面板只保留 **开始 / 停止** | `arm/wifi_rc.py`、`arm/main.py` |
| 2 | 障碍物检测：超声 50~80cm → 停车 → 复测 → 摄像头判高 → 抬腿或绕行 | `rtl/app/main_fsm.v`、`rtl/top.v`、`camera_vision.py` |
| 3 | 抬腿/降腿角度改 **45°**（带角度修正） | `rtl/app/config_rx.v` |
| 4 | 左右转 = **原地差速 45°**（沿用已验证的差速方案） | 无需改动（`motion_decision.v` 已是该方案） |
| 5 | 大障碍：测宽 → 转向 → 跑 1s → 停 → 反方向转 → 直行 | `rtl/app/main_fsm.v` |

---

## 二、逐条说明

### 1) 遥控面板：只有【开始 / 停止】

```
改动前：前进/后退/左转/右转/停 + 升高/降低 + 速度滑条
改动后：开始 / 停止 两个按钮（外加状态显示）
```

- **开始** → 发 `CONFIG 0x22 = 1`
- **停止** → 发 `CONFIG 0x22 = 0`
- 开机不再进手动模式（`CONFIG 0x20 = 0`，即自动模式），全程自动驾驶。

FPGA 侧 `top.v` 把网页命令和物理按键做"或"：

```verilog
assign start_pulse = (物理 key 下降沿) | start_cfg;   // 开始：每 500ms 一个脉冲
assign estop_sig   = (物理急停)        | ~cfg_run_cmd; // 停止：保持 estop
```

> 为什么开始要连发脉冲：状态机从 ESTOP 要先回 IDLE 再进 AUTO，
> 一个脉冲只能走一步。每 500ms 补一个脉冲就自动走完两步；
> 进了 AUTO 之后 start 不再被消费，没有副作用。

### 2) 障碍物流程

```
AUTO 前进
  │  TOF < 800mm（50~80cm 区间）
  ▼
S_CHECK  停车（等摄像头判高度，超时 1.5s）
  │  摄像头：读遥测里的 dist_mm（停车后的复测值）
  │           + 自身安装高度 12cm
  │           + 障碍物在画面里的像素高
  │           → 实物高度 ≈ 像素高 × 距离 / 焦距(554px)
  ▼
  高度 ≤ 9cm ──► S_LIFT   （抬腿跨过）
  高度 > 9cm ──► S_AVOID  （绕行）
```

**摄像头判高实现**（`camera_vision.py`）：

```python
CAM_HEIGHT_MM    = 120   # 安装高度（低位 12cm）
CAM_FOCAL_PX     = 659   # 焦距（官方镜头 H65°/V40° 在 640x480 下换算）
OBST_LIFT_MAX_MM = 90    # >9cm 必须绕行

def estimate_obst_height_mm(blob, dist_mm):
    return int(blob.h() * dist_mm / CAM_FOCAL_PX)
```

只在 FPGA 处于 `S_CHECK`（状态码 7）且距离在 30~90cm 时才判高，
其它时候仍按颜色分类，不影响正常巡线。

**抬腿流程**（`main_fsm.v` 的 `S_LIFT`，模式 0）：

```
LP_UP    抬腿到 45°（等 lift_done，舵机没到位不启动下一步）
LP_CROSS 保持抬起，全速直行 1s（LIFT_CROSS_MS）
LP_DOWN  降腿（等 600ms 走完行程）
         → 回 AUTO
```

### 3) 抬腿 45°

`config_rx.v` 里 `DEF_LIFT_HEIGHT = 25`（原来是 100）。

PCA9685 计数换算：`lift = COUNT_MIN + height × COUNT_PER_H = 102 + 25×4 = 202`
≈ 1.0ms 脉宽 ≈ 45°。

角度修正沿用 `stance_ctrl.v` 的 `LEVEL_CNT`（站立高度那一套，未改）。

### 4) 转弯 45°

沿用已验证的差速方案（`motion_decision.v`，一侧正转一侧反转），
转角 45° 由转向时长决定。`main_fsm.v` 里：

```verilog
parameter integer TURN_MS = 600;   // 45° 转向时长（现场可标定）
```

### 5) 大障碍绕行

```
S_AVOID 四阶段：
  AP_TURN1  转向 45°（方向由摄像头给的 vr_target_x 决定）
  AP_RUN    直行（全速），时长 = 障碍物实测宽度 / 最大速度 + 0.2s
  AP_STOP   停车 0.3s
  AP_TURN2  反方向转 45°（恢复朝向）
           → 回 AUTO 直行
```

**直行时长不再是固定 1s**，而是按障碍物实际宽度算：

```
run_ms = 宽度(mm) / V_MAX_MMPS(1257mm/s) × 1000 + 200ms 余量
```

- 宽度由摄像头用同一套针孔模型测：`blob.w() × 距离 ÷ 焦距`
- 200ms 余量 = 起步加速 + 收尾减速的时间
- 算好后摄像头发 `CONFIG 0x23`（2 字节，毫秒，钳位 200~10000），
  FPGA 的 `AP_RUN` 用这个值；没收到就退回默认 1000ms

举例：障碍物宽 600mm → 600/1257×1000+200 ≈ 677ms；
宽 1500mm → 1193+200 ≈ 1393ms。障碍物越大，跑得越久，不会蹭上去。

转向方向：

```verilog
n_turn_1st = (vr_target_x >= 16'h8000) ? 右转 : 左转;
//  摄像头给的 x 中心 >= 画面中心 -> 右转；否则左转（居中按右转）
```

---

## 三、参数速查（都在文件顶部，现场可调）

| 参数 | 位置 | 值 | 含义 |
|---|---|---|---|
| `DIST_CHECK_MM` | main_fsm.v | 800 | 检测到多近就停车检查 |
| `CHECK_TIMEOUT_MS` | main_fsm.v | 1500 | 等摄像头判高的超时 |
| `TURN_MS` | main_fsm.v | 600 | 45° 转向时长 |
| `RUN_MS` | main_fsm.v | 1000 | 绕行直行时长**默认值**（摄像头会用 0x23 覆盖）|
| `V_MAX_MMPS` | camera_vision.py | 1257 | 满速（mm/s），用于算绕行时长 |
| `RUN_MARGIN_MS` | camera_vision.py | 200 | 绕行时长的起步/减速余量 |
| `AV_STOP_MS` | main_fsm.v | 300 | 绕行中间停车时长 |
| `LIFT_CROSS_MS` | main_fsm.v | 1000 | 抬腿后越障直行时长 |
| `LIFT_AUTO_TIMEOUT_MS` | main_fsm.v | 3000 | 抬腿到位超时 |
| `DEF_LIFT_HEIGHT` | config_rx.v | 25 | 抬腿高度（≈45°）|
| `CAM_HEIGHT_MM` | camera_vision.py | 120 | 摄像头安装高度 |
| `CAM_FOCAL_PX` | camera_vision.py | 659 | 焦距（像素），按官方 H65°/V40° 换算 |
| `OBST_LIFT_MAX_MM` | camera_vision.py | 90 | 可抬腿的最大障碍高度 |

---

## 四、编译

```bash
cd tools/fpga_build
gw_sh build_delivery.tcl      # 本目录专用（路径已指向 delivery_20261103）
```

> `build_all.tcl` 里的路径是写死指向生产目录的，**本目录不要用它**，
> 用 `build_delivery.tcl`。

ARM 侧把 `arm/wifi_rc.py`、`arm/main.py`、`camera_vision.py` 拷到 K230 的 `/sdcard/`。

---

## 五、与生产版的差异清单

| 文件 | 改动 |
|---|---|
| `rtl/app/config_rx.v` | +`run_cmd`（0x22）、`DEF_LIFT_HEIGHT` 100→25 |
| `rtl/app/main_fsm.v` | +`S_CHECK` 状态、+绕行四阶段、+`turn_dir`/`vr_target_x` 端口、抬腿改 45°+全速 1s |
| `rtl/top.v` | 网页开始/停止接入 start/estop、`turn_dir` 覆盖视觉转向 |
| `arm/wifi_rc.py` | 面板改开始/停止、开机自动模式、`/cmd?run=` 支持 |
| `arm/k230_.../camera_vision.py` | 新增高度估计 + S_CHECK 时判高 |
| `tools/fpga_build/build_delivery.tcl` | 新增（指向本目录的构建脚本）|

其余文件与生产版一致。
