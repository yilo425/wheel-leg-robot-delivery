# 自主版改动说明（2026-11-03 · v2）

> 目标：**去掉遥控驾驶**，车辆完全自主；其余逻辑一律不动（宁稳勿泛）。
> 本文件已按第二轮审查修订。

---

## 一、改了 5 个文件

| 文件 | 改动 |
|---|---|
| `rtl/top.v` | 删遥控链路；开关编码器闭环；start 脉冲改成上升沿发两次 |
| `rtl/app/motion_decision.v` | 只删了一处**重复声明**（逻辑没动）|
| `arm/wifi_rc.py` | 删 RC_CMD 发送；合并摄像头；加 3 秒心跳看门狗；心跳策略修正 |
| `arm/camera_vision.py` | 媒体库懒加载；CSI 自动探测；关预览；新增 `start_camera()` |
| `tools/fpga_build/build_auto.tcl` | 路径改成相对定位 |

**其余文件**（`main_fsm.v` / `config_rx.v` / `vision_rx.v` / `constraints/top.cst`）与交付版逐字节一致 ✓

> ⚠️ 上一版本文写"只改了 2 个文件"是错的，实际 5 个，已修正。

---

## 二、FPGA 侧

### ① 删掉遥控接收
- 删 `rc_rx` 例化、所有 `rc_*` wire、载荷 mux 的 `0x05` 分支
- UART 只解析 `0x02`（视觉）和 `0x04`（配置）

### ② 没有手动模式
```verilog
assign manual_sel = 1'b0;          // CONFIG 0x20 保留但不再生效
assign steer_sel        = steer_auto;
assign speed_scale_sel  = speed_scale;
assign motor_enable_sel = motor_enable_fsm;
```

### ③ 编码器闭环
```verilog
parameter integer FORCE_OPEN_LOOP = 0;   // 原为 1（强制开环）
```

### ④ start 脉冲只在「开始」上升沿发两次
```
run_cmd 0->1 : 立刻发 1 个 + 200ms 后发 1 个  -> ESTOP/IDLE -> AUTO
之后不再发   : 急停/到终点必须重新点【开始】
```

---

## 三、ARM 侧

### 摄像头合并进主程序

```
之前：main.py 只启动 wifi_rc.run()，摄像头根本没跑；
      两个脚本抢同一个 UART2(GPIO5/6)，不能同时运行。

现在：一个程序干完全部
      ① 网页（开始/停止）
      ② 10Hz 视觉 -> VISION_RESULT(0x02)
      ③ 收遥测(0x10)（判高用 dist_mm）
      ④ CONFIG（自动模式 / 开始停止 / 绕行时长 0x23）
```

摄像头起不来**不致命**：自动降级为「纯超声 + 状态机」。

### CSI 口自动探测

```python
SENSOR_TRY = (1, 2, 0)    # 依次试 CSI1 -> CSI2 -> CSI0
```
换排线口不用改代码，启动日志会打印 `[cam] sensor OK on CSI1`。

### 关掉 IDE 预览

```python
USE_DISPLAY      = False
USE_IDE_OUTPUT   = False
sensor = cv.start_camera(use_display=False)
```

### 3 秒心跳看门狗

```
网页 JS 每秒发 /cmd 心跳；无 JS 页面用 <meta refresh> + <img src="/hb"> 心跳
3 秒没有任何请求 -> 自动发 CONFIG 0x22=0 停车
```

**心跳策略（重要）**：
- JS 页面**只在运行中才发心跳**（`if (run) sendRun(run)`），
  所以"刷新页面"不会再把车停掉；
- 无 JS 页面每 2 秒自动刷新一次，顺带打 `/hb` 心跳。

> ⚠️ 上一版本文写"断开 WiFi 车不停"是错的 —— **现在会停**（3 秒后），已修正。

---

## 四、录像前 20 分钟检查单

```
□ 用带 JS 的首页（/），不要用 /t；打开页面后再点开始；中途不要刷新
□ 页面保持前台、屏幕常亮（3 秒没心跳会停车）
□ 看 rc_boot.txt 出现 "[cam] sensor OK on CSI1/2/0"
□ 轮子离地、低速：前进方向四轮一致；手轻挡能回落
□ 急停按下即停；松开不自动回 AUTO；再点【开始】才恢复
□ 到终点停车后不会 0.5 秒又自己走
□ 确认烧的是 build_auto.tcl 的产物，不是旧 delivery 的 bitstream
```

---

## 五、闭环方向验证（不做会放飞）

```
1. 轮子离地，给前进指令，四个轮子方向一致
2. 用手轻挡某个轮子，速度能回落
3. 某轮越跑越快 -> 立刻断电，翻 DIR_INV_MASK 对应位
4. FORCE_OPEN_LOOP=1 只用于排查，不要拿开环去录"闭环"镜头
```

---

## 六、编译与烧录

```powershell
cd tools\fpga_build
D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe build_auto.tcl
```

烧 `build\robot\impl\pnr\robot.fs`（SRAM，7 秒）。

> 归档里预编译了 `bitstream/robot.fs` 和 `bitstream/robot.bin`，
> 但**必须在现场重新编译确认**——源码改过，旧比特流不能用。
