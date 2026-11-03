> **👈 队友第一次拿到代码：先看 [START_HERE.md](START_HERE.md)**

# 轮足复合式 FPGA 实时感知与自主控制机器人

> 2026 全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道（高云半导体）
> 平台：小梅哥 ACG720 / **GW5AT-138**（PBGA484A 封装） · 本科组

本仓库是 **FPGA 主体** 侧工程。ARM 只负责视觉，通过 UART 把识别结果交给 FPGA；
所有实时控制（传感器采集、电机闭环、避障状态机、升降腿）都在 FPGA 内完成。

---

## 1. 目录结构

```
robot_project/
├── rtl/
│   ├── project_config.vh        # 全局参数宏
│   ├── top.v                    # 顶层, 只例化不写复杂逻辑（已接入完整决策链路）
│   ├── app/                     # 应用层: 主状态机、运动决策        (已完成, 仿真通过)
│   ├── driver/                  # 驱动层: PWM/编码器/UART          (已完成, 仿真通过)
│   │                            #         TOF/IMU 待传感器选型后补
│   └── hal/                     # 硬件抽象层: 时序/复位/时钟        (已完成, 仿真通过)
├── sim/                         # Testbench + ModelSim 回归脚本（21/21 通过 / 825 项检查）
│   └── toolchain/               # Gowin ↔ ModelSim 联通脚本（见 2.2）
├── constraints/                 # 引脚约束
│   ├── pin_table.csv            #   ★ 引脚分配填报表（填这个）
│   └── top.cst                  #   由 tools/gen_cst.py 生成，别手改
├── tools/                       # 参数自检、RTL 静态检查
└── doc/                         # 文档（见下方"文档导航"）
```

### 文档导航

| 文档 | 什么时候看 |
|---|---|
| **[doc/onboard_build_flow.md](doc/onboard_build_flow.md)** | **★ 上板流程**（综合→比特流→下载，含两个大坑） |
| **[doc/board_pinout.md](doc/board_pinout.md)** | **★ 板载资源与引脚分配速查**（接线的看这个） |
| **[handoff/README.md](handoff/README.md)** | **★★ 三人任务分工与 DSH 提示词** —— 发给队友先看这个 |
| **[interface_delivery.md](doc/interface_delivery.md)** | **★ 发给队友的第一份文件** —— 接口全表 + UART 协议 + 机械待办 + 上电时序 |
| **[git_basics.md](doc/git_basics.md)** | **★ 完全没用过 Git 的先看这个** —— 从零上手，每条命令都能直接复制 |
| **[team_sync_guide.md](doc/team_sync_guide.md)** | **★ 三个人怎么同步进度** —— Git 工作流、分支策略、接口冻结规则 |
| [arm/README.md](arm/README.md) | **B 用** —— 可直接跑的 ARM 侧协议代码（不需要硬件） |
| **[design_guide.md](doc/design_guide.md)** | **想搞懂"每个模块内部怎么运转、为什么这么设计"** —— 最详细的一份 |
| [interface_spec.md](doc/interface_spec.md) | 要改接口、要对齐跨模块约定 |
| [runbook_verify_toolchain.md](doc/runbook_verify_toolchain.md) | **第一次配环境/跑仿真**，逐步命令手册 |
| [toolchain_gowin_modelsim.md](doc/toolchain_gowin_modelsim.md) | 工具链出问题时的原理与排查 |
| [uart_frame_protocol.md](doc/uart_frame_protocol.md) | ARM 侧同学实现通信协议（协议完整定义） |
| [team_handoff.md](doc/team_handoff.md) | 分工、阻塞项、需要机械/ARM 提供什么 |
| [progress_review.md](doc/progress_review.md) | 缺陷复盘（可作答辩素材） |

## 2. 快速开始

> 🚩 **第一次上手请先看 [doc/runbook_verify_toolchain.md](doc/runbook_verify_toolchain.md)**
> —— 那是逐步骤的执行手册（命令 → 预期输出 → 失败对策），
> 从"建库目录"到"看到波形"一共 8 关，照着敲就行。

### 2.1 不需要仿真器的检查（随时可跑）

```powershell
# 参数自检: PWM 频率/占空比标度、波特率误差、测速窗口换算
python tools/params_check.py                 # 50MHz
python tools/params_check.py --clk 100000000 # PLL 换成 100MHz 后

# RTL 静态检查: 模块配平、重复声明、未声明信号、疑似魔法数字
python tools/rtl_lint.py
python tools/check_encoding.py .
```

### 2.2 Git 一键配置（队友第一次必做）

**工程根目录的 `setup_git.ps1`：右键 → 使用 PowerShell 运行，3 个问题一路回车。**

它会配置提交身份、生成 SSH 密钥、绑定 Gitee 仓库、推送、建分支。
详见 **[doc/git_basics.md](doc/git_basics.md)**。

> ⚠️ 必须在 PowerShell 窗口里运行，不能用管道喂输入（脚本会自检并拒绝）。

---

### 2.2 ModelSim 仿真回归

#### 队友首次配置（★ 按顺序做完这 3 步）

```powershell
cd sim\toolchain

# ---- 第 1 步：建本机路径配置 ----
#   toolchain.local.ps1 已被 .gitignore 排除，各人配置互不影响。
#   （仓库里只有 .template，**这是故意的** —— 路径写死会让别人跑不了）
Copy-Item toolchain.local.ps1.template toolchain.local.ps1
notepad toolchain.local.ps1          # 改里面的 ModelsimHome / WorkDir 等
#   ⚠️ WorkDir 必须是【纯 ASCII 路径】—— ModelSim 2020.4 在含中文的路径下
#      无法创建 work 库（本机用户名是「忆洛」，C:\Users\忆洛\... 必挂）

# ---- 第 2 步：编译 Gowin 原语库（只需做一次）----
#   不做这步的话，代码一例化 PLL 就报
#       Instantiation of 'PLL' failed. The design unit was not found
powershell -ExecutionPolicy Bypass -File setup_gowin_modelsim.ps1 -Check    # 先诊断
powershell -ExecutionPolicy Bypass -File setup_gowin_modelsim.ps1          # 正式配置

# ---- 第 3 步：一条命令跑完整回归 ----
cd ..
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1
#   期望：21 个 tb 全部 RESULT: PASS，共 825 项检查
#   跑完会生成 verify\<你>_<机器>_<commit>.md —— 提交它，队友就知道你验证过了
```

> **`compile_all.do` / `run_sim.do` / `wave.do` / `vcd.do` 都不在仓库里** ——
> 它们是 `emit_dofiles.ps1` **生成**的，因为内容含各人的绝对路径。
> 改了 RTL 文件列表后重新生成（**不需要传参**，自动读 `toolchain.local.ps1`）：
> ```powershell
> cd sim
> powershell -ExecutionPolicy Bypass -File toolchain\emit_dofiles.ps1
> ```

<details>
<summary>想手动一步步跑（不用 <code>run_tb.ps1</code>）</summary>

```powershell
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\emit_dofiles.ps1   # 生成 .do
vsim -c -do compile_all.do     # 建 work 库 + 挂 Gowin 原语库 + 编译全部源码
vsim -c -do run_sim.do         # 跑回归
vsim    -do run_sim.do         # 开 GUI 看波形
```
</details>

原理与排查见 **[doc/toolchain_gowin_modelsim.md](doc/toolchain_gowin_modelsim.md)**。

若想**一条命令跑完整回归并看汇总**（推荐，会自动清库 + 编译 + 跑全部 tb + 打印每个 tb 的 RESULT）：

```powershell
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1              # 全部
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1 tb_uart_frame # 单个
```

> **⚠️ 旧的 `simulate.do` 已删除**（2026-10-04）。
> 原因有两个，都会实实在在坑到人：
>
> 1. 它的 tb 列表**停留在 5 个**（缺 `tb_uart_frame` / `tb_main_fsm` / `tb_motion_decision`），
>    编译列表也缺 3 个 RTL。**跑完会显示"全 PASS"，但实际上漏跑了 3 个 tb** ——
>    这种"假通过"比报错更危险。
> 2. 文件里的注释是中文。`.do` 必须**纯 ASCII** —— Tcl 按 ANSI 代码页读取，
>    中文会被解成乱码；更糟的是如果某个字节被解成行尾反斜杠，会导致**行继续**，
>    把下一行当命令执行。
>
> **统一用 `compile_all.do` + `run_sim.do`（由 `emit_dofiles.ps1` 生成），
> 或上面的 `run_tb.ps1`。** 生成器会保证 `.do` 是纯 ASCII 且 tb 列表齐全。

> ⚠️ 不要把 `modelsim.ini` 手工拷进 `sim/`。那份文件里存的是绝对路径，
> 工程一移动或路径含中文就会出现
> `init_dbinfo() DATABASE ERROR: (sqlite3_open ...) unable to open database file`，
> 这个报错和代码无关，纯粹是库映射路径问题。
>
> ⚠️ 已知环境限制：ModelSim 2020.4 无法在**含中文的路径**下创建 work 库
> （本机用户名是 `忆洛`，`C:\Users\忆洛\...` 必挂，已实测复现）。
> 规避办法是让 `work` 库落在纯 ASCII 路径 ——
> `setup_gowin_modelsim.ps1` 会自动这么做，**源码可以继续留在中文路径下**。

### 2.3 上板

用 Gowin IDE（云源软件）新建工程，加入 `rtl/` 下所有 `.v`，
约束用 `constraints/top.cst`。

> ⚠️ **`top.cst` 是自动生成的，不要手改。** 管脚号现在还是占位值，
> 填法：编辑 `constraints/pin_table.csv`（只填 `pin` 列），
> 然后跑 `python tools/gen_cst.py`。
> 脚本会检查漏填、**重复球号**、引脚名格式和电气属性 ——
> 手抄 140 行约束最容易犯的错是"把 m_pwm[2] 抄成 m_pwm[1] 的球号"，
> 这种错 Gowin **不会报**，结果是两个轮子一起动。

---

## 3. 模块状态

> 状态列的含义：**✅ 仿真通过** = 已在 ModelSim 实跑，`RESULT: PASS`。

### 3.0 验证结果总览（2026-10-04 实测，**7/7 全通过**）

| Testbench | 检查项 | 结果 |
|---|---|---|
| `tb_reset_sync` | 11 | ✅ **PASS** |
| `tb_pwm_motor` | 21 | ✅ **PASS** |
| `tb_encoder_decode` | 17 | ✅ **PASS** |
| `tb_uart` | 17 | ✅ **PASS** |
| `tb_main_fsm` | 63 | ✅ **PASS** |
| `tb_motion_decision` | 32 | ✅ **PASS** |
| `tb_top_integration` | 52 | ✅ **PASS** |

**整条链路已在仿真中闭环**：

```
传感器 ─► main_fsm ─► motion_decision ─► pwm_motor ×4 ─► 电机
                                      ▲
                                encoder_decode ×4
```

复位 → 按键启动 → 避障 → 越障三阶段 → 编码器闭环 → 急停 → 到达终点，
全流程在 `tb_top_integration` 中断言通过。

集成测试的关键数值（可作答辩数据）：

| 项目 | 实测 | 设计值 | 说明 |
|---|---|---|---|
| UART 起始位宽度 | 869 clk | `CLK/BAUD` = 868 | 偏差 1 拍（边沿对齐） |
| UART 回环 | 8/8 字节正确 | — | 含背靠背流控 |
| 同步释放延迟 | 3 拍 | `SYNC_STAGES` = 3 | 完全一致 |
| 编码器测速 | 48 计数/窗口 | 理论 ≈ 50 | 偏差来自窗口边界 |
| 避障超时 | 100001 拍 | 阈值 100000 | 超时保护按预期触发 |
| 抬腿超时 | 100001 拍 | 阈值 100000 | 同上 |
| 急停断使能 | **0 拍** | 组合逻辑直通 | FPGA 硬实时的核心指标 |

### 3.0.1 模块状态

| 模块 | 文件 | Testbench | 状态 |
|---|---|---|---|
| `reset_sync` | `rtl/hal/reset_sync.v` | `tb_reset_sync.v` | ✅ **仿真通过**（11 checks, 0 errors） |
| `pwm_motor` | `rtl/driver/pwm_motor.v` | `tb_pwm_motor.v` | ✅ **仿真通过**（21 checks, 0 errors） |
| `encoder_decode` | `rtl/driver/encoder_decode.v` | `tb_encoder_decode.v` | ✅ **仿真通过**（17 checks, 0 errors） |
| `uart_tx` | `rtl/driver/uart_tx.v` | `tb_uart.v` | ✅ **仿真通过**（17 checks, 0 errors） |
| `uart_rx` | `rtl/driver/uart_rx.v` | `tb_uart.v` | ✅ **仿真通过**（同上） |
| `main_fsm` | `rtl/app/main_fsm.v` | `tb_main_fsm.v` | ✅ **仿真通过**（93 checks, 0 errors） |
| `motion_decision` | `rtl/app/motion_decision.v` | `tb_motion_decision.v` | ✅ **仿真通过**（32 checks, 0 errors） |
| `uart_frame` | `rtl/app/uart_frame.v` | `tb_uart_frame.v` | ✅ **仿真通过**（58 checks, 0 errors） |
| `clk_gen` | `rtl/hal/clk_gen.v` | `tb_clk_gen.v` | ✅ **仿真通过**（9 checks, 0 errors），默认旁路 50MHz |
| `servo_lift` | `rtl/driver/servo_lift.v` | `tb_servo_lift.v` | ✅ **仿真通过**（20 checks, 0 errors） |
| `i2c_master` | `rtl/driver/i2c_master.v` | `tb_tof_driver.v`（被 TOF 驱动一起验证） | ✅ **仿真通过** |
| `tof_driver` | `rtl/driver/tof_driver.v` | `tb_tof_driver.v`（68 checks） | ✅ **仿真通过**；已集成进 `top.v`（`tb_top_tof_i2c.v` 42 checks 走**真实 I2C** 验证）<br>初始化是**官方 ST 序列（107 步）**，含防篡改校验和断言；⚠️ 单一来源、未经第二份交叉核对（见 `doc/进度与交接.md`） |
| `top`（集成） | `rtl/top.v` | `tb_top_integration.v`(66) + `tb_top_tof_i2c.v`(42) | ✅ **仿真通过**；整机综合 + 时序（TNS=0）也通过 |
| `imu_driver` | `rtl/driver/imu_driver.v` | `tb_imu_driver.v`（53 checks） | ✅ **仿真通过**；**已集成进 `top.v`**（`IMU_ENABLE` 默认 1，roll/pitch/yaw 进 telemetry）<br>⚠️ roll/pitch 是小角度近似、陀螺零偏未标定（yaw 会漂） |
| `i2c_bus_arb` | `rtl/driver/i2c_bus_arb.v` | `tb_top_i2c_share.v`（28 checks） | ✅ **仿真通过**：TOF + IMU **共用一条 I2C**（开漏线与 + 轮询授权，同一时刻只允许一个 master 说话） |
| `motor_guard` | `rtl/driver/motor_guard.v` | `tb_motor_guard.v`（41 checks） | ✅ **仿真通过**：软启动（上升限速、**下降当拍**）+ 堵转保护（断电 + 冷却 + 断续试探），夹在 `motion_decision` 与 `pwm_motor` 之间 |
| `display_ctrl` | — | — | ⬜ 待实现 |

### 3.1 仿真中发现的 RTL 缺陷（已修，务必不要再改回去）

这些都是**仿真几分钟能发现、上板可能要查几天**的类型，值得在答辩材料里提。

| # | 模块 | 缺陷 | 上板后的现象 |
|---|---|---|---|
| 1 | `uart_tx` / `uart_rx` | `BAUD_RATE * 2^24` 在 32 位里**溢出**（1.93e12 回绕成 0），`baud_tick` 永不触发 | **一个字节都发不出去**；会误以为是线序/波特率/共地问题 |
| 2 | `uart_tx` | 用"计数归零"当比特结束标志，进入 START 时 `div_cnt` 正好是 0 → 起始位只有 1 个 clk | 收发全废；起始位宽度为 0 |
| 3 | `uart_rx` | 下降沿判据写成了**上升沿**（`s2==0 && s1==1`） | **永远收不到任何字节**；且 TX 看起来正常，更难查 |
| 4 | `reset_sync` | `POR_CYCLES=0` 时用 `always @(*)` 写 `por_done`，空敏感列表 + 与声明初值竞争 → `por_done` 恒为 0 | **上电后复位永不释放，整块电路锁死** |
| 5 | `pwm_motor` | `duty_in` 的符号同时被当作方向和幅值 | 倒车变成"反向 PWM 减速" |

**配套的验证方法**（这几条是本工程验证体系的骨架）：

- **Python 周期精确模型**（`tools/verify_uart_model.py`）：UART 这类纯时序状态机，
  改完 RTL 先在这里秒级验证逻辑，再去跑 ModelSim —— 避免"改一次、跑一次"的来回。
- **Testbench 用 `$fwrite` 自己写结果文件**：ModelSim 批处理模式下 `$display` 走 stdout，
  PowerShell 管道经常抓不到，导致"跑完了但看不到 PASS/FAIL"。
  落盘的文件不受这个影响，**关键测量值也必须落盘**（否则事后无法定位）。
- **判据要用无歧义写法**：例如测同步释放延迟，不要"数到它变高"
  （受 NBA/active 区时序影响会差一拍），而应验证"第 N-1 拍仍低、第 N 拍已高"。

### 3.2 仿真环境说明

本工程已在 ModelSim SE-64 2020.4 上**实跑通过**（4/5 个 Testbench PASS）。

两个已解决的坑：

1. **中文路径**：ModelSim 2020.4 用 ANSI 编码打开库数据库，路径含中文必失败。
   解决：work 库放在纯 ASCII 路径（`D:\gowin_lib\work`），源码可留在中文路径。
2. **Gowin 原语库**：Gowin 只提供 `prim_sim.v` 源码，必须先用 `vlog` 编译成库
   （`D:\modeltech64_2020.4\gw5a`），并在仿真时用 `-L gw5a` 挂上，
   否则一旦例化 PLL 就会报 `Instantiation of 'PLL' failed`。

详见 [doc/toolchain_gowin_modelsim.md](doc/toolchain_gowin_modelsim.md) 与
[doc/runbook_verify_toolchain.md](doc/runbook_verify_toolchain.md)。

---

## 4. 编码约定（新人先读这一节）

1. **统一时钟、统一复位**
   `sys_clk` 一个时钟域；`rst_n` 由 `hal/reset_sync.v` 产生
   （异步 assert / 同步 release），业务模块不得自行同步复位。

2. **禁止魔法数字**
   时钟频率、波特率、PWM 频率、窗口长度一律 `parameter` 或写进
   `project_config.vh`。

3. **厂商隔离**
   只有 `hal/` 下的文件可以例化 `gowin_pll` 等原语。业务模块不碰原语，
   这样换平台时只改 hal。

4. **状态机三段式**
   状态寄存器 / 次态组合 / 输出逻辑，且**必须有 `default` 分支**。

5. **跨时钟域**
   用双时钟 FIFO，或至少 2 级同步器。**禁止**直接打拍。

6. **有符号数要小心**
   本工程踩过的坑：把 `{{29{1'b0}}, step_s}` 这种拼接结果整体当有符号数用，
   `-1` 会被当成 `+1`。**幅值和符号分开处理**最省心。
   `encoder_decode.v` 里就是按这个思路写的（正/反向分别累加）。

7. **方向不要藏在占空比符号里**
   普通直流驱动板只有一根方向脚，负 duty 表达不了倒车。
   `pwm_motor` 的 `duty_in` 只给幅值，方向走 `dir_in`，详见
   `doc/interface_spec.md` §2.2。

8. **每个模块先写 Testbench 仿真通过，再上板。**

---

## 5. 上板调试铁律

1. **先只给 FPGA 供电**，烧最简 LED 程序，确认板子活着、时钟在跑。
2. 连信号线，**不接电机电源**，用逻辑分析仪抓 PWM/方向/编码器波形。
3. 接电机电源，**低速短时**（占空比 ≤ 30%，每次 ≤ 2 秒）。
4. 确认转向与编码器符号一致后，再上闭环。
5. 每次上电前确认：**电机、FPGA、ARM、电池必须共地。**

---

## 6. 分工与协作

| 角色 | 主责 | 协办 |
|---|---|---|
| A：FPGA 实时控制 | 顶层、PLL、主状态机、PWM、编码器、UART、TOF/IMU 驱动、时序优化 | 通信协议定义、RTL 接口 |
| B：ARM 视觉与通信 | ARM 选型烧录、摄像头驱动、轻量视觉算法、ARM-FPGA 协议、Python 模拟 | RTL 接口理解、UART 收发 |
| C：机械硬件与集成 | 底盘/升降腿、3D 打印装配、电源分配、传感器安装、场地测试、文档/视频/PPT | 整机调试、逻辑分析仪 |

- 每日 15 分钟站会
- **接口先行**：先改 `doc/interface_spec.md`，再改 RTL
- Git 分支：`main` 保持可编译，功能开 `feature/xxx`
- 每周半天集成联调；每天提交
