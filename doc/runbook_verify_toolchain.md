# 执行手册：验证工具链 + 检查你的 Gowin 工程

> 照着从上往下做。每一关都有**命令 → 预期输出 → 通关条件 → 失败怎么办**。
> 任何一关没过就停下来，把完整输出发我，不要往下跳。

---

## 0. 就地验证即可（仓库已经在纯 ASCII 路径下）

**本仓库已迁到 `D:\dsh-ws\robot_project`**（纯 ASCII），
所以**不需要**再复制试验田 —— 直接在这个目录里按下面走。

```powershell
cd D:\dsh-ws\robot_project
Test-Path .\rtl\top.v        # 期望 True
```

> ⚠️ 如果你是从旧仓库同步过来的：**别用 `robocopy /MIR` 往新仓库同步**，
> 它会把 `sim\*.do`（生成物）覆盖成旧版，还会删掉目标里多出来的文件。
> 见 `doc/踩坑与经验.md` 4.8。

> 为什么一切都要在纯 ASCII 路径下做：Gowin 按当前工作目录派生日志路径、
> ModelSim 按 ANSI 把路径传给 sqlite —— 路径里有中文（比如用户名「忆洛」）
> 就会变成 `??`，报 `alog error` / `DATABASE ERROR`，而且**错误信息里
> 一个字都不提"路径"**。详见 `doc/踩坑与经验.md` 1.1 / 1.2。

---

## 1. 确认本机路径配置存在（30 秒）

work 库必须是**纯 ASCII 路径**，而且**必须和 `.do` 里写的一致**。
这两件事统一由 `sim\toolchain\toolchain.local.ps1` 决定（该文件不入库，每台机器一份）：

```powershell
cd D:\dsh-ws\robot_project\sim\toolchain
Test-Path toolchain.local.ps1
# False -> 从模板复制一份，再把里面的路径改成你自己机器的：
#   Copy-Item toolchain.local.ps1.template toolchain.local.ps1
```

**本机（队友 A）的配置**：

| 项 | 值 |
|---|---|
| ModelSim | `D:\modeltech64_2020.4` |
| Gowin 原语库 | `D:\modeltech64_2020.4\gw5a`（库名 `gw5a`） |
| work 库 | `D:\dsh-ws\robot_project\.simlib\work`（**在仓库内**，迁移时跟着走） |
| Gowin IDE | `D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64` |

> ❌ **不要再手工传 `-LibDir`**。让脚本从配置读，
> 才能保证"建库的脚本"和"生成 `.do` 的脚本"用的是**同一个库名和路径**。
> 两边不一致时的症状是：找不到原语，但**不报错**。

---

## 2. 确认两个工具都在、版本对不对（1 分钟）

```powershell
# 2.1 Gowin 安装（★ 必须是 V1.9.12：V1.9.11 教育版器件表里没有 GW5AT-138）
Test-Path 'D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64\IDE\simlib\gw5a\prim_sim.v'

# 2.2 ModelSim 是否在 PATH 里
Get-Command vsim.exe -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source

# 2.3 ★ 原语库是哪个 Gowin 版本建的？（没有输出 = V1.9.12，正确）
& 'D:\modeltech64_2020.4\win64\vdir.exe' -lib 'D:/modeltech64_2020.4/gw5a' | findstr "MODULE DQS"
```

**预期输出**：

- 2.1 → `True`
- 2.2 → 打印出 `vsim.exe` 的完整路径
- 2.3 → **没有任何输出**（`DQS` 只在 V1.9.11 的 `prim_sim.v` 里有；
  有输出说明库是用**旧版本** Gowin 建的 → 仿真模型和综合器不是一个版本，
  详见 `doc/toolchain_gowin_modelsim.md` 的「2026-10-15 更新」U2）

**2.2 没有输出（说明不在 PATH）**：无所谓，后面一律用绝对路径。先确认哪份可用：

```powershell
& 'C:\modeltech64_2020.4\win64\vsim.exe' -version
& 'D:\modeltech64_2020.4\win64\vsim.exe' -version
```

**预期输出**：各打印一行 `ModelSim SE-64 vsim 2020.4 ...`

记住你选用的那份，下面统一叫 `$VSIM`：

```powershell
# 二选一，把可用的那份写进变量（本次会话有效）
$VSIM = 'C:\modeltech64_2020.4\win64\vsim.exe'
# $VSIM = 'D:\modeltech64_2020.4\win64\vsim.exe'
```

**通关条件**：至少一份能输出版本号。

**失败（报 license 错误）**：把报错原文发我。本机 `LICENSE.TXT` 已确认是真许可
（`SERVER YILUO-LAPTOP ... 27001`），理论上不该失败；若失败多半是 `LM_LICENSE_FILE` 被别的东西覆盖了。

---

## 3. 先跑静态检查（30 秒，不需要仿真器）

这一关和工具链无关，但能提前排除代码问题，避免把"代码错"误判成"工具链错"。

```powershell
cd D:\dsh-ws\robot_project

# 找到本机的 python（DSH 自带那个）
$PY = 'C:\Users\忆洛\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'

& $PY tools\params_check.py
& $PY tools\rtl_lint.py
```

**预期输出**：

- `params_check.py` → 末尾 `=== 10 checks, 0 errors ===` 和 `RESULT: PASS`
- `rtl_lint.py` → 末尾 `errors=0 warnings=0` 和 `RESULT: PASS`

> 中文可能显示成乱码（控制台代码页问题），不影响判断——看 `RESULT: PASS` 就行。

**通关条件**：两个都是 `RESULT: PASS`。**失败就停**，把输出发我。

---

## 4. 诊断工具链（1 分钟，安全，不改文件）

```powershell
cd D:\dsh-ws\robot_project\sim
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1 -Check
```

**预期输出**（重点看这几行）：

```
=== 0) 定位 Gowin 与 ModelSim ===
  [ OK ] Gowin    : D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64
  [ OK ] ModelSim : D:\modeltech64_2020.4
=== 1) 检查 Gowin 仿真库文件 ===
  [ OK ] 器件族 : gw5a
  [ OK ] 包含 module PLL
  [ OK ] 包含 module OSC
  [ OK ] 包含 module GSR
=== 2.5) 验证 ModelSim 能否在该目录建库（关键）===
  [ OK ] 建库成功，_info 已生成 —— 路径问题已规避
```

> ★ 同时确认它选的是 **V1.9.12**（不是 V1.9.11）。
> 装了多个 Gowin 版本时，脚本会按版本号取最新并在输出里列出它看到的全部版本。

**通关条件**：没有 `[FAIL]`。

**失败对照**：

| 报错 | 原因 | 对策 |
|---|---|---|
| `[FAIL] 指定的 -LibDir 含非 ASCII 字符` | 路径里有中文 | 不传 `-LibDir`，改用 `toolchain.local.ps1` 的 `GowinSimLib` |
| `[FAIL] 没能自动找到既纯 ASCII 又可写的目录` | 候选盘都不可写 | 在 `toolchain.local.ps1` 里把 `GowinSimLib` 指到有写权限的纯 ASCII 目录 |
| `[FAIL] ModelSim 建库失败！` + `DATABASE ERROR` | 库路径仍非 ASCII，或许可问题 | 换纯 ASCII 路径；仍失败把输出发我 |
| `[FAIL] 没找到 Gowin 安装目录` | 自动探测没命中 | 在 `toolchain.local.ps1` 里设 `GowinHome`（本机 `D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64`） |
| 脚本报"无法加载文件…未对文件进行数字签名" | 执行策略 | 命令前加 `-ExecutionPolicy Bypass`（已加） |

**顺便记下诊断结果里的 ModelSim 路径**，它决定后面用哪份：

```
[ OK ] ModelSim : C:\modeltech64_2020.4     ← 就用这个
```

---

## 5. 正式配置工具链（1–2 分钟）

这一步是**真正的修复**：编译 Gowin 原语库 + 建立映射 + 生成脚本。

```powershell
cd D:\dsh-ws\robot_project\sim
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1
```

**预期输出**（关键几行；库名/位置**由 `toolchain.local.ps1` 决定**）：

```
=== 4) 编译 Gowin 原语库 ===
  [ OK ] 库名取自 toolchain.local.ps1: gw5a
  vlib gw5a
  vlog -work gw5a <prim_sim.v>
    Errors: 0, Warnings: 0
  [ OK ] 原语库编译完成: D:\modeltech64_2020.4\gw5a
=== 5) 建立库映射（写进工程的 modelsim.ini，不动安装目录）===
  [ OK ] 更新已有映射: gw5a = D:/modeltech64_2020.4/gw5a
=== 6) 生成 compile_all.do / run_sim.do ===
  [ OK ] 已生成 compile_all.do 与 run_sim.do
```

> ⚠️ 库名是 **`gw5a`**（取 `GowinSimLib` 路径的最后一段），不是老文档里的
> `prim_gw5a`。**库名必须和 `compile_all.do` 里的 `-L` 一致**，
> 否则 `.do` 会指向一个不存在/另一份的库 —— 这种错**不会报错**，只会找不到原语。

**通关条件**：看到 `[ OK ] 原语库编译完成`，且 `Errors: 0, Warnings: 0`。

> ✅ 这一步**已经实测跑通**（2026-10-15）：V1.9.12 的 `prim_sim.v`（708,585 字节）
> 在 ModelSim SE-64 2020.4 下编译 **0 errors / 0 warnings / 132 个模块**，
> 里面的 UDP（`primitive mux2 ...`）没有任何问题。
> 原来担心的"UDP 可能不支持"这条风险已排除。

**验证产物**（无论成功与否都建议看一眼）：

```powershell
Test-Path 'D:\modeltech64_2020.4\gw5a\_info'          # 期望 True
(Get-ChildItem 'D:\modeltech64_2020.4\gw5a').Count    # 期望 > 5
Select-String -Path D:\dsh-ws\robot_project\sim\modelsim.ini -Pattern 'gw5a'   # 期望有一行映射

# ★ 库的"版本"对不对（2026-10-15 新增，重要）：
& 'D:\modeltech64_2020.4\win64\vdir.exe' -lib 'D:/modeltech64_2020.4/gw5a' |
    Select-String 'MODULE DQS'     # 期望**没有输出**（无输出 = V1.9.12 建的）
```

---

## 6. 编译工程（1 分钟）

```powershell
cd D:\dsh-ws\robot_project\sim
& $VSIM -c -do compile_all.do
```

**预期输出**：

```
# ** Note: (vlog-...)
########## COMPILE DONE ##########
If there is no Error above, run run_sim.do next.
Gowin primitive lib: D:/modeltech64_2020.4/gw5a
```

**通关条件**：**没有任何 `** Error:`**，且看到 `COMPILE DONE`。

**失败对照**：

| 报错 | 原因 | 对策 |
|---|---|---|
| `Cannot open include file "project_config.vh"` | `+incdir+` 没生效或路径不对 | 把输出发我 |
| `Instantiation of 'PLL' failed` | 编译时没挂 Gowin 原语库（缺 `-L gw5a`） | 用 `compile_all.do`（由 `emit_dofiles.ps1` 生成，已含 `-L gw5a`） |
| `Failed to create directory` / `DATABASE ERROR` | work 库落在中文路径 | 回第 1 关；或告诉我工程路径，我调整 |
| `near "..." : syntax error` | RTL 语法问题 | 把报错和文件名发我，我改代码 |

> 注意：这一步只编译，不运行。有 Warning 无妨，有 Error 必须解决。

---

## 7. 跑仿真回归 ← 你最想看波形的一步（2–5 分钟）

### 7.1 先跑批处理看结果

```powershell
cd D:\dsh-ws\robot_project\sim
& $VSIM -c -do run_sim.do
```

**预期输出**（每个 tb 一段，重点是最后的 RESULT 行）：

```
# RUN tb_reset_sync
=== tb_reset_sync: N checks, 0 errors ===
RESULT: PASS

... 共 13 条 tb ...

# RUN tb_gowin_prim
=== tb_gowin_prim: 17 checks, 0 errors ===
RESULT: PASS

########## SIMULATION FINISHED ##########
```

> `N` 不用对数字。**只看两件事：`errors=0` 和 `RESULT: PASS`。**
>
> ★ 其中最后那条 **`tb_gowin_prim` 是"工具链冒烟"**：它例化真实 Gowin 原语
> （`LUT4`/`IOBUF`/`GSR`/`OSC`），专门验证"原语库挂上了、版本对"。
> **它红了 = 工具链的问题，不是项目代码的问题**，先去看 §2.3 的库版本检查。

**通关条件**：全部 tb 都是 `RESULT: PASS`（当前是 13 条、446 项检查）。

**失败怎么办**（这很正常，Testbench 从没被执行过）：

1. 把 **`[FAIL]` 那几行** + 对应 tb 名发我
2. 优先怀疑测量相位（我在 README §3 列了三处已知脆弱点）
3. 只跑单个 tb 定位：

```powershell
& $VSIM -c -do "do run_sim.do tb_uart"
```

### 7.2 再看波形（GUI）

```powershell
cd D:\dsh-ws\robot_project\sim
& $VSIM -do run_sim.do
```

ModelSim 打开后：

1. 等它把 5 个 tb 依次跑完（Transcript 窗口会滚动）
2. 在 **Transcript** 窗口输入：

```tcl
add wave -r /*
```

3. 或者手动看单个：左侧 `sim` 窗口展开 `work` → 双击 `tb_pwm_motor` → 在 Objects 窗口选中信号 → 右键 `Add Wave`

**想单独深入看某个模块的波形**，建议这样跑：

```powershell
& $VSIM -do "vsim -L work -L gw5a work.tb_pwm_motor; add wave -r /*; run -all"
```

---

## 8. 汇报给我什么（第二个过程）

请把下面三样发我：

### 8.1 四段完整输出

```
① 第 3 关：params_check.py + rtl_lint.py 的结果
② 第 4 关：setup -Check 的完整输出
③ 第 5 关：setup 正式执行的完整输出   ← 最关键，含 prim_sim.v 编译结果
④ 第 7 关：run_sim.do 的完整输出（或 FAIL 那几行）
```

### 8.2 你的工程信息

**问题一：你的 Gowin 工程在哪？**

我扫过 `C:\Users\忆洛\Documents`、`Desktop`、`D:\`、`E:\`，**没找到任何 `.gprj`**。
你上个会话"把 Gowin 代码挪进 ModelSim"，说明工程确实存在。请给我：

```powershell
# 帮你找，把输出发我
Get-ChildItem -Path 'C:\','D:\','E:\' -Recurse -Depth 5 -Filter '*.gprj' -ErrorAction SilentlyContinue |
  Select-Object FullName, LastWriteTime
```

如果搜不到，直接告诉我你平时**在哪个目录打开 Gowin**。

**问题二：器件型号确认**

```powershell
# 打开 Gowin 工程后看 Project > Configuration，或贴 .gprj 里的 device 行
Select-String -Path '<你的工程>.gprj' -Pattern 'device|Device'
```

我需要知道是 `GW5AST-138` 还是 `GW5AST-138B` / `GW5AST-138B`。

### 8.3 你希望我下一步做什么（可多选）

- [ ] **把工具链脚本搬进你的真实工程**，并按你工程的真实文件列表生成 `compile_all.do`
- [ ] **用 `gw_sh.exe` 命令行跑一遍综合**，先确认综合能过（不依赖 ModelSim）
- [ ] **生成 PLL IP**：`clk_gen.v` 需要例化 Gowin PLL，告诉我输入频率（50MHz?）和目标频率
- [ ] **修第 7 关里 FAIL 的 Testbench**
- [ ] **继续写缺失模块**（`main_fsm` / `uart_frame` / `tof_driver` …）

---

## 附：一页速查

## 每次新开终端，第一件事（预设条目）

**第一次用**（工具脚本已经在仓库里了，**不需要再 copy**）：

```powershell
# 只需要确认本机路径配置存在（不入库，每台机器一份）
cd D:\dsh-ws\robot_project\sim\toolchain
Test-Path toolchain.local.ps1     # False 就 Copy-Item toolchain.local.ps1.template toolchain.local.ps1 再改路径
```

**以后每次开新终端，就这一条**（注意开头那个 `.` 不能少）：

```powershell
. D:\dsh-ws\robot_project\sim\toolchain\start_here.ps1
```

它会自动设好并检查：

| 变量 / 检查 | 内容 |
|---|---|
| `$VSIM` | ModelSim 可执行文件路径 |
| `$PY` | Python 解释器（跑静态检查用） |
| `$PRJ` | 工程根目录 |
| `$SIM` | `sim` 目录 |
| 许可变量 | `LM_LICENSE_FILE` / `MGLS_LICENSE_FILE` |
| 环境预检 | ModelSim、许可条目数、Gowin 原语库、关键 `.do` 文件 |

> **为什么前面必须加 `.`？**
> 不加点是"开子进程执行"，子进程里设的环境变量不会留给你当前终端。
> 加点（点源）才是"在当前终端里执行"，`$VSIM` 这些才会生效。

如果 `start_here.ps1` 报"禁止运行脚本"，就用这条替代：

```powershell
powershell -ExecutionPolicy Bypass -File D:\dsh-ws\robot_project\sim\toolchain\start_here.ps1
```

（但这样变量不会留在当前终端，需要再手工设一次 `$VSIM`。）

## 日常循环（预设好的话，直接抄）

```powershell
# ---------- 只拿测试结果（最省事，推荐）----------
cd $SIM
powershell -ExecutionPolicy Bypass -File toolchain\run_all.ps1 -NoVcd
Get-Content summary.txt

# ---------- 一条龙：编译 + 测试 + 导出波形 ----------
powershell -ExecutionPolicy Bypass -File toolchain\run_all.ps1

# ---------- 手动单跑 ----------
& $VSIM -c -do compile_all.do                  # 编译
& $VSIM -c -do run_sim.do                      # 跑全部 tb
& $VSIM -c -do "do run_sim.do tb_uart"         # 只跑一个

# ---------- 看波形（批处理导出 VCD，不需要 GUI 许可）----------
& $VSIM -c -do "do vcd.do tb_pwm_motor 3000"   # 只抓前 3us
gtkwave tb_pwm_motor.vcd                       # 用 GTKWave 打开

# ---------- 改了 RTL 文件列表后重新生成 .do ----------
# 不需要传参：路径自动从 toolchain.local.ps1 读
powershell -ExecutionPolicy Bypass -File toolchain\emit_dofiles.ps1

```

> ✅ **GUI 实测可用**（推荐）—— 但**必须先设好许可环境变量**：
> ```powershell
> $env:LM_LICENSE_FILE   = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
> $env:MGLS_LICENSE_FILE = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
> ```
> 早先报 `Unable to checkout a viewer license` 就是**漏了这一步**，

> **GUI 里的缩放命令要加 `wave` 前缀**（实测确认，见 ModelSim 文档
> `modelsim_se_ref -> Command_Wave`）：
>
> | 想做什么 | 正确命令 | 错的写法 |
> |---|---|---|
> | 看全貌 | `wave zoom full` | ~~`zoom full`~~ → `invalid command name "zoom"` |
> | 看指定区间 | `wave zoom range 30us 60us` | `wave zoom range 30000 60000`（裸数字按**精度单位**解释，会缩到 30ns） |
> | 放大/缩小 | `wave zoom in` / `wave zoom out` | — |
> | 撤销缩放 | `wave zoom last` | — |
>
> 原因：`zoom` 是**波形窗口的命令**，Transcript 是主解释器上下文，找不到它。
> GUI 里也可以用：在波形区**右键 → Zoom → ...**，或拖选一段后 Zoom Selected。
> 不是许可缺 feature（文件里确实有 `INCREMENT vsimviewer_c`）。
> 走上面的 VCD 路线即可，功能完全一样。

## 三条排错铁律

1. **看到 `DATABASE ERROR` / `unable to open database file`** → 库路径含中文，换纯 ASCII 路径。
2. **看到 `Instantiation of 'XXX' failed`** → 原语库没挂上，检查 `vsim` 有没有 `-L gw5a`。
3. **看到 `(vmap-20) Cannot access for writing file "modelsim.ini"`** → 那份 ini 缺 `[Library]` 段，从 `D:\modeltech64_2020.4\modelsim.ini` 复制一份完整的过来。

## 单独排错用的脚本

| 脚本 | 用途 |
|---|---|
| `toolchain\start_here.ps1` | 新终端环境预设 + 预检 |
| `toolchain\run_all.ps1` | 一键编译+测试+导出波形+汇总 |
| `toolchain\diag_vlib.ps1` | 建库失败时抓真实报错 |
| `toolchain\setup_gowin_modelsim.ps1` | 编译 Gowin 原语库、建映射 |
| `toolchain\emit_dofiles.ps1` | 生成/重新生成 .do 文件 |

