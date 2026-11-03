# 把 Gowin 和 ModelSim 真正连起来

这篇文档解释**为什么两个软件"没联系起来"**，以及怎么一次修好。
结论先行：问题不在你的代码，而在 **Gowin 的仿真库从来没编译进 ModelSim**，
外加 **ModelSim 2020.4 的一个中文路径缺陷**。

---

## 1. 症状对照表

| 你看到的现象 | 真正的原因 | 修法 |
|---|---|---|
| ModelSim 报 `Instantiation of 'PLL' failed. The design unit was not found` | Gowin 原语库没编译进 ModelSim | 跑 `setup_gowin_modelsim.ps1` |
| 报 `(vlib-35) Failed to create directory` / `Permission denied` | 目标目录不可写 | 换到 `D:\gowin_lib` 之类 |
| 报 `init_dbinfo() DATABASE ERROR: (sqlite3_open ...) unable to open database file` | **路径含中文**，ModelSim 用 ANSI 编码开路径，非 ASCII 必挂 | work 库放到纯 ASCII 路径 |
| 每次都要把 `.v` 文件拷进 ModelSim 目录 | 没有统一编译脚本 | 用生成的 `compile_all.do` |
| Gowin 里点仿真没反应 | IDE 里没配外部仿真器路径 | 见 §5 |

---

## 2. 根因详解

### 2.1 Gowin 的仿真库是"源码"，不是"编译好的库"

Gowin 随 IDE 提供的是**未编译的 Verilog 源码**：

```
D:\Gowin\Gowin_V1.9.11.03_Education_x64\IDE\simlib\
├── gw1n\prim_sim.v     ← GW1N/GW1NR 系列
├── gw2a\prim_sim.v     ← GW2A/GW2AR 系列
└── gw5a\prim_sim.v     ← GW5A 系列  ★ 你的板子用这个（709 KB）
```

里面是 `PLL`、`OSC`、`GSR`、`LUT4`、`MUX2`… 这些**原语的仿真模型**。
ModelSim 不认 Gowin 的器件原语，必须先把这份源码编译成一个库，再在仿真时挂上。

**没做这一步的后果**：只要你的设计里例化了 PLL（`clk_gen.v` 一定会用到），
ModelSim 就会在 elaboration 阶段失败——因为找不到 `PLL` 这个 design unit。
这就是"两个软件没联系"的核心。

验证方法：

```powershell
# 看原语文件是否存在、是否包含你要的原语
Select-String -Path 'D:\Gowin\...\IDE\simlib\gw5a\prim_sim.v' -Pattern '^module (PLL|OSC|GSR)\b'
```

### 2.2 ModelSim 2020.4 的中文路径缺陷

复现命令（在任何含中文的路径下执行）：

```
vlib work
# ** Error: init_dbinfo() DATABASE ERROR:
#   (sqlite3_open C:/Users/<乱码>/.../work/_lib.qdb): unable to open database file
```

原因：2020.4 版把工作库的编译数据库交给 sqlite3，而路径是按 **ANSI 代码页**转换的。
路径里一旦有中文（最典型的就是 Windows 用户名是中文，`C:\Users\忆洛\...`），
转换后变成乱码，文件就打不开了。

**这条限制和你的代码、和 Gowin 都无关**，纯属工具缺陷。规避方法只有两个：

1. 把**编译数据库所在目录**（work 库）放到纯 ASCII 路径，例如 `D:\gowin_lib\work`；
2. 或者把整个工程搬到纯 ASCII 路径，例如 `D:\work\robot_project`。

> ⚠️ 注意：**源码**放在中文路径下是没问题的，ModelSim 读源码用的是另一套 IO。
> 出问题的只有**库目录**（`_lib.qdb` 那些文件）。

---

## 3. 一键修复

在 `sim\` 目录下执行：

```powershell
# 先诊断（不改任何文件）
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1 -Check

# 确认没有 FAIL 后正式执行
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1
```

脚本会做这几件事：

1. **自动探测** Gowin 与 ModelSim 的安装位置（也支持 `-GowinHome` / `-ModelsimHome` 手工指定）
2. 找到 `IDE\simlib\gw5a\prim_sim.v`，并确认里面含 `PLL` / `OSC` / `GSR`
3. **挑一个"纯 ASCII 且真的可写"的目录**放库
   （只看 ACL 不可靠——`C:\` 根目录的 ACL 上写着可写，非管理员照样被拒，
   所以脚本会实际写一个临时文件来验证）
4. 用 `vlib` + `vlog` 把原语编译成 `prim_gw5a` 库
5. 在**工程的 `modelsim.ini`**里建立映射（不动 ModelSim 安装目录，避免污染）
6. 生成 `compile_all.do` 和 `run_sim.do`

常用参数：

```powershell
# 手工指定库位置（推荐放非系统盘，通常不需要管理员权限）
... -LibDir 'D:\gowin_lib'

# 指定 ModelSim（本机有两份：C: 与 D:）
... -ModelsimHome 'D:\modeltech64_2020.4'
```

---

## 4. 日常使用

```powershell
cd sim

# 编译（自动挂 Gowin 原语库）
vsim -c -do compile_all.do

# 跑全部 Testbench
vsim -c -do run_sim.do

# 只跑一个
vsim -c -do "do run_sim.do tb_uart"

# 看波形（GUI）
vsim -do run_sim.do
```

生成的 `compile_all.do` 关键三句（理解这三句就理解了整个联系）：

```tcl
vlib  $WORK_DIR                  ;# work 库建在纯 ASCII 路径 —— 绕开中文路径 bug
vmap  work $WORK_DIR
vmap  prim_gw5a $GOWIN_LIB       ;# ★ 把 Gowin 原语库挂上 —— PLL/OSC 才能解析
vlog  -work work ../rtl/xxx.v    ;# 直接引用工程源码, 不用拷贝
```

仿真时还要在 `vsim` 上加 `-L`：

```tcl
vsim -L work -L prim_gw5a work.tb_top_integration
```

`-L` 的意思是"允许从这里解析未定义的模块"，漏了它照样找不到 `PLL`。

---

## 5. 在 Gowin IDE 里指向 ModelSim

Gowin 自己也能调起 ModelSim，配置在：

```
Gowin IDE → Tools → Options → 外部工具/仿真器设置
   Simulator     : ModelSim
   Executable    : <ModelSim>\win64\vsim.exe
```

配好之后：
- **Tools → Simulate** 或工具栏仿真按钮会调起 ModelSim
- Gowin 会生成自己的 `.do`，里面用**相对路径**引用源文件

> 即使 Gowin 能调起 ModelSim，**原语库仍然要单独编译**（§3 那一步）。
> IDE 不会自动帮你编译 `prim_sim.v`。

---

## 6. 本机实测结论（2026-10-04）

| 项目 | 结论 |
|---|---|
| Gowin | `D:\Gowin\Gowin_V1.9.11.03_Education_x64`，GW5AST-138 器件支持齐全 |
| Gowin 仿真库 | `IDE\simlib\gw5a\prim_sim.v` 存在，709,303 字节，含 `PLL`/`OSC`/`GSR` |
| ModelSim | 两份：`C:\modeltech64_2020.4`（不可写）、`D:\modeltech64_2020.4`（可写） |
| ModelSim 许可 | ✅ 正常，`D:\modeltech64_2020.4\win64\LICENSE.TXT` 是真许可（`SERVER YILUO-LAPTOP ... 27001`），已通过 `LM_LICENSE_FILE` 配好，`vsim -version` 能正常输出版本 |
| **Gowin 库是否已编译进 ModelSim** | ❌ **没有** —— 这就是要修的主因 |
| 环境变量 | 没有任何 `MODELSIM_HOME` / 仿真器相关配置，说明 Gowin 侧也没指过 ModelSim |
| 中文路径 | 用户名 `忆洛` 导致 `C:\Users\忆洛\...` 无法建库（已复现） |
| Gowin 工程 `.gprj` | **未找到** —— 你的 Gowin 工程不在这个工作区里 |

---

## 8. 验证状态与已知风险（请务必读）

我把能在这台机器上验证的都验证了，**验证不了的明确列出来**，不含糊。

### 8.1 已经实测确认的

| 结论 | 验证方式 |
|---|---|
| Gowin 装在 `D:\Gowin\Gowin_V1.9.11.03_Education_x64`，支持 GW5AST-138 | 目录 + 器件 ini 检查 |
| GW5A 原语文件存在且完整 | `IDE\simlib\gw5a\prim_sim.v`，709,303 字节 |
| 里面确实有 `PLL` / `OSC` / `GSR` | 正则匹配确认 |
| **Gowin 库从未编译进 ModelSim** | `C:\modeltech64_2020.4\gw5a` 不存在 |
| Gowin IDE 里从未配过 ModelSim | 整个 IDE 目录搜不到 `modelsim` 关键字；注册表只有 ModelSim 自己的项 |
| ModelSim 许可正常 | `vsim -version` 正常输出版本；`LICENSE.TXT` 是真许可 |
| 两份 ModelSim 都是 2020.4 SE-64 | 分别执行 `vsim -version` |
| **中文路径下 ModelSim 必然建库失败** | 实际复现出 `sqlite3_open ... unable to open database file` |
| `vmap` 库映射本身工作正常 | 实际执行 `vmap` 并查看映射表 |
| 生成的 `.do` 路径已转正斜杠、Tcl 转义正确 | 实际生成并逐行核对输出 |
| 两个 PowerShell 脚本无语法错误 | 用 `Parser::ParseFile` 做 AST 校验，0 错误 |

### 8.2 ❗ 我无法验证、必须你来跑的两件事

本机是受限沙箱环境（禁止写工作区之外的任何路径），所以下面两条只能由你在真实机器上确认：

**① `D:\modeltech64_2020.4`（或你选的 `-LibDir`）是否真的可写**

沙箱把所有 `D:\` 写入都拒了。真实权限待确认：

```powershell
# 先测一下能不能写
New-Item -ItemType Directory -Force -Path 'D:\gowin_lib' | Out-Null
'probe' | Set-Content 'D:\gowin_lib\_probe.txt'
Remove-Item 'D:\gowin_lib\_probe.txt'
```

- 能写 → 完美，直接 `-LibDir 'D:\gowin_lib'`
- 报 `Permission denied` → 换个你有权限的盘，或用**管理员身份**运行 PowerShell
  （注意：`C:\` 根目录、`Program Files` 对普通用户都不可写）

**② Gowin 那份 709KB 的 `prim_sim.v` 能否被 ModelSim 2020.4 成功编译**

这是整个方案的关键前提。风险在于 `prim_sim.v` 里用了 Verilog **UDP（用户自定义原语）**，
例如文件开头就是：

```verilog
primitive mux2 (O, I0, I1, S);
  ...
endprimitive
```

UDP 是标准 Verilog-1364 特性，ModelSim 支持它，**大概率没问题**。
但我没能在本机跑一次，所以不能替你打包票。跑完看退出码即可：

```powershell
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1
```

- 看到 `[ OK ] 原语库编译完成` → 通了，继续 §4
- 看到 `[FAIL] vlog 编译原语库失败` → 把报错原文发我，一般是某条 UDP 语法或器件族版本问题

### 8.3 建议的执行顺序

```powershell
# 第 1 步：诊断（只会建一个临时库再删掉，安全）
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1 -Check -LibDir 'D:\gowin_lib'

# 第 2 步：正式执行
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1 -LibDir 'D:\gowin_lib'

# 第 3 步：编译 + 仿真
& 'C:\modeltech64_2020.4\win64\vsim.exe' -c -do compile_all.do
& 'C:\modeltech64_2020.4\win64\vsim.exe' -c -do run_sim.do
```

把每一步的完整输出发我，我按实际情况继续修。

---

## 9. 下一步需要你确认的

1. **你的 Gowin 工程在哪？** 扫描 `C:\Users\忆洛\Documents`、`Desktop`、`D:\`、`E:\`
   都没找到 `.gprj`。把路径告诉我，我可以：
   - 把 `toolchain\` 直接放进你的工程
   - 按你工程里的**实际文件列表**生成 `compile_all.do`（避免"少编一个文件"这类隐蔽错误）
   - 用 `gw_sh.exe` 命令行跑一遍综合，先确认综合能过

2. **器件是不是 `GW5AST-138`？** ✅ 已确认是 GW5AST-138（PBGA484A）。
   原语库文件相同（都在 `simlib\gw5a\`），但脚本里的族名和工程约束要同步。

3. **要不要把 PLL 的 IP 一起生成好？** `clk_gen.v` 需要例化 Gowin PLL：
   - 推荐用 Gowin IDE 的 **IP Core Generator** 生成（图形界面填参数，不易出错）
   - 也可以手写 `PLL` 原语例化，我能写，但需要你给出输入/输出频率

---

# ★ 2026-10-15 更新：链路已打通 + 修了两个真问题

> 上面 §1~§9 是 2026-10-04 的现场记录，**保留不改**（当时的判断有当时的依据）。
> 这一节是后来的实测结论，**与上面冲突时以本节为准**。

## U1. 先给结论

| 问题 | 结论 |
|---|---|
| Gowin V1.9.12 ↔ ModelSim 链路通不通？ | ✅ **通**。V1.9.12 的 `prim_sim.v` 编译进 ModelSim：**0 errors / 0 warnings / 132 个模块** |
| 之前那份库有没有问题？ | ❌ **有，而且很隐蔽**：它是用 **V1.9.11** 的 `prim_sim.v` 建的（见 U2）。已重建为 V1.9.12 |
| §8.2 里"我无法验证的两件事" | ✅ 两条现在都实测结掉了：库目录可写、709KB 的 `prim_sim.v` 能被 ModelSim 2020.4 正常编译（UDF 没有任何问题） |
| 器件型号 | ⚠️ **上面 §6/§8.1/§9.2 写的 `GW5AST-138` 是错的**，本板是 **`GW5AT-138`**（ID `0x0011081B`，差一位）。已同步修正 `clk_gen.v` 等文档 |

## U2. 那个隐蔽问题：库是旧版本建的（怎么发现的）

**症状**：`gw5a` 库存在、能用、回归全绿 —— 但它是 **Gowin V1.9.11** 的仿真模型，
而综合/烧录用的是 **V1.9.12**。**仿真模型和综合器不是一个版本**，
出问题时属于"仿真过了、上板不对"那一类最难查的。

**三条独立证据**：

| 证据 | 内容 |
|---|---|
| 时间线 | V1.9.11 于 **09-17 13:55** 安装；库文件 `_info` 建于 **09-17 18:30**；V1.9.12 到 **10-13 19:32** 才装 —— 建库时 V1.9.12 还不存在 |
| 内容 | 库里有 `MODULE DQS`。**V1.9.12 把 `DQS` 从 `prim_sim.v` 移走了**（挪进 `DQS_*.vp`），只有 V1.9.11 的 `prim_sim.v` 里有它 |
| 计数 | 库 133 个模块（V1.9.11）vs V1.9.12 的 132 个 |

**判版本的一句话命令**（以后换 Gowin 版本时先跑这个）：

```powershell
& 'D:\modeltech64_2020.4\win64\vdir.exe' -lib 'D:/modeltech64_2020.4/gw5a' | findstr "MODULE DQS"
# 有输出 -> V1.9.11 建的（过期）
# 无输出 -> V1.9.12 建的（正确，模块总数 132）
```

**修复动作**（已完成，可回退）：

```powershell
# 旧库原样备份（没有删）
D:\modeltech64_2020.4\gw5a_v1911_backup_20261015
# 新库 = V1.9.12 的 prim_sim.v 编译出来的，路径/库名都不变（.do 不用改）
D:\modeltech64_2020.4\gw5a
```

## U3. 修掉的第二个问题：重跑 setup 会把库"建到别处"

`setup_gowin_modelsim.ps1` 原来**不看工程实际用的库**，一律建成
`<ModelSim 同级>\gowin_lib\prim_gw5a`；而工程用的是 `<ModelSim>\gw5a`。
后果是"照文档重跑一次 setup"就会：库名和路径都变 → `.do` 被重新指向新库
→ **和原来那份库分家，而且不报任何错**。它还可能在**多个 Gowin 版本**里
取到旧的那个（这正是 U2 的成因）。

已修：

| 改动 | 效果 |
|---|---|
| 自动探测时**按版本号取最新** | 同时装着 V1.9.11 和 V1.9.12 时，稳定选 V1.9.12（原来靠目录枚举顺序） |
| 读 `toolchain.local.ps1` 的 `GowinSimLib` | 库建到**工程实际使用的位置**，库名取其最后一段（= `gw5a`），`.do` 不会被改指 |
| 读 `toolchain.local.ps1` 的 `WorkDir` | 不再把 work 库改到 `sim\work`，和工程一致用 `.simlib\work` |
| 没配置文件时 | 行为完全不变（队友不受影响） |

## U4. `toolchain.local.ps1` 必须存在（原来缺了）

模板在 `sim\toolchain\toolchain.local.ps1.template`，**本机这份之前没建**。
不建的话：脚本走内置默认值，`emit_dofiles.ps1` 会把 work 库写到
`D:\gowin_lib\work`（仓库外），而工程一直用 `.simlib\work` —— 库一分家，
换机器/换目录就找不到，且不报错。

已按本机实际情况建好（该文件被 `.gitignore` 排除，不入库）。
**验证方式**：不带任何参数跑一次生成器，产物应与显式传参**完全一致**（已实测一致）：

```powershell
cd sim\toolchain
powershell -ExecutionPolicy Bypass -File emit_dofiles.ps1
```

## U5. 新增的回归覆盖：`tb_gowin_prim`（17 项）

**这是本次最重要的"防复发"措施。** 原来工程的 12 条 tb **全是纯 RTL**，
一个 Gowin 原语都不碰（`clk_gen` 默认 `BYPASS=1` 是纯连线）——
也就是说：**原语库没挂上、版本不对、路径不对，原来的 429 项回归照样全绿。**
等真上 PLL 才会突然报 `Instantiation of 'PLL' failed`。

`sim/tb_gowin_prim.v` 故意例化真实原语并检查行为：

| 原语 | 查什么 |
|---|---|
| `LUT4` | `INIT` 位序（I0 是最低位）+ 组合查表 |
| `IOBUF` | `OEN` 方向控制、释放后靠上拉、`O` 跟随 `IO` |
| `GSR` | 单元可解析（能 elaborate） |
| `OSC` | 真的在翻转（2000ns 内 ≥50 个上升沿） |

> ⚠️ 这里**故意不含 `PLL`**：GW5A 的 PLL 模型会按参数实时算时钟周期，
> 参数配错时可能出现 0 周期（`#0` 死循环 → 整个仿真挂住）。
> PLL 改用下面 U5.1 的"只 elaborate、不跑时间"的办法单独验。

### U5.1 PLL 原语能不能被解析 —— 用 `vopt` 验（一正一反）

`clk_gen.v` 里最吓人的那句报错是
`Instantiation of 'PLL' failed. The design unit was not found`，
而它**只在真的例化 PLL 时才会出现** —— 工程现在 `BYPASS=1`（纯连线），
所以整条回归碰不到 PLL。为此加了一个只做 elaborate 的探针
`sim/pll_elab_probe.v`（已进 `compile_all.do` 的编译列表，**不进** TB_LIST）。

判据用 `vopt` 而不是 `vsim`（后者只给一句没细节的 `Error loading design`）：

```powershell
cd D:\dsh-ws\robot_project\sim

# 正向：挂了库 -> 期望 "Errors: 0, Warnings: 0"，退出码 0
& 'D:\modeltech64_2020.4\win64\vopt.exe' -work work pll_elab_probe -o pll_opt -L gw5a

# 反向对照：故意不挂库 -> 期望报 PLL 找不到（证明这个检查真的有效）
& 'D:\modeltech64_2020.4\win64\vopt.exe' -work work pll_elab_probe -o pll_opt2
```

**实测（2026-10-15，库 = V1.9.12 的 `prim_sim.v`）**：

| 命令 | 输出 | 退出码 |
|---|---|---|
| 带 `-L gw5a` | `Errors: 0, Warnings: 0` | 0 |
| 不带 `-L gw5a` | `** Error: pll_elab_probe.v(<PLL 例化那一行>): Module 'PLL' is not defined.`（行号随文件改动会变） | 2 |

> ⚠️ 探针里的 `$display` 在 `vsim -c` 批处理下**不会**出现在 stdout
> （`run 0` 也一样），所以**不要拿它当判据** —— 看 `vopt` 的 Errors / 退出码。
>
> 这个探针**只证明"PLL 单元能被解析"**，**不证明分频比/LOCK 正确** ——
> 后者必须用 IP Core Generator 生成参数后再单独做（见 U6）。

## U6. 还没做的（下一步）

1. **真正的 PLL 还没生成**：`clk_gen.v` 的 `g_pll` 分支仍是空的
   （`BYPASS=1` 走 50MHz，`BYPASS=0` 会故意 `locked=0` 让板子不动，现象明显）。
   生成时注意：器件选 **GW5AT-138**、Gowin 用 **V1.9.12**。
   ⚠️ 换 100MHz 前先看时序：`dist_mm → m_en` 那条路径 50MHz 下 slack 只有 2.09ns，
   100MHz（周期 10ns）必然为负 —— 需要先把 `main_fsm` 的输出打一拍。
2. 库里同时有 `prim_tsim.v`（时序仿真模型，4.3MB）。做**门级/后仿**时才需要它，
   现在的功能仿真用 `prim_sim.v` 就对。

## U7. 本节涉及的命令（可复现）

```powershell
# 0) 本机路径配置（必须存在）
#    sim\toolchain\toolchain.local.ps1

# 1) 诊断 / 重建原语库（自动选最新 Gowin + 按配置定库名/路径）
cd D:\dsh-ws\robot_project\sim
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1 -Check
powershell -ExecutionPolicy Bypass -File toolchain\setup_gowin_modelsim.ps1

# 2) 判库版本（无输出 = V1.9.12，正确）
& 'D:\modeltech64_2020.4\win64\vdir.exe' -lib 'D:/modeltech64_2020.4/gw5a' | findstr "MODULE DQS"

# 3) 编译 + 全量回归（13 条 tb / 446 项）
$env:LM_LICENSE_FILE   = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
$env:MGLS_LICENSE_FILE = 'D:\modeltech64_2020.4\win64\LICENSE.TXT'
& 'D:\modeltech64_2020.4\win64\vsim.exe' -c -do compile_all.do
& 'D:\modeltech64_2020.4\win64\vsim.exe' -c -do run_sim.do

# 4) 只验原语链（最快，17 项）
& 'D:\modeltech64_2020.4\win64\vsim.exe' -c -do "set TB_SEL tb_gowin_prim; do run_sim.do"
```

**实测结果**：`setup` 选到 `D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64`，
库名 `gw5a`，编译 `Errors: 0, Warnings: 0`；
`vdir` 132 个模块、无 `DQS`；
`tb_gowin_prim` 17/17 PASS；
全量回归 **13/13 PASS，446 项检查，0 错误**。

