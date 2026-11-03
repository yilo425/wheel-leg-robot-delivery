# 团队协作与进度同步方案

> **问题**：三个人各自在自己的机器上（各自的 DSH 环境）改同一个工程，
> 怎么保证大家改的是同一份、且不会互相覆盖？
>
> **结论**：**代码用 Git 同步，上下文用文档同步。** 这两件事必须分开做。
>
> 更新：2026-10-04

---

## 0. 先说结论：为什么不能用压缩包

现在的做法是打 `robot_project_handoff_*.zip` 发给队友。**这个方式有个致命缺陷**：

| 压缩包交换 | Git |
|---|---|
| **全量覆盖** —— 收到后你只能选择"全用他的"或"全用我的" | **增量合并** —— 能精确到"这一行谁改的、改成什么" |
| 无法回答"这行代码为什么变成这样" | `git log -p` 直接给出 |
| 两个人同时改 → **必然丢一份工作** | 冲突会被**显式标出**，由人决定怎么合 |
| 无法回退到"上周三那个能跑的版本" | `git checkout <commit>` 一条命令 |
| 一周后就没人知道哪个 zip 是最新的 | `main` 永远是大家认可的稳定版 |

**你还有 30 天，三个人并行开发。压缩包方案在第 2 周一定会出问题。**

> ⚠️ 压缩包**仍然有用** —— 用来发给**不写代码的人**（比如指导老师）、
> 或者作为"某个里程碑的存档"。但它**不能当日常同步手段**。

---

## 1. 关键区分：代码 vs 上下文

你们每个人有自己的 DSH 环境。**这里有一个容易混淆的点**：

| 要同步的东西 | 怎么同步 | 为什么 |
|---|---|---|
| **代码**（`.v` `.py` `.cst` `.md`） | **Git** | 需要合并、需要历史、需要回退 |
| **设计意图 / 踩过的坑 / 为什么这么设计** | **文档**（进 Git） | DSH 的**会话上下文不共享** —— 队友的 DSH 不知道我们这段对话 |
| **DSH 会话本身** | **不同步、也没法同步** | 每个会话是独立的 |

**所以 `doc/` 目录的重要性不亚于 `rtl/`。**

具体说：队友的 DSH 打开工程时，它**看不到**我们这几天讨论出的
"为什么急停要放在 `main_fsm` 里而不是 `top.v`"、"为什么 TX 状态机的分支顺序不能反"。
**这些必须落在文档里**，否则队友的 DSH 会重新踩一遍。

**已经准备好的文档**：

| 文档 | 承接什么上下文 |
|---|---|
| `interface_delivery.md` | 接口全貌、UART 协议、机械待办 |
| `design_guide.md` | **每个模块内部怎么运转、为什么这么设计** |
| `progress_review.md` | **踩过的坑与修复过程**（最重要的一份"防重复踩坑"） |
| `uart_frame_protocol.md` | 协议契约 |
| `team_handoff.md` | 分工与阻塞项 |

> **给队友的一句话**：**先读 `interface_delivery.md`，再让 DSH 读 `design_guide.md` 和 `progress_review.md`。**

---

## 2. 托管在哪里

| 方案 | 优点 | 缺点 | 建议 |
|---|---|---|---|
| **Gitee（码云）** | 国内快、免费私有仓库、学生认证送额度 | — | **★ 推荐** |
| GitHub | 生态好 | 国内访问不稳，队友可能拉不下来 | 备选 |
| 自建 / 局域网 Git | 完全可控 | 要维护、队友不在同一网络就废了 | 不推荐 |
| U 盘 / 群文件 | 零成本 | 就是压缩包方案 | ❌ |

**用私有仓库**（比赛项目，别公开）。

---

## 3. 仓库结构

**只提交这一层**：

```
robot_project/            ← Git 仓库根目录
├── .gitignore            ← ★ 关键，见下
├── README.md
├── rtl/                  ← 源码，入库
├── sim/                  ← Testbench + toolchain 脚本，入库
│   └── (work/ 等生成物不入库)
├── tools/                ← Python 检查工具，入库
├── constraints/          ← .cst 引脚约束，入库
└── doc/                  ← ★ 文档，必须入库
```

### `.gitignore` 已经配好

最容易被忽略的是 **`*_result.txt`** —— 仿真结果文件每次跑都变，
三个人各自跑一次就产生"冲突"，而冲突内容毫无意义（只是时间戳不同），
**会淹没真正的代码改动**。已经排除。

---

## 4. 分支策略（简单版，够用）

**不要用复杂的分支模型。三个人、30 天，用这个就行：**

```
main          ← 稳定版，任何时候 checkout 都能跑通全部仿真
 ├── fpga     ← A 的日常分支（FPGA 实时控制）
 ├── vision   ← B 的日常分支（ARM 视觉与通信）
 └── mech     ← C 的日常分支（机械/集成文档、引脚）
```

**规则只有两条**：

1. **各人在自己的分支上干活，随便提交**
2. **只有"8 个 Testbench 全 PASS"时才合并进 `main`**

> 为什么这么定：`main` 是"给队友看的可信版本"。
> 如果 `main` 上跑不通，队友 pull 下来会一脸茫然，然后开始互相怀疑。

---

## 5. 每天的日常命令

### 开始干活前（★ 必须先 pull）

```bash
git checkout main
git pull                       # 先拿最新的稳定版
git checkout fpga
git merge main                 # 把稳定版的改动合进自己的分支
```

### 干完一段活

```bash
git add -A
git commit -m "uart_frame 接入 top，tb_top_integration 52->61 项全过"
git push
```

### 验证通过后，合并进 main

```bash
# 1) 先在自己的分支上跑全部仿真
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1
# 确认 8 个 tb 全 PASS

# 2) 再合并
git checkout main
git pull
git merge fpga
git push
```

### 查看谁改了什么

```bash
git log --oneline -20                  # 最近 20 次提交
git log -p rtl/app/uart_frame.v        # 某个文件的完整改动历史
git diff main fpga                     # 两个分支的差异
```

---

## 6. ★ 验证记录：`verify/` 目录

**Git 能同步代码，但回答不了"这个版本谁跑通过"。** 这个需求由 `verify/` 解决。

### 怎么工作

每次跑完 `run_tb.ps1`，脚本**自动**在 `verify/` 下生成一个报告：

```
verify/忆洛_DESKTOP-A1B2_47a2f59.md
verify/B_ThinkPad_47a2f59.md          ← 队友对同一个 commit 的独立验证
```

**文件名 = `<用户>_<机器名>_<commit>[_dirty].md`**

**因为文件名唯一，三个人各生成各的 —— `git pull` 只会"多一个文件"，永远不会冲突。**

### 报告内容

| 项 | 值 |
|---|---|
| 时间 / 机器 / 用户 / 分支 | 谁在哪台机器上跑的 |
| **commit** | **对应哪个版本**（关键） |
| 总计 | N 个 tb / M 项检查 / K 个错误 |
| **结论** | **PASS / FAIL** |
| 各 Testbench | 逐个列出 checks / errors / 结果 |

### 怎么用

```bash
# 跑完仿真后
git add verify/
git commit -m "验证: 47a2f59 PASS (8 tb / 271 checks)"
git push

# 看谁验证过什么
git log --oneline verify/
ls verify/

# 拉最新代码后先看一眼
git pull
ls verify/ | sort
```

### `_dirty` 后缀的含义

如果报告文件名带 `_dirty`，表示**跑的时候工作区有未提交改动**。

**这种报告只能当参考，不能作为"这个 commit 验证通过"的证据。**

### ⚠️ 它不替代合并前的自查

| 手段 | 作用 |
|---|---|
| `verify/` 报告 | **记录**"谁验证过什么"（事后可追溯） |
| 合并前跑一遍 | **保证**"进 `main` 的都是跑通的" |

**两条都要有。** 只靠报告是事后才发现；只靠自查是没人知道。

---

## 7. ★ 接口冻结规则（最重要的协作纪律）

**你们已经有一份现成的契约**：`doc/uart_frame_protocol.md` 和 `doc/interface_delivery.md`。

**规则**：

| 改什么 | 需要谁同意 |
|---|---|
| **帧格式 / 帧类型 / 类别编码 / 波特率** | **A 和 B 都要同意**，且**必须同步改protocol 文档** |
| `top.v` 的端口增减 | A 决定，但要**通知 B 和 C**（C 要重新排引脚） |
| 引脚分配 | C 统筹 |
| 模块内部实现（不改接口） | 各自决定，不用问 |

**为什么这条最重要**：FPGA 和 ARM 是两个独立开发的模块，**唯一的耦合点就是协议**。
协议一变，两边不同步改，联调那天必然对不上，而且**现象是"通信全是乱码"**，很难查。

**具体做法**：协议改动**必须**是一个独立的 commit，message 里写清楚：

```
协议: VISION_RESULT 增加"目标宽度"字段 (7B -> 9B/目标)

影响: ARM 侧 struct 要改, FPGA 侧 uart_frame 的载荷解析要改
版本: 0x01 -> 0x02 (不兼容变更必须升版本号)
双方确认: A ✓  B ✓
```

---

## 8. ⚠️ 硬编码路径问题（队友会直接踩到）

**当前工程里有很多写死的绝对路径**，队友的目录结构不一样，**脚本会直接跑不了**。

### 现状

| 类型 | 出现位置 | 数量 |
|---|---|---|
| `D:\robot_trial` | `sim/toolchain/*.ps1`、多份文档 | 6 个文件 |
| `C:\Users\忆洛` | `sim/toolchain/start_here.ps1`、多份文档 | 5 个文件 |
| `D:\gowin_lib` | `run_tb.ps1`、`emit_dofiles.ps1` 等 | 11 个文件 |
| `C:\modeltech64_2020.4` / `D:\modeltech64_2020.4` | 多处 | 12 个文件 |
| `D:\Gowin\...` | 多份文档 | 13 个文件 |

### 两类路径要分开处理

| 类别 | 例子 | 怎么处理 |
|---|---|---|
| **工具安装路径** | `D:\modeltech64_2020.4`、`D:\Gowin\...` | 队友**按同样位置安装**就一致 —— 写进文档要求即可 |
| **工作目录 / 库目录** | `D:\robot_trial`、`D:\gowin_lib` | **各人不同，必须参数化** |

### 推荐做法：本地配置文件（不进 Git）

**脚本已经支持参数了**：

```powershell
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1 tb_uart_frame `
  -WorkDir 'D:\gowin_lib\work' `
  -LibPath 'D:\modeltech64_2020.4\gw5a' `
  -ModelsimIni 'C:\modeltech64_2020.4\modelsim.ini'
```

**建议再加一层**：每个队友在 `sim/toolchain/` 下建一个 `toolchain.local.ps1`
（**已被 `.gitignore` 排除，不会入库**）：

```powershell
# toolchain.local.ps1 -- 本机专属路径，不要提交
$LocalPaths = @{
    WorkDir     = 'D:\gowin_lib\work'
    LibPath     = 'D:\modeltech64_2020.4\gw5a'
    ModelsimIni = 'C:\modeltech64_2020.4\modelsim.ini'
    TrialDir    = 'D:\robot_trial'
}
```

**⚠️ 这一层目前还没实现** —— 现在脚本仍要求每次手动传参。
队友第一次配环境时如果遇到"路径不对"，就是这里。

> **行动项**：把 `run_tb.ps1` / `emit_dofiles.ps1` 改成"优先读 `toolchain.local.ps1`，
> 没有则用默认值"。这样队友只要写一个本地文件，之后命令就统一了。

---

## 9. 遇到冲突怎么办

**冲突不可怕，可怕的是不知道怎么解。** 两种常见情况：

### 情况 A：`*_result.txt` 冲突

**不该发生** —— 如果发生了，说明 `.gitignore` 没生效：

```bash
git rm --cached sim/*_result.txt
git commit -m "把仿真结果文件移出版本控制"
```

### 情况 B：真的代码冲突

```bash
git pull                # 提示 conflict
git status              # 列出冲突文件
```

打开冲突文件会看到：

```
<<<<<<< HEAD
（你改的）
=======
（队友改的）
>>>>>>> fpga
```

**处理原则**：

1. **先看 `git log -p <文件>`**，搞清楚对方**为什么**这么改
2. **不要直接选一边** —— 大部分情况是"两个改动都要"
3. 改完删掉 `<<<<<<<` / `=======` / `>>>>>>>` 三行标记
4. **改完必须重跑仿真**，确认没把对方的逻辑改坏

```bash
git add <冲突文件>
git commit              # 完成合并
```

### 情况 C：改乱了想重来

```bash
git checkout -- <文件>          # 丢弃本地改动，恢复到上次提交
git reset --hard HEAD           # 丢弃**所有**未提交改动（★ 危险，会丢工作）
git checkout <commit> -- <文件> # 只把某个文件恢复到某个历史版本
```

---

## 10. 首次搭建步骤（A 做一次，然后通知队友）

```bash
# ---- 1) 在 robot_project 下初始化 ----
cd <你的>\robot_project
git init -b main

# ---- 2) 确认要入库的文件 ----
git add -A
git status                    # ★ 仔细看：work/ 和 *_result.txt 不该出现

# ---- 3) 首次提交 ----
git commit -m "初始提交: 驱动层+应用层+通信层全部仿真通过 (8 tb / 271 checks)"

# ---- 4) 建分支 ----
git branch fpga
git branch vision
git branch mech

# ---- 5) 推到 Gitee ----
# 先在 gitee.com 建一个**私有**仓库，然后：
git remote add origin git@gitee.com:<你的账号>/robot-fpga.git
git push -u origin main
git push origin fpga vision mech
```

**然后通知队友**：

```bash
git clone git@gitee.com:<账号>/robot-fpga.git
cd robot-fpga
git checkout vision        # B 用 vision，C 用 mech
```

**队友第一次要做的**：

1. 读完 `doc/interface_delivery.md`
2. 让他的 DSH 读 `doc/design_guide.md` 和 `doc/progress_review.md`（补上下文）
3. 建 `sim/toolchain/toolchain.local.ps1` 填自己的路径
4. 跑一次 `python tools/verify_uart_frame_model.py`（**不需要 ModelSim**，秒级）

---

## 11. 一页速查

```bash
# 每天开始
git checkout main && git pull && git checkout <我的分支> && git merge main

# 干完一段
git add -A && git commit -m "说明" && git push

# 仿真全过后并回 main
git checkout main && git pull && git merge <我的分支> && git push

# 看历史
git log --oneline -20
git log -p <文件>

# 出事了
git checkout -- <文件>        # 丢弃这个文件的改动
git reset --hard HEAD         # ★ 丢弃全部未提交改动
```

**三条纪律**：

1. **每天开工先 `git pull`**
2. **只有仿真全 PASS 才合并进 `main`**
3. **改协议 = 独立 commit + 双方确认 + 升版本号**

---

## 12. 与压缩包的分工

| 手段 | 用途 |
|---|---|
| **Git** | **日常同步（唯一手段）** |
| 压缩包 | 发给不写代码的人（老师/评委）；里程碑存档 |
| 文档 | 传递 DSH 会话里"看不见的上下文" |

**三者不冲突，但职责不能混。**
