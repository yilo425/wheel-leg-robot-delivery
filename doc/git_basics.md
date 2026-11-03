# Git 从零上手（本工程专用）

> **这份文档假设你完全没用过 Git。**
> 所有命令都可以直接复制执行，每一步都有"怎么验证做对了"。
>
> 团队协作的**规则**（分支策略、接口冻结、commit 规范）在
> [`team_sync_guide.md`](team_sync_guide.md)，本文只讲**怎么操作**。

---

## ⚡ 懒得看文档？跑一个脚本就行

工程根目录有一个 **`setup_git.ps1`**，右键 → 使用 PowerShell 运行即可。

它会问你 **3 个问题**，每个都能**直接回车**：

```
[1/6] 检查 Git ...              ← 自动
[2/6] 检查仓库 ...              ← 自动
[3/6] 你的名字                  ← 回车 = 用你的 Windows 用户名
[4/6] 你的邮箱                  ← 回车 = 跳过（不影响使用）
[5/6] Gitee 仓库地址            ← 粘贴地址，或回车 = 以后再说
[6/6] 推送                      ← 自动
```

**它自动做完**：配置身份 → 修掉旧提交的占位作者 → 生成 SSH 密钥 →
打印要粘到 Gitee 的公钥 → 绑定远程 → 推送 → 建三个分支。

> ⚠️ **必须在 PowerShell 窗口里运行**，不能用管道喂输入。
> 脚本会自己检查并拒绝执行 —— 管道喂 `Read-Host` 的返回值不可预期，
> 会把 git 配置写成占位文本（实测踩过，已加防护）。

**配置完就不用再跑它了。** 日常只用下面 5 条命令。

---

## 日常只要记 5 条命令

```powershell
git pull                              # ① 开工前：拿队友的最新改动
git status                            # ② 看自己改了哪些
git add -A                            # ③ 把改动加入本次提交
git commit -m "说明"                  # ④ 记录这次改动
git push                              # ⑤ 推给队友
```

```
pull ──► 干活 ──► add ──► commit ──► push
  ↑                                     │
  └────────── 明天再来 ─────────────────┘
```

**三条纪律**：
1. **每天开工先 `git pull`**
2. **只有仿真全 PASS 才合并进 `main`**
3. **改协议 = 独立 commit + 双方确认 + 升版本号**

---

> **下面是从零开始的完整说明。** 上面的脚本够用了，想搞懂原理再看。

---

## 0. 一句话理解 Git

**Git 就是一个"带历史记录的共享文件夹"。**

| 你熟悉的做法 | Git 的做法 |
|---|---|
| 改完文件，压缩成 zip 发群里 | `git commit` + `git push` |
| 队友下载 zip 覆盖自己的 | `git pull` |
| 想回到上周三能跑的版本 | `git checkout <那次提交>` |
| "这行代码谁改的？为什么？" | `git log -p <文件>` |
| 两个人同时改了同一个文件 → 丢一份 | **Git 会标出冲突，让你决定** |

**核心区别**：zip 是**全量覆盖**，Git 是**增量合并**。
三个人改同一个工程，zip 方案第 2 周必出问题。

---

## 1. 先看你在哪一步

```powershell
cd <你的>\robot_project
git status
```

| 你看到什么 | 你该看哪一节 |
|---|---|
| `fatal: not a git repository` | 工程还没有 Git 仓库 → 找 A 要仓库地址，看 **§3 克隆** |
| 正常显示分支和文件状态 | 已就绪 → 跳到 **§4 日常循环** |
| 想从头配一遍 | 继续往下看 |

---

## 2. 一次性配置（每人只做一次）

### 2.1 配置你的身份

**提交记录里会显示这个名字和邮箱**，队友靠它知道是谁改的。

```powershell
git config --global user.name  "你的名字"
git config --global user.email "你的邮箱"

# 验证
git config --global user.name
git config --global user.email
```

> **邮箱建议用你 Gitee 账号的邮箱** —— 这样 Gitee 能把提交关联到你的账号，
> 贡献统计里能看到你。
>
> `--global` 表示对这台机器上所有仓库生效，只需配一次。
> 如果只想对这个工程生效，去掉 `--global`（然后在工程目录里执行）。

### 2.2 检查 Git 能不能用

```powershell
git --version
# 期望：git version 2.x.x
```

没装的话去 <https://git-scm.com/download/win> 下载，一路默认即可。

---

## 3. 拿到仓库（队友第一次做）

### 3.1 克隆

```powershell
cd <你想放工程的目录>          # 注意：路径别带中文！
git clone <A 给你的仓库地址>
cd robot_project
```

### 3.2 ⚠️ 路径不要带中文

ModelSim 2020.4 **无法在含中文的路径下创建 work 库**。本机用户名是「忆洛」，
所以 `C:\Users\忆洛\...` 下的工程**必然出问题**。

**建议把工程放在 `D:\` 下**，例如 `D:\robot\`。

> 源码放在中文路径下**没问题**，出问题的是 work 库。
> `toolchain.local.ps1` 里可以把 WorkDir 指到纯 ASCII 路径。

### 3.3 切到你的分支

```powershell
git checkout vision     # B 用这个
git checkout mech       # C 用这个
# A 用 main 或者 fpga
```

> 分支就是"平行的版本线"。各人在自己的分支上干活，互不干扰，
> 只有验证通过才合并进 `main`。

---

## 4. 日常循环（每天重复）

```
   ┌─ ① git pull ──────────── 先拿队友的最新改动
   │
   │  ② 干活、改文件
   │
   │  ③ git status ────────── 看改了哪些
   │
   │  ④ git add + commit ──── 记录这次改动
   │
   └─ ⑤ git push ──────────── 推给队友
```

### ① 开工前：拉取最新

```powershell
git checkout main
git pull                        # 拿最新稳定版
git checkout fpga               # 回自己的分支（B 用 vision，C 用 mech）
git merge main                  # 把稳定版的改动合进来
```

> **为什么必须先 pull**：如果队友改了 `main` 而你不知道，
> 你后面提交时会产生冲突，解决起来更麻烦。

### ② 干活、改文件

正常改就行，和平时一样。

### ③ 看改了哪些

```powershell
git status
```

会看到类似：

```
修改尚未暂存以备提交的变更：
       修改：     rtl/app/uart_frame.v
       修改：     README.md
未跟踪的文件：
       doc/new_doc.md
```

| 词 | 意思 |
|---|---|
| **未跟踪** | 新文件，Git 还没开始管它 |
| **修改尚未暂存** | 改了但还没"放进这次提交" |
| **暂存** | 已放进这次提交，等着 `commit` |
| **工作区干净** | 没有未提交的改动 ✓ |

### ④ 提交这次改动

```powershell
git add -A                       # 把所有改动加入本次提交
git status                       # 再看一眼，确认没有不该提交的文件
git commit -m "uart_frame 接入 top，集成测试 52->61 项全过"
```

**commit 信息怎么写**：

| 好 | 差 |
|---|---|
| `uart_frame 接入 top，集成测试 52->61 项全过` | `更新` |
| `修复 TX 状态机优先级遮蔽：状态永不推进` | `改了点东西` |
| `删除过时的 simulate.do（tb 列表停在 5 个会假通过）` | `asdf` |

**为什么要写清楚**：三个月后你会靠这些信息找回"当时为什么这么改"。

### ⑤ 推给队友

```powershell
git push
```

第一次推新分支可能要：

```powershell
git push -u origin fpga        # -u 记住这个分支的上游，之后直接 git push
```

---

## 5. 验证通过后合并进 main

**只有跑通全部仿真才合并** —— `main` 是"给队友看的可信版本"。

```powershell
# ① 先在自己分支上跑一遍
cd sim
powershell -ExecutionPolicy Bypass -File toolchain\run_tb.ps1
#   确认 8 个 tb 全是 RESULT: PASS

# ② 提交验证报告（让队友知道你验证过了）
cd ..
git add verify/
git commit -m "验证: 8 tb / 271 checks 全过"

# ③ 合并进 main
git checkout main
git pull
git merge fpga
git push
```

---

## 6. 出错了怎么办（最常见的 6 种）

### ① 打错字了，想撤销还没提交的改动

```powershell
git checkout -- rtl/app/uart_frame.v     # 只撤销这一个文件
git checkout -- .                         # 撤销全部（★ 会丢工作，想清楚）
```

### ② commit 信息写错了（还没 push）

```powershell
git commit --amend -m "新的信息"
```

### ③ 忘了加一个文件进上次提交

```powershell
git add 忘记的文件
git commit --amend --no-edit             # 加进上次提交，信息不变
```

### ④ 想看看某个文件改了什么

```powershell
git diff rtl/app/uart_frame.v            # 和上次提交比
git log -p rtl/app/uart_frame.v          # 完整历史（谁什么时候改的）
git log --oneline -20                     # 最近 20 次提交，一行一条
```

### ⑤ pull 时说冲突（conflict）

**别慌**，这是正常的。打开冲突文件会看到：

```
<<<<<<< HEAD
（你改的）
=======
（队友改的）
>>>>>>> main
```

**处理原则**：

1. **先搞清楚对方为什么这么改**：`git log -p <文件>`
2. **不要直接选一边** —— 大部分情况是"两个改动都要保留"
3. 手动改成你要的最终内容，**删掉 `<<<<<<<` `=======` `>>>>>>>` 三行**
4. **改完必须重跑仿真**（否则可能把对方的逻辑改坏）
5. 完成合并：

```powershell
git add <冲突文件>
git commit
```

### ⑥ 彻底改乱了，想回到上次提交

```powershell
git reset --hard HEAD          # ★ 丢弃所有未提交改动，会丢工作
git checkout <commit号> -- <文件>   # 只把某个文件恢复到某个历史版本
```

---

## 7. 建远程仓库并推送（A 做一次）

**现在你的仓库只在本地，队友拿不到。** 需要一个远程仓库。

### 7.1 在 Gitee 建仓库

1. 打开 <https://gitee.com> 注册/登录
2. 右上角 `+` → **新建仓库**
3. 填写：
   - **仓库名称**：`robot-fpga`（随意）
   - **是否开源**：**私有**（比赛项目，别公开）
   - **不要**勾选"初始化仓库"（会和我们本地历史冲突）
4. 创建后复制仓库地址（SSH 形式：`git@gitee.com:你的账号/robot-fpga.git`）

### 7.2 配置 SSH 密钥（推荐，比每次输密码方便）

```powershell
# 生成密钥（一路回车即可）
ssh-keygen -t ed25519 -C "你的邮箱"

# 查看公钥
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub
```

把输出的**一整行**复制，粘到 Gitee → 设置 → **SSH 公钥** → 新建。

验证：

```powershell
ssh -T git@gitee.com
# 期望：Hi xxx! You've successfully authenticated...
```

### 7.3 绑定远程并推送

```powershell
cd <你的>\robot_project

git remote add origin git@gitee.com:你的账号/robot-fpga.git
git remote -v                     # 验证

git push -u origin main
```

### 7.4 建三个分支

```powershell
git branch fpga
git branch vision
git branch mech
git push origin fpga vision mech
```

### 7.5 通知队友

把仓库地址发群里，告诉他们：

```powershell
git clone git@gitee.com:你的账号/robot-fpga.git
cd robot-fpga
git checkout vision      # B
git checkout mech        # C
```

---

## 8. 不想敲命令？有图形界面

| 工具 | 说明 |
|---|---|
| **`gitk`** | 自带的历史查看器。`gitk --all` 看所有分支 |
| **`git-gui`** | 自带的提交界面。工程目录下敲 `git-gui` |
| **VS Code** | 侧边栏"源代码管理"，点点鼠标就能提交/推送（需先装） |
| **SourceTree / GitHub Desktop** | 更友好的第三方客户端 |

> **但建议至少学会 `git status` / `git add` / `git commit` / `git push` / `git pull` 这 5 条。**
> 出问题时图形界面帮不了你，命令行能。

---

## 9. 一页速查卡

```powershell
# ---- 每天开工 ----
git checkout main && git pull && git checkout <我的分支> && git merge main

# ---- 干完一段活 ----
git status                     # 看改了哪些
git add -A                     # 全部加入本次提交
git commit -m "说明"           # 提交
git push                       # 推给队友

# ---- 看历史 ----
git log --oneline -20                  # 最近 20 次
git log -p <文件>                      # 某文件的完整改动史
git diff                               # 未提交的改动
git status                             # 当前状态

# ---- 出事了 ----
git checkout -- <文件>          # 撤销这个文件的未提交改动
git commit --amend -m "新信息"  # 改最后一次提交的信息
git reset --hard HEAD           # ★ 丢弃全部未提交改动

# ---- 分支 ----
git branch                              # 看本地分支
git branch -a                           # 看所有分支（含远程）
git checkout vision                     # 切分支
git merge main                          # 把 main 合进当前分支
```

---

## 10. 三条纪律（写在这里是因为最重要）

1. **每天开工先 `git pull`**
2. **只有仿真全 PASS 才合并进 `main`**
3. **改协议 = 独立 commit + 双方确认 + 升版本号**

> 第 3 条最要紧：**FPGA 和 ARM 唯一的耦合点就是协议**。
> 一边改一边不改，联调那天现象是"通信全是乱码"，非常难查。

---

## 11. 还搞不定？

| 情况 | 怎么办 |
|---|---|
| 命令报错看不懂 | **把完整报错发群里**（不要只贴一行） |
| 不确定会不会丢工作 | 先 `git status` 和 `git diff`，**看清楚再动手** |
| 冲突解决不了 | **先别提交**，把 `git status` 发给 A |
| 真的搞乱了 | `git stash` 暂存改动，或者让 A 帮你从远程重新 clone |

**最后一条保险**：只要改动**已经 push 过**，就永远不会丢。
所以**每天下班前 push 一次**。
