# 验证报告

> 本文件由 sim\toolchain\run_tb.ps1 自动生成，**可以提交**。
> 文件名带【用户 + 机器 + commit】，所以三个人各生成各的，**永远不会冲突**。
> 想看谁验证过什么：`git log --oneline verify/`

| 项 | 值 |
|---|---|
| 时间 | 2026-10-28 23:18:01 |
| 机器 | YILUO-LAPTOP |
| 用户 | 忆洛 |
| 分支 | main |
| commit | `5b1bd03` (dirty（有未提交改动）) |
| 仿真工具 | D:\modeltech64_2020.4\win64\vsim.exe |
| 工作库 | D:\dsh-ws\robot_project\.simlib\work |
| **总计** | **24 个 tb / 914 项检查 / 0 个错误** |
| **结论** | **PASS** |

## 各 Testbench

| Testbench | checks | errors | 结果 |
|---|---|---|---|
| clk_gen | 9 | 0 | PASS |
| config_rx | 126 | 0 | PASS |
| encoder_decode | 17 | 0 | PASS |
| gowin_prim | 17 | 0 | PASS |
| imu_driver | 53 | 0 | PASS |
| main_fsm | 110 | 0 | PASS |
| motion_decision | 32 | 0 | PASS |
| motion_loop | 10 | 0 | PASS |
| motor_guard | 41 | 0 | PASS |
| pca9685_drv | 25 | 0 | PASS |
| pwm_motor | 21 | 0 | PASS |
| rc_rx | 28 | 0 | PASS |
| reset_sync | 11 | 0 | PASS |
| servo_lift | 25 | 0 | PASS |
| sr04 | 53 | 0 | PASS |
| telemetry_check | 3 | 0 | PASS |
| telemetry_payload | 27 | 0 | PASS |
| tof_driver | 68 | 0 | PASS |
| top_i2c_share | 28 | 0 | PASS |
| top_integration | 67 | 0 | PASS |
| top_servo_pca | 26 | 0 | PASS |
| top_tof_i2c | 42 | 0 | PASS |
| uart_frame | 58 | 0 | PASS |
| uart | 17 | 0 | PASS |

---

**队友注意**：如果你在 `main` 上看到某个 tb 是 FAIL，说明有人提交了没跑通的代码 ——
在群里问一下，别直接在上面继续改。
