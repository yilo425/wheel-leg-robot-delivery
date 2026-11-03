# -*- coding: utf-8 -*-
"""把站立高度接进 config_rx + top.v。

config_rx: 新增 CONFIG ID 0x21（0=高位/0状态, 1=低位），输出 stance_low
top.v    : 例化 stance_ctrl，接 config_rx 的 stance_low 和 pca9685_drv 的计数输入
"""
#===========================================================================
# 1) config_rx.v
#===========================================================================
p1 = r'rtl/app/config_rx.v'
s = open(p1, encoding='utf-8').read()

pairs = [
# 1.1 输出
("""    output reg         manual_sel,     // 0x20""",
 """    output reg         manual_sel,     // 0x20
    output reg         stance_low,     // ★ 0x21：0=高位(0状态) 1=低位"""),

# 1.2 ID
("""                     ID_MANUAL    = 8'h20;   // ★ 扩展：手动模式开关""",
 """                     ID_MANUAL    = 8'h20,   // ★ 扩展：手动模式开关
                     ID_STANCE    = 8'h21;   // ★ 扩展：站立高度（0=高位 1=低位）"""),

# 1.3 长度检查（1 字节）
("""                        ID_LIFT_H, ID_MANUAL: begin""",
 """                        ID_LIFT_H, ID_MANUAL, ID_STANCE: begin"""),

# 1.4 应用
("""                        ID_MANUAL: begin""",
 """                        ID_STANCE: begin
                            // 0x21 只接受 0/1（0=升高到 0 状态，1=降低）
                            if (val8_w > 8'd1) begin
                                cfg_bad     <= 1'b1;
                                cfg_bad_cnt <= sat16(cfg_bad_cnt);
                                state       <= S_IDLE;
                            end else begin
                                stance_low <= val8_w[0];
                                state      <= S_OK;
                            end
                        end

                        ID_MANUAL: begin"""),
]
for old, new in pairs:
    assert old in s, 'config_rx anchor: ' + old[:60]
    s = s.replace(old, new, 1)
open(p1, 'w', encoding='utf-8').write(s)
print('OK config_rx.v')

#===========================================================================
# 2) top.v
#===========================================================================
p2 = r'rtl/top.v'
s = open(p2, encoding='utf-8').read()

# 2.1 参数
old = """    parameter integer STEER_SHIFT   = 6,"""
new = """    parameter integer STEER_SHIFT   = 6,

    // ★ 2026-10-30：站立高度（遥控升高/降低）
    //   STEP_MS 越大越慢（限制舵机电流）：102 步 × 20ms ≈ 2 秒走完 45°
    parameter integer STANCE_STEP_MS   = 20,
    parameter integer STANCE_CNT_MID   = 307,   // 1.5ms 中立位（0 状态=高位）
    parameter integer STANCE_CNT_SPAN  = 102,   // 45° ≈ 0.5ms
"""
assert old in s, 'top param anchor'
s = s.replace(old, new, 1)

# 2.2 信号 + 例化（放在 g_servo_pca 之前）
old2 = """    generate
        if (SERVO_IF != 0) begin : g_servo_pca"""
new2 = """    //==================================================================
    // ★ 2026-10-30：站立高度控制（遥控"升高/降低"）
    //   0 状态 = 高位 = 舵机 1.5ms 中立位（装配中立）
    //   降低 = 前腿 +45°（顺时针）、后腿 -45°（逆时针），2 秒缓慢走完
    //==================================================================
    wire        stance_low;
    wire [11:0] stance_front;
    wire [11:0] stance_rear;
    wire [7:0]  stance_pct;
    wire        stance_at_low;
    wire        stance_at_high;

    stance_ctrl #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .STEP_MS     (STANCE_STEP_MS),
        .CNT_MID     (STANCE_CNT_MID),
        .CNT_SPAN    (STANCE_CNT_SPAN)
    ) u_stance (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .target_low (stance_low),
        .cnt_front  (stance_front),
        .cnt_rear   (stance_rear),
        .pos_pct    (stance_pct),
        .at_low     (stance_at_low),
        .at_high    (stance_at_high)
    );

    generate
        if (SERVO_IF != 0) begin : g_servo_pca"""
assert old2 in s, 'top generate anchor'
s = s.replace(old2, new2, 1)

# 2.3 pca9685_drv 实例加两个端口
old3 = """                .lift_cmd    (lift_cmd),
                .lift_height (cfg_lift_height),"""
new3 = """                .lift_cmd    (lift_cmd),
                .lift_height (cfg_lift_height),
                .stance_front(stance_front),
                .stance_rear (stance_rear),"""
assert old3 in s, 'drv ports anchor'
s = s.replace(old3, new3, 1)

open(p2, 'w', encoding='utf-8').write(s)
print('OK top.v (param + instance + drv ports)')
print()
print('⚠️ 还需要手工确认：config_rx 实例的 .stance_low(stance_low) 有没有接上')
for i, line in enumerate(s.split(chr(10)), 1):
    if 'config_rx #(' in line:
        print('  config_rx 实例在第 %d 行' % i)
