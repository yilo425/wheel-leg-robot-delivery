//=============================================================================
// body_v5.scad —— 轮足机器人车身 v5（在队友 body_v3 基础上改进）
//
// ★ 基础：队友 body_v3(2).scad（250×150，下层45，上层50）
//   保留不动的：舵机穿墙孔（42×21）+ 耳孔（3.2）+ 内壁加强筋、圆角壳体、
//               四角固定桩（±118,±68 实心、从底板长到顶）
//   改进的：
//     1. 电池区和隔板原来【撞在一起】（v3 隔板在 x=-41.7，电池槽到 x=-31）
//        → 隔板重排为 x=-33 / x=+60，中间留 y=±30 以外的走线通道
//     2. 按手绘布局放模块：后=电池+12→5降压；中=电机板+舵机板+端子排；
//        前=超声波+12→9降压
//     3. 每个模块都有固定：电机板/舵机板 四角边夹；降压/端子排/保险丝
//        浅沉台 + 扎带孔；电池槽 + 扎带
//     4. 电机线孔放大到 16×24（6 排线插头），左右对称
//     5. 分区矮墙 + 走线缺口，区块一眼看清
//
// ★ 接线规则（查历史会话 + doc/供电与接线方案.md + doc/接线总表.md 附录A）：
//   电池 XT60 → 5A 片式保险丝 → 船型开关 → 端子排【12V 汇流：红+/黑-】
//     12V 汇流 → 四路驱动板 Vin± (18AWG)
//     12V 汇流 → UBEC 12→6V/5A → 舵机分配板(+1000µF) → 4 舵机
//     12V 汇流 → DC-DC 12→9V/2A → 5.5×2.1 免焊插头 → FPGA DC 圆口
//   所以：端子排(12V汇流) 放在电池与两个降压之间；9V降压靠前（FPGA DC 线
//   从上层前部电源孔上来）；6V降压靠舵机板。
//
// 打印：PETG / 0.2 / 壁 3 圈 / 填充 15% / 不加支撑 / brim 5mm
//=============================================================================

// ---------------- 输出 ----------------
PART = "lower";           // "assembly" | "lower" | "upper" | "print"（两层并排，交付用）
SHOW_MODULES = false;     // ★ 导出打印件必须 false；核对布局时改 true
SHOW_HELPERS = false;

$fn = 48;

// ---------------- 总体（沿用 v3）----------------
L     = 242;      // ★ 236 -> 242（用户要求，给第二层留余量；252 台面每边 5mm）
W     = 150;
H1    = 45;       // 下层高
H2    = 50;       // 上层高
FLOOR = 3;
WALL  = 3;
CR    = 8;

// ---------------- 上下层连接（沿用 v3：四角实心桩）----------------
POST_D  = 10;
PILOT_D = 2.6;
POST_X  = L/2 - 7;   // 跟车身长度联动（L=222 -> 104），保证桩贴着前后壁不露头
POST_Y  = 68;

// ---------------- 舵机（沿用 v3，一字未改）----------------
SV_X    = 85;
ZV      = 26;
SV_HW   = 42;
SV_HH   = 21;
EAR_DX  = 49;
EAR_DY  = 10.5;
BOSS_H  = 6;

// ---------------- 电池（沿用 v3）----------------
BATT_L  = 91;
BATT_W  = 64;
BATT_D  = 2.0;    // 浅槽深度（3mm 底板留 1mm 膜，电池不会掉下去）

// ---------------- 传感器 ----------------
SR04_W  = 46;
SR04_H  = 21;
CAM_D   = 16;

// ---------------- FPGA / K230（上层）----------------
FX = 140; FY = 92; FH = 34;
KX = 56;  KY = 85;

//=============================================================================
// 模块布局（下层，按手绘）
//=============================================================================
// 电池：后排中间（离后壁留 4mm，否则电池槽和墙之间只剩 2mm 薄片 -> 非流形）
//   ★ Y 只能居中：4 个腿舵机在 x=±85 从两侧壁向内侵入到 y≈±30，
//     电池(61 宽)在后排正好跨过舵机 x，偏 Y 会撞舵机（见 find_nonmanifold 排查时的约束）
BATT_CX = -68.5;   BATT_CY = 0;   // 后沿 -114，离后壁 (-118) 留 4mm
// 电机驱动板 50×50（四角 φ3.2 孔）
//   ★ 2026-10-27 第六版：(26,-5) -> (21,-2) 即 X-5 / Y+3
DRV_CX  = 21;      DRV_CY  = -2;    DRV = 50;
DRV_HOLE_DX = 44;  DRV_HOLE_DY = 44;   // ★ 估的（照片四角内缩约 3mm），实测后改
// 舵机板 PCA9685 62.3×25.5×15（四角 φ3.2 孔）
//   ★ 2026-10-27 第四版：**Y +6**  →  (20,42) -> (20,48)
SB_CX   = 20;      SB_CY   = 48;    SB_W = 62.3;  SB_D = 25.5;  SB_H = 15;
SB_HOLE_DX = 56;   SB_HOLE_DY = 19;   // ★ 估的，实测后改
// 端子排（12V 汇流）：KF301-5.0，每路 5.0 间距，深 7.7 高 10
TERM_CX = 15;      TERM_CY = -67;   TERM_W = 76;  TERM_D = 8;
// 保险丝座 + 船型开关：电池正前方
//   ★ 2026-10-27：电机板后移到 y -40…+10，会和保险丝(y 6…24)撞
//     -> 保险丝往上挪 7mm（y 13…31），两边各留 3mm / 4mm
FUSE_CX = 20;     FUSE_CY = -45;    FUSE_W = 30;  FUSE_D = 18;
// 降压：LM2596S 43×21，四角 φ3.1 孔，孔距 30×15（实测数据）
BUCK_W  = 43;      BUCK_D  = 21;
BUCK_HOLE_DX = 30; BUCK_HOLE_DY = 15;
B5_CX   = -40;     B5_CY   = 49;    // 12→5（舵机）
B9_CX   = 83;      B9_CY   = 0;     // 12→9（FPGA），受 L 和 P2 限制

// ---------------- 分区矮墙 ----------------
P1_X = -20;        // 电池(-23) 与电机板(-8) 之间（L=242）
P2_X = 58;         // 中部 | 前部
P_H  = 12;         // 墙高
P_T  = 2.5;        // 墙厚
P_Y  = 30;         // 墙半宽（y=±30 以外留走线通道）

//=============================================================================
// 基础
//=============================================================================
module rrect(w, h, r) { offset(r = r) square([w - 2*r, h - 2*r], center = true); }

module shell(hgt) {
    difference() {
        linear_extrude(height = hgt) rrect(L, W, CR);
        translate([0, 0, FLOOR])
            linear_extrude(height = hgt)
                rrect(L - 2*WALL, W - 2*WALL, CR - WALL);
    }
}

//=============================================================================
// 舵机（v3 原样）
//=============================================================================
module servo_cutout(sx, sy) {
    translate([sx*SV_X - SV_HW/2, sy*W/2 - WALL/2 - 2, ZV - SV_HH/2])
        cube([SV_HW, WALL + 4, SV_HH]);
    for (dx = [-1, 1], dz = [-1, 1])
        translate([sx*SV_X + dx*EAR_DX/2, sy*(W/2 - 9), ZV + dz*EAR_DY/2])
            rotate([90, 0, 0]) cylinder(d = 3.2, h = 18, center = true);
}

module servo_boss(sx, sy) {
    // ★ 加强筋按穿孔两侧摆，但**自动夹在内腔范围内** ——
    //   车身缩短后（L 小），外侧那条筋会伸出后/前壁，不夹住就会凸出车体、
    //   导 STL 报非流形（实测 L=222 就是这样炸的）。
    IN = L/2 - WALL;                       // 内腔半长
    for (dx = [-1, 1]) {
        x0 = sx*SV_X + (dx > 0 ? 21 : -36);
        x1 = x0 + 15;
        xa = max(min(x0, x1), -IN);
        xb = min(max(x0, x1),  IN);
        if (xb > xa + 0.5)
            translate([xa,
                       sy > 0 ? (W/2 - WALL - BOSS_H) : -(W/2 - WALL),
                       FLOOR])
                cube([xb - xa, BOSS_H, H1 - FLOOR]);
    }
}

//=============================================================================
// 挡件凸台（侧壁外，每腿 1 个 ×4）—— 配合 mech/stop_block.scad
//   凸台 16(X) × 6(Y) × 14(Z)，外表面 y = W/2 + 6；底孔 φ2.8 沿 Y，z=30，x=±55±4.5
//=============================================================================
module stop_boss(sx, sy) {
    // sy>0: y 从 +W/2 往外 6mm；sy<0: y 从 -W/2-6 到 -W/2（**必须往外**，
    // 早先写成 sy*W/2 会让 -Y 侧凸台长到墙体里面去）
    translate([sx*55 - 8, sy > 0 ? W/2 : -(W/2 + 6), 23]) cube([16, 6, 14]);
}
module stop_boss_holes(sx, sy) {
    // 凸台已删，但两个 M3 底孔保留（左右各 2 孔、共面），直接打在 3mm 侧壁上
    for (dx = [-1, 1])
        translate([sx*55 + dx*4.5, sy*(W/2 + 2), 30])
            rotate([90, 0, 0]) cylinder(d = 2.8, h = 12, center = true);
}

//=============================================================================
// 固定结构
//=============================================================================
// 浅沉台（定位，depth 从底面往上）
module pocket(w, d, dep) {
    translate([-w/2, -d/2, -1]) cube([w, d, dep + 1]);
}

// 单对扎带孔（跨 span）
module zip_pair(span = 16) {
    for (s = [-1, 1])
        translate([s*span/2, 0, -1]) cube([6, 3, FLOOR + 2], center = true);
}

// ---- 以下两个【当前未用】：模块固定已改成“四角穿孔柱”，边夹留作备选 ----
// L 形边夹：爪压在板上表面（厚 t）；lift = 板底抬空量（给底部焊点留空间）
module board_clamp(t = 1.6, reach = 6, th = 4.0, gap = 0.3, lift = 2.0) {
    translate([-th/2, 0, 0]) {
        cube([th, th, lift + t + gap + t]);
        translate([0, -reach, lift + t + gap]) cube([th, th + reach, t]);
    }
}

// 矩形板四角边夹（上下两边各 2 个）
module clamp_rect(cx, cy, bx, by, t = 1.6, lift = 2.0) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([cx + sx*(bx/2 - 6), cy + sy*(by/2), FLOOR])
            rotate([0, 0, sy > 0 ? 0 : 180]) board_clamp(t, lift = lift);
}

// 4 根 φ2.6 定位柱：穿过板角孔（φ3.1/3.2），细一点好插、能固定即可
//   底部嵌入底板 1mm，保证是同一个实体（否则只贴合面 -> 两个 volume）
module pcb_posts_through(cx, cy, dx, dy, d = 2.6, h = 10) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([cx + sx*dx/2, cy + sy*dy/2, FLOOR - 1])
            cylinder(d = d, h = h + 1);
}

// ★ 2 根对角定位柱（左上 + 右下）—— LM2596 模块**只有 2 个圆孔**
//   用户实物照片确认：孔在左上角、右下角（对角布置）
module pcb_posts_2(cx, cy, dx, dy, d = 2.6, h = 6) {
    for (s = [[-1, 1], [1, -1]])         // 左上(-x,+y) / 右下(+x,-y)
        translate([cx + s[0]*dx/2, cy + s[1]*dy/2, FLOOR - 1])
            cylinder(d = d, h = h + 1);
}

//=============================================================================
// ★ 定位柱坐标表 + 通风孔避让
//
//   为什么要有这个：定位柱是从底板往上长 6mm 的实心柱，柱脚靠底板连成一体。
//   如果柱心正好落在通风孔(φ5)上，柱脚就被挖空了 -> **柱子悬空**，
//   CGAL 会把它算成独立实体（STL 里 Volumes 从 2 变 3），切片出来是一根掉落的柱子。
//   之前挪动电机板/降压柱时就踩到过（(43,-24) 撞 (44,-24) 的通风孔）。
//=============================================================================
MOD_POSTS = concat(
    [for (sx = [-1, 1], sy = [-1, 1]) [DRV_CX + sx*DRV_HOLE_DX/2, DRV_CY + sy*DRV_HOLE_DY/2]],
    [for (sx = [-1, 1], sy = [-1, 1]) [SB_CX  + sx*SB_HOLE_DX/2,  SB_CY  + sy*SB_HOLE_DY/2]],
    [for (s = [[-1, 1], [1, -1]])      [B5_CX + s[0]*BUCK_HOLE_DX/2,  B5_CY + s[1]*BUCK_HOLE_DY/2]],
    [for (s = [[-1, 1], [1, -1]])      [B9_CX + s[0]*BUCK_HOLE_DX/2,  B9_CY + s[1]*BUCK_HOLE_DY/2]],
    [for (s = [[-1, 1], [1, -1]])      [B5_CX + s[0]*BUCK_HOLE_DX/2, -B5_CY + s[1]*BUCK_HOLE_DY/2]]
);

// 通风孔离任何一根定位柱 < 5mm 就跳过（5 = 孔半径 2.5 + 柱半径 1.3 + 余量 1.2）
function vent_hits_post(x, y) =
    len([for (p = MOD_POSTS) if (norm([x - p[0], y - p[1]]) < 5) 1]) > 0;

//=============================================================================
// 下层
//=============================================================================
module lower() {
    difference() {
        union() {
            shell(H1);
            // 四角固定桩（v3：从底板长到顶）
            for (sx = [-1, 1], sy = [-1, 1])
                translate([sx*POST_X, sy*POST_Y, 0]) cylinder(d = POST_D, h = H1);
            // 舵机内壁加强筋：**保留**（在墙内侧 y 66…72，给耳孔螺丝 6mm 咬合；
            //   它不在舵机身体的 x 范围内，不会顶到舵机）
            for (sx = [-1, 1], sy = [-1, 1]) servo_boss(sx, sy);
            // 挡件凸台：★ 2026-10-27 **删除** —— 它从墙外表面凸出 6mm，
            //   正好压在舵机耳片端部（x 60.5…63），耳片贴不平、舵机装不进去。
            //   删掉后左右两侧墙外表面共面，挡件直接贴墙装。
            // for (sx = [-1, 1], sy = [-1, 1]) stop_boss(sx, sy);
            // 分区矮墙：★ 2026-10-27 两道**全删**（用户要求）；电池刻槽/浅槽照旧保留
            // partition(P1_X);    // 电池区 | 中部  ← 删
            // partition(P2_X);    // 中部 | 前部    ← 删
            // 模块固定（实体）：四角穿孔柱
            pcb_posts_through(DRV_CX, DRV_CY, DRV_HOLE_DX, DRV_HOLE_DY);   // 电机板
            // 舵机板：★ 柱高 10 -> 5（减一半）、直径 2.6 -> 1.6（减 1mm），不再凸出板面
            pcb_posts_through(SB_CX,  SB_CY,  SB_HOLE_DX,  SB_HOLE_DY, d = 1.6, h = 5);
            // 两块 LM2596：每块**只有 2 个圆孔** -> 只做 2 根对角柱（左上 + 右下）
            pcb_posts_2(B5_CX,  B5_CY,  BUCK_HOLE_DX, BUCK_HOLE_DY, h = 6); // 12→5/6（舵机）
            pcb_posts_2(B9_CX,  B9_CY,  BUCK_HOLE_DX, BUCK_HOLE_DY, h = 6); // 12→9（FPGA）
            // ★ 2026-10-27：再放一组**对称的** 2 柱（把 B5 的柱阵按 Y=0 镜像）
            //   给第二块 LM / UBEC 的托板留安装位
            //   ⚠️ 位置要挪就改 B5_CX / B5_CY（这一组跟着镜像走）
            pcb_posts_2(B5_CX, -B5_CY, BUCK_HOLE_DX, BUCK_HOLE_DY, h = 6);
        }

        // ---- 通风孔（自动避开定位柱，见 MOD_POSTS 说明）----
        for (x = [-100 : 24 : 100], y = [-48 : 24 : 48])
            if (!vent_hits_post(x, y))
                translate([x, y, -1]) cylinder(d = 5, h = FLOOR + 2);

        // ---- 舵机穿墙孔 + 耳孔（v3 原样）----
        for (sx = [-1, 1], sy = [-1, 1]) servo_cutout(sx, sy);

        // ---- 固定桩 M3 自攻底孔 ----
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx*POST_X, sy*POST_Y, H1 - 10]) cylinder(d = PILOT_D, h = 11);

        // ---- 电机线孔 16×24（6 排线，左右对称）----
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx*SV_X - 8, sy*(W/2 - 18) - 12, -1]) cube([16, 24, FLOOR + 2]);

        // ---- 挡件凸台 M3 自攻底孔 ----
        for (sx = [-1, 1], sy = [-1, 1]) stop_boss_holes(sx, sy);

        // ---- 前壁超声波孔 46×21 + 沉台 ----
        translate([L/2 - 5, -SR04_W/2, 8]) cube([10, SR04_W, SR04_H]);
        translate([L/2 - WALL - 2.01, -SR04_W/2 - 1, 7])
            cube([2.01, SR04_W + 2, SR04_H + 2]);

        // ---- 电池：2mm 浅槽（保住 1mm 底板，电池不会掉下去）----
        //   v3 原是 4mm 深槽挖在 3mm 底板上 = 整块穿透，88×61 的电池会掉出去
        //   电源插头在长边（可翻转）=> 底部不开孔，线从 2mm 台阶上方直接出
        translate([BATT_CX - BATT_L/2, -BATT_W/2, FLOOR - BATT_D])
            cube([BATT_L, BATT_W, BATT_D + 0.5]);
        // 电池扎带孔（4 根，跨电池压住；离槽边留 >5mm 避免薄片）
        for (dx = [-20, 20], sy = [-1, 1])   // y=+-30：避开通风孔边缘 (y=+-26.5)
            translate([BATT_CX + dx, sy*30, 0]) zip_pair(24);

        // ---- 12→5 / 12→9 降压：扎带双保险（12→9 图上四角是焊盘，孔位存疑）----
        for (bc = [[B5_CX, B5_CY], [B9_CX, B9_CY]])
            translate([bc[0], bc[1], 0])
                for (dy = [-8, 8]) translate([0, dy, 0]) zip_pair(BUCK_W + 6);

        // ---- 端子排：浅槽 + 扎带 ----
        translate([TERM_CX, TERM_CY, 0]) {
            pocket(TERM_W + 2, TERM_D + 2, 1.5);
            for (dx = [-20, 0, 20]) translate([dx, 0, 0]) zip_pair(TERM_D + 6);
        }
        // ---- 保险丝座 + 开关：浅沉台 + 扎带 ----
        translate([FUSE_CX, FUSE_CY, 0]) {
            pocket(FUSE_W + 2, FUSE_D + 2, 1.5);
            for (dx = [-10, 10]) translate([dx, 0, 0]) zip_pair(FUSE_D + 6);
        }
    }

    // ---- 模块占位（核对位置）----
    if (SHOW_MODULES) modules_lower();
}

// 分区矮墙：x 处，y∈[-P_Y,P_Y]，高 P_H（线从上方跨过）
module partition(x) {
    translate([x - P_T/2, -P_Y, FLOOR]) cube([P_T, 2*P_Y, P_H]);
}

//=============================================================================
// 模块占位（仅供核对，颜色区分）
//=============================================================================
module modules_lower() {
    // 电池 88×61×25（实际电池尺寸 = 槽内尺寸 - 1.5/边）
    color([0.85, 0.35, 0.25, 0.75])
        translate([BATT_CX, BATT_CY, FLOOR + 12.5]) cube([BATT_L - 3, BATT_W - 3, 25], center = true);
    // 电机驱动板 50×50×12（带电容/螺丝端子）
    color([0.55, 0.35, 0.70, 0.8])
        translate([DRV_CX, DRV_CY, FLOOR + 6]) cube([DRV, DRV, 12], center = true);
    // 舵机板 62.3×25.5×15
    color([0.20, 0.60, 0.40, 0.8])
        translate([SB_CX, SB_CY, FLOOR + SB_H/2]) cube([SB_W, SB_D, SB_H], center = true);
    // 端子排 76×8×10
    color([0.25, 0.45, 0.75, 0.8])
        translate([TERM_CX, TERM_CY, FLOOR + 5]) cube([TERM_W, TERM_D, 10], center = true);
    // 保险丝+开关
    color([0.90, 0.75, 0.20, 0.8])
        translate([FUSE_CX, FUSE_CY, FLOOR + 6]) cube([FUSE_W, FUSE_D, 12], center = true);
    // 两个降压 LM2596S 43×21×14
    color([0.30, 0.75, 0.85, 0.8])
        translate([B5_CX, B5_CY, FLOOR + 7]) cube([BUCK_W, BUCK_D, 14], center = true);
    color([0.30, 0.75, 0.85, 0.8])
        translate([B9_CX, B9_CY, FLOOR + 7]) cube([BUCK_W, BUCK_D, 14], center = true);
    // 超声波（贴前壁内的薄板）
    color([0.95, 0.60, 0.20, 0.85])
        translate([L/2 - WALL - 1.5, 0, 18.5]) cube([2, SR04_W - 1, SR04_H], center = true);
    // 舵机占位 ×4（车外）
    for (sx = [-1, 1], sy = [-1, 1]) {
        color([0.28, 0.28, 0.33, 0.85])
            translate([sx*SV_X, sy*(W/2 - 3 - 21), ZV]) cube([40.7, 42, 19.7], center = true);
        color([0.35, 0.35, 0.40, 0.85])
            translate([sx*SV_X, sy*(W/2 + 1), ZV]) cube([EAR_DX + 4, 2, EAR_DY + 4], center = true);
    }
}

//=============================================================================
// 上层（第二层）：FPGA（后）+ K230（前）+ 摄像头（前壁）
//=============================================================================
// FPGA 149.86×109.22×34（实测；自带升高支架，不做支撑）；紧贴后壁
FPG_CX = -43;  FPG_CY = 0;   FPG_W = 150;  FPG_D = 109;
// K230 56×85（56 沿 X，长边沿 Y）；转 180° 让 Type-C 朝 −Y（正对充电宝）
K3_CX  = 87;   K3_CY  = 17.5; K3_W  = 56;   K3_D  = 85;
// 充电宝 79×35×27：Type-C 在 79 长边中间 -> 79 沿 X、35 沿 Y，贴 −Y 侧（不挡 DC 座）
PB_CX  = 73.5; PB_CY  = -50.5; PB_W = 79;  PB_D = 35;  PB_H = 27;
// FPGA DC 免焊弯头（5.5×2.1，朝上，X 只占 14mm）：在 FPGA 前边 +Y 侧，与充电宝错开
DCJ_X0 = 32;   DCJ_X1 = 46;   DCJ_CY = 45;
// 摄像头中心高度：要让开 K230（16 高 -> z<=19），相机 26 高 -> 中心 >= 32
ZCAM   = 33;

// 四角限位柱（在板外，只挡横向）；底部嵌入底板 1mm 保证连通
module corner_posts(cx, cy, bx, by, h = 14, s = 8, gap = 0.4) {
    for (sx = [-1, 1], sy = [-1, 1])
        translate([cx + sx*(bx/2 + gap + s/2) - s/2,
                   cy + sy*(by/2 + gap + s/2) - s/2,
                   FLOOR - 1])
            cube([s, s, h + 1]);
}

// 充电宝改为【内置】：占位见 modules_upper()，槽 + 扎带见 upper() 的 difference。

module upper() {
    difference() {
        union() {
            shell(H2);
            // FPGA 自带升高支架 -> 不做支撑（只留位置）
            corner_posts(K3_CX,  K3_CY,  K3_W,  K3_D, s = 4);   // K230（柱缩到 4mm 省 8mm）
        }

        // ---- 固定桩过孔 ×4（接下层）----
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx*POST_X, sy*POST_Y, -1]) cylinder(d = 3.4, h = FLOOR + 2);

        // ---- 层间走线孔 ----
        // 9V 进 FPGA DC 座：公头下方开孔，线从下层穿上来直接进插头螺丝端子
        translate([DCJ_X0 + 4, DCJ_CY - 8, -1]) cube([14, 16, FLOOR + 2]);
        // 两侧宽条（FPGA 侧边外）：电机/编码器 / 舵机 / 信号束
        translate([-114, 56, -1]) cube([74, 14, FLOOR + 2]);
        translate([-114, -70, -1]) cube([74, 14, FLOOR + 2]);

        // ---- 充电宝：2mm 浅槽 + 2 组扎带（跨 35 宽，沿 Y 绕）----
        translate([PB_CX, PB_CY, 0]) pocket(PB_W + 1, PB_D + 1, 2);
        for (dx = [-24, 24])
            translate([PB_CX + dx, PB_CY, 0]) rotate([0, 0, 90]) zip_pair(PB_D + 4);

        // ---- 摄像头：抬到 K230 上方（z=33），前壁方形沉台 26×26×2 + φ14 通孔 ----
        //   抬高后相机不再占 X 空间（K230 只到 x=117，相机在它上面），省 ~8mm
        translate([L/2 - WALL, 0, ZCAM])
            rotate([0, 90, 0]) {
                translate([0, 0, -2.01])
                    hull() for (sx = [-1, 1], sy = [-1, 1])
                        translate([sx*11, sy*11, 0]) cylinder(r = 2, h = 2.01);
                translate([0, 0, -5]) cylinder(d = CAM_D, h = 9);
            }

        // ---- 通风孔 ----
        for (x = [-90 : 24 : 90], y = [-40 : 24 : 40])
            translate([x, y, -1]) cylinder(d = 5, h = FLOOR + 2);
    }

    if (SHOW_MODULES) modules_upper();
}

// 上层模块占位
module modules_upper() {
    // FPGA 149.86×109.22×34
    color([0.30, 0.50, 0.78, 0.8])
        translate([FPG_CX, FPG_CY, FLOOR + 17]) cube([FPG_W, FPG_D, 34], center = true);
    // K230 56×85×16
    color([0.85, 0.35, 0.40, 0.8])
        translate([K3_CX, K3_CY, FLOOR + 8]) cube([K3_W, K3_D, 16], center = true);
    // 摄像头 26×26×8（贴前壁内，抬高到 K230 上方）
    color([0.95, 0.65, 0.20, 0.85])
        translate([L/2 - WALL - 4, 0, ZCAM]) cube([8, 26, 26], center = true);
    // 充电宝 79×35×27（内置，贴 −Y 侧）
    color([0.92, 0.92, 0.95, 0.85])
        translate([PB_CX, PB_CY, FLOOR + PB_H/2]) cube([PB_W, PB_D, PB_H], center = true);
    // 充电宝 Type-C 口（+Y 长边中间）—— 示意
    color([0.40, 0.40, 0.45, 0.9])
        translate([PB_CX, PB_CY + PB_D/2, FLOOR + 10]) cube([9, 4, 3.2], center = true);
    // K230 的 Type-C 口（−X 长边，靠 −Y 端，转 180° 后）—— 示意
    color([0.40, 0.40, 0.45, 0.9])
        translate([K3_CX - K3_W/2, K3_CY - K3_D/2 + 10, FLOOR + 10]) cube([4, 9, 3.2], center = true);
    // FPGA DC 免焊公头 5.5×2.1（弯头朝上，X 只占 14mm）
    //   起点 +1mm 避开和 FPGA 占位块"面贴面"（否则导 STL 报非流形）
    color([0.25, 0.25, 0.28, 0.9])            // 插头段（横插进 FPGA 的 DC 座）
        translate([DCJ_X0 + 1, DCJ_CY, 13]) rotate([0, 90, 0]) cylinder(d = 8, h = 6);
    color([0.35, 0.75, 0.55, 0.9])            // 弯头本体（向上）
        translate([DCJ_X0 + 7, DCJ_CY, 24]) cube([14, 16, 22], center = true);
}

//=============================================================================
// 装配
//=============================================================================
module assembly() {
    color([0.86, 0.88, 0.91]) lower();
    translate([0, 0, H1]) color([0.93, 0.87, 0.74]) upper();
}

//=============================================================================
// ★ 交付打印排版：下层 + 上层【并排】，两个独立实体
//
//   注意：两层都是 242×150，252 台面**放不下两个**，所以这份 STL 只是让
//   打印室/切片软件清楚地看到"这是两个独立零件"，实际要分两次打
//   （PrusaSlicer/Orca 里右键 -> Split to objects 即可拆成两个对象）。
//   如果打印室要求单件文件，直接给 body_v5_lower.stl / body_v5_upper.stl。
//=============================================================================
module print_layout() {
    lower();
    translate([0, W + 14, 0]) upper();     // 上层挪到下层上方（Y 方向错开 164）
}

if (PART == "assembly")  assembly();
else if (PART == "lower") lower();
else if (PART == "upper") upper();
else if (PART == "print") print_layout();
