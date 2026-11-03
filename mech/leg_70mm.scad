//=============================================================================
// leg_70mm.scad —— 舵机驱动轮腿（参数化 v2，2026-10-22）
//
// 用法：
//   1) 装 OpenSCAD（免费 20MB）：https://openscad.org/downloads.html
//   2) 改最上面【必改：实测参数】
//   3) 按 F6 → File → Export → Export as STL
//
// 打印：PETG / 填充 60~80% / 4 圈壁 / **平躺**（8mm 厚朝 Z）
//=============================================================================

//---------------- ★ 必改：实测参数 ----------------
STAR_RO     = 16.0;     // 量：舵机盘尖到圆心
STAR_RI     = 8.0;     // 量：舵机盘凹处到圆心
STAR_DEP    = 2.1;    // 量：舵机盘厚度 + 0.1
MOT_BOLT_SQ = 12.0; // 量：MG310 对角孔距 / 1.414
MOT_BOSS_D  = 9.2;  // 量：MG310 凸台直径

//---------------- 结构参数 ----------------
L          = 70.0;
TH         = 6.0;
ROOT_TH    = 9.0;
ROOT_R     = 19.5;
ROOT_SEG   = 14.0;
STAR_INTER = -0.15;
ARM_W      = 7.0;
ARM_W_TIP  = 5.0;
HUB_R      = 3.4;
ARM_R0     = 4.8;
CENTER_D   = 3.6;
CSNK_D     = 8.5;
CSNK_DEP   = 4.0;
MOT_BOLT_D = 3.4;
TIE_D      = 2.2;
TIE_X1     = 30.0;
TIE_X2     = 44.0;
TIE_DY     = 7.0;    // ★ 横向间距：2 孔跨 ±7（扎带横着绕，压住沿腿走的电机线）
CBORE_DEP  = 2.0;
CBORE_D    = 5.8;

WORK_ANG   = 45.0;
PAD_R      = 25.0;
PAD_L      = 14.0;
PAD_H      = 5.0;
PAD_FLARE  = 8.0;
PAD_EN     = 0;
PAD_SURF_Y = 12.3;

$fn = 64;

//---------------- 舵机盘（六芒星）凹槽 ----------------
//   按实物重建：**6 根圆头粗臂 + 中心盘**，不是细尖星。
//     臂宽 ARM_W，端头半圆，谷底半径 = ri，臂尖 = ro
//   过盈通过 shrink 实现（臂宽/半径各缩小一点）
module horn2d(ro, ri, w_base, w_tip, n, shrink) {
    ro2 = ro - shrink;
    ri2 = ri - shrink;
    wb  = w_base - 2*shrink;
    wt  = w_tip  - 2*shrink;
    union() {
        circle(r = HUB_R - shrink);   // 只填中心，让 6 个谷底成为【尖 V】
        for (i = [0 : n-1])
            rotate([0, 0, i * 360/n])
                hull() {
                    translate([ARM_R0 - shrink, 0]) circle(d = wb);
                    translate([ro2 - wt/2, 0]) circle(d = wt);
                }
    }
}

//---------------- 腿的外形 ----------------
module profile_2d() {
    hull() {
        circle(r = 19.5);
        translate([18.0, 0]) circle(r = 15.0);
        translate([42.0, 0]) circle(r = 13.0);
        translate([60.0, 0]) circle(r = 13.0);
        translate([72.0, 0]) circle(r = 13.0);
    }
}

//---------------- ★ 根部加厚（8 -> 11mm）----------------
module root_boss() {
    hull() {
        translate([0, 0, TH]) linear_extrude(height = ROOT_TH - TH)
            circle(r = ROOT_R);
        translate([ROOT_SEG, 0, TH]) linear_extrude(height = ROOT_TH - TH)
            circle(r = 12.0);
    }
}

//---------------- ★ 挡件平台 ----------------
//   局部坐标里绕 Z 转 WORK_ANG：它的 +Y 面在腿转到 -WORK_ANG（工作位）时
//   正好变成"水平朝上"，挡件才能水平压住它。
//   方案 A（保留凸台，端头做钝）：为什么不能"削平"见 doc 说明 ——
//   腿的上边缘几乎平行于腿轴，而接触面要求与腿轴成 45°，
//   两者夹角 45° => 任何"削平"都会切掉一大块楔形，反而更难看/更弱。
//   所以改成"堆一个小凸台 + 外侧倒圆"，接触面仍是 45°。
module stop_pad() {
    translate([PAD_R, PAD_SURF_Y - 3, 0])
        rotate([0, 0, WORK_ANG])
            linear_extrude(height = TH)
                // 梯形（底部外扩）+ 顶端倒角，避免刀刃状尖角
                polygon([[ -PAD_L/2 - PAD_FLARE, 0 ],
                         [  PAD_L/2 + PAD_FLARE, 0 ],
                         [  PAD_L/2 - 2,         PAD_H + 3 ],
                         [ -PAD_L/2 + 2,         PAD_H + 3 ]]);
}

//---------------- 主体（封装成 module，方便排版 4 条）----------------
module leg() {
difference() {
    union() {
        linear_extrude(height = TH) profile_2d();   // 主承力体
        root_boss();                                // 根部加厚
        if (PAD_EN != 0) stop_pad();                                 // 挡件平台
    }

    // 六芒星凹槽（从底面 z=0 往上挖）
    translate([0, 0, -1])
        linear_extrude(height = STAR_DEP + 1)
            horn2d(STAR_RO, STAR_RI, ARM_W, ARM_W_TIP, 6, STAR_INTER/2);

    // 中心 M3 通孔
    translate([0, 0, -1])
        cylinder(d = CENTER_D, h = ROOT_TH + 2);

    // ★ 顶面沉孔（φ8 垫片 + 螺丝头）
    translate([0, 0, ROOT_TH - CSNK_DEP])
        cylinder(d = CSNK_D, h = CSNK_DEP + 1);

    // 电机凸台让位孔
    translate([L, 0, -1])
        cylinder(d = MOT_BOSS_D, h = TH + 2);

    // 4 个电机安装孔
    for (sx = [-1, 1], sy = [-1, 1])
        translate([L + sx * MOT_BOLT_SQ/2, sy * MOT_BOLT_SQ/2, -1])
            cylinder(d = MOT_BOLT_D, h = TH + 2);

    // ★ 扎带孔：横向布置（跨腿宽）—— 2 处 × 2 孔
    //   原来两孔沿 X 前后排（扎带顺着腿绕），改成左右排（横着绕），
    //   这样能压住沿腿长方向走的电机线；孔轴仍沿 Z（穿透 6mm 板厚）。
    for (tx = [TIE_X1, TIE_X2], ty = [-1, 1])
        translate([tx, ty * TIE_DY, -1])
            cylinder(d = TIE_D, h = TH + 2);

    // 5) ★ 电机螺丝沉槽：顶面往下 CBORE_DEP，直径 = 孔径 + CBORE_D
    //    让原装螺丝的头藏进腿里（照片里那种小盘头螺丝）
    for (sx = [-1, 1], sy = [-1, 1])
        translate([L + sx * MOT_BOLT_SQ/2, sy * MOT_BOLT_SQ/2, TH - CBORE_DEP])
            cylinder(d = CBORE_D, h = CBORE_DEP + 1);
}
}   // module leg

//=============================================================================
// ★★ 打印排版（交付给打印室：一个文件出【4 条腿 + 4 个挡件】）
//
//   排版原则
//     1) 全部平放，z=0 贴床；**不需要支撑**
//        （腿的六芒星凹槽是从底面往上 2.1mm 的开口凹坑，不是封闭腔，
//          所以不会出现"悬空盖板"，直接打就行）
//     2) 件与件留 ≥11mm 间距（PETG 收缩/毛边不会连在一起）
//     3) 腿 2×2 阵列，右上角空出来放挡件，整板约 222×135，252 台面很宽松
//
//   打印参数（这板混装，取折中值）
//       PETG / 0.2mm / 4 圈壁 / **50~60% 填充**
//       （腿要受力所以壁要厚；挡件只受 ~8N，50% 足够）
//=============================================================================
PARTS_LAYOUT = true;    // true = 4 腿 + 4 挡件排版；false = 只出 1 条腿（单体调试）

// 挡件参数 + 模块一起进来（lib 顶层无几何，不会多画东西）
include <stop_block_lib.scad>
$fn = 64;               // lib 里是 32，这里恢复腿的精细度（顺带挡件也变细）

LEG_STEP_X = 117;       // 腿阵列 X 间距 = 腿长 104.5 + 12.5 间隙
LEG_STEP_Y = 52;        // 腿阵列 Y 间距 = 腿宽 39 + 13 间隙
LEG_ORG_X  = 20;        // 让整板落在正坐标（腿的轮廓 x 从 -19.5 起）
LEG_ORG_Y  = 20;
BLOCK_Y    = 126;       // 挡件那一排的 Y
BLOCK_STEP = 22;        // 挡件间距 = 14 + 8

if (PARTS_LAYOUT) {
    // 4 条腿：2×2
    for (ix = [0 : 1], iy = [0 : 1])
        translate([LEG_ORG_X + ix * LEG_STEP_X, LEG_ORG_Y + iy * LEG_STEP_Y, 0])
            leg();

    // 4 个挡件：一排，底面贴床
    for (k = [0 : 3])
        translate([LEG_ORG_X + k * BLOCK_STEP, BLOCK_Y, 0])
            stop_block();
} else {
    leg();
}
