//=============================================================================
// horn_fit.scad —— 六芒星试配板（5 档过盈量），2026-10-23
//   从左到右 1..5，凹槽下方凹点数 = 编号
//   打出来把舵机盘逐个压进去，报"几号最合适"
//=============================================================================
STAR_RO = 16.0;
STAR_RI = 8.0;
ARM_W = 7.0;
ARM_W_TIP = 5.0;
HUB_R = 3.4;
ARM_R0 = 4.8;
STAR_DEP = 2.4;
PLATE_T = 3.0;
$fn = 64;

// 舵机盘外形（和腿零件里的完全一样）
module horn2d(ro, ri, w_base, w_tip, n, shrink) {
    ro2 = ro - shrink;
    ri2 = ri - shrink;
    wb  = w_base - 2*shrink;
    wt  = w_tip  - 2*shrink;
    union() {
        circle(r = HUB_R - shrink);
        for (i = [0 : n-1])
            rotate([0, 0, i * 360/n])
                hull() {
                    translate([ARM_R0 - shrink, 0]) circle(d = wb);
                    translate([ro2 - wt/2, 0]) circle(d = wt);
                }
    }
}

module pocket(shrink) {
    // 从顶面往下挖 STAR_DEP
    translate([0, 0, PLATE_T - STAR_DEP])
        linear_extrude(height = STAR_DEP + 0.01)
            horn2d(STAR_RO, STAR_RI, ARM_W, ARM_W_TIP, 6, shrink);
}

module dimples(cnt) {
    // 编号用凹点：cnt 个小圆坑
    for (i = [0 : cnt-1])
        translate([(i - (cnt-1)/2) * 4.2, -(STAR_RO + 7), PLATE_T - 1.2])
            cylinder(d = 2.2, h = 2.0);
}

module plate() {
    difference() {
        // 底板
        translate([-98.0, -20.0, 0])
            cube([196.0, 40.0, PLATE_T]);
        // 5 个凹槽 + 编号凹点
        translate([-72.0, 0, 0]) pocket(0.3);
        translate([-72.0, 0, 0]) dimples(1);
        translate([-36.0, 0, 0]) pocket(0.15);
        translate([-36.0, 0, 0]) dimples(2);
        translate([0.0, 0, 0]) pocket(0.0);
        translate([0.0, 0, 0]) dimples(3);
        translate([36.0, 0, 0]) pocket(-0.15);
        translate([36.0, 0, 0]) dimples(4);
        translate([72.0, 0, 0]) pocket(-0.3);
        translate([72.0, 0, 0]) dimples(5);
    }
}
plate();
