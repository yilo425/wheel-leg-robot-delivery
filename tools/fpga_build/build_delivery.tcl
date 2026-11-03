#=============================================================================
# build_all.tcl -- synthesize / place-and-route the whole robot design
#-----------------------------------------------------------------------------
# Device: GW5AT-138 / PBGA484A   (Xiaomeige ACG720, 138K variant)
#
# Usage:
#   D:\Gowin\Gowin_V1.9.12_x64\Gowin_V1.9.12_x64\IDE\bin\gw_sh.exe build_all.tcl
#
# Two hard prerequisites:
#   1) Run from a PURE ASCII path. Gowin cannot handle Chinese paths:
#      it derives the log file path from the current directory and fails with
#      "alog error: failed to open log file: 'C:/Users/??/...'".
#   2) A valid license must exist at C:\Gowin\gowin.lic, otherwise startup
#      fails with "License verification failed".
#
# THIS FILE IS DELIBERATELY PURE ASCII. gw_sh reads scripts using the system
# code page (GBK on a Chinese Windows), so UTF-8 Chinese comments would turn
# into garbage.
#=============================================================================

#-----------------------------------------------------------------------------
# Paths
#
#   SRC  points straight at the migrated (ASCII) repository -- no copy step.
#   ROOT is the build sandbox; keep it inside the repo so it stays inside the
#   DSH session workspace (no elevation needed to write it).
#-----------------------------------------------------------------------------
set SRC       "D:/dsh-ws/delivery_20261103"
set ROOT      "D:/dsh-ws/delivery_20261103/build"
set PN        "GW5AT-LV138PG484AC1/I0"
set DEVVER    "B"

puts "=== create project ==="
create_project -name robot -dir $ROOT -pn $PN -device_version $DEVVER -force

puts "=== options ==="
set_option -top_module top
set_option -verilog_std v2001
set_option -output_base_name robot

#-----------------------------------------------------------------------------
# RTL sources
#
#   tof_driver / i2c_master are included even though top.v does not
#   instantiate them yet. They are unused modules, but feeding them to the
#   synthesizer is a free extra check -- Gowin catches some things ModelSim
#   does not.
#-----------------------------------------------------------------------------
puts "=== rtl: hal ==="
add_file $SRC/rtl/hal/reset_sync.v
add_file $SRC/rtl/hal/clk_gen.v

puts "=== rtl: driver ==="
add_file $SRC/rtl/driver/uart_rx.v
add_file $SRC/rtl/driver/uart_tx.v
add_file $SRC/rtl/driver/pwm_motor.v
add_file $SRC/rtl/driver/encoder_decode.v
add_file $SRC/rtl/driver/servo_lift.v
add_file $SRC/rtl/driver/i2c_master.v
# pca9685_drv: PCA9685 舵机板驱动（SERVO_IF=1 时 top.v 会例化）
add_file $SRC/rtl/driver/pca9685_drv.v
# i2c_bus_arb: TOF and IMU share one I2C bus, this grants the bus.
#   Without it: "ERROR (EX3937) : Instantiating unknown module 'i2c_bus_arb'"
add_file $SRC/rtl/driver/i2c_bus_arb.v
add_file $SRC/rtl/driver/tof_driver.v
# tof_sr04: HC-SR04 ultrasonic (4-wire TRIG/ECHO), used when TOF_IF = 1
add_file $SRC/rtl/driver/tof_sr04.v
# imu_driver is enabled by default (top.v IMU_ENABLE = 1).
add_file $SRC/rtl/driver/imu_driver.v
# motor_guard: soft-start + stall protection, sits between motion_decision
# and pwm_motor (top.v section 5.5).
add_file $SRC/rtl/driver/motor_guard.v

puts "=== rtl: app ==="
add_file $SRC/rtl/app/uart_frame.v
add_file $SRC/rtl/app/vision_rx.v
# config_rx: CONFIG frame (0x04) -> runtime-configurable params + manual mode
add_file $SRC/rtl/app/config_rx.v
add_file $SRC/rtl/app/telemetry_tx.v
add_file $SRC/rtl/app/main_fsm.v
add_file $SRC/rtl/app/motion_decision.v
add_file $SRC/rtl/app/rc_rx.v
# stance_ctrl: 站立高度（遥控升高/降低）
add_file $SRC/rtl/app/stance_ctrl.v

puts "=== rtl: top ==="
add_file $SRC/rtl/top.v

#-----------------------------------------------------------------------------
# Constraints
#-----------------------------------------------------------------------------
puts "=== constraints ==="
add_file $SRC/constraints/top.cst
add_file $SRC/constraints/top.sdc

#-----------------------------------------------------------------------------
# Run everything: synthesis -> place & route -> bitstream
#-----------------------------------------------------------------------------
puts "=== run all ==="
run all

puts "=== done ==="
puts "RESULT_OK"
