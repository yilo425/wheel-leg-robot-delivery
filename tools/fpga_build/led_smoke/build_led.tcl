#=============================================================================
# build_led.tcl -- Gowin batch build for the LED smoke test
#-----------------------------------------------------------------------------
# Usage (run from a PURE ASCII directory, gw_sh derives its log path from cwd):
#     gw_sh.exe build_led.tcl
#
# Two defects fixed on 2026-10-15 (this script had never been run as written):
#
#   1) -pn was "GW5AST-LV138PG484AC1/I0" -- the WRONG device.
#      This board is GW5AT-138 (chip ID 0x0011081B read back over JTAG),
#      NOT GW5AST-138 (0x0001081B). Same package, different silicon.
#      Building for the wrong part gives a bitstream that does not match
#      the board (or a refusal at programming time).
#
#   2) ROOT pointed at D:/robot_build/led_test and the sources were expected
#      to be sitting in that directory. They actually live in
#      tools/fpga_build/led_smoke/ (this file's own directory), so SRC now
#      points straight at them -- no copy step.
#
# THIS FILE IS DELIBERATELY PURE ASCII: gw_sh reads scripts using the system
# code page (GBK on a Chinese Windows), so UTF-8 Chinese comments break it.
#=============================================================================

set SRC  "D:/dsh-ws/robot_project/tools/fpga_build/led_smoke"
# ROOT deliberately lives OUTSIDE the repository: build/ is not fully
# gitignored and these smoke-test artifacts would pollute "git status".
set ROOT "D:/dsh-ws/led_test"

puts "=== create project ==="
create_project -name led_test -dir $ROOT -pn GW5AT-LV138PG484AC1/I0 -device_version B -force

puts "=== set options ==="
set_option -top_module led_test
set_option -verilog_std v2001
set_option -output_base_name led_test

puts "=== add files ==="
add_file $SRC/led_test.v
add_file $SRC/led_test.cst
add_file $SRC/led_test.sdc

puts "=== run all (syn -> pnr -> bitstream) ==="
run all

puts "=== done ==="
puts "RESULT_OK"
