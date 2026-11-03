#=============================================================================
# build_beacon.tcl -- Gowin batch build for the pin-connectivity beacon
#
#   Drives C19 with a ~3Hz square wave so the ARM can count edges on its
#   pin 13 and tell us definitively whether that wire is on C19.
#
# THIS FILE IS PURE ASCII: gw_sh reads scripts with the system code page
# (GBK on a Chinese Windows), so UTF-8 comments break it.
#=============================================================================

set SRC  "D:/dsh-ws/robot_project/tools/fpga_build/beacon"
set ROOT "D:/dsh-ws/beacon_build"

puts "=== create project ==="
create_project -name beacon -dir $ROOT -pn GW5AT-LV138PG484AC1/I0 -device_version B -force

puts "=== set options ==="
set_option -top_module beacon
set_option -verilog_std v2001
set_option -output_base_name beacon

puts "=== add files ==="
add_file $SRC/beacon.v
add_file $SRC/beacon.cst
add_file $SRC/beacon.sdc

puts "=== run all ==="
run all

puts "=== done ==="
puts "RESULT_OK"
