#=============================================================================
# build_keymon.tcl -- Gowin batch build for the key/pin monitor
#   PURE ASCII: gw_sh reads scripts with the system code page (GBK here).
#=============================================================================

set SRC  "D:/dsh-ws/robot_project/tools/fpga_build/keymon"
set ROOT "D:/dsh-ws/keymon_build"

puts "=== create project ==="
create_project -name keymon -dir $ROOT -pn GW5AT-LV138PG484AC1/I0 -device_version B -force

puts "=== set options ==="
set_option -top_module keymon
set_option -verilog_std v2001
set_option -output_base_name keymon

puts "=== add files ==="
add_file $SRC/keymon.v
add_file $SRC/keymon.cst
add_file $SRC/keymon.sdc

puts "=== run all ==="
run all

puts "=== done ==="
puts "RESULT_OK"
