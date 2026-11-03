#=============================================================================
# build_motortest.tcl -- Gowin batch build for the motor-path self test
#   PURE ASCII: gw_sh reads scripts with the system code page (GBK here).
#=============================================================================

set SRC  "D:/dsh-ws/robot_project/tools/fpga_build/motortest"
set ROOT "D:/dsh-ws/motortest_build"

puts "=== create project ==="
create_project -name motortest -dir $ROOT -pn GW5AT-LV138PG484AC1/I0 -device_version B -force

puts "=== set options ==="
set_option -top_module motortest
set_option -verilog_std v2001
set_option -output_base_name motortest

puts "=== add files ==="
add_file $SRC/motortest.v
add_file $SRC/motortest.cst
add_file $SRC/motortest.sdc

puts "=== run all ==="
run all

puts "=== done ==="
puts "RESULT_OK"
