#=============================================================================
# beacon.sdc -- timing constraints (PURE ASCII, see led_test.sdc header)
#=============================================================================

create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

set_false_path -from [get_clocks {clk_50m}] -to [get_ports {beacon_out}]
set_false_path -from [get_clocks {clk_50m}] -to [get_ports {led}]
set_false_path -from [get_clocks {clk_50m}] -to [get_ports {led2[*]}]
