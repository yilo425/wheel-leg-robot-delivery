#=============================================================================
# keymon.sdc -- timing constraints (PURE ASCII)
#=============================================================================

create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

set_false_path -from [get_clocks {clk_50m}] -to [get_ports {led[*]}]
