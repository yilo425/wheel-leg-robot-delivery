#=============================================================================
# led_test.sdc -- timing constraints for the LED smoke test
#-----------------------------------------------------------------------------
# REWRITTEN 2026-10-13. The previous version had two defects:
#
#   1) Chinese comments stored as UTF-8. Gowin reads .sdc / .tcl using the
#      system code page (GBK on a Chinese Windows), so every Chinese comment
#      had rotted into "????". Garbled bytes can even be parsed as syntax
#      errors. THIS FILE IS NOW PURE ASCII.
#
#   2) It used "//" as the comment character. SDC is Tcl; the comment
#      character is "#". Gowin happened to tolerate "//", but that is not
#      valid Tcl and will break other tools (Vivado, PrimeTime, ...).
#
# The constraint values themselves were correct and are unchanged.
#=============================================================================

#-----------------------------------------------------------------------------
# Clock
#
#   The board's 50 MHz oscillator drives pin Y18, which is wired straight to
#   the PLL input. This smoke test bypasses the PLL, so sys_clk = 50 MHz.
#       period 20 ns, rising edge at 0 ns, falling edge at 10 ns
#-----------------------------------------------------------------------------
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]

#-----------------------------------------------------------------------------
# Reset button
#
#   Asynchronous input. The RTL runs it through a synchronizer, so a relaxed
#   input delay is all it needs.
#-----------------------------------------------------------------------------
set_input_delay -clock clk_50m -max 5 [get_ports {rst_key_n}]

#-----------------------------------------------------------------------------
# LEDs
#
#   Slow output -- the eye cannot see 50 MHz. No point closing timing there.
#-----------------------------------------------------------------------------
set_output_delay -clock clk_50m -max 5 [get_ports {led[*]}]
# NOTE: get_clocks needs BRACES -- "[get_clocks clk_50m]" is rejected by
# Gowin V1.9.12 with:
#     ERROR (TA2000) : 'syntax error' near token 'clk_50m]'
# (see doc/pitfalls, section 4.4 (2)). The braced form works.
set_false_path -from [get_clocks {clk_50m}] -to [get_ports {led[*]}]
