# design_1_unified clock-domain contract.
#
# The candidate intentionally treats the 50 MHz CSR plane, the 150 MHz
# processing/AXIS plane, and the 200 MHz TDC bus plane as asynchronous.  The
# RTL owns every crossing with XPM CDC primitives, handshakes, or explicit
# ASYNC_REG synchronizers.  Keep FCLK1 and its generated 150 MHz clock in the
# same group because the clock wizard preserves that synchronous relationship.
#
# This file belongs only to the design_1_unified constraint set.  Keep it out
# of the baseline constrs_1 set instead of using Tcl conditionals; Vivado's XDC
# parser deliberately supports only a restricted command subset.

set_clock_groups -name lidar_unified_async_domains -asynchronous \
    -group [get_clocks clk_fpga_0] \
    -group [get_clocks {clk_fpga_1 \
        clk_out1_design_1_unified_proc_clk_wiz_0}] \
    -group [get_clocks clk_fpga_2]
