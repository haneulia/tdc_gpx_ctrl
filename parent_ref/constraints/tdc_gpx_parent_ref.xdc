# PS FCLK0 (AXI control), FCLK1 (AXIS processing), and FCLK2 (TDC bus) are
# independent domains. XPM instances carry their own point-to-point max-delay,
# bus-skew, and false-path constraints, so a global asynchronous clock group
# must not override them. Apply exceptions only to explicitly named first-stage
# ASYNC_REG registers; stage1 -> stage2 remains timed.
#
# Exact pin counts and ASYNC_REG ownership are checked after synthesis and
# implementation by verify_parent_manual_cdc in create_parent_ref.tcl.

set_false_path -to [get_pins -quiet -hier -regexp {^.*/u_config_ctrl/.*u_bus_phy/s_(ef1|ef2|lf1|lf2|irflag|errflag)_meta_r_reg/D$}]

set_false_path -to [get_pins -quiet -hier -regexp {^.*/u_config_ctrl/s_(cmd_collision|err_bus_fatal|init_cfg_coalesced)_meta_r_reg\[[0-9]+\]/D$}]

set_false_path -to [get_pins -quiet -hier -regexp {^.*/s_cdc_all_idle_ff_reg\[0\]/D$}]

set_false_path -to [get_pins -quiet -hier -regexp {^.*/s_cmd_(soft_reset|force_reinit)_sync_tdc_r_reg\[0\]/D$}]
