# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  #Adding Page
  set Clock_and_Output [ipgui::add_page $IPINST -name "Clock and Output"]
  #Adding Group
  set Clock_Contract [ipgui::add_group $IPINST -name "Clock Contract" -parent ${Clock_and_Output}]
  ipgui::add_param $IPINST -name "G_CSR_CLK_MHZ" -parent ${Clock_Contract} -widget comboBox
  ipgui::add_param $IPINST -name "G_PROC_CLK_MHZ" -parent ${Clock_Contract} -widget comboBox
  ipgui::add_param $IPINST -name "G_TDC_CLK_MHZ" -parent ${Clock_Contract} -widget comboBox
  ipgui::add_param $IPINST -name "G_STREAM_CLK_MODE" -parent ${Clock_Contract} -widget comboBox
  ipgui::add_static_text $IPINST -name "gpx_reference_clock_contract" -parent ${Clock_Contract} -text {MANDATORY HW CONTRACT: Supply 40 MHz (Tref = 25 ns) to every external TDC-GPX reference-clock pin. G_TDC_CLK_MHZ controls only the PL bus/acquisition clock; this IP neither generates nor verifies the 40 MHz reference.}
  ipgui::add_static_text $IPINST -name "clock_help" -parent ${Clock_Contract} -text {ASYNC allows either Processing/TDC frequency order. SYNC requires equal values and the same physical clock net. V3 H6 routine sign-off covers 150/200 and 200/150 MHz.}

  #Adding Group
  set Frame_Output [ipgui::add_group $IPINST -name "Frame Output" -parent ${Clock_and_Output}]
  ipgui::add_param $IPINST -name "G_OUTPUT_WIDTH" -parent ${Frame_Output} -widget comboBox
  ipgui::add_param $IPINST -name "G_NUM_FACES" -parent ${Frame_Output} -widget comboBox
  ipgui::add_static_text $IPINST -name "output_help" -parent ${Frame_Output} -text {Output width is fixed at synthesis. Runtime Return count changes the active HSIZE only at an acknowledged Face boundary.}


  #Adding Page
  set TDC-GPX_Topology [ipgui::add_page $IPINST -name "TDC-GPX Topology"]
  #Adding Group
  set Physical_Geometry [ipgui::add_group $IPINST -name "Physical Geometry" -parent ${TDC-GPX_Topology}]
  ipgui::add_param $IPINST -name "G_NUM_CHIPS" -parent ${Physical_Geometry} -widget comboBox
  ipgui::add_param $IPINST -name "G_STOPS_PER_CHIP" -parent ${Physical_Geometry} -widget comboBox
  ipgui::add_param $IPINST -name "G_MAX_RETURNS_PER_STOP" -parent ${Physical_Geometry} -widget comboBox
  ipgui::add_static_text $IPINST -name "geometry_help" -parent ${Physical_Geometry} -text {Physical GPX bus, EF/LF/ERRFLAG and Echo STOP vector widths follow chip count and STOP count. Maximum Return is compile-time capacity; runtime selection may use 1 through that capacity.}

  #Adding Group
  set Slope_Capability [ipgui::add_group $IPINST -name "Slope Capability" -parent ${TDC-GPX_Topology}]
  ipgui::add_param $IPINST -name "G_RISE_CAPABILITY_MASK" -parent ${Slope_Capability}
  ipgui::add_param $IPINST -name "G_FALL_CAPABILITY_MASK" -parent ${Slope_Capability}
  ipgui::add_static_text $IPINST -name "slope_help" -parent ${Slope_Capability} -text {Masks may overlap, so every active chip can acquire both rising and falling edges. Use falling mask 0000 for a rising-only build.}

  #Adding Group
  set GPX_Pin_Contract [ipgui::add_group $IPINST -name "GPX Pin Contract" -parent ${TDC-GPX_Topology}]
  ipgui::add_param $IPINST -name "G_OEN_MODE" -parent ${GPX_Pin_Contract} -widget comboBox


  #Adding Page
  set Echo_Frontend [ipgui::add_page $IPINST -name "Echo Frontend"]
  #Adding Group
  set Build_Options [ipgui::add_group $IPINST -name "Build Options" -parent ${Echo_Frontend}]
  ipgui::add_param $IPINST -name "G_ENABLE_ECHO_RECEIVER" -parent ${Build_Options}
  ipgui::add_param $IPINST -name "G_ENABLE_ECHO_SIMULATION" -parent ${Build_Options}
  ipgui::add_static_text $IPINST -name "echo_help" -parent ${Build_Options} -text {Disabling Echo Receiver removes the LVDS frontend and hides its physical ports in IPI. Echo simulation requires the receiver build and never drives a physical laser pulse.}


  #Adding Page
  set Physical_Timing [ipgui::add_page $IPINST -name "Physical Timing"]
  #Adding Group
  set Operation_Timeout [ipgui::add_group $IPINST -name "Operation Timeout" -parent ${Physical_Timing}]
  ipgui::add_param $IPINST -name "G_PHASE_TIMEOUT_US" -parent ${Operation_Timeout}

  #Adding Group
  set TDC_Bus_Time_Values [ipgui::add_group $IPINST -name "TDC Bus Time Values" -parent ${Physical_Timing}]
  ipgui::add_param $IPINST -name "G_POWERUP_TIME_NS" -parent ${TDC_Bus_Time_Values}
  ipgui::add_param $IPINST -name "G_RECOVERY_TIME_NS" -parent ${TDC_Bus_Time_Values}
  ipgui::add_param $IPINST -name "G_ALU_PULSE_TIME_NS" -parent ${TDC_Bus_Time_Values}
  ipgui::add_param $IPINST -name "G_BUS_IDLE_STABLE_TIME_NS" -parent ${TDC_Bus_Time_Values}
  ipgui::add_param $IPINST -name "G_DRAIN_MARGIN_TIME_NS" -parent ${TDC_Bus_Time_Values}
  ipgui::add_static_text $IPINST -name "timing_help" -parent ${TDC_Bus_Time_Values} -text {All values are entered as time. RTL converts them to the selected clock domain with ceiling arithmetic so changing a supported build clock preserves the physical interval.}



}

proc update_PARAM_VALUE.G_ALU_PULSE_TIME_NS { PARAM_VALUE.G_ALU_PULSE_TIME_NS } {
	# Procedure called to update G_ALU_PULSE_TIME_NS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_ALU_PULSE_TIME_NS { PARAM_VALUE.G_ALU_PULSE_TIME_NS } {
	# Procedure called to validate G_ALU_PULSE_TIME_NS
	return true
}

proc update_PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS { PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS } {
	# Procedure called to update G_BUS_IDLE_STABLE_TIME_NS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS { PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS } {
	# Procedure called to validate G_BUS_IDLE_STABLE_TIME_NS
	return true
}

proc update_PARAM_VALUE.G_CSR_CLK_MHZ { PARAM_VALUE.G_CSR_CLK_MHZ } {
	# Procedure called to update G_CSR_CLK_MHZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_CSR_CLK_MHZ { PARAM_VALUE.G_CSR_CLK_MHZ } {
	# Procedure called to validate G_CSR_CLK_MHZ
	return true
}

proc update_PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS { PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS } {
	# Procedure called to update G_DRAIN_MARGIN_TIME_NS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS { PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS } {
	# Procedure called to validate G_DRAIN_MARGIN_TIME_NS
	return true
}

proc update_PARAM_VALUE.G_ENABLE_ECHO_RECEIVER { PARAM_VALUE.G_ENABLE_ECHO_RECEIVER } {
	# Procedure called to update G_ENABLE_ECHO_RECEIVER when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_ENABLE_ECHO_RECEIVER { PARAM_VALUE.G_ENABLE_ECHO_RECEIVER } {
	# Procedure called to validate G_ENABLE_ECHO_RECEIVER
	return true
}

proc update_PARAM_VALUE.G_ENABLE_ECHO_SIMULATION { PARAM_VALUE.G_ENABLE_ECHO_SIMULATION } {
	# Procedure called to update G_ENABLE_ECHO_SIMULATION when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_ENABLE_ECHO_SIMULATION { PARAM_VALUE.G_ENABLE_ECHO_SIMULATION } {
	# Procedure called to validate G_ENABLE_ECHO_SIMULATION
	return true
}

proc update_PARAM_VALUE.G_FALL_CAPABILITY_MASK { PARAM_VALUE.G_FALL_CAPABILITY_MASK } {
	# Procedure called to update G_FALL_CAPABILITY_MASK when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_FALL_CAPABILITY_MASK { PARAM_VALUE.G_FALL_CAPABILITY_MASK } {
	# Procedure called to validate G_FALL_CAPABILITY_MASK
	return true
}

proc update_PARAM_VALUE.G_MAX_RETURNS_PER_STOP { PARAM_VALUE.G_MAX_RETURNS_PER_STOP } {
	# Procedure called to update G_MAX_RETURNS_PER_STOP when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_MAX_RETURNS_PER_STOP { PARAM_VALUE.G_MAX_RETURNS_PER_STOP } {
	# Procedure called to validate G_MAX_RETURNS_PER_STOP
	return true
}

proc update_PARAM_VALUE.G_NUM_CHIPS { PARAM_VALUE.G_NUM_CHIPS } {
	# Procedure called to update G_NUM_CHIPS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_NUM_CHIPS { PARAM_VALUE.G_NUM_CHIPS } {
	# Procedure called to validate G_NUM_CHIPS
	return true
}

proc update_PARAM_VALUE.G_NUM_FACES { PARAM_VALUE.G_NUM_FACES } {
	# Procedure called to update G_NUM_FACES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_NUM_FACES { PARAM_VALUE.G_NUM_FACES } {
	# Procedure called to validate G_NUM_FACES
	return true
}

proc update_PARAM_VALUE.G_OEN_MODE { PARAM_VALUE.G_OEN_MODE } {
	# Procedure called to update G_OEN_MODE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_OEN_MODE { PARAM_VALUE.G_OEN_MODE } {
	# Procedure called to validate G_OEN_MODE
	return true
}

proc update_PARAM_VALUE.G_OUTPUT_WIDTH { PARAM_VALUE.G_OUTPUT_WIDTH } {
	# Procedure called to update G_OUTPUT_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_OUTPUT_WIDTH { PARAM_VALUE.G_OUTPUT_WIDTH } {
	# Procedure called to validate G_OUTPUT_WIDTH
	return true
}

proc update_PARAM_VALUE.G_PHASE_TIMEOUT_US { PARAM_VALUE.G_PHASE_TIMEOUT_US } {
	# Procedure called to update G_PHASE_TIMEOUT_US when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_PHASE_TIMEOUT_US { PARAM_VALUE.G_PHASE_TIMEOUT_US } {
	# Procedure called to validate G_PHASE_TIMEOUT_US
	return true
}

proc update_PARAM_VALUE.G_POWERUP_TIME_NS { PARAM_VALUE.G_POWERUP_TIME_NS } {
	# Procedure called to update G_POWERUP_TIME_NS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_POWERUP_TIME_NS { PARAM_VALUE.G_POWERUP_TIME_NS } {
	# Procedure called to validate G_POWERUP_TIME_NS
	return true
}

proc update_PARAM_VALUE.G_PROC_CLK_MHZ { PARAM_VALUE.G_PROC_CLK_MHZ } {
	# Procedure called to update G_PROC_CLK_MHZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_PROC_CLK_MHZ { PARAM_VALUE.G_PROC_CLK_MHZ } {
	# Procedure called to validate G_PROC_CLK_MHZ
	return true
}

proc update_PARAM_VALUE.G_RECOVERY_TIME_NS { PARAM_VALUE.G_RECOVERY_TIME_NS } {
	# Procedure called to update G_RECOVERY_TIME_NS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_RECOVERY_TIME_NS { PARAM_VALUE.G_RECOVERY_TIME_NS } {
	# Procedure called to validate G_RECOVERY_TIME_NS
	return true
}

proc update_PARAM_VALUE.G_RISE_CAPABILITY_MASK { PARAM_VALUE.G_RISE_CAPABILITY_MASK } {
	# Procedure called to update G_RISE_CAPABILITY_MASK when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_RISE_CAPABILITY_MASK { PARAM_VALUE.G_RISE_CAPABILITY_MASK } {
	# Procedure called to validate G_RISE_CAPABILITY_MASK
	return true
}

proc update_PARAM_VALUE.G_STOPS_PER_CHIP { PARAM_VALUE.G_STOPS_PER_CHIP } {
	# Procedure called to update G_STOPS_PER_CHIP when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_STOPS_PER_CHIP { PARAM_VALUE.G_STOPS_PER_CHIP } {
	# Procedure called to validate G_STOPS_PER_CHIP
	return true
}

proc update_PARAM_VALUE.G_STREAM_CLK_MODE { PARAM_VALUE.G_STREAM_CLK_MODE } {
	# Procedure called to update G_STREAM_CLK_MODE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_STREAM_CLK_MODE { PARAM_VALUE.G_STREAM_CLK_MODE } {
	# Procedure called to validate G_STREAM_CLK_MODE
	return true
}

proc update_PARAM_VALUE.G_TDC_CLK_MHZ { PARAM_VALUE.G_TDC_CLK_MHZ } {
	# Procedure called to update G_TDC_CLK_MHZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.G_TDC_CLK_MHZ { PARAM_VALUE.G_TDC_CLK_MHZ } {
	# Procedure called to validate G_TDC_CLK_MHZ
	return true
}


proc update_MODELPARAM_VALUE.G_CSR_CLK_MHZ { MODELPARAM_VALUE.G_CSR_CLK_MHZ PARAM_VALUE.G_CSR_CLK_MHZ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_CSR_CLK_MHZ}] ${MODELPARAM_VALUE.G_CSR_CLK_MHZ}
}

proc update_MODELPARAM_VALUE.G_PROC_CLK_MHZ { MODELPARAM_VALUE.G_PROC_CLK_MHZ PARAM_VALUE.G_PROC_CLK_MHZ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_PROC_CLK_MHZ}] ${MODELPARAM_VALUE.G_PROC_CLK_MHZ}
}

proc update_MODELPARAM_VALUE.G_TDC_CLK_MHZ { MODELPARAM_VALUE.G_TDC_CLK_MHZ PARAM_VALUE.G_TDC_CLK_MHZ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_TDC_CLK_MHZ}] ${MODELPARAM_VALUE.G_TDC_CLK_MHZ}
}

proc update_MODELPARAM_VALUE.G_STREAM_CLK_MODE { MODELPARAM_VALUE.G_STREAM_CLK_MODE PARAM_VALUE.G_STREAM_CLK_MODE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_STREAM_CLK_MODE}] ${MODELPARAM_VALUE.G_STREAM_CLK_MODE}
}

proc update_MODELPARAM_VALUE.G_NUM_CHIPS { MODELPARAM_VALUE.G_NUM_CHIPS PARAM_VALUE.G_NUM_CHIPS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_NUM_CHIPS}] ${MODELPARAM_VALUE.G_NUM_CHIPS}
}

proc update_MODELPARAM_VALUE.G_STOPS_PER_CHIP { MODELPARAM_VALUE.G_STOPS_PER_CHIP PARAM_VALUE.G_STOPS_PER_CHIP } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_STOPS_PER_CHIP}] ${MODELPARAM_VALUE.G_STOPS_PER_CHIP}
}

proc update_MODELPARAM_VALUE.G_MAX_RETURNS_PER_STOP { MODELPARAM_VALUE.G_MAX_RETURNS_PER_STOP PARAM_VALUE.G_MAX_RETURNS_PER_STOP } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_MAX_RETURNS_PER_STOP}] ${MODELPARAM_VALUE.G_MAX_RETURNS_PER_STOP}
}

proc update_MODELPARAM_VALUE.G_RISE_CAPABILITY_MASK { MODELPARAM_VALUE.G_RISE_CAPABILITY_MASK PARAM_VALUE.G_RISE_CAPABILITY_MASK } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_RISE_CAPABILITY_MASK}] ${MODELPARAM_VALUE.G_RISE_CAPABILITY_MASK}
}

proc update_MODELPARAM_VALUE.G_FALL_CAPABILITY_MASK { MODELPARAM_VALUE.G_FALL_CAPABILITY_MASK PARAM_VALUE.G_FALL_CAPABILITY_MASK } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_FALL_CAPABILITY_MASK}] ${MODELPARAM_VALUE.G_FALL_CAPABILITY_MASK}
}

proc update_MODELPARAM_VALUE.G_OUTPUT_WIDTH { MODELPARAM_VALUE.G_OUTPUT_WIDTH PARAM_VALUE.G_OUTPUT_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_OUTPUT_WIDTH}] ${MODELPARAM_VALUE.G_OUTPUT_WIDTH}
}

proc update_MODELPARAM_VALUE.G_NUM_FACES { MODELPARAM_VALUE.G_NUM_FACES PARAM_VALUE.G_NUM_FACES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_NUM_FACES}] ${MODELPARAM_VALUE.G_NUM_FACES}
}

proc update_MODELPARAM_VALUE.G_ENABLE_ECHO_RECEIVER { MODELPARAM_VALUE.G_ENABLE_ECHO_RECEIVER PARAM_VALUE.G_ENABLE_ECHO_RECEIVER } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_ENABLE_ECHO_RECEIVER}] ${MODELPARAM_VALUE.G_ENABLE_ECHO_RECEIVER}
}

proc update_MODELPARAM_VALUE.G_ENABLE_ECHO_SIMULATION { MODELPARAM_VALUE.G_ENABLE_ECHO_SIMULATION PARAM_VALUE.G_ENABLE_ECHO_SIMULATION } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_ENABLE_ECHO_SIMULATION}] ${MODELPARAM_VALUE.G_ENABLE_ECHO_SIMULATION}
}

proc update_MODELPARAM_VALUE.G_OEN_MODE { MODELPARAM_VALUE.G_OEN_MODE PARAM_VALUE.G_OEN_MODE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_OEN_MODE}] ${MODELPARAM_VALUE.G_OEN_MODE}
}

proc update_MODELPARAM_VALUE.G_PHASE_TIMEOUT_US { MODELPARAM_VALUE.G_PHASE_TIMEOUT_US PARAM_VALUE.G_PHASE_TIMEOUT_US } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_PHASE_TIMEOUT_US}] ${MODELPARAM_VALUE.G_PHASE_TIMEOUT_US}
}

proc update_MODELPARAM_VALUE.G_POWERUP_TIME_NS { MODELPARAM_VALUE.G_POWERUP_TIME_NS PARAM_VALUE.G_POWERUP_TIME_NS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_POWERUP_TIME_NS}] ${MODELPARAM_VALUE.G_POWERUP_TIME_NS}
}

proc update_MODELPARAM_VALUE.G_RECOVERY_TIME_NS { MODELPARAM_VALUE.G_RECOVERY_TIME_NS PARAM_VALUE.G_RECOVERY_TIME_NS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_RECOVERY_TIME_NS}] ${MODELPARAM_VALUE.G_RECOVERY_TIME_NS}
}

proc update_MODELPARAM_VALUE.G_ALU_PULSE_TIME_NS { MODELPARAM_VALUE.G_ALU_PULSE_TIME_NS PARAM_VALUE.G_ALU_PULSE_TIME_NS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_ALU_PULSE_TIME_NS}] ${MODELPARAM_VALUE.G_ALU_PULSE_TIME_NS}
}

proc update_MODELPARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS { MODELPARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS}] ${MODELPARAM_VALUE.G_BUS_IDLE_STABLE_TIME_NS}
}

proc update_MODELPARAM_VALUE.G_DRAIN_MARGIN_TIME_NS { MODELPARAM_VALUE.G_DRAIN_MARGIN_TIME_NS PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.G_DRAIN_MARGIN_TIME_NS}] ${MODELPARAM_VALUE.G_DRAIN_MARGIN_TIME_NS}
}

