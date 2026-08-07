# Canonical Vivado XGUI for tdc_gpx_lidar_ctrl_v2_top.

namespace eval ::tglcv2_xgui {
  variable supported_clocks {50 100 125 150 200}

  proc as_integer {value label} {
    if {![string is entier -strict $value]} {
      error "$label must be an integer."
    }
    return [expr {wide($value)}]
  }

  proc is_true {value} {
    set normalized [string tolower [string trim $value {"}]]
    return [expr {$normalized in {1 true yes}}]
  }

  proc normalize_bits {value width label} {
    set bits [string trim [string map {_ {}} $value] {"}]
    if {[regexp -nocase {^0x([0-9a-f]+)$} $bits -> hex]} {
      scan $hex %x numeric
      set bits [format %0*b $width $numeric]
    }
    if {![regexp [format {^[01]{%d}$} $width] $bits]} {
      error "$label must contain exactly $width binary digits."
    }
    return $bits
  }

  proc bits_to_integer {bits} {
    set value 0
    foreach bit [split $bits {}] {
      set value [expr {($value << 1) | $bit}]
    }
    return $value
  }

  proc popcount {value} {
    set count 0
    while {$value != 0} {
      incr count [expr {$value & 1}]
      set value [expr {$value >> 1}]
    }
    return $count
  }

  proc validate_clock_contract {proc_mhz tdc_mhz stream_mode} {
    variable supported_clocks
    if {[catch {
      set proc_frequency [as_integer $proc_mhz {Processing clock frequency}]
      set tdc_frequency [as_integer $tdc_mhz {TDC clock frequency}]
    } message]} {
      return [list false $message]
    }
    set mode [string toupper [string trim $stream_mode {"}]]
    if {$proc_frequency ni $supported_clocks ||
        $tdc_frequency ni $supported_clocks} {
      return [list false \
        {Processing and TDC clocks must be 50, 100, 125, 150, or 200 MHz.}]
    }
    if {$mode ni {ASYNC SYNC}} {
      return [list false {Stream clock mode must be ASYNC or SYNC.}]
    }
    if {$mode eq {SYNC} && $proc_frequency != $tdc_frequency} {
      return [list false \
        {SYNC mode requires equal frequencies and one shared physical clock net.}]
    }
    return [list true {}]
  }

  proc validate_topology {num_chips rise_mask fall_mask} {
    if {[catch {
      set chip_count [as_integer $num_chips {TDC-GPX chip count}]
      set rise [bits_to_integer [normalize_bits $rise_mask 4 \
        {Rising capability mask}]]
      set fall [bits_to_integer [normalize_bits $fall_mask 4 \
        {Falling capability mask}]]
    } message]} {
      return [list false $message]
    }
    if {$chip_count < 1 || $chip_count > 4} {
      return [list false {TDC-GPX chip count must be from 1 to 4.}]
    }
    set present [expr {(1 << $chip_count) - 1}]
    if {(($rise | $fall) & ((~$present) & 0xF)) != 0} {
      return [list false \
        {Rise/Fall masks must not select a chip outside G_NUM_CHIPS.}]
    }
    if {(($rise | $fall) & $present) != $present} {
      return [list false \
        {Every active chip must be assigned to rising and/or falling.}]
    }
    if {[popcount [expr {$rise & $present}]] <
        [popcount [expr {$fall & $present}]]} {
      return [list false \
        {Rising-capable chip count must not be smaller than falling-capable chip count.}]
    }
    return [list true {}]
  }

  proc set_validation_result {parameter result} {
    lassign $result valid message
    set_property errmsg $message $parameter
    return $valid
  }
}

proc init_gui {IPINST} {
  set clock_page [ipgui::add_page $IPINST -name {Clock and Output}]
  set clocks [ipgui::add_group $IPINST -name {Clock Contract} \
    -parent $clock_page]
  ipgui::add_param $IPINST -name G_CSR_CLK_MHZ -parent $clocks \
    -widget comboBox
  ipgui::add_param $IPINST -name G_PROC_CLK_MHZ -parent $clocks \
    -widget comboBox
  ipgui::add_param $IPINST -name G_TDC_CLK_MHZ -parent $clocks \
    -widget comboBox
  ipgui::add_param $IPINST -name G_STREAM_CLK_MODE -parent $clocks \
    -widget comboBox
  ipgui::add_static_text $IPINST -name clock_help -parent $clocks -text \
    {ASYNC allows either Processing/TDC frequency order. SYNC requires equal values and the same physical clock net. K0 routine sign-off covers 150/200 and 200/150 MHz.}

  set output [ipgui::add_group $IPINST -name {Frame Output} \
    -parent $clock_page]
  ipgui::add_param $IPINST -name G_OUTPUT_WIDTH -parent $output \
    -widget comboBox
  ipgui::add_param $IPINST -name G_NUM_FACES -parent $output \
    -widget comboBox
  ipgui::add_static_text $IPINST -name output_help -parent $output -text \
    {Output width is fixed at synthesis. Runtime Return count changes the active HSIZE only at an acknowledged Face boundary.}

  set topology_page [ipgui::add_page $IPINST -name {TDC-GPX Topology}]
  set geometry [ipgui::add_group $IPINST -name {Physical Geometry} \
    -parent $topology_page]
  ipgui::add_param $IPINST -name G_NUM_CHIPS -parent $geometry \
    -widget comboBox
  ipgui::add_param $IPINST -name G_STOPS_PER_CHIP -parent $geometry \
    -widget comboBox
  ipgui::add_param $IPINST -name G_MAX_RETURNS_PER_STOP -parent $geometry \
    -widget comboBox
  ipgui::add_static_text $IPINST -name geometry_help -parent $geometry -text \
    {Physical GPX bus, EF/LF/ERRFLAG and Echo STOP vector widths follow chip count and STOP count. Maximum Return is compile-time capacity; runtime selection may use 1 through that capacity.}

  set slope [ipgui::add_group $IPINST -name {Slope Capability} \
    -parent $topology_page]
  ipgui::add_param $IPINST -name G_RISE_CAPABILITY_MASK -parent $slope
  ipgui::add_param $IPINST -name G_FALL_CAPABILITY_MASK -parent $slope
  ipgui::add_static_text $IPINST -name slope_help -parent $slope -text \
    {Masks may overlap, so every active chip can acquire both rising and falling edges. Use falling mask 0000 for a rising-only build.}

  set pins [ipgui::add_group $IPINST -name {GPX Pin Contract} \
    -parent $topology_page]
  ipgui::add_param $IPINST -name G_OEN_MODE -parent $pins \
    -widget comboBox

  set echo_page [ipgui::add_page $IPINST -name {Echo Frontend}]
  set echo [ipgui::add_group $IPINST -name {Build Options} \
    -parent $echo_page]
  ipgui::add_param $IPINST -name G_ENABLE_ECHO_RECEIVER -parent $echo \
    -widget checkBox
  ipgui::add_param $IPINST -name G_ENABLE_ECHO_SIMULATION -parent $echo \
    -widget checkBox
  ipgui::add_static_text $IPINST -name echo_help -parent $echo -text \
    {Disabling Echo Receiver removes the LVDS frontend and hides its physical ports in IPI. Echo simulation requires the receiver build and never drives a physical laser pulse.}

  set timing_page [ipgui::add_page $IPINST -name {Physical Timing}]
  set lifecycle [ipgui::add_group $IPINST -name {Operation Timeout} \
    -parent $timing_page]
  ipgui::add_param $IPINST -name G_PHASE_TIMEOUT_US -parent $lifecycle

  set bus [ipgui::add_group $IPINST -name {TDC Bus Time Values} \
    -parent $timing_page]
  foreach name {
    G_POWERUP_TIME_NS G_RECOVERY_TIME_NS G_ALU_PULSE_TIME_NS
    G_BUS_IDLE_STABLE_TIME_NS G_DRAIN_MARGIN_TIME_NS
  } {
    ipgui::add_param $IPINST -name $name -parent $bus
  }
  ipgui::add_static_text $IPINST -name timing_help -parent $bus -text \
    {All values are entered as time. RTL converts them to the selected clock domain with ceiling arithmetic so changing a supported build clock preserves the physical interval.}
}

proc update_PARAM_VALUE.G_CSR_CLK_MHZ {PARAM_VALUE.G_CSR_CLK_MHZ} {
  set_property value_validation_list {50 100 125 150 200} \
    ${PARAM_VALUE.G_CSR_CLK_MHZ}
}
proc update_PARAM_VALUE.G_PROC_CLK_MHZ {PARAM_VALUE.G_PROC_CLK_MHZ} {
  set_property value_validation_list {50 100 125 150 200} \
    ${PARAM_VALUE.G_PROC_CLK_MHZ}
}
proc update_PARAM_VALUE.G_TDC_CLK_MHZ {PARAM_VALUE.G_TDC_CLK_MHZ} {
  set_property value_validation_list {50 100 125 150 200} \
    ${PARAM_VALUE.G_TDC_CLK_MHZ}
}
proc update_PARAM_VALUE.G_STREAM_CLK_MODE {PARAM_VALUE.G_STREAM_CLK_MODE} {
  set_property value_validation_list {ASYNC SYNC} \
    ${PARAM_VALUE.G_STREAM_CLK_MODE}
}
proc update_PARAM_VALUE.G_NUM_CHIPS {PARAM_VALUE.G_NUM_CHIPS} {
  set_property value_validation_list {1 2 3 4} ${PARAM_VALUE.G_NUM_CHIPS}
}
proc update_PARAM_VALUE.G_STOPS_PER_CHIP {PARAM_VALUE.G_STOPS_PER_CHIP} {
  set_property value_validation_list {1 2 3 4 5 6 7 8} \
    ${PARAM_VALUE.G_STOPS_PER_CHIP}
}
proc update_PARAM_VALUE.G_MAX_RETURNS_PER_STOP {
  PARAM_VALUE.G_MAX_RETURNS_PER_STOP
} {
  set_property value_validation_list {1 2 3 4 5 6 7} \
    ${PARAM_VALUE.G_MAX_RETURNS_PER_STOP}
}
proc update_PARAM_VALUE.G_OUTPUT_WIDTH {PARAM_VALUE.G_OUTPUT_WIDTH} {
  set_property value_validation_list {32 64 128} ${PARAM_VALUE.G_OUTPUT_WIDTH}
}
proc update_PARAM_VALUE.G_NUM_FACES {PARAM_VALUE.G_NUM_FACES} {
  set_property value_validation_list {1 2 3 4 5} ${PARAM_VALUE.G_NUM_FACES}
}
proc update_PARAM_VALUE.G_OEN_MODE {PARAM_VALUE.G_OEN_MODE} {
  set_property value_validation_list \
    {DYNAMIC_CONNECTED PULLUP_OR_NOT_CONNECTED} ${PARAM_VALUE.G_OEN_MODE}
}
proc update_PARAM_VALUE.G_ENABLE_ECHO_SIMULATION {
  PARAM_VALUE.G_ENABLE_ECHO_SIMULATION PARAM_VALUE.G_ENABLE_ECHO_RECEIVER
} {
  if {![::tglcv2_xgui::is_true \
      [get_property value ${PARAM_VALUE.G_ENABLE_ECHO_RECEIVER}]]} {
    set_property value false ${PARAM_VALUE.G_ENABLE_ECHO_SIMULATION}
  }
}

foreach name {
  G_RISE_CAPABILITY_MASK G_FALL_CAPABILITY_MASK
  G_ENABLE_ECHO_RECEIVER G_PHASE_TIMEOUT_US G_POWERUP_TIME_NS
  G_RECOVERY_TIME_NS G_ALU_PULSE_TIME_NS G_BUS_IDLE_STABLE_TIME_NS
  G_DRAIN_MARGIN_TIME_NS
} {
  proc update_PARAM_VALUE.$name [list PARAM_VALUE.$name] {}
}

proc validate_PARAM_VALUE.G_PROC_CLK_MHZ {
  PARAM_VALUE.G_PROC_CLK_MHZ PARAM_VALUE.G_TDC_CLK_MHZ
  PARAM_VALUE.G_STREAM_CLK_MODE
} {
  return [::tglcv2_xgui::set_validation_result \
    ${PARAM_VALUE.G_PROC_CLK_MHZ} \
    [::tglcv2_xgui::validate_clock_contract \
      [get_property value ${PARAM_VALUE.G_PROC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_TDC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_STREAM_CLK_MODE}]]]
}
proc validate_PARAM_VALUE.G_TDC_CLK_MHZ {
  PARAM_VALUE.G_TDC_CLK_MHZ PARAM_VALUE.G_PROC_CLK_MHZ
  PARAM_VALUE.G_STREAM_CLK_MODE
} {
  return [::tglcv2_xgui::set_validation_result \
    ${PARAM_VALUE.G_TDC_CLK_MHZ} \
    [::tglcv2_xgui::validate_clock_contract \
      [get_property value ${PARAM_VALUE.G_PROC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_TDC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_STREAM_CLK_MODE}]]]
}
proc validate_PARAM_VALUE.G_STREAM_CLK_MODE {
  PARAM_VALUE.G_STREAM_CLK_MODE PARAM_VALUE.G_PROC_CLK_MHZ
  PARAM_VALUE.G_TDC_CLK_MHZ
} {
  return [::tglcv2_xgui::set_validation_result \
    ${PARAM_VALUE.G_STREAM_CLK_MODE} \
    [::tglcv2_xgui::validate_clock_contract \
      [get_property value ${PARAM_VALUE.G_PROC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_TDC_CLK_MHZ}] \
      [get_property value ${PARAM_VALUE.G_STREAM_CLK_MODE}]]]
}

foreach name {G_NUM_CHIPS G_RISE_CAPABILITY_MASK G_FALL_CAPABILITY_MASK} {
  set body [format {
    return [::tglcv2_xgui::set_validation_result ${PARAM_VALUE.%s} \
      [::tglcv2_xgui::validate_topology \
        [get_property value ${PARAM_VALUE.G_NUM_CHIPS}] \
        [get_property value ${PARAM_VALUE.G_RISE_CAPABILITY_MASK}] \
        [get_property value ${PARAM_VALUE.G_FALL_CAPABILITY_MASK}]]]
  } $name]
  proc validate_PARAM_VALUE.$name [list PARAM_VALUE.G_NUM_CHIPS \
    PARAM_VALUE.G_RISE_CAPABILITY_MASK \
    PARAM_VALUE.G_FALL_CAPABILITY_MASK] $body
}

foreach name {
  G_CSR_CLK_MHZ G_PROC_CLK_MHZ G_TDC_CLK_MHZ G_STREAM_CLK_MODE
  G_NUM_CHIPS G_STOPS_PER_CHIP G_MAX_RETURNS_PER_STOP
  G_RISE_CAPABILITY_MASK G_FALL_CAPABILITY_MASK G_OUTPUT_WIDTH G_NUM_FACES
  G_ENABLE_ECHO_RECEIVER G_ENABLE_ECHO_SIMULATION G_OEN_MODE
  G_PHASE_TIMEOUT_US G_POWERUP_TIME_NS G_RECOVERY_TIME_NS
  G_ALU_PULSE_TIME_NS G_BUS_IDLE_STABLE_TIME_NS G_DRAIN_MARGIN_TIME_NS
} {
  set model_arg MODELPARAM_VALUE.$name
  set user_arg PARAM_VALUE.$name
  set body [format {
    set_property value [get_property value ${%s}] ${%s}
  } $user_arg $model_arg]
  proc update_MODELPARAM_VALUE.$name [list $model_arg $user_arg] $body
}
