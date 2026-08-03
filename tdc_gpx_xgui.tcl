# Canonical Vivado XGUI for tdc_gpx_top.

namespace eval ::tdc_gpx_xgui {
  variable supported_clocks {50 100 125 150 200}
  variable supported_output_widths {32 64 128}

  proc as_integer {value label} {
    if {![string is entier -strict $value]} {
      error "$label must be an integer."
    }
    return [expr {wide($value)}]
  }

  proc normalize_bits {value width label} {
    set bits [string trim [string map {_ {}} $value] {"}]
    if {[regexp -nocase {^0x([0-9a-f]+)$} $bits -> hex]} {
      scan $hex %x numeric
      set bits [format %0*b $width $numeric]
    }
    if {![regexp "^[01]{$width}$" $bits]} {
      error "$label must contain exactly $width binary digits."
    }
    return $bits
  }

  proc bits_to_int {bits} {
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

  proc validate_topology {num_chips present_mask rise_mask fall_mask} {
    if {[catch {
      set n [as_integer $num_chips {Physical chip count}]
      set present [bits_to_int [normalize_bits $present_mask 4 {Present chip mask}]]
      set rise [bits_to_int [normalize_bits $rise_mask 4 {Rising chip mask}]]
      set fall [bits_to_int [normalize_bits $fall_mask 4 {Falling chip mask}]]
    } message]} {
      return [list false $message]
    }
    if {$n < 1 || $n > 4} {
      return [list false {Physical chip count must be from 1 to 4.}]
    }
    if {$present == 0} {
      return [list false {At least one TDC-GPX chip must be present.}]
    }
    if {$n != [popcount $present]} {
      return [list false {Physical chip count must equal the number of 1 bits in Present chip mask.}]
    }
    if {(($rise | $fall) & ((~$present) & 0xF)) != 0} {
      return [list false {Rise/Fall masks must not select an absent chip.}]
    }
    if {(($rise | $fall) & $present) != $present} {
      return [list false {Every present chip must be assigned to rising and/or falling acquisition.}]
    }
    if {[popcount [expr {$rise & $present}]] <
        [popcount [expr {$fall & $present}]]} {
      return [list false {The rising-capable chip count must not be smaller than the falling-capable chip count.}]
    }
    return [list true {}]
  }

  proc validate_clocks {axis_mhz tdc_mhz stream_mode} {
    variable supported_clocks
    if {[catch {
      set axis [as_integer $axis_mhz {AXIS clock frequency}]
      set tdc [as_integer $tdc_mhz {TDC clock frequency}]
    } message]} {
      return [list false $message]
    }
    set mode [string toupper [string trim $stream_mode {"}]]
    if {$axis ni $supported_clocks || $tdc ni $supported_clocks} {
      return [list false {Clock frequency must be 50, 100, 125, 150, or 200 MHz.}]
    }
    if {$mode ni {ASYNC SYNC}} {
      return [list false {Stream clock mode must be ASYNC or SYNC.}]
    }
    if {$mode eq {SYNC} && $axis != $tdc} {
      return [list false {SYNC stream mode requires equal AXIS and TDC clock frequencies and one shared physical clock net.}]
    }
    return [list true {}]
  }

  proc validate_output_width {output_width} {
    variable supported_output_widths
    if {[catch {
      set width [as_integer $output_width {AXI4-Stream output width}]
    } message]} {
      return [list false $message]
    }
    if {$width ni $supported_output_widths} {
      return [list false {AXI4-Stream output width must be 32, 64, or 128 bits.}]
    }
    return [list true {}]
  }

  proc time_to_clocks {time_ns clock_mhz} {
    set ns [as_integer $time_ns {Time value}]
    set mhz [as_integer $clock_mhz {Clock frequency}]
    if {$ns <= 0 || $mhz <= 0} {
      error {Time and clock frequency must be positive.}
    }
    return [expr {($ns * $mhz + 999) / 1000}]
  }

  proc validate_time_limit {time_ns clock_mhz maximum label} {
    if {[catch {set clocks [time_to_clocks $time_ns $clock_mhz]} message]} {
      return [list false $message]
    }
    if {$clocks > $maximum} {
      return [list false "$label converts to $clocks clocks; the RTL limit is $maximum."]
    }
    return [list true {}]
  }
}

proc init_gui {IPINST} {
  set build_page [ipgui::add_page $IPINST -name {Build Profile}]
  set identity [ipgui::add_group $IPINST -name {Identity and Output} -parent $build_page]
  ipgui::add_param $IPINST -name g_HW_VERSION -parent $identity
  ipgui::add_param $IPINST -name g_OUTPUT_WIDTH -parent $identity -widget comboBox

  set control_plane [ipgui::add_group $IPINST -name {Control Plane Owner} -parent $build_page]
  ipgui::add_param $IPINST -name g_ENABLE_LOCAL_CSR -parent $control_plane -widget checkBox
  ipgui::add_static_text $IPINST -name control_plane_help -parent $control_plane -text {Enabled exposes both legacy AXI4-Lite CSR banks. Disabled removes those banks and exposes one named TDC-GPX Unified CSR interface for the parent system CSR.}

  set geometry [ipgui::add_group $IPINST -name {Physical TDC-GPX Geometry} -parent $build_page]
  ipgui::add_param $IPINST -name g_NUM_CHIPS -parent $geometry -widget comboBox
  ipgui::add_param $IPINST -name g_PRESENT_CHIP_MASK -parent $geometry
  ipgui::add_param $IPINST -name g_MAX_STOPS_PER_CHIP -parent $geometry -widget comboBox
  ipgui::add_param $IPINST -name g_MAX_HITS_PER_STOP -parent $geometry -widget comboBox
  ipgui::add_static_text $IPINST -name geometry_help -parent $geometry -text {Physical pin widths follow g_NUM_CHIPS. Logical chip IDs follow the 1 bits of the four-bit present mask in ascending order.}

  set slope_page [ipgui::add_page $IPINST -name {Slope Topology}]
  set slope [ipgui::add_group $IPINST -name {Per-chip Edge Capability} -parent $slope_page]
  ipgui::add_param $IPINST -name g_RISE_CHIP_MASK -parent $slope
  ipgui::add_param $IPINST -name g_FALL_CHIP_MASK -parent $slope
  ipgui::add_static_text $IPINST -name slope_help -parent $slope -text {Masks may overlap for same-chip dual-edge acquisition. Use fall mask 0000 for a rising-only build. Every present chip needs a role, and rising-capable chip count must be at least the falling-capable count.}

  set clock_page [ipgui::add_page $IPINST -name {Clock Contract}]
  set clocks [ipgui::add_group $IPINST -name {Signal-processing Clocks} -parent $clock_page]
  ipgui::add_param $IPINST -name g_AXIS_CLK_MHZ -parent $clocks -widget comboBox
  ipgui::add_param $IPINST -name g_TDC_CLK_MHZ -parent $clocks -widget comboBox
  ipgui::add_param $IPINST -name g_STREAM_CLK_MODE -parent $clocks -widget comboBox
  ipgui::add_static_text $IPINST -name clock_help -parent $clocks -text {These values do not generate clocks and must match the connected FCLKs. ASYNC permits either frequency order. SYNC requires equal values and the same physical clock net; two independent equal-frequency clocks are not synchronous. A slower TDC clock is supported structurally, but GPX bus timing, drain margin, and shot throughput must be re-verified for the selected clock pair.}

  set timing_page [ipgui::add_page $IPINST -name {Physical Timing}]
  set chip_timing [ipgui::add_group $IPINST -name {TDC Bus Domain (ns)} -parent $timing_page]
  foreach name {
    g_POWERUP_TIME_NS g_RECOVERY_TIME_NS g_ALU_PULSE_TIME_NS
    g_BUS_READ_PERIOD_MIN_TIME_NS g_BUS_IDLE_STABLE_TIME_NS
    g_DRAIN_MARGIN_TIME_NS
  } {
    ipgui::add_param $IPINST -name $name -parent $chip_timing
  }
  ipgui::add_param $IPINST -name g_OEN_MODE -parent $chip_timing -widget comboBox

  set axis_timing [ipgui::add_group $IPINST -name {AXIS Domain (ns)} -parent $timing_page]
  foreach name {
    g_ERR_DEBOUNCE_TIME_NS g_CELL_QUARANTINE_MARGIN_TIME_NS
    g_CELL_IFIFO2_MARGIN_TIME_NS
  } {
    ipgui::add_param $IPINST -name $name -parent $axis_timing
  }
  ipgui::add_param $IPINST -name g_ERR_MAX_RETRIES -parent $axis_timing
  ipgui::add_static_text $IPINST -name timing_help -parent $timing_page -text {All time generics are converted to local clock counts at elaboration. Runtime distance and scan limits remain CSR values expressed in 200 MHz reference ticks (5 ns).}

  set csr_page [ipgui::add_page $IPINST -name {Interfaces and CSR}]
  set interface_group [ipgui::add_group $IPINST -name {IPI Interfaces} -parent $csr_page]
  ipgui::add_static_text $IPINST -name interface_help0 -parent $interface_group -text {Local mode: s_axi is the 9-bit chip bank and s_axi_pipe is the 7-bit pipeline bank. Unified mode: both are replaced by tdc_unified_csr and i_unified_cfg_clk.}
  ipgui::add_static_text $IPINST -name interface_help1 -parent $interface_group -text {o_m_axis and o_m_axis_fall are independent AXI4-Stream masters in i_axis_aclk. Falling AXIS may remain unconnected for a rising-only build.}
  ipgui::add_static_text $IPINST -name interface_help2 -parent $interface_group -text {i_n_faces is static geometry from Motor Decoder. i_shot_start, face index, and i_stop_tdc are synchronous to i_axis_aclk.}

  set csr_group [ipgui::add_group $IPINST -name {Runtime Address Windows} -parent $csr_page]
  ipgui::add_static_text $IPINST -name csr_chip -parent $csr_group -text {Chip bank: CTL0..31 at 0x000..0x07C, STAT0..31 at 0x080..0x0FC, interrupt registers at 0x100..0x10C.}
  ipgui::add_static_text $IPINST -name csr_pipe -parent $csr_group -text {Pipeline bank: CTL0..7 at 0x00..0x1C and published STAT0..7 at 0x40..0x5C. Pipeline interrupt source is reserved.}
}

proc tdc_set_topology_error {parameter num_chips present rise fall} {
  lassign [::tdc_gpx_xgui::validate_topology $num_chips $present $rise $fall] valid message
  set_property errmsg $message $parameter
  return $valid
}

proc tdc_set_clock_error {parameter axis tdc mode} {
  lassign [::tdc_gpx_xgui::validate_clocks $axis $tdc $mode] valid message
  set_property errmsg $message $parameter
  return $valid
}

proc tdc_set_output_width_error {parameter output_width} {
  lassign [::tdc_gpx_xgui::validate_output_width $output_width] valid message
  set_property errmsg $message $parameter
  return $valid
}

proc tdc_set_time_error {parameter time_ns clock_mhz maximum label} {
  lassign [::tdc_gpx_xgui::validate_time_limit $time_ns $clock_mhz $maximum $label] valid message
  set_property errmsg $message $parameter
  return $valid
}

proc validate_PARAM_VALUE.g_NUM_CHIPS {PARAM_VALUE.g_NUM_CHIPS PARAM_VALUE.g_PRESENT_CHIP_MASK PARAM_VALUE.g_RISE_CHIP_MASK PARAM_VALUE.g_FALL_CHIP_MASK} {
  return [tdc_set_topology_error ${PARAM_VALUE.g_NUM_CHIPS} [get_property value ${PARAM_VALUE.g_NUM_CHIPS}] [get_property value ${PARAM_VALUE.g_PRESENT_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_RISE_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_FALL_CHIP_MASK}]]
}
proc validate_PARAM_VALUE.g_PRESENT_CHIP_MASK {PARAM_VALUE.g_PRESENT_CHIP_MASK PARAM_VALUE.g_NUM_CHIPS PARAM_VALUE.g_RISE_CHIP_MASK PARAM_VALUE.g_FALL_CHIP_MASK} {
  return [tdc_set_topology_error ${PARAM_VALUE.g_PRESENT_CHIP_MASK} [get_property value ${PARAM_VALUE.g_NUM_CHIPS}] [get_property value ${PARAM_VALUE.g_PRESENT_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_RISE_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_FALL_CHIP_MASK}]]
}
proc validate_PARAM_VALUE.g_RISE_CHIP_MASK {PARAM_VALUE.g_RISE_CHIP_MASK PARAM_VALUE.g_NUM_CHIPS PARAM_VALUE.g_PRESENT_CHIP_MASK PARAM_VALUE.g_FALL_CHIP_MASK} {
  return [tdc_set_topology_error ${PARAM_VALUE.g_RISE_CHIP_MASK} [get_property value ${PARAM_VALUE.g_NUM_CHIPS}] [get_property value ${PARAM_VALUE.g_PRESENT_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_RISE_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_FALL_CHIP_MASK}]]
}
proc validate_PARAM_VALUE.g_FALL_CHIP_MASK {PARAM_VALUE.g_FALL_CHIP_MASK PARAM_VALUE.g_NUM_CHIPS PARAM_VALUE.g_PRESENT_CHIP_MASK PARAM_VALUE.g_RISE_CHIP_MASK} {
  return [tdc_set_topology_error ${PARAM_VALUE.g_FALL_CHIP_MASK} [get_property value ${PARAM_VALUE.g_NUM_CHIPS}] [get_property value ${PARAM_VALUE.g_PRESENT_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_RISE_CHIP_MASK}] [get_property value ${PARAM_VALUE.g_FALL_CHIP_MASK}]]
}

proc validate_PARAM_VALUE.g_OUTPUT_WIDTH {PARAM_VALUE.g_OUTPUT_WIDTH} {
  return [tdc_set_output_width_error ${PARAM_VALUE.g_OUTPUT_WIDTH} [get_property value ${PARAM_VALUE.g_OUTPUT_WIDTH}]]
}

proc validate_PARAM_VALUE.g_AXIS_CLK_MHZ {PARAM_VALUE.g_AXIS_CLK_MHZ PARAM_VALUE.g_TDC_CLK_MHZ PARAM_VALUE.g_STREAM_CLK_MODE} {
  return [tdc_set_clock_error ${PARAM_VALUE.g_AXIS_CLK_MHZ} [get_property value ${PARAM_VALUE.g_AXIS_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_STREAM_CLK_MODE}]]
}
proc validate_PARAM_VALUE.g_TDC_CLK_MHZ {PARAM_VALUE.g_TDC_CLK_MHZ PARAM_VALUE.g_AXIS_CLK_MHZ PARAM_VALUE.g_STREAM_CLK_MODE} {
  return [tdc_set_clock_error ${PARAM_VALUE.g_TDC_CLK_MHZ} [get_property value ${PARAM_VALUE.g_AXIS_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_STREAM_CLK_MODE}]]
}
proc validate_PARAM_VALUE.g_STREAM_CLK_MODE {PARAM_VALUE.g_STREAM_CLK_MODE PARAM_VALUE.g_AXIS_CLK_MHZ PARAM_VALUE.g_TDC_CLK_MHZ} {
  return [tdc_set_clock_error ${PARAM_VALUE.g_STREAM_CLK_MODE} [get_property value ${PARAM_VALUE.g_AXIS_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] [get_property value ${PARAM_VALUE.g_STREAM_CLK_MODE}]]
}

proc validate_PARAM_VALUE.g_POWERUP_TIME_NS {PARAM_VALUE.g_POWERUP_TIME_NS PARAM_VALUE.g_TDC_CLK_MHZ} {
  return [tdc_set_time_error ${PARAM_VALUE.g_POWERUP_TIME_NS} [get_property value ${PARAM_VALUE.g_POWERUP_TIME_NS}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] 65535 {Power-up time}]
}
proc validate_PARAM_VALUE.g_BUS_READ_PERIOD_MIN_TIME_NS {PARAM_VALUE.g_BUS_READ_PERIOD_MIN_TIME_NS PARAM_VALUE.g_TDC_CLK_MHZ} {
  return [tdc_set_time_error ${PARAM_VALUE.g_BUS_READ_PERIOD_MIN_TIME_NS} [get_property value ${PARAM_VALUE.g_BUS_READ_PERIOD_MIN_TIME_NS}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] 253 {Bus read capture window}]
}
proc validate_PARAM_VALUE.g_BUS_IDLE_STABLE_TIME_NS {PARAM_VALUE.g_BUS_IDLE_STABLE_TIME_NS PARAM_VALUE.g_TDC_CLK_MHZ} {
  return [tdc_set_time_error ${PARAM_VALUE.g_BUS_IDLE_STABLE_TIME_NS} [get_property value ${PARAM_VALUE.g_BUS_IDLE_STABLE_TIME_NS}] [get_property value ${PARAM_VALUE.g_TDC_CLK_MHZ}] 16777215 {Bus idle stable time}]
}
proc validate_PARAM_VALUE.g_CELL_QUARANTINE_MARGIN_TIME_NS {PARAM_VALUE.g_CELL_QUARANTINE_MARGIN_TIME_NS PARAM_VALUE.g_AXIS_CLK_MHZ} {
  return [tdc_set_time_error ${PARAM_VALUE.g_CELL_QUARANTINE_MARGIN_TIME_NS} [get_property value ${PARAM_VALUE.g_CELL_QUARANTINE_MARGIN_TIME_NS}] [get_property value ${PARAM_VALUE.g_AXIS_CLK_MHZ}] 65535 {Cell quarantine margin}]
}

proc update_PARAM_VALUE.g_OEN_MODE {PARAM_VALUE.g_OEN_MODE} {
  set_property enabled false ${PARAM_VALUE.g_OEN_MODE}
}

foreach name {
  g_HW_VERSION g_ENABLE_LOCAL_CSR g_OUTPUT_WIDTH
  g_RISE_CHIP_MASK g_FALL_CHIP_MASK
  g_NUM_CHIPS g_PRESENT_CHIP_MASK g_MAX_STOPS_PER_CHIP
  g_MAX_HITS_PER_STOP g_AXIS_CLK_MHZ g_TDC_CLK_MHZ
  g_POWERUP_TIME_NS g_RECOVERY_TIME_NS g_ALU_PULSE_TIME_NS
  g_BUS_READ_PERIOD_MIN_TIME_NS g_BUS_IDLE_STABLE_TIME_NS
  g_DRAIN_MARGIN_TIME_NS g_ERR_DEBOUNCE_TIME_NS g_ERR_MAX_RETRIES
  g_CELL_QUARANTINE_MARGIN_TIME_NS g_CELL_IFIFO2_MARGIN_TIME_NS
  g_STREAM_CLK_MODE
} {
  proc update_PARAM_VALUE.$name [list PARAM_VALUE.$name] {}
}

foreach name {
  g_HW_VERSION g_ENABLE_LOCAL_CSR g_OUTPUT_WIDTH
  g_RISE_CHIP_MASK g_FALL_CHIP_MASK
  g_NUM_CHIPS g_PRESENT_CHIP_MASK g_MAX_STOPS_PER_CHIP
  g_MAX_HITS_PER_STOP g_AXIS_CLK_MHZ g_TDC_CLK_MHZ
  g_POWERUP_TIME_NS g_RECOVERY_TIME_NS g_ALU_PULSE_TIME_NS
  g_BUS_READ_PERIOD_MIN_TIME_NS g_BUS_IDLE_STABLE_TIME_NS
  g_DRAIN_MARGIN_TIME_NS g_ERR_DEBOUNCE_TIME_NS g_ERR_MAX_RETRIES
  g_CELL_QUARANTINE_MARGIN_TIME_NS g_CELL_IFIFO2_MARGIN_TIME_NS
  g_OEN_MODE g_STREAM_CLK_MODE
} {
  proc update_MODELPARAM_VALUE.$name \
      [list MODELPARAM_VALUE.$name PARAM_VALUE.$name] \
      [string map [list @NAME@ $name] {
    set_property value [get_property value ${PARAM_VALUE.@NAME@}] \
      ${MODELPARAM_VALUE.@NAME@}
  }]
}
