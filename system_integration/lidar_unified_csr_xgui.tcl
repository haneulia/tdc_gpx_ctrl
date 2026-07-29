# Canonical Vivado XGUI for the LiDAR unified CSR owner.

proc init_gui {IPINST} {
  set overview [ipgui::add_page $IPINST -name {Unified CSR}]
  set identity [ipgui::add_group $IPINST -name {Fixed ABI Identity} -parent $overview]
  foreach name {g_VERSION_WORD g_CAPABILITY_WORD} {
    set widget [ipgui::add_param $IPINST -name $name -parent $identity]
    set_property enabled false $widget
  }
  ipgui::add_static_text $IPINST -name identity_help -parent $identity -text {Read-only build identity. STAT0 is the ABI version and STAT1 advertises Motor, Laser, Echo, TDC, indexed windows, epochs, and register capacity.}

  set ownership [ipgui::add_group $IPINST -name {Control-plane Ownership} -parent $overview]
  ipgui::add_static_text $IPINST -name owner_help -parent $ownership -text {This IP is the only AXI4-Lite register owner in unified mode: 32 Control, 32 Status, and four interrupt registers. Child IPs must use g_ENABLE_LOCAL_CSR=false.}
  ipgui::add_static_text $IPINST -name realtime_help -parent $ownership -text {Motor position, Laser fire/start/stop, Echo STOP, GPX physical bus, and VDMA AXI4-Stream remain direct real-time paths and never pass through this CSR bank.}

  set map_page [ipgui::add_page $IPINST -name {Register Map}]
  set control [ipgui::add_group $IPINST -name {Control 0x000 - 0x07C} -parent $map_page]
  ipgui::add_static_text $IPINST -name ctl_system -parent $control -text {CTL0..1: System control, reset epoch, and shared configuration epoch}
  ipgui::add_static_text $IPINST -name ctl_motor -parent $control -text {CTL2..7: Motor configuration, timing, latency, Z, and indexed Face geometry}
  ipgui::add_static_text $IPINST -name ctl_laser -parent $control -text {CTL8..14: Laser fire, round-trip, TDC width, simulation, and scheduler values}
  ipgui::add_static_text $IPINST -name ctl_echo -parent $control -text {CTL15..16: Echo indexed delay command and data}
  ipgui::add_static_text $IPINST -name ctl_tdc -parent $control -text {CTL17..25: TDC bus timing, image, scan, range, pipeline, and command values}
  ipgui::add_static_text $IPINST -name ctl_reserved -parent $control -text {CTL26..31: Reserved; writes read back but have no processing effect}

  set status [ipgui::add_group $IPINST -name {Status 0x080 - 0x0FC} -parent $map_page]
  ipgui::add_static_text $IPINST -name stat_system -parent $status -text {STAT0..5: ABI identity, capability, epochs, and static TDC geometry}
  ipgui::add_static_text $IPINST -name stat_motor -parent $status -text {STAT6..11: Motor runtime, selected Face, apply state, diagnostics, and revolution period}
  ipgui::add_static_text $IPINST -name stat_laser -parent $status -text {STAT12..18: Laser state, measured latency, frame counters, and timeout count}
  ipgui::add_static_text $IPINST -name stat_echo -parent $status -text {STAT19..22: Echo edge masks, diagnostics, and indexed delay readback}
  ipgui::add_static_text $IPINST -name stat_tdc -parent $status -text {STAT23..30: GPX register results, pipeline diagnostics, and image readback}
  ipgui::add_static_text $IPINST -name stat_summary -parent $status -text {STAT31: Cross-IP config/reset acceptance, busy, reject, and valid summary}

  set irq_page [ipgui::add_page $IPINST -name {Interrupts}]
  set irq_regs [ipgui::add_group $IPINST -name {Registers 0x100 - 0x10C} -parent $irq_page]
  ipgui::add_static_text $IPINST -name irq_en -parent $irq_regs -text {0x100 INTR_EN, 0x104 INTR_STATUS, 0x108 INTR_FLAG (W1C), 0x10C INTR_MODE}
  ipgui::add_static_text $IPINST -name irq_service -parent $irq_regs -text {Configure MODE, clear stale FLAG bits, then enable EN. Manual mode keeps irq asserted until the corresponding FLAG bit is cleared.}

  set irq_map [ipgui::add_group $IPINST -name {Source Allocation} -parent $irq_page]
  ipgui::add_static_text $IPINST -name irq_motor -parent $irq_map -text {IRQ4..7 Motor; IRQ8..10 Laser; IRQ16..17 Echo; IRQ21..27 TDC-GPX}
  ipgui::add_static_text $IPINST -name irq_reserved -parent $irq_map -text {IRQ0..3, 11..15, 18..20, and 28..31 are reserved and driven low.}

  set interface_page [ipgui::add_page $IPINST -name {IPI Contract}]
  set buses [ipgui::add_group $IPINST -name {Interfaces} -parent $interface_page]
  ipgui::add_static_text $IPINST -name axi_help -parent $buses -text {Connect s_axi_csr to one PS AXI4-Lite master. The connected FCLK defines the CSR and unified-configuration clock domain.}
  ipgui::add_static_text $IPINST -name child_help -parent $buses -text {Connect the Motor, Laser, Echo, and TDC Unified CSR Master interfaces to matching child Slave interfaces. The custom buses include controls, coherent status, transaction acknowledgements, and IRQ causes.}
  ipgui::add_static_text $IPINST -name static_help -parent $buses -text {TDC max rows, cell size, and max HSIZE are static build sidebands used only for STAT3..STAT5 readback.}
}

proc update_PARAM_VALUE.g_VERSION_WORD {PARAM_VALUE.g_VERSION_WORD} {
  set_property enabled false ${PARAM_VALUE.g_VERSION_WORD}
}

proc update_PARAM_VALUE.g_CAPABILITY_WORD {PARAM_VALUE.g_CAPABILITY_WORD} {
  set_property enabled false ${PARAM_VALUE.g_CAPABILITY_WORD}
}

proc validate_PARAM_VALUE.g_VERSION_WORD {PARAM_VALUE.g_VERSION_WORD} {
  return true
}

proc validate_PARAM_VALUE.g_CAPABILITY_WORD {PARAM_VALUE.g_CAPABILITY_WORD} {
  return true
}

proc update_MODELPARAM_VALUE.g_VERSION_WORD {
  MODELPARAM_VALUE.g_VERSION_WORD PARAM_VALUE.g_VERSION_WORD
} {
  set_property value [get_property value ${PARAM_VALUE.g_VERSION_WORD}] \
    ${MODELPARAM_VALUE.g_VERSION_WORD}
}

proc update_MODELPARAM_VALUE.g_CAPABILITY_WORD {
  MODELPARAM_VALUE.g_CAPABILITY_WORD PARAM_VALUE.g_CAPABILITY_WORD
} {
  set_property value [get_property value ${PARAM_VALUE.g_CAPABILITY_WORD}] \
    ${MODELPARAM_VALUE.g_CAPABILITY_WORD}
}
