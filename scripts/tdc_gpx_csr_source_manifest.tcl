# Canonical source-level CSR dependencies used by tdc_gpx.
#
# The returned files are ordered for VHDL analysis.  Keeping this list in one
# place prevents standalone, parent-reference, and package builds from silently
# falling back to generated XCI products.
proc tdc_gpx_csr_source_manifest {ip_root} {
    set csr8_dir  [file normalize [file join $ip_root my_axil_csr HDL]]
    set csr32_dir [file normalize [file join $ip_root my_axil_csr32 HDL]]

    return [list \
        [file join $csr8_dir axil_fsm.vhd] \
        [file join $csr8_dir axil_ctrl_regs.vhd] \
        [file join $csr8_dir axil_stat_regs.vhd] \
        [file join $csr8_dir axil_intr.vhd] \
        [file join $csr8_dir my_axil_csr_top.vhd] \
        [file join $csr32_dir my_axil_csr32_pkg.vhd] \
        [file join $csr32_dir axil_fsm_32.vhd] \
        [file join $csr32_dir axil_ctrl_regs_32.vhd] \
        [file join $csr32_dir axil_stat_regs_32.vhd] \
        [file join $csr32_dir axil_intr_32.vhd] \
        [file join $csr32_dir my_axil_csr32_top.vhd]]
}
