# Canonical synthesis-order manifest shared by standalone OOC and parent flows.
proc tdc_gpx_rtl_manifest {} {
    return [list \
        px_utility_pkg.vhd \
        tdc_gpx_pkg.vhd \
        tdc_gpx_cfg_pkg.vhd \
        tdc_gpx_unified_cdc_snapshot.vhd \
        tdc_gpx_unified_csr_adapter.vhd \
        tdc_gpx_atomic_snapshot_cdc.vhd \
        tdc_gpx_bus_phy.vhd \
        tdc_gpx_skid_buffer.vhd \
        tdc_gpx_sync_fifo.vhd \
        tdc_gpx_cell_builder.vhd \
        tdc_gpx_cell_pipe.vhd \
        tdc_gpx_chip_init.vhd \
        tdc_gpx_chip_run.vhd \
        tdc_gpx_chip_reg.vhd \
        tdc_gpx_chip_ctrl.vhd \
        tdc_gpx_csr_chip.vhd \
        tdc_gpx_cmd_arb.vhd \
        tdc_gpx_err_handler.vhd \
        tdc_gpx_cfg_image_override.vhd \
        tdc_gpx_reg_rsp_cdc.vhd \
        tdc_gpx_config_ctrl.vhd \
        tdc_gpx_decoder_i_mode.vhd \
        tdc_gpx_raw_event_builder.vhd \
        tdc_gpx_decode_pipe.vhd \
        tdc_gpx_face_assembler.vhd \
        tdc_gpx_line_packer.vhd \
        tdc_gpx_header_inserter.vhd \
        tdc_gpx_face_seq.vhd \
        tdc_gpx_output_stage.vhd \
        tdc_gpx_csr_pipeline.vhd \
        tdc_gpx_status_agg.vhd \
        tdc_gpx_top.vhd]
}
