-- =============================================================================
-- LiDAR unified CSR IP-Integrator wrapper
--
-- The functional register owner remains lidar_unified_csr_top. This wrapper
-- only gives each child-IP bus a unique physical SYS_CTRL/SYS_CFG_APPLY port
-- so Vivado can group four independent custom Master interfaces without
-- mapping one physical port into several interfaces.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

use work.lidar_unified_csr_pkg.all;

entity lidar_unified_csr_ip_top is
    generic (
        g_VERSION_WORD : std_logic_vector(31 downto 0) :=
            x"4C010000";
        g_CAPABILITY_WORD : std_logic_vector(31 downto 0) :=
            x"01041AFF"
    );
    port (
        s_axi_csr_aclk    : in  std_logic;
        s_axi_csr_aresetn : in  std_logic;
        s_axi_csr_awaddr  : in  std_logic_vector(8 downto 0);
        s_axi_csr_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_csr_awvalid : in  std_logic;
        s_axi_csr_awready : out std_logic;
        s_axi_csr_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_csr_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_csr_wvalid  : in  std_logic;
        s_axi_csr_wready  : out std_logic;
        s_axi_csr_bresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_bvalid  : out std_logic;
        s_axi_csr_bready  : in  std_logic;
        s_axi_csr_araddr  : in  std_logic_vector(8 downto 0);
        s_axi_csr_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_csr_arvalid : in  std_logic;
        s_axi_csr_arready : out std_logic;
        s_axi_csr_rdata   : out std_logic_vector(31 downto 0);
        s_axi_csr_rresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_rvalid  : out std_logic;
        s_axi_csr_rready  : in  std_logic;
        o_irq             : out std_logic;

        -- Motor Decoder unified CSR Master interface.
        o_motor_sys_ctrl       : out std_logic_vector(31 downto 0);
        o_motor_sys_cfg_apply  : out std_logic_vector(31 downto 0);
        o_motor_cfg            : out std_logic_vector(31 downto 0);
        o_motor_ticks_lo       : out std_logic_vector(31 downto 0);
        o_motor_sched_latency  : out std_logic_vector(31 downto 0);
        o_motor_z_param        : out std_logic_vector(31 downto 0);
        o_motor_face_index     : out std_logic_vector(31 downto 0);
        o_motor_face_geometry  : out std_logic_vector(31 downto 0);
        i_motor_status         : in  std_logic_vector(31 downto 0);
        i_motor_face_geometry  : in  std_logic_vector(31 downto 0);
        i_motor_cfg_status     : in  std_logic_vector(31 downto 0);
        i_motor_quad_invalid   : in  std_logic_vector(31 downto 0);
        i_motor_axis_drop      : in  std_logic_vector(31 downto 0);
        i_motor_rev_period     : in  std_logic_vector(31 downto 0);
        i_motor_irq_cause      : in  std_logic_vector(3 downto 0);

        -- Laser Controller unified CSR Master interface.
        o_laser_sys_ctrl      : out std_logic_vector(31 downto 0);
        o_laser_sys_cfg_apply : out std_logic_vector(31 downto 0);
        o_laser_fire_cfg      : out std_logic_vector(31 downto 0);
        o_laser_roundtrip     : out std_logic_vector(31 downto 0);
        o_laser_tdc_width     : out std_logic_vector(31 downto 0);
        o_laser_sim_delay     : out std_logic_vector(31 downto 0);
        o_laser_sched0        : out std_logic_vector(31 downto 0);
        o_laser_sched1        : out std_logic_vector(31 downto 0);
        o_laser_sched2        : out std_logic_vector(31 downto 0);
        i_laser_status          : in std_logic_vector(31 downto 0);
        i_laser_encoder_to_fire : in std_logic_vector(31 downto 0);
        i_laser_fire_done       : in std_logic_vector(31 downto 0);
        i_laser_fire_to_tdc     : in std_logic_vector(31 downto 0);
        i_laser_metric_flags    : in std_logic_vector(31 downto 0);
        i_laser_frame_count     : in std_logic_vector(31 downto 0);
        i_laser_timeout_count   : in std_logic_vector(31 downto 0);
        i_laser_cfg_epoch_accepted : in std_logic_vector(7 downto 0);
        i_laser_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_laser_cfg_busy   : in std_logic;
        i_laser_cfg_reject : in std_logic;
        i_laser_cfg_valid  : in std_logic;
        i_laser_irq_cause  : in std_logic_vector(2 downto 0);

        -- Echo Receiver unified CSR Master interface.
        o_echo_sys_ctrl  : out std_logic_vector(31 downto 0);
        o_echo_delay_cmd : out std_logic_vector(31 downto 0);
        o_echo_delay_data : out std_logic_vector(31 downto 0);
        i_echo_rise_mask      : in std_logic_vector(31 downto 0);
        i_echo_fall_mask      : in std_logic_vector(31 downto 0);
        i_echo_status         : in std_logic_vector(31 downto 0);
        i_echo_delay_readback : in std_logic_vector(31 downto 0);
        i_echo_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_echo_irq_cause      : in std_logic_vector(4 downto 0);

        -- TDC-GPX unified CSR Master interface.
        o_tdc_sys_ctrl      : out std_logic_vector(31 downto 0);
        o_tdc_sys_cfg_apply : out std_logic_vector(31 downto 0);
        o_tdc_bus_timing    : out std_logic_vector(31 downto 0);
        o_tdc_start_offset  : out std_logic_vector(31 downto 0);
        o_tdc_cfg_reg7      : out std_logic_vector(31 downto 0);
        o_tdc_image_cmd     : out std_logic_vector(31 downto 0);
        o_tdc_image_data    : out std_logic_vector(31 downto 0);
        o_tdc_scan_cfg      : out std_logic_vector(31 downto 0);
        o_tdc_pipeline_main : out std_logic_vector(31 downto 0);
        o_tdc_range_cols    : out std_logic_vector(31 downto 0);
        o_tdc_aux_cmd       : out std_logic_vector(31 downto 0);
        i_tdc_chip0_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip1_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip2_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip3_result    : in std_logic_vector(31 downto 0);
        i_tdc_pipeline_status : in std_logic_vector(31 downto 0);
        i_tdc_status_ext      : in std_logic_vector(31 downto 0);
        i_tdc_status_ext2     : in std_logic_vector(31 downto 0);
        i_tdc_cfg_epoch_accepted   : in std_logic_vector(7 downto 0);
        i_tdc_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_tdc_cfg_busy       : in std_logic;
        i_tdc_cfg_reject     : in std_logic;
        i_tdc_cfg_valid      : in std_logic;
        i_tdc_cmd_epoch_accepted : in std_logic_vector(7 downto 0);
        i_tdc_cmd_busy       : in std_logic;
        i_tdc_command_reject : in std_logic;
        i_tdc_image_epoch_accepted : in std_logic_vector(7 downto 0);
        i_tdc_image_reject   : in std_logic;
        i_tdc_image_selected_data : in std_logic_vector(31 downto 0);
        i_tdc_irq_cause      : in std_logic_vector(6 downto 0);

        -- Static/build geometry exposed in STAT3..STAT5.
        i_tdc_max_rows  : in std_logic_vector(31 downto 0) :=
            (others => '0');
        i_tdc_cell_size : in std_logic_vector(31 downto 0) :=
            (others => '0');
        i_tdc_max_hsize : in std_logic_vector(31 downto 0) :=
            (others => '0')
    );

end entity lidar_unified_csr_ip_top;

architecture rtl of lidar_unified_csr_ip_top is
    signal s_sys_ctrl      : std_logic_vector(31 downto 0);
    signal s_sys_cfg_apply : std_logic_vector(31 downto 0);
begin
    o_motor_sys_ctrl      <= s_sys_ctrl;
    o_laser_sys_ctrl      <= s_sys_ctrl;
    o_echo_sys_ctrl       <= s_sys_ctrl;
    o_tdc_sys_ctrl        <= s_sys_ctrl;
    o_motor_sys_cfg_apply <= s_sys_cfg_apply;
    o_laser_sys_cfg_apply <= s_sys_cfg_apply;
    o_tdc_sys_cfg_apply   <= s_sys_cfg_apply;

    u_core : entity work.lidar_unified_csr_top
        generic map (
            g_VERSION_WORD    => g_VERSION_WORD,
            g_CAPABILITY_WORD => g_CAPABILITY_WORD
        )
        port map (
            s_axi_csr_aclk    => s_axi_csr_aclk,
            s_axi_csr_aresetn => s_axi_csr_aresetn,
            s_axi_csr_awaddr  => s_axi_csr_awaddr,
            s_axi_csr_awprot  => s_axi_csr_awprot,
            s_axi_csr_awvalid => s_axi_csr_awvalid,
            s_axi_csr_awready => s_axi_csr_awready,
            s_axi_csr_wdata   => s_axi_csr_wdata,
            s_axi_csr_wstrb   => s_axi_csr_wstrb,
            s_axi_csr_wvalid  => s_axi_csr_wvalid,
            s_axi_csr_wready  => s_axi_csr_wready,
            s_axi_csr_bresp   => s_axi_csr_bresp,
            s_axi_csr_bvalid  => s_axi_csr_bvalid,
            s_axi_csr_bready  => s_axi_csr_bready,
            s_axi_csr_araddr  => s_axi_csr_araddr,
            s_axi_csr_arprot  => s_axi_csr_arprot,
            s_axi_csr_arvalid => s_axi_csr_arvalid,
            s_axi_csr_arready => s_axi_csr_arready,
            s_axi_csr_rdata   => s_axi_csr_rdata,
            s_axi_csr_rresp   => s_axi_csr_rresp,
            s_axi_csr_rvalid  => s_axi_csr_rvalid,
            s_axi_csr_rready  => s_axi_csr_rready,
            o_irq             => o_irq,

            o_sys_ctrl      => s_sys_ctrl,
            o_sys_cfg_apply => s_sys_cfg_apply,
            o_motor_cfg           => o_motor_cfg,
            o_motor_ticks_lo      => o_motor_ticks_lo,
            o_motor_sched_latency => o_motor_sched_latency,
            o_motor_z_param       => o_motor_z_param,
            o_motor_face_index    => o_motor_face_index,
            o_motor_face_geometry => o_motor_face_geometry,
            o_laser_fire_cfg  => o_laser_fire_cfg,
            o_laser_roundtrip => o_laser_roundtrip,
            o_laser_tdc_width => o_laser_tdc_width,
            o_laser_sim_delay => o_laser_sim_delay,
            o_laser_sched0    => o_laser_sched0,
            o_laser_sched1    => o_laser_sched1,
            o_laser_sched2    => o_laser_sched2,
            o_echo_delay_cmd  => o_echo_delay_cmd,
            o_echo_delay_data => o_echo_delay_data,
            o_tdc_bus_timing    => o_tdc_bus_timing,
            o_tdc_start_offset  => o_tdc_start_offset,
            o_tdc_cfg_reg7      => o_tdc_cfg_reg7,
            o_tdc_image_cmd     => o_tdc_image_cmd,
            o_tdc_image_data    => o_tdc_image_data,
            o_tdc_scan_cfg      => o_tdc_scan_cfg,
            o_tdc_pipeline_main => o_tdc_pipeline_main,
            o_tdc_range_cols    => o_tdc_range_cols,
            o_tdc_aux_cmd       => o_tdc_aux_cmd,

            i_tdc_max_rows  => i_tdc_max_rows,
            i_tdc_cell_size => i_tdc_cell_size,
            i_tdc_max_hsize => i_tdc_max_hsize,
            i_motor_status        => i_motor_status,
            i_motor_face_geometry => i_motor_face_geometry,
            i_motor_cfg_status    => i_motor_cfg_status,
            i_motor_quad_invalid  => i_motor_quad_invalid,
            i_motor_axis_drop     => i_motor_axis_drop,
            i_motor_rev_period    => i_motor_rev_period,
            i_motor_irq_cause     => i_motor_irq_cause,
            i_laser_status          => i_laser_status,
            i_laser_encoder_to_fire => i_laser_encoder_to_fire,
            i_laser_fire_done       => i_laser_fire_done,
            i_laser_fire_to_tdc     => i_laser_fire_to_tdc,
            i_laser_metric_flags    => i_laser_metric_flags,
            i_laser_frame_count     => i_laser_frame_count,
            i_laser_timeout_count   => i_laser_timeout_count,
            i_laser_cfg_epoch_accepted => i_laser_cfg_epoch_accepted,
            i_laser_reset_epoch_accepted =>
                i_laser_reset_epoch_accepted,
            i_laser_cfg_busy   => i_laser_cfg_busy,
            i_laser_cfg_reject => i_laser_cfg_reject,
            i_laser_cfg_valid  => i_laser_cfg_valid,
            i_laser_irq_cause  => i_laser_irq_cause,
            i_echo_rise_mask      => i_echo_rise_mask,
            i_echo_fall_mask      => i_echo_fall_mask,
            i_echo_status         => i_echo_status,
            i_echo_delay_readback => i_echo_delay_readback,
            i_echo_reset_epoch_accepted =>
                i_echo_reset_epoch_accepted,
            i_echo_irq_cause      => i_echo_irq_cause,
            i_tdc_chip0_result    => i_tdc_chip0_result,
            i_tdc_chip1_result    => i_tdc_chip1_result,
            i_tdc_chip2_result    => i_tdc_chip2_result,
            i_tdc_chip3_result    => i_tdc_chip3_result,
            i_tdc_pipeline_status => i_tdc_pipeline_status,
            i_tdc_status_ext      => i_tdc_status_ext,
            i_tdc_status_ext2     => i_tdc_status_ext2,
            i_tdc_image_selected_data => i_tdc_image_selected_data,
            i_tdc_cfg_epoch_accepted => i_tdc_cfg_epoch_accepted,
            i_tdc_reset_epoch_accepted => i_tdc_reset_epoch_accepted,
            i_tdc_cfg_busy       => i_tdc_cfg_busy,
            i_tdc_cfg_reject     => i_tdc_cfg_reject,
            i_tdc_cfg_valid      => i_tdc_cfg_valid,
            i_tdc_cmd_busy       => i_tdc_cmd_busy,
            i_tdc_command_reject => i_tdc_command_reject,
            i_tdc_image_reject   => i_tdc_image_reject,
            i_tdc_irq_cause      => i_tdc_irq_cause
        );

    -- CMD/IMAGE accepted epochs are already carried in TDC_PIPELINE_STATUS
    -- [23:16]/[31:24]. Dedicated pins remain in the custom bus so its Master
    -- contract exactly matches tdc_gpx_top and can evolve without repackaging
    -- either endpoint.
end architecture rtl;
