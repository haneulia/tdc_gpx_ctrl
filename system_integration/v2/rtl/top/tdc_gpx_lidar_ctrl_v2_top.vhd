library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_processing_pkg.all;

-- Stage 8 K0 integration boundary. Completed sub-stages use their production
-- owners; not-yet-integrated physical/data boundaries remain fail-safe. This
-- architecture must not be packaged as the release IP until all K0 gates pass.
entity tdc_gpx_lidar_ctrl_v2_top is
    generic (
        G_CSR_CLK_MHZ      : positive range 50 to 200 := 100;
        G_PROC_CLK_MHZ     : positive range 50 to 200 := 150;
        G_TDC_CLK_MHZ      : positive range 50 to 200 := 200;
        G_STREAM_CLK_MODE  : string := "ASYNC";
        G_NUM_CHIPS        : positive range 1 to C_MAX_CHIPS := 4;
        G_STOPS_PER_CHIP   : positive range 1 to
            C_MAX_STOPS_PER_CHIP := 8;
        G_MAX_RETURNS_PER_STOP : positive range 1 to
            C_MAX_RETURNS_PER_STOP := 7;
        G_RISE_CAPABILITY_MASK : std_logic_vector(
            C_MAX_CHIPS - 1 downto 0) := "0011";
        G_FALL_CAPABILITY_MASK : std_logic_vector(
            C_MAX_CHIPS - 1 downto 0) := "1100";
        G_OUTPUT_WIDTH     : positive := 32;
        G_NUM_FACES        : positive range 1 to C_MAX_FACES := 5;
        G_ENABLE_ECHO_RECEIVER   : boolean := true;
        G_ENABLE_ECHO_SIMULATION : boolean := false;
        G_OEN_MODE                : string := "DYNAMIC_CONNECTED";
        G_PHASE_TIMEOUT_US        : positive := 1000;
        G_POWERUP_TIME_NS         : positive := 240;
        G_RECOVERY_TIME_NS        : positive := 40;
        G_ALU_PULSE_TIME_NS       : positive := 20;
        G_BUS_IDLE_STABLE_TIME_NS : positive := 20480;
        G_DRAIN_MARGIN_TIME_NS    : positive := 6000
    );
    port (
        s_axi_csr_aclk    : in  std_logic;
        s_axi_csr_aresetn : in  std_logic;
        proc_aclk         : in  std_logic;
        proc_aresetn      : in  std_logic;
        i_tdc_clk         : in  std_logic;
        i_tdc_aresetn     : in  std_logic;

        s_axi_csr_awaddr  : in  std_logic_vector(8 downto 0) :=
            (others => '0');
        s_axi_csr_awprot  : in  std_logic_vector(2 downto 0) :=
            (others => '0');
        s_axi_csr_awvalid : in  std_logic := '0';
        s_axi_csr_awready : out std_logic;
        s_axi_csr_wdata   : in  std_logic_vector(31 downto 0) :=
            (others => '0');
        s_axi_csr_wstrb   : in  std_logic_vector(3 downto 0) :=
            (others => '0');
        s_axi_csr_wvalid  : in  std_logic := '0';
        s_axi_csr_wready  : out std_logic;
        s_axi_csr_bresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_bvalid  : out std_logic;
        s_axi_csr_bready  : in  std_logic := '0';
        s_axi_csr_araddr  : in  std_logic_vector(8 downto 0) :=
            (others => '0');
        s_axi_csr_arprot  : in  std_logic_vector(2 downto 0) :=
            (others => '0');
        s_axi_csr_arvalid : in  std_logic := '0';
        s_axi_csr_arready : out std_logic;
        s_axi_csr_rdata   : out std_logic_vector(31 downto 0);
        s_axi_csr_rresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_rvalid  : out std_logic;
        s_axi_csr_rready  : in  std_logic := '0';
        o_irq             : out std_logic;

        i_external_laser_permit : in  std_logic := '0';
        i_enc_a                  : in  std_logic := '0';
        i_enc_b                  : in  std_logic := '0';
        i_enc_z                  : in  std_logic := '0';
        i_fire_done              : in  std_logic := '0';
        o_fire_pulse             : out std_logic;
        o_start_tdc              : out std_logic;
        o_stop_tdc               : out std_logic;
        o_shot_start             : out std_logic;
        o_shot_face_index        : out std_logic_vector(2 downto 0);
        o_n_faces                : out std_logic_vector(2 downto 0);

        i_pd_lvds_p : in std_logic_vector(
            G_NUM_CHIPS * G_STOPS_PER_CHIP - 1 downto 0) :=
            (others => '0');
        i_pd_lvds_n : in std_logic_vector(
            G_NUM_CHIPS * G_STOPS_PER_CHIP - 1 downto 0) :=
            (others => '0');
        o_tdc_stop : out std_logic_vector(
            G_NUM_CHIPS * G_STOPS_PER_CHIP - 1 downto 0);

        io_tdc_d : inout std_logic_vector(
            G_NUM_CHIPS * 28 - 1 downto 0);
        o_tdc_adr : out std_logic_vector(
            G_NUM_CHIPS * 4 - 1 downto 0);
        o_tdc_csn        : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_rdn        : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_wrn        : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_oen        : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_stopdis    : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_alutrigger : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        o_tdc_puresn     : out std_logic_vector(G_NUM_CHIPS - 1 downto 0);
        i_tdc_ef1        : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '1');
        i_tdc_ef2        : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '1');
        i_tdc_lf1        : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '0');
        i_tdc_lf2        : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '0');
        i_tdc_irflag     : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '0');
        i_tdc_errflag    : in  std_logic_vector(G_NUM_CHIPS - 1 downto 0) :=
            (others => '0');

        m_axis_monitor_tdata  : out std_logic_vector(
            C_PROCESSING_MONITOR_TDATA_WIDTH - 1 downto 0);
        m_axis_monitor_tkeep  : out std_logic_vector(
            C_PROCESSING_MONITOR_TKEEP_WIDTH - 1 downto 0);
        m_axis_monitor_tuser  : out std_logic_vector(
            C_PROCESSING_MONITOR_TUSER_WIDTH - 1 downto 0);
        m_axis_monitor_tvalid : out std_logic;
        m_axis_monitor_tlast  : out std_logic;
        m_axis_monitor_tready : in  std_logic := '0';

        m_axis_rise_tdata  : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        m_axis_rise_tkeep  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        m_axis_rise_tstrb  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        m_axis_rise_tuser  : out std_logic_vector(0 downto 0);
        m_axis_rise_tvalid : out std_logic;
        m_axis_rise_tlast  : out std_logic;
        m_axis_rise_tready : in  std_logic := '0';

        m_axis_fall_tdata  : out std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
        m_axis_fall_tkeep  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        m_axis_fall_tstrb  : out std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0);
        m_axis_fall_tuser  : out std_logic_vector(0 downto 0);
        m_axis_fall_tvalid : out std_logic;
        m_axis_fall_tlast  : out std_logic;
        m_axis_fall_tready : in  std_logic := '0';

        o_vdma_rise_cfg_valid  : out std_logic;
        i_vdma_rise_cfg_ready  : in  std_logic := '0';
        o_vdma_rise_cfg_enable : out std_logic;
        o_vdma_rise_hsize_bytes  : out unsigned(15 downto 0);
        o_vdma_rise_vsize_lines  : out unsigned(15 downto 0);
        o_vdma_rise_stride_bytes : out unsigned(15 downto 0);
        o_vdma_fall_cfg_valid  : out std_logic;
        i_vdma_fall_cfg_ready  : in  std_logic := '0';
        o_vdma_fall_cfg_enable : out std_logic;
        o_vdma_fall_hsize_bytes  : out unsigned(15 downto 0);
        o_vdma_fall_vsize_lines  : out unsigned(15 downto 0);
        o_vdma_fall_stride_bytes : out unsigned(15 downto 0)
    );

    attribute X_INTERFACE_INFO : string;
    attribute X_INTERFACE_PARAMETER : string;
    attribute X_INTERFACE_INFO of s_axi_csr_aclk : signal is
        "xilinx.com:signal:clock:1.0 s_axi_csr_aclk CLK";
    attribute X_INTERFACE_PARAMETER of s_axi_csr_aclk : signal is
        "ASSOCIATED_BUSIF s_axi_csr, ASSOCIATED_RESET s_axi_csr_aresetn";
    attribute X_INTERFACE_INFO of proc_aclk : signal is
        "xilinx.com:signal:clock:1.0 proc_aclk CLK";
    attribute X_INTERFACE_PARAMETER of proc_aclk : signal is
        "ASSOCIATED_BUSIF m_axis_monitor:m_axis_rise:m_axis_fall, ASSOCIATED_RESET proc_aresetn";
    attribute X_INTERFACE_INFO of i_tdc_clk : signal is
        "xilinx.com:signal:clock:1.0 i_tdc_clk CLK";
    attribute X_INTERFACE_PARAMETER of i_tdc_clk : signal is
        "ASSOCIATED_RESET i_tdc_aresetn";
    attribute X_INTERFACE_INFO of o_irq : signal is
        "xilinx.com:signal:interrupt:1.0 o_irq INTERRUPT";
    attribute X_INTERFACE_PARAMETER of o_irq : signal is
        "SENSITIVITY LEVEL_HIGH";
end entity tdc_gpx_lidar_ctrl_v2_top;

architecture rtl of tdc_gpx_lidar_ctrl_v2_top is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        if G_STREAM_CLK_MODE = "SYNC" then
            result.stream_clock_mode := STREAM_CLOCK_SYNC;
        else
            result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        end if;
        result.num_chips := G_NUM_CHIPS;
        result.stops_per_chip := G_STOPS_PER_CHIP;
        result.max_returns_per_stop := G_MAX_RETURNS_PER_STOP;
        result.rise_capability_mask := G_RISE_CAPABILITY_MASK;
        result.fall_capability_mask := G_FALL_CAPABILITY_MASK;
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := G_NUM_FACES;
        result.enable_echo_receiver := G_ENABLE_ECHO_RECEIVER;
        result.enable_echo_simulation := G_ENABLE_ECHO_SIMULATION;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    signal csr_clear_status_c : std_logic;
    signal csr_soft_reset_c : std_logic;
    signal system_command_ready_c : std_logic;
    signal system_command_rejected_c : std_logic;
    signal proc_clear_status_c : std_logic;
    signal proc_soft_reset_c : std_logic;
    signal tdc_clear_status_c : std_logic;
    signal tdc_soft_reset_c : std_logic;

    signal config_busy_c : std_logic;
    signal config_done_c : std_logic;
    signal config_rejected_c : std_logic;
    signal config_reject_error_c : lidar_cfg_error_t;
    signal config_error_c : lidar_cfg_error_t;
    signal config_recovery_c : std_logic;
    signal active_valid_c : std_logic;
    signal active_config_c : lidar_active_config_t;
    signal proc_enable_c : std_logic;
    signal proc_active_valid_c : std_logic;
    signal proc_active_config_c : lidar_active_config_t;
    signal tdc_enable_c : std_logic;
    signal tdc_active_valid_c : std_logic;
    signal tdc_active_config_c : lidar_active_config_t;
    signal proc_activate_start_c : std_logic;
    signal proc_activate_complete_c : std_logic;
    signal proc_activate_fault_c : std_logic;

    signal operation_state_c : operation_state_t;
    signal processing_pipeline_idle_c : std_logic;
    signal processing_safe_c : std_logic;
    signal shot_start_event_c : shot_start_event_t;
    signal shot_result_event_c : shot_result_t;

    signal echo_profile_ready_c : std_logic;
    signal echo_profile_busy_c : std_logic;
    signal echo_profile_version_c : u16_t;
    signal echo_idle_c : std_logic;

    signal rise_active_profile_c : gpx_vdma_lane_profile_t;
    signal fall_active_profile_c : gpx_vdma_lane_profile_t;

begin

    assert fn_is_legal_clock_mhz(G_CSR_CLK_MHZ)
        report "V2-K0-TOP-001 illegal CSR clock frequency"
        severity failure;
    assert G_STREAM_CLK_MODE = "ASYNC" or G_STREAM_CLK_MODE = "SYNC"
        report "V2-K0-TOP-002 stream clock mode must be ASYNC or SYNC"
        severity failure;
    assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
        report "V2-K0-TOP-003 invalid build configuration"
        severity failure;
    assert G_OEN_MODE = "DYNAMIC_CONNECTED" or
           G_OEN_MODE = "PULLUP_OR_NOT_CONNECTED"
        report "V2-K0-TOP-004 invalid GPX OEN mode"
        severity failure;

    assert false
        report "LIDAR_V2_TOP_SHELL_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    assert false
        report "LIDAR_V2_TOP_K03_CONFIG_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    assert false
        report "LIDAR_V2_TOP_K04_PROCESSING_ECHO_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    u_csr_config : entity work.lidar_csr_config_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_CSR_CLK_MHZ => G_CSR_CLK_MHZ,
            G_PHASE_TIMEOUT_US => G_PHASE_TIMEOUT_US,
            G_PROC_DEFER_ACTIVATE_ACK => true,
            G_TDC_DEFER_ACTIVATE_ACK => false
        )
        port map (
            i_csr_clk => s_axi_csr_aclk,
            i_csr_rst_n => s_axi_csr_aresetn,
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_aresetn,
            s_axi_awaddr => s_axi_csr_awaddr,
            s_axi_awprot => s_axi_csr_awprot,
            s_axi_awvalid => s_axi_csr_awvalid,
            s_axi_awready => s_axi_csr_awready,
            s_axi_wdata => s_axi_csr_wdata,
            s_axi_wstrb => s_axi_csr_wstrb,
            s_axi_wvalid => s_axi_csr_wvalid,
            s_axi_wready => s_axi_csr_wready,
            s_axi_bresp => s_axi_csr_bresp,
            s_axi_bvalid => s_axi_csr_bvalid,
            s_axi_bready => s_axi_csr_bready,
            s_axi_araddr => s_axi_csr_araddr,
            s_axi_arprot => s_axi_csr_arprot,
            s_axi_arvalid => s_axi_csr_arvalid,
            s_axi_arready => s_axi_csr_arready,
            s_axi_rdata => s_axi_csr_rdata,
            s_axi_rresp => s_axi_csr_rresp,
            s_axi_rvalid => s_axi_csr_rvalid,
            s_axi_rready => s_axi_csr_rready,
            i_proc_safe => processing_safe_c,
            i_tdc_safe => '1',
            i_proc_soft_reset => proc_soft_reset_c,
            i_external_laser_permit => i_external_laser_permit,
            i_tdc_config_ready => '0',
            i_tdc_config_done => '0',
            i_tdc_config_fault => '0',
            i_proc_activate_complete => proc_activate_complete_c,
            i_proc_activate_fault => proc_activate_fault_c,
            i_system_command_ready => system_command_ready_c,
            i_system_command_rejected => system_command_rejected_c,
            o_irq => o_irq,
            o_clear_status => csr_clear_status_c,
            o_soft_reset_request => csr_soft_reset_c,
            o_busy => config_busy_c,
            o_done => config_done_c,
            o_commit_rejected => config_rejected_c,
            o_reject_error => config_reject_error_c,
            o_error => config_error_c,
            o_recovery_required => config_recovery_c,
            o_active_valid => active_valid_c,
            o_active => active_config_c,
            o_proc_enable => proc_enable_c,
            o_proc_active_valid => proc_active_valid_c,
            o_proc_active => proc_active_config_c,
            o_tdc_enable => tdc_enable_c,
            o_tdc_active_valid => tdc_active_valid_c,
            o_tdc_active => tdc_active_config_c,
            o_tdc_register_image => open,
            o_tdc_config_apply => open,
            o_proc_activate_start => proc_activate_start_c,
            o_prepare_req => open,
            o_activate_req => open,
            o_release_req => open,
            o_operation_state => operation_state_c,
            o_operation_command_accepted => open,
            o_operation_command_rejected => open,
            o_operation_permit_trip => open,
            o_operation_safe_to_prepare => open
        );

    u_system_command_cdc : entity work.lidar_system_command_cdc
        port map (
            i_source_clk => s_axi_csr_aclk,
            i_source_rst_n => s_axi_csr_aresetn,
            i_clear_status => csr_clear_status_c,
            i_soft_reset => csr_soft_reset_c,
            o_source_ready => system_command_ready_c,
            o_source_busy => open,
            o_source_rejected => system_command_rejected_c,
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            o_proc_clear_status => proc_clear_status_c,
            o_proc_soft_reset => proc_soft_reset_c,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_aresetn,
            o_tdc_clear_status => tdc_clear_status_c,
            o_tdc_soft_reset => tdc_soft_reset_c
        );

    processing_safe_c <= processing_pipeline_idle_c and echo_idle_c;

    u_processing : entity work.lidar_processing_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => proc_aclk,
            i_rst_n => proc_aresetn,
            i_active_valid => proc_active_valid_c,
            i_active_config => proc_active_config_c,
            i_operation_state => operation_state_c,
            i_enc_a => i_enc_a,
            i_enc_b => i_enc_b,
            i_enc_z => i_enc_z,
            i_fire_done_raw => i_fire_done,
            i_clear_diagnostics => proc_clear_status_c,
            i_face_close_ready => '1',
            o_face_close_event => open,
            o_face_close_overflow_sticky => open,
            m_mon_axis_tready => m_axis_monitor_tready,
            m_mon_axis_tvalid => m_axis_monitor_tvalid,
            m_mon_axis_tdata => m_axis_monitor_tdata,
            m_mon_axis_tkeep => m_axis_monitor_tkeep,
            m_mon_axis_tuser => m_axis_monitor_tuser,
            m_mon_axis_tlast => m_axis_monitor_tlast,
            o_fire_pulse => o_fire_pulse,
            o_start_tdc => o_start_tdc,
            o_stop_tdc => o_stop_tdc,
            o_position_event => open,
            o_face_event => open,
            o_shot_request => open,
            o_shot_start => shot_start_event_c,
            o_shot_result => shot_result_event_c,
            o_current_request => open,
            o_current_position => open,
            o_current_direction => open,
            o_executor_ready => open,
            o_request_accept => open,
            o_request_drop => open,
            o_executor_busy => open,
            o_physical_arm => open,
            o_rearm_active => open,
            o_pipeline_idle => processing_pipeline_idle_c,
            o_virtual_a => open,
            o_virtual_b => open,
            o_virtual_z => open,
            o_b0_to_accept_clks => open,
            o_physical_to_fire_clks => open,
            o_virtual_to_accept_clks => open,
            o_fire_done_sync_clks => open,
            o_rearm_margin_clks => open,
            o_diagnostics => open
        );

    u_echo : entity work.lidar_echo_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => proc_aclk,
            i_rst_n => proc_aresetn,
            i_clear_diagnostics => proc_clear_status_c,
            i_active_valid => proc_active_valid_c,
            i_active_config => proc_active_config_c,
            i_shot_start => shot_start_event_c,
            i_shot_result => shot_result_event_c,
            i_pd_lvds_p => i_pd_lvds_p,
            i_pd_lvds_n => i_pd_lvds_n,
            o_tdc_stop => o_tdc_stop,
            o_simulation_active => open,
            o_profile_ready => echo_profile_ready_c,
            o_profile_busy => echo_profile_busy_c,
            o_profile_version => echo_profile_version_c,
            o_window_active => open,
            o_idle => echo_idle_c,
            o_diagnostics => open
        );

    u_processing_activation :
        entity work.lidar_processing_activation_barrier
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => proc_aclk,
            i_rst_n => proc_aresetn,
            i_abort => proc_soft_reset_c,
            i_activate_start => proc_activate_start_c,
            i_active_valid => proc_active_valid_c,
            i_active_config => proc_active_config_c,
            i_datapath_idle => processing_pipeline_idle_c,
            i_echo_profile_ready => echo_profile_ready_c,
            i_echo_profile_busy => echo_profile_busy_c,
            i_echo_profile_version => echo_profile_version_c,
            o_rise_cfg_valid => o_vdma_rise_cfg_valid,
            i_rise_cfg_ready => i_vdma_rise_cfg_ready,
            o_rise_cfg_enable => o_vdma_rise_cfg_enable,
            o_rise_hsize_bytes => o_vdma_rise_hsize_bytes,
            o_rise_vsize_lines => o_vdma_rise_vsize_lines,
            o_rise_stride_bytes => o_vdma_rise_stride_bytes,
            o_fall_cfg_valid => o_vdma_fall_cfg_valid,
            i_fall_cfg_ready => i_vdma_fall_cfg_ready,
            o_fall_cfg_enable => o_vdma_fall_cfg_enable,
            o_fall_hsize_bytes => o_vdma_fall_hsize_bytes,
            o_fall_vsize_lines => o_vdma_fall_vsize_lines,
            o_fall_stride_bytes => o_vdma_fall_stride_bytes,
            o_rise_active_profile => rise_active_profile_c,
            o_fall_active_profile => fall_active_profile_c,
            o_activate_complete => proc_activate_complete_c,
            o_activate_fault => proc_activate_fault_c,
            o_busy => open
        );

    -- K0-4 transfers the Processing and Echo outputs to their production
    -- owners. GPX bus and payload streams remain fail-safe until K0-5..K0-7.
    o_shot_start <= shot_start_event_c.valid;
    o_shot_face_index <= std_logic_vector(
        shot_start_event_c.request.face_index);
    o_n_faces         <= std_logic_vector(to_unsigned(G_NUM_FACES, 3));

    io_tdc_d       <= (others => 'Z');
    o_tdc_adr      <= (others => '0');
    o_tdc_csn      <= (others => '1');
    o_tdc_rdn      <= (others => '1');
    o_tdc_wrn      <= (others => '1');
    o_tdc_oen      <= (others => '1');
    o_tdc_stopdis  <= (others => '1');
    o_tdc_alutrigger <= (others => '0');
    o_tdc_puresn     <= (others => '1');

    m_axis_rise_tdata  <= (others => '0');
    m_axis_rise_tkeep  <= (others => '0');
    m_axis_rise_tstrb  <= (others => '0');
    m_axis_rise_tuser  <= (others => '0');
    m_axis_rise_tvalid <= '0';
    m_axis_rise_tlast  <= '0';
    m_axis_fall_tdata  <= (others => '0');
    m_axis_fall_tkeep  <= (others => '0');
    m_axis_fall_tstrb  <= (others => '0');
    m_axis_fall_tuser  <= (others => '0');
    m_axis_fall_tvalid <= '0';
    m_axis_fall_tlast  <= '0';

end architecture rtl;
