library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_processing_pkg.all;
use work.lidar_status_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- V3 H6-B1 통합 경계. V2의 CSR/COMMIT/IRQ, 모터·레이저·Echo 운용 계약은
-- 유지하고, GPX IFIFO Drain 이후 데이터 경로만 H1~H4 HLS로 교체한다.
-- V2 Top은 Golden 기준으로 보존하며 이 파일이 V3 통합 Top을 소유한다.
entity tdc_gpx_lidar_ctrl_v3_top is
    generic (
        G_CSR_CLK_MHZ      : positive range 50 to 200 := 100;
        G_PROC_CLK_MHZ     : positive range 50 to 200 := 150;
        -- PL 내부 GPX bus/acquisition clock이다. 외부 TDC-GPX 기준 클럭은
        -- 이 Generic과 별개이며 PCB/HW에서 반드시 40 MHz를 공급한다.
        G_TDC_CLK_MHZ      : positive range 50 to 200 := 200;
        G_STREAM_CLK_MODE  : string := "ASYNC";
        -- Keep public generic limits literal so Vivado IP Packager can
        -- preserve their legal ranges without resolving package constants.
        G_NUM_CHIPS        : positive range 1 to 4 := 4;
        G_STOPS_PER_CHIP   : positive range 1 to 8 := 8;
        G_MAX_RETURNS_PER_STOP : positive range 1 to 7 := 7;
        G_RISE_CAPABILITY_MASK : std_logic_vector(3 downto 0) := "0011";
        G_FALL_CAPABILITY_MASK : std_logic_vector(3 downto 0) := "1100";
        G_OUTPUT_WIDTH     : positive := 32;
        G_NUM_FACES        : positive range 1 to 5 := 5;
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

        -- The monitor ABI is fixed at 64-bit data, 8-byte keep and 8-bit
        -- user. Literal public widths avoid unresolved package expressions in
        -- component.xml while the typed internal record keeps one definition.
        m_axis_monitor_tdata  : out std_logic_vector(63 downto 0);
        m_axis_monitor_tkeep  : out std_logic_vector(7 downto 0);
        m_axis_monitor_tuser  : out std_logic_vector(7 downto 0);
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
        m_axis_fall_tready : in  std_logic := '0'
    );

end entity tdc_gpx_lidar_ctrl_v3_top;

architecture rtl of tdc_gpx_lidar_ctrl_v3_top is

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
    signal diag_request_valid_c : std_logic;
    signal diag_request_ready_c : std_logic;
    signal diag_request_index_c : lidar_diag_index_t;
    signal diag_response_valid_c : std_logic;
    signal diag_response_ready_c : std_logic;
    signal diag_response_c : lidar_diag_response_t;
    signal runtime_irq_c : lidar_runtime_irq_t;
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
    signal tdc_register_image_c : gpx_register_image_t;
    signal tdc_config_apply_c : std_logic;
    signal tdc_config_ready_c : std_logic;
    signal tdc_config_done_c : std_logic;
    signal proc_activate_start_c : std_logic;
    signal proc_activate_complete_c : std_logic;
    signal proc_activate_fault_c : std_logic;

    signal operation_state_c : operation_state_t;
    signal processing_pipeline_idle_c : std_logic;
    signal processing_safe_c : std_logic;
    signal stop_tdc_c : std_logic;
    signal shot_start_event_c : shot_start_event_t;
    signal shot_result_event_c : shot_result_t;
    signal gpx_proc_shot_ready_c : std_logic;
    signal face_close_event_c : face_close_event_t;
    signal face_close_ready_c : std_logic;
    signal face_close_overflow_sticky_c : std_logic;
    signal processing_diagnostics_c : processing_diagnostics_t;
    signal b0_to_accept_clks_c : processing_latency_t;
    signal physical_to_fire_clks_c : processing_latency_t;
    signal virtual_to_accept_clks_c : processing_latency_t;
    signal fire_done_sync_clks_c : unsigned(15 downto 0);
    signal rearm_margin_clks_c : unsigned(15 downto 0);

    signal echo_profile_ready_c : std_logic;
    signal echo_profile_busy_c : std_logic;
    signal echo_profile_version_c : u16_t;
    signal echo_idle_c : std_logic;
    signal echo_diagnostics_c : echo_diagnostics_t;

    signal rise_active_profile_c : gpx_vdma_lane_profile_t;
    signal fall_active_profile_c : gpx_vdma_lane_profile_t;

    -- The processing-domain profile request is transported atomically into
    -- the unified CSR. Software reads CTL25..29, programs the external VDMA,
    -- and writes the matching ACK bit in CTL25.
    signal rise_cfg_valid_proc_c : std_logic;
    signal rise_cfg_ready_proc_c : std_logic;
    signal rise_cfg_enable_proc_c : std_logic;
    signal rise_hsize_proc_c : unsigned(15 downto 0);
    signal rise_vsize_proc_c : unsigned(15 downto 0);
    signal rise_stride_proc_c : unsigned(15 downto 0);
    signal fall_cfg_valid_proc_c : std_logic;
    signal fall_cfg_ready_proc_c : std_logic;
    signal fall_cfg_enable_proc_c : std_logic;
    signal fall_hsize_proc_c : unsigned(15 downto 0);
    signal fall_vsize_proc_c : unsigned(15 downto 0);
    signal fall_stride_proc_c : unsigned(15 downto 0);

    signal rise_cfg_valid_csr_c : std_logic;
    signal rise_cfg_enable_csr_c : std_logic;
    signal rise_hsize_csr_c : unsigned(15 downto 0);
    signal rise_vsize_csr_c : unsigned(15 downto 0);
    signal rise_stride_csr_c : unsigned(15 downto 0);
    signal rise_cfg_ack_csr_c : std_logic;
    signal fall_cfg_valid_csr_c : std_logic;
    signal fall_cfg_enable_csr_c : std_logic;
    signal fall_hsize_csr_c : unsigned(15 downto 0);
    signal fall_vsize_csr_c : unsigned(15 downto 0);
    signal fall_stride_csr_c : unsigned(15 downto 0);
    signal fall_cfg_ack_csr_c : std_logic;

    signal tdc_run_sync_c : std_logic;
    signal tdc_run_enable_c : std_logic;
    signal gpx_proc_idle_c : std_logic;
    signal gpx_tdc_safe_c : std_logic;
    signal gpx_cdc_reset_busy_c : std_logic;
    signal gpx_axis_idle_c : std_logic;
    signal gpx_context_fault_sticky_c : std_logic;
    signal gpx_active_mask_c : chip_mask_t;
    signal gpx_terminal_mask_c : chip_mask_t;
    signal gpx_tdc_status_c : gpx_lane_status_array_t;
    signal gpx_tdc_faults_c : gpx_lane_faults_array_t;
    signal gpx_register_service_pause_c : std_logic;
    signal gpx_register_read_c : gpx_register_read_request_t :=
        C_GPX_REGISTER_READ_REQUEST_IDLE;
    signal gpx_register_read_ready_c : std_logic;
    signal gpx_register_read_response_c : gpx_register_read_response_t :=
        C_GPX_REGISTER_READ_RESPONSE_IDLE;
    signal gpx_register_read_response_ready_c : std_logic;
    signal gpx_shot_drop_sticky_c : std_logic;
    signal gpx_stop_drop_sticky_c : std_logic;
    signal gpx_hit_fault_sticky_c : gpx_hit_decoder_faults_t;
    signal gpx_cell_fault_sticky_c : gpx_cell_collector_faults_t;
    signal gpx_frame_fault_sticky_c : gpx_frame_assembler_faults_t;
    signal gpx_rise_formatter_fault_sticky_c :
        lidar_gpx_word_formatter_faults_t;
    signal gpx_fall_formatter_fault_sticky_c :
        lidar_gpx_word_formatter_faults_t;

    signal gpx_adr_c : gpx_bus_address_array_t;
    signal gpx_csn_c : chip_mask_t;
    signal gpx_rdn_c : chip_mask_t;
    signal gpx_wrn_c : chip_mask_t;
    signal gpx_oen_c : chip_mask_t;
    signal gpx_d_in_c : gpx_bus_data_array_t;
    signal gpx_d_out_c : gpx_bus_data_array_t;
    signal gpx_d_tri_c : gpx_bus_data_array_t;
    signal gpx_ef1_c : chip_mask_t;
    signal gpx_ef2_c : chip_mask_t;
    signal gpx_lf1_c : chip_mask_t;
    signal gpx_lf2_c : chip_mask_t;
    signal gpx_irflag_c : chip_mask_t;
    signal gpx_errflag_c : chip_mask_t;
    signal gpx_stopdis_c : chip_mask_t;
    signal gpx_alutrigger_c : chip_mask_t;
    signal gpx_puresn_c : chip_mask_t;

begin

    assert fn_is_legal_clock_mhz(G_CSR_CLK_MHZ)
        report "V3-H6B-TOP-001 illegal CSR clock frequency"
        severity failure;
    assert G_STREAM_CLK_MODE = "ASYNC" or G_STREAM_CLK_MODE = "SYNC"
        report "V3-H6B-TOP-002 stream clock mode must be ASYNC or SYNC"
        severity failure;
    assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
        report "V3-H6B-TOP-003 invalid build configuration"
        severity failure;
    assert G_OEN_MODE = "DYNAMIC_CONNECTED" or
           G_OEN_MODE = "PULLUP_OR_NOT_CONNECTED"
        report "V3-H6B-TOP-004 invalid GPX OEN mode"
        severity failure;

    assert false
        report "LIDAR_V3_TOP_SHELL_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    assert false
        report "LIDAR_V3_TOP_CONFIG_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    assert false
        report "LIDAR_V3_TOP_PROCESSING_ECHO_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
        severity note;

    u_csr_config : entity work.lidar_csr_config_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_CSR_CLK_MHZ => G_CSR_CLK_MHZ,
            G_PHASE_TIMEOUT_US => G_PHASE_TIMEOUT_US,
            G_PROC_DEFER_ACTIVATE_ACK => true,
            G_TDC_DEFER_ACTIVATE_ACK => true
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
            i_tdc_safe => gpx_tdc_safe_c,
            i_proc_soft_reset => proc_soft_reset_c,
            i_external_laser_permit => i_external_laser_permit,
            i_tdc_config_ready => tdc_config_ready_c,
            i_tdc_config_done => tdc_config_done_c,
            i_tdc_config_fault => '0',
            i_proc_activate_complete => proc_activate_complete_c,
            i_proc_activate_fault => proc_activate_fault_c,
            i_system_command_ready => system_command_ready_c,
            i_system_command_rejected => system_command_rejected_c,
            i_runtime_irq => runtime_irq_c,
            i_vdma_rise_cfg_valid => rise_cfg_valid_csr_c,
            i_vdma_rise_cfg_enable => rise_cfg_enable_csr_c,
            i_vdma_rise_hsize_bytes => rise_hsize_csr_c,
            i_vdma_rise_vsize_lines => rise_vsize_csr_c,
            i_vdma_rise_stride_bytes => rise_stride_csr_c,
            o_vdma_rise_cfg_ack => rise_cfg_ack_csr_c,
            i_vdma_fall_cfg_valid => fall_cfg_valid_csr_c,
            i_vdma_fall_cfg_enable => fall_cfg_enable_csr_c,
            i_vdma_fall_hsize_bytes => fall_hsize_csr_c,
            i_vdma_fall_vsize_lines => fall_vsize_csr_c,
            i_vdma_fall_stride_bytes => fall_stride_csr_c,
            o_vdma_fall_cfg_ack => fall_cfg_ack_csr_c,
            o_diag_request_valid => diag_request_valid_c,
            i_diag_request_ready => diag_request_ready_c,
            o_diag_request_index => diag_request_index_c,
            i_diag_response_valid => diag_response_valid_c,
            o_diag_response_ready => diag_response_ready_c,
            i_diag_response => diag_response_c,
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
            o_tdc_register_image => tdc_register_image_c,
            o_tdc_config_apply => tdc_config_apply_c,
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

    u_rise_vdma_profile_cdc : entity work.lidar_vdma_profile_cdc
        port map (
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            i_proc_cfg_valid => rise_cfg_valid_proc_c,
            i_proc_cfg_enable => rise_cfg_enable_proc_c,
            i_proc_hsize_bytes => rise_hsize_proc_c,
            i_proc_vsize_lines => rise_vsize_proc_c,
            i_proc_stride_bytes => rise_stride_proc_c,
            o_proc_cfg_ready => rise_cfg_ready_proc_c,
            i_csr_clk => s_axi_csr_aclk,
            i_csr_rst_n => s_axi_csr_aresetn,
            o_csr_cfg_valid => rise_cfg_valid_csr_c,
            o_csr_cfg_enable => rise_cfg_enable_csr_c,
            o_csr_hsize_bytes => rise_hsize_csr_c,
            o_csr_vsize_lines => rise_vsize_csr_c,
            o_csr_stride_bytes => rise_stride_csr_c,
            i_csr_cfg_ack => rise_cfg_ack_csr_c
        );

    u_fall_vdma_profile_cdc : entity work.lidar_vdma_profile_cdc
        port map (
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            i_proc_cfg_valid => fall_cfg_valid_proc_c,
            i_proc_cfg_enable => fall_cfg_enable_proc_c,
            i_proc_hsize_bytes => fall_hsize_proc_c,
            i_proc_vsize_lines => fall_vsize_proc_c,
            i_proc_stride_bytes => fall_stride_proc_c,
            o_proc_cfg_ready => fall_cfg_ready_proc_c,
            i_csr_clk => s_axi_csr_aclk,
            i_csr_rst_n => s_axi_csr_aresetn,
            o_csr_cfg_valid => fall_cfg_valid_csr_c,
            o_csr_cfg_enable => fall_cfg_enable_csr_c,
            o_csr_hsize_bytes => fall_hsize_csr_c,
            o_csr_vsize_lines => fall_vsize_csr_c,
            o_csr_stride_bytes => fall_stride_csr_c,
            i_csr_cfg_ack => fall_cfg_ack_csr_c
        );

    processing_safe_c <= processing_pipeline_idle_c and echo_idle_c and
        gpx_proc_idle_c and gpx_axis_idle_c and
        not gpx_cdc_reset_busy_c;

    u_status_snapshot : entity work.lidar_v3_status_snapshot_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_csr_clk => s_axi_csr_aclk,
            i_csr_rst_n => s_axi_csr_aresetn,
            i_csr_request_valid => diag_request_valid_c,
            o_csr_request_ready => diag_request_ready_c,
            i_csr_request_index => diag_request_index_c,
            o_csr_response_valid => diag_response_valid_c,
            i_csr_response_ready => diag_response_ready_c,
            o_csr_response => diag_response_c,
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            i_proc_clear_status => proc_clear_status_c,
            i_processing_diagnostics => processing_diagnostics_c,
            i_echo_diagnostics => echo_diagnostics_c,
            i_face_close_overflow_sticky =>
                face_close_overflow_sticky_c,
            i_pipeline_idle => processing_pipeline_idle_c,
            i_echo_idle => echo_idle_c,
            i_gpx_proc_idle => gpx_proc_idle_c,
            i_gpx_axis_idle => gpx_axis_idle_c,
            i_echo_profile_ready => echo_profile_ready_c,
            i_echo_profile_busy => echo_profile_busy_c,
            i_echo_profile_version => echo_profile_version_c,
            i_rise_profile => rise_active_profile_c,
            i_fall_profile => fall_active_profile_c,
            i_gpx_context_fault_sticky => gpx_context_fault_sticky_c,
            i_gpx_shot_drop_sticky => gpx_shot_drop_sticky_c,
            i_gpx_stop_drop_sticky => gpx_stop_drop_sticky_c,
            i_gpx_cdc_reset_busy => gpx_cdc_reset_busy_c,
            i_hit_fault_sticky => gpx_hit_fault_sticky_c,
            i_cell_fault_sticky => gpx_cell_fault_sticky_c,
            i_frame_fault_sticky => gpx_frame_fault_sticky_c,
            i_rise_formatter_fault_sticky =>
                gpx_rise_formatter_fault_sticky_c,
            i_fall_formatter_fault_sticky =>
                gpx_fall_formatter_fault_sticky_c,
            i_b0_to_accept_clks => b0_to_accept_clks_c,
            i_physical_to_fire_clks => physical_to_fire_clks_c,
            i_virtual_to_accept_clks => virtual_to_accept_clks_c,
            i_fire_done_sync_clks => fire_done_sync_clks_c,
            i_rearm_margin_clks => rearm_margin_clks_c,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_aresetn,
            i_tdc_clear_status => tdc_clear_status_c,
            i_active_mask => gpx_active_mask_c,
            i_terminal_mask => gpx_terminal_mask_c,
            i_lane_status => gpx_tdc_status_c,
            i_lane_faults => gpx_tdc_faults_c,
            i_tdc_safe => gpx_tdc_safe_c,
            i_tdc_run_enable => tdc_run_enable_c,
            i_tdc_active_valid => tdc_active_valid_c,
            i_tdc_config_ready => tdc_config_ready_c,
            o_tdc_register_service_pause =>
                gpx_register_service_pause_c,
            o_tdc_register_read => gpx_register_read_c,
            i_tdc_register_read_ready => gpx_register_read_ready_c,
            i_tdc_register_read_response =>
                gpx_register_read_response_c,
            o_tdc_register_read_response_ready =>
                gpx_register_read_response_ready_c,
            o_runtime_irq => runtime_irq_c
        );

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
            i_acquisition_ready => gpx_proc_shot_ready_c,
            i_face_close_ready => face_close_ready_c,
            o_face_close_event => face_close_event_c,
            o_face_close_overflow_sticky =>
                face_close_overflow_sticky_c,
            m_mon_axis_tready => m_axis_monitor_tready,
            m_mon_axis_tvalid => m_axis_monitor_tvalid,
            m_mon_axis_tdata => m_axis_monitor_tdata,
            m_mon_axis_tkeep => m_axis_monitor_tkeep,
            m_mon_axis_tuser => m_axis_monitor_tuser,
            m_mon_axis_tlast => m_axis_monitor_tlast,
            o_fire_pulse => o_fire_pulse,
            o_start_tdc => o_start_tdc,
            o_stop_tdc => stop_tdc_c,
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
            o_b0_to_accept_clks => b0_to_accept_clks_c,
            o_physical_to_fire_clks => physical_to_fire_clks_c,
            o_virtual_to_accept_clks => virtual_to_accept_clks_c,
            o_fire_done_sync_clks => fire_done_sync_clks_c,
            o_rearm_margin_clks => rearm_margin_clks_c,
            o_diagnostics => processing_diagnostics_c
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
            o_diagnostics => echo_diagnostics_c
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
            i_datapath_idle => processing_pipeline_idle_c and
                gpx_proc_idle_c and gpx_axis_idle_c and
                not gpx_cdc_reset_busy_c,
            i_echo_profile_ready => echo_profile_ready_c,
            i_echo_profile_busy => echo_profile_busy_c,
            i_echo_profile_version => echo_profile_version_c,
            o_rise_cfg_valid => rise_cfg_valid_proc_c,
            i_rise_cfg_ready => rise_cfg_ready_proc_c,
            o_rise_cfg_enable => rise_cfg_enable_proc_c,
            o_rise_hsize_bytes => rise_hsize_proc_c,
            o_rise_vsize_lines => rise_vsize_proc_c,
            o_rise_stride_bytes => rise_stride_proc_c,
            o_fall_cfg_valid => fall_cfg_valid_proc_c,
            i_fall_cfg_ready => fall_cfg_ready_proc_c,
            o_fall_cfg_enable => fall_cfg_enable_proc_c,
            o_fall_hsize_bytes => fall_hsize_proc_c,
            o_fall_vsize_lines => fall_vsize_proc_c,
            o_fall_stride_bytes => fall_stride_proc_c,
            o_rise_active_profile => rise_active_profile_c,
            o_fall_active_profile => fall_active_profile_c,
            o_activate_complete => proc_activate_complete_c,
            o_activate_fault => proc_activate_fault_c,
            o_busy => open
        );

    u_gpx_run_cdc : entity work.lidar_gpx_run_enable_cdc
        port map (
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_aresetn,
            i_proc_run_enable => operation_state_c.running,
            o_tdc_run_enable => tdc_run_sync_c
        );

    -- 물리 Register 진단은 DISARM 상태에서 acquisition만 잠시 정지한다.
    -- 완료 후 RUN 상태는 보존되며, ARM 전까지 새 Shot은 생성되지 않는다.
    tdc_run_enable_c <= tdc_enable_c and tdc_run_sync_c and
        not gpx_register_service_pause_c;

    -- V2의 acquisition(B5~B8)과 별도 AXIS output 두 계층을 H6-A의
    -- 단일 Parent 데이터 서브시스템으로 교체한다. CSR/COMMIT이 만든
    -- Processing/TDC Active 설정과 VDMA Profile은 기존 소유 경계를
    -- 그대로 사용하며, Face 완료는 마지막 Footer Beat 승인까지 기다린다.
    u_gpx_hls_parent_data : entity work.lidar_gpx_hls_parent_data_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_OEN_MODE => G_OEN_MODE,
            G_POWERUP_TIME_NS => G_POWERUP_TIME_NS,
            G_RECOVERY_TIME_NS => G_RECOVERY_TIME_NS,
            G_ALU_PULSE_TIME_NS => G_ALU_PULSE_TIME_NS,
            G_BUS_IDLE_STABLE_TIME_NS => G_BUS_IDLE_STABLE_TIME_NS,
            G_DRAIN_MARGIN_TIME_NS => G_DRAIN_MARGIN_TIME_NS
        )
        port map (
            i_proc_clk => proc_aclk,
            i_proc_rst_n => proc_aresetn,
            i_proc_abort => proc_soft_reset_c,
            i_proc_clear_status => proc_clear_status_c,
            i_proc_active_valid => proc_active_valid_c,
            i_proc_active_config => proc_active_config_c,
            i_proc_shot => shot_start_event_c,
            o_proc_shot_ready => gpx_proc_shot_ready_c,
            i_proc_stop_tdc => stop_tdc_c,
            i_face_close_event => face_close_event_c,
            o_face_close_ready => face_close_ready_c,
            i_rise_active_profile => rise_active_profile_c,
            i_fall_active_profile => fall_active_profile_c,
            o_rise_tdata => m_axis_rise_tdata,
            o_rise_tkeep => m_axis_rise_tkeep,
            o_rise_tstrb => m_axis_rise_tstrb,
            o_rise_tuser => m_axis_rise_tuser,
            o_rise_tvalid => m_axis_rise_tvalid,
            o_rise_tlast => m_axis_rise_tlast,
            i_rise_tready => m_axis_rise_tready,
            o_fall_tdata => m_axis_fall_tdata,
            o_fall_tkeep => m_axis_fall_tkeep,
            o_fall_tstrb => m_axis_fall_tstrb,
            o_fall_tuser => m_axis_fall_tuser,
            o_fall_tvalid => m_axis_fall_tvalid,
            o_fall_tlast => m_axis_fall_tlast,
            i_fall_tready => m_axis_fall_tready,
            o_shot_done => open,
            o_shot_done_context => open,
            o_frame_output_done => open,
            o_processing_idle => gpx_proc_idle_c,
            o_axis_output_idle => gpx_axis_idle_c,
            o_proc_idle => open,
            o_outstanding_shots => open,
            o_decoder_inflight => open,
            o_rise_emitted_lines => open,
            o_fall_emitted_lines => open,
            o_context_fault_sticky => gpx_context_fault_sticky_c,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_aresetn,
            i_tdc_active_valid => tdc_active_valid_c,
            i_tdc_active_config => tdc_active_config_c,
            i_tdc_register_image => tdc_register_image_c,
            i_tdc_config_apply => tdc_config_apply_c,
            o_tdc_config_ready => tdc_config_ready_c,
            o_tdc_config_done => tdc_config_done_c,
            i_tdc_run_enable => tdc_run_enable_c,
            i_tdc_soft_reset => tdc_soft_reset_c,
            i_tdc_force_reinit => '0',
            i_tdc_clear_status => tdc_clear_status_c,
            o_tdc_safe => gpx_tdc_safe_c,
            o_tdc_shot_complete => open,
            i_tdc_register_read => gpx_register_read_c,
            o_tdc_register_read_ready => gpx_register_read_ready_c,
            o_tdc_register_read_response =>
                gpx_register_read_response_c,
            i_tdc_register_read_response_ready =>
                gpx_register_read_response_ready_c,
            o_cdc_reset_busy => gpx_cdc_reset_busy_c,
            o_adr => gpx_adr_c,
            o_csn => gpx_csn_c,
            o_rdn => gpx_rdn_c,
            o_wrn => gpx_wrn_c,
            o_oen => gpx_oen_c,
            i_d => gpx_d_in_c,
            o_d => gpx_d_out_c,
            o_d_tri => gpx_d_tri_c,
            i_ef1 => gpx_ef1_c,
            i_ef2 => gpx_ef2_c,
            i_lf1 => gpx_lf1_c,
            i_lf2 => gpx_lf2_c,
            i_irflag => gpx_irflag_c,
            i_errflag => gpx_errflag_c,
            o_stopdis => gpx_stopdis_c,
            o_alutrigger => gpx_alutrigger_c,
            o_puresn => gpx_puresn_c,
            o_active_mask => gpx_active_mask_c,
            o_terminal_mask => gpx_terminal_mask_c,
            o_tdc_status => gpx_tdc_status_c,
            o_tdc_faults => gpx_tdc_faults_c,
            o_shot_drop_sticky => gpx_shot_drop_sticky_c,
            o_stop_drop_sticky => gpx_stop_drop_sticky_c,
            o_hit_fault_sticky => gpx_hit_fault_sticky_c,
            o_cell_fault_sticky => gpx_cell_fault_sticky_c,
            o_frame_fault_sticky => gpx_frame_fault_sticky_c,
            o_rise_formatter_fault_sticky =>
                gpx_rise_formatter_fault_sticky_c,
            o_fall_formatter_fault_sticky =>
                gpx_fall_formatter_fault_sticky_c
        );

    gen_gpx_active_pins : for index in 0 to G_NUM_CHIPS - 1 generate
        constant C_DATA_LO : natural := index * 28;
        constant C_ADR_LO : natural := index * 4;
        constant C_ADR_HI : natural := (index + 1) * 4 - 1;
    begin
        -- Packaged-IP 경계에서도 양방향 GPX 데이터 핀의 I/O/T 경로가
        -- 보존되도록 보드 검증된 v1과 같은 명시적 IOBUF를 사용한다.
        -- 공개 io_tdc_d 포트와 논리 chip 순서는 바뀌지 않는다.
        gen_gpx_data_iobuf : for bit_index in
                0 to C_GPX_BUS_DATA_WIDTH - 1 generate
            constant C_DATA_INDEX : natural := C_DATA_LO + bit_index;
        begin
            u_gpx_data_iobuf : IOBUF
                port map (
                    IO => io_tdc_d(C_DATA_INDEX),
                    I  => gpx_d_out_c(index)(bit_index),
                    O  => gpx_d_in_c(index)(bit_index),
                    T  => gpx_d_tri_c(index)(bit_index)
                );
        end generate gen_gpx_data_iobuf;

        o_tdc_adr(C_ADR_HI downto C_ADR_LO) <= gpx_adr_c(index);
        gpx_ef1_c(index) <= i_tdc_ef1(index);
        gpx_ef2_c(index) <= i_tdc_ef2(index);
        gpx_lf1_c(index) <= i_tdc_lf1(index);
        gpx_lf2_c(index) <= i_tdc_lf2(index);
        gpx_irflag_c(index) <= i_tdc_irflag(index);
        gpx_errflag_c(index) <= i_tdc_errflag(index);
    end generate gen_gpx_active_pins;

    gen_gpx_unused_pins : for index in G_NUM_CHIPS to C_MAX_CHIPS - 1 generate
    begin
        gpx_d_in_c(index) <= (others => '0');
        gpx_ef1_c(index) <= '1';
        gpx_ef2_c(index) <= '1';
        gpx_lf1_c(index) <= '0';
        gpx_lf2_c(index) <= '0';
        gpx_irflag_c(index) <= '0';
        gpx_errflag_c(index) <= '0';
    end generate gen_gpx_unused_pins;

    -- H1~H4가 GPX Raw28을 canonical 32-bit Shot/Cell/Hole/Footer Word로
    -- 변환하고, 폭 공통 RTL packer가 외부 Rise/Fall AXI 32/64-bit Beat로 묶는다.
    o_shot_start <= shot_start_event_c.valid;
    o_shot_face_index <= std_logic_vector(
        shot_start_event_c.request.face_index);
    o_n_faces         <= std_logic_vector(to_unsigned(G_NUM_FACES, 3));
    o_stop_tdc <= stop_tdc_c;

    o_tdc_csn <= gpx_csn_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_rdn <= gpx_rdn_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_wrn <= gpx_wrn_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_oen <= gpx_oen_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_stopdis <= gpx_stopdis_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_alutrigger <= gpx_alutrigger_c(G_NUM_CHIPS - 1 downto 0);
    o_tdc_puresn <= gpx_puresn_c(G_NUM_CHIPS - 1 downto 0);

end architecture rtl;
