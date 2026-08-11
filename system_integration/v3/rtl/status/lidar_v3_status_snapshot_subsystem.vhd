library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_status_pkg.all;

-- V3 H6-B1 Runtime 상태 단일 소유자. CSR의 8-bit 진단 index 요청과
-- 33-bit {error,data} 응답만 clock domain 경계를 원자적으로 건넌다.
-- Processing/TDC의 큰 진단 record를 CSR clock에서 Bit별로 직접 읽지 않는다.
entity lidar_v3_status_snapshot_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_csr_clk   : in  std_logic;
        i_csr_rst_n : in  std_logic;
        i_csr_request_valid : in  std_logic;
        o_csr_request_ready : out std_logic;
        i_csr_request_index : in  lidar_diag_index_t;
        o_csr_response_valid : out std_logic;
        i_csr_response_ready : in  std_logic;
        o_csr_response       : out lidar_diag_response_t;

        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        i_proc_clear_status : in  std_logic;
        i_processing_diagnostics : in processing_diagnostics_t;
        i_echo_diagnostics       : in echo_diagnostics_t;
        i_face_close_overflow_sticky : in std_logic;
        i_pipeline_idle     : in std_logic;
        i_echo_idle         : in std_logic;
        i_gpx_proc_idle     : in std_logic;
        i_gpx_axis_idle     : in std_logic;
        i_echo_profile_ready   : in std_logic;
        i_echo_profile_busy    : in std_logic;
        i_echo_profile_version : in unsigned(15 downto 0);
        i_rise_profile : in gpx_vdma_lane_profile_t;
        i_fall_profile : in gpx_vdma_lane_profile_t;
        i_gpx_context_fault_sticky : in std_logic;
        i_gpx_shot_drop_sticky : in std_logic;
        i_gpx_stop_drop_sticky : in std_logic;
        i_gpx_cdc_reset_busy   : in std_logic;
        i_hit_fault_sticky   : in gpx_hit_decoder_faults_t;
        i_cell_fault_sticky  : in gpx_cell_collector_faults_t;
        i_frame_fault_sticky : in gpx_frame_assembler_faults_t;
        i_rise_formatter_fault_sticky : in std_logic_vector(7 downto 0);
        i_fall_formatter_fault_sticky : in std_logic_vector(7 downto 0);
        i_b0_to_accept_clks      : in processing_latency_t;
        i_physical_to_fire_clks  : in processing_latency_t;
        i_virtual_to_accept_clks : in processing_latency_t;
        i_fire_done_sync_clks    : in unsigned(15 downto 0);
        i_rearm_margin_clks      : in unsigned(15 downto 0);

        i_tdc_clk          : in  std_logic;
        i_tdc_rst_n        : in  std_logic;
        i_tdc_clear_status : in  std_logic;
        i_active_mask      : in chip_mask_t;
        i_terminal_mask    : in chip_mask_t;
        i_lane_status      : in gpx_lane_status_array_t;
        i_lane_faults      : in gpx_lane_faults_array_t;
        i_tdc_safe         : in std_logic;
        i_tdc_run_enable   : in std_logic;
        i_tdc_active_valid : in std_logic;
        i_tdc_config_ready : in std_logic;
        o_tdc_register_service_pause : out std_logic;
        o_tdc_register_read : out gpx_register_read_request_t;
        i_tdc_register_read_ready : in std_logic := '0';
        i_tdc_register_read_response : in gpx_register_read_response_t :=
            C_GPX_REGISTER_READ_RESPONSE_IDLE;
        o_tdc_register_read_response_ready : out std_logic;

        o_runtime_irq : out lidar_runtime_irq_t
    );
end entity lidar_v3_status_snapshot_subsystem;

architecture rtl of lidar_v3_status_snapshot_subsystem is

    type owner_t is (OWNER_NONE, OWNER_PROCESSING, OWNER_TDC, OWNER_LOCAL);
    signal owner_r : owner_t := OWNER_NONE;
    signal local_response_valid_r : std_logic := '0';
    signal local_response_r : lidar_diag_response_t := (others => '0');

    signal proc_request_source_valid : std_logic;
    signal proc_request_source_ready : std_logic;
    signal proc_request_dest_valid : std_logic;
    signal proc_request_dest_ready : std_logic;
    signal proc_request_data : lidar_diag_index_t;
    signal proc_response_source_valid : std_logic;
    signal proc_response_source_ready : std_logic;
    signal proc_response_source : lidar_diag_response_t;
    signal proc_response_dest_valid : std_logic;
    signal proc_response_dest_ready : std_logic;
    signal proc_response_dest : lidar_diag_response_t;

    signal tdc_request_source_valid : std_logic;
    signal tdc_request_source_ready : std_logic;
    signal tdc_request_dest_valid : std_logic;
    signal tdc_request_dest_ready : std_logic;
    signal tdc_request_data : lidar_diag_index_t;
    signal tdc_response_source_valid : std_logic;
    signal tdc_response_source_ready : std_logic;
    signal tdc_response_source : lidar_diag_response_t;
    signal tdc_response_dest_valid : std_logic;
    signal tdc_response_dest_ready : std_logic;
    signal tdc_response_dest : lidar_diag_response_t;

    signal irq_processing_c : std_logic;
    signal irq_laser_timeout_c : std_logic;
    signal irq_echo_c : std_logic;
    signal irq_gpx_transport_proc_c : std_logic;
    signal irq_gpx_transport_c : std_logic;
    signal irq_gpx_data_c : std_logic;

    signal proc_irq_native_c : std_logic_vector(4 downto 0);
    signal proc_irq_meta_r   : std_logic_vector(4 downto 0) :=
        (others => '0');
    signal proc_irq_sync_r   : std_logic_vector(4 downto 0) :=
        (others => '0');
    signal tdc_irq_meta_r    : std_logic := '0';
    signal tdc_irq_sync_r    : std_logic := '0';

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of proc_irq_meta_r : signal is "TRUE";
    attribute ASYNC_REG of proc_irq_sync_r : signal is "TRUE";
    attribute ASYNC_REG of tdc_irq_meta_r  : signal is "TRUE";
    attribute ASYNC_REG of tdc_irq_sync_r  : signal is "TRUE";

begin

    proc_request_source_valid <= i_csr_request_valid when
        owner_r = OWNER_NONE and
        fn_diag_is_processing(i_csr_request_index) else '0';
    tdc_request_source_valid <= i_csr_request_valid when
        owner_r = OWNER_NONE and
        fn_diag_is_tdc(i_csr_request_index) else '0';

    p_csr_mux : process (
        owner_r,
        i_csr_request_index,
        proc_request_source_ready,
        tdc_request_source_ready,
        local_response_valid_r,
        local_response_r,
        proc_response_dest_valid,
        proc_response_dest,
        tdc_response_dest_valid,
        tdc_response_dest
    )
    begin
        o_csr_request_ready <= '0';
        o_csr_response_valid <= '0';
        o_csr_response <= (others => '0');

        if owner_r = OWNER_NONE then
            if fn_diag_is_processing(i_csr_request_index) then
                o_csr_request_ready <= proc_request_source_ready;
            elsif fn_diag_is_tdc(i_csr_request_index) then
                o_csr_request_ready <= tdc_request_source_ready;
            else
                o_csr_request_ready <= '1';
            end if;
        elsif owner_r = OWNER_PROCESSING then
            o_csr_response_valid <= proc_response_dest_valid;
            o_csr_response <= proc_response_dest;
        elsif owner_r = OWNER_TDC then
            o_csr_response_valid <= tdc_response_dest_valid;
            o_csr_response <= tdc_response_dest;
        else
            o_csr_response_valid <= local_response_valid_r;
            o_csr_response <= local_response_r;
        end if;
    end process p_csr_mux;

    proc_response_dest_ready <= i_csr_response_ready when
        owner_r = OWNER_PROCESSING else '0';
    tdc_response_dest_ready <= i_csr_response_ready when
        owner_r = OWNER_TDC else '0';

    p_csr_owner : process (i_csr_clk, i_csr_rst_n)
    begin
        if i_csr_rst_n = '0' then
            owner_r <= OWNER_NONE;
            local_response_valid_r <= '0';
            local_response_r <= (others => '0');
        elsif rising_edge(i_csr_clk) then
            if owner_r = OWNER_NONE and i_csr_request_valid = '1' and
                    o_csr_request_ready = '1' then
                if fn_diag_is_processing(i_csr_request_index) then
                    owner_r <= OWNER_PROCESSING;
                elsif fn_diag_is_tdc(i_csr_request_index) then
                    owner_r <= OWNER_TDC;
                else
                    owner_r <= OWNER_LOCAL;
                    local_response_r <= fn_pack_diag_response(
                        (others => '0'), '1');
                    local_response_valid_r <= '1';
                end if;
            elsif o_csr_response_valid = '1' and
                    i_csr_response_ready = '1' then
                owner_r <= OWNER_NONE;
                local_response_valid_r <= '0';
            end if;
        end if;
    end process p_csr_owner;

    u_proc_mailbox : entity work.lidar_diag_snapshot_mailbox
        port map (
            i_source_clk => i_csr_clk,
            i_source_rst_n => i_csr_rst_n,
            i_source_request_valid => proc_request_source_valid,
            o_source_request_ready => proc_request_source_ready,
            i_source_request => i_csr_request_index,
            o_source_response_valid => proc_response_dest_valid,
            i_source_response_ready => proc_response_dest_ready,
            o_source_response => proc_response_dest,
            i_domain_clk => i_proc_clk,
            i_domain_rst_n => i_proc_rst_n,
            o_domain_request_valid => proc_request_dest_valid,
            i_domain_request_ready => proc_request_dest_ready,
            o_domain_request => proc_request_data,
            i_domain_response_valid => proc_response_source_valid,
            o_domain_response_ready => proc_response_source_ready,
            i_domain_response => proc_response_source
        );

    u_proc_status : entity work.lidar_v3_processing_status_source
        port map (
            i_clk => i_proc_clk,
            i_rst_n => i_proc_rst_n,
            i_clear_status => i_proc_clear_status,
            i_request_valid => proc_request_dest_valid,
            o_request_ready => proc_request_dest_ready,
            i_request_index => proc_request_data,
            o_response_valid => proc_response_source_valid,
            i_response_ready => proc_response_source_ready,
            o_response => proc_response_source,
            i_processing_diagnostics => i_processing_diagnostics,
            i_echo_diagnostics => i_echo_diagnostics,
            i_face_close_overflow_sticky =>
                i_face_close_overflow_sticky,
            i_pipeline_idle => i_pipeline_idle,
            i_echo_idle => i_echo_idle,
            i_gpx_proc_idle => i_gpx_proc_idle,
            i_gpx_axis_idle => i_gpx_axis_idle,
            i_echo_profile_ready => i_echo_profile_ready,
            i_echo_profile_busy => i_echo_profile_busy,
            i_echo_profile_version => i_echo_profile_version,
            i_rise_profile => i_rise_profile,
            i_fall_profile => i_fall_profile,
            i_gpx_context_fault_sticky => i_gpx_context_fault_sticky,
            i_gpx_shot_drop_sticky => i_gpx_shot_drop_sticky,
            i_gpx_stop_drop_sticky => i_gpx_stop_drop_sticky,
            i_gpx_cdc_reset_busy => i_gpx_cdc_reset_busy,
            i_hit_fault_sticky => i_hit_fault_sticky,
            i_cell_fault_sticky => i_cell_fault_sticky,
            i_frame_fault_sticky => i_frame_fault_sticky,
            i_rise_formatter_fault_sticky =>
                i_rise_formatter_fault_sticky,
            i_fall_formatter_fault_sticky =>
                i_fall_formatter_fault_sticky,
            i_b0_to_accept_clks => i_b0_to_accept_clks,
            i_physical_to_fire_clks => i_physical_to_fire_clks,
            i_virtual_to_accept_clks => i_virtual_to_accept_clks,
            i_fire_done_sync_clks => i_fire_done_sync_clks,
            i_rearm_margin_clks => i_rearm_margin_clks,
            o_irq_processing_warning => irq_processing_c,
            o_irq_laser_timeout => irq_laser_timeout_c,
            o_irq_echo_diagnostic => irq_echo_c,
            o_irq_gpx_transport => irq_gpx_transport_proc_c,
            o_irq_gpx_data => irq_gpx_data_c
        );

    u_tdc_mailbox : entity work.lidar_diag_snapshot_mailbox
        port map (
            i_source_clk => i_csr_clk,
            i_source_rst_n => i_csr_rst_n,
            i_source_request_valid => tdc_request_source_valid,
            o_source_request_ready => tdc_request_source_ready,
            i_source_request => i_csr_request_index,
            o_source_response_valid => tdc_response_dest_valid,
            i_source_response_ready => tdc_response_dest_ready,
            o_source_response => tdc_response_dest,
            i_domain_clk => i_tdc_clk,
            i_domain_rst_n => i_tdc_rst_n,
            o_domain_request_valid => tdc_request_dest_valid,
            i_domain_request_ready => tdc_request_dest_ready,
            o_domain_request => tdc_request_data,
            i_domain_response_valid => tdc_response_source_valid,
            o_domain_response_ready => tdc_response_source_ready,
            i_domain_response => tdc_response_source
        );

    u_tdc_status : entity work.lidar_tdc_status_source
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk => i_tdc_clk,
            i_rst_n => i_tdc_rst_n,
            i_clear_status => i_tdc_clear_status,
            i_request_valid => tdc_request_dest_valid,
            o_request_ready => tdc_request_dest_ready,
            i_request_index => tdc_request_data,
            o_response_valid => tdc_response_source_valid,
            i_response_ready => tdc_response_source_ready,
            o_response => tdc_response_source,
            i_active_mask => i_active_mask,
            i_terminal_mask => i_terminal_mask,
            i_lane_status => i_lane_status,
            i_lane_faults => i_lane_faults,
            i_tdc_safe => i_tdc_safe,
            i_run_enable => i_tdc_run_enable,
            i_active_valid => i_tdc_active_valid,
            i_config_ready => i_tdc_config_ready,
            o_register_service_pause => o_tdc_register_service_pause,
            o_register_read => o_tdc_register_read,
            i_register_read_ready => i_tdc_register_read_ready,
            i_register_read_response => i_tdc_register_read_response,
            o_register_read_response_ready =>
                o_tdc_register_read_response_ready,
            o_irq_gpx_transport => irq_gpx_transport_c
        );

    proc_irq_native_c(0) <= irq_processing_c;
    proc_irq_native_c(1) <= irq_laser_timeout_c;
    proc_irq_native_c(2) <= irq_echo_c;
    proc_irq_native_c(3) <= irq_gpx_data_c;
    proc_irq_native_c(4) <= irq_gpx_transport_proc_c;

    -- Runtime IRQ sources are sticky levels in their native domains. Two CSR
    -- clock stages make them safe inputs to the AXI interrupt controller. A
    -- CLEAR_STATUS command first clears the native source; software then W1C
    -- clears the corresponding pending flag after the synchronized level drops.
    p_irq_to_csr : process (i_csr_clk, i_csr_rst_n)
    begin
        if i_csr_rst_n = '0' then
            proc_irq_meta_r <= (others => '0');
            proc_irq_sync_r <= (others => '0');
            tdc_irq_meta_r <= '0';
            tdc_irq_sync_r <= '0';
        elsif rising_edge(i_csr_clk) then
            proc_irq_meta_r <= proc_irq_native_c;
            proc_irq_sync_r <= proc_irq_meta_r;
            tdc_irq_meta_r <= irq_gpx_transport_c;
            tdc_irq_sync_r <= tdc_irq_meta_r;
        end if;
    end process p_irq_to_csr;

    o_runtime_irq(C_RUNTIME_IRQ_PROCESSING_WARNING) <= proc_irq_sync_r(0);
    o_runtime_irq(C_RUNTIME_IRQ_LASER_TIMEOUT) <= proc_irq_sync_r(1);
    o_runtime_irq(C_RUNTIME_IRQ_ECHO_DIAGNOSTIC) <= proc_irq_sync_r(2);
    o_runtime_irq(C_RUNTIME_IRQ_GPX_TRANSPORT) <=
        proc_irq_sync_r(4) or tdc_irq_sync_r;
    o_runtime_irq(C_RUNTIME_IRQ_GPX_DATA) <= proc_irq_sync_r(3);

end architecture rtl;
