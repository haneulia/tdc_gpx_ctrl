library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_status_pkg.all;

-- Processing-domain owner for indexed diagnostics. The request index is
-- registered before the 32-bit result is built, so the portal adds a clean
-- pipeline boundary instead of placing a wide live-status mux on the CSR CDC.
entity lidar_processing_status_source is
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_clear_status : in  std_logic;

        i_request_valid : in  std_logic;
        o_request_ready : out std_logic;
        i_request_index : in  lidar_diag_index_t;
        o_response_valid : out std_logic;
        i_response_ready : in  std_logic;
        o_response       : out lidar_diag_response_t;

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
        i_b0_to_accept_clks      : in processing_latency_t;
        i_physical_to_fire_clks  : in processing_latency_t;
        i_virtual_to_accept_clks : in processing_latency_t;
        i_fire_done_sync_clks    : in unsigned(15 downto 0);
        i_rearm_margin_clks      : in unsigned(15 downto 0);

        o_irq_processing_warning : out std_logic;
        o_irq_laser_timeout      : out std_logic;
        o_irq_echo_diagnostic    : out std_logic;
        o_irq_gpx_transport      : out std_logic;
        o_irq_gpx_data           : out std_logic
    );
end entity lidar_processing_status_source;

architecture rtl of lidar_processing_status_source is

    type source_state_t is (
        SOURCE_IDLE,
        SOURCE_BUILD,
        SOURCE_PACK,
        SOURCE_RESPONSE
    );
    signal state_r : source_state_t := SOURCE_IDLE;
    signal index_r : lidar_diag_index_t := (others => '0');
    signal response_r : lidar_diag_response_t := (others => '0');
    signal response_data_r : lidar_diag_word_t := (others => '0');
    signal response_error_r : std_logic := '0';

    signal source_switch_sticky_r : std_logic := '0';
    signal echo_timeout_sticky_r  : std_logic := '0';
    signal echo_aborted_sticky_r  : std_logic := '0';
    signal irq_processing_r : std_logic := '0';
    signal irq_laser_timeout_r : std_logic := '0';
    signal irq_echo_r : std_logic := '0';
    signal irq_gpx_transport_r : std_logic := '0';
    signal irq_gpx_data_r : std_logic := '0';

    -- Idle is a diagnostic observation, not a real-time control input. Sample
    -- the four domains before building a response so the indexed portal sees
    -- one coherent cycle and deep downstream-idle decode cannot extend into
    -- the response register in the same 200 MHz cycle.
    signal pipeline_idle_r : std_logic := '1';
    signal echo_idle_r     : std_logic := '1';
    signal gpx_proc_idle_r : std_logic := '1';
    signal gpx_axis_idle_r : std_logic := '1';

begin

    o_request_ready <= '1' when state_r = SOURCE_IDLE else '0';
    o_response_valid <= '1' when state_r = SOURCE_RESPONSE else '0';
    o_response <= response_r;
    o_irq_processing_warning <= irq_processing_r;
    o_irq_laser_timeout <= irq_laser_timeout_r;
    o_irq_echo_diagnostic <= irq_echo_r;
    o_irq_gpx_transport <= irq_gpx_transport_r;
    o_irq_gpx_data <= irq_gpx_data_r;

    p_source : process (i_clk, i_rst_n)
        variable v_index : natural;
        variable v_data  : lidar_diag_word_t;
        variable v_error : std_logic;
        variable v_hit_fault   : std_logic;
        variable v_cell_fault  : std_logic;
        variable v_frame_fault : std_logic;
        variable v_processing_warning : std_logic;
        variable v_echo_warning : std_logic;
        variable v_gpx_transport_warning : std_logic;
        variable v_gpx_data_warning : std_logic;
        variable v_channel : natural;
    begin
        if i_rst_n = '0' then
            state_r <= SOURCE_IDLE;
            index_r <= (others => '0');
            response_r <= (others => '0');
            response_data_r <= (others => '0');
            response_error_r <= '0';
            source_switch_sticky_r <= '0';
            echo_timeout_sticky_r <= '0';
            echo_aborted_sticky_r <= '0';
            irq_processing_r <= '0';
            irq_laser_timeout_r <= '0';
            irq_echo_r <= '0';
            irq_gpx_transport_r <= '0';
            irq_gpx_data_r <= '0';
            pipeline_idle_r <= '1';
            echo_idle_r <= '1';
            gpx_proc_idle_r <= '1';
            gpx_axis_idle_r <= '1';
        elsif rising_edge(i_clk) then
            pipeline_idle_r <= i_pipeline_idle;
            echo_idle_r <= i_echo_idle;
            gpx_proc_idle_r <= i_gpx_proc_idle;
            gpx_axis_idle_r <= i_gpx_axis_idle;

            v_hit_fault := i_hit_fault_sticky.chip_index_error or
                i_hit_fault_sticky.stop_index_error or
                i_hit_fault_sticky.slope_role_error;
            v_cell_fault := i_cell_fault_sticky.context_mismatch or
                i_cell_fault_sticky.return_overflow or
                i_cell_fault_sticky.start_number_nonzero or
                i_cell_fault_sticky.hit_capacity_drop;
            v_frame_fault := i_frame_fault_sticky.context_mismatch or
                i_frame_fault_sticky.unexpected_cell or
                i_frame_fault_sticky.duplicate_cell or
                i_frame_fault_sticky.duplicate_terminal or
                i_frame_fault_sticky.missing_cell or
                i_frame_fault_sticky.geometry_error or
                i_frame_fault_sticky.column_gap or
                i_frame_fault_sticky.masked_payload_drop;

            if i_clear_status = '1' then
                source_switch_sticky_r <= '0';
                echo_timeout_sticky_r <= '0';
                echo_aborted_sticky_r <= '0';
                irq_processing_r <= '0';
                irq_laser_timeout_r <= '0';
                irq_echo_r <= '0';
                irq_gpx_transport_r <= '0';
                irq_gpx_data_r <= '0';
            else
                if i_processing_diagnostics.source_switch_pulse = '1' then
                    source_switch_sticky_r <= '1';
                end if;
                if i_echo_diagnostics.snapshot.timeout = '1' then
                    echo_timeout_sticky_r <= '1';
                end if;
                if i_echo_diagnostics.snapshot.aborted = '1' then
                    echo_aborted_sticky_r <= '1';
                end if;

                v_processing_warning :=
                    i_processing_diagnostics.invalid_transition_sticky or
                    i_processing_diagnostics.virtual_z_fault(0) or
                    i_processing_diagnostics.virtual_z_fault(1) or
                    i_processing_diagnostics.face_overlap_sticky or
                    i_processing_diagnostics.schedule_overrun_sticky or
                    i_face_close_overflow_sticky or
                    i_processing_diagnostics.monitor_drop_sticky or
                    i_processing_diagnostics.laser.request_drop_sticky or
                    i_processing_diagnostics.laser.operation_abort_sticky or
                    i_processing_diagnostics.laser.unexpected_done_sticky;
                v_echo_warning :=
                    i_echo_diagnostics.outside_window_sticky or
                    i_echo_diagnostics.overlap_sticky or
                    i_echo_diagnostics.profile_not_ready_sticky or
                    i_echo_diagnostics.snapshot.timeout or
                    i_echo_diagnostics.snapshot.aborted or
                    echo_timeout_sticky_r or echo_aborted_sticky_r;
                v_gpx_transport_warning := i_gpx_shot_drop_sticky or
                    i_gpx_stop_drop_sticky;
                v_gpx_data_warning := i_gpx_context_fault_sticky or
                    v_hit_fault or v_cell_fault or v_frame_fault;

                if v_processing_warning = '1' then
                    irq_processing_r <= '1';
                end if;
                if i_processing_diagnostics.laser.fire_done_timeout_sticky =
                        '1' then
                    irq_laser_timeout_r <= '1';
                end if;
                if v_echo_warning = '1' then
                    irq_echo_r <= '1';
                end if;
                if v_gpx_transport_warning = '1' then
                    irq_gpx_transport_r <= '1';
                end if;
                if v_gpx_data_warning = '1' then
                    irq_gpx_data_r <= '1';
                end if;
            end if;

            case state_r is
                when SOURCE_IDLE =>
                    if i_request_valid = '1' then
                        index_r <= i_request_index;
                        state_r <= SOURCE_BUILD;
                    end if;

                when SOURCE_BUILD =>
                    v_index := to_integer(unsigned(index_r));
                    v_data := (others => '0');
                    v_error := '0';

                    case v_index is
                        when C_DIAG_PROC_FLAGS =>
                            v_data(0) :=
                                i_processing_diagnostics.invalid_transition_sticky;
                            v_data(1) := source_switch_sticky_r;
                            v_data(3 downto 2) :=
                                i_processing_diagnostics.virtual_z_fault;
                            v_data(4) :=
                                i_processing_diagnostics.face_overlap_sticky;
                            v_data(5) :=
                                i_processing_diagnostics.schedule_overrun_sticky;
                            v_data(6) := i_face_close_overflow_sticky;
                            v_data(7) :=
                                i_processing_diagnostics.monitor_drop_sticky;
                            v_data(8) := i_processing_diagnostics.laser.
                                request_drop_sticky;
                            v_data(9) := i_processing_diagnostics.laser.
                                fire_done_timeout_sticky;
                            v_data(10) := i_processing_diagnostics.laser.
                                operation_abort_sticky;
                            v_data(11) := i_processing_diagnostics.laser.
                                unexpected_done_sticky;
                            v_data(12) := pipeline_idle_r;
                            v_data(13) := echo_idle_r;
                            v_data(14) := gpx_proc_idle_r;
                            v_data(15) := gpx_axis_idle_r;
                            v_data(16) := i_echo_profile_ready;
                            v_data(17) := i_echo_profile_busy;
                            v_data(18) := i_rise_profile.enabled;
                            v_data(19) := i_fall_profile.enabled;
                            v_data(20) := i_gpx_context_fault_sticky;
                            v_data(21) := v_hit_fault;
                            v_data(22) := v_cell_fault;
                            v_data(23) := v_frame_fault;
                            v_data(24) := i_gpx_shot_drop_sticky;
                            v_data(25) := i_gpx_stop_drop_sticky;
                            v_data(26) := i_gpx_cdc_reset_busy;
                        when C_DIAG_PROC_INVALID_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.
                                    invalid_transition_count);
                        when C_DIAG_PROC_FACE_OVERLAP_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.face_overlap_count);
                        when C_DIAG_PROC_OVERRUN_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.
                                    schedule_overrun_count);
                        when C_DIAG_PROC_MON_DROP_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.monitor_drop_count);
                        when C_DIAG_LASER_REQ_DROP_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.laser.
                                    request_drop_count);
                        when C_DIAG_LASER_TIMEOUT_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.laser.
                                    fire_done_timeout_count);
                        when C_DIAG_LASER_ABORT_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.laser.
                                    operation_abort_count);
                        when C_DIAG_LASER_UNEXPECTED_COUNT =>
                            v_data := std_logic_vector(
                                i_processing_diagnostics.laser.
                                    unexpected_done_count);
                        when C_DIAG_PROC_LATENCY_CONTRACT =>
                            v_data(7 downto 0) := std_logic_vector(
                                i_b0_to_accept_clks);
                            v_data(15 downto 8) := std_logic_vector(
                                i_physical_to_fire_clks);
                            v_data(23 downto 16) := std_logic_vector(
                                i_virtual_to_accept_clks);
                            v_data(27 downto 24) := std_logic_vector(
                                i_fire_done_sync_clks(3 downto 0));
                            v_data(31 downto 28) := std_logic_vector(
                                i_rearm_margin_clks(3 downto 0));
                        when C_DIAG_GPX_PROC_FAULTS =>
                            v_data(0) := i_gpx_context_fault_sticky;
                            v_data(1) :=
                                i_hit_fault_sticky.chip_index_error;
                            v_data(2) :=
                                i_hit_fault_sticky.stop_index_error;
                            v_data(3) :=
                                i_hit_fault_sticky.slope_role_error;
                            v_data(4) :=
                                i_cell_fault_sticky.context_mismatch;
                            v_data(5) :=
                                i_cell_fault_sticky.return_overflow;
                            v_data(6) :=
                                i_cell_fault_sticky.start_number_nonzero;
                            v_data(7) :=
                                i_cell_fault_sticky.hit_capacity_drop;
                            v_data(8) :=
                                i_frame_fault_sticky.context_mismatch;
                            v_data(9) :=
                                i_frame_fault_sticky.unexpected_cell;
                            v_data(10) :=
                                i_frame_fault_sticky.duplicate_cell;
                            v_data(11) :=
                                i_frame_fault_sticky.duplicate_terminal;
                            v_data(12) :=
                                i_frame_fault_sticky.missing_cell;
                            v_data(13) :=
                                i_frame_fault_sticky.geometry_error;
                            v_data(14) :=
                                i_frame_fault_sticky.column_gap;
                            v_data(15) :=
                                i_frame_fault_sticky.masked_payload_drop;
                            v_data(16) := i_face_close_overflow_sticky;
                            v_data(17) := i_gpx_shot_drop_sticky;
                            v_data(18) := i_gpx_stop_drop_sticky;
                        when C_DIAG_PROC_PROFILE_STATE =>
                            v_data(0) := i_echo_profile_ready;
                            v_data(1) := i_echo_profile_busy;
                            v_data(2) := i_rise_profile.valid;
                            v_data(3) := i_rise_profile.enabled;
                            v_data(4) := i_fall_profile.valid;
                            v_data(5) := i_fall_profile.enabled;
                            v_data(6) := pipeline_idle_r;
                            v_data(7) := gpx_axis_idle_r;
                            v_data(8) := i_gpx_cdc_reset_busy;
                            v_data(31 downto 16) := std_logic_vector(
                                i_echo_profile_version);
                        when C_DIAG_RISE_GEOMETRY =>
                            v_data(15 downto 0) := std_logic_vector(
                                i_rise_profile.hsize_bytes);
                            v_data(31 downto 16) := std_logic_vector(
                                i_rise_profile.vsize_lines);
                        when C_DIAG_RISE_STRIDE =>
                            v_data(15 downto 0) := std_logic_vector(
                                i_rise_profile.stride_bytes);
                            v_data(18 downto 16) := std_logic_vector(
                                i_rise_profile.visible_returns);
                            v_data(24 downto 19) := std_logic_vector(
                                i_rise_profile.slot_count);
                            v_data(25) := i_rise_profile.valid;
                            v_data(26) := i_rise_profile.enabled;
                            v_data(28 downto 27) := std_logic_vector(
                                i_rise_profile.footer_lines);
                        when C_DIAG_FALL_GEOMETRY =>
                            v_data(15 downto 0) := std_logic_vector(
                                i_fall_profile.hsize_bytes);
                            v_data(31 downto 16) := std_logic_vector(
                                i_fall_profile.vsize_lines);
                        when C_DIAG_FALL_STRIDE =>
                            v_data(15 downto 0) := std_logic_vector(
                                i_fall_profile.stride_bytes);
                            v_data(18 downto 16) := std_logic_vector(
                                i_fall_profile.visible_returns);
                            v_data(24 downto 19) := std_logic_vector(
                                i_fall_profile.slot_count);
                            v_data(25) := i_fall_profile.valid;
                            v_data(26) := i_fall_profile.enabled;
                            v_data(28 downto 27) := std_logic_vector(
                                i_fall_profile.footer_lines);
                        when C_DIAG_ECHO_FLAGS =>
                            v_data(0) := i_echo_diagnostics.window_active;
                            v_data(1) :=
                                i_echo_diagnostics.simulation_active;
                            v_data(2) :=
                                i_echo_diagnostics.outside_window_sticky;
                            v_data(3) := i_echo_diagnostics.overlap_sticky;
                            v_data(4) := i_echo_diagnostics.
                                profile_not_ready_sticky;
                            v_data(5) := i_echo_diagnostics.snapshot.valid;
                            v_data(6) := echo_timeout_sticky_r;
                            v_data(7) := echo_aborted_sticky_r;
                        when C_DIAG_ECHO_OUTSIDE_COUNT =>
                            v_data := std_logic_vector(
                                i_echo_diagnostics.outside_window_count);
                        when C_DIAG_ECHO_OVERLAP_COUNT =>
                            v_data := std_logic_vector(
                                i_echo_diagnostics.overlap_count);
                        when C_DIAG_ECHO_NOT_READY_COUNT =>
                            v_data := std_logic_vector(
                                i_echo_diagnostics.profile_not_ready_count);
                        when C_DIAG_ECHO_TOTALS =>
                            v_data(15 downto 0) := std_logic_vector(
                                i_echo_diagnostics.snapshot.total_rise);
                            v_data(31 downto 16) := std_logic_vector(
                                i_echo_diagnostics.snapshot.total_fall);
                        when C_DIAG_ECHO_RISE_MASK =>
                            v_data := i_echo_diagnostics.snapshot.rise_mask;
                        when C_DIAG_ECHO_FALL_MASK =>
                            v_data := i_echo_diagnostics.snapshot.fall_mask;
                        when C_DIAG_ECHO_CHANNEL_FIRST to
                             C_DIAG_ECHO_CHANNEL_LAST =>
                            v_channel := v_index -
                                C_DIAG_ECHO_CHANNEL_FIRST;
                            v_data(7 downto 0) := std_logic_vector(
                                i_echo_diagnostics.snapshot.
                                    rise_count(v_channel));
                            v_data(15 downto 8) := std_logic_vector(
                                i_echo_diagnostics.snapshot.
                                    fall_count(v_channel));
                        when others =>
                            v_error := '1';
                    end case;

                    -- Indexed live-status MUX의 32-bit 결과를 먼저
                    -- 등록한다. 응답 valid/error 포장은 다음 clock에서
                    -- 수행하여 index decode, 대형 MUX와 mailbox 응답
                    -- register가 한 200 MHz 경로에 합쳐지지 않게 한다.
                    response_data_r <= v_data;
                    response_error_r <= v_error;
                    state_r <= SOURCE_PACK;

                when SOURCE_PACK =>
                    response_r <= fn_pack_diag_response(
                        response_data_r, response_error_r);
                    state_r <= SOURCE_RESPONSE;

                when SOURCE_RESPONSE =>
                    if i_response_ready = '1' then
                        state_r <= SOURCE_IDLE;
                    end if;
            end case;
        end if;
    end process p_source;

end architecture rtl;
