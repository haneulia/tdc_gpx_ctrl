library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- V3 HLS Hit-to-Cell collector boundary adapter.
--
-- The public integration API remains the V2 record contract. This adapter
-- only owns deterministic bit packing, ap_ctrl_hs continuous execution,
-- abort generation transport, stale-output draining and fault state.
entity lidar_gpx_cell_collector_hls_adapter is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_version    : in unsigned(15 downto 0);
        i_max_hits_per_stop : in unsigned(2 downto 0);
        i_active_rise_mask  : in chip_mask_t :=
            G_BUILD_CONFIG.rise_capability_mask;
        i_active_fall_mask  : in chip_mask_t :=
            G_BUILD_CONFIG.fall_capability_mask;

        i_hit_event : in  gpx_hit_event_t;
        o_hit_ready : out std_logic;

        o_cell_event : out gpx_cell_event_t;
        i_cell_ready : in  std_logic;

        -- ?•œ STOP ì±„ë„?˜ Return ?ˆ˜ì§? ?˜¸ì¶œê³¼ abort stale-output Drain?´
        -- ëª¨ë‘ ??‚œ ?’¤?—ë§? ?™œ?„±?™”?˜?Š” ?†µ?•© ?œ ?œ´ ?ƒ?ƒœ?‹¤.
        o_idle : out std_logic;

        o_fault_pulse  : out gpx_cell_collector_faults_t;
        o_fault_sticky : out gpx_cell_collector_faults_t
    );
end entity lidar_gpx_cell_collector_hls_adapter;

architecture rtl of lidar_gpx_cell_collector_hls_adapter is

    component h2_collector_ip is
        port (
            ap_clk    : in  std_logic;
            ap_rst_n  : in  std_logic;
            ap_start  : in  std_logic;
            ap_done   : out std_logic;
            ap_idle   : out std_logic;
            ap_ready  : out std_logic;

            decoded_hit_event_in_TDATA  : in  std_logic_vector(
                C_V3_H2_COLLECTOR_INPUT_AXIS_BITS - 1 downto 0);
            decoded_hit_event_in_TVALID : in  std_logic;
            decoded_hit_event_in_TREADY : out std_logic;

            collector_result_out_TDATA  : out std_logic_vector(
                C_V3_H2_COLLECTOR_RESULT_AXIS_BITS - 1 downto 0);
            collector_result_out_TVALID : out std_logic;
            collector_result_out_TREADY : in  std_logic;

            build_tdc_chip_count : in std_logic_vector(7 downto 0);
            build_stop_channels_per_chip : in std_logic_vector(7 downto 0);
            build_max_return_count_per_stop : in std_logic_vector(7 downto 0);
            runtime_enabled_rise_chip_mask : in std_logic_vector(7 downto 0);
            runtime_enabled_fall_chip_mask : in std_logic_vector(7 downto 0);
            runtime_visible_return_count : in std_logic_vector(7 downto 0);
            active_configuration_version : in std_logic_vector(15 downto 0)
        );
    end component;

    signal packed_hit_axis_data_c : std_logic_vector(
        C_V3_H2_COLLECTOR_INPUT_AXIS_BITS - 1 downto 0);
    signal packed_hit_axis_valid_c : std_logic;
    signal packed_hit_axis_ready_c : std_logic;
    signal hit_skid_flush_c : std_logic;

    signal hit_axis_data_c  : std_logic_vector(
        C_V3_H2_COLLECTOR_INPUT_AXIS_BITS - 1 downto 0);
    signal hit_axis_valid_c : std_logic;
    signal hit_axis_ready_c : std_logic;

    signal result_axis_data_c  : std_logic_vector(
        C_V3_H2_COLLECTOR_RESULT_AXIS_BITS - 1 downto 0);
    signal result_axis_valid_c : std_logic;
    signal result_axis_ready_c : std_logic;
    signal hls_result_axis_data_c  : std_logic_vector(
        C_V3_H2_COLLECTOR_RESULT_AXIS_BITS - 1 downto 0);
    signal hls_result_axis_valid_c : std_logic;
    signal hls_result_axis_ready_c : std_logic;
    signal result_skid_flush_c     : std_logic;
    signal result_emit_c       : std_logic;
    signal result_fire_c       : std_logic;
    signal hit_fire_c          : std_logic;

    signal hls_done_c : std_logic;

    signal num_chips_c            : std_logic_vector(7 downto 0);
    signal stops_per_chip_c       : std_logic_vector(7 downto 0);
    signal max_returns_c          : std_logic_vector(7 downto 0);
    signal rise_mask_c            : std_logic_vector(7 downto 0);
    signal fall_mask_c            : std_logic_vector(7 downto 0);
    signal visible_returns_c      : std_logic_vector(7 downto 0);
    signal active_version_c       : std_logic_vector(15 downto 0);

    signal reset_epoch_r  : unsigned(7 downto 0) := (others => '0');
    signal abort_d_r      : std_logic := '0';
    signal hls_inflight_r : std_logic := '0';
    signal flush_active_r : std_logic := '0';
    signal input_accept_enable_r : std_logic := '0';

    signal cell_event_c   : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal fault_pulse_r  : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-HLS-B7-001 illegal build configuration"
        severity failure;
    assert C_GPX_SHOT_CONTEXT_WIDTH = 162
        report "V3-HLS-B7-002 Shot context width changed"
        severity failure;
    assert C_MAX_RETURNS_PER_STOP = 7
        report "V3-HLS-B7-003 physical Return capacity changed"
        severity failure;

    -- Packing is wiring only. Bits [223:218] are reserved-zero and the abort
    -- generation in [231:224] is sampled with the accepted Hit handshake.
    p_pack_hit : process (all)
        variable payload : std_logic_vector(
            C_V3_H2_COLLECTOR_INPUT_AXIS_BITS - 1 downto 0);
    begin
        payload := (others => '0');
        payload(
            C_V3_H1_HIT_EVENT_KIND_HI downto
            C_V3_H1_HIT_EVENT_KIND_LO) := std_logic_vector(to_unsigned(
            gpx_hit_event_kind_t'pos(i_hit_event.kind), 2));
        payload(C_V3_H1_HIT_CHIP_INDEX_HI downto
                C_V3_H1_HIT_CHIP_INDEX_LO) :=
            std_logic_vector(i_hit_event.chip_index);
        payload(C_V3_H1_HIT_IFIFO_BANK_BIT) := i_hit_event.ififo_id;
        payload(C_V3_H1_HIT_CHANNEL_HI downto C_V3_H1_HIT_CHANNEL_LO) :=
            std_logic_vector(i_hit_event.channel_code);
        payload(C_V3_H1_HIT_STOP_INDEX_HI downto
                C_V3_H1_HIT_STOP_INDEX_LO) :=
            std_logic_vector(i_hit_event.stop_index);
        payload(C_V3_H1_HIT_START_NUMBER_HI downto
                C_V3_H1_HIT_START_NUMBER_LO) :=
            std_logic_vector(i_hit_event.start_number);
        payload(C_V3_H1_HIT_SLOPE_RISE_BIT) :=
            fn_gpx_slope_to_bit(i_hit_event.slope);
        payload(C_V3_H1_HIT_DISTANCE_17BIT_HI downto
                C_V3_H1_HIT_DISTANCE_17BIT_LO) :=
            std_logic_vector(i_hit_event.hit);
        payload(C_V3_H1_HIT_FAULTED_BIT) := i_hit_event.faulted;
        payload(C_V3_H1_HIT_TIMEOUT_CAUSE_HI downto
                C_V3_H1_HIT_TIMEOUT_CAUSE_LO) := i_hit_event.timeout_cause;
        payload(C_V3_H1_HIT_SHOT_CONTEXT_HI downto
                C_V3_H1_HIT_SHOT_CONTEXT_LO) :=
            fn_pack_shot_context(i_hit_event.shot_context);
        payload(C_V3_H1_HIT_CHIP_SHOT_SEQ_HI downto
                C_V3_H1_HIT_CHIP_SHOT_SEQ_LO) :=
            std_logic_vector(i_hit_event.chip_shot_seq);
        payload(C_V3_H2_INPUT_RESET_EPOCH_HI downto
                C_V3_H2_INPUT_RESET_EPOCH_LO) :=
            std_logic_vector(reset_epoch_r);
        packed_hit_axis_data_c <= payload;
    end process p_pack_hit;

    packed_hit_axis_valid_c <= i_hit_event.valid and i_rst_n and
                               not i_abort and input_accept_enable_r;
    o_hit_ready <= packed_hit_axis_ready_c and i_rst_n and
                   not i_abort and input_accept_enable_r;
    hit_fire_c <= hit_axis_valid_c and hit_axis_ready_c;
    hit_skid_flush_c <= i_abort or flush_active_r;

    -- H1?˜ ?­?´ ?° Hit ê³„ì•½?„ H2 HLS ?ž…? ¥ Registerê¹Œì¡×€ ?•œ Cycle?— ì§ì ‘
    -- ? „?‹¬?•˜ì§€ ?•Š?Š”?‹¤. 2-slot Bufferê°€ Readyë¥¨ù Register?•˜ê³? ì²˜ë¦¬?œ¨??€
    -- 1 Event/clock?œ¼ë¡? ?œ ì§€?•˜?—¬ Stage ê°? Routing ì§€?—°?„ ? œ?•œ?•œ?‹¤.
    u_hit_input_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_V3_H2_COLLECTOR_INPUT_AXIS_BITS
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => hit_skid_flush_c,
            i_s_valid => packed_hit_axis_valid_c,
            o_s_ready => packed_hit_axis_ready_c,
            i_s_data  => packed_hit_axis_data_c,
            o_m_valid => hit_axis_valid_c,
            i_m_ready => hit_axis_ready_c,
            o_m_data  => hit_axis_data_c
        );

    num_chips_c <= std_logic_vector(to_unsigned(
        G_BUILD_CONFIG.num_chips, num_chips_c'length));
    stops_per_chip_c <= std_logic_vector(to_unsigned(
        G_BUILD_CONFIG.stops_per_chip, stops_per_chip_c'length));
    max_returns_c <= std_logic_vector(to_unsigned(
        G_BUILD_CONFIG.max_returns_per_stop, max_returns_c'length));
    rise_mask_c <= "0000" & i_active_rise_mask;
    fall_mask_c <= "0000" & i_active_fall_mask;
    visible_returns_c <= std_logic_vector(resize(
        i_max_hits_per_stop, visible_returns_c'length));
    active_version_c <= std_logic_vector(i_active_version);

    result_emit_c <= result_axis_data_c(
        C_V3_H2_RESULT_CONTAINS_CELL_BIT);
    result_skid_flush_c <= i_abort or flush_active_r;

    -- HLS ?ž?™ AXIS ì¶œë ¥ Register ??€?‹  RTL?´ ëª…ì‹œ? ?œ¼ë¡? 2-slot ê²½ê³„ë¥¨ù
    -- ?†Œ?œ ?•œ?‹¤. Backpressure ?š©?Ÿ‰??€ ?œ ì§€?•˜ë©´ì„œ HLS ?‚´ë¶€ Ready-to-CE ê²½ë¡œë¥¨ù
    -- ?´ Register?—?„œ ?Š?–´ 200 MHz Processing timing?„ ?•ˆ? •?™”?•œ?‹¤.
    u_result_output_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_V3_H2_COLLECTOR_RESULT_AXIS_BITS
        )
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => result_skid_flush_c,
            i_s_valid => hls_result_axis_valid_c,
            o_s_ready => hls_result_axis_ready_c,
            i_s_data  => hls_result_axis_data_c,
            o_m_valid => result_axis_valid_c,
            i_m_ready => result_axis_ready_c,
            o_m_data  => result_axis_data_c
        );

    -- Status acknowledgements never wait for the Cell consumer. During abort
    -- or stale-output flush every result is consumed and intentionally hidden.
    result_axis_ready_c <= '1' when
        i_rst_n = '1' and
        (i_abort = '1' or flush_active_r = '1' or
         result_axis_valid_c = '0' or result_emit_c = '0' or
         i_cell_ready = '1') else
        '0';
    result_fire_c <= result_axis_valid_c and result_axis_ready_c;

    p_unpack_result : process (all)
        variable result     : gpx_cell_event_t;
        variable kind_value : natural range 0 to 3;
        variable bit_lo     : natural;
    begin
        result := C_GPX_CELL_EVENT_IDLE;
        if result_axis_valid_c = '1' then
            result.valid := result_emit_c and i_rst_n and
                            not i_abort and not flush_active_r;
            kind_value := to_integer(unsigned(
                result_axis_data_c(
                    C_V3_H2_CELL_KIND_HI downto C_V3_H2_CELL_KIND_LO)));
            result.kind := gpx_cell_event_kind_t'val(kind_value);
            result.chip_index := unsigned(result_axis_data_c(
                C_V3_H2_CELL_CHIP_INDEX_HI downto
                C_V3_H2_CELL_CHIP_INDEX_LO));
            result.ififo_id := result_axis_data_c(
                C_V3_H2_CELL_IFIFO_BANK_BIT);
            result.stop_index := unsigned(result_axis_data_c(
                C_V3_H2_CELL_STOP_INDEX_HI downto
                C_V3_H2_CELL_STOP_INDEX_LO));
            result.slope := fn_gpx_slope_from_bit(result_axis_data_c(
                C_V3_H2_CELL_SLOPE_RISE_BIT));
            result.hit_count := unsigned(result_axis_data_c(
                C_V3_H2_CELL_VISIBLE_RETURNS_HI downto
                C_V3_H2_CELL_VISIBLE_RETURNS_LO));
            result.max_hits := unsigned(result_axis_data_c(
                C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_HI downto
                C_V3_H2_CELL_SERIALIZED_RETURN_SLOTS_LO));
            for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                bit_lo := C_V3_H2_CELL_PACKED_HITS_LO +
                          hit_index * C_GPX_HIT_WIDTH;
                result.hits(hit_index) := unsigned(result_axis_data_c(
                    bit_lo + C_GPX_HIT_WIDTH - 1 downto bit_lo));
            end loop;
            result.hit_dropped := result_axis_data_c(
                C_V3_H2_CELL_HIT_DROPPED_BIT);
            result.return_overflow := result_axis_data_c(
                C_V3_H2_CELL_RETURN_OVERFLOW_BIT);
            result.error_fill := result_axis_data_c(
                C_V3_H2_CELL_ERROR_FILL_BIT);
            result.faulted := result_axis_data_c(
                C_V3_H2_CELL_FAULTED_BIT);
            result.timeout_cause := result_axis_data_c(
                C_V3_H2_CELL_TIMEOUT_CAUSE_HI downto
                C_V3_H2_CELL_TIMEOUT_CAUSE_LO);
            result.shot_context := fn_unpack_shot_context(
                result_axis_data_c(
                    C_V3_H2_CELL_SHOT_CONTEXT_HI downto
                    C_V3_H2_CELL_SHOT_CONTEXT_LO));
            result.chip_shot_seq := unsigned(
                result_axis_data_c(
                    C_V3_H2_CELL_CHIP_SHOT_SEQ_HI downto
                    C_V3_H2_CELL_CHIP_SHOT_SEQ_LO));
        end if;
        cell_event_c <= result;
    end process p_unpack_result;

    o_cell_event   <= cell_event_c;
    o_idle <= '1' when i_rst_n = '1' and i_abort = '0' and
        hls_inflight_r = '0' and flush_active_r = '0' and
        hit_axis_valid_c = '0' and hls_result_axis_valid_c = '0' and
        result_axis_valid_c = '0' and
        i_hit_event.valid = '0' else '0';
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    -- flush_activeë¥¨ù Upstream Ready?— ì§ì ‘ ì¡°í•© ?—°ê²°í•˜ë©¢¥ H2 ë³µêµ¬ ?ƒ?ƒœê°€
    -- H1 HLS ?ž…? ¥ CEê¹Œì¡×€ ?—­? „?ŒŒ?œ?‹¤. ?ž…? ¥ ?—ˆ?š© ì°½ì„ Register?•˜?—¬ abort
    -- ë³µêµ¬ ê²½ë¡œë¥¨ù ?•œ Cycle ?Š¦ê²? ?—¬?Š” ??€?‹  ? •?ƒ ì²˜ë¦¬?œ¨??€ ê·¸ë?€ë¡? ?œ ì§€?•œ?‹¤.
    p_input_accept_enable : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' or flush_active_r = '1' then
                input_accept_enable_r <= '0';
            else
                input_accept_enable_r <= '1';
            end if;
        end if;
    end process p_input_accept_enable;

    p_control_and_faults : process (i_clk)
        variable pulse_v  : gpx_cell_collector_faults_t;
        variable sticky_v : gpx_cell_collector_faults_t;
        variable flush_v  : std_logic;
        variable inflight_v : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                reset_epoch_r  <= (others => '0');
                abort_d_r      <= '0';
                hls_inflight_r <= '0';
                flush_active_r <= '0';
                fault_pulse_r  <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                pulse_v := C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                sticky_v := fault_sticky_r;
                flush_v := flush_active_r;
                inflight_v := hls_inflight_r;

                if hls_done_c = '1' then
                    inflight_v := '0';
                end if;
                -- ?™„ë£Œì?€ ?‹¤?Œ HLS ?ž…? ¥ ?Š¹?¸?´ ê°™ì?€ Cycle?´ë©¢¥ ?ƒˆ ?˜¸ì¶œì´
                -- ?š°?„ ?•˜ë¯€ë¡? ?†µ?•© idle?´ ì¡°ê¸°?— ?˜¬?¼ê°€ì§€ ?•Š?Š”?‹¤.
                if hit_fire_c = '1' then
                    inflight_v := '1';
                end if;

                if i_clear_sticky = '1' then
                    sticky_v := C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                end if;

                if i_abort = '1' and abort_d_r = '0' then
                    reset_epoch_r <= reset_epoch_r + 1;
                    -- TREADY may rise before an invocation has retired all
                    -- results, so it cannot prove that no stale output exists.
                    -- Track the accepted-input to ap_done interval explicitly.
                    flush_v := inflight_v or hls_result_axis_valid_c or
                               result_axis_valid_c;
                end if;
                abort_d_r <= i_abort;

                if hls_done_c = '1' then
                    flush_v := '0';
                end if;

                if i_abort = '0' and flush_active_r = '0' and
                   result_fire_c = '1' then
                    assert result_axis_data_c(
                        C_V3_H2_RESULT_RESERVED_HI downto
                        C_V3_H2_RESULT_RESERVED_LO) = "0000"
                        report "V3-HLS-B7-004 nonzero reserved result bits"
                        severity failure;

                    pulse_v.context_mismatch := result_axis_data_c(
                        C_V3_H2_RESULT_CONTEXT_FAULT_BIT);
                    pulse_v.return_overflow := result_axis_data_c(
                        C_V3_H2_RESULT_OVERFLOW_FAULT_BIT);
                    pulse_v.start_number_nonzero := result_axis_data_c(
                        C_V3_H2_RESULT_START_FAULT_BIT);
                    pulse_v.hit_capacity_drop := result_axis_data_c(
                        C_V3_H2_RESULT_CAPACITY_FAULT_BIT);
                    sticky_v.context_mismatch :=
                        sticky_v.context_mismatch or result_axis_data_c(
                            C_V3_H2_RESULT_CONTEXT_FAULT_BIT);
                    sticky_v.return_overflow :=
                        sticky_v.return_overflow or result_axis_data_c(
                            C_V3_H2_RESULT_OVERFLOW_FAULT_BIT);
                    sticky_v.start_number_nonzero :=
                        sticky_v.start_number_nonzero or
                        result_axis_data_c(C_V3_H2_RESULT_START_FAULT_BIT);
                    sticky_v.hit_capacity_drop :=
                        sticky_v.hit_capacity_drop or result_axis_data_c(
                            C_V3_H2_RESULT_CAPACITY_FAULT_BIT);
                end if;

                flush_active_r <= flush_v;
                hls_inflight_r <= inflight_v;
                fault_pulse_r  <= pulse_v;
                fault_sticky_r <= sticky_v;
            end if;
        end if;
    end process p_control_and_faults;

    u_hls_collector : h2_collector_ip
        port map (
            ap_clk    => i_clk,
            ap_rst_n  => i_rst_n,
            ap_start  => '1',
            ap_done   => hls_done_c,
            ap_idle   => open,
            ap_ready  => open,

            decoded_hit_event_in_TDATA  => hit_axis_data_c,
            decoded_hit_event_in_TVALID => hit_axis_valid_c,
            decoded_hit_event_in_TREADY => hit_axis_ready_c,

            collector_result_out_TDATA  => hls_result_axis_data_c,
            collector_result_out_TVALID => hls_result_axis_valid_c,
            collector_result_out_TREADY => hls_result_axis_ready_c,

            build_tdc_chip_count => num_chips_c,
            build_stop_channels_per_chip => stops_per_chip_c,
            build_max_return_count_per_stop => max_returns_c,
            runtime_enabled_rise_chip_mask => rise_mask_c,
            runtime_enabled_fall_chip_mask => fall_mask_c,
            runtime_visible_return_count => visible_returns_c,
            active_configuration_version => active_version_c
        );

end architecture rtl;
