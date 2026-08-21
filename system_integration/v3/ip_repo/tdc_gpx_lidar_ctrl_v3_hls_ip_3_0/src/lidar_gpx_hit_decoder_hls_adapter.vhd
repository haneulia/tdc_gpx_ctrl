library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_v3_hls_contract_pkg.all;

-- V3 HLS Hit decoder boundary adapter.
--
-- The V2 record contract remains the integration API. This adapter owns only
-- byte-aligned AXI packing, HLS reset/abort behavior, result unpacking and the
-- decoder fault pulse/sticky state. Active Rise/Fall masks must remain stable
-- while a Face has accepted Raw events; unified COMMIT already enforces that.
entity lidar_gpx_hit_decoder_hls_adapter is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_rise_mask : in chip_mask_t :=
            G_BUILD_CONFIG.rise_capability_mask;
        i_active_fall_mask : in chip_mask_t :=
            G_BUILD_CONFIG.fall_capability_mask;

        i_raw_event : in  gpx_raw_event_t;
        o_raw_ready : out std_logic;

        o_hit_event : out gpx_hit_event_t;
        i_hit_ready : in  std_logic;

        -- H5 ?†µ?•© ?œ ?œ´ ?Œ? •?š©?´?‹¤. accepted Raw ?‚¬ê±¢¥ ?ˆ˜??€ HLSê°€ ë°˜í™˜?•œ
        -- ê²°ê³¼ ?ˆ˜?˜ ì°¨ì´ë¥¨ù ë³´ì¡´?•˜ë¯€ë¡?, Hit?œ¼ë¡? ë°©ì¶œ?˜ì§€ ?•Š?Š” ? œ?–´ ?‚¬ê±´ë„
        -- ì²˜ë¦¬ ?™„ë£? ? „?—?Š” idleë¡? ?ž˜ëª? ?Œ? •?˜ì§€ ?•Š?Š”?‹¤.
        o_idle           : out std_logic;
        o_inflight_count : out unsigned(7 downto 0);

        o_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_fault_sticky : out gpx_hit_decoder_faults_t
    );
end entity lidar_gpx_hit_decoder_hls_adapter;

architecture rtl of lidar_gpx_hit_decoder_hls_adapter is

    component h1_decoder_ip is
        port (
            ap_clk            : in  std_logic;
            ap_rst_n          : in  std_logic;
            raw_event_in_TDATA  : in  std_logic_vector(
                C_V3_H1_RAW_EVENT_AXIS_BITS - 1 downto 0);
            raw_event_in_TVALID : in  std_logic;
            raw_event_in_TREADY : out std_logic;
            decoder_result_out_TDATA  : out std_logic_vector(
                C_V3_H1_DECODER_RESULT_AXIS_BITS - 1 downto 0);
            decoder_result_out_TVALID : out std_logic;
            decoder_result_out_TREADY : in  std_logic;
            build_tdc_chip_count : in std_logic_vector(7 downto 0);
            build_stop_channels_per_chip : in std_logic_vector(7 downto 0);
            runtime_enabled_rise_chip_mask : in std_logic_vector(7 downto 0);
            runtime_enabled_fall_chip_mask : in std_logic_vector(7 downto 0)
        );
    end component;

    signal hls_rst_n_c       : std_logic;
    signal raw_axis_data_c   : std_logic_vector(
        C_V3_H1_RAW_EVENT_AXIS_BITS - 1 downto 0);
    signal raw_axis_valid_c  : std_logic;
    signal raw_axis_ready_c  : std_logic;
    signal result_axis_data_c : std_logic_vector(
        C_V3_H1_DECODER_RESULT_AXIS_BITS - 1 downto 0);
    signal result_axis_valid_c : std_logic;
    signal result_axis_ready_c : std_logic;
    signal result_emit_c       : std_logic;
    signal result_fire_c       : std_logic;
    signal raw_fire_c          : std_logic;
    signal inflight_count_r    : unsigned(7 downto 0) := (others => '0');

    signal num_chips_c      : std_logic_vector(7 downto 0);
    signal stops_per_chip_c : std_logic_vector(7 downto 0);
    signal rise_mask_c      : std_logic_vector(7 downto 0);
    signal fall_mask_c      : std_logic_vector(7 downto 0);

    signal hit_event_c    : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal fault_pulse_r  : gpx_hit_decoder_faults_t :=
        C_GPX_HIT_DECODER_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_hit_decoder_faults_t :=
        C_GPX_HIT_DECODER_FAULTS_CLEAR;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V3-HLS-B6-001 illegal build configuration"
        severity failure;

    assert C_GPX_RAW_EVENT_PAYLOAD_WIDTH = 215
        report "V3-HLS-B6-002 Raw payload width changed"
        severity failure;
    assert C_GPX_SHOT_CONTEXT_WIDTH = 162
        report "V3-HLS-B6-003 Shot context width changed"
        severity failure;

    -- HLS AXI4-Stream TDATA is byte aligned. Bit 215 and result bits
    -- [223:222] are explicit reserved-zero bits in the V3 boundary contract.
    raw_axis_data_c <= '0' & fn_pack_raw_event(i_raw_event);
    raw_axis_valid_c <= i_raw_event.valid and i_rst_n and not i_abort;
    o_raw_ready <= raw_axis_ready_c and i_rst_n and not i_abort;
    raw_fire_c <= raw_axis_valid_c and raw_axis_ready_c;

    o_inflight_count <= inflight_count_r;
    o_idle <= '1' when i_rst_n = '1' and i_abort = '0' and
        i_raw_event.valid = '0' and inflight_count_r = 0 and
        result_axis_valid_c = '0' else '0';

    hls_rst_n_c <= i_rst_n and not i_abort;
    num_chips_c <= std_logic_vector(to_unsigned(
        G_BUILD_CONFIG.num_chips, num_chips_c'length));
    stops_per_chip_c <= std_logic_vector(to_unsigned(
        G_BUILD_CONFIG.stops_per_chip, stops_per_chip_c'length));
    rise_mask_c <= "0000" & i_active_rise_mask;
    fall_mask_c <= "0000" & i_active_fall_mask;

    result_emit_c <= result_axis_data_c(
        C_V3_H1_RESULT_CONTAINS_HIT_BIT);
    result_axis_ready_c <= '1' when
        i_rst_n = '1' and i_abort = '0' and
        (result_axis_valid_c = '0' or result_emit_c = '0' or
         i_hit_ready = '1') else
        '0';
    result_fire_c <= result_axis_valid_c and result_axis_ready_c;

    p_unpack_result : process (all)
        variable result : gpx_hit_event_t;
        variable kind_value : natural range 0 to 3;
    begin
        result := C_GPX_HIT_EVENT_IDLE;
        if result_axis_valid_c = '1' then
            result.valid := result_emit_c and i_rst_n and not i_abort;
            kind_value := to_integer(unsigned(
                result_axis_data_c(
                    C_V3_H1_HIT_EVENT_KIND_HI downto
                    C_V3_H1_HIT_EVENT_KIND_LO)));
            result.kind := gpx_hit_event_kind_t'val(kind_value);
            result.chip_index := unsigned(result_axis_data_c(
                C_V3_H1_HIT_CHIP_INDEX_HI downto
                C_V3_H1_HIT_CHIP_INDEX_LO));
            result.ififo_id := result_axis_data_c(
                C_V3_H1_HIT_IFIFO_BANK_BIT);
            result.channel_code := unsigned(result_axis_data_c(
                C_V3_H1_HIT_CHANNEL_HI downto C_V3_H1_HIT_CHANNEL_LO));
            result.stop_index := unsigned(result_axis_data_c(
                C_V3_H1_HIT_STOP_INDEX_HI downto
                C_V3_H1_HIT_STOP_INDEX_LO));
            result.start_number := unsigned(result_axis_data_c(
                C_V3_H1_HIT_START_NUMBER_HI downto
                C_V3_H1_HIT_START_NUMBER_LO));
            result.slope := fn_gpx_slope_from_bit(result_axis_data_c(
                C_V3_H1_HIT_SLOPE_RISE_BIT));
            result.hit := unsigned(result_axis_data_c(
                C_V3_H1_HIT_DISTANCE_17BIT_HI downto
                C_V3_H1_HIT_DISTANCE_17BIT_LO));
            result.faulted := result_axis_data_c(
                C_V3_H1_HIT_FAULTED_BIT);
            result.timeout_cause := result_axis_data_c(
                C_V3_H1_HIT_TIMEOUT_CAUSE_HI downto
                C_V3_H1_HIT_TIMEOUT_CAUSE_LO);
            result.shot_context := fn_unpack_shot_context(
                result_axis_data_c(
                    C_V3_H1_HIT_SHOT_CONTEXT_HI downto
                    C_V3_H1_HIT_SHOT_CONTEXT_LO));
            result.chip_shot_seq := unsigned(
                result_axis_data_c(
                    C_V3_H1_HIT_CHIP_SHOT_SEQ_HI downto
                    C_V3_H1_HIT_CHIP_SHOT_SEQ_LO));
        end if;
        hit_event_c <= result;
    end process p_unpack_result;

    o_hit_event    <= hit_event_c;
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    p_inflight_count : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                inflight_count_r <= (others => '0');
            elsif raw_fire_c = '1' and result_fire_c = '0' then
                assert inflight_count_r /= (inflight_count_r'range => '1')
                    report "V3-HLS-B6-005 decoder inflight counter overflow"
                    severity failure;
                inflight_count_r <= inflight_count_r + 1;
            elsif raw_fire_c = '0' and result_fire_c = '1' then
                assert inflight_count_r /= 0
                    report "V3-HLS-B6-006 decoder result without input"
                    severity failure;
                inflight_count_r <= inflight_count_r - 1;
            end if;
        end if;
    end process p_inflight_count;

    p_fault_state : process (i_clk)
        variable pulse_v  : gpx_hit_decoder_faults_t;
        variable sticky_v : gpx_hit_decoder_faults_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                fault_pulse_r  <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
            else
                pulse_v := C_GPX_HIT_DECODER_FAULTS_CLEAR;
                sticky_v := fault_sticky_r;

                if i_clear_sticky = '1' then
                    sticky_v := C_GPX_HIT_DECODER_FAULTS_CLEAR;
                end if;

                if i_abort = '0' and result_fire_c = '1' then
                    assert result_axis_data_c(
                        C_V3_H1_RESULT_RESERVED_HI downto
                        C_V3_H1_RESULT_RESERVED_LO) = "00"
                        report "V3-HLS-B6-004 nonzero reserved result bits"
                        severity failure;

                    pulse_v.chip_index_error := result_axis_data_c(
                        C_V3_H1_RESULT_CHIP_FAULT_BIT);
                    pulse_v.stop_index_error := result_axis_data_c(
                        C_V3_H1_RESULT_STOP_FAULT_BIT);
                    pulse_v.slope_role_error := result_axis_data_c(
                        C_V3_H1_RESULT_SLOPE_FAULT_BIT);
                    sticky_v.chip_index_error :=
                        sticky_v.chip_index_error or result_axis_data_c(
                            C_V3_H1_RESULT_CHIP_FAULT_BIT);
                    sticky_v.stop_index_error :=
                        sticky_v.stop_index_error or result_axis_data_c(
                            C_V3_H1_RESULT_STOP_FAULT_BIT);
                    sticky_v.slope_role_error :=
                        sticky_v.slope_role_error or result_axis_data_c(
                            C_V3_H1_RESULT_SLOPE_FAULT_BIT);
                end if;

                fault_pulse_r  <= pulse_v;
                fault_sticky_r <= sticky_v;
            end if;
        end if;
    end process p_fault_state;

    u_hls_decoder : h1_decoder_ip
        port map (
            ap_clk            => i_clk,
            ap_rst_n          => hls_rst_n_c,
            raw_event_in_TDATA      => raw_axis_data_c,
            raw_event_in_TVALID     => raw_axis_valid_c,
            raw_event_in_TREADY     => raw_axis_ready_c,
            decoder_result_out_TDATA  => result_axis_data_c,
            decoder_result_out_TVALID => result_axis_valid_c,
            decoder_result_out_TREADY => result_axis_ready_c,
            build_tdc_chip_count => num_chips_c,
            build_stop_channels_per_chip => stops_per_chip_c,
            runtime_enabled_rise_chip_mask => rise_mask_c,
            runtime_enabled_fall_chip_mask => fall_mask_c
        );

end architecture rtl;
