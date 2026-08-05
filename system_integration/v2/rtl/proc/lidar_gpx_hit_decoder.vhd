library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- B6 registered GPX I-Mode decoder.
--
-- One accepted raw event produces at most one typed Hit/control event. Return
-- counters are independent for every Chip, STOP and slope. The one-entry
-- elastic output register holds the complete record under backpressure.
entity lidar_gpx_hit_decoder is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_raw_event : in  gpx_raw_event_t;
        o_raw_ready : out std_logic;

        o_hit_event : out gpx_hit_event_t;
        i_hit_ready : in  std_logic;

        o_fault_pulse  : out gpx_hit_decoder_faults_t;
        o_fault_sticky : out gpx_hit_decoder_faults_t
    );
end entity lidar_gpx_hit_decoder;

architecture rtl of lidar_gpx_hit_decoder is

    subtype return_count_t is unsigned(2 downto 0);
    type return_count_by_slope_t is array (gpx_slope_t) of return_count_t;
    type return_count_by_stop_t is array (
        0 to C_MAX_STOPS_PER_CHIP - 1) of return_count_by_slope_t;
    type return_count_by_chip_t is array (
        0 to C_MAX_CHIPS - 1) of return_count_by_stop_t;

    signal return_count_r : return_count_by_chip_t :=
        (others => (others => (others => (others => '0'))));
    signal hit_event_r    : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal fault_pulse_r  : gpx_hit_decoder_faults_t :=
        C_GPX_HIT_DECODER_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_hit_decoder_faults_t :=
        C_GPX_HIT_DECODER_FAULTS_CLEAR;
    signal raw_ready_c    : std_logic;

    function fn_hit_kind(
        raw_kind : gpx_raw_event_kind_t
    ) return gpx_hit_event_kind_t is
    begin
        case raw_kind is
            when GPX_RAW_DATA        => return GPX_HIT_DATA;
            when GPX_RAW_IFIFO1_DONE => return GPX_HIT_IFIFO1_DONE;
            when GPX_RAW_DRAIN_DONE  => return GPX_HIT_DRAIN_DONE;
            when GPX_RAW_TIMEOUT     => return GPX_HIT_TIMEOUT;
        end case;
    end function fn_hit_kind;

    function fn_slope_supported(
        chip_index : natural;
        slope      : gpx_slope_t
    ) return boolean is
    begin
        if slope = GPX_SLOPE_RISE then
            return G_BUILD_CONFIG.rise_capability_mask(chip_index) = '1';
        end if;
        return G_BUILD_CONFIG.fall_capability_mask(chip_index) = '1';
    end function fn_slope_supported;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B6-001 illegal build configuration"
        severity failure;

    raw_ready_c <= '0' when i_rst_n = '0' or i_abort = '1' else
                   '1' when hit_event_r.valid = '0' or i_hit_ready = '1' else
                   '0';

    o_raw_ready    <= raw_ready_c;
    o_hit_event    <= hit_event_r;
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    p_decode : process (i_clk)
        variable result        : gpx_hit_event_t;
        variable chip_index    : natural range 0 to C_MAX_CHIPS - 1;
        variable channel_index : natural range 0 to 3;
        variable stop_index    : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable slope_value   : gpx_slope_t;
        variable count_value   : return_count_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                return_count_r <=
                    (others => (others => (others => (others => '0'))));
                hit_event_r    <= C_GPX_HIT_EVENT_IDLE;
                fault_pulse_r  <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
            else
                fault_pulse_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;

                if i_clear_sticky = '1' then
                    fault_sticky_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
                end if;

                if i_abort = '1' then
                    return_count_r <=
                        (others => (others => (others => (others => '0'))));
                    hit_event_r <= C_GPX_HIT_EVENT_IDLE;
                else
                    if hit_event_r.valid = '1' and i_hit_ready = '1' then
                        hit_event_r.valid <= '0';
                    end if;

                    if i_raw_event.valid = '1' and raw_ready_c = '1' then
                        result := C_GPX_HIT_EVENT_IDLE;
                        result.kind          := fn_hit_kind(i_raw_event.kind);
                        result.chip_index    := i_raw_event.chip_index;
                        result.ififo_id      := i_raw_event.ififo_id;
                        result.faulted       := i_raw_event.faulted;
                        result.timeout_cause := i_raw_event.timeout_cause;
                        result.shot_context  := i_raw_event.shot_context;
                        result.chip_shot_seq := i_raw_event.chip_shot_seq;

                        chip_index := to_integer(i_raw_event.chip_index);
                        if chip_index >= G_BUILD_CONFIG.num_chips then
                            fault_pulse_r.chip_index_error  <= '1';
                            fault_sticky_r.chip_index_error <= '1';
                        elsif i_raw_event.kind = GPX_RAW_DATA then
                            channel_index := to_integer(unsigned(
                                i_raw_event.raw_word(
                                    C_GPX_RAW_CHACODE_HI downto
                                    C_GPX_RAW_CHACODE_LO)));
                            if i_raw_event.ififo_id = '1' then
                                stop_index := channel_index + 4;
                            else
                                stop_index := channel_index;
                            end if;

                            slope_value := fn_gpx_slope_from_bit(
                                i_raw_event.raw_word(C_GPX_RAW_SLOPE_BIT));

                            if stop_index >= G_BUILD_CONFIG.stops_per_chip then
                                fault_pulse_r.stop_index_error  <= '1';
                                fault_sticky_r.stop_index_error <= '1';
                            elsif not fn_slope_supported(
                                      chip_index, slope_value) then
                                fault_pulse_r.slope_role_error  <= '1';
                                fault_sticky_r.slope_role_error <= '1';
                            else
                                count_value := return_count_r(
                                    chip_index)(stop_index)(slope_value);
                                if count_value < to_unsigned(
                                        G_BUILD_CONFIG.max_returns_per_stop,
                                        count_value'length) then
                                    result.valid        := '1';
                                    result.channel_code := to_unsigned(
                                        channel_index,
                                        result.channel_code'length);
                                    result.stop_index := to_unsigned(
                                        stop_index, result.stop_index'length);
                                    result.start_number := unsigned(
                                        i_raw_event.raw_word(
                                            C_GPX_RAW_START_HI downto
                                            C_GPX_RAW_START_LO));
                                    result.slope := slope_value;
                                    result.return_index := count_value;
                                    result.hit := unsigned(
                                        i_raw_event.raw_word(
                                            C_GPX_RAW_HIT_HI downto
                                            C_GPX_RAW_HIT_LO));
                                    hit_event_r <= result;
                                    return_count_r(chip_index)(stop_index)(
                                        slope_value) <= count_value + 1;
                                else
                                    fault_pulse_r.return_overflow  <= '1';
                                    fault_sticky_r.return_overflow <= '1';
                                end if;
                            end if;
                        else
                            result.valid := '1';
                            hit_event_r <= result;

                            if i_raw_event.kind = GPX_RAW_DRAIN_DONE or
                               i_raw_event.kind = GPX_RAW_TIMEOUT then
                                for stop in 0 to
                                        C_MAX_STOPS_PER_CHIP - 1 loop
                                    return_count_r(chip_index)(stop)(
                                        GPX_SLOPE_FALL) <= (others => '0');
                                    return_count_r(chip_index)(stop)(
                                        GPX_SLOPE_RISE) <= (others => '0');
                                end loop;
                            end if;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process p_decode;

end architecture rtl;
