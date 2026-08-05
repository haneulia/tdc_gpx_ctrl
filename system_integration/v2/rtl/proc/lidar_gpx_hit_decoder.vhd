library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- B6 registered GPX I-Mode decoder.
--
-- One accepted raw event produces at most one typed Hit/control event. B6 owns
-- only I-Mode field parsing and topology validation; B7 owns Return ordering.
-- A registered input stage breaks the raw source from the decode/output cone,
-- and the output register holds the complete record under backpressure.
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

    signal raw_stage_r    : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
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

    -- Intentionally no same-cycle refill: ready is derived only from the local
    -- input register, so downstream backpressure cannot become a cross-module
    -- combinational ready chain.
    raw_ready_c <= '1' when i_rst_n = '1' and i_abort = '0' and
                            raw_stage_r.valid = '0' else
                   '0';

    o_raw_ready    <= raw_ready_c;
    o_hit_event    <= hit_event_r;
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    p_decode : process (i_clk)
        variable result           : gpx_hit_event_t;
        variable chip_index       : natural range 0 to C_MAX_CHIPS - 1;
        variable channel_index    : natural range 0 to 3;
        variable stop_index       : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable slope_value      : gpx_slope_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                raw_stage_r    <= C_GPX_RAW_EVENT_IDLE;
                hit_event_r    <= C_GPX_HIT_EVENT_IDLE;
                fault_pulse_r  <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
            else
                fault_pulse_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;

                if i_clear_sticky = '1' then
                    fault_sticky_r <= C_GPX_HIT_DECODER_FAULTS_CLEAR;
                end if;

                if i_abort = '1' then
                    raw_stage_r <= C_GPX_RAW_EVENT_IDLE;
                    hit_event_r <= C_GPX_HIT_EVENT_IDLE;
                else
                    if hit_event_r.valid = '1' and i_hit_ready = '1' then
                        hit_event_r.valid <= '0';
                    elsif raw_stage_r.valid = '1' and
                          hit_event_r.valid = '0' then
                        result := C_GPX_HIT_EVENT_IDLE;
                        result.kind          := fn_hit_kind(raw_stage_r.kind);
                        result.chip_index    := raw_stage_r.chip_index;
                        result.ififo_id      := raw_stage_r.ififo_id;
                        result.faulted       := raw_stage_r.faulted;
                        result.timeout_cause := raw_stage_r.timeout_cause;
                        result.shot_context  := raw_stage_r.shot_context;
                        result.chip_shot_seq := raw_stage_r.chip_shot_seq;

                        chip_index := to_integer(raw_stage_r.chip_index);
                        if chip_index >= G_BUILD_CONFIG.num_chips then
                            fault_pulse_r.chip_index_error  <= '1';
                            fault_sticky_r.chip_index_error <= '1';
                        elsif raw_stage_r.kind = GPX_RAW_DATA then
                            channel_index := to_integer(unsigned(
                                raw_stage_r.raw_word(
                                    C_GPX_RAW_CHACODE_HI downto
                                    C_GPX_RAW_CHACODE_LO)));
                            if raw_stage_r.ififo_id = '1' then
                                stop_index := channel_index + 4;
                            else
                                stop_index := channel_index;
                            end if;

                            slope_value := fn_gpx_slope_from_bit(
                                raw_stage_r.raw_word(C_GPX_RAW_SLOPE_BIT));

                            if stop_index >= G_BUILD_CONFIG.stops_per_chip then
                                fault_pulse_r.stop_index_error  <= '1';
                                fault_sticky_r.stop_index_error <= '1';
                            elsif not fn_slope_supported(
                                      chip_index, slope_value) then
                                fault_pulse_r.slope_role_error  <= '1';
                                fault_sticky_r.slope_role_error <= '1';
                            else
                                result.valid        := '1';
                                result.channel_code := to_unsigned(
                                    channel_index,
                                    result.channel_code'length);
                                result.stop_index := to_unsigned(
                                    stop_index, result.stop_index'length);
                                result.start_number := unsigned(
                                    raw_stage_r.raw_word(
                                        C_GPX_RAW_START_HI downto
                                        C_GPX_RAW_START_LO));
                                result.slope := slope_value;
                                result.hit := unsigned(
                                    raw_stage_r.raw_word(
                                        C_GPX_RAW_HIT_HI downto
                                        C_GPX_RAW_HIT_LO));
                                hit_event_r <= result;
                            end if;
                        else
                            result.valid := '1';
                            hit_event_r <= result;
                        end if;
                        raw_stage_r.valid <= '0';
                    end if;

                    if i_raw_event.valid = '1' and raw_ready_c = '1' then
                        raw_stage_r       <= i_raw_event;
                        raw_stage_r.valid <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_decode;

end architecture rtl;
