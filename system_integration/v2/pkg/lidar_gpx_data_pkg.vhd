library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

package lidar_gpx_data_pkg is

    -- TDC-GPX I-Mode raw-word fields. Keep these positions in one package so
    -- later Cell/Frame stages do not duplicate bit literals.
    constant C_GPX_RAW_CHACODE_LO : natural := 26;
    constant C_GPX_RAW_CHACODE_HI : natural := 27;
    constant C_GPX_RAW_START_LO   : natural := 18;
    constant C_GPX_RAW_START_HI   : natural := 25;
    constant C_GPX_RAW_SLOPE_BIT  : natural := 17;
    constant C_GPX_RAW_HIT_LO     : natural := 0;
    constant C_GPX_RAW_HIT_HI     : natural := 16;
    constant C_GPX_HIT_WIDTH      : positive := 17;

    subtype gpx_stop_index_t is unsigned(2 downto 0);
    subtype gpx_return_index_t is unsigned(2 downto 0);
    subtype gpx_channel_code_t is unsigned(1 downto 0);
    subtype gpx_start_number_t is unsigned(7 downto 0);
    subtype gpx_hit_value_t is unsigned(C_GPX_HIT_WIDTH - 1 downto 0);

    type gpx_slope_t is (
        GPX_SLOPE_FALL,
        GPX_SLOPE_RISE
    );

    type gpx_hit_event_kind_t is (
        GPX_HIT_DATA,
        GPX_HIT_IFIFO1_DONE,
        GPX_HIT_DRAIN_DONE,
        GPX_HIT_TIMEOUT
    );

    -- B6 typed boundary. Data and control events share the Shot/Chip identity
    -- fields, while kind explicitly states which remaining fields are valid.
    type gpx_hit_event_t is record
        valid         : std_logic;
        kind          : gpx_hit_event_kind_t;
        chip_index    : gpx_chip_index_t;
        ififo_id      : std_logic;
        channel_code  : gpx_channel_code_t;
        stop_index    : gpx_stop_index_t;
        start_number  : gpx_start_number_t;
        slope         : gpx_slope_t;
        return_index  : gpx_return_index_t;
        hit           : gpx_hit_value_t;
        faulted       : std_logic;
        timeout_cause : std_logic_vector(2 downto 0);
        shot_context  : shot_start_event_t;
        chip_shot_seq : unsigned(15 downto 0);
    end record gpx_hit_event_t;

    -- Pulse and sticky records intentionally share one shape. They stay out
    -- of CSR ownership until the Stage 6 status aggregation is finalized.
    type gpx_hit_decoder_faults_t is record
        chip_index_error : std_logic;
        stop_index_error : std_logic;
        slope_role_error : std_logic;
        return_overflow  : std_logic;
    end record gpx_hit_decoder_faults_t;

    constant C_GPX_HIT_EVENT_IDLE : gpx_hit_event_t := (
        valid         => '0',
        kind          => GPX_HIT_DATA,
        chip_index    => (others => '0'),
        ififo_id      => '0',
        channel_code  => (others => '0'),
        stop_index    => (others => '0'),
        start_number  => (others => '0'),
        slope         => GPX_SLOPE_FALL,
        return_index  => (others => '0'),
        hit           => (others => '0'),
        faulted       => '0',
        timeout_cause => (others => '0'),
        shot_context  => C_SHOT_START_EVENT_IDLE,
        chip_shot_seq => (others => '0')
    );

    constant C_GPX_HIT_DECODER_FAULTS_CLEAR : gpx_hit_decoder_faults_t := (
        chip_index_error => '0',
        stop_index_error => '0',
        slope_role_error => '0',
        return_overflow  => '0'
    );

    function fn_gpx_slope_from_bit(value : std_logic) return gpx_slope_t;
    function fn_gpx_slope_to_bit(value : gpx_slope_t) return std_logic;

end package lidar_gpx_data_pkg;

package body lidar_gpx_data_pkg is

    function fn_gpx_slope_from_bit(value : std_logic) return gpx_slope_t is
    begin
        if value = '1' then
            return GPX_SLOPE_RISE;
        end if;
        return GPX_SLOPE_FALL;
    end function fn_gpx_slope_from_bit;

    function fn_gpx_slope_to_bit(value : gpx_slope_t) return std_logic is
    begin
        if value = GPX_SLOPE_RISE then
            return '1';
        end if;
        return '0';
    end function fn_gpx_slope_to_bit;

end package body lidar_gpx_data_pkg;
