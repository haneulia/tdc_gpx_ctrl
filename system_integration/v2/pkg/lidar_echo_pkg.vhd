library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

package lidar_echo_pkg is

    constant C_ECHO_MAX_CHANNELS : positive :=
        C_MAX_CHIPS * C_MAX_STOPS_PER_CHIP;
    -- CH0 + 31 * STEP can exceed 16 bits even though each CSR source field is
    -- 16 bits. Keep the expanded delay and local-clock counters at 32 bits.
    constant C_ECHO_DELAY_WIDTH : positive := 32;
    constant C_ECHO_COUNT_WIDTH : positive := 8;

    subtype echo_delay_ticks_t is unsigned(C_ECHO_DELAY_WIDTH - 1 downto 0);
    subtype echo_delay_clocks_t is unsigned(C_ECHO_DELAY_WIDTH - 1 downto 0);
    subtype echo_count_t is unsigned(C_ECHO_COUNT_WIDTH - 1 downto 0);
    subtype echo_total_count_t is unsigned(15 downto 0);
    subtype echo_channel_mask_t is
        std_logic_vector(C_ECHO_MAX_CHANNELS - 1 downto 0);

    type echo_delay_clocks_array_t is array (
        0 to C_ECHO_MAX_CHANNELS - 1
    ) of echo_delay_clocks_t;
    type echo_count_array_t is array (
        0 to C_ECHO_MAX_CHANNELS - 1
    ) of echo_count_t;

    type echo_shot_snapshot_t is record
        valid       : std_logic;
        request     : shot_request_t;
        timeout     : std_logic;
        aborted     : std_logic;
        rise_mask   : echo_channel_mask_t;
        fall_mask   : echo_channel_mask_t;
        rise_count  : echo_count_array_t;
        fall_count  : echo_count_array_t;
        total_rise  : echo_total_count_t;
        total_fall  : echo_total_count_t;
    end record echo_shot_snapshot_t;

    type echo_diagnostics_t is record
        window_active         : std_logic;
        simulation_active     : std_logic;
        outside_window_pulse  : std_logic;
        outside_window_sticky : std_logic;
        outside_window_count  : unsigned(31 downto 0);
        overlap_pulse         : std_logic;
        overlap_sticky        : std_logic;
        overlap_count         : unsigned(31 downto 0);
        profile_not_ready_pulse  : std_logic;
        profile_not_ready_sticky : std_logic;
        profile_not_ready_count  : unsigned(31 downto 0);
        snapshot              : echo_shot_snapshot_t;
    end record echo_diagnostics_t;

    constant C_ECHO_DELAY_CLOCKS_CLEAR : echo_delay_clocks_array_t :=
        (others => (others => '0'));
    constant C_ECHO_COUNTS_CLEAR : echo_count_array_t :=
        (others => (others => '0'));

    constant C_ECHO_SHOT_SNAPSHOT_CLEAR : echo_shot_snapshot_t := (
        valid      => '0',
        request    => C_SHOT_REQUEST_IDLE,
        timeout    => '0',
        aborted    => '0',
        rise_mask  => (others => '0'),
        fall_mask  => (others => '0'),
        rise_count => C_ECHO_COUNTS_CLEAR,
        fall_count => C_ECHO_COUNTS_CLEAR,
        total_rise => (others => '0'),
        total_fall => (others => '0')
    );

    constant C_ECHO_DIAGNOSTICS_CLEAR : echo_diagnostics_t := (
        window_active         => '0',
        simulation_active     => '0',
        outside_window_pulse  => '0',
        outside_window_sticky => '0',
        outside_window_count  => (others => '0'),
        overlap_pulse         => '0',
        overlap_sticky        => '0',
        overlap_count         => (others => '0'),
        profile_not_ready_pulse  => '0',
        profile_not_ready_sticky => '0',
        profile_not_ready_count  => (others => '0'),
        snapshot              => C_ECHO_SHOT_SNAPSHOT_CLEAR
    );

    function fn_echo_channel_count(
        build_config : lidar_build_config_t
    ) return positive;

    function fn_echo_channel_index(
        chip_index      : natural;
        stop_index      : natural;
        stops_per_chip  : positive
    ) return natural;

    function fn_echo_ticks_to_proc_clocks(
        ticks_5ns : echo_delay_ticks_t;
        clock_mhz : positive
    ) return echo_delay_clocks_t;

end package lidar_echo_pkg;

package body lidar_echo_pkg is

    function fn_echo_channel_count(
        build_config : lidar_build_config_t
    ) return positive is
    begin
        return build_config.num_chips * build_config.stops_per_chip;
    end function fn_echo_channel_count;

    function fn_echo_channel_index(
        chip_index      : natural;
        stop_index      : natural;
        stops_per_chip  : positive
    ) return natural is
    begin
        return chip_index * stops_per_chip + stop_index;
    end function fn_echo_channel_index;

    function fn_echo_ticks_to_proc_clocks(
        ticks_5ns : echo_delay_ticks_t;
        clock_mhz : positive
    ) return echo_delay_clocks_t is
        variable ticks_ext : unsigned(C_ECHO_DELAY_WIDTH + 2 downto 0);
        variable scaled    : unsigned(C_ECHO_DELAY_WIDTH + 2 downto 0);
    begin
        ticks_ext := resize(ticks_5ns, ticks_ext'length);

        case clock_mhz is
            when 50 =>
                scaled := ticks_ext + to_unsigned(3, scaled'length);
                return resize(shift_right(scaled, 2), C_ECHO_DELAY_WIDTH);
            when 100 =>
                scaled := ticks_ext + to_unsigned(1, scaled'length);
                return resize(shift_right(scaled, 1), C_ECHO_DELAY_WIDTH);
            when 125 =>
                scaled := shift_left(ticks_ext, 2) + ticks_ext
                    + to_unsigned(7, scaled'length);
                return resize(shift_right(scaled, 3), C_ECHO_DELAY_WIDTH);
            when 150 =>
                scaled := shift_left(ticks_ext, 1) + ticks_ext
                    + to_unsigned(3, scaled'length);
                return resize(shift_right(scaled, 2), C_ECHO_DELAY_WIDTH);
            when 200 =>
                return ticks_5ns;
            when others =>
                return ticks_5ns;
        end case;
    end function fn_echo_ticks_to_proc_clocks;

end package body lidar_echo_pkg;
