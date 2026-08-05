library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

package lidar_gpx_pkg is

    constant C_GPX_BUS_DATA_WIDTH : positive := 28;
    constant C_GPX_BUS_ADDR_WIDTH : positive := 4;

    subtype gpx_bus_data_t is
        std_logic_vector(C_GPX_BUS_DATA_WIDTH - 1 downto 0);
    subtype gpx_bus_address_t is
        std_logic_vector(C_GPX_BUS_ADDR_WIDTH - 1 downto 0);

    type gpx_bus_timing_t is record
        clock_div : unsigned(5 downto 0);
        ticks     : unsigned(2 downto 0);
    end record gpx_bus_timing_t;

    type gpx_bus_request_t is record
        valid         : std_logic;
        write         : std_logic;
        address       : gpx_bus_address_t;
        write_data    : gpx_bus_data_t;
        oen_permanent : std_logic;
        burst         : std_logic;
    end record gpx_bus_request_t;

    type gpx_bus_response_t is record
        valid     : std_logic;
        write_ack : std_logic;
        address   : gpx_bus_address_t;
        read_data : gpx_bus_data_t;
    end record gpx_bus_response_t;

    type gpx_pin_status_t is record
        ef1     : std_logic;
        ef2     : std_logic;
        lf1     : std_logic;
        lf2     : std_logic;
        irflag  : std_logic;
        errflag : std_logic;
    end record gpx_pin_status_t;

    -- Typed v2 boundary for the 16 GPX registers. No default image is copied
    -- here: the acquisition integration layer converts the single board-
    -- proven c_GPX_DEFAULT_IMAGE source when it builds its reset image.
    type gpx_register_image_t is array (0 to 15) of
        std_logic_vector(31 downto 0);

    -- Live lane observations. Control-ready signals remain explicit ports so
    -- they cannot be confused with diagnostic readback.
    type gpx_lane_status_t is record
        initialized       : std_logic;
        run_active        : std_logic;
        shot_outstanding  : std_logic;
        controller_busy   : std_logic;
        bus_busy          : std_logic;
        response_pending  : std_logic;
        pin_status        : gpx_pin_status_t;
        effective_ticks   : unsigned(2 downto 0);
        chip_shot_seq     : unsigned(15 downto 0);
    end record gpx_lane_status_t;

    -- Names ending in _pulse are one-clock observations. The remaining error
    -- fields are sticky in the proven controller and clear only through reset
    -- or the explicit status-clear command.
    type gpx_lane_faults_t is record
        drain_timeout_pulse       : std_logic;
        sequence_pulse            : std_logic;
        response_mismatch_sticky  : std_logic;
        raw_drop_sticky           : std_logic;
        drain_cap_sticky          : std_logic;
        register_overflow_sticky  : std_logic;
        run_timeout_pulse         : std_logic;
        run_timeout_cause         : std_logic_vector(2 downto 0);
        init_cfg_coalesced_sticky : std_logic;
        command_collision_sticky  : std_logic;
        bus_fatal_sticky          : std_logic;
    end record gpx_lane_faults_t;

    -- Fixed maximum-size arrays keep the internal v2 boundary strongly typed.
    -- The public IP wrapper may flatten only G_BUILD_CONFIG.num_chips lanes.
    type gpx_bus_data_array_t is array (0 to C_MAX_CHIPS - 1) of
        gpx_bus_data_t;
    type gpx_bus_address_array_t is array (0 to C_MAX_CHIPS - 1) of
        gpx_bus_address_t;
    type gpx_lane_status_array_t is array (0 to C_MAX_CHIPS - 1) of
        gpx_lane_status_t;
    type gpx_lane_faults_array_t is array (0 to C_MAX_CHIPS - 1) of
        gpx_lane_faults_t;

    constant C_GPX_BUS_TIMING_DEFAULT : gpx_bus_timing_t := (
        clock_div => to_unsigned(2, 6),
        ticks     => to_unsigned(5, 3)
    );

    constant C_GPX_BUS_REQUEST_IDLE : gpx_bus_request_t := (
        valid         => '0',
        write         => '0',
        address       => (others => '0'),
        write_data    => (others => '0'),
        oen_permanent => '0',
        burst         => '0'
    );

    constant C_GPX_BUS_RESPONSE_IDLE : gpx_bus_response_t := (
        valid     => '0',
        write_ack => '0',
        address   => (others => '0'),
        read_data => (others => '0')
    );

    constant C_GPX_PIN_STATUS_RESET : gpx_pin_status_t := (
        ef1     => '1',
        ef2     => '1',
        lf1     => '0',
        lf2     => '0',
        irflag  => '0',
        errflag => '0'
    );

    constant C_GPX_LANE_STATUS_RESET : gpx_lane_status_t := (
        initialized      => '0',
        run_active       => '0',
        shot_outstanding => '0',
        controller_busy  => '1',
        bus_busy         => '0',
        response_pending => '0',
        pin_status       => C_GPX_PIN_STATUS_RESET,
        effective_ticks  => C_GPX_BUS_TIMING_DEFAULT.ticks,
        chip_shot_seq    => (others => '0')
    );

    constant C_GPX_LANE_FAULTS_CLEAR : gpx_lane_faults_t := (
        drain_timeout_pulse       => '0',
        sequence_pulse            => '0',
        response_mismatch_sticky  => '0',
        raw_drop_sticky           => '0',
        drain_cap_sticky          => '0',
        register_overflow_sticky  => '0',
        run_timeout_pulse         => '0',
        run_timeout_cause         => (others => '0'),
        init_cfg_coalesced_sticky => '0',
        command_collision_sticky  => '0',
        bus_fatal_sticky          => '0'
    );

end package lidar_gpx_pkg;
