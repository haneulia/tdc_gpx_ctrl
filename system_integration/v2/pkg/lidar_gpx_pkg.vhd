library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

-- 외부 TDC-GPX Chip bus와 acquisition lane의 타입/용량 계약.
-- 물리 bus는 주소 4 bit, 데이터 28 bit이고 최대 배열 크기는 Build 패키지의
-- C_MAX_CHIPS를 따른다. runtime Return 수는 출력 가시성만 바꾸며 물리 IFIFO
-- drain 용량과 FIFO sizing 상한은 합성 topology에서 계산한다.
package lidar_gpx_pkg is

    constant C_GPX_BUS_DATA_WIDTH : positive := 28;
    constant C_GPX_BUS_ADDR_WIDTH : positive := 4;
    constant C_GPX_REGISTER_COUNT : positive := 16;

    -- Compile-time acquisition capacities. Each physical IFIFO owns at most
    -- four STOP channels. A chip that can be active in both slope lanes needs
    -- twice the Return capacity of a dedicated-slope chip. These bounds are
    -- derived from immutable topology, never from runtime formatter settings.
    function fn_gpx_drain_cap_quads(
        build_cfg  : lidar_build_config_t;
        chip_index : natural
    ) return positive;
    function fn_gpx_events_per_shot_capacity(
        build_cfg : lidar_build_config_t
    ) return positive;
    function fn_gpx_result_fifo_depth(
        build_cfg : lidar_build_config_t
    ) return positive;

    subtype gpx_bus_data_t is
        std_logic_vector(C_GPX_BUS_DATA_WIDTH - 1 downto 0);
    subtype gpx_bus_address_t is
        std_logic_vector(C_GPX_BUS_ADDR_WIDTH - 1 downto 0);

    -- 외부 TDC-GPX 물리 레지스터 읽기 서비스 계약.
    -- request.valid/ready 한 번이 정확히 한 Chip의 한 Register 읽기를
    -- 의미한다. response는 소비될 때까지 valid와 payload를 유지하므로
    -- 진단 경로의 backpressure가 1-clock read pulse를 잃게 하지 않는다.
    subtype gpx_chip_select_t is unsigned(1 downto 0);
    type gpx_register_read_request_t is record
        valid   : std_logic;
        chip    : gpx_chip_select_t;
        address : gpx_bus_address_t;
    end record gpx_register_read_request_t;

    type gpx_register_read_response_t is record
        valid     : std_logic;
        error     : std_logic;
        chip      : gpx_chip_select_t;
        address   : gpx_bus_address_t;
        read_data : gpx_bus_data_t;
    end record gpx_register_read_response_t;

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
    type gpx_register_image_t is array (0 to C_GPX_REGISTER_COUNT - 1) of
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
    type gpx_register_read_request_array_t is array (
        0 to C_MAX_CHIPS - 1) of gpx_register_read_request_t;
    type gpx_register_read_response_array_t is array (
        0 to C_MAX_CHIPS - 1) of gpx_register_read_response_t;

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

    constant C_GPX_REGISTER_READ_REQUEST_IDLE :
        gpx_register_read_request_t := (
            valid   => '0',
            chip    => (others => '0'),
            address => (others => '0')
        );

    constant C_GPX_REGISTER_READ_RESPONSE_IDLE :
        gpx_register_read_response_t := (
            valid     => '0',
            error     => '0',
            chip      => (others => '0'),
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

package body lidar_gpx_pkg is

    function fn_chip_slope_capacity(
        build_cfg  : lidar_build_config_t;
        chip_index : natural
    ) return positive is
        variable result : natural := 0;
    begin
        assert chip_index < build_cfg.num_chips
            report "lidar_gpx_pkg: capacity requested for absent GPX chip"
            severity failure;

        if build_cfg.rise_capability_mask(chip_index) = '1' then
            result := result + 1;
        end if;
        if build_cfg.fall_capability_mask(chip_index) = '1' then
            result := result + 1;
        end if;

        assert result > 0
            report "lidar_gpx_pkg: GPX chip has no slope capability"
            severity failure;
        if result = 0 then
            return 1;
        end if;
        return result;
    end function fn_chip_slope_capacity;

    function fn_div_ceil(
        value   : positive;
        divisor : positive
    ) return positive is
    begin
        return (value + divisor - 1) / divisor;
    end function fn_div_ceil;

    function fn_power_of_two_ceil(value : positive) return positive is
        variable result : positive := 1;
    begin
        while result < value loop
            result := result * 2;
        end loop;
        return result;
    end function fn_power_of_two_ceil;

    function fn_gpx_drain_cap_quads(
        build_cfg  : lidar_build_config_t;
        chip_index : natural
    ) return positive is
        variable ififo_stop_channels : positive;
        variable slope_capacity      : positive;
    begin
        if build_cfg.stops_per_chip > 4 then
            ififo_stop_channels := 4;
        else
            ififo_stop_channels := build_cfg.stops_per_chip;
        end if;
        slope_capacity := fn_chip_slope_capacity(build_cfg, chip_index);

        return fn_div_ceil(
            ififo_stop_channels * build_cfg.max_returns_per_stop *
                slope_capacity,
            4);
    end function fn_gpx_drain_cap_quads;

    function fn_gpx_events_per_shot_capacity(
        build_cfg : lidar_build_config_t
    ) return positive is
        variable events_total : natural := 0;
    begin
        for chip_index in 0 to build_cfg.num_chips - 1 loop
            -- The legacy GPX controller applies one four-word-unit cap to
            -- both IFIFOs. Size for the rounded physical boundary plus the
            -- IFIFO1_DONE and final TERMINAL control events.
            events_total := events_total +
                2 * 4 * fn_gpx_drain_cap_quads(build_cfg, chip_index) + 2;
        end loop;
        return events_total;
    end function fn_gpx_events_per_shot_capacity;

    function fn_gpx_result_fifo_depth(
        build_cfg : lidar_build_config_t
    ) return positive is
        variable required_depth : positive;
    begin
        required_depth := fn_power_of_two_ceil(
            fn_gpx_events_per_shot_capacity(build_cfg));
        if required_depth < 16 then
            return 16;
        end if;
        return required_depth;
    end function fn_gpx_result_fifo_depth;

end package body lidar_gpx_pkg;
