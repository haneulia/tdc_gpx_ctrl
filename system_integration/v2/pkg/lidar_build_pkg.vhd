library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package lidar_build_pkg is

    constant C_MAX_CHIPS           : positive := 4;
    constant C_MAX_STOPS_PER_CHIP  : positive := 8;
    constant C_MAX_RETURNS_PER_STOP : positive := 7;
    constant C_MAX_FACES           : positive := 5;
    constant C_POSITION_WIDTH      : positive := 15;
    -- The proven GPX acquisition watchdog and per-shot capture budget are
    -- 16 bit. Commit validation rejects larger derived TDC-clock windows so
    -- an accepted runtime value can never be truncated at the v1 boundary.
    constant C_GPX_CAPTURE_COUNTER_MAX_CLKS : positive := 65_535;

    subtype chip_mask_t is std_logic_vector(C_MAX_CHIPS - 1 downto 0);
    subtype face_mask_t is std_logic_vector(C_MAX_FACES - 1 downto 0);
    subtype position_t is unsigned(C_POSITION_WIDTH - 1 downto 0);
    type face_position_array_t is array (0 to C_MAX_FACES - 1) of position_t;

    type stream_clock_mode_t is (
        STREAM_CLOCK_ASYNC,
        STREAM_CLOCK_SYNC
    );

    type decode_mode_t is (
        DECODE_X1,
        DECODE_X2,
        DECODE_X4
    );

    type direction_t is (
        DIRECTION_CW,
        DIRECTION_CCW
    );

    type lidar_build_config_t is record
        proc_clk_mhz        : natural;
        tdc_clk_mhz         : natural;
        stream_clock_mode   : stream_clock_mode_t;
        num_chips           : natural;
        stops_per_chip      : natural;
        max_returns_per_stop : natural;
        rise_capability_mask : chip_mask_t;
        fall_capability_mask : chip_mask_t;
        output_width        : natural;
        num_faces           : natural;
        enable_echo_receiver : boolean;
        enable_echo_simulation : boolean;
    end record lidar_build_config_t;

    type lidar_cfg_error_t is (
        CFG_OK,
        CFG_BUILD_PROC_CLOCK,
        CFG_BUILD_TDC_CLOCK,
        CFG_BUILD_SYNC_CLOCK_MISMATCH,
        CFG_BUILD_CHIP_COUNT,
        CFG_BUILD_STOP_COUNT,
        CFG_BUILD_RETURN_COUNT,
        CFG_BUILD_SLOPE_MASK_UNKNOWN,
        CFG_BUILD_SLOPE_MASK_OUTSIDE_CHIPS,
        CFG_BUILD_SLOPE_ROLE_MISSING,
        CFG_BUILD_SLOPE_BALANCE,
        CFG_BUILD_OUTPUT_WIDTH,
        CFG_BUILD_FACE_COUNT,
        CFG_BUILD_ECHO_SIM_WITHOUT_ECHO,
        CFG_RUNTIME_CPR,
        CFG_RUNTIME_TOTAL_STATES,
        CFG_RUNTIME_VIRTUAL_TICKS,
        CFG_RUNTIME_VIRTUAL_HI_COUNT,
        CFG_RUNTIME_Z_PARAM,
        CFG_RUNTIME_FACE_CENTER,
        CFG_RUNTIME_FACE_HALF_WIDTH,
        CFG_RUNTIME_FACE_OVERLAP,
        CFG_RUNTIME_FACE_ENABLE_MASK,
        CFG_RUNTIME_SHOT_ANGLE,
        CFG_RUNTIME_SHOT_BELOW_ONE_STATE,
        CFG_RUNTIME_FIRE_WIDTH,
        CFG_RUNTIME_FIRE_TIMEOUT,
        CFG_RUNTIME_RANGE_WINDOW,
        CFG_RUNTIME_CAPTURE_WINDOW,
        CFG_RUNTIME_ACTIVE_CHIP_MASK,
        CFG_RUNTIME_MAX_HITS,
        CFG_RUNTIME_BUS_TIMING,
        CFG_RUNTIME_SOURCE_MODE,
        CFG_INTERNAL_ARITHMETIC,
        CFG_TRANSACTION_BUSY,
        CFG_TRANSACTION_PREPARE_TIMEOUT,
        CFG_TRANSACTION_GATEWAY_PROTOCOL,
        CFG_TRANSACTION_ACTIVATE_TIMEOUT,
        CFG_TRANSACTION_RELEASE_TIMEOUT,
        CFG_TRANSACTION_CLEAR_TIMEOUT
    );

    constant C_DEFAULT_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz          => 150,
        tdc_clk_mhz           => 200,
        stream_clock_mode     => STREAM_CLOCK_ASYNC,
        num_chips             => 4,
        stops_per_chip        => 8,
        max_returns_per_stop  => 7,
        rise_capability_mask  => "0011",
        fall_capability_mask  => "1100",
        output_width          => 32,
        num_faces             => 5,
        enable_echo_receiver  => true,
        enable_echo_simulation => false
    );

    function fn_is_legal_clock_mhz(value : natural) return boolean;
    function fn_is_legal_output_width(value : natural) return boolean;
    function fn_is_binary(value : std_logic_vector) return boolean;
    function fn_popcount(value : std_logic_vector) return natural;
    function fn_present_chip_mask(num_chips : natural) return chip_mask_t;
    function fn_active_face_mask(num_faces : natural) return face_mask_t;
    function fn_decode_multiplier(mode : decode_mode_t) return positive;
    function fn_validate_build_config(
        cfg : lidar_build_config_t
    ) return lidar_cfg_error_t;
    function fn_cfg_error_code(
        error_value : lidar_cfg_error_t
    ) return std_logic_vector;

end package lidar_build_pkg;

package body lidar_build_pkg is

    function fn_is_legal_clock_mhz(value : natural) return boolean is
    begin
        return value = 50 or value = 100 or value = 125
            or value = 150 or value = 200;
    end function fn_is_legal_clock_mhz;

    function fn_is_legal_output_width(value : natural) return boolean is
    begin
        return value = 32 or value = 64 or value = 128;
    end function fn_is_legal_output_width;

    function fn_is_binary(value : std_logic_vector) return boolean is
    begin
        for index in value'range loop
            if value(index) /= '0' and value(index) /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_is_binary;

    function fn_popcount(value : std_logic_vector) return natural is
        variable result : natural := 0;
    begin
        for index in value'range loop
            if value(index) = '1' then
                result := result + 1;
            end if;
        end loop;
        return result;
    end function fn_popcount;

    function fn_present_chip_mask(num_chips : natural) return chip_mask_t is
        variable result : chip_mask_t := (others => '0');
    begin
        if num_chips >= 1 and num_chips <= C_MAX_CHIPS then
            for index in 0 to num_chips - 1 loop
                result(index) := '1';
            end loop;
        end if;
        return result;
    end function fn_present_chip_mask;

    function fn_active_face_mask(num_faces : natural) return face_mask_t is
        variable result : face_mask_t := (others => '0');
    begin
        if num_faces >= 1 and num_faces <= C_MAX_FACES then
            for index in 0 to num_faces - 1 loop
                result(index) := '1';
            end loop;
        end if;
        return result;
    end function fn_active_face_mask;

    function fn_decode_multiplier(mode : decode_mode_t) return positive is
    begin
        case mode is
            when DECODE_X1 => return 1;
            when DECODE_X2 => return 2;
            when DECODE_X4 => return 4;
        end case;
    end function fn_decode_multiplier;

    function fn_validate_build_config(
        cfg : lidar_build_config_t
    ) return lidar_cfg_error_t is
        variable present_mask : chip_mask_t;
        variable zero_mask    : chip_mask_t := (others => '0');
    begin
        if not fn_is_legal_clock_mhz(cfg.proc_clk_mhz) then
            return CFG_BUILD_PROC_CLOCK;
        elsif not fn_is_legal_clock_mhz(cfg.tdc_clk_mhz) then
            return CFG_BUILD_TDC_CLOCK;
        elsif cfg.stream_clock_mode = STREAM_CLOCK_SYNC
              and cfg.proc_clk_mhz /= cfg.tdc_clk_mhz then
            return CFG_BUILD_SYNC_CLOCK_MISMATCH;
        elsif cfg.num_chips < 1 or cfg.num_chips > C_MAX_CHIPS then
            return CFG_BUILD_CHIP_COUNT;
        elsif cfg.stops_per_chip < 2
              or cfg.stops_per_chip > C_MAX_STOPS_PER_CHIP then
            return CFG_BUILD_STOP_COUNT;
        elsif cfg.max_returns_per_stop < 1
              or cfg.max_returns_per_stop > C_MAX_RETURNS_PER_STOP then
            return CFG_BUILD_RETURN_COUNT;
        elsif not fn_is_binary(cfg.rise_capability_mask)
              or not fn_is_binary(cfg.fall_capability_mask) then
            return CFG_BUILD_SLOPE_MASK_UNKNOWN;
        end if;

        present_mask := fn_present_chip_mask(cfg.num_chips);
        if ((cfg.rise_capability_mask or cfg.fall_capability_mask)
            and not present_mask) /= zero_mask then
            return CFG_BUILD_SLOPE_MASK_OUTSIDE_CHIPS;
        elsif (cfg.rise_capability_mask or cfg.fall_capability_mask)
              /= present_mask then
            return CFG_BUILD_SLOPE_ROLE_MISSING;
        elsif fn_popcount(cfg.rise_capability_mask)
              < fn_popcount(cfg.fall_capability_mask) then
            return CFG_BUILD_SLOPE_BALANCE;
        elsif not fn_is_legal_output_width(cfg.output_width) then
            return CFG_BUILD_OUTPUT_WIDTH;
        elsif cfg.num_faces < 1 or cfg.num_faces > C_MAX_FACES then
            return CFG_BUILD_FACE_COUNT;
        elsif cfg.enable_echo_simulation and not cfg.enable_echo_receiver then
            return CFG_BUILD_ECHO_SIM_WITHOUT_ECHO;
        end if;

        return CFG_OK;
    end function fn_validate_build_config;

    function fn_cfg_error_code(
        error_value : lidar_cfg_error_t
    ) return std_logic_vector is
        variable result : unsigned(7 downto 0);
    begin
        case error_value is
            when CFG_OK                            => result := x"00";
            when CFG_BUILD_PROC_CLOCK              => result := x"01";
            when CFG_BUILD_TDC_CLOCK               => result := x"02";
            when CFG_BUILD_SYNC_CLOCK_MISMATCH     => result := x"03";
            when CFG_BUILD_CHIP_COUNT              => result := x"04";
            when CFG_BUILD_STOP_COUNT              => result := x"05";
            when CFG_BUILD_RETURN_COUNT            => result := x"06";
            when CFG_BUILD_SLOPE_MASK_UNKNOWN      => result := x"07";
            when CFG_BUILD_SLOPE_MASK_OUTSIDE_CHIPS => result := x"08";
            when CFG_BUILD_SLOPE_ROLE_MISSING      => result := x"09";
            when CFG_BUILD_SLOPE_BALANCE           => result := x"0A";
            when CFG_BUILD_OUTPUT_WIDTH            => result := x"0B";
            when CFG_BUILD_FACE_COUNT              => result := x"0C";
            when CFG_BUILD_ECHO_SIM_WITHOUT_ECHO   => result := x"0D";
            when CFG_RUNTIME_CPR                    => result := x"20";
            when CFG_RUNTIME_TOTAL_STATES           => result := x"21";
            when CFG_RUNTIME_VIRTUAL_TICKS          => result := x"22";
            when CFG_RUNTIME_VIRTUAL_HI_COUNT       => result := x"23";
            when CFG_RUNTIME_Z_PARAM                 => result := x"24";
            when CFG_RUNTIME_FACE_CENTER            => result := x"25";
            when CFG_RUNTIME_FACE_HALF_WIDTH        => result := x"26";
            when CFG_RUNTIME_FACE_OVERLAP           => result := x"27";
            when CFG_RUNTIME_FACE_ENABLE_MASK       => result := x"28";
            when CFG_RUNTIME_SHOT_ANGLE             => result := x"29";
            when CFG_RUNTIME_SHOT_BELOW_ONE_STATE   => result := x"2A";
            when CFG_RUNTIME_FIRE_WIDTH             => result := x"2B";
            when CFG_RUNTIME_FIRE_TIMEOUT           => result := x"2C";
            when CFG_RUNTIME_RANGE_WINDOW           => result := x"2D";
            when CFG_RUNTIME_CAPTURE_WINDOW         => result := x"2E";
            when CFG_RUNTIME_ACTIVE_CHIP_MASK       => result := x"2F";
            when CFG_RUNTIME_MAX_HITS               => result := x"30";
            when CFG_RUNTIME_BUS_TIMING             => result := x"31";
            when CFG_RUNTIME_SOURCE_MODE            => result := x"32";
            when CFG_INTERNAL_ARITHMETIC             => result := x"70";
            when CFG_TRANSACTION_BUSY                => result := x"71";
            when CFG_TRANSACTION_PREPARE_TIMEOUT     => result := x"72";
            when CFG_TRANSACTION_GATEWAY_PROTOCOL   => result := x"73";
            when CFG_TRANSACTION_ACTIVATE_TIMEOUT    => result := x"74";
            when CFG_TRANSACTION_RELEASE_TIMEOUT     => result := x"75";
            when CFG_TRANSACTION_CLEAR_TIMEOUT       => result := x"76";
        end case;
        return std_logic_vector(result);
    end function fn_cfg_error_code;

end package body lidar_build_pkg;
