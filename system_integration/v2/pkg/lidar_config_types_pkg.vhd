library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;

package lidar_config_types_pkg is

    constant C_MAX_CPR                 : positive := 8191;
    constant C_5NS_TICK_RATE_MHZ       : positive := 200;
    constant C_ANGLE_UDEG_PER_DEG      : positive := 1_000_000;
    constant C_FULL_MECHANICAL_UDEG    : positive := 360_000_000;
    constant C_MIRROR_OPTICAL_GAIN     : positive := 2;
    constant C_FULL_OPTICAL_SCALE_UDEG : positive :=
        C_FULL_MECHANICAL_UDEG * C_MIRROR_OPTICAL_GAIN;

    subtype u16_t is unsigned(15 downto 0);
    subtype u32_t is unsigned(31 downto 0);
    subtype angle_udeg_t is unsigned(29 downto 0);
    subtype capture_adjust_t is signed(16 downto 0);

    type motor_config_source_t is record
        cpr                    : u16_t;
        decode_mode            : decode_mode_t;
        direction              : direction_t;
        virtual_ticks_lo       : u32_t;
        virtual_hi_count       : unsigned(14 downto 0);
        z_offset               : position_t;
        z_width                : position_t;
        z_early                : std_logic;
        simulation_mode        : std_logic;
    end record motor_config_source_t;

    type mirror_config_source_t is record
        face_centers       : face_position_array_t;
        common_half_width  : position_t;
    end record mirror_config_source_t;

    type laser_config_source_t is record
        fire_width_5ns_ticks        : u16_t;
        fire_done_timeout_5ns_ticks : u16_t;
        target_range_window_5ns     : u32_t;
        start_width_5ns_ticks       : u16_t;
        stop_width_5ns_ticks        : u16_t;
        simulation_start_delay_5ns  : u32_t;
        optical_shot_interval_udeg  : angle_udeg_t;
        face_enable_mask            : face_mask_t;
    end record laser_config_source_t;

    type tdc_config_source_t is record
        bus_clk_div         : unsigned(5 downto 0);
        bus_ticks           : unsigned(5 downto 0);
        start_offset        : unsigned(17 downto 0);
        scan_timeout_5ns    : u32_t;
        active_chip_mask    : chip_mask_t;
        falling_enable     : std_logic;
        max_hits_per_stop  : unsigned(2 downto 0);
        capture_adjust_5ns : capture_adjust_t;
    end record tdc_config_source_t;

    type lidar_runtime_config_t is record
        motor  : motor_config_source_t;
        mirror : mirror_config_source_t;
        laser  : laser_config_source_t;
        tdc    : tdc_config_source_t;
    end record lidar_runtime_config_t;

    type lidar_derived_config_t is record
        total_states                    : u16_t;
        virtual_ticks_hi                : u32_t;
        mechanical_angle_per_state_udeg : angle_udeg_t;
        optical_angle_per_state_udeg    : angle_udeg_t;
        face_lower                      : face_position_array_t;
        face_upper                      : face_position_array_t;
        face_active_positions           : u16_t;
        face_angular_intervals          : u16_t;
        shot_interval_states            : u16_t;
        columns_per_face                : u16_t;
        present_chip_mask               : chip_mask_t;
        active_rise_mask                : chip_mask_t;
        active_fall_mask                : chip_mask_t;
        fire_width_proc_clks             : u32_t;
        fire_done_timeout_proc_clks      : u32_t;
        target_range_proc_clks          : u32_t;
        start_width_proc_clks            : u32_t;
        stop_width_proc_clks             : u32_t;
        simulation_start_delay_proc_clks : u32_t;
        capture_window_5ns              : u32_t;
        capture_window_tdc_clks         : u32_t;
        scan_timeout_tdc_clks            : u32_t;
    end record lidar_derived_config_t;

    type lidar_active_config_t is record
        version : u16_t;
        source  : lidar_runtime_config_t;
        derived : lidar_derived_config_t;
    end record lidar_active_config_t;

    constant C_DEFAULT_FACE_CENTERS : face_position_array_t := (
        0 => to_unsigned(1440, C_POSITION_WIDTH),
        1 => to_unsigned(4320, C_POSITION_WIDTH),
        2 => to_unsigned(7200, C_POSITION_WIDTH),
        3 => to_unsigned(10080, C_POSITION_WIDTH),
        4 => to_unsigned(12960, C_POSITION_WIDTH)
    );

    constant C_DEFAULT_RUNTIME_CONFIG : lidar_runtime_config_t := (
        motor => (
            cpr                   => to_unsigned(3600, 16),
            decode_mode           => DECODE_X4,
            direction             => DIRECTION_CW,
            virtual_ticks_lo      => to_unsigned(520, 32),
            virtual_hi_count      => to_unsigned(12000, 15),
            z_offset              => (others => '0'),
            z_width               => (others => '0'),
            z_early               => '0',
            simulation_mode       => '0'
        ),
        mirror => (
            face_centers      => C_DEFAULT_FACE_CENTERS,
            common_half_width => to_unsigned(1200, C_POSITION_WIDTH)
        ),
        laser => (
            fire_width_5ns_ticks        => to_unsigned(13, 16),
            fire_done_timeout_5ns_ticks => to_unsigned(288, 16),
            target_range_window_5ns     => to_unsigned(288, 32),
            start_width_5ns_ticks       => to_unsigned(5, 16),
            stop_width_5ns_ticks        => to_unsigned(5, 16),
            simulation_start_delay_5ns  => to_unsigned(133, 32),
            optical_shot_interval_udeg  => to_unsigned(50_000, 30),
            face_enable_mask            => (others => '1')
        ),
        tdc => (
            bus_clk_div         => to_unsigned(2, 6),
            bus_ticks           => to_unsigned(5, 6),
            start_offset        => to_unsigned(1234, 18),
            scan_timeout_5ns    => (others => '0'),
            active_chip_mask    => (others => '1'),
            falling_enable     => '1',
            max_hits_per_stop  => to_unsigned(7, 3),
            capture_adjust_5ns => (others => '0')
        )
    );

    -- Produces a legal reset shadow for the selected build topology.  Runtime
    -- timing defaults remain common, while Face centers/masks and the active
    -- chip/slope defaults are derived from synthesis-time capabilities.
    function fn_default_runtime_config(
        build_cfg : lidar_build_config_t
    ) return lidar_runtime_config_t;

end package lidar_config_types_pkg;

package body lidar_config_types_pkg is

    function fn_default_runtime_config(
        build_cfg : lidar_build_config_t
    ) return lidar_runtime_config_t is
        constant C_DEFAULT_TOTAL_STATES : positive :=
            3600 * fn_decode_multiplier(DECODE_X4);
        variable result       : lidar_runtime_config_t :=
            C_DEFAULT_RUNTIME_CONFIG;
        variable center_value : natural;
        variable zero_mask    : chip_mask_t := (others => '0');
    begin
        result.laser.face_enable_mask := fn_active_face_mask(
            build_cfg.num_faces);
        result.tdc.active_chip_mask := fn_present_chip_mask(
            build_cfg.num_chips);

        if build_cfg.fall_capability_mask = zero_mask then
            result.tdc.falling_enable := '0';
        else
            result.tdc.falling_enable := '1';
        end if;

        result.mirror.face_centers := (others => (others => '0'));
        if build_cfg.num_faces >= 1 and build_cfg.num_faces <= C_MAX_FACES then
            for face_index in 0 to build_cfg.num_faces - 1 loop
                center_value := ((2 * face_index + 1)
                    * C_DEFAULT_TOTAL_STATES) / (2 * build_cfg.num_faces);
                result.mirror.face_centers(face_index) := to_unsigned(
                    center_value, C_POSITION_WIDTH);
            end loop;
        end if;

        return result;
    end function fn_default_runtime_config;

end package body lidar_config_types_pkg;
