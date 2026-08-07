library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

-- Reference oracle for configuration validation and derived-value arithmetic.
--
-- This package intentionally uses wide division to state the exact contract in
-- one readable place. It belongs to package tests and equivalence checking; it
-- must not be called from a production combinational or clocked datapath. The
-- synthesizable commit calculator implements the same contract sequentially.
package lidar_config_reference_pkg is

    function fn_total_states(
        cpr         : u16_t;
        decode_mode : decode_mode_t
    ) return natural;

    function fn_face_lower(
        center       : natural;
        half_width   : natural;
        total_states : positive
    ) return natural;

    function fn_face_upper(
        center       : natural;
        half_width   : natural;
        total_states : positive
    ) return natural;

    function fn_shot_interval_states(
        optical_interval_udeg : angle_udeg_t;
        total_states          : positive
    ) return positive;

    function fn_columns_per_face(
        angular_intervals : natural;
        shot_interval     : positive
    ) return natural;

    function fn_ticks_to_clocks(
        ticks_5ns : u32_t;
        clock_mhz : positive
    ) return u32_t;

    function fn_gpx_mtimer_ref_ticks(
        target_range_5ns : u32_t
    ) return u32_t;

    function fn_gpx_effective_target_range_5ns(
        target_range_5ns : u32_t
    ) return u32_t;

    function fn_validate_runtime_config(
        build_cfg   : lidar_build_config_t;
        runtime_cfg : lidar_runtime_config_t
    ) return lidar_cfg_error_t;

    function fn_derive_runtime_config(
        build_cfg   : lidar_build_config_t;
        runtime_cfg : lidar_runtime_config_t
    ) return lidar_derived_config_t;

end package lidar_config_reference_pkg;

package body lidar_config_reference_pkg is

    function fn_total_states(
        cpr         : u16_t;
        decode_mode : decode_mode_t
    ) return natural is
    begin
        return to_integer(cpr) * fn_decode_multiplier(decode_mode);
    end function fn_total_states;

    function fn_face_lower(
        center       : natural;
        half_width   : natural;
        total_states : positive
    ) return natural is
    begin
        if center >= half_width then
            return center - half_width;
        end if;
        return total_states - (half_width - center);
    end function fn_face_lower;

    function fn_face_upper(
        center       : natural;
        half_width   : natural;
        total_states : positive
    ) return natural is
        variable result : natural;
    begin
        result := center + half_width;
        if result >= total_states then
            result := result - total_states;
        end if;
        return result;
    end function fn_face_upper;

    function fn_shot_interval_states(
        optical_interval_udeg : angle_udeg_t;
        total_states          : positive
    ) return positive is
        variable product     : unsigned(63 downto 0);
        variable denominator : unsigned(63 downto 0);
        variable quotient    : unsigned(63 downto 0);
        variable remainder_v : unsigned(63 downto 0);
        variable result      : natural;
    begin
        product := resize(optical_interval_udeg, 32)
            * to_unsigned(total_states, 32);
        denominator := to_unsigned(C_FULL_OPTICAL_SCALE_UDEG, 64);
        quotient := product / denominator;
        remainder_v := product mod denominator;
        if remainder_v /= 0 then
            quotient := quotient + 1;
        end if;
        result := to_integer(quotient(30 downto 0));
        if result = 0 then
            return 1;
        end if;
        return result;
    end function fn_shot_interval_states;

    function fn_columns_per_face(
        angular_intervals : natural;
        shot_interval     : positive
    ) return natural is
    begin
        if angular_intervals = 0 then
            return 0;
        end if;
        return (angular_intervals + shot_interval - 1) / shot_interval;
    end function fn_columns_per_face;

    function fn_ticks_to_clocks(
        ticks_5ns : u32_t;
        clock_mhz : positive
    ) return u32_t is
        variable product     : unsigned(63 downto 0);
        variable denominator : unsigned(63 downto 0);
        variable quotient    : unsigned(63 downto 0);
    begin
        product := ticks_5ns * to_unsigned(clock_mhz, 32);
        denominator := to_unsigned(C_5NS_TICK_RATE_MHZ, 64);
        quotient := product / denominator;
        if product mod denominator /= 0 then
            quotient := quotient + 1;
        end if;
        return quotient(31 downto 0);
    end function fn_ticks_to_clocks;

    function fn_gpx_mtimer_ref_ticks(
        target_range_5ns : u32_t
    ) return u32_t is
        variable quotient  : u32_t;
        constant DIVISOR_C : u32_t := to_unsigned(
            C_GPX_REFERENCE_TICK_5NS, 32);
    begin
        quotient := target_range_5ns / DIVISOR_C;
        if target_range_5ns mod DIVISOR_C /= 0 then
            quotient := quotient + 1;
        end if;
        return quotient;
    end function fn_gpx_mtimer_ref_ticks;

    function fn_gpx_effective_target_range_5ns(
        target_range_5ns : u32_t
    ) return u32_t is
        variable mtimer : u32_t;
    begin
        mtimer := fn_gpx_mtimer_ref_ticks(target_range_5ns);
        return shift_left(mtimer, 2) + mtimer;
    end function fn_gpx_effective_target_range_5ns;

    function fn_circular_distance(
        left_value  : natural;
        right_value : natural;
        modulus     : positive
    ) return natural is
        variable direct_distance : natural;
    begin
        if left_value >= right_value then
            direct_distance := left_value - right_value;
        else
            direct_distance := right_value - left_value;
        end if;
        if direct_distance > modulus - direct_distance then
            return modulus - direct_distance;
        end if;
        return direct_distance;
    end function fn_circular_distance;

    function fn_capture_window_ext(
        runtime_cfg : lidar_runtime_config_t
    ) return signed is
        variable range_ext  : signed(33 downto 0);
        variable adjust_ext : signed(33 downto 0);
    begin
        range_ext := signed(resize(fn_gpx_effective_target_range_5ns(
            runtime_cfg.laser.target_range_window_5ns), 34));
        adjust_ext := resize(runtime_cfg.tdc.capture_adjust_5ns, 34);
        return range_ext + adjust_ext;
    end function fn_capture_window_ext;

    function fn_validate_runtime_config(
        build_cfg   : lidar_build_config_t;
        runtime_cfg : lidar_runtime_config_t
    ) return lidar_cfg_error_t is
        variable build_error       : lidar_cfg_error_t;
        variable total_states      : natural;
        variable half_width        : natural;
        variable center_left       : natural;
        variable center_right      : natural;
        variable circular_distance : natural;
        variable angle_product     : unsigned(63 downto 0);
        variable present_mask      : chip_mask_t;
        variable allowed_faces     : face_mask_t;
        variable zero_chip_mask    : chip_mask_t := (others => '0');
        variable zero_face_mask    : face_mask_t := (others => '0');
        variable capture_window    : signed(33 downto 0);
        variable capture_clock_product : unsigned(63 downto 0);
        variable effective_rise    : chip_mask_t;
        variable effective_fall    : chip_mask_t;
        variable mtimer_ref_ticks  : u32_t;
    begin
        build_error := fn_validate_build_config(build_cfg);
        if build_error /= CFG_OK then
            return build_error;
        end if;

        if to_integer(runtime_cfg.motor.cpr) < 1
           or to_integer(runtime_cfg.motor.cpr) > C_MAX_CPR then
            return CFG_RUNTIME_CPR;
        end if;

        total_states := fn_total_states(
            runtime_cfg.motor.cpr, runtime_cfg.motor.decode_mode);
        if total_states < 2 or total_states > (2 ** C_POSITION_WIDTH) - 1 then
            return CFG_RUNTIME_TOTAL_STATES;
        elsif runtime_cfg.motor.virtual_ticks_lo = 0
              or runtime_cfg.motor.virtual_ticks_lo = unsigned'(x"FFFFFFFF") then
            return CFG_RUNTIME_VIRTUAL_TICKS;
        elsif to_integer(runtime_cfg.motor.virtual_hi_count) > total_states then
            return CFG_RUNTIME_VIRTUAL_HI_COUNT;
        elsif to_integer(runtime_cfg.motor.z_offset) >= total_states
              or to_integer(runtime_cfg.motor.z_width) >= total_states then
            return CFG_RUNTIME_Z_PARAM;
        elsif runtime_cfg.motor.simulation_mode /= '0'
              and runtime_cfg.motor.simulation_mode /= '1' then
            return CFG_RUNTIME_SOURCE_MODE;
        end if;

        half_width := to_integer(runtime_cfg.mirror.common_half_width);
        if half_width = 0 or (2 * half_width) + 1 >= total_states then
            return CFG_RUNTIME_FACE_HALF_WIDTH;
        end if;

        for face_index in 0 to build_cfg.num_faces - 1 loop
            if to_integer(runtime_cfg.mirror.face_centers(face_index))
               >= total_states then
                return CFG_RUNTIME_FACE_CENTER;
            end if;
        end loop;

        if build_cfg.num_faces > 1 then
            for left_index in 0 to build_cfg.num_faces - 2 loop
                for right_index in left_index + 1 to build_cfg.num_faces - 1 loop
                    center_left := to_integer(
                        runtime_cfg.mirror.face_centers(left_index));
                    center_right := to_integer(
                        runtime_cfg.mirror.face_centers(right_index));
                    circular_distance := fn_circular_distance(
                        center_left, center_right, total_states);
                    if circular_distance <= 2 * half_width then
                        return CFG_RUNTIME_FACE_OVERLAP;
                    end if;
                end loop;
            end loop;
        end if;

        allowed_faces := fn_active_face_mask(build_cfg.num_faces);
        if not fn_is_binary(runtime_cfg.laser.face_enable_mask)
           or (runtime_cfg.laser.face_enable_mask and not allowed_faces)
              /= zero_face_mask
           or (runtime_cfg.laser.face_enable_mask and allowed_faces)
              = zero_face_mask then
            return CFG_RUNTIME_FACE_ENABLE_MASK;
        end if;

        if runtime_cfg.laser.optical_shot_interval_udeg = 0
           or to_integer(runtime_cfg.laser.optical_shot_interval_udeg)
              > C_FULL_MECHANICAL_UDEG then
            return CFG_RUNTIME_SHOT_ANGLE;
        end if;
        angle_product := resize(
            runtime_cfg.laser.optical_shot_interval_udeg, 32)
            * to_unsigned(total_states, 32);
        if angle_product < to_unsigned(C_FULL_OPTICAL_SCALE_UDEG, 64) then
            return CFG_RUNTIME_SHOT_BELOW_ONE_STATE;
        elsif runtime_cfg.laser.fire_width_5ns_ticks = 0
              or runtime_cfg.laser.start_width_5ns_ticks = 0
              or runtime_cfg.laser.stop_width_5ns_ticks = 0 then
            return CFG_RUNTIME_FIRE_WIDTH;
        elsif runtime_cfg.laser.target_range_window_5ns = 0 then
            return CFG_RUNTIME_RANGE_WINDOW;
        elsif runtime_cfg.laser.target_range_window_5ns > to_unsigned(
              C_GPX_MTIMER_MAX * C_GPX_REFERENCE_TICK_5NS, 32) then
            return CFG_RUNTIME_GPX_MTIMER_RANGE;
        elsif runtime_cfg.laser.fire_done_timeout_5ns_ticks = 0
              or resize(runtime_cfg.laser.fire_done_timeout_5ns_ticks, 32)
                 > runtime_cfg.laser.target_range_window_5ns then
            return CFG_RUNTIME_FIRE_TIMEOUT;
        end if;

        mtimer_ref_ticks := fn_gpx_mtimer_ref_ticks(
            runtime_cfg.laser.target_range_window_5ns);
        assert mtimer_ref_ticks <= to_unsigned(C_GPX_MTIMER_MAX, 32)
            report "V2-CFG-002 validated GPX MTimer exceeds Reg7 field"
            severity failure;

        capture_window := fn_capture_window_ext(runtime_cfg);
        if capture_window <= 0
           or capture_window(33 downto 32) /= "00" then
            return CFG_RUNTIME_CAPTURE_WINDOW;
        end if;
        capture_clock_product := resize(
            unsigned(capture_window(31 downto 0)), 32)
            * to_unsigned(build_cfg.tdc_clk_mhz, 32);
        if capture_clock_product > to_unsigned(
            C_GPX_CAPTURE_COUNTER_MAX_CLKS * C_5NS_TICK_RATE_MHZ,
            capture_clock_product'length) then
            return CFG_RUNTIME_CAPTURE_WINDOW;
        end if;

        present_mask := fn_present_chip_mask(build_cfg.num_chips);
        if not fn_is_binary(runtime_cfg.tdc.active_chip_mask)
           or (runtime_cfg.tdc.falling_enable /= '0'
               and runtime_cfg.tdc.falling_enable /= '1')
           or runtime_cfg.tdc.active_chip_mask = zero_chip_mask
           or (runtime_cfg.tdc.active_chip_mask and not present_mask)
              /= zero_chip_mask then
            return CFG_RUNTIME_ACTIVE_CHIP_MASK;
        end if;

        if runtime_cfg.tdc.falling_enable = '1' then
            effective_rise := runtime_cfg.tdc.active_chip_mask
                and build_cfg.rise_capability_mask;
            effective_fall := runtime_cfg.tdc.active_chip_mask
                and build_cfg.fall_capability_mask;
            if fn_popcount(effective_rise) < fn_popcount(effective_fall) then
                return CFG_RUNTIME_ACTIVE_CHIP_MASK;
            end if;
        end if;

        if to_integer(runtime_cfg.tdc.max_hits_per_stop) < 1
           or to_integer(runtime_cfg.tdc.max_hits_per_stop)
              > build_cfg.max_returns_per_stop then
            return CFG_RUNTIME_MAX_HITS;
        elsif runtime_cfg.tdc.bus_clk_div = 0
              or runtime_cfg.tdc.bus_ticks = 0
              or runtime_cfg.tdc.bus_ticks > 7 then
            return CFG_RUNTIME_BUS_TIMING;
        end if;

        return CFG_OK;
    end function fn_validate_runtime_config;

    function fn_derive_runtime_config(
        build_cfg   : lidar_build_config_t;
        runtime_cfg : lidar_runtime_config_t
    ) return lidar_derived_config_t is
        variable result            : lidar_derived_config_t;
        variable validation_error  : lidar_cfg_error_t;
        variable total_states      : positive;
        variable half_width        : natural;
        variable shot_interval     : positive;
        variable angular_intervals : natural;
        variable capture_window    : signed(33 downto 0);
        variable mtimer_ref_ticks  : u32_t;
        variable effective_target : u32_t;
    begin
        validation_error := fn_validate_runtime_config(build_cfg, runtime_cfg);
        assert validation_error = CFG_OK
            report "V2-CFG-001 derive requested for invalid configuration"
            severity failure;

        total_states := fn_total_states(
            runtime_cfg.motor.cpr, runtime_cfg.motor.decode_mode);
        half_width := to_integer(runtime_cfg.mirror.common_half_width);
        angular_intervals := 2 * half_width;
        shot_interval := fn_shot_interval_states(
            runtime_cfg.laser.optical_shot_interval_udeg, total_states);
        capture_window := fn_capture_window_ext(runtime_cfg);
        mtimer_ref_ticks := fn_gpx_mtimer_ref_ticks(
            runtime_cfg.laser.target_range_window_5ns);
        effective_target := fn_gpx_effective_target_range_5ns(
            runtime_cfg.laser.target_range_window_5ns);

        result.total_states := to_unsigned(total_states, 16);
        result.virtual_ticks_hi := runtime_cfg.motor.virtual_ticks_lo + 1;
        result.mechanical_angle_per_state_udeg := to_unsigned(
            C_FULL_MECHANICAL_UDEG / total_states, 30);
        result.optical_angle_per_state_udeg := to_unsigned(
            C_FULL_OPTICAL_SCALE_UDEG / total_states, 30);

        for face_index in 0 to C_MAX_FACES - 1 loop
            if face_index < build_cfg.num_faces then
                result.face_lower(face_index) := to_unsigned(fn_face_lower(
                    to_integer(runtime_cfg.mirror.face_centers(face_index)),
                    half_width, total_states), C_POSITION_WIDTH);
                result.face_upper(face_index) := to_unsigned(fn_face_upper(
                    to_integer(runtime_cfg.mirror.face_centers(face_index)),
                    half_width, total_states), C_POSITION_WIDTH);
            else
                result.face_lower(face_index) := (others => '0');
                result.face_upper(face_index) := (others => '0');
            end if;
        end loop;

        result.face_active_positions := to_unsigned(
            angular_intervals + 1, 16);
        result.face_angular_intervals := to_unsigned(
            angular_intervals, 16);
        result.shot_interval_states := to_unsigned(shot_interval, 16);
        result.columns_per_face := to_unsigned(fn_columns_per_face(
            angular_intervals, shot_interval), 16);

        result.present_chip_mask := fn_present_chip_mask(build_cfg.num_chips);
        if runtime_cfg.tdc.falling_enable = '1' then
            result.active_rise_mask := runtime_cfg.tdc.active_chip_mask
                and build_cfg.rise_capability_mask;
            result.active_fall_mask := runtime_cfg.tdc.active_chip_mask
                and build_cfg.fall_capability_mask;
        else
            result.active_rise_mask := runtime_cfg.tdc.active_chip_mask;
            result.active_fall_mask := (others => '0');
        end if;

        result.gpx_mtimer_ref_ticks := mtimer_ref_ticks(
            C_GPX_MTIMER_WIDTH - 1 downto 0);
        result.effective_target_range_5ns := effective_target;

        result.fire_width_proc_clks := fn_ticks_to_clocks(resize(
            runtime_cfg.laser.fire_width_5ns_ticks, 32),
            build_cfg.proc_clk_mhz);
        result.fire_done_timeout_proc_clks := fn_ticks_to_clocks(resize(
            runtime_cfg.laser.fire_done_timeout_5ns_ticks, 32),
            build_cfg.proc_clk_mhz);
        result.target_range_proc_clks := fn_ticks_to_clocks(
            effective_target,
            build_cfg.proc_clk_mhz);
        result.start_width_proc_clks := fn_ticks_to_clocks(resize(
            runtime_cfg.laser.start_width_5ns_ticks, 32),
            build_cfg.proc_clk_mhz);
        result.stop_width_proc_clks := fn_ticks_to_clocks(resize(
            runtime_cfg.laser.stop_width_5ns_ticks, 32),
            build_cfg.proc_clk_mhz);
        result.simulation_start_delay_proc_clks := fn_ticks_to_clocks(
            runtime_cfg.laser.simulation_start_delay_5ns,
            build_cfg.proc_clk_mhz);
        result.capture_window_5ns := unsigned(capture_window(31 downto 0));
        result.capture_window_tdc_clks := fn_ticks_to_clocks(
            unsigned(capture_window(31 downto 0)), build_cfg.tdc_clk_mhz);
        result.scan_timeout_tdc_clks := fn_ticks_to_clocks(
            runtime_cfg.tdc.scan_timeout_5ns, build_cfg.tdc_clk_mhz);

        return result;
    end function fn_derive_runtime_config;

end package body lidar_config_reference_pkg;
