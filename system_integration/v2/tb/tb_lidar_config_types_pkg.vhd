library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;

entity tb_lidar_config_types_pkg is
end entity tb_lidar_config_types_pkg;

architecture sim of tb_lidar_config_types_pkg is

    procedure check(condition : boolean; message_text : string) is
    begin
        assert condition
            report message_text
            severity failure;
    end procedure check;

    procedure check_error(
        actual_error   : lidar_cfg_error_t;
        expected_error : lidar_cfg_error_t;
        message_text   : string
    ) is
    begin
        assert actual_error = expected_error
            report message_text
                & ": expected=" & lidar_cfg_error_t'image(expected_error)
                & ", actual=" & lidar_cfg_error_t'image(actual_error)
            severity failure;
    end procedure check_error;

begin

    p_test : process
        variable build_cfg   : lidar_build_config_t;
        variable runtime_cfg : lidar_runtime_config_t;
        variable derived_cfg : lidar_derived_config_t;
    begin
        -- Build-profile ownership and topology rules.
        build_cfg := C_DEFAULT_BUILD_CONFIG;
        check_error(fn_validate_build_config(build_cfg), CFG_OK,
            "V2-BUILD-001 default build profile");
        check(fn_present_chip_mask(4) = "1111",
            "V2-BUILD-002 four-chip present mask");
        check(fn_present_chip_mask(3) = "0111",
            "V2-BUILD-003 three-chip present mask");
        check(fn_present_chip_mask(1) = "0001",
            "V2-BUILD-004 one-chip present mask");
        check(fn_active_face_mask(5) = "11111",
            "V2-BUILD-005 five-face mask");

        build_cfg.proc_clk_mhz := 135;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_PROC_CLOCK, "V2-BUILD-006 unsupported processing clock");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.tdc_clk_mhz := 75;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_TDC_CLOCK, "V2-BUILD-007 unsupported TDC clock");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.stream_clock_mode := STREAM_CLOCK_SYNC;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_SYNC_CLOCK_MISMATCH,
            "V2-BUILD-008 synchronous numeric clock mismatch");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_chips := 1;
        build_cfg.rise_capability_mask := "0001";
        build_cfg.fall_capability_mask := "0001";
        check_error(fn_validate_build_config(build_cfg), CFG_OK,
            "V2-BUILD-009 one-chip dual-edge topology");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_chips := 3;
        build_cfg.rise_capability_mask := "0011";
        build_cfg.fall_capability_mask := "0100";
        check_error(fn_validate_build_config(build_cfg), CFG_OK,
            "V2-BUILD-010 three-chip two-rise one-fall topology");

        build_cfg.fall_capability_mask := "1100";
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_SLOPE_MASK_OUTSIDE_CHIPS,
            "V2-BUILD-011 slope bit outside pin-scaled chip count");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.fall_capability_mask := "0100";
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_SLOPE_ROLE_MISSING,
            "V2-BUILD-012 present chip without a slope role");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_chips := 3;
        build_cfg.rise_capability_mask := "0001";
        build_cfg.fall_capability_mask := "0110";
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_SLOPE_BALANCE,
            "V2-BUILD-013 falling-chip count exceeds rising-chip count");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.rise_capability_mask := "1111";
        build_cfg.fall_capability_mask := "0000";
        check_error(fn_validate_build_config(build_cfg), CFG_OK,
            "V2-BUILD-014 falling-disabled all-rise capability");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.rise_capability_mask(0) := 'X';
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_SLOPE_MASK_UNKNOWN,
            "V2-BUILD-015 unknown slope mask bit");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.output_width := 96;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_OUTPUT_WIDTH, "V2-BUILD-016 unsupported output width");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_faces := 6;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_FACE_COUNT, "V2-BUILD-017 face count above five");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.enable_echo_receiver := false;
        build_cfg.enable_echo_simulation := true;
        check_error(fn_validate_build_config(build_cfg),
            CFG_BUILD_ECHO_SIM_WITHOUT_ECHO,
            "V2-BUILD-018 Echo simulation without Echo frontend");

        -- Default runtime profile and all key derived values.
        build_cfg := C_DEFAULT_BUILD_CONFIG;
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-002 default runtime profile");
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);

        check(to_integer(derived_cfg.total_states) = 14400,
            "V2-DERIVE-001 decoded states/revolution");
        check(to_integer(derived_cfg.virtual_ticks_hi) = 521,
            "V2-DERIVE-002 ticks_hi is ticks_lo plus one");
        check(to_integer(derived_cfg.mechanical_angle_per_state_udeg) = 25000,
            "V2-DERIVE-003 mechanical angle/state");
        check(to_integer(derived_cfg.optical_angle_per_state_udeg) = 50000,
            "V2-DERIVE-004 optical angle/state");
        check(to_integer(derived_cfg.face_active_positions) = 2401,
            "V2-DERIVE-005 inclusive Face positions");
        check(to_integer(derived_cfg.face_angular_intervals) = 2400,
            "V2-DERIVE-006 Face angular intervals");
        check(to_integer(derived_cfg.shot_interval_states) = 1,
            "V2-DERIVE-007 one-state shot interval");
        check(to_integer(derived_cfg.columns_per_face) = 2400,
            "V2-DERIVE-008 exit-exclusive shot columns");
        check(to_integer(derived_cfg.face_lower(0)) = 240
              and to_integer(derived_cfg.face_upper(0)) = 2640,
            "V2-DERIVE-009 Face 0 inclusive boundaries");
        check(to_integer(derived_cfg.face_lower(4)) = 11760
              and to_integer(derived_cfg.face_upper(4)) = 14160,
            "V2-DERIVE-010 Face 4 inclusive boundaries");
        check(derived_cfg.present_chip_mask = "1111"
              and derived_cfg.active_rise_mask = "0011"
              and derived_cfg.active_fall_mask = "1100",
            "V2-DERIVE-011 dedicated slope masks");
        check(to_integer(derived_cfg.target_range_proc_clks) = 216,
            "V2-DERIVE-012 288 fixed ticks to 150 MHz");
        check(to_integer(derived_cfg.capture_window_tdc_clks) = 288,
            "V2-DERIVE-013 288 fixed ticks to 200 MHz");

        -- Runtime validation failures are deterministic and non-overlapping.
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.motor.cpr := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_CPR, "V2-CFG-003 zero CPR");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.motor.cpr := to_unsigned(8192, 16);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_CPR, "V2-CFG-004 CPR above supported range");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.motor.virtual_ticks_lo := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_VIRTUAL_TICKS, "V2-CFG-005 zero virtual ticks");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.motor.cpr := to_unsigned(100, 16);
        runtime_cfg.motor.decode_mode := DECODE_X1;
        runtime_cfg.motor.virtual_hi_count := to_unsigned(101, 15);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_VIRTUAL_HI_COUNT,
            "V2-CFG-006 fractional high count above total states");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.motor.cpr := to_unsigned(100, 16);
        runtime_cfg.motor.decode_mode := DECODE_X1;
        runtime_cfg.motor.virtual_hi_count := to_unsigned(0, 15);
        runtime_cfg.motor.z_offset := to_unsigned(100, C_POSITION_WIDTH);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_Z_PARAM, "V2-CFG-007 Z offset outside revolution");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.mirror.common_half_width := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_FACE_HALF_WIDTH, "V2-CFG-008 zero Face half-width");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.mirror.face_centers(0) := to_unsigned(14400,
            C_POSITION_WIDTH);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_FACE_CENTER, "V2-CFG-009 Face center outside revolution");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.mirror.face_centers(1) := to_unsigned(2000,
            C_POSITION_WIDTH);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_FACE_OVERLAP, "V2-CFG-010 overlapping Face windows");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_faces := 4;
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_FACE_ENABLE_MASK,
            "V2-CFG-011 enabled Face bit above build count");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.laser.optical_shot_interval_udeg := to_unsigned(49_999, 30);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_SHOT_BELOW_ONE_STATE,
            "V2-CFG-012 requested angle below one optical state");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.laser.optical_shot_interval_udeg := to_unsigned(50_001, 30);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-013 quantized two-state angle");
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);
        check(to_integer(derived_cfg.shot_interval_states) = 2
              and to_integer(derived_cfg.columns_per_face) = 1200,
            "V2-DERIVE-014 ceil angle quantization and columns");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.laser.fire_done_timeout_5ns_ticks := to_unsigned(289, 16);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_FIRE_TIMEOUT,
            "V2-CFG-014 fire-done timeout above range window");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.laser.target_range_window_5ns := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_RANGE_WINDOW, "V2-CFG-015 zero target range window");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.tdc.capture_adjust_5ns := to_signed(-289, 17);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_CAPTURE_WINDOW,
            "V2-CFG-016 non-positive adjusted capture window");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.tdc.active_chip_mask := "1100";
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_ACTIVE_CHIP_MASK,
            "V2-CFG-017 runtime falling lanes exceed rising lanes");

        runtime_cfg.tdc.falling_enable := '0';
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-018 falling disabled promotes active chips to rising");
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);
        check(derived_cfg.active_rise_mask = "1100"
              and derived_cfg.active_fall_mask = "0000",
            "V2-DERIVE-015 all active chips become rising");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.tdc.max_hits_per_stop := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_MAX_HITS, "V2-CFG-019 zero is not a max-hit alias");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.tdc.bus_clk_div := (others => '0');
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg),
            CFG_RUNTIME_BUS_TIMING, "V2-CFG-020 zero GPX bus divider");

        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.tdc.capture_adjust_5ns := to_signed(-21, 17);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-021 explicit v1-compatible capture adjustment");
        build_cfg.tdc_clk_mhz := 100;
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);
        check(to_integer(derived_cfg.capture_window_5ns) = 267
              and to_integer(derived_cfg.capture_window_tdc_clks) = 134,
            "V2-DERIVE-016 adjusted range and ceil conversion at 100 MHz");

        -- Direction changes traversal, never the stored geometry or columns.
        build_cfg := C_DEFAULT_BUILD_CONFIG;
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);
        runtime_cfg.motor.direction := DIRECTION_CCW;
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-022 CCW profile");
        check(fn_derive_runtime_config(build_cfg, runtime_cfg).face_lower(0)
              = derived_cfg.face_lower(0)
              and fn_derive_runtime_config(build_cfg, runtime_cfg).face_upper(0)
              = derived_cfg.face_upper(0),
            "V2-DERIVE-017 direction-independent stored geometry");

        -- Circular Face bounds remain inclusive while the shot lattice is
        -- explicitly exit-exclusive.
        build_cfg.num_faces := 1;
        runtime_cfg := C_DEFAULT_RUNTIME_CONFIG;
        runtime_cfg.laser.face_enable_mask := "00001";
        runtime_cfg.mirror.face_centers(0) := to_unsigned(100,
            C_POSITION_WIDTH);
        check_error(fn_validate_runtime_config(build_cfg, runtime_cfg), CFG_OK,
            "V2-CFG-023 wrapped single-Face profile");
        derived_cfg := fn_derive_runtime_config(build_cfg, runtime_cfg);
        check(to_integer(derived_cfg.face_lower(0)) = 13300
              and to_integer(derived_cfg.face_upper(0)) = 1300,
            "V2-DERIVE-018 wrapped inclusive boundaries");

        -- Stable external error-code examples.
        check(fn_cfg_error_code(CFG_OK) = x"00"
              and fn_cfg_error_code(CFG_RUNTIME_SHOT_BELOW_ONE_STATE) = x"2A"
              and fn_cfg_error_code(CFG_RUNTIME_BUS_TIMING) = x"31",
            "V2-CFG-024 stable error codes");

        report "LIDAR_V2_CONFIG_TYPES_PASS" severity note;
        wait;
    end process p_test;

end architecture sim;
