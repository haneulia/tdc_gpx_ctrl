library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_csr_map_pkg.all;

entity tb_lidar_csr_map_pkg is
end entity tb_lidar_csr_map_pkg;

architecture sim of tb_lidar_csr_map_pkg is
begin
    p_test : process
        variable build_cfg   : lidar_build_config_t;
        variable runtime_cfg : lidar_runtime_config_t;
        variable unpacked    : lidar_runtime_config_t;
        variable words       : csr_word_array_t;

        procedure check(
            constant condition_value : boolean;
            constant message_value   : string
        ) is
        begin
            assert condition_value
                report message_value
                severity failure;
        end procedure check;

        procedure check_profile(
            constant profile       : lidar_build_config_t;
            constant expected_face0 : natural;
            constant expected_mask  : chip_mask_t;
            constant expected_fall  : std_logic
        ) is
            variable local_cfg   : lidar_runtime_config_t;
            variable local_words : csr_word_array_t;
            variable local_copy  : lidar_runtime_config_t;
        begin
            check(fn_validate_build_config(profile) = CFG_OK,
                "V2-CSR-MAP build profile is illegal");
            local_cfg := fn_default_runtime_config(profile);
            check(fn_validate_runtime_config(profile, local_cfg) = CFG_OK,
                "V2-CSR-MAP build-aware reset shadow is illegal");
            check(to_integer(local_cfg.mirror.face_centers(0)) =
                    expected_face0,
                "V2-CSR-MAP Face 0 default center mismatch");
            check(local_cfg.laser.face_enable_mask =
                    fn_active_face_mask(profile.num_faces),
                "V2-CSR-MAP active Face mask mismatch");
            check(local_cfg.tdc.active_chip_mask = expected_mask,
                "V2-CSR-MAP active chip mask mismatch");
            check(local_cfg.tdc.falling_enable = expected_fall,
                "V2-CSR-MAP falling default mismatch");

            local_words := fn_pack_runtime_config(local_cfg);
            local_copy := fn_unpack_runtime_config(local_words);
            check(local_copy = local_cfg,
                "V2-CSR-MAP pack/unpack round trip mismatch");
            check(local_words(C_CTL_COMMAND) = x"00000000",
                "V2-CSR-MAP command word must not hold state");
            for index in C_CTL_RESERVED_FIRST to C_CTL_RESERVED_LAST loop
                check(local_words(index) = x"00000000",
                    "V2-CSR-MAP reserved CTL word is nonzero");
            end loop;
        end procedure check_profile;
    begin
        build_cfg := C_DEFAULT_BUILD_CONFIG;
        check_profile(build_cfg, 1440, "1111", '1');

        runtime_cfg := fn_default_runtime_config(build_cfg);
        words := fn_pack_runtime_config(runtime_cfg);
        check(words(C_CTL_MOTOR_PROFILE) = x"00020E10",
            "V2-CSR-MAP default motor word mismatch");
        check(words(C_CTL_FACE_PROFILE) = x"001F04B0",
            "V2-CSR-MAP default Face word mismatch");
        check(words(C_CTL_LASER_FIRE_PROFILE) = x"0120000D",
            "V2-CSR-MAP default Laser word mismatch");
        check(words(C_CTL_TDC_BUS_PROFILE) = x"000FF142",
            "V2-CSR-MAP default TDC bus word mismatch");
        check(words(C_CTL_GPX_IMAGE_INDEX) = x"00000000" and
                words(C_CTL_GPX_IMAGE_DATA) = x"00000000",
            "V2-CSR-MAP GPX portal leaked into runtime config words");
        check(words(C_CTL_DIAG_INDEX) = x"00000000" and
                words(C_CTL_DIAG_DATA) = x"00000000",
            "V2-CSR-MAP diagnostic portal leaked into runtime config words");
        check(fn_ctl_word_encoding_valid(
                C_CTL_GPX_IMAGE_INDEX, x"0000010F"),
            "V2-CSR-MAP legal GPX image selector rejected");
        check(not fn_ctl_word_encoding_valid(
                C_CTL_GPX_IMAGE_INDEX, x"00000200"),
            "V2-CSR-MAP reserved GPX selector bit accepted");
        check(fn_ctl_word_encoding_valid(
                C_CTL_GPX_IMAGE_DATA, x"0FFFFFFF") and
              not fn_ctl_word_encoding_valid(
                C_CTL_GPX_IMAGE_DATA, x"10000000"),
            "V2-CSR-MAP GPX 28-bit image guard mismatch");
        check(fn_ctl_word_encoding_valid(
                C_CTL_DIAG_INDEX, x"000001FF") and
              not fn_ctl_word_encoding_valid(
                C_CTL_DIAG_INDEX, x"00000200") and
              not fn_ctl_word_encoding_valid(
                C_CTL_DIAG_DATA, x"00000000"),
            "V2-CSR-MAP diagnostic portal write guard mismatch");
        unpacked := fn_unpack_runtime_config(words);
        check(unpacked = runtime_cfg,
            "V2-CSR-MAP default round trip mismatch");

        runtime_cfg.echo.channel_0_delay_5ns := to_unsigned(16#1234#, 16);
        runtime_cfg.echo.channel_step_5ns := to_unsigned(16#ABCD#, 16);
        words := fn_pack_runtime_config(runtime_cfg);
        check(words(C_CTL_ECHO_DELAY_PROFILE) = x"ABCD1234",
            "V2-CSR-MAP compact Echo CH0/STEP word mismatch");
        unpacked := fn_unpack_runtime_config(words);
        check(unpacked.echo = runtime_cfg.echo,
            "V2-CSR-MAP compact Echo profile round trip mismatch");

        runtime_cfg.motor.simulation_mode := '1';
        words := fn_pack_runtime_config(runtime_cfg);
        check(words(C_CTL_MOTOR_PROFILE) = x"00120E10",
            "V2-CSR-MAP simulation-mode motor word mismatch");
        unpacked := fn_unpack_runtime_config(words);
        check(unpacked = runtime_cfg,
            "V2-CSR-MAP simulation-mode round trip mismatch");

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_faces := 4;
        build_cfg.num_chips := 2;
        build_cfg.rise_capability_mask := "0011";
        build_cfg.fall_capability_mask := "0000";
        check_profile(build_cfg, 1800, "0011", '0');

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_faces := 3;
        build_cfg.num_chips := 3;
        build_cfg.rise_capability_mask := "0011";
        build_cfg.fall_capability_mask := "0100";
        check_profile(build_cfg, 2400, "0111", '1');

        build_cfg := C_DEFAULT_BUILD_CONFIG;
        build_cfg.num_faces := 1;
        build_cfg.num_chips := 1;
        build_cfg.rise_capability_mask := "0001";
        build_cfg.fall_capability_mask := "0001";
        check_profile(build_cfg, 7200, "0001", '1');

        report "LIDAR_V2_CSR_MAP_PKG_PASS" severity note;
        wait;
    end process p_test;
end architecture sim;
