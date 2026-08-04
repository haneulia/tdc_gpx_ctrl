library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;

entity tb_lidar_commit_calculator is
    generic (
        G_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_lidar_commit_calculator;

architecture sim of tb_lidar_commit_calculator is

    constant C_HALF_PERIOD : time := G_HALF_PERIOD_PS * 1 ps;

    signal s_clk            : std_logic := '0';
    signal s_rst_n          : std_logic := '0';
    signal s_stop_clock     : boolean := false;
    signal s_start          : std_logic := '0';
    signal s_source         : lidar_runtime_config_t := C_DEFAULT_RUNTIME_CONFIG;
    signal s_busy           : std_logic;
    signal s_done           : std_logic;
    signal s_start_rejected : std_logic;
    signal s_error          : lidar_cfg_error_t;
    signal s_derived        : lidar_derived_config_t;

begin

    u_dut : entity work.lidar_commit_calculator
        generic map (
            G_BUILD_CONFIG => C_DEFAULT_BUILD_CONFIG
        )
        port map (
            i_clk            => s_clk,
            i_rst_n          => s_rst_n,
            i_start          => s_start,
            i_source         => s_source,
            o_busy           => s_busy,
            o_done           => s_done,
            o_start_rejected => s_start_rejected,
            o_error          => s_error,
            o_derived        => s_derived
        );

    p_clock : process
    begin
        while not s_stop_clock loop
            s_clk <= '0';
            wait for C_HALF_PERIOD;
            s_clk <= '1';
            wait for C_HALF_PERIOD;
        end loop;
        wait;
    end process p_clock;

    p_test : process
        procedure check(condition : boolean; message_text : string) is
        begin
            assert condition
                report message_text
                severity failure;
        end procedure check;

        procedure run_case(
            constant case_name      : in string;
            constant source_value   : in lidar_runtime_config_t;
            constant expected_error : in lidar_cfg_error_t
        ) is
            variable v_cycles   : natural := 0;
            variable v_expected : lidar_derived_config_t;
            variable v_before   : lidar_derived_config_t;
        begin
            v_before := s_derived;
            s_source <= source_value;
            wait until rising_edge(s_clk);
            s_start <= '1';
            wait until rising_edge(s_clk);
            s_start <= '0';

            loop
                wait until rising_edge(s_clk);
                v_cycles := v_cycles + 1;
                exit when s_done = '1';
                check(v_cycles < 1000,
                    case_name & ": calculator timeout");
            end loop;

            check(s_error = expected_error,
                case_name & ": unexpected error code");
            if expected_error = CFG_OK then
                v_expected := fn_derive_runtime_config(
                    C_DEFAULT_BUILD_CONFIG, source_value);
                check(s_derived = v_expected,
                    case_name & ": sequential/reference mismatch");
            else
                check(s_derived = v_before,
                    case_name & ": invalid request changed published result");
            end if;
            check(s_busy = '0', case_name & ": busy remained asserted");
            report case_name & ": PASS in "
                & natural'image(v_cycles) & " clocks" severity note;
        end procedure run_case;

        variable v_cfg       : lidar_runtime_config_t;
        variable v_expected  : lidar_derived_config_t;
        variable v_preserved : lidar_derived_config_t;
        variable v_cycles    : natural;
    begin
        s_rst_n <= '0';
        wait for 5 * C_HALF_PERIOD;
        wait until rising_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);

        run_case("V2-CALC-001 default", C_DEFAULT_RUNTIME_CONFIG, CFG_OK);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.optical_shot_interval_udeg := to_unsigned(50_001, 30);
        v_cfg.tdc.capture_adjust_5ns := to_signed(-21, 17);
        run_case("V2-CALC-002 ceil and signed calibration", v_cfg, CFG_OK);
        check(to_integer(s_derived.shot_interval_states) = 2
              and to_integer(s_derived.columns_per_face) = 1200
              and to_integer(s_derived.capture_window_5ns) = 267,
            "V2-CALC-002 derived edge values");

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.target_range_window_5ns := to_unsigned(289, 32);
        run_case("V2-CALC-003 fractional clock ceil", v_cfg, CFG_OK);
        check(to_integer(s_derived.target_range_proc_clks) = 217
              and to_integer(s_derived.capture_window_tdc_clks) = 289,
            "V2-CALC-003 150/200 MHz conversion");

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.target_range_window_5ns := unsigned'(x"FFFFFF00");
        run_case("V2-CALC-004 wide time product", v_cfg, CFG_OK);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.tdc.falling_enable := '0';
        v_cfg.tdc.active_chip_mask := "1100";
        run_case("V2-CALC-005 all-rise promotion", v_cfg, CFG_OK);
        check(s_derived.active_rise_mask = "1100"
              and s_derived.active_fall_mask = "0000",
            "V2-CALC-005 falling-disable mask result");

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.motor.direction := DIRECTION_CCW;
        run_case("V2-CALC-006 direction-independent geometry", v_cfg, CFG_OK);

        -- Every invalid commit below must leave the last valid result intact.
        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.motor.cpr := (others => '0');
        run_case("V2-CALC-101 bad CPR", v_cfg, CFG_RUNTIME_CPR);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.motor.virtual_ticks_lo := (others => '0');
        run_case("V2-CALC-102 bad virtual ticks", v_cfg,
            CFG_RUNTIME_VIRTUAL_TICKS);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.motor.virtual_hi_count := to_unsigned(14_401, 15);
        run_case("V2-CALC-103 bad high count", v_cfg,
            CFG_RUNTIME_VIRTUAL_HI_COUNT);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.motor.z_offset := to_unsigned(14_400, C_POSITION_WIDTH);
        run_case("V2-CALC-104 bad Z parameter", v_cfg, CFG_RUNTIME_Z_PARAM);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.mirror.common_half_width := (others => '0');
        run_case("V2-CALC-105 bad half width", v_cfg,
            CFG_RUNTIME_FACE_HALF_WIDTH);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.mirror.face_centers(0) := to_unsigned(
            14_400, C_POSITION_WIDTH);
        run_case("V2-CALC-106 bad face center", v_cfg,
            CFG_RUNTIME_FACE_CENTER);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.mirror.face_centers(1) := to_unsigned(2000, C_POSITION_WIDTH);
        run_case("V2-CALC-107 overlapping faces", v_cfg,
            CFG_RUNTIME_FACE_OVERLAP);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.face_enable_mask := (others => '0');
        run_case("V2-CALC-108 empty face mask", v_cfg,
            CFG_RUNTIME_FACE_ENABLE_MASK);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.optical_shot_interval_udeg := (others => '0');
        run_case("V2-CALC-109 zero shot angle", v_cfg,
            CFG_RUNTIME_SHOT_ANGLE);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.optical_shot_interval_udeg := to_unsigned(49_999, 30);
        run_case("V2-CALC-110 sub-state shot angle", v_cfg,
            CFG_RUNTIME_SHOT_BELOW_ONE_STATE);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.fire_width_5ns_ticks := (others => '0');
        run_case("V2-CALC-111 zero pulse width", v_cfg,
            CFG_RUNTIME_FIRE_WIDTH);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.target_range_window_5ns := (others => '0');
        run_case("V2-CALC-112 zero range", v_cfg,
            CFG_RUNTIME_RANGE_WINDOW);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.fire_done_timeout_5ns_ticks := to_unsigned(289, 16);
        run_case("V2-CALC-113 timeout beyond range", v_cfg,
            CFG_RUNTIME_FIRE_TIMEOUT);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.laser.fire_done_timeout_5ns_ticks := to_unsigned(1, 16);
        v_cfg.laser.target_range_window_5ns := to_unsigned(1, 32);
        v_cfg.tdc.capture_adjust_5ns := to_signed(-2, 17);
        run_case("V2-CALC-114 capture underflow", v_cfg,
            CFG_RUNTIME_CAPTURE_WINDOW);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.tdc.active_chip_mask := (others => '0');
        run_case("V2-CALC-115 empty active chips", v_cfg,
            CFG_RUNTIME_ACTIVE_CHIP_MASK);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.tdc.max_hits_per_stop := (others => '0');
        run_case("V2-CALC-116 zero max hits", v_cfg,
            CFG_RUNTIME_MAX_HITS);

        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        v_cfg.tdc.bus_ticks := (others => '0');
        run_case("V2-CALC-117 zero bus ticks", v_cfg,
            CFG_RUNTIME_BUS_TIMING);

        -- A command while BUSY is rejected, and changing i_source cannot alter
        -- the snapshot already being calculated.
        v_cfg := C_DEFAULT_RUNTIME_CONFIG;
        s_source <= v_cfg;
        wait until rising_edge(s_clk);
        s_start <= '1';
        wait until rising_edge(s_clk);
        s_start <= '0';
        while s_busy /= '1' loop
            wait until rising_edge(s_clk);
        end loop;

        v_expected := fn_derive_runtime_config(C_DEFAULT_BUILD_CONFIG, v_cfg);
        v_cfg.laser.optical_shot_interval_udeg := to_unsigned(100_000, 30);
        s_source <= v_cfg;
        s_start <= '1';
        wait until rising_edge(s_clk);
        s_start <= '0';
        wait until rising_edge(s_clk);
        check(s_start_rejected = '1',
            "V2-CALC-201 BUSY command was not rejected");

        v_cycles := 0;
        while s_done /= '1' loop
            wait until rising_edge(s_clk);
            v_cycles := v_cycles + 1;
            check(v_cycles < 1000, "V2-CALC-201 completion timeout");
        end loop;
        check(s_error = CFG_OK and s_derived = v_expected,
            "V2-CALC-201 source snapshot was not atomic");

        -- Reset aborts in-flight work without erasing the last published result.
        v_preserved := s_derived;
        s_source <= C_DEFAULT_RUNTIME_CONFIG;
        wait until rising_edge(s_clk);
        s_start <= '1';
        wait until rising_edge(s_clk);
        s_start <= '0';
        while s_busy /= '1' loop
            wait until rising_edge(s_clk);
        end loop;
        for cycle_index in 1 to 40 loop
            wait until rising_edge(s_clk);
        end loop;
        s_rst_n <= '0';
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);
        check(s_busy = '0' and s_done = '0',
            "V2-CALC-202 reset did not abort control state");
        check(s_derived = v_preserved,
            "V2-CALC-202 reset erased published payload");
        run_case("V2-CALC-203 restart after reset",
            C_DEFAULT_RUNTIME_CONFIG, CFG_OK);

        report "LIDAR_V2_COMMIT_CALCULATOR_PASS half_period_ps="
            & positive'image(G_HALF_PERIOD_PS) severity note;
        s_stop_clock <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
