-- ============================================================================
-- 테스트 자산 목적: 물리/가상 Encoder 상태를 하나의 모터 위치 event로 만드는 코어를 검증한다.
-- 핵심 검증 계약: CW/CCW, x1/x2/x4, virtual x4, Z/wrap/폭 제한과 측정 latency가 정확하다.
-- 관련 RTL: motor_position_core, quadrature/virtual encoder 하위 코어.
-- 실행 회귀: scripts/run_v2_motor_position.ps1
-- 유지보수 주의: decode 또는 방향 의미 변경 시 동일 원시 A/B/Z 벡터로 양 방향을 비교한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;

entity tb_motor_position_core is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333
    );
end entity tb_motor_position_core;

architecture sim of tb_motor_position_core is

    function fn_core_config(
        version_value : positive;
        simulation    : std_logic;
        decode_mode   : decode_mode_t;
        direction     : direction_t;
        total_states  : positive;
        ticks_lo      : positive;
        z_width_clks  : natural
    ) return lidar_active_config_t is
        variable result     : lidar_active_config_t;
        variable source_v   : lidar_runtime_config_t;
        variable multiplier : positive;
    begin
        source_v := C_DEFAULT_RUNTIME_CONFIG;
        multiplier := fn_decode_multiplier(decode_mode);
        source_v.motor.cpr := to_unsigned(
            total_states / multiplier, source_v.motor.cpr'length);
        source_v.motor.decode_mode := decode_mode;
        source_v.motor.direction := direction;
        source_v.motor.simulation_mode := simulation;
        source_v.motor.virtual_ticks_lo := to_unsigned(
            ticks_lo, source_v.motor.virtual_ticks_lo'length);
        source_v.motor.virtual_hi_count := (others => '0');
        source_v.motor.z_offset := (others => '0');
        source_v.motor.z_width := to_unsigned(
            z_width_clks, source_v.motor.z_width'length);
        source_v.motor.z_early := '0';

        result.version := to_unsigned(version_value, result.version'length);
        result.source := source_v;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, source_v);
        result.derived.total_states := to_unsigned(
            total_states, result.derived.total_states'length);
        result.derived.virtual_ticks_hi := to_unsigned(
            ticks_lo + 1, result.derived.virtual_ticks_hi'length);
        return result;
    end function fn_core_config;

    constant C_CFG_PHYS_X1 : lidar_active_config_t := fn_core_config(
        11, '0', DECODE_X1, DIRECTION_CW, 8, 8, 0);
    constant C_CFG_PHYS_X2 : lidar_active_config_t := fn_core_config(
        12, '0', DECODE_X2, DIRECTION_CW, 8, 8, 0);
    constant C_CFG_PHYS_X4 : lidar_active_config_t := fn_core_config(
        13, '0', DECODE_X4, DIRECTION_CW, 8, 8, 0);
    constant C_CFG_PHYS_INV : lidar_active_config_t := fn_core_config(
        14, '0', DECODE_X4, DIRECTION_CCW, 8, 8, 0);
    constant C_CFG_SIM_SLOW : lidar_active_config_t := fn_core_config(
        20, '1', DECODE_X4, DIRECTION_CW, 8, 8, 0);
    constant C_CFG_SIM_CW : lidar_active_config_t := fn_core_config(
        21, '1', DECODE_X4, DIRECTION_CW, 4, 2, 0);
    constant C_CFG_SIM_CCW : lidar_active_config_t := fn_core_config(
        22, '1', DECODE_X4, DIRECTION_CCW, 4, 2, 0);
    constant C_CFG_SIM_Z_CLAMP : lidar_active_config_t := fn_core_config(
        23, '1', DECODE_X4, DIRECTION_CW, 4, 2, 3);

    signal clk           : std_logic := '0';
    signal rst_n         : std_logic := '0';
    signal stop_clock    : boolean := false;
    signal enable        : std_logic := '0';
    signal active_valid  : std_logic := '0';
    signal active_config : lidar_active_config_t := C_CFG_PHYS_X4;
    signal enc_a         : std_logic := '0';
    signal enc_b         : std_logic := '0';
    signal enc_z         : std_logic := '0';
    signal clear_diag    : std_logic := '0';

    signal position_event : position_event_t;
    signal current_position : position_t;
    signal current_direction : direction_t;
    signal invalid_transition : std_logic;
    signal invalid_sticky : std_logic;
    signal invalid_count : u32_t;
    signal source_switch : std_logic;
    signal virtual_a     : std_logic;
    signal virtual_b     : std_logic;
    signal virtual_z     : std_logic;
    signal virtual_z_fault : virtual_z_fault_t;

begin

    u_dut : entity work.motor_position_core
        port map (
            i_clk                       => clk,
            i_rst_n                     => rst_n,
            i_enable                    => enable,
            i_active_valid              => active_valid,
            i_active_config             => active_config,
            i_enc_a                     => enc_a,
            i_enc_b                     => enc_b,
            i_enc_z                     => enc_z,
            i_clear_diagnostics         => clear_diag,
            o_position_event            => position_event,
            o_current_position          => current_position,
            o_current_direction         => current_direction,
            o_invalid_transition        => invalid_transition,
            o_invalid_transition_sticky => invalid_sticky,
            o_invalid_transition_count  => invalid_count,
            o_source_switch             => source_switch,
            o_virtual_a                 => virtual_a,
            o_virtual_b                 => virtual_b,
            o_virtual_z                 => virtual_z,
            o_virtual_z_fault           => virtual_z_fault
        );

    p_clock : process
    begin
        while not stop_clock loop
            clk <= '0';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
            clk <= '1';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
        end loop;
        wait;
    end process p_clock;

    p_test : process
        procedure check(
            condition    : boolean;
            message_text : string
        ) is
        begin
            assert condition report message_text severity failure;
        end procedure check;

        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
        end procedure wait_clocks;

        procedure reset_dut is
        begin
            enable       <= '0';
            active_valid <= '0';
            clear_diag   <= '0';
            enc_a        <= '0';
            enc_b        <= '0';
            enc_z        <= '0';
            rst_n        <= '0';
            wait_clocks(3);
            rst_n <= '1';
            wait_clocks(2);
        end procedure reset_dut;

        procedure load_config(
            constant config_value : in lidar_active_config_t
        ) is
        begin
            enable        <= '0';
            active_config <= config_value;
            active_valid  <= '1';
            wait_clocks(6);
            check(position_event.valid = '0',
                "V2-MOTOR-P00 event escaped configuration quiesce");
            enable <= '1';
            wait_clocks(1);
        end procedure load_config;

        procedure drive_physical(
            constant next_a           : in std_logic;
            constant next_b           : in std_logic;
            constant next_z           : in std_logic;
            constant expect_event     : in boolean;
            constant expect_invalid   : in boolean;
            constant expected_position: in natural;
            constant expected_dir     : in direction_t;
            constant expected_z       : in std_logic;
            constant case_name        : in string
        ) is
        begin
            wait until falling_edge(clk);
            enc_a <= next_a;
            enc_b <= next_b;
            enc_z <= next_z;

            -- First destination sampling edge is latency cycle zero.
            wait until rising_edge(clk);
            wait for 1 ps;
            check(position_event.valid = '0' and
                  invalid_transition = '0',
                case_name & " changed before the synchronizer latency");

            for latency in 1 to 4 loop
                wait until rising_edge(clk);
                wait for 1 ps;
                if latency < 4 then
                    check(position_event.valid = '0' and
                          invalid_transition = '0',
                        case_name & " changed at physical latency " &
                        integer'image(latency));
                else
                    check((position_event.valid = '1') = expect_event,
                        case_name & " event-valid mismatch");
                    check((invalid_transition = '1') = expect_invalid,
                        case_name & " invalid-transition mismatch");
                    if expect_event then
                        check(to_integer(position_event.position) =
                              expected_position,
                            case_name & " position mismatch");
                        check(position_event.direction = expected_dir,
                            case_name & " direction mismatch");
                        check(position_event.z_event = expected_z,
                            case_name & " Z classification mismatch");
                        check(position_event.source_sim = '0' and
                              position_event.source_latency_valid = '1' and
                              position_event.source_latency_clks =
                                C_POSITION_PHYSICAL_LATENCY_CLKS,
                            case_name & " physical latency metadata mismatch");
                        check(position_event.active_version =
                              active_config.version,
                            case_name & " active version mismatch");
                    end if;
                end if;
            end loop;
        end procedure drive_physical;

        procedure wait_virtual_transition(
            constant expected_a        : in std_logic;
            constant expected_b        : in std_logic;
            constant expected_position : in natural;
            constant expected_dir      : in direction_t;
            constant expected_z        : in std_logic;
            constant case_name         : in string
        ) is
            variable previous_a_v : std_logic;
            variable previous_b_v : std_logic;
            variable waited_v     : natural := 0;
        begin
            previous_a_v := virtual_a;
            previous_b_v := virtual_b;
            loop
                wait until rising_edge(clk);
                wait for 1 ps;
                waited_v := waited_v + 1;
                exit when virtual_a /= previous_a_v or
                          virtual_b /= previous_b_v;
                check(waited_v < 20,
                    case_name & " virtual transition timeout");
            end loop;

            check(virtual_a = expected_a and virtual_b = expected_b,
                case_name & " virtual phase mismatch");
            check(position_event.valid = '0',
                case_name & " virtual event was not registered");

            wait until rising_edge(clk);
            wait for 1 ps;
            check(position_event.valid = '1',
                case_name & " missing one-clock virtual event");
            check(to_integer(position_event.position) = expected_position,
                case_name & " position mismatch");
            check(position_event.direction = expected_dir,
                case_name & " direction mismatch");
            check(position_event.z_event = expected_z,
                case_name & " Z classification mismatch");
            check(position_event.source_sim = '1' and
                  position_event.source_latency_valid = '1' and
                  position_event.source_latency_clks =
                    C_POSITION_VIRTUAL_LATENCY_CLKS,
                case_name & " virtual latency metadata mismatch");
            check(position_event.active_version = active_config.version,
                case_name & " active version mismatch");
        end procedure wait_virtual_transition;

    begin
        check(G_PROC_CLK_MHZ = 150 or G_PROC_CLK_MHZ = 200,
            "V2-MOTOR-TB-001 unsupported Processing profile");

        -- P00: source selection changes only while quiesced and cannot create
        -- a synthetic A/B or Z event.
        reset_dut;
        load_config(C_CFG_PHYS_X4);
        check(source_switch = '0', "V2-MOTOR-P00 initial source switch");
        wait_clocks(2);
        enable <= '0';
        active_config <= C_CFG_SIM_SLOW;
        wait_clocks(1);
        check(source_switch = '1' and position_event.valid = '0',
            "V2-MOTOR-P00 physical-to-simulation reseed");
        wait_clocks(1);
        check(source_switch = '0', "V2-MOTOR-P00 switch pulse width");
        enable <= '1';
        wait_clocks(3);
        check(position_event.valid = '0',
            "V2-MOTOR-P00 synthetic simulation event");
        enable <= '0';
        active_config <= C_CFG_PHYS_X4;
        wait_clocks(1);
        check(source_switch = '1' and position_event.valid = '0',
            "V2-MOTOR-P00 simulation-to-physical reseed");
        report "V2-MOTOR-P00 source reseed: PASS" severity note;

        -- P01: exact physical decode sequences and installation inversion.
        reset_dut;
        load_config(C_CFG_PHYS_X4);
        drive_physical('1', '0', '0', true, false, 1,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x4 CW A+");
        drive_physical('1', '1', '0', true, false, 2,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x4 CW B+");
        drive_physical('0', '1', '0', true, false, 3,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x4 CW A-");
        drive_physical('0', '0', '0', true, false, 4,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x4 CW B-");
        drive_physical('0', '1', '0', true, false, 3,
            DIRECTION_CCW, '0', "V2-MOTOR-P01 x4 CCW B+");

        reset_dut;
        load_config(C_CFG_PHYS_X1);
        drive_physical('1', '0', '0', true, false, 1,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x1 A+");
        drive_physical('1', '1', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x1 ignores B+");
        drive_physical('0', '1', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x1 ignores A-");
        drive_physical('0', '0', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x1 ignores B-");
        drive_physical('0', '1', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x1 reverse setup");
        drive_physical('1', '1', '0', true, false, 0,
            DIRECTION_CCW, '0', "V2-MOTOR-P01 x1 CCW A+");

        reset_dut;
        load_config(C_CFG_PHYS_X2);
        drive_physical('1', '0', '0', true, false, 1,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x2 A+");
        drive_physical('1', '1', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x2 ignores B+");
        drive_physical('0', '1', '0', true, false, 2,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x2 A-");
        drive_physical('0', '0', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x2 ignores B-");
        drive_physical('0', '1', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P01 x2 reverse setup");
        drive_physical('1', '1', '0', true, false, 1,
            DIRECTION_CCW, '0', "V2-MOTOR-P01 x2 CCW A+");

        reset_dut;
        load_config(C_CFG_PHYS_INV);
        drive_physical('1', '0', '0', true, false, 7,
            DIRECTION_CCW, '0', "V2-MOTOR-P01 physical inversion");
        report "V2-MOTOR-P01 physical decode: PASS" severity note;

        -- P02: a sampled two-bit Gray change never moves position.
        reset_dut;
        load_config(C_CFG_PHYS_X4);
        drive_physical('1', '1', '0', false, true, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P02 illegal 00-to-11");
        check(current_position = 0 and invalid_sticky = '1' and
              invalid_count = 1,
            "V2-MOTOR-P02 invalid diagnostic mismatch");
        clear_diag <= '1';
        wait_clocks(1);
        clear_diag <= '0';
        check(invalid_sticky = '0' and invalid_count = 0,
            "V2-MOTOR-P02 diagnostic clear mismatch");
        report "V2-MOTOR-P02 invalid transition: PASS" severity note;

        -- P03: qualified Z has priority over position motion and preserves
        -- the last applied traversal direction.
        reset_dut;
        load_config(C_CFG_PHYS_X4);
        drive_physical('1', '0', '0', true, false, 1,
            DIRECTION_CW, '0', "V2-MOTOR-P03 setup motion");
        drive_physical('1', '0', '1', true, false, 0,
            DIRECTION_CW, '1', "V2-MOTOR-P03 physical Z+");
        drive_physical('1', '0', '0', false, false, 0,
            DIRECTION_CW, '0', "V2-MOTOR-P03 Z fall ignored");
        check(current_position = 0 and
              current_direction = DIRECTION_CW,
            "V2-MOTOR-P03 Z state mismatch");
        report "V2-MOTOR-P03 Z alignment: PASS" severity note;

        -- P04: the registered virtual source is decoded one Processing clock
        -- later. The first revolution qualifies Z; the second emits it.
        reset_dut;
        load_config(C_CFG_SIM_CW);
        wait_virtual_transition('1', '0', 1, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 0");
        wait_virtual_transition('1', '1', 2, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 1");
        wait_virtual_transition('0', '1', 3, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 2");
        wait_virtual_transition('0', '0', 0, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW first wrap");
        wait_virtual_transition('1', '0', 1, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 4");
        wait_virtual_transition('1', '1', 2, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 5");
        wait_virtual_transition('0', '1', 3, DIRECTION_CW, '0',
            "V2-MOTOR-P04 virtual CW 6");
        wait_virtual_transition('0', '0', 0, DIRECTION_CW, '1',
            "V2-MOTOR-P04 virtual CW qualified Z");
        check(virtual_z_fault = (virtual_z_fault'range => '0'),
            "V2-MOTOR-P04 unexpected virtual Z fault");

        reset_dut;
        load_config(C_CFG_SIM_CCW);
        wait_virtual_transition('0', '1', 3, DIRECTION_CCW, '0',
            "V2-MOTOR-P04 virtual CCW 0");
        wait_virtual_transition('1', '1', 2, DIRECTION_CCW, '0',
            "V2-MOTOR-P04 virtual CCW 1");
        wait_virtual_transition('1', '0', 1, DIRECTION_CCW, '0',
            "V2-MOTOR-P04 virtual CCW 2");
        wait_virtual_transition('0', '0', 0, DIRECTION_CCW, '0',
            "V2-MOTOR-P04 virtual CCW first wrap");
        report "V2-MOTOR-P04 virtual source and latency: PASS" severity note;

        -- P05: requested Z width 3 clocks exceeds the selected 2-clock
        -- encoder-state interval. The source must use the precomputed LO
        -- limit, so Z is low on the clock immediately after its second high
        -- cycle. This guards the timing optimization as a functional contract.
        reset_dut;
        load_config(C_CFG_SIM_Z_CLAMP);
        wait_virtual_transition('1', '0', 1, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 0");
        wait_virtual_transition('1', '1', 2, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 1");
        wait_virtual_transition('0', '1', 3, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 2");
        wait_virtual_transition('0', '0', 0, DIRECTION_CW, '0',
            "V2-MOTOR-P05 first wrap guard");
        wait_virtual_transition('1', '0', 1, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 4");
        wait_virtual_transition('1', '1', 2, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 5");
        wait_virtual_transition('0', '1', 3, DIRECTION_CW, '0',
            "V2-MOTOR-P05 virtual CW 6");
        wait_virtual_transition('0', '0', 0, DIRECTION_CW, '1',
            "V2-MOTOR-P05 qualified Z");
        check(virtual_z = '1',
            "V2-MOTOR-P05 Z was not high on its second clamped cycle");
        wait_clocks(1);
        check(virtual_z = '0',
            "V2-MOTOR-P05 Z width was not clamped from 3 to 2 clocks");
        report "V2-MOTOR-P05 virtual Z width clamp: PASS" severity note;

        report "LIDAR_V2_MOTOR_POSITION_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) severity note;
        stop_clock <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
