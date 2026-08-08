-- ============================================================================
-- 테스트 자산 목적: 모터 위치에서 현재 Face와 활성 경계 event를 만드는 동작을 검증한다.
-- 핵심 검증 계약: 1~5 Face, wrap, CW/CCW, inclusive 경계와 겹침 진단이 정확하다.
-- 관련 RTL: face_tracker.
-- 실행 회귀: scripts/run_v2_face_tracker.ps1
-- 유지보수 주의: lower/upper 또는 방향 계약 변경 시 모든 Face 수의 경계 벡터를 갱신한다.
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

entity tb_face_tracker is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333;
        G_DUT_NUM_FACES       : positive range 1 to C_MAX_FACES := 5
    );
end entity tb_face_tracker;

architecture sim of tb_face_tracker is

    function fn_face_build_config(
        num_faces_value : positive
    ) return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.num_faces := num_faces_value;
        return result;
    end function fn_face_build_config;

    type face_point_array_t is array (0 to C_MAX_FACES - 1) of natural;
    constant C_FACE_TEST_POINTS : face_point_array_t :=
        (15, 32, 42, 52, 62);

    function fn_tracker_config(
        version_value : positive;
        face_mask     : face_mask_t;
        wrap_face_zero: boolean := false;
        overlap_test  : boolean := false;
        adjacent_test : boolean := false
    ) return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := C_DEFAULT_RUNTIME_CONFIG;
        source_v.laser.face_enable_mask := face_mask;
        result.version := to_unsigned(version_value, result.version'length);
        result.source := source_v;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, source_v);
        result.derived.total_states := to_unsigned(
            100, result.derived.total_states'length);

        result.derived.face_lower(0) := to_unsigned(
            10, C_POSITION_WIDTH);
        result.derived.face_upper(0) := to_unsigned(
            20, C_POSITION_WIDTH);
        result.derived.face_lower(1) := to_unsigned(
            30, C_POSITION_WIDTH);
        result.derived.face_upper(1) := to_unsigned(
            35, C_POSITION_WIDTH);
        result.derived.face_lower(2) := to_unsigned(
            40, C_POSITION_WIDTH);
        result.derived.face_upper(2) := to_unsigned(
            45, C_POSITION_WIDTH);
        result.derived.face_lower(3) := to_unsigned(
            50, C_POSITION_WIDTH);
        result.derived.face_upper(3) := to_unsigned(
            55, C_POSITION_WIDTH);
        result.derived.face_lower(4) := to_unsigned(
            60, C_POSITION_WIDTH);
        result.derived.face_upper(4) := to_unsigned(
            65, C_POSITION_WIDTH);

        if wrap_face_zero then
            result.derived.face_lower(0) := to_unsigned(
                90, C_POSITION_WIDTH);
            result.derived.face_upper(0) := to_unsigned(
                5, C_POSITION_WIDTH);
        end if;
        if overlap_test then
            result.derived.face_lower(1) :=
                result.derived.face_lower(0);
            result.derived.face_upper(1) :=
                result.derived.face_upper(0);
        end if;
        if adjacent_test then
            result.derived.face_lower(1) := to_unsigned(
                21, C_POSITION_WIDTH);
            result.derived.face_upper(1) := to_unsigned(
                30, C_POSITION_WIDTH);
        end if;
        return result;
    end function fn_tracker_config;

    constant C_ALL_FACE_MASK : face_mask_t := (others => '1');
    constant C_BASE_CONFIG : lidar_active_config_t := fn_tracker_config(
        101, C_ALL_FACE_MASK);

    signal clk           : std_logic := '0';
    signal rst_n         : std_logic := '0';
    signal stop_clock    : boolean := false;
    signal enable        : std_logic := '0';
    signal active_valid  : std_logic := '0';
    signal active_config : lidar_active_config_t := C_BASE_CONFIG;
    signal position_event: position_event_t := C_POSITION_EVENT_IDLE;
    signal clear_diag    : std_logic := '0';

    signal face_event      : face_event_t;
    signal overlap_sticky  : std_logic;
    signal overlap_count   : u32_t;

begin

    u_dut : entity work.face_tracker
        generic map (
            G_BUILD_CONFIG => fn_face_build_config(G_DUT_NUM_FACES)
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_enable            => enable,
            i_active_valid      => active_valid,
            i_active_config     => active_config,
            i_position_event    => position_event,
            i_clear_diagnostics => clear_diag,
            o_face_event        => face_event,
            o_overlap_sticky    => overlap_sticky,
            o_overlap_count     => overlap_count
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
            enable        <= '0';
            active_valid  <= '0';
            position_event <= C_POSITION_EVENT_IDLE;
            clear_diag    <= '0';
            rst_n         <= '0';
            wait_clocks(3);
            rst_n <= '1';
            wait_clocks(2);
        end procedure reset_dut;

        procedure load_config(
            constant config_value : in lidar_active_config_t
        ) is
        begin
            enable         <= '0';
            active_config  <= config_value;
            active_valid   <= '1';
            position_event <= C_POSITION_EVENT_IDLE;
            wait_clocks(3);
            check(face_event.valid = '0',
                "V2-FACE-P13 event escaped configuration quiesce");
            enable <= '1';
            wait_clocks(1);
        end procedure load_config;

        procedure drive_position(
            constant position_value : in natural;
            constant direction_value: in direction_t;
            constant expected_inside: in std_logic;
            constant expected_enter : in std_logic;
            constant expected_exit  : in std_logic;
            constant expected_face  : in natural;
            constant expected_overlap : in std_logic;
            constant case_name      : in string
        ) is
            variable event_v : position_event_t;
        begin
            event_v := C_POSITION_EVENT_IDLE;
            event_v.valid := '1';
            event_v.position := to_unsigned(
                position_value, event_v.position'length);
            event_v.direction := direction_value;
            event_v.source_sim := '0';
            event_v.source_latency_clks :=
                C_POSITION_PHYSICAL_LATENCY_CLKS;
            event_v.source_latency_valid := '1';
            event_v.active_version := active_config.version;

            wait until falling_edge(clk);
            position_event <= event_v;
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '0',
                case_name & " bypassed the registered membership stage");

            position_event <= C_POSITION_EVENT_IDLE;
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '1',
                case_name & " missing registered Face event");
            check(face_event.inside = expected_inside,
                case_name & " inside mismatch");
            check(face_event.enter_event = expected_enter,
                case_name & " enter mismatch");
            check(face_event.exit_event = expected_exit,
                case_name & " exit mismatch");
            check(to_integer(face_event.face_index) = expected_face,
                case_name & " Face index mismatch");
            check(face_event.overlap = expected_overlap,
                case_name & " overlap mismatch");
            check(to_integer(face_event.position) = position_value and
                  face_event.direction = direction_value,
                case_name & " position context mismatch");
            check(face_event.source_sim = '0' and
                  face_event.source_latency_valid = '1' and
                  face_event.source_latency_clks =
                    C_POSITION_PHYSICAL_LATENCY_CLKS,
                case_name & " source latency context mismatch");
            check(face_event.active_version = active_config.version,
                case_name & " active version mismatch");

            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '0',
                case_name & " Face event was not one cycle");
        end procedure drive_position;

        procedure drive_p10_burst is
            variable event_v : position_event_t;

            procedure apply_position(position_value : natural) is
            begin
                event_v := C_POSITION_EVENT_IDLE;
                event_v.valid := '1';
                event_v.position := to_unsigned(
                    position_value, event_v.position'length);
                event_v.direction := DIRECTION_CW;
                event_v.source_latency_clks :=
                    C_POSITION_PHYSICAL_LATENCY_CLKS;
                event_v.source_latency_valid := '1';
                event_v.active_version := active_config.version;
                position_event <= event_v;
            end procedure apply_position;
        begin
            wait until falling_edge(clk);
            apply_position(9);
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '0',
                "V2-FACE-P10 burst bypassed membership register");

            wait until falling_edge(clk);
            apply_position(10);
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '1' and
                  face_event.position = 9 and
                  face_event.inside = '0',
                "V2-FACE-P10 burst result 0 mismatch");

            wait until falling_edge(clk);
            apply_position(11);
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '1' and
                  face_event.position = 10 and
                  face_event.inside = '1' and
                  face_event.enter_event = '1',
                "V2-FACE-P10 burst result 1 mismatch");

            wait until falling_edge(clk);
            position_event <= C_POSITION_EVENT_IDLE;
            wait until rising_edge(clk);
            wait for 1 ps;
            check(face_event.valid = '1' and
                  face_event.position = 11 and
                  face_event.inside = '1' and
                  face_event.enter_event = '0',
                "V2-FACE-P10 burst result 2 mismatch");
            wait_clocks(1);
            check(face_event.valid = '0',
                "V2-FACE-P10 burst did not drain");
        end procedure drive_p10_burst;

        variable config_v : lidar_active_config_t;
        variable mask_v   : face_mask_t;
    begin
        reset_dut;

        -- P10: non-wrapping inclusive lower/upper bounds.
        config_v := fn_tracker_config(110, C_ALL_FACE_MASK);
        load_config(config_v);
        drive_position(9, DIRECTION_CW, '0', '0', '0', 0, '0',
            "V2-FACE-P10 below lower");
        drive_position(10, DIRECTION_CW, '1', '1', '0', 0, '0',
            "V2-FACE-P10 inclusive lower");
        drive_position(20, DIRECTION_CW, '1', '0', '0', 0, '0',
            "V2-FACE-P10 inclusive upper");
        drive_position(21, DIRECTION_CW, '0', '0', '1', 0, '0',
            "V2-FACE-P10 above upper");
        load_config(config_v);
        drive_p10_burst;
        report "V2-FACE-P10 PASS faces=" &
            integer'image(G_DUT_NUM_FACES) severity note;

        -- P11: one Face crossing state zero.
        config_v := fn_tracker_config(
            111, C_ALL_FACE_MASK, wrap_face_zero => true);
        load_config(config_v);
        drive_position(89, DIRECTION_CW, '0', '0', '0', 0, '0',
            "V2-FACE-P11 before wrapped lower");
        drive_position(90, DIRECTION_CW, '1', '1', '0', 0, '0',
            "V2-FACE-P11 wrapped lower");
        drive_position(99, DIRECTION_CW, '1', '0', '0', 0, '0',
            "V2-FACE-P11 high segment");
        drive_position(0, DIRECTION_CW, '1', '0', '0', 0, '0',
            "V2-FACE-P11 zero segment");
        drive_position(5, DIRECTION_CW, '1', '0', '0', 0, '0',
            "V2-FACE-P11 wrapped upper");
        drive_position(6, DIRECTION_CW, '0', '0', '1', 0, '0',
            "V2-FACE-P11 after wrapped upper");
        report "V2-FACE-P11 PASS faces=" &
            integer'image(G_DUT_NUM_FACES) severity note;

        -- P12: CCW boundary order and reversal inside an active Face.
        config_v := fn_tracker_config(112, C_ALL_FACE_MASK);
        load_config(config_v);
        drive_position(21, DIRECTION_CCW, '0', '0', '0', 0, '0',
            "V2-FACE-P12 CCW outside");
        drive_position(20, DIRECTION_CCW, '1', '1', '0', 0, '0',
            "V2-FACE-P12 CCW upper entry");
        drive_position(10, DIRECTION_CCW, '1', '0', '0', 0, '0',
            "V2-FACE-P12 CCW inclusive lower");
        drive_position(9, DIRECTION_CCW, '0', '0', '1', 0, '0',
            "V2-FACE-P12 CCW lower exit");

        load_config(config_v);
        drive_position(9, DIRECTION_CW, '0', '0', '0', 0, '0',
            "V2-FACE-P12 reversal seed");
        drive_position(10, DIRECTION_CW, '1', '1', '0', 0, '0',
            "V2-FACE-P12 CW entry");
        drive_position(12, DIRECTION_CW, '1', '0', '0', 0, '0',
            "V2-FACE-P12 CW interior");
        drive_position(11, DIRECTION_CCW, '1', '1', '1', 0, '0',
            "V2-FACE-P12 in-Face reversal");
        drive_position(9, DIRECTION_CCW, '0', '0', '1', 0, '0',
            "V2-FACE-P12 reversed exit");

        if G_DUT_NUM_FACES >= 2 then
            mask_v := (others => '0');
            mask_v(0) := '1';
            mask_v(1) := '1';
            config_v := fn_tracker_config(
                113, mask_v, adjacent_test => true);
            load_config(config_v);
            drive_position(20, DIRECTION_CW,
                '1', '1', '0', 0, '0',
                "V2-FACE-P12 zero-gap old Face");
            drive_position(21, DIRECTION_CW,
                '1', '1', '1', 1, '0',
                "V2-FACE-P12 zero-gap new Face");
        end if;
        report "V2-FACE-P12 PASS faces=" &
            integer'image(G_DUT_NUM_FACES) severity note;

        -- P13: every runtime mask size through the build Face count.
        for active_faces in 1 to G_DUT_NUM_FACES loop
            mask_v := fn_active_face_mask(active_faces);
            config_v := fn_tracker_config(120 + active_faces, mask_v);
            load_config(config_v);
            for face_index in 0 to C_MAX_FACES - 1 loop
                drive_position(80, DIRECTION_CW,
                    '0', '0', '0', 0, '0',
                    "V2-FACE-P13 outside separator");
                if face_index < active_faces then
                    drive_position(C_FACE_TEST_POINTS(face_index),
                        DIRECTION_CW, '1', '1', '0', face_index, '0',
                        "V2-FACE-P13 active Face");
                    drive_position(80, DIRECTION_CW,
                        '0', '0', '1', face_index, '0',
                        "V2-FACE-P13 active Face exit");
                else
                    drive_position(C_FACE_TEST_POINTS(face_index),
                        DIRECTION_CW, '0', '0', '0', 0, '0',
                        "V2-FACE-P13 runtime-masked Face");
                end if;
            end loop;
        end loop;

        -- Defensive build mask: malformed runtime bits beyond build num_faces
        -- still cannot activate entity capacity that was not built.
        config_v := fn_tracker_config(130, C_ALL_FACE_MASK);
        load_config(config_v);
        for face_index in 0 to C_MAX_FACES - 1 loop
            drive_position(80, DIRECTION_CW,
                '0', '0', '0', 0, '0',
                "V2-FACE-P13 build-mask separator");
            if face_index < G_DUT_NUM_FACES then
                drive_position(C_FACE_TEST_POINTS(face_index),
                    DIRECTION_CW, '1', '1', '0', face_index, '0',
                    "V2-FACE-P13 built Face");
                drive_position(80, DIRECTION_CW,
                    '0', '0', '1', face_index, '0',
                    "V2-FACE-P13 built Face exit");
            else
                drive_position(C_FACE_TEST_POINTS(face_index),
                    DIRECTION_CW, '0', '0', '0', 0, '0',
                    "V2-FACE-P13 outside build Face count");
            end if;
        end loop;

        -- Overlap cannot pass the normal commit validator, but the runtime
        -- boundary remains deterministic and reports the malformed state.
        if G_DUT_NUM_FACES >= 2 then
            mask_v := (others => '0');
            mask_v(0) := '1';
            mask_v(1) := '1';
            config_v := fn_tracker_config(
                140, mask_v, overlap_test => true);
            load_config(config_v);
            drive_position(15, DIRECTION_CW,
                '1', '1', '0', 0, '1',
                "V2-FACE-P13 overlap priority");
            check(overlap_sticky = '1' and overlap_count = 1,
                "V2-FACE-P13 overlap diagnostics mismatch");
            clear_diag <= '1';
            wait_clocks(1);
            clear_diag <= '0';
            wait_clocks(1);
            check(overlap_sticky = '0' and overlap_count = 0,
                "V2-FACE-P13 overlap clear mismatch");
        end if;
        report "V2-FACE-P13 PASS faces=" &
            integer'image(G_DUT_NUM_FACES) severity note;

        report "LIDAR_V2_FACE_TRACKER_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " faces=" &
            integer'image(G_DUT_NUM_FACES) severity note;
        wait_clocks(2);
        stop_clock <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
