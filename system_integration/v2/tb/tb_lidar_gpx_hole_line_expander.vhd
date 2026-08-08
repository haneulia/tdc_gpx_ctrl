-- ============================================================================
-- 테스트 자산 목적: 누락된 Shot column을 명시적 Hole Line으로 채우는 동작을 검증한다.
-- 핵심 검증 계약: leading/interior/trailing/all-hole, 순서와 backpressure 보존이다.
-- 관련 RTL: lidar_gpx_hole_line_expander.
-- 실행 회귀: scripts/run_v2_gpx_hole_line_expander.ps1
-- 유지보수 주의: Hole은 시간축 column 보존용이며 실제 Hit 손실과 혼동하지 않는다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_hole_line_expander is
    generic (
        G_CLK_MHZ : positive := 150
    );
end entity tb_lidar_gpx_hole_line_expander;

architecture sim of tb_lidar_gpx_hole_line_expander is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_SLOT_COUNT : gpx_frame_slot_t := to_unsigned(1, 6);
    constant C_CELL_WORD_COUNT : gpx_vdma_word_count_t := to_unsigned(2, 3);
    constant C_LINE_WORD_COUNT : natural := 6;

    signal clk       : std_logic := '0';
    signal rst_n     : std_logic := '0';
    signal abort_run : std_logic := '0';

    signal real_word       : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal real_word_ready : std_logic;
    signal close_in        : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal close_in_ready  : std_logic;
    signal line_word       : gpx_vdma_line_word_event_t;
    signal line_word_ready : std_logic := '0';
    signal close_out       : gpx_frame_close_event_t;
    signal close_out_ready : std_logic := '1';
    signal idle            : std_logic;

    signal phase           : natural range 0 to 6 := 0;
    signal line_phase_done : std_logic := '0';
    signal close_phase_done : std_logic := '0';
    signal ready_cycle     : natural := 0;

    function fn_context(
        shot_index : natural;
        direction  : direction_t;
        source_sim : std_logic;
        last_shot  : std_logic
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(
            16#1200# + shot_index, C_POSITION_WIDTH);
        result.request.direction := direction;
        result.request.shot_index := to_unsigned(shot_index, 16);
        result.request.last_in_face := last_shot;
        result.request.source_sim := source_sim;
        result.request.source_latency_clks := to_unsigned(4, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := x"0021";
        result.fire_to_t0_clks := to_unsigned(3, 32);
        result.t0_timestamp_ticks := to_unsigned(
            16#10000# + shot_index, C_T0_TIMESTAMP_WIDTH);
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '0';
        return result;
    end function fn_context;

    function fn_real_word(
        shot_index : natural;
        word_index : natural;
        gap_before : natural;
        last_shot  : std_logic
    ) return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    begin
        result.valid := '1';
        result.data := std_logic_vector(to_unsigned(
            16#5A000000# + shot_index * 256 + word_index, 32));
        if word_index < C_GPX_VDMA_SHOT_META_WORDS then
            result.kind := GPX_VDMA_LINE_SHOT_METADATA;
        else
            result.kind := GPX_VDMA_LINE_CELL_DATA;
        end if;
        result.word_index := to_unsigned(word_index, 9);
        result.line_word_count := to_unsigned(C_LINE_WORD_COUNT, 9);
        result.line_start := '1' when word_index = 0 else '0';
        result.line_end := '1' when word_index + 1 =
            C_LINE_WORD_COUNT else '0';
        result.last_column := last_shot;
        if word_index = 0 then
            result.gap_before := to_unsigned(gap_before, 16);
        end if;
        result.slot_count := C_SLOT_COUNT;
        result.cell_word_count := C_CELL_WORD_COUNT;
        result.shot_context := fn_context(
            shot_index, DIRECTION_CCW, '1', last_shot);
        return result;
    end function fn_real_word;

    function fn_close_event(
        columns      : natural;
        trailing_gap : natural;
        all_hole     : std_logic;
        direction    : direction_t;
        source_sim   : std_logic
    ) return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_index := to_unsigned(2, 3);
        result.direction := direction;
        result.source_sim := source_sim;
        result.active_version := x"0021";
        result.columns_per_face := to_unsigned(columns, 16);
        result.trailing_gap := to_unsigned(trailing_gap, 16);
        result.all_hole := all_hole;
        return result;
    end function fn_close_event;

    function fn_hole_data(
        shot_index : natural;
        word_index : natural;
        direction  : direction_t;
        source_sim : std_logic;
        last_shot  : std_logic;
        faulted    : std_logic
    ) return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := (others => '0');
    begin
        if word_index = 2 then
            result(15 downto 0) := std_logic_vector(to_unsigned(
                shot_index, 16));
            result(31 downto 16) := C_GPX_VDMA_HOLE_POSITION;
        elsif word_index = 3 then
            result(C_GPX_SHOT_META_HOLE) := '1';
            if direction = DIRECTION_CCW then
                result(C_GPX_SHOT_META_DIRECTION_CCW) := '1';
            end if;
            result(C_GPX_SHOT_META_SOURCE_SIM) := source_sim;
            result(C_GPX_SHOT_META_LINE_FAULTED) := faulted;
            result(C_GPX_SHOT_META_LAST_IN_FACE) := last_shot;
        end if;
        return result;
    end function fn_hole_data;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_hole_line_expander
        port map (
            i_clk                    => clk,
            i_rst_n                  => rst_n,
            i_abort                  => abort_run,
            i_active_slot_count      => C_SLOT_COUNT,
            i_active_cell_word_count => C_CELL_WORD_COUNT,
            i_real_line_word         => real_word,
            o_real_line_word_ready   => real_word_ready,
            i_frame_close_event      => close_in,
            o_frame_close_ready      => close_in_ready,
            o_line_word              => line_word,
            i_line_word_ready        => line_word_ready,
            o_frame_close_event      => close_out,
            i_frame_close_ready      => close_out_ready,
            o_idle                   => idle
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                ready_cycle <= 0;
                line_word_ready <= '0';
            else
                ready_cycle <= ready_cycle + 1;
                if ready_cycle mod 5 = 2 or ready_cycle mod 5 = 3 then
                    line_word_ready <= '0';
                else
                    line_word_ready <= '1';
                end if;
            end if;
        end if;
    end process p_ready;

    p_monitor : process (clk)
        variable active_phase : natural := 0;
        variable count_v : natural := 0;
        variable line_number : natural;
        variable word_number : natural;
        variable shot_number : natural;
        variable expected : gpx_vdma_line_word_event_t;
        variable expected_data : gpx_vdma_word_t;
        variable expected_last : std_logic;
        variable expected_start : std_logic;
        variable expected_end : std_logic;
        variable held_valid : boolean := false;
        variable held_word : gpx_vdma_line_word_event_t;
    begin
        if rising_edge(clk) then
            line_phase_done <= '0';
            close_phase_done <= '0';
            if rst_n = '0' or abort_run = '1' then
                active_phase := phase;
                count_v := 0;
                held_valid := false;
            else
                if phase /= active_phase then
                    active_phase := phase;
                    count_v := 0;
                end if;

                if line_word.valid = '1' and line_word_ready = '0' then
                    if held_valid then
                        assert line_word = held_word
                            report "V2-B9-J5B-TB output changed under stall"
                            severity failure;
                    else
                        held_word := line_word;
                        held_valid := true;
                    end if;
                else
                    held_valid := false;
                end if;

                if line_word.valid = '1' and line_word_ready = '1' then
                    line_number := count_v / C_LINE_WORD_COUNT;
                    word_number := count_v mod C_LINE_WORD_COUNT;

                    if active_phase = 1 then
                        if line_number < 2 then
                            shot_number := line_number;
                            expected_data := fn_hole_data(
                                shot_number, word_number,
                                DIRECTION_CCW, '1', '0', '0');
                            assert line_word.line_hole = '1' and
                                   line_word.data = expected_data and
                                   to_integer(line_word.shot_context.request.
                                       shot_index) = shot_number
                                report "V2-B9-J5B-TB leading Hole mismatch"
                                severity failure;
                        else
                            expected := fn_real_word(
                                2, word_number, 0, '0');
                            assert line_word = expected
                                report "V2-B9-J5B-TB real Line mismatch"
                                severity failure;
                        end if;
                        if count_v + 1 = 3 * C_LINE_WORD_COUNT then
                            line_phase_done <= '1';
                        end if;
                    elsif active_phase = 2 then
                        if line_number = 0 then
                            shot_number := 3;
                            expected_data := fn_hole_data(
                                shot_number, word_number,
                                DIRECTION_CCW, '1', '0', '0');
                            assert line_word.line_hole = '1' and
                                   line_word.data = expected_data and
                                   to_integer(line_word.shot_context.request.
                                       shot_index) = shot_number
                                report "V2-B9-J5B-TB interior Hole mismatch"
                                severity failure;
                        else
                            expected := fn_real_word(
                                4, word_number, 0, '0');
                            assert line_word = expected
                                report "V2-B9-J5B-TB post-Hole real mismatch"
                                severity failure;
                        end if;
                        if count_v + 1 = 2 * C_LINE_WORD_COUNT then
                            line_phase_done <= '1';
                        end if;
                    elsif active_phase = 3 then
                        shot_number := 5 + line_number;
                        expected_last := '0';
                        if shot_number = 6 then
                            expected_last := '1';
                        end if;
                        expected_data := fn_hole_data(
                            shot_number, word_number,
                            DIRECTION_CCW, '1', expected_last, '0');
                        assert line_word.line_hole = '1' and
                               line_word.data = expected_data and
                               to_integer(line_word.shot_context.request.
                                   shot_index) = shot_number
                            report "V2-B9-J5B-TB trailing Hole mismatch"
                            severity failure;
                        if count_v + 1 = 2 * C_LINE_WORD_COUNT then
                            line_phase_done <= '1';
                        end if;
                    elsif active_phase = 4 then
                        shot_number := line_number;
                        expected_last := '0';
                        if shot_number = 2 then
                            expected_last := '1';
                        end if;
                        expected_data := fn_hole_data(
                            shot_number, word_number,
                            DIRECTION_CW, '0', expected_last, '0');
                        assert line_word.line_hole = '1' and
                               line_word.data = expected_data and
                               to_integer(line_word.shot_context.request.
                                   shot_index) = shot_number
                            report "V2-B9-J5B-TB all-Hole mismatch"
                            severity failure;
                        if count_v + 1 = 3 * C_LINE_WORD_COUNT then
                            line_phase_done <= '1';
                        end if;
                    else
                        assert false
                            report "V2-B9-J5B-TB unexpected Line output"
                            severity failure;
                    end if;

                    expected_start := '0';
                    expected_end := '0';
                    if word_number = 0 then
                        expected_start := '1';
                    end if;
                    if word_number + 1 = C_LINE_WORD_COUNT then
                        expected_end := '1';
                    end if;
                    assert to_integer(line_word.word_index) = word_number and
                           to_integer(line_word.line_word_count) =
                               C_LINE_WORD_COUNT and
                           line_word.line_start = expected_start and
                           line_word.line_end = expected_end and
                           line_word.gap_before = 0
                        report "V2-B9-J5B-TB Line geometry mismatch"
                        severity failure;
                    count_v := count_v + 1;
                end if;

                if close_out.valid = '1' and close_out_ready = '1' then
                    if active_phase = 3 then
                        assert close_out = fn_close_event(
                            7, 2, '0', DIRECTION_CCW, '1')
                            report "V2-B9-J5B-TB trailing close mismatch"
                            severity failure;
                    elsif active_phase = 4 then
                        assert close_out = fn_close_event(
                            3, 3, '1', DIRECTION_CW, '0')
                            report "V2-B9-J5B-TB all-Hole close mismatch"
                            severity failure;
                    elsif active_phase = 5 then
                        assert close_out = fn_close_event(
                            3, 0, '0', DIRECTION_CW, '0')
                            report "V2-B9-J5B-TB zero-gap close mismatch"
                            severity failure;
                    else
                        assert false
                            report "V2-B9-J5B-TB unexpected close output"
                            severity failure;
                    end if;
                    close_phase_done <= '1';
                end if;
            end if;
        end if;
    end process p_monitor;

    p_test : process
        procedure send_real_word(
            constant value : in gpx_vdma_line_word_event_t
        ) is
        begin
            real_word <= value;
            loop
                wait until rising_edge(clk);
                exit when real_word_ready = '1';
            end loop;
            real_word.valid <= '0';
        end procedure send_real_word;

        procedure send_close(
            constant value : in gpx_frame_close_event_t
        ) is
        begin
            close_in <= value;
            loop
                wait until rising_edge(clk);
                exit when close_in_ready = '1';
            end loop;
            close_in.valid <= '0';
        end procedure send_close;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        -- Two leading holes followed by one real Shot Line.
        phase <= 1;
        for word_index in 0 to C_LINE_WORD_COUNT - 1 loop
            send_real_word(fn_real_word(2, word_index, 2, '0'));
        end loop;
        loop
            wait until rising_edge(clk);
            exit when line_phase_done = '1';
        end loop;
        wait until rising_edge(clk);
        assert idle = '1'
            report "V2-B9-J5B-TB did not idle after real Line"
            severity failure;

        -- A later real Shot at index 4 expands the interior Hole at index 3.
        phase <= 2;
        for word_index in 0 to C_LINE_WORD_COUNT - 1 loop
            send_real_word(fn_real_word(4, word_index, 1, '0'));
        end loop;
        loop
            wait until rising_edge(clk);
            exit when line_phase_done = '1';
        end loop;

        -- Face close expands trailing Shots 5 and 6 before forwarding close.
        phase <= 3;
        send_close(fn_close_event(7, 2, '0', DIRECTION_CCW, '1'));
        loop
            wait until rising_edge(clk);
            exit when line_phase_done = '1';
        end loop;
        loop
            wait until rising_edge(clk);
            exit when close_phase_done = '1';
        end loop;

        -- A Face with no real Shot still produces all three planned Lines.
        phase <= 4;
        send_close(fn_close_event(3, 3, '1', DIRECTION_CW, '0'));
        loop
            wait until rising_edge(clk);
            exit when line_phase_done = '1';
        end loop;
        loop
            wait until rising_edge(clk);
            exit when close_phase_done = '1';
        end loop;

        -- A complete Face forwards close without inventing a Line.
        phase <= 5;
        send_close(fn_close_event(3, 0, '0', DIRECTION_CW, '0'));
        loop
            wait until rising_edge(clk);
            exit when close_phase_done = '1';
        end loop;

        -- Abort discards a partially generated Hole Line and pending real Line.
        phase <= 6;
        real_word <= fn_real_word(1, 0, 1, '0');
        loop
            wait until rising_edge(clk);
            exit when real_word_ready = '1';
        end loop;
        real_word.valid <= '0';
        wait until line_word.valid = '1';
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        wait until falling_edge(clk);
        assert idle = '1' and line_word.valid = '0' and
               close_out.valid = '0'
            report "V2-B9-J5B-TB abort did not clear pending output"
            severity failure;

        report "LIDAR_V2_GPX_HOLE_LINE_EXPANDER_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_hole_line_expander_150 is
end entity tb_lidar_gpx_hole_line_expander_150;

architecture sim of tb_lidar_gpx_hole_line_expander_150 is
begin
    u_tb : entity work.tb_lidar_gpx_hole_line_expander
        generic map (G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_hole_line_expander_200 is
end entity tb_lidar_gpx_hole_line_expander_200;

architecture sim of tb_lidar_gpx_hole_line_expander_200 is
begin
    u_tb : entity work.tb_lidar_gpx_hole_line_expander
        generic map (G_CLK_MHZ => 200);
end architecture sim;
