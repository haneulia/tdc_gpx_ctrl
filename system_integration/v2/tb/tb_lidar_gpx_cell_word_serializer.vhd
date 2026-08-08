-- ============================================================================
-- 테스트 자산 목적: canonical Cell을 폭 독립 PACKED17 32-bit word열로 직렬화한다.
-- 핵심 검증 계약: Hit[16], Cell Metadata, Return 수, blank와 stall 중 payload 안정성이다.
-- 관련 RTL: lidar_gpx_cell_word_serializer.
-- 실행 회귀: scripts/run_v2_gpx_cell_word_serializer.ps1
-- 유지보수 주의: ABI word 순서 변경 시 DDR/PS/HTML Golden을 같은 변경으로 묶는다.
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

entity tb_lidar_gpx_cell_word_serializer is
    generic (
        G_CLK_MHZ : positive := 150
    );
end entity tb_lidar_gpx_cell_word_serializer;

architecture sim of tb_lidar_gpx_cell_word_serializer is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk        : std_logic := '0';
    signal rst_n      : std_logic := '0';
    signal abort_run  : std_logic := '0';
    signal cell_event : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal cell_ready : std_logic;
    signal word_event : gpx_vdma_word_event_t;
    signal word_ready : std_logic := '0';
    signal idle       : std_logic;

    function fn_test_hit(
        seed      : natural;
        hit_index : natural
    ) return gpx_hit_value_t is
        variable value : natural;
    begin
        value := seed * 256 + hit_index + 1;
        if (hit_index mod 2) = 1 then
            value := value + 65_536;
        end if;
        return to_unsigned(value, C_GPX_HIT_WIDTH);
    end function fn_test_hit;

    function fn_bool_sl(value : boolean) return std_logic is
    begin
        if value then
            return '1';
        end if;
        return '0';
    end function fn_bool_sl;

    function fn_cell(
        max_hits  : positive;
        hit_count : natural;
        slope     : gpx_slope_t;
        chip      : natural;
        blank     : std_logic;
        shot      : natural
    ) return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.cell.valid := '1';
        result.cell.kind := GPX_CELL_DATA;
        result.cell.chip_index := to_unsigned(
            chip, result.cell.chip_index'length);
        result.cell.stop_index := to_unsigned(
            shot mod C_MAX_STOPS_PER_CHIP,
            result.cell.stop_index'length);
        result.cell.slope := slope;
        result.cell.hit_count := to_unsigned(
            hit_count, result.cell.hit_count'length);
        result.cell.max_hits := to_unsigned(
            max_hits, result.cell.max_hits'length);
        for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
            result.cell.hits(hit_index) := fn_test_hit(max_hits, hit_index);
        end loop;
        result.cell.hit_dropped := '1' when blank = '0' and
            max_hits = 7 else '0';
        result.cell.return_overflow := '1' when blank = '0' and
            max_hits = 6 else '0';
        result.cell.error_fill := blank;
        result.cell.faulted := blank;
        result.cell.timeout_cause := "101" when blank = '1' else "000";
        result.cell.shot_context.valid := '1';
        result.cell.shot_context.request.valid := '1';
        result.cell.shot_context.request.face_index := to_unsigned(
            chip, result.cell.shot_context.request.face_index'length);
        result.cell.shot_context.request.shot_index := to_unsigned(
            shot, result.cell.shot_context.request.shot_index'length);
        result.cell.shot_context.request.active_version := to_unsigned(
            16#1200# + max_hits, 16);
        result.slot_index := to_unsigned(0, result.slot_index'length);
        result.slot_count := to_unsigned(1, result.slot_count'length);
        result.line_start := '1';
        result.line_end := '1';
        result.first_column := '1' when shot = 0 else '0';
        result.last_column := '0';
        result.gap_before := to_unsigned(max_hits - 1,
            result.gap_before'length);
        result.slot_blank := blank;
        result.line_faulted := blank;
        return result;
    end function fn_cell;

    function fn_expected_word(
        value      : gpx_frame_cell_event_t;
        word_index : natural
    ) return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := (others => '0');
        variable active : std_logic_vector(
            C_MAX_RETURNS_PER_STOP - 1 downto 0) := (others => '0');
        variable max_hits : positive := to_integer(value.cell.max_hits);
        variable hit_words : positive := (max_hits + 1) / 2;
        variable hit_index : natural;
    begin
        if value.slot_blank = '0' then
            for index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                if index < to_integer(value.cell.hit_count) and
                   index < max_hits then
                    active(index) := '1';
                end if;
            end loop;
        end if;

        if word_index < hit_words then
            hit_index := word_index * 2;
            if hit_index < C_MAX_RETURNS_PER_STOP and
               active(hit_index) = '1' then
                result(15 downto 0) := std_logic_vector(
                    value.cell.hits(hit_index)(15 downto 0));
            end if;
            hit_index := hit_index + 1;
            if hit_index < C_MAX_RETURNS_PER_STOP and
               active(hit_index) = '1' then
                result(31 downto 16) := std_logic_vector(
                    value.cell.hits(hit_index)(15 downto 0));
            end if;
        else
            for index in 0 to C_MAX_RETURNS_PER_STOP - 1 loop
                if active(index) = '1' then
                    result(index) := value.cell.hits(index)(16);
                end if;
                result(7 + index) := active(index);
            end loop;
            result(16 downto 14) := std_logic_vector(value.cell.hit_count);
            result(17) := fn_bool_sl(
                value.cell.slope = GPX_SLOPE_RISE);
            result(19 downto 18) := std_logic_vector(
                value.cell.chip_index);
            result(22 downto 20) := std_logic_vector(
                value.cell.stop_index);
            result(23) := value.slot_blank;
            result(24) := value.cell.error_fill;
            result(25) := value.cell.hit_dropped;
            result(26) := value.cell.return_overflow;
            result(27) := value.cell.faulted or value.line_faulted;
            result(30 downto 28) := value.cell.timeout_cause;
            result(31) := '1';
        end if;
        return result;
    end function fn_expected_word;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_cell_word_serializer
        port map (
            i_clk        => clk,
            i_rst_n      => rst_n,
            i_abort      => abort_run,
            i_cell_event => cell_event,
            o_cell_ready => cell_ready,
            o_word_event => word_event,
            i_word_ready => word_ready,
            o_idle       => idle
        );

    p_test : process
        procedure send_cell(
            constant value : in gpx_frame_cell_event_t
        ) is
        begin
            cell_event <= value;
            loop
                wait until rising_edge(clk);
                exit when cell_ready = '1';
            end loop;
            cell_event.valid <= '0';
        end procedure send_cell;

        procedure expect_cell(
            constant value       : in gpx_frame_cell_event_t;
            constant inject_stall : in boolean
        ) is
            variable word_count : positive := fn_gpx_vdma_cell_word_count(
                to_integer(value.cell.max_hits));
            variable held : gpx_vdma_word_event_t;
        begin
            for word_index in 0 to word_count - 1 loop
                word_ready <= '0';
                loop
                    wait until falling_edge(clk);
                    exit when word_event.valid = '1';
                end loop;

                assert word_event.data = fn_expected_word(value, word_index)
                    report "V2-B9-J1-TB canonical word mismatch"
                    severity failure;
                assert to_integer(word_event.word_index) = word_index and
                       to_integer(word_event.word_count) = word_count and
                       word_event.cell_start = fn_bool_sl(word_index = 0) and
                       word_event.cell_end = fn_bool_sl(
                           word_index + 1 = word_count)
                    report "V2-B9-J1-TB Cell word sideband mismatch"
                    severity failure;
                assert word_event.line_start = fn_bool_sl(word_index = 0) and
                       word_event.line_end = fn_bool_sl(
                           word_index + 1 = word_count)
                    report "V2-B9-J1-TB line boundary mismatch"
                    severity failure;
                if word_index = 0 then
                    assert word_event.gap_before = value.gap_before
                        report "V2-B9-J1-TB first-word gap mismatch"
                        severity failure;
                else
                    assert word_event.gap_before = 0
                        report "V2-B9-J1-TB non-first gap was not zero"
                        severity failure;
                end if;

                if inject_stall and (word_index mod 2) = 0 then
                    held := word_event;
                    wait until rising_edge(clk);
                    wait until falling_edge(clk);
                    assert word_event = held
                        report "V2-B9-J1-TB output changed under backpressure"
                        severity failure;
                end if;

                word_ready <= '1';
                wait until rising_edge(clk);
            end loop;
            word_ready <= '0';
        end procedure expect_cell;

        variable value_a : gpx_frame_cell_event_t;
        variable value_b : gpx_frame_cell_event_t;
        variable expected_value : gpx_frame_cell_event_t;
        variable next_accept : boolean;
        variable observed : natural;
        variable expected_index : natural;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        assert fn_gpx_vdma_cell_bytes(1) = 8 and
               fn_gpx_vdma_cell_bytes(2) = 8 and
               fn_gpx_vdma_cell_bytes(3) = 12 and
               fn_gpx_vdma_cell_bytes(4) = 12 and
               fn_gpx_vdma_cell_bytes(5) = 16 and
               fn_gpx_vdma_cell_bytes(6) = 16 and
               fn_gpx_vdma_cell_bytes(7) = 20
            report "V2-B9-J1-TB canonical Cell byte table mismatch"
            severity failure;
        assert fn_gpx_vdma_hsize_bytes(16, 7) = 368 and
               fn_gpx_vdma_hsize_bytes(32, 7) = 688 and
               fn_gpx_vdma_hsize_bytes(16, 1) = 176 and
               fn_gpx_vdma_hsize_bytes(0, 7) = 0
            report "V2-B9-J1-TB HSIZE geometry mismatch"
            severity failure;
        assert fn_gpx_vdma_line_beats(16, 7, 32) = 92 and
               fn_gpx_vdma_line_beats(16, 7, 64) = 46 and
               fn_gpx_vdma_line_beats(16, 7, 128) = 23
            report "V2-B9-J1-TB width beat ratio mismatch"
            severity failure;

        assert fn_gpx_vdma_shot_hsize_bytes(16, 7, 32) = 336 and
               fn_gpx_vdma_shot_hsize_bytes(16, 7, 64) = 336 and
               fn_gpx_vdma_shot_hsize_bytes(16, 7, 128) = 336 and
               fn_gpx_vdma_shot_line_beats(16, 7, 32) = 84 and
               fn_gpx_vdma_shot_line_beats(16, 7, 64) = 42 and
               fn_gpx_vdma_shot_line_beats(16, 7, 128) = 21
            report "V2-B9-J4-TB target Shot geometry mismatch"
            severity failure;
        assert fn_gpx_vdma_shot_hsize_bytes(1, 1, 32) = 24 and
               fn_gpx_vdma_shot_hsize_bytes(1, 1, 64) = 24 and
               fn_gpx_vdma_shot_hsize_bytes(1, 1, 128) = 32 and
               fn_gpx_vdma_footer_lines(24) = 2 and
               fn_gpx_vdma_footer_lines(32) = 1
            report "V2-B9-J4-TB minimum geometry mismatch"
            severity failure;
        assert fn_gpx_vdma_stride_bytes(32, 7, 32) = 656 and
               fn_gpx_vdma_stride_bytes(32, 7, 64) = 656 and
               fn_gpx_vdma_stride_bytes(32, 7, 128) = 656 and
               fn_gpx_vdma_vsize_lines(1800, 336) = 1801 and
               fn_gpx_vdma_max_vsize_lines(1800, 32) = 1802 and
               fn_gpx_vdma_max_vsize_lines(1800, 64) = 1802 and
               fn_gpx_vdma_max_vsize_lines(1800, 128) = 1801 and
               fn_gpx_vdma_frame_allocation_bytes(
                   32, 7, 1800, 32) = 1_182_112
            report "V2-B9-J4-TB allocation geometry mismatch"
            severity failure;

        for max_hits in 1 to C_MAX_RETURNS_PER_STOP loop
            if (max_hits mod 2) = 1 then
                value_a := fn_cell(max_hits, max_hits,
                    GPX_SLOPE_RISE, max_hits mod C_MAX_CHIPS, '0', max_hits);
            else
                value_a := fn_cell(max_hits, max_hits,
                    GPX_SLOPE_FALL, max_hits mod C_MAX_CHIPS, '0', max_hits);
            end if;
            send_cell(value_a);
            expect_cell(value_a, true);
        end loop;

        value_a := fn_cell(7, 0, GPX_SLOPE_RISE, 3, '1', 20);
        send_cell(value_a);
        expect_cell(value_a, true);

        value_a := fn_cell(7, 3, GPX_SLOPE_FALL, 1, '0', 21);
        send_cell(value_a);
        expect_cell(value_a, false);

        -- Last-word generation and next-Cell acceptance overlap. Once the
        -- first output appears, six words from two 3-word Cells must remain
        -- continuous with no Cell-boundary bubble.
        wait until idle = '1';
        value_a := fn_cell(3, 3, GPX_SLOPE_RISE, 0, '0', 30);
        value_b := fn_cell(3, 2, GPX_SLOPE_FALL, 1, '0', 31);
        word_ready <= '1';
        send_cell(value_a);
        cell_event <= value_b;
        observed := 0;

        loop
            wait until falling_edge(clk);
            next_accept := cell_event.valid = '1' and cell_ready = '1';

            if observed = 0 and word_event.valid = '0' then
                null;
            else
                assert word_event.valid = '1'
                    report "V2-B9-J1-TB bubble between consecutive Cells"
                    severity failure;
                if observed < 3 then
                    expected_value := value_a;
                    expected_index := observed;
                else
                    expected_value := value_b;
                    expected_index := observed - 3;
                end if;
                assert word_event.data = fn_expected_word(
                           expected_value, expected_index)
                    report "V2-B9-J1-TB consecutive Cell ordering mismatch"
                    severity failure;
                observed := observed + 1;
            end if;

            wait until rising_edge(clk);
            if next_accept then
                cell_event.valid <= '0';
            end if;
            exit when observed = 6;
        end loop;

        word_ready <= '0';
        wait until rising_edge(clk);
        wait until rising_edge(clk);
        assert idle = '1'
            report "V2-B9-J1-TB serializer did not return idle"
            severity failure;

        report "LIDAR_V2_GPX_CELL_WORD_SERIALIZER_PASS" severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_cell_word_serializer_150 is
end entity tb_lidar_gpx_cell_word_serializer_150;

architecture sim of tb_lidar_gpx_cell_word_serializer_150 is
begin
    u_tb : entity work.tb_lidar_gpx_cell_word_serializer
        generic map (G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_cell_word_serializer_200 is
end entity tb_lidar_gpx_cell_word_serializer_200;

architecture sim of tb_lidar_gpx_cell_word_serializer_200 is
begin
    u_tb : entity work.tb_lidar_gpx_cell_word_serializer
        generic map (G_CLK_MHZ => 200);
end architecture sim;
