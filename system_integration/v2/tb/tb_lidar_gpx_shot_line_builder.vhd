library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_shot_line_builder is
    generic (
        G_CLK_MHZ : positive := 150
    );
end entity tb_lidar_gpx_shot_line_builder;

architecture sim of tb_lidar_gpx_shot_line_builder is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_TIMESTAMP : t0_timestamp_t := x"0123456789ABCDEF";

    signal clk             : std_logic := '0';
    signal rst_n           : std_logic := '0';
    signal abort_run       : std_logic := '0';
    signal cell_word       : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal cell_word_ready : std_logic;
    signal line_word       : gpx_vdma_line_word_event_t;
    signal line_word_ready : std_logic := '0';
    signal idle            : std_logic;

    function fn_context(
        shot_index : natural
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(3, 3);
        result.request.position := to_unsigned(16#1234#, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CCW;
        result.request.shot_index := to_unsigned(shot_index, 16);
        result.request.last_in_face := '1';
        result.request.source_sim := '1';
        result.request.source_latency_clks := to_unsigned(5,
            result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := x"4567";
        result.fire_to_t0_clks := to_unsigned(9, 32);
        result.t0_timestamp_ticks := C_TIMESTAMP;
        result.t0_timestamp_valid := '1';
        result.t0_time_sync_valid := '1';
        return result;
    end function fn_context;

    function fn_cell_word(
        slot_index : natural;
        word_index : natural;
        line_fault : std_logic
    ) return gpx_vdma_word_event_t is
        variable result : gpx_vdma_word_event_t :=
            C_GPX_VDMA_WORD_EVENT_IDLE;
    begin
        result.valid := '1';
        result.data := std_logic_vector(to_unsigned(
            16#1000# + slot_index * 16 + word_index, 32));
        result.word_index := to_unsigned(word_index,
            result.word_index'length);
        result.word_count := to_unsigned(2, result.word_count'length);
        result.cell_start := '1' when word_index = 0 else '0';
        result.cell_end := '1' when word_index = 1 else '0';
        result.line_start := '1' when slot_index = 0 and
            word_index = 0 else '0';
        result.line_end := '1' when slot_index = 1 and
            word_index = 1 else '0';
        result.first_column := '1';
        result.last_column := '1';
        result.slot_index := to_unsigned(slot_index,
            result.slot_index'length);
        result.slot_count := to_unsigned(2, result.slot_count'length);
        if result.line_start = '1' then
            result.gap_before := to_unsigned(2, result.gap_before'length);
        end if;
        result.line_faulted := line_fault;
        result.shot_context := fn_context(16#42#);
        return result;
    end function fn_cell_word;

    function fn_expected_flags return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := (others => '0');
    begin
        result(C_GPX_SHOT_META_VALID) := '1';
        result(C_GPX_SHOT_META_DIRECTION_CCW) := '1';
        result(C_GPX_SHOT_META_SOURCE_SIM) := '1';
        result(C_GPX_SHOT_META_LINE_FAULTED) := '1';
        result(C_GPX_SHOT_META_T0_VALID) := '1';
        result(C_GPX_SHOT_META_TIME_SYNC_VALID) := '1';
        result(C_GPX_SHOT_META_LAST_IN_FACE) := '1';
        result(C_GPX_SHOT_META_SOURCE_LATENCY_VALID) := '1';
        result(C_GPX_SHOT_META_SOURCE_LATENCY_HI downto
               C_GPX_SHOT_META_SOURCE_LATENCY_LO) :=
            std_logic_vector(to_unsigned(5, 8));
        return result;
    end function fn_expected_flags;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_shot_line_builder
        port map (
            i_clk             => clk,
            i_rst_n           => rst_n,
            i_abort           => abort_run,
            i_cell_word       => cell_word,
            o_cell_word_ready => cell_word_ready,
            o_line_word       => line_word,
            i_line_word_ready => line_word_ready,
            o_idle            => idle
        );

    p_test : process
        procedure send_word(
            constant value : in gpx_vdma_word_event_t
        ) is
        begin
            cell_word <= value;
            loop
                wait until rising_edge(clk);
                exit when cell_word_ready = '1';
            end loop;
            cell_word.valid <= '0';
        end procedure send_word;

        procedure expect_word(
            constant expected_data  : in gpx_vdma_word_t;
            constant expected_index : in natural;
            constant expected_kind  : in gpx_vdma_line_word_kind_t;
            constant expected_start : in std_logic;
            constant expected_end   : in std_logic;
            constant stall_output   : in boolean
        ) is
            variable held : gpx_vdma_line_word_event_t;
        begin
            line_word_ready <= '0';
            loop
                wait until falling_edge(clk);
                exit when line_word.valid = '1';
            end loop;

            assert line_word.data = expected_data and
                   to_integer(line_word.word_index) = expected_index and
                   to_integer(line_word.line_word_count) = 8 and
                   line_word.kind = expected_kind and
                   line_word.line_start = expected_start and
                   line_word.line_end = expected_end
                report "V2-B9-J5-TB data/index/boundary mismatch"
                severity failure;
            assert line_word.first_column = '1' and
                   line_word.last_column = '1' and
                   to_integer(line_word.slot_count) = 2 and
                   to_integer(line_word.cell_word_count) = 2 and
                   line_word.line_faulted = '1' and
                   line_word.line_hole = '0' and
                   line_word.shot_context = fn_context(16#42#)
                report "V2-B9-J5-TB sideband mismatch"
                severity failure;
            if expected_start = '1' then
                assert line_word.gap_before = 2
                    report "V2-B9-J5-TB leading gap sideband mismatch"
                    severity failure;
            else
                assert line_word.gap_before = 0
                    report "V2-B9-J5-TB non-first gap was not zero"
                    severity failure;
            end if;

            if stall_output then
                held := line_word;
                wait until rising_edge(clk);
                wait until falling_edge(clk);
                assert line_word = held
                    report "V2-B9-J5-TB output changed under backpressure"
                    severity failure;
            end if;

            line_word_ready <= '1';
            wait until rising_edge(clk);
            line_word_ready <= '0';
        end procedure expect_word;

        variable expected_w2 : gpx_vdma_word_t;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        send_word(fn_cell_word(0, 0, '1'));

        expect_word(x"89ABCDEF", 0, GPX_VDMA_LINE_SHOT_METADATA,
            '1', '0', true);
        expect_word(x"01234567", 1, GPX_VDMA_LINE_SHOT_METADATA,
            '0', '0', false);
        expected_w2 := x"12340042";
        expect_word(expected_w2, 2, GPX_VDMA_LINE_SHOT_METADATA,
            '0', '0', true);
        expect_word(fn_expected_flags, 3,
            GPX_VDMA_LINE_SHOT_METADATA, '0', '0', false);
        expect_word(fn_cell_word(0, 0, '1').data, 4,
            GPX_VDMA_LINE_CELL_DATA, '0', '0', true);

        send_word(fn_cell_word(0, 1, '1'));
        expect_word(fn_cell_word(0, 1, '1').data, 5,
            GPX_VDMA_LINE_CELL_DATA, '0', '0', false);
        send_word(fn_cell_word(1, 0, '1'));
        expect_word(fn_cell_word(1, 0, '1').data, 6,
            GPX_VDMA_LINE_CELL_DATA, '0', '0', true);
        send_word(fn_cell_word(1, 1, '1'));
        expect_word(fn_cell_word(1, 1, '1').data, 7,
            GPX_VDMA_LINE_CELL_DATA, '0', '1', false);

        wait until rising_edge(clk);
        assert idle = '1'
            report "V2-B9-J5-TB builder did not return idle"
            severity failure;

        -- Abort before any line word is accepted must discard the local line.
        send_word(fn_cell_word(0, 0, '1'));
        loop
            wait until falling_edge(clk);
            exit when line_word.valid = '1';
        end loop;
        assert line_word.valid = '1'
            report "V2-B9-J5-TB metadata did not start before abort"
            severity failure;
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        wait until falling_edge(clk);
        assert idle = '1' and line_word.valid = '0'
            report "V2-B9-J5-TB abort did not clear local capture"
            severity failure;

        report "LIDAR_V2_GPX_SHOT_LINE_BUILDER_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_shot_line_builder_150 is
end entity tb_lidar_gpx_shot_line_builder_150;

architecture sim of tb_lidar_gpx_shot_line_builder_150 is
begin
    u_tb : entity work.tb_lidar_gpx_shot_line_builder
        generic map (G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_shot_line_builder_200 is
end entity tb_lidar_gpx_shot_line_builder_200;

architecture sim of tb_lidar_gpx_shot_line_builder_200 is
begin
    u_tb : entity work.tb_lidar_gpx_shot_line_builder
        generic map (G_CLK_MHZ => 200);
end architecture sim;
