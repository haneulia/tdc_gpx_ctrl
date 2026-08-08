-- ============================================================================
-- 테스트 자산 목적: J2 legacy lane formatter의 등록 전송과 backpressure 기준선을 검증한다.
-- 핵심 검증 계약: SOF/EOL, 32/64/128 폭, stall/abort에서 byte 보존을 확인한다.
-- 관련 RTL: lidar_gpx_vdma_lane_formatter.
-- 실행 회귀: scripts/run_v2_gpx_vdma_lane_formatter.ps1
-- 유지보수 주의: 반복 prefix 방식은 최종 ABI가 아니며 회귀 기준선 용도로만 유지한다.
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

entity tb_lidar_gpx_vdma_lane_formatter is
    generic (
        G_OUTPUT_WIDTH : positive := 32;
        G_CLK_MHZ      : positive := 150
    );
end entity tb_lidar_gpx_vdma_lane_formatter;

architecture sim of tb_lidar_gpx_vdma_lane_formatter is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH / 32;

    signal clk        : std_logic := '0';
    signal rst_n      : std_logic := '0';
    signal abort_run  : std_logic := '0';
    signal word_event : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal word_ready : std_logic;
    signal prefix_blocks : gpx_vdma_prefix_blocks_t;

    signal axis_tdata  : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal axis_tkeep  : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal axis_tstrb  : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal axis_tvalid : std_logic;
    signal axis_tlast  : std_logic;
    signal axis_tuser  : std_logic_vector(0 downto 0);
    signal axis_tready : std_logic := '0';

    signal hsize_bytes       : unsigned(15 downto 0);
    signal line_done         : std_logic;
    signal line_done_faulted : std_logic;
    signal idle              : std_logic;

    function fn_prefix_word(index : natural) return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := x"A5000000";
    begin
        result(7 downto 0) := std_logic_vector(to_unsigned(index, 8));
        return result;
    end function fn_prefix_word;

    function fn_payload_word(
        line_tag   : natural;
        word_index : natural
    ) return gpx_vdma_word_t is
        variable result : gpx_vdma_word_t := x"5A000000";
    begin
        result(23 downto 16) := std_logic_vector(to_unsigned(line_tag, 8));
        result(15 downto 0) := std_logic_vector(to_unsigned(word_index, 16));
        return result;
    end function fn_payload_word;

    function fn_prefix_blocks return gpx_vdma_prefix_blocks_t is
        variable result : gpx_vdma_prefix_blocks_t :=
            C_GPX_VDMA_PREFIX_BLOCKS_ZERO;
    begin
        for block_index in 0 to C_GPX_VDMA_PREFIX_BLOCKS - 1 loop
            for word_index in 0 to C_GPX_VDMA_WORDS_PER_BLOCK - 1 loop
                result(block_index)(word_index * 32 + 31 downto
                    word_index * 32) := fn_prefix_word(
                        block_index * C_GPX_VDMA_WORDS_PER_BLOCK +
                        word_index);
            end loop;
        end loop;
        return result;
    end function fn_prefix_blocks;

    function fn_expected_beat(
        line_tag      : natural;
        first_column  : boolean;
        payload_words : natural;
        beat_index    : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0) :=
            (others => '0');
        variable stream_word : natural;
        variable payload_word : natural;
    begin
        for lane in 0 to C_WORDS_PER_BEAT - 1 loop
            stream_word := beat_index * C_WORDS_PER_BEAT + lane;
            if stream_word < C_GPX_VDMA_PREFIX_WORDS then
                if first_column then
                    result(lane * 32 + 31 downto lane * 32) :=
                        fn_prefix_word(stream_word);
                end if;
            else
                payload_word := stream_word - C_GPX_VDMA_PREFIX_WORDS;
                if payload_word < payload_words then
                    result(lane * 32 + 31 downto lane * 32) :=
                        fn_payload_word(line_tag, payload_word);
                end if;
            end if;
        end loop;
        return result;
    end function fn_expected_beat;

begin

    clk <= not clk after C_CLK_PERIOD / 2;
    prefix_blocks <= fn_prefix_blocks;

    u_dut : entity work.lidar_gpx_vdma_lane_formatter
        generic map (
            g_OUTPUT_WIDTH => G_OUTPUT_WIDTH
        )
        port map (
            i_clk                => clk,
            i_rst_n              => rst_n,
            i_abort              => abort_run,
            i_word_event         => word_event,
            o_word_ready         => word_ready,
            i_prefix_blocks      => prefix_blocks,
            o_m_axis_tdata       => axis_tdata,
            o_m_axis_tkeep       => axis_tkeep,
            o_m_axis_tstrb       => axis_tstrb,
            o_m_axis_tvalid      => axis_tvalid,
            o_m_axis_tlast       => axis_tlast,
            o_m_axis_tuser       => axis_tuser,
            i_m_axis_tready      => axis_tready,
            o_hsize_bytes        => hsize_bytes,
            o_line_done          => line_done,
            o_line_done_faulted  => line_done_faulted,
            o_idle               => idle
        );

    p_test : process
        procedure send_line(
            constant line_tag     : in natural;
            constant slot_count   : in positive;
            constant word_count   : in positive;
            constant first_column : in boolean;
            constant faulted      : in boolean
        ) is
            variable total_words : positive := slot_count * word_count;
            variable value : gpx_vdma_word_event_t;
        begin
            for stream_index in 0 to total_words - 1 loop
                value := C_GPX_VDMA_WORD_EVENT_IDLE;
                value.valid := '1';
                value.data := fn_payload_word(line_tag, stream_index);
                value.word_index := to_unsigned(
                    stream_index mod word_count,
                    value.word_index'length);
                value.word_count := to_unsigned(
                    word_count, value.word_count'length);
                if (stream_index mod word_count) = 0 then
                    value.cell_start := '1';
                end if;
                if (stream_index mod word_count) = word_count - 1 then
                    value.cell_end := '1';
                end if;
                if stream_index = 0 then
                    value.line_start := '1';
                end if;
                if stream_index = total_words - 1 then
                    value.line_end := '1';
                end if;
                if first_column then
                    value.first_column := '1';
                end if;
                value.slot_index := to_unsigned(
                    stream_index / word_count,
                    value.slot_index'length);
                value.slot_count := to_unsigned(
                    slot_count, value.slot_count'length);
                if faulted then
                    value.line_faulted := '1';
                end if;

                word_event <= value;
                loop
                    wait until rising_edge(clk);
                    exit when word_ready = '1';
                end loop;
            end loop;
            word_event <= C_GPX_VDMA_WORD_EVENT_IDLE;
        end procedure send_line;

        procedure expect_line(
            constant line_tag      : in natural;
            constant slot_count    : in positive;
            constant word_count    : in positive;
            constant first_column  : in boolean;
            constant faulted       : in boolean;
            constant inject_stalls : in boolean;
            constant abort_output  : in boolean := false
        ) is
            variable payload_words : positive := slot_count * word_count;
            variable payload_blocks : positive :=
                (payload_words + C_GPX_VDMA_WORDS_PER_BLOCK - 1) /
                C_GPX_VDMA_WORDS_PER_BLOCK;
            variable expected_hsize : positive := C_GPX_VDMA_PREFIX_BYTES +
                payload_blocks * C_GPX_VDMA_BLOCK_BYTES;
            variable expected_beats : positive := expected_hsize /
                (G_OUTPUT_WIDTH / 8);
            variable held_data : std_logic_vector(
                G_OUTPUT_WIDTH - 1 downto 0);
            variable held_last : std_logic;
            variable held_user : std_logic;
            variable handshake_on_observe : boolean;
        begin
            assert to_integer(hsize_bytes) = expected_hsize
                report "V2-B9-J2-TB HSIZE mismatch"
                severity failure;

            if inject_stalls or abort_output then
                axis_tready <= '0';
            else
                axis_tready <= '1';
            end if;

            for beat_index in 0 to expected_beats - 1 loop
                loop
                    wait until rising_edge(clk);
                    exit when axis_tvalid = '1';
                end loop;
                handshake_on_observe := axis_tready = '1';

                assert axis_tdata = fn_expected_beat(
                           line_tag, first_column, payload_words, beat_index)
                    report "V2-B9-J2-TB byte stream mismatch line=" &
                        integer'image(line_tag) & " beat=" &
                        integer'image(beat_index) & " actual=0x" &
                        to_hstring(axis_tdata) & " expected=0x" &
                        to_hstring(fn_expected_beat(
                            line_tag, first_column, payload_words, beat_index))
                    severity failure;
                assert axis_tkeep = (axis_tkeep'range => '1') and
                       axis_tstrb = (axis_tstrb'range => '1')
                    report "V2-B9-J2-TB TKEEP/TSTRB must be full"
                    severity failure;
                if beat_index = 0 and first_column then
                    assert axis_tuser(0) = '1'
                        report "V2-B9-J2-TB missing SOF"
                        severity failure;
                else
                    assert axis_tuser(0) = '0'
                        report "V2-B9-J2-TB unexpected SOF"
                        severity failure;
                end if;
                if beat_index = expected_beats - 1 then
                    assert axis_tlast = '1'
                        report "V2-B9-J2-TB missing EOL"
                        severity failure;
                else
                    assert axis_tlast = '0'
                        report "V2-B9-J2-TB early EOL"
                        severity failure;
                end if;

                if abort_output and beat_index = 0 then
                    abort_run <= '1';
                    wait until rising_edge(clk);
                    abort_run <= '0';
                end if;

                if (inject_stalls and (beat_index mod 5) = 1) or
                   (abort_output and beat_index = 0) then
                    held_data := axis_tdata;
                    held_last := axis_tlast;
                    held_user := axis_tuser(0);
                    for hold_cycle in 0 to 1 loop
                        wait until rising_edge(clk);
                        assert axis_tvalid = '1' and
                               axis_tdata = held_data and
                               axis_tlast = held_last and
                               axis_tuser(0) = held_user
                            report "V2-B9-J2-TB output changed under stall"
                            severity failure;
                    end loop;
                end if;

                if not handshake_on_observe then
                    axis_tready <= '1';
                    wait until rising_edge(clk);
                end if;
                if inject_stalls and beat_index < expected_beats - 1 then
                    axis_tready <= '0';
                end if;
            end loop;

            loop
                wait until rising_edge(clk);
                exit when line_done = '1';
            end loop;
            if faulted or abort_output then
                assert line_done_faulted = '1'
                    report "V2-B9-J2-TB missing faulted completion"
                    severity failure;
            else
                assert line_done_faulted = '0'
                    report "V2-B9-J2-TB unexpected faulted completion"
                    severity failure;
            end if;
            wait until rising_edge(clk);
            assert idle = '1'
                report "V2-B9-J2-TB formatter did not return idle"
                severity failure;
            axis_tready <= '0';
        end procedure expect_line;

        procedure cancel_partial_line is
            variable value : gpx_vdma_word_event_t :=
                C_GPX_VDMA_WORD_EVENT_IDLE;
        begin
            value.valid := '1';
            value.data := fn_payload_word(16#EE#, 0);
            value.word_index := to_unsigned(0, value.word_index'length);
            value.word_count := to_unsigned(5, value.word_count'length);
            value.cell_start := '1';
            value.line_start := '1';
            value.slot_index := to_unsigned(0, value.slot_index'length);
            value.slot_count := to_unsigned(32, value.slot_count'length);
            word_event <= value;
            wait until rising_edge(clk) and word_ready = '1';
            word_event <= C_GPX_VDMA_WORD_EVENT_IDLE;
            abort_run <= '1';
            wait until rising_edge(clk);
            abort_run <= '0';
            wait until rising_edge(clk);
            assert idle = '1' and axis_tvalid = '0'
                report "V2-B9-J2-TB capture abort leaked partial AXIS data"
                severity failure;
        end procedure cancel_partial_line;

    begin
        wait for 8 * C_CLK_PERIOD;
        rst_n <= '1';
        wait until rising_edge(clk);

        send_line(1, 1, 2, true, false);
        expect_line(1, 1, 2, true, false, true);

        send_line(2, 3, 3, false, false);
        expect_line(2, 3, 3, false, false, false);

        send_line(3, 5, 4, false, true);
        expect_line(3, 5, 4, false, true, true);

        cancel_partial_line;

        send_line(4, 2, 2, false, false);
        expect_line(4, 2, 2, false, false, true, true);

        send_line(5, 32, 5, true, false);
        expect_line(5, 32, 5, true, false, false);

        report "LIDAR_V2_GPX_VDMA_LANE_FORMATTER_PASS" severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_32_150 is
end entity tb_lidar_gpx_vdma_lane_formatter_32_150;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_32_150 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 32, G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_64_150 is
end entity tb_lidar_gpx_vdma_lane_formatter_64_150;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_64_150 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 64, G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_128_150 is
end entity tb_lidar_gpx_vdma_lane_formatter_128_150;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_128_150 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 128, G_CLK_MHZ => 150);
end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_32_200 is
end entity tb_lidar_gpx_vdma_lane_formatter_32_200;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_32_200 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 32, G_CLK_MHZ => 200);
end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_64_200 is
end entity tb_lidar_gpx_vdma_lane_formatter_64_200;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_64_200 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 64, G_CLK_MHZ => 200);
end architecture sim;

entity tb_lidar_gpx_vdma_lane_formatter_128_200 is
end entity tb_lidar_gpx_vdma_lane_formatter_128_200;

architecture sim of tb_lidar_gpx_vdma_lane_formatter_128_200 is
begin
    u_tb : entity work.tb_lidar_gpx_vdma_lane_formatter
        generic map (G_OUTPUT_WIDTH => 128, G_CLK_MHZ => 200);
end architecture sim;
