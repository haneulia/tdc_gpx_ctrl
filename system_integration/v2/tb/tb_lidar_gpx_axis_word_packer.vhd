library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_axis_word_packer is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
end entity tb_lidar_gpx_axis_word_packer;

architecture sim of tb_lidar_gpx_axis_word_packer is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH / 32;
    constant C_LINE_WORDS : positive := 9;
    constant C_BEATS_PER_LINE : positive :=
        (C_LINE_WORDS + C_WORDS_PER_BEAT - 1) / C_WORDS_PER_BEAT;

    signal clk       : std_logic := '0';
    signal rst_n     : std_logic := '0';
    signal abort_run : std_logic := '0';
    signal force_stall : std_logic := '0';

    signal line_word       : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal line_word_ready : std_logic;
    signal tdata  : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal tkeep  : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tstrb  : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal tuser  : std_logic_vector(0 downto 0);
    signal tvalid : std_logic;
    signal tlast  : std_logic;
    signal tready : std_logic := '0';
    signal line_done : std_logic;
    signal frame_done : std_logic;
    signal idle : std_logic;
    signal ready_cycle : natural := 0;
    signal output_done : std_logic := '0';

    function fn_word(
        line_index : natural;
        word_index : natural
    ) return gpx_vdma_line_word_event_t is
        variable result : gpx_vdma_line_word_event_t :=
            C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    begin
        result.valid := '1';
        result.data := std_logic_vector(to_unsigned(
            16#10000000# + line_index * 16#01000000# + word_index, 32));
        if word_index < C_GPX_VDMA_SHOT_META_WORDS then
            result.kind := GPX_VDMA_LINE_SHOT_METADATA;
        else
            result.kind := GPX_VDMA_LINE_CELL_DATA;
        end if;
        result.word_index := to_unsigned(word_index, 9);
        result.line_word_count := to_unsigned(C_LINE_WORDS, 9);
        result.line_start := '1' when word_index = 0 else '0';
        result.line_end := '1' when word_index + 1 = C_LINE_WORDS else '0';
        result.first_column := '1' when line_index = 0 else '0';
        result.slot_count := to_unsigned(1, result.slot_count'length);
        result.cell_word_count := to_unsigned(
            C_LINE_WORDS - C_GPX_VDMA_SHOT_META_WORDS,
            result.cell_word_count'length);
        return result;
    end function fn_word;

    function fn_expected_beat(
        line_index : natural;
        beat_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
        variable word_index : natural;
    begin
        for lane_index in 0 to C_WORDS_PER_BEAT - 1 loop
            word_index := beat_index * C_WORDS_PER_BEAT + lane_index;
            if word_index < C_LINE_WORDS then
                result((lane_index + 1) * 32 - 1 downto
                       lane_index * 32) := fn_word(
                    line_index, word_index).data;
            end if;
        end loop;
        return result;
    end function fn_expected_beat;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_axis_word_packer
        generic map (
            G_OUTPUT_WIDTH => G_OUTPUT_WIDTH
        )
        port map (
            i_clk             => clk,
            i_rst_n           => rst_n,
            i_abort           => abort_run,
            i_line_word       => line_word,
            o_line_word_ready => line_word_ready,
            o_m_axis_tdata    => tdata,
            o_m_axis_tkeep    => tkeep,
            o_m_axis_tstrb    => tstrb,
            o_m_axis_tuser    => tuser,
            o_m_axis_tvalid   => tvalid,
            o_m_axis_tlast    => tlast,
            i_m_axis_tready   => tready,
            o_line_done       => line_done,
            o_frame_done      => frame_done,
            o_idle            => idle
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                ready_cycle <= 0;
                tready <= '0';
            else
                ready_cycle <= ready_cycle + 1;
                if force_stall = '1' or ready_cycle mod 5 = 2 or
                   ready_cycle mod 5 = 3 then
                    tready <= '0';
                else
                    tready <= '1';
                end if;
            end if;
        end if;
    end process p_ready;

    p_monitor : process (clk)
        variable beat_count : natural := 0;
        variable line_index : natural;
        variable beat_index : natural;
        variable held_valid : boolean := false;
        variable held_data : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0);
        variable held_user : std_logic_vector(0 downto 0);
        variable held_last : std_logic;
        variable expected_user : std_logic;
        variable expected_last : std_logic;
    begin
        if rising_edge(clk) then
            output_done <= '0';
            if rst_n = '0' or abort_run = '1' then
                beat_count := 0;
                held_valid := false;
            else
                if tvalid = '1' and tready = '0' then
                    if held_valid then
                        assert tdata = held_data and tuser = held_user and
                               tlast = held_last
                            report "V2-B9-J6-TB AXIS changed under stall"
                            severity failure;
                    else
                        held_data := tdata;
                        held_user := tuser;
                        held_last := tlast;
                        held_valid := true;
                    end if;
                else
                    held_valid := false;
                end if;

                if tvalid = '1' and tready = '1' then
                    line_index := beat_count / C_BEATS_PER_LINE;
                    beat_index := beat_count mod C_BEATS_PER_LINE;
                    assert line_index < 2 and
                           tdata = fn_expected_beat(line_index, beat_index)
                        report "V2-B9-J6-TB packed data/order mismatch"
                        severity failure;
                    assert tkeep = (tkeep'range => '1') and
                           tstrb = (tstrb'range => '1')
                        report "V2-B9-J6-TB TKEEP/TSTRB not full"
                        severity failure;
                    expected_user := '0';
                    expected_last := '0';
                    if line_index = 0 and beat_index = 0 then
                        expected_user := '1';
                    end if;
                    if beat_index + 1 = C_BEATS_PER_LINE then
                        expected_last := '1';
                    end if;
                    assert tuser(0) = expected_user and
                           tlast = expected_last
                        report "V2-B9-J6-TB SOF/TLAST mismatch"
                        severity failure;
                    if beat_count + 1 = 2 * C_BEATS_PER_LINE then
                        output_done <= '1';
                    end if;
                    beat_count := beat_count + 1;
                end if;
            end if;
        end if;
    end process p_monitor;

    p_test : process
        procedure send_word(
            constant value : in gpx_vdma_line_word_event_t
        ) is
        begin
            line_word <= value;
            loop
                wait until rising_edge(clk);
                exit when line_word_ready = '1';
            end loop;
            line_word.valid <= '0';
        end procedure send_word;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        for line_index in 0 to 1 loop
            for word_index in 0 to C_LINE_WORDS - 1 loop
                send_word(fn_word(line_index, word_index));
            end loop;
        end loop;

        loop
            wait until rising_edge(clk);
            exit when output_done = '1';
        end loop;
        wait until rising_edge(clk);
        assert idle = '1'
            report "V2-B9-J6-TB packer did not return idle"
            severity failure;

        -- Abort clears either a partial assembly or a held 32-bit Beat.
        force_stall <= '1';
        send_word(fn_word(0, 0));
        wait until rising_edge(clk);
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        force_stall <= '0';
        wait until falling_edge(clk);
        assert idle = '1' and tvalid = '0'
            report "V2-B9-J6-TB abort did not clear packer"
            severity failure;

        report "LIDAR_V2_GPX_AXIS_WORD_PACKER_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_axis_word_packer_150_32 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_150_32 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 32);
end architecture;

entity tb_lidar_gpx_axis_word_packer_150_64 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_150_64 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 64);
end architecture;

entity tb_lidar_gpx_axis_word_packer_150_128 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_150_128 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 128);
end architecture;

entity tb_lidar_gpx_axis_word_packer_200_32 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_200_32 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 32);
end architecture;

entity tb_lidar_gpx_axis_word_packer_200_64 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_200_64 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 64);
end architecture;

entity tb_lidar_gpx_axis_word_packer_200_128 is end entity;
architecture sim of tb_lidar_gpx_axis_word_packer_200_128 is begin
    u : entity work.tb_lidar_gpx_axis_word_packer
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 128);
end architecture;
