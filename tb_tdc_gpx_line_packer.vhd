-- =============================================================================
-- tb_tdc_gpx_line_packer.vhd
-- Width-generic regression for canonical cross-cell word packing.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_line_packer is
    generic (
        G_WIDTH : natural := 32
    );
end entity tb_tdc_gpx_line_packer;

architecture sim of tb_tdc_gpx_line_packer is
    constant C_CLK_PERIOD       : time := 10 ns;
    constant C_MAX_HITS         : natural := 7;
    constant C_CELLS            : natural := 3;
    constant C_WORDS_PER_BEAT   : natural := G_WIDTH / 32;
    constant C_HIT_WORDS        : natural := fn_ceil_div(C_MAX_HITS, 2);
    constant C_HIT_BEATS        : natural :=
        fn_ceil_div(C_MAX_HITS, G_WIDTH / c_HIT_SLOT_DATA_WIDTH);
    constant C_CELL_WORDS       : natural := fn_canonical_cell_words(C_MAX_HITS);
    constant C_DATA_WORDS       : natural := C_CELLS * C_CELL_WORDS;
    constant C_ALIGNED_WORDS    : natural := fn_align_up(C_DATA_WORDS, 4);
    constant C_EXPECTED_BEATS   : natural := C_ALIGNED_WORDS / C_WORDS_PER_BEAT;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal sim_done : boolean := false;
    signal cfg_latch : std_logic := '0';

    signal s_tdata  : std_logic_vector(G_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tlast  : std_logic := '0';
    signal s_tready : std_logic;

    signal m_tdata  : std_logic_vector(G_WIDTH - 1 downto 0);
    signal m_tvalid : std_logic;
    signal m_tlast  : std_logic;
    signal m_tready : std_logic := '1';
    signal pack_idle : std_logic;

    signal cycle_count : natural := 0;

    function fn_expected_word(word_idx : natural) return std_logic_vector is
        variable v_word    : std_logic_vector(31 downto 0) := (others => '0');
        variable v_cell    : natural;
        variable v_in_cell : natural;
    begin
        if word_idx >= C_DATA_WORDS then
            return v_word;
        end if;
        v_cell    := word_idx / C_CELL_WORDS;
        v_in_cell := word_idx mod C_CELL_WORDS;
        if v_in_cell < C_HIT_WORDS then
            v_word := std_logic_vector(to_unsigned(v_cell * 16 + v_in_cell + 1, 32));
        else
            v_word := x"A0000000";
            v_word(7 downto 0) := std_logic_vector(to_unsigned(v_cell, 8));
        end if;
        return v_word;
    end function;

begin
    clk <= not clk after C_CLK_PERIOD / 2 when not sim_done else '0';

    u_dut : entity work.tdc_gpx_line_packer
        generic map (
            g_TDATA_WIDTH => G_WIDTH
        )
        port map (
            i_clk           => clk,
            i_rst_n         => rst_n,
            i_abort         => '0',
            i_max_hits_cfg  => to_unsigned(C_MAX_HITS, 3),
            i_cfg_latch     => cfg_latch,
            i_s_axis_tdata  => s_tdata,
            i_s_axis_tvalid => s_tvalid,
            i_s_axis_tlast  => s_tlast,
            o_s_axis_tready => s_tready,
            o_m_axis_tdata  => m_tdata,
            o_m_axis_tvalid => m_tvalid,
            o_m_axis_tlast  => m_tlast,
            i_m_axis_tready => m_tready,
            o_idle          => pack_idle
        );

    p_backpressure : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                cycle_count <= 0;
                m_tready    <= '1';
            else
                cycle_count <= cycle_count + 1;
                if cycle_count mod 7 = 3 then
                    m_tready <= '0';
                else
                    m_tready <= '1';
                end if;
            end if;
        end if;
    end process;

    p_stim : process
        procedure send_beat(
            constant data_v : in std_logic_vector(G_WIDTH - 1 downto 0);
            constant last_v : in std_logic
        ) is
        begin
            s_tdata  <= data_v;
            s_tlast  <= last_v;
            s_tvalid <= '1';
            loop
                wait until rising_edge(clk);
                exit when s_tready = '1';
            end loop;
            s_tvalid <= '0';
            s_tlast  <= '0';
            s_tdata  <= (others => '0');
        end procedure;

        variable v_beat     : std_logic_vector(G_WIDTH - 1 downto 0);
        variable v_word_idx : natural;
    begin
        rst_n <= '0';
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        cfg_latch <= '1';
        wait until rising_edge(clk);
        cfg_latch <= '0';
        wait until rising_edge(clk);

        for cell_idx in 0 to C_CELLS - 1 loop
            for beat_idx in 0 to C_HIT_BEATS - 1 loop
                v_beat := (others => '0');
                for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                    v_word_idx := beat_idx * C_WORDS_PER_BEAT + lane;
                    if v_word_idx < C_HIT_WORDS then
                        v_beat(32 * lane + 31 downto 32 * lane) :=
                            std_logic_vector(to_unsigned(
                                cell_idx * 16 + v_word_idx + 1, 32));
                    end if;
                end loop;
                send_beat(v_beat, '0');
            end loop;

            v_beat := (others => '0');
            v_beat(31 downto 0) := x"A0000000";
            v_beat(7 downto 0) := std_logic_vector(to_unsigned(cell_idx, 8));
            if cell_idx = C_CELLS - 1 then
                send_beat(v_beat, '1');
            else
                send_beat(v_beat, '0');
            end if;
        end loop;

        wait until pack_idle = '1';
        wait for 2 * C_CLK_PERIOD;
        report "PASS: line_packer width=" & integer'image(G_WIDTH)
             & " canonical words=" & integer'image(C_DATA_WORDS)
             & " aligned words=" & integer'image(C_ALIGNED_WORDS)
            severity note;
        sim_done <= true;
        stop;
        wait;
    end process;

    p_check : process(clk)
        variable v_word_idx : natural := 0;
        variable v_beats    : natural := 0;
        variable v_actual   : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_word_idx := 0;
                v_beats    := 0;
            elsif m_tvalid = '1' and m_tready = '1' then
                for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                    v_actual := m_tdata(32 * lane + 31 downto 32 * lane);
                    assert v_actual = fn_expected_word(v_word_idx)
                        report "line_packer word mismatch at index "
                             & integer'image(v_word_idx)
                        severity error;
                    v_word_idx := v_word_idx + 1;
                end loop;
                v_beats := v_beats + 1;
                if v_beats = C_EXPECTED_BEATS then
                    assert m_tlast = '1'
                        report "line_packer final beat missing tlast"
                        severity error;
                    assert v_word_idx = C_ALIGNED_WORDS
                        report "line_packer aligned word count mismatch"
                        severity error;
                else
                    assert m_tlast = '0'
                        report "line_packer asserted tlast early"
                        severity error;
                end if;
            end if;
        end if;
    end process;

end architecture sim;
