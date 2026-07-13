--------------------------------------------------------------------------------
-- tb_tdc_gpx_face_assembler_c07_direct.vhd
--
-- C07 CHAIN-P1-02 direct regression for C04 face_assembler ready boundary.
--
-- Scenario:
--   - active chips 0 and 1
--   - stops_per_chip = 8, max_hits_cfg = 7
--   - downstream m_axis_tready is held low long enough to fill/stall the
--     output FIFO path, then periodically toggled
--   - both chip streams are accepted through the input ready boundary and
--     later emitted in strict chip order without beat loss or stale ready
--
-- Standard: VHDL-2008
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_face_assembler_c07_direct is
    generic (
        G_TDATA_WIDTH : natural := 64
    );
end entity tb_tdc_gpx_face_assembler_c07_direct;

architecture sim of tb_tdc_gpx_face_assembler_c07_direct is

    constant c_CLK_PERIOD      : time := 5 ns;
    constant c_STOPS_PER_CHIP  : natural := 8;
    constant c_MAX_HITS        : natural := 7;
    constant c_BEATS_PER_CELL  : natural := fn_beats_per_cell_rt(c_MAX_HITS, G_TDATA_WIDTH);
    constant c_BEATS_PER_CHIP  : natural := c_STOPS_PER_CHIP * c_BEATS_PER_CELL;
    constant c_ACTIVE_CHIPS    : natural := 2;
    constant c_TOTAL_OUT_BEATS : natural := c_ACTIVE_CHIPS * c_BEATS_PER_CHIP;

    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';
    signal s_done  : boolean := false;

    signal s_tdata_0 : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tdata_1 : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tdata_2 : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tdata_3 : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_tuser   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    signal s_shot_start : std_logic := '0';
    signal s_abort      : std_logic := '0';

    signal m_tdata  : std_logic_vector(G_TDATA_WIDTH - 1 downto 0);
    signal m_tvalid : std_logic;
    signal m_tlast  : std_logic;
    signal m_tready : std_logic := '0';

    signal s_row_done           : std_logic;
    signal s_row_done_faulted   : std_logic;
    signal s_chip_error_flags   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_partial : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_blank   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_overrun       : std_logic;
    signal s_face_abort         : std_logic;
    signal s_idle               : std_logic;
    signal s_shot_flush_drop    : std_logic;
    signal s_shot_flush_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_overrun_count      : unsigned(7 downto 0);

    signal s_out_count      : natural := 0;
    signal s_tlast_count    : natural := 0;
    signal s_row_done_count : natural := 0;
    signal s_stall_seen     : std_logic := '0';

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_dut : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => 4,
            g_TDATA_WIDTH    => G_TDATA_WIDTH
        )
        port map (
            i_clk                => s_clk,
            i_rst_n              => s_rst_n,
            i_s_axis_tdata_0     => s_tdata_0,
            i_s_axis_tdata_1     => s_tdata_1,
            i_s_axis_tdata_2     => s_tdata_2,
            i_s_axis_tdata_3     => s_tdata_3,
            i_s_axis_tvalid      => s_tvalid,
            i_s_axis_tlast       => s_tlast,
            o_s_axis_tready      => s_tready,
            i_s_axis_tuser       => s_tuser,
            i_shot_start         => s_shot_start,
            i_abort              => s_abort,
            i_active_chip_mask   => "0011",
            i_stops_per_chip     => to_unsigned(c_STOPS_PER_CHIP, 4),
            i_max_hits_cfg       => to_unsigned(c_MAX_HITS, 3),
            i_max_scan_clks      => to_unsigned(2000, 16),
            o_m_axis_tdata       => m_tdata,
            o_m_axis_tvalid      => m_tvalid,
            o_m_axis_tlast       => m_tlast,
            i_m_axis_tready      => m_tready,
            o_row_done           => s_row_done,
            o_row_done_faulted   => s_row_done_faulted,
            o_chip_error_flags   => s_chip_error_flags,
            o_chip_error_partial => s_chip_error_partial,
            o_chip_error_blank   => s_chip_error_blank,
            o_shot_overrun       => s_shot_overrun,
            o_face_abort         => s_face_abort,
            o_idle               => s_idle,
            o_shot_flush_drop    => s_shot_flush_drop,
            o_shot_flush_drop_mask => s_shot_flush_mask,
            o_shot_overrun_count => s_overrun_count
        );

    p_monitor : process(s_clk)
        function fn_expected_data(chip : natural; beat : natural) return std_logic_vector is
            variable v : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
            variable v_word : natural;
        begin
            v_word := (chip + 1) * 4096 + beat + 1;
            v(15 downto 0) := std_logic_vector(to_unsigned(v_word mod 65536, 16));
            v(23 downto 16) := std_logic_vector(to_unsigned(chip, 8));
            if (beat mod c_BEATS_PER_CELL) = (c_BEATS_PER_CELL - 1) then
                v(31 downto 25) := (others => '1');
                v(24 downto 18) := (others => '1');
                v(15 downto 12) := std_logic_vector(to_unsigned(c_MAX_HITS, 4));
                v(11) := '0';
                v(10) := '0';
                v(9 downto 8) := std_logic_vector(to_unsigned(chip, 2));
                v(6 downto 0) := "1010101";
            end if;
            return v;
        end function;

        variable v_chip : natural;
        variable v_beat : natural;
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                s_out_count      <= 0;
                s_tlast_count    <= 0;
                s_row_done_count <= 0;
                s_stall_seen     <= '0';
            else
                if (s_tvalid and (not s_tready)) /= "0000"
                   or (m_tvalid = '1' and m_tready = '0') then
                    s_stall_seen <= '1';
                end if;
                if m_tvalid = '1' and m_tready = '1' then
                    v_chip := s_out_count / c_BEATS_PER_CHIP;
                    v_beat := s_out_count mod c_BEATS_PER_CHIP;
                    assert v_chip < c_ACTIVE_CHIPS
                        report "FAIL: face_assembler emitted too many beats"
                        severity failure;
                    assert m_tdata = fn_expected_data(v_chip, v_beat)
                        report "FAIL: face_assembler data/order mismatch, width=" &
                               integer'image(G_TDATA_WIDTH) &
                               " out_idx=" & integer'image(s_out_count)
                        severity failure;
                    if s_out_count = c_TOTAL_OUT_BEATS - 1 then
                        assert m_tlast = '1'
                            report "FAIL: final face_assembler beat missing tlast"
                            severity failure;
                    else
                        assert m_tlast = '0'
                            report "FAIL: early face_assembler tlast"
                            severity failure;
                    end if;
                    if m_tlast = '1' then
                        s_tlast_count <= s_tlast_count + 1;
                    end if;
                    s_out_count <= s_out_count + 1;
                end if;

                if s_row_done = '1' then
                    s_row_done_count <= s_row_done_count + 1;
                end if;
            end if;
        end if;
    end process p_monitor;

    p_ready : process
    begin
        m_tready <= '0';
        wait until s_rst_n = '1';
        wait for 2 us;
        while not s_done loop
            m_tready <= '1';
            for i in 0 to 6 loop
                wait until rising_edge(s_clk);
            end loop;
            m_tready <= '0';
            for i in 0 to 2 loop
                wait until rising_edge(s_clk);
            end loop;
        end loop;
        m_tready <= '1';
        wait;
    end process p_ready;

    p_stim : process
        function fn_drive_data(chip : natural; beat : natural) return std_logic_vector is
            variable v : std_logic_vector(G_TDATA_WIDTH - 1 downto 0) := (others => '0');
            variable v_word : natural;
        begin
            v_word := (chip + 1) * 4096 + beat + 1;
            v(15 downto 0) := std_logic_vector(to_unsigned(v_word mod 65536, 16));
            v(23 downto 16) := std_logic_vector(to_unsigned(chip, 8));
            if (beat mod c_BEATS_PER_CELL) = (c_BEATS_PER_CELL - 1) then
                v(31 downto 25) := (others => '1');
                v(24 downto 18) := (others => '1');
                v(15 downto 12) := std_logic_vector(to_unsigned(c_MAX_HITS, 4));
                v(11) := '0';
                v(10) := '0';
                v(9 downto 8) := std_logic_vector(to_unsigned(chip, 2));
                v(6 downto 0) := "1010101";
            end if;
            return v;
        end function;

        procedure wait_cycles(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(s_clk);
            end loop;
            wait for 0 ns;
        end procedure;

        procedure pulse_shot is
        begin
            s_shot_start <= '1';
            wait_cycles(1);
            s_shot_start <= '0';
            wait_cycles(4);
        end procedure;

        procedure send_chip(chip : natural) is
            variable v_timeout : natural;
            variable v_valid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
            variable v_last    : std_logic_vector(c_N_CHIPS - 1 downto 0);
        begin
            for beat in 0 to c_BEATS_PER_CHIP - 1 loop
                v_valid := (others => '0');
                v_last  := (others => '0');
                v_valid(chip) := '1';
                if beat = c_BEATS_PER_CHIP - 1 then
                    v_last(chip) := '1';
                    s_tuser(chip) <= '1';
                else
                    s_tuser(chip) <= '0';
                end if;

                if chip = 0 then
                    s_tdata_0 <= fn_drive_data(chip, beat);
                elsif chip = 1 then
                    s_tdata_1 <= fn_drive_data(chip, beat);
                elsif chip = 2 then
                    s_tdata_2 <= fn_drive_data(chip, beat);
                else
                    s_tdata_3 <= fn_drive_data(chip, beat);
                end if;
                s_tlast  <= v_last;
                s_tvalid <= v_valid;

                v_timeout := 0;
                loop
                    wait_cycles(1);
                    exit when s_tready(chip) = '1';
                    v_timeout := v_timeout + 1;
                    assert v_timeout < 5000
                        report "FAIL: face_assembler input ready timeout"
                        severity failure;
                end loop;
            end loop;

            s_tvalid <= (others => '0');
            s_tlast  <= (others => '0');
            s_tuser  <= (others => '0');
            wait_cycles(1);
        end procedure;
    begin
        assert fn_output_width_supported(G_TDATA_WIDTH)
            report "FAIL: unsupported G_TDATA_WIDTH"
            severity failure;

        s_rst_n <= '0';
        s_abort <= '0';
        wait_cycles(10);
        s_rst_n <= '1';
        wait_cycles(5);

        pulse_shot;
        send_chip(0);
        send_chip(1);

        wait until s_out_count = c_TOTAL_OUT_BEATS for 100 us;
        wait_cycles(20);

        assert s_out_count = c_TOTAL_OUT_BEATS
            report "FAIL: face_assembler output beat count mismatch"
            severity failure;
        assert s_tlast_count = 1
            report "FAIL: face_assembler tlast count mismatch"
            severity failure;
        assert s_row_done_count = 1
            report "FAIL: face_assembler row_done count mismatch"
            severity failure;
        assert s_stall_seen = '1'
            report "FAIL: face_assembler stall/ready path was not stressed"
            severity failure;
        assert s_row_done_faulted = '0'
            report "FAIL: clean face_assembler row reported faulted"
            severity failure;
        assert s_chip_error_flags = "0000"
            report "FAIL: unexpected face_assembler chip error"
            severity failure;
        assert s_shot_overrun = '0' and s_overrun_count = 0
            report "FAIL: unexpected face_assembler shot overrun"
            severity failure;
        assert s_face_abort = '0'
            report "FAIL: deprecated face_abort unexpectedly asserted"
            severity failure;

        report "PASS: C07 C04 face_assembler ready boundary width=" &
               integer'image(G_TDATA_WIDTH) &
               " beats=" & integer'image(c_TOTAL_OUT_BEATS)
            severity note;

        s_done <= true;
        wait_cycles(2);
        finish;
        wait;
    end process p_stim;

    p_watchdog : process
    begin
        wait for 200 us;
        assert s_done
            report "FAIL: tb_tdc_gpx_face_assembler_c07_direct watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
