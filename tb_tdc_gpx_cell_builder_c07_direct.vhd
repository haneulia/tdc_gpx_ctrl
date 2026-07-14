--------------------------------------------------------------------------------
-- tb_tdc_gpx_cell_builder_c07_direct.vhd
--
-- C07 chain-audit direct regression for C03 cell_builder behavior.
--
-- Scenarios:
--   G_SCENARIO=0 : output-width x max_hits matrix, beat packing, metadata
--   G_SCENARIO=1 : IFIFO2 wait timeout emits a faulted synthetic EOS
--   G_SCENARIO=2 : dual-buffer next-shot collection while prior output stalls
--   G_SCENARIO=3 : no-free-buffer drop, DROP->QUARANTINE, clean final-drain exit
--
-- Standard: VHDL-2008
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_cell_builder_c07_direct is
    generic (
        G_TDATA_WIDTH : natural := 64;
        G_MAX_HITS    : natural := 7;
        G_SCENARIO    : natural := 0
    );
end entity tb_tdc_gpx_cell_builder_c07_direct;

architecture sim of tb_tdc_gpx_cell_builder_c07_direct is

    constant c_CLK_PERIOD       : time := 5 ns;
    constant c_SLOTS_PER_BEAT_G : natural := fn_slots_per_beat(G_TDATA_WIDTH);
    constant c_BEATS_PER_CELL_G : natural := fn_beats_per_cell_rt(G_MAX_HITS, G_TDATA_WIDTH);

    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';
    signal s_done  : boolean := false;

    signal s_s_tvalid : std_logic := '0';
    signal s_s_tdata  : t_evt_axis_tdata := (others => '0');
    signal s_s_tuser  : t_evt_axis_tuser := (others => '0');
    signal s_s_tready : std_logic;

    signal s_shot_start : std_logic := '0';
    signal s_abort      : std_logic := '0';
    signal s_stops      : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_max_hits   : unsigned(2 downto 0) := to_unsigned(G_MAX_HITS, 3);
    signal s_max_range  : unsigned(15 downto 0) := to_unsigned(1, 16);

    signal s_m_tdata  : std_logic_vector(G_TDATA_WIDTH - 1 downto 0);
    signal s_m_tvalid : std_logic;
    signal s_m_tlast  : std_logic;
    signal s_m_tuser  : std_logic_vector(0 downto 0);
    signal s_m_tready : std_logic := '1';

    signal s_slice_done        : std_logic;
    signal s_hit_dropped       : std_logic;
    signal s_stop_id_error     : std_logic;
    signal s_shot_dropped      : std_logic;
    signal s_slice_timeout     : std_logic;
    signal s_quarantine_escape : std_logic;

    signal s_shot_drop_count : natural := 0;
    signal s_timeout_count   : natural := 0;
    signal s_tlast_count     : natural := 0;

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_dut : entity work.tdc_gpx_cell_builder
        generic map (
            g_CHIP_ID                => 0,
            g_TDATA_WIDTH            => G_TDATA_WIDTH,
            g_QUARANTINE_MARGIN_CLKS => 8,
            g_IFIFO2_MARGIN_CLKS     => 8
        )
        port map (
            i_clk               => s_clk,
            i_rst_n             => s_rst_n,
            i_s_axis_tvalid     => s_s_tvalid,
            i_s_axis_tdata      => s_s_tdata,
            i_s_axis_tuser      => s_s_tuser,
            o_s_axis_tready     => s_s_tready,
            i_shot_start        => s_shot_start,
            i_abort             => s_abort,
            i_stops_per_chip    => s_stops,
            i_max_hits_cfg      => s_max_hits,
            i_max_range_axis_clks => s_max_range,
            o_m_axis_tdata      => s_m_tdata,
            o_m_axis_tvalid     => s_m_tvalid,
            o_m_axis_tlast      => s_m_tlast,
            o_m_axis_tuser      => s_m_tuser,
            i_m_axis_tready     => s_m_tready,
            o_slice_done        => s_slice_done,
            o_hit_dropped_any   => s_hit_dropped,
            o_stop_id_error     => s_stop_id_error,
            o_shot_dropped      => s_shot_dropped,
            o_slice_timeout     => s_slice_timeout,
            o_quarantine_escape_sticky => s_quarantine_escape
        );

    p_status_watch : process(s_clk)
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                s_shot_drop_count <= 0;
                s_timeout_count   <= 0;
                s_tlast_count     <= 0;
            else
                if s_shot_dropped = '1' then
                    s_shot_drop_count <= s_shot_drop_count + 1;
                end if;
                if s_slice_timeout = '1' then
                    s_timeout_count <= s_timeout_count + 1;
                end if;
                if s_m_tvalid = '1' and s_m_tready = '1' and s_m_tlast = '1' then
                    s_tlast_count <= s_tlast_count + 1;
                end if;
            end if;
        end if;
    end process p_status_watch;

    p_stim : process
        function fn_tuser_data(
            slope    : std_logic;
            stop_id  : natural;
            ififo_id : std_logic;
            hit_seq  : natural;
            shot_seq : natural
        ) return t_evt_axis_tuser is
            variable v : t_evt_axis_tuser := (others => '0');
        begin
            v(0)           := slope;
            v(2 downto 1)  := "00";
            v(5 downto 3)  := std_logic_vector(to_unsigned(stop_id, 3));
            v(6)           := ififo_id;
            v(7)           := '0';
            v(10 downto 8) := std_logic_vector(to_unsigned(hit_seq mod 8, 3));
            v(15 downto 11) := std_logic_vector(to_unsigned(shot_seq mod 32, 5));
            return v;
        end function;

        function fn_tuser_drain(ififo_id : std_logic; faulted : std_logic) return t_evt_axis_tuser is
            variable v : t_evt_axis_tuser := (others => '0');
        begin
            v(5) := faulted;
            v(6) := ififo_id;
            v(7) := '1';
            return v;
        end function;

        function fn_hit(base : natural; seq : natural) return t_evt_axis_tdata is
            variable v : t_evt_axis_tdata := (others => '0');
        begin
            v(15 downto 0) := std_logic_vector(to_unsigned(base + seq, 16));
            if (seq mod 2) = 0 then
                v(16) := '1';
            else
                v(16) := '0';
            end if;
            return v;
        end function;

        function fn_expected_valid(count : natural) return std_logic_vector is
            variable v : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0) := (others => '0');
        begin
            for i in 0 to c_MAX_HITS_PER_STOP - 1 loop
                if i < count then
                    v(i) := '1';
                end if;
            end loop;
            return v;
        end function;

        function fn_expected_msb(count : natural) return std_logic_vector is
            variable v : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0) := (others => '0');
        begin
            for i in 0 to c_MAX_HITS_PER_STOP - 1 loop
                if i < count and (i mod 2) = 0 then
                    v(i) := '1';
                end if;
            end loop;
            return v;
        end function;

        procedure wait_cycles(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(s_clk);
                wait for 0 ns;
            end loop;
        end procedure;

        procedure reset_dut is
        begin
            s_rst_n      <= '0';
            s_s_tvalid   <= '0';
            s_s_tdata    <= (others => '0');
            s_s_tuser    <= (others => '0');
            s_shot_start <= '0';
            s_abort      <= '0';
            s_m_tready   <= '1';
            wait_cycles(8);
            s_rst_n <= '1';
            wait_cycles(4);
        end procedure;

        procedure pulse_shot is
        begin
            s_shot_start <= '1';
            wait_cycles(1);
            s_shot_start <= '0';
            wait_cycles(1);
        end procedure;

        procedure send_evt(data : t_evt_axis_tdata; user : t_evt_axis_tuser) is
            variable v_timeout : natural := 0;
        begin
            s_s_tdata  <= data;
            s_s_tuser  <= user;
            s_s_tvalid <= '1';
            loop
                wait_cycles(1);
                exit when s_s_tready = '1';
                v_timeout := v_timeout + 1;
                assert v_timeout < 2000
                    report "FAIL: input event tready timeout"
                    severity failure;
            end loop;
            s_s_tvalid <= '0';
            wait_cycles(1);
        end procedure;

        procedure send_hits(base : natural; count : natural; shot_seq : natural) is
        begin
            for i in 0 to count - 1 loop
                send_evt(fn_hit(base, i), fn_tuser_data('1', 0, '0', i, shot_seq));
            end loop;
        end procedure;

        procedure check_data_beat(base : natural; beat_idx : natural; count : natural) is
            variable v_slot_idx : natural;
            variable v_got      : unsigned(15 downto 0);
            variable v_exp      : unsigned(15 downto 0);
        begin
            for sl in 0 to c_SLOTS_PER_BEAT_G - 1 loop
                v_slot_idx := beat_idx * c_SLOTS_PER_BEAT_G + sl;
                v_got := unsigned(s_m_tdata((sl + 1) * c_HIT_SLOT_DATA_WIDTH - 1
                                             downto sl * c_HIT_SLOT_DATA_WIDTH));
                if v_slot_idx < count then
                    v_exp := to_unsigned(base + v_slot_idx, 16);
                else
                    v_exp := (others => '0');
                end if;
                assert v_got = v_exp
                    report "FAIL: data beat slot mismatch, width=" & integer'image(G_TDATA_WIDTH) &
                           " max_hits=" & integer'image(G_MAX_HITS) &
                           " beat=" & integer'image(beat_idx) &
                           " slot=" & integer'image(sl)
                    severity failure;
            end loop;
        end procedure;

        procedure check_meta_beat(count : natural) is
            variable v_valid : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
            variable v_msb   : std_logic_vector(c_MAX_HITS_PER_STOP - 1 downto 0);
        begin
            v_valid := fn_expected_valid(count);
            v_msb   := fn_expected_msb(count);
            assert s_m_tdata(31 downto 25) = v_valid
                report "FAIL: metadata hit_valid mismatch"
                severity failure;
            assert s_m_tdata(24 downto 18) = v_valid
                report "FAIL: metadata slope_vec mismatch"
                severity failure;
            assert unsigned(s_m_tdata(15 downto 12)) = to_unsigned(count, 4)
                report "FAIL: metadata hit_count mismatch"
                severity failure;
            assert s_m_tdata(11) = '0'
                report "FAIL: metadata hit_dropped unexpectedly set"
                severity failure;
            assert s_m_tdata(10) = '0'
                report "FAIL: metadata error_fill unexpectedly set"
                severity failure;
            assert s_m_tdata(9 downto 8) = "00"
                report "FAIL: metadata chip_id mismatch"
                severity failure;
            assert s_m_tdata(6 downto 0) = v_msb
                report "FAIL: metadata Hit[16] vector mismatch"
                severity failure;
        end procedure;

        procedure expect_slice(base : natural; count : natural; label_text : string) is
            variable v_timeout : natural := 0;
            variable v_beat    : natural := 0;
            variable v_seen    : boolean := false;
        begin
            while not v_seen loop
                wait_cycles(1);
                if s_m_tvalid = '1' and s_m_tready = '1' then
                    if v_beat < c_BEATS_PER_CELL_G - 1 then
                        check_data_beat(base, v_beat, count);
                        assert s_m_tlast = '0'
                            report "FAIL: early tlast in " & label_text
                            severity failure;
                    else
                        check_meta_beat(count);
                        assert s_m_tlast = '1'
                            report "FAIL: final metadata beat missing tlast in " & label_text
                            severity failure;
                        assert s_m_tuser(0) = '0'
                            report "FAIL: non-faulted slice has tuser fault bit set"
                            severity failure;
                        v_seen := true;
                    end if;
                    v_beat := v_beat + 1;
                end if;
                v_timeout := v_timeout + 1;
                assert v_timeout < 2000
                    report "FAIL: output slice timeout in " & label_text
                    severity failure;
            end loop;
            assert v_beat = c_BEATS_PER_CELL_G
                report "FAIL: output beat count mismatch in " & label_text
                severity failure;
        end procedure;

        procedure expect_ififo2_timeout_slice is
            constant c_EXPECTED_BEATS : natural := 4 * c_BEATS_PER_CELL_G + 1;
            variable v_timeout : natural := 0;
            variable v_beats   : natural := 0;
            variable v_seen    : boolean := false;
        begin
            while not v_seen loop
                wait_cycles(1);
                if s_m_tvalid = '1' and s_m_tready = '1' then
                    v_beats := v_beats + 1;
                    if s_m_tlast = '1' then
                        assert v_beats = c_EXPECTED_BEATS
                            report "FAIL: IFIFO2 timeout beat count mismatch"
                            severity failure;
                        assert s_m_tdata = (s_m_tdata'range => '0')
                            report "FAIL: IFIFO2 timeout synthetic EOS data is not zero"
                            severity failure;
                        assert s_m_tuser(0) = '1'
                            report "FAIL: IFIFO2 timeout synthetic EOS missing faulted tuser bit"
                            severity failure;
                        v_seen := true;
                    end if;
                end if;
                v_timeout := v_timeout + 1;
                assert v_timeout < 2000
                    report "FAIL: IFIFO2 timeout scenario did not finish"
                    severity failure;
            end loop;
            assert s_timeout_count >= 1
                report "FAIL: IFIFO2 timeout status pulse was not observed"
                severity failure;
        end procedure;

        procedure wait_for_output_valid is
            variable v_timeout : natural := 0;
        begin
            loop
                wait_cycles(1);
                exit when s_m_tvalid = '1';
                v_timeout := v_timeout + 1;
                assert v_timeout < 1000
                    report "FAIL: output valid did not assert"
                    severity failure;
            end loop;
        end procedure;

        procedure wait_for_drop_count(target : natural) is
            variable v_timeout : natural := 0;
        begin
            loop
                wait_cycles(1);
                exit when s_shot_drop_count >= target;
                v_timeout := v_timeout + 1;
                assert v_timeout < 2000
                    report "FAIL: shot drop counter did not reach target"
                    severity failure;
            end loop;
        end procedure;

        procedure scenario_matrix is
        begin
            reset_dut;
            s_stops    <= to_unsigned(1, 4);
            s_max_hits <= to_unsigned(G_MAX_HITS, 3);
            wait_cycles(2);
            pulse_shot;
            send_hits(16#1000#, G_MAX_HITS, 0);
            send_evt((others => '0'), fn_tuser_drain('0', '0'));
            expect_slice(16#1000#, G_MAX_HITS, "matrix");
            assert s_hit_dropped = '0' and s_stop_id_error = '0'
                report "FAIL: unexpected data-path status pulse in matrix scenario"
                severity failure;
            assert s_shot_drop_count = 0 and s_timeout_count = 0
                report "FAIL: unexpected drop/timeout in matrix scenario"
                severity failure;
            report "PASS: C07 C03 direct matrix width=" & integer'image(G_TDATA_WIDTH) &
                   " max_hits=" & integer'image(G_MAX_HITS) &
                   " beats_per_cell=" & integer'image(c_BEATS_PER_CELL_G)
                severity note;
        end procedure;

        procedure scenario_ififo2_timeout is
        begin
            reset_dut;
            s_stops    <= to_unsigned(8, 4);
            s_max_hits <= to_unsigned(G_MAX_HITS, 3);
            s_max_range <= to_unsigned(1, 16);
            wait_cycles(2);
            pulse_shot;
            send_evt((others => '0'), fn_tuser_drain('0', '0'));
            expect_ififo2_timeout_slice;
            report "PASS: C07 C03 IFIFO2 timeout width=" & integer'image(G_TDATA_WIDTH) &
                   " max_hits=" & integer'image(G_MAX_HITS)
                severity note;
        end procedure;

        procedure scenario_dual_buffer is
        begin
            reset_dut;
            s_stops    <= to_unsigned(1, 4);
            s_max_hits <= to_unsigned(G_MAX_HITS, 3);
            s_m_tready <= '0';
            wait_cycles(2);

            pulse_shot;
            send_hits(16#2000#, G_MAX_HITS, 1);
            send_evt((others => '0'), fn_tuser_drain('0', '0'));
            wait_for_output_valid;

            pulse_shot;
            send_hits(16#3000#, G_MAX_HITS, 2);
            send_evt((others => '0'), fn_tuser_drain('0', '0'));
            wait_cycles(8);

            assert s_shot_drop_count = 0
                report "FAIL: dual-buffer next-shot caused an unexpected shot drop"
                severity failure;
            s_m_tready <= '1';
            expect_slice(16#2000#, G_MAX_HITS, "dual-buffer shot1");
            expect_slice(16#3000#, G_MAX_HITS, "dual-buffer shot2");
            assert s_shot_drop_count = 0
                report "FAIL: dual-buffer scenario observed shot drop after release"
                severity failure;
            report "PASS: C07 C03 dual-buffer next-shot II width=" & integer'image(G_TDATA_WIDTH) &
                   " max_hits=" & integer'image(G_MAX_HITS)
                severity note;
        end procedure;

        procedure scenario_drop_quarantine is
            variable v_tlast_after : natural;
        begin
            reset_dut;
            s_stops     <= to_unsigned(1, 4);
            s_max_hits  <= to_unsigned(G_MAX_HITS, 3);
            s_max_range <= to_unsigned(1, 16);
            s_m_tready  <= '0';
            wait_cycles(2);

            pulse_shot;
            send_hits(16#4000#, G_MAX_HITS, 1);
            send_evt((others => '0'), fn_tuser_drain('0', '0'));
            wait_for_output_valid;

            pulse_shot;
            send_hits(16#5000#, G_MAX_HITS, 2);
            send_evt((others => '0'), fn_tuser_drain('0', '0'));

            pulse_shot;
            wait_for_drop_count(1);
            wait_for_drop_count(2);
            send_evt((others => '0'), fn_tuser_drain('1', '0'));

            assert s_quarantine_escape = '0'
                report "FAIL: clean quarantine exit unexpectedly latched escape sticky"
                severity failure;
            s_m_tready <= '1';
            expect_slice(16#4000#, G_MAX_HITS, "drop/quarantine shot1");
            expect_slice(16#5000#, G_MAX_HITS, "drop/quarantine shot2");
            v_tlast_after := s_tlast_count;
            wait_cycles(40);
            assert s_tlast_count = v_tlast_after
                report "FAIL: dropped shot produced an unexpected output slice"
                severity failure;
            assert s_shot_drop_count >= 2
                report "FAIL: DROP and QUARANTINE transition pulses were not observed"
                severity failure;
            report "PASS: C07 C03 drop/quarantine width=" & integer'image(G_TDATA_WIDTH) &
                   " max_hits=" & integer'image(G_MAX_HITS)
                severity note;
        end procedure;

    begin
        assert fn_output_width_supported(G_TDATA_WIDTH)
            report "FAIL: unsupported G_TDATA_WIDTH"
            severity failure;
        assert G_MAX_HITS = 1 or G_MAX_HITS = 3 or G_MAX_HITS = 5 or G_MAX_HITS = 7
            report "FAIL: this C07 direct TB sweeps max_hits 1/3/5/7"
            severity failure;

        case G_SCENARIO is
            when 0 =>
                scenario_matrix;
            when 1 =>
                scenario_ififo2_timeout;
            when 2 =>
                scenario_dual_buffer;
            when 3 =>
                scenario_drop_quarantine;
            when others =>
                assert false
                    report "FAIL: unsupported G_SCENARIO"
                    severity failure;
        end case;

        s_done <= true;
        wait_cycles(2);
        finish;
        wait;
    end process p_stim;

    p_watchdog : process
    begin
        wait for 200 us;
        assert s_done
            report "FAIL: tb_tdc_gpx_cell_builder_c07_direct watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
