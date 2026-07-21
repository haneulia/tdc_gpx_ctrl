-- =============================================================================
-- tb_tdc_gpx_cell_pipe_c03_fix.vhd
--
-- C03 regression checks for:
--   1. Datasheet Hit[16:0] preservation through cell metadata hit_msb_vec.
--   2. Pre-shot accepted event preservation across the cell_pipe input skid.
--   3. Per-slope abort clearing/dropping stale slope-local beats.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_cell_pipe_c03_fix is
end entity;

architecture sim of tb_tdc_gpx_cell_pipe_c03_fix is

    constant c_CLK_PERIOD   : time := 5 ns;
    constant c_OUTPUT_WIDTH : natural := 64;

    signal s_clk    : std_logic := '0';
    signal s_rst_n  : std_logic := '0';
    signal s_done   : boolean := false;

    signal s_evt_tvalid : std_logic_vector(c_MAX_CHIPS-1 downto 0) := (others => '0');
    signal s_evt_tready : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_evt_tdata  : t_evt_axis_tdata_array := (others => (others => '0'));
    signal s_evt_tuser  : t_evt_axis_tuser_array := (others => (others => '0'));

    signal s_shot_start : std_logic_vector(c_MAX_CHIPS-1 downto 0) := (others => '0');
    signal s_abort_rise : std_logic := '0';
    signal s_abort_fall : std_logic := '0';
    signal s_face_stops : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_max_hits   : unsigned(2 downto 0) := to_unsigned(7, 3);

    signal s_cell_rise_tdata_0 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_rise_tdata_1 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_rise_tdata_2 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_rise_tdata_3 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_rise_tvalid  : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_rise_tlast   : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_rise_tuser   : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_rise_tready  : std_logic_vector(c_MAX_CHIPS-1 downto 0) := (others => '1');

    signal s_cell_fall_tdata_0 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_fall_tdata_1 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_fall_tdata_2 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_fall_tdata_3 : std_logic_vector(c_OUTPUT_WIDTH-1 downto 0);
    signal s_cell_fall_tvalid  : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_fall_tlast   : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_fall_tuser   : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_cell_fall_tready  : std_logic_vector(c_MAX_CHIPS-1 downto 0) := (others => '1');

    signal s_hit_dropped      : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_hit_fall_dropped : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_stop_id_error    : std_logic_vector(c_MAX_CHIPS-1 downto 0);
    signal s_stop_id_fall_err : std_logic_vector(c_MAX_CHIPS-1 downto 0);

    function fn_tuser_data(
        slope    : natural;
        chip_id  : natural;
        stop_id  : natural;
        ififo_id : natural
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_EVT_AXIS_TUSER_WIDTH-1 downto 0) := (others => '0');
    begin
        v_result(0)          := std_logic(to_unsigned(slope, 1)(0));
        v_result(2 downto 1) := std_logic_vector(to_unsigned(chip_id, 2));
        v_result(5 downto 3) := std_logic_vector(to_unsigned(stop_id, 3));
        v_result(6)          := std_logic(to_unsigned(ififo_id, 1)(0));
        return v_result;
    end function;

    function fn_tuser_drain(
        ififo_id : natural;
        faulted  : std_logic
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_EVT_AXIS_TUSER_WIDTH-1 downto 0) := (others => '0');
    begin
        v_result(7) := '1';
        v_result(6) := std_logic(to_unsigned(ififo_id, 1)(0));
        v_result(5) := faulted;
        return v_result;
    end function;

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2 when not s_done else '0';

    u_dut : entity work.tdc_gpx_cell_pipe
        generic map (
            g_OUTPUT_WIDTH => c_OUTPUT_WIDTH,
            g_AXIS_CLK_MHZ => 200
        )
        port map (
            i_clk                    => s_clk,
            i_rst_n                  => s_rst_n,
            i_evt_sk_tvalid          => s_evt_tvalid,
            i_evt_sk_tdata           => s_evt_tdata,
            i_evt_sk_tuser           => s_evt_tuser,
            o_evt_sk_tready          => s_evt_tready,
            i_shot_start_per_chip    => s_shot_start,
            i_abort                  => '0',
            i_abort_rise             => s_abort_rise,
            i_abort_fall             => s_abort_fall,
            i_face_stops_per_chip    => s_face_stops,
            i_max_hits_cfg           => s_max_hits,
            i_max_range_5ns_ticks    => (others => '0'),
            o_cell_rise_tdata_0      => s_cell_rise_tdata_0,
            o_cell_rise_tdata_1      => s_cell_rise_tdata_1,
            o_cell_rise_tdata_2      => s_cell_rise_tdata_2,
            o_cell_rise_tdata_3      => s_cell_rise_tdata_3,
            o_cell_rise_tvalid       => s_cell_rise_tvalid,
            o_cell_rise_tlast        => s_cell_rise_tlast,
            i_cell_rise_tready       => s_cell_rise_tready,
            o_cell_fall_tdata_0      => s_cell_fall_tdata_0,
            o_cell_fall_tdata_1      => s_cell_fall_tdata_1,
            o_cell_fall_tdata_2      => s_cell_fall_tdata_2,
            o_cell_fall_tdata_3      => s_cell_fall_tdata_3,
            o_cell_fall_tvalid       => s_cell_fall_tvalid,
            o_cell_fall_tlast        => s_cell_fall_tlast,
            i_cell_fall_tready       => s_cell_fall_tready,
            o_hit_dropped            => s_hit_dropped,
            o_hit_fall_dropped       => s_hit_fall_dropped,
            o_shot_dropped           => open,
            o_shot_fall_dropped      => open,
            o_slice_timeout          => open,
            o_slice_fall_timeout     => open,
            o_stop_id_error          => s_stop_id_error,
            o_stop_id_fall_error     => s_stop_id_fall_err,
            o_quarantine_escape_rise => open,
            o_quarantine_escape_fall => open,
            o_cell_rise_tuser        => s_cell_rise_tuser,
            o_cell_fall_tuser        => s_cell_fall_tuser
        );

    p_stim : process
        procedure reset_dut is
        begin
            s_rst_n <= '0';
            s_evt_tvalid <= (others => '0');
            s_evt_tdata  <= (others => (others => '0'));
            s_evt_tuser  <= (others => (others => '0'));
            s_shot_start <= (others => '0');
            s_abort_rise <= '0';
            s_abort_fall <= '0';
            wait for 10 * c_CLK_PERIOD;
            wait until rising_edge(s_clk);
            s_rst_n <= '1';
            wait until rising_edge(s_clk);
            wait until rising_edge(s_clk);
        end procedure;

        procedure send_evt(
            data_value : std_logic_vector(c_EVT_AXIS_TDATA_WIDTH-1 downto 0);
            user_value : std_logic_vector(c_EVT_AXIS_TUSER_WIDTH-1 downto 0)
        ) is
        begin
            s_evt_tdata(0)  <= data_value;
            s_evt_tuser(0)  <= user_value;
            s_evt_tvalid(0) <= '1';
            loop
                wait until rising_edge(s_clk);
                exit when s_evt_tready(0) = '1';
            end loop;
            s_evt_tvalid(0) <= '0';
            s_evt_tdata(0)  <= (others => '0');
            s_evt_tuser(0)  <= (others => '0');
            wait until rising_edge(s_clk);
        end procedure;

        procedure pulse_shot is
        begin
            s_shot_start(0) <= '1';
            wait until rising_edge(s_clk);
            s_shot_start(0) <= '0';
            wait until rising_edge(s_clk);
        end procedure;

        procedure pulse_abort_fall(cycles : natural) is
        begin
            s_abort_fall <= '1';
            for i in 1 to cycles loop
                wait until rising_edge(s_clk);
            end loop;
            s_abort_fall <= '0';
            wait until rising_edge(s_clk);
        end procedure;

        procedure wait_rise_meta(
            expected_msb_vec : std_logic_vector(c_MAX_HITS_PER_STOP-1 downto 0);
            expected_count   : natural
        ) is
            variable v_timeout : natural := 0;
        begin
            loop
                wait until rising_edge(s_clk);
                if s_cell_rise_tvalid(0) = '1' and s_cell_rise_tlast(0) = '1' then
                    assert s_cell_rise_tdata_0(6 downto 0) = expected_msb_vec
                        report "FAIL: rising metadata hit_msb_vec mismatch"
                        severity error;
                    assert s_cell_rise_tdata_0(31 downto 25) = "1111111"
                        report "FAIL: rising metadata hit_valid mismatch"
                        severity error;
                    assert s_cell_rise_tdata_0(24 downto 18) = "1111111"
                        report "FAIL: rising metadata slope_vec mismatch"
                        severity error;
                    assert s_cell_rise_tdata_0(15 downto 12) = std_logic_vector(to_unsigned(expected_count, 4))
                        report "FAIL: rising metadata hit_count mismatch"
                        severity error;
                    exit;
                end if;
                v_timeout := v_timeout + 1;
                assert v_timeout < 400
                    report "FAIL: timeout waiting for rising metadata"
                    severity error;
            end loop;
        end procedure;

        procedure wait_fall_empty_meta is
            variable v_timeout : natural := 0;
        begin
            loop
                wait until rising_edge(s_clk);
                if s_cell_fall_tvalid(0) = '1' and s_cell_fall_tlast(0) = '1' then
                    assert s_cell_fall_tdata_0(31 downto 25) = "0000000"
                        report "FAIL: fall metadata kept stale hit_valid after abort"
                        severity error;
                    assert s_cell_fall_tdata_0(6 downto 0) = "0000000"
                        report "FAIL: fall metadata kept stale hit_msb_vec after abort"
                        severity error;
                    assert s_cell_fall_tdata_0(15 downto 12) = x"0"
                        report "FAIL: fall metadata kept stale hit_count after abort"
                        severity error;
                    exit;
                end if;
                v_timeout := v_timeout + 1;
                assert v_timeout < 400
                    report "FAIL: timeout waiting for falling metadata"
                    severity error;
            end loop;
        end procedure;
    begin
        reset_dut;

        -- Case 1: first hit is accepted before shot_start, then released after
        -- the builder becomes active. Hit[16] pattern is 1010101.
        send_evt(x"00010001", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        pulse_shot;
        send_evt(x"00000002", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00010003", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00000004", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00010005", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00000006", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00010007", fn_tuser_data(slope => 1, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt((others => '0'), fn_tuser_drain(ififo_id => 0, faulted => '0'));
        send_evt((others => '0'), fn_tuser_drain(ififo_id => 1, faulted => '0'));
        wait_rise_meta("1010101", 7);

        reset_dut;

        -- Case 2: two falling hits are accepted before shot_start. A fall-only
        -- abort clears the demux holding register and drains the input skid;
        -- the later slice must not report those stale hits.
        send_evt(x"0001AAAA", fn_tuser_data(slope => 0, chip_id => 0, stop_id => 0, ififo_id => 0));
        send_evt(x"00005555", fn_tuser_data(slope => 0, chip_id => 0, stop_id => 0, ififo_id => 0));
        pulse_abort_fall(3);
        pulse_shot;
        send_evt((others => '0'), fn_tuser_drain(ififo_id => 0, faulted => '0'));
        send_evt((others => '0'), fn_tuser_drain(ififo_id => 1, faulted => '0'));
        wait_fall_empty_meta;

        report "PASS: C03 cell_pipe Hit[16], input skid, and per-slope abort regression passed."
            severity note;
        s_done <= true;
        wait;
    end process p_stim;

    p_watchdog : process
    begin
        wait for 20 us;
        assert s_done
            report "FAIL: tb_tdc_gpx_cell_pipe_c03_fix watchdog timeout"
            severity failure;
        wait;
    end process p_watchdog;

end architecture sim;
