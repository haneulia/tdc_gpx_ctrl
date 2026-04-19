-- =============================================================================
-- tb_tdc_gpx_face_seq.vhd
-- Focused testbench for tdc_gpx_face_seq Round 11 item 1 pulse-width guarantee.
--
-- Verifies:
--   A. 1-cycle raw pulse  -> o_packet_start / o_face_start / o_shot_start_gated
--                            each exactly 1 cycle wide
--   B. Multi-cycle raw level (3 cycles) -> internal pulses still 1 cycle each
--   C. Back-to-back raw edges (2 shots) -> 2 distinct 1-cycle pulses
--   D. Raw during WAIT_SHOT + hdr busy  -> deferred latch, single 1-cycle pulse
--                                           after hdr becomes idle
--
-- Pass criterion: for every o_cmd_start_accepted, each of the three start
-- signals asserts high for exactly 1 clk cycle per shot, and global_shot_seq
-- counter advances by exactly 1 per raw pulse event.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_face_seq is
end entity tb_tdc_gpx_face_seq;

architecture sim of tb_tdc_gpx_face_seq is

    constant C_CLK_PER : time := 5 ns;

    signal clk                    : std_logic := '0';
    signal rst_n                  : std_logic := '0';

    -- commands
    signal cmd_start              : std_logic := '0';
    signal cmd_stop               : std_logic := '0';
    signal cmd_soft_reset         : std_logic := '0';

    -- pipeline idle indicators (all idle by default so cmd_start accepts)
    signal chip_busy              : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal reg_outstanding        : std_logic := '0';
    signal face_asm_idle          : std_logic := '1';
    signal face_asm_fall_idle     : std_logic := '1';
    signal hdr_idle               : std_logic := '1';
    signal hdr_fall_idle          : std_logic := '1';
    signal face_tvalid            : std_logic := '0';
    signal face_fall_tvalid       : std_logic := '0';
    signal face_buf_tvalid        : std_logic := '0';
    signal face_fall_buf_tvalid   : std_logic := '0';
    signal m_axis_tvalid          : std_logic := '0';
    signal m_axis_fall_tvalid     : std_logic := '0';

    -- events
    signal shot_start_raw         : std_logic := '0';
    signal frame_done             : std_logic := '0';
    signal frame_fall_done        : std_logic := '0';
    signal face_abort             : std_logic := '0';
    signal face_fall_abort        : std_logic := '0';
    signal shot_overrun           : std_logic := '0';
    signal shot_fall_overrun      : std_logic := '0';
    signal hdr_draining           : std_logic := '0';
    signal hdr_fall_draining      : std_logic := '0';

    -- config: cols_per_face = 4 so a single face accepts up to 4 shots
    signal cfg                    : t_tdc_cfg := (
        active_chip_mask    => "0001",
        packet_scope        => '0',
        hit_store_mode      => "00",
        dist_scale          => "000",
        drain_mode          => '0',
        pipeline_en         => '0',
        n_faces             => to_unsigned(1, 3),
        stops_per_chip      => to_unsigned(2, 4),
        n_drain_cap         => (others => '0'),
        stopdis_override    => (others => '0'),
        bus_clk_div         => to_unsigned(2, 6),
        bus_ticks           => to_unsigned(5, 3),
        max_range_clks      => to_unsigned(100, 16),
        cols_per_face       => to_unsigned(4, 16),
        start_off1          => (others => '0'),
        cfg_reg7            => (others => '0'),
        max_scan_clks       => to_unsigned(0, 16),
        max_hits_cfg        => to_unsigned(1, 3)
    );

    -- outputs
    signal cmd_start_accepted     : std_logic;
    signal face_state_idle        : std_logic;
    signal packet_start           : std_logic;
    signal face_start             : std_logic;
    signal face_start_gated       : std_logic;
    signal shot_start_gated_s     : std_logic;
    signal face_closing           : std_logic;
    signal pipeline_abort         : std_logic;
    signal pipeline_abort_rise    : std_logic;
    signal pipeline_abort_fall    : std_logic;
    signal shot_drop_cnt          : unsigned(15 downto 0);
    signal cfg_rejected           : std_logic;
    signal shot_start_per_chip    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal face_id                : unsigned(7 downto 0);
    signal frame_id               : unsigned(31 downto 0);
    signal global_shot_seq        : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal frame_abort_cnt        : unsigned(15 downto 0);
    signal frame_done_both        : std_logic;
    signal face_active_mask       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal face_stops_per_chip    : unsigned(3 downto 0);
    signal face_cols_per_face     : unsigned(15 downto 0);
    signal face_n_faces           : unsigned(3 downto 0);
    signal rows_per_face          : unsigned(15 downto 0);
    signal hsize_bytes            : unsigned(15 downto 0);
    signal cfg_face_out           : t_tdc_cfg;

    -- monitoring
    signal packet_start_high_cnt  : natural := 0;
    signal face_start_high_cnt    : natural := 0;
    signal shot_gated_high_cnt    : natural := 0;
    signal test_done              : boolean := false;

begin

    -- Clock
    clk <= not clk after C_CLK_PER / 2 when not test_done else '0';

    -- DUT
    dut : entity work.tdc_gpx_face_seq
        generic map (g_OUTPUT_WIDTH => 32)
        port map (
            i_clk                   => clk,
            i_rst_n                 => rst_n,
            i_cmd_start             => cmd_start,
            i_cmd_stop              => cmd_stop,
            i_cmd_soft_reset        => cmd_soft_reset,
            i_chip_busy             => chip_busy,
            i_reg_outstanding       => reg_outstanding,
            i_face_asm_idle         => face_asm_idle,
            i_face_asm_fall_idle    => face_asm_fall_idle,
            i_hdr_idle              => hdr_idle,
            i_hdr_fall_idle         => hdr_fall_idle,
            i_face_tvalid           => face_tvalid,
            i_face_fall_tvalid      => face_fall_tvalid,
            i_face_buf_tvalid       => face_buf_tvalid,
            i_face_fall_buf_tvalid  => face_fall_buf_tvalid,
            i_m_axis_tvalid         => m_axis_tvalid,
            i_m_axis_fall_tvalid    => m_axis_fall_tvalid,
            i_shot_start_raw        => shot_start_raw,
            i_frame_done            => frame_done,
            i_frame_fall_done       => frame_fall_done,
            i_face_abort            => face_abort,
            i_face_fall_abort       => face_fall_abort,
            i_shot_overrun          => shot_overrun,
            i_shot_fall_overrun     => shot_fall_overrun,
            i_hdr_draining          => hdr_draining,
            i_hdr_fall_draining     => hdr_fall_draining,
            i_cfg                   => cfg,
            o_cmd_start_accepted    => cmd_start_accepted,
            o_face_state_idle       => face_state_idle,
            o_packet_start          => packet_start,
            o_face_start            => face_start,
            o_face_start_gated      => face_start_gated,
            o_shot_start_gated      => shot_start_gated_s,
            o_face_closing          => face_closing,
            o_pipeline_abort        => pipeline_abort,
            o_pipeline_abort_rise   => pipeline_abort_rise,
            o_pipeline_abort_fall   => pipeline_abort_fall,
            o_shot_drop_cnt         => shot_drop_cnt,
            o_cfg_rejected          => cfg_rejected,
            o_shot_start_per_chip   => shot_start_per_chip,
            o_face_id               => face_id,
            o_frame_id              => frame_id,
            o_global_shot_seq       => global_shot_seq,
            o_frame_abort_cnt       => frame_abort_cnt,
            o_frame_done_both       => frame_done_both,
            o_face_active_mask      => face_active_mask,
            o_face_stops_per_chip   => face_stops_per_chip,
            o_face_cols_per_face    => face_cols_per_face,
            o_face_n_faces          => face_n_faces,
            o_rows_per_face         => rows_per_face,
            o_hsize_bytes           => hsize_bytes,
            o_cfg_face              => cfg_face_out
        );

    -- Continuous assertions: no output pulse should exceed 1 cycle.
    -- Strategy: count consecutive-high cycles; fail if >1.
    p_monitor : process(clk)
    begin
        if rising_edge(clk) then
            if packet_start = '1' then
                packet_start_high_cnt <= packet_start_high_cnt + 1;
            else
                packet_start_high_cnt <= 0;
            end if;
            if face_start = '1' then
                face_start_high_cnt <= face_start_high_cnt + 1;
            else
                face_start_high_cnt <= 0;
            end if;
            if shot_start_gated_s = '1' then
                shot_gated_high_cnt <= shot_gated_high_cnt + 1;
            else
                shot_gated_high_cnt <= 0;
            end if;

            assert packet_start_high_cnt <= 1
                report "FAIL: packet_start held high >1 cycle"
                severity failure;
            assert face_start_high_cnt <= 1
                report "FAIL: face_start held high >1 cycle"
                severity failure;
            assert shot_gated_high_cnt <= 1
                report "FAIL: shot_start_gated held high >1 cycle"
                severity failure;
        end if;
    end process;

    -- Stimulus
    p_stim : process
        variable v_seq_before : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);

        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        procedure start_run is
            variable v_guard : natural := 0;
        begin
            wait until rising_edge(clk);
            cmd_start <= '1';
            -- Hold high until accepted or guard trips.
            loop
                wait until rising_edge(clk);
                exit when cmd_start_accepted = '1';
                v_guard := v_guard + 1;
                assert v_guard < 100
                    report "start_run: cmd_start never accepted after 100 cycles"
                    severity failure;
            end loop;
            cmd_start <= '0';
            wait_clk(2);
        end procedure;

        procedure end_run is
        begin
            cmd_stop <= '1';
            wait_clk(2);
            cmd_stop <= '0';
            wait_clk(5);
        end procedure;
    begin
        -- Reset
        rst_n <= '0';
        wait_clk(10);
        rst_n <= '1';
        wait_clk(5);

        -------------------------------------------------------------------------
        -- Scenario A: 1-cycle raw pulse, gating met
        -------------------------------------------------------------------------
        report "=== Scenario A: 1-cycle raw ===" severity note;
        start_run;
        v_seq_before := global_shot_seq;

        shot_start_raw <= '1';
        wait_clk(1);
        shot_start_raw <= '0';

        -- Let pulses propagate
        wait_clk(8);

        assert global_shot_seq = v_seq_before + 1
            report "FAIL A: shot counter expected +1, got " & integer'image(to_integer(global_shot_seq))
            severity failure;
        report "PASS A: counter incremented by 1" severity note;

        end_run;

        -------------------------------------------------------------------------
        -- Scenario B: 3-cycle raw level, gating met
        -------------------------------------------------------------------------
        report "=== Scenario B: 3-cycle raw level ===" severity note;
        start_run;
        v_seq_before := global_shot_seq;

        shot_start_raw <= '1';
        wait_clk(3);
        shot_start_raw <= '0';

        wait_clk(8);

        assert global_shot_seq = v_seq_before + 1
            report "FAIL B: shot counter expected +1, got " & integer'image(to_integer(global_shot_seq - v_seq_before))
            severity failure;
        report "PASS B: multi-cycle raw still counts as 1 shot" severity note;

        end_run;

        -------------------------------------------------------------------------
        -- Scenario C: Back-to-back raw edges (2 shots, spaced)
        -------------------------------------------------------------------------
        report "=== Scenario C: 2 back-to-back shots ===" severity note;
        start_run;
        v_seq_before := global_shot_seq;

        shot_start_raw <= '1';
        wait_clk(1);
        shot_start_raw <= '0';
        wait_clk(6);  -- wait for first shot to complete propagation

        shot_start_raw <= '1';
        wait_clk(1);
        shot_start_raw <= '0';
        wait_clk(8);

        assert global_shot_seq = v_seq_before + 2
            report "FAIL C: shot counter expected +2, got " & integer'image(to_integer(global_shot_seq - v_seq_before))
            severity failure;
        report "PASS C: 2 edges -> 2 counter increments" severity note;

        end_run;

        -------------------------------------------------------------------------
        -- Scenario D: Raw during WAIT_SHOT while hdr busy, then hdr idles
        -- Accept the run first (hdr idle), then drop hdr during WAIT_SHOT so
        -- the packet_start gating fails and the raw pulse has to park in
        -- s_shot_deferred_r until hdr comes back idle.
        -------------------------------------------------------------------------
        report "=== Scenario D: deferred path (hdr busy) ===" severity note;
        start_run;
        v_seq_before := global_shot_seq;

        -- Now in WAIT_SHOT. Drop hdr_idle so packet_start gating fails.
        hdr_idle <= '0';
        hdr_fall_idle <= '0';
        wait_clk(2);

        shot_start_raw <= '1';
        wait_clk(1);
        shot_start_raw <= '0';
        wait_clk(5);  -- shot should be deferred, not yet accepted

        assert global_shot_seq = v_seq_before
            report "FAIL D-pre: counter changed while hdr busy"
            severity failure;

        -- Release hdr: deferred shot should now fire exactly once
        hdr_idle <= '1';
        hdr_fall_idle <= '1';
        wait_clk(8);

        assert global_shot_seq = v_seq_before + 1
            report "FAIL D: deferred shot expected +1, got " & integer'image(to_integer(global_shot_seq - v_seq_before))
            severity failure;
        report "PASS D: deferred -> single 1-cycle pulse" severity note;

        end_run;

        -------------------------------------------------------------------------
        -- Scenario E: stops_per_chip > c_MAX_STOPS_PER_CHIP (=8) must reject
        -------------------------------------------------------------------------
        report "=== Scenario E: stops_per_chip > 8 reject ===" severity note;
        cfg.stops_per_chip <= to_unsigned(9, 4);
        wait_clk(2);
        wait until rising_edge(clk);
        cmd_start <= '1';
        wait until rising_edge(clk);
        cmd_start <= '0';
        -- Expect cfg_rejected pulse within a few cycles; cmd_start_accepted
        -- must NEVER fire.
        for i in 1 to 10 loop
            wait until rising_edge(clk);
            assert cmd_start_accepted = '0'
                report "FAIL E: cmd_start accepted with stops_per_chip=9"
                severity failure;
            exit when cfg_rejected = '1';
        end loop;
        assert cfg_rejected = '1'
            report "FAIL E: cfg_rejected did not pulse within 10 cycles"
            severity failure;
        report "PASS E: stops_per_chip=9 rejected via cfg_rejected" severity note;

        -- Restore valid config
        cfg.stops_per_chip <= to_unsigned(2, 4);
        wait_clk(3);

        -------------------------------------------------------------------------
        report "====================================" severity note;
        report "ALL SCENARIOS PASSED (A,B,C,D,E)"    severity note;
        report "====================================" severity note;
        test_done <= true;
        wait;
    end process;

end architecture sim;
