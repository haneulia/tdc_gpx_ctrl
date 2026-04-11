-- =============================================================================
-- tb_tdc_gpx_downstream.vhd
-- Testbench for face_assembler + header_inserter pipeline
-- =============================================================================
--
-- Verifies downstream pipeline: synthetic cell data -> face_assembler ->
-- header_inserter -> AXI-Stream frame output.
--
-- Tests:
--   T1: Full frame (4 chips, 8 stops, 2 shots) — header, SOF, beat count, frame_done
--   T2: Partial chip mask (2 chips active) — reduced row, correct frame
--   T3: Back-pressure (tready toggles) — no data loss, no overrun
--   T4: 4-stop mode (stops_per_chip=4) — smaller rows, correct geometry
--
-- Standard: VHDL-2008 (testbench only; DUTs are VHDL-93)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_downstream is
end entity;

architecture sim of tb_tdc_gpx_downstream is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant CLK_PERIOD : time := 5 ns;     -- 200 MHz

    -- =========================================================================
    -- Signals: clock / reset / control
    -- =========================================================================
    signal clk       : std_logic := '0';
    signal rst_n     : std_logic := '0';
    signal done      : boolean   := false;

    -- =========================================================================
    -- Face assembler ports
    -- =========================================================================
    signal cell_tdata      : t_axis_tdata_array := (others => (others => '0'));
    signal cell_tvalid     : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal cell_tlast      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal cell_tready     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    signal shot_start      : std_logic := '0';
    signal active_mask     : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal stops_per_chip  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal max_range_clks  : unsigned(15 downto 0) := to_unsigned(60000, 16);
    signal bus_ticks       : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal bus_clk_div     : unsigned(7 downto 0) := to_unsigned(1, 8);

    -- face_asm -> header
    signal face_tdata      : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    signal face_tvalid     : std_logic;
    signal face_tlast      : std_logic;
    signal face_tready     : std_logic;
    signal row_done        : std_logic;
    signal chip_error      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal shot_overrun    : std_logic;

    -- =========================================================================
    -- Header inserter ports
    -- =========================================================================
    signal face_start      : std_logic := '0';
    signal cfg             : t_tdc_cfg := c_TDC_CFG_INIT;
    signal vdma_frame_id   : unsigned(31 downto 0) := x"AABB0001";
    signal face_id_sig     : unsigned(7 downto 0)  := to_unsigned(3, 8);
    signal shot_seq        : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := to_unsigned(100, c_SHOT_SEQ_WIDTH);
    signal timestamp_ns    : unsigned(63 downto 0) := x"0000000012345678";
    signal chip_err_mask   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal chip_err_cnt    : std_logic_vector(31 downto 0) := (others => '0');
    signal bin_res_ps      : unsigned(15 downto 0) := to_unsigned(81, 16);
    signal k_dist          : unsigned(31 downto 0) := to_unsigned(54321, 32);
    signal rows_per_face   : unsigned(15 downto 0) := to_unsigned(32, 16);

    -- output AXI-Stream
    signal out_tdata       : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    signal out_tvalid      : std_logic;
    signal out_tlast       : std_logic;
    signal out_tuser       : std_logic_vector(0 downto 0);
    signal out_tready      : std_logic := '1';
    signal frame_done      : std_logic;

    -- =========================================================================
    -- Monitor signals (concurrent accumulation)
    -- =========================================================================
    signal mon_beat_in_line    : natural := 0;
    signal mon_line_cnt        : natural := 0;
    signal mon_total_beats     : natural := 0;
    signal mon_sof_cnt         : natural := 0;
    signal mon_frame_done_cnt  : natural := 0;
    signal mon_row_done_cnt    : natural := 0;
    signal mon_overrun_cnt     : natural := 0;
    signal mon_magic_ok_cnt    : natural := 0;
    -- per-line beat count (up to 16 lines)
    type t_nat16 is array(0 to 15) of natural;
    signal mon_line_beats      : t_nat16 := (others => 0);
    -- header beat 1 capture (frame_id, line 0 only)
    signal mon_hdr_frame_id    : std_logic_vector(31 downto 0) := (others => '0');

    -- =========================================================================
    -- Back-pressure control
    -- =========================================================================
    signal bp_enable  : boolean   := false;
    signal bp_phase_r : std_logic := '1';

begin

    -- =========================================================================
    -- Clock
    -- =========================================================================
    clk <= not clk after CLK_PERIOD / 2 when not done else '0';

    -- =========================================================================
    -- DUT [1]: face_assembler
    -- =========================================================================
    u_face_asm : entity work.tdc_gpx_face_assembler
        generic map (g_ALU_PULSE_CLKS => 4)
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_s_axis_tdata     => cell_tdata,
            i_s_axis_tvalid    => cell_tvalid,
            i_s_axis_tlast     => cell_tlast,
            o_s_axis_tready    => cell_tready,
            i_shot_start       => shot_start,
            i_active_chip_mask => active_mask,
            i_stops_per_chip   => stops_per_chip,
            i_max_range_clks   => max_range_clks,
            i_bus_ticks        => bus_ticks,
            i_bus_clk_div      => bus_clk_div,
            o_m_axis_tdata     => face_tdata,
            o_m_axis_tvalid    => face_tvalid,
            o_m_axis_tlast     => face_tlast,
            i_m_axis_tready    => face_tready,
            o_row_done         => row_done,
            o_chip_error_flags => chip_error,
            o_shot_overrun     => shot_overrun
        );

    -- =========================================================================
    -- DUT [2]: header_inserter
    -- =========================================================================
    u_header : entity work.tdc_gpx_header_inserter
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_face_start        => face_start,
            i_cfg               => cfg,
            i_vdma_frame_id     => vdma_frame_id,
            i_face_id           => face_id_sig,
            i_shot_seq_start    => shot_seq,
            i_timestamp_ns      => timestamp_ns,
            i_chip_error_mask   => chip_err_mask,
            i_chip_error_cnt    => chip_err_cnt,
            i_bin_resolution_ps => bin_res_ps,
            i_k_dist_fixed      => k_dist,
            i_rows_per_face     => rows_per_face,
            i_s_axis_tdata      => face_tdata,
            i_s_axis_tvalid     => face_tvalid,
            i_s_axis_tlast      => face_tlast,
            o_s_axis_tready     => face_tready,
            o_m_axis_tdata      => out_tdata,
            o_m_axis_tvalid     => out_tvalid,
            o_m_axis_tlast      => out_tlast,
            o_m_axis_tuser      => out_tuser,
            i_m_axis_tready     => out_tready,
            o_frame_done        => frame_done
        );

    -- =========================================================================
    -- Per-chip cell data generators (4 concurrent processes)
    --
    -- Each process waits for shot_start, then outputs stops_per_chip cells
    -- of c_BEATS_PER_CELL beats each.  Arrival is staggered by chip index
    -- to exercise FCFS scheduling.
    --
    -- Data pattern:
    --   beat 0..META-1, META+1..last: chip[31:28] | stop[27:24] | beat[23:20] | 0[19:0]
    --   beat c_META_BEAT_IDX (meta): hit_valid | slope | hit_count=MAX_HITS |
    --                  dropped=0 | error=0 | chip_id=c | pad=0
    -- =========================================================================
    gen_stim : for c in 0 to c_N_CHIPS - 1 generate
        p_cell_gen : process
            variable v_stops : natural;
        begin
            cell_tvalid(c) <= '0';
            cell_tlast(c)  <= '0';
            cell_tdata(c)  <= (others => '0');

            loop
                -- Wait for next shot_start pulse
                wait until rising_edge(clk) and shot_start = '1';

                if active_mask(c) = '0' then
                    next;   -- skip inactive chip
                end if;

                v_stops := to_integer(stops_per_chip);

                -- Stagger arrival: chip c waits (c*3 + 1) clocks
                for d in 0 to c * 3 loop
                    wait until rising_edge(clk);
                end loop;

                -- Output cells for each stop channel
                for s in 0 to v_stops - 1 loop
                    for b in 0 to c_BEATS_PER_CELL - 1 loop
                        cell_tvalid(c) <= '1';

                        -- tlast only on the very last beat of the last cell
                        if s = v_stops - 1 and b = c_BEATS_PER_CELL - 1 then
                            cell_tlast(c) <= '1';
                        else
                            cell_tlast(c) <= '0';
                        end if;

                        -- Beat data
                        if b = c_META_BEAT_IDX then
                            -- Metadata beat (auto-layout from c_MAX_HITS_PER_STOP)
                            cell_tdata(c) <= (others => '0');
                            cell_tdata(c)(31 downto 32 - c_MAX_HITS_PER_STOP)
                                <= (others => '1');                              -- hit_valid
                            cell_tdata(c)(31 - c_MAX_HITS_PER_STOP downto 32 - 2*c_MAX_HITS_PER_STOP)
                                <= "1010101";                                    -- slope_vec (alternating)
                            cell_tdata(c)(15 downto 12)
                                <= std_logic_vector(to_unsigned(c_MAX_HITS_PER_STOP, 4)); -- hit_count
                            cell_tdata(c)(9 downto 8)
                                <= std_logic_vector(to_unsigned(c, 2));          -- chip_id
                        else
                            -- Identifiable pattern
                            cell_tdata(c) <=
                                std_logic_vector(to_unsigned(c, 4))             -- chip  [31:28]
                              & std_logic_vector(to_unsigned(s, 4))             -- stop  [27:24]
                              & std_logic_vector(to_unsigned(b, 4))             -- beat  [23:20]
                              & x"00000";                                       -- pad   [19:0]
                        end if;

                        -- AXI-Stream handshake: wait for tready
                        loop
                            wait until rising_edge(clk);
                            exit when cell_tready(c) = '1';
                        end loop;
                    end loop;
                end loop;

                cell_tvalid(c) <= '0';
                cell_tlast(c)  <= '0';
            end loop;
        end process;
    end generate;

    -- =========================================================================
    -- Back-pressure generator
    -- =========================================================================
    p_bp : process(clk)
    begin
        if rising_edge(clk) then
            if bp_enable then
                bp_phase_r <= not bp_phase_r;
                out_tready <= bp_phase_r;       -- toggle 1/0/1/0 (~50% throughput)
            else
                out_tready <= '1';
                bp_phase_r <= '1';
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Concurrent output monitor
    --
    -- Counts beats, lines, SOF, frame_done, row_done, shot_overrun.
    -- Records per-line beat count and checks header magic word.
    -- Reset by rst_n = '0'.
    -- =========================================================================
    p_monitor : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mon_beat_in_line   <= 0;
                mon_line_cnt       <= 0;
                mon_total_beats    <= 0;
                mon_sof_cnt        <= 0;
                mon_frame_done_cnt <= 0;
                mon_row_done_cnt   <= 0;
                mon_overrun_cnt    <= 0;
                mon_magic_ok_cnt   <= 0;
                mon_line_beats     <= (others => 0);
                mon_hdr_frame_id   <= (others => '0');
            else
                -- Output AXI-Stream beat
                if out_tvalid = '1' and out_tready = '1' then
                    mon_total_beats <= mon_total_beats + 1;

                    -- SOF check
                    if out_tuser(0) = '1' then
                        mon_sof_cnt <= mon_sof_cnt + 1;
                    end if;

                    -- Header magic on first beat of each line
                    if mon_beat_in_line = 0 and out_tdata = c_HEADER_MAGIC then
                        mon_magic_ok_cnt <= mon_magic_ok_cnt + 1;
                    end if;

                    -- Capture frame_id from header beat 1 of line 0
                    if mon_beat_in_line = 1 and mon_line_cnt = 0 then
                        mon_hdr_frame_id <= out_tdata;
                    end if;

                    -- Line boundary
                    if out_tlast = '1' then
                        if mon_line_cnt < 16 then
                            mon_line_beats(mon_line_cnt) <= mon_beat_in_line + 1;
                        end if;
                        mon_line_cnt     <= mon_line_cnt + 1;
                        mon_beat_in_line <= 0;
                    else
                        mon_beat_in_line <= mon_beat_in_line + 1;
                    end if;
                end if;

                -- Event counters
                if frame_done = '1' then
                    mon_frame_done_cnt <= mon_frame_done_cnt + 1;
                end if;
                if row_done = '1' then
                    mon_row_done_cnt <= mon_row_done_cnt + 1;
                end if;
                if shot_overrun = '1' then
                    mon_overrun_cnt <= mon_overrun_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Main test sequencer
    -- =========================================================================
    p_main : process

        -- ----- helpers -------------------------------------------------------

        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        procedure reset_dut is
        begin
            rst_n <= '0';
            wait_clk(5);
            rst_n <= '1';
            wait_clk(3);
        end procedure;

        procedure pulse_face_and_shot is
        begin
            face_start <= '1';  shot_start <= '1';
            wait until rising_edge(clk);
            face_start <= '0';  shot_start <= '0';
        end procedure;

        procedure pulse_shot is
        begin
            shot_start <= '1';
            wait until rising_edge(clk);
            shot_start <= '0';
        end procedure;

        procedure wait_row_done_or_fail(timeout : natural) is
        begin
            for i in 1 to timeout loop
                wait until rising_edge(clk);
                if row_done = '1' then return; end if;
            end loop;
            assert false report "TIMEOUT waiting for row_done" severity failure;
        end procedure;

        procedure wait_frame_done_or_fail(timeout : natural) is
        begin
            for i in 1 to timeout loop
                wait until rising_edge(clk);
                if frame_done = '1' then return; end if;
            end loop;
            assert false report "TIMEOUT waiting for frame_done" severity failure;
        end procedure;

        -- ----- variables -----------------------------------------------------
        variable v_exp_data_beats : natural;
        variable v_exp_line_beats : natural;
        variable v_n_active       : natural;

    begin

        -- =============================================================
        -- T1: Full frame, 4 chips × 8 stops, cols_per_face = 2
        --   data_beats  = 4 × 8 × 8 = 256
        --   line_beats  = 12 + 256   = 268
        --   total_beats = 2 × 268    = 536
        -- =============================================================
        report "===== T1: Full frame, 4 chips, 8 stops, 2 shots =====" severity note;
        reset_dut;

        active_mask                <= (others => '1');
        stops_per_chip             <= to_unsigned(8, 4);
        cfg.cols_per_face          <= to_unsigned(2, 16);
        cfg.stops_per_chip         <= to_unsigned(8, 4);
        cfg.active_chip_mask       <= (others => '1');
        rows_per_face              <= to_unsigned(32, 16);
        bp_enable                  <= false;
        wait_clk(2);

        v_n_active       := c_N_CHIPS;
        v_exp_data_beats := v_n_active * 8 * c_BEATS_PER_CELL;       -- 256
        v_exp_line_beats := c_HDR_PREFIX_BEATS + v_exp_data_beats;    -- 268

        -- Shot 0 (first of face)
        pulse_face_and_shot;
        wait_row_done_or_fail(5000);
        wait_clk(3);

        -- Shot 1
        pulse_shot;
        wait_frame_done_or_fail(5000);
        wait_clk(15);

        -- --- checks ---
        assert mon_line_cnt = 2
            report "T1 FAIL: line_cnt=" & integer'image(mon_line_cnt) & " exp=2"
            severity error;
        assert mon_frame_done_cnt = 1
            report "T1 FAIL: frame_done_cnt=" & integer'image(mon_frame_done_cnt) & " exp=1"
            severity error;
        assert mon_row_done_cnt = 2
            report "T1 FAIL: row_done_cnt=" & integer'image(mon_row_done_cnt) & " exp=2"
            severity error;
        assert mon_sof_cnt = 1
            report "T1 FAIL: sof_cnt=" & integer'image(mon_sof_cnt) & " exp=1"
            severity error;
        assert mon_line_beats(0) = v_exp_line_beats
            report "T1 FAIL: line0 beats=" & integer'image(mon_line_beats(0))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;
        assert mon_line_beats(1) = v_exp_line_beats
            report "T1 FAIL: line1 beats=" & integer'image(mon_line_beats(1))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;
        assert mon_total_beats = 2 * v_exp_line_beats
            report "T1 FAIL: total_beats=" & integer'image(mon_total_beats)
                   & " exp=" & integer'image(2 * v_exp_line_beats)
            severity error;
        -- Only line 0 carries the real header magic; line 1 is zero-filled
        assert mon_magic_ok_cnt >= 1
            report "T1 FAIL: magic word not seen on line 0"
            severity error;
        assert mon_hdr_frame_id = std_logic_vector(vdma_frame_id)
            report "T1 FAIL: header frame_id mismatch"
            severity error;
        assert mon_overrun_cnt = 0
            report "T1 FAIL: unexpected shot overrun"
            severity error;

        report "T1 PASS" severity note;

        -- =============================================================
        -- T2: Partial mask (chips 0,1 active), cols_per_face = 2
        --   data_beats  = 2 × 8 × 8 = 128
        --   line_beats  = 12 + 128   = 140
        -- =============================================================
        report "===== T2: Partial mask (2 chips), 2 shots =====" severity note;
        reset_dut;

        active_mask                <= "0011";
        stops_per_chip             <= to_unsigned(8, 4);
        cfg.cols_per_face          <= to_unsigned(2, 16);
        cfg.stops_per_chip         <= to_unsigned(8, 4);
        cfg.active_chip_mask       <= "0011";
        rows_per_face              <= to_unsigned(16, 16);
        bp_enable                  <= false;
        wait_clk(2);

        v_n_active       := 2;
        v_exp_data_beats := v_n_active * 8 * c_BEATS_PER_CELL;       -- 128
        v_exp_line_beats := c_HDR_PREFIX_BEATS + v_exp_data_beats;    -- 140

        pulse_face_and_shot;
        wait_row_done_or_fail(5000);
        wait_clk(3);

        pulse_shot;
        wait_frame_done_or_fail(5000);
        wait_clk(15);

        assert mon_line_cnt = 2
            report "T2 FAIL: line_cnt=" & integer'image(mon_line_cnt) & " exp=2"
            severity error;
        assert mon_frame_done_cnt = 1
            report "T2 FAIL: frame_done_cnt=" & integer'image(mon_frame_done_cnt) & " exp=1"
            severity error;
        assert mon_line_beats(0) = v_exp_line_beats
            report "T2 FAIL: line0 beats=" & integer'image(mon_line_beats(0))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;
        assert mon_line_beats(1) = v_exp_line_beats
            report "T2 FAIL: line1 beats=" & integer'image(mon_line_beats(1))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;
        assert mon_total_beats = 2 * v_exp_line_beats
            report "T2 FAIL: total_beats=" & integer'image(mon_total_beats)
                   & " exp=" & integer'image(2 * v_exp_line_beats)
            severity error;

        report "T2 PASS" severity note;

        -- =============================================================
        -- T3: Back-pressure (4 chips, 8 stops, 1 shot)
        --   tready toggles every clock (~50% throughput).
        --   Verify same beat count as T1-single-line, no overrun.
        -- =============================================================
        report "===== T3: Back-pressure, 4 chips, 1 shot =====" severity note;
        reset_dut;

        active_mask                <= (others => '1');
        stops_per_chip             <= to_unsigned(8, 4);
        cfg.cols_per_face          <= to_unsigned(1, 16);
        cfg.stops_per_chip         <= to_unsigned(8, 4);
        cfg.active_chip_mask       <= (others => '1');
        rows_per_face              <= to_unsigned(32, 16);
        bp_enable                  <= true;
        wait_clk(2);

        v_exp_data_beats := c_N_CHIPS * 8 * c_BEATS_PER_CELL;        -- 256
        v_exp_line_beats := c_HDR_PREFIX_BEATS + v_exp_data_beats;    -- 268

        pulse_face_and_shot;
        wait_frame_done_or_fail(20000);   -- longer timeout for back-pressure
        wait_clk(15);
        bp_enable <= false;
        wait_clk(5);

        assert mon_line_cnt = 1
            report "T3 FAIL: line_cnt=" & integer'image(mon_line_cnt) & " exp=1"
            severity error;
        assert mon_frame_done_cnt = 1
            report "T3 FAIL: frame_done_cnt=" & integer'image(mon_frame_done_cnt) & " exp=1"
            severity error;
        assert mon_line_beats(0) = v_exp_line_beats
            report "T3 FAIL: line0 beats=" & integer'image(mon_line_beats(0))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;
        assert mon_overrun_cnt = 0
            report "T3 FAIL: shot overrun during back-pressure"
            severity error;

        report "T3 PASS" severity note;

        -- =============================================================
        -- T4: 4-stop mode (stops_per_chip=4, 4 chips, 1 shot)
        --   data_beats  = 4 × 4 × 8 = 128
        --   line_beats  = 12 + 128   = 140
        -- =============================================================
        report "===== T4: 4-stop mode, 4 chips, 1 shot =====" severity note;
        reset_dut;

        active_mask                <= (others => '1');
        stops_per_chip             <= to_unsigned(4, 4);
        cfg.cols_per_face          <= to_unsigned(1, 16);
        cfg.stops_per_chip         <= to_unsigned(4, 4);
        cfg.active_chip_mask       <= (others => '1');
        rows_per_face              <= to_unsigned(16, 16);
        bp_enable                  <= false;
        wait_clk(2);

        v_exp_data_beats := c_N_CHIPS * 4 * c_BEATS_PER_CELL;        -- 128
        v_exp_line_beats := c_HDR_PREFIX_BEATS + v_exp_data_beats;    -- 140

        pulse_face_and_shot;
        wait_frame_done_or_fail(5000);
        wait_clk(15);

        assert mon_line_cnt = 1
            report "T4 FAIL: line_cnt=" & integer'image(mon_line_cnt) & " exp=1"
            severity error;
        assert mon_frame_done_cnt = 1
            report "T4 FAIL: frame_done_cnt=" & integer'image(mon_frame_done_cnt) & " exp=1"
            severity error;
        assert mon_line_beats(0) = v_exp_line_beats
            report "T4 FAIL: line0 beats=" & integer'image(mon_line_beats(0))
                   & " exp=" & integer'image(v_exp_line_beats)
            severity error;

        report "T4 PASS" severity note;

        -- =============================================================
        -- Done
        -- =============================================================
        report "===== ALL TESTS PASSED =====" severity note;
        wait_clk(10);
        done <= true;
        wait;
    end process;

end architecture;
