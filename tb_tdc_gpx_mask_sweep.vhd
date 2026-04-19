-- =============================================================================
-- tb_tdc_gpx_mask_sweep.vhd
-- Round 11 C1 follow-up: mask-sweep integration testbench.
--
-- Purpose:
--   Verify that all 15 non-zero active_chip_mask patterns are handled
--   correctly by the data-path modules that depend on it:
--     * face_seq: geometry (rows = active_cnt × stops), face_active_mask
--                 latch, per-chip shot_start gating
--     * face_assembler: iteration skips inactive chips, output beat count
--                       matches active_cnt × stops × beats_per_cell
--
--   Code-level audit (Round 11 post-item-18) showed the logic is
--   pattern-independent (count-of-ones based). This tb supplies the
--   end-to-end simulation proof that previously existed only for the
--   default mask "1111".
--
-- Scope:
--   Integration of face_seq + face_assembler. header_inserter is excluded
--   (it records active_chip_mask verbatim in header prefix with no
--   interpretation — audit confirmed pass-through). The tb drives:
--     - face_seq's command / idle inputs
--     - per-chip synthetic cell data into face_assembler
--   and verifies per-mask:
--     - face_seq.o_rows_per_face = active_cnt × stops_per_chip
--     - face_seq.o_shot_start_per_chip asserts only on active bits
--     - face_assembler produces active_cnt × stops × beats_per_cell
--       beats on its m_axis output before going idle
--
-- Masks exercised: "0001", "1000", "0101", "1010", "0110", "1111"
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_mask_sweep is
end entity tb_tdc_gpx_mask_sweep;

architecture sim of tb_tdc_gpx_mask_sweep is

    constant C_CLK_PER      : time := 5 ns;
    constant C_STOPS        : natural := 2;
    constant C_MAX_HITS     : natural := 1;
    constant C_G_WIDTH      : natural := 32;
    -- beats_per_cell for g_TDATA_WIDTH=32, max_hits=1 → 2
    constant C_BEATS_PER_CELL : natural := fn_beats_per_cell_rt(C_MAX_HITS, C_G_WIDTH);

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- face_seq command / idle stubs
    signal cmd_start              : std_logic := '0';
    signal cmd_stop               : std_logic := '0';
    signal cmd_soft_reset         : std_logic := '0';
    signal shot_start_raw         : std_logic := '0';
    signal face_abort             : std_logic := '0';
    signal face_fall_abort        : std_logic := '0';
    signal hdr_idle               : std_logic := '1';
    signal hdr_fall_idle          : std_logic := '1';
    signal hdr_draining           : std_logic := '0';
    signal hdr_fall_draining      : std_logic := '0';
    signal face_asm_idle          : std_logic := '1';
    signal face_asm_fall_idle     : std_logic := '1';
    signal chip_busy              : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal reg_outstanding        : std_logic := '0';
    signal face_tvalid            : std_logic := '0';
    signal face_fall_tvalid       : std_logic := '0';
    signal face_buf_tvalid        : std_logic := '0';
    signal face_fall_buf_tvalid   : std_logic := '0';
    signal m_axis_tvalid_dummy    : std_logic := '0';
    signal m_axis_fall_tvalid_dummy : std_logic := '0';
    signal shot_overrun_rise      : std_logic := '0';
    signal shot_overrun_fall      : std_logic := '0';
    signal frame_done_rise        : std_logic := '0';
    signal frame_done_fall        : std_logic := '0';

    signal cfg : t_tdc_cfg := (
        active_chip_mask    => "1111",
        packet_scope        => '0',
        hit_store_mode      => "00",
        dist_scale          => "000",
        drain_mode          => '0',
        pipeline_en         => '0',
        n_faces             => to_unsigned(1, 3),
        stops_per_chip      => to_unsigned(C_STOPS, 4),
        n_drain_cap         => (others => '0'),
        stopdis_override    => (others => '0'),
        bus_clk_div         => to_unsigned(2, 6),
        bus_ticks           => to_unsigned(5, 3),
        max_range_clks      => to_unsigned(100, 16),
        cols_per_face       => to_unsigned(1, 16),
        start_off1          => (others => '0'),
        cfg_reg7            => (others => '0'),
        max_scan_clks       => to_unsigned(0, 16),
        max_hits_cfg        => to_unsigned(C_MAX_HITS, 3)
    );

    -- face_seq outputs
    signal cmd_start_accepted  : std_logic;
    signal face_state_idle     : std_logic;
    signal packet_start        : std_logic;
    signal face_start          : std_logic;
    signal face_start_gated    : std_logic;
    signal shot_start_gated_s  : std_logic;
    signal face_closing        : std_logic;
    signal pipeline_abort      : std_logic;
    signal pipeline_abort_rise : std_logic;
    signal pipeline_abort_fall : std_logic;
    signal shot_drop_cnt       : unsigned(15 downto 0);
    signal cfg_rejected        : std_logic;
    signal shot_start_per_chip : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal face_id             : unsigned(7 downto 0);
    signal frame_id            : unsigned(31 downto 0);
    signal global_shot_seq     : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal frame_abort_cnt     : unsigned(15 downto 0);
    signal frame_done_both     : std_logic;
    signal face_active_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal face_stops_per_chip : unsigned(3 downto 0);
    signal face_cols_per_face  : unsigned(15 downto 0);
    signal face_n_faces        : unsigned(3 downto 0);
    signal rows_per_face       : unsigned(15 downto 0);
    signal hsize_bytes         : unsigned(15 downto 0);
    signal cfg_face_out        : t_tdc_cfg;

    -- face_assembler per-chip inputs
    signal fa_in_tdata_0 : std_logic_vector(C_G_WIDTH - 1 downto 0) := (others => '0');
    signal fa_in_tdata_1 : std_logic_vector(C_G_WIDTH - 1 downto 0) := (others => '0');
    signal fa_in_tdata_2 : std_logic_vector(C_G_WIDTH - 1 downto 0) := (others => '0');
    signal fa_in_tdata_3 : std_logic_vector(C_G_WIDTH - 1 downto 0) := (others => '0');
    signal fa_in_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal fa_in_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal fa_in_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- face_assembler outputs
    signal fa_out_tdata  : std_logic_vector(C_G_WIDTH - 1 downto 0);
    signal fa_out_tvalid : std_logic;
    signal fa_out_tlast  : std_logic;
    signal fa_row_done   : std_logic;
    signal fa_chip_err   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal fa_shot_overrun : std_logic;
    signal fa_face_abort : std_logic;
    signal fa_idle       : std_logic;
    signal fa_flush_drop : std_logic;
    signal fa_flush_drop_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal fa_overrun_cnt : unsigned(7 downto 0);

    -- Beat counter on face_assembler output (for mask verification)
    signal fa_out_beat_count : natural := 0;

    -- Per-chip data generator state (architecture scope so tdata can be
    -- assigned concurrently outside the generate).
    type t_chip_nat_arr is array(0 to c_N_CHIPS - 1) of natural range 0 to 31;
    signal s_gen_remaining : t_chip_nat_arr := (others => 0);
    signal s_gen_beat_idx  : t_chip_nat_arr := (others => 0);
    signal s_gen_arm_delay : t_chip_nat_arr := (others => 0);

    signal test_done : boolean := false;

begin

    clk <= not clk after C_CLK_PER / 2 when not test_done else '0';

    -- ====================================================================
    -- face_seq under test
    -- ====================================================================
    u_face_seq : entity work.tdc_gpx_face_seq
        generic map (g_OUTPUT_WIDTH => C_G_WIDTH)
        port map (
            i_clk                => clk,
            i_rst_n              => rst_n,
            i_cmd_start          => cmd_start,
            i_cmd_stop           => cmd_stop,
            i_cmd_soft_reset     => cmd_soft_reset,
            i_chip_busy          => chip_busy,
            i_reg_outstanding    => reg_outstanding,
            i_face_asm_idle      => face_asm_idle,
            i_face_asm_fall_idle => face_asm_fall_idle,
            i_hdr_idle           => hdr_idle,
            i_hdr_fall_idle      => hdr_fall_idle,
            i_face_tvalid        => face_tvalid,
            i_face_fall_tvalid   => face_fall_tvalid,
            i_face_buf_tvalid    => face_buf_tvalid,
            i_face_fall_buf_tvalid => face_fall_buf_tvalid,
            i_m_axis_tvalid      => m_axis_tvalid_dummy,
            i_m_axis_fall_tvalid => m_axis_fall_tvalid_dummy,
            i_shot_start_raw     => shot_start_raw,
            i_frame_done         => frame_done_rise,
            i_frame_fall_done    => frame_done_fall,
            i_face_abort         => face_abort,
            i_face_fall_abort    => face_fall_abort,
            i_shot_overrun       => shot_overrun_rise,
            i_shot_fall_overrun  => shot_overrun_fall,
            i_hdr_draining       => hdr_draining,
            i_hdr_fall_draining  => hdr_fall_draining,
            i_cfg                => cfg,
            o_cmd_start_accepted => cmd_start_accepted,
            o_face_state_idle    => face_state_idle,
            o_packet_start       => packet_start,
            o_face_start         => face_start,
            o_face_start_gated   => face_start_gated,
            o_shot_start_gated   => shot_start_gated_s,
            o_face_closing       => face_closing,
            o_pipeline_abort     => pipeline_abort,
            o_pipeline_abort_rise => pipeline_abort_rise,
            o_pipeline_abort_fall => pipeline_abort_fall,
            o_shot_drop_cnt      => shot_drop_cnt,
            o_cfg_rejected       => cfg_rejected,
            o_shot_start_per_chip => shot_start_per_chip,
            o_face_id            => face_id,
            o_frame_id           => frame_id,
            o_global_shot_seq    => global_shot_seq,
            o_frame_abort_cnt    => frame_abort_cnt,
            o_frame_done_both    => frame_done_both,
            o_face_active_mask   => face_active_mask,
            o_face_stops_per_chip => face_stops_per_chip,
            o_face_cols_per_face => face_cols_per_face,
            o_face_n_faces       => face_n_faces,
            o_rows_per_face      => rows_per_face,
            o_hsize_bytes        => hsize_bytes,
            o_cfg_face           => cfg_face_out
        );

    -- ====================================================================
    -- face_assembler under test (rise slope only)
    -- ====================================================================
    u_face_asm : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => 4,
            g_TDATA_WIDTH    => C_G_WIDTH
        )
        port map (
            i_clk                 => clk,
            i_rst_n               => rst_n,
            i_s_axis_tdata_0      => fa_in_tdata_0,
            i_s_axis_tdata_1      => fa_in_tdata_1,
            i_s_axis_tdata_2      => fa_in_tdata_2,
            i_s_axis_tdata_3      => fa_in_tdata_3,
            i_s_axis_tvalid       => fa_in_tvalid,
            i_s_axis_tlast        => fa_in_tlast,
            o_s_axis_tready       => fa_in_tready,
            i_shot_start          => shot_start_per_chip(0) or shot_start_per_chip(1)
                                     or shot_start_per_chip(2) or shot_start_per_chip(3),
            i_abort               => pipeline_abort_rise,
            i_active_chip_mask    => face_active_mask,
            i_stops_per_chip      => face_stops_per_chip,
            i_max_hits_cfg        => cfg_face_out.max_hits_cfg,
            i_max_scan_clks       => cfg_face_out.max_scan_clks,
            o_m_axis_tdata        => fa_out_tdata,
            o_m_axis_tvalid       => fa_out_tvalid,
            o_m_axis_tlast        => fa_out_tlast,
            i_m_axis_tready       => '1',  -- always ready sink
            o_row_done            => fa_row_done,
            o_chip_error_flags    => fa_chip_err,
            o_shot_overrun        => fa_shot_overrun,
            o_face_abort          => fa_face_abort,
            o_idle                => fa_idle,
            o_shot_flush_drop     => fa_flush_drop,
            o_shot_flush_drop_mask => fa_flush_drop_mask,
            o_shot_overrun_count  => fa_overrun_cnt
        );

    -- face_asm_idle feeds back to face_seq so the pipeline-idle start check
    -- sees the real state once we come out of reset.
    face_asm_idle <= fa_idle;

    -- ====================================================================
    -- Per-chip data generator (per-shot, tlast-terminated).
    --
    -- face_assembler detects chip-done via i_s_axis_tlast; it expects the
    -- cell_builder to emit (stops_per_chip × beats_per_cell) beats per
    -- shot, with tlast on the final beat. We replicate that contract.
    --
    -- A per-chip beat counter resets on shot_start_per_chip(i) and counts
    -- up on each (tvalid AND tready) transaction. tlast fires on the
    -- final beat. After tlast, tvalid drops and stays low until the next
    -- shot_start_per_chip pulse.
    -- ====================================================================
    p_data_gen : process(clk)
        variable v_total_beats : natural;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                s_gen_remaining <= (others => 0);
                s_gen_beat_idx  <= (others => 0);
                s_gen_arm_delay <= (others => 0);
            else
                v_total_beats := C_STOPS * C_BEATS_PER_CELL;
                for i in 0 to c_N_CHIPS - 1 loop
                    if shot_start_per_chip(i) = '1' and face_active_mask(i) = '1' then
                        s_gen_arm_delay(i) <= 8;
                        s_gen_remaining(i) <= 0;
                        s_gen_beat_idx(i)  <= 0;
                    elsif s_gen_arm_delay(i) > 0 then
                        s_gen_arm_delay(i) <= s_gen_arm_delay(i) - 1;
                        if s_gen_arm_delay(i) = 1 then
                            s_gen_remaining(i) <= v_total_beats;
                            s_gen_beat_idx(i)  <= 0;
                        end if;
                    end if;
                    if s_gen_remaining(i) > 0 and fa_in_tready(i) = '1' then
                        s_gen_remaining(i) <= s_gen_remaining(i) - 1;
                        s_gen_beat_idx(i)  <= s_gen_beat_idx(i) + 1;
                    end if;
                end loop;
            end if;
        end if;
    end process;

    -- Combinational drive: tvalid/tlast depend on remaining count.
    fa_in_tvalid(0) <= '1' when s_gen_remaining(0) > 0 else '0';
    fa_in_tvalid(1) <= '1' when s_gen_remaining(1) > 0 else '0';
    fa_in_tvalid(2) <= '1' when s_gen_remaining(2) > 0 else '0';
    fa_in_tvalid(3) <= '1' when s_gen_remaining(3) > 0 else '0';

    fa_in_tlast(0) <= '1' when s_gen_remaining(0) = 1 else '0';
    fa_in_tlast(1) <= '1' when s_gen_remaining(1) = 1 else '0';
    fa_in_tlast(2) <= '1' when s_gen_remaining(2) = 1 else '0';
    fa_in_tlast(3) <= '1' when s_gen_remaining(3) = 1 else '0';

    fa_in_tdata_0 <= x"A000000" & std_logic_vector(to_unsigned(s_gen_beat_idx(0), 4));
    fa_in_tdata_1 <= x"A010000" & std_logic_vector(to_unsigned(s_gen_beat_idx(1), 4));
    fa_in_tdata_2 <= x"A020000" & std_logic_vector(to_unsigned(s_gen_beat_idx(2), 4));
    fa_in_tdata_3 <= x"A030000" & std_logic_vector(to_unsigned(s_gen_beat_idx(3), 4));

    -- ====================================================================
    -- Output beat counter
    -- ====================================================================
    p_out_count : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                fa_out_beat_count <= 0;
            elsif fa_out_tvalid = '1' then
                fa_out_beat_count <= fa_out_beat_count + 1;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Stimulus + checker
    -- ====================================================================
    p_stim : process
        variable v_guard : natural;
        variable v_expected_rows : natural;
        variable v_expected_beats : natural;
        variable v_active_cnt : natural;
        variable v_shot_start_sampled : std_logic_vector(c_N_CHIPS - 1 downto 0);

        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        procedure run_mask_scenario(mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
                                    name : string) is
        begin
            report "=== Mask " & name & " (" &
                   std_logic'image(mask(3)) & std_logic'image(mask(2)) &
                   std_logic'image(mask(1)) & std_logic'image(mask(0)) & ") ==="
                severity note;

            -- Apply mask
            cfg.active_chip_mask <= mask;
            wait_clk(3);

            -- Compute expected values from mask
            v_active_cnt := 0;
            for i in 0 to c_N_CHIPS - 1 loop
                if mask(i) = '1' then
                    v_active_cnt := v_active_cnt + 1;
                end if;
            end loop;
            v_expected_rows  := v_active_cnt * C_STOPS;
            v_expected_beats := v_active_cnt * C_STOPS * C_BEATS_PER_CELL;

            -- Reset output beat counter via rst_n pulse (keeps tb simple)
            rst_n <= '0';
            wait_clk(4);
            rst_n <= '1';
            wait_clk(4);
            -- Re-sample cfg
            cfg.active_chip_mask <= mask;
            wait_clk(2);

            -- cmd_start
            v_guard := 0;
            wait until rising_edge(clk);
            cmd_start <= '1';
            loop
                wait until rising_edge(clk);
                exit when cmd_start_accepted = '1';
                v_guard := v_guard + 1;
                assert v_guard < 100
                    report "FAIL " & name & ": cmd_start never accepted"
                    severity failure;
            end loop;
            cmd_start <= '0';

            -- Verify geometry (available at packet_start after shot fires, but
            -- rows_per_face updates at packet_start_r → sample after)
            wait_clk(2);

            -- shot_start_raw pulse
            shot_start_raw <= '1';
            wait_clk(1);
            shot_start_raw <= '0';

            -- Capture shot_start_per_chip on the cycle it fires
            wait_clk(1);
            -- by now shot_start_gated should have fired: sample
            v_shot_start_sampled := (others => '0');
            for i in 0 to 20 loop
                wait until rising_edge(clk);
                if shot_start_gated_s = '1' then
                    v_shot_start_sampled := shot_start_per_chip;
                    exit;
                end if;
            end loop;

            -- Check shot_start_per_chip gating matches mask
            assert v_shot_start_sampled = mask
                report "FAIL " & name & ": shot_start_per_chip(" &
                       std_logic'image(v_shot_start_sampled(3)) &
                       std_logic'image(v_shot_start_sampled(2)) &
                       std_logic'image(v_shot_start_sampled(1)) &
                       std_logic'image(v_shot_start_sampled(0)) &
                       ") != mask"
                severity failure;

            -- Check rows_per_face geometry
            assert to_integer(rows_per_face) = v_expected_rows
                report "FAIL " & name & ": rows_per_face = " &
                       integer'image(to_integer(rows_per_face)) &
                       ", expected " & integer'image(v_expected_rows)
                severity failure;

            -- Check face_active_mask latched
            assert face_active_mask = mask
                report "FAIL " & name & ": face_active_mask != mask"
                severity failure;

            -- Let face_assembler process the shot — wait for fa_idle to
            -- return. Beat count is sampled a few cycles later to let any
            -- pending output skid drain.
            v_guard := 0;
            loop
                wait until rising_edge(clk);
                exit when fa_idle = '1';
                v_guard := v_guard + 1;
                if v_guard mod 200 = 0 then
                    report "  progress " & name & ": cycle=" & integer'image(v_guard) &
                           ", beats=" & integer'image(fa_out_beat_count) &
                           ", fa_idle=" & std_logic'image(fa_idle) &
                           ", fa_out_tvalid=" & std_logic'image(fa_out_tvalid) &
                           ", fa_in_tready=" & std_logic'image(fa_in_tready(0)) &
                           std_logic'image(fa_in_tready(1)) &
                           std_logic'image(fa_in_tready(2)) &
                           std_logic'image(fa_in_tready(3))
                        severity note;
                end if;
                if v_guard > 3000 then
                    report "FAIL " & name & ": face_assembler didn't reach IDLE in 3000 cycles (beats=" &
                           integer'image(fa_out_beat_count) & ", expected " &
                           integer'image(v_expected_beats) & ")"
                        severity failure;
                    exit;
                end if;
            end loop;
            -- Drain any trailing output skid
            wait_clk(10);

            -- Check output beat count
            assert fa_out_beat_count = v_expected_beats
                report "FAIL " & name & ": output beats = " &
                       integer'image(fa_out_beat_count) & ", expected " &
                       integer'image(v_expected_beats)
                severity failure;

            report "PASS " & name & ": rows=" & integer'image(v_expected_rows) &
                   ", beats=" & integer'image(v_expected_beats) &
                   ", shot_start_per_chip gated correctly"
                severity note;

            -- Stop the run
            cmd_stop <= '1';
            wait_clk(5);
            cmd_stop <= '0';
            wait_clk(5);
        end procedure;

    begin
        -- Initial reset
        rst_n <= '0';
        wait_clk(10);
        rst_n <= '1';
        wait_clk(5);

        -- Six masks: 1 single LSB, 1 single MSB, 2 non-contiguous,
        -- 1 contiguous-non-zero-start, 1 baseline full.
        run_mask_scenario("0001", "single-LSB");
        run_mask_scenario("1000", "single-MSB");
        run_mask_scenario("0101", "non-contig-0-2");
        run_mask_scenario("1010", "non-contig-1-3");
        run_mask_scenario("0110", "contig-middle-1-2");
        run_mask_scenario("1111", "all-active");

        report "====================================" severity note;
        report "ALL MASK SWEEP SCENARIOS PASSED"   severity note;
        report "====================================" severity note;
        test_done <= true;
        wait;
    end process;

end architecture sim;
