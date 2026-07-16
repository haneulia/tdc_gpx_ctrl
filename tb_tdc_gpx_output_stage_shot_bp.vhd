-- =============================================================================
-- tb_tdc_gpx_output_stage_shot_bp.vhd
-- Testbench for tdc_gpx_output_stage — shot-boundary backpressure stress
-- =============================================================================
--
-- Purpose (CHAIN-P0-01 follow-up):
--   Proves that a shot_start_gated pulse arriving while the PREVIOUS line's
--   beats are still buffered in the output path (face_assembler u_fifo_out
--   and/or output_stage face FIFO, held there by VDMA-side backpressure)
--   does NOT destroy those beats.
--
--   The pre-fix RTL resets both FIFOs on every shot_start_gated (xsim
--   xpm_fifo_axis tready-unlock workaround). Under backpressure at the shot
--   boundary that reset drops the previous line's tail (including tlast),
--   desyncing line_packer word alignment and header_inserter column count:
--   the face never completes. This TB fails on that RTL and passes once the
--   reset is gated on FIFO-empty.
--
-- Scenarios:
--   A (sanity, tready high) : two shots back-to-back, no backpressure.
--       Verifies normal geometry AND that the empty-gated reset still
--       delivers the xsim tready-unlock pulse (no beats flow if the FIFO
--       relocks).
--   B (shot-boundary backpressure) : m_axis_tready held low; shot 1's full
--       line is absorbed into the FIFO chain; shot_start for shot 2 fires
--       while those beats are provably in flight (face_buf_tvalid = '1');
--       then tready is released. Expect both lines intact.
--
-- Geometry (32-bit output, fixed):
--   rise mask "0001", 6 stops, max_hits_cfg=7, cols_per_face=2
--   beats/cell (input)   = 5   -> slice = 30 beats
--   canonical words/cell = 5   -> line data = 30 words -> pad -> 32 words
--   line = 12 prefix + 32 data = 44 beats; face = 2 lines = 88 beats
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_output_stage_shot_bp is
end entity tb_tdc_gpx_output_stage_shot_bp;

architecture sim of tb_tdc_gpx_output_stage_shot_bp is

    constant C_OUTPUT_WIDTH   : natural := 32;
    constant C_ALU_PULSE_CLKS : natural := 4;
    constant C_CLK_PERIOD     : time    := 5 ns;   -- 200 MHz
    constant C_KEEP_WIDTH     : natural := fn_axis_keep_width(C_OUTPUT_WIDTH);

    constant C_MAX_HITS       : natural := 7;
    constant C_STOPS          : natural := 6;
    constant C_BEATS_PER_CELL : natural := fn_beats_per_cell_rt(C_MAX_HITS, C_OUTPUT_WIDTH);  -- 5
    constant C_SLICE_BEATS    : natural := C_STOPS * C_BEATS_PER_CELL;                        -- 30
    constant C_WORDS_PER_CELL : natural := fn_canonical_cell_words(C_MAX_HITS);               -- 5
    constant C_DATA_WORDS     : natural := C_STOPS * C_WORDS_PER_CELL;                        -- 30
    constant C_LINE_DATA_WORDS : natural := fn_align_up(C_DATA_WORDS, 4);                     -- 32
    constant C_HDR_BEATS      : natural := fn_hdr_prefix_beats(C_OUTPUT_WIDTH);               -- 12
    constant C_LINE_BEATS     : natural := C_HDR_BEATS + C_LINE_DATA_WORDS;                   -- 44
    constant C_COLS           : natural := 2;
    constant C_FACE_BEATS     : natural := C_COLS * C_LINE_BEATS;                             -- 88

    constant C_BASE_A0 : natural := 16#0100#;
    constant C_BASE_A1 : natural := 16#0200#;
    constant C_BASE_B0 : natural := 16#0300#;
    constant C_BASE_B1 : natural := 16#0400#;

    -- DUT signals
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal cell_rise_tdata_0 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tvalid  : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tlast   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tready  : std_logic_vector(3 downto 0);

    signal shot_start_gated  : std_logic := '0';
    signal pipeline_abort    : std_logic := '0';
    signal face_start_gated  : std_logic := '0';

    signal cfg_face          : t_tdc_cfg := c_TDC_CFG_INIT;

    signal m_axis_tdata      : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
    signal m_axis_tkeep      : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_tstrb      : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_tvalid     : std_logic;
    signal m_axis_tlast      : std_logic;
    signal m_axis_tuser      : std_logic_vector(0 downto 0);
    signal m_axis_tready     : std_logic := '1';

    signal m_axis_fall_tready : std_logic := '1';

    signal row_done             : std_logic;
    signal frame_done           : std_logic;
    signal hdr_idle             : std_logic;
    signal hdr_fall_idle        : std_logic;
    signal face_asm_idle        : std_logic;
    signal face_asm_fall_idle   : std_logic;
    signal face_buf_tvalid      : std_logic;
    signal hdr_drain_timeout_rise : std_logic;
    signal row_done_faulted_rise  : std_logic;

    -- Monitor state
    signal s_beat_cnt        : natural := 0;
    signal s_tlast_cnt       : natural := 0;
    signal s_sof_cnt         : natural := 0;
    signal s_frame_done_cnt  : natural := 0;
    signal s_row_faulted_cnt : natural := 0;
    signal s_data_ok         : boolean := true;
    signal s_keep_ok         : boolean := true;

    -- Expected data bases for the CURRENT face, set by stimulus per scenario
    signal exp_base_line0 : natural := 0;
    signal exp_base_line1 : natural := 0;

    function fn_exp_word(base : natural; word_idx : natural)
        return std_logic_vector is
        variable v : unsigned(31 downto 0);
    begin
        if word_idx >= C_DATA_WORDS then
            return x"00000000";                       -- line-end padding
        end if;
        v := to_unsigned(base + word_idx, 32);
        if (word_idx mod C_WORDS_PER_CELL) = C_WORDS_PER_CELL - 1 then
            v(6 downto 0) := to_unsigned(85, 7);      -- metadata marker
        end if;
        return std_logic_vector(v);
    end function;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    cfg_face.active_chip_mask <= "0001";
    cfg_face.stops_per_chip   <= to_unsigned(C_STOPS, 4);
    cfg_face.max_scan_clks    <= to_unsigned(4000, 16);
    cfg_face.max_hits_cfg     <= to_unsigned(C_MAX_HITS, 3);
    cfg_face.cols_per_face    <= to_unsigned(C_COLS, 16);

    -- =========================================================================
    -- DUT
    -- =========================================================================
    u_dut : entity work.tdc_gpx_output_stage
        generic map (
            g_OUTPUT_WIDTH   => C_OUTPUT_WIDTH,
            g_ALU_PULSE_CLKS => C_ALU_PULSE_CLKS
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            i_cell_rise_tdata_0    => cell_rise_tdata_0,
            i_cell_rise_tdata_1    => (others => '0'),
            i_cell_rise_tdata_2    => (others => '0'),
            i_cell_rise_tdata_3    => (others => '0'),
            i_cell_rise_tvalid     => cell_rise_tvalid,
            i_cell_rise_tlast      => cell_rise_tlast,
            o_cell_rise_tready     => cell_rise_tready,
            i_cell_fall_tdata_0    => (others => '0'),
            i_cell_fall_tdata_1    => (others => '0'),
            i_cell_fall_tdata_2    => (others => '0'),
            i_cell_fall_tdata_3    => (others => '0'),
            i_cell_fall_tvalid     => (others => '0'),
            i_cell_fall_tlast      => (others => '0'),
            o_cell_fall_tready     => open,
            i_cell_rise_tuser      => (others => '0'),
            i_cell_fall_tuser      => (others => '0'),
            i_shot_start_gated     => shot_start_gated,
            i_pipeline_abort       => pipeline_abort,
            i_pipeline_abort_rise  => '0',
            i_pipeline_abort_fall  => '0',
            i_face_start_gated     => face_start_gated,
            i_face_rise_mask       => "0001",
            i_face_fall_mask       => "0000",
            i_face_stops_per_chip  => to_unsigned(C_STOPS, 4),
            i_max_hits_cfg         => to_unsigned(C_MAX_HITS, 3),
            i_max_scan_clks        => to_unsigned(4000, 16),
            i_cell_slots_rise      => to_unsigned(C_STOPS, 16),
            i_cell_slots_fall      => (others => '0'),
            i_cfg_face             => cfg_face,
            i_frame_id             => x"00000001",
            i_face_id              => x"00",
            i_global_shot_seq      => (others => '0'),
            i_timestamp            => (others => '0'),
            i_chip_error_merged    => "0000",
            i_error_count          => (others => '0'),
            i_bin_resolution_ps    => to_unsigned(81, 16),
            i_k_dist_fixed         => (others => '0'),
            o_m_axis_tdata         => m_axis_tdata,
            o_m_axis_tkeep         => m_axis_tkeep,
            o_m_axis_tstrb         => m_axis_tstrb,
            o_m_axis_tvalid        => m_axis_tvalid,
            o_m_axis_tlast         => m_axis_tlast,
            o_m_axis_tuser         => m_axis_tuser,
            i_m_axis_tready        => m_axis_tready,
            o_m_axis_fall_tdata    => open,
            o_m_axis_fall_tkeep    => open,
            o_m_axis_fall_tstrb    => open,
            o_m_axis_fall_tvalid   => open,
            o_m_axis_fall_tlast    => open,
            o_m_axis_fall_tuser    => open,
            i_m_axis_fall_tready   => m_axis_fall_tready,
            o_row_done             => row_done,
            o_row_fall_done        => open,
            o_chip_error_flags     => open,
            o_chip_fall_error      => open,
            o_chip_error_partial_rise => open,
            o_chip_error_blank_rise   => open,
            o_chip_error_partial_fall => open,
            o_chip_error_blank_fall   => open,
            o_shot_overrun         => open,
            o_shot_fall_overrun    => open,
            o_face_abort           => open,
            o_face_fall_abort      => open,
            o_face_asm_idle        => face_asm_idle,
            o_face_asm_fall_idle   => face_asm_fall_idle,
            o_frame_done           => frame_done,
            o_frame_fall_done      => open,
            o_hdr_draining         => open,
            o_hdr_fall_draining    => open,
            o_hdr_idle             => hdr_idle,
            o_hdr_fall_idle        => hdr_fall_idle,
            o_face_tvalid          => open,
            o_face_fall_tvalid     => open,
            o_face_buf_tvalid      => face_buf_tvalid,
            o_face_fall_buf_tvalid => open,
            o_shot_flush_drop_rise => open,
            o_shot_flush_drop_fall => open,
            o_shot_flush_drop_mask_rise => open,
            o_shot_flush_drop_mask_fall => open,
            o_shot_overrun_count_rise => open,
            o_shot_overrun_count_fall => open,
            o_hdr_face_start_collapsed_rise => open,
            o_hdr_face_start_collapsed_fall => open,
            o_hdr_drain_timeout_rise        => hdr_drain_timeout_rise,
            o_hdr_drain_timeout_fall        => open,
            o_hdr_abort_truncated_rise      => open,
            o_hdr_abort_truncated_fall      => open,
            o_frame_done_faulted_rise       => open,
            o_frame_done_faulted_fall       => open,
            o_row_done_faulted_rise         => row_done_faulted_rise,
            o_row_done_faulted_fall         => open
        );

    -- =========================================================================
    -- Rise VDMA output monitor: beat/tlast/SOF counters + exact word check
    -- =========================================================================
    p_monitor : process(clk)
        variable v_line_beat : natural := 0;   -- beat index within current line
        variable v_line_idx  : natural := 0;   -- line index within current face
        variable v_word_idx  : natural;
        variable v_base      : natural;
    begin
        if rising_edge(clk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                s_beat_cnt <= s_beat_cnt + 1;

                if m_axis_tuser(0) = '1' then
                    s_sof_cnt  <= s_sof_cnt + 1;
                    v_line_beat := 0;
                    v_line_idx  := 0;
                end if;

                if m_axis_tkeep /= (m_axis_tkeep'range => '1')
                   or m_axis_tstrb /= (m_axis_tstrb'range => '1') then
                    s_keep_ok <= false;
                end if;

                if v_line_beat >= C_HDR_BEATS then
                    v_word_idx := v_line_beat - C_HDR_BEATS;
                    if v_line_idx = 0 then
                        v_base := exp_base_line0;
                    else
                        v_base := exp_base_line1;
                    end if;
                    if m_axis_tdata /= fn_exp_word(v_base, v_word_idx) then
                        s_data_ok <= false;
                        report "shot_bp monitor: data mismatch line="
                               & integer'image(v_line_idx)
                               & " word=" & integer'image(v_word_idx)
                            severity warning;
                    end if;
                end if;

                if m_axis_tlast = '1' then
                    s_tlast_cnt <= s_tlast_cnt + 1;
                    v_line_beat := 0;
                    v_line_idx  := v_line_idx + 1;
                else
                    v_line_beat := v_line_beat + 1;
                end if;
            end if;

            if frame_done = '1' then
                s_frame_done_cnt <= s_frame_done_cnt + 1;
            end if;
            if row_done_faulted_rise = '1' then
                s_row_faulted_cnt <= s_row_faulted_cnt + 1;
            end if;
        end if;
    end process p_monitor;

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    p_stim : process
        procedure drive_slice(base : natural) is
            variable v_beat : unsigned(31 downto 0);
        begin
            for beat_idx in 0 to C_SLICE_BEATS - 1 loop
                while cell_rise_tready(0) = '0' loop
                    wait until rising_edge(clk);
                end loop;
                v_beat := to_unsigned(base + beat_idx, 32);
                if (beat_idx mod C_BEATS_PER_CELL) = C_BEATS_PER_CELL - 1 then
                    v_beat(6 downto 0) := to_unsigned(85, 7);
                end if;
                cell_rise_tdata_0 <= std_logic_vector(v_beat);
                cell_rise_tvalid  <= "0001";
                if beat_idx = C_SLICE_BEATS - 1 then
                    cell_rise_tlast <= "0001";
                else
                    cell_rise_tlast <= "0000";
                end if;
                wait until rising_edge(clk);
            end loop;
            cell_rise_tvalid <= "0000";
            cell_rise_tlast  <= "0000";
        end procedure;

        procedure pulse(signal s : out std_logic) is
        begin
            s <= '1';
            wait until rising_edge(clk);
            s <= '0';
        end procedure;

        variable v_base_beats  : natural;
        variable v_base_tlast  : natural;
        variable v_base_sof    : natural;
        variable v_base_frame  : natural;
    begin
        rst_n <= '0';
        wait for 100 ns;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait for 10 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- SCENARIO A: no-backpressure sanity (also proves the empty-gated
        -- reset still delivers the xsim tready-unlock pulse)
        -- =====================================================================
        report "===== SHOT_BP SCENARIO A: no-backpressure sanity =====" severity note;
        exp_base_line0 <= C_BASE_A0;
        exp_base_line1 <= C_BASE_A1;
        m_axis_tready  <= '1';
        wait until rising_edge(clk);

        pulse(face_start_gated);
        wait for 3 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        pulse(shot_start_gated);
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        drive_slice(C_BASE_A0);
        wait until row_done = '1' for 20 us;
        assert row_done = '1'
            report "SCENARIO A: row_done (shot 1) missing" severity failure;
        wait for 20 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        pulse(shot_start_gated);
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        drive_slice(C_BASE_A1);

        wait until frame_done = '1' for 50 us;
        wait for 10 * C_CLK_PERIOD;

        report "SCENARIO A: beats=" & integer'image(s_beat_cnt)
               & " tlast=" & integer'image(s_tlast_cnt)
               & " sof=" & integer'image(s_sof_cnt)
               & " frame_done=" & integer'image(s_frame_done_cnt) severity note;

        if s_beat_cnt = C_FACE_BEATS and s_tlast_cnt = C_COLS
           and s_sof_cnt = 1 and s_frame_done_cnt = 1
           and s_data_ok and s_keep_ok
           and s_row_faulted_cnt = 0
           and hdr_drain_timeout_rise = '0' then
            report "*** SHOT_BP SCENARIO A PASS ***" severity note;
        else
            report "*** SHOT_BP SCENARIO A FAIL ***" severity failure;
        end if;

        v_base_beats := s_beat_cnt;
        v_base_tlast := s_tlast_cnt;
        v_base_sof   := s_sof_cnt;
        v_base_frame := s_frame_done_cnt;

        wait until hdr_idle = '1' and face_asm_idle = '1' for 20 us;
        assert hdr_idle = '1' and face_asm_idle = '1'
            report "SCENARIO A: rise pipeline did not return to idle" severity failure;

        -- =====================================================================
        -- SCENARIO B: shot_start while previous line is buffered under
        -- VDMA backpressure. Pre-fix RTL wipes the buffered tail here.
        -- =====================================================================
        report "===== SHOT_BP SCENARIO B: shot-boundary backpressure =====" severity note;
        exp_base_line0 <= C_BASE_B0;
        exp_base_line1 <= C_BASE_B1;
        m_axis_tready  <= '0';               -- VDMA stalled from the start
        wait until rising_edge(clk);

        pulse(face_start_gated);
        wait for 3 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        pulse(shot_start_gated);
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        drive_slice(C_BASE_B0);
        wait until row_done = '1' for 20 us;
        assert row_done = '1'
            report "SCENARIO B: row_done (shot 1) missing - slice not absorbed"
            severity failure;
        wait for 30 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- Source marker: previous line's beats must be provably in flight.
        assert face_buf_tvalid = '1'
            report "SCENARIO B: expected buffered face data before shot 2 "
                   & "(face_buf_tvalid=0 - test premise broken)"
            severity failure;

        -- THE trigger: next shot while the previous line is still buffered.
        -- On pre-fix RTL both output-path FIFOs are reset by this pulse and
        -- the buffered line-0 tail is destroyed at this exact moment.
        pulse(shot_start_gated);
        wait for 10 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- Release the VDMA stall, then feed shot 2. (Releasing before the
        -- second slice keeps the TB free of a buffer-capacity deadlock; the
        -- destructive event, if any, has already happened at the pulse.)
        m_axis_tready <= '1';
        wait until rising_edge(clk);
        drive_slice(C_BASE_B1);

        wait until frame_done = '1' for 100 us;
        wait for 10 * C_CLK_PERIOD;

        report "SCENARIO B: beats=" & integer'image(s_beat_cnt - v_base_beats)
               & " tlast=" & integer'image(s_tlast_cnt - v_base_tlast)
               & " sof=" & integer'image(s_sof_cnt - v_base_sof)
               & " frame_done=" & integer'image(s_frame_done_cnt - v_base_frame)
               severity note;

        if (s_beat_cnt - v_base_beats) = C_FACE_BEATS
           and (s_tlast_cnt - v_base_tlast) = C_COLS
           and (s_sof_cnt - v_base_sof) = 1
           and (s_frame_done_cnt - v_base_frame) = 1
           and s_data_ok and s_keep_ok
           and s_row_faulted_cnt = 0
           and hdr_drain_timeout_rise = '0' then
            report "*** SHOT_BP SCENARIO B PASS ***" severity note;
        else
            report "*** SHOT_BP SCENARIO B FAIL (shot-boundary flush destroyed "
                   & "buffered line data) ***" severity failure;
        end if;

        report "*** TB_SHOT_BP ALL PASS ***" severity note;
        wait for 10 * C_CLK_PERIOD;
        std.env.finish;
    end process p_stim;

end architecture sim;
