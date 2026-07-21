-- =============================================================================
-- tb_tdc_gpx_output_stage.vhd
-- Testbench for tdc_gpx_output_stage (Cluster 4 wrapper)
-- =============================================================================
--
-- Purpose:
--   Smoke-test: feed cell data on chip 0 rising input, verify that
--   face_assembler picks it up, header is prepended, VDMA output fires,
--   and frame_done asserts.
--
-- Configuration:
--   G_OUTPUT_WIDTH   = 64 default, run also with 32 and 128 for Phase A
--   1 active chip (mask "0001"), 2 stops, 2 rows
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_output_stage is
    generic (
        G_OUTPUT_WIDTH        : natural := 64;
        G_RUN_MAX_HITS_SWEEP : boolean := false
    );
end entity tb_tdc_gpx_output_stage;

architecture sim of tb_tdc_gpx_output_stage is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_OUTPUT_WIDTH   : natural := G_OUTPUT_WIDTH;
    constant C_CLK_PERIOD     : time    := 5 ns;   -- 200 MHz
    constant C_WATCHDOG       : time    := 50 us;
    constant C_KEEP_WIDTH     : natural := fn_axis_keep_width(C_OUTPUT_WIDTH);

    -- Beats per cell follows the runtime max_hits_cfg contract used by
    -- C03 cell_builder and C04 face_assembler.
    constant C_MAX_HITS_CFG_N : natural := 7;
    constant C_BEATS_PER_CELL : natural := fn_beats_per_cell_rt(C_MAX_HITS_CFG_N, C_OUTPUT_WIDTH);
    constant C_HDR_PREFIX_BEATS : natural := fn_hdr_prefix_beats(C_OUTPUT_WIDTH);
    constant C_WORDS_PER_BEAT : natural := C_OUTPUT_WIDTH / 32;
    constant C_STOPS          : natural := 2;
    constant C_TOTAL_BEATS    : natural := C_STOPS * C_BEATS_PER_CELL;

    -- =========================================================================
    -- DUT signals
    -- =========================================================================
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- Cell rise inputs (chip 0..3)
    signal cell_rise_tdata_0 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_1 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_2 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tdata_3 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_rise_tvalid  : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tlast   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tuser   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_rise_tready  : std_logic_vector(3 downto 0);

    -- Cell fall inputs (all tied low)
    signal cell_fall_tdata_0 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_1 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_2 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tdata_3 : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
    signal cell_fall_tvalid  : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_fall_tlast   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_fall_tuser   : std_logic_vector(3 downto 0) := (others => '0');
    signal cell_fall_tready  : std_logic_vector(3 downto 0);

    -- Control
    signal shot_start_gated  : std_logic := '0';
    signal pipeline_abort    : std_logic := '0';
    -- #22 Sprint 3 plumbing: per-slope abort drives.
    -- Default both to '0' so the existing smoke test keeps its current
    -- behavior (no abort). Future slope-independence tests wiggle them
    -- independently to verify fall-only abort leaves rise output alive.
    signal pipeline_abort_rise : std_logic := '0';
    signal pipeline_abort_fall : std_logic := '0';
    signal face_start_gated  : std_logic := '0';

    -- Configuration
    signal face_active_mask    : std_logic_vector(3 downto 0) := "0001";
    signal face_stops_per_chip : unsigned(3 downto 0)         := to_unsigned(2, 4);
    signal max_hits_cfg        : unsigned(2 downto 0)         := to_unsigned(7, 3);
    signal cfg_max_hits_cfg    : unsigned(2 downto 0)         := to_unsigned(7, 3);
    signal max_scan_clks       : unsigned(15 downto 0)        := to_unsigned(1000, 16);
    signal rows_per_face       : unsigned(15 downto 0)        := to_unsigned(2, 16);

    -- Header metadata
    signal cfg_face           : t_tdc_cfg := c_TDC_CFG_INIT;
    signal frame_id           : unsigned(31 downto 0)                       := x"00000001";
    signal face_id            : unsigned(7 downto 0)                        := x"00";
    signal global_shot_seq    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0)     := (others => '0');
    signal timestamp          : unsigned(63 downto 0)                       := (others => '0');
    signal chip_error_merged  : std_logic_vector(3 downto 0)                := "0000";
    signal error_count        : unsigned(31 downto 0)                       := (others => '0');
    signal bin_resolution_ps  : unsigned(15 downto 0)                       := to_unsigned(81, 16);
    signal k_dist_fixed       : unsigned(31 downto 0)                       := (others => '0');

    -- VDMA rise output
    signal m_axis_tdata       : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
    signal m_axis_tkeep       : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_tstrb       : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_tvalid      : std_logic;
    signal m_axis_tlast       : std_logic;
    signal m_axis_tuser       : std_logic_vector(0 downto 0);
    signal m_axis_tready      : std_logic := '1';

    -- VDMA fall output
    signal m_axis_fall_tdata  : std_logic_vector(C_OUTPUT_WIDTH - 1 downto 0);
    signal m_axis_fall_tkeep  : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_fall_tstrb  : std_logic_vector(C_KEEP_WIDTH - 1 downto 0);
    signal m_axis_fall_tvalid : std_logic;
    signal m_axis_fall_tlast  : std_logic;
    signal m_axis_fall_tuser  : std_logic_vector(0 downto 0);
    signal m_axis_fall_tready : std_logic := '1';

    -- Status
    signal row_done             : std_logic;
    signal row_fall_done        : std_logic;
    signal chip_error_flags     : std_logic_vector(3 downto 0);
    signal chip_fall_error      : std_logic_vector(3 downto 0);
    signal chip_error_partial_rise : std_logic_vector(3 downto 0);
    signal chip_error_blank_rise   : std_logic_vector(3 downto 0);
    signal chip_error_partial_fall : std_logic_vector(3 downto 0);
    signal chip_error_blank_fall   : std_logic_vector(3 downto 0);
    signal shot_overrun         : std_logic;
    signal shot_fall_overrun    : std_logic;
    signal face_abort           : std_logic;
    signal face_fall_abort      : std_logic;
    signal face_asm_idle        : std_logic;
    signal face_asm_fall_idle   : std_logic;
    signal frame_done           : std_logic;
    signal frame_fall_done      : std_logic;
    signal hdr_draining         : std_logic;
    signal hdr_fall_draining    : std_logic;
    signal hdr_idle             : std_logic;
    signal hdr_fall_idle        : std_logic;
    signal face_tvalid          : std_logic;
    signal face_fall_tvalid     : std_logic;
    signal face_buf_tvalid      : std_logic;
    signal face_fall_buf_tvalid : std_logic;
    signal shot_flush_drop_mask_rise : std_logic_vector(3 downto 0);
    signal shot_flush_drop_mask_fall : std_logic_vector(3 downto 0);
    signal hdr_drain_timeout_rise    : std_logic;
    signal hdr_drain_timeout_fall    : std_logic;
    signal hdr_abort_truncated_rise  : std_logic;
    signal hdr_abort_truncated_fall  : std_logic;
    signal frame_done_faulted_rise   : std_logic;
    signal frame_done_faulted_fall   : std_logic;
    signal row_done_faulted_rise     : std_logic;
    signal row_done_faulted_fall     : std_logic;

    -- Monitor counters
    signal v_out_beat_cnt : natural := 0;
    signal v_sof_seen     : boolean := false;
    signal v_eol_seen     : boolean := false;
    -- #22 slope-independent test: per-slope output monitors
    signal v_out_fall_beat_cnt : natural := 0;
    signal v_fall_eol_seen     : boolean := false;
    signal v_frame_done_cnt    : natural := 0;
    signal v_frame_fall_done_cnt : natural := 0;
    signal s_sof_count          : natural := 0;
    signal s_row_faulted_rise_cnt : natural := 0;
    signal s_row_faulted_fall_cnt : natural := 0;
    signal s_frame_faulted_rise_cnt : natural := 0;
    signal s_frame_faulted_fall_cnt : natural := 0;
    signal s_keep_ok            : boolean := true;
    signal s_fall_keep_ok       : boolean := true;
    signal s_metadata_hit_msb_ok : boolean := true;
    signal s_metadata_seen_count  : natural := 0;
    signal s_fall_metadata_hit_msb_ok : boolean := true;
    signal s_fall_metadata_seen_count  : natural := 0;

    function fn_effective_max_hits_cfg(
        cfg : unsigned(2 downto 0)
    ) return natural is
    begin
        if cfg = "000" then
            return 7;
        end if;
        return to_integer(cfg);
    end function;

begin

    -- =========================================================================
    -- Clock generation (200 MHz)
    -- =========================================================================
    clk <= not clk after C_CLK_PERIOD / 2;

    -- =========================================================================
    -- DUT instantiation
    -- =========================================================================
    u_dut : entity work.tdc_gpx_output_stage
        generic map (
            g_OUTPUT_WIDTH => C_OUTPUT_WIDTH
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            -- Rise cell inputs
            i_cell_rise_tdata_0    => cell_rise_tdata_0,
            i_cell_rise_tdata_1    => cell_rise_tdata_1,
            i_cell_rise_tdata_2    => cell_rise_tdata_2,
            i_cell_rise_tdata_3    => cell_rise_tdata_3,
            i_cell_rise_tvalid     => cell_rise_tvalid,
            i_cell_rise_tlast      => cell_rise_tlast,
            o_cell_rise_tready     => cell_rise_tready,
            i_cell_rise_tuser      => cell_rise_tuser,
            -- Fall cell inputs
            i_cell_fall_tdata_0    => cell_fall_tdata_0,
            i_cell_fall_tdata_1    => cell_fall_tdata_1,
            i_cell_fall_tdata_2    => cell_fall_tdata_2,
            i_cell_fall_tdata_3    => cell_fall_tdata_3,
            i_cell_fall_tvalid     => cell_fall_tvalid,
            i_cell_fall_tlast      => cell_fall_tlast,
            o_cell_fall_tready     => cell_fall_tready,
            i_cell_fall_tuser      => cell_fall_tuser,
            -- Control
            i_shot_start_gated     => shot_start_gated,
            i_pipeline_abort       => pipeline_abort,
            i_pipeline_abort_rise  => pipeline_abort_rise,
            i_pipeline_abort_fall  => pipeline_abort_fall,
            i_face_start_gated     => face_start_gated,
            -- Configuration
            i_face_rise_mask       => face_active_mask,
            i_face_fall_mask       => face_active_mask,
            i_face_stops_per_chip  => face_stops_per_chip,
            i_max_hits_cfg         => max_hits_cfg,
            i_max_scan_clks        => max_scan_clks,
            i_cell_slots_rise      => rows_per_face,
            i_cell_slots_fall      => rows_per_face,
            -- Header metadata
            i_cfg_face             => cfg_face,
            i_frame_id             => frame_id,
            i_face_id              => face_id,
            i_global_shot_seq      => global_shot_seq,
            i_timestamp            => timestamp,
            i_chip_error_merged    => chip_error_merged,
            i_error_count          => error_count,
            i_bin_resolution_ps    => bin_resolution_ps,
            i_k_dist_fixed         => k_dist_fixed,
            -- VDMA rise output
            o_m_axis_tdata         => m_axis_tdata,
            o_m_axis_tkeep         => m_axis_tkeep,
            o_m_axis_tstrb         => m_axis_tstrb,
            o_m_axis_tvalid        => m_axis_tvalid,
            o_m_axis_tlast         => m_axis_tlast,
            o_m_axis_tuser         => m_axis_tuser,
            i_m_axis_tready        => m_axis_tready,
            -- VDMA fall output
            o_m_axis_fall_tdata    => m_axis_fall_tdata,
            o_m_axis_fall_tkeep    => m_axis_fall_tkeep,
            o_m_axis_fall_tstrb    => m_axis_fall_tstrb,
            o_m_axis_fall_tvalid   => m_axis_fall_tvalid,
            o_m_axis_fall_tlast    => m_axis_fall_tlast,
            o_m_axis_fall_tuser    => m_axis_fall_tuser,
            i_m_axis_fall_tready   => m_axis_fall_tready,
            -- Status
            o_row_done             => row_done,
            o_row_fall_done        => row_fall_done,
            o_chip_error_flags     => chip_error_flags,
            o_chip_fall_error      => chip_fall_error,
            o_chip_error_partial_rise => chip_error_partial_rise,
            o_chip_error_blank_rise   => chip_error_blank_rise,
            o_chip_error_partial_fall => chip_error_partial_fall,
            o_chip_error_blank_fall   => chip_error_blank_fall,
            o_shot_overrun         => shot_overrun,
            o_shot_fall_overrun    => shot_fall_overrun,
            o_face_abort           => face_abort,
            o_face_fall_abort      => face_fall_abort,
            o_face_asm_idle        => face_asm_idle,
            o_face_asm_fall_idle   => face_asm_fall_idle,
            o_frame_done           => frame_done,
            o_frame_fall_done      => frame_fall_done,
            o_hdr_draining         => hdr_draining,
            o_hdr_fall_draining    => hdr_fall_draining,
            o_hdr_idle             => hdr_idle,
            o_hdr_fall_idle        => hdr_fall_idle,
            o_face_tvalid          => face_tvalid,
            o_face_fall_tvalid     => face_fall_tvalid,
            o_face_buf_tvalid      => face_buf_tvalid,
            o_face_fall_buf_tvalid => face_fall_buf_tvalid,
            o_shot_flush_drop_rise    => open,
            o_shot_flush_drop_fall    => open,
            o_shot_flush_drop_mask_rise => shot_flush_drop_mask_rise,
            o_shot_flush_drop_mask_fall => shot_flush_drop_mask_fall,
            o_shot_overrun_count_rise => open,
            o_shot_overrun_count_fall => open,
            o_hdr_face_start_collapsed_rise => open,
            o_hdr_face_start_collapsed_fall => open,
            o_hdr_drain_timeout_rise        => hdr_drain_timeout_rise,
            o_hdr_drain_timeout_fall        => hdr_drain_timeout_fall,
            o_hdr_abort_truncated_rise      => hdr_abort_truncated_rise,
            o_hdr_abort_truncated_fall      => hdr_abort_truncated_fall,
            o_frame_done_faulted_rise       => frame_done_faulted_rise,
            o_frame_done_faulted_fall       => frame_done_faulted_fall,
            o_row_done_faulted_rise         => row_done_faulted_rise,
            o_row_done_faulted_fall         => row_done_faulted_fall
        );

    -- =========================================================================
    -- Override cfg_face fields for this test
    -- =========================================================================
    cfg_face.active_chip_mask <= "0001";
    cfg_face.stops_per_chip   <= to_unsigned(2, 4);
    cfg_face.max_scan_clks    <= to_unsigned(1000, 16);
    cfg_face.max_hits_cfg     <= cfg_max_hits_cfg;
    cfg_face.cols_per_face    <= to_unsigned(1, 16);  -- 1 col = 1 shot per frame

    -- =========================================================================
    -- VDMA output monitor
    -- =========================================================================
    p_monitor : process(clk)
        variable v_out_idx  : natural;
        variable v_data_word_idx : natural;
        variable v_rise_line_beat_cnt : natural := 0;
        variable v_mon_words_per_cell : natural;
        variable v_mon_data_words : natural;
        variable v_fall_out_idx  : natural;
        variable v_fall_data_word_idx : natural;
        variable v_fall_line_beat_cnt : natural := 0;
        variable v_fall_words_per_cell : natural;
        variable v_fall_data_words : natural;
        variable v_meta_inc : natural range 0 to 4;
        variable v_fall_meta_inc : natural range 0 to 4;
        variable v_word : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            if m_axis_tvalid = '1' and m_axis_tready = '1' then
                v_mon_words_per_cell := fn_canonical_cell_words(
                    fn_effective_max_hits_cfg(max_hits_cfg));
                v_mon_data_words := C_STOPS * v_mon_words_per_cell;
                if m_axis_tuser(0) = '1' then
                    v_rise_line_beat_cnt := 0;
                end if;
                v_out_idx := v_rise_line_beat_cnt;
                v_out_beat_cnt <= v_out_beat_cnt + 1;
                if m_axis_tkeep /= (m_axis_tkeep'range => '1')
                   or m_axis_tstrb /= (m_axis_tstrb'range => '1') then
                    s_keep_ok <= false;
                end if;
                if v_out_idx >= C_HDR_PREFIX_BEATS then
                    v_meta_inc := 0;
                    for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                        v_data_word_idx := (v_out_idx - C_HDR_PREFIX_BEATS)
                                           * C_WORDS_PER_BEAT + lane;
                        if v_data_word_idx < v_mon_data_words
                           and (v_data_word_idx mod v_mon_words_per_cell)
                               = v_mon_words_per_cell - 1 then
                            v_meta_inc := v_meta_inc + 1;
                            v_word := m_axis_tdata(32 * lane + 31 downto 32 * lane);
                            if v_word(6 downto 0) /= "1010101" then
                                s_metadata_hit_msb_ok <= false;
                            end if;
                        end if;
                    end loop;
                    if v_meta_inc > 0 then
                        s_metadata_seen_count <= s_metadata_seen_count + v_meta_inc;
                    end if;
                end if;
                if m_axis_tuser(0) = '1' then
                    v_sof_seen <= true;
                    s_sof_count <= s_sof_count + 1;
                end if;
                if m_axis_tlast = '1' then
                    v_eol_seen <= true;
                    v_rise_line_beat_cnt := 0;
                else
                    v_rise_line_beat_cnt := v_rise_line_beat_cnt + 1;
                end if;
            end if;
            -- Fall-side monitor (#22)
            if m_axis_fall_tvalid = '1' and m_axis_fall_tready = '1' then
                v_fall_words_per_cell := fn_canonical_cell_words(
                    fn_effective_max_hits_cfg(max_hits_cfg));
                v_fall_data_words := C_STOPS * v_fall_words_per_cell;
                if m_axis_fall_tuser(0) = '1' then
                    v_fall_line_beat_cnt := 0;
                end if;
                v_fall_out_idx := v_fall_line_beat_cnt;
                v_out_fall_beat_cnt <= v_out_fall_beat_cnt + 1;
                if m_axis_fall_tkeep /= (m_axis_fall_tkeep'range => '1')
                   or m_axis_fall_tstrb /= (m_axis_fall_tstrb'range => '1') then
                    s_fall_keep_ok <= false;
                end if;
                if v_fall_out_idx >= C_HDR_PREFIX_BEATS then
                    v_fall_meta_inc := 0;
                    for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                        v_fall_data_word_idx :=
                            (v_fall_out_idx - C_HDR_PREFIX_BEATS)
                            * C_WORDS_PER_BEAT + lane;
                        if v_fall_data_word_idx < v_fall_data_words
                           and (v_fall_data_word_idx mod v_fall_words_per_cell)
                               = v_fall_words_per_cell - 1 then
                            v_fall_meta_inc := v_fall_meta_inc + 1;
                            v_word := m_axis_fall_tdata(32 * lane + 31 downto 32 * lane);
                            if v_word(6 downto 0) /= "1010101" then
                                s_fall_metadata_hit_msb_ok <= false;
                            end if;
                        end if;
                    end loop;
                    if v_fall_meta_inc > 0 then
                        s_fall_metadata_seen_count <=
                            s_fall_metadata_seen_count + v_fall_meta_inc;
                    end if;
                end if;
                if m_axis_fall_tlast = '1' then
                    v_fall_eol_seen <= true;
                    v_fall_line_beat_cnt := 0;
                else
                    v_fall_line_beat_cnt := v_fall_line_beat_cnt + 1;
                end if;
            end if;
            -- frame_done pulse counters (1-cycle pulses, need to count)
            if frame_done = '1' then
                v_frame_done_cnt <= v_frame_done_cnt + 1;
            end if;
            if frame_fall_done = '1' then
                v_frame_fall_done_cnt <= v_frame_fall_done_cnt + 1;
            end if;
            if row_done_faulted_rise = '1' then
                s_row_faulted_rise_cnt <= s_row_faulted_rise_cnt + 1;
            end if;
            if row_done_faulted_fall = '1' then
                s_row_faulted_fall_cnt <= s_row_faulted_fall_cnt + 1;
            end if;
            if frame_done_faulted_rise = '1' then
                s_frame_faulted_rise_cnt <= s_frame_faulted_rise_cnt + 1;
            end if;
            if frame_done_faulted_fall = '1' then
                s_frame_faulted_fall_cnt <= s_frame_faulted_fall_cnt + 1;
            end if;
        end if;
    end process p_monitor;

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stim : process
        variable v_beat_data : unsigned(C_OUTPUT_WIDTH - 1 downto 0);
        variable v_sweep_base_out : natural;
        variable v_sweep_base_meta : natural;
        variable v_sweep_base_meta_fall : natural;
        variable v_sweep_base_sof : natural;
        variable v_sweep_base_frame : natural;
        variable v_sweep_base_frame_fall : natural;
        variable v_sweep_effective_max_hits : natural;
        variable v_sweep_beats_per_cell : natural;
        variable v_sweep_total_beats : natural;
        variable v_sweep_expected_out : natural;
    begin
        -- =====================================================================
        -- 1. Reset
        -- =====================================================================
        rst_n <= '0';
        wait for 100 ns;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        wait until rising_edge(clk);

        if G_RUN_MAX_HITS_SWEEP then
            report "===== SCENARIO 3: max_hits_cfg sweep 0..7 =====" severity note;

            for cfg_idx in 0 to 7 loop
                v_sweep_effective_max_hits := cfg_idx;
                if cfg_idx = 0 then
                    v_sweep_effective_max_hits := 7;
                end if;
                v_sweep_beats_per_cell := fn_beats_per_cell_rt(
                    v_sweep_effective_max_hits,
                    C_OUTPUT_WIDTH
                );
                v_sweep_total_beats := C_STOPS * v_sweep_beats_per_cell;
                v_sweep_expected_out :=
                    fn_vdma_line_bytes(C_STOPS, v_sweep_effective_max_hits)
                    / (C_OUTPUT_WIDTH / 8);

                v_sweep_base_out  := v_out_beat_cnt;
                v_sweep_base_meta := s_metadata_seen_count;
                v_sweep_base_meta_fall := s_fall_metadata_seen_count;
                v_sweep_base_sof  := s_sof_count;
                v_sweep_base_frame := v_frame_done_cnt;
                v_sweep_base_frame_fall := v_frame_fall_done_cnt;

                max_hits_cfg     <= to_unsigned(cfg_idx, 3);
                cfg_max_hits_cfg <= to_unsigned(cfg_idx, 3);
                wait for 4 * C_CLK_PERIOD;
                wait until rising_edge(clk);

                face_start_gated <= '1';
                wait until rising_edge(clk);
                face_start_gated <= '0';
                wait for 3 * C_CLK_PERIOD;
                wait until rising_edge(clk);

                shot_start_gated <= '1';
                wait until rising_edge(clk);
                shot_start_gated <= '0';
                wait for 5 * C_CLK_PERIOD;
                wait until rising_edge(clk);

                for beat_idx in 0 to v_sweep_total_beats - 1 loop
                    while cell_rise_tready(0) = '0' or cell_fall_tready(0) = '0' loop
                        wait until rising_edge(clk);
                    end loop;

                    v_beat_data := to_unsigned((cfg_idx + 1) * 16 + beat_idx + 1, C_OUTPUT_WIDTH);
                    if (beat_idx mod v_sweep_beats_per_cell) = (v_sweep_beats_per_cell - 1) then
                        v_beat_data(6 downto 0) := to_unsigned(85, 7);
                    end if;
                    cell_rise_tdata_0 <= std_logic_vector(v_beat_data);
                    cell_fall_tdata_0 <= std_logic_vector(v_beat_data);
                    cell_rise_tvalid  <= "0001";
                    cell_fall_tvalid  <= "0001";

                    if beat_idx = v_sweep_total_beats - 1 then
                        cell_rise_tlast <= "0001";
                        cell_fall_tlast <= "0001";
                    else
                        cell_rise_tlast <= "0000";
                        cell_fall_tlast <= "0000";
                    end if;
                    cell_rise_tuser <= "0000";
                    cell_fall_tuser <= "0000";

                    wait until rising_edge(clk);
                end loop;

                cell_rise_tvalid <= "0000";
                cell_fall_tvalid <= "0000";
                cell_rise_tlast  <= "0000";
                cell_fall_tlast  <= "0000";
                cell_rise_tuser  <= "0000";
                cell_fall_tuser  <= "0000";

                wait until (v_frame_done_cnt > v_sweep_base_frame)
                           and (v_frame_fall_done_cnt > v_sweep_base_frame_fall)
                           for C_WATCHDOG;
                wait for 4 * C_CLK_PERIOD;

                assert (v_frame_done_cnt - v_sweep_base_frame) = 1
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": frame_done delta="
                           & integer'image(v_frame_done_cnt - v_sweep_base_frame)
                           & " expected=1"
                    severity failure;
                assert (v_frame_fall_done_cnt - v_sweep_base_frame_fall) = 1
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": fall frame_done delta="
                           & integer'image(v_frame_fall_done_cnt - v_sweep_base_frame_fall)
                           & " expected=1"
                    severity failure;
                assert (v_out_beat_cnt - v_sweep_base_out) = v_sweep_expected_out
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": output beats delta="
                           & integer'image(v_out_beat_cnt - v_sweep_base_out)
                           & " expected=" & integer'image(v_sweep_expected_out)
                    severity failure;
                assert (s_metadata_seen_count - v_sweep_base_meta) = C_STOPS
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": metadata seen delta="
                           & integer'image(s_metadata_seen_count - v_sweep_base_meta)
                           & " expected=" & integer'image(C_STOPS)
                    severity failure;
                assert (s_fall_metadata_seen_count - v_sweep_base_meta_fall) = C_STOPS
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": fall metadata seen delta="
                           & integer'image(s_fall_metadata_seen_count - v_sweep_base_meta_fall)
                           & " expected=" & integer'image(C_STOPS)
                    severity failure;
                assert (s_sof_count - v_sweep_base_sof) = 1
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": SOF delta="
                           & integer'image(s_sof_count - v_sweep_base_sof)
                           & " expected=1"
                    severity failure;
                assert s_metadata_hit_msb_ok
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": metadata[6:0] Hit[16] preserve failed"
                    severity failure;
                assert s_fall_metadata_hit_msb_ok
                    report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                           & ": fall metadata[6:0] Hit[16] preserve failed"
                    severity failure;

                report "SCENARIO 3 cfg=" & integer'image(cfg_idx)
                       & " effective_hits=" & integer'image(v_sweep_effective_max_hits)
                       & " beats_per_cell=" & integer'image(v_sweep_beats_per_cell)
                       & " output_delta=" & integer'image(v_out_beat_cnt - v_sweep_base_out)
                       & " PASS" severity note;

                wait for 8 * C_CLK_PERIOD;
            end loop;

            report "*** SCENARIO 3 (max_hits_cfg sweep 0..7) PASS ***" severity note;

            wait for 10 * C_CLK_PERIOD;
            std.env.finish;
        end if;

        -- =====================================================================
        -- 2. Assert face_start_gated for 1 clock
        -- =====================================================================
        face_start_gated <= '1';
        wait until rising_edge(clk);
        face_start_gated <= '0';

        -- Small gap before shot_start
        wait for 3 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- 3. Assert shot_start_gated for 1 clock
        -- =====================================================================
        shot_start_gated <= '1';
        wait until rising_edge(clk);
        shot_start_gated <= '0';

        -- Wait a few clocks for face_assembler to be ready
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- 4. Feed cell data on chip 0 rising input
        --    2 stops x runtime beats_per_cell (incrementing pattern)
        -- =====================================================================
        for beat_idx in 0 to C_TOTAL_BEATS - 1 loop
            -- Wait for tready before driving
            while cell_rise_tready(0) = '0' loop
                wait until rising_edge(clk);
            end loop;

            v_beat_data := to_unsigned(beat_idx + 1, C_OUTPUT_WIDTH);
            if (beat_idx mod C_BEATS_PER_CELL) = (C_BEATS_PER_CELL - 1) then
                v_beat_data(6 downto 0) := to_unsigned(85, 7);
            end if;
            cell_rise_tdata_0 <= std_logic_vector(v_beat_data);
            cell_rise_tvalid  <= "0001";    -- only chip 0

            -- tlast ONLY on the very last beat of the entire chip slice
            -- (face_assembler expects tlast at chip slice boundary, not per-cell)
            if beat_idx = C_TOTAL_BEATS - 1 then
                cell_rise_tlast <= "0001";
                cell_rise_tuser <= "0001";
            else
                cell_rise_tlast <= "0000";
                cell_rise_tuser <= "0000";
            end if;

            wait until rising_edge(clk);
        end loop;

        -- De-assert after last beat
        cell_rise_tvalid <= "0000";
        cell_rise_tlast  <= "0000";
        cell_rise_tuser  <= "0000";

        -- =====================================================================
        -- 5. Wait for frame_done or watchdog
        -- =====================================================================
        wait until frame_done = '1' for C_WATCHDOG;

        -- =====================================================================
        -- 6. Report results
        -- =====================================================================
        report "=== TB Results ===" severity note;
        report "  Output beats: " & integer'image(v_out_beat_cnt) severity note;
        report "  SOF seen:     " & boolean'image(v_sof_seen)     severity note;
        report "  EOL seen:     " & boolean'image(v_eol_seen)     severity note;
        report "  frame_done:   " & std_logic'image(frame_done)   severity note;
        report "  SOF count:    " & integer'image(s_sof_count)    severity note;
        report "  row_faulted_rise count: " & integer'image(s_row_faulted_rise_cnt) severity note;
        report "  metadata Hit[16] preserve ok: " & boolean'image(s_metadata_hit_msb_ok) severity note;
        report "  metadata seen count:  " & integer'image(s_metadata_seen_count) severity note;

        if frame_done = '1' and v_sof_seen and v_eol_seen and v_out_beat_cnt > 0
           and s_keep_ok
           and s_metadata_hit_msb_ok
           and s_metadata_seen_count = C_STOPS
           and s_sof_count = 1
           and s_row_faulted_rise_cnt = 1
           and s_row_faulted_fall_cnt = 0
           and s_frame_faulted_rise_cnt = 0
           and s_frame_faulted_fall_cnt = 0 then
            report "*** SCENARIO 1 (rise-only smoke) PASS ***" severity note;
        else
            report "*** SCENARIO 1 FAIL ***" severity failure;
        end if;

        -- Isolate scenario 2 from the rise-only smoke setup. shot_start is
        -- common to both slopes, so the fall assembler would otherwise keep
        -- waiting for scenario-1 fall data and reinterpret the next shot_start
        -- as an overrun/blank-fill row.
        pipeline_abort_fall <= '1';
        wait until rising_edge(clk);
        pipeline_abort_fall <= '0';
        wait until face_asm_fall_idle = '1' and hdr_fall_idle = '1' for C_WATCHDOG;
        assert face_asm_fall_idle = '1' and hdr_fall_idle = '1'
            report "SCENARIO 1 cleanup: fall pipeline did not return idle after abort"
            severity failure;

        -- =====================================================================
        -- SCENARIO 2: slope-independent abort (#22)
        -- Feed rise AND fall data in parallel. Mid-way pulse
        -- pipeline_abort_fall for 1 cycle. Verify: rise completes (new
        -- frame_done), fall output count stays below the rise count (fall
        -- was aborted mid-flight but rise stream stayed alive).
        -- =====================================================================
        wait for 10 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- Snapshot baseline so we can check per-scenario deltas.
        report "===== SCENARIO 2: slope-independent fall-only abort =====" severity note;

        -- Declare local snapshot variables (re-use the report loop later).
        -- NOTE: variables must be declared at the top of the process; we use
        -- the already-declared v_beat_data as an integer-typed snapshot.

        -- face_start + shot_start for scenario 2
        face_start_gated <= '1';
        wait until rising_edge(clk);
        face_start_gated <= '0';
        wait for 3 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        shot_start_gated <= '1';
        wait until rising_edge(clk);
        shot_start_gated <= '0';
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);

        -- Parallel feed: drive BOTH rise and fall chip 0 cell data.
        for beat_idx in 0 to C_TOTAL_BEATS - 1 loop
            while cell_rise_tready(0) = '0' or cell_fall_tready(0) = '0' loop
                wait until rising_edge(clk);
            end loop;

            v_beat_data := to_unsigned(beat_idx + 1, C_OUTPUT_WIDTH);
            if (beat_idx mod C_BEATS_PER_CELL) = (C_BEATS_PER_CELL - 1) then
                v_beat_data(6 downto 0) := to_unsigned(85, 7);
            end if;
            cell_rise_tdata_0 <= std_logic_vector(v_beat_data);
            cell_fall_tdata_0 <= std_logic_vector(v_beat_data);
            cell_rise_tvalid  <= "0001";
            cell_fall_tvalid  <= "0001";

            if beat_idx = C_TOTAL_BEATS - 1 then
                cell_rise_tlast <= "0001";
                cell_fall_tlast <= "0001";
                cell_rise_tuser <= "0000";
                cell_fall_tuser <= "0000";
            else
                cell_rise_tlast <= "0000";
                cell_fall_tlast <= "0000";
                cell_rise_tuser <= "0000";
                cell_fall_tuser <= "0000";
            end if;

            -- Mid-flight fall abort (after a few real beats, before tlast)
            if beat_idx = C_TOTAL_BEATS / 2 then
                pipeline_abort_fall <= '1';
            elsif beat_idx = C_TOTAL_BEATS / 2 + 1 then
                pipeline_abort_fall <= '0';
            end if;

            wait until rising_edge(clk);
        end loop;

        cell_rise_tvalid <= "0000";
        cell_fall_tvalid <= "0000";
        cell_rise_tlast  <= "0000";
        cell_fall_tlast  <= "0000";
        cell_rise_tuser  <= "0000";
        cell_fall_tuser  <= "0000";
        pipeline_abort_fall <= '0';

        -- Allow rise pipeline to reach its frame_done
        wait until frame_done = '1' for C_WATCHDOG;
        wait for 10 * C_CLK_PERIOD;

        -- Scenario 2 checks:
        --   v_out_beat_cnt (rise) grew further than v_out_fall_beat_cnt
        --   because rise completed but fall was aborted mid-flight.
        report "=== SCENARIO 2 Results ===" severity note;
        report "  Rise beats: " & integer'image(v_out_beat_cnt)      severity note;
        report "  Fall beats: " & integer'image(v_out_fall_beat_cnt) severity note;
        report "  frame_done: " & std_logic'image(frame_done)        severity note;
        report "  SOF count:  " & integer'image(s_sof_count)         severity note;

        assert v_out_fall_beat_cnt < v_out_beat_cnt
            report "SCENARIO 2: fall beat count should be lower than rise (fall abort not effective)"
            severity failure;
        -- Rise frame_done must have pulsed TWICE (once per scenario).
        -- Fall frame_done must stay at scenario-1 level (fall aborted, no pulse).
        assert v_frame_done_cnt = 2
            report "SCENARIO 2: rise frame_done count = "
                   & integer'image(v_frame_done_cnt) & " (expected 2)"
            severity failure;
        assert v_frame_fall_done_cnt = 0
            report "SCENARIO 2: fall frame_done count = "
                   & integer'image(v_frame_fall_done_cnt)
                   & " (fall should not complete after abort)"
            severity failure;
        assert s_sof_count = 2
            report "SCENARIO 2: SOF count = "
                   & integer'image(s_sof_count) & " (expected 2)"
            severity failure;
        assert s_row_faulted_rise_cnt = 1
            report "SCENARIO 2: clean second rise row should not add row_done_faulted"
            severity failure;
        assert s_row_faulted_fall_cnt = 0
            report "SCENARIO 2: fall abort should not report row_done_faulted"
            severity failure;
        assert s_frame_faulted_rise_cnt = 0 and s_frame_faulted_fall_cnt = 0
            report "SCENARIO 2: frame_done_faulted should remain reserved for header drain timeout"
            severity failure;
        assert s_keep_ok and s_fall_keep_ok
            report "SCENARIO 2: AXIS tkeep/tstrb must remain all-ones on accepted beats"
            severity failure;
        assert s_metadata_hit_msb_ok
            report "SCENARIO 2: C04 final VDMA metadata[6:0] must preserve Hit[16] vector"
            severity failure;

        report "*** SCENARIO 2 (slope-independent abort) PASS ***" severity note;

        wait for 10 * C_CLK_PERIOD;
        std.env.finish;
    end process p_stim;

    -- =========================================================================
    -- Watchdog timeout
    -- =========================================================================
    p_watchdog : process
    begin
        wait for C_WATCHDOG;
        report "WATCHDOG TIMEOUT (50 us) - aborting simulation" severity failure;
        std.env.finish;
    end process p_watchdog;

end architecture sim;
