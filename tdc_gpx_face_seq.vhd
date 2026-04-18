-- =============================================================================
-- tdc_gpx_face_seq.vhd
-- TDC-GPX Controller - Face Sequencer
-- =============================================================================
--
-- Purpose:
--   Manages face/shot/frame sequencing extracted from tdc_gpx_top.
--   Controls cmd_start acceptance, shot gating, face/frame ID tracking,
--   config snapshot at face boundaries, and geometry calculation.
--
-- Extracted processes:
--   p_face_seq, p_global_shot_seq, p_frame_abort_cnt, p_face_shot_cnt,
--   p_geometry, p_face_cfg_latch, p_frame_done_both
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_face_seq is
    generic (
        g_OUTPUT_WIDTH : natural := 32  -- 32 or 64 (for hsize/geometry calculation)
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- Commands
        i_cmd_start          : in  std_logic;
        i_cmd_stop           : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;

        -- Pipeline idle indicators (for start acceptance)
        i_chip_busy          : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_reg_outstanding    : in  std_logic;
        i_face_asm_idle      : in  std_logic;
        i_face_asm_fall_idle : in  std_logic;
        i_hdr_idle           : in  std_logic;
        i_hdr_fall_idle      : in  std_logic;
        i_face_tvalid        : in  std_logic;
        i_face_fall_tvalid   : in  std_logic;
        i_face_buf_tvalid    : in  std_logic;
        i_face_fall_buf_tvalid : in  std_logic;
        i_m_axis_tvalid      : in  std_logic;
        i_m_axis_fall_tvalid : in  std_logic;

        -- Shot/frame events
        i_shot_start_raw     : in  std_logic;   -- from laser_ctrl
        i_frame_done         : in  std_logic;   -- rising pipeline
        i_frame_fall_done    : in  std_logic;
        i_face_abort         : in  std_logic;   -- assembler overrun (rise)
        i_face_fall_abort    : in  std_logic;   -- assembler overrun (fall)
        i_shot_overrun       : in  std_logic;
        i_shot_fall_overrun  : in  std_logic;
        i_hdr_draining       : in  std_logic;
        i_hdr_fall_draining  : in  std_logic;

        -- Configuration
        i_cfg                : in  t_tdc_cfg;

        -- Outputs to top routing
        o_cmd_start_accepted : out std_logic;
        o_face_state_idle    : out std_logic;
        o_packet_start       : out std_logic;
        o_face_start         : out std_logic;   -- 1-clk delayed packet_start
        o_face_start_gated   : out std_logic;   -- stop/reset gated face_start
        o_shot_start_gated   : out std_logic;
        o_face_closing       : out std_logic;
        o_pipeline_abort     : out std_logic;
        o_shot_drop_cnt      : out unsigned(15 downto 0);
        o_cfg_rejected       : out std_logic;  -- 1-clk pulse: cmd_start rejected due to invalid config
        o_shot_start_per_chip : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- ID outputs
        o_face_id            : out unsigned(7 downto 0);
        o_frame_id           : out unsigned(31 downto 0);
        o_global_shot_seq    : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        o_frame_abort_cnt    : out unsigned(15 downto 0);
        o_frame_done_both    : out std_logic;

        -- Config snapshots (latched at packet_start)
        o_face_active_mask   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_face_stops_per_chip : out unsigned(3 downto 0);
        o_face_cols_per_face : out unsigned(15 downto 0);
        o_face_n_faces       : out unsigned(3 downto 0);
        o_rows_per_face      : out unsigned(15 downto 0);
        o_hsize_bytes        : out unsigned(15 downto 0);
        o_cfg_face           : out t_tdc_cfg   -- full config snapshot for header
    );
end entity tdc_gpx_face_seq;

architecture rtl of tdc_gpx_face_seq is

    constant C_ZEROS_CHIPS : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    type t_face_state is (ST_IDLE, ST_WAIT_SHOT, ST_IN_FACE);
    signal s_face_state_r     : t_face_state := ST_IDLE;

    signal s_face_id_r        : unsigned(7 downto 0)  := (others => '0');
    signal s_frame_id_r       : unsigned(31 downto 0) := (others => '0');
    signal s_shot_overrun_r   : std_logic := '0';
    signal s_cmd_start_accepted_r : std_logic := '0';

    signal s_global_shot_seq_r : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');
    signal s_frame_abort_cnt_r : unsigned(15 downto 0) := (others => '0');
    signal s_face_shot_cnt_r   : unsigned(15 downto 0) := (others => '0');

    signal s_face_stops_per_chip_r : unsigned(3 downto 0)  := to_unsigned(8, 4);
    signal s_face_active_mask_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal s_face_cols_per_face_r  : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal s_face_n_faces_r        : unsigned(3 downto 0)  := to_unsigned(1, 4);
    signal s_cfg_face_r            : t_tdc_cfg := c_TDC_CFG_INIT;

    signal s_rows_per_face_r  : unsigned(15 downto 0) := to_unsigned(c_MAX_ROWS_PER_FACE, 16);
    signal s_hsize_bytes_r    : unsigned(15 downto 0) := (others => '0');  -- computed at packet_start

    signal s_frame_rise_done_r : std_logic := '0';
    signal s_frame_fall_done_r : std_logic := '0';
    signal s_frame_done_both   : std_logic;

    -- Shot deferral / face_start delay (absorbed from top p_face_start_delay)
    signal s_face_start_r      : std_logic := '0';
    signal s_face_active_r     : std_logic := '0';
    signal s_shot_pending_r    : std_logic := '0';
    signal s_shot_deferred_r   : std_logic := '0';
    signal s_shot_drop_cnt_r   : unsigned(15 downto 0) := (others => '0');
    signal s_cfg_rejected_r    : std_logic := '0';
    signal s_shot_start_gated  : std_logic;
    signal s_pipeline_abort    : std_logic;
    signal s_abort_quiesce_r   : std_logic := '0';  -- 1-cycle guard after abort

    -- Pending cmd_start: accepted when pipeline reports busy. Re-evaluated
    -- every cycle in ST_IDLE. Cleared on cmd_stop or abort (SW must re-arm
    -- after abort). Prevents silent loss of 1-cycle cmd_start pulses.
    signal s_cmd_start_pending_r : std_logic := '0';
    signal s_all_shots_fired   : std_logic;
    signal s_face_closing      : std_logic;

    signal s_packet_start      : std_logic := '0';

begin

    -- =========================================================================
    -- p_face_seq: face FSM
    -- =========================================================================
    p_face_seq : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_cmd_soft_reset = '1' then
                s_face_state_r        <= ST_IDLE;
                s_face_id_r           <= (others => '0');
                s_frame_id_r          <= (others => '0');
                s_shot_overrun_r      <= '0';
                s_cmd_start_accepted_r <= '0';
                s_abort_quiesce_r     <= '0';
                s_cmd_start_pending_r <= '0';
            else
                s_cmd_start_accepted_r <= '0';
                s_cfg_rejected_r       <= '0';
                -- 1-cycle quiesce guard: latch abort, clear next cycle
                s_abort_quiesce_r <= s_pipeline_abort;

                if i_shot_overrun = '1' or i_shot_fall_overrun = '1' then
                    s_shot_overrun_r <= '1';
                end if;

                case s_face_state_r is
                    when ST_IDLE =>
                        if i_cmd_start = '1' or s_cmd_start_pending_r = '1' then
                            -- Config validation: reject if geometry is degenerate
                            if i_cfg.active_chip_mask = "0000"
                               or i_cfg.stops_per_chip < 2
                               or i_cfg.cols_per_face < 1 then
                                -- Invalid config: reject start, pulse cfg_rejected
                                s_cfg_rejected_r      <= '1';
                                s_cmd_start_pending_r <= '0';  -- drop pending on config error
                            elsif i_chip_busy = C_ZEROS_CHIPS
                               and i_reg_outstanding = '0'
                               and i_face_asm_idle = '1'
                               and i_face_asm_fall_idle = '1'
                               and i_hdr_idle = '1'
                               and i_hdr_fall_idle = '1'
                               and i_face_tvalid = '0'
                               and i_face_fall_tvalid = '0'
                               and i_face_buf_tvalid = '0'
                               and i_face_fall_buf_tvalid = '0'
                               and i_m_axis_tvalid = '0'
                               and i_m_axis_fall_tvalid = '0' then
                                s_face_id_r            <= (others => '0');
                                s_shot_overrun_r       <= '0';
                                s_cmd_start_accepted_r <= '1';
                                s_cmd_start_pending_r  <= '0';  -- consume
                                s_face_state_r         <= ST_WAIT_SHOT;
                            else
                                -- Pipeline busy: latch pending so the next
                                -- IDLE cycle can retry without needing SW to
                                -- re-pulse cmd_start.
                                s_cmd_start_pending_r <= '1';
                                -- synthesis translate_off
                                assert false
                                    report "face_seq: cmd_start pending-latched (pipeline busy)"
                                    severity note;
                                -- synthesis translate_on
                            end if;
                        end if;
                        -- cmd_stop / abort cancels any queued start.
                        if i_cmd_stop = '1' or s_pipeline_abort = '1' then
                            s_cmd_start_pending_r <= '0';
                        end if;

                    when ST_WAIT_SHOT =>
                        if i_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_packet_start = '1' then
                            s_face_state_r <= ST_IN_FACE;
                        end if;

                    when ST_IN_FACE =>
                        if i_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_frame_done_both = '1' then
                            s_frame_id_r <= s_frame_id_r + 1;
                            if s_face_n_faces_r = 0 or s_face_id_r >= resize(s_face_n_faces_r, 8) - 1 then
                                s_face_id_r <= (others => '0');
                            else
                                s_face_id_r <= s_face_id_r + 1;
                            end if;
                            s_face_state_r <= ST_WAIT_SHOT;
                        end if;
                end case;
            end if;
        end if;
    end process p_face_seq;

    -- =========================================================================
    -- Counters
    -- =========================================================================
    p_global_shot_seq : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or s_cmd_start_accepted_r = '1' then
                s_global_shot_seq_r <= (others => '0');
            elsif o_shot_start_gated = '1' then
                s_global_shot_seq_r <= s_global_shot_seq_r + 1;
            end if;
        end if;
    end process;

    p_frame_abort_cnt : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or s_cmd_start_accepted_r = '1' then
                s_frame_abort_cnt_r <= (others => '0');
            elsif s_pipeline_abort = '1' and s_face_state_r = ST_IN_FACE then
                s_frame_abort_cnt_r <= s_frame_abort_cnt_r + 1;
            end if;
        end if;
    end process;

    p_face_shot_cnt : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or s_packet_start = '1'
               or s_frame_done_both = '1' then
                s_face_shot_cnt_r <= (others => '0');
            elsif o_shot_start_gated = '1' then
                s_face_shot_cnt_r <= s_face_shot_cnt_r + 1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Geometry & config snapshot
    -- =========================================================================
    p_geometry : process(i_clk)
        variable v_active_cnt : natural range 0 to c_N_CHIPS;
        variable v_rows       : natural range 0 to c_MAX_ROWS_PER_FACE;
        variable v_data_beats : natural range 0 to c_DATA_BEATS_MAX;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_rows_per_face_r <= to_unsigned(c_MAX_ROWS_PER_FACE, 16);
                s_hsize_bytes_r   <= (others => '0');
            elsif s_packet_start = '1' then
                v_active_cnt := fn_count_ones(i_cfg.active_chip_mask);
                v_rows := v_active_cnt * to_integer(i_cfg.stops_per_chip);
                if v_rows < 2 then v_rows := 2; end if;
                s_rows_per_face_r <= to_unsigned(v_rows, 16);
                -- Runtime beats_per_cell lookup (same pattern as cell_builder)
                case i_cfg.max_hits_cfg is
                    when "001" => v_data_beats := v_rows * fn_beats_per_cell_rt(1, g_OUTPUT_WIDTH);
                    when "010" => v_data_beats := v_rows * fn_beats_per_cell_rt(2, g_OUTPUT_WIDTH);
                    when "011" => v_data_beats := v_rows * fn_beats_per_cell_rt(3, g_OUTPUT_WIDTH);
                    when "100" => v_data_beats := v_rows * fn_beats_per_cell_rt(4, g_OUTPUT_WIDTH);
                    when "101" => v_data_beats := v_rows * fn_beats_per_cell_rt(5, g_OUTPUT_WIDTH);
                    when "110" => v_data_beats := v_rows * fn_beats_per_cell_rt(6, g_OUTPUT_WIDTH);
                    when others => v_data_beats := v_rows * fn_beats_per_cell_rt(7, g_OUTPUT_WIDTH);
                end case;
                s_hsize_bytes_r <= to_unsigned(
                    (v_data_beats + fn_hdr_prefix_beats(g_OUTPUT_WIDTH))
                    * (g_OUTPUT_WIDTH / 8), 16);
            end if;
        end if;
    end process;

    p_face_cfg_latch : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_face_stops_per_chip_r <= to_unsigned(8, 4);
                s_face_active_mask_r    <= (others => '1');
                s_face_cols_per_face_r  <= to_unsigned(1, 16);
                s_face_n_faces_r        <= to_unsigned(1, 4);
                s_cfg_face_r            <= i_cfg;
            elsif s_packet_start = '1' then
                s_face_stops_per_chip_r <= i_cfg.stops_per_chip;
                s_face_active_mask_r    <= i_cfg.active_chip_mask;
                s_face_cols_per_face_r  <= i_cfg.cols_per_face;
                s_face_n_faces_r        <= resize(i_cfg.n_faces, 4);
                s_cfg_face_r            <= i_cfg;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Frame done combiner
    -- =========================================================================
    p_frame_done_both : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or s_packet_start = '1' then
                s_frame_rise_done_r <= '0';
                s_frame_fall_done_r <= '0';
            else
                if i_frame_done = '1' or s_pipeline_abort = '1' then
                    s_frame_rise_done_r <= '1';
                end if;
                if i_frame_fall_done = '1' or s_pipeline_abort = '1' then
                    s_frame_fall_done_r <= '1';
                end if;
            end if;
        end if;
    end process;

    s_frame_done_both <= '1' when (s_frame_rise_done_r = '1' or i_frame_done = '1' or s_pipeline_abort = '1')
                                  and (s_frame_fall_done_r = '1' or i_frame_fall_done = '1' or s_pipeline_abort = '1')
                         else '0';

    s_all_shots_fired <= '1' when s_face_shot_cnt_r >= s_face_cols_per_face_r
                                  and s_face_cols_per_face_r /= 0
                         else '0';

    s_face_closing <= '1' when s_all_shots_fired = '1'
                               or s_pipeline_abort = '1'
                               or (s_frame_rise_done_r = '1' or i_frame_done = '1'
                                   or i_hdr_draining = '1')
                               or (s_frame_fall_done_r = '1' or i_frame_fall_done = '1'
                                   or i_hdr_fall_draining = '1')
                     else '0';

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    o_cmd_start_accepted  <= s_cmd_start_accepted_r;
    o_face_state_idle     <= '1' when s_face_state_r = ST_IDLE else '0';
    o_packet_start        <= s_packet_start;
    o_face_closing        <= s_face_closing;
    o_face_id             <= s_face_id_r;
    o_frame_id            <= s_frame_id_r;
    o_global_shot_seq     <= s_global_shot_seq_r;
    o_frame_abort_cnt     <= s_frame_abort_cnt_r;
    o_frame_done_both     <= s_frame_done_both;
    o_face_active_mask    <= s_face_active_mask_r;
    o_face_stops_per_chip <= s_face_stops_per_chip_r;
    o_face_cols_per_face  <= s_face_cols_per_face_r;
    o_face_n_faces        <= s_face_n_faces_r;
    o_rows_per_face       <= s_rows_per_face_r;
    o_hsize_bytes         <= s_hsize_bytes_r;
    o_cfg_face            <= s_cfg_face_r;

    -- =========================================================================
    -- Packet start (combinational, from internal FSM state)
    -- NOTE: kept combinational for zero-latency face start. Assertions below
    -- guard against unexpected double-fire or illegal-state glitches.
    -- =========================================================================
    s_packet_start <= '1' when s_face_state_r = ST_WAIT_SHOT
                               and (i_shot_start_raw = '1' or s_shot_deferred_r = '1')
                               and i_cmd_stop = '0'
                               and i_cmd_soft_reset = '0'
                               and s_pipeline_abort = '0'
                               and s_abort_quiesce_r = '0'
                               and i_hdr_idle = '1'
                               and i_hdr_fall_idle = '1'
                     else '0';

    -- Pipeline abort
    s_pipeline_abort <= i_face_abort or i_face_fall_abort
                        or i_cmd_stop or i_cmd_soft_reset;

    -- =========================================================================
    -- Face start delay + shot deferral (absorbed from top p_face_start_delay)
    -- =========================================================================
    p_face_start_delay : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_cmd_soft_reset = '1' then
                s_face_start_r    <= '0';
                s_face_active_r   <= '0';
                s_shot_pending_r  <= '0';
                s_shot_deferred_r <= '0';
                s_shot_drop_cnt_r <= (others => '0');
            elsif i_cmd_stop = '1' then
                s_face_start_r    <= '0';
                s_face_active_r   <= '0';
                s_shot_pending_r  <= '0';
                s_shot_deferred_r <= '0';
            else
                s_face_start_r <= s_packet_start;

                -- synthesis translate_off
                assert not (s_packet_start = '1' and s_face_state_r /= ST_WAIT_SHOT)
                    report "face_seq: packet_start while not ST_WAIT_SHOT"
                    severity error;
                -- synthesis translate_on

                if s_packet_start = '1' then
                    s_face_active_r <= '1';
                elsif s_frame_done_both = '1' then
                    s_face_active_r <= '0';
                end if;

                -- Shot deferral
                if s_packet_start = '1' then
                    if s_shot_deferred_r = '1' and i_shot_start_raw = '1' then
                        s_shot_deferred_r <= '1';
                    else
                        s_shot_deferred_r <= '0';
                    end if;
                elsif i_shot_start_raw = '1' and s_shot_deferred_r = '0' then
                    if (s_face_state_r = ST_IN_FACE and s_face_closing = '1')
                       or (s_face_state_r = ST_WAIT_SHOT and s_packet_start = '0') then
                        s_shot_deferred_r <= '1';
                    end if;
                elsif i_shot_start_raw = '1' and s_shot_deferred_r = '1' then
                    s_shot_drop_cnt_r <= s_shot_drop_cnt_r + 1;
                end if;

                -- Auto-defer killed pending
                if s_shot_pending_r = '1' and s_shot_start_gated = '0'
                   and i_cmd_stop = '0' and i_cmd_soft_reset = '0'
                   and s_pipeline_abort = '0' then
                    if s_shot_deferred_r = '0' then
                        s_shot_deferred_r <= '1';
                    else
                        s_shot_drop_cnt_r <= s_shot_drop_cnt_r + 1;
                    end if;
                end if;

                -- Registered shot acceptance
                if (s_face_state_r = ST_WAIT_SHOT
                        and (i_shot_start_raw = '1' or s_shot_deferred_r = '1')
                        and i_cmd_stop = '0')
                   or (s_face_active_r = '1'
                        and s_frame_done_both = '0'
                        and s_face_closing = '0'
                        and i_cmd_stop = '0'
                        and i_shot_start_raw = '1') then
                    s_shot_pending_r <= '1';
                else
                    s_shot_pending_r <= '0';
                end if;
            end if;
        end if;
    end process;

    -- Shot gated with closing/abort kill
    s_shot_start_gated <= s_shot_pending_r
                          when s_face_closing = '0'
                               and i_cmd_stop = '0'
                               and i_cmd_soft_reset = '0'
                               and s_pipeline_abort = '0'
                               and s_abort_quiesce_r = '0'
                          else '0';

    -- Per-chip shot masking
    gen_shot_mask : for i in 0 to c_N_CHIPS - 1 generate
        o_shot_start_per_chip(i) <= s_shot_start_gated and s_face_active_mask_r(i);
    end generate;

    -- Output assignments
    o_face_start       <= s_face_start_r;
    o_face_start_gated <= s_face_start_r when i_cmd_stop = '0' and i_cmd_soft_reset = '0'
                                            and s_pipeline_abort = '0' and s_abort_quiesce_r = '0'
                          else '0';
    o_shot_start_gated <= s_shot_start_gated;
    o_pipeline_abort   <= s_pipeline_abort;
    o_shot_drop_cnt    <= s_shot_drop_cnt_r;
    o_cfg_rejected     <= s_cfg_rejected_r;

end architecture rtl;
