-- =============================================================================
-- tdc_gpx_cell_builder.vhd
-- TDC-GPX Controller - Cell Builder + Chip Slice Output (Ping-Pong)
-- =============================================================================
--
-- Purpose:
--   Converts sparse raw_event AXI-Stream into dense cell array, then serializes
--   cells to AXI-Stream as a chip slice (stops_per_chip cells, ascending order).
--
-- Architecture:
--   Ping-pong dual buffer with two independent processes:
--
--   p_collect (ST_C_IDLE / ST_C_ACTIVE):
--     Writes hits into the WRITE buffer from AXI-Stream slave input.
--     On drain_done (tuser[7] control beat): swaps buffer roles and signals
--     p_output to begin serialization.  Immediately ready for next shot.
--
--   p_output (ST_O_IDLE / ST_O_LOAD / ST_O_ACTIVE):
--     Reads from the READ buffer and serializes beats to AXI-Stream master.
--     Pipeline: cell MUX (ST_O_LOAD, 1-clk bubble) → beat MUX (ST_O_ACTIVE).
--
--   Dual-buffer benefit: collect(shot N+1) overlaps with output(shot N),
--   reducing effective per-shot latency to max(drain_time, output_time).
--
-- Overrun:
--   If p_output is still active when the next drain_done arrives,
--   the swap is skipped (data in write buffer will be overwritten by next shot).
--
-- AXI-Stream slave input (from raw_event_builder / slope demux):
--   tdata[16:0]  = raw_hit (17-bit, lower 16 stored as HIT_SLOT_DATA_WIDTH)
--   tuser[0]     = slope
--   tuser[5:3]   = stop_id_local (0..7)
--   tuser[7]     = drain_done (control beat: triggers output phase)
--
-- AXI-Stream master output (to face_assembler):
--   tdata         = g_TDATA_WIDTH bits (32 or 64)
--   tlast         = last beat of chip slice
--
-- Beat layout (auto-calculated from MAX_HITS / g_TDATA_WIDTH):
--   Beats 0..HIT_DATA_BEATS-1: hit_slot pairs (SLOTS_PER_BEAT per beat)
--   Beat META_BEAT_IDX:        metadata (hit_valid, slope_vec, hit_count, flags)
--   Remaining beats:           padding (zeros)
--   32-bit: 8 beats/cell, 64-bit: 4 beats/cell.
--
-- Signal ownership (no multi-driver):
--   p_collect WRITES: s_cell_buf_r, s_wr_buf_r, s_cstate_r, s_output_req_r
--   p_output  WRITES: s_cell_sel_r, s_ostate_r, s_tdata_r, s_tvalid_r, etc.
--   p_output  READS:  s_cell_buf_r (no write conflict)
--
-- All outputs are registered (module boundary = FF).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cell_builder is
    generic (
        g_CHIP_ID           : natural range 0 to 3 := 0;
        g_TDATA_WIDTH       : natural := c_TDATA_WIDTH    -- 32 or 64
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- AXI-Stream slave: raw event input (from raw_event_builder / slope demux)
        --   tdata[16:0]  = raw_hit (17-bit, lower 16 stored)
        --   tdata[31:17] = 0
        --   tuser[0]     = slope
        --   tuser[2:1]   = chip_id
        --   tuser[5:3]   = stop_id_local (0..7)
        --   tuser[6]     = ififo_id
        --   tuser[7]     = drain_done (control beat: begin output phase)
        --   tuser[10:8]  = hit_seq_local (0..7)
        --   tuser[15:11] = 0
        i_s_axis_tvalid     : in  std_logic;
        i_s_axis_tdata      : in  std_logic_vector(31 downto 0);
        i_s_axis_tuser      : in  std_logic_vector(15 downto 0);
        o_s_axis_tready     : out std_logic;

        -- Control (from chip_ctrl)
        i_shot_start        : in  std_logic;   -- new shot: clear cell buffers
        -- drain_done is received via i_s_axis_tuser(7) control beat

        -- Configuration (latched at packet_start)
        i_stops_per_chip    : in  unsigned(3 downto 0);

        -- AXI-Stream master (chip slice output)
        o_m_axis_tdata      : out std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid     : out std_logic;
        o_m_axis_tlast      : out std_logic;
        i_m_axis_tready     : in  std_logic;

        -- Status
        o_slice_done        : out std_logic;    -- 1-clk pulse: chip slice complete
        o_hit_dropped_any   : out std_logic     -- 1-clk pulse: any cell overflow
    );
end entity tdc_gpx_cell_builder;

architecture rtl of tdc_gpx_cell_builder is

    -- =========================================================================
    -- Ping-pong dual cell buffer (register-based, 2 × MAX_STOPS entries)
    -- p_collect writes to buffer[wr_buf], p_output reads from buffer[rd_buf].
    -- =========================================================================
    type t_cell_array is array (0 to c_MAX_STOPS_PER_CHIP - 1) of t_cell;
    type t_dual_cell_buf is array (0 to 1) of t_cell_array;
    signal s_cell_buf_r : t_dual_cell_buf := (others => (others => c_CELL_INIT));

    function fn_buf_idx(sel : std_logic) return natural is
    begin
        if sel = '1' then return 1; else return 0; end if;
    end function;

    -- =========================================================================
    -- Collect FSM (p_collect)
    -- =========================================================================
    type t_collect_state is (ST_C_IDLE, ST_C_ACTIVE);
    signal s_cstate_r     : t_collect_state := ST_C_IDLE;
    signal s_wr_buf_r     : std_logic := '0';       -- which buffer is write target

    -- Handshake: p_collect → p_output
    signal s_output_req_r : std_logic := '0';        -- 1-clk: start output (ififo1_done or full)
    signal s_output_full_r : std_logic := '0';       -- 1-clk: all IFIFOs done (output stops 4~7)
    signal s_rd_buf_idx_r : std_logic := '0';        -- which buffer p_output should read

    -- =========================================================================
    -- Output FSM (p_output)
    -- =========================================================================
    type t_output_state is (ST_O_IDLE, ST_O_LOAD, ST_O_ACTIVE, ST_O_WAIT_IFIFO2);
    signal s_ostate_r     : t_output_state := ST_O_IDLE;
    signal s_rd_buf_r     : std_logic := '0';       -- latched read buffer index

    -- Pipeline register: selected cell for output serialization
    signal s_cell_sel_r  : t_cell := c_CELL_INIT;

    -- Output serializer counters
    signal s_stop_idx_r  : unsigned(2 downto 0) := (others => '0');
    signal s_beat_idx_r  : unsigned(2 downto 0) := (others => '0');
    signal s_last_stop_r : unsigned(2 downto 0) := (others => '0');

    -- Generic-derived beat layout constants
    constant c_G_SLOTS_PER_BEAT  : natural := fn_slots_per_beat(g_TDATA_WIDTH);
    constant c_G_HIT_DATA_BEATS  : natural := fn_hit_data_beats(g_TDATA_WIDTH);
    constant c_G_META_BEAT_IDX   : natural := fn_meta_beat_idx(g_TDATA_WIDTH);
    constant c_G_BEATS_PER_CELL  : natural := fn_beats_per_cell(g_TDATA_WIDTH);
    constant c_LAST_BEAT : unsigned(2 downto 0) := to_unsigned(c_G_BEATS_PER_CELL - 1, 3);

    -- Registered outputs
    signal s_tdata_r     : std_logic_vector(g_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid_r    : std_logic := '0';
    signal s_tlast_r     : std_logic := '0';
    signal s_slice_done_r    : std_logic := '0';
    signal s_hit_dropped_r   : std_logic := '0';

    -- =========================================================================
    -- Cell-to-beat mux (combinational, used inside p_output)
    -- Auto-calculated from c_MAX_HITS_PER_STOP / g_TDATA_WIDTH.
    --
    -- 32-bit mode (8 beats/cell):
    --   Beat 0-3: hit_slot pairs (2 slots/beat)
    --   Beat 4:   metadata (hit_valid, slope_vec, hit_count, flags)
    --   Beat 5-7: padding (zeros)
    --
    -- 64-bit mode (4 beats/cell):
    --   Beat 0-1: hit_slot quads (4 slots/beat)
    --   Beat 2:   metadata in lower 32b, upper 32b = 0
    --   Beat 3:   padding (zeros)
    -- =========================================================================
    function fn_cell_beat(
        cell     : t_cell;
        beat_idx : unsigned(2 downto 0)
    ) return std_logic_vector is
        variable v_result   : std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        variable v_beat     : natural range 0 to 7;
        variable v_slot_idx : natural;
        variable v_lo       : natural;
    begin
        v_beat   := to_integer(beat_idx);
        v_result := (others => '0');

        if v_beat < c_G_HIT_DATA_BEATS then
            -- Hit-data beat: pack c_G_SLOTS_PER_BEAT slots per beat
            for sl in 0 to c_G_SLOTS_PER_BEAT - 1 loop
                v_slot_idx := v_beat * c_G_SLOTS_PER_BEAT + sl;
                if v_slot_idx < c_MAX_HITS_PER_STOP then
                    v_lo := sl * c_HIT_SLOT_DATA_WIDTH;
                    v_result(v_lo + c_HIT_SLOT_DATA_WIDTH - 1 downto v_lo)
                        := std_logic_vector(cell.hit_slot(v_slot_idx));
                end if;
            end loop;

        elsif v_beat = c_G_META_BEAT_IDX then
            -- Metadata beat: hit_valid | slope_vec | hit_count | flags | chip_id
            v_result(31 downto 32 - c_MAX_HITS_PER_STOP)
                := cell.hit_valid;
            v_result(31 - c_MAX_HITS_PER_STOP downto 32 - 2*c_MAX_HITS_PER_STOP)
                := cell.slope_vec;
            v_result(15 downto 12) := std_logic_vector(cell.hit_count_actual);
            v_result(11)           := cell.hit_dropped;
            v_result(10)           := cell.error_fill;
            v_result(9 downto 8)   := std_logic_vector(to_unsigned(g_CHIP_ID, 2));
            v_result(7 downto 0)   := (others => '0');

        else
            -- Padding beats: zeros
            v_result := (others => '0');
        end if;

        return v_result;
    end function;

begin

    -- AXI-Stream slave: always accept during collect phase (no backpressure)
    o_s_axis_tready <= '1' when s_cstate_r = ST_C_ACTIVE else '0';

    -- =========================================================================
    -- p_collect: write hits into write-buffer, handle drain_done swap
    -- Owns: s_cell_buf_r, s_wr_buf_r, s_cstate_r, s_output_req_r, s_rd_buf_idx_r
    -- =========================================================================
    p_collect : process(i_clk)
        variable v_wr   : natural range 0 to 1;
        variable v_stop : natural range 0 to c_MAX_STOPS_PER_CHIP - 1;
        variable v_seq  : natural range 0 to c_MAX_HITS_PER_STOP - 1;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_cell_buf_r   <= (others => (others => c_CELL_INIT));
                s_cstate_r     <= ST_C_IDLE;
                s_wr_buf_r     <= '0';
                s_output_req_r  <= '0';
                s_output_full_r <= '0';
                s_rd_buf_idx_r  <= '0';
                s_hit_dropped_r <= '0';
            else
                -- Default: clear single-cycle pulses
                s_output_req_r  <= '0';
                s_output_full_r <= '0';
                s_hit_dropped_r <= '0';

                v_wr := fn_buf_idx(s_wr_buf_r);

                case s_cstate_r is

                    when ST_C_IDLE =>
                        if i_shot_start = '1' then
                            s_cell_buf_r(v_wr) <= (others => c_CELL_INIT);
                            s_cstate_r         <= ST_C_ACTIVE;
                        end if;

                    when ST_C_ACTIVE =>
                        -- Raw event write (exclude drain_done control beat)
                        if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '0' then
                            v_stop := to_integer(unsigned(i_s_axis_tuser(5 downto 3)));

                            if s_cell_buf_r(v_wr)(v_stop).hit_count_actual < c_MAX_HITS_PER_STOP then
                                v_seq := to_integer(s_cell_buf_r(v_wr)(v_stop).hit_count_actual(2 downto 0));
                                s_cell_buf_r(v_wr)(v_stop).hit_slot(v_seq)  <= unsigned(i_s_axis_tdata(c_HIT_SLOT_DATA_WIDTH - 1 downto 0));
                                s_cell_buf_r(v_wr)(v_stop).hit_valid(v_seq) <= '1';
                                s_cell_buf_r(v_wr)(v_stop).slope_vec(v_seq) <= i_s_axis_tuser(0);
                                s_cell_buf_r(v_wr)(v_stop).hit_count_actual <= s_cell_buf_r(v_wr)(v_stop).hit_count_actual + 1;
                            else
                                s_cell_buf_r(v_wr)(v_stop).hit_dropped <= '1';
                                s_hit_dropped_r <= '1';
                            end if;
                        end if;

                        -- drain_done control beats:
                        --   tuser[7]=1, tuser[6]=0: ififo1_done → start output stops 0~3
                        --   tuser[7]=1, tuser[6]=1: final done  → output stops 4~7 + swap
                        if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '1' then
                            if i_s_axis_tuser(6) = '0' then
                                -- IFIFO1 done: trigger early output of stops 0~3
                                if s_ostate_r = ST_O_IDLE then
                                    s_rd_buf_idx_r <= s_wr_buf_r;
                                    s_output_req_r <= '1';
                                end if;
                            else
                                -- Final drain_done: trigger stops 4~7, swap buffer
                                s_output_full_r <= '1';
                                if s_ostate_r = ST_O_IDLE and s_output_req_r = '0' then
                                    -- No ififo1_done was sent (both done simultaneously)
                                    s_rd_buf_idx_r <= s_wr_buf_r;
                                    s_output_req_r <= '1';
                                end if;
                                s_wr_buf_r <= not s_wr_buf_r;
                                s_cell_buf_r(fn_buf_idx(not s_wr_buf_r)) <= (others => c_CELL_INIT);
                            end if;
                        end if;

                        -- shot_start override: clear write buffer, stay active
                        if i_shot_start = '1' then
                            s_cell_buf_r(v_wr) <= (others => c_CELL_INIT);
                        end if;

                end case;
            end if;
        end if;
    end process p_collect;

    -- =========================================================================
    -- p_output: serialize read-buffer cells to AXI-Stream master
    -- Owns: s_ostate_r, s_rd_buf_r, s_cell_sel_r, s_stop_idx_r, s_beat_idx_r,
    --       s_last_stop_r, s_tdata_r, s_tvalid_r, s_tlast_r, s_slice_done_r
    -- Reads (no write): s_cell_buf_r, s_output_req_r, s_rd_buf_idx_r
    -- =========================================================================
    p_output : process(i_clk)
        variable v_rd       : natural range 0 to 1;
        variable v_nxt_stop : unsigned(2 downto 0);
        variable v_nxt_beat : unsigned(2 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_ostate_r    <= ST_O_IDLE;
                s_rd_buf_r    <= '0';
                s_cell_sel_r  <= c_CELL_INIT;
                s_stop_idx_r  <= (others => '0');
                s_beat_idx_r  <= (others => '0');
                s_last_stop_r <= (others => '0');
                s_tdata_r     <= (others => '0');
                s_tvalid_r    <= '0';
                s_tlast_r     <= '0';
                s_slice_done_r <= '0';
            else
                s_slice_done_r <= '0';

                case s_ostate_r is

                    when ST_O_IDLE =>
                        s_tvalid_r <= '0';
                        s_tlast_r  <= '0';

                        if s_output_req_r = '1' then
                            -- Latch read buffer, load first cell
                            s_rd_buf_r    <= s_rd_buf_idx_r;
                            v_rd          := fn_buf_idx(s_rd_buf_idx_r);
                            s_cell_sel_r  <= s_cell_buf_r(v_rd)(0);
                            s_stop_idx_r  <= (others => '0');
                            s_beat_idx_r  <= (others => '0');
                            s_last_stop_r <= i_stops_per_chip(2 downto 0) - 1;
                            s_ostate_r    <= ST_O_LOAD;
                        end if;

                    when ST_O_LOAD =>
                        -- Beat MUX from pipeline register (1-clk bubble)
                        s_tdata_r  <= fn_cell_beat(s_cell_sel_r, s_beat_idx_r);
                        s_tvalid_r <= '1';
                        if s_stop_idx_r = s_last_stop_r
                           and s_beat_idx_r = c_LAST_BEAT then
                            s_tlast_r <= '1';
                        else
                            s_tlast_r <= '0';
                        end if;
                        s_ostate_r <= ST_O_ACTIVE;

                    when ST_O_ACTIVE =>
                        if s_tvalid_r = '1' and i_m_axis_tready = '1' then
                            v_rd := fn_buf_idx(s_rd_buf_r);

                            if s_stop_idx_r = s_last_stop_r
                               and s_beat_idx_r = c_LAST_BEAT then
                                -- Chip slice complete
                                s_tvalid_r     <= '0';
                                s_tlast_r      <= '0';
                                s_slice_done_r <= '1';
                                s_ostate_r     <= ST_O_IDLE;

                            elsif s_beat_idx_r = c_LAST_BEAT then
                                -- Cell boundary: check IFIFO1/2 split point (stop 3→4)
                                v_nxt_stop := s_stop_idx_r + 1;
                                if s_stop_idx_r = "011" and s_output_full_r = '0' then
                                    -- IFIFO1 stops done, IFIFO2 not ready: wait
                                    s_tvalid_r <= '0';
                                    s_tlast_r  <= '0';
                                    s_ostate_r <= ST_O_WAIT_IFIFO2;
                                else
                                    -- Normal cell boundary: load next cell
                                    s_stop_idx_r <= v_nxt_stop;
                                    s_beat_idx_r <= (others => '0');
                                    s_cell_sel_r <= s_cell_buf_r(v_rd)(to_integer(v_nxt_stop));
                                    s_tvalid_r   <= '0';
                                    s_tlast_r    <= '0';
                                    s_ostate_r   <= ST_O_LOAD;
                                end if;

                            else
                                -- Same cell: beat MUX
                                v_nxt_beat   := s_beat_idx_r + 1;
                                s_beat_idx_r <= v_nxt_beat;
                                s_tdata_r    <= fn_cell_beat(s_cell_sel_r, v_nxt_beat);
                                if s_stop_idx_r = s_last_stop_r
                                   and v_nxt_beat = c_LAST_BEAT then
                                    s_tlast_r <= '1';
                                else
                                    s_tlast_r <= '0';
                                end if;
                            end if;
                        end if;

                    when ST_O_WAIT_IFIFO2 =>
                        -- Stall: stops 0~3 output done, waiting for IFIFO2 data.
                        -- Resume when s_output_full_r is asserted by p_collect.
                        if s_output_full_r = '1' then
                            v_rd := fn_buf_idx(s_rd_buf_r);
                            s_stop_idx_r <= "100";    -- stop 4
                            s_beat_idx_r <= (others => '0');
                            s_cell_sel_r <= s_cell_buf_r(v_rd)(4);
                            s_ostate_r   <= ST_O_LOAD;
                        end if;

                end case;

                -- shot_start abort: cancel output, return to idle
                if i_shot_start = '1' and s_ostate_r /= ST_O_IDLE then
                    s_tvalid_r <= '0';
                    s_tlast_r  <= '0';
                    s_ostate_r <= ST_O_IDLE;
                end if;
            end if;
        end if;
    end process p_output;

    -- =========================================================================
    -- Output assignments (all registered)
    -- =========================================================================
    o_m_axis_tdata    <= s_tdata_r;
    o_m_axis_tvalid   <= s_tvalid_r;
    o_m_axis_tlast    <= s_tlast_r;
    o_slice_done      <= s_slice_done_r;
    o_hit_dropped_any <= s_hit_dropped_r;

end architecture rtl;
