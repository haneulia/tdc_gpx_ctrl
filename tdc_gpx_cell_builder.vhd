-- =============================================================================
-- tdc_gpx_cell_builder.vhd
-- TDC-GPX Controller - Cell Builder + Chip Slice Output (Per-Buffer Owner FSM)
-- =============================================================================
--
-- Purpose:
--   Converts sparse raw_event AXI-Stream into dense cell array, then serializes
--   cells to AXI-Stream as a chip slice (stops_per_chip cells, ascending order).
--
-- Architecture:
--   Dual cell buffer with explicit per-buffer ownership FSM managed by p_collect.
--   Each buffer has an independent state:
--
--     BUF_FREE    - Nobody owns it. Available for next shot_start.
--     BUF_COLLECT - p_collect owns it. Writing hits from AXI-Stream.
--     BUF_SHARED  - p_collect may still write stops 4-7, p_output reads.
--                   Entered on ififo1_done. buf_full flag indicates stops 4-7 ready.
--
--   Two independent processes share the dual buffer:
--
--   p_collect (ST_C_IDLE / ST_C_ACTIVE):
--     Owns ALL buffer state transitions (no multi-driver on s_buf_state_r).
--     On shot_start: finds BUF_FREE buffer, transitions to BUF_COLLECT.
--     On ififo1_done: BUF_COLLECT -> BUF_SHARED, signals p_output to start.
--     On final_done: sets buf_full flag (stops 4-7 ready).
--     On s_output_done_r: BUF_SHARED -> BUF_FREE, auto-starts next if queued.
--     On i_abort: all buffers -> BUF_FREE, FSM -> ST_C_IDLE.
--
--   p_output (ST_O_IDLE / ST_O_LOAD / ST_O_ACTIVE / ST_O_WAIT_IFIFO2 /
--             ST_O_TIMEOUT_EOS):
--     Reads from the buffer indicated by s_rd_buf_idx_r.
--     Pipeline: cell MUX (ST_O_LOAD, 1-clk bubble) -> beat MUX (ST_O_ACTIVE).
--     Per-IFIFO early output: starts stops 0~3 on ififo1_done, stalls at
--     stop 3->4 boundary in ST_O_WAIT_IFIFO2 until buf_full flag is set.
--     Signals completion via s_output_done_r pulse.
--
--   ST_O_TIMEOUT_EOS (Round 1 #2):
--     Entered from ST_O_WAIT_IFIFO2 on the 65K-cycle watchdog expiring
--     (IFIFO2 data never arrived). Emits ONE synthetic final beat with
--     tvalid='1', tlast='1', zero data so the downstream face_assembler
--     can complete this chip slice via its tlast-based detection. The
--     o_slice_timeout sticky flags the truncation for SW visibility; SW
--     uses that + cell metadata error bits to identify corrupted slices.
--
-- Pulse-retention latches:
--   s_shot_pending_r (Round 1 #8/#9): latches i_shot_start when it
--     cannot be consumed this cycle (same-cycle drain_done in ACTIVE,
--     or arrival during ST_C_DROP). ST_C_IDLE's handler consumes it.
--     Cleared on i_abort.
--
-- No-free-buffer on shot_start (Round 2 #4):
--   From ST_C_IDLE, if both buffers are busy and a shot_start (or pending
--   replay) fires, the FSM enters ST_C_DROP so the incoming shot's beats
--   are actually absorbed (tready='1' in DROP) instead of stalling the
--   upstream raw_event stream. s_shot_dropped_r pulses for SW visibility.
--
--   Handshake signals (no multi-driver):
--     s_output_req_r  : p_collect -> p_output (1-clk pulse: start output)
--     s_rd_buf_idx_r  : p_collect -> p_output (which buffer to read)
--     s_output_done_r : p_output -> p_collect (1-clk pulse: slice complete)
--
-- Runtime max_hits_cfg (i_max_hits_cfg, 1~7):
--   Controls output beats/cell dynamically. Buffer always allocated for MAX=7.
--   s_rt_last_beat_r is latched from elaboration-safe case lookup at output start.
--   Hit overflow guard uses runtime max_hits_cfg, not compile-time constant.
--   Enables distance-adaptive throughput:
--     max_hits | cell_size | beats @32b | beats @64b
--     ---------|-----------|-----------|----------
--        1     |    4B     |     1     |     1
--        3     |    8B     |     2     |     1
--        5     |   16B     |     4     |     2
--        7     |   32B     |     8     |     4
--
-- Overrun:
--   If no BUF_FREE buffer is available on shot_start, the shot is dropped.
--
-- AXI-Stream slave input (from raw_event_builder / slope demux):
--   tdata[16:0]  = raw_hit (17-bit, lower 16 stored as HIT_SLOT_DATA_WIDTH)
--   tuser[0]     = slope
--   tuser[2:1]   = chip_id
--   tuser[5:3]   = stop_id_local (0..7)
--   tuser[6]     = ififo_id
--   tuser[7]     = drain_done (control beat: triggers output phase)
--   tuser[10:8]  = hit_seq_local (0..7)
--   tuser[15:11] = 0
--
-- AXI-Stream master output (to face_assembler):
--   tdata         = g_TDATA_WIDTH bits (32 or 64)
--   tlast         = last beat of chip slice
--
-- Beat layout (compile-time MAX=7, runtime truncated by max_hits_cfg):
--   Beats 0..HIT_DATA_BEATS-1: hit_slot pairs (SLOTS_PER_BEAT per beat)
--   Beat META_BEAT_IDX:        metadata (hit_valid, slope_vec, hit_count, flags)
--   Remaining beats:           padding (zeros)
--   Runtime beats/cell: fn_beats_per_cell_rt(max_hits_cfg, g_TDATA_WIDTH)
--   Examples @64b: max_hits=7->4, max_hits=3->1, max_hits=1->1
--
-- Signal ownership (no multi-driver):
--   p_collect WRITES: s_cell_buf_r, s_buf_state_r, s_buf_full_r, s_wr_buf_r,
--                     s_cstate_r, s_output_req_r, s_rd_buf_idx_r
--   p_output  WRITES: s_cell_sel_r, s_ostate_r, s_tdata_r, s_tvalid_r,
--                     s_tlast_r, s_output_done_r, etc.
--   p_output  READS:  s_cell_buf_r, s_buf_full_r, s_output_req_r,
--                     s_rd_buf_idx_r (no write conflict)
--   p_collect READS:  s_ostate_r, s_output_done_r (no write conflict)
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
        i_abort             : in  std_logic;   -- abort: free all buffers, return to idle
        -- drain_done is received via i_s_axis_tuser(7) control beat

        -- Configuration (latched at packet_start)
        i_stops_per_chip    : in  unsigned(3 downto 0);
        i_max_hits_cfg      : in  unsigned(2 downto 0);   -- 1~7, runtime MAX_HITS

        -- AXI-Stream master (chip slice output)
        o_m_axis_tdata      : out std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid     : out std_logic;
        o_m_axis_tlast      : out std_logic;
        i_m_axis_tready     : in  std_logic;

        -- Status
        o_slice_done        : out std_logic;    -- 1-clk pulse: chip slice complete
        o_hit_dropped_any   : out std_logic;    -- 1-clk pulse: cell hit overflow
        o_shot_dropped      : out std_logic;    -- 1-clk pulse: entire shot dropped (no free buffer)
        o_slice_timeout     : out std_logic     -- 1-clk pulse: slice truncated by IFIFO2 timeout
    );
end entity tdc_gpx_cell_builder;

architecture rtl of tdc_gpx_cell_builder is

    -- =========================================================================
    -- Ping-pong dual cell buffer (register-based, 2 x MAX_STOPS entries)
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
    -- Buffer ownership FSM (p_collect owns ALL transitions)
    -- =========================================================================
    type t_buf_owner is (BUF_FREE, BUF_COLLECT, BUF_SHARED);
    type t_buf_state_array is array(0 to 1) of t_buf_owner;
    signal s_buf_state_r : t_buf_state_array := (others => BUF_FREE);
    signal s_buf_seq_r   : unsigned(7 downto 0) := (others => '0');  -- monotonic shot counter
    type t_buf_age_array is array(0 to 1) of unsigned(7 downto 0);
    signal s_buf_age_r   : t_buf_age_array := (others => (others => '0'));  -- per-buffer age tag
    signal s_buf_full_r  : std_logic_vector(1 downto 0) := "00";

    type t_collect_state is (ST_C_IDLE, ST_C_ACTIVE, ST_C_DROP);
    signal s_cstate_r     : t_collect_state := ST_C_IDLE;
    signal s_wr_buf_r     : std_logic := '0';

    -- Pending shot_start: set when shot_start arrives but cannot be consumed
    -- this cycle (DROP state, or simultaneous with drain_done in ACTIVE).
    -- Processed on the next cycle where the blocking condition clears.
    signal s_shot_pending_r : std_logic := '0';

    -- p_collect -> p_output handshake
    signal s_output_req_r  : std_logic := '0';
    signal s_rd_buf_idx_r  : std_logic := '0';

    -- p_output -> p_collect handshake
    signal s_output_done_r    : std_logic := '0';
    signal s_out_timeout_r    : unsigned(15 downto 0) := (others => '0');  -- output watchdog
    signal s_slice_timeout_r  : std_logic := '0';  -- 1-clk pulse: IFIFO2 timeout truncation

    -- =========================================================================
    -- Output FSM (p_output)
    -- =========================================================================
    type t_output_state is (ST_O_IDLE, ST_O_LOAD, ST_O_ACTIVE, ST_O_WAIT_IFIFO2,
                            ST_O_TIMEOUT_EOS);
    signal s_ostate_r     : t_output_state := ST_O_IDLE;
    signal s_rd_buf_r     : std_logic := '0';       -- latched read buffer index

    -- Pipeline register: selected cell for output serialization
    signal s_cell_sel_r  : t_cell := c_CELL_INIT;

    -- Output serializer counters
    signal s_stop_idx_r  : unsigned(2 downto 0) := (others => '0');
    signal s_beat_idx_r  : unsigned(2 downto 0) := (others => '0');
    signal s_last_stop_r : unsigned(2 downto 0) := (others => '0');

    -- Generic-derived beat layout constants (compile-time, MAX=7)
    constant c_G_SLOTS_PER_BEAT  : natural := fn_slots_per_beat(g_TDATA_WIDTH);
    constant c_G_HIT_DATA_BEATS  : natural := fn_hit_data_beats(g_TDATA_WIDTH);
    constant c_G_META_BEAT_IDX   : natural := fn_meta_beat_idx(g_TDATA_WIDTH);
    constant c_G_BEATS_PER_CELL  : natural := fn_beats_per_cell(g_TDATA_WIDTH);

    -- Runtime beats/cell: latched from i_max_hits_cfg at output start.
    -- fn_cell_beat still uses compile-time constants for slot packing;
    -- the runtime limit only controls HOW MANY beats are emitted.
    signal s_rt_last_beat_r : unsigned(2 downto 0) := to_unsigned(c_G_BEATS_PER_CELL - 1, 3);
    signal s_rt_max_hits_r  : unsigned(2 downto 0) := to_unsigned(c_MAX_HITS_PER_STOP, 3);

    -- Registered outputs
    signal s_tdata_r     : std_logic_vector(g_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid_r    : std_logic := '0';
    signal s_tlast_r     : std_logic := '0';
    signal s_hit_dropped_r   : std_logic := '0';
    signal s_shot_dropped_r  : std_logic := '0';  -- shot-level drop (no free buffer)
    signal s_timeout_cnt_r   : unsigned(15 downto 0) := (others => '0');  -- watchdog

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
    -- fn_cell_beat: serialize cell to beat.
    -- last_beat_idx: runtime last beat index (from max_hits_cfg).
    -- Metadata is placed at last_beat_idx (not fixed compile-time position).
    -- Hit-data beats fill indices 0..last_beat_idx-1.
    function fn_cell_beat(
        cell          : t_cell;
        beat_idx      : unsigned(2 downto 0);
        last_beat_idx : unsigned(2 downto 0)
    ) return std_logic_vector is
        variable v_result   : std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        variable v_beat     : natural range 0 to 7;
        variable v_slot_idx : natural;
        variable v_lo       : natural;
    begin
        v_beat   := to_integer(beat_idx);
        v_result := (others => '0');

        if beat_idx = last_beat_idx then
            -- Last beat = metadata (always, regardless of max_hits_cfg)
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
            -- Hit-data beat: pack SLOTS_PER_BEAT slots
            for sl in 0 to c_G_SLOTS_PER_BEAT - 1 loop
                v_slot_idx := v_beat * c_G_SLOTS_PER_BEAT + sl;
                if v_slot_idx < c_MAX_HITS_PER_STOP then
                    v_lo := sl * c_HIT_SLOT_DATA_WIDTH;
                    v_result(v_lo + c_HIT_SLOT_DATA_WIDTH - 1 downto v_lo)
                        := std_logic_vector(cell.hit_slot(v_slot_idx));
                end if;
            end loop;
        end if;

        return v_result;
    end function;

begin

    -- AXI-Stream slave: accept during collect or drop (drop silently discards)
    o_s_axis_tready <= '1' when s_cstate_r = ST_C_ACTIVE or s_cstate_r = ST_C_DROP
                  else '0';

    -- =========================================================================
    -- p_collect: write hits into write-buffer, manage buffer ownership FSM
    -- Owns: s_cell_buf_r, s_buf_state_r, s_buf_full_r, s_wr_buf_r, s_cstate_r,
    --       s_output_req_r, s_rd_buf_idx_r, s_hit_dropped_r
    -- Reads (no write): s_ostate_r, s_output_done_r
    -- =========================================================================
    p_collect : process(i_clk)
        variable v_wr       : natural range 0 to 1;
        variable v_other    : natural range 0 to 1;
        variable v_done_buf : natural range 0 to 1;
        variable v_stop  : natural range 0 to c_MAX_STOPS_PER_CHIP - 1;
        variable v_seq   : natural range 0 to c_MAX_HITS_PER_STOP - 1;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_cell_buf_r    <= (others => (others => c_CELL_INIT));
                s_buf_state_r   <= (others => BUF_FREE);
                s_buf_full_r    <= "00";
                s_cstate_r      <= ST_C_IDLE;
                s_wr_buf_r      <= '0';
                s_output_req_r  <= '0';
                s_rd_buf_idx_r  <= '0';
                s_hit_dropped_r  <= '0';
                s_shot_dropped_r <= '0';
                s_timeout_cnt_r  <= (others => '0');
                s_shot_pending_r <= '0';
                s_buf_seq_r      <= (others => '0');
                s_buf_age_r      <= (others => (others => '0'));
            else
                -- Default: clear single-cycle pulses
                s_output_req_r   <= '0';
                s_hit_dropped_r  <= '0';
                s_shot_dropped_r <= '0';

                -- ---------------------------------------------------------
                -- Abort: free all buffers, return to idle
                -- ---------------------------------------------------------
                if i_abort = '1' then
                    s_buf_state_r <= (others => BUF_FREE);
                    s_buf_full_r  <= "00";
                    s_cstate_r    <= ST_C_IDLE;
                    s_shot_pending_r <= '0';  -- drop any pending shot on abort
                    -- Do NOT clear cell_buf (unnecessary, will be cleared on next shot_start)
                else
                    -- ---------------------------------------------------------
                    -- Handle output completion: BUF_SHARED -> BUF_FREE
                    -- ---------------------------------------------------------
                    if s_output_done_r = '1' then
                        -- Free ONLY the buffer that output just finished reading.
                        -- Do NOT touch the other buffer (it may be BUF_SHARED
                        -- for the next shot, waiting for output to start).
                        v_done_buf := fn_buf_idx(s_rd_buf_r);
                        s_buf_state_r(v_done_buf) <= BUF_FREE;
                        s_buf_full_r(v_done_buf)  <= '0';
                        -- Auto-start: if the OTHER buffer is SHARED, start output
                        v_other := 1 - v_done_buf;
                        if s_buf_state_r(v_other) = BUF_SHARED then
                            s_output_req_r <= '1';
                            if v_other = 0 then
                                s_rd_buf_idx_r <= '0';
                            else
                                s_rd_buf_idx_r <= '1';
                            end if;
                        end if;
                    end if;

                    v_wr := fn_buf_idx(s_wr_buf_r);

                    case s_cstate_r is

                        when ST_C_IDLE =>
                            -- Accept new shot_start OR a pending one latched earlier
                            -- (e.g. from DROP state that just completed its final_done).
                            if i_shot_start = '1' or s_shot_pending_r = '1' then
                                s_shot_pending_r <= '0';  -- consume
                                -- Find a FREE buffer
                                if s_buf_state_r(0) = BUF_FREE then
                                    s_wr_buf_r        <= '0';
                                    s_buf_state_r(0)  <= BUF_COLLECT;
                                    s_cell_buf_r(0)   <= (others => c_CELL_INIT);
                                    s_cstate_r        <= ST_C_ACTIVE;
                                elsif s_buf_state_r(1) = BUF_FREE then
                                    s_wr_buf_r        <= '1';
                                    s_buf_state_r(1)  <= BUF_COLLECT;
                                    s_cell_buf_r(1)   <= (others => c_CELL_INIT);
                                    s_cstate_r        <= ST_C_ACTIVE;
                                else
                                    -- No free buffer: enter DROP to actually absorb
                                    -- the upcoming shot's beats (tready='1'). Staying
                                    -- in IDLE would leave tready='0' and stall the
                                    -- upstream decoder even though we declared the
                                    -- shot dropped.
                                    s_cstate_r      <= ST_C_DROP;
                                    s_shot_dropped_r <= '1';
                                    s_timeout_cnt_r  <= (others => '0');
                                end if;
                            end if;

                        when ST_C_ACTIVE =>
                            -- Raw event write (exclude drain_done control beat)
                            if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '0' then
                                v_stop := to_integer(unsigned(i_s_axis_tuser(5 downto 3)));

                                -- synthesis translate_off
                                assert i_max_hits_cfg /= "000"
                                    report "cell_builder: i_max_hits_cfg=0 is invalid"
                                    severity error;
                                assert unsigned(i_s_axis_tuser(2 downto 1)) = g_CHIP_ID
                                    report "cell_builder: input chip_id mismatch (got " &
                                           integer'image(to_integer(unsigned(i_s_axis_tuser(2 downto 1)))) &
                                           ", expected " & integer'image(g_CHIP_ID) & ")"
                                    severity warning;
                                -- synthesis translate_on

                                -- Runtime stop_id bounds check
                                if ('0' & unsigned(i_s_axis_tuser(5 downto 3))) >= i_stops_per_chip then
                                    -- Out-of-range stop_id: discard (distinct from hit overflow)
                                    -- synthesis translate_off
                                    assert false
                                        report "cell_builder: stop_id " &
                                               integer'image(to_integer(unsigned(i_s_axis_tuser(5 downto 3)))) &
                                               " >= stops_per_chip " &
                                               integer'image(to_integer(i_stops_per_chip))
                                        severity warning;
                                    -- synthesis translate_on
                                    s_hit_dropped_r <= '1';  -- TODO: separate o_stop_id_invalid when port available
                                elsif s_cell_buf_r(v_wr)(v_stop).hit_count_actual < ('0' & i_max_hits_cfg) then
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
                            --   tuser[7]=1, tuser[6]=0: ififo1_done -> BUF_SHARED, start output
                            --   tuser[7]=1, tuser[6]=1: final_done  -> set buf_full flag
                            if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '1' then
                                if i_s_axis_tuser(6) = '0' then
                                    -- IFIFO1 done: BUF_COLLECT -> BUF_SHARED
                                    -- Output reads stops 0~3 from same buffer.
                                    -- Collect continues writing stops 4~7.
                                    s_buf_state_r(v_wr) <= BUF_SHARED;
                                    s_buf_full_r(v_wr)  <= '0';
                                    s_buf_age_r(v_wr)   <= s_buf_seq_r;
                                    s_buf_seq_r         <= s_buf_seq_r + 1;
                                    if s_ostate_r = ST_O_IDLE then
                                        s_output_req_r <= '1';
                                        s_rd_buf_idx_r <= s_wr_buf_r;
                                    end if;
                                    -- Note: if output is busy, the SHARED buffer waits.
                                    -- When output finishes the current slice and returns
                                    -- to IDLE, it will pick up this SHARED buffer via
                                    -- the auto-start check in the output_done handler.
                                    -- Actually, auto-start fires on the SAME cycle as
                                    -- output_done, so we need a separate check below.
                                else
                                    -- Final drain_done: mark stops 4~7 ready
                                    s_buf_full_r(v_wr) <= '1';
                                    if s_buf_state_r(v_wr) = BUF_COLLECT then
                                        -- Both IFIFOs done simultaneously (no prior ififo1_done)
                                        s_buf_state_r(v_wr) <= BUF_SHARED;
                                        s_buf_age_r(v_wr)   <= s_buf_seq_r;
                                        s_buf_seq_r         <= s_buf_seq_r + 1;
                                        if s_ostate_r = ST_O_IDLE then
                                            s_output_req_r <= '1';
                                            s_rd_buf_idx_r <= s_wr_buf_r;
                                        end if;
                                    end if;
                                    -- If already BUF_SHARED: just setting full flag is enough.
                                    -- p_output will see buf_full and exit WAIT_IFIFO2.
                                end if;
                            end if;

                            -- shot_start during active: switch to other buffer for next shot.
                            -- Same-cycle drain_done defers shot_start to a pending latch,
                            -- and any previously-latched pending shot is consumed here.
                            if (i_shot_start = '1'
                                  and not (i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '1'))
                               or s_shot_pending_r = '1' then
                                s_shot_pending_r <= '0';  -- consume
                                -- Find the OTHER buffer
                                if v_wr = 0 then v_other := 1; else v_other := 0; end if;
                                if s_buf_state_r(v_other) = BUF_FREE then
                                    s_wr_buf_r <= not s_wr_buf_r;
                                    s_buf_state_r(v_other) <= BUF_COLLECT;
                                    s_cell_buf_r(v_other)  <= (others => c_CELL_INIT);
                                else
                                    -- No free buffer: enter drop mode to prevent shot mixing.
                                    -- All incoming data for this shot is silently discarded.
                                    -- tready stays '1' so upstream doesn't stall.
                                    s_cstate_r      <= ST_C_DROP;
                                    s_shot_dropped_r <= '1';  -- distinct from hit overflow
                                    s_timeout_cnt_r  <= (others => '0');
                                end if;
                            elsif i_shot_start = '1' then
                                -- Same-cycle drain_done blocks shot_start: latch as pending
                                -- so the next cycle can switch buffers without losing the shot.
                                s_shot_pending_r <= '1';
                            end if;

                        when ST_C_DROP =>
                            -- Silently discard all incoming data until final drain_done.
                            -- A new shot_start arriving during DROP is latched so the next
                            -- shot is not lost (ST_C_IDLE handler will pick it up).
                            if i_shot_start = '1' then
                                s_shot_pending_r <= '1';
                            end if;
                            if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '1'
                               and i_s_axis_tuser(6) = '1' then
                                s_cstate_r     <= ST_C_IDLE;
                                s_timeout_cnt_r <= (others => '0');
                            elsif s_timeout_cnt_r = x"FFFF" then
                                -- Timeout: drain_done never came, force IDLE
                                s_cstate_r     <= ST_C_IDLE;
                                s_timeout_cnt_r <= (others => '0');
                            else
                                s_timeout_cnt_r <= s_timeout_cnt_r + 1;
                            end if;

                    end case;

                    -- ---------------------------------------------------------
                    -- Deferred auto-start: if output just went IDLE (not via
                    -- output_done, e.g. abort recovery) and a SHARED buffer
                    -- exists without an active output_req, start it.
                    -- This also catches the case where ififo1_done arrived
                    -- while output was busy with the previous slice.
                    -- ---------------------------------------------------------
                    if s_ostate_r = ST_O_IDLE and s_output_req_r = '0'
                       and s_output_done_r = '0' then
                        -- Age-based selection: if both SHARED, pick older (smaller seq)
                        if s_buf_state_r(0) = BUF_SHARED and s_buf_state_r(1) = BUF_SHARED then
                            s_output_req_r <= '1';
                            -- Wrap-safe age compare: (age1 - age0) < 128 means age0 is older
                            -- Example: age0=10, age1=11 → (11-10)=1 < 128 → buf0 is older → pick buf0
                            if (s_buf_age_r(1) - s_buf_age_r(0)) < 128 then
                                s_rd_buf_idx_r <= '0';  -- buf0 is older
                            else
                                s_rd_buf_idx_r <= '1';
                            end if;
                        elsif s_buf_state_r(0) = BUF_SHARED then
                            s_output_req_r <= '1';
                            s_rd_buf_idx_r <= '0';
                        elsif s_buf_state_r(1) = BUF_SHARED then
                            s_output_req_r <= '1';
                            s_rd_buf_idx_r <= '1';
                        end if;
                    end if;  -- deferred auto-start

                end if;  -- not abort
            end if;  -- rst_n
        end if;  -- rising_edge
    end process p_collect;

    -- =========================================================================
    -- p_output: serialize read-buffer cells to AXI-Stream master
    -- Owns: s_ostate_r, s_rd_buf_r, s_cell_sel_r, s_stop_idx_r, s_beat_idx_r,
    --       s_last_stop_r, s_tdata_r, s_tvalid_r, s_tlast_r, s_output_done_r
    -- Reads (no write): s_cell_buf_r, s_buf_full_r, s_output_req_r,
    --       s_rd_buf_idx_r
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
                s_output_done_r   <= '0';
                s_slice_timeout_r <= '0';
                s_out_timeout_r   <= (others => '0');
                s_rt_last_beat_r  <= to_unsigned(c_G_BEATS_PER_CELL - 1, 3);
                s_rt_max_hits_r   <= to_unsigned(c_MAX_HITS_PER_STOP, 3);
            else
                s_output_done_r  <= '0';
                s_slice_timeout_r <= '0';

                -- ---------------------------------------------------------
                -- Abort: return to idle, clear tvalid
                -- ---------------------------------------------------------
                if i_abort = '1' then
                    s_ostate_r <= ST_O_IDLE;
                    s_tvalid_r <= '0';
                    s_tlast_r  <= '0';
                else

                    case s_ostate_r is

                        when ST_O_IDLE =>
                            s_tvalid_r <= '0';
                            s_tlast_r  <= '0';

                            if s_output_req_r = '1' then
                                -- Latch read buffer, load first cell
                                s_rd_buf_r    <= s_rd_buf_idx_r;
                                v_rd          := fn_buf_idx(s_rd_buf_idx_r);
                                s_cell_sel_r    <= s_cell_buf_r(v_rd)(0);
                                s_stop_idx_r    <= (others => '0');
                                s_beat_idx_r    <= (others => '0');
                                if i_stops_per_chip >= 2 then
                                    s_last_stop_r <= i_stops_per_chip(2 downto 0) - 1;
                                else
                                    s_last_stop_r <= (others => '0');  -- degenerate: clamp to 1 stop
                                end if;
                                -- Runtime beats/cell lookup (elaboration-safe)
                                case i_max_hits_cfg is
                                    when "001" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(1, g_TDATA_WIDTH) - 1, 3);
                                    when "010" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(2, g_TDATA_WIDTH) - 1, 3);
                                    when "011" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(3, g_TDATA_WIDTH) - 1, 3);
                                    when "100" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(4, g_TDATA_WIDTH) - 1, 3);
                                    when "101" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(5, g_TDATA_WIDTH) - 1, 3);
                                    when "110" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(6, g_TDATA_WIDTH) - 1, 3);
                                    when others => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(7, g_TDATA_WIDTH) - 1, 3);
                                end case;
                                s_rt_max_hits_r  <= i_max_hits_cfg;
                                s_ostate_r       <= ST_O_LOAD;
                            end if;

                        when ST_O_LOAD =>
                            -- Beat MUX from pipeline register (1-clk bubble)
                            s_tdata_r  <= fn_cell_beat(s_cell_sel_r, s_beat_idx_r, s_rt_last_beat_r);
                            s_tvalid_r <= '1';
                            if s_stop_idx_r = s_last_stop_r
                               and s_beat_idx_r = s_rt_last_beat_r then
                                s_tlast_r <= '1';
                            else
                                s_tlast_r <= '0';
                            end if;
                            s_ostate_r <= ST_O_ACTIVE;

                        when ST_O_ACTIVE =>
                            if s_tvalid_r = '1' and i_m_axis_tready = '1' then
                                v_rd := fn_buf_idx(s_rd_buf_r);

                                if s_stop_idx_r = s_last_stop_r
                                   and s_beat_idx_r = s_rt_last_beat_r then
                                    -- Chip slice complete
                                    s_tvalid_r      <= '0';
                                    s_tlast_r       <= '0';
                                    s_output_done_r <= '1';
                                    s_ostate_r      <= ST_O_IDLE;

                                elsif s_beat_idx_r = s_rt_last_beat_r then
                                    -- Cell boundary: check IFIFO1/2 split point (stop 3->4)
                                    v_nxt_stop := s_stop_idx_r + 1;
                                    if s_stop_idx_r = "011"
                                       and s_buf_full_r(fn_buf_idx(s_rd_buf_r)) = '0' then
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
                                    s_tdata_r    <= fn_cell_beat(s_cell_sel_r, v_nxt_beat, s_rt_last_beat_r);
                                    if s_stop_idx_r = s_last_stop_r
                                       and v_nxt_beat = s_rt_last_beat_r then
                                        s_tlast_r <= '1';
                                    else
                                        s_tlast_r <= '0';
                                    end if;
                                end if;
                            end if;

                        when ST_O_TIMEOUT_EOS =>
                            -- Synthetic EOS beat: wait for tready, then signal
                            -- output_done and return to idle. p_collect picks up
                            -- s_output_done_r and releases the buffer normally.
                            if i_m_axis_tready = '1' then
                                s_tvalid_r      <= '0';
                                s_tlast_r       <= '0';
                                s_output_done_r <= '1';
                                s_ostate_r      <= ST_O_IDLE;
                            end if;

                        when ST_O_WAIT_IFIFO2 =>
                            -- Stall: stops 0~3 output done, waiting for IFIFO2 data.
                            if s_buf_full_r(fn_buf_idx(s_rd_buf_r)) = '1' then
                                v_rd := fn_buf_idx(s_rd_buf_r);
                                s_stop_idx_r    <= "100";    -- stop 4
                                s_beat_idx_r    <= (others => '0');
                                s_cell_sel_r    <= s_cell_buf_r(v_rd)(4);
                                s_out_timeout_r <= (others => '0');
                                s_ostate_r      <= ST_O_LOAD;
                            elsif s_out_timeout_r = x"FFFF" then
                                -- Timeout: IFIFO2 data never arrived. Emit a
                                -- synthetic final beat with tlast so downstream
                                -- (face_assembler) completes this chip slice.
                                -- Data is zeroed; s_slice_timeout_r flags the
                                -- truncation for SW visibility via status.
                                s_tdata_r         <= (others => '0');
                                s_tvalid_r        <= '1';
                                s_tlast_r         <= '1';
                                s_slice_timeout_r <= '1';
                                s_out_timeout_r   <= (others => '0');
                                s_ostate_r        <= ST_O_TIMEOUT_EOS;
                            else
                                s_out_timeout_r <= s_out_timeout_r + 1;
                            end if;

                    end case;

                end if;  -- not abort
            end if;  -- rst_n
        end if;  -- rising_edge
    end process p_output;

    -- =========================================================================
    -- Output assignments (all registered)
    -- =========================================================================
    o_m_axis_tdata    <= s_tdata_r;
    o_m_axis_tvalid   <= s_tvalid_r;
    o_m_axis_tlast    <= s_tlast_r;
    o_slice_done      <= s_output_done_r;
    o_hit_dropped_any <= s_hit_dropped_r;
    o_shot_dropped    <= s_shot_dropped_r;
    o_slice_timeout   <= s_slice_timeout_r;

end architecture rtl;
