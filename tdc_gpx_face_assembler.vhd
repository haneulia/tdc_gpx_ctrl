-- =============================================================================
-- tdc_gpx_face_assembler.vhd
-- TDC-GPX Controller - Face Assembler (Packed Row, FCFS)
-- =============================================================================
--
-- Purpose:
--   Collects 4 chip cells (AXI-Stream) and assembles into a packed row.
--   Inactive chips are skipped. Timed-out chips receive blank cells
--   with error_fill=1.
--
-- Generics:
--   g_TDATA_WIDTH : 32 or 64 (output bus width, affects beats/cell)
--
-- Input FIFOs:
--   4× xpm_fifo_axis (Xilinx XPM, 16-deep, distributed RAM).
--   Standard AXI-Stream interface — no manual bundle/unbundle.
--   Per-chip inputs via individual ports (i_s_axis_tdata_0..3).
--
-- Output FIFO:
--   1× xpm_fifo_axis (16-deep) for backpressure decoupling.
--
--   Strict In-Order scheduling:
--     Chips are always output in ascending order (chip0 → chip1 → chip2 → chip3).
--     Inactive chips are skipped. Timed-out chips receive blank cells.
--     Deterministic output order eliminates SW reordering overhead.
--     Each cell carries a chip_id tag in metadata beat for verification.
--
--   Per-shot flow:
--     1. shot_start → ST_SCAN (or row_done immediate-pulse if zero active mask)
--     2. ST_SCAN: priority encode undone chip (ready first, timeout blank)
--     3. ST_RESOLVE: compute is_last_chip (1 clk)
--     4. ST_FORWARD: produce beats to output FIFO
--     5. Chip done → mark s_chip_done_r, back to ST_SCAN
--     6. All active chips done → row_done, ST_IDLE
--
-- Zero active_chip_mask policy (Round 3 #19):
--   If i_active_chip_mask = "0000" at shot_start, row_done pulses
--   immediately and the FSM returns to ST_IDLE (self-consistent without
--   needing face_seq to pre-gate zero-mask shots).
--
-- shot_start overrun policy — BLANK-FILL (Q&A #36, Round 4):
--   If shot_start arrives while still in ST_SCAN/ST_RESOLVE/ST_FORWARD
--   (current line not finished), we do NOT face-abort. Instead:
--     - s_shot_pending_r <= '1' (new shot becomes next line)
--     - s_shot_overrun_r <= '1' (SW visibility)
--     - s_chip_error_r OR'd with un-done chips (SW sees error_fill per
--       affected chip's cells)
--     - s_shot_cnt_r <= FFFF so subsequent chips enter blank via ST_SCAN
--       hard-cap
--     - If mid-ST_FORWARD real-chip: switch s_is_blank_r='1' and resume
--       blank generation from (s_fwd_stop_r, s_fwd_beat_r) position so
--       the chip's beat count stays exact
--     - face_abort is NOT pulsed; line completes in-place and the face
--       continues naturally.
--   Net effect: VDMA frame size is preserved; SW detects per-cell errors
--   via error_fill bit in cell metadata; next shot becomes the next line.
--
-- Forward position tracking (Round 4 #36 support):
--   s_fwd_stop_r / s_fwd_beat_r (both 3-bit) track (stop_idx, beat_idx_in_cell)
--   during ST_FORWARD real-chip forwarding. Reset at ST_RESOLVE → ST_FORWARD
--   transition, incremented per forwarded beat, handed off to blank
--   generator on overrun.
--
-- All outputs are registered (module boundary = FF).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library xpm;
use xpm.vcomponents.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_face_assembler is
    generic (
        g_ALU_PULSE_CLKS     : natural := 4;
        g_TDATA_WIDTH        : natural := c_TDATA_WIDTH   -- 32 or 64
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- 4 chip cell AXI-Stream inputs (from cell_builders, g_TDATA_WIDTH per chip)
        i_s_axis_tdata_0     : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tdata_1     : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tdata_2     : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tdata_3     : in  std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        i_s_axis_tvalid      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_s_axis_tlast       : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_s_axis_tready      : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Shot control
        i_shot_start         : in  std_logic;
        i_abort              : in  std_logic;    -- cmd_stop/soft_reset: flush + ST_IDLE

        -- Configuration (latched at packet_start)
        i_active_chip_mask   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_stops_per_chip     : in  unsigned(3 downto 0);
        i_max_hits_cfg       : in  unsigned(2 downto 0);   -- runtime max_hits (1~7)
        -- Scan timeout: max clock cycles before declaring a chip's cell blank.
        -- SW must bake in all margins (bus roundtrip + drain + ALU service).
        -- 0 = disabled (no timeout, wait indefinitely for chip data).
        i_max_scan_clks     : in  unsigned(15 downto 0);

        -- AXI-Stream master (packed row output)
        o_m_axis_tdata       : out std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid      : out std_logic;
        o_m_axis_tlast       : out std_logic;
        i_m_axis_tready      : in  std_logic;

        -- Status
        o_row_done           : out std_logic;    -- 1-clk pulse: packed row complete
        -- Round 13 axis 1c: 1-clk pulse co-asserts with o_row_done when
        -- the row contains synthetic/blank-fill data (i.e. some chip's
        -- data was truncated or never arrived). SW distinguishes clean
        -- rows from faulted rows per event without needing the sticky.
        o_row_done_faulted   : out std_logic;
        o_chip_error_flags   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        -- Round 12 #18: split chip error into partial/blank causes.
        o_chip_error_partial : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_chip_error_blank   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_shot_overrun       : out std_logic;    -- 1-clk pulse: shot truncated (was not idle)
        -- DEPRECATED (Round 5 #19 + Round 12 #14): o_face_abort is a stub.
        -- Permanent '0' since Round 4 — face_assembler self-completes via
        -- blank-fill and never asserts this port. Top wires it to 'open'
        -- (tdc_gpx_top.vhd:721) and output_stage merely propagates it.
        --
        -- REMOVAL CHECKLIST for a future Round:
        --   1. Delete this port from face_assembler entity + architecture.
        --   2. Delete the forwarding in tdc_gpx_output_stage.vhd:109,
        --      :230, :268 (port itself, plus both gen instances).
        --   3. Delete the `o_face_abort => open` line in tdc_gpx_top.vhd.
        --   4. Update the 3 testbenches that still reference the port:
        --        tb_tdc_gpx_mask_sweep.vhd, tb_tdc_gpx_downstream.vhd,
        --        tb_tdc_gpx_output_stage.vhd.
        -- Round 12 chose NOT to remove the port to avoid a breaking
        -- interface churn alongside the many observability additions
        -- landing in the same release — migration is deferred until a
        -- future interface-cleanup round.
        o_face_abort         : out std_logic;    -- stub '0' (see DEPRECATED note above)
        o_idle               : out std_logic;    -- '1' when FSM is in ST_IDLE
        -- Trace / status (Round 5 #15 #16)
        o_shot_flush_drop    : out std_logic;    -- sticky: any chip had old-shot tail dropped
        -- Round 11 item 15: per-chip breakdown of the above sticky.
        -- Bit i = '1' (latched) if chip i's input FIFO still held data at
        -- the moment shot_start flushed. The OR-reduction of this mask is
        -- what o_shot_flush_drop exposes.
        o_shot_flush_drop_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_shot_overrun_count : out unsigned(7 downto 0)  -- wrapping count of mid-shot overruns (blank-fill invocations)
    );
end entity tdc_gpx_face_assembler;

architecture rtl of tdc_gpx_face_assembler is

    -- Per-chip tdata array type
    type t_tdata_arr is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
    -- Input port mapping (before FIFO)
    signal s_in_tdata_src : t_tdata_arr;
    -- FIFO output (after FIFO, to FSM)
    signal s_in_tdata     : t_tdata_arr;

    -- =========================================================================
    -- Derived constants
    -- =========================================================================
    constant c_G_BEATS_PER_CELL : natural := fn_beats_per_cell(g_TDATA_WIDTH);

    -- =========================================================================
    -- Input FIFO output signals (xpm_fifo_axis → FSM)
    -- =========================================================================
    -- s_in_tdata: defined above as t_tdata_arr (g_TDATA_WIDTH per chip)
    signal s_in_tvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_in_tlast    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_in_tready   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Output pipe signals (FSM → output skid buffer)
    -- =========================================================================
    signal s_pipe_tdata_r  : std_logic_vector(g_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_pipe_tvalid_r : std_logic := '0';
    signal s_pipe_tlast_r  : std_logic := '0';
    signal s_pipe_tready   : std_logic;  -- from output FIFO

    -- =========================================================================
    -- Synchronous flush for FIFOs (active-low aresetn pulse)
    -- =========================================================================
    signal s_flush        : std_logic;  -- input FIFO flush (every shot_start/abort)
    signal s_fifo_rst_n   : std_logic;  -- active-low reset for input FIFOs
    signal s_out_fifo_rst_n : std_logic;  -- active-low reset for output FIFO (abort only)

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_SCAN, ST_RESOLVE, ST_FORWARD);
    signal s_state_r : t_state := ST_IDLE;

    -- =========================================================================
    -- Per-chip status
    -- =========================================================================
    signal s_chip_ready_r    : std_logic_vector(3 downto 0) := (others => '0');
    signal s_chip_done_r     : std_logic_vector(3 downto 0) := (others => '0');

    -- =========================================================================
    -- Single shot counter (runs from shot_start through ST_SCAN/ST_FORWARD)
    -- =========================================================================
    signal s_shot_cnt_r      : unsigned(15 downto 0) := (others => '0');
    signal s_timeout_limit_r : unsigned(15 downto 0) := (others => '0');
    -- Round 9 #2: intra-forward stall watchdog. ST_SCAN has a timeout for
    -- "first beat never arrives", but ST_FORWARD real-chip path has no
    -- watchdog for "first beat arrived, then the stream stalls mid-chip".
    -- This counter increments when we're in ST_FORWARD, the current chip is
    -- real (not blank), and i_s_axis_tvalid is low. On overflow we flip the
    -- current chip to blank mode from the currently-tracked (fwd_stop,
    -- fwd_beat) position and flag chip_error — the remainder is generated
    -- as blank beats so the row still completes.
    signal s_fwd_stall_cnt_r : unsigned(15 downto 0) := (others => '0');

    -- Combinational timeout limit (latched into s_timeout_limit_r on shot_start)
    signal s_timeout_limit   : unsigned(15 downto 0);

    -- =========================================================================
    -- Forwarding state
    -- =========================================================================
    signal s_cur_chip_r      : unsigned(1 downto 0) := (others => '0');
    signal s_next_chip_r     : unsigned(1 downto 0) := (others => '0');  -- strict in-order pointer
    signal s_is_blank_r      : std_logic := '0';   -- current chip is timed out
    signal s_is_last_chip_r  : std_logic := '0';   -- no more undone active chips after this

    -- Blank beat generation counters
    signal s_blank_stop_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7
    signal s_blank_beat_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7
    signal s_last_stop_r     : unsigned(2 downto 0) := (others => '0');  -- pre-computed: stops-1
    signal s_rt_last_beat_r  : unsigned(2 downto 0) := to_unsigned(c_G_BEATS_PER_CELL - 1, 3);

    -- Forward position trackers for current chip (Q&A #36).
    -- Separately track (stop_idx, beat_idx_in_cell) during ST_FORWARD real-chip
    -- path so shot_start overrun can hand off to blank mode at the exact cell
    -- boundary (works for any runtime cell size = s_rt_last_beat_r + 1).
    signal s_fwd_stop_r      : unsigned(2 downto 0) := (others => '0');
    signal s_fwd_beat_r      : unsigned(2 downto 0) := (others => '0');

    -- Latched config: snapshot at shot_start to prevent mid-frame changes
    -- from corrupting priority encoder / is_last_chip computation.
    signal s_active_mask_r   : std_logic_vector(3 downto 0) := (others => '0');

    -- =========================================================================
    -- Can FSM produce a new beat?
    --   True when: pipe empty OR pipe handshake occurring.
    --   s_pipe_tready is REGISTERED (from output skid buffer).
    -- =========================================================================
    signal s_can_produce     : std_logic;

    -- =========================================================================
    -- Status registers
    -- =========================================================================
    signal s_row_done_r      : std_logic := '0';
    signal s_chip_error_r    : std_logic_vector(3 downto 0) := (others => '0');
    -- Round 12 #18: split chip_error into "partial (mid-forward at overrun)"
    -- vs "blank (not-yet-started)" so SW can distinguish a chip whose data
    -- was truncated mid-cell from one that was never reached before the
    -- overrun boundary. Previously both folded into s_chip_error_r.
    signal s_chip_error_partial_r : std_logic_vector(3 downto 0) := (others => '0');
    signal s_chip_error_blank_r   : std_logic_vector(3 downto 0) := (others => '0');
    -- Round 13 axis 1c: per-row "faulted" pulse, co-asserts with o_row_done
    -- when ANY chip in the row had partial/blank error (i.e. the row
    -- contains synthetic fill). Gives SW a 1-clk event distinct from the
    -- persistent chip_error masks so "this frame's rows were faulted"
    -- can be tracked without polling.
    signal s_row_done_faulted_r   : std_logic := '0';
    signal s_shot_overrun_r  : std_logic := '0';
    signal s_face_abort_r    : std_logic := '0';
    signal s_shot_pending_r  : std_logic := '0';  -- shot arrived on row-complete edge
    -- Round 5 #15/#16 trace counters / stickies
    signal s_shot_flush_drop_r      : std_logic := '0';  -- sticky: shot_start flushed non-empty FIFO
    signal s_shot_flush_drop_mask_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');  -- Round 11 item 15: per-chip
    signal s_shot_overrun_cnt_r : unsigned(7 downto 0) := (others => '0');  -- wrapping mid-shot overrun count

    -- =========================================================================
    -- Blank beat data: all zeros except metadata beat has error_fill + chip_id
    -- Metadata beat index = c_META_BEAT_IDX (auto-derived from c_MAX_HITS_PER_STOP)
    -- =========================================================================
    function fn_blank_beat(
        beat_idx  : unsigned(2 downto 0);
        last_beat : unsigned(2 downto 0);
        chip_id   : unsigned(1 downto 0)
    ) return std_logic_vector is
        variable v_result : std_logic_vector(g_TDATA_WIDTH - 1 downto 0);
    begin
        v_result := (others => '0');
        -- Metadata on last beat (matches cell_builder fn_cell_beat convention)
        if beat_idx = last_beat then
            v_result(10)          := '1';   -- error_fill
            v_result(9 downto 8)  := std_logic_vector(chip_id);
        end if;
        return v_result;
    end function;

begin

    -- Map individual tdata ports to internal array
    s_in_tdata_src(0) <= i_s_axis_tdata_0;
    s_in_tdata_src(1) <= i_s_axis_tdata_1;
    s_in_tdata_src(2) <= i_s_axis_tdata_2;
    s_in_tdata_src(3) <= i_s_axis_tdata_3;

    -- =========================================================================
    -- Input AXI-Stream FIFOs (×4): xpm_fifo_axis, 16-deep
    -- Standard AXI-Stream interface — no manual bundle/unbundle.
    -- =========================================================================
    gen_in_fifo : for i in 0 to c_N_CHIPS - 1 generate
        u_fifo_in : xpm_fifo_axis
            generic map (
                CASCADE_HEIGHT    => 0,
                CDC_SYNC_STAGES   => 2,
                CLOCKING_MODE     => "common_clock",
                ECC_MODE          => "no_ecc",
                FIFO_DEPTH        => 16,
                FIFO_MEMORY_TYPE  => "distributed",
                PACKET_FIFO       => "false",
                TDATA_WIDTH       => g_TDATA_WIDTH,
                TDEST_WIDTH       => 1,
                TID_WIDTH         => 1,
                TUSER_WIDTH       => 1,
                USE_ADV_FEATURES  => "0000"
            )
            port map (
                s_aclk          => i_clk,
                s_aresetn       => s_fifo_rst_n,
                s_axis_tdata    => s_in_tdata_src(i),
                s_axis_tvalid   => i_s_axis_tvalid(i),
                s_axis_tready   => o_s_axis_tready(i),
                s_axis_tlast    => i_s_axis_tlast(i),
                s_axis_tkeep    => (others => '1'),
                s_axis_tstrb    => (others => '1'),
                s_axis_tuser    => "0",
                s_axis_tid      => "0",
                s_axis_tdest    => "0",
                m_axis_tdata    => s_in_tdata(i),
                m_axis_tvalid   => s_in_tvalid(i),
                m_axis_tready   => s_in_tready(i),
                m_axis_tlast    => s_in_tlast(i),
                m_axis_tkeep    => open,
                m_axis_tstrb    => open,
                m_axis_tuser    => open,
                m_axis_tid      => open,
                m_axis_tdest    => open,
                m_aclk          => '0',
                injectsbiterr_axis => '0',
                injectdbiterr_axis => '0'
            );
    end generate gen_in_fifo;

    -- =========================================================================
    -- Output AXI-Stream FIFO (×1): xpm_fifo_axis, 16-deep
    -- =========================================================================
    u_fifo_out : xpm_fifo_axis
        generic map (
            CASCADE_HEIGHT    => 0,
            CDC_SYNC_STAGES   => 2,
            CLOCKING_MODE     => "common_clock",
            ECC_MODE          => "no_ecc",
            FIFO_DEPTH        => 16,
            FIFO_MEMORY_TYPE  => "distributed",
            PACKET_FIFO       => "false",
            TDATA_WIDTH       => g_TDATA_WIDTH,
            TDEST_WIDTH       => 1,
            TID_WIDTH         => 1,
            TUSER_WIDTH       => 1,
            USE_ADV_FEATURES  => "0000"
        )
        port map (
            s_aclk          => i_clk,
            s_aresetn       => s_out_fifo_rst_n,  -- flush on abort ONLY (not shot_start)
            s_axis_tdata    => s_pipe_tdata_r,
            s_axis_tvalid   => s_pipe_tvalid_r,
            s_axis_tready   => s_pipe_tready,
            s_axis_tlast    => s_pipe_tlast_r,
            s_axis_tkeep    => (others => '1'),
            s_axis_tstrb    => (others => '1'),
            s_axis_tuser    => "0",
            s_axis_tid      => "0",
            s_axis_tdest    => "0",
            m_axis_tdata    => o_m_axis_tdata,
            m_axis_tvalid   => o_m_axis_tvalid,
            m_axis_tready   => i_m_axis_tready,
            m_axis_tlast    => o_m_axis_tlast,
            m_axis_tkeep    => open,
            m_axis_tstrb    => open,
            m_axis_tuser    => open,
            m_axis_tid      => open,
            m_axis_tdest    => open,
            m_aclk          => '0',
            injectsbiterr_axis => '0',
            injectdbiterr_axis => '0'
        );

    -- =========================================================================
    -- Concurrent signals
    -- =========================================================================

    -- Timeout limit: max_range_clks only (drain/ALU margins TBD after bench)
    s_timeout_limit <= i_max_scan_clks;

    -- Flush input FIFOs on shot_start (new shot) or abort (stop/reset).
    -- Late-arriving beats from previous shot are intentionally dropped.
    -- This policy prioritizes shot boundary integrity over data completeness.
    -- Drops are reflected in top-level frame_abort_count / shot_drop_count.
    --
    -- Round 11 item 15: observability cause distinction.
    --   s_shot_flush_drop_r (per-slope) fires when shot_start flushes
    --   while any input FIFO held data (i.e. old-shot tail present).
    --   The abort path does NOT set this sticky — abort is explicit SW
    --   intent, not boundary drift. That separation is already present
    --   (see the guard `i_shot_start='1' and s_in_tvalid /= "0000"`
    --   below which excludes the abort-only flush case).
    --
    --   Per-chip breakdown of WHICH input FIFO still had data would
    --   require s_shot_flush_drop_mask_r[c_N_CHIPS]; it is not added
    --   here because SW typically only needs the binary indication
    --   "is the pipeline losing shot-boundary alignment?" to trigger a
    --   full re-init. Per-chip resolution can be added later if a
    --   specific chip's stream needs fault isolation.
    s_flush          <= i_shot_start or i_abort;
    s_fifo_rst_n     <= i_rst_n and (not s_flush);         -- input FIFOs: flush on shot_start + abort
    s_out_fifo_rst_n <= i_rst_n and (not i_abort);         -- output FIFO: flush on abort ONLY
    -- NOTE: shot_start must NOT flush the output FIFO — there may be
    -- tail beats from the previous row still in transit to header_inserter.

    -- Flush output skid on face_abort only (registered, 1-cycle after
    -- overrun detection).  Uses s_face_abort_r which is only set in the
    -- overrun override (guarded by not v_row_completing), so it never
    -- fires on a normal row-completion edge.  The depth-16 FIFO between
    -- assembler and header absorbs any 1-cycle delay window.
    -- (output FIFO flush handled by s_aresetn in xpm_fifo_axis)

    -- Pipe capacity: can FSM produce? (all inputs registered → ~0.5ns)
    s_can_produce <= '1' when (s_pipe_tvalid_r = '0')
                              or (s_pipe_tvalid_r = '1' and s_pipe_tready = '1')
                     else '0';

    -- =========================================================================
    -- Input skid buffer m_ready control
    --   Assert for current chip when FSM is forwarding real data and pipe
    --   has space. All inputs registered → ~1.5ns combinational.
    -- =========================================================================
    p_in_tready : process(s_state_r, s_cur_chip_r, s_is_blank_r, s_can_produce)
    begin
        s_in_tready <= (others => '0');
        if s_state_r = ST_FORWARD and s_is_blank_r = '0'
           and s_can_produce = '1' then
            s_in_tready(to_integer(s_cur_chip_r)) <= '1';
        end if;
    end process p_in_tready;

    -- =========================================================================
    -- Main FSM process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_can_produce    : boolean;
        variable v_cur_chip       : unsigned(1 downto 0);
        variable v_is_last        : boolean;
        variable v_found          : boolean;
        variable v_blank_last     : boolean;
        variable v_done_after     : std_logic_vector(3 downto 0);
        variable v_chip_idx       : natural range 0 to 3;
        variable v_row_completing : boolean;  -- row finishes on this edge
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r         <= ST_IDLE;
                s_chip_ready_r    <= (others => '0');
                s_chip_done_r     <= (others => '0');
                s_shot_cnt_r      <= (others => '0');
                s_timeout_limit_r <= (others => '0');
                s_cur_chip_r      <= (others => '0');
                s_next_chip_r     <= (others => '0');
                s_is_blank_r      <= '0';
                s_is_last_chip_r  <= '0';
                s_blank_stop_r    <= (others => '0');
                s_blank_beat_r    <= (others => '0');
                s_last_stop_r     <= (others => '0');
                s_fwd_stop_r      <= (others => '0');
                s_fwd_beat_r      <= (others => '0');
                s_pipe_tdata_r    <= (others => '0');
                s_pipe_tvalid_r   <= '0';
                s_pipe_tlast_r    <= '0';
                s_row_done_r      <= '0';
                s_row_done_faulted_r <= '0';
                s_chip_error_r    <= (others => '0');
                s_chip_error_partial_r <= (others => '0');
                s_chip_error_blank_r   <= (others => '0');
                s_shot_overrun_r  <= '0';
                s_face_abort_r   <= '0';
                s_shot_pending_r <= '0';
                s_shot_flush_drop_r  <= '0';
                s_shot_flush_drop_mask_r <= (others => '0');
                s_shot_overrun_cnt_r <= (others => '0');
                s_fwd_stall_cnt_r    <= (others => '0');
            else
                -- Default: clear single-cycle pulses
                s_row_done_r     <= '0';
                s_shot_overrun_r <= '0';
                s_face_abort_r   <= '0';

                -- Round 5 #15 trace: if shot_start flushes input FIFOs while
                -- any of them still holds data, the old-shot tail is being
                -- dropped. Record as sticky so SW can correlate with shot
                -- timing anomalies. (abort-triggered flushes are excluded —
                -- they are explicit SW intent, not boundary drift.)
                if i_shot_start = '1' and s_in_tvalid /= "0000" then
                    s_shot_flush_drop_r <= '1';
                    -- Round 11 item 15: record WHICH chips had data pending
                    -- so SW can isolate a faulty chip stream.
                    s_shot_flush_drop_mask_r <=
                        s_shot_flush_drop_mask_r or s_in_tvalid;
                end if;

                -- Default: deassert pipe valid after handshake
                if s_pipe_tvalid_r = '1' and s_pipe_tready = '1' then
                    s_pipe_tvalid_r <= '0';
                end if;

                v_row_completing := false;

                -- =============================================================
                -- Common logic for ST_SCAN / ST_FORWARD:
                --   shot counter + tvalid latch (runs until back to ST_IDLE)
                -- =============================================================
                -- Timeout counter: chip-ready deadline only (excludes
                -- forwarding/backpressure time in ST_FORWARD).
                if s_state_r = ST_SCAN or s_state_r = ST_RESOLVE then
                    if s_shot_cnt_r /= x"FFFF" then
                        s_shot_cnt_r <= s_shot_cnt_r + 1;
                    end if;
                end if;

                -- Latch tvalid from input skid buffers (all active states)
                if s_state_r = ST_SCAN or s_state_r = ST_RESOLVE
                   or s_state_r = ST_FORWARD then
                    for i in 0 to c_N_CHIPS - 1 loop
                        if s_active_mask_r(i) = '1'
                           and s_in_tvalid(i) = '1' then
                            s_chip_ready_r(i) <= '1';
                        end if;
                    end loop;
                end if;

                v_can_produce := (s_can_produce = '1');

                case s_state_r is

                -- ==============================================================
                -- ST_IDLE: wait for shot_start
                -- ==============================================================
                when ST_IDLE =>
                    if i_shot_start = '1' or s_shot_pending_r = '1' then
                        s_shot_pending_r <= '0';  -- consume pending
                        s_timeout_limit_r <= s_timeout_limit;
                        s_chip_ready_r    <= (others => '0');
                        s_chip_done_r     <= (others => '0');
                        s_shot_cnt_r      <= (others => '0');
                        s_chip_error_r    <= (others => '0');
                s_chip_error_partial_r <= (others => '0');
                s_chip_error_blank_r   <= (others => '0');
                        s_active_mask_r   <= i_active_chip_mask;    -- latch config
                        if i_stops_per_chip >= 2 then
                            s_last_stop_r <= i_stops_per_chip(2 downto 0) - 1;
                        else
                            s_last_stop_r <= (others => '0');  -- degenerate: clamp to 1 stop
                        end if;
                        -- Runtime beats/cell for blank generation
                        case i_max_hits_cfg is
                            when "001" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(1, g_TDATA_WIDTH) - 1, 3);
                            when "010" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(2, g_TDATA_WIDTH) - 1, 3);
                            when "011" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(3, g_TDATA_WIDTH) - 1, 3);
                            when "100" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(4, g_TDATA_WIDTH) - 1, 3);
                            when "101" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(5, g_TDATA_WIDTH) - 1, 3);
                            when "110" => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(6, g_TDATA_WIDTH) - 1, 3);
                            when others => s_rt_last_beat_r <= to_unsigned(fn_beats_per_cell_rt(7, g_TDATA_WIDTH) - 1, 3);
                        end case;
                        -- Strict in-order: start from lowest active chip
                        s_next_chip_r     <= (others => '0');
                        if i_active_chip_mask = "0000" then
                            -- Zero active mask: emit row_done immediately so
                            -- upstream seq sees this shot as completed. Module
                            -- becomes self-consistent without relying on
                            -- face_seq to pre-gate zero-mask shots.
                            s_row_done_r <= '1';
                            -- Round 13 axis 1c: zero-mask is a config error
                            -- situation — mark the row as faulted.
                            s_row_done_faulted_r <= '1';
                            s_state_r    <= ST_IDLE;
                        else
                            s_state_r     <= ST_SCAN;
                        end if;
                    end if;

                -- ==============================================================
                -- ST_SCAN: strict in-order chip selection
                --   Always processes s_next_chip_r in ascending order.
                --   Skips inactive chips. Waits for current chip's data
                --   or blanks on timeout.
                --   Output order guaranteed: chip0 → chip1 → chip2 → chip3
                -- ==============================================================
                when ST_SCAN =>
                    v_chip_idx := to_integer(s_next_chip_r);

                    -- Skip inactive chips (not in mask): advance pointer
                    if s_active_mask_r(v_chip_idx) = '0'
                       or s_chip_done_r(v_chip_idx) = '1' then
                        -- This chip is inactive or already done → skip
                        -- Check if all done
                        v_done_after := s_chip_done_r;
                        v_done_after(v_chip_idx) := '1';  -- treat skipped as done
                        if (v_done_after or (not s_active_mask_r)) = "1111" then
                            -- All chips processed → row complete, go to IDLE
                            s_state_r <= ST_IDLE;
                        else
                            s_next_chip_r <= s_next_chip_r + 1;
                        end if;
                    elsif s_chip_ready_r(v_chip_idx) = '1'
                          or s_in_tvalid(v_chip_idx) = '1' then
                        -- Current chip has data ready → forward
                        s_is_blank_r <= '0';
                        s_cur_chip_r <= s_next_chip_r;
                        s_state_r    <= ST_RESOLVE;
                    elsif s_timeout_limit_r /= 0
                          and s_shot_cnt_r >= s_timeout_limit_r then
                        -- Timeout: current chip didn't deliver → blank cell
                        s_is_blank_r <= '1';
                        s_cur_chip_r <= s_next_chip_r;
                        s_chip_error_r(v_chip_idx) <= '1';
                        s_state_r    <= ST_RESOLVE;
                    elsif s_shot_cnt_r = x"FFFF" then
                        -- Hard safety cap: even if timeout_limit=0 (disabled),
                        -- don't wait forever. Force blank after ~330us @200MHz.
                        s_is_blank_r <= '1';
                        s_cur_chip_r <= s_next_chip_r;
                        s_chip_error_r(v_chip_idx) <= '1';
                        s_state_r    <= ST_RESOLVE;
                    end if;

                -- ==============================================================
                -- ST_RESOLVE: compute is_last_chip from registered s_cur_chip_r
                --   Pipeline split: priority encoder (ST_SCAN) | is_last (here)
                -- ==============================================================
                when ST_RESOLVE =>
                    v_done_after := s_chip_done_r;
                    v_done_after(to_integer(s_cur_chip_r)) := '1';
                    v_is_last := (v_done_after or (not s_active_mask_r)) = "1111";

                    if v_is_last then
                        s_is_last_chip_r <= '1';
                    else
                        s_is_last_chip_r <= '0';
                    end if;

                    s_blank_stop_r   <= (others => '0');
                    s_blank_beat_r   <= (others => '0');
                    s_fwd_stop_r     <= (others => '0');  -- reset position for new chip
                    s_fwd_beat_r     <= (others => '0');
                    s_state_r        <= ST_FORWARD;

                -- ==============================================================
                -- ST_FORWARD: produce beats into output pipe
                --   FSM produces when s_can_produce is high.
                --   Transitions happen at production time.
                -- ==============================================================
                when ST_FORWARD =>
                    if v_can_produce then

                        if s_is_blank_r = '1' then
                            -- --------------------------------------------------
                            -- Blank chip: generate beat data
                            -- --------------------------------------------------
                            s_pipe_tdata_r  <= fn_blank_beat(s_blank_beat_r, s_rt_last_beat_r, s_cur_chip_r);
                            s_pipe_tvalid_r <= '1';

                            -- Check if this is the last blank beat (s_last_stop_r pre-computed)
                            v_blank_last :=
                                (s_blank_stop_r = s_last_stop_r)
                                and (s_blank_beat_r = s_rt_last_beat_r);

                            if v_blank_last and s_is_last_chip_r = '1' then
                                s_pipe_tlast_r <= '1';  -- row tlast
                            else
                                s_pipe_tlast_r <= '0';
                            end if;

                            -- Advance blank counters
                            if s_blank_beat_r = s_rt_last_beat_r then
                                s_blank_beat_r <= (others => '0');
                                s_blank_stop_r <= s_blank_stop_r + 1;
                            else
                                s_blank_beat_r <= s_blank_beat_r + 1;
                            end if;

                            -- State transition on chip done
                            if v_blank_last then
                                s_chip_done_r(to_integer(s_cur_chip_r)) <= '1';
                                if s_is_last_chip_r = '1' then
                                    s_row_done_r     <= '1';
                                    -- Round 13 axis 1c: this branch is the
                                    -- blank-fill completion path, so the row
                                    -- contains synthetic data → faulted.
                                    s_row_done_faulted_r <= '1';
                                    v_row_completing := true;
                                    if i_shot_start = '1' then
                                        s_shot_pending_r <= '1';
                                    end if;
                                    s_state_r        <= ST_IDLE;
                                else
                                    s_next_chip_r <= s_next_chip_r + 1;
                                    s_shot_cnt_r  <= (others => '0');  -- reset timeout for next chip
                                    s_state_r     <= ST_SCAN;
                                end if;
                            end if;

                        else
                            -- --------------------------------------------------
                            -- Ready chip: forward from input skid buffer
                            -- --------------------------------------------------
                            v_chip_idx := to_integer(s_cur_chip_r);

                            if s_in_tvalid(v_chip_idx) = '1' then
                                s_pipe_tdata_r  <= s_in_tdata(v_chip_idx);
                                s_pipe_tvalid_r <= '1';
                                s_fwd_stall_cnt_r <= (others => '0');  -- Round 9 #2: reset stall counter on progress
                                -- Track (stop_idx, beat_idx) position for Q&A #36
                                -- overrun hand-off to blank mode.
                                if s_fwd_beat_r = s_rt_last_beat_r then
                                    s_fwd_beat_r <= (others => '0');
                                    s_fwd_stop_r <= s_fwd_stop_r + 1;
                                else
                                    s_fwd_beat_r <= s_fwd_beat_r + 1;
                                end if;

                                -- Detect chip last from cell_builder's tlast
                                if s_in_tlast(v_chip_idx) = '1' then
                                    if s_is_last_chip_r = '1' then
                                        s_pipe_tlast_r <= '1';  -- row tlast
                                    else
                                        s_pipe_tlast_r <= '0';
                                    end if;

                                    -- Chip done: transition immediately
                                    s_chip_done_r(v_chip_idx) <= '1';
                                    if s_is_last_chip_r = '1' then
                                        s_row_done_r     <= '1';
                                        -- Round 13 axis 1c: if any chip had
                                        -- partial/blank error this shot, the
                                        -- row we're completing is faulted.
                                        if s_chip_error_partial_r /= "0000"
                                           or s_chip_error_blank_r /= "0000" then
                                            s_row_done_faulted_r <= '1';
                                        end if;
                                        v_row_completing := true;
                                        if i_shot_start = '1' then
                                            s_shot_pending_r <= '1';
                                        end if;
                                        s_state_r        <= ST_IDLE;
                                    else
                                        s_next_chip_r <= s_next_chip_r + 1;
                                        s_shot_cnt_r  <= (others => '0');  -- reset timeout for next chip
                                        s_state_r     <= ST_SCAN;
                                    end if;
                                else
                                    s_pipe_tlast_r <= '0';
                                end if;
                            else
                                -- Round 9 #2: intra-forward stall watchdog.
                                -- Chip data started but now tvalid is low. On
                                -- watchdog expiry, hand off to blank mode from
                                -- the current (fwd_stop, fwd_beat) position
                                -- and flag chip_error. Blank path then finishes
                                -- the chip and normal transition proceeds.
                                if s_fwd_stall_cnt_r = x"FFFF" then
                                    s_fwd_stall_cnt_r    <= (others => '0');
                                    s_is_blank_r         <= '1';
                                    s_blank_stop_r       <= s_fwd_stop_r;
                                    s_blank_beat_r       <= s_fwd_beat_r;
                                    s_chip_error_r(v_chip_idx) <= '1';
                                else
                                    s_fwd_stall_cnt_r <= s_fwd_stall_cnt_r + 1;
                                end if;
                            end if;

                        end if;  -- blank / real
                    end if;  -- v_can_produce

                end case;

                -- =============================================================
                -- shot_start override: BLANK-FILL current line (Q&A #36, c-simplified)
                --
                -- The current shot's line is completed by generating blank
                -- (error_fill=1) cells for the remainder of the chip in progress
                -- and for all not-yet-started chips. Line size is preserved so
                -- VDMA frame alignment stays intact (SW parses by fixed offset).
                --
                -- s_fwd_beat_cnt_r gives the exact position where overrun hit
                -- in the current chip's real-chip forward; blank mode resumes
                -- from that (stop_idx, beat_idx_in_cell) to backfill cleanly.
                --
                -- Subsequent chips enter blank via ST_SCAN's hard-cap path:
                -- setting s_shot_cnt_r = x"FFFF" forces ST_SCAN to immediately
                -- declare the next chip timed out → ST_RESOLVE → ST_FORWARD blank.
                --
                -- NOTE: face_abort is NOT pulsed here. The line is completed
                -- in-place; header_inserter sees a normal tlast and the face
                -- continues naturally. shot_pending latches the new shot so
                -- ST_IDLE picks it up as the next line.
                -- =============================================================
                if i_shot_start = '1' and s_state_r /= ST_IDLE
                                       and not v_row_completing then
                    s_shot_overrun_r <= '1';
                    s_shot_pending_r <= '1';  -- new shot becomes next line

                    -- Round 5 #16 trace: wrap-counter for blank-fill invocations
                    s_shot_overrun_cnt_r <= s_shot_overrun_cnt_r + 1;

                    -- Round 10 #6: mark only ACTIVE chips that haven't finished.
                    -- The previous OR with plain (not s_chip_done_r) flagged
                    -- inactive chips (which never had data) as error too —
                    -- making o_chip_error_flags inconsistent with the active
                    -- mask. Masking by s_active_mask_r keeps the error bits
                    -- aligned with the chips that were actually expected to
                    -- produce data this shot.
                    s_chip_error_r <= s_chip_error_r or
                                      (s_active_mask_r and (not s_chip_done_r));

                    -- Round 12 #18: partial vs blank classification.
                    -- "partial" = the chip that was mid-ST_FORWARD when the
                    --             new shot hit (at most 1 bit set per event).
                    -- "blank"   = active chips not-yet-done other than partial
                    --             (i.e. chips that would have been reached
                    --             next but are now force-blanked).
                    if s_state_r = ST_FORWARD and s_is_blank_r = '0' then
                        -- mark current chip as partial
                        for k in 0 to c_N_CHIPS - 1 loop
                            if k = to_integer(s_cur_chip_r) then
                                s_chip_error_partial_r(k) <= '1';
                            end if;
                        end loop;
                        -- blank mask = active AND not done AND NOT current chip
                        for k in 0 to c_N_CHIPS - 1 loop
                            if s_active_mask_r(k) = '1'
                               and s_chip_done_r(k) = '0'
                               and k /= to_integer(s_cur_chip_r) then
                                s_chip_error_blank_r(k) <= '1';
                            end if;
                        end loop;
                    else
                        -- Not mid-forward: all not-done active chips are blank
                        s_chip_error_blank_r <= s_chip_error_blank_r or
                                                (s_active_mask_r and (not s_chip_done_r));
                    end if;

                    -- Force upcoming chips (after current) into blank mode via
                    -- ST_SCAN's hard safety cap.
                    s_shot_cnt_r <= x"FFFF";

                    -- If we were mid ST_FORWARD real-chip, resume as blank
                    -- from the current position so the chip's expected beat
                    -- count stays exact.
                    if s_state_r = ST_FORWARD and s_is_blank_r = '0' then
                        s_is_blank_r   <= '1';
                        s_blank_stop_r <= s_fwd_stop_r;
                        s_blank_beat_r <= s_fwd_beat_r;
                    end if;
                end if;

                -- =============================================================
                -- External abort (cmd_stop / cmd_soft_reset from top).
                -- Flush everything and go to ST_IDLE.  Highest priority.
                -- =============================================================
                if i_abort = '1' then
                    s_chip_ready_r    <= (others => '0');
                    s_chip_done_r     <= (others => '0');
                    s_chip_error_r    <= (others => '0');
                s_chip_error_partial_r <= (others => '0');
                s_chip_error_blank_r   <= (others => '0');
                    s_pipe_tvalid_r   <= '0';
                    s_pipe_tlast_r    <= '0';
                    s_shot_pending_r  <= '0';
                    -- NOTE: do NOT set s_face_abort_r here.
                    -- o_face_abort is only for self-overrun notification.
                    -- External abort (i_abort) comes from top's s_pipeline_abort
                    -- which top already uses directly for frame_done_both.
                    -- Setting face_abort_r here would create a feedback loop:
                    -- A.o_face_abort → top → B.i_abort → B.o_face_abort → top → A.i_abort → ...
                    s_state_r         <= ST_IDLE;
                end if;
            end if;
        end if;
    end process p_main;

    -- =========================================================================
    -- Status output assignments (registered)
    -- =========================================================================
    o_row_done         <= s_row_done_r;
    o_row_done_faulted <= s_row_done_faulted_r;
    o_chip_error_flags <= s_chip_error_r;
    o_chip_error_partial <= s_chip_error_partial_r;
    o_chip_error_blank   <= s_chip_error_blank_r;
    o_shot_overrun     <= s_shot_overrun_r;
    -- #18: o_face_abort is retained as a port for backward compatibility but
    -- s_face_abort_r is never asserted after Round 4 (c-simplified overrun).
    -- Overrun no longer triggers face_abort → pipeline_abort; the tight
    -- 1-cycle feedback path concern from #18 is therefore moot.
    o_face_abort       <= s_face_abort_r;
    o_idle             <= '1' when s_state_r = ST_IDLE else '0';
    o_shot_flush_drop      <= s_shot_flush_drop_r;
    o_shot_flush_drop_mask <= s_shot_flush_drop_mask_r;
    o_shot_overrun_count <= s_shot_overrun_cnt_r;

end architecture rtl;
