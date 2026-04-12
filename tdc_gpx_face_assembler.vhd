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
--   FCFS (First-Come First-Served: 먼저 준비된 chip부터 출력) scheduling:
--     Ready chips are forwarded in arrival order (lowest index breaks ties).
--     While forwarding one chip, remaining chips continue accumulating
--     in their cell_builders, hiding drain latency.
--     Each cell carries a chip_id tag (beat 4, bits [9:8]) so downstream
--     can identify chip origin regardless of output order.
--
--   Per-shot flow:
--     1. shot_start → ST_SCAN
--     2. ST_SCAN: priority encode undone chip (ready first, timeout blank)
--     3. ST_RESOLVE: compute is_last_chip from registered result (1 clk)
--     4. ST_FORWARD: produce beats into output skid buffer
--     5. Chip done → mark s_chip_done_r, back to ST_SCAN
--     6. All active chips done → row_done, ST_IDLE
--
--   Packed Row: only active chips contribute rows. Chip order within
--   the row is non-deterministic (FCFS). SW uses chip_id tag to parse.
--
--   Timing closure (200MHz+):
--     5 skid buffers (4 input + 1 output) provide fully registered
--     ready paths at all module boundaries.
--     ST_SCAN/ST_RESOLVE pipeline split: priority encoder (ST_SCAN) and
--     is_last_chip computation (ST_RESOLVE) run in separate clock cycles.
--     Pre-computed s_last_stop_r eliminates runtime subtraction.
--
--   All outputs are registered (module boundary = FF).
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_face_assembler is
    generic (
        g_ALU_PULSE_CLKS     : natural := 4
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- 4 chip cell AXI-Stream inputs (from cell_builders)
        i_s_axis_tdata       : in  t_axis_tdata_array;
        i_s_axis_tvalid      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_s_axis_tlast       : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_s_axis_tready      : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Shot control
        i_shot_start         : in  std_logic;

        -- Configuration (latched at packet_start)
        i_active_chip_mask   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_stops_per_chip     : in  unsigned(3 downto 0);
        -- Scan timeout: max clock cycles before declaring a chip's cell blank.
        -- SW must bake in all margins (bus roundtrip + drain + ALU service).
        -- 0 = disabled (no timeout, wait indefinitely for chip data).
        i_max_range_clks     : in  unsigned(15 downto 0);

        -- AXI-Stream master (packed row output)
        o_m_axis_tdata       : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid      : out std_logic;
        o_m_axis_tlast       : out std_logic;
        i_m_axis_tready      : in  std_logic;

        -- Status
        o_row_done           : out std_logic;    -- 1-clk pulse: packed row complete
        o_chip_error_flags   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_shot_overrun       : out std_logic;    -- 1-clk pulse: shot truncated (was not idle)
        o_face_abort         : out std_logic;    -- 1-clk pulse: face aborted (overrun → ST_IDLE)
        o_idle               : out std_logic     -- '1' when FSM is in ST_IDLE
    );
end entity tdc_gpx_face_assembler;

architecture rtl of tdc_gpx_face_assembler is

    -- =========================================================================
    -- Skid buffer data width: tdata + tlast bundled
    -- =========================================================================
    constant c_SKID_WIDTH : natural := c_TDATA_WIDTH + 1;

    -- Bundle/unbundle types for input skid buffer array
    type t_skid_data_array is array (0 to c_N_CHIPS - 1)
        of std_logic_vector(c_SKID_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Input skid buffer interface signals (from skid_buffer outputs)
    -- =========================================================================
    signal s_in_tdata    : t_axis_tdata_array;
    signal s_in_tvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_in_tlast    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_in_tready   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_in_s_bundle : t_skid_data_array;  -- skid_buffer s_data input (tdata & tlast)
    signal s_in_bundle   : t_skid_data_array;  -- skid_buffer m_data output

    -- =========================================================================
    -- Output pipe signals (FSM → output skid buffer)
    -- =========================================================================
    signal s_pipe_tdata_r  : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_pipe_tvalid_r : std_logic := '0';
    signal s_pipe_tlast_r  : std_logic := '0';
    signal s_pipe_tready   : std_logic;  -- from output skid buffer (registered)
    signal s_pipe_bundle   : std_logic_vector(c_SKID_WIDTH - 1 downto 0);
    signal s_out_bundle    : std_logic_vector(c_SKID_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Synchronous flush for skid buffers (on shot_start)
    -- =========================================================================
    signal s_flush        : std_logic;  -- input skid flush (every shot_start)
    signal s_abort_flush  : std_logic;  -- output skid flush (combinational, same edge as overrun)

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

    -- Combinational timeout limit (latched into s_timeout_limit_r on shot_start)
    signal s_timeout_limit   : unsigned(15 downto 0);

    -- =========================================================================
    -- Forwarding state
    -- =========================================================================
    signal s_cur_chip_r      : unsigned(1 downto 0) := (others => '0');
    signal s_is_blank_r      : std_logic := '0';   -- current chip is timed out
    signal s_is_last_chip_r  : std_logic := '0';   -- no more undone active chips after this

    -- Blank beat generation counters
    signal s_blank_stop_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7
    signal s_blank_beat_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7
    signal s_last_stop_r     : unsigned(2 downto 0) := (others => '0');  -- pre-computed: stops-1

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
    signal s_shot_overrun_r  : std_logic := '0';
    signal s_face_abort_r    : std_logic := '0';

    -- =========================================================================
    -- Blank beat data: all zeros except metadata beat has error_fill + chip_id
    -- Metadata beat index = c_META_BEAT_IDX (auto-derived from c_MAX_HITS_PER_STOP)
    -- =========================================================================
    function fn_blank_beat(
        beat_idx : unsigned(2 downto 0);
        chip_id  : unsigned(1 downto 0)
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    begin
        v_result := (others => '0');
        if to_integer(beat_idx) = c_META_BEAT_IDX then
            v_result(10)          := '1';   -- error_fill
            v_result(9 downto 8)  := std_logic_vector(chip_id);
        end if;
        return v_result;
    end function;

begin

    -- =========================================================================
    -- Input skid buffers (×4): registered tready to cell_builders
    -- =========================================================================
    gen_in_fifo : for i in 0 to c_N_CHIPS - 1 generate
        s_in_s_bundle(i) <= i_s_axis_tdata(i) & i_s_axis_tlast(i);
        u_fifo_in : entity work.tdc_gpx_sync_fifo
            generic map (
                g_DATA_WIDTH => c_SKID_WIDTH,   -- 33 bits (tdata + tlast)
                g_DEPTH      => 16,
                g_LOG2_DEPTH => 4,
                g_IN_REG     => false,          -- no extra skid, FIFO is enough
                g_OUT_REG    => false
            )
            port map (
                i_clk     => i_clk,
                i_rst_n   => i_rst_n,
                i_flush   => s_flush,
                i_s_valid => i_s_axis_tvalid(i),
                o_s_ready => o_s_axis_tready(i),
                i_s_data  => s_in_s_bundle(i),
                o_m_valid => s_in_tvalid(i),
                i_m_ready => s_in_tready(i),
                o_m_data  => s_in_bundle(i)
            );
        -- Unbundle: tdata(31:0) & tlast(0)
        s_in_tdata(i) <= s_in_bundle(i)(c_SKID_WIDTH - 1 downto 1);
        s_in_tlast(i) <= s_in_bundle(i)(0);
    end generate gen_in_fifo;

    -- =========================================================================
    -- Output skid buffer (×1): registered tready from downstream
    -- =========================================================================
    s_pipe_bundle <= s_pipe_tdata_r & s_pipe_tlast_r;

    u_skid_out : entity work.tdc_gpx_skid_buffer
        generic map (g_DATA_WIDTH => c_SKID_WIDTH)
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_flush   => s_abort_flush,    -- Flush output skid on overrun
                                          -- (combinational, same edge as overrun
                                          -- detection — no 1-cycle leak window).
            i_s_valid => s_pipe_tvalid_r,
            o_s_ready => s_pipe_tready,
            i_s_data  => s_pipe_bundle,
            o_m_valid => o_m_axis_tvalid,
            i_m_ready => i_m_axis_tready,
            o_m_data  => s_out_bundle
        );

    o_m_axis_tdata <= s_out_bundle(c_SKID_WIDTH - 1 downto 1);
    o_m_axis_tlast <= s_out_bundle(0);

    -- =========================================================================
    -- Concurrent signals
    -- =========================================================================

    -- Timeout limit: max_range_clks only (drain/ALU margins TBD after bench)
    s_timeout_limit <= i_max_range_clks;

    -- Flush input skid buffers on every shot_start
    s_flush <= i_shot_start;

    -- Flush output skid ONLY on overrun (shot_start while not idle and not
    -- completing a row on this edge).  Combinational so it acts on the SAME
    -- edge as the overrun, preventing the 1-cycle stale-beat leak.
    s_abort_flush <= i_shot_start when s_state_r /= ST_IDLE else '0';

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
                s_is_blank_r      <= '0';
                s_is_last_chip_r  <= '0';
                s_blank_stop_r    <= (others => '0');
                s_blank_beat_r    <= (others => '0');
                s_last_stop_r     <= (others => '0');
                s_pipe_tdata_r    <= (others => '0');
                s_pipe_tvalid_r   <= '0';
                s_pipe_tlast_r    <= '0';
                s_row_done_r      <= '0';
                s_chip_error_r    <= (others => '0');
                s_shot_overrun_r  <= '0';
                s_face_abort_r   <= '0';
            else
                -- Default: clear single-cycle pulses
                s_row_done_r     <= '0';
                s_shot_overrun_r <= '0';
                s_face_abort_r   <= '0';

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
                    if i_shot_start = '1' then
                        s_timeout_limit_r <= s_timeout_limit;
                        s_chip_ready_r    <= (others => '0');
                        s_chip_done_r     <= (others => '0');
                        s_shot_cnt_r      <= (others => '0');
                        s_chip_error_r    <= (others => '0');
                        s_active_mask_r   <= i_active_chip_mask;    -- latch config
                        s_last_stop_r     <= i_stops_per_chip(2 downto 0) - 1;
                        s_state_r         <= ST_SCAN;
                    end if;

                -- ==============================================================
                -- ST_SCAN: find any undone active chip to forward
                --   Priority 1: ready chip (tvalid latched, lowest index)
                --   Priority 2: timeout → blank any undone chip
                --   Result registered → ST_RESOLVE computes is_last_chip
                -- ==============================================================
                when ST_SCAN =>
                    -- Priority 1: find undone + ready chip
                    v_found := false;
                    v_cur_chip := (others => '0');
                    for i in 0 to 3 loop
                        if s_active_mask_r(i) = '1'
                           and s_chip_done_r(i) = '0'
                           and (s_chip_ready_r(i) = '1'
                                or s_in_tvalid(i) = '1')
                           and not v_found then
                            v_cur_chip := to_unsigned(i, 2);
                            v_found := true;
                        end if;
                    end loop;

                    if v_found then
                        s_is_blank_r <= '0';
                        s_cur_chip_r <= v_cur_chip;
                        s_state_r    <= ST_RESOLVE;
                    elsif s_timeout_limit_r /= 0 and s_shot_cnt_r >= s_timeout_limit_r then
                        -- Priority 2: timeout → pick any undone chip
                        for i in 0 to 3 loop
                            if s_active_mask_r(i) = '1'
                               and s_chip_done_r(i) = '0'
                               and not v_found then
                                v_cur_chip := to_unsigned(i, 2);
                                v_found := true;
                            end if;
                        end loop;
                        if v_found then
                            s_is_blank_r <= '1';
                            s_cur_chip_r <= v_cur_chip;
                            s_chip_error_r(to_integer(v_cur_chip)) <= '1';
                            s_state_r    <= ST_RESOLVE;
                        end if;
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

                    s_blank_stop_r <= (others => '0');
                    s_blank_beat_r <= (others => '0');
                    s_state_r      <= ST_FORWARD;

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
                            s_pipe_tdata_r  <= fn_blank_beat(s_blank_beat_r, s_cur_chip_r);
                            s_pipe_tvalid_r <= '1';

                            -- Check if this is the last blank beat (s_last_stop_r pre-computed)
                            v_blank_last :=
                                (s_blank_stop_r = s_last_stop_r)
                                and (s_blank_beat_r = c_BEATS_PER_CELL - 1);

                            if v_blank_last and s_is_last_chip_r = '1' then
                                s_pipe_tlast_r <= '1';  -- row tlast
                            else
                                s_pipe_tlast_r <= '0';
                            end if;

                            -- Advance blank counters
                            if s_blank_beat_r = c_BEATS_PER_CELL - 1 then
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
                                    v_row_completing := true;
                                    s_state_r        <= ST_IDLE;
                                else
                                    s_state_r    <= ST_SCAN;
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
                                        v_row_completing := true;
                                        s_state_r        <= ST_IDLE;
                                    else
                                        s_state_r    <= ST_SCAN;
                                    end if;
                                else
                                    s_pipe_tlast_r <= '0';
                                end if;
                            end if;

                        end if;  -- blank / real
                    end if;  -- v_can_produce

                end case;

                -- =============================================================
                -- shot_start override: ABORT the current face.
                --
                -- Partial beats already consumed by header_inserter cannot
                -- be reclaimed, so blank-filling would produce an oversized
                -- line.  Instead we abort immediately: go to ST_IDLE, flush
                -- the output skid (discard any partial beats still in it),
                -- and pulse o_face_abort.  Top treats this as frame_done for
                -- the frame_done_both combiner, closing the face.  SW uses
                -- shot_overrun status to identify and discard corrupted frames.
                -- =============================================================
                if i_shot_start = '1' and s_state_r /= ST_IDLE
                                       and not v_row_completing then
                    s_chip_ready_r    <= (others => '0');
                    s_chip_done_r     <= (others => '0');
                    s_chip_error_r    <= (others => '0');
                    s_shot_overrun_r  <= '1';
                    s_face_abort_r    <= '1';
                    s_pipe_tvalid_r   <= '0';
                    s_pipe_tlast_r    <= '0';
                    s_state_r         <= ST_IDLE;
                end if;
            end if;
        end if;
    end process p_main;

    -- =========================================================================
    -- Status output assignments (registered)
    -- =========================================================================
    o_row_done         <= s_row_done_r;
    o_chip_error_flags <= s_chip_error_r;
    o_shot_overrun     <= s_shot_overrun_r;
    o_face_abort       <= s_face_abort_r;
    o_idle             <= '1' when s_state_r = ST_IDLE else '0';

end architecture rtl;
