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
--     2. ST_SCAN: find any undone chip with tvalid (ready first),
--        or blank on timeout
--     3. ST_FORWARD: produce beats into output skid buffer
--     4. Chip done → mark s_chip_done_r, back to ST_SCAN
--     5. All active chips done → row_done, ST_IDLE
--
--   Packed Row: only active chips contribute rows. Chip order within
--   the row is non-deterministic (FCFS). SW uses chip_id tag to parse.
--
--   Timing closure (200MHz+):
--     Output skid buffer (2-deep) breaks downstream tready from reaching
--     upstream cell_builder tready. All critical paths are ≤ 2.5ns:
--       tready to cell_builder: f(state, cur_chip, is_blank, can_produce)
--         → all registered inputs, ~1.5ns combinational
--       data MUX: i_s_axis_tdata(cur_chip_r) → 4:1 MUX → skid/out reg
--         → ~1.5ns combinational
--
--   All outputs are registered (module boundary = FF).
--   tready to cell_builders is combinational from registered signals only
--   (no downstream tready dependency).
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
        i_max_range_clks     : in  unsigned(15 downto 0);   -- capture timeout (SW: 2*dist/c*fclk)
        i_bus_ticks          : in  unsigned(2 downto 0);    -- capture timeout
        i_bus_clk_div        : in  unsigned(7 downto 0);    -- capture timeout

        -- AXI-Stream master (packed row output)
        o_m_axis_tdata       : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid      : out std_logic;
        o_m_axis_tlast       : out std_logic;
        i_m_axis_tready      : in  std_logic;

        -- Status
        o_row_done           : out std_logic;    -- 1-clk pulse: packed row complete
        o_chip_error_flags   : out std_logic_vector(c_N_CHIPS - 1 downto 0)
    );
end entity tdc_gpx_face_assembler;

architecture rtl of tdc_gpx_face_assembler is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_SCAN, ST_FORWARD);
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

    -- =========================================================================
    -- Output skid buffer (2-deep)
    --   Breaks downstream tready from upstream tready path.
    --   FSM produces into {out, skid}. Downstream drains from out.
    --   can_produce = NOT (out_valid AND skid_valid).
    -- =========================================================================
    signal s_out_tdata_r     : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_out_tvalid_r    : std_logic := '0';
    signal s_out_tlast_r     : std_logic := '0';

    signal s_skid_tdata_r    : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_skid_tvalid_r   : std_logic := '0';
    signal s_skid_tlast_r    : std_logic := '0';

    -- Pipe capacity: can FSM produce a new beat?
    -- Depends on registered signals only — no downstream tready dependency.
    signal s_can_produce     : std_logic;

    -- =========================================================================
    -- Status registers
    -- =========================================================================
    signal s_row_done_r      : std_logic := '0';
    signal s_chip_error_r    : std_logic_vector(3 downto 0) := (others => '0');

    -- =========================================================================
    -- Blank beat data: all zeros except beat 4 has error_fill + chip_id
    -- =========================================================================
    function fn_blank_beat(
        beat_idx : unsigned(2 downto 0);
        chip_id  : unsigned(1 downto 0)
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    begin
        v_result := (others => '0');
        if to_integer(beat_idx) = 4 then
            v_result(10)          := '1';   -- error_fill
            v_result(9 downto 8)  := std_logic_vector(chip_id);
        end if;
        return v_result;
    end function;

begin

    -- Timeout limit: max_range_clks only (drain/ALU margins TBD after bench)
    s_timeout_limit <= i_max_range_clks;

    -- Pipe has space when not FULL (both out and skid occupied)
    s_can_produce <= '0' when (s_out_tvalid_r = '1' and s_skid_tvalid_r = '1')
                     else '1';

    -- =========================================================================
    -- Combinational tready to cell_builders
    --   Depends ONLY on registered signals (state, cur_chip, is_blank,
    --   can_produce). NO dependency on downstream i_m_axis_tready.
    --   This breaks the combinational tready chain for timing closure.
    -- =========================================================================
    p_tready : process(s_state_r, s_cur_chip_r, s_is_blank_r, s_can_produce)
    begin
        o_s_axis_tready <= (others => '0');
        if s_state_r = ST_FORWARD and s_is_blank_r = '0'
           and s_can_produce = '1' then
            o_s_axis_tready(to_integer(s_cur_chip_r)) <= '1';
        end if;
    end process p_tready;

    -- =========================================================================
    -- Main process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_can_produce    : boolean;
        variable v_out_consumed   : boolean;
        variable v_cur_chip       : unsigned(1 downto 0);
        variable v_is_last        : boolean;
        variable v_found          : boolean;
        variable v_blank_last     : boolean;
        variable v_done_after     : std_logic_vector(3 downto 0);
        variable v_new_data       : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        variable v_new_last       : std_logic;
        variable v_producing      : boolean;
        variable v_chip_idx       : natural range 0 to 3;
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
                s_out_tdata_r     <= (others => '0');
                s_out_tvalid_r    <= '0';
                s_out_tlast_r     <= '0';
                s_skid_tdata_r    <= (others => '0');
                s_skid_tvalid_r   <= '0';
                s_skid_tlast_r    <= '0';
                s_row_done_r      <= '0';
                s_chip_error_r    <= (others => '0');
            else
                -- Default: clear single-cycle pulses
                s_row_done_r <= '0';

                -- =============================================================
                -- Skid buffer drain: downstream consuming output register
                --   This block runs FIRST. FSM produce (below) may OVERRIDE
                --   these assignments (last assignment wins in VHDL process).
                -- =============================================================
                v_out_consumed := (s_out_tvalid_r = '1' and i_m_axis_tready = '1');

                if v_out_consumed then
                    if s_skid_tvalid_r = '1' then
                        -- Promote skid to output
                        s_out_tdata_r   <= s_skid_tdata_r;
                        s_out_tlast_r   <= s_skid_tlast_r;
                        s_skid_tvalid_r <= '0';
                    else
                        -- Output drained, nothing in skid
                        s_out_tvalid_r <= '0';
                        s_out_tlast_r  <= '0';
                    end if;
                end if;

                -- =============================================================
                -- Common logic for ST_SCAN / ST_FORWARD:
                --   shot counter + tvalid latch (runs until back to ST_IDLE)
                -- =============================================================
                if s_state_r = ST_SCAN or s_state_r = ST_FORWARD then
                    -- Increment shot counter (saturate at max)
                    if s_shot_cnt_r /= x"FFFF" then
                        s_shot_cnt_r <= s_shot_cnt_r + 1;
                    end if;
                    -- Latch tvalid for all active chips
                    for i in 0 to c_N_CHIPS - 1 loop
                        if i_active_chip_mask(i) = '1'
                           and i_s_axis_tvalid(i) = '1' then
                            s_chip_ready_r(i) <= '1';
                        end if;
                    end loop;
                end if;

                v_can_produce := (s_can_produce = '1');
                v_producing   := false;
                v_new_data    := (others => '0');
                v_new_last    := '0';

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
                        s_state_r         <= ST_SCAN;
                    end if;

                -- ==============================================================
                -- ST_SCAN: find any undone active chip to forward
                --   Priority 1: ready chip (tvalid latched, lowest index)
                --   Priority 2: timeout → blank any undone chip
                -- ==============================================================
                when ST_SCAN =>
                    -- Priority 1: find undone + ready chip
                    v_found := false;
                    v_cur_chip := (others => '0');
                    for i in 0 to 3 loop
                        if i_active_chip_mask(i) = '1'
                           and s_chip_done_r(i) = '0'
                           and (s_chip_ready_r(i) = '1'
                                or i_s_axis_tvalid(i) = '1')
                           and not v_found then
                            v_cur_chip := to_unsigned(i, 2);
                            v_found := true;
                        end if;
                    end loop;

                    if v_found then
                        s_is_blank_r <= '0';
                    elsif s_shot_cnt_r >= s_timeout_limit_r then
                        -- Priority 2: timeout → pick any undone chip
                        for i in 0 to 3 loop
                            if i_active_chip_mask(i) = '1'
                               and s_chip_done_r(i) = '0'
                               and not v_found then
                                v_cur_chip := to_unsigned(i, 2);
                                v_found := true;
                            end if;
                        end loop;
                        if v_found then
                            s_is_blank_r <= '1';
                            s_chip_error_r(to_integer(v_cur_chip)) <= '1';
                        end if;
                    end if;

                    if v_found then
                        s_cur_chip_r <= v_cur_chip;

                        -- Is this the last undone active chip?
                        v_done_after := s_chip_done_r;
                        v_done_after(to_integer(v_cur_chip)) := '1';
                        v_is_last := (v_done_after or (not i_active_chip_mask)) = "1111";

                        if v_is_last then
                            s_is_last_chip_r <= '1';
                        else
                            s_is_last_chip_r <= '0';
                        end if;

                        s_blank_stop_r <= (others => '0');
                        s_blank_beat_r <= (others => '0');
                        s_state_r      <= ST_FORWARD;
                    end if;

                -- ==============================================================
                -- ST_FORWARD: produce beats into output skid buffer
                --   Transitions happen at production time (not consumption).
                --   Real data: forward from cell_builder when tvalid & can_produce.
                --   Blank data: generate when can_produce.
                -- ==============================================================
                when ST_FORWARD =>
                    if v_can_produce then

                        if s_is_blank_r = '1' then
                            -- --------------------------------------------------
                            -- Blank chip: generate beat data
                            -- --------------------------------------------------
                            v_producing := true;
                            v_new_data  := fn_blank_beat(s_blank_beat_r, s_cur_chip_r);

                            -- Check if this is the last blank beat
                            v_blank_last :=
                                (s_blank_stop_r = i_stops_per_chip(2 downto 0) - 1)
                                and (s_blank_beat_r = c_BEATS_PER_CELL - 1);

                            if v_blank_last and s_is_last_chip_r = '1' then
                                v_new_last := '1';  -- row tlast
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
                                    s_row_done_r <= '1';
                                    s_state_r    <= ST_IDLE;
                                else
                                    s_state_r    <= ST_SCAN;
                                end if;
                            end if;

                        else
                            -- --------------------------------------------------
                            -- Ready chip: forward from cell_builder
                            --   tready asserted by p_tready (combinational,
                            --   no downstream dependency)
                            -- --------------------------------------------------
                            v_chip_idx := to_integer(s_cur_chip_r);

                            if i_s_axis_tvalid(v_chip_idx) = '1' then
                                v_producing := true;
                                v_new_data  := i_s_axis_tdata(v_chip_idx);

                                -- Detect chip last from cell_builder's tlast
                                if i_s_axis_tlast(v_chip_idx) = '1' then
                                    if s_is_last_chip_r = '1' then
                                        v_new_last := '1';  -- row tlast
                                    end if;

                                    -- Chip done: transition immediately
                                    s_chip_done_r(v_chip_idx) <= '1';
                                    if s_is_last_chip_r = '1' then
                                        s_row_done_r <= '1';
                                        s_state_r    <= ST_IDLE;
                                    else
                                        s_state_r    <= ST_SCAN;
                                    end if;
                                end if;
                            end if;

                        end if;  -- blank / real
                    end if;  -- v_can_produce

                end case;

                -- =============================================================
                -- Skid buffer write: FSM produced a beat → insert into pipe
                --   OVERRIDES drain assignments above (last assignment wins).
                --   Write to output register if available, else to skid.
                -- =============================================================
                if v_producing then
                    if s_out_tvalid_r = '0' or v_out_consumed then
                        -- Output slot available (empty or being drained)
                        s_out_tdata_r  <= v_new_data;
                        s_out_tvalid_r <= '1';
                        s_out_tlast_r  <= v_new_last;
                    else
                        -- Output occupied: store in skid
                        s_skid_tdata_r  <= v_new_data;
                        s_skid_tvalid_r <= '1';
                        s_skid_tlast_r  <= v_new_last;
                    end if;
                end if;

                -- =============================================================
                -- shot_start override for ST_SCAN/ST_FORWARD only.
                -- Abort current operation, flush pipe, restart.
                -- =============================================================
                if i_shot_start = '1' and s_state_r /= ST_IDLE then
                    s_timeout_limit_r <= s_timeout_limit;
                    s_chip_ready_r    <= (others => '0');
                    s_chip_done_r     <= (others => '0');
                    s_shot_cnt_r      <= (others => '0');
                    s_chip_error_r    <= (others => '0');
                    s_out_tvalid_r    <= '0';
                    s_out_tlast_r     <= '0';
                    s_skid_tvalid_r   <= '0';
                    s_skid_tlast_r    <= '0';
                    s_state_r         <= ST_SCAN;
                end if;
            end if;
        end if;
    end process p_main;

    -- =========================================================================
    -- Output assignments (all from registered skid buffer)
    -- =========================================================================
    o_m_axis_tdata     <= s_out_tdata_r;
    o_m_axis_tvalid    <= s_out_tvalid_r;
    o_m_axis_tlast     <= s_out_tlast_r;
    o_row_done         <= s_row_done_r;
    o_chip_error_flags <= s_chip_error_r;

end architecture rtl;
