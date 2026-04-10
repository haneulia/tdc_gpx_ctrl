-- =============================================================================
-- tdc_gpx_face_assembler.vhd
-- TDC-GPX Controller - Face Assembler (Packed Row)
-- =============================================================================
--
-- Purpose:
--   Collects 4 chip cells (AXI-Stream) and concatenates active chips into
--   a single packed line (VDMA line). Inactive chips are skipped.
--   Timed-out chips receive blank cells with error_fill=1.
--
--   Per-shot flow:
--     1. shot_start → start per-chip timeout counters
--     2. Wait until all active chips have tvalid (ready) or timeout
--     3. Forward cells in ascending chip order to output AXI-Stream
--        (blank data for timed-out chips)
--     4. Assert line_done when the packed line is complete
--
--   Packed Row policy: only active chips contribute rows.
--     active_mask=0b1010, stops_per_chip=4 → rows_per_face = 2×4 = 8
--
--   All outputs are registered (module boundary = FF).
--   tready to cell_builders is combinational (internal interface).
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
        i_bus_ticks          : in  unsigned(2 downto 0);
        i_bus_clk_div        : in  unsigned(7 downto 0);

        -- AXI-Stream master (packed line output)
        o_m_axis_tdata       : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid      : out std_logic;
        o_m_axis_tlast       : out std_logic;
        i_m_axis_tready      : in  std_logic;

        -- Status
        o_line_done          : out std_logic;    -- 1-clk pulse: packed line complete
        o_chip_error_flags   : out std_logic_vector(c_N_CHIPS - 1 downto 0)
    );
end entity tdc_gpx_face_assembler;

architecture rtl of tdc_gpx_face_assembler is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_WAIT, ST_FORWARD);
    signal s_state_r : t_state := ST_IDLE;

    -- =========================================================================
    -- Per-chip status
    -- =========================================================================
    signal s_chip_ready_r   : std_logic_vector(3 downto 0) := (others => '0');
    signal s_chip_timeout_r : std_logic_vector(3 downto 0) := (others => '0');

    -- Timeout counter (16-bit): max_range + drain_max + alu + 1
    type t_timeout_cnt_array is array(0 to c_N_CHIPS - 1) of unsigned(15 downto 0);
    signal s_timeout_cnt_r   : t_timeout_cnt_array := (others => (others => '0'));
    signal s_timeout_limit_r : unsigned(15 downto 0) := (others => '0');

    -- Combinational timeout limit (latched into s_timeout_limit_r on shot_start)
    signal s_timeout_limit   : unsigned(15 downto 0);

    -- =========================================================================
    -- Forwarding state
    -- =========================================================================
    signal s_cur_chip_r      : unsigned(1 downto 0) := (others => '0');
    signal s_is_blank_r      : std_logic := '0';   -- current chip is timed out
    signal s_is_last_chip_r  : std_logic := '0';   -- no more active chips after this

    -- Blank beat generation counters
    signal s_blank_stop_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7
    signal s_blank_beat_r    : unsigned(2 downto 0) := (others => '0');  -- 0..7

    -- Is the current output beat the last of the current chip?
    signal s_chip_last_r     : std_logic := '0';

    -- =========================================================================
    -- Output registers
    -- =========================================================================
    signal s_tdata_r         : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid_r        : std_logic := '0';
    signal s_tlast_r         : std_logic := '0';
    signal s_line_done_r     : std_logic := '0';
    signal s_chip_error_r    : std_logic_vector(3 downto 0) := (others => '0');

    -- =========================================================================
    -- Blank beat data: all zeros except beat 4 has error_fill=1 (bit 10)
    -- =========================================================================
    function fn_blank_beat(
        beat_idx : unsigned(2 downto 0)
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    begin
        v_result := (others => '0');
        if to_integer(beat_idx) = 4 then
            v_result(10) := '1';   -- error_fill
        end if;
        return v_result;
    end function;

begin

    -- =========================================================================
    -- Combinational timeout limit computation
    --   timeout = max_range + stops*MAX_HITS*ticks*div + ALU + 1
    --   Config params are latched at packet_start and stable.
    -- =========================================================================
    p_timeout_calc : process(i_stops_per_chip, i_bus_ticks, i_bus_clk_div,
                             i_max_range_clks)
        variable v_reads_per_chip : unsigned(7 downto 0);
        variable v_clks_per_read  : unsigned(10 downto 0);
        variable v_drain_max      : unsigned(15 downto 0);
    begin
        v_reads_per_chip := i_stops_per_chip
                            * to_unsigned(c_MAX_HITS_PER_STOP, 4);
        v_clks_per_read  := i_bus_ticks
                            * resize(i_bus_clk_div, 8);
        v_drain_max      := resize(v_reads_per_chip * v_clks_per_read, 16);
        s_timeout_limit  <= i_max_range_clks;-- 확실한 것부터 점검
                        --   + v_drain_max
                        --   + to_unsigned(g_ALU_PULSE_CLKS + 1, 16);
    end process p_timeout_calc;

    -- =========================================================================
    -- Combinational tready to cell_builders
    --   Assert tready for the currently selected ready chip when the output
    --   register can accept new data (empty or being consumed).
    --   Suppressed on chip_last / line_last to prevent over-read.
    -- =========================================================================
    p_tready : process(s_state_r, s_cur_chip_r, s_is_blank_r,
                       s_tvalid_r, i_m_axis_tready,
                       s_chip_last_r, s_tlast_r)
        variable v_can_load : boolean;
    begin
        o_s_axis_tready <= (others => '0');
        v_can_load := (s_tvalid_r = '0')
                      or (s_tvalid_r = '1' and i_m_axis_tready = '1'
                          and s_chip_last_r = '0' and s_tlast_r = '0');

        if s_state_r = ST_FORWARD and s_is_blank_r = '0' and v_can_load then
            o_s_axis_tready(to_integer(s_cur_chip_r)) <= '1';
        end if;
    end process p_tready;

    -- =========================================================================
    -- Main process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_consumed       : boolean;
        variable v_can_load       : boolean;
        variable v_cur_chip       : unsigned(1 downto 0);
        variable v_is_blank       : boolean;
        variable v_is_last        : boolean;
        variable v_found          : boolean;
        variable v_blank_last     : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r        <= ST_IDLE;
                s_chip_ready_r   <= (others => '0');
                s_chip_timeout_r <= (others => '0');
                s_timeout_cnt_r  <= (others => (others => '0'));
                s_timeout_limit_r <= (others => '0');
                s_cur_chip_r     <= (others => '0');
                s_is_blank_r     <= '0';
                s_is_last_chip_r <= '0';
                s_blank_stop_r   <= (others => '0');
                s_blank_beat_r   <= (others => '0');
                s_chip_last_r    <= '0';
                s_tdata_r        <= (others => '0');
                s_tvalid_r       <= '0';
                s_tlast_r        <= '0';
                s_line_done_r    <= '0';
                s_chip_error_r   <= (others => '0');
            else
                -- Default: clear single-cycle pulses
                s_line_done_r <= '0';

                case s_state_r is

                -- ==============================================================
                -- ST_IDLE: wait for shot_start
                -- ==============================================================
                when ST_IDLE =>
                    s_tvalid_r    <= '0';
                    s_tlast_r     <= '0';
                    s_chip_last_r <= '0';

                    if i_shot_start = '1' then
                        s_timeout_limit_r <= s_timeout_limit;
                        s_chip_ready_r    <= (others => '0');
                        s_chip_timeout_r  <= (others => '0');
                        s_timeout_cnt_r   <= (others => (others => '0'));
                        s_chip_error_r    <= (others => '0');
                        s_state_r         <= ST_WAIT;
                    end if;

                -- ==============================================================
                -- ST_WAIT: per-chip timeout, wait for all active chips resolved
                -- ==============================================================
                when ST_WAIT =>
                    -- Update per-chip status
                    for i in 0 to c_N_CHIPS - 1 loop
                        if i_active_chip_mask(i) = '1' then
                            -- Check cell_ready (cell_builder tvalid)
                            if i_s_axis_tvalid(i) = '1' then
                                s_chip_ready_r(i) <= '1';
                            end if;
                            -- Timeout counter (only if not already ready/timed out)
                            if s_chip_ready_r(i) = '0'
                               and s_chip_timeout_r(i) = '0' then
                                if s_timeout_cnt_r(i) = s_timeout_limit_r then
                                    s_chip_timeout_r(i) <= '1';
                                    s_chip_error_r(i)   <= '1';
                                else
                                    s_timeout_cnt_r(i) <= s_timeout_cnt_r(i) + 1;
                                end if;
                            end if;
                        end if;
                    end loop;

                    -- All active chips resolved?
                    -- (ready OR timed_out OR inactive) = all '1'
                    if (s_chip_ready_r or s_chip_timeout_r
                        or (not i_active_chip_mask)) = "1111" then

                        -- Find first active chip
                        v_found := false;
                        v_cur_chip := (others => '0');
                        for i in 0 to 3 loop
                            if i_active_chip_mask(i) = '1' and not v_found then
                                v_cur_chip := to_unsigned(i, 2);
                                v_found := true;
                            end if;
                        end loop;

                        -- Is first chip blank (timed out)?
                        v_is_blank := (s_chip_timeout_r(to_integer(v_cur_chip)) = '1');

                        -- Is first chip also the last active chip?
                        v_is_last := true;
                        for i in 0 to 3 loop
                            if i > to_integer(v_cur_chip)
                               and i_active_chip_mask(i) = '1' then
                                v_is_last := false;
                            end if;
                        end loop;

                        s_cur_chip_r <= v_cur_chip;
                        if v_is_blank then
                            s_is_blank_r <= '1';
                        else
                            s_is_blank_r <= '0';
                        end if;
                        if v_is_last then
                            s_is_last_chip_r <= '1';
                        else
                            s_is_last_chip_r <= '0';
                        end if;

                        s_blank_stop_r <= (others => '0');
                        s_blank_beat_r <= (others => '0');
                        s_tvalid_r     <= '0';    -- first beat loaded on next clk
                        s_state_r      <= ST_FORWARD;
                    end if;

                -- ==============================================================
                -- ST_FORWARD: output beats from active chips in ascending order
                --   Ready chips: forward from cell_builder AXI-Stream
                --   Timed-out chips: generate blank beats (error_fill=1)
                --   1-cycle bubble between chips (chip switch)
                -- ==============================================================
                when ST_FORWARD =>
                    v_consumed := (s_tvalid_r = '1' and i_m_axis_tready = '1');
                    v_can_load := (s_tvalid_r = '0') or v_consumed;

                    -- ---- Case 1: last beat of entire packed line consumed ----
                    if v_consumed and s_tlast_r = '1' then
                        s_tvalid_r    <= '0';
                        s_tlast_r     <= '0';
                        s_chip_last_r <= '0';
                        s_line_done_r <= '1';
                        s_state_r     <= ST_IDLE;

                    -- ---- Case 2: last beat of current chip consumed ----
                    --   Insert 1-cycle bubble, advance to next chip
                    elsif v_consumed and s_chip_last_r = '1' then
                        s_tvalid_r    <= '0';
                        s_chip_last_r <= '0';

                        -- Find next active chip after current
                        v_found := false;
                        v_cur_chip := (others => '0');
                        for i in 0 to 3 loop
                            if i > to_integer(s_cur_chip_r)
                               and i_active_chip_mask(i) = '1'
                               and not v_found then
                                v_cur_chip := to_unsigned(i, 2);
                                v_found := true;
                            end if;
                        end loop;

                        v_is_blank := (s_chip_timeout_r(to_integer(v_cur_chip)) = '1');

                        v_is_last := true;
                        for i in 0 to 3 loop
                            if i > to_integer(v_cur_chip)
                               and i_active_chip_mask(i) = '1' then
                                v_is_last := false;
                            end if;
                        end loop;

                        s_cur_chip_r <= v_cur_chip;
                        if v_is_blank then
                            s_is_blank_r <= '1';
                        else
                            s_is_blank_r <= '0';
                        end if;
                        if v_is_last then
                            s_is_last_chip_r <= '1';
                        else
                            s_is_last_chip_r <= '0';
                        end if;
                        s_blank_stop_r <= (others => '0');
                        s_blank_beat_r <= (others => '0');

                    -- ---- Case 3: load next beat from current source ----
                    elsif v_can_load then

                        if s_is_blank_r = '1' then
                            -- Blank chip: generate beat data
                            s_tdata_r  <= fn_blank_beat(s_blank_beat_r);
                            s_tvalid_r <= '1';

                            -- Check if this is the last blank beat
                            v_blank_last :=
                                (s_blank_stop_r = i_stops_per_chip(2 downto 0) - 1)
                                and (s_blank_beat_r = c_BEATS_PER_CELL - 1);

                            if v_blank_last and s_is_last_chip_r = '1' then
                                s_tlast_r     <= '1';
                                s_chip_last_r <= '1';
                            elsif v_blank_last then
                                s_tlast_r     <= '0';
                                s_chip_last_r <= '1';
                            else
                                s_tlast_r     <= '0';
                                s_chip_last_r <= '0';
                            end if;

                            -- Advance blank counters
                            if s_blank_beat_r = c_BEATS_PER_CELL - 1 then
                                s_blank_beat_r <= (others => '0');
                                s_blank_stop_r <= s_blank_stop_r + 1;
                            else
                                s_blank_beat_r <= s_blank_beat_r + 1;
                            end if;

                        else
                            -- Ready chip: forward from cell_builder
                            -- (tready asserted combinationally by p_tready)
                            s_tdata_r  <= i_s_axis_tdata(to_integer(s_cur_chip_r));
                            s_tvalid_r <= '1';

                            -- Last beat detection via cell_builder's tlast
                            if i_s_axis_tlast(to_integer(s_cur_chip_r)) = '1' then
                                s_chip_last_r <= '1';
                                if s_is_last_chip_r = '1' then
                                    s_tlast_r <= '1';
                                else
                                    s_tlast_r <= '0';
                                end if;
                            else
                                s_chip_last_r <= '0';
                                s_tlast_r     <= '0';
                            end if;
                        end if;

                    end if;  -- v_consumed / v_can_load

                end case;

                -- shot_start override for ST_WAIT/ST_FORWARD only.
                -- Abort current operation and restart (like cell_builder).
                if i_shot_start = '1' and s_state_r /= ST_IDLE then
                    s_timeout_limit_r <= s_timeout_limit;
                    s_chip_ready_r    <= (others => '0');
                    s_chip_timeout_r  <= (others => '0');
                    s_timeout_cnt_r   <= (others => (others => '0'));
                    s_chip_error_r    <= (others => '0');
                    s_tvalid_r        <= '0';
                    s_tlast_r         <= '0';
                    s_chip_last_r     <= '0';
                    s_state_r         <= ST_WAIT;
                end if;
            end if;
        end if;
    end process p_main;

    -- =========================================================================
    -- Output assignments (all registered)
    -- =========================================================================
    o_m_axis_tdata     <= s_tdata_r;
    o_m_axis_tvalid    <= s_tvalid_r;
    o_m_axis_tlast     <= s_tlast_r;
    o_line_done        <= s_line_done_r;
    o_chip_error_flags <= s_chip_error_r;

end architecture rtl;
