-- =============================================================================
-- tdc_gpx_cell_builder.vhd
-- TDC-GPX Controller - Cell Builder + Chip Slice Output
-- =============================================================================
--
-- Purpose:
--   Converts sparse raw_event stream into dense cell array, then serializes
--   cells to AXI-Stream as a chip slice (stops_per_chip cells, ascending order).
--
--   Phase A (shot_start): clear all cell buffers
--   Phase B (raw_event):  store hits into cell_buffer using hit_count_actual
--                         as slot index; overflow (>=MAX_HITS) sets hit_dropped
--   Phase C (drain_done): serialize cells to m_axis (8 beats/cell)
--                         Pipeline: cell MUX (ST_LOAD) → beat MUX (ST_OUTPUT)
--                         1-clk bubble at each cell boundary for timing closure
--
--   "Beat" = one AXI-Stream transfer (tvalid & tready handshake).
--   1 beat = TDATA_WIDTH bits = 32 bits = 4 bytes.
--   1 cell = CELL_SIZE_BYTES / (TDATA_WIDTH/8) = 32/4 = 8 beats.
--
--   Beat layout (cell_format=0, 32-bit TDATA, 32 bytes/cell):
--     Beat 0: hit_slot[1](15:0) & hit_slot[0](15:0)
--     Beat 1: hit_slot[3](15:0) & hit_slot[2](15:0)
--     Beat 2: hit_slot[5](15:0) & hit_slot[4](15:0)
--     Beat 3: hit_slot[7](15:0) & hit_slot[6](15:0)
--     Beat 4: hit_valid(7:0) & slope_vec(7:0) & hit_count(3:0) &
--             hit_dropped & error_fill & chip_id(1:0) & "00000000"
--     Beat 5: x"00000000" (padding)
--     Beat 6: x"00000000" (padding)
--     Beat 7: x"00000000" (padding)
--
--   raw_hit is 17-bit; only lower 16 bits stored (HIT_SLOT_DATA_WIDTH=16).
--
--   Hit ordering: IFIFO guarantees time-sorted output. cell_builder stores
--   hits in arrival order using hit_count_actual as index:
--     hit_slot[0] = first hit, hit_slot[1] = second, ...
--   hit_valid bitmask indicates which slots contain valid data.
--   SW uses hit_valid + slot index to reconstruct arrival order.
--
--   All outputs are registered (module boundary = FF).
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cell_builder is
    generic (
        g_CHIP_ID           : natural range 0 to 3 := 0
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- Raw event input (from raw_event_builder)
        i_raw_event         : in  t_raw_event;
        i_raw_event_valid   : in  std_logic;

        -- Control (from chip_ctrl)
        i_shot_start        : in  std_logic;   -- new shot: clear cell buffers
        i_drain_done        : in  std_logic;   -- drain complete: begin output

        -- Configuration (latched at packet_start)
        i_stops_per_chip    : in  unsigned(3 downto 0);

        -- AXI-Stream master (chip slice output)
        o_m_axis_tdata      : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
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
    -- Cell buffer array (register-based, MAX_STOPS_PER_CHIP entries)
    -- =========================================================================
    type t_cell_array is array (0 to c_MAX_STOPS_PER_CHIP - 1) of t_cell;
    signal s_cell_buf_r : t_cell_array := (others => c_CELL_INIT);

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (ST_IDLE, ST_COLLECT, ST_LOAD, ST_OUTPUT);
    signal s_state_r : t_state := ST_IDLE;

    -- Pipeline register: selected cell for output serialization.
    -- Splits critical path: cell 8:1 MUX (ST_LOAD) | beat 8:1 MUX (ST_OUTPUT).
    signal s_cell_sel_r  : t_cell := c_CELL_INIT;

    -- Output serializer counters (registered)
    signal s_stop_idx_r  : unsigned(2 downto 0) := (others => '0');  -- 0..MAX_STOPS-1
    signal s_beat_idx_r  : unsigned(2 downto 0) := (others => '0');  -- 0..BEATS_PER_CELL-1
    signal s_last_stop_r : unsigned(2 downto 0) := (others => '0'); -- pre-computed: stops_per_chip-1

    -- Last beat index constant (for tlast comparison)
    constant c_LAST_BEAT : unsigned(2 downto 0) := to_unsigned(c_BEATS_PER_CELL - 1, 3);

    -- Registered outputs
    signal s_tdata_r     : std_logic_vector(c_TDATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tvalid_r    : std_logic := '0';
    signal s_tlast_r     : std_logic := '0';
    signal s_slice_done_r    : std_logic := '0';
    signal s_hit_dropped_r   : std_logic := '0';

    -- =========================================================================
    -- Cell-to-beat mux (combinational, used inside process)
    -- =========================================================================
    function fn_cell_beat(
        cell     : t_cell;
        beat_idx : unsigned(2 downto 0)
    ) return std_logic_vector is
        variable v_result : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        variable v_beat   : natural range 0 to 7;
    begin
        v_beat  := to_integer(beat_idx);
        v_result := (others => '0');

        case v_beat is
            when 0 =>
                v_result(31 downto 16) := std_logic_vector(cell.hit_slot(1));
                v_result(15 downto  0) := std_logic_vector(cell.hit_slot(0));
            when 1 =>
                v_result(31 downto 16) := std_logic_vector(cell.hit_slot(3));
                v_result(15 downto  0) := std_logic_vector(cell.hit_slot(2));
            when 2 =>
                v_result(31 downto 16) := std_logic_vector(cell.hit_slot(5));
                v_result(15 downto  0) := std_logic_vector(cell.hit_slot(4));
            when 3 =>
                v_result(31 downto 16) := std_logic_vector(cell.hit_slot(7));
                v_result(15 downto  0) := std_logic_vector(cell.hit_slot(6));
            when 4 =>
                v_result(31 downto 24) := cell.hit_valid;
                v_result(23 downto 16) := cell.slope_vec;
                v_result(15 downto 12) := std_logic_vector(cell.hit_count_actual);
                v_result(11)           := cell.hit_dropped;
                v_result(10)           := cell.error_fill;
                v_result(9 downto 8)   := std_logic_vector(to_unsigned(g_CHIP_ID, 2));
                v_result(7 downto 0)   := (others => '0');  -- reserved
            when others =>
                v_result := (others => '0');  -- beats 5~7: padding
        end case;

        return v_result;
    end function;

begin

    -- =========================================================================
    -- Main process
    -- =========================================================================
    p_main : process(i_clk)
        variable v_stop     : natural range 0 to c_MAX_STOPS_PER_CHIP - 1;
        variable v_seq      : natural range 0 to c_MAX_HITS_PER_STOP - 1;  -- slot index from hit_count_actual
        variable v_nxt_stop : unsigned(2 downto 0);
        variable v_nxt_beat : unsigned(2 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_cell_buf_r     <= (others => c_CELL_INIT);
                s_cell_sel_r     <= c_CELL_INIT;
                s_state_r        <= ST_IDLE;
                s_stop_idx_r     <= (others => '0');
                s_beat_idx_r     <= (others => '0');
                s_last_stop_r    <= (others => '0');
                s_tdata_r        <= (others => '0');
                s_tvalid_r       <= '0';
                s_tlast_r        <= '0';
                s_slice_done_r   <= '0';
                s_hit_dropped_r  <= '0';
            else
                -- Default: clear single-cycle pulses
                s_slice_done_r  <= '0';
                s_hit_dropped_r <= '0';

                case s_state_r is

                    -- ==========================================================
                    -- ST_IDLE: wait for shot_start
                    -- ==========================================================
                    when ST_IDLE =>
                        s_tvalid_r <= '0';
                        s_tlast_r  <= '0';

                        if i_shot_start = '1' then
                            s_cell_buf_r <= (others => c_CELL_INIT);
                            s_state_r    <= ST_COLLECT;
                        end if;

                    -- ==========================================================
                    -- ST_COLLECT: accumulate raw_events into cell_buffer
                    -- ==========================================================
                    when ST_COLLECT =>
                        if i_raw_event_valid = '1' then
                            v_stop := to_integer(i_raw_event.stop_id_local);

                            -- Use hit_count_actual (4-bit) as slot index + overflow guard
                            if s_cell_buf_r(v_stop).hit_count_actual < c_MAX_HITS_PER_STOP then
                                v_seq := to_integer(
                                    s_cell_buf_r(v_stop).hit_count_actual(2 downto 0));
                                s_cell_buf_r(v_stop).hit_slot(v_seq)
                                    <= i_raw_event.raw_hit(c_HIT_SLOT_DATA_WIDTH - 1 downto 0);
                                s_cell_buf_r(v_stop).hit_valid(v_seq) <= '1';
                                s_cell_buf_r(v_stop).slope_vec(v_seq) <= i_raw_event.slope;
                                s_cell_buf_r(v_stop).hit_count_actual
                                    <= s_cell_buf_r(v_stop).hit_count_actual + 1;
                            else
                                -- 9th+ hit: overflow, do not overwrite slot
                                s_cell_buf_r(v_stop).hit_dropped <= '1';
                                s_hit_dropped_r <= '1';
                            end if;
                        end if;

                        -- drain_done: latch first cell into pipeline register
                        if i_drain_done = '1' then
                            s_cell_sel_r  <= s_cell_buf_r(0);  -- cell MUX only
                            s_stop_idx_r  <= (others => '0');
                            s_beat_idx_r  <= (others => '0');
                            s_last_stop_r <= i_stops_per_chip(2 downto 0) - 1;
                            s_tvalid_r   <= '0';
                            s_tlast_r    <= '0';
                            s_state_r    <= ST_LOAD;
                        end if;

                    -- ==========================================================
                    -- ST_LOAD: beat MUX from pipeline register (1-clk bubble)
                    --   Critical path split: cell MUX ran in previous cycle,
                    --   result now in s_cell_sel_r. Only beat MUX here.
                    -- ==========================================================
                    when ST_LOAD =>
                        s_tdata_r  <= fn_cell_beat(s_cell_sel_r, s_beat_idx_r);
                        s_tvalid_r <= '1';

                        -- Pre-compute tlast for loaded beat (s_last_stop_r is registered)
                        if s_stop_idx_r = s_last_stop_r
                           and s_beat_idx_r = c_LAST_BEAT then
                            s_tlast_r <= '1';
                        else
                            s_tlast_r <= '0';
                        end if;

                        s_state_r <= ST_OUTPUT;

                    -- ==========================================================
                    -- ST_OUTPUT: serialize cells to AXI-Stream (registered)
                    --   Within a cell: beat MUX only (from s_cell_sel_r).
                    --   At cell boundary: load next cell into s_cell_sel_r,
                    --   insert 1-clk bubble via ST_LOAD.
                    -- ==========================================================
                    when ST_OUTPUT =>
                        if s_tvalid_r = '1' and i_m_axis_tready = '1' then
                            -- Check if this was the last beat of last cell
                            if s_stop_idx_r = s_last_stop_r
                               and s_beat_idx_r = c_LAST_BEAT then
                                -- Chip slice complete
                                s_tvalid_r     <= '0';
                                s_tlast_r      <= '0';
                                s_slice_done_r <= '1';
                                s_state_r      <= ST_IDLE;

                            elsif s_beat_idx_r = c_LAST_BEAT then
                                -- Cell boundary: load next cell (cell MUX only)
                                v_nxt_stop       := s_stop_idx_r + 1;
                                s_stop_idx_r     <= v_nxt_stop;
                                s_beat_idx_r     <= (others => '0');
                                s_cell_sel_r     <= s_cell_buf_r(to_integer(v_nxt_stop));
                                s_tvalid_r       <= '0';  -- 1-clk bubble
                                s_tlast_r        <= '0';
                                s_state_r        <= ST_LOAD;

                            else
                                -- Same cell: beat MUX only (from s_cell_sel_r)
                                v_nxt_beat   := s_beat_idx_r + 1;
                                s_beat_idx_r <= v_nxt_beat;
                                s_tdata_r    <= fn_cell_beat(s_cell_sel_r, v_nxt_beat);

                                -- Set tlast if NEXT beat is the final one
                                if s_stop_idx_r = s_last_stop_r
                                   and v_nxt_beat = c_LAST_BEAT then
                                    s_tlast_r <= '1';
                                else
                                    s_tlast_r <= '0';
                                end if;
                            end if;
                        end if;

                end case;

                -- shot_start override for ST_COLLECT/ST_OUTPUT only.
                -- Handles timeout recovery (cell_builder stuck in ST_OUTPUT).
                -- ST_IDLE is handled inside the case statement above.
                if i_shot_start = '1' and s_state_r /= ST_IDLE then
                    s_cell_buf_r <= (others => c_CELL_INIT);
                    s_cell_sel_r <= c_CELL_INIT;
                    s_state_r    <= ST_COLLECT;
                    s_tvalid_r   <= '0';
                    s_tlast_r    <= '0';
                end if;
            end if;
        end if;
    end process p_main;

    -- =========================================================================
    -- Output assignments (all registered)
    -- =========================================================================
    o_m_axis_tdata    <= s_tdata_r;
    o_m_axis_tvalid   <= s_tvalid_r;
    o_m_axis_tlast    <= s_tlast_r;
    o_slice_done      <= s_slice_done_r;
    o_hit_dropped_any <= s_hit_dropped_r;

end architecture rtl;
