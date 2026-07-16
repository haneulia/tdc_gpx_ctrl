-- =============================================================================
-- tdc_gpx_status_agg.vhd
-- TDC-GPX Controller - Status Aggregation
-- =============================================================================
--
-- Purpose:
--   Aggregates status signals, error counters, and timestamp counter.
--   Extracted from tdc_gpx_top to reduce top-level complexity.
--
-- Extracted processes: p_timestamp, p_error_cnt, p_err_sticky
-- Plus registered status aggregation assignments.
--
-- SW-initiated sticky/count clear (Q&A #40, Round 4):
--   i_soft_clear (default '0') clears s_err_drain_sticky_r,
--   s_err_seq_sticky_r, and s_error_count_r (resync baseline) without
--   a hard reset. Shared with err_handler.i_soft_clear via top-level
--   s_err_soft_clear so SW sees consistent error-history clearing.
--
-- Port rename (Round 3 #39):
--   o_error_count → o_error_cycle_count (the value is cycle count, not
--   event count — matches the signal's actual semantic).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_status_agg is
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;
        i_cmd_start_accepted : in  std_logic;
        -- SW-initiated clear for sticky errors + error cycle counter.
        -- Shared with err_handler.i_soft_clear (same bit drives both so SW
        -- sees consistent "clear all error history" semantics). Default '0'
        -- keeps legacy behavior unchanged until top wires it up.
        i_soft_clear         : in  std_logic := '0';

        -- Pipeline state inputs
        i_face_state_idle    : in  std_logic;
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

        -- Error inputs
        i_stop_id_error      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_hit_dropped        : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_hit_fall_dropped   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_err_drain_timeout  : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_err_sequence       : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_chip_error_merged  : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_face_active_mask   : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_shot_overrun       : in  std_logic;
        i_shot_fall_overrun  : in  std_logic;

        -- Outputs.
        --
        -- LATENT-BUG FIX (2026-07-17): these were one `o_status : out
        -- t_tdc_status` record port while this module assigned only 4 of
        -- its fields. In VHDL an out-port is a SOURCE for every element of
        -- the actual, so the dozens of fields assigned in tdc_gpx_top
        -- resolved against this port's unassigned-'U' contributions and
        -- read 'U' in simulation. The Verilog XPM CDC then squashed 'U' to
        -- 0, so every top-driven STAT5/6/7 sticky silently read as zero at
        -- the CSR — the exact-zero compares always "passed". Each
        -- t_tdc_status field must have exactly ONE source: this module now
        -- exports only the fields it owns, as scalars, and top assembles
        -- the record.
        o_busy               : out std_logic;
        o_pipeline_overrun   : out std_logic;
        o_rise_overrun       : out std_logic;
        o_fall_overrun       : out std_logic;
        o_timestamp          : out unsigned(63 downto 0);
        o_error_cycle_count  : out unsigned(31 downto 0);  -- error-active cycle count (not event count)
        o_err_drain_sticky   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_seq_sticky     : out std_logic_vector(c_N_CHIPS - 1 downto 0)
    );
end entity tdc_gpx_status_agg;

architecture rtl of tdc_gpx_status_agg is

    constant C_ZEROS : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    signal s_timestamp_r        : unsigned(63 downto 0) := (others => '0');
    signal s_error_count_r      : unsigned(31 downto 0) := (others => '0');
    signal s_chip_error_prev_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_drain_sticky_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_seq_sticky_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_busy_r             : std_logic := '0';
    signal s_pipeline_overrun_r : std_logic := '0';
    signal s_rise_overrun_r     : std_logic := '0';
    signal s_fall_overrun_r     : std_logic := '0';

begin

    -- =========================================================================
    -- Timestamp (free-running)
    -- =========================================================================
    p_timestamp : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_timestamp_r <= (others => '0');
            else
                s_timestamp_r <= s_timestamp_r + 1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Error cycle counter: increments by 1 per clock cycle where ANY error
    -- source is active. Multiple simultaneous errors = still +1.
    -- Semantics: "number of error-active cycles", NOT "number of events".
    -- =========================================================================
    p_error_cnt : process(i_clk)
        variable v_merged_rising : std_logic_vector(c_N_CHIPS - 1 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_cmd_soft_reset = '1' then
                s_error_count_r     <= (others => '0');
                s_chip_error_prev_r <= (others => '0');
            elsif i_soft_clear = '1' then
                -- SW-initiated counter clear (Q&A #40)
                s_error_count_r     <= (others => '0');
                s_chip_error_prev_r <= i_chip_error_merged;  -- resync baseline
            else
                v_merged_rising := i_chip_error_merged and (not s_chip_error_prev_r);
                s_chip_error_prev_r <= i_chip_error_merged;

                if i_stop_id_error /= C_ZEROS or
                   i_hit_dropped /= C_ZEROS or
                   i_hit_fall_dropped /= C_ZEROS or
                   (i_err_drain_timeout and i_face_active_mask) /= C_ZEROS or
                   (i_err_sequence and i_face_active_mask) /= C_ZEROS or
                   v_merged_rising /= C_ZEROS then
                    s_error_count_r <= s_error_count_r + 1;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Sticky error latches
    -- =========================================================================
    p_err_sticky : process(i_clk)
    begin
        if rising_edge(i_clk) then
            -- Sticky errors: cleared on hard reset or SW-initiated soft_clear.
            -- SW can read sticky history then pulse soft_clear to resume fresh
            -- observation without a hard reset.
            if i_rst_n = '0' or i_soft_clear = '1' then
                s_err_drain_sticky_r <= (others => '0');
                s_err_seq_sticky_r   <= (others => '0');
            else
                s_err_drain_sticky_r <= s_err_drain_sticky_r
                    or (i_err_drain_timeout and i_face_active_mask);
                s_err_seq_sticky_r   <= s_err_seq_sticky_r
                    or (i_err_sequence and i_face_active_mask);
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Registered live status aggregation.
    -- The wide busy/overrun fan-in is closed at this module boundary to keep
    -- CSR status packing from inheriting a long combinational path.
    -- =========================================================================
    p_live_status : process(i_clk)
        variable v_busy : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_busy_r             <= '0';
                s_pipeline_overrun_r <= '0';
                s_rise_overrun_r     <= '0';
                s_fall_overrun_r     <= '0';
            else
                v_busy := '0';
                if i_face_state_idle = '0'
                   or i_chip_busy /= C_ZEROS
                   or i_face_asm_idle = '0'
                   or i_face_asm_fall_idle = '0'
                   or i_hdr_idle = '0'
                   or i_hdr_fall_idle = '0'
                   or i_face_tvalid = '1'
                   or i_face_fall_tvalid = '1'
                   or i_face_buf_tvalid = '1'
                   or i_face_fall_buf_tvalid = '1'
                   or i_m_axis_tvalid = '1'
                   or i_m_axis_fall_tvalid = '1'
                   or i_reg_outstanding = '1' then
                    v_busy := '1';
                end if;

                s_busy_r             <= v_busy;
                s_pipeline_overrun_r <= i_shot_overrun or i_shot_fall_overrun;
                s_rise_overrun_r     <= i_shot_overrun;
                s_fall_overrun_r     <= i_shot_fall_overrun;
            end if;
        end if;
    end process p_live_status;

    o_busy              <= s_busy_r;
    o_pipeline_overrun  <= s_pipeline_overrun_r;
    o_rise_overrun      <= s_rise_overrun_r;
    o_fall_overrun      <= s_fall_overrun_r;

    o_timestamp       <= s_timestamp_r;
    o_error_cycle_count <= s_error_count_r;
    o_err_drain_sticky <= s_err_drain_sticky_r;
    o_err_seq_sticky   <= s_err_seq_sticky_r;

end architecture rtl;
