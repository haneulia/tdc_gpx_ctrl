-- =============================================================================
-- tdc_gpx_raw_event_builder.vhd
-- TDC-GPX Controller - Raw Event Builder (Registered)
-- =============================================================================
--
-- Purpose:
--   Enriches decoded IFIFO fields with context (chip_id, shot_seq) and
--   assigns per-stop hit_seq_local counter (3-bit, 0..7 per slope).
--   Registered pipeline with AXI-Stream slave input and master output.
--
--   drain_done propagation: input tuser[7]='1' control beat resets hit
--   counters and is forwarded on the output AXI-Stream as tuser[7]='1'.
--
--   Range check: stop_id_local >= stops_per_chip => discard + error pulse.
--
-- AXI-Stream slave (from decode_i):
--   tdata[16:0] = raw_hit,  tuser[7] = drain_done
--   tuser[0] = slope, [2:1] = cha_code, [5:3] = stop_id, [6] = ififo_id
--
-- AXI-Stream master (to cell_builder via skid buffer):
--   tdata[16:0] = raw_hit
--   tuser[0] = slope, [2:1] = chip_id, [5:3] = stop_id, [6] = ififo_id,
--   tuser[7] = drain_done, [10:8] = hit_seq_local
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_raw_event_builder is
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- AXI-Stream slave (from decode_i, registered)
        --   tdata[16:0]  = raw_hit (17-bit, 0 for drain_done beat)
        --   tuser[0]     = slope
        --   tuser[2:1]   = cha_code_raw
        --   tuser[5:3]   = stop_id_local
        --   tuser[6]     = ififo_id
        --   tuser[7]     = drain_done ('1' = control beat: reset hit counters)
        i_s_axis_tvalid   : in  std_logic;
        i_s_axis_tdata    : in  std_logic_vector(31 downto 0);
        i_s_axis_tuser    : in  std_logic_vector(7 downto 0);
        o_s_axis_tready   : out std_logic;

        -- Context (from chip_ctrl / TOP)
        i_chip_id         : in  unsigned(1 downto 0);
        i_shot_seq        : in  unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);

        -- Configuration
        i_stops_per_chip  : in  unsigned(3 downto 0);

        -- AXI-Stream master (to cell_builder via slope demux in top)
        --   tdata[16:0]  = raw_hit (17-bit measurement)
        --   tdata[31:17] = 0 (reserved)
        --   tuser[0]     = slope
        --   tuser[2:1]   = chip_id
        --   tuser[5:3]   = stop_id_local (0..7)
        --   tuser[6]     = ififo_id
        --   tuser[7]    = drain_done (control beat: no data, triggers output phase)
        --   tuser[10:8]   = hit_seq_local (0..7 per slope)
        --   tuser[15:11] = 0 (reserved)
        o_m_axis_tvalid   : out std_logic;
        o_m_axis_tdata    : out std_logic_vector(31 downto 0);
        o_m_axis_tuser    : out std_logic_vector(15 downto 0);
        i_m_axis_tready   : in  std_logic;

        -- Error
        o_stop_id_error   : out std_logic    -- 1-clk pulse on out-of-range stop_id
    );
end entity tdc_gpx_raw_event_builder;

architecture rtl of tdc_gpx_raw_event_builder is

    -- Per-stop hit sequence counters (3-bit: 0..7 per slope, MAX_HITS=7)
    type t_hit_cnt_array is array (0 to c_MAX_STOPS_PER_CHIP - 1)
        of unsigned(2 downto 0);

    signal s_hit_cnt_r        : t_hit_cnt_array := (others => (others => '0'));

    -- AXI-Stream output registers
    signal s_tvalid_r         : std_logic := '0';
    signal s_tdata_r          : std_logic_vector(31 downto 0) := (others => '0');
    signal s_tuser_r          : std_logic_vector(15 downto 0) := (others => '0');
    signal s_stop_id_error_r  : std_logic := '0';

begin

    -- Always-accept (no stall in this pipeline stage)
    o_s_axis_tready <= '1';

    p_builder : process(i_clk)
        variable v_stop_idx : natural range 0 to c_MAX_STOPS_PER_CHIP - 1;
        variable v_raw_hit  : unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
        variable v_slope    : std_logic;
        variable v_stop_id  : unsigned(2 downto 0);
        variable v_ififo_id : std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_hit_cnt_r        <= (others => (others => '0'));
                s_tvalid_r         <= '0';
                s_tdata_r          <= (others => '0');
                s_tuser_r          <= (others => '0');
                s_stop_id_error_r  <= '0';
            else
                -- Default: clear single-cycle pulses
                s_tvalid_r        <= '0';
                s_stop_id_error_r <= '0';

                if i_s_axis_tvalid = '1' and i_s_axis_tuser(7) = '1' then
                    -- drain_done control beat: forward with ififo_id preserved.
                    -- ififo_id=0 (ififo1_done): do NOT reset counters (IFIFO2 active)
                    -- ififo_id=1 (final done):  reset ALL hit counters (shot boundary)
                    if i_s_axis_tuser(6) = '1' then
                        s_hit_cnt_r <= (others => (others => '0'));
                    end if;
                    s_tdata_r     <= (others => '0');
                    s_tuser_r     <= (others => '0');
                    s_tuser_r(7)  <= '1';   -- drain_done flag
                    s_tuser_r(6)  <= i_s_axis_tuser(6);  -- preserve ififo_id
                    s_tvalid_r    <= '1';
                elsif i_s_axis_tvalid = '1' then
                    -- Unpack input AXI-Stream sideband
                    v_raw_hit  := unsigned(i_s_axis_tdata(c_RAW_HIT_WIDTH - 1 downto 0));
                    v_slope    := i_s_axis_tuser(0);
                    v_stop_id  := unsigned(i_s_axis_tuser(5 downto 3));
                    v_ififo_id := i_s_axis_tuser(6);

                    -- Range check: stop_id_local < stops_per_chip
                    if ('0' & v_stop_id) >= i_stops_per_chip then
                        s_stop_id_error_r <= '1';
                    else
                        v_stop_idx := to_integer(v_stop_id);

                        -- Pack output AXI-Stream
                        s_tdata_r                                     <= (others => '0');
                        s_tdata_r(c_RAW_HIT_WIDTH - 1 downto 0)      <= std_logic_vector(v_raw_hit);
                        s_tuser_r(0)            <= v_slope;
                        s_tuser_r(2 downto 1)   <= std_logic_vector(i_chip_id);
                        s_tuser_r(5 downto 3)   <= std_logic_vector(v_stop_id);
                        s_tuser_r(6)            <= v_ififo_id;
                        s_tuser_r(7)            <= '0';   -- not drain_done
                        s_tuser_r(10 downto 8)  <= std_logic_vector(s_hit_cnt_r(v_stop_idx));
                        s_tuser_r(15 downto 11) <= (others => '0');

                        s_hit_cnt_r(v_stop_idx) <= s_hit_cnt_r(v_stop_idx) + 1;
                        s_tvalid_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_builder;

    o_m_axis_tvalid <= s_tvalid_r;
    o_m_axis_tdata  <= s_tdata_r;
    o_m_axis_tuser  <= s_tuser_r;
    o_stop_id_error <= s_stop_id_error_r;

end architecture rtl;
