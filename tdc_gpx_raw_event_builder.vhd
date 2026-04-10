-- =============================================================================
-- tdc_gpx_raw_event_builder.vhd
-- TDC-GPX Controller - Raw Event Builder (Registered)
-- =============================================================================
--
-- Purpose:
--   Enriches decoded IFIFO fields with context (chip_id, shot_seq) and
--   assigns per-stop hit_seq_local counter. Outputs t_raw_event record.
--
--   hit_seq_local: sequential index for hits on the same {chip, shot, stop}.
--   FIFO ordering guarantees time-sorted sequence within each stop channel.
--   Counter resets on drain_done (shot boundary).
--
--   Range check: stop_id_local >= stops_per_chip => discard + error count.
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_raw_event_builder is
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- Decoded fields (from decode_i, combinational)
        i_raw_hit         : in  unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
        i_slope           : in  std_logic;
        i_cha_code_raw    : in  unsigned(1 downto 0);
        i_stop_id_local   : in  unsigned(2 downto 0);
        i_decoded_valid   : in  std_logic;
        i_ififo_id        : in  std_logic;

        -- Context (from chip_ctrl / TOP)
        i_chip_id         : in  unsigned(1 downto 0);
        i_shot_seq        : in  unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        i_drain_done      : in  std_logic;

        -- Configuration
        i_stops_per_chip  : in  unsigned(3 downto 0);

        -- Output
        o_raw_event       : out t_raw_event;
        o_raw_event_valid : out std_logic;

        -- Error
        o_stop_id_error   : out std_logic    -- 1-clk pulse on out-of-range stop_id
    );
end entity tdc_gpx_raw_event_builder;

architecture rtl of tdc_gpx_raw_event_builder is

    -- Per-stop hit sequence counters (0..7 per stop channel)
    type t_hit_cnt_array is array (0 to c_MAX_STOPS_PER_CHIP - 1)
        of unsigned(2 downto 0);

    signal s_hit_cnt_r        : t_hit_cnt_array := (others => (others => '0'));

    -- Output registers
    signal s_raw_event_r      : t_raw_event := c_RAW_EVENT_INIT;
    signal s_raw_event_valid_r : std_logic := '0';
    signal s_stop_id_error_r  : std_logic := '0';

begin

    p_builder : process(i_clk)
        variable v_stop_idx : natural range 0 to c_MAX_STOPS_PER_CHIP - 1;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_hit_cnt_r         <= (others => (others => '0'));
                s_raw_event_r       <= c_RAW_EVENT_INIT;
                s_raw_event_valid_r <= '0';
                s_stop_id_error_r   <= '0';
            else
                -- Default: clear single-cycle pulses
                s_raw_event_valid_r <= '0';
                s_stop_id_error_r   <= '0';

                -- drain_done: reset all hit counters (shot boundary)
                if i_drain_done = '1' then
                    s_hit_cnt_r <= (others => (others => '0'));
                end if;

                if i_decoded_valid = '1' then
                    -- Range check: stop_id_local < stops_per_chip
                    if ('0' & i_stop_id_local) >= i_stops_per_chip then
                        -- Out-of-range stop_id: discard event, flag error
                        s_stop_id_error_r <= '1';
                    else
                        v_stop_idx := to_integer(i_stop_id_local);

                        -- Build raw_event record
                        s_raw_event_r.valid         <= '1';
                        s_raw_event_r.chip_id       <= i_chip_id;
                        s_raw_event_r.ififo_id      <= i_ififo_id;
                        s_raw_event_r.stop_id_local <= i_stop_id_local;
                        s_raw_event_r.slope         <= i_slope;
                        s_raw_event_r.raw_hit       <= i_raw_hit;
                        s_raw_event_r.shot_seq      <= i_shot_seq;
                        s_raw_event_r.hit_seq_local <= s_hit_cnt_r(v_stop_idx);

                        -- Increment hit counter for this stop
                        s_hit_cnt_r(v_stop_idx) <= s_hit_cnt_r(v_stop_idx) + 1;

                        s_raw_event_valid_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_builder;

    -- Output assignments
    o_raw_event       <= s_raw_event_r;
    o_raw_event_valid <= s_raw_event_valid_r;
    o_stop_id_error   <= s_stop_id_error_r;

end architecture rtl;
