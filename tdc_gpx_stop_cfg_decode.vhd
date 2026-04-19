-- =============================================================================
-- tdc_gpx_stop_cfg_decode.vhd
-- TDC-GPX Controller - Stop Event Decoder
-- =============================================================================
--
-- Purpose:
--   Decodes per-chip IFIFO expected drain counts from echo_receiver.
--   Also handles cfg_image register override (StartOff1, MasterAluTrig, Reg7).
--
-- CONTRACT (echo_receiver → stop_cfg_decode stream):
--   - i_shot_start_gated pulses once per shot; counts reset to zero on this edge.
--   - Between shot_start pulses, echo_receiver emits i_stop_evt_tvalid each time
--     a stop pulse is observed. Each beat carries the RUNNING total of stop
--     pulses seen so far in the current shot (not a delta).
--   - Counts MUST monotonically increase within a shot window (a running total
--     can only grow or stay the same). Decrease is a contract violation and is
--     flagged by the sim-only monotonicity checker below.
--   - The last beat before irflag carries the FINAL expected count for this
--     shot; o_expected_ififo1/2 retain that value (overwrite semantics).
--
-- Distance-based shot window (Round 13 follow-up, audit 5번):
--   Orphan detection timing is derived from i_cfg.max_range_clks (physical
--   time-of-flight bound) rather than an arbitrary generic. Design contract:
--
--     shot_start < R = max_range_clks < W = R + margin < T1 = next shot_start
--                   │                     │                 │
--                   │ (valid beats done)  │ (orphan zone)   │ (window reopens)
--
--   Operating rule: SW sets shot_period = 1.5 × round-trip(max_distance)
--   i.e. 50% PRF headroom. Under this rule, gap (T1 − R) = 0.5 × R, which
--   comfortably accommodates the margin (echo_receiver emission latency +
--   pipeline stages) and still leaves an orphan-detection zone.
--
--   Reference use cases @ 200 MHz (see Doc/vdma_packet_structure.html §5,
--   Doc/260419/task_distance_bounded_windows_2026-04-19.md, and the project
--   memory project_tdc_window_timing.md for rationale):
--
--     | distance | R (cy) | shot_period (cy) | PRF     | orphan zone @ margin=32 |
--     |---------:|-------:|-----------------:|--------:|------------------------:|
--     | 100  m   |  134   |  200  (1.0 µs)   | 1.0 MHz |  34 cy (170 ns)         |
--     | 250  m   |  334   |  500  (2.5 µs)   | 400 kHz | 134 cy (670 ns)         |
--     | 500  m   |  667   | 1000  (5.0 µs)   | 200 kHz | 301 cy (1.51 µs)        |
--     | 750  m   | 1000   | 1500  (7.5 µs)   | 133 kHz | 468 cy (2.34 µs)        |
--     | 1000 m   | 1334   | 2000 (10.0 µs)   | 100 kHz | 634 cy (3.17 µs)        |
--
--   If SW runs tighter PRF (no headroom): window close timer never beats the
--   next shot_start → orphan detection degrades to "pre-first-shot only".
--   That is intentional; tightening margin further would only create false
--   positives. max_range_clks = 0 disables distance-based close entirely
--   (chip_run convention).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_stop_cfg_decode is
    generic (
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH;
        -- Round 13 follow-up (audit 5번): stop-event window margin.
        -- The effective window close = snapshot(i_cfg.max_range_clks) + this
        -- margin. max_range_clks is the physical distance-of-flight bound
        -- for this shot (all valid beats must arrive within it); the margin
        -- absorbs echo_receiver internal latency + pipeline stage delay on
        -- the stop_evt path. Default 32 cycles (~160ns @200MHz).
        --
        -- Sizing guidance (see Doc/vdma_packet_structure.html §5 for the
        -- distance / max_hits / shot-period table):
        --   - Must be > typical echo_receiver emission delay (few cycles).
        --   - Must be < (shot_period - max_range_clks) for orphan detection
        --     to have a non-empty zone; at the near-max-PRF configurations
        --     (500m+ rows in the doc) this is already near zero, so the
        --     orphan detection degrades naturally into "pre-first-shot
        --     only" for tight PRF workloads. That is intentional: trying
        --     to push margin smaller only buys false positives.
        g_WINDOW_MARGIN_CLKS : natural := 32
    );
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- Stop event AXI-Stream slave
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready : out std_logic;

        -- Shot boundary clear
        i_shot_start_gated : in  std_logic;

        -- Per-chip expected IFIFO counts
        o_expected_ififo1 : out t_expected_array;
        o_expected_ififo2 : out t_expected_array;

        -- Config image override
        i_cfg             : in  t_tdc_cfg;
        i_cfg_image_raw   : in  t_cfg_image;
        o_cfg_image       : out t_cfg_image;

        -- Round 11 item 10: runtime monotonicity sticky (per-chip).
        -- Bit i = '1' (latched) if chip i's IFIFO1 or IFIFO2 running total
        -- decreased within a shot window — a contract violation from
        -- echo_receiver. Sim-only assert still fires too for regression.
        -- Cleared only by i_rst_n; SW reads it to detect silent upstream
        -- format drift that would otherwise corrupt chip_run's drain
        -- policy via a bogus expected_ififo count.
        o_monotonic_violation_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 #16 + Round 13 follow-up (audit 5번): "orphan stop event"
        -- sticky. Fires when i_stop_evt_tvalid pulses while NO shot window
        -- is active — i.e. either before the first shot_start_gated since
        -- reset (pre-first-shot case) OR after the current shot's window
        -- has timed out (inter-shot gap case). Indicates upstream format
        -- drift that the monotonic check alone cannot catch.
        o_orphan_stop_evt_sticky   : out std_logic
    );
end entity tdc_gpx_stop_cfg_decode;

architecture rtl of tdc_gpx_stop_cfg_decode is

    -- Round 11 item 10: synth-side monotonic tracker.
    -- Remembers the previous running total per chip/IFIFO within a shot so
    -- a decrease latches s_mono_viol_r. Reset on shot boundary + global rst.
    type t_prev_arr is array(0 to c_N_CHIPS - 1) of unsigned(7 downto 0);
    signal s_prev_ififo1_r : t_prev_arr := (others => (others => '0'));
    signal s_prev_ififo2_r : t_prev_arr := (others => (others => '0'));
    signal s_track_r       : std_logic  := '0';   -- has at least one beat been seen this shot?
    signal s_mono_viol_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    -- Round 13 follow-up (audit 5번): distance-bounded shot window.
    --
    -- Timing contract:
    --   shot_start < max_range_clks (R) < window_close (W) < next_shot_start
    --   where W = snapshot(max_range_clks) + g_WINDOW_MARGIN_CLKS.
    --
    -- R is the physical time-of-flight bound — all VALID stop events arrive
    -- within it. The margin between R and W absorbs echo_receiver internal
    -- latency so legitimate late beats are not misclassified. Beats arriving
    -- between W and the next shot_start are in the inter-shot gap and flagged
    -- as orphan. Snapshot semantics at shot_start prevent mid-shot SW config
    -- writes from shifting the active window.
    --
    -- max_range_clks = 0 is treated as "distance check disabled" (matches
    -- chip_run's range-counter convention) — window stays open until the
    -- next shot_start, i.e. orphan only fires pre-first-shot.
    --
    -- Counter width = 17 bits: max_range_clks (16 bit) + margin (up to ~256)
    -- fits comfortably with headroom.
    constant c_WINDOW_CNT_WIDTH : natural := 17;
    signal s_window_active_r    : std_logic := '0';
    signal s_window_cnt_r       : unsigned(c_WINDOW_CNT_WIDTH - 1 downto 0)
                                  := (others => '0');
    signal s_max_range_snap_r   : unsigned(15 downto 0) := (others => '0');
    signal s_window_cap_r       : unsigned(c_WINDOW_CNT_WIDTH - 1 downto 0)
                                  := (others => '0');
    signal s_orphan_evt_sticky_r : std_logic := '0';

begin

    o_stop_evt_tready <= '1';
    o_monotonic_violation_mask <= s_mono_viol_r;
    o_orphan_stop_evt_sticky <= s_orphan_evt_sticky_r;

    -- Round 11 item 10: runtime violation detector (synth-live).
    p_mono_live : process(i_clk)
        variable v_new1 : unsigned(7 downto 0);
        variable v_new2 : unsigned(7 downto 0);
        variable v_lo   : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_prev_ififo1_r <= (others => (others => '0'));
                s_prev_ififo2_r <= (others => (others => '0'));
                s_track_r       <= '0';
                s_mono_viol_r   <= (others => '0');
                s_window_active_r     <= '0';
                s_window_cnt_r        <= (others => '0');
                s_max_range_snap_r    <= (others => '0');
                s_window_cap_r        <= (others => '0');
                s_orphan_evt_sticky_r <= '0';
            else
                -- Round 13 follow-up (audit 5번): distance-bounded window.
                -- Snapshot max_range_clks at shot_start so a mid-shot config
                -- change cannot retroactively shift the window. Effective
                -- cap = snapshot + margin. If the snapshot is zero (distance
                -- check disabled), keep the window open — no timer-based
                -- close, only the next shot_start re-opens.
                if i_shot_start_gated = '1' then
                    s_max_range_snap_r <= i_cfg.max_range_clks;
                    s_window_cap_r     <= resize(i_cfg.max_range_clks,
                                                  c_WINDOW_CNT_WIDTH)
                                          + to_unsigned(g_WINDOW_MARGIN_CLKS,
                                                         c_WINDOW_CNT_WIDTH);
                    s_window_active_r  <= '1';
                    s_window_cnt_r     <= (others => '0');
                elsif s_window_active_r = '1' and s_max_range_snap_r /= 0 then
                    if s_window_cnt_r < s_window_cap_r then
                        s_window_cnt_r <= s_window_cnt_r + 1;
                    else
                        s_window_active_r <= '0';
                    end if;
                end if;

                if i_shot_start_gated = '1' then
                    s_prev_ififo1_r <= (others => (others => '0'));
                    s_prev_ififo2_r <= (others => (others => '0'));
                    s_track_r       <= '0';
                    -- NOTE: sticky s_mono_viol_r is NOT cleared on shot boundary.
                    -- A violation in shot N should remain visible to SW across
                    -- subsequent shots; only full reset clears it.
                elsif i_stop_evt_tvalid = '1' then
                    -- Round 13 follow-up (audit 5번): fire orphan sticky if
                    -- the beat arrived with no active shot window. Covers
                    -- the pre-first-shot case (window never opened) AND the
                    -- inter-shot gap case (distance window closed).
                    if s_window_active_r = '0' then
                        s_orphan_evt_sticky_r <= '1';
                    end if;
                    for i in 0 to c_N_CHIPS - 1 loop
                        v_lo   := i * 8;
                        v_new1 := resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                                + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                        v_new2 := resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                                + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                        if s_track_r = '1' then
                            if v_new1 < s_prev_ififo1_r(i) or v_new2 < s_prev_ififo2_r(i) then
                                s_mono_viol_r(i) <= '1';
                            end if;
                        end if;
                        s_prev_ififo1_r(i) <= v_new1;
                        s_prev_ififo2_r(i) <= v_new2;
                    end loop;
                    s_track_r <= '1';
                end if;
            end if;
        end if;
    end process;

    p_stop_decode : process(i_clk)
        variable v_lo : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_shot_start_gated = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo := i * 8;
                    o_expected_ififo1(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    o_expected_ififo2(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                end loop;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Sim-only monotonicity contract check.
    -- Catches echo_receiver bugs where the running total decreases within a
    -- shot window (should be non-decreasing per contract above). Zero-impact
    -- on synthesized RTL; purely an observability aid in regression.
    -- =========================================================================
    -- synthesis translate_off
    p_monotonic_check : process(i_clk)
        variable v_new1   : unsigned(7 downto 0);
        variable v_new2   : unsigned(7 downto 0);
        variable v_lo     : natural;
        type t_prev_arr is array(0 to c_N_CHIPS - 1) of unsigned(7 downto 0);
        variable v_prev1  : t_prev_arr := (others => (others => '0'));
        variable v_prev2  : t_prev_arr := (others => (others => '0'));
        variable v_track  : boolean := false;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_shot_start_gated = '1' then
                v_track := false;
                for i in 0 to c_N_CHIPS - 1 loop
                    v_prev1(i) := (others => '0');
                    v_prev2(i) := (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo   := i * 8;
                    v_new1 := resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                            + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    v_new2 := resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                            + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                    if v_track then
                        assert v_new1 >= v_prev1(i)
                            report "stop_cfg_decode: IFIFO1 running total decreased on chip "
                                   & integer'image(i) & " (contract violation)"
                            severity error;
                        assert v_new2 >= v_prev2(i)
                            report "stop_cfg_decode: IFIFO2 running total decreased on chip "
                                   & integer'image(i) & " (contract violation)"
                            severity error;
                    end if;
                    v_prev1(i) := v_new1;
                    v_prev2(i) := v_new2;
                end loop;
                v_track := true;
            end if;
        end if;
    end process;
    -- synthesis translate_on

    -- cfg_image override: force specific register bits
    p_cfg_override : process(i_cfg_image_raw, i_cfg)
        variable v_img : t_cfg_image;
    begin
        v_img := i_cfg_image_raw;
        v_img(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(i_cfg.start_off1);
        v_img(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_img(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        v_img(7) := i_cfg.cfg_reg7;
        o_cfg_image <= v_img;
    end process;

end architecture rtl;
