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
--   - Every fire_count beat carries the face-local 1-base shot/fire count in
--     i_fire_count_tdata[15:0]. A stop_evt beat is accepted only when the
--     same-cycle fire_count beat matches i_current_fire_count. The final beat
--     (`i_fire_count_tlast='1'`) uses the same value match and only marks the
--     current shot's expected count as FINAL. This is required so expected=0
--     can mean "known zero" instead of the legacy fallback encoding.
--   - An owned stop event enters a one-cycle, II=1 stage. Expected counts and
--     monotonic history update on the following edge. A final beat immediately
--     after the last stop event commits that staged event and FINAL together.
--
-- Distance-based shot window (Round 13 follow-up, audit 5번):
--   Orphan detection timing is derived from max_range_5ns_ticks after it is
--   converted to i_clk-domain cycles by config_ctrl. Design contract:
--
--     shot_start < R = max_range_axis_clks < W = R + margin < next shot_start
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
--     | 100  m   |  134   |  201  (1.005 us) | 995 kHz |  35 cy (175 ns)         |
--     | 250  m   |  334   |  501  (2.505 us) | 399 kHz | 135 cy (675 ns)         |
--     | 500  m   |  668   | 1002  (5.010 us) | 200 kHz | 302 cy (1.51 us)        |
--     | 750  m   | 1001   | 1502  (7.510 us) | 133 kHz | 469 cy (2.35 us)        |
--     | 1000 m   | 1335   | 2003 (10.015 us) | 99.9kHz | 636 cy (3.18 us)        |
--
--   R uses ceil(round_trip / 5 ns), and shot_period uses ceil(1.5 * R).
--   For 50/100/125/150 MHz AXIS operation, convert the same 5 ns CSR value
--   with fn_range_5ns_ticks_to_clks; do not reuse the 200 MHz count directly.
--
--   If SW runs tighter PRF (no headroom): window close timer never beats the
--   next shot_start → orphan detection degrades to "pre-first-shot only".
--   That is intentional; tightening margin further would only create false
--   positives. max_range_5ns_ticks = 0 disables distance-based close entirely
--   (chip_run convention).
--
-- Edge-case rules (P5/P6):
--   - Same-cycle `i_shot_start_gated` + `i_stop_evt_tvalid` → shot_start
--     wins (resets tracking, window re-opens). The co-incident beat is
--     NOT counted in running totals — upstream contract says beats
--     arrive STRICTLY between shot_start pulses, so this coincidence is
--     a contract violation and silent drop is acceptable.
--   - `max_range_5ns_ticks = 0` disables BOTH the range check (chip_run) and
--     this orphan window. If SW wants orphan detection without a range
--     limit, set `max_range_5ns_ticks` to the largest practical value rather
--     than 0.
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
        g_STOP_EVT_DWIDTH : natural := c_DEFAULT_STOP_EVT_DWIDTH;
        g_STOP_EVT_TUSER_WIDTH : natural := c_DEFAULT_STOP_EVT_TUSER_WIDTH;
        g_FIRE_COUNT_DWIDTH : natural := c_DEFAULT_FIRE_COUNT_DWIDTH;
        -- Round 13 follow-up (audit 5번): stop-event window margin.
        -- The effective window close = snapshot(i_max_range_axis_clks) + this
        -- margin. i_max_range_axis_clks is the 5 ns CSR range converted to
        -- this AXIS clock domain; the margin
        -- absorbs echo_receiver internal latency + pipeline stage delay on
        -- the stop_evt path. Production top derives this count from one
        -- physical margin and g_AXIS_CLK_MHZ. Standalone default is 32 clocks
        -- at the default 150 MHz AXIS clock.
        --
        -- Sizing guidance (see Doc/vdma_packet_structure.html §5 for the
        -- distance / max_hits / shot-period table):
        --   - Must be > typical echo_receiver emission delay (few cycles).
        --   - Must be < (shot_period - max_range_axis_clks) for orphan detection
        --     to have a non-empty zone; at near-max-PRF workloads the gap
        --     is already near zero, so orphan detection degrades naturally
        --     into "pre-first-shot only". That is intentional: tightening
        --     margin further would only buy false positives.
        --   - UPPER BOUND: <= 65535 (16-bit policy). The 24-bit combined
        --     counter safely covers the 16-bit range plus this margin.
        --     Use a generic override for slower pipelines but stay within
        --     this explicit elaboration bound.
        g_WINDOW_MARGIN_CLKS : natural := c_DEFAULT_STOP_WINDOW_MARGIN_CLKS
    );
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- Stop event AXI-Stream slave
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(g_STOP_EVT_TUSER_WIDTH - 1 downto 0);
        o_stop_evt_tready : out std_logic;

        -- Fire-count sideband/final stream from echo_receiver.
        -- tdata[15:0] must match i_current_fire_count for both stop-event
        -- ownership and final qualification. tlast='1' is only the final
        -- marker; it is not the ownership key by itself.
        i_fire_count_tvalid : in  std_logic;
        i_fire_count_tdata  : in  std_logic_vector(g_FIRE_COUNT_DWIDTH - 1 downto 0);
        i_fire_count_tlast  : in  std_logic;

        -- Shot boundary clear
        i_shot_start_gated : in  std_logic;
        -- Face-local 1-base shot/fire count aligned to the current TDC shot.
        i_current_fire_count : in unsigned(15 downto 0);
        -- CSR max_range_5ns_ticks converted once by config_ctrl for the AXIS
        -- clock. Never pass the raw 5 ns tick value to this local counter.
        i_max_range_axis_clks : in unsigned(15 downto 0);

        -- Per-chip expected IFIFO counts
        o_expected_ififo1 : out t_expected_array;
        o_expected_ififo2 : out t_expected_array;
        o_expected_final_valid : out std_logic;

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
    --   shot_start < max_range_axis_clks (R) < window_close (W) < next_shot_start
    --   where W = snapshot(max_range_axis_clks) + g_WINDOW_MARGIN_CLKS.
    --
    -- R is the physical time-of-flight bound — all VALID stop events arrive
    -- within it. The margin between R and W absorbs echo_receiver internal
    -- latency so legitimate late beats are not misclassified. Beats arriving
    -- between W and the next shot_start are in the inter-shot gap and flagged
    -- as orphan. Snapshot semantics at shot_start prevent mid-shot SW config
    -- writes from shifting the active window.
    --
    -- max_range_5ns_ticks = 0 is treated as "distance check disabled" (matches
    -- chip_run's range-counter convention) — window stays open until the
    -- next shot_start, i.e. orphan only fires pre-first-shot.
    --
    -- Counter width = 24 bits: max_range_axis_clks (16 bit) + margin (up to
    -- 16-bit generic) fits with ample headroom. 24 bits gives ~84 ms @
    -- 200 MHz minimum window span which comfortably covers the generic bound
    -- documented above, with zero overflow risk for any legal override.
    constant c_WINDOW_CNT_WIDTH : natural := 24;
    signal s_window_active_r    : std_logic := '0';
    signal s_window_cnt_r       : unsigned(c_WINDOW_CNT_WIDTH - 1 downto 0)
                                  := (others => '0');
    signal s_max_range_axis_snap_r : unsigned(15 downto 0) := (others => '0');
    signal s_window_cap_r       : unsigned(c_WINDOW_CNT_WIDTH - 1 downto 0)
                                  := (others => '0');
    signal s_orphan_evt_sticky_r : std_logic := '0';
    signal s_expected_final_r    : std_logic := '0';

    -- Register only the ownership decision. Raw count fields are sampled every
    -- cycle without an enable, so the 16-bit fire-count comparison drives one
    -- valid D pin instead of the CE pins of all expected/previous registers.
    signal s_fire_match_now      : std_logic;
    signal s_stop_owned_now      : std_logic;
    signal s_owned_evt_valid_r   : std_logic := '0';
    signal s_owned_evt_data_r    : std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0)
                                        := (others => '0');
    signal s_owned_evt_tuser_r   : std_logic_vector(g_STOP_EVT_TUSER_WIDTH - 1 downto 0)
                                        := (others => '0');

begin

    assert g_FIRE_COUNT_DWIDTH >= 16
        report "tdc_gpx_stop_cfg_decode: g_FIRE_COUNT_DWIDTH must be >= 16"
        severity failure;
    assert g_STOP_EVT_DWIDTH >= c_N_CHIPS * 8
        report "tdc_gpx_stop_cfg_decode: g_STOP_EVT_DWIDTH must cover 8 bits per chip"
        severity failure;
    assert g_STOP_EVT_TUSER_WIDTH >= c_N_CHIPS * 8
        report "tdc_gpx_stop_cfg_decode: g_STOP_EVT_TUSER_WIDTH must cover 8 bits per chip"
        severity failure;
    assert g_WINDOW_MARGIN_CLKS <= 65535
        report "tdc_gpx_stop_cfg_decode: g_WINDOW_MARGIN_CLKS must be <= 65535"
        severity failure;

    o_stop_evt_tready <= '1';
    o_monotonic_violation_mask <= s_mono_viol_r;
    o_orphan_stop_evt_sticky <= s_orphan_evt_sticky_r;
    o_expected_final_valid <= s_expected_final_r;

    s_fire_match_now <= '1'
        when i_fire_count_tvalid = '1'
         and unsigned(i_fire_count_tdata(15 downto 0)) = i_current_fire_count
        else '0';

    s_stop_owned_now <= '1'
        when i_stop_evt_tvalid = '1'
         and i_fire_count_tlast = '0'
         and s_fire_match_now = '1'
         and s_window_active_r = '1'
        else '0';

    -- The data registers deliberately have no runtime enable. Only the valid
    -- bit is qualified by ownership, keeping the comparison cone narrow while
    -- preserving one accepted event per clock.
    p_owned_evt_stage : process(i_clk)
    begin
        if rising_edge(i_clk) then
            s_owned_evt_data_r  <= i_stop_evt_tdata;
            s_owned_evt_tuser_r <= i_stop_evt_tuser;
            if i_rst_n = '0' or i_shot_start_gated = '1' then
                s_owned_evt_valid_r <= '0';
            else
                s_owned_evt_valid_r <= s_stop_owned_now;
            end if;
        end if;
    end process p_owned_evt_stage;

    -- Runtime expected-count decoder and monotonic violation detector.
    -- The staged event is decoded once and feeds both functions.
    p_runtime_state : process(i_clk)
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
                s_max_range_axis_snap_r <= (others => '0');
                s_window_cap_r        <= (others => '0');
                s_orphan_evt_sticky_r <= '0';
                s_expected_final_r    <= '0';
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            else
                -- Round 13 follow-up (audit 5번): distance-bounded window.
                -- Snapshot the converted AXIS count at shot_start so a
                -- mid-shot config change cannot retroactively shift the
                -- window. Effective
                -- cap = snapshot + margin. If the snapshot is zero (distance
                -- check disabled), keep the window open — no timer-based
                -- close, only the next shot_start re-opens.
                if i_shot_start_gated = '1' then
                    s_max_range_axis_snap_r <= i_max_range_axis_clks;
                    s_window_cap_r     <= resize(i_max_range_axis_clks,
                                                  c_WINDOW_CNT_WIDTH)
                                          + to_unsigned(g_WINDOW_MARGIN_CLKS,
                                                         c_WINDOW_CNT_WIDTH);
                    s_window_active_r  <= '1';
                    s_window_cnt_r     <= (others => '0');
                elsif s_window_active_r = '1' and s_max_range_axis_snap_r /= 0 then
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
                    s_expected_final_r <= '0';
                    for i in 0 to c_N_CHIPS - 1 loop
                        o_expected_ififo1(i) <= (others => '0');
                        o_expected_ififo2(i) <= (others => '0');
                    end loop;
                    -- NOTE: sticky s_mono_viol_r is NOT cleared on shot boundary.
                    -- A violation in shot N should remain visible to SW across
                    -- subsequent shots; only full reset clears it.
                else
                    -- Round 13 follow-up (audit 5번): fire orphan sticky if
                    -- the beat arrived with no active shot window. Covers
                    -- the pre-first-shot case (window never opened) AND the
                    -- inter-shot gap case (distance window closed).
                    if i_stop_evt_tvalid = '1' and s_stop_owned_now = '0' then
                        s_orphan_evt_sticky_r <= '1';
                    end if;

                    -- Commit the previous cycle's owned event. Back-to-back
                    -- events advance through this stage without bubbles.
                    if s_owned_evt_valid_r = '1' then
                        for i in 0 to c_N_CHIPS - 1 loop
                            v_lo   := i * 8;
                            v_new1 := resize(unsigned(s_owned_evt_data_r(v_lo + 3 downto v_lo)), 8)
                                    + resize(unsigned(s_owned_evt_tuser_r(v_lo + 3 downto v_lo)), 8);
                            v_new2 := resize(unsigned(s_owned_evt_data_r(v_lo + 7 downto v_lo + 4)), 8)
                                    + resize(unsigned(s_owned_evt_tuser_r(v_lo + 7 downto v_lo + 4)), 8);
                            if s_track_r = '1' then
                                if v_new1 < s_prev_ififo1_r(i) or v_new2 < s_prev_ififo2_r(i) then
                                    s_mono_viol_r(i) <= '1';
                                end if;
                            end if;
                            s_prev_ififo1_r(i) <= v_new1;
                            s_prev_ififo2_r(i) <= v_new2;
                            o_expected_ififo1(i) <= v_new1;
                            o_expected_ififo2(i) <= v_new2;
                        end loop;
                        s_track_r <= '1';
                    end if;

                end if;

                if i_shot_start_gated = '0'
                   and i_fire_count_tvalid = '1' then
                    if s_fire_match_now = '0' then
                        s_orphan_evt_sticky_r <= '1';
                    elsif i_fire_count_tlast = '1' then
                        if s_window_active_r = '1' then
                            s_expected_final_r <= '1';
                        else
                            s_orphan_evt_sticky_r <= '1';
                        end if;
                    elsif i_stop_evt_tvalid = '0' then
                        s_orphan_evt_sticky_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_runtime_state;

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
        variable v_prev1  : t_prev_arr := (others => (others => '0'));
        variable v_prev2  : t_prev_arr := (others => (others => '0'));
        variable v_lo     : natural;
        variable v_track  : boolean := false;
        variable v_fire_count : unsigned(15 downto 0);
        variable v_stop_owned : boolean;
    begin
        if rising_edge(i_clk) then
            v_fire_count := unsigned(i_fire_count_tdata(15 downto 0));
            v_stop_owned := (i_stop_evt_tvalid = '1')
                and (i_fire_count_tvalid = '1')
                and (i_fire_count_tlast = '0')
                and (v_fire_count = i_current_fire_count)
                and (s_window_active_r = '1');

            if i_rst_n = '0' or i_shot_start_gated = '1' then
                v_track := false;
                for i in 0 to c_N_CHIPS - 1 loop
                    v_prev1(i) := (others => '0');
                    v_prev2(i) := (others => '0');
                end loop;
            elsif v_stop_owned then
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
