-- =============================================================================
-- tdc_gpx_chip_run.vhd
-- TDC-GPX Controller - Measurement Cycle Sub-FSM
-- =============================================================================
--
-- Purpose:
--   Handles the complete measurement cycle: armed → capture → drain → ALU.
--   Extracted from tdc_gpx_chip_ctrl to reduce FSM complexity.
--
-- FSM States (16):
--   ST_OFF → (start) → ST_ARMED → ST_CAPTURE → ST_DRAIN_LATCH →
--   ST_DRAIN_CHECK → ST_DRAIN_DECIDE →
--   ST_DRAIN_EF1/EF2/BURST_PLAN/BURST_ARM/BURST/FLUSH
--                  → ST_DRAIN_SETTLE →
--   ST_ALU_PULSE → ST_ALU_RECOVERY → (done/armed)
--   ST_OVERRUN_FLUSH (overrun recovery path)
--
-- Features:
--   - EF1-first round-robin IFIFO drain
--   - LF-based burst read optimization
--   - Per-IFIFO early drain_done (ififo1_done intermediate beat)
--   - Shot overrun detection and recovery
--   - cmd_stop deferred handling (via s_stop_pending_r) including
--     ST_OVERRUN_FLUSH in the defer targets (Round 2)
--   - drain_mode/n_drain_cap/bus_clk_div already snapshotted by coordinator
--   - s_range_active_r is cleared on ALL drain timeout exit paths as well as
--     normal completion (Round 1 #1)
--   - GPX EF1/EF2 are the only normal drain-completion authority. External
--     STOP edge counters cannot prove how many words the GPX accepted.
--   - LF1/LF2 and Reg6 Fill select safe burst opportunities; accepted bus
--     responses are the only actual drained-word count.
--
-- ST_CAPTURE cmd_stop policy — GRACEFUL (Q&A #29, Round 4):
--   On i_cmd_stop during ST_CAPTURE, we latch s_stop_pending_r and raise
--   s_stopdis_r but do NOT purge. Let the natural irflag path drain the
--   shot normally so captured data is preserved. After drain + ALU, the
--   pending flag routes the FSM to ST_OFF.
--   Fallback watchdog (65535 cycles): if irflag never arrives (chip
--   malfunction), fall back to the original purge path with timeout
--   cause "111" (capture_stop_fallback).
--
-- Shot overrun policy (i_shot_start during non-idle, non-completion):
--   Post-case override forces ST_OVERRUN_FLUSH. Round 5 #5 changed the
--   DRAIN-state branches to PRESERVE any bus response fired in the same
--   cycle (s_raw_valid_r stays '1'), so drain accounting remains
--   consistent and no beat is dropped. The ST_CAPTURE/LATCH/CHECK/SETTLE
--   branch still defensively clears s_raw_valid_r — there is no request
--   outstanding in those states, so any fire would be stale data.
--   Round 6 B2 removed the now-dead s_err_overrun_drop_r sticky / port.
--
-- Timeout cause codes (o_timeout_cause):
--   001 = raw_busy, 010 = ef1_rsp, 011 = ef2_rsp, 100 = burst_rsp,
--   101 = flush_rsp, 110 = overrun_flush, 111 = capture_stop_fallback
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_chip_run is
    generic (
        g_BUS_DATA_WIDTH    : natural  := c_TDC_BUS_WIDTH;
        g_RECOVERY_CLKS     : positive := c_DEFAULT_RECOVERY_CLKS;
        g_ALU_PULSE_CLKS    : positive := c_DEFAULT_ALU_PULSE_CLKS;
        -- Drain/flush watchdog headroom above max_range_tdc_clks.
        -- The cap used by the shared registered wait-timeout detector is computed at
        -- shot_start as (i_max_range_tdc_clks + g_DRAIN_MARGIN_CLKS), saturating
        -- at x"FFFF". Production top preserves the requested physical margin
        -- as g_TDC_CLK_MHZ changes; standalone default is 256 clocks @200 MHz.
        -- This covers bus
        -- roundtrip + ALU service + downstream backpressure jitter with
        -- generous margin while still tightening the timeout from the
        -- legacy fixed-65535 value that hid upstream hangs for ~327 µs.
        g_DRAIN_MARGIN_CLKS : positive := c_DEFAULT_DRAIN_MARGIN_CLKS;
        -- C02: guard time after each IFIFO read before trusting synchronized
        -- EF pins. Production top computes ceil(11.8 ns / TDC period) plus
        -- the fixed local 2-FF status synchronizer latency.
        g_EF_SYNC_GUARD_CLKS : positive := c_DEFAULT_EF_SYNC_GUARD_CLKS
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- Control from coordinator
        i_start             : in  std_logic;        -- enter ARMED
        i_cmd_stop          : in  std_logic;
        o_done              : out std_logic;         -- 1-clk: returned to idle
        o_armed             : out std_logic;         -- '1' while in ST_ARMED

        -- Snapshot config (latched by coordinator at cmd_start)
        i_drain_mode        : in  std_logic;
        i_n_drain_cap       : in  unsigned(3 downto 0);
        i_cfg_image         : in  t_cfg_image;       -- for Fill (Reg6)

        -- Shot trigger
        i_shot_start        : in  std_logic;

        -- Phase B: shot-bounded watchdog cap driver.
        -- Snapshotted on the shot_start transition (ST_ARMED → ST_CAPTURE),
        -- NOT cmd_start, so mid-shot SW changes to the CSR do not leak
        -- into a running shot's timeout behavior. Zero is treated as
        -- "disabled" (fallback to the legacy x"FFFF" cap).
        i_max_range_tdc_clks : in unsigned(15 downto 0);

        -- Deprecated compatibility inputs. Echo-receiver edge counts describe
        -- activity before the GPX input and are not authoritative IFIFO word
        -- counts. They are intentionally ignored by the drain FSM and will be
        -- removed from the hierarchy after interface migration.
        i_expected_ififo1   : in  unsigned(7 downto 0);
        i_expected_ififo2   : in  unsigned(7 downto 0);
        i_expected_final_valid : in std_logic;

        -- Bus request (to coordinator mux)
        o_bus_req_valid     : out std_logic;
        o_bus_req_rw        : out std_logic;
        o_bus_req_addr      : out std_logic_vector(3 downto 0);
        o_bus_req_wdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_bus_oen_permanent : out std_logic;
        o_bus_req_burst     : out std_logic;

        -- Bus response (from coordinator, gated)
        -- i_bus_rsp_valid = FIRE = tvalid AND tready (response consumed this
        --                  cycle). Use for capturing rdata and state advance.
        -- i_bus_rsp_pending = response has arrived at bus_phy/skid but may
        --                  not have been accepted yet (downstream backpressure
        --                  on raw hold register can hold tready low during
        --                  PH_RUN). Use for drain-completion checks and to
        --                  freeze wait watchdogs so a legitimate pending
        --                  response is not mistaken for bus hang (Round 5 #1).
        i_bus_rsp_valid     : in  std_logic;
        i_bus_rsp_pending   : in  std_logic;
        i_bus_rsp_rdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_bus_busy          : in  std_logic;

        -- Sync status pins
        i_ef1_sync          : in  std_logic;
        i_ef2_sync          : in  std_logic;
        i_irflag_sync       : in  std_logic;
        i_lf1_sync          : in  std_logic;
        i_lf2_sync          : in  std_logic;

        -- Raw word output (to coordinator passthrough → decoder_i_mode)
        o_raw_word          : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_raw_valid         : out std_logic;
        o_ififo_id          : out std_logic;
        o_drain_done        : out std_logic;
        o_ififo1_done_beat  : out std_logic;

        -- Backpressure from coordinator hold register
        i_raw_busy          : in  std_logic;    -- '1' = hold register full, stall drain

        -- Range / timeout status
        o_range_active      : out std_logic;           -- '1' during capture+drain window
        o_timeout           : out std_logic;           -- 1-clk pulse: abnormal drain exit
        o_timeout_cause     : out std_logic_vector(2 downto 0);  -- cause code (valid with o_timeout)
        -- Round 13 axis 1a: 1-clk pulse, co-asserts with o_drain_done when
        -- the completion was a mismatch fallback. Lets supervisor SW treat
        -- the frame as suspect without losing the downstream drain_done
        -- signal that cell_builder / face_assembler depend on.
        o_drain_done_faulted : out std_logic;
        -- Cause codes: "001"=raw_busy, "010"=ef1_rsp, "011"=ef2_rsp,
        --              "100"=burst_rsp, "101"=flush_rsp, "110"=overrun_flush,
        --              "111"=capture_stop_fallback (irflag missing after cmd_stop)
        -- (Round 5 #5 removed the overrun-drop path; Round 6 B2 deleted the
        --  matching o_err_overrun_drop port — never asserted after Round 5.)

        -- Pin outputs
        o_stopdis           : out std_logic;
        o_alutrigger        : out std_logic;
        o_busy              : out std_logic;

        -- Shot sequence
        o_shot_seq          : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_chip_run;

architecture rtl of tdc_gpx_chip_run is

    type t_run_state is (
        ST_OFF,
        ST_ARMED,
        ST_CAPTURE,
        ST_DRAIN_LATCH,
        ST_DRAIN_CHECK,
        ST_DRAIN_DECIDE,
        ST_DRAIN_EF1,
        ST_DRAIN_EF2,
        ST_DRAIN_BURST_PLAN,
        ST_DRAIN_BURST_ARM,
        ST_DRAIN_BURST,
        ST_DRAIN_FLUSH,
        ST_DRAIN_SETTLE,
        ST_ALU_PULSE,
        ST_ALU_RECOVERY,
        ST_OVERRUN_FLUSH
    );

    signal s_state_r : t_run_state := ST_OFF;

    constant c_RECOVERY_LAST  : unsigned(15 downto 0) := to_unsigned(g_RECOVERY_CLKS - 1, 16);
    constant c_ALU_PULSE_LAST : unsigned(15 downto 0) := to_unsigned(g_ALU_PULSE_CLKS - 1, 16);
    constant c_FLAG_SETTLE_LAST : unsigned(15 downto 0) :=
        to_unsigned(g_EF_SYNC_GUARD_CLKS - 1, 16);
    -- Phase B: shot-bounded watchdog cap.
    -- Function: (max_range + margin), saturating at x"FFFF". max_range=0
    -- is the "disabled" encoding and also returns x"FFFF" so pre-config
    -- behavior matches the legacy fixed cap.
    function fn_timeout_cap(
        max_range : unsigned(15 downto 0);
        margin    : natural
    ) return unsigned is
        variable v_sum : unsigned(16 downto 0);
    begin
        if max_range = 0 then
            return x"FFFF";
        else
            v_sum := resize(max_range, 17) + to_unsigned(margin, 17);
            if v_sum(16) = '1' then
                return x"FFFF";  -- saturate
            else
                return v_sum(15 downto 0);
            end if;
        end if;
    end function;

    type t_drain_eval is record
        ififo1_done     : std_logic;
        ififo2_done     : std_logic;
        ififo1_can_read : std_logic;
        ififo2_can_read : std_logic;
        ififo1_burst    : std_logic;
        ififo2_burst    : std_logic;
        ififo1_cap_hit  : std_logic;
        ififo2_cap_hit  : std_logic;
    end record;

    constant c_DRAIN_EVAL_ZERO : t_drain_eval := (
        ififo1_done     => '0',
        ififo2_done     => '0',
        ififo1_can_read => '0',
        ififo2_can_read => '0',
        ififo1_burst    => '0',
        ififo2_burst    => '0',
        ififo1_cap_hit  => '0',
        ififo2_cap_hit  => '0'
    );

    -- Keep the wide count/cap arithmetic on the input side of the DECIDE
    -- register boundary. CHECK registers only compact predicates; SETTLE
    -- reuses the same evaluation to avoid one idle state per read burst.
    function fn_drain_eval(
        purge_mode       : std_logic;
        drain_mode       : std_logic;
        fill             : unsigned(7 downto 0);
        ef1_sync          : std_logic;
        ef2_sync          : std_logic;
        lf1_sync          : std_logic;
        lf2_sync          : std_logic;
        n_drain_cap       : unsigned(3 downto 0);
        drain_cnt_ififo1  : unsigned(7 downto 0);
        drain_cnt_ififo2  : unsigned(7 downto 0);
        ififo1_capped     : std_logic;
        ififo2_capped     : std_logic
    ) return t_drain_eval is
        variable v_result               : t_drain_eval := c_DRAIN_EVAL_ZERO;
        variable v_cap                  : unsigned(7 downto 0);
        variable v_cap_enabled          : boolean;
        variable v_ififo1_done         : boolean;
        variable v_ififo2_done         : boolean;
        variable v_ififo1_can_read     : boolean;
        variable v_ififo2_can_read     : boolean;
        variable v_ififo1_cap_hit      : boolean;
        variable v_ififo2_cap_hit      : boolean;
    begin
        v_cap := shift_left(resize(n_drain_cap, 8), 2);
        v_cap_enabled := purge_mode = '0' and n_drain_cap /= "0000";

        -- A configured cap is a protection boundary, not evidence that the
        -- GPX FIFO is empty. Remember capped IFIFOs, finish the bounded output
        -- for the other IFIFO, then purge any physical remainder before the
        -- final faulted drain_done.
        v_ififo1_cap_hit := v_cap_enabled and ififo1_capped = '0'
            and drain_cnt_ififo1 >= v_cap and ef1_sync = '0';
        v_ififo2_cap_hit := v_cap_enabled and ififo2_capped = '0'
            and drain_cnt_ififo2 >= v_cap and ef2_sync = '0';

        if purge_mode = '1' then
            v_ififo1_done := ef1_sync = '1';
            v_ififo2_done := ef2_sync = '1';
        else
            v_ififo1_done := ef1_sync = '1' or ififo1_capped = '1';
            v_ififo2_done := ef2_sync = '1' or ififo2_capped = '1';
        end if;

        v_ififo1_can_read := not v_ififo1_done and not v_ififo1_cap_hit
            and ef1_sync = '0';
        v_ififo2_can_read := not v_ififo2_done and not v_ififo2_cap_hit
            and ef2_sync = '0';

        if v_ififo1_done then
            v_result.ififo1_done := '1';
        end if;
        if v_ififo2_done then
            v_result.ififo2_done := '1';
        end if;
        if v_ififo1_can_read then
            v_result.ififo1_can_read := '1';
        end if;
        if v_ififo2_can_read then
            v_result.ififo2_can_read := '1';
        end if;
        if v_ififo1_cap_hit then
            v_result.ififo1_cap_hit := '1';
        end if;
        if v_ififo2_cap_hit then
            v_result.ififo2_cap_hit := '1';
        end if;
        if purge_mode = '0' and drain_mode = '1' and fill >= 2
           and ef1_sync = '0' and lf1_sync = '1'
           and v_ififo1_can_read
           and ((not v_cap_enabled) or drain_cnt_ififo1 + 2 <= v_cap) then
            v_result.ififo1_burst := '1';
        end if;
        if purge_mode = '0' and drain_mode = '1' and fill >= 2
           and ef2_sync = '0' and lf2_sync = '1'
           and v_ififo2_can_read
           and ((not v_cap_enabled) or drain_cnt_ififo2 + 2 <= v_cap) then
            v_result.ififo2_burst := '1';
        end if;

        return v_result;
    end function;

    signal s_wait_cnt_r        : unsigned(15 downto 0) := (others => '0');
    -- Phase B: latched once per shot on the ST_ARMED→ST_CAPTURE edge.
    -- Init = x"FFFF" so pre-first-shot behavior matches the legacy cap.
    signal s_wait_cap_r        : unsigned(15 downto 0) := x"FFFF";
    signal s_wait_expired_r    : std_logic := '0';
    signal s_req_valid_r       : std_logic := '0';
    signal s_req_rw_r          : std_logic := '0';
    signal s_req_addr_r        : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r       : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_req_burst_r       : std_logic := '0';
    signal s_oen_permanent_r   : std_logic := '0';

    signal s_stopdis_r         : std_logic := '1';
    signal s_alutrigger_r      : std_logic := '0';
    signal s_busy_r            : std_logic := '0';
    signal s_done_r            : std_logic := '0';
    signal s_timeout_r         : std_logic := '0';
    signal s_timeout_cause_r   : std_logic_vector(2 downto 0) := (others => '0');
    -- One-cycle pulse co-asserted with drain_done when completion is faulted.
    -- The pulse is carried on the final raw control beat as tuser[5].
    signal s_drain_done_faulted_r : std_logic := '0';

    -- Round 9 #1: Secondary watchdog for "pending stuck" deadlock.
    -- Round 5 #1 made EF1/EF2/BURST freeze s_wait_cnt_r while i_bus_rsp_pending
    -- is '1' (protects against false bus-hang timeout when downstream
    -- backpressure is merely briefly holding tready low). But if pending
    -- stays '1' forever — e.g. raw FIFO permanently full with no consumer —
    -- chip_run would hang indefinitely. This counter increments while the
    -- FSM is in a wait-for-response drain state AND pending='1' AND no fire;
    -- it fires a timeout if downstream never drains.
    signal s_pending_stuck_cnt_r : unsigned(15 downto 0) := (others => '0');
    signal s_pending_stuck_expired_r : std_logic := '0';

    signal s_raw_word_r        : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_raw_valid_r       : std_logic := '0';
    signal s_ififo_id_r        : std_logic := '0';
    signal s_drain_done_r      : std_logic := '0';
    signal s_ififo1_done_beat_r : std_logic := '0';
    signal s_ififo1_done_sent_r : std_logic := '0';

    signal s_irflag_prev_r     : std_logic := '0';
    signal s_shot_seq_r        : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');

    signal s_drain_cnt_ififo1_r : unsigned(7 downto 0) := (others => '0');
    signal s_drain_cnt_ififo2_r : unsigned(7 downto 0) := (others => '0');
    signal s_ififo1_capped_r     : std_logic := '0';
    signal s_ififo2_capped_r     : std_logic := '0';
    signal s_drain_fault_pending_r : std_logic := '0';
    signal s_eval_r : t_drain_eval := c_DRAIN_EVAL_ZERO;

    signal s_fill_r            : unsigned(7 downto 0) := (others => '0');
    -- Number of responses still needed before the current burst closes.
    -- This replaces the count/limit pair: (count + 1 >= limit) is exactly
    -- equivalent to (remaining <= 1), with a much smaller timing cone.
    signal s_burst_remaining_r : unsigned(7 downto 0) := (others => '0');

    signal s_range_active_r    : std_logic := '0';
    signal s_purge_mode_r      : std_logic := '0';
    signal s_stop_pending_r    : std_logic := '0';
    signal s_overrun_deferred_r : std_logic := '0';

begin

    p_irflag_edge : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_irflag_prev_r <= '0';
            else
                s_irflag_prev_r <= i_irflag_sync;
            end if;
        end if;
    end process;

    -- Share the seven watchdog comparisons through one registered boundary.
    -- A timeout is therefore acted on one TDC clock after the counter reaches
    -- its cap. This removes the wide comparator from every FSM output-control
    -- cone while preserving the shot-bounded timeout contract.
    p_wait_expired : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_wait_expired_r <= '0';
            elsif s_wait_cnt_r = s_wait_cap_r then
                s_wait_expired_r <= '1';
            else
                s_wait_expired_r <= '0';
            end if;
        end if;
    end process p_wait_expired;

    -- Register the wide watchdog comparison before it reaches FSM control.
    -- The look-ahead threshold preserves the original timeout cycle: while
    -- the FSM advances FF_FE -> FF_FF, this flag becomes visible and the next
    -- cycle takes the same exit formerly selected by a direct FF_FF compare.
    p_pending_stuck_expired : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_pending_stuck_expired_r <= '0';
            elsif s_pending_stuck_cnt_r = x"FFFE" then
                s_pending_stuck_expired_r <= '1';
            else
                s_pending_stuck_expired_r <= '0';
            end if;
        end if;
    end process p_pending_stuck_expired;

    p_fsm : process(i_clk)
        variable v_eval        : t_drain_eval;
        variable v_cap_words   : unsigned(7 downto 0);
        variable v_burst_words : unsigned(7 downto 0);
    begin
        if rising_edge(i_clk) then
            v_eval := fn_drain_eval(
                purge_mode      => s_purge_mode_r,
                drain_mode      => i_drain_mode,
                fill            => s_fill_r,
                ef1_sync        => i_ef1_sync,
                ef2_sync        => i_ef2_sync,
                lf1_sync        => i_lf1_sync,
                lf2_sync        => i_lf2_sync,
                n_drain_cap     => i_n_drain_cap,
                drain_cnt_ififo1 => s_drain_cnt_ififo1_r,
                drain_cnt_ififo2 => s_drain_cnt_ififo2_r,
                ififo1_capped    => s_ififo1_capped_r,
                ififo2_capped    => s_ififo2_capped_r
            );
            if i_rst_n = '0' then
                s_state_r           <= ST_OFF;
                s_wait_cnt_r        <= (others => '0');
                s_wait_cap_r        <= x"FFFF";  -- Phase B: legacy-compat until first shot
                s_req_valid_r       <= '0';
                s_req_burst_r       <= '0';
                s_oen_permanent_r   <= '0';
                s_stopdis_r         <= '1';
                s_alutrigger_r      <= '0';
                s_busy_r            <= '0';
                s_done_r            <= '0';
                s_raw_word_r        <= (others => '0');
                s_raw_valid_r       <= '0';
                s_ififo_id_r        <= '0';
                s_drain_done_r      <= '0';
                s_ififo1_done_beat_r <= '0';
                s_ififo1_done_sent_r <= '0';
                s_shot_seq_r        <= (others => '0');
                s_drain_cnt_ififo1_r <= (others => '0');
                s_drain_cnt_ififo2_r <= (others => '0');
                s_ififo1_capped_r    <= '0';
                s_ififo2_capped_r    <= '0';
                s_drain_fault_pending_r <= '0';
                s_eval_r <= c_DRAIN_EVAL_ZERO;
                s_burst_remaining_r <= (others => '0');
                s_range_active_r    <= '0';
                s_purge_mode_r      <= '0';
                s_stop_pending_r    <= '0';
                s_overrun_deferred_r <= '0';
                s_timeout_r          <= '0';
                s_timeout_cause_r    <= (others => '0');
                s_pending_stuck_cnt_r <= (others => '0');
                s_drain_done_faulted_r <= '0';
            else
                s_raw_valid_r        <= '0';
                s_drain_done_r       <= '0';
                s_drain_done_faulted_r <= '0';  -- Round 13 axis 1a: 1-clk pulse
                s_timeout_r          <= '0';
                s_ififo1_done_beat_r <= '0';
                s_done_r             <= '0';

                -- Deferred stop latch
                -- ST_OVERRUN_FLUSH included so a 1-cycle i_cmd_stop during
                -- overrun recovery is not lost (previously fell through
                -- `when others => null`).
                if i_cmd_stop = '1' then
                    case s_state_r is
                        when ST_DRAIN_LATCH | ST_DRAIN_CHECK | ST_DRAIN_DECIDE
                           | ST_DRAIN_EF1   | ST_DRAIN_EF2
                           | ST_DRAIN_BURST_PLAN | ST_DRAIN_BURST_ARM
                           | ST_DRAIN_BURST | ST_DRAIN_FLUSH
                           | ST_DRAIN_SETTLE
                           | ST_ALU_PULSE   | ST_ALU_RECOVERY
                           | ST_OVERRUN_FLUSH =>
                            s_stop_pending_r <= '1';
                        when others =>
                            null;
                    end case;
                end if;

                case s_state_r is

                    when ST_OFF =>
                        s_busy_r <= '0';
                        if i_start = '1' then
                            s_stopdis_r          <= '0';
                            s_stop_pending_r     <= '0';
                            s_ififo1_done_sent_r <= '0';
                            s_state_r            <= ST_ARMED;
                        end if;

                    when ST_ARMED =>
                        if i_cmd_stop = '1' then
                            s_stopdis_r <= '1';
                            s_done_r    <= '1';
                            s_state_r   <= ST_OFF;
                        elsif i_shot_start = '1' then
                            s_busy_r             <= '1';
                            s_range_active_r     <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_ififo1_capped_r     <= '0';
                            s_ififo2_capped_r     <= '0';
                            s_drain_fault_pending_r <= '0';
                            -- Start a fresh physical-drain accounting window.
                            -- Completion remains owned by the synchronized GPX
                            -- empty flags sampled after IrFlag.
                            s_ififo1_done_sent_r <= '0';
                            -- Phase B: shot-bounded watchdog cap snapshot.
                            -- Must be shot_start (not cmd_start) so a mid-shot
                            -- SW max_range update cannot change this shot's
                            -- timeout horizon. One registered detector shares
                            -- the cap comparison across the drain/flush path.
                            s_wait_cap_r         <= fn_timeout_cap(
                                                       i_max_range_tdc_clks,
                                                       g_DRAIN_MARGIN_CLKS);
                            s_state_r            <= ST_CAPTURE;
                        end if;

                    when ST_CAPTURE =>
                        -- Graceful stop (Q&A #29, Option A):
                        -- On cmd_stop, latch pending + enable chip-level stopdis
                        -- and start a local fallback watchdog. Let the natural
                        -- irflag path drive the drain so the CURRENT shot's
                        -- already-captured data is preserved (no purge). The
                        -- pending flag is honored at ALU_RECOVERY exit → ST_OFF.
                        --
                        -- Fallback: if irflag never arrives (chip malfunction),
                        -- the watchdog below forces the original purge path.
                        if i_cmd_stop = '1' then
                            s_stop_pending_r <= '1';
                            s_stopdis_r      <= '1';
                            s_wait_cnt_r     <= (others => '0');  -- start watchdog
                        end if;

                        if i_irflag_sync = '1' and s_irflag_prev_r = '0' then
                            s_fill_r <= unsigned(i_cfg_image(6)(
                                c_REG6_LF_THRESH_HI downto c_REG6_LF_THRESH_LO));
                            if i_drain_mode = '1' then
                                s_oen_permanent_r <= '1';
                            end if;
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_ififo1_capped_r     <= '0';
                            s_ififo2_capped_r     <= '0';
                            s_drain_fault_pending_r <= '0';
                            s_wait_cnt_r         <= (others => '0');
                            s_state_r <= ST_DRAIN_LATCH;
                        elsif s_stop_pending_r = '1' then
                            -- Fallback watchdog: irflag never arrived after
                            -- cmd_stop (chip malfunction). Fall back to the
                            -- original immediate-purge path so we don't hang.
                            if s_wait_expired_r = '1' then
                                s_range_active_r     <= '0';
                                s_raw_valid_r        <= '0';
                                s_drain_cnt_ififo1_r <= (others => '0');
                                s_drain_cnt_ififo2_r <= (others => '0');
                                s_ififo1_capped_r     <= '0';
                                s_ififo2_capped_r     <= '0';
                                s_drain_done_r       <= '0';
                                s_purge_mode_r       <= '1';
                                s_timeout_r          <= '1';
                                s_timeout_cause_r    <= "111";  -- capture-stop fallback
                                s_wait_cnt_r         <= (others => '0');
                                if i_drain_mode = '1' then
                                    s_oen_permanent_r <= '1';
                                else
                                    s_oen_permanent_r <= '0';
                                end if;
                                s_state_r <= ST_DRAIN_SETTLE;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;
                        end if;

                    when ST_DRAIN_LATCH =>
                        -- IrFlag has ended capture. Move directly to the first
                        -- synchronized EF/LF decision; no external edge-count
                        -- settling interval participates in this contract.
                        s_wait_cnt_r        <= (others => '0');
                        s_state_r           <= ST_DRAIN_CHECK;

                    when ST_DRAIN_CHECK =>
                      if i_raw_busy = '0' then
                        s_wait_cnt_r <= (others => '0');  -- clear raw_busy watchdog
                        -- Register only compact predicates here. The following
                        -- DECIDE state drives requests and state controls from
                        -- these one-bit values, cutting the drain counters and
                        -- cap arithmetic out of those timing cones.
                        s_eval_r <= v_eval;
                        s_state_r <= ST_DRAIN_DECIDE;
                      else
                        -- raw_busy watchdog: abort drain if stalled too long
                        s_wait_cnt_r <= s_wait_cnt_r + 1;
                        if s_wait_expired_r = '1' then
                            s_oen_permanent_r <= '0';
                            s_range_active_r  <= '0';
                            s_drain_done_r    <= '1';
                            s_ififo_id_r      <= '1';
                            s_timeout_r       <= '1';
                            s_timeout_cause_r <= "001";  -- raw_busy
                            s_state_r         <= ST_ALU_PULSE;
                        end if;
                      end if; -- i_raw_busy = '0'

                    when ST_DRAIN_DECIDE =>
                        -- Early IFIFO1 done beat
                        if s_eval_r.ififo1_done = '1'
                           and s_eval_r.ififo2_done = '0'
                           and s_ififo1_done_sent_r = '0'
                           and s_purge_mode_r = '0' then
                            s_ififo1_done_beat_r <= '1';
                            s_ififo_id_r         <= '0';
                            s_ififo1_done_sent_r <= '1';
                        end if;

                        -- A configured cap is a truncation boundary, never
                        -- proof that the physical IFIFO is empty.
                        if s_eval_r.ififo1_cap_hit = '1'
                           or s_eval_r.ififo2_cap_hit = '1' then
                            if s_eval_r.ififo1_cap_hit = '1' then
                                s_ififo1_capped_r <= '1';
                            end if;
                            if s_eval_r.ififo2_cap_hit = '1' then
                                s_ififo2_capped_r <= '1';
                            end if;
                            s_drain_fault_pending_r <= '1';
                            s_state_r <= ST_DRAIN_CHECK;

                        -- Completion
                        elsif s_eval_r.ififo1_done = '1'
                           and s_eval_r.ififo2_done = '1' then
                            s_wait_cnt_r      <= (others => '0');
                            if s_purge_mode_r = '0'
                               and s_drain_fault_pending_r = '1'
                               and (i_ef1_sync = '0' or i_ef2_sync = '0') then
                                -- The bounded output is complete, but the GPX
                                -- still owns unread words. Purge that tail so
                                -- the next shot starts from empty FIFOs.
                                s_purge_mode_r <= '1';
                                s_state_r      <= ST_DRAIN_SETTLE;
                            else
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                if s_purge_mode_r = '1' then
                                    s_purge_mode_r <= '0';
                                end if;
                                if s_drain_fault_pending_r = '1' then
                                    s_drain_done_faulted_r <= '1';
                                end if;
                                s_drain_fault_pending_r <= '0';
                                s_drain_done_r <= '1';
                                s_ififo_id_r   <= '1';
                                s_state_r      <= ST_ALU_PULSE;
                            end if;

                        -- Burst IFIFO1
                        elsif s_eval_r.ififo1_burst = '1' then
                            s_ififo_id_r  <= '0';
                            s_state_r     <= ST_DRAIN_BURST_PLAN;

                        -- Burst IFIFO2
                        elsif s_eval_r.ififo2_burst = '1' then
                            s_ififo_id_r  <= '1';
                            s_state_r     <= ST_DRAIN_BURST_PLAN;

                        -- EF single IFIFO1
                        elsif s_eval_r.ififo1_can_read = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG8_IFIFO1;
                            s_ififo_id_r  <= '0';
                            s_state_r     <= ST_DRAIN_EF1;

                        -- EF single IFIFO2
                        elsif s_eval_r.ififo2_can_read = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG9_IFIFO2;
                            s_ififo_id_r  <= '1';
                            s_state_r     <= ST_DRAIN_EF2;

                        else
                            -- Fallback completion
                            s_oen_permanent_r <= '0';
                            s_range_active_r  <= '0';
                            s_wait_cnt_r      <= (others => '0');
                            if s_purge_mode_r = '1' then
                                s_purge_mode_r <= '0';
                            end if;
                            -- A cap-triggered purge preserves the physical FIFO
                            -- boundary but means output data was truncated.
                            -- Carry that condition on the final control beat.
                            if s_drain_fault_pending_r = '1' then
                                -- Round 13 axis 1a: co-assert "faulted"
                                -- pulse so SW can distinguish a clean
                                -- drain_done from one that actually
                                -- reached this fallback after truncation.
                                -- The downstream cell_builder /
                                -- face_assembler see drain_done as
                                -- normal but supervisor-level SW can
                                -- flag the frame as suspect.
                                s_drain_done_faulted_r <= '1';
                            end if;
                            s_drain_fault_pending_r <= '0';
                            -- Always emit final drain_done (normal + purge)
                            s_drain_done_r <= '1';
                            s_ififo_id_r   <= '1';
                            s_state_r      <= ST_ALU_PULSE;
                        end if;

                    when ST_DRAIN_BURST_PLAN =>
                        -- LF guarantees Reg6 Fill words are available. A
                        -- configured output cap may shorten this burst.
                        v_cap_words   := shift_left(resize(i_n_drain_cap, 8), 2);
                        v_burst_words := s_fill_r;
                        if s_ififo_id_r = '0' then
                            if i_n_drain_cap /= 0 then
                                if v_cap_words - s_drain_cnt_ififo1_r < v_burst_words then
                                    v_burst_words := v_cap_words - s_drain_cnt_ififo1_r;
                                end if;
                            end if;
                        else
                            if i_n_drain_cap /= 0 then
                                if v_cap_words - s_drain_cnt_ififo2_r < v_burst_words then
                                    v_burst_words := v_cap_words - s_drain_cnt_ififo2_r;
                                end if;
                            end if;
                        end if;
                        s_burst_remaining_r <= v_burst_words;
                        s_state_r <= ST_DRAIN_BURST_ARM;

                    when ST_DRAIN_BURST_ARM =>
                        -- Convert total words to the existing "responses after
                        -- first request" countdown. CHECK guarantees >=2.
                        s_burst_remaining_r <= s_burst_remaining_r - 1;
                        s_req_valid_r <= '1';
                        s_req_burst_r <= '1';
                        s_req_rw_r    <= '0';
                        if s_ififo_id_r = '0' then
                            s_req_addr_r <= c_TDC_REG8_IFIFO1;
                        else
                            s_req_addr_r <= c_TDC_REG9_IFIFO2;
                        end if;
                        s_state_r <= ST_DRAIN_BURST;

                    when ST_DRAIN_EF1 =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r        <= '0';
                            s_raw_word_r         <= i_bus_rsp_rdata;
                            if s_purge_mode_r = '0' then
                                s_raw_valid_r    <= '1';
                            end if;
                            s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            s_wait_cnt_r         <= (others => '0');
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_state_r            <= ST_DRAIN_SETTLE;
                        elsif i_bus_rsp_pending = '1' then
                            -- v010: bus response may already be accepted into
                            -- chip_ctrl's skid while i_bus_rsp_valid is still
                            -- one cycle away. Drop the single-read request as
                            -- soon as pending is observed so bus_phy cannot
                            -- re-accept the same IFIFO read.
                            s_req_valid_r <= '0';
                            -- Response already at bus_phy/skid but tready held
                            -- low by raw hold backpressure. Hold bus-hang
                            -- watchdog (s_wait_cnt_r) but advance pending-stuck
                            -- watchdog (Round 9 #1). If pending persists to the
                            -- full 16-bit count, downstream has deadlocked —
                            -- force a safe drain exit so the coordinator can
                            -- recover the chip via soft_reset.
                            if s_pending_stuck_expired_r = '1' then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "001";  -- raw_busy (downstream deadlock)
                                s_pending_stuck_cnt_r <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            else
                                s_pending_stuck_cnt_r <= s_pending_stuck_cnt_r + 1;
                            end if;
                        else
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_expired_r = '1' then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "010";  -- ef1_rsp
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_EF2 =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r        <= '0';
                            s_raw_word_r         <= i_bus_rsp_rdata;
                            if s_purge_mode_r = '0' then
                                s_raw_valid_r    <= '1';
                            end if;
                            s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            s_wait_cnt_r         <= (others => '0');
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_state_r            <= ST_DRAIN_SETTLE;
                        elsif i_bus_rsp_pending = '1' then
                            -- v010: see ST_DRAIN_EF1. Pending means the
                            -- single read has reached the response path.
                            s_req_valid_r <= '0';
                            -- Round 9 #1 secondary watchdog (see ST_DRAIN_EF1)
                            if s_pending_stuck_expired_r = '1' then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "001";  -- raw_busy
                                s_pending_stuck_cnt_r <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            else
                                s_pending_stuck_cnt_r <= s_pending_stuck_cnt_r + 1;
                            end if;
                        else
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_expired_r = '1' then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "011";  -- ef2_rsp
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_BURST =>
                        if i_bus_rsp_valid = '1' then
                            s_raw_word_r  <= i_bus_rsp_rdata;
                            s_raw_valid_r <= '1';
                            if s_burst_remaining_r /= 0 then
                                s_burst_remaining_r <= s_burst_remaining_r - 1;
                            end if;
                            s_wait_cnt_r  <= (others => '0');
                            s_pending_stuck_cnt_r <= (others => '0');
                            if s_ififo_id_r = '0' then
                                s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            else
                                s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            end if;
                            if s_burst_remaining_r(7 downto 1) = 0 then
                                s_req_burst_r <= '0';
                                s_req_valid_r <= '0';
                                s_state_r     <= ST_DRAIN_FLUSH;
                            end if;
                        elsif i_bus_rsp_pending = '1' then
                            -- v010: pre-close the burst request when the
                            -- response currently pending in the skid is the
                            -- final planned burst beat. This prevents bus_phy
                            -- from launching one extra IFIFO read while the
                            -- final response waits one cycle in the skid.
                            if s_burst_remaining_r(7 downto 1) = 0 then
                                s_req_burst_r <= '0';
                                s_req_valid_r <= '0';
                            end if;
                            -- Round 9 #1 secondary watchdog
                            if s_pending_stuck_expired_r = '1' then
                                s_req_burst_r     <= '0';
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "001";  -- raw_busy
                                s_pending_stuck_cnt_r <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            else
                                s_pending_stuck_cnt_r <= s_pending_stuck_cnt_r + 1;
                            end if;
                        else
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_expired_r = '1' then
                                s_req_burst_r     <= '0';
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "100";  -- burst_rsp
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_FLUSH =>
                        if i_bus_rsp_valid = '1' then
                            s_raw_word_r  <= i_bus_rsp_rdata;
                            s_raw_valid_r <= '1';
                            s_wait_cnt_r  <= (others => '0');
                            s_pending_stuck_cnt_r <= (others => '0');
                            if s_ififo_id_r = '0' then
                                s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            else
                                s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            end if;
                        elsif i_bus_rsp_pending = '0' then
                            -- Nothing lingering at bus_phy/skid; advance watchdog
                            -- only when not being stalled by backpressure.
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            s_pending_stuck_cnt_r <= (others => '0');
                        else
                            -- Round 10 #1: pending stays high → raw path may be
                            -- permanently backed up. Independent pending-stuck
                            -- watchdog (separate from s_wait_cnt_r) forces an
                            -- exit if pending never releases, matching the
                            -- Round 9 #1 pattern for EF/BURST states.
                            if s_pending_stuck_expired_r = '1' then
                                s_oen_permanent_r <= '0';
                                s_range_active_r  <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "001";  -- raw_busy (downstream deadlock)
                                s_pending_stuck_cnt_r <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            else
                                s_pending_stuck_cnt_r <= s_pending_stuck_cnt_r + 1;
                            end if;
                        end if;
                        -- Flush complete only when bus is idle AND no response
                        -- is still pending anywhere in the bus_phy/skid path.
                        -- Using i_bus_rsp_pending (not i_bus_rsp_valid) prevents
                        -- premature exit while downstream backpressure holds a
                        -- response unfired in the skid (Round 5 #2).
                        if i_bus_busy = '0' and i_bus_rsp_pending = '0' then
                            s_wait_cnt_r <= (others => '0');
                            s_pending_stuck_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_SETTLE;
                        elsif s_wait_expired_r = '1' then
                            -- Bus hung during flush: force completion
                            s_oen_permanent_r <= '0';
                            s_range_active_r  <= '0';
                            s_drain_done_r    <= '1';
                            s_ififo_id_r      <= '1';
                            s_timeout_r       <= '1';
                            s_timeout_cause_r <= "101";  -- flush_rsp
                            s_state_r         <= ST_ALU_PULSE;
                        end if;

                    when ST_DRAIN_SETTLE =>
                        if s_wait_cnt_r = c_FLAG_SETTLE_LAST then
                            s_wait_cnt_r <= (others => '0');
                            if i_raw_busy = '0' then
                                -- The EF guard already consumed the wait
                                -- interval. Capture the same registered
                                -- predicates as CHECK and continue directly,
                                -- avoiding one idle clock after every burst.
                                s_eval_r <= v_eval;
                                s_state_r <= ST_DRAIN_DECIDE;
                            else
                                -- CHECK owns the bounded raw-busy watchdog.
                                s_state_r <= ST_DRAIN_CHECK;
                            end if;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_ALU_PULSE =>
                        s_alutrigger_r <= '1';
                        if s_wait_cnt_r = c_ALU_PULSE_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_ALU_RECOVERY;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_ALU_RECOVERY =>
                        s_alutrigger_r <= '0';
                        if s_wait_cnt_r = c_RECOVERY_LAST then
                            s_wait_cnt_r <= (others => '0');
                            if s_overrun_deferred_r = '1' then
                                s_overrun_deferred_r <= '0';
                                s_raw_valid_r        <= '0';
                                s_range_active_r     <= '1';
                                s_drain_cnt_ififo1_r <= (others => '0');
                                s_drain_cnt_ififo2_r <= (others => '0');
                                s_ififo1_capped_r     <= '0';
                                s_ififo2_capped_r     <= '0';
                                s_drain_fault_pending_r <= '0';
                                s_drain_done_r       <= '0';
                                s_purge_mode_r       <= '1';
                                if i_drain_mode = '1' then
                                    s_oen_permanent_r <= '1';
                                else
                                    s_oen_permanent_r <= '0';
                                end if;
                                s_state_r <= ST_DRAIN_SETTLE;
                            elsif s_stop_pending_r = '1' then
                                s_stop_pending_r <= '0';
                                s_stopdis_r      <= '1';
                                s_busy_r         <= '0';
                                s_done_r         <= '1';
                                s_state_r        <= ST_OFF;
                            else
                                s_shot_seq_r <= s_shot_seq_r + 1;
                                s_busy_r     <= '0';
                                s_ififo1_done_sent_r <= '0';
                                s_state_r    <= ST_ARMED;
                            end if;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_OVERRUN_FLUSH =>
                        s_raw_valid_r <= '0';
                        -- Use i_bus_rsp_pending (not i_bus_rsp_valid) so flush
                        -- doesn't exit while a response is still lingering at
                        -- bus_phy/skid unfired (Round 5 #2).
                        if i_bus_busy = '0' and i_bus_rsp_pending = '0' then
                            s_purge_mode_r       <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_ififo1_capped_r     <= '0';
                            s_ififo2_capped_r     <= '0';
                            s_drain_fault_pending_r <= '0';
                            if i_drain_mode = '1' then
                                s_oen_permanent_r <= '1';
                            end if;
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_SETTLE;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_expired_r = '1' then
                                -- Overrun flush timeout: force to drain settle
                                s_oen_permanent_r <= '0';
                                s_purge_mode_r    <= '1';
                                s_timeout_r       <= '1';
                                s_timeout_cause_r <= "110";  -- overrun_flush
                                s_wait_cnt_r      <= (others => '0');
                                s_state_r         <= ST_DRAIN_SETTLE;
                            end if;
                        end if;

                end case;

                -- Shot overrun handler (POST-CASE OVERRIDE — highest priority)
                -- Overwrites state/req set by the main case above. Same-cycle
                -- bus-response handling (Round 5 #5):
                --   * ST_DRAIN_EF1/EF2/BURST/FLUSH: the main case has just
                --     captured a legitimate same-shot drain beat into
                --     s_raw_word_r / s_raw_valid_r and incremented the drain
                --     counter. Preserve that beat (do NOT clear s_raw_valid_r)
                --     so drain accounting stays consistent. Only the state
                --     transitions to ST_OVERRUN_FLUSH.
                --   * ST_CAPTURE/LATCH/CHECK/DECIDE/SETTLE: no request is outstanding
                --     in these states, so any fire here would be stale. Keep
                --     the defensive clear of s_raw_valid_r.
                -- Round 6 B2: the s_err_overrun_drop_r sticky + o_err_overrun
                -- _drop port were deleted. With the preservation policy above
                -- there are no drops to report, so the signal was dead code.
                if i_shot_start = '1' and s_state_r /= ST_OFF
                   and s_state_r /= ST_ARMED then
                    case s_state_r is
                        when ST_CAPTURE | ST_DRAIN_LATCH
                           | ST_DRAIN_CHECK | ST_DRAIN_DECIDE
                           | ST_DRAIN_BURST_PLAN
                           | ST_DRAIN_BURST_ARM | ST_DRAIN_SETTLE =>
                            s_req_valid_r        <= '0';
                            s_req_burst_r        <= '0';
                            s_raw_valid_r        <= '0';
                            s_range_active_r     <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_ififo1_capped_r     <= '0';
                            s_ififo2_capped_r     <= '0';
                            s_drain_fault_pending_r <= '0';
                            s_overrun_deferred_r <= '0';
                            s_state_r            <= ST_OVERRUN_FLUSH;
                        when ST_DRAIN_EF1 | ST_DRAIN_EF2
                           | ST_DRAIN_BURST | ST_DRAIN_FLUSH =>
                            -- Preserve same-cycle fired beat; otherwise clear.
                            if i_bus_rsp_valid = '0' then
                                s_raw_valid_r    <= '0';
                            end if;
                            s_req_burst_r        <= '0';
                            s_req_valid_r        <= '0';
                            s_oen_permanent_r    <= '0';
                            s_overrun_deferred_r <= '0';
                            s_state_r            <= ST_OVERRUN_FLUSH;
                        when ST_ALU_PULSE | ST_ALU_RECOVERY =>
                            s_overrun_deferred_r <= '1';
                        when others =>
                            null;
                    end case;
                end if;

            end if;
        end if;
    end process p_fsm;

    o_bus_req_valid     <= s_req_valid_r;
    o_bus_req_rw        <= s_req_rw_r;
    o_bus_req_addr      <= s_req_addr_r;
    o_bus_req_wdata     <= s_req_wdata_r;
    o_bus_oen_permanent <= s_oen_permanent_r;
    o_bus_req_burst     <= s_req_burst_r;
    o_raw_word          <= s_raw_word_r;
    o_raw_valid         <= s_raw_valid_r;
    o_ififo_id          <= s_ififo_id_r;
    o_drain_done        <= s_drain_done_r;
    o_ififo1_done_beat  <= s_ififo1_done_beat_r;
    o_stopdis           <= s_stopdis_r;
    o_alutrigger        <= s_alutrigger_r;
    o_busy              <= s_busy_r;
    o_shot_seq          <= s_shot_seq_r;
    o_done              <= s_done_r;
    o_range_active      <= s_range_active_r;
    o_timeout           <= s_timeout_r;
    o_timeout_cause     <= s_timeout_cause_r;
    o_drain_done_faulted <= s_drain_done_faulted_r;
    o_armed             <= '1' when s_state_r = ST_ARMED else '0';

end architecture rtl;
