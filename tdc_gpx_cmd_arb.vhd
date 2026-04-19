-- =============================================================================
-- tdc_gpx_cmd_arb.vhd
-- TDC-GPX Controller - Command Arbitration
-- =============================================================================
--
-- Purpose:
--   Per-chip pending latch with multi-chip parallel dispatch, cfg_write
--   pending/gating with full pipeline idle, and IRQ/loop-resume outputs.
--
--   Key features:
--     - cfg_write pending latch + full pipeline idle gating (unchanged)
--     - Per-chip pending tracking for multi-chip parallel register access
--     - chip_mask-based target selection (parallel dispatch to N chips)
--     - All-done IRQ pulse and measurement loop resume output
--     - Per-chip outstanding tracking (replaces global 1-outstanding lock)
--
-- Reg timeout policy (Round 2 #3):
--   The global reg-access timeout counter only runs once every targeted
--   chip has been dispatched (s_reg_pending_r all-zero). Previously the
--   counter ran from s_reg_active_r='1', so busy-chip dispatch waits
--   consumed the timeout budget before any real response could arrive.
--   Now the counter measures response latency only.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cmd_arb is
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- Commands from CSR
        i_cmd_start          : in  std_logic;
        i_cmd_start_accepted : in  std_logic;
        i_cmd_stop           : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;
        i_cmd_cfg_write      : in  std_logic;
        i_cmd_reg_read       : in  std_logic;
        i_cmd_reg_write      : in  std_logic;
        i_cmd_reg_chip       : in  unsigned(1 downto 0);             -- backward compat
        i_cmd_reg_chip_address  : in  std_logic_vector(c_N_CHIPS - 1 downto 0);  -- target chip mask
        i_cmd_reg_addr       : in  std_logic_vector(3 downto 0);     -- register address

        -- Pipeline idle
        i_chip_busy          : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_face_asm_idle      : in  std_logic;
        i_face_asm_fall_idle : in  std_logic;
        i_hdr_idle           : in  std_logic;
        i_hdr_fall_idle      : in  std_logic;

        -- Per-chip reg done (from chip_ctrl)
        i_cmd_reg_done       : in  std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Gated outputs
        o_cmd_cfg_write_g    : out std_logic;
        o_cmd_reg_read_g     : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_cmd_reg_write_g    : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_reg_outstanding    : out std_logic;
        o_outstanding_chip   : out unsigned(1 downto 0);             -- backward compat

        -- New: IRQ / done / loop-resume
        o_cmd_reg_done_pulse : out std_logic;                        -- all-done 1-clk pulse
        o_cmd_reg_done_chip  : out unsigned(1 downto 0);             -- which chip just completed
        o_reg_loop_resume    : out std_logic;                        -- measurement loop resume 1-clk
        o_cmd_reg_addr_out   : out std_logic_vector(3 downto 0);     -- latched addr for csr_chip STAT

        -- Timeout status
        o_reg_timeout        : out std_logic;                        -- sticky: reg access timeout occurred
        o_reg_timeout_mask   : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- which chips timed out

        -- Overlap handling (Round 5 #10)
        --   1-depth pending queue absorbs a new reg request that arrives while
        --   another transaction is still in flight. A second overlapping
        --   request (queue already occupied) raises o_reg_rejected sticky so
        --   SW knows at least one request was lost.
        o_reg_rejected       : out std_logic;

        -- Zero-mask status (Round 5 #17 — was sim-only)
        --   Sticky: a reg read/write request arrived with i_cmd_reg_chip_address
        --   = 0. Previously ignored with a sim-only assert; now also surfaced
        --   as a runtime-observable flag. Cleared only by reset.
        o_reg_zero_mask      : out std_logic
    );
end entity tdc_gpx_cmd_arb;

architecture rtl of tdc_gpx_cmd_arb is

    constant C_ZEROS : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- cfg_write pending (unchanged from original)
    -- -------------------------------------------------------------------------
    signal s_cfg_write_pending_r : std_logic := '0';
    signal s_cmd_cfg_write_g_i   : std_logic;

    -- -------------------------------------------------------------------------
    -- Per-chip multi-chip reg dispatch
    -- -------------------------------------------------------------------------
    signal s_reg_active_r        : std_logic := '0';                          -- any reg op in progress
    signal s_reg_pending_rw_r    : std_logic := '0';                          -- '0'=read, '1'=write
    signal s_reg_pending_addr_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_reg_target_mask_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_reg_done_mask_r     : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_reg_dispatched_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_reg_pending_r       : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Dispatch pulses (1-clk each, registered)
    signal s_dispatch_pulse_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Done tracking
    signal s_done_pulse_r        : std_logic := '0';
    signal s_loop_resume_r       : std_logic := '0';
    signal s_done_chip_r         : unsigned(1 downto 0) := (others => '0');

    -- Backward compat: outstanding_chip tracks last chip that was dispatched
    signal s_outstanding_chip_r  : unsigned(1 downto 0) := (others => '0');

    -- Reset condition
    signal s_reset               : std_logic;

    -- All-done detection
    signal s_all_done            : std_logic;

    -- Reg access timeout (prevents permanent hang if chip never responds)
    signal s_reg_timeout_cnt_r   : unsigned(15 downto 0) := (others => '0');
    signal s_reg_timeout_r       : std_logic := '0';  -- sticky: timeout occurred
    signal s_reg_timeout_mask_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- 1-depth pending queue for overlapping reg requests (Round 5 #10)
    -- -------------------------------------------------------------------------
    signal s_reg_queue_valid_r   : std_logic := '0';
    signal s_reg_queue_mask_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_reg_queue_rw_r      : std_logic := '0';
    signal s_reg_queue_addr_r    : std_logic_vector(3 downto 0) := (others => '0');
    signal s_reg_rejected_r      : std_logic := '0';  -- sticky: 2nd overlap lost
    signal s_reg_zero_mask_r     : std_logic := '0';  -- sticky: zero-mask request observed (Round 5 #17)

    -- Round 9 #3: dispatch-wait watchdog. The existing s_reg_timeout_cnt_r
    -- only runs once every target chip has been dispatched (pending=0). If a
    -- target chip stays i_chip_busy forever, the dispatch itself never
    -- happens and no timeout is counted. This counter runs from transaction
    -- acceptance until all target chips are dispatched; on overflow we
    -- abort the transaction with reg_timeout and mark undispatched chips in
    -- the timeout mask.
    signal s_dispatch_wait_cnt_r : unsigned(15 downto 0) := (others => '0');

begin

    s_reset <= '1' when i_rst_n = '0' or i_cmd_soft_reset = '1' or i_cmd_stop = '1'
               else '0';

    -- =========================================================================
    -- cfg_write pending + full pipeline idle gating (preserved exactly)
    -- =========================================================================
    p_cfg_write_pending : process(i_clk)
        variable v_pipeline_idle : boolean;
    begin
        if rising_edge(i_clk) then
            if s_reset = '1' then
                s_cfg_write_pending_r <= '0';
            else
                v_pipeline_idle := i_chip_busy = C_ZEROS
                               and i_face_asm_idle = '1'
                               and i_face_asm_fall_idle = '1'
                               and i_hdr_idle = '1'
                               and i_hdr_fall_idle = '1'
                               and i_cmd_start = '0'
                               and i_cmd_start_accepted = '0';
                if i_cmd_cfg_write = '1' and not v_pipeline_idle then
                    s_cfg_write_pending_r <= '1';
                elsif s_cfg_write_pending_r = '1' and v_pipeline_idle then
                    s_cfg_write_pending_r <= '0';
                end if;
            end if;
        end if;
    end process;

    s_cmd_cfg_write_g_i <= (i_cmd_cfg_write or s_cfg_write_pending_r)
                           when i_chip_busy = C_ZEROS
                            and i_face_asm_idle = '1'
                            and i_face_asm_fall_idle = '1'
                            and i_hdr_idle = '1'
                            and i_hdr_fall_idle = '1'
                            and i_cmd_start = '0'
                            and i_cmd_start_accepted = '0'
                           else '0';

    -- =========================================================================
    -- All-done detection (combinational)
    -- =========================================================================
    s_all_done <= '1' when s_reg_active_r = '1'
                       and s_reg_target_mask_r /= C_ZEROS
                       and s_reg_done_mask_r = s_reg_target_mask_r
                 else '0';

    -- =========================================================================
    -- Per-chip pending latch + multi-chip parallel dispatch
    -- =========================================================================
    p_reg_pending : process(i_clk)
        variable v_new_request  : std_logic;
        variable v_rw           : std_logic;  -- '0'=read, '1'=write
    begin
        if rising_edge(i_clk) then
            if s_reset = '1' then
                s_reg_active_r       <= '0';
                s_reg_pending_rw_r   <= '0';
                s_reg_pending_addr_r <= (others => '0');
                s_reg_target_mask_r  <= (others => '0');
                s_reg_done_mask_r    <= (others => '0');
                s_reg_dispatched_r   <= (others => '0');
                s_reg_pending_r      <= (others => '0');
                s_dispatch_pulse_r   <= (others => '0');
                s_done_pulse_r       <= '0';
                s_loop_resume_r      <= '0';
                s_done_chip_r        <= (others => '0');
                s_outstanding_chip_r <= (others => '0');
                s_reg_timeout_cnt_r  <= (others => '0');
                s_reg_timeout_r      <= '0';
                s_reg_timeout_mask_r <= (others => '0');
                s_reg_queue_valid_r  <= '0';
                s_reg_queue_mask_r   <= (others => '0');
                s_reg_queue_rw_r     <= '0';
                s_reg_queue_addr_r   <= (others => '0');
                s_reg_rejected_r     <= '0';
                s_reg_zero_mask_r    <= '0';
                s_dispatch_wait_cnt_r <= (others => '0');
            else
                -- Default: clear single-cycle pulses
                s_dispatch_pulse_r <= (others => '0');
                s_done_pulse_r     <= '0';
                s_loop_resume_r    <= '0';

                -- Detect new request (read or write; write wins if both)
                -- Round 9 #24: write-wins is deliberate but ambiguous — flag
                -- the concurrent R+W case to the TB so a contract violation
                -- is visible instead of silently turning a read into a write.
                -- synthesis translate_off
                assert not (i_cmd_reg_read = '1' and i_cmd_reg_write = '1')
                    report "cmd_arb: simultaneous cmd_reg_read+cmd_reg_write; write wins, read intent dropped"
                    severity warning;
                -- synthesis translate_on
                v_new_request := i_cmd_reg_read or i_cmd_reg_write;
                if i_cmd_reg_write = '1' then
                    v_rw := '1';
                else
                    v_rw := '0';
                end if;

                -- ---- Timeout: force all-done if chip never responds ----
                -- Start counting only after all target chips have been dispatched
                -- (no chips still pending). Previous design counted from request
                -- acceptance, so busy-chip dispatch waits consumed the timeout
                -- budget before any real response could arrive.
                if s_reg_active_r = '1'
                   and s_reg_pending_r = (s_reg_pending_r'range => '0') then
                    if s_reg_timeout_cnt_r = x"FFFF" then
                        -- Timeout: force done for all target chips
                        s_reg_done_mask_r    <= s_reg_target_mask_r;
                        s_reg_timeout_r      <= '1';  -- sticky
                        -- Record which chips didn't respond
                        s_reg_timeout_mask_r <= s_reg_target_mask_r and (not s_reg_done_mask_r);
                    else
                        s_reg_timeout_cnt_r <= s_reg_timeout_cnt_r + 1;
                    end if;
                else
                    s_reg_timeout_cnt_r <= (others => '0');
                end if;

                -- ---- Round 9 #3: dispatch-wait watchdog ----
                -- If one or more target chips stay busy so long that their
                -- dispatch never happens, the response-timeout above never
                -- starts counting. Independent watchdog runs while any chip
                -- is still pending dispatch.
                if s_reg_active_r = '1'
                   and s_reg_pending_r /= (s_reg_pending_r'range => '0') then
                    if s_dispatch_wait_cnt_r = x"FFFF" then
                        -- Undispatched chips never became ready: abort the
                        -- transaction. Mark pending chips as timed out and
                        -- set them to done so s_all_done clears the FSM.
                        s_reg_done_mask_r    <= s_reg_target_mask_r;
                        s_reg_timeout_r      <= '1';
                        s_reg_timeout_mask_r <= s_reg_pending_r;  -- never dispatched
                        s_reg_pending_r      <= (others => '0');  -- stop trying to dispatch
                        s_dispatch_wait_cnt_r <= (others => '0');
                    else
                        s_dispatch_wait_cnt_r <= s_dispatch_wait_cnt_r + 1;
                    end if;
                else
                    s_dispatch_wait_cnt_r <= (others => '0');
                end if;

                -- ---- All-done: clear and fire IRQ ----
                if s_all_done = '1' then
                    s_reg_pending_r     <= (others => '0');
                    s_reg_done_mask_r   <= (others => '0');
                    s_reg_dispatched_r  <= (others => '0');
                    s_reg_timeout_cnt_r <= (others => '0');
                    s_done_pulse_r      <= '1';
                    s_loop_resume_r     <= '1';
                    -- Clear timeout status on normal completion.
                    -- Timeout sticky now means "last completed transaction timed out",
                    -- not "any historical timeout ever occurred".
                    if s_reg_timeout_r = '0' then
                        s_reg_timeout_mask_r <= (others => '0');
                    end if;

                    -- Round 10 #3: capture a same-cycle new request.
                    -- Priority inside the all-done cycle: queue takes over
                    -- first (already-latched prior overlap), then a fresh
                    -- in-flight pulse is absorbed into the queue so the
                    -- next cycle's queue-kickoff path picks it up. Without
                    -- this, a 1-clk v_new_request pulse that coincided with
                    -- s_all_done could be silently lost.
                    if s_reg_queue_valid_r = '1' then
                        -- Queue kickoff (Round 5 #10): prior overlap takes
                        -- priority over any brand-new pulse (both land in
                        -- the same cycle extremely rarely; queue-first is
                        -- the predictable ordering).
                        s_reg_queue_valid_r  <= '0';
                        s_reg_active_r       <= '1';
                        s_reg_target_mask_r  <= s_reg_queue_mask_r;
                        s_reg_pending_rw_r   <= s_reg_queue_rw_r;
                        s_reg_pending_addr_r <= s_reg_queue_addr_r;
                        s_reg_pending_r      <= s_reg_queue_mask_r;
                        s_reg_timeout_r      <= '0';
                        s_reg_timeout_mask_r <= (others => '0');
                        -- If a new request also arrived this cycle, latch
                        -- it into the now-emptied queue so the next cycle's
                        -- queue-kickoff serves it on transaction completion.
                        -- Round 11 C: mirror the zero-mask sticky so the
                        -- same-cycle path matches the normal accept path at
                        -- line 361-365 (observability symmetry).
                        if v_new_request = '1' then
                            if i_cmd_reg_chip_address = C_ZEROS then
                                s_reg_zero_mask_r <= '1';
                            else
                                s_reg_queue_valid_r <= '1';
                                s_reg_queue_mask_r  <= i_cmd_reg_chip_address;
                                s_reg_queue_rw_r    <= v_rw;
                                s_reg_queue_addr_r  <= i_cmd_reg_addr;
                            end if;
                        end if;
                    elsif v_new_request = '1'
                          and i_cmd_reg_chip_address /= C_ZEROS then
                        -- No prior queue — directly start the new request
                        -- so the pulse isn't lost.
                        s_reg_active_r       <= '1';
                        s_reg_target_mask_r  <= i_cmd_reg_chip_address;
                        s_reg_pending_rw_r   <= v_rw;
                        s_reg_pending_addr_r <= i_cmd_reg_addr;
                        s_reg_pending_r      <= i_cmd_reg_chip_address;
                        s_reg_timeout_r      <= '0';
                        s_reg_timeout_mask_r <= (others => '0');
                    elsif v_new_request = '1'
                          and i_cmd_reg_chip_address = C_ZEROS then
                        -- Round 11 C: zero-mask in the same-cycle no-queue
                        -- path also raises the sticky — identical observability
                        -- to the normal accept path.
                        s_reg_zero_mask_r   <= '1';
                        s_reg_active_r      <= '0';
                        s_reg_target_mask_r <= (others => '0');
                    else
                        s_reg_active_r      <= '0';
                        s_reg_target_mask_r <= (others => '0');
                    end if;

                -- ---- Accept new multi-chip request ----
                elsif v_new_request = '1' and s_reg_active_r = '0' then
                  if i_cmd_reg_chip_address = C_ZEROS then
                    -- Zero mask: silently ignore to prevent permanent lockup.
                    -- Raise runtime sticky (Round 5 #17) so SW sees the event.
                    s_reg_zero_mask_r <= '1';
                    -- synthesis translate_off
                    assert false
                        report "cmd_arb: reg request with zero chip mask ignored"
                        severity warning;
                    -- synthesis translate_on
                  else
                    -- Guard: zero mask → ignore (prevents permanent lockup)
                    s_reg_active_r       <= '1';
                    s_reg_target_mask_r  <= i_cmd_reg_chip_address;
                    -- Clear previous timeout status for new transaction
                    s_reg_timeout_r      <= '0';
                    s_reg_timeout_mask_r <= (others => '0');
                    s_reg_pending_rw_r   <= v_rw;
                    s_reg_pending_addr_r <= i_cmd_reg_addr;
                    s_reg_done_mask_r    <= (others => '0');
                    s_reg_dispatched_r   <= (others => '0');

                    -- Immediate dispatch for chips that are ready now
                    for i in 0 to c_N_CHIPS - 1 loop
                        if i_cmd_reg_chip_address(i) = '1' then
                            if i_chip_busy(i) = '0'
                               and i_cmd_start = '0'
                               and i_cmd_start_accepted = '0'
                               and s_cmd_cfg_write_g_i = '0'
                               and i_cmd_cfg_write = '0' then
                                -- Dispatch immediately
                                s_dispatch_pulse_r(i) <= '1';
                                s_reg_dispatched_r(i) <= '1';
                                s_reg_pending_r(i)    <= '0';
                                s_outstanding_chip_r  <= to_unsigned(i, 2);
                            else
                                -- Mark pending for later dispatch
                                s_reg_pending_r(i)    <= '1';
                                s_reg_dispatched_r(i) <= '0';
                            end if;
                        else
                            s_reg_pending_r(i)    <= '0';
                            s_reg_dispatched_r(i) <= '0';
                        end if;
                    end loop;
                  end if;  -- zero mask check

                -- ---- Active: continue dispatching pending + collect dones ----
                elsif s_reg_active_r = '1' then
                    -- Dispatch pending chips that become ready
                    for i in 0 to c_N_CHIPS - 1 loop
                        if s_reg_pending_r(i) = '1'
                           and s_reg_dispatched_r(i) = '0'
                           and s_reg_done_mask_r(i) = '0'
                           and i_chip_busy(i) = '0'
                           and i_cmd_start = '0'
                           and i_cmd_start_accepted = '0'
                           and s_cmd_cfg_write_g_i = '0'
                           and i_cmd_cfg_write = '0' then
                            s_dispatch_pulse_r(i) <= '1';
                            s_reg_dispatched_r(i) <= '1';
                            s_reg_pending_r(i)    <= '0';
                            s_outstanding_chip_r  <= to_unsigned(i, 2);
                        end if;
                    end loop;

                    -- Collect per-chip done signals
                    for i in 0 to c_N_CHIPS - 1 loop
                        if i_cmd_reg_done(i) = '1'
                           and s_reg_target_mask_r(i) = '1' then
                            s_reg_done_mask_r(i) <= '1';
                            s_done_chip_r <= to_unsigned(i, 2);
                        end if;
                    end loop;

                    -- Queue absorb (Round 5 #10 + Round 11 item 17): a new
                    -- reg request arriving while a transaction is in flight
                    -- used to be silently dropped. Latch into a 1-depth
                    -- queue; raise the reject sticky (s_reg_rejected_r) if
                    -- the queue is already occupied (2+ overlaps).
                    -- Ignore zero-mask requests (matches the accept path).
                    --
                    -- Round 11 item 17: queue depth rationale.
                    --   SW's typical reg-access pattern is single-shot read
                    --   or write per command, with the caller waiting for
                    --   o_cmd_reg_done before issuing the next. Burst reg
                    --   access is not a supported workload. The 1-depth
                    --   queue covers the common race where a second pulse
                    --   arrives during the current op's bus round-trip.
                    --   Deeper queuing (FIFO) would add area for a use case
                    --   SW doesn't actually exercise; the reject sticky is
                    --   the agreed signal that SW's pattern has deviated.
                    if v_new_request = '1'
                       and i_cmd_reg_chip_address /= C_ZEROS then
                        if s_reg_queue_valid_r = '0' then
                            s_reg_queue_valid_r <= '1';
                            s_reg_queue_mask_r  <= i_cmd_reg_chip_address;
                            s_reg_queue_rw_r    <= v_rw;
                            s_reg_queue_addr_r  <= i_cmd_reg_addr;
                        else
                            s_reg_rejected_r    <= '1';
                            -- synthesis translate_off
                            assert false
                                report "cmd_arb: overlapping reg request rejected (queue already full)"
                                severity warning;
                            -- synthesis translate_on
                        end if;
                    end if;
                end if;

                -- Also collect dones during accept cycle (chip may complete
                -- same cycle it was dispatched in a zero-wait-state path)
                if v_new_request = '1' and s_reg_active_r = '0' then
                    -- dones arriving this cycle apply to previous operation,
                    -- not the new one; done_mask was cleared above, so this
                    -- is a no-op by design.
                    null;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Output: gated read/write pulses (1-clk per chip)
    -- =========================================================================
    gen_reg_out : for i in 0 to c_N_CHIPS - 1 generate
        o_cmd_reg_read_g(i)  <= s_dispatch_pulse_r(i) when s_reg_pending_rw_r = '0' else '0';
        o_cmd_reg_write_g(i) <= s_dispatch_pulse_r(i) when s_reg_pending_rw_r = '1' else '0';
    end generate;

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    o_cmd_cfg_write_g    <= s_cmd_cfg_write_g_i;
    o_reg_outstanding    <= s_reg_active_r;
    o_outstanding_chip   <= s_outstanding_chip_r;
    o_cmd_reg_done_pulse <= s_done_pulse_r;
    o_cmd_reg_done_chip  <= s_done_chip_r;
    o_reg_loop_resume    <= s_loop_resume_r;
    o_cmd_reg_addr_out   <= s_reg_pending_addr_r;
    o_reg_timeout        <= s_reg_timeout_r;
    o_reg_timeout_mask   <= s_reg_timeout_mask_r;
    o_reg_rejected       <= s_reg_rejected_r;
    o_reg_zero_mask      <= s_reg_zero_mask_r;

end architecture rtl;
