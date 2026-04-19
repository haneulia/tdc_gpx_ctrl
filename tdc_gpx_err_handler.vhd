-- =============================================================================
-- tdc_gpx_err_handler.vhd
-- TDC-GPX Controller - Automatic ErrFlag Detection and Recovery
-- =============================================================================
--
-- Purpose:
--   Monitors per-chip ErrFlag lines (synchronized by bus_phy), debounces them,
--   reads Reg11 to classify the error cause, issues per-chip soft resets, and
--   gates hit data with error-fill until recovery completes at a clean frame/
--   shot boundary.
--
-- FSM States (6):
--   ST_IDLE -> ST_READ_REG12 -> ST_WAIT_READ -> ST_RECOVERY ->
--   ST_WAIT_RECOVERY -> ST_WAIT_FRAME_DONE -> ST_IDLE
--
-- ST_WAIT_READ watchdog (Round 2 #2):
--   A 16-bit counter bounds the wait for i_cmd_reg_done_pulse; on
--   expiration s_err_read_timeout_r sticky fires and the FSM proceeds
--   to ST_RECOVERY anyway (rvalid stayed '0' so no stale data is latched
--   as a classification cause). Prevents permanent stall if the reg
--   access subsystem never acknowledges.
--
-- i_soft_clear (Round 3 #10 / Round 4 #40):
--   SW-initiated clear that resets the FSM to ST_IDLE and drops fatal/
--   retry state without a hard reset. Shared at top with
--   status_agg.i_soft_clear so both observers see consistent history
--   clearing. Default '0' (backward compatible).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;  -- for ceil/log2 in generic width calculation

use work.tdc_gpx_pkg.all;

entity tdc_gpx_err_handler is
    generic (
        g_DEBOUNCE_CLKS : positive := 4;
        g_MAX_RETRIES   : positive := 3
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        -- ErrFlag (bus_phy synchronized, per-chip)
        i_errflag_sync      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        -- Chip status
        i_chip_busy         : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        -- Reg read results (from csr_chip STAT, per-chip).
        --
        -- NAMING DEBT (Round 5 #23): the port prefix "reg11" is historical;
        -- this path actually carries the TDC-GPX Reg12 (chip status) read
        -- result. Renaming ripples into config_ctrl and csr_chip wiring, so
        -- the port name is intentionally retained until a broader interface
        -- pass sweeps the chain. Treat the *_reg11_* inputs as "status read
        -- data" — read /Reg12/, not /Reg11/.
        i_reg11_data_0      : in  std_logic_vector(31 downto 0);  -- chip status (Reg12)
        i_reg11_data_1      : in  std_logic_vector(31 downto 0);  -- chip status (Reg12)
        i_reg11_data_2      : in  std_logic_vector(31 downto 0);  -- chip status (Reg12)
        i_reg11_data_3      : in  std_logic_vector(31 downto 0);  -- chip status (Reg12)
        i_cmd_reg_done_pulse: in  std_logic;
        i_cmd_reg_rvalid    : in  std_logic_vector(c_N_CHIPS - 1 downto 0);  -- per-chip read valid
        i_reg_outstanding   : in  std_logic;   -- cmd_arb has active reg access
        -- Frame done (from output_stage, for VDMA frame boundary)
        i_frame_done        : in  std_logic;
        -- Shot start (for sync recovery to shot boundary)
        i_shot_start        : in  std_logic;
        -- SW-initiated soft clear for fatal state (1-clk pulse)
        -- Allows SW to reset err_fatal + retry count without needing a hard reset.
        -- Tie to '0' if no SW clear path is needed.
        i_soft_clear        : in  std_logic := '0';
        -- Command outputs (OR'd in config_ctrl)
        o_cmd_soft_reset    : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_cmd_reg_read      : out std_logic;
        o_cmd_reg_addr      : out std_logic_vector(3 downto 0);
        o_cmd_reg_chip_addr : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        -- Error-fill (per-chip, '1' = hit data replaced with 0x1FFFF)
        o_err_fill          : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        -- Status
        o_err_active        : out std_logic;
        o_err_chip_mask     : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_cause         : out std_logic_vector(2 downto 0);
        o_err_fatal         : out std_logic;
        -- Sticky: ST_WAIT_READ watchdog expired (Round 5 #13 — was internal)
        --   Set whenever the watchdog on reg-read completion pulse fires,
        --   forcing the recovery FSM to classify the error without the read
        --   result. Indicates the reg-access subsystem failed to respond;
        --   cleared only by reset.
        o_err_read_timeout  : out std_logic
    );
end entity tdc_gpx_err_handler;

architecture rtl of tdc_gpx_err_handler is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (
        ST_IDLE,
        ST_READ_REG12,
        ST_WAIT_READ,
        ST_RECOVERY,
        ST_WAIT_RECOVERY,
        ST_WAIT_FRAME_DONE
    );

    signal s_state_r : t_state := ST_IDLE;

    -- =========================================================================
    -- Per-chip debounce counters (width derived from generic)
    -- =========================================================================
    constant C_DEB_WIDTH : natural := maximum(1, integer(ceil(log2(real(g_DEBOUNCE_CLKS)))));
    type t_debounce_arr is array (0 to c_N_CHIPS - 1) of unsigned(C_DEB_WIDTH - 1 downto 0);
    signal s_debounce_cnt_r : t_debounce_arr := (others => (others => '0'));

    -- =========================================================================
    -- Latched error state
    -- =========================================================================
    signal s_err_chip_mask_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_fill_r      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_cause_r     : std_logic_vector(2 downto 0)             := (others => '0');
    signal s_err_fatal_r     : std_logic                                := '0';

    -- =========================================================================
    -- Command registers (1-clk pulses)
    -- =========================================================================
    signal s_cmd_soft_reset_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_cmd_reg_read_r    : std_logic                                := '0';
    signal s_cmd_reg_addr_r    : std_logic_vector(3 downto 0)             := (others => '0');
    signal s_cmd_reg_chip_addr_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Recovery timeout & retry
    -- =========================================================================
    signal s_recov_timeout_r  : unsigned(13 downto 0) := (others => '0');
    signal s_recov_stable_r   : unsigned(2 downto 0)  := (others => '0');  -- stable-low counter
    constant C_RETRY_WIDTH : natural := maximum(1, integer(ceil(log2(real(g_MAX_RETRIES + 1)))));
    signal s_retry_cnt_r     : unsigned(C_RETRY_WIDTH - 1 downto 0)  := (others => '0');

    -- =========================================================================
    -- Frame-done seen flag (sub-state in ST_WAIT_FRAME_DONE)
    -- =========================================================================
    signal s_frame_done_seen_r : std_logic := '0';

    -- =========================================================================
    -- ST_WAIT_READ watchdog: guard against lost/never-arriving
    -- i_cmd_reg_done_pulse. Without this, the recovery FSM would park
    -- indefinitely if the reg access subsystem never responds.
    -- =========================================================================
    constant C_WAIT_READ_TIMEOUT : natural := 16#FFFF#;
    signal s_wait_read_cnt_r  : unsigned(15 downto 0) := (others => '0');
    signal s_err_read_timeout_r : std_logic := '0';  -- sticky: reg read timed out

    -- =========================================================================
    -- Reg11 data selection helper
    -- =========================================================================
    type t_reg11_arr is array (0 to c_N_CHIPS - 1) of std_logic_vector(31 downto 0);
    signal s_reg11_data : t_reg11_arr;

begin

    -- Map individual Reg11 inputs into an array for indexed access
    s_reg11_data(0) <= i_reg11_data_0;
    s_reg11_data(1) <= i_reg11_data_1;
    s_reg11_data(2) <= i_reg11_data_2;
    s_reg11_data(3) <= i_reg11_data_3;

    -- =========================================================================
    -- Main FSM process
    -- =========================================================================
    p_fsm : process (i_clk)
        variable v_any_debounced : boolean;
        variable v_reg12         : std_logic_vector(31 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_soft_clear = '1' then
                s_state_r            <= ST_IDLE;
                s_debounce_cnt_r     <= (others => (others => '0'));
                s_err_chip_mask_r    <= (others => '0');
                s_err_fill_r         <= (others => '0');
                s_err_cause_r        <= (others => '0');
                s_err_fatal_r        <= '0';
                s_cmd_soft_reset_r   <= (others => '0');
                s_cmd_reg_read_r     <= '0';
                s_cmd_reg_addr_r     <= (others => '0');
                s_cmd_reg_chip_addr_r <= (others => '0');
                s_recov_timeout_r    <= (others => '0');
                s_recov_stable_r     <= (others => '0');
                s_retry_cnt_r        <= (others => '0');
                s_frame_done_seen_r  <= '0';
                s_wait_read_cnt_r    <= (others => '0');
                s_err_read_timeout_r <= '0';
            else
                -- Default: clear command pulses every clock
                s_cmd_soft_reset_r   <= (others => '0');
                s_cmd_reg_read_r     <= '0';
                s_cmd_reg_addr_r     <= (others => '0');
                s_cmd_reg_chip_addr_r <= (others => '0');

                case s_state_r is

                    -- ---------------------------------------------------------
                    -- ST_IDLE: debounce per-chip ErrFlag
                    -- ---------------------------------------------------------
                    when ST_IDLE =>
                        v_any_debounced := false;

                        -- After fatal, block auto-recovery. SW must system reset.
                        if s_err_fatal_r = '0' then
                            for i in 0 to c_N_CHIPS - 1 loop
                                if i_errflag_sync(i) = '1' then
                                    if s_debounce_cnt_r(i) = to_unsigned(g_DEBOUNCE_CLKS - 1, C_DEB_WIDTH) then
                                        s_err_chip_mask_r(i) <= '1';
                                        s_err_fill_r(i)      <= '1';
                                        v_any_debounced      := true;
                                    else
                                        s_debounce_cnt_r(i) <= s_debounce_cnt_r(i) + 1;
                                    end if;
                                else
                                    s_debounce_cnt_r(i) <= (others => '0');
                                end if;
                            end loop;

                            if v_any_debounced then
                                s_state_r <= ST_READ_REG12;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- ST_READ_REG12: issue Reg12 read for error cause flags
                    -- NOTE: Reg11 contains unmask configuration bits, NOT status.
                    -- Reg12 contains actual error flags: HFifoFull, IFifoFull,
                    -- NotLocked. Reading Reg12 may clear some flags (datasheet),
                    -- so we latch cause internally on read completion.
                    -- ---------------------------------------------------------
                    when ST_READ_REG12 =>
                        -- Wait for any prior reg access to complete before issuing
                        -- our read. This prevents consuming a stale done_pulse from
                        -- a different reg transaction.
                        if i_reg_outstanding = '0' then
                            s_err_cause_r         <= (others => '0');
                            s_cmd_reg_read_r      <= '1';
                            s_cmd_reg_addr_r      <= c_TDC_REG12;
                            s_cmd_reg_chip_addr_r <= s_err_chip_mask_r;
                            s_wait_read_cnt_r     <= (others => '0');
                            s_state_r             <= ST_WAIT_READ;
                        end if;

                    -- ---------------------------------------------------------
                    -- ST_WAIT_READ: wait for read completion, classify error
                    -- ---------------------------------------------------------
                    when ST_WAIT_READ =>
                        if i_cmd_reg_done_pulse = '1' then
                            for i in 0 to c_N_CHIPS - 1 loop
                                if s_err_chip_mask_r(i) = '1' and i_cmd_reg_rvalid(i) = '1' then
                                    -- Only classify if read data is actually valid.
                                    -- If reg timeout occurred, rvalid stays '0' and
                                    -- stale data is not used for cause classification.
                                    v_reg12 := s_reg11_data(i);  -- data from Reg12 read
                                    -- Reg12 datasheet bit map:
                                    --   [7:0]  = HFifoFull per stop (8 flags)
                                    --   [9:8]  = IFifoFull (2 flags: IFIFO1, IFIFO2)
                                    --   [10]   = NotLocked (PLL lock lost)
                                    --   [11+]  = reserved / interrupt control
                                    -- NOTE: reading Reg12 clears HFifoFull/IFifoFull,
                                    -- so we latch cause immediately.
                                    if v_reg12(7 downto 0) /= x"00" then
                                        s_err_cause_r(0) <= '1';  -- HitFIFO overflow
                                    end if;
                                    if v_reg12(9 downto 8) /= "00" then
                                        s_err_cause_r(1) <= '1';  -- IFIFO overflow
                                    end if;
                                    if v_reg12(10) = '1' then
                                        s_err_cause_r(2) <= '1';  -- PLL not locked
                                    end if;
                                end if;
                            end loop;
                            s_wait_read_cnt_r <= (others => '0');
                            s_state_r         <= ST_RECOVERY;
                        elsif s_wait_read_cnt_r = to_unsigned(C_WAIT_READ_TIMEOUT, 16) then
                            -- Reg read never completed. Sticky-flag the loss and
                            -- proceed to recovery anyway (rvalid was '0' for all
                            -- chips so no stale cause was latched). This prevents
                            -- the recovery FSM from parking forever.
                            s_err_read_timeout_r <= '1';
                            s_wait_read_cnt_r    <= (others => '0');
                            s_state_r            <= ST_RECOVERY;
                        else
                            s_wait_read_cnt_r <= s_wait_read_cnt_r + 1;
                        end if;

                    -- ---------------------------------------------------------
                    -- ST_RECOVERY: issue per-chip soft reset
                    -- ---------------------------------------------------------
                    when ST_RECOVERY =>
                        s_cmd_soft_reset_r <= s_err_chip_mask_r;
                        s_recov_timeout_r  <= (others => '0');
                        s_recov_stable_r   <= (others => '0');
                        s_state_r          <= ST_WAIT_RECOVERY;

                    -- ---------------------------------------------------------
                    -- ST_WAIT_RECOVERY: wait for error chips to finish init
                    -- ---------------------------------------------------------
                    when ST_WAIT_RECOVERY =>
                        if ((i_chip_busy and s_err_chip_mask_r) = (s_err_chip_mask_r'range => '0'))
                           and ((i_errflag_sync and s_err_chip_mask_r) = (s_err_chip_mask_r'range => '0'))
                        then
                            -- Require 8-cycle stable-low before declaring success
                            if s_recov_stable_r = "111" then
                                s_frame_done_seen_r <= '0';
                                s_recov_stable_r    <= (others => '0');
                                s_state_r           <= ST_WAIT_FRAME_DONE;
                            else
                                s_recov_stable_r <= s_recov_stable_r + 1;
                            end if;
                        elsif s_recov_timeout_r = to_unsigned(9999, 14) then
                            -- Timeout
                            if s_retry_cnt_r < to_unsigned(g_MAX_RETRIES, C_RETRY_WIDTH) then
                                s_retry_cnt_r <= s_retry_cnt_r + 1;
                                s_state_r     <= ST_RECOVERY;
                            else
                                s_err_fatal_r <= '1';
                                -- Fatal: release reg-access path so SW can diagnose.
                                -- s_err_fatal_r stays sticky until system reset.
                                -- Clear err_fill to stop raw-data substitution.
                                -- Preserve chip_mask and cause for SW diagnostics.
                                s_err_fill_r  <= (others => '0');
                                -- s_err_chip_mask_r: KEPT for diagnosis
                                -- s_err_cause_r:     KEPT for diagnosis
                                s_state_r     <= ST_IDLE;
                            end if;
                        else
                            s_recov_stable_r  <= (others => '0');
                            s_recov_timeout_r <= s_recov_timeout_r + 1;
                        end if;

                    -- ---------------------------------------------------------
                    -- ST_WAIT_FRAME_DONE: re-sync at frame/shot boundary
                    -- ---------------------------------------------------------
                    when ST_WAIT_FRAME_DONE =>
                        -- Round 9 #19: s_frame_done_seen_r is now latched
                        -- unconditionally outside the case (below) so a
                        -- frame_done arriving just before this state entry
                        -- is still honored. Here we only wait for shot_start
                        -- once the latch has caught a frame_done.
                        if s_frame_done_seen_r = '1' and i_shot_start = '1' then
                            -- Clear error-fill and all internal state
                            s_err_fill_r        <= (others => '0');
                            s_err_chip_mask_r   <= (others => '0');
                            s_err_cause_r       <= (others => '0');
                            s_debounce_cnt_r    <= (others => (others => '0'));
                            s_retry_cnt_r       <= (others => '0');
                            s_frame_done_seen_r <= '0';
                            s_state_r           <= ST_IDLE;
                        end if;

                end case;

                -- Round 9 #19: edge-capture frame_done latch runs in parallel
                -- with the FSM. Once the recovery path starts (any state
                -- other than ST_IDLE), any frame_done pulse is remembered so
                -- ST_WAIT_FRAME_DONE sees a pulse that happened the cycle
                -- before it was entered. Cleared explicitly by the state's
                -- shot_start handshake above and by reset.
                if s_state_r /= ST_IDLE and i_frame_done = '1' then
                    s_frame_done_seen_r <= '1';
                end if;
            end if;
        end if;
    end process p_fsm;

    -- =========================================================================
    -- Output assignments (concurrent)
    -- =========================================================================
    o_cmd_soft_reset    <= s_cmd_soft_reset_r;
    o_cmd_reg_read      <= s_cmd_reg_read_r;
    o_cmd_reg_addr      <= s_cmd_reg_addr_r;
    o_cmd_reg_chip_addr <= s_cmd_reg_chip_addr_r;
    o_err_fill          <= s_err_fill_r;
    -- err_active reflects FSM busy (recovery in progress), NOT fatal status.
    -- Fatal state is reported separately via o_err_fatal.
    -- Keeping err_active=0 when fatal+idle allows SW manual reg access for diagnosis.
    o_err_active        <= '0' when s_state_r = ST_IDLE else '1';
    o_err_chip_mask     <= s_err_chip_mask_r;
    o_err_cause         <= s_err_cause_r;
    o_err_fatal         <= s_err_fatal_r;
    o_err_read_timeout  <= s_err_read_timeout_r;

end architecture rtl;
