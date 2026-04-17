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
--   ST_IDLE -> ST_READ_REG11 -> ST_WAIT_READ -> ST_RECOVERY ->
--   ST_WAIT_RECOVERY -> ST_WAIT_FRAME_DONE -> ST_IDLE
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
        -- Reg11 read results (from csr_chip STAT, per-chip)
        i_reg11_data_0      : in  std_logic_vector(31 downto 0);
        i_reg11_data_1      : in  std_logic_vector(31 downto 0);
        i_reg11_data_2      : in  std_logic_vector(31 downto 0);
        i_reg11_data_3      : in  std_logic_vector(31 downto 0);
        i_cmd_reg_done_pulse: in  std_logic;
        -- Frame done (from output_stage, for VDMA frame boundary)
        i_frame_done        : in  std_logic;
        -- Shot start (for sync recovery to shot boundary)
        i_shot_start        : in  std_logic;
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
        o_err_fatal         : out std_logic
    );
end entity tdc_gpx_err_handler;

architecture rtl of tdc_gpx_err_handler is

    -- =========================================================================
    -- FSM
    -- =========================================================================
    type t_state is (
        ST_IDLE,
        ST_READ_REG11,
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
            if i_rst_n = '0' then
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
                                s_state_r <= ST_READ_REG11;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- ST_READ_REG11: issue Reg12 read for error cause flags
                    -- NOTE: Reg11 contains unmask configuration bits, NOT status.
                    -- Reg12 contains actual error flags: HFifoFull, IFifoFull,
                    -- NotLocked. Reading Reg12 may clear some flags (datasheet),
                    -- so we latch cause internally on read completion.
                    -- ---------------------------------------------------------
                    when ST_READ_REG11 =>
                        s_err_cause_r         <= (others => '0');
                        s_cmd_reg_read_r      <= '1';
                        s_cmd_reg_addr_r      <= c_TDC_REG12;  -- Reg12: actual status flags
                        s_cmd_reg_chip_addr_r <= s_err_chip_mask_r;
                        s_state_r             <= ST_WAIT_READ;

                    -- ---------------------------------------------------------
                    -- ST_WAIT_READ: wait for read completion, classify error
                    -- ---------------------------------------------------------
                    when ST_WAIT_READ =>
                        if i_cmd_reg_done_pulse = '1' then
                            for i in 0 to c_N_CHIPS - 1 loop
                                if s_err_chip_mask_r(i) = '1' then
                                    v_reg12 := s_reg11_data(i);  -- data comes from reg read (now Reg12)
                                    -- Reg12 bits 23:16 = HFifoFull per stop (actual status)
                                    if v_reg12(23 downto 16) /= x"00" then
                                        s_err_cause_r(0) <= '1';
                                    end if;
                                    -- Reg12 bits 25:24 = IFifoFull (actual status)
                                    if v_reg12(25 downto 24) /= "00" then
                                        s_err_cause_r(1) <= '1';
                                    end if;
                                    -- Reg12 bit 26 = NotLocked (actual status)
                                    if v_reg12(26) = '1' then
                                        s_err_cause_r(2) <= '1';
                                    end if;
                                end if;
                            end loop;
                            s_state_r <= ST_RECOVERY;
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
                        if s_frame_done_seen_r = '0' then
                            if i_frame_done = '1' then
                                s_frame_done_seen_r <= '1';
                            end if;
                        else
                            if i_shot_start = '1' then
                                -- Clear error-fill and all internal state
                                s_err_fill_r        <= (others => '0');
                                s_err_chip_mask_r   <= (others => '0');
                                s_err_cause_r       <= (others => '0');
                                s_debounce_cnt_r    <= (others => (others => '0'));
                                s_retry_cnt_r       <= (others => '0');
                                s_frame_done_seen_r <= '0';
                                s_state_r           <= ST_IDLE;
                            end if;
                        end if;

                end case;
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

end architecture rtl;
