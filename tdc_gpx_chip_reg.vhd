-- =============================================================================
-- tdc_gpx_chip_reg.vhd
-- TDC-GPX Controller - Individual Register Access Sub-FSM
-- =============================================================================
--
-- Purpose:
--   Handles individual TDC-GPX register read/write operations.
--   Extracted from tdc_gpx_chip_ctrl to reduce FSM complexity.
--
-- Interface:
--   Coordinator dispatches read/write request via i_start_read / i_start_write.
--   Module performs bus transaction and asserts o_done when complete.
--   For reads: o_rdata + o_rvalid provide read result (rvalid = read only).
--   For writes: o_done pulses but rvalid stays low (STAT11 protection).
--
-- Pending-request queue (Round 3 #20/#37):
--   A 1-depth pending queue (s_pend_valid_r + addr/rw/wdata) absorbs a
--   second request pulse while the current transaction is in ST_ACTIVE.
--   It also handles the same-cycle read+write case (read wins, write
--   queued). A third pulse while the queue is already occupied raises
--   s_err_req_overflow_r sticky so SW can observe command loss.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_chip_reg is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;

        -- Control from coordinator
        i_start_read    : in  std_logic;
        i_start_write   : in  std_logic;
        i_addr          : in  std_logic_vector(3 downto 0);
        i_wdata         : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

        -- Result
        o_rdata         : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_rvalid        : out std_logic;         -- 1-clk pulse: read data valid
        o_done          : out std_logic;          -- 1-clk pulse: transaction complete
        o_timeout       : out std_logic;         -- 1-clk pulse: bus response timeout
        o_busy          : out std_logic;

        -- Bus request (to coordinator bus mux)
        o_bus_req_valid : out std_logic;
        o_bus_req_rw    : out std_logic;
        o_bus_req_addr  : out std_logic_vector(3 downto 0);
        o_bus_req_wdata : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

        -- Bus response (from coordinator, gated)
        i_bus_rsp_valid : in  std_logic;
        i_bus_rsp_rdata : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

        -- Sticky: 3rd-pulse queue overflow (Round 5 #12 — was internal only)
        --   Asserts '1' whenever a new i_start_read / i_start_write arrives
        --   while ST_ACTIVE AND the 1-depth pending queue is already full
        --   (i.e. a request was actually dropped). Cleared by reset or by
        --   i_soft_clear (Round 7 B-5, shared with status_agg / err_handler).
        o_err_req_overflow : out std_logic;

        -- SW-initiated sticky clear (Round 7 B-5). Default '0' keeps legacy.
        i_soft_clear       : in  std_logic := '0'
    );
end entity tdc_gpx_chip_reg;

architecture rtl of tdc_gpx_chip_reg is

    type t_reg_state is (ST_OFF, ST_ACTIVE);
    signal s_state_r     : t_reg_state := ST_OFF;

    signal s_req_valid_r : std_logic := '0';
    signal s_req_rw_r    : std_logic := '0';
    signal s_req_addr_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_is_read_r   : std_logic := '0';
    signal s_rdata_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_rvalid_r    : std_logic := '0';
    signal s_done_r         : std_logic := '0';
    signal s_timeout_out_r  : std_logic := '0';
    signal s_busy_r         : std_logic := '0';
    signal s_timeout_r      : unsigned(15 downto 0) := (others => '0');

    -- 1-depth pending queue: if a request pulse arrives while s_state_r =
    -- ST_ACTIVE, latch it (plus addr/wdata/rw) so the request is not silently
    -- dropped. Also protects against concurrent read+write in the same cycle
    -- (read wins, write queued).
    signal s_pend_valid_r : std_logic := '0';
    signal s_pend_rw_r    : std_logic := '0';
    signal s_pend_addr_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_pend_wdata_r : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    -- Sticky: 2nd request arrived while pending queue was already occupied.
    signal s_err_req_overflow_r : std_logic := '0';

begin

    p_fsm : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r     <= ST_OFF;
                s_req_valid_r <= '0';
                s_req_rw_r    <= '0';
                s_req_addr_r  <= (others => '0');
                s_req_wdata_r <= (others => '0');
                s_is_read_r   <= '0';
                s_rdata_r     <= (others => '0');
                s_rvalid_r    <= '0';
                s_done_r         <= '0';
                s_timeout_out_r  <= '0';
                s_busy_r         <= '0';
                s_timeout_r      <= (others => '0');
                s_pend_valid_r   <= '0';
                s_pend_rw_r      <= '0';
                s_pend_addr_r    <= (others => '0');
                s_pend_wdata_r   <= (others => '0');
                s_err_req_overflow_r <= '0';
            else
                s_rvalid_r      <= '0';
                s_done_r        <= '0';
                s_timeout_out_r <= '0';

                case s_state_r is

                    when ST_OFF =>
                        s_busy_r <= '0';
                        -- Prefer pending over live input so queued SW requests
                        -- are honored in arrival order.
                        if s_pend_valid_r = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= s_pend_rw_r;
                            s_req_addr_r  <= s_pend_addr_r;
                            if s_pend_rw_r = '1' and s_pend_addr_r = x"E" then
                                s_req_wdata_r    <= s_pend_wdata_r;
                                s_req_wdata_r(4) <= '0';
                            else
                                s_req_wdata_r <= s_pend_wdata_r;
                            end if;
                            s_is_read_r   <= not s_pend_rw_r;
                            s_timeout_r   <= (others => '0');
                            s_busy_r      <= '1';
                            s_pend_valid_r <= '0';
                            s_state_r     <= ST_ACTIVE;
                        elsif i_start_read = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= i_addr;
                            s_is_read_r   <= '1';
                            s_timeout_r   <= (others => '0');  -- rearm watchdog
                            s_busy_r      <= '1';
                            s_state_r     <= ST_ACTIVE;
                            -- Same-cycle write is queued so neither pulse is
                            -- lost (previous behavior silently dropped write).
                            if i_start_write = '1' then
                                s_pend_valid_r <= '1';
                                s_pend_rw_r    <= '1';
                                s_pend_addr_r  <= i_addr;
                                s_pend_wdata_r <= i_wdata;
                            end if;
                        elsif i_start_write = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '1';
                            s_req_addr_r  <= i_addr;
                            -- Safety: block dangerous register bits via direct write.
                            -- Reg14 bit4: 16-bit mode (CSN malfunction bug, no workaround).
                            -- Future: add service bit filters for other registers as needed.
                            if i_addr = x"E" then  -- Reg14
                                s_req_wdata_r    <= i_wdata;
                                s_req_wdata_r(4) <= '0';  -- force bit4=0
                            else
                                s_req_wdata_r <= i_wdata;
                            end if;
                            s_is_read_r   <= '0';
                            s_timeout_r   <= (others => '0');  -- rearm watchdog
                            s_busy_r      <= '1';
                            s_state_r     <= ST_ACTIVE;
                        end if;

                    when ST_ACTIVE =>
                        -- Queue incoming pulse while busy instead of dropping it.
                        -- If the queue is already occupied, raise sticky error
                        -- so SW can see that a request was lost.
                        --
                        -- Round 9 #16 + Round 11 item 16: simultaneous
                        -- read+write policy.
                        --   Synthesized behavior: WRITE wins.
                        --     `s_pend_rw_r <= i_start_write` assigns 1 when
                        --     write is asserted; if both are high, write wins
                        --     and read intent is dropped.
                        --   Sim assertion: WARNING fires below.
                        --   Overflow sticky (s_err_req_overflow_r): does NOT
                        --     fire for this case (the pending slot IS being
                        --     filled, just with the wrong request). A
                        --     dedicated "ambiguous overlap" sticky would be
                        --     needed to catch this in silicon.
                        --   Caller contract: the cmd_arb layer upstream is
                        --     expected to serialize read/write into non-
                        --     overlapping pulses. This assertion catches
                        --     contract violations during regression.
                        -- synthesis translate_off
                        assert not (i_start_read = '1' and i_start_write = '1')
                            report "chip_reg: simultaneous start_read+start_write in ST_ACTIVE; write wins, read intent dropped"
                            severity warning;
                        -- synthesis translate_on
                        if i_start_read = '1' or i_start_write = '1' then
                            if s_pend_valid_r = '0' then
                                s_pend_valid_r <= '1';
                                s_pend_rw_r    <= i_start_write;  -- read=0, write=1
                                s_pend_addr_r  <= i_addr;
                                s_pend_wdata_r <= i_wdata;
                            else
                                s_err_req_overflow_r <= '1';
                            end if;
                        end if;
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r <= '0';
                            s_busy_r      <= '0';
                            s_done_r      <= '1';
                            s_timeout_r   <= (others => '0');
                            s_state_r     <= ST_OFF;
                            if s_is_read_r = '1' then
                                s_rdata_r  <= i_bus_rsp_rdata;
                                s_rvalid_r <= '1';
                            end if;
                        elsif s_timeout_r = x"FFFF" then
                            -- Timeout: bus hung, force done + timeout flag
                            s_req_valid_r   <= '0';
                            s_busy_r        <= '0';
                            s_done_r        <= '1';
                            s_timeout_out_r <= '1';
                            s_timeout_r     <= (others => '0');  -- clear for next use
                            s_state_r       <= ST_OFF;
                        else
                            s_timeout_r <= s_timeout_r + 1;
                        end if;

                end case;

                -- Round 8 C-1: SW-initiated sticky clear placed AFTER the
                -- case so it wins over a concurrent set inside ST_ACTIVE
                -- (VHDL sequential semantics: last assignment wins).
                -- Matches the priority pattern used by status_agg and
                -- err_handler for their shared i_soft_clear paths.
                if i_soft_clear = '1' then
                    s_err_req_overflow_r <= '0';
                end if;
            end if;
        end if;
    end process p_fsm;

    o_bus_req_valid <= s_req_valid_r;
    o_bus_req_rw    <= s_req_rw_r;
    o_bus_req_addr  <= s_req_addr_r;
    o_bus_req_wdata <= s_req_wdata_r;
    o_rdata         <= s_rdata_r;
    o_rvalid        <= s_rvalid_r;
    o_done          <= s_done_r;
    o_timeout       <= s_timeout_out_r;
    o_busy          <= s_busy_r;
    o_err_req_overflow <= s_err_req_overflow_r;

end architecture rtl;
