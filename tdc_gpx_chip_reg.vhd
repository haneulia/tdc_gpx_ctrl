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
        o_busy          : out std_logic;

        -- Bus request (to coordinator bus mux)
        o_bus_req_valid : out std_logic;
        o_bus_req_rw    : out std_logic;
        o_bus_req_addr  : out std_logic_vector(3 downto 0);
        o_bus_req_wdata : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

        -- Bus response (from coordinator, gated)
        i_bus_rsp_valid : in  std_logic;
        i_bus_rsp_rdata : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0)
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
    signal s_done_r      : std_logic := '0';
    signal s_busy_r      : std_logic := '0';
    signal s_timeout_r   : unsigned(15 downto 0) := (others => '0');

begin

    p_fsm : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r     <= ST_OFF;
                s_req_valid_r <= '0';
                s_is_read_r   <= '0';
                s_rdata_r     <= (others => '0');
                s_rvalid_r    <= '0';
                s_done_r      <= '0';
                s_busy_r      <= '0';
            else
                s_rvalid_r <= '0';
                s_done_r   <= '0';

                case s_state_r is

                    when ST_OFF =>
                        s_busy_r <= '0';
                        if i_start_read = '1' then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= i_addr;
                            s_is_read_r   <= '1';
                            s_busy_r      <= '1';
                            s_state_r     <= ST_ACTIVE;
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
                            s_busy_r      <= '1';
                            s_state_r     <= ST_ACTIVE;
                        end if;

                    when ST_ACTIVE =>
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
                            -- Timeout: bus hung, force done
                            s_req_valid_r <= '0';
                            s_busy_r      <= '0';
                            s_done_r      <= '1';
                            s_state_r     <= ST_OFF;
                        else
                            s_timeout_r <= s_timeout_r + 1;
                        end if;

                end case;
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
    o_busy          <= s_busy_r;

end architecture rtl;
