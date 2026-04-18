-- =============================================================================
-- tdc_gpx_chip_init.vhd
-- TDC-GPX Controller - Chip Init Sub-FSM
-- =============================================================================
--
-- Purpose:
--   Handles powerup sequence, cfg_write, and master reset for one TDC-GPX chip.
--   Extracted from tdc_gpx_chip_ctrl to reduce FSM complexity.
--
-- FSM States (9):
--   ST_POWERUP → ST_PU_RELEASE → ST_STOPDIS_HIGH → ST_CFG_WRITE →
--   ST_CFG_WR_WAIT → ST_MASTER_RESET → ST_MR_WAIT → ST_RECOVERY →
--   ST_STOPDIS_LOW → done
--
-- Interface:
--   Coordinator asserts i_start to begin init sequence.
--   Module asserts o_done for 1 cycle when complete.
--   cfg_write (i_cfg_write_req) triggers runtime cfg_write (no master reset).
--
-- Concurrent request handling (Round 3 #21):
--   If i_start and i_cfg_write_req are asserted the same cycle in ST_OFF,
--   i_start wins and s_cfg_write_pending_r latches. After the init path
--   returns to ST_OFF, the pending cfg_write is auto-consumed so the SW
--   pulse is not silently lost.
--
-- Timeout-exit safe state (Round 2 #8):
--   ST_CFG_WR_WAIT and ST_MR_WAIT timeout exits drop s_stopdis_r <= '0'
--   so the chip ends in the same post-init pin state as a normal
--   ST_STOPDIS_LOW completion. Prevents "stops disabled" hangovers after
--   bus-hang timeouts.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_chip_init is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH;
        g_POWERUP_CLKS   : positive := 48;
        g_RECOVERY_CLKS  : positive := 8
    );
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;

        -- Control from coordinator
        i_start          : in  std_logic;        -- begin powerup init sequence
        i_cfg_write_req  : in  std_logic;        -- runtime cfg_write (no master reset)
        i_cfg_image      : in  t_cfg_image;      -- latched by coordinator at request time
        o_done           : out std_logic;         -- 1-clk: init/cfg_write complete
        o_timeout        : out std_logic;         -- 1-clk: bus response timeout (done also fires)

        -- Bus request (to coordinator bus mux)
        o_bus_req_valid  : out std_logic;
        o_bus_req_rw     : out std_logic;
        o_bus_req_addr   : out std_logic_vector(3 downto 0);
        o_bus_req_wdata  : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

        -- Bus response (from coordinator, gated by active phase)
        i_bus_rsp_valid  : in  std_logic;

        -- Physical pin outputs
        o_puresn         : out std_logic;
        o_stopdis        : out std_logic;
        o_busy           : out std_logic
    );
end entity tdc_gpx_chip_init;

architecture rtl of tdc_gpx_chip_init is

    type t_init_state is (
        ST_OFF,             -- inactive, waiting for coordinator
        ST_POWERUP,
        ST_PU_RELEASE,
        ST_STOPDIS_HIGH,
        ST_CFG_WRITE,
        ST_CFG_WR_WAIT,
        ST_MASTER_RESET,
        ST_MR_WAIT,
        ST_RECOVERY,
        ST_STOPDIS_LOW
    );

    signal s_state_r    : t_init_state := ST_OFF;

    constant c_POWERUP_LAST  : unsigned(15 downto 0) := to_unsigned(g_POWERUP_CLKS - 1, 16);
    constant c_RECOVERY_LAST : unsigned(15 downto 0) := to_unsigned(g_RECOVERY_CLKS - 1, 16);

    signal s_wait_cnt_r      : unsigned(15 downto 0) := (others => '0');
    signal s_cfg_idx_r       : unsigned(3 downto 0)  := (others => '0');
    signal s_cfg_image_snap_r : t_cfg_image := (others => (others => '0'));
    signal s_init_mode_r     : std_logic := '1';  -- '1' = powerup (full), '0' = runtime cfg_write
    -- Pending cfg_write: set when i_cfg_write_req arrives same cycle as i_start
    -- (start wins). Processed automatically after the init sequence returns to
    -- ST_OFF, preventing silent loss of SW cfg_write commands.
    signal s_cfg_write_pending_r : std_logic := '0';

    signal s_req_valid_r     : std_logic := '0';
    signal s_req_rw_r        : std_logic := '0';
    signal s_req_addr_r      : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_puresn_r        : std_logic := '0';
    signal s_stopdis_r       : std_logic := '1';
    signal s_busy_r          : std_logic := '0';
    signal s_done_r          : std_logic := '0';
    signal s_timeout_out_r   : std_logic := '0';
    signal s_rsp_timeout_r   : unsigned(15 downto 0) := (others => '0');  -- bus response watchdog

    type t_reg_idx_array is array(0 to 10) of natural range 0 to 15;
    constant c_CFG_WRITE_SEQ  : t_reg_idx_array := (0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 14);
    constant c_CFG_WRITE_LAST : natural := 10;

begin

    p_fsm : process(i_clk)
        variable v_reg_num : natural range 0 to 15;
        variable v_wr_data : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r        <= ST_OFF;
                s_wait_cnt_r     <= (others => '0');
                s_rsp_timeout_r  <= (others => '0');
                s_cfg_idx_r      <= (others => '0');
                s_req_valid_r    <= '0';
                s_req_rw_r       <= '0';
                s_req_addr_r     <= (others => '0');
                s_req_wdata_r    <= (others => '0');
                s_puresn_r       <= '0';
                s_stopdis_r      <= '1';
                s_busy_r         <= '0';
                s_done_r         <= '0';
                s_timeout_out_r  <= '0';
                s_init_mode_r    <= '1';
                s_cfg_image_snap_r <= (others => (others => '0'));
                s_cfg_write_pending_r <= '0';
            else
                s_done_r        <= '0';
                s_timeout_out_r <= '0';

                case s_state_r is

                    when ST_OFF =>
                        s_busy_r <= '0';
                        if i_start = '1' then
                            s_init_mode_r      <= '1';
                            s_cfg_image_snap_r <= i_cfg_image;
                            s_rsp_timeout_r    <= (others => '0');  -- rearm watchdog
                            s_busy_r           <= '1';
                            -- If a cfg_write was requested the same cycle, queue
                            -- it so the SW pulse is not lost when init wins.
                            if i_cfg_write_req = '1' then
                                s_cfg_write_pending_r <= '1';
                            end if;
                            s_state_r          <= ST_POWERUP;
                        elsif i_cfg_write_req = '1' or s_cfg_write_pending_r = '1' then
                            s_cfg_write_pending_r <= '0';  -- consume
                            s_init_mode_r      <= '0';
                            s_cfg_image_snap_r <= i_cfg_image;
                            s_cfg_idx_r        <= (others => '0');
                            s_rsp_timeout_r    <= (others => '0');  -- rearm watchdog
                            s_busy_r           <= '1';
                            s_state_r          <= ST_STOPDIS_HIGH;
                        end if;

                    when ST_POWERUP =>
                        s_puresn_r  <= '0';
                        s_stopdis_r <= '1';
                        if s_wait_cnt_r = c_POWERUP_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_PU_RELEASE;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_PU_RELEASE =>
                        s_puresn_r <= '1';
                        if s_wait_cnt_r = c_RECOVERY_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_STOPDIS_HIGH;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_STOPDIS_HIGH =>
                        s_stopdis_r <= '1';
                        s_cfg_idx_r <= (others => '0');
                        s_state_r   <= ST_CFG_WRITE;

                    when ST_CFG_WRITE =>
                        v_reg_num     := c_CFG_WRITE_SEQ(to_integer(s_cfg_idx_r));
                        s_req_valid_r <= '1';
                        s_req_rw_r    <= '1';
                        s_req_addr_r  <= std_logic_vector(to_unsigned(v_reg_num, 4));
                        v_wr_data     := s_cfg_image_snap_r(v_reg_num)(g_BUS_DATA_WIDTH - 1 downto 0);
                        -- Safety: force Reg14[4]=0 to block 16-bit mode.
                        -- TDC-GPX 16-bit mode has CSN malfunction bug and requires
                        -- workaround sequences not implemented in this RTL.
                        if v_reg_num = 14 then
                            v_wr_data(4) := '0';
                        end if;
                        s_req_wdata_r <= v_wr_data;
                        s_state_r     <= ST_CFG_WR_WAIT;

                    when ST_CFG_WR_WAIT =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r   <= '0';
                            s_rsp_timeout_r <= (others => '0');
                            if s_cfg_idx_r = c_CFG_WRITE_LAST then
                                if s_init_mode_r = '1' then
                                    s_state_r <= ST_MASTER_RESET;
                                else
                                    s_busy_r  <= '0';
                                    s_done_r  <= '1';
                                    s_state_r <= ST_OFF;
                                end if;
                            else
                                s_cfg_idx_r <= s_cfg_idx_r + 1;
                                s_state_r   <= ST_CFG_WRITE;
                            end if;
                        elsif s_rsp_timeout_r = x"FFFF" then
                            -- Timeout: force done with error (bus hung).
                            -- Drop stopdis to match the normal completion path
                            -- (ST_STOPDIS_LOW) so the chip ends in a consistent
                            -- post-init state regardless of success/failure.
                            s_req_valid_r   <= '0';
                            s_stopdis_r     <= '0';
                            s_busy_r        <= '0';
                            s_done_r        <= '1';
                            s_timeout_out_r <= '1';
                            s_rsp_timeout_r <= (others => '0');  -- clear for next use
                            s_state_r       <= ST_OFF;
                        else
                            s_rsp_timeout_r <= s_rsp_timeout_r + 1;
                        end if;

                    when ST_MASTER_RESET =>
                        v_wr_data     := s_cfg_image_snap_r(4)(g_BUS_DATA_WIDTH - 1 downto 0);
                        v_wr_data(22) := '1';
                        s_req_valid_r <= '1';
                        s_req_rw_r    <= '1';
                        s_req_addr_r  <= c_TDC_REG4;
                        s_req_wdata_r <= v_wr_data;
                        s_state_r     <= ST_MR_WAIT;

                    when ST_MR_WAIT =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r   <= '0';
                            s_wait_cnt_r    <= (others => '0');
                            s_rsp_timeout_r <= (others => '0');
                            s_state_r       <= ST_RECOVERY;
                        elsif s_rsp_timeout_r = x"FFFF" then
                            -- Timeout: match normal completion post-init state.
                            s_req_valid_r   <= '0';
                            s_stopdis_r     <= '0';
                            s_busy_r        <= '0';
                            s_done_r        <= '1';
                            s_timeout_out_r <= '1';
                            s_rsp_timeout_r <= (others => '0');
                            s_state_r       <= ST_OFF;
                        else
                            s_rsp_timeout_r <= s_rsp_timeout_r + 1;
                        end if;

                    when ST_RECOVERY =>
                        if s_wait_cnt_r = c_RECOVERY_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_STOPDIS_LOW;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_STOPDIS_LOW =>
                        s_stopdis_r <= '0';
                        s_busy_r    <= '0';
                        s_done_r    <= '1';
                        s_state_r   <= ST_OFF;

                end case;
            end if;
        end if;
    end process p_fsm;

    o_bus_req_valid <= s_req_valid_r;
    o_bus_req_rw    <= s_req_rw_r;
    o_bus_req_addr  <= s_req_addr_r;
    o_bus_req_wdata <= s_req_wdata_r;
    o_puresn        <= s_puresn_r;
    o_stopdis       <= s_stopdis_r;
    o_busy          <= s_busy_r;
    o_done          <= s_done_r;
    o_timeout       <= s_timeout_out_r;

end architecture rtl;
