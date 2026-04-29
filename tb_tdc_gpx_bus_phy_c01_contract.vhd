-- =============================================================================
-- tb_tdc_gpx_bus_phy_c01_contract.vhd
-- C01 GPX Bus Read contract regression
-- =============================================================================
--
-- Checks implemented from Doc/cluster_analysis/C01_GPX_Bus_Read v009:
--   1. div=1,ticks=4 is locally clamped to the legal 200 MHz READ timing
--      equivalent of ticks=5.
--   2. div=1,ticks=5 burst READ initiation interval is 25 ns.
--   3. PULLUP_OR_NOT_CONNECTED OEN mode keeps o_oen high and still supports
--      normal RDN-gated reads.
--   4. o_rsp_pending is a registered module-boundary output that asserts while
--      a response is pending or held by AXI tvalid.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_bus_phy_c01_contract is
end entity tb_tdc_gpx_bus_phy_c01_contract;

architecture sim of tb_tdc_gpx_bus_phy_c01_contract is

    constant c_CLK_PERIOD : time := 5 ns;
    constant c_DATA_W     : natural := c_TDC_BUS_WIDTH;

    signal s_done  : std_logic := '0';
    signal s_clk   : std_logic := '0';
    signal s_rst_n : std_logic := '0';

    -- Dynamic OEN DUT
    signal s_dyn_ticks       : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_dyn_req_valid   : std_logic := '0';
    signal s_dyn_req_rw      : std_logic := '0';
    signal s_dyn_req_addr    : std_logic_vector(3 downto 0) := (others => '0');
    signal s_dyn_req_wdata   : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_dyn_oen_perm    : std_logic := '0';
    signal s_dyn_req_burst   : std_logic := '0';
    signal s_dyn_busy        : std_logic;
    signal s_dyn_rsp_pending : std_logic;
    signal s_dyn_adr         : std_logic_vector(3 downto 0);
    signal s_dyn_csn         : std_logic;
    signal s_dyn_rdn         : std_logic;
    signal s_dyn_wrn         : std_logic;
    signal s_dyn_oen         : std_logic;
    signal s_dyn_d           : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_dyn_tvalid      : std_logic;
    signal s_dyn_tdata       : std_logic_vector(31 downto 0);
    signal s_dyn_tkeep       : std_logic_vector(3 downto 0);
    signal s_dyn_tuser       : std_logic_vector(7 downto 0);

    -- Pull-up / not-connected OEN DUT
    signal s_pu_req_valid   : std_logic := '0';
    signal s_pu_req_rw      : std_logic := '0';
    signal s_pu_req_addr    : std_logic_vector(3 downto 0) := (others => '0');
    signal s_pu_req_wdata   : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_pu_oen_perm    : std_logic := '0';
    signal s_pu_req_burst   : std_logic := '0';
    signal s_pu_busy        : std_logic;
    signal s_pu_rsp_pending : std_logic;
    signal s_pu_adr         : std_logic_vector(3 downto 0);
    signal s_pu_csn         : std_logic;
    signal s_pu_rdn         : std_logic;
    signal s_pu_wrn         : std_logic;
    signal s_pu_oen         : std_logic;
    signal s_pu_d           : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_pu_tvalid      : std_logic;
    signal s_pu_tdata       : std_logic_vector(31 downto 0);
    signal s_pu_tkeep       : std_logic_vector(3 downto 0);
    signal s_pu_tuser       : std_logic_vector(7 downto 0);

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2 when s_done = '0' else s_clk;

    -- Dynamic mode chip model: GPX output buffer follows OEN low and RDN low.
    s_dyn_d <= std_logic_vector(to_unsigned(16#12345#, c_DATA_W))
               when s_dyn_oen = '0' and s_dyn_rdn = '0'
               else (others => 'Z');

    -- Pull-up/NC mode chip model: OEN is effectively high; RDN gates output.
    s_pu_d <= std_logic_vector(to_unsigned(16#23456#, c_DATA_W))
              when s_pu_rdn = '0'
              else (others => 'Z');

    u_dyn : entity work.tdc_gpx_bus_phy
        generic map (
            g_OEN_MODE => "DYNAMIC_CONNECTED"
        )
        port map (
            i_clk           => s_clk,
            i_rst_n         => s_rst_n,
            i_tick_en       => '1',
            i_bus_ticks     => s_dyn_ticks,
            i_bus_clk_div   => to_unsigned(1, 6),
            i_req_valid     => s_dyn_req_valid,
            i_req_rw        => s_dyn_req_rw,
            i_req_addr      => s_dyn_req_addr,
            i_req_wdata     => s_dyn_req_wdata,
            i_oen_permanent => s_dyn_oen_perm,
            i_req_burst     => s_dyn_req_burst,
            o_busy          => s_dyn_busy,
            o_rsp_pending   => s_dyn_rsp_pending,
            o_adr           => s_dyn_adr,
            o_csn           => s_dyn_csn,
            o_rdn           => s_dyn_rdn,
            o_wrn           => s_dyn_wrn,
            o_oen           => s_dyn_oen,
            io_d            => s_dyn_d,
            i_ef1_pin       => '1',
            i_ef2_pin       => '1',
            i_lf1_pin       => '0',
            i_lf2_pin       => '0',
            i_irflag_pin    => '0',
            i_errflag_pin   => '0',
            o_m_axis_tvalid => s_dyn_tvalid,
            o_m_axis_tdata  => s_dyn_tdata,
            o_m_axis_tkeep  => s_dyn_tkeep,
            o_m_axis_tuser  => s_dyn_tuser,
            i_m_axis_tready => '1',
            o_ef1_sync      => open,
            o_ef2_sync      => open,
            o_lf1_sync      => open,
            o_lf2_sync      => open,
            o_irflag_sync   => open,
            o_errflag_sync  => open
        );

    u_pullup : entity work.tdc_gpx_bus_phy
        generic map (
            g_OEN_MODE => "PULLUP_OR_NOT_CONNECTED"
        )
        port map (
            i_clk           => s_clk,
            i_rst_n         => s_rst_n,
            i_tick_en       => '1',
            i_bus_ticks     => to_unsigned(5, 3),
            i_bus_clk_div   => to_unsigned(1, 6),
            i_req_valid     => s_pu_req_valid,
            i_req_rw        => s_pu_req_rw,
            i_req_addr      => s_pu_req_addr,
            i_req_wdata     => s_pu_req_wdata,
            i_oen_permanent => s_pu_oen_perm,
            i_req_burst     => s_pu_req_burst,
            o_busy          => s_pu_busy,
            o_rsp_pending   => s_pu_rsp_pending,
            o_adr           => s_pu_adr,
            o_csn           => s_pu_csn,
            o_rdn           => s_pu_rdn,
            o_wrn           => s_pu_wrn,
            o_oen           => s_pu_oen,
            io_d            => s_pu_d,
            i_ef1_pin       => '1',
            i_ef2_pin       => '1',
            i_lf1_pin       => '0',
            i_lf2_pin       => '0',
            i_irflag_pin    => '0',
            i_errflag_pin   => '0',
            o_m_axis_tvalid => s_pu_tvalid,
            o_m_axis_tdata  => s_pu_tdata,
            o_m_axis_tkeep  => s_pu_tkeep,
            o_m_axis_tuser  => s_pu_tuser,
            i_m_axis_tready => '1',
            o_ef1_sync      => open,
            o_ef2_sync      => open,
            o_lf1_sync      => open,
            o_lf2_sync      => open,
            o_irflag_sync   => open,
            o_errflag_sync  => open
        );

    p_oen_pullup_monitor : process(s_clk)
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '1' then
                assert s_pu_oen = '1'
                    report "PULLUP_OR_NOT_CONNECTED mode must keep o_oen high"
                    severity failure;
            end if;
        end if;
    end process p_oen_pullup_monitor;

    p_stim : process
        procedure wait_level(
            signal sig      : in std_logic;
            constant target : in std_logic;
            constant msg    : in string
        ) is
        begin
            for i in 0 to 100 loop
                if sig = target then
                    return;
                end if;
                wait until rising_edge(s_clk);
            end loop;
            assert false report msg severity failure;
        end procedure wait_level;

        variable v_fall_1 : time;
        variable v_fall_2 : time;
        variable v_pw     : time;
    begin
        s_rst_n <= '0';
        wait for 10 * c_CLK_PERIOD;
        wait until rising_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);

        -- [1] div=1,ticks=4 must be clamped to ticks=5 at 200 MHz.
        s_dyn_ticks     <= to_unsigned(4, 3);
        s_dyn_req_valid <= '1';
        s_dyn_req_rw    <= '0';
        s_dyn_req_addr  <= c_TDC_REG8_IFIFO1;
        wait until s_dyn_rdn = '0';
        v_fall_1 := now;
        wait until s_dyn_rdn = '1';
        v_pw := now - v_fall_1;
        assert v_pw = 15 ns
            report "div=1,ticks=4 was not clamped to legal ticks=5 timing"
            severity failure;
        wait_level(s_dyn_rsp_pending, '1',
                   "dynamic registered o_rsp_pending did not assert");
        wait_level(s_dyn_tvalid, '1',
                   "dynamic AXI response did not assert");
        assert s_dyn_tdata(c_DATA_W - 1 downto 0) =
               std_logic_vector(to_unsigned(16#12345#, c_DATA_W))
            report "dynamic read data mismatch"
            severity failure;
        s_dyn_req_valid <= '0';
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        assert s_dyn_rsp_pending = '0'
            report "registered o_rsp_pending did not clear after response handshake"
            severity failure;

        -- [2] div=1,ticks=5 burst READ II must be 25 ns at 200 MHz.
        s_dyn_ticks     <= to_unsigned(5, 3);
        s_dyn_req_valid <= '1';
        s_dyn_req_burst <= '1';
        s_dyn_oen_perm  <= '1';
        wait until s_dyn_rdn = '0';
        v_fall_1 := now;
        wait until s_dyn_rdn = '1';
        wait until s_dyn_rdn = '0';
        v_fall_2 := now;
        assert v_fall_2 - v_fall_1 = 25 ns
            report "div=1,ticks=5 burst READ II is not 25 ns"
            severity failure;
        s_dyn_req_burst <= '0';
        s_dyn_req_valid <= '0';
        s_dyn_oen_perm  <= '0';
        wait for 10 * c_CLK_PERIOD;

        -- [3] Pull-up/NC mode keeps OEN high but read still completes.
        s_pu_req_valid <= '1';
        s_pu_req_rw    <= '0';
        s_pu_req_addr  <= c_TDC_REG8_IFIFO1;
        s_pu_oen_perm  <= '1';
        wait until s_pu_rdn = '0';
        assert s_pu_oen = '1'
            report "pull-up/NC read drove OEN low"
            severity failure;
        wait_level(s_pu_tvalid, '1',
                   "pull-up/NC AXI response did not assert");
        assert s_pu_tdata(c_DATA_W - 1 downto 0) =
               std_logic_vector(to_unsigned(16#23456#, c_DATA_W))
            report "pull-up/NC read data mismatch"
            severity failure;
        s_pu_req_valid <= '0';
        s_pu_oen_perm  <= '0';

        wait for 5 * c_CLK_PERIOD;
        report "tb_tdc_gpx_bus_phy_c01_contract PASS" severity note;
        s_done <= '1';
        std.env.finish;
    end process p_stim;

end architecture sim;
