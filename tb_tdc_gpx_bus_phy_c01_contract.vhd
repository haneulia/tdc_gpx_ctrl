-- =============================================================================
-- tb_tdc_gpx_bus_phy_c01_contract.vhd
-- C01 GPX Bus Read contract regression
-- =============================================================================
--
-- Checks implemented from Doc/cluster_analysis/C01_GPX_Bus_Read v009:
--   1. Illegal div/ticks combinations are locally clamped to the legal
--      200 MHz READ timing before the physical strobe is generated.
--   2. div=1,ticks=5 burst READ initiation interval is 25 ns.
--   3. PULLUP_OR_NOT_CONNECTED OEN mode keeps o_oen high and still supports
--      normal RDN-gated reads.
--   4. o_rsp_pending is a registered module-boundary output that asserts while
--      a response is pending or held by AXI tvalid.
--   5. Burst backpressure never duplicates or drops a completed GPX read.
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
    signal s_dyn_tick_en     : std_logic := '0';
    signal s_dyn_div         : natural range 0 to 63 := 1;
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
    signal s_dyn_d_out       : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_dyn_d_tri       : std_logic;
    signal s_dyn_tvalid      : std_logic;
    signal s_dyn_tdata       : std_logic_vector(31 downto 0);
    signal s_dyn_tkeep       : std_logic_vector(3 downto 0);
    signal s_dyn_tuser       : std_logic_vector(7 downto 0);
    signal s_dyn_tready      : std_logic := '1';
    signal s_dyn_rsp_hs_cnt  : natural := 0;
    signal s_dyn_rdn_rise_cnt : natural := 0;

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
    signal s_pu_d_out       : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_pu_d_tri       : std_logic;
    signal s_pu_tvalid      : std_logic;
    signal s_pu_tdata       : std_logic_vector(31 downto 0);
    signal s_pu_tkeep       : std_logic_vector(3 downto 0);
    signal s_pu_tuser       : std_logic_vector(7 downto 0);

begin

    s_clk <= not s_clk after c_CLK_PERIOD / 2 when s_done = '0' else s_clk;

    s_dyn_d <= s_dyn_d_out when s_dyn_d_tri = '0' else (others => 'Z');
    s_pu_d  <= s_pu_d_out when s_pu_d_tri = '0' else (others => 'Z');

    p_dyn_tick_gen : process(s_clk)
        variable v_cnt : natural range 0 to 63 := 0;
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                v_cnt := 0;
                s_dyn_tick_en <= '0';
            else
                if s_dyn_div <= 1 or v_cnt = s_dyn_div - 1 then
                    v_cnt := 0;
                    s_dyn_tick_en <= '1';
                else
                    v_cnt := v_cnt + 1;
                    s_dyn_tick_en <= '0';
                end if;
            end if;
        end if;
    end process p_dyn_tick_gen;

    p_dyn_transfer_count : process(s_clk)
        variable v_rdn_d : std_logic := '1';
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                s_dyn_rsp_hs_cnt   <= 0;
                s_dyn_rdn_rise_cnt <= 0;
                v_rdn_d            := '1';
            else
                if s_dyn_tvalid = '1' and s_dyn_tready = '1' then
                    s_dyn_rsp_hs_cnt <= s_dyn_rsp_hs_cnt + 1;
                end if;
                if v_rdn_d = '0' and s_dyn_rdn = '1' then
                    s_dyn_rdn_rise_cnt <= s_dyn_rdn_rise_cnt + 1;
                end if;
                v_rdn_d := s_dyn_rdn;
            end if;
        end if;
    end process p_dyn_transfer_count;

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
            i_tick_en       => s_dyn_tick_en,
            i_bus_ticks     => s_dyn_ticks,
            i_bus_clk_div   => to_unsigned(s_dyn_div, 6),
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
            i_d             => s_dyn_d,
            o_d             => s_dyn_d_out,
            o_d_tri         => s_dyn_d_tri,
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
            i_m_axis_tready => s_dyn_tready,
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
            i_d             => s_pu_d,
            o_d             => s_pu_d_out,
            o_d_tri         => s_pu_d_tri,
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

        procedure check_dyn_read_low(
            constant tag       : in string;
            constant req_div   : in natural;
            constant req_ticks : in natural;
            constant exp_low   : in time
        ) is
            variable v_fall : time;
            variable v_pw_l : time;
        begin
            report tag & ": div=" & integer'image(req_div)
                & ", ticks=" & integer'image(req_ticks)
                & ", expected RDN low=" & time'image(exp_low)
                severity note;

            s_dyn_div       <= req_div;
            s_dyn_ticks     <= to_unsigned(req_ticks, 3);
            s_dyn_req_valid <= '0';
            s_dyn_req_burst <= '0';
            s_dyn_oen_perm  <= '0';
            wait until rising_edge(s_clk);
            wait until rising_edge(s_clk);

            s_dyn_req_valid <= '1';
            s_dyn_req_rw    <= '0';
            s_dyn_req_addr  <= c_TDC_REG8_IFIFO1;
            wait until s_dyn_rdn = '0';
            v_fall := now;
            wait until s_dyn_rdn = '1';
            v_pw_l := now - v_fall;
            assert v_pw_l = exp_low
                report tag & ": RDN low width mismatch, got "
                    & time'image(v_pw_l) & ", expected " & time'image(exp_low)
                severity failure;

            wait_level(s_dyn_rsp_pending, '1',
                       tag & ": registered o_rsp_pending did not assert");
            wait_level(s_dyn_tvalid, '1',
                       tag & ": AXI response did not assert");
            assert s_dyn_tdata(c_DATA_W - 1 downto 0) =
                   std_logic_vector(to_unsigned(16#12345#, c_DATA_W))
                report tag & ": read data mismatch"
                severity failure;
            s_dyn_req_valid <= '0';
            wait_level(s_dyn_busy, '0', tag & ": bus did not return idle");
            wait until rising_edge(s_clk);
            assert s_dyn_rsp_pending = '0'
                report tag & ": registered o_rsp_pending did not clear"
                severity failure;
        end procedure check_dyn_read_low;

        variable v_fall_1 : time;
        variable v_fall_2 : time;
        variable v_rsp_base : natural;
        variable v_rdn_base : natural;
    begin
        s_rst_n <= '0';
        wait for 10 * c_CLK_PERIOD;
        wait until rising_edge(s_clk);
        s_rst_n <= '1';
        wait until rising_edge(s_clk);

        -- [1] Local bus_phy clamp matrix samples. These prove the leaf
        -- boundary still protects GPX timing even if CSR policy is bypassed.
        check_dyn_read_low("[1a] div=0,ticks=0 local clamp", 0, 0, 25 ns);
        check_dyn_read_low("[1b] div=1,ticks=4 local clamp", 1, 4, 25 ns);
        check_dyn_read_low("[1c] div=2,ticks=3 local clamp", 2, 3, 30 ns);
        check_dyn_read_low("[1d] div=3,ticks=4 local clamp", 3, 4, 45 ns);

        -- [2] div=1,ticks=5 is clamped to ticks=7. Burst READ II is 35 ns
        -- while the RDN-low-to-IOB-capture window is the required 25 ns.
        s_dyn_div       <= 1;
        s_dyn_ticks     <= to_unsigned(5, 3);
        wait until rising_edge(s_clk);
        wait until rising_edge(s_clk);
        s_dyn_req_valid <= '1';
        s_dyn_req_burst <= '1';
        s_dyn_oen_perm  <= '1';
        wait until s_dyn_rdn = '0';
        v_fall_1 := now;
        wait until s_dyn_rdn = '1';
        wait until s_dyn_rdn = '0';
        v_fall_2 := now;
        assert v_fall_2 - v_fall_1 = 35 ns
            report "div=1,ticks=5 clamp did not produce 35 ns burst READ II"
            severity failure;
        s_dyn_req_burst <= '0';
        s_dyn_req_valid <= '0';
        s_dyn_oen_perm  <= '0';
        wait for 10 * c_CLK_PERIOD;

        -- [3] A held burst response must not be re-issued when tready returns.
        v_rsp_base := s_dyn_rsp_hs_cnt;
        v_rdn_base := s_dyn_rdn_rise_cnt;
        s_dyn_tready    <= '0';
        s_dyn_req_valid <= '1';
        s_dyn_req_burst <= '1';
        s_dyn_oen_perm  <= '1';

        -- Allow exactly two physical GPX reads, then hold the second response
        -- at Phase H for several clocks while AXIS is backpressured.
        wait until s_dyn_rdn_rise_cnt = v_rdn_base + 2;
        for i in 0 to 4 loop
            wait until rising_edge(s_clk);
        end loop;

        -- End the burst and release backpressure together. Both completed
        -- physical reads must emerge exactly once.
        s_dyn_req_burst <= '0';
        s_dyn_req_valid <= '0';
        s_dyn_oen_perm  <= '0';
        s_dyn_tready    <= '1';
        for i in 0 to 8 loop
            wait until rising_edge(s_clk);
        end loop;

        assert s_dyn_rdn_rise_cnt - v_rdn_base = 2
            report "burst backpressure issued an unexpected physical GPX read"
            severity failure;
        assert s_dyn_rsp_hs_cnt - v_rsp_base = 2
            report "burst backpressure AXIS response count="
                   & integer'image(s_dyn_rsp_hs_cnt - v_rsp_base)
                   & ", expected=2"
            severity failure;

        -- [4] Pull-up/NC mode keeps OEN high but read still completes.
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
