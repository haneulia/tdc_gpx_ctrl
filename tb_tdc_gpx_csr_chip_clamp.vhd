-- =============================================================================
-- tb_tdc_gpx_csr_chip_clamp.vhd
-- C01 Bus_Timing CSR clamp regression
-- =============================================================================
--
-- Drives tdc_gpx_csr_chip with the canonical source-level CSR32 implementation.
-- AXI4-Lite using px_axi_lite_writer from px_utility_pkg, then samples the
-- i_axis_aclk-domain outputs (o_bus_clk_div / o_bus_ticks) to verify the
-- C01 timing clamp. The matrix sweep covers raw CTL1 div/ticks values that
-- can be encoded by the CSR field. A final case uses px_axi_lite_reader to
-- read CTL1 back and confirm the written raw value persists in the CSR
-- register:
--
--   div >= c_BUS_CLK_DIV_MIN (=1)
--   if div = 1, ticks >= c_DEFAULT_BUS_READ_PERIOD_MIN_CLKS (=5)
--   else        ticks >= c_BUS_TICKS_MIN (=4)
--
-- BUS_TIMING register layout (CTL1 @ 0x04):
--   [5:0]  bus_clk_div
--   [8:6]  bus_ticks
--   [13:10] reg_target_addr
--   [15:14] reg_target_chip
--   [19:16] reg_target_chip_mask
--   [30]   reg_read_trigger
--   [31]   reg_write_trigger
--
-- The CSR transports CTL1 to i_axis_aclk via xpm_cdc_handshake, so the TB
-- waits a fixed CDC settling window (200 ns = 40 i_axis_aclk cycles @ 200 MHz)
-- after each AXI write before sampling the clamped output.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;
use work.px_utility_pkg.all;

entity tb_tdc_gpx_csr_chip_clamp is
end entity tb_tdc_gpx_csr_chip_clamp;

architecture sim of tb_tdc_gpx_csr_chip_clamp is

    constant c_AXI_PERIOD  : time := 10 ns;   -- 100 MHz
    constant c_AXIS_PERIOD : time := 5 ns;    -- 200 MHz
    constant c_RESET_HOLD  : time := 100 ns;
    constant c_CDC_WAIT    : time := 200 ns;  -- handshake settling window

    -- BUS_TIMING register byte address (CTL1 = 0x04)
    constant c_ADDR_BUS_TIMING : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) :=
        std_logic_vector(to_unsigned(16#04#, c_CSR_ADDR_WIDTH));

    -- =========================================================================
    -- Clock / reset
    -- =========================================================================
    signal sim_done   : boolean   := false;
    signal clk_axi    : std_logic := '0';
    signal clk_axis   : std_logic := '0';
    signal rstn_axi   : std_logic := '0';
    signal rstn_axis  : std_logic := '0';

    -- =========================================================================
    -- AXI4-Lite signals (driven by px_axi_lite_writer + px_axi_lite_reader)
    -- =========================================================================
    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;
    signal s_axi_awaddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_axi_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_axi_wvalid  : std_logic := '0';
    signal s_axi_wready  : std_logic;
    signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axi_wstrb   : std_logic_vector(3 downto 0)  := (others => '0');
    signal s_axi_bvalid  : std_logic;
    signal s_axi_bready  : std_logic := '0';
    signal s_axi_bresp   : std_logic_vector(1 downto 0);
    signal s_axi_arvalid : std_logic := '0';
    signal s_axi_arready : std_logic;
    signal s_axi_araddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_axi_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_axi_rvalid  : std_logic;
    signal s_axi_rready  : std_logic := '0';
    signal s_axi_rdata   : std_logic_vector(31 downto 0);
    signal s_axi_rresp   : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- DUT outputs (i_axis_aclk domain)
    -- =========================================================================
    signal o_cfg_image            : t_cfg_image;
    signal o_bus_clk_div          : unsigned(5 downto 0);
    signal o_bus_ticks            : unsigned(2 downto 0);
    signal o_start_off1           : unsigned(17 downto 0);
    signal o_cfg_reg7             : std_logic_vector(31 downto 0);
    signal o_max_scan_5ns_ticks   : unsigned(15 downto 0);
    signal o_max_hits_cfg         : unsigned(2 downto 0);

    signal o_cmd_reg_read         : std_logic;
    signal o_cmd_reg_write        : std_logic;
    signal o_cmd_reg_addr         : std_logic_vector(3 downto 0);
    signal o_cmd_reg_chip         : unsigned(1 downto 0);
    signal o_cmd_reg_chip_address : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_cdc_idle             : std_logic;
    signal o_irq                  : std_logic;

    -- Tied-off inputs from the rest of the chip (reg result side)
    signal i_cmd_reg_rdata_0      : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) := (others => '0');
    signal i_cmd_reg_rdata_1      : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) := (others => '0');
    signal i_cmd_reg_rdata_2      : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) := (others => '0');
    signal i_cmd_reg_rdata_3      : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) := (others => '0');
    signal i_cmd_reg_rvalid       : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal i_cmd_reg_done_pulse   : std_logic := '0';
    signal i_cmd_reg_addr_done    : std_logic_vector(3 downto 0) := (others => '0');

    -- =========================================================================
    -- Helpers
    -- =========================================================================
    function fn_pack_bus_timing(div : natural; ticks : natural)
        return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO) := std_logic_vector(to_unsigned(div, 6));
        v(c_BT_TICKS_HI   downto c_BT_TICKS_LO)   := std_logic_vector(to_unsigned(ticks, 3));
        return v;
    end function;

begin

    clk_axi  <= not clk_axi  after c_AXI_PERIOD  / 2 when not sim_done else '0';
    clk_axis <= not clk_axis after c_AXIS_PERIOD / 2 when not sim_done else '0';

    p_reset : process
    begin
        rstn_axi  <= '0';
        rstn_axis <= '0';
        wait for c_RESET_HOLD;
        wait until rising_edge(clk_axi);
        rstn_axi  <= '1';
        wait until rising_edge(clk_axis);
        rstn_axis <= '1';
        wait;
    end process p_reset;

    -- =========================================================================
    -- DUT: tdc_gpx_csr_chip + canonical source-level CSR32 implementation
    -- =========================================================================
    u_dut : entity work.tdc_gpx_csr_chip
        port map (
            -- AXI clock / reset
            s_axi_aclk          => clk_axi,
            s_axi_aresetn       => rstn_axi,
            s_axi_awvalid       => s_axi_awvalid,
            s_axi_awready       => s_axi_awready,
            s_axi_awaddr        => s_axi_awaddr,
            s_axi_awprot        => s_axi_awprot,
            s_axi_wvalid        => s_axi_wvalid,
            s_axi_wready        => s_axi_wready,
            s_axi_wdata         => s_axi_wdata,
            s_axi_wstrb         => s_axi_wstrb,
            s_axi_bvalid        => s_axi_bvalid,
            s_axi_bready        => s_axi_bready,
            s_axi_bresp         => s_axi_bresp,
            s_axi_arvalid       => s_axi_arvalid,
            s_axi_arready       => s_axi_arready,
            s_axi_araddr        => s_axi_araddr,
            s_axi_arprot        => s_axi_arprot,
            s_axi_rvalid        => s_axi_rvalid,
            s_axi_rready        => s_axi_rready,
            s_axi_rdata         => s_axi_rdata,
            s_axi_rresp         => s_axi_rresp,
            -- TDC / AXI-Stream clock
            i_axis_aclk         => clk_axis,
            i_axis_aresetn      => rstn_axis,
            -- Configuration outputs
            o_cfg_image         => o_cfg_image,
            o_bus_clk_div       => o_bus_clk_div,
            o_bus_ticks         => o_bus_ticks,
            o_start_off1        => o_start_off1,
            o_cfg_reg7          => o_cfg_reg7,
            o_max_scan_5ns_ticks => o_max_scan_5ns_ticks,
            o_max_hits_cfg      => o_max_hits_cfg,
            o_falling_enable    => open,
            -- Reg access commands
            o_cmd_reg_read      => o_cmd_reg_read,
            o_cmd_reg_write     => o_cmd_reg_write,
            o_cmd_reg_addr      => o_cmd_reg_addr,
            o_cmd_reg_chip      => o_cmd_reg_chip,
            -- Reg result inputs (tied off)
            i_cmd_reg_rdata_0   => i_cmd_reg_rdata_0,
            i_cmd_reg_rdata_1   => i_cmd_reg_rdata_1,
            i_cmd_reg_rdata_2   => i_cmd_reg_rdata_2,
            i_cmd_reg_rdata_3   => i_cmd_reg_rdata_3,
            i_cmd_reg_rvalid    => i_cmd_reg_rvalid,
            i_cmd_reg_done_pulse => i_cmd_reg_done_pulse,
            i_cmd_reg_addr_done  => i_cmd_reg_addr_done,
            -- Misc
            o_cmd_reg_chip_address => o_cmd_reg_chip_address,
            o_cdc_idle          => o_cdc_idle,
            o_irq               => o_irq
        );

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    p_stim : process
        variable v_pass : natural := 0;
        variable v_fail : natural := 0;
        variable v_exp_div       : natural := 0;
        variable v_exp_ticks     : natural := 0;
        variable v_exp_ticks_min : natural := 0;

        procedure check_clamp(constant tag         : in string;
                              constant req_div    : in natural;
                              constant req_ticks  : in natural;
                              constant exp_div    : in natural;
                              constant exp_ticks  : in natural) is
            variable v_word : std_logic_vector(31 downto 0);
        begin
            v_word := fn_pack_bus_timing(req_div, req_ticks);
            report tag & ": writing BUS_TIMING div=" & integer'image(req_div)
                & ", ticks=" & integer'image(req_ticks)
                severity note;
            px_axi_lite_writer(
                addr        => c_ADDR_BUS_TIMING,
                val         => v_word,
                axi_aclk    => clk_axi,
                axi_awaddr  => s_axi_awaddr,
                axi_awprot  => s_axi_awprot,
                axi_awvalid => s_axi_awvalid,
                axi_awready => s_axi_awready,
                axi_wdata   => s_axi_wdata,
                axi_wstrb   => s_axi_wstrb,
                axi_wvalid  => s_axi_wvalid,
                axi_wready  => s_axi_wready,
                axi_bresp   => s_axi_bresp,
                axi_bvalid  => s_axi_bvalid,
                axi_bready  => s_axi_bready
            );

            -- Wait for CDC handshake (s_axi_aclk -> i_axis_aclk) to settle.
            wait for c_CDC_WAIT;
            wait until rising_edge(clk_axis);

            if to_integer(o_bus_clk_div) = exp_div
               and to_integer(o_bus_ticks) = exp_ticks then
                v_pass := v_pass + 1;
                report tag
                    & ": PASS (div_clamped=" & integer'image(to_integer(o_bus_clk_div))
                    & ", ticks_clamped=" & integer'image(to_integer(o_bus_ticks)) & ")"
                    severity note;
            else
                v_fail := v_fail + 1;
                report tag
                    & ": FAIL  expected (div="  & integer'image(exp_div)
                    & ",ticks=" & integer'image(exp_ticks)
                    & "), got (div=" & integer'image(to_integer(o_bus_clk_div))
                    & ",ticks="      & integer'image(to_integer(o_bus_ticks)) & ")"
                    severity error;
            end if;
        end procedure check_clamp;
    begin
        -- Drive AXI signals to defaults
        s_axi_awvalid <= '0';
        s_axi_wvalid  <= '0';
        s_axi_arvalid <= '0';
        s_axi_bready  <= '0';
        s_axi_rready  <= '0';
        s_axi_wstrb   <= "0000";

        wait until rstn_axi = '1' and rstn_axis = '1';
        wait for 10 * c_AXI_PERIOD;

        -- ---------------------------------------------------------------------
        -- Initial state: confirm power-on init reflects c_INIT_BUS_TIMING.
        -- c_INIT_BUS_TIMING = x"00000142" -> div=2, ticks=5 (legal, no clamp).
        -- ---------------------------------------------------------------------
        wait for c_CDC_WAIT;
        if to_integer(o_bus_clk_div) = 2 and to_integer(o_bus_ticks) = 5 then
            v_pass := v_pass + 1;
            report "[init] PASS  div=2, ticks=5 (c_INIT_BUS_TIMING)" severity note;
        else
            v_fail := v_fail + 1;
            report "[init] FAIL  init div/ticks not 2/5" severity error;
        end if;

        -- ---------------------------------------------------------------------
        -- Clamp matrix
        -- ---------------------------------------------------------------------
        check_clamp("[c01] div=0,ticks=4", 0, 4, 1, 5);  -- div clamp -> 1, ticks_min=5
        check_clamp("[c02] div=1,ticks=3", 1, 3, 1, 5);  -- ticks clamp at div=1
        check_clamp("[c03] div=1,ticks=4", 1, 4, 1, 5);  -- C01 contract: ticks=4 illegal at div=1
        check_clamp("[c04] div=1,ticks=5", 1, 5, 1, 5);  -- legal boundary
        check_clamp("[c05] div=1,ticks=7", 1, 7, 1, 7);  -- legal, max
        check_clamp("[c06] div=2,ticks=3", 2, 3, 2, 4);  -- ticks clamp at div=2
        check_clamp("[c07] div=2,ticks=4", 2, 4, 2, 4);  -- legal boundary
        check_clamp("[c08] div=2,ticks=5", 2, 5, 2, 5);  -- default-equivalent
        check_clamp("[c09] div=3,ticks=4", 3, 4, 3, 4);  -- legal
        check_clamp("[c10] div=8,ticks=7", 8, 7, 8, 7);  -- legal, large div

        -- Exhaustive illegal/legal matrix for the timing-sensitive range.
        -- div=0..5 covers the clamp boundary and fast legal candidates.
        -- ticks=0..7 covers every encodable 3-bit BUS_TICKS value.
        for div_idx in 0 to 5 loop
            for ticks_idx in 0 to 7 loop
                if div_idx < c_BUS_CLK_DIV_MIN then
                    v_exp_div := c_BUS_CLK_DIV_MIN;
                else
                    v_exp_div := div_idx;
                end if;

                if v_exp_div = 1 then
                    v_exp_ticks_min := c_DEFAULT_BUS_READ_PERIOD_MIN_CLKS;
                else
                    v_exp_ticks_min := c_BUS_TICKS_MIN;
                end if;

                if ticks_idx < v_exp_ticks_min then
                    v_exp_ticks := v_exp_ticks_min;
                else
                    v_exp_ticks := ticks_idx;
                end if;

                check_clamp("[m] div=" & integer'image(div_idx)
                            & ",ticks=" & integer'image(ticks_idx),
                            div_idx, ticks_idx, v_exp_div, v_exp_ticks);
            end loop;
        end loop;

        -- Large divider edge: all ticks still use the absolute ticks>=4 rule.
        for ticks_idx in 0 to 7 loop
            if ticks_idx < c_BUS_TICKS_MIN then
                v_exp_ticks := c_BUS_TICKS_MIN;
            else
                v_exp_ticks := ticks_idx;
            end if;
            check_clamp("[m] div=63,ticks=" & integer'image(ticks_idx),
                        63, ticks_idx, 63, v_exp_ticks);
        end loop;

        -- ---------------------------------------------------------------------
        -- [c11] px_axi_lite_reader: confirm the last raw value written to CTL1
        --       is retained in the CSR register file. The reader observes the
        --       raw register, not the clamped output, so the expected value
        --       is exactly the final matrix write fn_pack_bus_timing(63, 7).
        -- ---------------------------------------------------------------------
        report "[c11] readback CTL1 via px_axi_lite_reader" severity note;
        px_axi_lite_reader(
            addr           => c_ADDR_BUS_TIMING,
            val            => fn_pack_bus_timing(63, 7),
            comp           => '1',  -- enable comparison
            fail_on_error  => '1',  -- assert FAILURE on mismatch
            axi_aclk       => clk_axi,
            axi_araddr     => s_axi_araddr,
            axi_arprot     => s_axi_arprot,
            axi_arvalid    => s_axi_arvalid,
            axi_arready    => s_axi_arready,
            axi_rdata      => s_axi_rdata,
            axi_rresp      => s_axi_rresp,
            axi_rvalid     => s_axi_rvalid,
            axi_rready     => s_axi_rready
        );
        v_pass := v_pass + 1;
        report "[c11] PASS  CTL1 readback matches last write (raw)"
            severity note;

        -- Summary
        report "------------------------------------------------------------"
            severity note;
        report "tb_tdc_gpx_csr_chip_clamp summary: "
            & integer'image(v_pass) & " pass, "
            & integer'image(v_fail) & " fail"
            severity note;
        report "------------------------------------------------------------"
            severity note;

        if v_fail = 0 then
            report "*** ALL TESTS PASSED *** (cases=" & integer'image(v_pass) & ")"
                severity note;
        else
            report "*** TESTS FAILED *** (fail=" & integer'image(v_fail) & ")"
                severity failure;
        end if;

        sim_done <= true;
        wait for 100 ns;
        std.env.finish;
    end process p_stim;

end architecture sim;
