-- =============================================================================
-- tb_tdc_gpx_csr_chip_image_cdc.vhd
-- Packed cfg_image CDC regression
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;
use work.px_utility_pkg.all;

entity tb_tdc_gpx_csr_chip_image_cdc is
end entity tb_tdc_gpx_csr_chip_image_cdc;

architecture sim of tb_tdc_gpx_csr_chip_image_cdc is
    constant C_AXI_PERIOD  : time := 5 ns;
    constant C_AXIS_PERIOD : time := 20 ns;
    constant C_RESET_HOLD  : time := 100 ns;

    signal sim_done  : boolean := false;
    signal clk_axi   : std_logic := '0';
    signal clk_axis  : std_logic := '0';
    signal rstn_axi  : std_logic := '0';
    signal rstn_axis : std_logic := '0';

    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;
    signal s_axi_awaddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_axi_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_axi_wvalid  : std_logic := '0';
    signal s_axi_wready  : std_logic;
    signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axi_wstrb   : std_logic_vector(3 downto 0) := (others => '0');
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

    signal s_expect_nonidle : std_logic := '0';
    signal s_idle_gap_seen  : std_logic := '0';

    function fn_image_word(tag : natural; index : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := x"A5000000";
    begin
        v_word(23 downto 16) := std_logic_vector(to_unsigned(tag mod 256, 8));
        v_word(15 downto 0) := std_logic_vector(to_unsigned(index mod 65536, 16));
        return v_word;
    end function;
begin
    clk_axi  <= not clk_axi after C_AXI_PERIOD / 2 when not sim_done else '0';
    clk_axis <= not clk_axis after C_AXIS_PERIOD / 2 when not sim_done else '0';

    p_reset : process
    begin
        wait for C_RESET_HOLD;
        wait until rising_edge(clk_axi);
        rstn_axi <= '1';
        wait until rising_edge(clk_axis);
        rstn_axis <= '1';
        wait;
    end process p_reset;

    p_idle_gap_monitor : process(clk_axi)
    begin
        if rising_edge(clk_axi) then
            if rstn_axi = '0' then
                s_idle_gap_seen <= '0';
            elsif s_expect_nonidle = '1' and o_cdc_idle = '1' then
                s_idle_gap_seen <= '1';
            end if;
        end if;
    end process p_idle_gap_monitor;

    u_dut : entity work.tdc_gpx_csr_chip
        port map (
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
            i_axis_aclk         => clk_axis,
            i_axis_aresetn      => rstn_axis,
            o_cfg_image         => o_cfg_image,
            o_bus_clk_div       => o_bus_clk_div,
            o_bus_ticks         => o_bus_ticks,
            o_start_off1        => o_start_off1,
            o_cfg_reg7          => o_cfg_reg7,
            o_max_scan_5ns_ticks => o_max_scan_5ns_ticks,
            o_max_hits_cfg      => o_max_hits_cfg,
            o_falling_enable    => open,
            o_cmd_reg_read      => o_cmd_reg_read,
            o_cmd_reg_write     => o_cmd_reg_write,
            o_cmd_reg_addr      => o_cmd_reg_addr,
            o_cmd_reg_chip      => o_cmd_reg_chip,
            i_cmd_reg_rdata_0   => (others => '0'),
            i_cmd_reg_rdata_1   => (others => '0'),
            i_cmd_reg_rdata_2   => (others => '0'),
            i_cmd_reg_rdata_3   => (others => '0'),
            i_cmd_reg_rvalid    => (others => '0'),
            i_cmd_reg_done_pulse => '0',
            i_cmd_reg_addr_done  => (others => '0'),
            o_cmd_reg_chip_address => o_cmd_reg_chip_address,
            o_cdc_idle          => o_cdc_idle,
            o_irq               => o_irq
        );

    p_stim : process
        variable v_expected : t_cfg_image := c_GPX_DEFAULT_IMAGE;
        variable v_word     : std_logic_vector(31 downto 0);
        variable v_wait     : natural;

        procedure write_image(constant index : in natural;
                              constant value : in std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => std_logic_vector(to_unsigned(c_ADDR_CFG_IMAGE_BASE + 4 * index,
                                                           c_CSR_ADDR_WIDTH)),
                val         => value,
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
        end procedure;
    begin
        wait until rstn_axi = '1' and rstn_axis = '1';

        v_wait := 0;
        while o_cdc_idle /= '1' and v_wait < 200 loop
            wait until rising_edge(clk_axi);
            v_wait := v_wait + 1;
        end loop;
        assert o_cdc_idle = '1'
            report "Initial cfg_image CDC did not become idle"
            severity failure;
        assert o_cfg_image = v_expected
            report "Initial destination cfg_image does not match board-proven defaults"
            severity failure;
        assert o_cfg_image(11) = x"07FF0000"
           and o_cfg_image(12) = x"02000000"
           and o_cfg_image(14) = x"00000000"
            report "Mandatory Reg11/12/14 defaults did not cross the packed CSR CDC"
            severity failure;

        -- Back-to-back writes force several source updates into one in-flight
        -- packed transfer. The final destination image must contain all words.
        for i in 0 to c_CFG_IMAGE_N_REGS - 1 loop
            v_word := fn_image_word(1, i);
            v_expected(i) := v_word;
            write_image(i, v_word);
        end loop;

        v_wait := 0;
        while o_cfg_image /= v_expected and v_wait < 200 loop
            wait until rising_edge(clk_axis);
            v_wait := v_wait + 1;
        end loop;
        assert o_cfg_image = v_expected
            report "Burst cfg_image transfer did not converge"
            severity failure;

        v_wait := 0;
        while o_cdc_idle /= '1' and v_wait < 200 loop
            wait until rising_edge(clk_axi);
            v_wait := v_wait + 1;
        end loop;
        assert o_cdc_idle = '1'
            report "Burst cfg_image transfer did not return idle"
            severity failure;

        -- Start a transfer, then rewrite data while its round trip is active.
        -- Idle must remain low until the replay carrying the latest image has
        -- reached the AXIS domain.
        v_expected(0) := fn_image_word(2, 0);
        write_image(0, v_expected(0));
        v_wait := 0;
        while o_cdc_idle /= '0' and v_wait < 50 loop
            wait until rising_edge(clk_axi);
            v_wait := v_wait + 1;
        end loop;
        assert o_cdc_idle = '0'
            report "Replay scenario never entered busy state"
            severity failure;

        s_expect_nonidle <= '1';
        v_expected(1) := fn_image_word(2, 1);
        write_image(1, v_expected(1));
        v_expected(2) := fn_image_word(2, 2);
        write_image(2, v_expected(2));
        v_expected(0) := fn_image_word(3, 0);
        write_image(0, v_expected(0));

        v_wait := 0;
        while o_cfg_image /= v_expected and v_wait < 200 loop
            wait until rising_edge(clk_axis);
            v_wait := v_wait + 1;
        end loop;
        assert o_cfg_image = v_expected
            report "In-flight cfg_image rewrite lost the latest snapshot"
            severity failure;
        s_expect_nonidle <= '0';
        wait until rising_edge(clk_axi);
        assert s_idle_gap_seen = '0'
            report "o_cdc_idle rose before the latest cfg_image arrived"
            severity failure;

        v_wait := 0;
        while o_cdc_idle /= '1' and v_wait < 200 loop
            wait until rising_edge(clk_axi);
            v_wait := v_wait + 1;
        end loop;
        assert o_cdc_idle = '1'
            report "Replay scenario did not return idle"
            severity failure;

        report "CFG_IMAGE_PACKED_CDC ALL PASS" severity note;
        sim_done <= true;
        wait for 100 ns;
        std.env.finish;
        wait;
    end process p_stim;
end architecture sim;
