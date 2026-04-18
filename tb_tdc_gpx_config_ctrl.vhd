-- =============================================================================
-- tb_tdc_gpx_config_ctrl.vhd
-- Smoke-test testbench for Cluster 1 wrapper (tdc_gpx_config_ctrl)
-- =============================================================================
-- Verifies that the cluster instantiates cleanly and that a basic
-- cmd_start -> chip_busy -> init-complete sequence finishes without hanging.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_config_ctrl is
end entity tb_tdc_gpx_config_ctrl;

architecture sim of tb_tdc_gpx_config_ctrl is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_AXIS_CLK_PERIOD : time := 5 ns;      -- 200 MHz
    constant C_AXI_CLK_PERIOD  : time := 10 ns;     -- 100 MHz
    constant C_RESET_HOLD      : time := 100 ns;
    constant C_WATCHDOG        : time := 50 us;

    constant C_STOP_EVT_DWIDTH : natural := 32;

    -- Sensible pipeline config defaults (all fields initialised)
    constant C_CFG_PIPELINE : t_tdc_cfg := (
        active_chip_mask    => "1111",
        packet_scope        => '0',
        hit_store_mode      => "00",
        dist_scale          => "000",
        drain_mode          => '0',
        pipeline_en         => '0',
        n_faces             => to_unsigned(5, 3),
        stops_per_chip      => to_unsigned(8, 4),
        n_drain_cap         => (others => '0'),
        stopdis_override    => (others => '0'),
        bus_clk_div         => to_unsigned(2, 6),
        bus_ticks           => to_unsigned(5, 3),
        max_range_clks      => to_unsigned(267, 16),
        cols_per_face       => to_unsigned(2400, 16),
        start_off1          => (others => '0'),
        cfg_reg7            => (others => '0'),
        max_scan_clks       => to_unsigned(0, 16),
        max_hits_cfg        => to_unsigned(7, 3)
    );

    -- =========================================================================
    -- Clock / reset
    -- =========================================================================
    signal clk_axis   : std_logic := '0';
    signal clk_axi    : std_logic := '0';
    signal rstn_axis  : std_logic := '0';
    signal rstn_axi   : std_logic := '0';
    signal sim_done   : boolean   := false;

    -- =========================================================================
    -- AXI4-Lite (unused, tie to 0)
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
    -- TDC physical pins
    -- =========================================================================
    signal io_tdc_d       : t_tdc_bus_array;      -- inout, leave at 'Z'
    signal o_tdc_adr      : t_tdc_adr_array;
    signal o_tdc_csn      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_rdn      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_wrn      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_oen      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_stopdis  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_alutrigger : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_puresn   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal i_tdc_ef1      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');  -- empty = '1'
    signal i_tdc_ef2      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal i_tdc_lf1      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_lf2      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_irflag   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_errflag  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Stop event stream (unused)
    -- =========================================================================
    signal i_stop_evt_tvalid : std_logic := '0';
    signal i_stop_evt_tdata  : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0) := (others => '0');
    signal i_stop_evt_tkeep  : std_logic_vector(C_STOP_EVT_DWIDTH/8 - 1 downto 0) := (others => '0');
    signal i_stop_evt_tuser  : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0) := (others => '0');
    signal o_stop_evt_tready : std_logic;
    signal i_stop_tdc        : std_logic := '0';

    -- =========================================================================
    -- Control inputs
    -- =========================================================================
    signal i_cmd_start          : std_logic := '0';
    signal i_cmd_start_accepted : std_logic := '0';
    signal i_cmd_stop           : std_logic := '0';
    signal i_cmd_soft_reset     : std_logic := '0';
    signal i_cmd_cfg_write      : std_logic := '0';
    signal i_shot_start_per_chip : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_shot_start_gated   : std_logic := '0';

    -- Cluster 4 idle inputs
    signal i_face_asm_idle      : std_logic := '1';
    signal i_face_asm_fall_idle : std_logic := '1';
    signal i_hdr_idle           : std_logic := '1';
    signal i_hdr_fall_idle      : std_logic := '1';

    -- Frame boundary
    signal i_frame_done         : std_logic := '0';
    signal i_frame_fall_done    : std_logic := '0';

    -- Raw skid tready
    signal i_raw_sk_tready      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');

    -- =========================================================================
    -- DUT outputs
    -- =========================================================================
    signal o_raw_sk_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_raw_sk_tdata   : t_slv32_array;
    signal o_raw_sk_tuser   : t_slv8_array;
    signal o_cfg            : t_tdc_cfg;
    signal o_cfg_image      : t_cfg_image;
    signal o_cmd_start      : std_logic;
    signal o_cmd_cfg_write_g : std_logic;
    signal o_chip_busy      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_chip_shot_seq  : t_shot_seq_array;
    signal o_errflag_sync   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_err_drain_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_err_sequence   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_reg_outstanding : std_logic;
    signal o_reg_loop_resume : std_logic;
    signal o_cdc_idle       : std_logic;
    signal o_err_active     : std_logic;
    signal o_err_fatal      : std_logic;
    signal o_err_chip_mask  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_err_cause      : std_logic_vector(2 downto 0);
    signal o_irq            : std_logic;

begin

    -- =========================================================================
    -- Clock generation (stop when sim_done)
    -- =========================================================================
    clk_axis <= not clk_axis after C_AXIS_CLK_PERIOD / 2 when not sim_done else '0';
    clk_axi  <= not clk_axi  after C_AXI_CLK_PERIOD  / 2 when not sim_done else '0';

    -- =========================================================================
    -- Reset generation
    -- =========================================================================
    p_reset : process
    begin
        rstn_axis <= '0';
        rstn_axi  <= '0';
        wait for C_RESET_HOLD;
        wait until rising_edge(clk_axis);
        rstn_axis <= '1';
        wait until rising_edge(clk_axi);
        rstn_axi  <= '1';
        wait;
    end process p_reset;

    -- =========================================================================
    -- Inout bus: drive high-Z when not externally driven
    -- =========================================================================
    gen_tdc_d : for i in 0 to c_N_CHIPS - 1 generate
        io_tdc_d(i) <= (others => 'Z');
    end generate gen_tdc_d;

    -- =========================================================================
    -- DUT instantiation (default generics)
    -- =========================================================================
    u_dut : entity work.tdc_gpx_config_ctrl
        port map (
            -- Clock / Reset
            i_axis_aclk          => clk_axis,
            i_axis_aresetn       => rstn_axis,
            s_axi_aclk           => clk_axi,
            s_axi_aresetn        => rstn_axi,
            -- AXI4-Lite
            s_axi_awvalid        => s_axi_awvalid,
            s_axi_awready        => s_axi_awready,
            s_axi_awaddr         => s_axi_awaddr,
            s_axi_awprot         => s_axi_awprot,
            s_axi_wvalid         => s_axi_wvalid,
            s_axi_wready         => s_axi_wready,
            s_axi_wdata          => s_axi_wdata,
            s_axi_wstrb          => s_axi_wstrb,
            s_axi_bvalid         => s_axi_bvalid,
            s_axi_bready         => s_axi_bready,
            s_axi_bresp          => s_axi_bresp,
            s_axi_arvalid        => s_axi_arvalid,
            s_axi_arready        => s_axi_arready,
            s_axi_araddr         => s_axi_araddr,
            s_axi_arprot         => s_axi_arprot,
            s_axi_rvalid         => s_axi_rvalid,
            s_axi_rready         => s_axi_rready,
            s_axi_rdata          => s_axi_rdata,
            s_axi_rresp          => s_axi_rresp,
            -- TDC physical pins
            io_tdc_d             => io_tdc_d,
            o_tdc_adr            => o_tdc_adr,
            o_tdc_csn            => o_tdc_csn,
            o_tdc_rdn            => o_tdc_rdn,
            o_tdc_wrn            => o_tdc_wrn,
            o_tdc_oen            => o_tdc_oen,
            o_tdc_stopdis        => o_tdc_stopdis,
            o_tdc_alutrigger     => o_tdc_alutrigger,
            o_tdc_puresn         => o_tdc_puresn,
            i_tdc_ef1            => i_tdc_ef1,
            i_tdc_ef2            => i_tdc_ef2,
            i_tdc_lf1            => i_tdc_lf1,
            i_tdc_lf2            => i_tdc_lf2,
            i_tdc_irflag         => i_tdc_irflag,
            i_tdc_errflag        => i_tdc_errflag,
            -- Stop event stream
            i_stop_evt_tvalid    => i_stop_evt_tvalid,
            i_stop_evt_tdata     => i_stop_evt_tdata,
            i_stop_evt_tkeep     => i_stop_evt_tkeep,
            i_stop_evt_tuser     => i_stop_evt_tuser,
            o_stop_evt_tready    => o_stop_evt_tready,
            i_stop_tdc           => i_stop_tdc,
            -- Control inputs
            i_cmd_start          => i_cmd_start,
            i_cmd_start_accepted => i_cmd_start_accepted,
            i_cmd_stop           => i_cmd_stop,
            i_cmd_soft_reset     => i_cmd_soft_reset,
            i_cmd_cfg_write      => i_cmd_cfg_write,
            i_shot_start_per_chip => i_shot_start_per_chip,
            i_shot_start_gated   => i_shot_start_gated,
            i_cfg_pipeline       => C_CFG_PIPELINE,
            -- Cluster 4 idle
            i_face_asm_idle      => i_face_asm_idle,
            i_face_asm_fall_idle => i_face_asm_fall_idle,
            i_hdr_idle           => i_hdr_idle,
            i_hdr_fall_idle      => i_hdr_fall_idle,
            -- Frame boundary
            i_frame_done         => i_frame_done,
            i_frame_fall_done    => i_frame_fall_done,
            i_pipeline_abort     => '0',
            -- Raw skid output
            o_raw_sk_tvalid      => o_raw_sk_tvalid,
            o_raw_sk_tdata       => o_raw_sk_tdata,
            o_raw_sk_tuser       => o_raw_sk_tuser,
            i_raw_sk_tready      => i_raw_sk_tready,
            -- Config outputs
            o_cfg                => o_cfg,
            o_cfg_image          => o_cfg_image,
            -- Command outputs
            o_cmd_start          => o_cmd_start,
            o_cmd_cfg_write_g    => o_cmd_cfg_write_g,
            -- Chip status outputs
            o_chip_busy          => o_chip_busy,
            o_chip_shot_seq      => o_chip_shot_seq,
            o_errflag_sync       => o_errflag_sync,
            o_err_drain_timeout  => o_err_drain_timeout,
            o_err_sequence       => o_err_sequence,
            o_err_rsp_mismatch   => open,
            o_reg_outstanding    => o_reg_outstanding,
            o_reg_loop_resume    => o_reg_loop_resume,
            o_run_timeout        => open,
            o_reg_arb_timeout    => open,
            o_cdc_idle           => o_cdc_idle,
            -- Error handler outputs
            o_err_active         => o_err_active,
            o_err_fatal          => o_err_fatal,
            o_err_chip_mask      => o_err_chip_mask,
            o_err_cause          => o_err_cause,
            -- Interrupt
            o_irq                => o_irq
        );

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stimulus : process
    begin
        -- Wait for reset release + settling time
        wait until rstn_axis = '1' and rstn_axi = '1';
        wait for 200 ns;

        -- -------------------------------------------------------------------
        -- Step 1: Assert i_cmd_start for 1 axis clock cycle
        -- -------------------------------------------------------------------
        report "TB: Asserting cmd_start" severity note;
        wait until rising_edge(clk_axis);
        i_cmd_start <= '1';
        wait until rising_edge(clk_axis);
        i_cmd_start <= '0';

        -- -------------------------------------------------------------------
        -- Step 2: Wait for chip_busy to go all-zeros (init complete)
        --         The chip_ctrl FSMs should run through powerup and config
        --         sequence then return to idle.
        -- -------------------------------------------------------------------
        report "TB: Waiting for chip_busy = 0000 (init complete)" severity note;
        wait until o_chip_busy = "0000" for C_WATCHDOG;

        if o_chip_busy /= "0000" then
            report "TB: FAIL -- chip_busy did not clear within watchdog timeout"
                severity failure;
        end if;

        -- -------------------------------------------------------------------
        -- Step 3: Check that o_cfg.active_chip_mask is valid
        -- -------------------------------------------------------------------
        if o_cfg.active_chip_mask /= "0000" then
            report "TB: PASS -- init complete, active_chip_mask = " &
                   integer'image(to_integer(unsigned(o_cfg.active_chip_mask)))
                severity note;
        else
            report "TB: FAIL -- active_chip_mask is 0000 after init"
                severity failure;
        end if;

        -- Done
        report "TB: Smoke test completed successfully" severity note;
        sim_done <= true;
        wait;
    end process p_stimulus;

    -- =========================================================================
    -- Watchdog: hard timeout to catch infinite hangs
    -- =========================================================================
    p_watchdog : process
    begin
        wait for C_WATCHDOG;
        if not sim_done then
            report "TB: FAIL -- watchdog timeout (" &
                   time'image(C_WATCHDOG) & ") reached"
                severity failure;
        end if;
        wait;
    end process p_watchdog;

end architecture sim;
