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
    generic (
        -- Selects the DUT raw stream clock-domain mode for this run.
        -- "SYNC"  : raw stream passes through directly (i_tdc_clk == i_axis_aclk).
        -- "ASYNC" : raw stream goes through xpm_fifo_async (CDC).
        g_DUT_STREAM_CLK_MODE : string := "SYNC"
    );
end entity tb_tdc_gpx_config_ctrl;

architecture sim of tb_tdc_gpx_config_ctrl is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_AXIS_CLK_PERIOD : time := 5 ns;      -- 200 MHz
    constant C_AXI_CLK_PERIOD  : time := 10 ns;     -- 100 MHz
    constant C_RESET_HOLD      : time := 100 ns;
    constant C_WATCHDOG        : time := 50 us;
    constant C_TIMEOUT_CLKS    : natural := 4000;

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
        max_range_5ns_ticks => to_unsigned(267, 16),
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

    -- 4-chip behavioral model state
    type t_fill_array is array (0 to c_N_CHIPS - 1) of natural;
    signal fifo1_fill   : t_fill_array := (others => 0);
    signal fifo2_fill   : t_fill_array := (others => 0);
    signal fifo1_rd_cnt : t_fill_array := (others => 0);
    signal fifo2_rd_cnt : t_fill_array := (others => 0);
    signal fifo_load_req : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal fifo_load_n1  : t_fill_array := (others => 0);
    signal fifo_load_n2  : t_fill_array := (others => 0);
    type t_chip_d_array is array (0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal chip_d_out : t_chip_d_array := (others => (others => '0'));
    signal chip_d_oe  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Stop event stream / fire-count contract input
    -- =========================================================================
    signal i_stop_evt_tvalid : std_logic := '0';
    signal i_stop_evt_tdata  : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0) := (others => '0');
    signal i_stop_evt_tkeep  : std_logic_vector(C_STOP_EVT_DWIDTH/8 - 1 downto 0) := (others => '0');
    signal i_stop_evt_tuser  : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0) := (others => '0');
    signal o_stop_evt_tready : std_logic;
    signal i_fire_count_tvalid : std_logic := '0';
    signal i_fire_count_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal i_fire_count_tkeep  : std_logic_vector(3 downto 0) := (others => '0');
    signal i_fire_count_tlast  : std_logic := '0';
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
    signal i_current_fire_count : unsigned(15 downto 0) := (others => '0');

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
    signal o_raw_sk_tdata   : t_raw_axis_tdata_array;
    signal o_raw_sk_tuser   : t_raw_axis_tuser_array;
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
    signal mon_raw_data_cnt : natural := 0;
    signal mon_raw_ctrl_cnt : natural := 0;

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
    -- 4-chip virtual TDC-GPX model
    -- =========================================================================
    gen_flags : for i in 0 to c_N_CHIPS - 1 generate
        i_tdc_ef1(i) <= '1' when fifo1_fill(i) = 0 else '0';
        i_tdc_ef2(i) <= '1' when fifo2_fill(i) = 0 else '0';
        i_tdc_lf1(i) <= '1' when fifo1_fill(i) >= 4 else '0';
        i_tdc_lf2(i) <= '1' when fifo2_fill(i) >= 4 else '0';
    end generate gen_flags;

    gen_chip_model : for i in 0 to c_N_CHIPS - 1 generate
        p_chip_model : process(clk_axis)
            variable v_rdn_prev  : std_logic := '1';
            variable v_load_prev : std_logic := '0';
            variable v_my_fill1  : natural   := 0;
            variable v_my_fill2  : natural   := 0;
            variable v_my_rd1    : natural   := 0;
            variable v_my_rd2    : natural   := 0;
        begin
            if rising_edge(clk_axis) then
                if rstn_axis = '0' then
                    v_my_fill1 := 0;
                    v_my_fill2 := 0;
                    v_my_rd1   := 0;
                    v_my_rd2   := 0;
                    chip_d_oe(i)  <= '0';
                    chip_d_out(i) <= (others => '0');
                else
                    chip_d_oe(i) <= '0';

                    if fifo_load_req(i) = '1' and v_load_prev = '0' then
                        v_my_fill1 := fifo_load_n1(i);
                        v_my_fill2 := fifo_load_n2(i);
                        v_my_rd1   := 0;
                        v_my_rd2   := 0;
                    end if;
                    v_load_prev := fifo_load_req(i);

                    if o_tdc_oen(i) = '0' and o_tdc_rdn(i) = '0'
                       and o_tdc_csn(i) = '0' then
                        chip_d_oe(i) <= '1';
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(i) <= "00" & x"00" & '0' &
                                std_logic_vector(to_unsigned((i * 256) + v_my_rd1 + 1,
                                                             c_RAW_HIT_WIDTH));
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(i) <= "00" & x"00" & '0' &
                                std_logic_vector(to_unsigned((i * 256) + 128 + v_my_rd2 + 1,
                                                             c_RAW_HIT_WIDTH));
                        else
                            chip_d_out(i) <= (others => '0');
                        end if;
                    end if;

                    if o_tdc_rdn(i) = '0' and v_rdn_prev = '1' then
                        assert not (o_tdc_adr(i) = c_TDC_REG8_IFIFO1 and v_my_fill1 = 0)
                            report "tb_tdc_gpx_config_ctrl: EMPTY IFIFO1 read attempted"
                            severity failure;
                        assert not (o_tdc_adr(i) = c_TDC_REG9_IFIFO2 and v_my_fill2 = 0)
                            report "tb_tdc_gpx_config_ctrl: EMPTY IFIFO2 read attempted"
                            severity failure;
                    end if;

                    if o_tdc_rdn(i) = '1' and v_rdn_prev = '0' then
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 and v_my_fill1 > 0 then
                            v_my_fill1 := v_my_fill1 - 1;
                            v_my_rd1   := v_my_rd1 + 1;
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 and v_my_fill2 > 0 then
                            v_my_fill2 := v_my_fill2 - 1;
                            v_my_rd2   := v_my_rd2 + 1;
                        end if;
                    end if;
                    v_rdn_prev := o_tdc_rdn(i);

                    fifo1_fill(i)   <= v_my_fill1;
                    fifo2_fill(i)   <= v_my_fill2;
                    fifo1_rd_cnt(i) <= v_my_rd1;
                    fifo2_rd_cnt(i) <= v_my_rd2;
                end if;
            end if;
        end process p_chip_model;

        io_tdc_d(i) <= chip_d_out(i) when chip_d_oe(i) = '1'
                                     else (others => 'Z');
    end generate gen_chip_model;

    p_raw_monitor : process(clk_axis)
        variable v_data_inc : natural;
        variable v_ctrl_inc : natural;
    begin
        if rising_edge(clk_axis) then
            if rstn_axis = '0' then
                mon_raw_data_cnt <= 0;
                mon_raw_ctrl_cnt <= 0;
            else
                v_data_inc := 0;
                v_ctrl_inc := 0;
                for i in 0 to c_N_CHIPS - 1 loop
                    if o_raw_sk_tvalid(i) = '1' and i_raw_sk_tready(i) = '1' then
                        if o_raw_sk_tuser(i)(7) = '1' then
                            v_ctrl_inc := v_ctrl_inc + 1;
                        else
                            v_data_inc := v_data_inc + 1;
                        end if;
                    end if;
                end loop;
                mon_raw_data_cnt <= mon_raw_data_cnt + v_data_inc;
                mon_raw_ctrl_cnt <= mon_raw_ctrl_cnt + v_ctrl_inc;
            end if;
        end if;
    end process p_raw_monitor;

    -- =========================================================================
    -- DUT instantiation (default generics)
    -- =========================================================================
    u_dut : entity work.tdc_gpx_config_ctrl
        generic map (
            g_AXIS_CLK_MHZ    => 200,
            g_TDC_CLK_MHZ     => 200,
            g_STREAM_CLK_MODE => g_DUT_STREAM_CLK_MODE
        )
        port map (
            -- Clock / Reset
            i_axis_aclk          => clk_axis,
            i_axis_aresetn       => rstn_axis,
            i_tdc_clk            => clk_axis,  -- same clock for single-clock TB
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
            i_fire_count_tvalid  => i_fire_count_tvalid,
            i_fire_count_tdata   => i_fire_count_tdata,
            i_fire_count_tkeep   => i_fire_count_tkeep,
            i_fire_count_tlast   => i_fire_count_tlast,
            i_stop_tdc           => i_stop_tdc,
            -- Control inputs
            i_cmd_start          => i_cmd_start,
            i_cmd_start_accepted => i_cmd_start_accepted,
            i_cmd_stop           => i_cmd_stop,
            i_cmd_soft_reset     => i_cmd_soft_reset,
            i_cmd_cfg_write      => i_cmd_cfg_write,
            i_shot_start_per_chip => i_shot_start_per_chip,
            i_shot_start_gated   => i_shot_start_gated,
            i_current_fire_count => i_current_fire_count,
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
            o_err_raw_overflow   => open,
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
        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk_axis);
            end loop;
        end procedure;

        procedure load_all_fifos(n1 : natural; n2 : natural) is
        begin
            for i in 0 to c_N_CHIPS - 1 loop
                fifo_load_n1(i) <= n1;
                fifo_load_n2(i) <= n2;
            end loop;
            wait until rising_edge(clk_axis);
            fifo_load_req <= (others => '1');
            wait until rising_edge(clk_axis);
            fifo_load_req <= (others => '0');
            wait until rising_edge(clk_axis);
        end procedure;

        procedure emit_expected_counts(shot_count : natural;
                                       ififo1_cnt : natural;
                                       ififo2_cnt : natural) is
            variable v_data : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0);
            variable v_user : std_logic_vector(C_STOP_EVT_DWIDTH - 1 downto 0);
            variable v_lo   : natural;
        begin
            assert ififo1_cnt <= 15 and ififo2_cnt <= 15
                report "TB: expected-count field overflow"
                severity failure;

            v_data := (others => '0');
            v_user := (others => '0');
            for i in 0 to c_N_CHIPS - 1 loop
                v_lo := i * 8;
                v_data(v_lo + 3 downto v_lo) :=
                    std_logic_vector(to_unsigned(ififo1_cnt, 4));
                v_data(v_lo + 7 downto v_lo + 4) :=
                    std_logic_vector(to_unsigned(ififo2_cnt, 4));
            end loop;

            wait until rising_edge(clk_axis);
            i_stop_evt_tdata <= v_data;
            i_stop_evt_tuser <= v_user;
            i_stop_evt_tkeep <= (others => '1');
            i_stop_evt_tvalid <= '1';
            i_fire_count_tdata <= std_logic_vector(to_unsigned(shot_count, 32));
            i_fire_count_tkeep <= (others => '1');
            i_fire_count_tlast <= '0';
            i_fire_count_tvalid <= '1';

            wait until rising_edge(clk_axis);
            i_stop_evt_tvalid <= '0';
            i_stop_evt_tdata <= (others => '0');
            i_stop_evt_tuser <= (others => '0');
            i_stop_evt_tkeep <= (others => '0');
            i_fire_count_tvalid <= '0';

            wait until rising_edge(clk_axis);
            i_fire_count_tdata <= std_logic_vector(to_unsigned(shot_count, 32));
            i_fire_count_tkeep <= (others => '1');
            i_fire_count_tlast <= '1';
            i_fire_count_tvalid <= '1';

            wait until rising_edge(clk_axis);
            i_fire_count_tvalid <= '0';
            i_fire_count_tlast <= '0';
            i_fire_count_tkeep <= (others => '0');
            i_fire_count_tdata <= (others => '0');
        end procedure;

        constant c_TEST_IFIFO_WORDS : natural := 2;
        constant c_EXPECTED_DATA_WORDS : natural :=
            c_N_CHIPS * c_TEST_IFIFO_WORDS * 2;
        variable v_found : boolean;
    begin
        -- Wait for reset release + settling time
        wait until rstn_axis = '1' and rstn_axi = '1';
        -- chip_ctrl starts in PH_INIT after reset. Let the automatic
        -- powerup/config/master-reset sequence finish before issuing START.
        wait_clk(3000);

        report "TB: g_DUT_STREAM_CLK_MODE = " & g_DUT_STREAM_CLK_MODE
            severity note;
        report "TB: OP-C02-03 expected-count CDC/top integration scenario"
            severity note;

        -- Arm chip_run through the same accepted-start pulse that face_seq
        -- produces in tdc_gpx_top.
        wait until rising_edge(clk_axis);
        i_cmd_start <= '1';
        i_cmd_start_accepted <= '1';
        wait until rising_edge(clk_axis);
        i_cmd_start <= '0';
        i_cmd_start_accepted <= '0';
        wait until o_tdc_stopdis = "0000" for 5 us;
        if o_tdc_stopdis /= "0000" then
            report "TB: FAIL -- chip_run did not enter ARMED after cmd_start_accepted"
                severity failure;
        end if;
        wait_clk(20);

        -- One I-Mode single shot. stop_cfg_decode owns counts by matching this
        -- face-local fire count against the fire_count stream.
        i_current_fire_count <= to_unsigned(1, 16);
        wait until rising_edge(clk_axis);
        i_shot_start_gated <= '1';
        i_shot_start_per_chip <= (others => '1');
        wait_clk(4);
        i_shot_start_gated <= '0';
        i_shot_start_per_chip <= (others => '0');

        wait until o_chip_busy = "1111" for 2 us;
        if o_chip_busy /= "1111" then
            report "TB: FAIL -- chip_run did not enter CAPTURE after shot_start"
                severity failure;
        end if;

        wait_clk(8);
        emit_expected_counts(1, c_TEST_IFIFO_WORDS, c_TEST_IFIFO_WORDS);
        wait_clk(80);

        load_all_fifos(c_TEST_IFIFO_WORDS, c_TEST_IFIFO_WORDS);
        wait_clk(10);
        i_tdc_irflag <= (others => '1');

        v_found := false;
        for i in 0 to C_TIMEOUT_CLKS loop
            wait_clk(1);
            if mon_raw_data_cnt >= c_EXPECTED_DATA_WORDS then
                v_found := true;
                exit;
            end if;
        end loop;

        if not v_found then
            report "TB: FAIL -- expected-count drain did not emit all data words, got "
                   & integer'image(mon_raw_data_cnt)
                severity failure;
        end if;

        wait_clk(20);
        i_tdc_irflag <= (others => '0');

        for i in 0 to c_N_CHIPS - 1 loop
            assert fifo1_rd_cnt(i) = c_TEST_IFIFO_WORDS
                report "TB: IFIFO1 read count mismatch on chip " & integer'image(i)
                       & " actual=" & integer'image(fifo1_rd_cnt(i))
                severity failure;
            assert fifo2_rd_cnt(i) = c_TEST_IFIFO_WORDS
                report "TB: IFIFO2 read count mismatch on chip " & integer'image(i)
                       & " actual=" & integer'image(fifo2_rd_cnt(i))
                severity failure;
        end loop;

        if mon_raw_ctrl_cnt < c_N_CHIPS * 2 then
            report "TB: FAIL -- expected drain control beats missing, got "
                   & integer'image(mon_raw_ctrl_cnt)
                severity failure;
        end if;

        if o_cfg.active_chip_mask /= "1111" then
            report "TB: FAIL -- active_chip_mask did not propagate"
                severity failure;
        end if;

        report "TB: PASS -- expected-count tuple CDC/top integration completed, data="
               & integer'image(mon_raw_data_cnt)
               & " ctrl=" & integer'image(mon_raw_ctrl_cnt)
            severity note;

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
