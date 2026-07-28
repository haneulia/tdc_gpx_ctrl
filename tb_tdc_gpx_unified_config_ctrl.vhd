-- =============================================================================
-- tb_tdc_gpx_unified_config_ctrl.vhd
-- Stage 5 focused bridge: unified adapter -> config_ctrl without local CSR
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_unified_config_ctrl is
end entity;

architecture sim of tb_tdc_gpx_unified_config_ctrl is
    constant C_CFG_PERIOD  : time := 10 ns;
    constant C_AXIS_PERIOD : time := 6666 ps;
    constant C_TDC_PERIOD  : time := 5 ns;

    signal s_done       : boolean := false;
    signal s_cfg_clk    : std_logic := '0';
    signal s_axis_clk   : std_logic := '0';
    signal s_tdc_clk    : std_logic := '0';
    signal s_cfg_rst_n  : std_logic := '0';
    signal s_axis_rst_n : std_logic := '0';

    signal s_sys_ctrl   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_sys_apply  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bus        : std_logic_vector(31 downto 0) := c_INIT_BUS_TIMING;
    signal s_start      : std_logic_vector(31 downto 0) := c_INIT_START_OFF1;
    signal s_reg7       : std_logic_vector(31 downto 0) := c_INIT_CFG_REG7;
    signal s_image_cmd  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_image_data : std_logic_vector(31 downto 0) := (others => '0');
    signal s_scan       : std_logic_vector(31 downto 0) := c_INIT_SCAN_TIMEOUT;
    signal s_main       : std_logic_vector(31 downto 0) := c_INIT_MAIN_CTRL;
    signal s_range      : std_logic_vector(31 downto 0) := c_INIT_RANGE_COLS;
    signal s_aux        : std_logic_vector(31 downto 0) := (others => '0');

    signal s_adapter_cfg   : t_tdc_cfg;
    signal s_adapter_image : t_cfg_image;
    signal s_cfg_out       : t_tdc_cfg;
    signal s_image_out     : t_cfg_image;
    signal s_status        : t_tdc_status := c_TDC_STATUS_INIT;

    signal s_cmd_start        : std_logic;
    signal s_cmd_stop         : std_logic;
    signal s_cmd_soft_reset   : std_logic;
    signal s_cmd_force_reinit : std_logic;
    signal s_err_soft_clear   : std_logic;
    signal s_cmd_cfg_write    : std_logic;
    signal s_cmd_reg_read     : std_logic;
    signal s_cmd_reg_write    : std_logic;
    signal s_cmd_reg_addr     : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip     : unsigned(1 downto 0);
    signal s_cmd_reg_mask     : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_cmd_ready        : std_logic;

    signal s_reg_rdata  : t_slv28_array := (others => (others => '0'));
    signal s_reg_rvalid : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_reg_done   : std_logic;
    signal s_reg_addr_done : std_logic_vector(3 downto 0);

    signal s_cfg_accepted   : std_logic_vector(7 downto 0);
    signal s_reset_accepted : std_logic_vector(7 downto 0);
    signal s_cfg_busy       : std_logic;
    signal s_cfg_reject     : std_logic;
    signal s_cfg_valid      : std_logic;
    signal s_cmd_accepted   : std_logic_vector(7 downto 0);
    signal s_cmd_busy       : std_logic;
    signal s_cmd_reject     : std_logic;
    signal s_image_accepted : std_logic_vector(7 downto 0);
    signal s_image_reject   : std_logic;
    signal s_image_selected : std_logic_vector(31 downto 0);
    signal s_irq_cause      : std_logic_vector(6 downto 0);
    signal s_irq_seen       : std_logic_vector(6 downto 0) := (others => '0');
    signal s_pipeline_status : std_logic_vector(31 downto 0);
    signal s_status_ext      : std_logic_vector(31 downto 0);
    signal s_status_ext2     : std_logic_vector(31 downto 0);

    signal s_axi_awready : std_logic;
    signal s_axi_wready  : std_logic;
    signal s_axi_bvalid  : std_logic;
    signal s_axi_bresp   : std_logic_vector(1 downto 0);
    signal s_axi_arready : std_logic;
    signal s_axi_rvalid  : std_logic;
    signal s_axi_rdata   : std_logic_vector(31 downto 0);
    signal s_axi_rresp   : std_logic_vector(1 downto 0);

    signal s_tdc_d : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
        (others => 'Z');
    signal s_tdc_adr : std_logic_vector(c_TDC_ADR_WIDTH - 1 downto 0);
    signal s_tdc_csn, s_tdc_rdn, s_tdc_wrn, s_tdc_oen : std_logic_vector(0 downto 0);
    signal s_tdc_stopdis, s_tdc_alutrigger, s_tdc_puresn : std_logic_vector(0 downto 0);

    signal s_raw_valid : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_raw_data  : t_raw_axis_tdata_array;
    signal s_raw_user  : t_raw_axis_tuser_array;
    signal s_cfg_write_count : natural := 0;

    function fn_image_cmd(index : natural; epoch : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(4 downto 0) := std_logic_vector(to_unsigned(index, 5));
        v_word(15 downto 8) := std_logic_vector(to_unsigned(epoch, 8));
        return v_word;
    end function;

    function fn_bus(clk_div : natural; ticks : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(5 downto 0) := std_logic_vector(to_unsigned(clk_div, 6));
        v_word(8 downto 6) := std_logic_vector(to_unsigned(ticks, 3));
        return v_word;
    end function;

    function fn_main(mask : natural; stops : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(3 downto 0) := std_logic_vector(to_unsigned(mask, 4));
        v_word(c_MC_PACKET_SCOPE) := '1';
        v_word(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO) := "10";
        v_word(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO) := "011";
        v_word(c_MC_DRAIN_MODE) := '1';
        v_word(c_MC_PIPELINE_EN) := '1';
        v_word(c_MC_STOPS_HI downto c_MC_STOPS_LO) :=
            std_logic_vector(to_unsigned(stops, 4));
        v_word(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO) := "0101";
        v_word(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO) := "10101";
        return v_word;
    end function;
begin
    p_cfg_clk : process
    begin
        while not s_done loop
            s_cfg_clk <= '0'; wait for C_CFG_PERIOD / 2;
            s_cfg_clk <= '1'; wait for C_CFG_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_axis_clk : process
    begin
        while not s_done loop
            s_axis_clk <= '0'; wait for C_AXIS_PERIOD / 2;
            s_axis_clk <= '1'; wait for C_AXIS_PERIOD / 2;
        end loop;
        wait;
    end process;

    p_tdc_clk : process
    begin
        while not s_done loop
            s_tdc_clk <= '0'; wait for C_TDC_PERIOD / 2;
            s_tdc_clk <= '1'; wait for C_TDC_PERIOD / 2;
        end loop;
        wait;
    end process;

    u_adapter : entity work.tdc_gpx_unified_csr_adapter
        generic map (
            g_PRESENT_CHIP_MASK => "0001",
            g_MAX_STOPS_PER_CHIP => 8,
            g_MAX_HITS_PER_STOP => 7,
            g_BUS_READ_PERIOD_MIN_CLKS => 5
        )
        port map (
            i_cfg_clk => s_cfg_clk,
            i_cfg_rst_n => s_cfg_rst_n,
            i_sys_ctrl => s_sys_ctrl,
            i_sys_cfg_apply => s_sys_apply,
            i_tdc_bus_timing => s_bus,
            i_tdc_start_offset => s_start,
            i_tdc_cfg_reg7 => s_reg7,
            i_tdc_image_cmd => s_image_cmd,
            i_tdc_image_data => s_image_data,
            i_tdc_scan_cfg => s_scan,
            i_tdc_pipeline_main => s_main,
            i_tdc_range_cols => s_range,
            i_tdc_aux_cmd => s_aux,
            i_n_faces => "100",
            i_axis_clk => s_axis_clk,
            i_axis_rst_n => s_axis_rst_n,
            i_cfg_ready => '1',
            i_cmd_ready => s_cmd_ready,
            o_cfg => s_adapter_cfg,
            o_cfg_image => s_adapter_image,
            o_cmd_start => s_cmd_start,
            o_cmd_stop => s_cmd_stop,
            o_cmd_soft_reset => s_cmd_soft_reset,
            o_cmd_force_reinit => s_cmd_force_reinit,
            o_err_soft_clear => s_err_soft_clear,
            o_cmd_cfg_write => s_cmd_cfg_write,
            o_cmd_reg_read => s_cmd_reg_read,
            o_cmd_reg_write => s_cmd_reg_write,
            o_cmd_reg_addr => s_cmd_reg_addr,
            o_cmd_reg_chip => s_cmd_reg_chip,
            o_cmd_reg_chip_address => s_cmd_reg_mask,
            i_cmd_reg_rdata_0 => s_reg_rdata(0),
            i_cmd_reg_rdata_1 => s_reg_rdata(1),
            i_cmd_reg_rdata_2 => s_reg_rdata(2),
            i_cmd_reg_rdata_3 => s_reg_rdata(3),
            i_cmd_reg_rvalid => s_reg_rvalid,
            i_cmd_reg_done_pulse => s_reg_done,
            i_cmd_reg_addr_done => s_reg_addr_done,
            i_status => s_status,
            o_chip0_result => open,
            o_chip1_result => open,
            o_chip2_result => open,
            o_chip3_result => open,
            o_pipeline_status => s_pipeline_status,
            o_status_ext => s_status_ext,
            o_status_ext2 => s_status_ext2,
            o_cfg_epoch_accepted => s_cfg_accepted,
            o_reset_epoch_accepted => s_reset_accepted,
            o_cfg_busy => s_cfg_busy,
            o_cfg_reject => s_cfg_reject,
            o_cfg_valid => s_cfg_valid,
            o_cmd_epoch_accepted => s_cmd_accepted,
            o_cmd_busy => s_cmd_busy,
            o_command_reject => s_cmd_reject,
            o_image_write_epoch_accepted => s_image_accepted,
            o_image_reject => s_image_reject,
            o_image_selected_data => s_image_selected,
            o_irq_cause => s_irq_cause
        );

    u_config_ctrl : entity work.tdc_gpx_config_ctrl
        generic map (
            g_ENABLE_LOCAL_CSR => false,
            g_AXIS_CLK_MHZ => 150,
            g_TDC_CLK_MHZ => 200,
            g_NUM_CHIPS => 1,
            g_PRESENT_CHIP_MASK => "0001",
            g_RISE_CHIP_MASK => "0001",
            g_FALL_CHIP_MASK => "0000",
            g_MAX_HITS_PER_STOP => 7,
            g_STREAM_CLK_MODE => "ASYNC"
        )
        port map (
            i_axis_aclk => s_axis_clk,
            i_axis_aresetn => s_axis_rst_n,
            i_tdc_clk => s_tdc_clk,
            s_axi_aclk => s_cfg_clk,
            s_axi_aresetn => s_cfg_rst_n,
            s_axi_awvalid => '0',
            s_axi_awready => s_axi_awready,
            s_axi_awaddr => (others => '0'),
            s_axi_awprot => (others => '0'),
            s_axi_wvalid => '0',
            s_axi_wready => s_axi_wready,
            s_axi_wdata => (others => '0'),
            s_axi_wstrb => (others => '0'),
            s_axi_bvalid => s_axi_bvalid,
            s_axi_bready => '0',
            s_axi_bresp => s_axi_bresp,
            s_axi_arvalid => '0',
            s_axi_arready => s_axi_arready,
            s_axi_araddr => (others => '0'),
            s_axi_arprot => (others => '0'),
            s_axi_rvalid => s_axi_rvalid,
            s_axi_rready => '0',
            s_axi_rdata => s_axi_rdata,
            s_axi_rresp => s_axi_rresp,
            i_ext_cfg => s_adapter_cfg,
            i_ext_cfg_image => s_adapter_image,
            i_ext_cmd_reg_read => s_cmd_reg_read,
            i_ext_cmd_reg_write => s_cmd_reg_write,
            i_ext_cmd_reg_addr => s_cmd_reg_addr,
            i_ext_cmd_reg_chip => s_cmd_reg_chip,
            i_ext_cmd_reg_chip_address => s_cmd_reg_mask,
            o_cmd_reg_rdata_0 => s_reg_rdata(0),
            o_cmd_reg_rdata_1 => s_reg_rdata(1),
            o_cmd_reg_rdata_2 => s_reg_rdata(2),
            o_cmd_reg_rdata_3 => s_reg_rdata(3),
            o_cmd_reg_rvalid => s_reg_rvalid,
            o_cmd_reg_done_pulse => s_reg_done,
            o_cmd_reg_addr_done => s_reg_addr_done,
            o_ext_cmd_ready => s_cmd_ready,
            io_tdc_d => s_tdc_d,
            o_tdc_adr => s_tdc_adr,
            o_tdc_csn => s_tdc_csn,
            o_tdc_rdn => s_tdc_rdn,
            o_tdc_wrn => s_tdc_wrn,
            o_tdc_oen => s_tdc_oen,
            o_tdc_stopdis => s_tdc_stopdis,
            o_tdc_alutrigger => s_tdc_alutrigger,
            o_tdc_puresn => s_tdc_puresn,
            i_tdc_ef1 => "1",
            i_tdc_ef2 => "1",
            i_tdc_lf1 => "0",
            i_tdc_lf2 => "0",
            i_tdc_irflag => "0",
            i_tdc_errflag => "0",
            i_stop_tdc => '0',
            i_cmd_start => s_cmd_start,
            i_cmd_start_accepted => s_cmd_start,
            i_cmd_stop => s_cmd_stop,
            i_cmd_soft_reset => s_cmd_soft_reset,
            i_cmd_force_reinit => s_cmd_force_reinit,
            i_cmd_cfg_write => s_cmd_cfg_write,
            i_err_soft_clear => s_err_soft_clear,
            i_shot_start_per_chip => (others => '0'),
            i_shot_start_gated => '0',
            i_cfg_pipeline => s_adapter_cfg,
            i_face_asm_idle => '1',
            i_face_asm_fall_idle => '1',
            i_hdr_idle => '1',
            i_hdr_fall_idle => '1',
            i_frame_done => '0',
            i_frame_fall_done => '0',
            i_pipeline_abort => '0',
            o_raw_sk_tvalid => s_raw_valid,
            o_raw_sk_tdata => s_raw_data,
            o_raw_sk_tuser => s_raw_user,
            i_raw_sk_tready => (others => '1'),
            o_cfg => s_cfg_out,
            o_cfg_image => s_image_out,
            o_cmd_start => open,
            o_cmd_cfg_write_g => open,
            o_chip_busy => open,
            o_chip_shot_seq => open,
            o_errflag_sync => open,
            o_err_drain_timeout => open,
            o_err_sequence => open,
            o_err_rsp_mismatch => open,
            o_err_raw_overflow => open,
            o_reg_outstanding => open,
            o_reg_loop_resume => open,
            o_cdc_idle => open,
            o_err_active => open,
            o_err_fatal => open,
            o_err_chip_mask => open,
            o_err_cause => open,
            o_run_timeout => open,
            o_reg_arb_timeout => open,
            o_irq => open
        );

    p_cfg_write_count : process(s_axis_clk)
    begin
        if rising_edge(s_axis_clk) then
            if s_axis_rst_n = '0' then
                s_cfg_write_count <= 0;
            elsif s_cmd_cfg_write = '1' then
                s_cfg_write_count <= s_cfg_write_count + 1;
            end if;
        end if;
    end process;

    p_irq_capture : process(s_cfg_clk)
    begin
        if rising_edge(s_cfg_clk) then
            if s_cfg_rst_n = '0' then
                s_irq_seen <= (others => '0');
            else
                s_irq_seen <= s_irq_seen or s_irq_cause;
            end if;
        end if;
    end process;

    p_stimulus : process
    begin
        wait for 50 ns;
        wait until falling_edge(s_cfg_clk);
        s_cfg_rst_n <= '1';
        wait until falling_edge(s_axis_clk);
        s_axis_rst_n <= '1';

        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cfg_valid = '1';
        end loop;
        assert s_cfg_valid = '1'
            report "unified config_ctrl bridge did not leave reset"
            severity failure;

        wait until falling_edge(s_cfg_clk);
        s_image_data <= x"12345678";
        -- Reg5 and Reg7 are controller-owned override words. Use Reg6 to
        -- prove that an ordinary staged image word survives both CDC stages.
        s_image_cmd <= fn_image_cmd(6, 1);
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_image_accepted = x"01";
        end loop;
        assert s_image_accepted = x"01" and s_image_reject = '0'
            and s_image_selected = x"12345678"
            report "unified image staging did not acknowledge"
            severity failure;

        wait until falling_edge(s_cfg_clk);
        s_bus   <= fn_bus(2, 5);
        s_start <= x"00012345";
        s_reg7  <= x"89ABCDEF";
        s_scan  <= x"00070064";
        s_main  <= fn_main(1, 8);
        s_range <= x"000201F4";
        s_sys_apply(7 downto 0) <= x"01";

        for i in 0 to 3000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cfg_accepted = x"01" and s_cfg_busy = '0';
        end loop;
        wait for 100 ns;

        assert s_cfg_accepted = x"01" and s_cfg_reject = '0'
            and s_cfg_write_count = 1
            report "unified configuration was not committed exactly once"
            severity failure;
        assert s_cfg_out.active_chip_mask = "0001"
            and s_cfg_out.bus_clk_div = 2
            and s_cfg_out.bus_ticks = 5
            and s_cfg_out.start_off1 = to_unsigned(16#12345#, 18)
            and s_cfg_out.cfg_reg7 = x"89ABCDEF"
            and s_cfg_out.max_scan_5ns_ticks = 100
            and s_cfg_out.max_hits_cfg = 7
            and s_cfg_out.falling_enable = '0'
            and s_cfg_out.stops_per_chip = 8
            and s_cfg_out.max_range_5ns_ticks = 500
            and s_cfg_out.cols_per_face = 2
            and s_cfg_out.n_faces = 4
            report "config_ctrl unified input selection changed configuration"
            severity failure;
        assert s_image_out(6) = x"12345678"
            report "config_ctrl unified image path changed the staged word"
            severity failure;
        assert s_image_out(7) = x"89ABCDEF"
            report "config_ctrl did not apply the CFG_REG7 ownership override"
            severity failure;
        assert s_axi_awready = '0' and s_axi_wready = '0'
            and s_axi_bvalid = '0' and s_axi_bresp = "10"
            and s_axi_arready = '0' and s_axi_rvalid = '0'
            and s_axi_rdata = x"00000000" and s_axi_rresp = "10"
            report "disabled local chip AXI interface is not quiescent"
            severity failure;

        -- Prove that the named unified STAT outputs carry the same packed
        -- diagnostic fields as the local pipeline CSR contract.
        wait until falling_edge(s_axis_clk);
        s_status.busy <= '1';
        s_status.pipeline_overrun <= '1';
        s_status.chip_error_mask <= "0101";
        s_status.drain_timeout_mask <= "0010";
        s_status.sequence_error_mask <= "1000";
        s_status.err_read_timeout <= '1';
        s_status.rise_shot_flush_drop <= '1';
        s_status.reg_timeout_mask <= "0011";
        s_status.masked_slope_drop_any <= '1';

        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_pipeline_status(c_STAT_BUSY) = '1'
                and s_status_ext(c_STAT6_ERR_READ_TIMEOUT) = '1'
                and s_status_ext2(c_STAT7_MASKED_SLOPE_DROP) = '1';
        end loop;

        assert s_pipeline_status(c_STAT_BUSY) = '1'
            and s_pipeline_status(c_STAT_OVERRUN) = '1'
            and s_pipeline_status(c_STAT_CHIP_ERR_HI downto c_STAT_CHIP_ERR_LO)
                = "0101"
            and s_pipeline_status(c_STAT_DRAIN_TO_HI downto c_STAT_DRAIN_TO_LO)
                = "0010"
            and s_pipeline_status(c_STAT_SEQ_ERR_HI downto c_STAT_SEQ_ERR_LO)
                = "1000"
            report "unified pipeline status packing mismatch"
            severity failure;
        assert s_status_ext(c_STAT6_ERR_READ_TIMEOUT) = '1'
            and s_status_ext(c_STAT6_SHOT_FLUSH_DROP_RISE) = '1'
            report "unified extended status packing mismatch"
            severity failure;
        assert s_status_ext2(c_STAT7_REG_TO_HI downto c_STAT7_REG_TO_LO)
                = "0011"
            and s_status_ext2(c_STAT7_MASKED_SLOPE_DROP) = '1'
            report "unified extended status-2 packing mismatch"
            severity failure;

        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_irq_seen(4 downto 1) = "1111";
        end loop;
        assert s_irq_seen(4 downto 1) = "1111"
            report "unified TDC diagnostic IRQ causes were not preserved"
            severity failure;

        report "TDC_GPX_UNIFIED_CONFIG_CTRL_PASS" severity note;
        s_done <= true;
        wait for C_CFG_PERIOD;
        stop;
        wait;
    end process;
end architecture;
