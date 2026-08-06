library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_tdc_gpx_lidar_ctrl_v2_k03 is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_k03;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_k03 is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.output_width := 32;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_RUNTIME : lidar_runtime_config_t :=
        fn_default_runtime_config(C_BUILD_CONFIG);
    constant C_DERIVED : lidar_derived_config_t :=
        fn_derive_runtime_config(C_BUILD_CONFIG, C_RUNTIME);
    constant C_DEFAULT_WORDS : csr_word_array_t :=
        fn_pack_runtime_config(C_RUNTIME);
    constant C_CSR_PERIOD : time := 10 ns;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;
    constant C_ACTIVE_RISE_SLOTS : positive := fn_popcount(
        C_DERIVED.active_rise_mask) * C_BUILD_CONFIG.stops_per_chip;
    constant C_ACTIVE_FALL_SLOTS : positive := fn_popcount(
        C_DERIVED.active_fall_mask) * C_BUILD_CONFIG.stops_per_chip;
    constant C_RISE_STRIDE : positive := fn_gpx_vdma_stride_bytes(
        C_BUILD_CONFIG.num_chips * C_BUILD_CONFIG.stops_per_chip,
        C_BUILD_CONFIG.max_returns_per_stop, 32);
    constant C_FALL_STRIDE : positive := fn_gpx_vdma_stride_bytes(
        fn_popcount(C_BUILD_CONFIG.fall_capability_mask) *
            C_BUILD_CONFIG.stops_per_chip,
        C_BUILD_CONFIG.max_returns_per_stop, 32);

    signal csr_clk : std_logic := '0';
    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';

    signal awaddr : std_logic_vector(8 downto 0) := (others => '0');
    signal awprot : std_logic_vector(2 downto 0) := (others => '0');
    signal awvalid : std_logic := '0';
    signal awready : std_logic;
    signal wdata : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb : std_logic_vector(3 downto 0) := (others => '0');
    signal wvalid : std_logic := '0';
    signal wready : std_logic;
    signal bresp : std_logic_vector(1 downto 0);
    signal bvalid : std_logic;
    signal bready : std_logic := '0';
    signal araddr : std_logic_vector(8 downto 0) := (others => '0');
    signal arprot : std_logic_vector(2 downto 0) := (others => '0');
    signal arvalid : std_logic := '0';
    signal arready : std_logic;
    signal rdata : std_logic_vector(31 downto 0);
    signal rresp : std_logic_vector(1 downto 0);
    signal rvalid : std_logic;
    signal rready : std_logic := '0';
    signal irq : std_logic;

    signal fire_pulse : std_logic;
    signal start_tdc : std_logic;
    signal stop_tdc : std_logic;
    signal shot_start : std_logic;
    signal tdc_stop : std_logic_vector(31 downto 0);
    signal pd_p : std_logic_vector(31 downto 0) := (others => '0');
    signal pd_n : std_logic_vector(31 downto 0) := (others => '1');
    signal tdc_d : std_logic_vector(111 downto 0) := (others => 'Z');
    signal tdc_csn : std_logic_vector(3 downto 0);
    signal tdc_rdn : std_logic_vector(3 downto 0);
    signal tdc_wrn : std_logic_vector(3 downto 0);
    signal tdc_oen : std_logic_vector(3 downto 0);
    signal tdc_stopdis : std_logic_vector(3 downto 0);
    signal tdc_alutrigger : std_logic_vector(3 downto 0);
    signal tdc_puresn : std_logic_vector(3 downto 0);
    signal monitor_valid : std_logic;
    signal rise_axis_valid : std_logic;
    signal fall_axis_valid : std_logic;

    signal rise_cfg_valid : std_logic;
    signal rise_cfg_ready : std_logic := '0';
    signal rise_cfg_enable : std_logic;
    signal rise_hsize : unsigned(15 downto 0);
    signal rise_vsize : unsigned(15 downto 0);
    signal rise_stride : unsigned(15 downto 0);
    signal fall_cfg_valid : std_logic;
    signal fall_cfg_ready : std_logic := '0';
    signal fall_cfg_enable : std_logic;
    signal fall_hsize : unsigned(15 downto 0);
    signal fall_vsize : unsigned(15 downto 0);
    signal fall_stride : unsigned(15 downto 0);

begin

    csr_clk <= not csr_clk after C_CSR_PERIOD / 2;
    proc_clk <= not proc_clk after C_PROC_PERIOD / 2;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2;

    u_dut : entity work.tdc_gpx_lidar_ctrl_v2_top
        generic map (
            G_CSR_CLK_MHZ => 100,
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_STREAM_CLK_MODE => "ASYNC",
            G_OUTPUT_WIDTH => 32,
            G_PHASE_TIMEOUT_US => 10
        )
        port map (
            s_axi_csr_aclk => csr_clk,
            s_axi_csr_aresetn => csr_rst_n,
            proc_aclk => proc_clk,
            proc_aresetn => proc_rst_n,
            i_tdc_clk => tdc_clk,
            i_tdc_aresetn => tdc_rst_n,
            s_axi_csr_awaddr => awaddr,
            s_axi_csr_awprot => awprot,
            s_axi_csr_awvalid => awvalid,
            s_axi_csr_awready => awready,
            s_axi_csr_wdata => wdata,
            s_axi_csr_wstrb => wstrb,
            s_axi_csr_wvalid => wvalid,
            s_axi_csr_wready => wready,
            s_axi_csr_bresp => bresp,
            s_axi_csr_bvalid => bvalid,
            s_axi_csr_bready => bready,
            s_axi_csr_araddr => araddr,
            s_axi_csr_arprot => arprot,
            s_axi_csr_arvalid => arvalid,
            s_axi_csr_arready => arready,
            s_axi_csr_rdata => rdata,
            s_axi_csr_rresp => rresp,
            s_axi_csr_rvalid => rvalid,
            s_axi_csr_rready => rready,
            o_irq => irq,
            i_external_laser_permit => '0',
            o_fire_pulse => fire_pulse,
            o_start_tdc => start_tdc,
            o_stop_tdc => stop_tdc,
            o_shot_start => shot_start,
            o_shot_face_index => open,
            o_n_faces => open,
            i_pd_lvds_p => pd_p,
            i_pd_lvds_n => pd_n,
            o_tdc_stop => tdc_stop,
            io_tdc_d => tdc_d,
            o_tdc_adr => open,
            o_tdc_csn => tdc_csn,
            o_tdc_rdn => tdc_rdn,
            o_tdc_wrn => tdc_wrn,
            o_tdc_oen => tdc_oen,
            o_tdc_stopdis => tdc_stopdis,
            o_tdc_alutrigger => tdc_alutrigger,
            o_tdc_puresn => tdc_puresn,
            m_axis_monitor_tdata => open,
            m_axis_monitor_tkeep => open,
            m_axis_monitor_tuser => open,
            m_axis_monitor_tvalid => monitor_valid,
            m_axis_monitor_tlast => open,
            m_axis_monitor_tready => '1',
            m_axis_rise_tdata => open,
            m_axis_rise_tkeep => open,
            m_axis_rise_tstrb => open,
            m_axis_rise_tuser => open,
            m_axis_rise_tvalid => rise_axis_valid,
            m_axis_rise_tlast => open,
            m_axis_rise_tready => '1',
            m_axis_fall_tdata => open,
            m_axis_fall_tkeep => open,
            m_axis_fall_tstrb => open,
            m_axis_fall_tuser => open,
            m_axis_fall_tvalid => fall_axis_valid,
            m_axis_fall_tlast => open,
            m_axis_fall_tready => '1',
            o_vdma_rise_cfg_valid => rise_cfg_valid,
            i_vdma_rise_cfg_ready => rise_cfg_ready,
            o_vdma_rise_cfg_enable => rise_cfg_enable,
            o_vdma_rise_hsize_bytes => rise_hsize,
            o_vdma_rise_vsize_lines => rise_vsize,
            o_vdma_rise_stride_bytes => rise_stride,
            o_vdma_fall_cfg_valid => fall_cfg_valid,
            i_vdma_fall_cfg_ready => fall_cfg_ready,
            o_vdma_fall_cfg_enable => fall_cfg_enable,
            o_vdma_fall_hsize_bytes => fall_hsize,
            o_vdma_fall_vsize_lines => fall_vsize,
            o_vdma_fall_stride_bytes => fall_stride
        );

    p_fail_safe : process (proc_clk)
    begin
        if rising_edge(proc_clk) and proc_rst_n = '1' then
            assert fire_pulse = '0' and start_tdc = '0' and
                   stop_tdc = '0' and shot_start = '0' and
                   tdc_stop = x"00000000" and monitor_valid = '0' and
                   rise_axis_valid = '0' and fall_axis_valid = '0'
                report "V2-K0-TOP-TB inactive Processing/Echo output active"
                severity failure;
        end if;
    end process p_fail_safe;

    p_tdc_fail_safe : process (tdc_clk)
    begin
        if rising_edge(tdc_clk) and tdc_rst_n = '1' then
            assert tdc_csn = "1111" and tdc_rdn = "1111" and
                   tdc_wrn = "1111" and tdc_oen = "1111" and
                   tdc_stopdis = "1111" and tdc_alutrigger = "0000" and
                   tdc_puresn = "1111" and
                   tdc_d = (tdc_d'range => 'Z')
                report "V2-K0-TOP-TB pre-K0-5 GPX pins not fail-safe"
                severity failure;
        end if;
    end process p_tdc_fail_safe;

    p_test : process
        variable status_word : std_logic_vector(31 downto 0);
        variable profile_word : std_logic_vector(31 downto 0);
        variable expected_hsize : natural;
        variable expected_vsize : natural;

        procedure csr_tick is
        begin
            wait until rising_edge(csr_clk);
            wait for 1 ps;
        end procedure csr_tick;

        procedure axi_write(
            constant address : natural;
            constant value : std_logic_vector(31 downto 0)
        ) is
            variable aw_sent : boolean := false;
            variable w_sent : boolean := false;
        begin
            wait until falling_edge(csr_clk);
            awaddr <= std_logic_vector(to_unsigned(address, awaddr'length));
            awvalid <= '1';
            wdata <= value;
            wstrb <= "1111";
            wvalid <= '1';
            while not (aw_sent and w_sent) loop
                wait until rising_edge(csr_clk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_sent := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wvalid <= '0';
                    w_sent := true;
                end if;
            end loop;
            wstrb <= (others => '0');
            while bvalid = '0' loop
                csr_tick;
            end loop;
            assert bresp = "00"
                report "V2-K0-TOP-TB AXI write response"
                severity failure;
            bready <= '1';
            csr_tick;
            bready <= '0';
        end procedure axi_write;

        procedure axi_read(
            constant address : natural;
            variable value : out std_logic_vector(31 downto 0)
        ) is
        begin
            wait until falling_edge(csr_clk);
            araddr <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(csr_clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                csr_tick;
            end loop;
            assert rresp = "00"
                report "V2-K0-TOP-TB AXI read response"
                severity failure;
            value := rdata;
            rready <= '1';
            csr_tick;
            rready <= '0';
        end procedure axi_read;

        procedure command(constant bit_index : natural) is
            variable value : std_logic_vector(31 downto 0) :=
                (others => '0');
        begin
            value(bit_index) := '1';
            axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), value);
        end procedure command;

        procedure wait_profile_request is
        begin
            -- The approved sequential calculator takes about 1,118 CSR
            -- clocks for the default profile. Keep a bounded margin without
            -- turning configuration arithmetic into a combinational path.
            for cycle in 0 to 3000 loop
                wait until rising_edge(proc_clk);
                exit when rise_cfg_valid = '1' and fall_cfg_valid = '1';
            end loop;
            assert rise_cfg_valid = '1' and fall_cfg_valid = '1'
                report "V2-K0-TOP-TB VDMA profile request timeout"
                severity failure;
        end procedure wait_profile_request;

        procedure pulse_ready(signal value : out std_logic) is
        begin
            wait until falling_edge(proc_clk);
            value <= '1';
            wait until falling_edge(proc_clk);
            value <= '0';
        end procedure pulse_ready;

        procedure wait_commit_done(
            variable value : out std_logic_vector(31 downto 0)
        ) is
        begin
            for cycle in 0 to 500 loop
                axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), value);
                exit when value(C_TXN_BUSY_BIT) = '0' and
                    value(C_TXN_DONE_STICKY_BIT) = '1';
            end loop;
            assert value(C_TXN_BUSY_BIT) = '0' and
                   value(C_TXN_DONE_STICKY_BIT) = '1'
                report "V2-K0-TOP-TB commit completion timeout"
                severity failure;
        end procedure wait_commit_done;

        procedure wait_irq(constant expected : std_logic) is
        begin
            for cycle in 0 to 30 loop
                csr_tick;
                exit when irq = expected;
            end loop;
            assert irq = expected
                report "V2-K0-TOP-TB IRQ level timeout"
                severity failure;
        end procedure wait_irq;
    begin
        wait for 50 ns;
        csr_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        for cycle in 0 to 12 loop
            csr_tick;
        end loop;

        axi_write(fn_irq_byte_offset(0), x"00000001");
        command(C_CMD_COMMIT_BIT);
        wait_profile_request;

        expected_hsize := fn_gpx_vdma_shot_hsize_bytes(
            C_ACTIVE_RISE_SLOTS, 7, 32);
        expected_vsize := to_integer(C_DERIVED.columns_per_face) +
            fn_gpx_vdma_footer_lines(expected_hsize);
        assert rise_cfg_enable = '1' and fall_cfg_enable = '1' and
               to_integer(rise_hsize) = expected_hsize and
               to_integer(fall_hsize) = fn_gpx_vdma_shot_hsize_bytes(
                   C_ACTIVE_FALL_SLOTS, 7, 32) and
               to_integer(rise_vsize) = expected_vsize and
               to_integer(rise_stride) = C_RISE_STRIDE and
               to_integer(fall_stride) = C_FALL_STRIDE
            report "V2-K0-TOP-TB Return7 VDMA geometry mismatch"
            severity failure;

        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), status_word);
        assert status_word(C_TXN_BUSY_BIT) = '1' and
               status_word(C_TXN_ACTIVE_VALID_BIT) = '0'
            report "V2-K0-TOP-TB config completed before VDMA ACK"
            severity failure;

        pulse_ready(rise_cfg_ready);
        for cycle in 0 to 4 loop
            wait until rising_edge(proc_clk);
        end loop;
        assert rise_cfg_valid = '0' and fall_cfg_valid = '1'
            report "V2-K0-TOP-TB independent lane ACK was not preserved"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), status_word);
        assert status_word(C_TXN_BUSY_BIT) = '1' and
               status_word(C_TXN_ACTIVE_VALID_BIT) = '0'
            report "V2-K0-TOP-TB Rise-only ACK released config"
            severity failure;

        pulse_ready(fall_cfg_ready);
        wait_commit_done(status_word);
        assert status_word(8 downto 0) = "001000110"
            report "V2-K0-TOP-TB first transaction status mismatch"
            severity failure;
        wait_irq('1');
        axi_read(fn_irq_byte_offset(2), status_word);
        assert status_word = x"00000001"
            report "V2-K0-TOP-TB commit-success IRQ flag mismatch"
            severity failure;
        axi_write(fn_irq_byte_offset(2), x"00000001");
        wait_irq('0');

        for cycle in 0 to 8 loop
            csr_tick;
        end loop;
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        assert unsigned(status_word(15 downto 0)) = 1 and
               status_word(C_OP_CONFIG_READY_BIT) = '1'
            report "V2-K0-TOP-TB first Active version mismatch"
            severity failure;

        command(C_CMD_CLEAR_STATUS_BIT);
        for cycle in 0 to 30 loop
            csr_tick;
        end loop;
        profile_word := C_DEFAULT_WORDS(C_CTL_TDC_BUS_PROFILE);
        profile_word(19 downto 17) := "001";
        axi_write(fn_ctl_byte_offset(C_CTL_TDC_BUS_PROFILE), profile_word);
        command(C_CMD_COMMIT_BIT);
        wait_profile_request;

        expected_hsize := fn_gpx_vdma_shot_hsize_bytes(
            C_ACTIVE_RISE_SLOTS, 1, 32);
        assert to_integer(rise_hsize) = expected_hsize and
               to_integer(fall_hsize) = fn_gpx_vdma_shot_hsize_bytes(
                   C_ACTIVE_FALL_SLOTS, 1, 32) and
               to_integer(rise_stride) = C_RISE_STRIDE and
               to_integer(fall_stride) = C_FALL_STRIDE
            report "V2-K0-TOP-TB Return1 HSIZE/STRIDE contract mismatch"
            severity failure;

        wait until falling_edge(proc_clk);
        rise_cfg_ready <= '1';
        fall_cfg_ready <= '1';
        wait until falling_edge(proc_clk);
        rise_cfg_ready <= '0';
        fall_cfg_ready <= '0';
        wait_commit_done(status_word);
        assert status_word(8 downto 0) = "001000110"
            report "V2-K0-TOP-TB second transaction status mismatch"
            severity failure;
        wait_irq('1');
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        assert unsigned(status_word(15 downto 0)) = 2
            report "V2-K0-TOP-TB second Active version mismatch"
            severity failure;

        report "LIDAR_V2_TOP_K03_INTEGRATION_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        stop;
        wait;
    end process p_test;

end architecture sim;
