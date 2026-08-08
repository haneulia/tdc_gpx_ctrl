-- ============================================================================
-- 테스트 자산 목적: v2 Top의 K0-5/K0-6 경계와 K1-2 Reg7 적용 계약을 검증한다.
-- 핵심 검증 계약: GPX identity, Reg7 Shadow/Active/Physical, Rise/Fall lane,
-- Hole/T0/Footer와 32/64/128 폭 출력을 한 실제 Top 흐름에서 확인한다.
-- 관련 RTL: tdc_gpx_lidar_ctrl_v2_top과 acquisition/data/AXIS 전체 계층.
-- 실행 회귀: scripts/run_v2_k05_integration.ps1, run_v2_k06_axis_integration.ps1
-- 유지보수 주의: 이 파일의 K05/K06 top을 구분하고 routine 두 clock 관계를 모두 유지한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_status_pkg.all;
use work.tdc_gpx_cfg_pkg.all;
use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_lidar_ctrl_v2_axis_core is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_OUTPUT_WIDTH : positive := 32;
        G_AXIS_STALL_CLKS : natural := 0
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_axis_core;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_axis_core is

    constant C_CHIPS : positive := 2;
    constant C_STOPS_PER_CHIP : positive := 8;
    constant C_RETURNS_PER_STOP : positive := 7;
    constant C_WORDS_PER_IFIFO : positive :=
        (C_STOPS_PER_CHIP / 2) * C_RETURNS_PER_STOP;
    constant C_WORDS_PER_CHIP : positive := 2 * C_WORDS_PER_IFIFO;

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := C_CHIPS;
        result.stops_per_chip := C_STOPS_PER_CHIP;
        result.max_returns_per_stop := C_RETURNS_PER_STOP;
        result.rise_capability_mask := "0011";
        result.fall_capability_mask := "0000";
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := 1;
        result.enable_echo_receiver := true;
        result.enable_echo_simulation := true;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_TARGET_A_5NS : positive := 48;
    constant C_TARGET_B_5NS : positive := 53;

    -- Software가 CTL22 staging view에 쓴 MTimer는 의도적으로 틀리게
    -- 만든다. COMMIT 뒤에는 CTL12 파생값으로 바뀌고, HSDiv/RefClkDiv 등
    -- MTimer 외 비트는 이 후보값 그대로 보존되어야 한다.
    function fn_manual_reg7(
        manual_mtimer : natural;
        hsdiv_xor     : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t :=
            C_GPX_REGISTER_IMAGE_DEFAULT(7)(27 downto 0);
    begin
        result(c_REG7_MTIMER_HI downto c_REG7_MTIMER_LO) :=
            std_logic_vector(to_unsigned(
                manual_mtimer, C_GPX_MTIMER_WIDTH));
        result(c_REG7_HSDIV_HI downto c_REG7_HSDIV_LO) :=
            result(c_REG7_HSDIV_HI downto c_REG7_HSDIV_LO) xor
            std_logic_vector(to_unsigned(hsdiv_xor, 8));
        return result;
    end function fn_manual_reg7;

    constant C_STAGED_REG7_A : gpx_bus_data_t :=
        fn_manual_reg7(1, 1);
    constant C_STAGED_REG7_B : gpx_bus_data_t :=
        fn_manual_reg7(C_GPX_MTIMER_MAX, 2);

    function fn_effective_reg7(
        staged_value    : gpx_bus_data_t;
        target_range_5ns : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := staged_value;
        variable mtimer : u32_t;
    begin
        mtimer := fn_gpx_mtimer_ref_ticks(
            to_unsigned(target_range_5ns, 32));
        result(c_REG7_MTIMER_HI downto c_REG7_MTIMER_LO) :=
            std_logic_vector(mtimer(C_GPX_MTIMER_WIDTH - 1 downto 0));
        return result;
    end function fn_effective_reg7;

    function fn_gpx_ctl_word(
        value : gpx_bus_data_t
    ) return std_logic_vector is
        variable result : std_logic_vector(31 downto 0) := (others => '0');
    begin
        result(27 downto 0) := value;
        return result;
    end function fn_gpx_ctl_word;

    constant C_EFFECTIVE_REG7_A : gpx_bus_data_t :=
        fn_effective_reg7(C_STAGED_REG7_A, C_TARGET_A_5NS);
    constant C_EFFECTIVE_REG7_B : gpx_bus_data_t :=
        fn_effective_reg7(C_STAGED_REG7_B, C_TARGET_B_5NS);

    function fn_runtime_config return lidar_runtime_config_t is
        variable result : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
    begin
        result.motor.cpr := to_unsigned(8, 16);
        result.motor.decode_mode := DECODE_X4;
        result.motor.direction := DIRECTION_CW;
        result.motor.virtual_ticks_lo := to_unsigned(3, 32);
        result.motor.virtual_hi_count := (others => '0');
        result.motor.simulation_mode := '1';
        result.mirror.face_centers := (others => (others => '0'));
        result.mirror.face_centers(0) :=
            to_unsigned(7, C_POSITION_WIDTH);
        result.mirror.common_half_width :=
            to_unsigned(1, C_POSITION_WIDTH);
        result.laser.face_enable_mask := "00001";
        result.laser.fire_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.fire_done_timeout_5ns_ticks := to_unsigned(16, 16);
        result.laser.target_range_window_5ns :=
            to_unsigned(C_TARGET_A_5NS, 32);
        result.laser.start_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.stop_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.simulation_start_delay_5ns := to_unsigned(3, 32);
        result.laser.optical_shot_interval_udeg :=
            to_unsigned(45_000_000, angle_udeg_t'length);
        result.echo.channel_0_delay_5ns := to_unsigned(2, 16);
        result.echo.channel_step_5ns := to_unsigned(1, 16);
        result.tdc.active_chip_mask := "0011";
        result.tdc.falling_enable := '0';
        result.tdc.max_hits_per_stop := to_unsigned(7, 3);
        return result;
    end function fn_runtime_config;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(27 downto 0) := (others => '0');
        variable hit_value : natural;
    begin
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(word_index mod 4, 2));
        result(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) :=
            (others => '0');
        result(c_RAW_SLOPE_BIT) := '1';
        hit_value := chip_index * 16#2000# + word_index;
        if ififo_id = '1' then
            hit_value := hit_value + 16#0800#;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    function fn_contains_word(
        value       : std_logic_vector;
        target_word : std_logic_vector(31 downto 0)
    ) return boolean is
    begin
        for word_index in 0 to value'length / 32 - 1 loop
            if value((word_index + 1) * 32 - 1 downto
                     word_index * 32) = target_word then
                return true;
            end if;
        end loop;
        return false;
    end function fn_contains_word;

    constant C_RUNTIME : lidar_runtime_config_t := fn_runtime_config;
    constant C_DERIVED : lidar_derived_config_t :=
        fn_derive_runtime_config(C_BUILD_CONFIG, C_RUNTIME);
    constant C_WORDS : csr_word_array_t :=
        fn_pack_runtime_config(C_RUNTIME);
    constant C_RISE_SLOT_COUNT : positive :=
        C_CHIPS * C_STOPS_PER_CHIP;
    constant C_EXPECTED_HSIZE_BYTES : positive :=
        fn_gpx_vdma_shot_hsize_bytes(
            C_RISE_SLOT_COUNT, C_RETURNS_PER_STOP, G_OUTPUT_WIDTH);
    constant C_EXPECTED_LINES : positive :=
        to_integer(C_DERIVED.columns_per_face) +
        fn_gpx_vdma_footer_lines(C_EXPECTED_HSIZE_BYTES);
    constant C_EXPECTED_BEATS : positive := C_EXPECTED_LINES *
        fn_gpx_vdma_shot_line_beats(
            C_RISE_SLOT_COUNT, C_RETURNS_PER_STOP, G_OUTPUT_WIDTH);
    constant C_CSR_PERIOD : time := 10 ns;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    type natural_array_t is array (0 to C_CHIPS - 1) of
        natural range 0 to 65535;
    type logic_array_t is array (0 to C_CHIPS - 1) of std_logic;
    type physical_register_image_t is array (0 to 15) of gpx_bus_data_t;
    type chip_register_image_array_t is array (0 to C_CHIPS - 1) of
        physical_register_image_t;

    signal csr_clk : std_logic := '0';
    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';
    signal done : boolean := false;

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

    signal fire_pulse : std_logic;
    signal start_tdc : std_logic;
    signal stop_tdc : std_logic;
    signal shot_start : std_logic;
    signal shot_face_index : std_logic_vector(2 downto 0);
    signal pd_p : std_logic_vector(15 downto 0) := (others => '0');
    signal pd_n : std_logic_vector(15 downto 0) := (others => '1');
    signal tdc_stop : std_logic_vector(15 downto 0);

    signal tdc_d : std_logic_vector(C_CHIPS * 28 - 1 downto 0) :=
        (others => 'Z');
    signal tdc_adr : std_logic_vector(C_CHIPS * 4 - 1 downto 0);
    signal tdc_csn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_rdn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_wrn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_oen : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_stopdis : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_alutrigger : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_puresn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal ef1 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal ef2 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal lf1 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal lf2 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal irflag : std_logic_vector(C_CHIPS - 1 downto 0) :=
        (others => '0');

    signal chip_d_out : std_logic_vector(C_CHIPS * 28 - 1 downto 0) :=
        (others => '0');
    signal chip_d_oe : std_logic_vector(C_CHIPS - 1 downto 0) :=
        (others => '0');
    signal chip_register_image : chip_register_image_array_t :=
        (others => (others => (others => '0')));
    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal read_count : natural_array_t := (others => 0);
    signal write_count : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';

    signal rise_cfg_valid : std_logic;
    signal rise_cfg_ready : std_logic := '0';
    signal fall_cfg_valid : std_logic;
    signal fall_cfg_ready : std_logic := '0';
    signal rise_hsize_bytes : unsigned(15 downto 0);
    signal rise_vsize_lines : unsigned(15 downto 0);
    signal rise_stride_bytes : unsigned(15 downto 0);
    signal rise_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal rise_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tuser : std_logic_vector(0 downto 0);
    signal rise_tvalid : std_logic;
    signal rise_tlast : std_logic;
    signal rise_tready : std_logic := '1';
    signal fall_tvalid : std_logic;
    signal rise_beat_count : natural := 0;
    signal rise_line_count : natural := 0;
    signal rise_sof_count : natural := 0;
    signal rise_footer_magic_count : natural := 0;
    signal rise_footer_commit_count : natural := 0;
    signal rise_all_hole_footer_count : natural := 0;
    signal axis_stall_injected : std_logic := '0';
    signal previous_shot_r : std_logic := '0';
    signal shot_count : natural := 0;

begin

    csr_clk <= not csr_clk after C_CSR_PERIOD / 2 when not done;
    proc_clk <= not proc_clk after C_PROC_PERIOD / 2 when not done;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2 when not done;

    gen_no_axis_stall : if G_AXIS_STALL_CLKS = 0 generate
    begin
        rise_tready <= '1';
        axis_stall_injected <= '0';
    end generate gen_no_axis_stall;

    gen_axis_stall : if G_AXIS_STALL_CLKS > 0 generate
    begin
        p_axis_stall : process (proc_clk)
            variable remaining_v : natural := 0;
        begin
            if rising_edge(proc_clk) then
                if proc_rst_n = '0' then
                    rise_tready <= '1';
                    axis_stall_injected <= '0';
                    remaining_v := 0;
                elsif remaining_v > 0 then
                    rise_tready <= '0';
                    remaining_v := remaining_v - 1;
                    if remaining_v = 0 then
                        rise_tready <= '1';
                    end if;
                else
                    rise_tready <= '1';
                    if axis_stall_injected = '0' and
                       rise_tvalid = '1' and rise_tready = '1' and
                       fn_contains_word(
                           rise_tdata, C_GPX_VDMA_FOOTER_MAGIC) then
                        axis_stall_injected <= '1';
                        remaining_v := G_AXIS_STALL_CLKS;
                        rise_tready <= '0';
                    end if;
                end if;
            end if;
        end process p_axis_stall;
    end generate gen_axis_stall;

    gen_chip_pins : for index in 0 to C_CHIPS - 1 generate
        constant C_DATA_LO : natural := index * 28;
        constant C_DATA_HI : natural := (index + 1) * 28 - 1;
    begin
        ef1(index) <= '1' when fifo1_fill(index) = 0 else '0';
        ef2(index) <= '1' when fifo2_fill(index) = 0 else '0';
        lf1(index) <= '1' when fifo1_fill(index) >= 2 else '0';
        lf2(index) <= '1' when fifo2_fill(index) >= 2 else '0';
        tdc_d(C_DATA_HI downto C_DATA_LO) <=
            chip_d_out(C_DATA_HI downto C_DATA_LO)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_chip_pins;

    u_dut : entity work.tdc_gpx_lidar_ctrl_v2_top
        generic map (
            G_CSR_CLK_MHZ => 100,
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_STREAM_CLK_MODE => "ASYNC",
            G_NUM_CHIPS => C_CHIPS,
            G_STOPS_PER_CHIP => C_STOPS_PER_CHIP,
            G_MAX_RETURNS_PER_STOP => C_RETURNS_PER_STOP,
            G_RISE_CAPABILITY_MASK => "0011",
            G_FALL_CAPABILITY_MASK => "0000",
            G_OUTPUT_WIDTH => G_OUTPUT_WIDTH,
            G_NUM_FACES => 1,
            G_ENABLE_ECHO_RECEIVER => true,
            G_ENABLE_ECHO_SIMULATION => true,
            G_PHASE_TIMEOUT_US => 50,
            G_DRAIN_MARGIN_TIME_NS => 6000
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
            o_irq => open,
            i_external_laser_permit => '0',
            i_enc_a => '0',
            i_enc_b => '0',
            i_enc_z => '0',
            i_fire_done => '0',
            o_fire_pulse => fire_pulse,
            o_start_tdc => start_tdc,
            o_stop_tdc => stop_tdc,
            o_shot_start => shot_start,
            o_shot_face_index => shot_face_index,
            o_n_faces => open,
            i_pd_lvds_p => pd_p,
            i_pd_lvds_n => pd_n,
            o_tdc_stop => tdc_stop,
            io_tdc_d => tdc_d,
            o_tdc_adr => tdc_adr,
            o_tdc_csn => tdc_csn,
            o_tdc_rdn => tdc_rdn,
            o_tdc_wrn => tdc_wrn,
            o_tdc_oen => tdc_oen,
            o_tdc_stopdis => tdc_stopdis,
            o_tdc_alutrigger => tdc_alutrigger,
            o_tdc_puresn => tdc_puresn,
            i_tdc_ef1 => ef1,
            i_tdc_ef2 => ef2,
            i_tdc_lf1 => lf1,
            i_tdc_lf2 => lf2,
            i_tdc_irflag => irflag,
            i_tdc_errflag => (others => '0'),
            m_axis_monitor_tdata => open,
            m_axis_monitor_tkeep => open,
            m_axis_monitor_tuser => open,
            m_axis_monitor_tvalid => open,
            m_axis_monitor_tlast => open,
            m_axis_monitor_tready => '1',
            m_axis_rise_tdata => rise_tdata,
            m_axis_rise_tkeep => rise_tkeep,
            m_axis_rise_tstrb => rise_tstrb,
            m_axis_rise_tuser => rise_tuser,
            m_axis_rise_tvalid => rise_tvalid,
            m_axis_rise_tlast => rise_tlast,
            m_axis_rise_tready => rise_tready,
            m_axis_fall_tdata => open,
            m_axis_fall_tkeep => open,
            m_axis_fall_tstrb => open,
            m_axis_fall_tuser => open,
            m_axis_fall_tvalid => fall_tvalid,
            m_axis_fall_tlast => open,
            m_axis_fall_tready => '1',
            o_vdma_rise_cfg_valid => rise_cfg_valid,
            i_vdma_rise_cfg_ready => rise_cfg_ready,
            o_vdma_rise_cfg_enable => open,
            o_vdma_rise_hsize_bytes => rise_hsize_bytes,
            o_vdma_rise_vsize_lines => rise_vsize_lines,
            o_vdma_rise_stride_bytes => rise_stride_bytes,
            o_vdma_fall_cfg_valid => fall_cfg_valid,
            i_vdma_fall_cfg_ready => fall_cfg_ready,
            o_vdma_fall_cfg_enable => open,
            o_vdma_fall_hsize_bytes => open,
            o_vdma_fall_vsize_lines => open,
            o_vdma_fall_stride_bytes => open
        );

    p_chip_models : process (tdc_clk)
        variable rdn_previous : logic_array_t := (others => '1');
        variable wrn_previous : logic_array_t := (others => '1');
        variable address_v : std_logic_vector(3 downto 0);
        variable data_v : std_logic_vector(27 downto 0);
    begin
        if rising_edge(tdc_clk) then
            chip_d_oe <= (others => '0');
            if tdc_rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                read_count <= (others => 0);
                write_count <= (others => 0);
                chip_register_image <=
                    (others => (others => (others => '0')));
                rdn_previous := (others => '1');
                wrn_previous := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    address_v := tdc_adr(
                        (index + 1) * 4 - 1 downto index * 4);

                    if fifo_load = '1' then
                        fifo1_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo2_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo1_read_index(index) <= 0;
                        fifo2_read_index(index) <= 0;
                        read_count(index) <= 0;
                    end if;

                    if tdc_oen(index) = '0' and tdc_rdn(index) = '0' then
                        chip_d_oe(index) <= '1';
                        if address_v = c_TDC_REG8_IFIFO1 then
                            data_v := fn_raw_word(index, '0',
                                fifo1_read_index(index));
                        elsif address_v = c_TDC_REG9_IFIFO2 then
                            data_v := fn_raw_word(index, '1',
                                fifo2_read_index(index));
                        else
                            data_v := chip_register_image(index)(
                                to_integer(unsigned(address_v)));
                        end if;
                        chip_d_out((index + 1) * 28 - 1 downto index * 28)
                            <= data_v;
                    end if;

                    if tdc_rdn(index) = '1' and
                       rdn_previous(index) = '0' then
                        if address_v = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) > 0 then
                            fifo1_fill(index) <= fifo1_fill(index) - 1;
                            fifo1_read_index(index) <=
                                fifo1_read_index(index) + 1;
                            read_count(index) <= read_count(index) + 1;
                        elsif address_v = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) > 0 then
                            fifo2_fill(index) <= fifo2_fill(index) - 1;
                            fifo2_read_index(index) <=
                                fifo2_read_index(index) + 1;
                            read_count(index) <= read_count(index) + 1;
                        end if;
                    end if;

                    if tdc_wrn(index) = '0' and
                       wrn_previous(index) = '1' and
                       tdc_csn(index) = '0' then
                        assert not is_x(tdc_d(
                            (index + 1) * 28 - 1 downto index * 28))
                            report "V2-K12-TOP GPX write data contains X/Z"
                            severity failure;
                        chip_register_image(index)(
                            to_integer(unsigned(address_v))) <= tdc_d(
                                (index + 1) * 28 - 1 downto index * 28);
                        write_count(index) <= write_count(index) + 1;
                    end if;
                    rdn_previous(index) := tdc_rdn(index);
                    wrn_previous(index) := tdc_wrn(index);
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_shot_observer : process (proc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                previous_shot_r <= '0';
                shot_count <= 0;
            else
                if shot_start = '1' and previous_shot_r = '0' then
                    shot_count <= shot_count + 1;
                end if;
                previous_shot_r <= shot_start;
                assert fire_pulse = '0'
                    report "V2-K05-TOP simulation asserted physical fire"
                    severity failure;
            end if;
        end if;
    end process p_shot_observer;

    p_axis_observer : process (proc_clk)
        variable footer_active_v : boolean := false;
        variable footer_word_index_v : natural range 0 to
            C_GPX_VDMA_FOOTER_WORDS - 1 := 0;
        variable word_v : std_logic_vector(31 downto 0);
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                rise_beat_count <= 0;
                rise_line_count <= 0;
                rise_sof_count <= 0;
                rise_footer_magic_count <= 0;
                rise_footer_commit_count <= 0;
                rise_all_hole_footer_count <= 0;
                footer_active_v := false;
                footer_word_index_v := 0;
            elsif rise_tvalid = '1' and rise_tready = '1' then
                rise_beat_count <= rise_beat_count + 1;
                assert rise_tkeep = (rise_tkeep'range => '1') and
                       rise_tstrb = (rise_tstrb'range => '1')
                    report "V2-K05-TOP Rise byte qualifier mismatch"
                    severity failure;
                if rise_tlast = '1' then
                    rise_line_count <= rise_line_count + 1;
                end if;
                if rise_tuser(0) = '1' then
                    rise_sof_count <= rise_sof_count + 1;
                end if;
                if fn_contains_word(
                       rise_tdata, C_GPX_VDMA_FOOTER_MAGIC) then
                    rise_footer_magic_count <= rise_footer_magic_count + 1;
                end if;
                if fn_contains_word(
                       rise_tdata, C_GPX_VDMA_FOOTER_COMMIT) then
                    rise_footer_commit_count <= rise_footer_commit_count + 1;
                end if;
                for word_index in 0 to G_OUTPUT_WIDTH / 32 - 1 loop
                    word_v := rise_tdata(
                        (word_index + 1) * 32 - 1 downto word_index * 32);
                    if word_v = C_GPX_VDMA_FOOTER_MAGIC then
                        footer_active_v := true;
                        footer_word_index_v := 0;
                    end if;
                    if footer_active_v then
                        if footer_word_index_v = 6 then
                            if word_v(C_GPX_FOOTER_W6_ALL_HOLE) = '1' then
                                rise_all_hole_footer_count <=
                                    rise_all_hole_footer_count + 1;
                            end if;
                        end if;
                        if footer_word_index_v =
                           C_GPX_VDMA_FOOTER_WORDS - 1 then
                            footer_active_v := false;
                            footer_word_index_v := 0;
                        else
                            footer_word_index_v := footer_word_index_v + 1;
                        end if;
                    end if;
                end loop;
            end if;
            if proc_rst_n = '1' then
                assert fall_tvalid = '0'
                    report "V2-K05-TOP disabled Fall lane asserted TVALID"
                    severity failure;
            end if;
        end if;
    end process p_axis_observer;

    p_axis_hold_contract : process (proc_clk)
        variable held_v : boolean := false;
        variable held_data_v : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
        variable held_keep_v : std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0) := (others => '0');
        variable held_user_v : std_logic_vector(0 downto 0) :=
            (others => '0');
        variable held_last_v : std_logic := '0';
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                held_v := false;
            else
                if held_v then
                    assert rise_tvalid = '1' and
                           rise_tdata = held_data_v and
                           rise_tkeep = held_keep_v and
                           rise_tuser = held_user_v and
                           rise_tlast = held_last_v
                        report "V2-K06-TOP AXIS changed under backpressure"
                        severity failure;
                end if;
                held_v := rise_tvalid = '1' and rise_tready = '0';
                held_data_v := rise_tdata;
                held_keep_v := rise_tkeep;
                held_user_v := rise_tuser;
                held_last_v := rise_tlast;
                if rise_tvalid = '1' and rise_tready = '0' then
                    assert shot_count = 1
                        report "V2-K06-TOP next Shot preceded Footer acceptance"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_axis_hold_contract;

    p_test : process
        variable status_word : std_logic_vector(31 downto 0);
        variable diag_sequence : natural := 0;
        variable writes_before_invalid : natural_array_t := (others => 0);
        variable active_version_before_invalid : unsigned(15 downto 0) :=
            (others => '0');

        procedure wait_proc(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_proc;

        procedure wait_tdc(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(tdc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_tdc;

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
                report "V2-K05-TOP AXI write response"
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
                report "V2-K05-TOP AXI read response"
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

        procedure accept_vdma_profile(constant case_name : string) is
            variable accepted : boolean := false;
        begin
            for cycle in 0 to 10000 loop
                wait_proc(1);
                if rise_cfg_valid = '1' and fall_cfg_valid = '1' then
                    accepted := true;
                    exit;
                end if;
            end loop;
            assert accepted
                report case_name & ": VDMA profile request timeout"
                severity failure;
            rise_cfg_ready <= '1';
            fall_cfg_ready <= '1';
            wait_proc(1);
            rise_cfg_ready <= '0';
            fall_cfg_ready <= '0';
        end procedure accept_vdma_profile;

        procedure wait_transaction_done(constant case_name : string) is
            variable complete : boolean := false;
        begin
            for cycle in 0 to 2000 loop
                axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), status_word);
                if status_word(C_TXN_BUSY_BIT) = '0' and
                   status_word(C_TXN_DONE_STICKY_BIT) = '1' then
                    complete := true;
                    exit;
                end if;
            end loop;
            assert complete
                report case_name & ": transaction completion timeout"
                severity failure;
        end procedure wait_transaction_done;

        procedure stage_reg7(constant value : gpx_bus_data_t) is
        begin
            axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX),
                x"00000007");
            axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA),
                fn_gpx_ctl_word(value));
        end procedure stage_reg7;

        procedure check_reg7_view(
            constant active_view : boolean;
            constant expected    : gpx_bus_data_t;
            constant case_name   : string
        ) is
            variable index_word : std_logic_vector(31 downto 0) :=
                (others => '0');
            variable actual : std_logic_vector(31 downto 0);
        begin
            index_word(3 downto 0) := x"7";
            if active_view then
                index_word(C_GPX_IMAGE_VIEW_ACTIVE_BIT) := '1';
            end if;
            axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), index_word);
            axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), actual);
            assert actual = fn_gpx_ctl_word(expected)
                report case_name & ": Reg7 image view mismatch"
                severity failure;
        end procedure check_reg7_view;

        procedure capture_physical_register(
            constant chip_index : natural;
            constant address    : natural;
            constant expected   : gpx_bus_data_t;
            constant case_name  : string
        ) is
            variable diag_index : natural;
            variable control_expected : std_logic_vector(31 downto 0) :=
                (others => '0');
            variable actual : std_logic_vector(31 downto 0);
            variable complete : boolean := false;
        begin
            assert chip_index < C_CHIPS and address < 16
                report case_name & ": invalid physical register selection"
                severity failure;
            diag_index := C_DIAG_GPX_REGISTER_FIRST +
                chip_index * 16 + address;
            axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX),
                std_logic_vector(to_unsigned(16#100# + diag_index, 32)));
            for attempt in 0 to 400 loop
                axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), actual);
                if actual(C_DIAG_VALID_BIT) = '1' then
                    complete := true;
                    exit;
                end if;
            end loop;
            assert complete
                report case_name & ": physical GPX read timeout"
                severity failure;

            diag_sequence := diag_sequence + 1;
            control_expected(C_DIAG_INDEX_MSB downto C_DIAG_INDEX_LSB) :=
                std_logic_vector(to_unsigned(diag_index, 8));
            control_expected(C_DIAG_VALID_BIT) := '1';
            control_expected(C_DIAG_SEQUENCE_MSB downto
                C_DIAG_SEQUENCE_LSB) :=
                std_logic_vector(to_unsigned(diag_sequence, 16));
            assert actual = control_expected
                report case_name & ": diagnostic control mismatch"
                severity failure;
            axi_read(fn_ctl_byte_offset(C_CTL_DIAG_DATA), actual);
            assert actual = fn_pack_gpx_register_read_word(
                std_logic_vector(to_unsigned(address, 4)), expected)
                report case_name & ": physical GPX data mismatch"
                severity failure;
        end procedure capture_physical_register;

        procedure wait_operation_bit(
            constant bit_index : natural;
            constant expected : std_logic;
            constant case_name : string
        ) is
        begin
            for cycle in 0 to 200 loop
                axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION),
                    status_word);
                exit when status_word(bit_index) = expected;
            end loop;
            assert status_word(bit_index) = expected
                report case_name severity failure;
        end procedure wait_operation_bit;
    begin
        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "V2-K05-TOP invalid build"
            severity failure;
        assert fn_validate_runtime_config(C_BUILD_CONFIG, C_RUNTIME) = CFG_OK
            report "V2-K05-TOP invalid runtime"
            severity failure;
        assert C_DERIVED.columns_per_face = 1
            report "V2-K05-TOP expected one planned Shot per Face"
            severity failure;

        wait for 50 ns;
        csr_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait_proc(8);

        for index in 1 to C_CTL_ECHO_DELAY_PROFILE loop
            axi_write(fn_ctl_byte_offset(index), C_WORDS(index));
        end loop;

        -- K1-2 A: staging에는 software가 쓴 수동 MTimer가 그대로 보여야
        -- 한다. 아직 성공 COMMIT이 없어 ACTIVE_VALID=0이므로 Active view는
        -- 내부 reset image를 노출하지 않고 0을 반환해야 한다.
        stage_reg7(C_STAGED_REG7_A);
        check_reg7_view(false, C_STAGED_REG7_A,
            "V2-K12 initial staging");
        check_reg7_view(true, gpx_bus_data_t'(others => '0'),
            "V2-K12 reset active");

        -- 첫 COMMIT이 진행 중일 때 TARGET_RANGE와 Reg7 staging을 B 후보로
        -- 바꾼다. 진행 transaction은 A snapshot을 끝까지 사용하고 B는
        -- 다음 COMMIT 소유가 되어야 한다.
        command(C_CMD_COMMIT_BIT);
        for cycle in 0 to 200 loop
            axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), status_word);
            exit when status_word(C_TXN_BUSY_BIT) = '1';
        end loop;
        assert status_word(C_TXN_BUSY_BIT) = '1'
            report "V2-K12 first COMMIT never entered BUSY"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_TARGET_RANGE),
            std_logic_vector(to_unsigned(C_TARGET_B_5NS, 32)));
        stage_reg7(C_STAGED_REG7_B);
        accept_vdma_profile("V2-K12 first COMMIT");
        wait_transaction_done("V2-K12 first COMMIT");
        assert status_word(C_TXN_SUCCESS_STICKY_BIT) = '1' and
               status_word(C_TXN_ACTIVE_VALID_BIT) = '1' and
               status_word(C_TXN_SHADOW_DIRTY_BIT) = '1'
            report "V2-K12 first snapshot/dirty contract failed"
            severity failure;
        axi_read(fn_ctl_byte_offset(C_CTL_TARGET_RANGE), status_word);
        assert unsigned(status_word) = C_TARGET_B_5NS
            report "V2-K12 next Shadow target was not preserved"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_SOURCE_BASE +
            C_CTL_TARGET_RANGE - 1), status_word);
        assert unsigned(status_word) = C_TARGET_A_5NS
            report "V2-K12 in-flight Shadow edit polluted Active source"
            severity failure;
        check_reg7_view(false, C_STAGED_REG7_B,
            "V2-K12 next staging after first COMMIT");
        check_reg7_view(true, C_EFFECTIVE_REG7_A,
            "V2-K12 first Active image");
        for index in 0 to C_CHIPS - 1 loop
            assert write_count(index) > 0
                report "V2-K05-TOP missing physical GPX configuration writes"
                severity failure;
            capture_physical_register(index, 7, C_EFFECTIVE_REG7_A,
                "V2-K12 first physical Reg7 Chip " & integer'image(index));
        end loop;

        -- K1-2 B: 다음 COMMIT은 B Shadow를 원자 적용한다. staging에는
        -- 수동 MTimer가 남고 Active/두 물리 Chip에는 ceil(53/5)=11만 보인다.
        command(C_CMD_CLEAR_STATUS_BIT);
        command(C_CMD_COMMIT_BIT);
        accept_vdma_profile("V2-K12 second COMMIT");
        wait_transaction_done("V2-K12 second COMMIT");
        assert status_word(C_TXN_SUCCESS_STICKY_BIT) = '1' and
               status_word(C_TXN_SHADOW_DIRTY_BIT) = '0'
            report "V2-K12 second COMMIT did not consume the Shadow revision"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_SOURCE_BASE +
            C_CTL_TARGET_RANGE - 1), status_word);
        assert unsigned(status_word) = C_TARGET_B_5NS
            report "V2-K12 second Active source target mismatch"
            severity failure;
        check_reg7_view(false, C_STAGED_REG7_B,
            "V2-K12 second staging");
        check_reg7_view(true, C_EFFECTIVE_REG7_B,
            "V2-K12 second Active image");
        for index in 0 to C_CHIPS - 1 loop
            capture_physical_register(index, 7, C_EFFECTIVE_REG7_B,
                "V2-K12 second physical Reg7 Chip " & integer'image(index));
        end loop;

        -- K1-2 C: 13-bit MTimer 상한을 한 tick 넘는 요청은 0x33으로
        -- 거절되고 Active image와 실제 Chip을 절대로 바꾸지 않아야 한다.
        writes_before_invalid := write_count;
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        active_version_before_invalid := unsigned(status_word(15 downto 0));
        command(C_CMD_CLEAR_STATUS_BIT);
        axi_write(fn_ctl_byte_offset(C_CTL_TARGET_RANGE),
            std_logic_vector(to_unsigned(
                C_GPX_MTIMER_MAX * C_GPX_REFERENCE_TICK_5NS + 1, 32)));
        command(C_CMD_COMMIT_BIT);
        wait_transaction_done("V2-K12 invalid MTimer COMMIT");
        assert status_word(C_TXN_ERROR_STICKY_BIT) = '1' and
               status_word(23 downto 16) =
                   fn_cfg_error_code(CFG_RUNTIME_GPX_MTIMER_RANGE) and
               status_word(C_TXN_SHADOW_DIRTY_BIT) = '1'
            report "V2-K12 MTimer overflow error contract mismatch"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        assert unsigned(status_word(15 downto 0)) =
                   active_version_before_invalid
            report "V2-K12 failed COMMIT changed Active version"
            severity failure;
        check_reg7_view(true, C_EFFECTIVE_REG7_B,
            "V2-K12 failed-COMMIT Active image");
        for index in 0 to C_CHIPS - 1 loop
            assert write_count(index) = writes_before_invalid(index)
                report "V2-K12 failed COMMIT reached physical GPX Chip"
                severity failure;
            capture_physical_register(index, 7, C_EFFECTIVE_REG7_B,
                "V2-K12 preserved physical Reg7 Chip " & integer'image(index));
        end loop;

        -- 잘못된 Shadow를 원래 B 값으로 복구하고 성공 COMMIT하여 이후
        -- acquisition이 clean Shadow/Active 상태에서 시작하도록 한다.
        command(C_CMD_CLEAR_STATUS_BIT);
        axi_write(fn_ctl_byte_offset(C_CTL_TARGET_RANGE),
            std_logic_vector(to_unsigned(C_TARGET_B_5NS, 32)));
        command(C_CMD_COMMIT_BIT);
        accept_vdma_profile("V2-K12 recovery COMMIT");
        wait_transaction_done("V2-K12 recovery COMMIT");
        assert status_word(C_TXN_SUCCESS_STICKY_BIT) = '1' and
               status_word(C_TXN_SHADOW_DIRTY_BIT) = '0'
            report "V2-K12 recovery COMMIT failed"
            severity failure;
        check_reg7_view(true, C_EFFECTIVE_REG7_B,
            "V2-K12 recovered Active image");
        for index in 0 to C_CHIPS - 1 loop
            capture_physical_register(index, 7, C_EFFECTIVE_REG7_B,
                "V2-K12 recovered physical Reg7 Chip " & integer'image(index));
        end loop;

        fifo_load <= '1';
        wait_tdc(1);
        fifo_load <= '0';
        wait_tdc(4);

        command(C_CMD_RUN_BIT);
        wait_operation_bit(C_OP_RUNNING_BIT, '1',
            "V2-K05-TOP RUN timeout");
        command(C_CMD_ARM_BIT);
        wait_operation_bit(C_OP_SIMULATION_ENABLE_BIT, '1',
            "V2-K05-TOP ARM timeout");

        for cycle in 0 to 10000 loop
            wait_proc(1);
            exit when shot_count >= 1 and start_tdc = '1';
        end loop;
        assert shot_count >= 1 and shot_face_index = "000"
            report "V2-K05-TOP first Shot missing"
            severity failure;

        for cycle in 0 to 10000 loop
            wait_proc(1);
            exit when stop_tdc = '1';
        end loop;
        assert stop_tdc = '1'
            report "V2-K05-TOP target-range STOP missing"
            severity failure;

        wait until falling_edge(tdc_clk);
        irflag <= (others => '1');
        wait_tdc(6);
        irflag <= (others => '0');

        for cycle in 0 to 500000 loop
            wait_proc(1);
            exit when read_count(0) = C_WORDS_PER_CHIP and
                read_count(1) = C_WORDS_PER_CHIP and
                fifo1_fill(0) = 0 and fifo2_fill(0) = 0 and
                fifo1_fill(1) = 0 and fifo2_fill(1) = 0;
        end loop;
        for index in 0 to C_CHIPS - 1 loop
            assert read_count(index) = C_WORDS_PER_CHIP and
                   fifo1_fill(index) = 0 and fifo2_fill(index) = 0
                report "V2-K05-TOP IFIFO drain count mismatch"
                severity failure;
        end loop;

        -- A second revolution can produce a Shot only after B5-B8 completes
        -- the first Shot and acknowledges the held Face-close event.
        for cycle in 0 to 500000 loop
            wait_proc(1);
            exit when shot_count >= 2;
        end loop;
        assert shot_count >= 2
            report "V2-K05-TOP B5-B8/Face-close chain did not reopen; " &
                "rise_beats=" & natural'image(rise_beat_count) &
                " rise_lines=" & natural'image(rise_line_count) &
                " rise_sof=" & natural'image(rise_sof_count) &
                " footer_magic=" &
                    natural'image(rise_footer_magic_count) &
                " footer_commit=" &
                    natural'image(rise_footer_commit_count) &
                " all_hole=" &
                    natural'image(rise_all_hole_footer_count) &
                " rise_tvalid=" & std_logic'image(rise_tvalid) &
                " fall_tvalid=" & std_logic'image(fall_tvalid) &
                " hsize=" & integer'image(to_integer(rise_hsize_bytes)) &
                " vsize=" & integer'image(to_integer(rise_vsize_lines)) &
                " stride=" & integer'image(to_integer(rise_stride_bytes))
            severity failure;

        assert rise_beat_count = C_EXPECTED_BEATS and
               rise_line_count = C_EXPECTED_LINES and
               rise_sof_count = 1 and
               rise_footer_magic_count = 1 and
               rise_footer_commit_count = 1 and
               rise_all_hole_footer_count = 0
            report "V2-K06-TOP AXIS frame contract mismatch; " &
                "width=" & positive'image(G_OUTPUT_WIDTH) &
                " beats=" & natural'image(rise_beat_count) &
                "/" & positive'image(C_EXPECTED_BEATS) &
                " lines=" & natural'image(rise_line_count) &
                "/" & positive'image(C_EXPECTED_LINES) &
                " sof=" & natural'image(rise_sof_count) &
                " magic=" & natural'image(rise_footer_magic_count) &
                " commit=" & natural'image(rise_footer_commit_count) &
                " all_hole=" & natural'image(
                    rise_all_hole_footer_count)
            severity failure;
        if G_AXIS_STALL_CLKS > 0 then
            assert axis_stall_injected = '1'
                report "V2-K06-TOP requested Footer stall was not injected"
                severity failure;
        end if;

        report "LIDAR_V2_TOP_K05_GPX_B5_B8_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        report "LIDAR_V2_K12_REG7_SHADOW_ACTIVE_PHYSICAL_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        report "LIDAR_V2_TOP_K06_AXIS_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) & " output_width=" &
            positive'image(G_OUTPUT_WIDTH) & " stall_clks=" &
            natural'image(G_AXIS_STALL_CLKS) severity note;
        done <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;

entity tb_tdc_gpx_lidar_ctrl_v2_k05 is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_k05;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_k05 is
begin
    u_core : entity work.tb_tdc_gpx_lidar_ctrl_v2_axis_core
        generic map (
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_OUTPUT_WIDTH => 32,
            G_AXIS_STALL_CLKS => 0
        );
end architecture sim;

entity tb_tdc_gpx_lidar_ctrl_v2_k06 is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_OUTPUT_WIDTH : positive := 32;
        G_AXIS_STALL_CLKS : natural := 13
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_k06;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_k06 is
begin
    u_core : entity work.tb_tdc_gpx_lidar_ctrl_v2_axis_core
        generic map (
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_OUTPUT_WIDTH => G_OUTPUT_WIDTH,
            G_AXIS_STALL_CLKS => G_AXIS_STALL_CLKS
        );
end architecture sim;
