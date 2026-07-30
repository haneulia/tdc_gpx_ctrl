-- =============================================================================
-- tdc_gpx_unified_csr_adapter.vhd
-- TDC-GPX adapter for the 32-CTL / 32-STAT unified LiDAR CSR bank
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library xpm;
use xpm.vcomponents.all;

library work;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_unified_csr_adapter is
    generic (
        g_PRESENT_CHIP_MASK : std_logic_vector(c_MAX_CHIPS - 1 downto 0) :=
            c_ALL_CHIPS_MASK;
        g_MAX_STOPS_PER_CHIP : positive range 2 to c_MAX_STOPS_PER_CHIP :=
            c_MAX_STOPS_PER_CHIP;
        g_MAX_HITS_PER_STOP : positive range 1 to c_MAX_HITS_PER_STOP :=
            c_MAX_HITS_PER_STOP;
        g_BUS_READ_PERIOD_MIN_CLKS : positive :=
            c_DEFAULT_BUS_READ_PERIOD_MIN_CLKS
    );
    port (
        -- Unified CSR domain.
        i_cfg_clk               : in  std_logic;
        i_cfg_rst_n             : in  std_logic;
        i_sys_ctrl              : in  std_logic_vector(31 downto 0);
        i_sys_cfg_apply         : in  std_logic_vector(31 downto 0);
        i_tdc_bus_timing        : in  std_logic_vector(31 downto 0);
        i_tdc_start_offset      : in  std_logic_vector(31 downto 0);
        i_tdc_cfg_reg7          : in  std_logic_vector(31 downto 0);
        i_tdc_image_cmd         : in  std_logic_vector(31 downto 0);
        i_tdc_image_data        : in  std_logic_vector(31 downto 0);
        i_tdc_scan_cfg          : in  std_logic_vector(31 downto 0);
        i_tdc_pipeline_main     : in  std_logic_vector(31 downto 0);
        i_tdc_range_cols       : in  std_logic_vector(31 downto 0);
        i_tdc_aux_cmd           : in  std_logic_vector(31 downto 0);
        -- Build-time Motor geometry sideband. It is static for one bitstream.
        i_n_faces               : in  std_logic_vector(2 downto 0);

        -- AXIS processing domain. Configuration is forwarded from here to
        -- config_ctrl, which retains the existing AXIS-to-TDC CDC boundary.
        i_axis_clk              : in  std_logic;
        i_axis_rst_n            : in  std_logic;
        i_cfg_ready             : in  std_logic;
        i_cmd_ready             : in  std_logic;
        o_cfg                   : out t_tdc_cfg;
        o_cfg_image             : out t_cfg_image;
        o_cmd_start             : out std_logic;
        o_cmd_stop              : out std_logic;
        o_cmd_soft_reset        : out std_logic;
        o_cmd_force_reinit      : out std_logic;
        o_err_soft_clear        : out std_logic;
        o_cmd_cfg_write         : out std_logic;
        o_cmd_reg_read          : out std_logic;
        o_cmd_reg_write         : out std_logic;
        o_cmd_reg_addr          : out std_logic_vector(3 downto 0);
        o_cmd_reg_chip          : out unsigned(1 downto 0);
        o_cmd_reg_chip_address  : out std_logic_vector(c_MAX_CHIPS - 1 downto 0);

        -- Raw GPX register responses. The adapter preserves the local CSR's
        -- address+28-bit-data latching contract.
        i_cmd_reg_rdata_0       : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_1       : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_2       : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_3       : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rvalid        : in  std_logic_vector(c_MAX_CHIPS - 1 downto 0);
        i_cmd_reg_done_pulse    : in  std_logic;
        i_cmd_reg_addr_done     : in  std_logic_vector(3 downto 0);
        i_status                : in  t_tdc_status;

        -- Unified STAT23..STAT29.
        o_chip0_result          : out std_logic_vector(31 downto 0);
        o_chip1_result          : out std_logic_vector(31 downto 0);
        o_chip2_result          : out std_logic_vector(31 downto 0);
        o_chip3_result          : out std_logic_vector(31 downto 0);
        o_pipeline_status       : out std_logic_vector(31 downto 0);
        o_status_ext            : out std_logic_vector(31 downto 0);
        o_status_ext2           : out std_logic_vector(31 downto 0);

        -- Transaction sideband for the System status aggregator.
        o_cfg_epoch_accepted    : out std_logic_vector(7 downto 0);
        o_reset_epoch_accepted  : out std_logic_vector(7 downto 0);
        o_cfg_busy              : out std_logic;
        o_cfg_reject            : out std_logic;
        o_cfg_valid             : out std_logic;
        o_cmd_epoch_accepted    : out std_logic_vector(7 downto 0);
        o_cmd_busy              : out std_logic;
        o_command_reject        : out std_logic;
        o_image_write_epoch_accepted : out std_logic_vector(7 downto 0);
        o_image_reject          : out std_logic;
        o_image_selected_data   : out std_logic_vector(31 downto 0);

        -- Local source order maps to unified IRQ[21..27].
        o_irq_cause             : out std_logic_vector(6 downto 0)
    );
end entity tdc_gpx_unified_csr_adapter;

architecture rtl of tdc_gpx_unified_csr_adapter is
    constant C_CFG_WIDTH     : positive := 755;
    constant C_STATUS_WIDTH  : positive := 256;
    constant C_RESET_EPOCH_LO : natural := 0;
    constant C_CFG_EPOCH_LO   : natural := 8;
    constant C_BUS_LO         : natural := 16;
    constant C_START_LO       : natural := 48;
    constant C_REG7_LO        : natural := 80;
    constant C_SCAN_LO        : natural := 112;
    constant C_MAIN_LO        : natural := 144;
    constant C_RANGE_LO       : natural := 176;
    constant C_AUX_LO         : natural := 208;
    constant C_N_FACES_LO     : natural := 240;
    constant C_IMAGE_LO       : natural := 243;

    type t_result_array is array(0 to c_MAX_CHIPS - 1) of
        std_logic_vector(31 downto 0);

    function fn_any(value : std_logic_vector) return std_logic is
        variable v_any : std_logic := '0';
    begin
        for i in value'range loop
            v_any := v_any or value(i);
        end loop;
        return v_any;
    end function fn_any;

    function fn_sanitize_n_faces(value : std_logic_vector(2 downto 0))
        return unsigned is
    begin
        case value is
            when "001" | "010" | "011" | "100" | "101" =>
                return unsigned(value);
            when others =>
                return to_unsigned(1, 3);
        end case;
    end function fn_sanitize_n_faces;

    function fn_decode_cfg(
        bus_word   : std_logic_vector(31 downto 0);
        start_word : std_logic_vector(31 downto 0);
        reg7_word  : std_logic_vector(31 downto 0);
        scan_word  : std_logic_vector(31 downto 0);
        main_word  : std_logic_vector(31 downto 0);
        range_word : std_logic_vector(31 downto 0);
        n_faces    : std_logic_vector(2 downto 0)
    ) return t_tdc_cfg is
        variable v_cfg       : t_tdc_cfg := c_TDC_CFG_INIT;
        variable v_mask      : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
        variable v_div       : unsigned(5 downto 0);
        variable v_ticks     : unsigned(2 downto 0);
        variable v_ticks_min : unsigned(2 downto 0);
        variable v_stops     : unsigned(3 downto 0);
    begin
        v_mask := main_word(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO)
            and g_PRESENT_CHIP_MASK;
        if v_mask = (v_mask'range => '0') then
            v_mask := fn_first_one_mask(g_PRESENT_CHIP_MASK);
        end if;
        v_cfg.active_chip_mask := v_mask;
        v_cfg.packet_scope := main_word(c_MC_PACKET_SCOPE);
        v_cfg.hit_store_mode := unsigned(
            main_word(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO));
        v_cfg.dist_scale := unsigned(
            main_word(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO));
        v_cfg.drain_mode := main_word(c_MC_DRAIN_MODE);
        v_cfg.pipeline_en := main_word(c_MC_PIPELINE_EN);
        v_cfg.n_faces := fn_sanitize_n_faces(n_faces);

        v_stops := unsigned(main_word(c_MC_STOPS_HI downto c_MC_STOPS_LO));
        if v_stops < to_unsigned(2, v_stops'length) then
            v_cfg.stops_per_chip := to_unsigned(2, 4);
        elsif v_stops > to_unsigned(g_MAX_STOPS_PER_CHIP, v_stops'length) then
            v_cfg.stops_per_chip := to_unsigned(g_MAX_STOPS_PER_CHIP, 4);
        else
            v_cfg.stops_per_chip := v_stops;
        end if;
        v_cfg.n_drain_cap := unsigned(
            main_word(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO));
        v_cfg.stopdis_override :=
            main_word(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO);

        v_div := unsigned(bus_word(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO));
        if v_div < to_unsigned(c_BUS_CLK_DIV_MIN, v_div'length) then
            v_div := to_unsigned(c_BUS_CLK_DIV_MIN, v_div'length);
        end if;
        if v_div = to_unsigned(1, v_div'length) then
            v_ticks_min := to_unsigned(g_BUS_READ_PERIOD_MIN_CLKS, 3);
        else
            v_ticks_min := to_unsigned(c_BUS_TICKS_MIN, 3);
        end if;
        v_ticks := unsigned(bus_word(c_BT_TICKS_HI downto c_BT_TICKS_LO));
        if v_ticks < v_ticks_min then
            v_ticks := v_ticks_min;
        end if;
        v_cfg.bus_clk_div := v_div;
        v_cfg.bus_ticks := v_ticks;

        v_cfg.max_range_5ns_ticks := unsigned(
            range_word(c_RC_MAX_RANGE_5NS_HI downto c_RC_MAX_RANGE_5NS_LO));
        if unsigned(range_word(c_RC_COLS_HI downto c_RC_COLS_LO)) = 0 then
            v_cfg.cols_per_face := to_unsigned(1, 16);
        else
            v_cfg.cols_per_face := unsigned(
                range_word(c_RC_COLS_HI downto c_RC_COLS_LO));
        end if;
        v_cfg.start_off1 := unsigned(start_word(17 downto 0));
        v_cfg.cfg_reg7 := reg7_word;
        v_cfg.max_scan_5ns_ticks := unsigned(
            scan_word(c_ST_MAX_SCAN_HI downto c_ST_MAX_SCAN_LO));
        v_cfg.max_hits_cfg := to_unsigned(fn_effective_max_hits(
            unsigned(scan_word(c_ST_MAX_HITS_HI downto c_ST_MAX_HITS_LO)),
            g_MAX_HITS_PER_STOP), 3);
        v_cfg.falling_enable := scan_word(c_ST_FALLING_ENABLE);
        return v_cfg;
    end function fn_decode_cfg;

    function fn_initial_cfg return t_tdc_cfg is
        variable v_cfg : t_tdc_cfg;
    begin
        v_cfg := fn_decode_cfg(
            c_INIT_BUS_TIMING,
            c_INIT_START_OFF1,
            c_INIT_CFG_REG7,
            c_INIT_SCAN_TIMEOUT,
            c_INIT_MAIN_CTRL,
            c_INIT_RANGE_COLS,
            "101");
        return v_cfg;
    end function fn_initial_cfg;

    constant C_INITIAL_CFG   : t_tdc_cfg := fn_initial_cfg;
    constant C_DEFAULT_IMAGE : t_cfg_image := c_GPX_DEFAULT_IMAGE;

    signal s_image_staging : t_cfg_image := C_DEFAULT_IMAGE;
    signal s_image_selected_index : natural range 0 to 31 := 0;
    signal s_image_seen_epoch     : std_logic_vector(7 downto 0) := (others => '0');
    signal s_image_accepted_epoch : std_logic_vector(7 downto 0) := (others => '0');
    signal s_image_seen_reset_epoch : std_logic_vector(7 downto 0) := (others => '0');
    signal s_image_reject_sticky  : std_logic := '0';
    signal s_image_reject_pulse   : std_logic := '0';

    signal s_cfg_src  : std_logic_vector(C_CFG_WIDTH - 1 downto 0);
    signal s_cfg_dest : std_logic_vector(C_CFG_WIDTH - 1 downto 0);
    signal s_cfg_image_dest : t_cfg_image;

    signal s_reset_epoch_word : std_logic_vector(7 downto 0);
    signal s_cfg_epoch_word   : std_logic_vector(7 downto 0);
    signal s_bus_word         : std_logic_vector(31 downto 0);
    signal s_start_word       : std_logic_vector(31 downto 0);
    signal s_reg7_word        : std_logic_vector(31 downto 0);
    signal s_scan_word        : std_logic_vector(31 downto 0);
    signal s_main_word        : std_logic_vector(31 downto 0);
    signal s_range_word       : std_logic_vector(31 downto 0);
    signal s_aux_word         : std_logic_vector(31 downto 0);
    signal s_n_faces_word     : std_logic_vector(2 downto 0);

    signal s_active_cfg    : t_tdc_cfg := C_INITIAL_CFG;
    signal s_active_image  : t_cfg_image := C_DEFAULT_IMAGE;
    signal s_pending_cfg   : t_tdc_cfg := C_INITIAL_CFG;
    signal s_pending_image : t_cfg_image := C_DEFAULT_IMAGE;
    signal s_pending_cfg_epoch : std_logic_vector(7 downto 0) := (others => '0');
    signal s_apply_pending : std_logic := '0';
    signal s_cfg_write_delay : std_logic := '0';

    signal s_seen_reset_epoch     : std_logic_vector(7 downto 0) := (others => '0');
    signal s_accepted_reset_epoch : std_logic_vector(7 downto 0) := (others => '0');
    signal s_seen_cfg_epoch       : std_logic_vector(7 downto 0) := (others => '0');
    signal s_accepted_cfg_epoch   : std_logic_vector(7 downto 0) := (others => '0');
    signal s_seen_cmd_epoch       : std_logic_vector(7 downto 0) := (others => '0');
    signal s_accepted_cmd_epoch   : std_logic_vector(7 downto 0) := (others => '0');
    signal s_cfg_reject_sticky    : std_logic := '0';
    signal s_cmd_reject_sticky    : std_logic := '0';
    signal s_cfg_valid            : std_logic := '1';

    signal s_cmd_pending       : std_logic := '0';
    signal s_pending_cmd_epoch : std_logic_vector(7 downto 0) := (others => '0');
    signal s_pending_opcode    : unsigned(2 downto 0) := (others => '0');
    signal s_cmd_reg_addr_r    : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_reg_chip_r    : unsigned(1 downto 0) := (others => '0');
    signal s_cmd_reg_mask_r    : std_logic_vector(c_MAX_CHIPS - 1 downto 0) :=
        (others => '0');

    signal s_cmd_start_pulse        : std_logic := '0';
    signal s_cmd_stop_pulse         : std_logic := '0';
    signal s_cmd_soft_reset_pulse   : std_logic := '0';
    signal s_cmd_force_reinit_pulse : std_logic := '0';
    signal s_err_soft_clear_pulse   : std_logic := '0';
    signal s_cmd_cfg_write_pulse    : std_logic := '0';
    signal s_cmd_reg_read_pulse     : std_logic := '0';
    signal s_cmd_reg_write_pulse    : std_logic := '0';

    signal s_result_r : t_result_array := (others => (others => '0'));
    signal s_reg12_fault_mask_r : std_logic_vector(c_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal s_pipeline_status_word : std_logic_vector(31 downto 0);
    signal s_status_ext_word      : std_logic_vector(31 downto 0);
    signal s_status_ext2_word     : std_logic_vector(31 downto 0);
    signal s_adapter_status_word  : std_logic_vector(31 downto 0);
    signal s_status_src           : std_logic_vector(C_STATUS_WIDTH - 1 downto 0);
    signal s_status_dest          : std_logic_vector(C_STATUS_WIDTH - 1 downto 0);
    signal s_adapter_status_dest  : std_logic_vector(31 downto 0);

    signal s_irq_level      : std_logic_vector(5 downto 0);
    signal s_irq_prev       : std_logic_vector(5 downto 0) := (others => '0');
    signal s_irq_edge_pulse : std_logic_vector(5 downto 0) := (others => '0');
    signal s_irq_axis_pulse : std_logic_vector(6 downto 0);
    signal s_irq_dest       : std_logic_vector(6 downto 0);
begin
    assert g_PRESENT_CHIP_MASK /= (g_PRESENT_CHIP_MASK'range => '0')
        report "g_PRESENT_CHIP_MASK must enable at least one GPX chip"
        severity failure;
    assert g_BUS_READ_PERIOD_MIN_CLKS <= 7
        report "g_BUS_READ_PERIOD_MIN_CLKS must fit the 3-bit bus-ticks field"
        severity failure;

    -- Indexed image staging is local to the unified CSR clock domain.
    p_image_stage : process(i_cfg_clk)
        variable v_epoch : std_logic_vector(7 downto 0);
        variable v_index : natural range 0 to 31;
        variable v_reset : std_logic_vector(7 downto 0);
    begin
        if rising_edge(i_cfg_clk) then
            if i_cfg_rst_n = '0' then
                s_image_staging          <= C_DEFAULT_IMAGE;
                s_image_selected_index   <= 0;
                s_image_seen_epoch       <= (others => '0');
                s_image_accepted_epoch   <= (others => '0');
                s_image_seen_reset_epoch <= (others => '0');
                s_image_reject_sticky    <= '0';
                s_image_reject_pulse     <= '0';
            else
                s_image_reject_pulse <= '0';
                v_epoch := i_tdc_image_cmd(15 downto 8);
                v_index := to_integer(unsigned(i_tdc_image_cmd(4 downto 0)));
                v_reset := i_sys_ctrl(15 downto 8);
                s_image_selected_index <= v_index;

                if v_reset /= s_image_seen_reset_epoch then
                    s_image_seen_reset_epoch <= v_reset;
                    s_image_seen_epoch       <= v_epoch;
                    s_image_accepted_epoch   <= v_epoch;
                    s_image_reject_sticky    <= '0';
                elsif v_epoch /= s_image_seen_epoch then
                    s_image_seen_epoch     <= v_epoch;
                    s_image_accepted_epoch <= v_epoch;
                    if v_index < c_CFG_IMAGE_N_REGS then
                        s_image_staging(v_index) <= i_tdc_image_data;
                        s_image_reject_sticky <= '0';
                    else
                        s_image_reject_sticky <= '1';
                        s_image_reject_pulse  <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_image_stage;

    o_image_selected_data <= s_image_staging(s_image_selected_index)
        when s_image_selected_index < c_CFG_IMAGE_N_REGS else (others => '0');
    o_image_write_epoch_accepted <= s_image_accepted_epoch;
    o_image_reject <= s_image_reject_sticky;

    -- One coherent snapshot carries all accepted staging data into AXIS.
    s_cfg_src(C_RESET_EPOCH_LO + 7 downto C_RESET_EPOCH_LO) <=
        i_sys_ctrl(15 downto 8);
    s_cfg_src(C_CFG_EPOCH_LO + 7 downto C_CFG_EPOCH_LO) <=
        i_sys_cfg_apply(7 downto 0);
    s_cfg_src(C_BUS_LO + 31 downto C_BUS_LO) <= i_tdc_bus_timing;
    s_cfg_src(C_START_LO + 31 downto C_START_LO) <= i_tdc_start_offset;
    s_cfg_src(C_REG7_LO + 31 downto C_REG7_LO) <= i_tdc_cfg_reg7;
    s_cfg_src(C_SCAN_LO + 31 downto C_SCAN_LO) <= i_tdc_scan_cfg;
    s_cfg_src(C_MAIN_LO + 31 downto C_MAIN_LO) <= i_tdc_pipeline_main;
    s_cfg_src(C_RANGE_LO + 31 downto C_RANGE_LO) <= i_tdc_range_cols;
    s_cfg_src(C_AUX_LO + 31 downto C_AUX_LO) <= i_tdc_aux_cmd;
    s_cfg_src(C_N_FACES_LO + 2 downto C_N_FACES_LO) <= i_n_faces;

    g_image_pack : for i in 0 to c_CFG_IMAGE_N_REGS - 1 generate
        s_cfg_src(C_IMAGE_LO + 32 * i + 31 downto C_IMAGE_LO + 32 * i) <=
            s_image_staging(i);
        s_cfg_image_dest(i) <=
            s_cfg_dest(C_IMAGE_LO + 32 * i + 31 downto C_IMAGE_LO + 32 * i);
    end generate g_image_pack;

    u_cfg_snapshot : entity work.tdc_gpx_unified_cdc_snapshot
        generic map (
            G_WIDTH => C_CFG_WIDTH
        )
        port map (
            src_clk     => i_cfg_clk,
            src_rst_n   => i_cfg_rst_n,
            i_src_data  => s_cfg_src,
            dest_clk    => i_axis_clk,
            o_dest_data => s_cfg_dest
        );

    s_reset_epoch_word <= s_cfg_dest(C_RESET_EPOCH_LO + 7 downto C_RESET_EPOCH_LO);
    s_cfg_epoch_word   <= s_cfg_dest(C_CFG_EPOCH_LO + 7 downto C_CFG_EPOCH_LO);
    s_bus_word   <= s_cfg_dest(C_BUS_LO + 31 downto C_BUS_LO);
    s_start_word <= s_cfg_dest(C_START_LO + 31 downto C_START_LO);
    s_reg7_word  <= s_cfg_dest(C_REG7_LO + 31 downto C_REG7_LO);
    s_scan_word  <= s_cfg_dest(C_SCAN_LO + 31 downto C_SCAN_LO);
    s_main_word  <= s_cfg_dest(C_MAIN_LO + 31 downto C_MAIN_LO);
    s_range_word <= s_cfg_dest(C_RANGE_LO + 31 downto C_RANGE_LO);
    s_aux_word   <= s_cfg_dest(C_AUX_LO + 31 downto C_AUX_LO);
    s_n_faces_word <= s_cfg_dest(C_N_FACES_LO + 2 downto C_N_FACES_LO);

    p_control : process(i_axis_clk)
        variable v_opcode : natural range 0 to 7;
        variable v_mask   : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
        variable v_chip   : natural range 0 to c_MAX_CHIPS - 1;
    begin
        if rising_edge(i_axis_clk) then
            if i_axis_rst_n = '0' then
                s_active_cfg            <= C_INITIAL_CFG;
                s_active_image          <= C_DEFAULT_IMAGE;
                s_pending_cfg           <= C_INITIAL_CFG;
                s_pending_image         <= C_DEFAULT_IMAGE;
                s_pending_cfg_epoch     <= (others => '0');
                s_apply_pending         <= '0';
                s_cfg_write_delay       <= '0';
                s_seen_reset_epoch      <= (others => '0');
                s_accepted_reset_epoch  <= (others => '0');
                s_seen_cfg_epoch        <= (others => '0');
                s_accepted_cfg_epoch    <= (others => '0');
                s_seen_cmd_epoch        <= (others => '0');
                s_accepted_cmd_epoch    <= (others => '0');
                s_cfg_reject_sticky     <= '0';
                s_cmd_reject_sticky     <= '0';
                s_cfg_valid             <= '1';
                s_cmd_pending           <= '0';
                s_pending_cmd_epoch     <= (others => '0');
                s_pending_opcode        <= (others => '0');
                s_cmd_reg_addr_r         <= (others => '0');
                s_cmd_reg_chip_r         <= (others => '0');
                s_cmd_reg_mask_r         <= (others => '0');
                s_cmd_start_pulse        <= '0';
                s_cmd_stop_pulse         <= '0';
                s_cmd_soft_reset_pulse   <= '0';
                s_cmd_force_reinit_pulse <= '0';
                s_err_soft_clear_pulse   <= '0';
                s_cmd_cfg_write_pulse    <= '0';
                s_cmd_reg_read_pulse     <= '0';
                s_cmd_reg_write_pulse    <= '0';
            else
                s_cmd_start_pulse        <= '0';
                s_cmd_stop_pulse         <= '0';
                s_cmd_soft_reset_pulse   <= '0';
                s_cmd_force_reinit_pulse <= '0';
                s_err_soft_clear_pulse   <= '0';
                s_cmd_cfg_write_pulse    <= '0';
                s_cmd_reg_read_pulse     <= '0';
                s_cmd_reg_write_pulse    <= '0';

                -- Reset consumes co-travelling config and command epochs so
                -- stale commands cannot execute after recovery.
                if s_reset_epoch_word /= s_seen_reset_epoch then
                    s_seen_reset_epoch     <= s_reset_epoch_word;
                    s_accepted_reset_epoch <= s_reset_epoch_word;
                    s_seen_cfg_epoch       <= s_cfg_epoch_word;
                    s_accepted_cfg_epoch   <= s_cfg_epoch_word;
                    s_seen_cmd_epoch       <= s_aux_word(15 downto 8);
                    s_accepted_cmd_epoch   <= s_aux_word(15 downto 8);
                    s_apply_pending        <= '0';
                    s_cfg_write_delay      <= '0';
                    s_cmd_pending          <= '0';
                    s_cfg_reject_sticky    <= '0';
                    s_cmd_reject_sticky    <= '0';
                    s_cmd_soft_reset_pulse <= '1';
                else
                    if s_cfg_epoch_word /= s_seen_cfg_epoch then
                        s_seen_cfg_epoch <= s_cfg_epoch_word;
                        if s_apply_pending = '0' and s_cfg_write_delay = '0' then
                            s_pending_cfg <= fn_decode_cfg(
                                s_bus_word,
                                s_start_word,
                                s_reg7_word,
                                s_scan_word,
                                s_main_word,
                                s_range_word,
                                s_n_faces_word);
                            s_pending_image     <= s_cfg_image_dest;
                            s_pending_cfg_epoch <= s_cfg_epoch_word;
                            s_apply_pending     <= '1';
                            s_cfg_reject_sticky <= '0';
                        else
                            s_cfg_reject_sticky <= '1';
                        end if;
                    end if;

                    if s_apply_pending = '1' and i_cfg_ready = '1' then
                        s_active_cfg      <= s_pending_cfg;
                        s_active_image    <= s_pending_image;
                        s_apply_pending   <= '0';
                        s_cfg_write_delay <= '1';
                    elsif s_cfg_write_delay = '1' then
                        s_cfg_write_delay      <= '0';
                        s_cmd_cfg_write_pulse  <= '1';
                        s_accepted_cfg_epoch   <= s_pending_cfg_epoch;
                        s_cfg_valid            <= '1';
                    end if;

                    if s_aux_word(15 downto 8) /= s_seen_cmd_epoch then
                        s_seen_cmd_epoch <= s_aux_word(15 downto 8);
                        v_opcode := to_integer(unsigned(s_aux_word(2 downto 0)));
                        if v_opcode >= 1 and v_opcode <= 6
                           and s_cmd_pending = '0' then
                            s_pending_opcode    <= to_unsigned(v_opcode, 3);
                            s_pending_cmd_epoch <= s_aux_word(15 downto 8);
                            s_cmd_pending       <= '1';
                            s_cmd_reject_sticky <= '0';
                            s_cmd_reg_addr_r <= s_bus_word(
                                c_BT_REG_ADDR_HI downto c_BT_REG_ADDR_LO);
                            s_cmd_reg_chip_r <= unsigned(s_bus_word(
                                c_BT_REG_CHIP_HI downto c_BT_REG_CHIP_LO));
                            v_mask := s_bus_word(
                                c_BT_REG_CHIP_ADDR_HI downto c_BT_REG_CHIP_ADDR_LO);
                            if v_mask = (v_mask'range => '0') then
                                v_chip := to_integer(unsigned(s_bus_word(
                                    c_BT_REG_CHIP_HI downto c_BT_REG_CHIP_LO)));
                                v_mask := std_logic_vector(shift_left(
                                    to_unsigned(1, c_MAX_CHIPS), v_chip));
                            end if;
                            s_cmd_reg_mask_r <= v_mask;
                        else
                            s_cmd_reject_sticky <= '1';
                        end if;
                    end if;

                    if s_cmd_pending = '1' and i_cmd_ready = '1' then
                        case to_integer(s_pending_opcode) is
                            when 1 => s_cmd_start_pulse <= '1';
                            when 2 => s_cmd_stop_pulse <= '1';
                            when 3 => s_cmd_force_reinit_pulse <= '1';
                            when 4 => s_err_soft_clear_pulse <= '1';
                            when 5 => s_cmd_reg_read_pulse <= '1';
                            when 6 => s_cmd_reg_write_pulse <= '1';
                            when others => null;
                        end case;
                        s_accepted_cmd_epoch <= s_pending_cmd_epoch;
                        s_cmd_pending <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process p_control;

    p_cfg_output : process(all)
        variable v_cfg : t_tdc_cfg;
    begin
        v_cfg := s_active_cfg;
        v_cfg.n_faces := fn_sanitize_n_faces(s_n_faces_word);
        o_cfg <= v_cfg;
    end process p_cfg_output;

    o_cfg_image <= s_active_image;
    o_cmd_start        <= s_cmd_start_pulse;
    o_cmd_stop         <= s_cmd_stop_pulse;
    o_cmd_soft_reset   <= s_cmd_soft_reset_pulse;
    o_cmd_force_reinit <= s_cmd_force_reinit_pulse;
    o_err_soft_clear   <= s_err_soft_clear_pulse;
    o_cmd_cfg_write    <= s_cmd_cfg_write_pulse;
    o_cmd_reg_read     <= s_cmd_reg_read_pulse;
    o_cmd_reg_write    <= s_cmd_reg_write_pulse;
    o_cmd_reg_addr     <= s_cmd_reg_addr_r;
    o_cmd_reg_chip     <= s_cmd_reg_chip_r;
    o_cmd_reg_chip_address <= s_cmd_reg_mask_r;

    p_result_latch : process(i_axis_clk)
    begin
        if rising_edge(i_axis_clk) then
            if i_axis_rst_n = '0' then
                s_result_r <= (others => (others => '0'));
                s_reg12_fault_mask_r <= (others => '0');
            elsif s_err_soft_clear_pulse = '1' then
                s_reg12_fault_mask_r <= (others => '0');
            else
                if i_cmd_reg_rvalid(0) = '1' then
                    s_result_r(0) <= i_cmd_reg_addr_done & i_cmd_reg_rdata_0;
                    if i_cmd_reg_addr_done = c_TDC_REG12
                       and i_cmd_reg_rdata_0(10 downto 0) /=
                           (i_cmd_reg_rdata_0(10 downto 0)'range => '0') then
                        s_reg12_fault_mask_r(0) <= '1';
                    end if;
                end if;
                if i_cmd_reg_rvalid(1) = '1' then
                    s_result_r(1) <= i_cmd_reg_addr_done & i_cmd_reg_rdata_1;
                    if i_cmd_reg_addr_done = c_TDC_REG12
                       and i_cmd_reg_rdata_1(10 downto 0) /=
                           (i_cmd_reg_rdata_1(10 downto 0)'range => '0') then
                        s_reg12_fault_mask_r(1) <= '1';
                    end if;
                end if;
                if i_cmd_reg_rvalid(2) = '1' then
                    s_result_r(2) <= i_cmd_reg_addr_done & i_cmd_reg_rdata_2;
                    if i_cmd_reg_addr_done = c_TDC_REG12
                       and i_cmd_reg_rdata_2(10 downto 0) /=
                           (i_cmd_reg_rdata_2(10 downto 0)'range => '0') then
                        s_reg12_fault_mask_r(2) <= '1';
                    end if;
                end if;
                if i_cmd_reg_rvalid(3) = '1' then
                    s_result_r(3) <= i_cmd_reg_addr_done & i_cmd_reg_rdata_3;
                    if i_cmd_reg_addr_done = c_TDC_REG12
                       and i_cmd_reg_rdata_3(10 downto 0) /=
                           (i_cmd_reg_rdata_3(10 downto 0)'range => '0') then
                        s_reg12_fault_mask_r(3) <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_result_latch;

    p_status_words : process(all)
        variable v_status : std_logic_vector(31 downto 0);
        variable v_ext    : std_logic_vector(31 downto 0);
        variable v_ext2   : std_logic_vector(31 downto 0);
    begin
        v_status := (others => '0');
        v_status(c_STAT_BUSY) := i_status.busy;
        v_status(c_STAT_OVERRUN) := i_status.pipeline_overrun;
        v_status(c_STAT_ERR_FATAL) := i_status.err_fatal;
        v_status(c_STAT_CHIP_ERR_HI downto c_STAT_CHIP_ERR_LO) :=
            i_status.chip_error_mask or s_reg12_fault_mask_r;
        v_status(c_STAT_DRAIN_TO_HI downto c_STAT_DRAIN_TO_LO) :=
            i_status.drain_timeout_mask;
        v_status(c_STAT_SEQ_ERR_HI downto c_STAT_SEQ_ERR_LO) :=
            i_status.sequence_error_mask;
        v_status(23 downto 16) := s_accepted_cmd_epoch;

        v_ext := (others => '0');
        v_ext(c_STAT6_ERR_READ_TIMEOUT) := i_status.err_read_timeout;
        v_ext(c_STAT6_REG_REJECTED) := i_status.reg_rejected;
        v_ext(c_STAT6_REG_ZERO_MASK) := i_status.reg_zero_mask;
        v_ext(c_STAT6_SHOT_FLUSH_DROP_RISE) := i_status.rise_shot_flush_drop;
        v_ext(c_STAT6_SHOT_FLUSH_DROP_FALL) := i_status.fall_shot_flush_drop;
        v_ext(c_STAT6_HDR_DRAIN_TO_RISE) := i_status.rise_hdr_drain_timeout;
        v_ext(c_STAT6_HDR_DRAIN_TO_FALL) := i_status.fall_hdr_drain_timeout;
        v_ext(c_STAT6_FRAME_WAIT_ESCAPE) := i_status.err_frame_wait_escape;
        v_ext(c_STAT6_OVRUN_CNT_RISE_HI downto c_STAT6_OVRUN_CNT_RISE_LO) :=
            std_logic_vector(i_status.rise_shot_overrun_count(3 downto 0));
        v_ext(c_STAT6_SHOT_FLUSH_MASK_HI downto c_STAT6_SHOT_FLUSH_MASK_LO) :=
            i_status.shot_flush_drop_mask;
        v_ext(c_STAT6_OVRUN_CNT_FALL_HI downto c_STAT6_OVRUN_CNT_FALL_LO) :=
            std_logic_vector(i_status.fall_shot_overrun_count(3 downto 0));
        v_ext(c_STAT6_CMD_COLL_HI downto c_STAT6_CMD_COLL_LO) :=
            i_status.cmd_collision_mask;
        v_ext(c_STAT6_ERR_REG_OVR_HI downto c_STAT6_ERR_REG_OVR_LO) :=
            i_status.err_reg_overflow_mask;
        v_ext(c_STAT6_RUN_DRAIN_COMP_HI downto c_STAT6_RUN_DRAIN_COMP_LO) :=
            i_status.run_drain_complete_mask;

        v_ext2 := (others => '0');
        v_ext2(c_STAT7_REG_TO_HI downto c_STAT7_REG_TO_LO) :=
            i_status.reg_timeout_mask;
        v_ext2(c_STAT7_STOP_ID_ERR_HI downto c_STAT7_STOP_ID_ERR_LO) :=
            i_status.stop_id_error_mask;
        v_ext2(c_STAT7_RUN_CAUSE_HI downto c_STAT7_RUN_CAUSE_LO) :=
            i_status.run_timeout_cause_last;
        v_ext2(c_STAT7_QUARANTINE_HI downto c_STAT7_QUARANTINE_LO) :=
            i_status.quarantine_escape_mask;
        v_ext2(c_STAT7_MASKED_SLOPE_DROP) := i_status.masked_slope_drop_any;
        v_ext2(c_STAT7_FS_COLL_RISE_HI downto c_STAT7_FS_COLL_RISE_LO) :=
            std_logic_vector(i_status.rise_face_start_collapsed_count(3 downto 0));
        v_ext2(c_STAT7_REG12_FAULT_HI downto c_STAT7_REG12_FAULT_LO) :=
            s_reg12_fault_mask_r;
        v_ext2(c_STAT7_FS_COLL_FALL_HI downto c_STAT7_FS_COLL_FALL_LO) :=
            std_logic_vector(i_status.fall_face_start_collapsed_count(3 downto 0));
        v_ext2(c_STAT7_INIT_COALESCE_HI downto c_STAT7_INIT_COALESCE_LO) :=
            i_status.init_cfg_coalesced_mask;

        s_pipeline_status_word <= v_status;
        s_status_ext_word      <= v_ext;
        s_status_ext2_word     <= v_ext2;
    end process p_status_words;

    s_adapter_status_word <=
        (31 downto 28 => '0')
        & s_cmd_reject_sticky
        & s_cfg_valid
        & s_cfg_reject_sticky
        & (s_apply_pending or s_cfg_write_delay)
        & s_accepted_cmd_epoch
        & s_accepted_reset_epoch
        & s_accepted_cfg_epoch;

    s_status_src(31 downto 0)    <= s_result_r(0);
    s_status_src(63 downto 32)   <= s_result_r(1);
    s_status_src(95 downto 64)   <= s_result_r(2);
    s_status_src(127 downto 96)  <= s_result_r(3);
    s_status_src(159 downto 128) <= s_pipeline_status_word;
    s_status_src(191 downto 160) <= s_status_ext_word;
    s_status_src(223 downto 192) <= s_status_ext2_word;
    s_status_src(255 downto 224) <= s_adapter_status_word;

    u_status_snapshot : entity work.tdc_gpx_unified_cdc_snapshot
        generic map (
            G_WIDTH => C_STATUS_WIDTH
        )
        port map (
            src_clk     => i_axis_clk,
            src_rst_n   => i_axis_rst_n,
            i_src_data  => s_status_src,
            dest_clk    => i_cfg_clk,
            o_dest_data => s_status_dest
        );

    o_chip0_result <= s_status_dest(31 downto 0);
    o_chip1_result <= s_status_dest(63 downto 32);
    o_chip2_result <= s_status_dest(95 downto 64);
    o_chip3_result <= s_status_dest(127 downto 96);

    p_pipeline_status_output : process(all)
        variable v_status : std_logic_vector(31 downto 0);
    begin
        v_status := s_status_dest(159 downto 128);
        v_status(31 downto 24) := s_image_accepted_epoch;
        o_pipeline_status <= v_status;
    end process p_pipeline_status_output;

    o_status_ext  <= s_status_dest(191 downto 160);
    o_status_ext2 <= s_status_dest(223 downto 192);
    s_adapter_status_dest <= s_status_dest(255 downto 224);

    o_cfg_epoch_accepted   <= s_adapter_status_dest(7 downto 0);
    o_reset_epoch_accepted <= s_adapter_status_dest(15 downto 8);
    o_cmd_epoch_accepted   <= s_adapter_status_dest(23 downto 16);
    o_cfg_valid            <= s_adapter_status_dest(26);
    o_command_reject       <= s_adapter_status_dest(27);
    o_cfg_reject <= s_adapter_status_dest(25) or s_image_reject_sticky;
    o_cfg_busy <= '1' when s_adapter_status_dest(24) = '1'
        or (i_sys_cfg_apply(7 downto 0) /= s_adapter_status_dest(7 downto 0)
            and s_adapter_status_dest(25) = '0')
        else '0';
    o_cmd_busy <= '1' when
        i_tdc_aux_cmd(15 downto 8) /= s_adapter_status_dest(23 downto 16)
        and s_adapter_status_dest(27) = '0'
        else '0';

    -- Seven independent interrupt identities. Sticky diagnostics are converted
    -- to one event on their inactive-to-active transition.
    s_irq_level(0) <= i_status.pipeline_overrun or i_status.err_fatal;
    s_irq_level(1) <= fn_any(i_status.chip_error_mask)
        or fn_any(s_reg12_fault_mask_r);
    s_irq_level(2) <= fn_any(i_status.drain_timeout_mask)
        or fn_any(i_status.reg_timeout_mask)
        or i_status.err_read_timeout
        or i_status.rise_hdr_drain_timeout
        or i_status.fall_hdr_drain_timeout;
    s_irq_level(3) <= fn_any(i_status.sequence_error_mask)
        or fn_any(i_status.stop_id_error_mask)
        or fn_any(i_status.cmd_collision_mask)
        or i_status.masked_slope_drop_any
        or i_status.reg_rejected
        or i_status.reg_zero_mask;
    s_irq_level(4) <= i_status.cfg_rejected or s_cfg_reject_sticky;
    s_irq_level(5) <= s_cmd_reject_sticky;

    p_irq_edge : process(i_axis_clk)
    begin
        if rising_edge(i_axis_clk) then
            if i_axis_rst_n = '0' then
                s_irq_prev       <= (others => '0');
                s_irq_edge_pulse <= (others => '0');
            else
                s_irq_prev       <= s_irq_level;
                s_irq_edge_pulse <= s_irq_level and not s_irq_prev;
            end if;
        end if;
    end process p_irq_edge;

    s_irq_axis_pulse(0) <= i_cmd_reg_done_pulse;
    s_irq_axis_pulse(1) <= s_irq_edge_pulse(0);
    s_irq_axis_pulse(2) <= s_irq_edge_pulse(1);
    s_irq_axis_pulse(3) <= s_irq_edge_pulse(2);
    s_irq_axis_pulse(4) <= s_irq_edge_pulse(3);
    s_irq_axis_pulse(5) <= s_irq_edge_pulse(4);
    s_irq_axis_pulse(6) <= s_irq_edge_pulse(5);

    g_irq_cdc : for i in 0 to 6 generate
        u_irq_cdc : xpm_cdc_pulse
            generic map (
                DEST_SYNC_FF   => 4,
                INIT_SYNC_FF   => 0,
                REG_OUTPUT     => 1,
                RST_USED       => 1,
                SIM_ASSERT_CHK => 1
            )
            port map (
                src_clk    => i_axis_clk,
                src_rst    => not i_axis_rst_n,
                src_pulse  => s_irq_axis_pulse(i),
                dest_clk   => i_cfg_clk,
                dest_rst   => not i_cfg_rst_n,
                dest_pulse => s_irq_dest(i)
            );
    end generate g_irq_cdc;

    p_irq_output : process(all)
        variable v_irq : std_logic_vector(6 downto 0);
    begin
        v_irq := s_irq_dest;
        v_irq(5) := v_irq(5) or s_image_reject_pulse;
        o_irq_cause <= v_irq;
    end process p_irq_output;
end architecture rtl;
