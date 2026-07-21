-- =============================================================================
-- tb_tdc_gpx_chip_ctrl.vhd
-- TDC-GPX Controller - chip_ctrl + bus_phy Integration Test
-- =============================================================================
--
-- Integration test verifying the full per-chip pipeline:
--   chip_ctrl -> bus_phy -> TDC-GPX chip model
--
-- A behavioral TDC-GPX chip model is included to emulate:
--   - Register writes (capture and verify cfg_image)
--   - IFIFO reads (return sequential test data, honor EF/LF flags)
--   - IrFlag assertion after configurable delay (simulates MTimer expiry)
--   - EF/LF flag management based on FIFO fill level
--
-- Test scenarios:
--   [1] Powerup + cfg_write + master_reset sequence
--   [2] Arm -> shot_start -> IrFlag -> Legacy drain (drain_mode='0')
--   [2b] Legacy EF fallback when expected count is absent
--   [2c] Known-zero expected final completes cleanly with empty EF
--   [2d] Known-zero expected final blocks EF fallback reads on conflict
--   [3] Arm -> shot_start -> IrFlag -> Burst drain (drain_mode='1')
--   [4] n_drain_cap enforcement (per-IFIFO cap=n×4, e.g. cap=2 -> max 8/IFIFO)
--   [5] Soft reset recovery
--   [6] cfg_write register content verification
--   [7] EF=1 -> no reads (INV-4)
--   [8] AluTrigger pulse width >= 10ns
--   [9] IFIFO edge cases (1 entry, 32 entries)
--   [10] Consecutive 2+ shots
--   [16a] No-backpressure first-data latency measurement
--   [16b] Bounded raw AXI backpressure with T1a/T1b split timing
--   [16c] Raw FIFO reserve-threshold backpressure (no data/control loss)
--   [17] Stale expected-count mismatch -> faulted drain_done, no empty read
--   [18] Global C02 monitors: no empty IFIFO reads, raw tuser contract clean
--   [19] PH_RESP_DRAIN stuck/fatal quarantine and auto-recover.
--   [20] Forced pending response trips the secondary deadlock watchdog.
--   Negative modes:
--     g_NEGATIVE_MODE=1: force empty IFIFO read monitor and fail intentionally.
--     g_NEGATIVE_MODE=2: force raw tuser monitor and fail intentionally.
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.tdc_gpx_pkg.all;
use work.tb_tdc_gpx_pkg.all;

entity tb_tdc_gpx_chip_ctrl is
    generic (
        g_CHIP_ID       : natural := 0; -- override via -generic_top "g_CHIP_ID=0..3"
        g_NEGATIVE_MODE : natural := 0  -- 0=positive, 1=empty-read, 2=tuser
    );
end entity tb_tdc_gpx_chip_ctrl;

architecture sim of tb_tdc_gpx_chip_ctrl is

    -- =========================================================================
    -- TB constants
    -- =========================================================================
    constant c_CLK_PERIOD       : time    := 5 ns;       -- 200 MHz
    constant c_DATA_W           : natural := c_TDC_BUS_WIDTH;  -- 28
    constant c_CHIP_ID          : natural := g_CHIP_ID;

    -- Powerup/recovery generics (short for sim speed)
    constant c_POWERUP_CLKS     : natural := 10;
    constant c_RECOVERY_CLKS    : natural := 4;
    constant c_ALU_PULSE_CLKS   : natural := 3;  -- 3 ticks: 2 clks high = 10ns @ 200MHz

    -- Timeout
    constant c_TIMEOUT          : natural := 2000;

    -- IFIFO model: number of words per IFIFO
    constant c_FIFO_DEPTH       : natural := 32;
    -- Nominal I-Mode single-shot load used by normal IFIFO1/2 tests:
    -- each IFIFO covers 4 stop channels, with 7 echoes per stop.
    constant c_IFIFO_STOP_CHANNELS : natural := 4;
    constant c_ECHOES_PER_STOP     : natural := 7;
    constant c_IFIFO_NOMINAL_WORDS : natural := c_IFIFO_STOP_CHANNELS * c_ECHOES_PER_STOP;
    constant c_IFIFO_NOMINAL_TOTAL : natural := 2 * c_IFIFO_NOMINAL_WORDS;
    -- LF threshold: LF='1' when fill >= threshold
    constant c_LF_THRESHOLD     : natural := 4;
    subtype t_imode_word is std_logic_vector(c_DATA_W - 1 downto 0);

    -- =========================================================================
    -- Simulation control
    -- =========================================================================
    signal s_sim_done           : std_logic := '0';

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal s_clk                : std_logic := '0';
    signal s_rst_n              : std_logic := '0';

    -- =========================================================================
    -- CSR configuration
    -- =========================================================================
    signal s_cfg                : t_tdc_cfg := c_TDC_CFG_INIT;
    signal s_cfg_image          : t_cfg_image := (others => (others => '0'));

    -- =========================================================================
    -- CSR commands
    -- =========================================================================
    signal s_cmd_start          : std_logic := '0';
    signal s_cmd_stop           : std_logic := '0';
    signal s_cmd_soft_reset     : std_logic := '0';
    signal s_cmd_cfg_write      : std_logic := '0';
    signal s_cmd_reg_read       : std_logic := '0';
    signal s_cmd_reg_write      : std_logic := '0';
    signal s_cmd_reg_addr       : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_reg_wdata      : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_cmd_reg_rdata      : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_cmd_reg_rvalid     : std_logic;

    -- =========================================================================
    -- Shot trigger
    -- =========================================================================
    signal s_shot_start         : std_logic := '0';

    -- =========================================================================
    -- chip_ctrl <-> bus_phy internal wires
    -- =========================================================================
    signal s_bus_req_valid      : std_logic;
    signal s_bus_req_rw         : std_logic;
    signal s_bus_req_addr       : std_logic_vector(3 downto 0);
    signal s_bus_req_wdata      : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_bus_oen_perm       : std_logic;
    signal s_bus_req_burst      : std_logic;
    signal s_bus_busy           : std_logic;
    signal s_bus_rsp_pending    : std_logic;
    signal s_bus_busy_to_ctrl   : std_logic;
    signal s_bus_rsp_pending_to_ctrl : std_logic;
    signal s_force_resp_drain_stuck : std_logic := '0';
    signal s_force_bus_rsp_pending  : std_logic := '0';
    signal s_force_bus_rsp_hide     : std_logic := '0';
    signal s_tick_en            : std_logic;

    -- bus_phy → chip_ctrl AXI-Stream (response)
    signal s_brsp_axis_tvalid   : std_logic;
    signal s_brsp_axis_tvalid_to_ctrl : std_logic;
    signal s_brsp_axis_tdata    : std_logic_vector(31 downto 0);
    signal s_brsp_axis_tkeep    : std_logic_vector(3 downto 0);
    signal s_brsp_axis_tuser    : std_logic_vector(7 downto 0);
    signal s_brsp_axis_tready   : std_logic;

    -- =========================================================================
    -- bus_phy physical pins
    -- =========================================================================
    signal s_adr                : std_logic_vector(3 downto 0);
    signal s_csn                : std_logic;
    signal s_rdn                : std_logic;
    signal s_wrn                : std_logic;
    signal s_oen                : std_logic;
    signal s_io_d               : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_d_out              : std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_d_tri              : std_logic;

    -- Status pins (driven by chip model)
    signal s_ef1_pin            : std_logic := '1';
    signal s_ef2_pin            : std_logic := '1';
    signal s_lf1_pin            : std_logic := '0';
    signal s_lf2_pin            : std_logic := '0';
    signal s_irflag_pin         : std_logic := '0';
    signal s_errflag_pin        : std_logic := '0';

    -- Sync outputs
    signal s_ef1_sync           : std_logic;
    signal s_ef2_sync           : std_logic;
    signal s_lf1_sync           : std_logic;
    signal s_lf2_sync           : std_logic;
    signal s_irflag_sync        : std_logic;
    signal s_errflag_sync       : std_logic;

    -- =========================================================================
    -- chip_ctrl outputs
    -- =========================================================================
    signal s_stopdis            : std_logic;
    signal s_alutrigger         : std_logic;
    signal s_puresn             : std_logic;
    -- chip_ctrl raw AXI-Stream outputs
    signal s_raw_axis_tvalid    : std_logic;
    signal s_raw_axis_tdata     : std_logic_vector(31 downto 0);
    signal s_raw_axis_tuser     : std_logic_vector(7 downto 0);
    signal s_raw_axis_tready    : std_logic := '1';
    signal s_drain_done         : std_logic;
    signal s_shot_seq           : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal s_ctrl_busy          : std_logic;
    signal s_stop_tdc           : std_logic := '0';
    signal s_err_drain_timeout  : std_logic;
    signal s_err_sequence       : std_logic;
    signal s_run_drain_complete : std_logic;
    signal s_err_raw_overflow   : std_logic;
    signal s_err_raw_drop       : std_logic;
    signal s_err_drain_cap      : std_logic;
    signal s_err_bus_fatal      : std_logic;
    signal s_run_timeout        : std_logic;
    signal s_run_timeout_cause  : std_logic_vector(2 downto 0);
    signal s_expected_ififo1    : unsigned(7 downto 0) := (others => '0');
    signal s_expected_ififo2    : unsigned(7 downto 0) := (others => '0');
    signal s_expected_final_valid : std_logic := '0';

    -- =========================================================================
    -- TDC-GPX Chip Model: FIFO state
    -- All fill/counter signals driven exclusively by p_chip_model.
    -- Stimulus communicates via load request signals.
    -- =========================================================================
    signal s_fifo1_fill         : natural range 0 to c_FIFO_DEPTH := 0;
    signal s_fifo2_fill         : natural range 0 to c_FIFO_DEPTH := 0;
    signal s_fifo1_rd_cnt       : natural := 0;
    signal s_fifo2_rd_cnt       : natural := 0;

    -- Load request: stim sets these, chip model process loads on rising edge
    signal s_fifo_load_req      : std_logic := '0';
    signal s_fifo_load_n1       : natural range 0 to c_FIFO_DEPTH := 0;
    signal s_fifo_load_n2       : natural range 0 to c_FIFO_DEPTH := 0;

    -- Chip model: D-bus drive
    signal s_chip_d_out         : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_chip_d_oe          : std_logic := '0';

    -- =========================================================================
    -- Monitor: raw AXI output capture and strict C02 contract checks
    -- =========================================================================
    signal s_raw_word_cnt       : natural := 0;
    signal s_raw_data_cnt       : natural := 0;
    signal s_raw_ctrl_cnt       : natural := 0;
    signal s_raw_ififo1_data_cnt : natural := 0;
    signal s_raw_ififo2_data_cnt : natural := 0;
    signal s_raw_ififo1_done_ctrl_cnt : natural := 0;
    signal s_raw_final_done_ctrl_cnt : natural := 0;
    signal s_raw_faulted_ctrl_cnt : natural := 0;
    signal s_raw_tuser_err_cnt  : natural := 0;
    signal s_raw_order_err_cnt  : natural := 0;
    signal s_raw_order_check_en : std_logic := '0';
    signal s_raw_order_reset    : std_logic := '0';
    signal s_empty_read_cnt     : natural := 0;
    signal s_clk_cnt            : natural := 0;
    signal s_force_empty_read_req : std_logic := '0';
    signal s_force_tuser_err_req : std_logic := '0';

    -- =========================================================================
    -- Chip model: write capture (for cfg_write verification)
    -- =========================================================================
    type t_wr_capture_array is array(0 to 15) of std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_wr_capture         : t_wr_capture_array := (others => (others => '0'));
    signal s_wr_capture_cnt     : natural := 0;

    -- =========================================================================
    -- AluTrigger timing measurement
    -- =========================================================================
    signal s_alu_rise_time      : time := 0 ns;
    signal s_alu_fall_time      : time := 0 ns;
    signal s_alu_pulse_width    : time := 0 ns;

    -- Helper: std_logic to character
    function sl_chr(s : std_logic) return character is
    begin
        if s = '1' then return '1'; else return '0'; end if;
    end function;

    function fn_imode_word(rd_cnt : natural) return t_imode_word is
        variable v_word     : t_imode_word := (others => '0');
        variable v_cha_code : natural;
        variable v_echo_idx : natural;
        variable v_hit      : natural;
    begin
        -- I-Mode SINGLE_SHOT raw layout:
        --   [27:26] ChaCode, [25:18] StartNum=0, [17] Slope, [16:0] Hit.
        -- The nominal tests therefore model 4 local stop channels with
        -- 7 echoes/channel by loading 28 words per IFIFO.
        v_cha_code := rd_cnt mod c_IFIFO_STOP_CHANNELS;
        v_echo_idx := rd_cnt / c_IFIFO_STOP_CHANNELS;
        v_hit      := (v_cha_code * 16#100#) + v_echo_idx + 1;

        v_word(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(v_cha_code, c_RAW_CHACODE_HI - c_RAW_CHACODE_LO + 1));
        v_word(c_RAW_SLOPE_BIT) := '0';
        v_word(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(v_hit, c_RAW_HIT_WIDTH));
        return v_word;
    end function;

begin

    -- =========================================================================
    -- Clock
    -- =========================================================================
    s_clk <= not s_clk after c_CLK_PERIOD / 2 when s_sim_done = '0' else s_clk;

    -- =========================================================================
    -- DUT: chip_ctrl
    -- =========================================================================
    s_bus_busy_to_ctrl <= s_bus_busy or s_force_resp_drain_stuck;
    s_bus_rsp_pending_to_ctrl <= s_bus_rsp_pending or s_force_bus_rsp_pending;
    s_brsp_axis_tvalid_to_ctrl <= s_brsp_axis_tvalid and not s_force_bus_rsp_hide;

    u_chip_ctrl : entity work.tdc_gpx_chip_ctrl
        generic map (
            g_CHIP_ID        => c_CHIP_ID,
            g_POWERUP_CLKS   => c_POWERUP_CLKS,
            g_RECOVERY_CLKS  => c_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS => c_ALU_PULSE_CLKS,
            g_BUS_IDLE_STABLE_CLKS => 16
        )
        port map (
            i_clk               => s_clk,
            i_rst_n             => s_rst_n,
            i_cfg               => s_cfg,
            i_cfg_image         => s_cfg_image,
            i_cmd_start         => s_cmd_start,
            i_cmd_stop          => s_cmd_stop,
            i_cmd_soft_reset    => s_cmd_soft_reset,
            i_cmd_soft_reset_err => '0',
            i_cmd_cfg_write     => s_cmd_cfg_write,
            i_cmd_reg_read      => s_cmd_reg_read,
            i_cmd_reg_write     => s_cmd_reg_write,
            i_cmd_reg_addr      => s_cmd_reg_addr,
            i_cmd_reg_wdata     => s_cmd_reg_wdata,
            o_cmd_reg_rdata     => s_cmd_reg_rdata,
            o_cmd_reg_rvalid    => s_cmd_reg_rvalid,
            o_cmd_reg_done      => open,
            i_shot_start        => s_shot_start,
            i_max_range_tdc_clks => fn_range_5ns_ticks_to_clks(
                s_cfg.max_range_5ns_ticks, 200),
            i_stop_tdc          => s_stop_tdc,
            i_expected_ififo1   => s_expected_ififo1,
            i_expected_ififo2   => s_expected_ififo2,
            i_expected_final_valid => s_expected_final_valid,
            o_bus_req_valid     => s_bus_req_valid,
            o_bus_req_rw        => s_bus_req_rw,
            o_bus_req_addr      => s_bus_req_addr,
            o_bus_req_wdata     => s_bus_req_wdata,
            o_bus_oen_permanent => s_bus_oen_perm,
            o_bus_req_burst     => s_bus_req_burst,
            o_bus_clk_div_snap  => open,
            o_bus_ticks_snap    => open,
            i_s_axis_tvalid     => s_brsp_axis_tvalid_to_ctrl,
            i_s_axis_tdata      => s_brsp_axis_tdata,
            i_s_axis_tuser      => s_brsp_axis_tuser,
            o_s_axis_tready     => s_brsp_axis_tready,
            i_bus_busy          => s_bus_busy_to_ctrl,
            i_bus_rsp_pending   => s_bus_rsp_pending_to_ctrl,
            i_ef1_sync          => s_ef1_sync,
            i_ef2_sync          => s_ef2_sync,
            i_irflag_sync       => s_irflag_sync,
            i_lf1_sync          => s_lf1_sync,
            i_lf2_sync          => s_lf2_sync,
            o_tick_en           => s_tick_en,
            o_stopdis           => s_stopdis,
            o_alutrigger        => s_alutrigger,
            o_puresn            => s_puresn,
            o_m_raw_axis_tvalid => s_raw_axis_tvalid,
            o_m_raw_axis_tdata  => s_raw_axis_tdata,
            o_m_raw_axis_tuser  => s_raw_axis_tuser,
            i_m_raw_axis_tready => s_raw_axis_tready,
            o_drain_done        => s_drain_done,
            o_shot_seq          => s_shot_seq,
            o_busy              => s_ctrl_busy,
            o_err_drain_timeout => s_err_drain_timeout,
            o_err_sequence      => s_err_sequence,
            o_err_rsp_mismatch  => open,
            o_err_raw_overflow  => s_err_raw_overflow,
            o_err_raw_drop      => s_err_raw_drop,
            o_err_drain_cap     => s_err_drain_cap,
            o_err_reg_overflow  => open,
            o_run_timeout       => s_run_timeout,
            o_run_timeout_cause => s_run_timeout_cause,
            o_init_cfg_coalesced => open,
            o_err_cmd_collision => open,
            o_err_bus_fatal => s_err_bus_fatal,
            o_run_drain_complete => s_run_drain_complete
        );

    -- =========================================================================
    -- DUT: bus_phy
    -- =========================================================================
    u_bus_phy : entity work.tdc_gpx_bus_phy
        port map (
            i_clk           => s_clk,
            i_rst_n         => s_rst_n,
            i_tick_en       => s_tick_en,
            i_bus_ticks     => s_cfg.bus_ticks,
            i_bus_clk_div   => s_cfg.bus_clk_div,
            i_req_valid     => s_bus_req_valid,
            i_req_rw        => s_bus_req_rw,
            i_req_addr      => s_bus_req_addr,
            i_req_wdata     => s_bus_req_wdata,
            i_oen_permanent => s_bus_oen_perm,
            i_req_burst     => s_bus_req_burst,
            o_m_axis_tvalid => s_brsp_axis_tvalid,
            o_m_axis_tdata  => s_brsp_axis_tdata,
            o_m_axis_tkeep  => s_brsp_axis_tkeep,
            o_m_axis_tuser  => s_brsp_axis_tuser,
            i_m_axis_tready => s_brsp_axis_tready,
            o_busy          => s_bus_busy,
            o_rsp_pending   => s_bus_rsp_pending,
            o_adr           => s_adr,
            o_csn           => s_csn,
            o_rdn           => s_rdn,
            o_wrn           => s_wrn,
            o_oen           => s_oen,
            i_d             => s_io_d,
            o_d             => s_d_out,
            o_d_tri         => s_d_tri,
            i_ef1_pin       => s_ef1_pin,
            i_ef2_pin       => s_ef2_pin,
            i_lf1_pin       => s_lf1_pin,
            i_lf2_pin       => s_lf2_pin,
            i_irflag_pin    => s_irflag_pin,
            i_errflag_pin   => s_errflag_pin,
            o_ef1_sync      => s_ef1_sync,
            o_ef2_sync      => s_ef2_sync,
            o_lf1_sync      => s_lf1_sync,
            o_lf2_sync      => s_lf2_sync,
            o_irflag_sync   => s_irflag_sync,
            o_errflag_sync  => s_errflag_sync
        );

    -- =========================================================================
    -- TDC-GPX Chip Behavior Model
    --
    -- IFIFO model: each FIFO has a fill level.
    -- On RDN falling (read strobe): decrement fill, output sequential data.
    -- EF='1' when fill=0, LF='1' when fill >= LF_THRESHOLD.
    -- =========================================================================

    -- EF/LF flags (active HIGH for EF=empty, active HIGH for LF=loaded)
    s_ef1_pin <= '1' when s_fifo1_fill = 0 else '0';
    s_ef2_pin <= '1' when s_fifo2_fill = 0 else '0';
    s_lf1_pin <= '1' when s_fifo1_fill >= c_LF_THRESHOLD else '0';
    s_lf2_pin <= '1' when s_fifo2_fill >= c_LF_THRESHOLD else '0';

    -- Chip drives D-bus when OEN='0' and RDN='0'
    -- Also handles FIFO load requests from stimulus process.
    p_chip_model : process(s_clk)
        variable v_rdn_prev      : std_logic := '1';
        variable v_wrn_prev      : std_logic := '1';
        variable v_load_prev     : std_logic := '0';
        variable v_empty_read_active : boolean := false;
        variable v_wr_data_hold  : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
        variable v_wr_addr_hold  : std_logic_vector(3 downto 0) := (others => '0');
    begin
        if rising_edge(s_clk) then
            s_chip_d_oe <= '0';

            -- FIFO load request (rising edge of s_fifo_load_req)
            if s_fifo_load_req = '1' and v_load_prev = '0' then
                s_fifo1_fill   <= s_fifo_load_n1;
                s_fifo2_fill   <= s_fifo_load_n2;
                s_fifo1_rd_cnt <= 0;
                s_fifo2_rd_cnt <= 0;
            end if;
            v_load_prev := s_fifo_load_req;

            if s_force_empty_read_req = '1' then
                s_empty_read_cnt <= s_empty_read_cnt + 1;
                assert false
                    report "TB chip model: forced EMPTY IFIFO read monitor violation"
                    severity error;
            end if;

            if s_oen = '0' and s_rdn = '0' then
                s_chip_d_oe <= '1';

                -- Read data: encode FIFO ID + counter in data word
                if s_adr = c_TDC_REG8_IFIFO1 then
                    s_chip_d_out <= fn_imode_word(s_fifo1_rd_cnt);
                elsif s_adr = c_TDC_REG9_IFIFO2 then
                    s_chip_d_out <= fn_imode_word(s_fifo2_rd_cnt);
                else
                    s_chip_d_out <= (others => '0');
                end if;
            end if;

            -- Empty IFIFO read is an access-attempt violation as soon as
            -- RDN falls. Count it here while the read address is still valid;
            -- the RDN rising-edge block below only updates successful reads.
            if s_rdn = '0' and v_rdn_prev = '1' then
                v_empty_read_active := false;
                if s_adr = c_TDC_REG8_IFIFO1 and s_fifo1_fill = 0 then
                    s_empty_read_cnt <= s_empty_read_cnt + 1;
                    v_empty_read_active := true;
                    assert false
                        report "TB chip model: EMPTY IFIFO1 read attempted"
                        severity error;
                elsif s_adr = c_TDC_REG9_IFIFO2 and s_fifo2_fill = 0 then
                    s_empty_read_cnt <= s_empty_read_cnt + 1;
                    v_empty_read_active := true;
                    assert false
                        report "TB chip model: EMPTY IFIFO2 read attempted"
                        severity error;
                end if;
            end if;

            -- On RDN rising edge (end of read strobe): update counters & fill
            -- Note: CSN may go high simultaneously with RDN at Phase H,
            -- so we only check RDN edge + address (address is stable).
            if s_rdn = '1' and v_rdn_prev = '0' then
                if s_adr = c_TDC_REG8_IFIFO1 then
                    if s_fifo1_fill > 0 then
                        s_fifo1_fill   <= s_fifo1_fill - 1;
                        s_fifo1_rd_cnt <= s_fifo1_rd_cnt + 1;
                    elsif not v_empty_read_active then
                        s_empty_read_cnt <= s_empty_read_cnt + 1;
                        assert false
                            report "TB chip model: EMPTY IFIFO1 read attempted"
                            severity error;
                    end if;
                elsif s_adr = c_TDC_REG9_IFIFO2 then
                    if s_fifo2_fill > 0 then
                        s_fifo2_fill   <= s_fifo2_fill - 1;
                        s_fifo2_rd_cnt <= s_fifo2_rd_cnt + 1;
                    elsif not v_empty_read_active then
                        s_empty_read_cnt <= s_empty_read_cnt + 1;
                        assert false
                            report "TB chip model: EMPTY IFIFO2 read attempted"
                            severity error;
                    end if;
                end if;
                v_empty_read_active := false;
            end if;

            v_rdn_prev := s_rdn;

            -- Write capture: hold data/addr while WRN='0' (strobe active),
            -- commit on WRN rising edge. Avoids delta-cycle issue where
            -- bus_phy releases D-bus at the same edge WRN goes high.
            if s_wrn = '0' then
                v_wr_data_hold := s_io_d;
                v_wr_addr_hold := s_adr;
            end if;

            if s_wrn = '1' and v_wrn_prev = '0' then
                s_wr_capture(to_integer(unsigned(v_wr_addr_hold))) <= v_wr_data_hold;
                s_wr_capture_cnt <= s_wr_capture_cnt + 1;
            end if;
            v_wrn_prev := s_wrn;
        end if;
    end process p_chip_model;

    -- D-bus drive
    s_io_d <= s_d_out when s_d_tri = '0' else (others => 'Z');
    s_io_d <= s_chip_d_out when s_chip_d_oe = '1' else (others => 'Z');

    -- =========================================================================
    -- Global cycle counter used by latency / II measurements
    -- =========================================================================
    p_cycle_counter : process(s_clk)
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                s_clk_cnt <= 0;
            else
                s_clk_cnt <= s_clk_cnt + 1;
            end if;
        end if;
    end process p_cycle_counter;

    -- =========================================================================
    -- Raw word monitor + debug trace
    -- =========================================================================
    p_raw_monitor : process(s_clk)
        variable v_clk_cnt : natural := 0;
        variable v_tuser_err_inc : natural := 0;
        variable v_order_err_inc : natural := 0;
        variable v_order_ififo1_idx : natural := 0;
        variable v_order_ififo2_idx : natural := 0;
    begin
        if rising_edge(s_clk) then
            v_clk_cnt := v_clk_cnt + 1;
            v_tuser_err_inc := 0;
            v_order_err_inc := 0;

            if s_raw_order_reset = '1' then
                v_order_ififo1_idx := 0;
                v_order_ififo2_idx := 0;
                s_raw_order_err_cnt <= 0;
            end if;

            if s_raw_axis_tvalid = '1' and s_raw_axis_tready = '1' then
                s_raw_word_cnt <= s_raw_word_cnt + 1;

                if s_raw_axis_tuser(7) = '1' then
                    s_raw_ctrl_cnt <= s_raw_ctrl_cnt + 1;
                    if s_raw_axis_tuser(0) = '0' then
                        s_raw_ififo1_done_ctrl_cnt <= s_raw_ififo1_done_ctrl_cnt + 1;
                    else
                        s_raw_final_done_ctrl_cnt <= s_raw_final_done_ctrl_cnt + 1;
                    end if;

                    if s_raw_axis_tuser(5) = '1' then
                        s_raw_faulted_ctrl_cnt <= s_raw_faulted_ctrl_cnt + 1;
                    end if;

                    if s_raw_axis_tuser(6) /= '0'
                       or s_raw_axis_tuser(4 downto 1) /= "0000" then
                        v_tuser_err_inc := v_tuser_err_inc + 1;
                        assert false
                            report "TB raw monitor: control tuser reserved bits violated"
                            severity error;
                    end if;

                    if s_raw_order_check_en = '1' then
                        if s_raw_axis_tuser(0) = '0' then
                            if v_order_ififo1_idx /= c_IFIFO_NOMINAL_WORDS
                               or v_order_ififo2_idx /= 0 then
                                v_order_err_inc := v_order_err_inc + 1;
                                assert false
                                    report "TB raw order: IFIFO1-done control out of order"
                                    severity error;
                            end if;
                        elsif v_order_ififo1_idx /= c_IFIFO_NOMINAL_WORDS
                              or v_order_ififo2_idx /= c_IFIFO_NOMINAL_WORDS then
                            v_order_err_inc := v_order_err_inc + 1;
                            assert false
                                report "TB raw order: final control out of order"
                                severity error;
                        end if;
                    end if;

                    pr_info("  @" & nat_img(v_clk_cnt)
                            & " raw_ctrl ififo=" & sl_chr(s_raw_axis_tuser(0))
                            & " faulted=" & sl_chr(s_raw_axis_tuser(5)));
                else
                    s_raw_data_cnt <= s_raw_data_cnt + 1;
                    if s_raw_axis_tuser(0) = '0' then
                        s_raw_ififo1_data_cnt <= s_raw_ififo1_data_cnt + 1;
                    else
                        s_raw_ififo2_data_cnt <= s_raw_ififo2_data_cnt + 1;
                    end if;

                    if s_raw_axis_tuser(6 downto 1) /= "000000" then
                        v_tuser_err_inc := v_tuser_err_inc + 1;
                        assert false
                            report "TB raw monitor: data tuser reserved bits violated"
                            severity error;
                    end if;

                    if s_raw_order_check_en = '1' then
                        if s_raw_axis_tuser(0) = '0' then
                            if v_order_ififo1_idx >= c_IFIFO_NOMINAL_WORDS
                               or s_raw_axis_tdata(c_DATA_W - 1 downto 0)
                                  /= fn_imode_word(v_order_ififo1_idx) then
                                v_order_err_inc := v_order_err_inc + 1;
                                assert false
                                    report "TB raw order: IFIFO1 payload mismatch at index "
                                           & integer'image(v_order_ififo1_idx)
                                    severity error;
                            end if;
                            v_order_ififo1_idx := v_order_ififo1_idx + 1;
                        else
                            if v_order_ififo1_idx /= c_IFIFO_NOMINAL_WORDS
                               or v_order_ififo2_idx >= c_IFIFO_NOMINAL_WORDS
                               or s_raw_axis_tdata(c_DATA_W - 1 downto 0)
                                  /= fn_imode_word(v_order_ififo2_idx) then
                                v_order_err_inc := v_order_err_inc + 1;
                                assert false
                                    report "TB raw order: IFIFO2 payload mismatch at index "
                                           & integer'image(v_order_ififo2_idx)
                                    severity error;
                            end if;
                            v_order_ififo2_idx := v_order_ififo2_idx + 1;
                        end if;
                    end if;

                    pr_info("  @" & nat_img(v_clk_cnt)
                            & " raw_word=0x" & hex_img(s_raw_axis_tdata(c_DATA_W - 1 downto 0))
                            & " ififo=" & sl_chr(s_raw_axis_tuser(0)));
                end if;
            end if;

            if s_force_tuser_err_req = '1' then
                v_tuser_err_inc := v_tuser_err_inc + 1;
                assert false
                    report "TB raw monitor: forced raw tuser violation"
                    severity error;
            end if;

            if v_tuser_err_inc /= 0 then
                s_raw_tuser_err_cnt <= s_raw_tuser_err_cnt + v_tuser_err_inc;
            end if;

            if s_raw_order_reset = '0' and v_order_err_inc /= 0 then
                s_raw_order_err_cnt <= s_raw_order_err_cnt + v_order_err_inc;
            end if;

            if s_drain_done = '1' then
                pr_info("  @" & nat_img(v_clk_cnt) & " DRAIN_DONE");
            end if;

            -- Debug: periodic status during drain (every 200 clocks)
            if v_clk_cnt mod 200 = 0 and v_clk_cnt > 0 then
                pr_info("  @" & nat_img(v_clk_cnt)
                        & " ef1=" & sl_chr(s_ef1_sync)
                        & " ef2=" & sl_chr(s_ef2_sync)
                        & " lf1=" & sl_chr(s_lf1_sync)
                        & " lf2=" & sl_chr(s_lf2_sync)
                        & " irflag=" & sl_chr(s_irflag_sync)
                        & " busy=" & sl_chr(s_ctrl_busy)
                        & " bus_busy=" & sl_chr(s_bus_busy)
                        & " fill1=" & nat_img(s_fifo1_fill)
                        & " fill2=" & nat_img(s_fifo2_fill));
            end if;
        end if;
    end process p_raw_monitor;

    -- =========================================================================
    -- AluTrigger pulse width measurement
    -- =========================================================================
    p_alu_timing : process
    begin
        wait until s_alutrigger = '1';
        s_alu_rise_time <= now;
        wait until s_alutrigger = '0';
        s_alu_fall_time <= now;
        s_alu_pulse_width <= now - s_alu_rise_time;
    end process p_alu_timing;

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stim : process
        variable v_fail         : natural := 0;
        variable v_found        : boolean;
        variable v_raw_cnt_snap : natural;
        variable v_raw_data_snap : natural;
        variable v_empty_read_snap : natural;
        variable v_drain_words  : natural;
        variable v_t0_cycle     : natural;
        variable v_first_data_cycle : natural;
        variable v_last_data_cycle : natural;
        variable v_run_complete_cycle : natural;
        variable v_drain_done_cycle : natural;
        variable v_first_valid_cycle : natural;
        variable v_ready_low_start_cycle : natural;
        variable v_ready_release_cycle : natural;
        variable v_prev_data_total : natural;
        variable v_prev_ififo1_data_total : natural;
        variable v_prev_ififo2_data_total : natural;
        variable v_prev_ififo1_done_total : natural;
        variable v_prev_final_done_total : natural;
        variable v_meas_data_cnt : natural;
        variable v_meas_ififo1_data_cnt : natural;
        variable v_meas_ififo2_data_cnt : natural;
        variable v_min_ii       : natural;
        variable v_max_ii       : natural;
        variable v_ififo1_first_cycle : natural;
        variable v_ififo1_last_cycle  : natural;
        variable v_ififo2_first_cycle : natural;
        variable v_ififo2_last_cycle  : natural;
        variable v_ififo1_done_cycle  : natural;
        variable v_final_ctrl_cycle   : natural;
        variable v_ififo1_min_ii : natural;
        variable v_ififo1_max_ii : natural;
        variable v_ififo2_min_ii : natural;
        variable v_ififo2_max_ii : natural;
        variable v_faulted_snap : natural;
        variable v_gap          : natural;

        procedure wait_clk(n : natural) is
        begin
            tb_wait_clk(s_clk, n);
        end procedure;

        procedure pulse(signal s : out std_logic) is
        begin
            s <= '1';
            wait_clk(1);
            s <= '0';
        end procedure;

        procedure wait_drain_done(timeout : natural; found : out boolean) is
        begin
            tb_wait_sig_value(s_clk, s_drain_done, '1', timeout, found);
        end procedure;

        procedure wait_ctrl_idle(timeout : natural; found : out boolean) is
        begin
            tb_wait_sig_value(s_clk, s_ctrl_busy, '0', timeout, found);
        end procedure;

        procedure set_expected_unknown is
        begin
            s_expected_ififo1 <= (others => '0');
            s_expected_ififo2 <= (others => '0');
            s_expected_final_valid <= '0';
        end procedure;

        procedure set_expected_counts(n1 : natural; n2 : natural) is
        begin
            s_expected_ififo1 <= to_unsigned(n1, s_expected_ififo1'length);
            s_expected_ififo2 <= to_unsigned(n2, s_expected_ififo2'length);
            s_expected_final_valid <= '1';
        end procedure;

        -- Fill both FIFOs with test data and publish count-known expectations.
        procedure fill_fifos(n1 : natural; n2 : natural) is
        begin
            set_expected_counts(n1, n2);
            s_fifo_load_n1  <= n1;
            s_fifo_load_n2  <= n2;
            s_fifo_load_req <= '1';
            wait_clk(2);
            s_fifo_load_req <= '0';
        end procedure;

        -- Fill FIFOs while modeling an absent echo_receiver. Expected counts
        -- remain zero, so chip_run must use the EF fallback path.
        procedure fill_fifos_unknown(n1 : natural; n2 : natural) is
        begin
            set_expected_unknown;
            s_fifo_load_n1  <= n1;
            s_fifo_load_n2  <= n2;
            s_fifo_load_req <= '1';
            wait_clk(2);
            s_fifo_load_req <= '0';
        end procedure;

        -- Load actual IFIFO fill while publishing a deliberately different
        -- expected count. Used to verify stale-count fault propagation.
        procedure fill_fifos_with_expected(
            actual1 : natural;
            actual2 : natural;
            expected1 : natural;
            expected2 : natural
        ) is
        begin
            set_expected_counts(expected1, expected2);
            s_fifo_load_n1  <= actual1;
            s_fifo_load_n2  <= actual2;
            s_fifo_load_req <= '1';
            wait_clk(2);
            s_fifo_load_req <= '0';
        end procedure;

    begin
        -- =============================================================
        -- Initialize config
        -- =============================================================
        s_cfg <= c_TDC_CFG_INIT;
        s_cfg.bus_clk_div    <= to_unsigned(1, 6);   -- tick_en every clock
        s_cfg.bus_ticks      <= to_unsigned(5, 3);   -- 5 ticks/transaction
        s_cfg.stops_per_chip <= to_unsigned(4, 4);   -- 4 stops
        s_cfg.drain_mode     <= '0';                 -- legacy initially
        s_cfg.n_drain_cap    <= (others => '0');     -- unlimited

        -- Minimal cfg_image (doesn't matter for integration test)
        for i in 0 to 15 loop
            s_cfg_image(i) <= std_logic_vector(to_unsigned(i * 16#1000#, 32));
        end loop;
        -- Reg6 Fill/LF threshold drives chip_run burst length. Use 4 so
        -- burst-mode tests exercise ST_DRAIN_BURST instead of falling back
        -- to single EF reads.
        s_cfg_image(6)(7 downto 0) <= std_logic_vector(to_unsigned(c_LF_THRESHOLD, 8));

        -- =============================================================
        -- Reset
        -- =============================================================
        s_rst_n <= '0';
        wait_clk(10);
        s_rst_n <= '1';
        wait_clk(5);

        pr_sep;
        pr_info("tdc_gpx_chip_ctrl_tb START (integration: chip_ctrl + bus_phy)");
        pr_sep;

        -- =============================================================
        -- [1] Powerup + cfg_write + master_reset
        -- =============================================================
        pr_info("[1] Powerup sequence (PuResN, StopDis, cfg_write, master_reset)");

        -- chip_ctrl starts in ST_POWERUP automatically
        -- Wait for powerup + cfg write + master reset + recovery + stopdis_low
        -- This takes: POWERUP + RECOVERY + 1(STOPDIS_HIGH) +
        --   11 writes * (bus_ticks + overhead) + 1 MR write + RECOVERY + 1
        wait_ctrl_idle(c_TIMEOUT, v_found);

        if v_found then
            pr_pass("[1] Powerup sequence completed, chip_ctrl idle");
        else
            pr_fail("[1] Powerup sequence timeout", v_fail);
        end if;

        -- Verify PuResN released
        if s_puresn = '1' then
            pr_pass("[1] PuResN='1' (released)");
        else
            pr_fail("[1] PuResN should be '1' after powerup", v_fail);
        end if;

        -- Verify StopDis released
        if s_stopdis = '0' then
            pr_pass("[1] StopDis='0' (stops enabled)");
        else
            pr_fail("[1] StopDis should be '0' after init", v_fail);
        end if;

        wait_clk(5);

        if g_NEGATIVE_MODE = 1 then
            pr_info("[N1] NEGATIVE: forced empty IFIFO read monitor");
            set_expected_unknown;
            fill_fifos(0, 0);
            wait_clk(5);

            pulse(s_force_empty_read_req);
            wait_clk(2);

            if s_empty_read_cnt > 0 then
                pr_pass("[N1] empty-read monitor detected forced violation");
            else
                pr_fail("[N1] empty-read monitor did not detect forced violation",
                        v_fail);
            end if;

            pr_print("*** NEGATIVE TEST INTENTIONALLY FAILED: empty-read evidence ***");
            assert false
                report "C02 negative evidence: forced empty IFIFO read"
                severity failure;
            wait;
        elsif g_NEGATIVE_MODE = 2 then
            pr_info("[N2] NEGATIVE: forced raw tuser monitor violation");
            s_force_tuser_err_req <= '1';
            wait_clk(1);
            s_force_tuser_err_req <= '0';
            wait_clk(2);

            if s_raw_tuser_err_cnt > 0 then
                pr_pass("[N2] raw tuser monitor detected forced violation");
            else
                pr_fail("[N2] raw tuser monitor did not detect forced violation",
                        v_fail);
            end if;

            pr_print("*** NEGATIVE TEST INTENTIONALLY FAILED: raw tuser evidence ***");
            assert false
                report "C02 negative evidence: forced raw tuser violation"
                severity failure;
            wait;
        end if;

        -- =============================================================
        -- [2] Legacy drain (drain_mode='0')
        --   Nominal I-Mode single-shot: IFIFO1=28, IFIFO2=28
        --   (4 stop channels per IFIFO x 7 echoes)
        -- =============================================================
        pr_info("[2] Legacy drain (drain_mode='0')");

        s_cfg.drain_mode <= '0';
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);   -- let 2-FF sync settle

        -- ARM
        pulse(s_cmd_start);
        wait_clk(2);

        -- Shot
        pulse(s_shot_start);
        wait_clk(5);

        -- Assert IrFlag (simulates MTimer expiry)
        s_irflag_pin <= '1';
        wait_clk(5);   -- 2-FF sync latency

        -- Record data/empty counters before drain
        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[2] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[2] drain_done received, words=" & nat_img(v_drain_words));

            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[2] drain count exact "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & ", no empty IFIFO reads");
            else
                pr_fail("[2] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        -- Clear IrFlag
        s_irflag_pin <= '0';

        -- Wait for AluTrigger pulse and recovery -> chip_ctrl back to ARMED
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [2b] EF fallback drain when echo_receiver count is absent
        --   Use the same nominal 28/28 physical FIFO load, but keep
        --   expected_final_valid=0 so chip_run must fall back to EF.
        -- =============================================================
        pr_info("[2b] EF fallback drain (expected count absent)");

        s_cfg.drain_mode <= '0';
        fill_fifos_unknown(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[2b] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[2b] EF fallback drained exact "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words without empty read");
            else
                pr_fail("[2b] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " fallback data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [2c] Known-zero expected final completes cleanly with empty EF
        --   Publish final expected=0/0 while the behavioral FIFO is empty.
        --   The controller must trust the final bound, avoid reads, and
        --   finish without a fault.
        -- =============================================================
        pr_info("[2c] Known-zero expected final completes cleanly");

        s_cfg.drain_mode <= '0';
        fill_fifos_with_expected(0, 0, 0, 0);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_t0_cycle := s_clk_cnt;
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;
        v_faulted_snap := s_raw_faulted_ctrl_cnt;

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[2c] drain_done timeout", v_fail);
        else
            v_drain_done_cycle := s_clk_cnt;
            pr_pass("[2c] zero-stop latency measured: output_done="
                    & nat_img(v_drain_done_cycle - v_t0_cycle) & "clk");
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = 0 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[2c] known-zero final produced no IFIFO reads");
            else
                pr_fail("[2c] expected zero reads/no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;

            if s_raw_faulted_ctrl_cnt = v_faulted_snap then
                pr_pass("[2c] known-zero empty shot completed without fault");
            else
                pr_fail("[2c] unexpected fault indication for known-zero empty shot", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [2d] Known-zero expected final blocks EF fallback reads
        --   Publish final expected=0/0 while the behavioral FIFO is
        --   deliberately non-empty. The controller must trust the final
        --   expected bound, avoid reads, and mark the shot as suspect.
        -- =============================================================
        pr_info("[2d] Known-zero expected final blocks EF fallback reads");

        s_cfg.drain_mode <= '0';
        fill_fifos_with_expected(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS, 0, 0);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_t0_cycle := s_clk_cnt;
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;
        v_faulted_snap := s_raw_faulted_ctrl_cnt;

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[2d] drain_done timeout", v_fail);
        else
            v_drain_done_cycle := s_clk_cnt;
            pr_pass("[2d] zero-stop conflict latency measured: output_done="
                    & nat_img(v_drain_done_cycle - v_t0_cycle) & "clk");
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = 0 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[2d] known-zero final produced no IFIFO reads");
            else
                pr_fail("[2d] expected zero reads/no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;

            if s_raw_faulted_ctrl_cnt = v_faulted_snap + 1 then
                pr_pass("[2d] zero-count/EF conflict flagged via raw tuser fault");
            else
                pr_fail("[2d] missing fault indication for zero-count/EF conflict", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [3] Burst drain (drain_mode='1')
        --   Nominal I-Mode single-shot: IFIFO1=28, IFIFO2=28
        --   (4 stop channels per IFIFO x 7 echoes)
        -- =============================================================
        pr_info("[3] Burst drain (drain_mode='1')");

        s_cfg.drain_mode <= '1';
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        -- Shot (already ARMED from [2])
        pulse(s_shot_start);
        wait_clk(5);

        -- Assert IrFlag
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[3] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[3] burst drain_done, words=" & nat_img(v_drain_words));

            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[3] burst drain count exact "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & ", no empty IFIFO reads");
            else
                pr_fail("[3] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [4] n_drain_cap enforcement (cap=2 -> max 8/IFIFO, 16 total)
        --   Load nominal 28/28 but cap at 8/IFIFO.
        -- =============================================================
        pr_info("[4] n_drain_cap enforcement (cap=2, max=8/IFIFO, 16 total)");

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= to_unsigned(2, 4);  -- 2 x 4 = 8 reads per IFIFO

        -- Re-arm with new config (snapshot taken at cmd_start)
        pulse(s_cmd_stop);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);

        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        -- Shot
        pulse(s_shot_start);
        wait_clk(5);

        -- IrFlag
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[4] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[4] drain_done, words=" & nat_img(v_drain_words));

            if v_drain_words = 16 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[4] drain capped exactly at 16 data words, no empty reads");
            else
                pr_fail("[4] expected 16 capped data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        s_cfg.n_drain_cap <= (others => '0');  -- restore unlimited
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [5] Soft reset recovery
        -- =============================================================
        pr_info("[5] Soft reset recovery");

        -- Issue soft reset
        pulse(s_cmd_soft_reset);

        -- Wait for FSM to enter ST_POWERUP (busy goes high)
        tb_wait_sig_value(s_clk, s_ctrl_busy, '1', 10, v_found);

        -- Should go through powerup sequence again
        wait_ctrl_idle(c_TIMEOUT, v_found);

        if v_found then
            pr_pass("[5] Soft reset -> powerup -> idle completed");
        else
            pr_fail("[5] Soft reset recovery timeout", v_fail);
        end if;

        -- Verify clean state (wait for signal propagation)
        wait_clk(2);
        if s_puresn = '1' and s_stopdis = '0' then
            pr_pass("[5] Clean state after soft reset (PuResN=1, StopDis=0)");
        else
            pr_info("[5] PuResN=" & sl_chr(s_puresn)
                    & " StopDis=" & sl_chr(s_stopdis));
            pr_fail("[5] Unexpected state after soft reset", v_fail);
        end if;

        wait_clk(10);

        -- =============================================================
        -- [6] cfg_write register content verification
        -- =============================================================
        pr_info("[6] cfg_write register content verification");

        -- Powerup already ran at [1] and soft reset at [5].
        -- Write captures reflect writes from the most recent powerup ([5]).
        pr_info("[6] Write capture count = " & nat_img(s_wr_capture_cnt));

        if s_wr_capture_cnt >= 12 then
            pr_pass("[6] Write capture count >= 12 (11 cfg + 1 master reset)");
        else
            pr_fail("[6] Expected >= 12 writes, got " & nat_img(s_wr_capture_cnt), v_fail);
        end if;

        -- Verify each register in CFG_WRITE_SEQ: 0..7, 11, 12, 14
        -- For regs other than 4, captured value should match cfg_image lower 28 bits.
        -- Reg4 last write is MasterReset (bit22=1), so skip exact match for Reg4.
        for i in 0 to 7 loop
            if i = 4 then
                -- Reg4: MasterReset write is last, verify bit 22 set
                if s_wr_capture(4)(22) = '1' then
                    pr_pass("[6] Reg4 MasterReset bit22='1'");
                else
                    pr_fail("[6] Reg4 MasterReset bit22 should be '1'", v_fail);
                end if;
            else
                if s_wr_capture(i) = s_cfg_image(i)(c_DATA_W - 1 downto 0) then
                    pr_pass("[6] Reg" & nat_img(i) & " content matches cfg_image");
                else
                    pr_fail("[6] Reg" & nat_img(i) & " mismatch: got 0x"
                            & hex_img(s_wr_capture(i))
                            & " exp 0x" & hex_img(s_cfg_image(i)(c_DATA_W - 1 downto 0)),
                            v_fail);
                end if;
            end if;
        end loop;

        -- Reg11
        if s_wr_capture(11) = s_cfg_image(11)(c_DATA_W - 1 downto 0) then
            pr_pass("[6] Reg11 content matches cfg_image");
        else
            pr_fail("[6] Reg11 mismatch: got 0x" & hex_img(s_wr_capture(11))
                    & " exp 0x" & hex_img(s_cfg_image(11)(c_DATA_W - 1 downto 0)),
                    v_fail);
        end if;

        -- Reg12
        if s_wr_capture(12) = s_cfg_image(12)(c_DATA_W - 1 downto 0) then
            pr_pass("[6] Reg12 content matches cfg_image");
        else
            pr_fail("[6] Reg12 mismatch: got 0x" & hex_img(s_wr_capture(12))
                    & " exp 0x" & hex_img(s_cfg_image(12)(c_DATA_W - 1 downto 0)),
                    v_fail);
        end if;

        -- Reg14
        if s_wr_capture(14) = s_cfg_image(14)(c_DATA_W - 1 downto 0) then
            pr_pass("[6] Reg14 content matches cfg_image");
        else
            pr_fail("[6] Reg14 mismatch: got 0x" & hex_img(s_wr_capture(14))
                    & " exp 0x" & hex_img(s_cfg_image(14)(c_DATA_W - 1 downto 0)),
                    v_fail);
        end if;

        wait_clk(5);

        -- =============================================================
        -- [7] EF=1 -> no reads (INV-4)
        -- =============================================================
        pr_info("[7] EF=1 -> no reads (INV-4)");

        s_cfg.drain_mode <= '0';
        s_cfg.n_drain_cap <= (others => '0');

        -- Fill both FIFOs with 0 (both empty, EF=1)
        fill_fifos(0, 0);
        wait_clk(5);

        -- ARM
        pulse(s_cmd_start);
        wait_clk(2);

        -- Shot
        pulse(s_shot_start);
        wait_clk(5);

        -- Record data count before drain
        v_raw_data_snap := s_raw_data_cnt;

        -- Assert IrFlag — drain_done may fire within 3-4 clocks
        -- (both EF='1' → immediate drain complete), so go straight to wait.
        s_irflag_pin <= '1';

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[7] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = 0 then
                pr_pass("[7] No reads when both FIFOs empty (EF=1)");
            else
                pr_fail("[7] Expected 0 reads with EF=1, got " & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [8] AluTrigger pulse width >= 10ns
        -- =============================================================
        pr_info("[8] AluTrigger pulse width >= 10ns");

        -- Run a shot cycle to generate an AluTrigger pulse
        s_cfg.drain_mode <= '0';
        fill_fifos(1, 1);
        wait_clk(5);

        -- Shot (already ARMED from [7])
        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        -- Assert IrFlag
        s_irflag_pin <= '1';
        wait_clk(5);

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[8] drain_done timeout", v_fail);
        else
            -- Wait for AluTrigger pulse to complete and recovery
            wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

            -- Check measured pulse width
            if s_alu_pulse_width >= 10 ns then
                pr_pass("[8] AluTrigger pulse width = "
                        & time'image(s_alu_pulse_width)
                        & " (>= 10ns)");
            else
                pr_fail("[8] AluTrigger pulse width = "
                        & time'image(s_alu_pulse_width)
                        & " (< 10ns required)", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(10);

        -- =============================================================
        -- [9] IFIFO edge cases (1 entry, 32 entries)
        -- =============================================================
        pr_info("[9] IFIFO edge cases");

        -- [9a] 1 entry per FIFO (minimal non-zero)
        pr_info("[9a] IFIFO 1 entry each");

        s_cfg.drain_mode  <= '0';
        s_cfg.n_drain_cap <= (others => '0');
        fill_fifos(1, 1);
        wait_clk(5);

        -- Shot (already ARMED)
        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[9a] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[9a] drain_done, words=" & nat_img(v_drain_words));

            if v_drain_words = 2 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[9a] drain count exact 2, no empty reads");
            else
                pr_fail("[9a] expected 2 data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- [9b] 32 entries per FIFO (full FIFO depth)
        pr_info("[9b] IFIFO 32 entries each (full depth)");

        s_cfg.drain_mode <= '1';    -- burst for speed
        fill_fifos(c_FIFO_DEPTH, c_FIFO_DEPTH);
        wait_clk(5);

        -- Shot (already ARMED)
        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[9b] drain_done timeout", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[9b] drain_done, words=" & nat_img(v_drain_words));

            if v_drain_words = 64 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[9b] drain count exact 64, no empty reads");
            else
                pr_fail("[9b] expected 64 data words and no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [10] Consecutive 2+ shots
        -- =============================================================
        pr_info("[10] Consecutive 2+ shots");

        s_cfg.drain_mode  <= '0';
        s_cfg.n_drain_cap <= (others => '0');

        -- First shot
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        -- ARM fresh (stop first to go to IDLE, then start)
        pulse(s_cmd_start);
        wait_clk(2);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_cnt_snap := s_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[10] First shot drain_done timeout", v_fail);
        else
            v_drain_words := s_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[10] Shot 1 drain_done, words=" & nat_img(v_drain_words));
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- Second shot (already re-armed automatically)
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_cnt_snap := s_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[10] Second shot drain_done timeout", v_fail);
        else
            v_drain_words := s_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[10] Shot 2 drain_done, words=" & nat_img(v_drain_words));
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- Verify shot_seq incremented (should be >= 2 after two shots in this test)
        pr_info("[10] shot_seq = " & nat_img(to_integer(s_shot_seq)));
        if to_integer(s_shot_seq) >= 2 then
            pr_pass("[10] shot_seq >= 2 after consecutive shots");
        else
            pr_fail("[10] shot_seq should be >= 2, got "
                    & nat_img(to_integer(s_shot_seq)), v_fail);
        end if;

        wait_clk(10);

        -- =============================================================
        -- [11] Bug 5 check: reg read asserts rvalid, reg write does NOT
        -- =============================================================
        pr_info("[11] Reg read -> rvalid=1; Reg write -> rvalid stays 0");

        -- Return to IDLE first (test [10] leaves FSM in ARMED)
        pulse(s_cmd_stop);
        wait_clk(5);

        -- First: individual reg read (use Reg0 so the strict IFIFO-empty
        -- monitor does not treat the command as an IFIFO read).
        s_cmd_reg_addr  <= x"0";
        pulse(s_cmd_reg_read);
        -- Wait for rvalid
        tb_wait_sig_value(s_clk, s_cmd_reg_rvalid, '1', c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[11a] Reg READ -> rvalid asserted");
        else
            pr_fail("[11a] Reg READ -> rvalid timeout", v_fail);
        end if;
        wait_ctrl_idle(c_TIMEOUT, v_found);
        wait_clk(5);

        -- Second: individual reg write (addr 0x00)
        s_cmd_reg_addr  <= x"0";
        s_cmd_reg_wdata <= s_cfg_image(0)(c_DATA_W - 1 downto 0);
        pulse(s_cmd_reg_write);
        -- Wait for idle (write completes)
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[11b] Reg WRITE -> busy timeout", v_fail);
        else
            -- rvalid should NOT have pulsed during the write
            -- (we check it's '0' now; it was default-cleared each cycle)
            if s_cmd_reg_rvalid = '0' then
                pr_pass("[11b] Reg WRITE -> rvalid stays 0 (no STAT11 pollution)");
            else
                pr_fail("[11b] Reg WRITE -> rvalid unexpectedly asserted", v_fail);
            end if;
        end if;
        wait_clk(10);

        -- =============================================================
        -- [12] Bug 2 check: cmd_stop during drain -> returns to IDLE
        -- =============================================================
        pr_info("[12] cmd_stop during drain -> deferred stop, FSM to IDLE");

        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        -- ARM
        pulse(s_cmd_start);
        wait_clk(2);

        -- Shot
        pulse(s_shot_start);
        wait_clk(5);

        -- Assert IrFlag to trigger drain
        s_irflag_pin <= '1';
        wait_clk(10);  -- let drain start

        -- Issue cmd_stop DURING drain (FSM should be in ST_DRAIN_*)
        pulse(s_cmd_stop);

        -- drain_done should still arrive (drain finishes normally)
        wait_drain_done(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[12] drain_done timeout (stop during drain)", v_fail);
        else
            pr_pass("[12] drain_done received despite mid-drain cmd_stop");
        end if;

        s_irflag_pin <= '0';

        -- Wait for ALU pulse + recovery to complete
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 20);

        -- After deferred stop: FSM should be IDLE (busy='0'),
        -- and a new cmd_start should work (proving it's not stuck in ARMED)
        if s_ctrl_busy = '0' then
            pr_pass("[12] busy='0' after deferred stop");
        else
            pr_fail("[12] busy should be '0' after deferred stop", v_fail);
        end if;

        -- StopDis should be '1' (stop-disable asserted by deferred stop)
        if s_stopdis = '1' then
            pr_pass("[12] StopDis='1' (FSM in IDLE, not ARMED)");
        else
            pr_fail("[12] StopDis should be '1' if FSM returned to IDLE", v_fail);
        end if;

        -- Verify we can start a new cycle (proves FSM is in IDLE, not ARMED)
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := s_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := s_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[12] Post-stop restart: drain_done, words=" & nat_img(v_drain_words));
        else
            pr_fail("[12] Post-stop restart: drain_done timeout", v_fail);
        end if;
        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- Stop again cleanly
        pulse(s_cmd_stop);
        wait_clk(10);

        -- =============================================================
        -- [13] cmd_stop during ST_CAPTURE -> graceful drain (Q&A #29 Option A)
        -- =============================================================
        pr_info("[13] cmd_stop in ST_CAPTURE -> graceful drain + ALU + recovery -> IDLE");

        -- Fill FIFOs so there's data to drain (graceful mode preserves it)
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        -- ARM
        pulse(s_cmd_start);
        wait_clk(2);

        -- Shot (enters ST_CAPTURE, but IrFlag NOT asserted yet)
        pulse(s_shot_start);
        wait_clk(5);

        -- Issue cmd_stop DURING ST_CAPTURE (before IrFlag).
        -- Graceful policy: latch stop_pending + stopdis, let natural irflag
        -- drive the normal drain so captured data is preserved. After drain
        -- + ALU_RECOVERY, stop_pending routes FSM to ST_OFF.
        pulse(s_cmd_stop);
        wait_clk(5);

        -- Provide irflag so the current shot drains normally (the chip would
        -- naturally assert this after its internal processing completes).
        s_irflag_pin <= '1';
        wait_clk(10);
        s_irflag_pin <= '0';

        -- Wait for FSM to complete drain + ALU + recovery.
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[13] capture-stop cleanup timeout", v_fail);
        end if;

        -- After cleanup: busy='0', StopDis='1' (FSM in IDLE)
        if s_ctrl_busy = '0' then
            pr_pass("[13] busy='0' after capture-stop cleanup");
        else
            pr_fail("[13] busy should be '0' after capture-stop cleanup", v_fail);
        end if;

        if s_stopdis = '1' then
            pr_pass("[13] StopDis='1' (FSM in IDLE after purge)");
        else
            pr_fail("[13] StopDis should be '1' after capture-stop", v_fail);
        end if;

        -- Verify restart works (FSM really is in IDLE)
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := s_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := s_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[13] Post-capture-stop restart: drain_done, words=" & nat_img(v_drain_words));
        else
            pr_fail("[13] Post-capture-stop restart: drain_done timeout", v_fail);
        end if;
        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- Stop cleanly
        pulse(s_cmd_stop);
        wait_clk(10);

        -- =============================================================
        -- [14] cmd_stop in ST_CAPTURE with NO irflag -> fallback watchdog
        -- (#29 Q&A A graceful stop + fallback)
        -- =============================================================
        pr_info("[14] cmd_stop in ST_CAPTURE without irflag -> fallback watchdog");

        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_cmd_start);
        wait_clk(2);

        pulse(s_shot_start);
        wait_clk(5);

        -- DO NOT raise irflag here — graceful stop latches pending and waits.
        -- The 16-bit fallback watchdog should fire after ~65535 cycles and
        -- fall through to the original purge path with timeout_cause "111".
        pulse(s_cmd_stop);

        -- Long wait: 65535 + drain settle + ALU + recovery margin.
        wait_ctrl_idle(70000, v_found);
        if not v_found then
            pr_fail("[14] fallback watchdog did not complete in 70000 cycles", v_fail);
        else
            pr_pass("[14] fallback watchdog recovered FSM to IDLE");
        end if;

        if s_ctrl_busy = '0' then
            pr_pass("[14] busy='0' after fallback watchdog");
        else
            pr_fail("[14] busy stuck high after fallback watchdog", v_fail);
        end if;

        -- Verify restart works after fallback recovery
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := s_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := s_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[14] Post-fallback restart: drain_done, words="
                    & nat_img(v_drain_words));
        else
            pr_fail("[14] Post-fallback restart: drain_done timeout", v_fail);
        end if;
        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        pulse(s_cmd_stop);
        wait_clk(10);

        -- =============================================================
        -- [15] chip_reg 1-depth pending queue (Round 3 #20/#37).
        -- Issue read + write in the SAME cycle: spec says read wins and
        -- executes first; write is latched into the pending slot and
        -- automatically processed when chip_reg returns to ST_OFF.
        -- Verify both transactions complete (read rvalid pulses, then
        -- FSM returns busy='0' after write finishes).
        -- =============================================================
        pr_info("[15] concurrent reg read+write: read first, write queued");

        -- Make sure we are idle at PH_IDLE
        wait_ctrl_idle(c_TIMEOUT, v_found);
        wait_clk(5);

        -- Same-cycle read + write pulses
        s_cmd_reg_addr   <= x"0";  -- non-IFIFO read target for strict empty-read monitor
        s_cmd_reg_wdata  <= s_cfg_image(0)(c_DATA_W - 1 downto 0);
        s_cmd_reg_read   <= '1';
        s_cmd_reg_write  <= '1';
        wait_clk(1);
        s_cmd_reg_read   <= '0';
        s_cmd_reg_write  <= '0';

        -- Read should fire first and pulse rvalid.
        tb_wait_sig_value(s_clk, s_cmd_reg_rvalid, '1', c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[15] concurrent: read executed first (rvalid asserted)");
        else
            pr_fail("[15] concurrent: read never fired (rvalid timeout)", v_fail);
        end if;

        -- Queued write should then execute; FSM returns to idle when done.
        wait_clk(2);
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[15] concurrent: queued write completed (FSM returned to idle)");
        else
            pr_fail("[15] concurrent: queued write did not complete", v_fail);
        end if;
        wait_clk(10);

        -- =============================================================
        -- [16a] No-backpressure first-data latency measurement
        -- =============================================================
        pr_info("[16a] No raw backpressure first-data latency measurement");

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= (others => '0');
        s_raw_axis_tready <= '1';
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap      := s_raw_data_cnt;
        v_empty_read_snap    := s_empty_read_cnt;
        v_t0_cycle           := s_clk_cnt;
        v_first_valid_cycle  := 0;
        v_first_data_cycle   := 0;
        v_run_complete_cycle := 0;
        v_drain_done_cycle   := 0;
        v_prev_data_total    := s_raw_data_cnt;
        v_found              := false;

        s_irflag_pin <= '1';

        for i in 0 to c_TIMEOUT loop
            wait_clk(1);
            wait for 0 ns;

            if v_first_valid_cycle = 0
               and s_raw_axis_tvalid = '1'
               and s_raw_axis_tuser(7) = '0' then
                v_first_valid_cycle := s_clk_cnt;
            end if;

            if s_raw_data_cnt > v_prev_data_total then
                if v_first_valid_cycle = 0 then
                    v_first_valid_cycle := s_clk_cnt;
                end if;
                if v_first_data_cycle = 0 then
                    v_first_data_cycle := s_clk_cnt;
                end if;
                v_prev_data_total := s_raw_data_cnt;
            end if;

            if v_run_complete_cycle = 0 and s_run_drain_complete = '1' then
                v_run_complete_cycle := s_clk_cnt;
            end if;

            if s_drain_done = '1' then
                v_found := true;
                v_drain_done_cycle := s_clk_cnt;
                exit;
            end if;
        end loop;

        if not v_found then
            pr_fail("[16a] drain_done timeout without raw backpressure", v_fail);
        else
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[16a] data count exact "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " and no empty IFIFO reads");
            else
                pr_fail("[16a] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words/no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;

            if v_first_valid_cycle /= 0 and v_first_data_cycle /= 0
               and v_run_complete_cycle /= 0 and v_drain_done_cycle /= 0 then
                pr_pass("[16a] no-backpressure timing: first_valid="
                        & nat_img(v_first_valid_cycle - v_t0_cycle)
                        & "clk, first_accept="
                        & nat_img(v_first_data_cycle - v_t0_cycle)
                        & "clk, valid_to_accept="
                        & nat_img(v_first_data_cycle - v_first_valid_cycle)
                        & "clk, run_complete="
                        & nat_img(v_run_complete_cycle - v_t0_cycle)
                        & "clk, output_done="
                        & nat_img(v_drain_done_cycle - v_t0_cycle)
                        & "clk");
            else
                pr_fail("[16a] first-data timing measurement incomplete", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        pulse(s_cmd_stop);
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[16a] stop-to-idle timeout before bounded test", v_fail);
        end if;
        wait_clk(10);

        -- =============================================================
        -- [16b] Bounded raw AXI backpressure + latency/II measurement
        -- =============================================================
        pr_info("[16b] Bounded raw AXI backpressure + latency/II measurement");

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= (others => '0');
        s_raw_axis_tready <= '1';
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap      := s_raw_data_cnt;
        v_empty_read_snap    := s_empty_read_cnt;
        v_t0_cycle           := s_clk_cnt;
        v_first_valid_cycle  := 0;
        v_first_data_cycle   := 0;
        v_last_data_cycle    := 0;
        v_run_complete_cycle := 0;
        v_drain_done_cycle   := 0;
        v_ready_low_start_cycle := 0;
        v_ready_release_cycle := 0;
        v_prev_data_total    := s_raw_data_cnt;
        v_prev_ififo1_data_total := s_raw_ififo1_data_cnt;
        v_prev_ififo2_data_total := s_raw_ififo2_data_cnt;
        v_prev_ififo1_done_total := s_raw_ififo1_done_ctrl_cnt;
        v_prev_final_done_total  := s_raw_final_done_ctrl_cnt;
        v_meas_data_cnt      := 0;
        v_meas_ififo1_data_cnt := 0;
        v_meas_ififo2_data_cnt := 0;
        v_min_ii             := 999999;
        v_max_ii             := 0;
        v_ififo1_first_cycle := 0;
        v_ififo1_last_cycle  := 0;
        v_ififo2_first_cycle := 0;
        v_ififo2_last_cycle  := 0;
        v_ififo1_done_cycle  := 0;
        v_final_ctrl_cycle   := 0;
        v_ififo1_min_ii      := 999999;
        v_ififo1_max_ii      := 0;
        v_ififo2_min_ii      := 999999;
        v_ififo2_max_ii      := 0;
        v_found              := false;

        s_irflag_pin <= '1';

        for i in 0 to c_TIMEOUT loop
            -- Hold downstream ready low long enough to prove the raw FIFO
            -- absorbs bounded stalls, but below the intentional data-drop
            -- threshold used to reserve slots for control beats.
            if i >= 8 and i < 38 then
                s_raw_axis_tready <= '0';
            else
                s_raw_axis_tready <= '1';
            end if;

            wait_clk(1);
            wait for 0 ns;

            if i = 8 and v_ready_low_start_cycle = 0 then
                v_ready_low_start_cycle := s_clk_cnt;
            elsif i = 38 and v_ready_release_cycle = 0 then
                v_ready_release_cycle := s_clk_cnt;
            end if;

            if v_first_valid_cycle = 0
               and s_raw_axis_tvalid = '1'
               and s_raw_axis_tuser(7) = '0' then
                v_first_valid_cycle := s_clk_cnt;
            end if;

            if s_raw_data_cnt > v_prev_data_total then
                v_meas_data_cnt := v_meas_data_cnt + 1;
                if v_first_data_cycle = 0 then
                    v_first_data_cycle := s_clk_cnt;
                end if;
                if v_last_data_cycle /= 0 then
                    v_gap := s_clk_cnt - v_last_data_cycle;
                    if v_gap < v_min_ii then
                        v_min_ii := v_gap;
                    end if;
                    if v_gap > v_max_ii then
                        v_max_ii := v_gap;
                    end if;
                end if;
                v_last_data_cycle := s_clk_cnt;
                v_prev_data_total := s_raw_data_cnt;
            end if;

            if s_raw_ififo1_data_cnt > v_prev_ififo1_data_total then
                v_meas_ififo1_data_cnt := v_meas_ififo1_data_cnt
                    + (s_raw_ififo1_data_cnt - v_prev_ififo1_data_total);
                if v_ififo1_first_cycle = 0 then
                    v_ififo1_first_cycle := s_clk_cnt;
                end if;
                if v_ififo1_last_cycle /= 0 then
                    v_gap := s_clk_cnt - v_ififo1_last_cycle;
                    if v_gap < v_ififo1_min_ii then
                        v_ififo1_min_ii := v_gap;
                    end if;
                    if v_gap > v_ififo1_max_ii then
                        v_ififo1_max_ii := v_gap;
                    end if;
                end if;
                v_ififo1_last_cycle := s_clk_cnt;
                v_prev_ififo1_data_total := s_raw_ififo1_data_cnt;
            end if;

            if s_raw_ififo2_data_cnt > v_prev_ififo2_data_total then
                v_meas_ififo2_data_cnt := v_meas_ififo2_data_cnt
                    + (s_raw_ififo2_data_cnt - v_prev_ififo2_data_total);
                if v_ififo2_first_cycle = 0 then
                    v_ififo2_first_cycle := s_clk_cnt;
                end if;
                if v_ififo2_last_cycle /= 0 then
                    v_gap := s_clk_cnt - v_ififo2_last_cycle;
                    if v_gap < v_ififo2_min_ii then
                        v_ififo2_min_ii := v_gap;
                    end if;
                    if v_gap > v_ififo2_max_ii then
                        v_ififo2_max_ii := v_gap;
                    end if;
                end if;
                v_ififo2_last_cycle := s_clk_cnt;
                v_prev_ififo2_data_total := s_raw_ififo2_data_cnt;
            end if;

            if s_raw_ififo1_done_ctrl_cnt > v_prev_ififo1_done_total then
                if v_ififo1_done_cycle = 0 then
                    v_ififo1_done_cycle := s_clk_cnt;
                end if;
                v_prev_ififo1_done_total := s_raw_ififo1_done_ctrl_cnt;
            end if;

            if s_raw_final_done_ctrl_cnt > v_prev_final_done_total then
                if v_final_ctrl_cycle = 0 then
                    v_final_ctrl_cycle := s_clk_cnt;
                end if;
                v_prev_final_done_total := s_raw_final_done_ctrl_cnt;
            end if;

            if v_run_complete_cycle = 0 and s_run_drain_complete = '1' then
                v_run_complete_cycle := s_clk_cnt;
            end if;

            if s_drain_done = '1' then
                v_found := true;
                v_drain_done_cycle := s_clk_cnt;
                exit;
            end if;
        end loop;

        s_raw_axis_tready <= '1';

        if not v_found then
            pr_fail("[16b] drain_done timeout under bounded raw backpressure", v_fail);
        else
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            pr_pass("[16b] drain_done received under bounded raw backpressure, words="
                    & nat_img(v_drain_words));

            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and v_meas_data_cnt = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[16b] data count exact "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " and no empty IFIFO reads");
            else
                pr_fail("[16b] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words/no empty reads, got "
                        & nat_img(v_drain_words)
                        & " measured=" & nat_img(v_meas_data_cnt), v_fail);
            end if;

            if s_err_raw_drop = '0' and s_err_raw_overflow = '0' then
                pr_pass("[16b] raw FIFO absorbed bounded backpressure without drop");
            else
                pr_fail("[16b] raw drop/overflow set during bounded backpressure", v_fail);
            end if;

            if v_first_data_cycle /= 0 and v_run_complete_cycle /= 0
               and v_drain_done_cycle /= 0 and v_meas_data_cnt > 1 then
                pr_pass("[16b] latency/II measured: first_accept="
                        & nat_img(v_first_data_cycle - v_t0_cycle)
                        & "clk, run_complete="
                        & nat_img(v_run_complete_cycle - v_t0_cycle)
                        & "clk, output_done="
                        & nat_img(v_drain_done_cycle - v_t0_cycle)
                        & "clk, output_hold="
                        & nat_img(v_drain_done_cycle - v_run_complete_cycle)
                        & "clk, II_min=" & nat_img(v_min_ii)
                        & "clk, II_max=" & nat_img(v_max_ii) & "clk");
                if v_first_valid_cycle /= 0 and v_ready_low_start_cycle /= 0
                   and v_ready_release_cycle /= 0 then
                    pr_pass("[16b] first-data split: first_valid="
                            & nat_img(v_first_valid_cycle - v_t0_cycle)
                            & "clk, first_accept="
                            & nat_img(v_first_data_cycle - v_t0_cycle)
                            & "clk, valid_to_accept="
                            & nat_img(v_first_data_cycle - v_first_valid_cycle)
                            & "clk, ready_low_start="
                            & nat_img(v_ready_low_start_cycle - v_t0_cycle)
                            & "clk, ready_release="
                            & nat_img(v_ready_release_cycle - v_t0_cycle)
                            & "clk, release_to_accept="
                            & nat_img(v_first_data_cycle - v_ready_release_cycle)
                            & "clk");
                else
                    pr_fail("[16b] first-valid/backpressure timing incomplete",
                            v_fail);
                end if;

                pr_pass("[16b] segmented IFIFO1: words="
                        & nat_img(v_meas_ififo1_data_cnt)
                        & ", first=" & nat_img(v_ififo1_first_cycle - v_t0_cycle)
                        & "clk, last=" & nat_img(v_ififo1_last_cycle - v_t0_cycle)
                        & "clk, done_ctrl=" & nat_img(v_ififo1_done_cycle - v_t0_cycle)
                        & "clk, II_min=" & nat_img(v_ififo1_min_ii)
                        & "clk, II_max=" & nat_img(v_ififo1_max_ii) & "clk");
                pr_pass("[16b] segmented IFIFO2: words="
                        & nat_img(v_meas_ififo2_data_cnt)
                        & ", first=" & nat_img(v_ififo2_first_cycle - v_t0_cycle)
                        & "clk, last=" & nat_img(v_ififo2_last_cycle - v_t0_cycle)
                        & "clk, final_ctrl=" & nat_img(v_final_ctrl_cycle - v_t0_cycle)
                        & "clk, II_min=" & nat_img(v_ififo2_min_ii)
                        & "clk, II_max=" & nat_img(v_ififo2_max_ii) & "clk");
                pr_pass("[16b] segmented gaps: ififo1_last_to_done="
                        & nat_img(v_ififo1_done_cycle - v_ififo1_last_cycle)
                        & "clk, ififo1_done_to_ififo2_first="
                        & nat_img(v_ififo2_first_cycle - v_ififo1_done_cycle)
                        & "clk, ififo2_last_to_final="
                        & nat_img(v_final_ctrl_cycle - v_ififo2_last_cycle)
                        & "clk, final_to_output_done="
                        & nat_img(v_drain_done_cycle - v_final_ctrl_cycle)
                        & "clk");
            else
                pr_fail("[16b] latency/II measurement incomplete", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [16c] Reserve-threshold raw AXI backpressure
        -- =============================================================
        -- [16b] proves a short bounded stall. This scenario deliberately
        -- holds tready low long enough to reach the FIFO's source-busy
        -- threshold. chip_ctrl must stop accepting GPX responses, retain all
        -- data already in flight, and later emit both control beats in order.
        pr_info("[16c] Raw FIFO reserve-threshold backpressure");

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= (others => '0');
        s_raw_axis_tready <= '0';
        fill_fifos(c_IFIFO_NOMINAL_WORDS, c_IFIFO_NOMINAL_WORDS);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap   := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;
        v_found           := false;
        s_raw_order_reset <= '1';
        wait_clk(1);
        s_raw_order_reset <= '0';
        s_raw_order_check_en <= '1';
        s_irflag_pin      <= '1';

        for i in 0 to c_TIMEOUT loop
            if i = 80 then
                s_raw_axis_tready <= '1';
            end if;

            wait_clk(1);
            wait for 0 ns;

            if s_drain_done = '1' then
                v_found := true;
                exit;
            end if;
        end loop;

        s_raw_axis_tready <= '1';

        if not v_found then
            pr_fail("[16c] drain_done timeout after reserve-threshold stall",
                    v_fail);
        else
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = c_IFIFO_NOMINAL_TOTAL
               and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[16c] all " & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words preserved across threshold stall");
            else
                pr_fail("[16c] expected "
                        & nat_img(c_IFIFO_NOMINAL_TOTAL)
                        & " data words/no empty reads, got "
                        & nat_img(v_drain_words), v_fail);
            end if;

            if s_err_raw_drop = '0' and s_err_raw_overflow = '0' then
                pr_pass("[16c] source backpressure prevented data/control drop");
            else
                pr_fail("[16c] raw drop set at reserve threshold", v_fail);
            end if;

            if s_raw_order_err_cnt = 0 then
                pr_pass("[16c] payload and IFIFO/control order preserved");
            else
                pr_fail("[16c] payload/control order mismatch", v_fail);
            end if;
        end if;

        s_raw_order_check_en <= '0';
        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [17] Stale expected-count mismatch fault propagation
        -- =============================================================
        pr_info("[17] Stale expected-count mismatch -> faulted drain_done");

        s_cfg.drain_mode  <= '0';
        s_cfg.n_drain_cap <= (others => '0');
        s_raw_axis_tready <= '1';
        fill_fifos_with_expected(2, 0, 4, 1);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_data_snap   := s_raw_data_cnt;
        v_empty_read_snap := s_empty_read_cnt;
        v_faulted_snap    := s_raw_faulted_ctrl_cnt;

        s_irflag_pin <= '1';
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[17] drain_done timeout for stale expected-count case", v_fail);
        else
            wait_clk(1);
            v_drain_words := s_raw_data_cnt - v_raw_data_snap;
            if v_drain_words = 2 and s_empty_read_cnt = v_empty_read_snap then
                pr_pass("[17] stale expected count stopped at EF without empty read");
            else
                pr_fail("[17] expected 2 actual words and no empty read, got "
                        & nat_img(v_drain_words), v_fail);
            end if;

            if s_raw_faulted_ctrl_cnt = v_faulted_snap + 1 then
                pr_pass("[17] mismatch propagated via raw tuser fault flag");
            else
                pr_fail("[17] missing expected drain mismatch/faulted indication", v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [18] C02 global monitors: empty read and raw tuser contract
        -- =============================================================
        pr_info("[18] C02 global monitor summary");
        if s_empty_read_cnt = 0 then
            pr_pass("[18] No IFIFO read occurred when the modeled FIFO was empty");
        else
            pr_fail("[18] Empty IFIFO read count="
                    & nat_img(s_empty_read_cnt), v_fail);
        end if;

        if s_raw_tuser_err_cnt = 0 then
            pr_pass("[18] Raw AXI tuser contract clean");
        else
            pr_fail("[18] Raw AXI tuser error count="
                    & nat_img(s_raw_tuser_err_cnt), v_fail);
        end if;

        if s_raw_ififo1_done_ctrl_cnt > 0
           and s_raw_final_done_ctrl_cnt > 0 then
            pr_pass("[18] Raw control beat IDs observed: ififo1_done="
                    & nat_img(s_raw_ififo1_done_ctrl_cnt)
                    & " final_done="
                    & nat_img(s_raw_final_done_ctrl_cnt));
        else
            pr_fail("[18] Missing raw control beat ID class: ififo1_done="
                    & nat_img(s_raw_ififo1_done_ctrl_cnt)
                    & " final_done="
                    & nat_img(s_raw_final_done_ctrl_cnt), v_fail);
        end if;

        if s_err_raw_drop = '0' and s_err_drain_cap = '0'
           and s_err_bus_fatal = '0' then
            pr_pass("[18] Raw/drop and PH_RESP_DRAIN fatal indicators clean");
        else
            pr_fail("[18] Unexpected raw/drop or bus fatal indicator set", v_fail);
        end if;

        -- =============================================================
        -- [19] PH_RESP_DRAIN stuck/fatal quarantine and auto-recover
        -- =============================================================
        pr_info("[19] PH_RESP_DRAIN stuck/fatal quarantine + auto-recover");

        if s_ctrl_busy = '0' then
            pulse(s_cmd_start);
            wait_clk(2);
        else
            pr_info("[19] chip_ctrl already armed/running before stuck-response test");
        end if;

        s_cfg.drain_mode <= '0';
        s_raw_axis_tready <= '1';
        fill_fifos_with_expected(0, 0, 0, 0);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        s_force_resp_drain_stuck <= '1';
        s_irflag_pin <= '1';
        wait_drain_done(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[19] drain_done timeout before PH_RESP_DRAIN stuck phase",
                    v_fail);
        else
            pr_pass("[19] drain_done observed before forced PH_RESP_DRAIN stuck");
        end if;
        s_irflag_pin <= '0';
        pulse(s_cmd_stop);

        tb_wait_sig_value(s_clk, s_err_drain_cap, '1', 200, v_found);
        if v_found then
            pr_pass("[19] PH_RESP_DRAIN hard cap sticky asserted");
        else
            pr_fail("[19] PH_RESP_DRAIN hard cap sticky did not assert",
                    v_fail);
        end if;

        tb_wait_sig_value(s_clk, s_err_bus_fatal, '1', 70000, v_found);
        if v_found then
            pr_pass("[19] PH_RESP_DRAIN quarantine escalated to bus fatal");
        else
            pr_fail("[19] PH_RESP_DRAIN bus fatal did not assert", v_fail);
        end if;

        s_force_resp_drain_stuck <= '0';
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[19] chip_ctrl returned to idle after PH_RESP_DRAIN recovery");
        else
            pr_fail("[19] chip_ctrl did not return to idle after recovery",
                    v_fail);
        end if;

        -- =============================================================
        -- [20] Forced pending response -> secondary deadlock watchdog
        -- =============================================================
        pr_info("[20] Forced pending response -> secondary watchdog timeout");

        -- Start from a clean controller because [19] intentionally raised
        -- the response-drain quarantine diagnostics.
        pulse(s_cmd_soft_reset);
        wait_ctrl_idle(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[20] soft-reset setup did not return to idle", v_fail);
        end if;

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= (others => '0');
        s_raw_axis_tready <= '1';
        fill_fifos_with_expected(1, 0, 1, 0);
        wait_clk(5);

        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        -- Hide the bus response from chip_ctrl while presenting a persistent
        -- pending level. This isolates the wait-state secondary watchdog from
        -- the shorter ST_DRAIN_CHECK raw-busy watchdog.
        s_force_bus_rsp_pending <= '1';
        s_force_bus_rsp_hide    <= '1';
        s_irflag_pin <= '1';
        v_t0_cycle := s_clk_cnt;

        tb_wait_sig_value(s_clk, s_run_timeout, '1', 70000, v_found);
        wait for 0 ns;
        if not v_found then
            pr_fail("[20] pending-response watchdog did not fire", v_fail);
        elsif s_run_timeout_cause /= "001" then
            pr_fail("[20] wrong pending-response timeout cause=0x"
                    & hex_img(s_run_timeout_cause), v_fail);
        elsif s_clk_cnt - v_t0_cycle < 65530
           or s_clk_cnt - v_t0_cycle > 65750 then
            pr_fail("[20] pending watchdog fired outside expected window: "
                    & nat_img(s_clk_cnt - v_t0_cycle) & "clk", v_fail);
        else
            pr_pass("[20] pending watchdog fired with cause 001 after "
                    & nat_img(s_clk_cnt - v_t0_cycle) & "clk");
        end if;

        s_force_bus_rsp_pending <= '0';
        s_force_bus_rsp_hide    <= '0';
        wait_drain_done(200, v_found);
        if v_found then
            pr_pass("[20] timeout completion reached raw drain_done output");
        else
            pr_fail("[20] timeout completion did not reach drain_done", v_fail);
        end if;

        s_irflag_pin <= '0';
        pulse(s_cmd_stop);
        wait_ctrl_idle(5000, v_found);
        if v_found then
            pr_pass("[20] controller recovered after releasing backpressure");
        else
            pr_fail("[20] controller did not recover after timeout", v_fail);
        end if;

        -- =============================================================
        -- Summary
        -- =============================================================
        pr_sep;
        if v_fail = 0 then
            pr_print("*** ALL TESTS PASSED *** (total_raw_words="
                     & nat_img(s_raw_word_cnt) & ")");
        else
            pr_print("*** " & nat_img(v_fail) & " TEST(S) FAILED ***");
        end if;
        pr_sep;

        assert v_fail = 0
            report "tb_tdc_gpx_chip_ctrl detected test failure(s)"
            severity failure;

        s_sim_done <= '1';
        wait;
    end process p_stim;

end architecture sim;
