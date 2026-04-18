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
--   [3] Arm -> shot_start -> IrFlag -> Burst drain (drain_mode='1')
--   [4] n_drain_cap enforcement (per-IFIFO cap=n×4, e.g. cap=2 -> max 8/IFIFO)
--   [5] Soft reset recovery
--   [6] cfg_write register content verification
--   [7] EF=1 -> no reads (INV-4)
--   [8] AluTrigger pulse width >= 10ns
--   [9] IFIFO edge cases (1 entry, 32 entries)
--   [10] Consecutive 2+ shots
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
end entity tb_tdc_gpx_chip_ctrl;

architecture sim of tb_tdc_gpx_chip_ctrl is

    -- =========================================================================
    -- TB constants
    -- =========================================================================
    constant c_CLK_PERIOD       : time    := 5 ns;       -- 200 MHz
    constant c_DATA_W           : natural := c_TDC_BUS_WIDTH;  -- 28
    constant c_CHIP_ID          : natural := 0;

    -- Powerup/recovery generics (short for sim speed)
    constant c_POWERUP_CLKS     : natural := 10;
    constant c_RECOVERY_CLKS    : natural := 4;
    constant c_ALU_PULSE_CLKS   : natural := 3;  -- 3 ticks: 2 clks high = 10ns @ 200MHz

    -- Timeout
    constant c_TIMEOUT          : natural := 2000;

    -- IFIFO model: number of words per IFIFO
    constant c_FIFO_DEPTH       : natural := 32;
    -- LF threshold: LF='1' when fill >= threshold
    constant c_LF_THRESHOLD     : natural := 4;

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
    signal s_tick_en            : std_logic;

    -- bus_phy → chip_ctrl AXI-Stream (response)
    signal s_brsp_axis_tvalid   : std_logic;
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
    -- Monitor: raw_word output capture
    -- =========================================================================
    signal sv_raw_word_cnt      : natural := 0;

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

begin

    -- =========================================================================
    -- Clock
    -- =========================================================================
    s_clk <= not s_clk after c_CLK_PERIOD / 2 when s_sim_done = '0' else s_clk;

    -- =========================================================================
    -- DUT: chip_ctrl
    -- =========================================================================
    u_chip_ctrl : entity work.tdc_gpx_chip_ctrl
        generic map (
            g_CHIP_ID        => c_CHIP_ID,
            g_POWERUP_CLKS   => c_POWERUP_CLKS,
            g_RECOVERY_CLKS  => c_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS => c_ALU_PULSE_CLKS
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
            i_max_range_clks    => s_cfg.max_range_clks,
            i_stop_tdc          => s_stop_tdc,
            i_expected_ififo1   => (others => '0'),  -- TB: 0 = EF-based fallback
            i_expected_ififo2   => (others => '0'),
            o_bus_req_valid     => s_bus_req_valid,
            o_bus_req_rw        => s_bus_req_rw,
            o_bus_req_addr      => s_bus_req_addr,
            o_bus_req_wdata     => s_bus_req_wdata,
            o_bus_oen_permanent => s_bus_oen_perm,
            o_bus_req_burst     => s_bus_req_burst,
            o_bus_ticks_snap    => open,
            i_s_axis_tvalid     => s_brsp_axis_tvalid,
            i_s_axis_tdata      => s_brsp_axis_tdata,
            i_s_axis_tuser      => s_brsp_axis_tuser,
            o_s_axis_tready     => s_brsp_axis_tready,
            i_bus_busy          => s_bus_busy,
            i_bus_rsp_pending   => '0',
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
            o_err_raw_overflow  => open,
            o_err_reg_overflow  => open,
            o_run_timeout       => open
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
            o_adr           => s_adr,
            o_csn           => s_csn,
            o_rdn           => s_rdn,
            o_wrn           => s_wrn,
            o_oen           => s_oen,
            io_d            => s_io_d,
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

            if s_oen = '0' and s_rdn = '0' then
                s_chip_d_oe <= '1';

                -- Read data: encode FIFO ID + counter in data word
                if s_adr = c_TDC_REG8_IFIFO1 then
                    -- IFIFO1 data: 0x8_xxxxxx (address in [27:24], counter in [23:0])
                    s_chip_d_out <= x"8" & std_logic_vector(
                        to_unsigned(s_fifo1_rd_cnt, 24));
                elsif s_adr = c_TDC_REG9_IFIFO2 then
                    -- IFIFO2 data: 0x9_xxxxxx
                    s_chip_d_out <= x"9" & std_logic_vector(
                        to_unsigned(s_fifo2_rd_cnt, 24));
                else
                    s_chip_d_out <= (others => '0');
                end if;
            end if;

            -- On RDN rising edge (end of read strobe): update counters & fill
            -- Note: CSN may go high simultaneously with RDN at Phase H,
            -- so we only check RDN edge + address (address is stable).
            if s_rdn = '1' and v_rdn_prev = '0' then
                if s_adr = c_TDC_REG8_IFIFO1 and s_fifo1_fill > 0 then
                    s_fifo1_fill   <= s_fifo1_fill - 1;
                    s_fifo1_rd_cnt <= s_fifo1_rd_cnt + 1;
                elsif s_adr = c_TDC_REG9_IFIFO2 and s_fifo2_fill > 0 then
                    s_fifo2_fill   <= s_fifo2_fill - 1;
                    s_fifo2_rd_cnt <= s_fifo2_rd_cnt + 1;
                end if;
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
    s_io_d <= s_chip_d_out when s_chip_d_oe = '1' else (others => 'Z');

    -- =========================================================================
    -- Raw word monitor + debug trace
    -- =========================================================================
    p_raw_monitor : process(s_clk)
        variable v_clk_cnt : natural := 0;
    begin
        if rising_edge(s_clk) then
            v_clk_cnt := v_clk_cnt + 1;

            if s_raw_axis_tvalid = '1' then
                sv_raw_word_cnt <= sv_raw_word_cnt + 1;
                pr_info("  @" & nat_img(v_clk_cnt)
                        & " raw_word=0x" & hex_img(s_raw_axis_tdata(c_DATA_W - 1 downto 0))
                        & " ififo=" & sl_chr(s_raw_axis_tuser(0)));
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
        variable v_drain_words  : natural;

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

        -- Fill both FIFOs with test data (via load request to chip model)
        procedure fill_fifos(n1 : natural; n2 : natural) is
        begin
            s_fifo_load_n1  <= n1;
            s_fifo_load_n2  <= n2;
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

        -- =============================================================
        -- [2] Legacy drain (drain_mode='0')
        --   Fill FIFO1=8, FIFO2=4 -> drain all 12 words
        -- =============================================================
        pr_info("[2] Legacy drain (drain_mode='0')");

        s_cfg.drain_mode <= '0';
        fill_fifos(8, 4);
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

        -- Record raw_word count before drain
        v_raw_cnt_snap := sv_raw_word_cnt;

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[2] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[2] drain_done received, words=" & nat_img(v_drain_words));

            -- 2-FF sync latency on EF causes 1-2 extra reads per IFIFO
            -- after the FIFO empties (sync delay before EF='1' is visible).
            -- Expect 12..16 words (12 actual + up to 4 sync-latency reads).
            if v_drain_words >= 12 and v_drain_words <= 16 then
                pr_pass("[2] drain count in range 12..16 (sync overhead ok)");
            else
                pr_fail("[2] expected 12..16 words, got " & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        -- Clear IrFlag
        s_irflag_pin <= '0';

        -- Wait for AluTrigger pulse and recovery -> chip_ctrl back to ARMED
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [3] Burst drain (drain_mode='1')
        --   Fill FIFO1=16, FIFO2=8 -> burst read all 24 words
        -- =============================================================
        pr_info("[3] Burst drain (drain_mode='1')");

        s_cfg.drain_mode <= '1';
        fill_fifos(16, 8);
        wait_clk(5);

        -- Shot (already ARMED from [2])
        pulse(s_shot_start);
        wait_clk(5);

        -- Assert IrFlag
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[3] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[3] burst drain_done, words=" & nat_img(v_drain_words));

            -- Burst drain includes: burst reads + flush in-flight +
            -- EF/LF sync latency overhead. Expect 24..28.
            if v_drain_words >= 24 and v_drain_words <= 28 then
                pr_pass("[3] burst drain count in range 24..28 (sync overhead ok)");
            else
                pr_fail("[3] expected 24..28 words, got " & nat_img(v_drain_words), v_fail);
            end if;
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- =============================================================
        -- [4] n_drain_cap enforcement (cap=2 -> max 16 reads)
        --   Fill FIFO1=16, FIFO2=16 (32 total) but cap at 16
        -- =============================================================
        pr_info("[4] n_drain_cap enforcement (cap=2, max=16 reads)");

        s_cfg.drain_mode  <= '1';
        s_cfg.n_drain_cap <= to_unsigned(2, 4);  -- 2 × 8 = 16 max reads

        -- Re-arm with new config (snapshot taken at cmd_start)
        pulse(s_cmd_stop);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);

        fill_fifos(16, 16);
        wait_clk(5);

        -- Shot
        pulse(s_shot_start);
        wait_clk(5);

        -- IrFlag
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[4] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[4] drain_done, words=" & nat_img(v_drain_words));

            -- Cap=16 + up to 2 in-flight/flush reads = max 18
            if v_drain_words <= 18 then
                pr_pass("[4] drain capped correctly (<= 18, cap=16+flush)");
            else
                pr_fail("[4] expected <= 18 words, got " & nat_img(v_drain_words), v_fail);
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

        -- Record raw count before drain
        v_raw_cnt_snap := sv_raw_word_cnt;

        -- Assert IrFlag — drain_done may fire within 3-4 clocks
        -- (both EF='1' → immediate drain complete), so go straight to wait.
        s_irflag_pin <= '1';

        -- Wait for drain_done
        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[7] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
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

        v_raw_cnt_snap := sv_raw_word_cnt;

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

        v_raw_cnt_snap := sv_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[9a] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[9a] drain_done, words=" & nat_img(v_drain_words));

            -- 2 actual + up to 4 sync-latency reads
            if v_drain_words >= 2 and v_drain_words <= 6 then
                pr_pass("[9a] drain count in range 2..6");
            else
                pr_fail("[9a] expected 2..6 words, got " & nat_img(v_drain_words), v_fail);
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

        v_raw_cnt_snap := sv_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[9b] drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[9b] drain_done, words=" & nat_img(v_drain_words));

            -- 64 actual + up to 4 sync-latency reads
            if v_drain_words >= 64 and v_drain_words <= 68 then
                pr_pass("[9b] drain count in range 64..68");
            else
                pr_fail("[9b] expected 64..68 words, got " & nat_img(v_drain_words), v_fail);
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
        fill_fifos(4, 4);
        wait_clk(5);

        -- ARM fresh (stop first to go to IDLE, then start)
        pulse(s_cmd_start);
        wait_clk(2);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[10] First shot drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
            pr_pass("[10] Shot 1 drain_done, words=" & nat_img(v_drain_words));
        end if;

        s_irflag_pin <= '0';
        wait_clk(c_ALU_PULSE_CLKS + c_RECOVERY_CLKS + 10);

        -- Second shot (already re-armed automatically)
        fill_fifos(4, 4);
        wait_clk(5);

        pulse(s_shot_start);
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;

        s_irflag_pin <= '1';
        wait_clk(5);

        wait_drain_done(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[10] Second shot drain_done timeout", v_fail);
        else
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
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

        -- First: individual reg read (addr 0x08 = read-only status reg)
        s_cmd_reg_addr  <= x"8";
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

        fill_fifos(16, 8);
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
        fill_fifos(4, 4);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
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
        fill_fifos(8, 4);
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
        fill_fifos(4, 4);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
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

        fill_fifos(8, 4);
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
        fill_fifos(4, 4);
        wait_clk(5);
        pulse(s_cmd_start);
        wait_clk(2);
        pulse(s_shot_start);
        wait_clk(5);
        s_irflag_pin <= '1';
        wait_clk(5);

        v_raw_cnt_snap := sv_raw_word_cnt;
        wait_drain_done(c_TIMEOUT, v_found);
        if v_found then
            v_drain_words := sv_raw_word_cnt - v_raw_cnt_snap;
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
        s_cmd_reg_addr   <= x"8";  -- read target: status/read-only
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
        -- Summary
        -- =============================================================
        pr_sep;
        if v_fail = 0 then
            pr_print("*** ALL TESTS PASSED *** (total_raw_words="
                     & nat_img(sv_raw_word_cnt) & ")");
        else
            pr_print("*** " & nat_img(v_fail) & " TEST(S) FAILED ***");
        end if;
        pr_sep;

        s_sim_done <= '1';
        wait;
    end process p_stim;

end architecture sim;
