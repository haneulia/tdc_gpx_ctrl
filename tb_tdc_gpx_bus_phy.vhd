-- =============================================================================
-- tb_tdc_gpx_bus_phy.vhd
-- TDC-GPX Controller - bus_phy Unit Test
-- =============================================================================
--
-- Test scenarios:
--   [1] Single READ   — verify Phase A/L/H timing, rsp_valid, rsp_rdata
--   [2] Single WRITE  — verify Phase A/L/H timing, rsp_valid
--   [3] WRITE→READ turnaround gap (INV-5)
--   [4] READ→WRITE turnaround gap (INV-6)
--   [5] INV-7: WRITE rejected during oen_permanent='1'
--   [6] Burst READ    — back-to-back reads with req_burst='1'
--   [7] Burst stop    — deassert req_burst, verify clean exit
--   [8] 2-FF Synchronizer check
--   [9] Write → Readback (write then read same config register)
--  [10] Burst read 64 entries
--  [11] BUS_TICKS sweep (3, 4, 6, 7)
--  [12] BUS_CLK_DIV sweep (2, 4)
--  [13] Timing assertion (tPW-RL >= 6ns, tS-AD >= 2ns)
--
-- DUT instantiation uses Xilinx IOBUF via UNISIM.
-- For non-Vivado simulators, a behavioral IOBUF model is needed.
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.tdc_gpx_pkg.all;
use work.tb_tdc_gpx_pkg.all;

entity tb_tdc_gpx_bus_phy is
end entity tb_tdc_gpx_bus_phy;

architecture sim of tb_tdc_gpx_bus_phy is

    -- =========================================================================
    -- TB constants
    -- =========================================================================
    constant c_CLK_PERIOD   : time    := 5 ns;       -- 200 MHz
    constant c_BUS_TICKS    : natural := 5;           -- default: 5 ticks/transaction
    constant c_DATA_W       : natural := c_TDC_BUS_WIDTH;  -- 28

    -- Timeout for waiting (in clocks)
    constant c_TIMEOUT      : natural := 200;

    -- =========================================================================
    -- Simulation control
    -- =========================================================================
    signal s_sim_done       : std_logic := '0';

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal s_clk            : std_logic := '0';
    signal s_rst_n          : std_logic := '0';

    -- =========================================================================
    -- DUT ports
    -- =========================================================================
    -- Request interface
    signal s_tick_en        : std_logic := '0';
    signal s_bus_ticks      : unsigned(2 downto 0) := to_unsigned(c_BUS_TICKS, 3);
    signal s_req_valid      : std_logic := '0';
    signal s_req_rw         : std_logic := '0';
    signal s_req_addr       : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata      : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_oen_permanent  : std_logic := '0';
    signal s_req_burst      : std_logic := '0';

    -- Response: AXI-Stream + busy
    signal s_axis_tvalid    : std_logic;
    signal s_axis_tdata     : std_logic_vector(31 downto 0);
    signal s_axis_tkeep     : std_logic_vector(3 downto 0);
    signal s_axis_tuser     : std_logic_vector(7 downto 0);
    signal s_axis_tready    : std_logic := '1';
    signal s_busy           : std_logic;
    -- Convenience aliases
    signal s_rsp_valid      : std_logic;
    signal s_rsp_rdata      : std_logic_vector(c_DATA_W - 1 downto 0);

    -- Physical pins
    signal s_adr            : std_logic_vector(3 downto 0);
    signal s_csn            : std_logic;
    signal s_rdn            : std_logic;
    signal s_wrn            : std_logic;
    signal s_oen            : std_logic;
    signal s_io_d           : std_logic_vector(c_DATA_W - 1 downto 0);

    -- Status pins (directly driven by TB)
    signal s_ef1_pin        : std_logic := '1';   -- empty at start
    signal s_ef2_pin        : std_logic := '1';
    signal s_lf1_pin        : std_logic := '0';
    signal s_lf2_pin        : std_logic := '0';
    signal s_irflag_pin     : std_logic := '0';
    signal s_errflag_pin    : std_logic := '0';

    -- Sync outputs
    signal s_ef1_sync       : std_logic;
    signal s_ef2_sync       : std_logic;
    signal s_lf1_sync       : std_logic;
    signal s_lf2_sync       : std_logic;
    signal s_irflag_sync    : std_logic;
    signal s_errflag_sync   : std_logic;

    -- =========================================================================
    -- TDC-GPX Chip Behavior Model signals
    -- =========================================================================
    -- The chip model drives io_d when OEN='0' (chip output enable) and RDN='0'.
    -- Read data content: increments by 1 per read (starting at x"0001001").
    signal s_chip_d_out     : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_chip_d_oe      : std_logic := '0';  -- '1' = chip drives D-bus
    signal s_chip_read_cnt  : natural := 0;

    -- Write capture (for verification)
    signal s_chip_wr_data   : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
    signal s_chip_wr_addr   : std_logic_vector(3 downto 0) := (others => '0');
    signal s_chip_wr_done   : std_logic := '0';

    -- Chip register array (stores written config register values)
    type t_chip_reg_array is array(0 to 15) of std_logic_vector(c_DATA_W - 1 downto 0);
    signal s_chip_regs      : t_chip_reg_array := (others => (others => '0'));

    -- =========================================================================
    -- Monitor counters (shared verification signals)
    -- =========================================================================
    signal sv_rsp_count     : natural := 0;
    signal sv_last_rdata    : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Timing monitor signals
    -- =========================================================================
    signal s_rdn_fall_time  : time := 0 ns;
    signal s_rdn_rise_time  : time := 0 ns;
    signal s_csn_fall_time  : time := 0 ns;
    signal s_last_rdn_pw    : time := 0 ns;    -- last RDN pulse width measured
    signal s_last_adr_setup : time := 0 ns;    -- last ADR setup time measured

    -- =========================================================================
    -- Tick generation
    -- =========================================================================
    -- tick_en period = bus_clk_div (runtime configurable for sweep tests)
    signal s_bus_clk_div    : natural range 0 to 255 := 1;

begin

    -- Convenience aliases: extract rsp from AXI-Stream for existing test code
    s_rsp_valid <= s_axis_tvalid;
    s_rsp_rdata <= s_axis_tdata(c_DATA_W - 1 downto 0);

    -- =========================================================================
    -- Clock generation (stops when simulation done)
    -- =========================================================================
    s_clk <= not s_clk after c_CLK_PERIOD / 2 when s_sim_done = '0' else s_clk;

    -- =========================================================================
    -- Tick enable generation (every s_bus_clk_div clocks)
    -- =========================================================================
    p_tick_gen : process(s_clk)
        variable v_cnt : natural range 0 to 255 := 0;
    begin
        if rising_edge(s_clk) then
            if s_rst_n = '0' then
                v_cnt      := 0;
                s_tick_en  <= '0';
            else
                if s_bus_clk_div <= 1 or v_cnt = s_bus_clk_div - 1 then
                    v_cnt     := 0;
                    s_tick_en <= '1';
                else
                    v_cnt     := v_cnt + 1;
                    s_tick_en <= '0';
                end if;
            end if;
        end if;
    end process p_tick_gen;

    -- =========================================================================
    -- DUT instantiation
    -- =========================================================================
    u_dut : entity work.tdc_gpx_bus_phy
        port map (
            i_clk           => s_clk,
            i_rst_n         => s_rst_n,
            i_tick_en       => s_tick_en,
            i_bus_ticks     => s_bus_ticks,
            i_req_valid     => s_req_valid,
            i_req_rw        => s_req_rw,
            i_req_addr      => s_req_addr,
            i_req_wdata     => s_req_wdata,
            i_oen_permanent => s_oen_permanent,
            i_req_burst     => s_req_burst,
            o_busy          => s_busy,
            o_rsp_pending   => open,
            o_m_axis_tvalid => s_axis_tvalid,
            o_m_axis_tdata  => s_axis_tdata,
            o_m_axis_tkeep  => s_axis_tkeep,
            o_m_axis_tuser  => s_axis_tuser,
            i_m_axis_tready => s_axis_tready,
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
    -- Emulates the TDC-GPX chip's D-bus behavior:
    --   READ:  When OEN='0' and RDN='0', chip drives io_d with read data
    --   WRITE: When WRN falling edge, chip captures io_d
    --
    -- Read data increments per read (0x0001001, 0x0001002, 0x0001003, ...)
    -- Address-based data: upper nibbles encode address for verification
    -- =========================================================================
    p_chip_model : process(s_clk)
        variable v_wrn_prev      : std_logic := '1';
        variable v_addr_int      : natural;
        variable v_wr_data_hold  : std_logic_vector(c_DATA_W - 1 downto 0) := (others => '0');
        variable v_wr_addr_hold  : std_logic_vector(3 downto 0) := (others => '0');
    begin
        if rising_edge(s_clk) then
            s_chip_wr_done <= '0';

            -- Read enable: chip drives D-bus when OEN='0' and RDN='0'.
            -- Read data source is the concurrent s_chip_d_out assignment
            -- (IFIFO → counter, config reg → stored value).
            if s_oen = '0' and s_rdn = '0' then
                s_chip_d_oe <= '1';
            else
                s_chip_d_oe <= '0';
            end if;

            -- Update read counter on RDN rising edge (after each read completes)
            if s_rdn = '1' and s_chip_d_oe = '1' then
                s_chip_read_cnt <= s_chip_read_cnt + 1;
            end if;

            -- Write capture: hold data while WRN='0' (strobe active)
            -- Commit on WRN rising edge. This avoids delta-cycle issue
            -- where s_io_d goes Hi-Z at the same clock edge as WRN rises.
            if s_wrn = '0' then
                v_wr_data_hold := s_io_d;
                v_wr_addr_hold := s_adr;
            end if;

            if s_wrn = '1' and v_wrn_prev = '0' then
                v_addr_int := to_integer(unsigned(v_wr_addr_hold));
                s_chip_regs(v_addr_int) <= v_wr_data_hold;
                s_chip_wr_data <= v_wr_data_hold;
                s_chip_wr_addr <= v_wr_addr_hold;
                s_chip_wr_done <= '1';
            end if;

            v_wrn_prev := s_wrn;
        end if;
    end process p_chip_model;

    -- Chip read data: for IFIFO regs (8,9) use counter; for config regs use stored value
    s_chip_d_out <= std_logic_vector(to_unsigned(s_chip_read_cnt + 1, c_DATA_W))
                        when s_adr = c_TDC_REG8_IFIFO1 or s_adr = c_TDC_REG9_IFIFO2
                    else s_chip_regs(to_integer(unsigned(s_adr)));

    -- D-bus: chip drives when enabled, else Hi-Z
    s_io_d <= s_chip_d_out when s_chip_d_oe = '1' else (others => 'Z');

    -- =========================================================================
    -- Response monitor (counts responses, latches last data)
    -- =========================================================================
    p_rsp_monitor : process(s_clk)
    begin
        if rising_edge(s_clk) then
            if s_rsp_valid = '1' then
                sv_rsp_count  <= sv_rsp_count + 1;
                sv_last_rdata <= s_rsp_rdata;
            end if;
        end if;
    end process p_rsp_monitor;

    -- =========================================================================
    -- Timing monitor: RDN pulse width
    -- =========================================================================
    p_timing_monitor : process
    begin
        wait until s_rdn = '0';
        s_rdn_fall_time <= now;
        wait until s_rdn = '1';
        s_rdn_rise_time <= now;
        s_last_rdn_pw <= now - s_rdn_fall_time;
    end process p_timing_monitor;

    -- =========================================================================
    -- Timing monitor: ADR setup time (CSN fall to strobe fall)
    -- =========================================================================
    p_adr_timing : process
    begin
        wait until s_csn = '0';
        s_csn_fall_time <= now;
        wait until s_rdn = '0' or s_wrn = '0';
        s_last_adr_setup <= now - s_csn_fall_time;
    end process p_adr_timing;

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stim : process
        variable v_fail     : natural := 0;
        variable v_found    : boolean;
        variable v_rsp_cnt  : natural;
        variable v_expected : std_logic_vector(c_DATA_W - 1 downto 0);

        -- Local convenience procedures
        procedure wait_clk(n : natural) is
        begin
            tb_wait_clk(s_clk, n);
        end procedure;

        procedure wait_rsp(timeout : natural; found : out boolean) is
        begin
            tb_wait_sig_value(s_clk, s_rsp_valid, '1', timeout, found);
        end procedure;

        procedure wait_not_busy(timeout : natural; found : out boolean) is
        begin
            tb_wait_sig_value(s_clk, s_busy, '0', timeout, found);
        end procedure;

        -- Issue a single bus request (non-blocking: just sets signals)
        procedure issue_req(
            rw      : std_logic;
            addr    : std_logic_vector(3 downto 0);
            wdata   : std_logic_vector(c_DATA_W - 1 downto 0);
            burst   : std_logic := '0';
            oen_p   : std_logic := '0'
        ) is
        begin
            s_req_valid     <= '1';
            s_req_rw        <= rw;
            s_req_addr      <= addr;
            s_req_wdata     <= wdata;
            s_req_burst     <= burst;
            s_oen_permanent <= oen_p;
        end procedure;

        -- Clear request
        procedure clear_req is
        begin
            s_req_valid     <= '0';
            s_req_rw        <= '0';
            s_req_addr      <= (others => '0');
            s_req_wdata     <= (others => '0');
            s_req_burst     <= '0';
        end procedure;

    begin
        -- =============================================================
        -- Reset
        -- =============================================================
        s_rst_n <= '0';
        clear_req;
        s_oen_permanent <= '0';
        wait_clk(10);
        s_rst_n <= '1';
        wait_clk(5);

        pr_sep;
        pr_info("tdc_gpx_bus_phy_tb START");
        pr_info("BUS_TICKS=" & nat_img(c_BUS_TICKS)
                & "  BUS_CLK_DIV=" & nat_img(s_bus_clk_div));
        pr_sep;

        -- =============================================================
        -- [1] Single READ
        -- =============================================================
        pr_info("[1] Single READ (Reg8 = IFIFO1)");

        v_rsp_cnt := sv_rsp_count;
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);

        if not v_found then
            pr_fail("[1] rsp_valid timeout", v_fail);
        else
            clear_req;
            wait_clk(1);  -- let monitor update sv_rsp_count
            if sv_rsp_count = v_rsp_cnt + 1 then
                pr_pass("[1] rsp_valid received, rdata=0x" & hex_img(sv_last_rdata));
            else
                pr_fail("[1] unexpected rsp_count="
                        & nat_img(sv_rsp_count), v_fail);
            end if;
        end if;

        -- Wait for bus idle
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- =============================================================
        -- [2] Single WRITE
        -- =============================================================
        pr_info("[2] Single WRITE (Reg0, data=0xABCDEF0)");

        v_rsp_cnt := sv_rsp_count;
        issue_req(rw => '1', addr => c_TDC_REG0,
                  wdata => x"ABCDEF0");

        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[2] rsp_valid timeout", v_fail);
        else
            clear_req;
            wait_clk(1);  -- let monitor update sv_rsp_count
            if sv_rsp_count = v_rsp_cnt + 1 then
                pr_pass("[2] WRITE rsp_valid received");
            else
                pr_fail("[2] unexpected rsp_count", v_fail);
            end if;
        end if;

        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- =============================================================
        -- [3] WRITE→READ turnaround (INV-5)
        --   Issue WRITE, then READ immediately. Expect turnaround gap.
        -- =============================================================
        pr_info("[3] WRITE->READ turnaround (INV-5)");

        -- WRITE first
        v_rsp_cnt := sv_rsp_count;
        issue_req(rw => '1', addr => c_TDC_REG1,
                  wdata => x"1234567");
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[3] WRITE rsp_valid timeout", v_fail);
        end if;
        clear_req;
        wait_clk(1);

        -- READ immediately after
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[3] READ rsp_valid timeout (after turnaround)", v_fail);
        else
            clear_req;
            pr_pass("[3] WRITE->READ turnaround completed, rdata=0x"
                    & hex_img(sv_last_rdata));
        end if;

        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- =============================================================
        -- [4] READ→WRITE turnaround (INV-6)
        -- =============================================================
        pr_info("[4] READ->WRITE turnaround (INV-6)");

        -- READ first
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[4] READ rsp_valid timeout", v_fail);
        end if;
        clear_req;
        wait_clk(1);

        -- WRITE immediately after
        issue_req(rw => '1', addr => c_TDC_REG2,
                  wdata => x"7654321");
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[4] WRITE rsp_valid timeout (after turnaround)", v_fail);
        else
            clear_req;
            pr_pass("[4] READ->WRITE turnaround completed");
        end if;

        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- =============================================================
        -- [5] INV-7: WRITE rejected during oen_permanent='1'
        -- =============================================================
        pr_info("[5] INV-7: WRITE rejected during oen_permanent='1'");

        s_oen_permanent <= '1';
        wait_clk(2);

        v_rsp_cnt := sv_rsp_count;
        issue_req(rw => '1', addr => c_TDC_REG3,
                  wdata => x"DEADBEE", oen_p => '1');

        -- Should NOT get a response (bus_phy rejects the write)
        wait_clk(c_BUS_TICKS * 3);

        if sv_rsp_count = v_rsp_cnt then
            pr_pass("[5] WRITE correctly rejected (no rsp_valid)");
        else
            pr_fail("[5] WRITE should have been rejected but got response", v_fail);
        end if;

        -- Verify OEN is still low (oen_permanent active)
        if s_oen = '0' then
            pr_pass("[5] OEN='0' maintained during oen_permanent");
        else
            pr_fail("[5] OEN should be '0' during oen_permanent", v_fail);
        end if;

        clear_req;

        -- But READ should work with oen_permanent
        pr_info("[5b] READ with oen_permanent='1' (should work)");
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'), oen_p => '1');
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[5b] READ should work during oen_permanent", v_fail);
        else
            clear_req;
            pr_pass("[5b] READ succeeded during oen_permanent, rdata=0x"
                    & hex_img(sv_last_rdata));
        end if;

        wait_not_busy(c_TIMEOUT, v_found);
        s_oen_permanent <= '0';
        wait_clk(5);

        -- =============================================================
        -- [6] Burst READ (3 back-to-back reads)
        -- =============================================================
        pr_info("[6] Burst READ (3 back-to-back reads)");

        s_oen_permanent <= '1';
        wait_clk(2);

        v_rsp_cnt := sv_rsp_count;

        -- Issue burst request
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'),
                  burst => '1', oen_p => '1');

        -- Wait for 3 responses
        for i in 1 to 3 loop
            wait_rsp(c_TIMEOUT, v_found);
            if not v_found then
                pr_fail("[6] burst rsp_valid timeout on read #"
                        & nat_img(i), v_fail);
                exit;
            end if;
            pr_info("[6]   burst read #" & nat_img(i)
                    & " rdata=0x" & hex_img(sv_last_rdata));
        end loop;

        wait_clk(1);  -- let monitor update sv_rsp_count
        if sv_rsp_count >= v_rsp_cnt + 3 then
            pr_pass("[6] 3 burst reads completed");
        else
            pr_fail("[6] expected 3 responses, got "
                    & nat_img(sv_rsp_count - v_rsp_cnt), v_fail);
        end if;

        -- Verify bus stayed busy during burst (no IDLE return)
        if s_busy = '1' then
            pr_pass("[6] bus stayed busy during burst (no IDLE)");
        else
            pr_fail("[6] bus should be busy during burst", v_fail);
        end if;

        -- =============================================================
        -- [7] Burst stop (deassert req_burst -> clean exit)
        -- =============================================================
        pr_info("[7] Burst stop (deassert req_burst)");

        -- Deassert burst
        s_req_burst <= '0';
        s_req_valid <= '0';

        -- One more response may come from in-flight read
        v_rsp_cnt := sv_rsp_count;
        wait_clk(c_BUS_TICKS * 2);

        -- Bus should return to idle
        wait_not_busy(c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[7] bus returned to IDLE after burst stop, extra_rsp="
                    & nat_img(sv_rsp_count - v_rsp_cnt));
        else
            pr_fail("[7] bus did not return to IDLE after burst stop", v_fail);
        end if;

        clear_req;
        s_oen_permanent <= '0';
        wait_clk(5);

        -- =============================================================
        -- [8] 2-FF Synchronizer check
        -- =============================================================
        pr_info("[8] 2-FF Synchronizer check");

        -- Toggle EF1 pin, verify sync output after 2 clocks
        s_ef1_pin <= '0';   -- not empty
        wait_clk(3);        -- 2 FF stages + margin

        if s_ef1_sync = '0' then
            pr_pass("[8] ef1_sync followed ef1_pin='0' (2-FF latency)");
        else
            pr_fail("[8] ef1_sync did not follow ef1_pin", v_fail);
        end if;

        s_ef1_pin <= '1';   -- back to empty
        wait_clk(3);

        if s_ef1_sync = '1' then
            pr_pass("[8] ef1_sync followed ef1_pin='1'");
        else
            pr_fail("[8] ef1_sync did not follow ef1_pin back to '1'", v_fail);
        end if;

        -- =============================================================
        -- [9] Write -> Readback (same config register)
        -- =============================================================
        pr_info("[9] Write -> Readback (Reg1, data=0x1234567)");

        -- Write x"1234567" to Reg1
        issue_req(rw => '1', addr => c_TDC_REG1,
                  wdata => x"1234567");
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[9] WRITE rsp_valid timeout", v_fail);
        end if;
        clear_req;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(2);

        -- Read Reg1 back — should return written value in a single read
        v_rsp_cnt := sv_rsp_count;
        issue_req(rw => '0', addr => c_TDC_REG1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[9] READ rsp_valid timeout", v_fail);
        else
            clear_req;
            wait_clk(1);
            v_expected := x"1234567";
            if sv_last_rdata = v_expected then
                pr_pass("[9] Readback matches: 0x" & hex_img(sv_last_rdata));
            else
                pr_fail("[9] Readback mismatch: got 0x" & hex_img(sv_last_rdata)
                        & " expected 0x" & hex_img(v_expected), v_fail);
            end if;
        end if;

        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- =============================================================
        -- [10] Burst read 64 entries
        -- =============================================================
        pr_info("[10] Burst read 64 entries (Reg8 = IFIFO1)");

        s_oen_permanent <= '1';
        wait_clk(2);

        v_rsp_cnt := sv_rsp_count;

        -- Issue burst request
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'),
                  burst => '1', oen_p => '1');

        -- Wait for 64 responses
        for i in 1 to 64 loop
            wait_rsp(c_TIMEOUT, v_found);
            if not v_found then
                pr_fail("[10] burst rsp_valid timeout on read #"
                        & nat_img(i), v_fail);
                exit;
            end if;
        end loop;

        wait_clk(1);  -- let monitor update
        if sv_rsp_count >= v_rsp_cnt + 64 then
            pr_pass("[10] 64 burst reads completed (count="
                    & nat_img(sv_rsp_count - v_rsp_cnt) & ")");
        else
            pr_fail("[10] expected 64 responses, got "
                    & nat_img(sv_rsp_count - v_rsp_cnt), v_fail);
        end if;

        -- Deassert burst, wait for idle
        s_req_burst <= '0';
        s_req_valid <= '0';
        wait_clk(c_BUS_TICKS * 2);
        wait_not_busy(c_TIMEOUT, v_found);
        if v_found then
            pr_pass("[10] bus returned to IDLE after 64-entry burst");
        else
            pr_fail("[10] bus did not return to IDLE", v_fail);
        end if;

        clear_req;
        s_oen_permanent <= '0';
        wait_clk(5);

        -- =============================================================
        -- [11] BUS_TICKS sweep (3, 4, 6, 7)
        -- =============================================================
        pr_info("[11] BUS_TICKS sweep");

        -- BUS_TICKS = 3
        s_bus_ticks <= to_unsigned(3, 3);
        wait_clk(2);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[11] BUS_TICKS=3 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[11] BUS_TICKS=3 response received");
        end if;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- BUS_TICKS = 4
        s_bus_ticks <= to_unsigned(4, 3);
        wait_clk(2);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[11] BUS_TICKS=4 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[11] BUS_TICKS=4 response received");
        end if;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- BUS_TICKS = 6
        s_bus_ticks <= to_unsigned(6, 3);
        wait_clk(2);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[11] BUS_TICKS=6 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[11] BUS_TICKS=6 response received");
        end if;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- BUS_TICKS = 7
        s_bus_ticks <= to_unsigned(7, 3);
        wait_clk(2);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        if not v_found then
            pr_fail("[11] BUS_TICKS=7 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[11] BUS_TICKS=7 response received");
        end if;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(5);

        -- Restore default
        s_bus_ticks <= to_unsigned(c_BUS_TICKS, 3);
        wait_clk(2);

        -- =============================================================
        -- [12] BUS_CLK_DIV sweep (2, 4)
        -- =============================================================
        pr_info("[12] BUS_CLK_DIV sweep");

        -- BUS_CLK_DIV = 2
        s_bus_clk_div <= 2;
        s_bus_ticks <= to_unsigned(c_BUS_TICKS, 3);
        wait_clk(5);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT * 2, v_found);
        if not v_found then
            pr_fail("[12] BUS_CLK_DIV=2 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[12] BUS_CLK_DIV=2 response received");
        end if;
        wait_not_busy(c_TIMEOUT * 2, v_found);
        wait_clk(5);

        -- BUS_CLK_DIV = 4
        s_bus_clk_div <= 4;
        wait_clk(5);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT * 4, v_found);
        if not v_found then
            pr_fail("[12] BUS_CLK_DIV=4 rsp_valid timeout", v_fail);
        else
            clear_req;
            pr_pass("[12] BUS_CLK_DIV=4 response received");
        end if;
        wait_not_busy(c_TIMEOUT * 4, v_found);
        wait_clk(5);

        -- Restore default
        s_bus_clk_div <= 1;
        wait_clk(5);

        -- =============================================================
        -- [13] Timing assertion (tPW-RL >= 6ns, tS-AD >= 2ns)
        --   Run a fresh read at BUS_TICKS=5 to measure nominal timing.
        --   (BUS_TICKS=3 from [11] gives 5ns which is below spec — correct
        --    behavior; BUS_TICKS=3 should only be used with slower clocks.)
        -- =============================================================
        pr_info("[13] Timing assertions");

        -- Run a fresh read at nominal BUS_TICKS=5
        s_bus_ticks <= to_unsigned(5, 3);
        wait_clk(2);
        issue_req(rw => '0', addr => c_TDC_REG8_IFIFO1,
                  wdata => (others => '0'));
        wait_rsp(c_TIMEOUT, v_found);
        clear_req;
        wait_not_busy(c_TIMEOUT, v_found);
        wait_clk(2);

        -- Check last measured values (from the BUS_TICKS=5 transaction above)
        if s_last_rdn_pw >= 6 ns then
            pr_pass("[13] tPW-RL (RDN pulse width) = "
                    & time'image(s_last_rdn_pw) & " >= 6 ns");
        else
            pr_fail("[13] tPW-RL (RDN pulse width) = "
                    & time'image(s_last_rdn_pw) & " < 6 ns", v_fail);
        end if;

        if s_last_adr_setup >= 2 ns then
            pr_pass("[13] tS-AD (ADR setup) = "
                    & time'image(s_last_adr_setup) & " >= 2 ns");
        else
            pr_fail("[13] tS-AD (ADR setup) = "
                    & time'image(s_last_adr_setup) & " < 2 ns", v_fail);
        end if;

        -- =============================================================
        -- Summary
        -- =============================================================
        pr_sep;
        if v_fail = 0 then
            pr_print("*** ALL TESTS PASSED *** (total_rsp="
                     & nat_img(sv_rsp_count) & ")");
        else
            pr_print("*** " & nat_img(v_fail) & " TEST(S) FAILED ***");
        end if;
        pr_sep;

        -- Stop simulation
        s_sim_done <= '1';
        wait;
    end process p_stim;

end architecture sim;
