-- =============================================================================
-- tdc_gpx_external_chip_model.vhd
-- Behavioral TDC-GPX I-Mode model for system integration simulation only.
--
-- Ownership boundary:
--   * echo_receiver creates physical or synthetic STOP waveforms.
--   * This model measures START-to-STOP time and stores GPX I-Mode words.
--   * tdc_gpx_top only reads the external 28-bit GPX bus.
--
-- I-Mode SINGLE_SHOT word:
--   [27:26] ChaCode  = channel within IFIFO (0..3)
--   [25:18] StartNum = 0
--   [17]    Slope
--   [16:0]  Hit       = floor((STOP time - START time) / bin resolution)
--
-- This file must remain in the simulation source set. It is not synthesizable.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_external_chip_model is
    generic (
        g_NUM_CHIPS              : positive range 1 to c_MAX_CHIPS := c_MAX_CHIPS;
        g_STOPS_PER_CHIP         : positive range 1 to c_MAX_STOPS_PER_CHIP := c_MAX_STOPS_PER_CHIP;
        g_TDC_CLK_MHZ            : positive := 200;
        g_CAPTURE_WINDOW_5NS_TICKS : positive := 1335;
        g_BIN_RESOLUTION_PS      : positive := 81;
        g_FIFO_DEPTH             : positive := 32;
        g_LF_THRESHOLD           : positive := 4;
        g_CHIP_SLOPE_MASK        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := "0011"
    );
    port (
        i_tdc_clk       : in  std_logic;
        i_rst_n         : in  std_logic;

        -- Physical GPX timing inputs from Laser/Echo integration.
        i_start_tdc     : in  std_logic;
        i_tdc_stop      : in  std_logic_vector(
            g_NUM_CHIPS * g_STOPS_PER_CHIP - 1 downto 0);

        -- Parallel bus and control pins connected to tdc_gpx_top.
        io_tdc_d        : inout std_logic_vector(
            g_NUM_CHIPS * c_TDC_BUS_WIDTH - 1 downto 0);
        i_tdc_adr       : in  std_logic_vector(
            g_NUM_CHIPS * c_TDC_ADR_WIDTH - 1 downto 0);
        i_tdc_csn       : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_rdn       : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_wrn       : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_oen       : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_stopdis   : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_alutrigger: in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_puresn    : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);

        o_tdc_ef1       : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_ef2       : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_lf1       : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_lf2       : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_irflag    : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_errflag   : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);

        -- Testbench observability. One value per chip; the integration
        -- profile emits one channel-0 hit per chip and Shot.
        o_last_hit      : out std_logic_vector(
            g_NUM_CHIPS * c_RAW_HIT_WIDTH - 1 downto 0);
        o_capture_count : out std_logic_vector(
            g_NUM_CHIPS * 16 - 1 downto 0)
    );
end entity tdc_gpx_external_chip_model;

architecture behavioral of tdc_gpx_external_chip_model is
    constant c_CHANNELS : positive := g_NUM_CHIPS * g_STOPS_PER_CHIP;
    constant c_CAPTURE_CLKS : positive := fn_time_ps_to_clks_ceil(
        g_CAPTURE_WINDOW_5NS_TICKS * 5000, g_TDC_CLK_MHZ);
    constant c_MAX_HIT : natural := 2 ** c_RAW_HIT_WIDTH - 1;

    subtype t_raw_word is std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    type t_raw_fifo is array (0 to g_FIFO_DEPTH - 1) of t_raw_word;
    type t_reg_bank is array (0 to c_CFG_IMAGE_N_REGS - 1) of t_raw_word;
    type t_time_array is array (natural range <>) of time;

    signal s_start_time   : time := 0 ns;
    signal s_start_toggle : std_logic := '0';
    signal s_stop_time    : t_time_array(0 to c_CHANNELS - 1) :=
        (others => 0 ns);
    signal s_stop_toggle  : std_logic_vector(c_CHANNELS - 1 downto 0) :=
        (others => '0');

    type t_natural_chip_array is array (0 to g_NUM_CHIPS - 1) of natural;
    signal s_fifo1_fill : t_natural_chip_array := (others => 0);
    signal s_fifo2_fill : t_natural_chip_array := (others => 0);
    signal s_last_hit   : t_raw_hit_array := (others => (others => '0'));
    signal s_capture_count : t_natural_chip_array := (others => 0);
    signal s_cfg_error : std_logic_vector(g_NUM_CHIPS - 1 downto 0) :=
        (others => '0');

    type t_chip_word_array is array (0 to g_NUM_CHIPS - 1) of t_raw_word;
    signal s_bus_data : t_chip_word_array := (others => (others => '0'));
    signal s_bus_oe   : std_logic_vector(g_NUM_CHIPS - 1 downto 0) :=
        (others => '0');

    function fn_i_mode_word(
        stop_id : natural;
        slope   : std_logic;
        hit     : natural
    ) return t_raw_word is
        variable v_word : t_raw_word := (others => '0');
    begin
        v_word(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(stop_id mod 4, 2));
        v_word(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) := (others => '0');
        v_word(c_RAW_SLOPE_BIT) := slope;
        v_word(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit, c_RAW_HIT_WIDTH));
        return v_word;
    end function;

begin
    assert g_LF_THRESHOLD <= g_FIFO_DEPTH
        report "tdc_gpx_external_chip_model: LF threshold exceeds FIFO depth"
        severity failure;

    assert fn_range_clk_mhz_supported(g_TDC_CLK_MHZ)
        report "tdc_gpx_external_chip_model: unsupported TDC clock frequency"
        severity failure;

    -- The GPX time interpolator is asynchronous to the FPGA bus clock. Capture
    -- exact event timestamps here, then transfer event toggles into p_chip.
    p_start_timestamp : process(i_start_tdc)
    begin
        if rising_edge(i_start_tdc) then
            s_start_time   <= now;
            s_start_toggle <= not s_start_toggle;
        end if;
    end process p_start_timestamp;

    gen_stop_timestamp : for channel in 0 to c_CHANNELS - 1 generate
        constant c_CHIP : natural := channel / g_STOPS_PER_CHIP;
    begin
        gen_rising_stop : if g_CHIP_SLOPE_MASK(c_CHIP) = '1' generate
            p_stop_timestamp : process(i_tdc_stop(channel))
            begin
                if rising_edge(i_tdc_stop(channel)) then
                    s_stop_time(channel)   <= now;
                    s_stop_toggle(channel) <= not s_stop_toggle(channel);
                end if;
            end process p_stop_timestamp;
        end generate gen_rising_stop;

        gen_falling_stop : if g_CHIP_SLOPE_MASK(c_CHIP) = '0' generate
            p_stop_timestamp : process(i_tdc_stop(channel))
            begin
                if falling_edge(i_tdc_stop(channel)) then
                    s_stop_time(channel)   <= now;
                    s_stop_toggle(channel) <= not s_stop_toggle(channel);
                end if;
            end process p_stop_timestamp;
        end generate gen_falling_stop;
    end generate gen_stop_timestamp;

    gen_chip : for chip in 0 to g_NUM_CHIPS - 1 generate
        constant c_STOP_BASE : natural := chip * g_STOPS_PER_CHIP;
    begin
        p_chip : process(i_tdc_clk)
            variable v_fifo1 : t_raw_fifo := (others => (others => '0'));
            variable v_fifo2 : t_raw_fifo := (others => (others => '0'));
            variable v_fill1 : natural range 0 to g_FIFO_DEPTH := 0;
            variable v_fill2 : natural range 0 to g_FIFO_DEPTH := 0;
            variable v_rd1   : natural range 0 to g_FIFO_DEPTH := 0;
            variable v_rd2   : natural range 0 to g_FIFO_DEPTH := 0;
            variable v_start_seen : std_logic := '0';
            variable v_stop_seen  : std_logic_vector(
                g_STOPS_PER_CHIP - 1 downto 0) := (others => '0');
            variable v_alu_prev   : std_logic := '0';
            variable v_rdn_prev   : std_logic := '1';
            variable v_wrn_prev   : std_logic := '1';
            variable v_wr_hold    : t_raw_word := (others => '0');
            variable v_wr_addr    : natural range 0 to c_CFG_IMAGE_N_REGS - 1 := 0;
            variable v_regs       : t_reg_bank := (others => (others => '0'));
            variable v_mtimer     : natural range 0 to c_CAPTURE_CLKS := 0;
            variable v_irflag     : std_logic := '0';
            variable v_armed      : boolean := false;
            variable v_cfg_valid  : boolean;
            variable v_t0         : time := 0 ns;
            variable v_elapsed_ps : natural;
            variable v_hit        : natural;
            variable v_word       : t_raw_word;
            variable v_addr       : std_logic_vector(c_TDC_ADR_WIDTH - 1 downto 0);
            variable v_channel    : natural;
            variable v_count      : natural := 0;
        begin
            if rising_edge(i_tdc_clk) then
                if i_rst_n = '0' or i_tdc_puresn(chip) = '0' then
                    v_fifo1 := (others => (others => '0'));
                    v_fifo2 := (others => (others => '0'));
                    v_fill1 := 0;
                    v_fill2 := 0;
                    v_rd1 := 0;
                    v_rd2 := 0;
                    v_start_seen := s_start_toggle;
                    for stop_id in 0 to g_STOPS_PER_CHIP - 1 loop
                        v_stop_seen(stop_id) := s_stop_toggle(c_STOP_BASE + stop_id);
                    end loop;
                    v_alu_prev := i_tdc_alutrigger(chip);
                    v_rdn_prev := i_tdc_rdn(chip);
                    v_wrn_prev := i_tdc_wrn(chip);
                    v_wr_hold := (others => '0');
                    v_wr_addr := 0;
                    v_regs := (others => (others => '0'));
                    v_mtimer := 0;
                    v_irflag := '0';
                    v_armed := false;
                    v_t0 := 0 ns;
                    v_count := 0;
                    s_bus_data(chip) <= (others => '0');
                    s_bus_oe(chip) <= '0';
                    s_last_hit(chip) <= (others => '0');
                    s_capture_count(chip) <= 0;
                    s_cfg_error(chip) <= '0';
                else
                    s_bus_oe(chip) <= '0';

                    -- GPX configuration writes hold address/data throughout
                    -- WRN low and commit on the rising edge. Sampling the held
                    -- values avoids the delta-cycle in which bus_phy releases
                    -- io_tdc_d as WRN returns high.
                    if i_tdc_csn(chip) = '0' and i_tdc_wrn(chip) = '0' then
                        v_wr_addr := to_integer(unsigned(i_tdc_adr(
                            (chip + 1) * c_TDC_ADR_WIDTH - 1 downto
                            chip * c_TDC_ADR_WIDTH)));
                        v_wr_hold := io_tdc_d(
                            (chip + 1) * c_TDC_BUS_WIDTH - 1 downto
                            chip * c_TDC_BUS_WIDTH);
                    end if;
                    if i_tdc_wrn(chip) = '1' and v_wrn_prev = '0' then
                        if v_wr_addr = 4 and v_wr_hold(22) = '1' then
                            v_fifo1 := (others => (others => '0'));
                            v_fifo2 := (others => (others => '0'));
                            v_fill1 := 0;
                            v_fill2 := 0;
                            v_rd1 := 0;
                            v_rd2 := 0;
                            v_mtimer := 0;
                            v_irflag := '0';
                            v_armed := false;
                            v_wr_hold(22) := '0';
                        end if;
                        v_regs(v_wr_addr) := v_wr_hold;
                    end if;
                    v_wrn_prev := i_tdc_wrn(chip);

                    -- ALU trigger clears the previous measurement result. It
                    -- never creates a START event in this system model.
                    if i_tdc_alutrigger(chip) = '1' and v_alu_prev = '0' then
                        v_fifo1 := (others => (others => '0'));
                        v_fifo2 := (others => (others => '0'));
                        v_fill1 := 0;
                        v_fill2 := 0;
                        v_rd1 := 0;
                        v_rd2 := 0;
                        v_mtimer := 0;
                        v_irflag := '0';
                        v_armed := false;
                    end if;
                    v_alu_prev := i_tdc_alutrigger(chip);

                    -- Physical START is common to all chips. A new START opens
                    -- MTimer and starts a clean SINGLE_SHOT acquisition only
                    -- after the programmed image satisfies the RTL contract.
                    if s_start_toggle /= v_start_seen then
                        v_start_seen := s_start_toggle;
                        v_fifo1 := (others => (others => '0'));
                        v_fifo2 := (others => (others => '0'));
                        v_fill1 := 0;
                        v_fill2 := 0;
                        v_rd1 := 0;
                        v_rd2 := 0;
                        v_t0 := s_start_time;
                        v_irflag := '0';
                        v_cfg_valid := true;

                        if v_regs(0)(c_REG0_TSTART_RISE) /= '1'
                           or v_regs(0)(c_REG0_TSTART_FALL) /= '0' then
                            assert false
                                report "tdc_gpx_external_chip_model: TStart must use the common rising edge"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(0)(9 downto 7) /= "001"
                           or v_regs(0)(0) /= '1' then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg0 service/start-oscillator bits are invalid"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        for stop_id in 0 to g_STOPS_PER_CHIP - 1 loop
                            if g_CHIP_SLOPE_MASK(chip) = '1' then
                                if v_regs(0)(c_REG0_TSTOP_RISE_LO + stop_id) /= '1' then
                                    assert false
                                        report "tdc_gpx_external_chip_model: rising STOP edge is not enabled"
                                        severity failure;
                                    v_cfg_valid := false;
                                end if;
                            else
                                if v_regs(0)(c_REG0_TSTOP_FALL_LO + stop_id) /= '1' then
                                    assert false
                                        report "tdc_gpx_external_chip_model: falling STOP edge is not enabled"
                                        severity failure;
                                    v_cfg_valid := false;
                                end if;
                            end if;
                        end loop;
                        if v_regs(2) /= x"0000002" then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg2 is not configured for I-mode"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(4)(26 downto 25) /= "11" then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg4 quiet/EFlag policy is invalid"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(5)(c_REG5_MASTER_ALU_TRIG) /= '1' then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg5 MasterAluTrig is disabled"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(7) = x"0000000" then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg7 timing configuration is zero"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(12)(25) /= '1' then
                            assert false
                                report "tdc_gpx_external_chip_model: Reg12 MTimer interrupt is disabled"
                                severity failure;
                            v_cfg_valid := false;
                        end if;
                        if v_regs(14)(4) /= '0' then
                            assert false
                                report "tdc_gpx_external_chip_model: unsupported 16-bit GPX bus mode"
                                severity failure;
                            v_cfg_valid := false;
                        end if;

                        if v_cfg_valid then
                            v_mtimer := c_CAPTURE_CLKS;
                            v_armed := true;
                            s_cfg_error(chip) <= '0';
                        else
                            v_mtimer := 0;
                            v_armed := false;
                            s_cfg_error(chip) <= '1';
                        end if;
                    end if;

                    -- Each asynchronous STOP edge becomes one I-Mode word.
                    for stop_id in 0 to g_STOPS_PER_CHIP - 1 loop
                        v_channel := c_STOP_BASE + stop_id;
                        if s_stop_toggle(v_channel) /= v_stop_seen(stop_id) then
                            v_stop_seen(stop_id) := s_stop_toggle(v_channel);
                            if v_armed and v_mtimer > 0
                               and i_tdc_stopdis(chip) = '0' then
                                assert s_stop_time(v_channel) >= v_t0
                                    report "tdc_gpx_external_chip_model: STOP preceded START"
                                    severity failure;
                                v_elapsed_ps := (s_stop_time(v_channel) - v_t0) / 1 ps;
                                v_hit := v_elapsed_ps / g_BIN_RESOLUTION_PS;
                                assert v_hit <= c_MAX_HIT
                                    report "tdc_gpx_external_chip_model: Hit exceeds 17-bit I-Mode field"
                                    severity failure;
                                if v_hit > c_MAX_HIT then
                                    v_hit := c_MAX_HIT;
                                end if;
                                v_word := fn_i_mode_word(
                                    stop_id, g_CHIP_SLOPE_MASK(chip), v_hit);
                                if stop_id < 4 then
                                    assert v_fill1 < g_FIFO_DEPTH
                                        report "tdc_gpx_external_chip_model: IFIFO1 overflow"
                                        severity failure;
                                    if v_fill1 < g_FIFO_DEPTH then
                                        v_fifo1(v_fill1) := v_word;
                                        v_fill1 := v_fill1 + 1;
                                    end if;
                                else
                                    assert v_fill2 < g_FIFO_DEPTH
                                        report "tdc_gpx_external_chip_model: IFIFO2 overflow"
                                        severity failure;
                                    if v_fill2 < g_FIFO_DEPTH then
                                        v_fifo2(v_fill2) := v_word;
                                        v_fill2 := v_fill2 + 1;
                                    end if;
                                end if;
                                v_count := (v_count + 1) mod 65536;
                                s_last_hit(chip) <= to_unsigned(v_hit, c_RAW_HIT_WIDTH);
                                s_capture_count(chip) <= v_count;
                            end if;
                        end if;
                    end loop;

                    if v_mtimer > 0 then
                        v_mtimer := v_mtimer - 1;
                        if v_mtimer = 0 then
                            v_irflag := '1';
                            v_armed := false;
                        end if;
                    end if;

                    v_addr := i_tdc_adr(
                        (chip + 1) * c_TDC_ADR_WIDTH - 1 downto
                        chip * c_TDC_ADR_WIDTH);

                    -- GPX drives the bus only during an enabled read cycle.
                    if i_tdc_csn(chip) = '0' and i_tdc_oen(chip) = '0'
                       and i_tdc_rdn(chip) = '0' then
                        s_bus_oe(chip) <= '1';
                        if v_addr = c_TDC_REG8_IFIFO1 and v_rd1 < v_fill1 then
                            s_bus_data(chip) <= v_fifo1(v_rd1);
                        elsif v_addr = c_TDC_REG9_IFIFO2 and v_rd2 < v_fill2 then
                            s_bus_data(chip) <= v_fifo2(v_rd2);
                        else
                            s_bus_data(chip) <= (others => '0');
                        end if;
                    end if;

                    -- The external GPX pops the selected IFIFO after RDN.
                    if i_tdc_rdn(chip) = '1' and v_rdn_prev = '0' then
                        if v_addr = c_TDC_REG8_IFIFO1 and v_rd1 < v_fill1 then
                            v_rd1 := v_rd1 + 1;
                        elsif v_addr = c_TDC_REG9_IFIFO2 and v_rd2 < v_fill2 then
                            v_rd2 := v_rd2 + 1;
                        end if;
                    end if;
                    v_rdn_prev := i_tdc_rdn(chip);
                end if;

                s_fifo1_fill(chip) <= v_fill1 - v_rd1;
                s_fifo2_fill(chip) <= v_fill2 - v_rd2;
                o_tdc_irflag(chip) <= v_irflag;
            end if;
        end process p_chip;

        o_tdc_ef1(chip) <= '1' when s_fifo1_fill(chip) = 0 else '0';
        o_tdc_ef2(chip) <= '1' when s_fifo2_fill(chip) = 0 else '0';
        o_tdc_lf1(chip) <= '1' when s_fifo1_fill(chip) >= g_LF_THRESHOLD else '0';
        o_tdc_lf2(chip) <= '1' when s_fifo2_fill(chip) >= g_LF_THRESHOLD else '0';
        o_tdc_errflag(chip) <= s_cfg_error(chip);

        io_tdc_d((chip + 1) * c_TDC_BUS_WIDTH - 1 downto
                 chip * c_TDC_BUS_WIDTH) <=
            s_bus_data(chip) when s_bus_oe(chip) = '1' else (others => 'Z');

        o_last_hit((chip + 1) * c_RAW_HIT_WIDTH - 1 downto
                   chip * c_RAW_HIT_WIDTH) <= std_logic_vector(s_last_hit(chip));
        o_capture_count((chip + 1) * 16 - 1 downto chip * 16) <=
            std_logic_vector(to_unsigned(s_capture_count(chip), 16));
    end generate gen_chip;

end architecture behavioral;
