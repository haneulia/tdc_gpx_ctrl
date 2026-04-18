-- =============================================================================
-- tdc_gpx_stop_cfg_decode.vhd
-- TDC-GPX Controller - Stop Event Decoder
-- =============================================================================
--
-- Purpose:
--   Decodes per-chip IFIFO expected drain counts from echo_receiver.
--   Also handles cfg_image register override (StartOff1, MasterAluTrig, Reg7).
--
-- CONTRACT (echo_receiver → stop_cfg_decode stream):
--   - i_shot_start_gated pulses once per shot; counts reset to zero on this edge.
--   - Between shot_start pulses, echo_receiver emits i_stop_evt_tvalid each time
--     a stop pulse is observed. Each beat carries the RUNNING total of stop
--     pulses seen so far in the current shot (not a delta).
--   - Counts MUST monotonically increase within a shot window (a running total
--     can only grow or stay the same). Decrease is a contract violation and is
--     flagged by the sim-only monotonicity checker below.
--   - The last beat before irflag carries the FINAL expected count for this
--     shot; o_expected_ififo1/2 retain that value (overwrite semantics).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_stop_cfg_decode is
    generic (
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH
    );
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;

        -- Stop event AXI-Stream slave
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready : out std_logic;

        -- Shot boundary clear
        i_shot_start_gated : in  std_logic;

        -- Per-chip expected IFIFO counts
        o_expected_ififo1 : out t_expected_array;
        o_expected_ififo2 : out t_expected_array;

        -- Config image override
        i_cfg             : in  t_tdc_cfg;
        i_cfg_image_raw   : in  t_cfg_image;
        o_cfg_image       : out t_cfg_image
    );
end entity tdc_gpx_stop_cfg_decode;

architecture rtl of tdc_gpx_stop_cfg_decode is
begin

    o_stop_evt_tready <= '1';

    p_stop_decode : process(i_clk)
        variable v_lo : natural;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_shot_start_gated = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    o_expected_ififo1(i) <= (others => '0');
                    o_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo := i * 8;
                    o_expected_ififo1(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    o_expected_ififo2(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                end loop;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Sim-only monotonicity contract check.
    -- Catches echo_receiver bugs where the running total decreases within a
    -- shot window (should be non-decreasing per contract above). Zero-impact
    -- on synthesized RTL; purely an observability aid in regression.
    -- =========================================================================
    -- synthesis translate_off
    p_monotonic_check : process(i_clk)
        variable v_new1   : unsigned(7 downto 0);
        variable v_new2   : unsigned(7 downto 0);
        variable v_lo     : natural;
        type t_prev_arr is array(0 to c_N_CHIPS - 1) of unsigned(7 downto 0);
        variable v_prev1  : t_prev_arr := (others => (others => '0'));
        variable v_prev2  : t_prev_arr := (others => (others => '0'));
        variable v_track  : boolean := false;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_shot_start_gated = '1' then
                v_track := false;
                for i in 0 to c_N_CHIPS - 1 loop
                    v_prev1(i) := (others => '0');
                    v_prev2(i) := (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo   := i * 8;
                    v_new1 := resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                            + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    v_new2 := resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                            + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                    if v_track then
                        assert v_new1 >= v_prev1(i)
                            report "stop_cfg_decode: IFIFO1 running total decreased on chip "
                                   & integer'image(i) & " (contract violation)"
                            severity error;
                        assert v_new2 >= v_prev2(i)
                            report "stop_cfg_decode: IFIFO2 running total decreased on chip "
                                   & integer'image(i) & " (contract violation)"
                            severity error;
                    end if;
                    v_prev1(i) := v_new1;
                    v_prev2(i) := v_new2;
                end loop;
                v_track := true;
            end if;
        end if;
    end process;
    -- synthesis translate_on

    -- cfg_image override: force specific register bits
    p_cfg_override : process(i_cfg_image_raw, i_cfg)
        variable v_img : t_cfg_image;
    begin
        v_img := i_cfg_image_raw;
        v_img(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(i_cfg.start_off1);
        v_img(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_img(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        v_img(7) := i_cfg.cfg_reg7;
        o_cfg_image <= v_img;
    end process;

end architecture rtl;
