-- =============================================================================
-- tdc_gpx_cmd_arb.vhd
-- TDC-GPX Controller - Command Arbitration
-- =============================================================================
--
-- Purpose:
--   Manages reg access 1-outstanding lock, cfg_write pending/gating,
--   and reg demux with start/cfg collision prevention.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cmd_arb is
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- Commands from CSR
        i_cmd_start          : in  std_logic;
        i_cmd_start_accepted : in  std_logic;
        i_cmd_stop           : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;
        i_cmd_cfg_write      : in  std_logic;
        i_cmd_reg_read       : in  std_logic;
        i_cmd_reg_write      : in  std_logic;
        i_cmd_reg_chip       : in  unsigned(1 downto 0);

        -- Pipeline idle
        i_chip_busy          : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_face_asm_idle      : in  std_logic;
        i_face_asm_fall_idle : in  std_logic;
        i_hdr_idle           : in  std_logic;
        i_hdr_fall_idle      : in  std_logic;

        -- Per-chip reg done (from chip_ctrl)
        i_cmd_reg_done       : in  std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Gated outputs
        o_cmd_cfg_write_g    : out std_logic;
        o_cmd_reg_read_g     : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_cmd_reg_write_g    : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_reg_outstanding    : out std_logic;
        o_outstanding_chip   : out unsigned(1 downto 0)
    );
end entity tdc_gpx_cmd_arb;

architecture rtl of tdc_gpx_cmd_arb is

    constant C_ZEROS : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    signal s_cfg_write_pending_r    : std_logic := '0';
    signal s_reg_outstanding_r      : std_logic := '0';
    signal s_outstanding_chip_r     : unsigned(1 downto 0) := (others => '0');

    signal s_cmd_cfg_write_g_i      : std_logic;
    signal s_cmd_reg_read_g_i       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_g_i      : std_logic_vector(c_N_CHIPS - 1 downto 0);

begin

    -- Reg demux: 1-outstanding + start/cfg collision prevention
    gen_reg_demux : for i in 0 to c_N_CHIPS - 1 generate
        s_cmd_reg_read_g_i(i) <= i_cmd_reg_read
                                when to_integer(i_cmd_reg_chip) = i
                                 and s_reg_outstanding_r = '0'
                                 and i_cmd_start = '0'
                                 and i_cmd_start_accepted = '0'
                                 and s_cmd_cfg_write_g_i = '0'
                                 and i_cmd_cfg_write = '0'
                                 and i_chip_busy(i) = '0'
                                else '0';
        s_cmd_reg_write_g_i(i) <= i_cmd_reg_write
                                when to_integer(i_cmd_reg_chip) = i
                                 and s_reg_outstanding_r = '0'
                                 and i_cmd_start = '0'
                                 and i_cmd_start_accepted = '0'
                                 and s_cmd_cfg_write_g_i = '0'
                                 and i_cmd_cfg_write = '0'
                                 and i_chip_busy(i) = '0'
                                else '0';
    end generate;

    -- cfg_write pending + full pipeline idle gating
    p_cfg_write_pending : process(i_clk)
        variable v_pipeline_idle : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_cmd_soft_reset = '1' or i_cmd_stop = '1' then
                s_cfg_write_pending_r <= '0';
            else
                v_pipeline_idle := i_chip_busy = C_ZEROS
                               and i_face_asm_idle = '1'
                               and i_face_asm_fall_idle = '1'
                               and i_hdr_idle = '1'
                               and i_hdr_fall_idle = '1'
                               and i_cmd_start = '0'
                               and i_cmd_start_accepted = '0';
                if i_cmd_cfg_write = '1' and not v_pipeline_idle then
                    s_cfg_write_pending_r <= '1';
                elsif s_cfg_write_pending_r = '1' and v_pipeline_idle then
                    s_cfg_write_pending_r <= '0';
                end if;
            end if;
        end if;
    end process;

    s_cmd_cfg_write_g_i <= (i_cmd_cfg_write or s_cfg_write_pending_r)
                           when i_chip_busy = C_ZEROS
                            and i_face_asm_idle = '1'
                            and i_face_asm_fall_idle = '1'
                            and i_hdr_idle = '1'
                            and i_hdr_fall_idle = '1'
                            and i_cmd_start = '0'
                            and i_cmd_start_accepted = '0'
                           else '0';

    -- 1-outstanding lock
    p_reg_latch : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_cmd_soft_reset = '1' or i_cmd_stop = '1' then
                s_outstanding_chip_r <= (others => '0');
                s_reg_outstanding_r  <= '0';
            else
                if (s_cmd_reg_read_g_i /= C_ZEROS
                    or s_cmd_reg_write_g_i /= C_ZEROS) then
                    s_outstanding_chip_r <= i_cmd_reg_chip;
                    s_reg_outstanding_r  <= '1';
                end if;
                if s_reg_outstanding_r = '1'
                   and i_cmd_reg_done(to_integer(s_outstanding_chip_r)) = '1' then
                    s_reg_outstanding_r <= '0';
                end if;
            end if;
        end if;
    end process;

    o_cmd_cfg_write_g  <= s_cmd_cfg_write_g_i;
    o_cmd_reg_read_g   <= s_cmd_reg_read_g_i;
    o_cmd_reg_write_g  <= s_cmd_reg_write_g_i;
    o_reg_outstanding  <= s_reg_outstanding_r;
    o_outstanding_chip <= s_outstanding_chip_r;

end architecture rtl;
