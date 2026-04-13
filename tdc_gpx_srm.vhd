-- =============================================================================
-- tdc_gpx_srm.vhd
-- AXI4-Lite Simple Register Map (converted from Verilog tdc_gpx_srm.v)
-- =============================================================================
-- VERSION: 2024.12.22
--
-- Revision history:
--   2024.04.09: Started by Seungyong Park (Verilog).
--   2024.12.22: Updated by Haneul Jung (Verilog).
--   2026.04.13: Converted to VHDL-2008, AXI bug fixes, naming convention.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tdc_gpx_srm is
    generic (
        g_AXI_DATA_WIDTH : natural := 32;
        g_AXI_ADDR_WIDTH : natural := 16;
        g_REG_DATA_WIDTH : natural := 32
    );
    port (
        -- AXI4-Lite clock / reset
        S_AXI_ARESETN : in  std_logic;
        S_AXI_ACLK    : in  std_logic;

        -- AXI4-Lite write address channel
        S_AXI_AWADDR  : in  std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0);
        S_AXI_AWPROT  : in  std_logic_vector(2 downto 0);
        S_AXI_AWVALID : in  std_logic;
        S_AXI_AWREADY : out std_logic;

        -- AXI4-Lite write data channel
        S_AXI_WDATA   : in  std_logic_vector(g_AXI_DATA_WIDTH - 1 downto 0);
        S_AXI_WSTRB   : in  std_logic_vector(g_AXI_DATA_WIDTH / 8 - 1 downto 0);
        S_AXI_WVALID  : in  std_logic;
        S_AXI_WREADY  : out std_logic;
        S_AXI_BRESP   : out std_logic_vector(1 downto 0);
        S_AXI_BVALID  : out std_logic;
        S_AXI_BREADY  : in  std_logic;

        -- AXI4-Lite read address channel
        S_AXI_ARADDR  : in  std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0);
        S_AXI_ARPROT  : in  std_logic_vector(2 downto 0);
        S_AXI_ARVALID : in  std_logic;
        S_AXI_ARREADY : out std_logic;

        -- AXI4-Lite read data channel
        S_AXI_RDATA   : out std_logic_vector(g_AXI_DATA_WIDTH - 1 downto 0);
        S_AXI_RRESP   : out std_logic_vector(1 downto 0);
        S_AXI_RVALID  : out std_logic;
        S_AXI_RREADY  : in  std_logic;

        -- Register ports
        o_magic_num         : out std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0);
        i_system_status     : in  std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0);
        o_dbg_irq           : out std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0);
        i_fpga_build_time   : in  std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_srm;

architecture rtl of tdc_gpx_srm is

    -- =========================================================================
    -- Register address constants
    -- =========================================================================
    constant c_ADDR_MAGIC_NUM      : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := x"0000";
    constant c_ADDR_SYSTEM_STATUS  : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := x"000C";
    constant c_ADDR_DBG_IRQ        : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := x"FF00";
    constant c_ADDR_BUILD_TIME     : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := x"FFF0";

    constant c_MAGIC_RESET_VAL     : std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0) := x"0202_0101";

    -- =========================================================================
    -- AXI4-Lite internal registers
    -- =========================================================================
    signal s_araddr_r  : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_arready_r : std_logic := '0';
    signal s_rresp_r   : std_logic_vector(1 downto 0) := "00";
    signal s_rvalid_r  : std_logic := '0';
    signal s_awaddr_r  : std_logic_vector(g_AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_awready_r : std_logic := '0';
    signal s_wready_r  : std_logic := '0';
    signal s_bresp_r   : std_logic_vector(1 downto 0) := "00";
    signal s_bvalid_r  : std_logic := '0';

    -- Register map read data (held until consumed)
    signal s_rdata_r   : std_logic_vector(g_AXI_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Write/read enable (combinational)
    signal s_wr_en     : std_logic;
    signal s_rd_en     : std_logic;

    -- Register storage
    signal s_magic_num_r : std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0) := c_MAGIC_RESET_VAL;
    signal s_dbg_irq_r   : std_logic_vector(g_REG_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Byte-lane merge: apply WSTRB to combine new WDATA with old register value.
    -- =========================================================================
    function fn_strb_merge(
        v_old  : std_logic_vector;
        v_new  : std_logic_vector;
        v_strb : std_logic_vector
    ) return std_logic_vector is
        variable v_result : std_logic_vector(v_old'range);
    begin
        for b in v_strb'range loop
            if v_strb(b) = '1' then
                v_result(b * 8 + 7 downto b * 8) := v_new(b * 8 + 7 downto b * 8);
            else
                v_result(b * 8 + 7 downto b * 8) := v_old(b * 8 + 7 downto b * 8);
            end if;
        end loop;
        return v_result;
    end function;

begin

    -- =========================================================================
    -- AXI output port assignments
    -- =========================================================================
    S_AXI_ARREADY <= s_arready_r;
    S_AXI_RDATA   <= s_rdata_r when s_rvalid_r = '1' else (others => '0');
    S_AXI_RRESP   <= s_rresp_r;
    S_AXI_RVALID  <= s_rvalid_r;
    S_AXI_AWREADY <= s_awready_r;
    S_AXI_WREADY  <= s_wready_r;
    S_AXI_BRESP   <= s_bresp_r;
    S_AXI_BVALID  <= s_bvalid_r;

    o_magic_num <= s_magic_num_r;
    o_dbg_irq   <= s_dbg_irq_r;

    -- =========================================================================
    -- Write / read enable (combinational)
    -- =========================================================================
    s_wr_en <= s_awready_r and S_AXI_AWVALID and s_wready_r and S_AXI_WVALID;
    s_rd_en <= s_arready_r and S_AXI_ARVALID and (not s_rvalid_r);

    -- =========================================================================
    -- Register write from AXI (with WSTRB byte-lane merge)
    -- =========================================================================
    p_reg_write : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_magic_num_r <= c_MAGIC_RESET_VAL;
                s_dbg_irq_r   <= (others => '0');
            elsif s_wr_en = '1' then
                case s_awaddr_r is
                    when c_ADDR_MAGIC_NUM =>
                        s_magic_num_r <= fn_strb_merge(s_magic_num_r, S_AXI_WDATA, S_AXI_WSTRB);
                    when c_ADDR_DBG_IRQ =>
                        s_dbg_irq_r   <= fn_strb_merge(s_dbg_irq_r,  S_AXI_WDATA, S_AXI_WSTRB);
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_reg_write;

    -- =========================================================================
    -- Register read from AXI (data held until next read)
    -- =========================================================================
    p_reg_read : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_rdata_r <= (others => '0');
            elsif s_rd_en = '1' then
                case s_araddr_r is
                    when c_ADDR_MAGIC_NUM     => s_rdata_r <= s_magic_num_r;
                    when c_ADDR_SYSTEM_STATUS => s_rdata_r <= i_system_status;
                    when c_ADDR_BUILD_TIME    => s_rdata_r <= i_fpga_build_time;
                    when others               => s_rdata_r <= x"DEAD_BEEF";
                end case;
            end if;
        end if;
    end process p_reg_read;

    -- =========================================================================
    -- AXI_AWREADY generation (1-cycle pulse)
    -- =========================================================================
    p_awready : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_awready_r <= '0';
            elsif s_awready_r = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' then
                s_awready_r <= '1';
            else
                s_awready_r <= '0';
            end if;
        end if;
    end process p_awready;

    -- =========================================================================
    -- AXI_AWADDR latching
    -- =========================================================================
    p_awaddr : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_awaddr_r <= (others => '0');
            elsif s_awready_r = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' then
                s_awaddr_r <= S_AXI_AWADDR;
            end if;
        end if;
    end process p_awaddr;

    -- =========================================================================
    -- AXI_WREADY generation (1-cycle pulse)
    -- =========================================================================
    p_wready : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_wready_r <= '0';
            elsif s_awready_r = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' then
                s_wready_r <= '1';
            else
                s_wready_r <= '0';
            end if;
        end if;
    end process p_wready;

    -- =========================================================================
    -- AXI_BVALID and AXI_BRESP
    -- =========================================================================
    p_bresp : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_bvalid_r <= '0';
                s_bresp_r  <= "00";
            elsif s_awready_r = '1' and S_AXI_AWVALID = '1'
              and s_bvalid_r = '0' and s_wready_r = '1' and S_AXI_WVALID = '1' then
                s_bvalid_r <= '1';
                s_bresp_r  <= "00";     -- OKAY
            elsif S_AXI_BREADY = '1' and s_bvalid_r = '1' then
                s_bvalid_r <= '0';
                s_bresp_r  <= "00";
            end if;
        end if;
    end process p_bresp;

    -- =========================================================================
    -- AXI_ARREADY and AXI_ARADDR latching
    -- =========================================================================
    p_arready : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_arready_r <= '0';
                s_araddr_r  <= (others => '0');
            elsif s_arready_r = '0' and S_AXI_ARVALID = '1' then
                s_arready_r <= '1';
                s_araddr_r  <= S_AXI_ARADDR;
            else
                s_arready_r <= '0';
            end if;
        end if;
    end process p_arready;

    -- =========================================================================
    -- AXI_RVALID and AXI_RRESP
    -- =========================================================================
    p_rvalid : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                s_rvalid_r <= '0';
                s_rresp_r  <= "00";
            elsif s_arready_r = '1' and S_AXI_ARVALID = '1' and s_rvalid_r = '0' then
                s_rvalid_r <= '1';
                s_rresp_r  <= "00";     -- OKAY
            elsif s_rvalid_r = '1' and S_AXI_RREADY = '1' then
                s_rvalid_r <= '0';
                s_rresp_r  <= "00";
            end if;
        end if;
    end process p_rvalid;

end architecture rtl;
