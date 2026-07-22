-- =============================================================================
-- tdc_gpx_parent_core.vhd
-- Board-independent IP Integrator wrapper for tdc_gpx_top.
--
-- The production core exports compact flat TDC pin vectors sized by
-- g_NUM_CHIPS. This wrapper preserves that synthesis-time physical contract
-- across the IP Integrator module-reference boundary and adds explicit
-- AXI/clock interface metadata.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_parent_core is
    generic (
        g_HW_VERSION        : std_logic_vector(31 downto 0) := x"00010000";
        g_OUTPUT_WIDTH       : natural := 32;
        -- IP Integrator module-reference parsing does not resolve package
        -- constants in generic defaults/ranges. These literals mirror the
        -- production core's fixed four-chip ABI and package maxima.
        g_NUM_CHIPS          : positive range 1 to 4 := 4;
        g_PRESENT_CHIP_MASK  : std_logic_vector(3 downto 0) := "1111";
        g_RISE_CHIP_MASK     : std_logic_vector(3 downto 0) := "0011";
        g_FALL_CHIP_MASK     : std_logic_vector(3 downto 0) := "1100";
        g_MAX_STOPS_PER_CHIP : positive range 2 to 8 := 8;
        g_MAX_HITS_PER_STOP  : positive range 1 to 7 := 7;
        g_AXIS_CLK_MHZ       : positive := 150;
        g_TDC_CLK_MHZ        : positive := 200;
        -- Physical timing policy passed through to tdc_gpx_top. Unit-bearing
        -- generics keep this parent independent of the selected FCLK periods.
        g_POWERUP_TIME_NS             : positive := 240;
        g_RECOVERY_TIME_NS            : positive := 40;
        g_ALU_PULSE_TIME_NS           : positive := 20;
        g_BUS_READ_PERIOD_MIN_TIME_NS : positive := 25;
        g_BUS_IDLE_STABLE_TIME_NS     : positive := 20480;
        g_DRAIN_MARGIN_TIME_NS        : positive := 1280;
        g_ERR_DEBOUNCE_TIME_NS        : positive := 25;
        g_ERR_MAX_RETRIES             : positive := 3;
        g_CELL_QUARANTINE_MARGIN_TIME_NS : positive := 3410;
        g_CELL_IFIFO2_MARGIN_TIME_NS  : positive := 1705;
        g_OEN_MODE           : string   := "DYNAMIC_CONNECTED";
        g_STREAM_CLK_MODE    : string   := "ASYNC"
    );
    port (
        i_axis_aclk    : in std_logic;
        i_axis_aresetn : in std_logic;
        i_tdc_clk      : in std_logic;

        i_ctrl_aclk    : in std_logic;
        i_ctrl_aresetn : in std_logic;

        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        s_axi_awaddr  : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        s_axi_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        s_axi_araddr  : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);

        s_axi_pipe_awvalid : in  std_logic;
        s_axi_pipe_awready : out std_logic;
        s_axi_pipe_awaddr  : in  std_logic_vector(6 downto 0);
        s_axi_pipe_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_pipe_wvalid  : in  std_logic;
        s_axi_pipe_wready  : out std_logic;
        s_axi_pipe_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_pipe_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_pipe_bvalid  : out std_logic;
        s_axi_pipe_bready  : in  std_logic;
        s_axi_pipe_bresp   : out std_logic_vector(1 downto 0);
        s_axi_pipe_arvalid : in  std_logic;
        s_axi_pipe_arready : out std_logic;
        s_axi_pipe_araddr  : in  std_logic_vector(6 downto 0);
        s_axi_pipe_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_pipe_rvalid  : out std_logic;
        s_axi_pipe_rready  : in  std_logic;
        s_axi_pipe_rdata   : out std_logic_vector(31 downto 0);
        s_axi_pipe_rresp   : out std_logic_vector(1 downto 0);

        i_lsr_valid : in std_logic;
        i_lsr_data  : in std_logic_vector(31 downto 0);
        i_shot_start : in std_logic;
        i_stop_tdc   : in std_logic;

        -- Compact physical lanes exported by tdc_gpx_top. Lane order follows
        -- ascending asserted bits in g_PRESENT_CHIP_MASK.
        io_tdc_d : inout std_logic_vector(
            g_NUM_CHIPS * c_TDC_BUS_WIDTH - 1 downto 0);
        o_tdc_adr : out std_logic_vector(
            g_NUM_CHIPS * c_TDC_ADR_WIDTH - 1 downto 0);

        o_tdc_csn        : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_rdn        : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_wrn        : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_oen        : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_stopdis    : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_alutrigger : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        o_tdc_puresn     : out std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_ef1        : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_ef2        : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_lf1        : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_lf2        : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_irflag     : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);
        i_tdc_errflag    : in  std_logic_vector(g_NUM_CHIPS - 1 downto 0);

        m_axis_tdata  : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        m_axis_tkeep  : out std_logic_vector(g_OUTPUT_WIDTH/8 - 1 downto 0);
        m_axis_tstrb  : out std_logic_vector(g_OUTPUT_WIDTH/8 - 1 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tuser  : out std_logic_vector(0 downto 0);
        m_axis_tready : in  std_logic;

        m_axis_fall_tdata  : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        m_axis_fall_tkeep  : out std_logic_vector(g_OUTPUT_WIDTH/8 - 1 downto 0);
        m_axis_fall_tstrb  : out std_logic_vector(g_OUTPUT_WIDTH/8 - 1 downto 0);
        m_axis_fall_tvalid : out std_logic;
        m_axis_fall_tlast  : out std_logic;
        m_axis_fall_tuser  : out std_logic_vector(0 downto 0);
        m_axis_fall_tready : in  std_logic;

        o_vdma_hsize_bytes_rise : out std_logic_vector(15 downto 0);
        o_vdma_hsize_bytes_fall : out std_logic_vector(15 downto 0);
        o_vdma_vsize_lines      : out std_logic_vector(15 downto 0);

        i_bin_resolution_ps : in std_logic_vector(15 downto 0);
        i_k_dist_fixed      : in std_logic_vector(31 downto 0);

        o_irq      : out std_logic;
        o_irq_pipe : out std_logic
    );

end tdc_gpx_parent_core;

architecture rtl of tdc_gpx_parent_core is
    signal s_vdma_hsize_rise : unsigned(15 downto 0);
    signal s_vdma_hsize_fall : unsigned(15 downto 0);
    signal s_vdma_vsize      : unsigned(15 downto 0);

    -- Geometry is produced in the AXIS domain but consumed by PS software in
    -- the control domain. Transfer all three fields as one atomic snapshot so
    -- AXI GPIO can never expose a mixed HSIZE/VSIZE tuple during reconfigure.
    signal s_geometry_axis : std_logic_vector(47 downto 0);
    signal s_geometry_ctrl : std_logic_vector(47 downto 0);

    attribute DONT_TOUCH : string;
    attribute DONT_TOUCH of u_tdc_gpx : label is "true";
begin
    s_geometry_axis <= std_logic_vector(s_vdma_vsize)
                     & std_logic_vector(s_vdma_hsize_fall)
                     & std_logic_vector(s_vdma_hsize_rise);

    o_vdma_hsize_bytes_rise <= s_geometry_ctrl(15 downto 0);
    o_vdma_hsize_bytes_fall <= s_geometry_ctrl(31 downto 16);
    o_vdma_vsize_lines      <= s_geometry_ctrl(47 downto 32);

    u_geometry_cdc : entity work.tdc_gpx_atomic_snapshot_cdc
        generic map (
            g_WIDTH   => 48,
            g_SYNC_FF => 4
        )
        port map (
            i_src_clk     => i_axis_aclk,
            i_src_resetn  => i_axis_aresetn,
            i_src_data    => s_geometry_axis,
            i_dest_clk    => i_ctrl_aclk,
            i_dest_resetn => i_ctrl_aresetn,
            o_dest_data   => s_geometry_ctrl
        );

    u_tdc_gpx : entity work.tdc_gpx_top
        generic map (
            g_HW_VERSION        => g_HW_VERSION,
            g_OUTPUT_WIDTH       => g_OUTPUT_WIDTH,
            g_NUM_CHIPS          => g_NUM_CHIPS,
            g_PRESENT_CHIP_MASK  => g_PRESENT_CHIP_MASK,
            g_RISE_CHIP_MASK     => g_RISE_CHIP_MASK,
            g_FALL_CHIP_MASK     => g_FALL_CHIP_MASK,
            g_MAX_STOPS_PER_CHIP => g_MAX_STOPS_PER_CHIP,
            g_MAX_HITS_PER_STOP  => g_MAX_HITS_PER_STOP,
            g_AXIS_CLK_MHZ       => g_AXIS_CLK_MHZ,
            g_TDC_CLK_MHZ        => g_TDC_CLK_MHZ,
            g_POWERUP_TIME_NS    => g_POWERUP_TIME_NS,
            g_RECOVERY_TIME_NS   => g_RECOVERY_TIME_NS,
            g_ALU_PULSE_TIME_NS  => g_ALU_PULSE_TIME_NS,
            g_BUS_READ_PERIOD_MIN_TIME_NS => g_BUS_READ_PERIOD_MIN_TIME_NS,
            g_BUS_IDLE_STABLE_TIME_NS => g_BUS_IDLE_STABLE_TIME_NS,
            g_DRAIN_MARGIN_TIME_NS => g_DRAIN_MARGIN_TIME_NS,
            g_ERR_DEBOUNCE_TIME_NS => g_ERR_DEBOUNCE_TIME_NS,
            g_ERR_MAX_RETRIES => g_ERR_MAX_RETRIES,
            g_CELL_QUARANTINE_MARGIN_TIME_NS => g_CELL_QUARANTINE_MARGIN_TIME_NS,
            g_CELL_IFIFO2_MARGIN_TIME_NS => g_CELL_IFIFO2_MARGIN_TIME_NS,
            g_OEN_MODE           => g_OEN_MODE,
            g_STREAM_CLK_MODE    => g_STREAM_CLK_MODE
        )
        port map (
            i_axis_aclk    => i_axis_aclk,
            i_axis_aresetn => i_axis_aresetn,
            i_tdc_clk      => i_tdc_clk,
            s_axi_aclk     => i_ctrl_aclk,
            s_axi_aresetn  => i_ctrl_aresetn,

            s_axi_awvalid => s_axi_awvalid,
            s_axi_awready => s_axi_awready,
            s_axi_awaddr  => s_axi_awaddr,
            s_axi_awprot  => s_axi_awprot,
            s_axi_wvalid  => s_axi_wvalid,
            s_axi_wready  => s_axi_wready,
            s_axi_wdata   => s_axi_wdata,
            s_axi_wstrb   => s_axi_wstrb,
            s_axi_bvalid  => s_axi_bvalid,
            s_axi_bready  => s_axi_bready,
            s_axi_bresp   => s_axi_bresp,
            s_axi_arvalid => s_axi_arvalid,
            s_axi_arready => s_axi_arready,
            s_axi_araddr  => s_axi_araddr,
            s_axi_arprot  => s_axi_arprot,
            s_axi_rvalid  => s_axi_rvalid,
            s_axi_rready  => s_axi_rready,
            s_axi_rdata   => s_axi_rdata,
            s_axi_rresp   => s_axi_rresp,

            s_axi_pipe_awvalid => s_axi_pipe_awvalid,
            s_axi_pipe_awready => s_axi_pipe_awready,
            s_axi_pipe_awaddr  => s_axi_pipe_awaddr,
            s_axi_pipe_awprot  => s_axi_pipe_awprot,
            s_axi_pipe_wvalid  => s_axi_pipe_wvalid,
            s_axi_pipe_wready  => s_axi_pipe_wready,
            s_axi_pipe_wdata   => s_axi_pipe_wdata,
            s_axi_pipe_wstrb   => s_axi_pipe_wstrb,
            s_axi_pipe_bvalid  => s_axi_pipe_bvalid,
            s_axi_pipe_bready  => s_axi_pipe_bready,
            s_axi_pipe_bresp   => s_axi_pipe_bresp,
            s_axi_pipe_arvalid => s_axi_pipe_arvalid,
            s_axi_pipe_arready => s_axi_pipe_arready,
            s_axi_pipe_araddr  => s_axi_pipe_araddr,
            s_axi_pipe_arprot  => s_axi_pipe_arprot,
            s_axi_pipe_rvalid  => s_axi_pipe_rvalid,
            s_axi_pipe_rready  => s_axi_pipe_rready,
            s_axi_pipe_rdata   => s_axi_pipe_rdata,
            s_axi_pipe_rresp   => s_axi_pipe_rresp,

            i_lsr_tvalid => i_lsr_valid,
            i_lsr_tdata  => i_lsr_data,
            i_shot_start => i_shot_start,
            i_stop_tdc   => i_stop_tdc,

            io_tdc_d            => io_tdc_d,
            o_tdc_adr           => o_tdc_adr,
            o_tdc_csn        => o_tdc_csn,
            o_tdc_rdn        => o_tdc_rdn,
            o_tdc_wrn        => o_tdc_wrn,
            o_tdc_oen        => o_tdc_oen,
            o_tdc_stopdis    => o_tdc_stopdis,
            o_tdc_alutrigger => o_tdc_alutrigger,
            o_tdc_puresn     => o_tdc_puresn,
            i_tdc_ef1        => i_tdc_ef1,
            i_tdc_ef2        => i_tdc_ef2,
            i_tdc_lf1        => i_tdc_lf1,
            i_tdc_lf2        => i_tdc_lf2,
            i_tdc_irflag     => i_tdc_irflag,
            i_tdc_errflag    => i_tdc_errflag,

            o_m_axis_tdata  => m_axis_tdata,
            o_m_axis_tkeep  => m_axis_tkeep,
            o_m_axis_tstrb  => m_axis_tstrb,
            o_m_axis_tvalid => m_axis_tvalid,
            o_m_axis_tlast  => m_axis_tlast,
            o_m_axis_tuser  => m_axis_tuser,
            i_m_axis_tready => m_axis_tready,

            o_m_axis_fall_tdata  => m_axis_fall_tdata,
            o_m_axis_fall_tkeep  => m_axis_fall_tkeep,
            o_m_axis_fall_tstrb  => m_axis_fall_tstrb,
            o_m_axis_fall_tvalid => m_axis_fall_tvalid,
            o_m_axis_fall_tlast  => m_axis_fall_tlast,
            o_m_axis_fall_tuser  => m_axis_fall_tuser,
            i_m_axis_fall_tready => m_axis_fall_tready,

            o_vdma_hsize_bytes_rise => s_vdma_hsize_rise,
            o_vdma_hsize_bytes_fall => s_vdma_hsize_fall,
            o_vdma_vsize_lines      => s_vdma_vsize,

            i_bin_resolution_ps => unsigned(i_bin_resolution_ps),
            i_k_dist_fixed      => unsigned(i_k_dist_fixed),
            o_irq               => o_irq,
            o_irq_pipe          => o_irq_pipe
        );
end architecture rtl;
