-- =============================================================================
-- tdc_gpx_config_ctrl.vhd
-- TDC-GPX Controller - Cluster 1: Chip Configuration & Control
-- =============================================================================
--
-- Purpose:
--   Pure structural wrapper for TDC-GPX chip configuration and control.
--   Instantiates:
--     csr_chip       : AXI-Lite CSR + SRM + CDC for chip-owned registers
--     cmd_arb        : command arbitration (cfg_write, reg read/write gating)
--     err_handler    : automatic ErrFlag detection, Reg11 read, recovery FSM
--     stop_decode    : stop event decode + cfg_image override
--     bus_phy x4     : TDC-GPX bus physical layer (IOBUF + timing FSM)
--     sk_brsp x4     : skid buffer bus_phy -> chip_ctrl (40b)
--     chip_ctrl x4   : chip FSM coordinator (powerup/cfg/arm/capture/drain)
--     sk_raw x4      : skid buffer chip_ctrl -> decode_pipe (40b)
--
--   Config merging: pipeline fields from i_cfg_pipeline, chip fields from
--   csr_chip outputs.  Merged config drives stop_decode and chip_ctrl.
--
-- Clock domains:
--   i_axis_aclk  : TDC processing / AXI-Stream domain (~200 MHz)
--   s_axi_aclk   : AXI4-Lite PS domain
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_config_ctrl is
    generic (
        g_HW_VERSION      : std_logic_vector(31 downto 0) := x"00010000";
        g_POWERUP_CLKS    : positive := 48;
        g_RECOVERY_CLKS   : positive := 8;
        g_ALU_PULSE_CLKS  : positive := 4;
        g_STOP_EVT_DWIDTH : natural := 32
    );
    port (
        -- Clock / Reset: processing domain
        i_axis_aclk          : in  std_logic;
        i_axis_aresetn       : in  std_logic;

        -- Clock / Reset: AXI-Lite domain (PS)
        s_axi_aclk           : in  std_logic;
        s_axi_aresetn        : in  std_logic;

        -- =====================================================================
        -- AXI4-Lite slave pass-through (9-bit address)
        -- =====================================================================
        s_axi_awvalid        : in  std_logic;
        s_axi_awready        : out std_logic;
        s_axi_awaddr         : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot         : in  std_logic_vector(2 downto 0);
        s_axi_wvalid         : in  std_logic;
        s_axi_wready         : out std_logic;
        s_axi_wdata          : in  std_logic_vector(31 downto 0);
        s_axi_wstrb          : in  std_logic_vector(3 downto 0);
        s_axi_bvalid         : out std_logic;
        s_axi_bready         : in  std_logic;
        s_axi_bresp          : out std_logic_vector(1 downto 0);
        s_axi_arvalid        : in  std_logic;
        s_axi_arready        : out std_logic;
        s_axi_araddr         : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot         : in  std_logic_vector(2 downto 0);
        s_axi_rvalid         : out std_logic;
        s_axi_rready         : in  std_logic;
        s_axi_rdata          : out std_logic_vector(31 downto 0);
        s_axi_rresp          : out std_logic_vector(1 downto 0);

        -- =====================================================================
        -- TDC-GPX physical pins pass-through (per chip, x4)
        -- =====================================================================
        io_tdc_d             : inout t_tdc_bus_array;
        o_tdc_adr            : out   t_tdc_adr_array;
        o_tdc_csn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_rdn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_wrn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_oen            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_stopdis        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_alutrigger     : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_puresn         : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef1            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef2            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf1            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf2            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_irflag         : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_errflag        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- =====================================================================
        -- External inputs (stop event stream + deadline pulse)
        -- =====================================================================
        i_stop_evt_tvalid    : in  std_logic;
        i_stop_evt_tdata     : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tkeep     : in  std_logic_vector(g_STOP_EVT_DWIDTH/8 - 1 downto 0);
        i_stop_evt_tuser     : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready    : out std_logic;
        i_stop_tdc           : in  std_logic;

        -- =====================================================================
        -- Control inputs from face_seq / csr_pipeline (top-level routing)
        -- =====================================================================
        i_cmd_start          : in  std_logic;
        i_cmd_start_accepted : in  std_logic;
        i_cmd_stop           : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;
        i_cmd_cfg_write      : in  std_logic;
        i_shot_start_per_chip : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_shot_start_gated   : in  std_logic;
        i_cfg_pipeline       : in  t_tdc_cfg;

        -- =====================================================================
        -- Cluster 4 idle inputs (for cmd_arb gating)
        -- =====================================================================
        i_face_asm_idle      : in  std_logic;
        i_face_asm_fall_idle : in  std_logic;
        i_hdr_idle           : in  std_logic;
        i_hdr_fall_idle      : in  std_logic;

        -- =====================================================================
        -- Frame boundary (from output_stage, for err_handler)
        -- =====================================================================
        i_frame_done         : in  std_logic;
        i_frame_fall_done    : in  std_logic;

        -- Pipeline abort: flush raw-path skid buffers
        i_pipeline_abort     : in  std_logic;

        -- =====================================================================
        -- Output to Cluster 2: AXI-Stream x4 (from sk_raw)
        -- =====================================================================
        o_raw_sk_tvalid      : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_raw_sk_tdata       : out t_slv32_array;
        o_raw_sk_tuser       : out t_slv8_array;
        i_raw_sk_tready      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- =====================================================================
        -- Configuration outputs (merged from csr_chip + csr_pipeline)
        -- =====================================================================
        o_cfg                : out t_tdc_cfg;
        o_cfg_image          : out t_cfg_image;

        -- =====================================================================
        -- Command outputs
        -- =====================================================================
        o_cmd_start          : out std_logic;
        o_cmd_cfg_write_g    : out std_logic;

        -- =====================================================================
        -- Chip status outputs (for face_seq + status_agg)
        -- =====================================================================
        o_chip_busy          : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_chip_shot_seq      : out t_shot_seq_array;
        o_errflag_sync       : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_drain_timeout  : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_sequence       : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_rsp_mismatch   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_reg_outstanding    : out std_logic;
        o_reg_loop_resume    : out std_logic;
        o_cdc_idle           : out std_logic;

        -- =====================================================================
        -- Error handler status outputs
        -- =====================================================================
        o_err_active         : out std_logic;
        o_err_fatal          : out std_logic;
        o_err_chip_mask      : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_cause          : out std_logic_vector(2 downto 0);

        -- =====================================================================
        -- Interrupt
        -- =====================================================================
        o_irq                : out std_logic
    );
end entity tdc_gpx_config_ctrl;

architecture rtl of tdc_gpx_config_ctrl is

    attribute KEEP_HIERARCHY : string;
    attribute KEEP_HIERARCHY of rtl : architecture is "yes";

    -- =========================================================================
    -- csr_chip configuration outputs (i_axis_aclk domain)
    -- =========================================================================
    signal s_bus_clk_div     : unsigned(5 downto 0);
    signal s_bus_ticks       : unsigned(2 downto 0);
    signal s_start_off1      : unsigned(17 downto 0);
    signal s_cfg_reg7        : std_logic_vector(31 downto 0);
    signal s_max_scan_clks   : unsigned(15 downto 0);
    signal s_max_hits_cfg    : unsigned(2 downto 0);
    signal s_cfg_image_raw   : t_cfg_image;

    -- =========================================================================
    -- Merged configuration (pipeline + chip fields)
    -- =========================================================================
    signal s_cfg_merged      : t_tdc_cfg;

    -- =========================================================================
    -- cmd_arb internal signals
    -- =========================================================================
    signal s_cmd_reg_read    : std_logic;
    signal s_cmd_reg_write   : std_logic;
    signal s_cmd_reg_addr    : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip    : unsigned(1 downto 0);
    signal s_cmd_reg_wdata   : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal s_cmd_reg_rdata   : t_slv28_array;
    signal s_cmd_reg_rvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_done    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_read_g  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_g : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_chip_address    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_done_pulse   : std_logic;
    signal s_cmd_reg_done_chip    : unsigned(1 downto 0);
    signal s_reg_loop_resume      : std_logic;
    signal s_cmd_reg_addr_out     : std_logic_vector(3 downto 0);

    -- =========================================================================
    -- stop_decode outputs
    -- =========================================================================
    signal s_expected_ififo1 : t_expected_array;
    signal s_expected_ififo2 : t_expected_array;

    -- =========================================================================
    -- Per-chip: bus_phy <-> chip_ctrl
    -- =========================================================================
    signal s_bus_req_valid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_rw      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_addr    : t_slv4_array;
    signal s_bus_req_wdata   : t_slv28_array;
    signal s_bus_oen_perm    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_burst   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_ticks_snap  : t_u3_array;
    signal s_bus_busy        : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy response AXI-Stream
    signal s_brsp_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_axis_tdata  : t_slv32_array;
    signal s_brsp_axis_tkeep  : t_slv4_array;
    signal s_brsp_axis_tuser  : t_slv8_array;
    signal s_brsp_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer bus_phy -> chip_ctrl
    signal s_brsp_sk_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_sk_tdata   : t_slv32_array;
    signal s_brsp_sk_tuser   : t_slv8_array;
    signal s_brsp_sk_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy synchronized status
    signal s_ef1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_ef2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_irflag_sync     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_errflag_sync    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl -> downstream raw AXI-Stream
    signal s_raw_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_axis_tdata  : t_slv32_array;
    signal s_raw_axis_tuser  : t_slv8_array;
    signal s_raw_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl status
    signal s_drain_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_shot_seq   : t_shot_seq_array;
    signal s_chip_busy       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_tick_en         : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_sequence      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_rsp_mismatch  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- err_handler outputs
    -- =========================================================================
    signal s_err_cmd_soft_reset    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_cmd_reg_read      : std_logic;
    signal s_err_cmd_reg_addr      : std_logic_vector(3 downto 0);
    signal s_err_cmd_reg_chip_addr : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_fill              : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_active            : std_logic;

    -- =========================================================================
    -- MUX signals: err_handler / csr_chip -> cmd_arb
    -- =========================================================================
    signal s_cmd_reg_read_mux         : std_logic;
    signal s_cmd_reg_write_mux        : std_logic;
    signal s_cmd_reg_addr_mux         : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip_address_mux : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Per-chip: sk_raw output (before err_fill gating)
    -- =========================================================================
    signal s_sk_raw_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_sk_raw_tdata  : t_slv32_array;
    signal s_sk_raw_tuser  : t_slv8_array;

begin

    -- =========================================================================
    -- Config merging: pipeline fields + chip fields -> s_cfg_merged
    -- =========================================================================

    -- Pipeline fields (from i_cfg_pipeline)
    s_cfg_merged.active_chip_mask <= i_cfg_pipeline.active_chip_mask;
    s_cfg_merged.packet_scope     <= i_cfg_pipeline.packet_scope;
    s_cfg_merged.hit_store_mode   <= i_cfg_pipeline.hit_store_mode;
    s_cfg_merged.dist_scale       <= i_cfg_pipeline.dist_scale;
    s_cfg_merged.drain_mode       <= i_cfg_pipeline.drain_mode;
    s_cfg_merged.pipeline_en      <= i_cfg_pipeline.pipeline_en;
    s_cfg_merged.n_faces          <= i_cfg_pipeline.n_faces;
    s_cfg_merged.stops_per_chip   <= i_cfg_pipeline.stops_per_chip;
    s_cfg_merged.n_drain_cap      <= i_cfg_pipeline.n_drain_cap;
    s_cfg_merged.stopdis_override <= i_cfg_pipeline.stopdis_override;
    s_cfg_merged.max_range_clks   <= i_cfg_pipeline.max_range_clks;
    s_cfg_merged.cols_per_face    <= i_cfg_pipeline.cols_per_face;

    -- Chip fields (from csr_chip outputs)
    s_cfg_merged.bus_clk_div      <= s_bus_clk_div;
    s_cfg_merged.bus_ticks        <= s_bus_ticks;
    s_cfg_merged.start_off1       <= s_start_off1;
    s_cfg_merged.cfg_reg7         <= s_cfg_reg7;
    s_cfg_merged.max_scan_clks    <= s_max_scan_clks;
    s_cfg_merged.max_hits_cfg     <= s_max_hits_cfg;

    -- Drive merged config to output
    o_cfg <= s_cfg_merged;

    -- =========================================================================
    -- Pass-through outputs
    -- =========================================================================
    o_cmd_start         <= i_cmd_start;
    o_reg_loop_resume   <= s_reg_loop_resume;
    o_chip_busy         <= s_chip_busy;
    o_chip_shot_seq     <= s_chip_shot_seq;
    o_errflag_sync      <= s_errflag_sync;
    o_err_drain_timeout <= s_err_drain_timeout;
    o_err_sequence      <= s_err_sequence;
    o_err_rsp_mismatch  <= s_err_rsp_mismatch;
    o_err_active        <= s_err_active;

    -- =========================================================================
    -- MUX: when err_handler is active, it owns the cmd_arb reg-access path
    -- =========================================================================
    s_cmd_reg_read_mux         <= s_err_cmd_reg_read      when s_err_active = '1'
                                  else s_cmd_reg_read;
    s_cmd_reg_addr_mux         <= s_err_cmd_reg_addr      when s_err_active = '1'
                                  else s_cmd_reg_addr;
    s_cmd_reg_chip_address_mux <= s_err_cmd_reg_chip_addr when s_err_active = '1'
                                  else s_cmd_reg_chip_address;
    s_cmd_reg_write_mux        <= '0' when s_err_active = '1'
                                  else s_cmd_reg_write;

    -- =========================================================================
    -- Per-chip err_fill gating on raw AXI-Stream output
    -- =========================================================================
    gen_err_fill : for i in 0 to c_N_CHIPS - 1 generate
        -- Error-fill: replace ONLY hit[16:0] with all-ones (0x1FFFF).
        -- Preserve upper raw bits (stop_id, slope, cha_code, ififo_id)
        -- so downstream decode/cell_builder see correct stop/slope routing.
        o_raw_sk_tdata(i)(16 downto 0)  <= s_sk_raw_tdata(i)(16 downto 0) when s_err_fill(i) = '0'
                                           else (others => '1');
        o_raw_sk_tdata(i)(31 downto 17) <= s_sk_raw_tdata(i)(31 downto 17);  -- always pass through
        o_raw_sk_tvalid(i) <= s_sk_raw_tvalid(i);
        o_raw_sk_tuser(i)  <= s_sk_raw_tuser(i);
    end generate gen_err_fill;

    -- =========================================================================
    -- s_cmd_reg_wdata: cfg_image indexed by target register address
    -- =========================================================================
    s_cmd_reg_wdata <= o_cfg_image(to_integer(unsigned(s_cmd_reg_addr_mux)))(c_TDC_BUS_WIDTH - 1 downto 0);

    -- =========================================================================
    -- [1] csr_chip: AXI-Lite CSR + SRM + CDC
    -- =========================================================================
    u_csr_chip : entity work.tdc_gpx_csr_chip
        port map (
            s_axi_aclk      => s_axi_aclk,
            s_axi_aresetn   => s_axi_aresetn,
            s_axi_awvalid   => s_axi_awvalid,
            s_axi_awready   => s_axi_awready,
            s_axi_awaddr    => s_axi_awaddr,
            s_axi_awprot    => s_axi_awprot,
            s_axi_wvalid    => s_axi_wvalid,
            s_axi_wready    => s_axi_wready,
            s_axi_wdata     => s_axi_wdata,
            s_axi_wstrb     => s_axi_wstrb,
            s_axi_bvalid    => s_axi_bvalid,
            s_axi_bready    => s_axi_bready,
            s_axi_bresp     => s_axi_bresp,
            s_axi_arvalid   => s_axi_arvalid,
            s_axi_arready   => s_axi_arready,
            s_axi_araddr    => s_axi_araddr,
            s_axi_arprot    => s_axi_arprot,
            s_axi_rvalid    => s_axi_rvalid,
            s_axi_rready    => s_axi_rready,
            s_axi_rdata     => s_axi_rdata,
            s_axi_rresp     => s_axi_rresp,
            i_axis_aclk     => i_axis_aclk,
            i_axis_aresetn  => i_axis_aresetn,
            o_cfg_image     => s_cfg_image_raw,
            o_bus_clk_div   => s_bus_clk_div,
            o_bus_ticks     => s_bus_ticks,
            o_start_off1    => s_start_off1,
            o_cfg_reg7      => s_cfg_reg7,
            o_max_scan_clks => s_max_scan_clks,
            o_max_hits_cfg  => s_max_hits_cfg,
            o_cmd_reg_read  => s_cmd_reg_read,
            o_cmd_reg_write => s_cmd_reg_write,
            o_cmd_reg_addr  => s_cmd_reg_addr,
            o_cmd_reg_chip  => s_cmd_reg_chip,
            i_cmd_reg_rdata_0    => s_cmd_reg_rdata(0),
            i_cmd_reg_rdata_1    => s_cmd_reg_rdata(1),
            i_cmd_reg_rdata_2    => s_cmd_reg_rdata(2),
            i_cmd_reg_rdata_3    => s_cmd_reg_rdata(3),
            i_cmd_reg_rvalid     => s_cmd_reg_rvalid,
            i_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            i_cmd_reg_addr_done  => s_cmd_reg_addr_out,
            o_cmd_reg_chip_address  => s_cmd_reg_chip_address,
            o_cdc_idle      => o_cdc_idle,
            o_irq           => o_irq
        );

    -- =========================================================================
    -- [2] cmd_arb: command arbitration
    -- =========================================================================
    u_cmd_arb : entity work.tdc_gpx_cmd_arb
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            i_cmd_start          => i_cmd_start,
            i_cmd_start_accepted => i_cmd_start_accepted,
            i_cmd_stop           => i_cmd_stop,
            i_cmd_soft_reset     => i_cmd_soft_reset,
            i_cmd_cfg_write      => i_cmd_cfg_write,
            i_cmd_reg_read       => s_cmd_reg_read_mux,
            i_cmd_reg_write      => s_cmd_reg_write_mux,
            i_cmd_reg_chip       => s_cmd_reg_chip,
            i_cmd_reg_chip_address  => s_cmd_reg_chip_address_mux,
            i_cmd_reg_addr       => s_cmd_reg_addr_mux,
            i_chip_busy          => s_chip_busy,
            i_face_asm_idle      => i_face_asm_idle,
            i_face_asm_fall_idle => i_face_asm_fall_idle,
            i_hdr_idle           => i_hdr_idle,
            i_hdr_fall_idle      => i_hdr_fall_idle,
            i_cmd_reg_done       => s_cmd_reg_done,
            o_cmd_cfg_write_g    => o_cmd_cfg_write_g,
            o_cmd_reg_read_g     => s_cmd_reg_read_g,
            o_cmd_reg_write_g    => s_cmd_reg_write_g,
            o_reg_outstanding    => o_reg_outstanding,
            o_outstanding_chip   => open,
            o_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            o_cmd_reg_done_chip  => s_cmd_reg_done_chip,
            o_reg_loop_resume    => s_reg_loop_resume,
            o_cmd_reg_addr_out   => s_cmd_reg_addr_out
        );

    -- =========================================================================
    -- [2b] err_handler: automatic ErrFlag detection and recovery
    -- =========================================================================
    u_err_handler : entity work.tdc_gpx_err_handler
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            i_errflag_sync       => s_errflag_sync,
            i_chip_busy          => s_chip_busy,
            i_reg11_data_0       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata(0),
            i_reg11_data_1       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata(1),
            i_reg11_data_2       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata(2),
            i_reg11_data_3       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata(3),
            i_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            i_frame_done         => i_frame_done or i_frame_fall_done,
            i_shot_start         => i_shot_start_gated,
            o_cmd_soft_reset     => s_err_cmd_soft_reset,
            o_cmd_reg_read       => s_err_cmd_reg_read,
            o_cmd_reg_addr       => s_err_cmd_reg_addr,
            o_cmd_reg_chip_addr  => s_err_cmd_reg_chip_addr,
            o_err_fill           => s_err_fill,
            o_err_active         => s_err_active,
            o_err_chip_mask      => o_err_chip_mask,
            o_err_cause          => o_err_cause,
            o_err_fatal          => o_err_fatal
        );

    -- =========================================================================
    -- [3] stop_decode: stop event decode + cfg_image override
    -- =========================================================================
    u_stop_decode : entity work.tdc_gpx_stop_cfg_decode
        generic map (g_STOP_EVT_DWIDTH => g_STOP_EVT_DWIDTH)
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_stop_evt_tvalid  => i_stop_evt_tvalid,
            i_stop_evt_tdata   => i_stop_evt_tdata,
            i_stop_evt_tuser   => i_stop_evt_tuser,
            o_stop_evt_tready  => o_stop_evt_tready,
            i_shot_start_gated => i_shot_start_gated,
            o_expected_ififo1  => s_expected_ififo1,
            o_expected_ififo2  => s_expected_ififo2,
            i_cfg              => s_cfg_merged,
            i_cfg_image_raw    => s_cfg_image_raw,
            o_cfg_image        => o_cfg_image
        );

    -- =========================================================================
    -- [4-19] Per-chip pipeline (generate x4)
    --   bus_phy + sk_brsp + chip_ctrl + sk_raw
    -- =========================================================================
    gen_chip : for i in 0 to c_N_CHIPS - 1 generate

        -- ----- bus_phy: physical bus timing FSM + IOBUF + 2-FF sync -----
        u_bus_phy : entity work.tdc_gpx_bus_phy
            port map (
                i_clk           => i_axis_aclk,
                i_rst_n         => i_axis_aresetn,
                i_tick_en       => s_tick_en(i),
                i_bus_ticks     => s_bus_ticks_snap(i),
                i_req_valid     => s_bus_req_valid(i),
                i_req_rw        => s_bus_req_rw(i),
                i_req_addr      => s_bus_req_addr(i),
                i_req_wdata     => s_bus_req_wdata(i),
                i_oen_permanent => s_bus_oen_perm(i),
                i_req_burst     => s_bus_req_burst(i),
                o_busy          => s_bus_busy(i),
                o_m_axis_tvalid => s_brsp_axis_tvalid(i),
                o_m_axis_tdata  => s_brsp_axis_tdata(i),
                o_m_axis_tkeep  => s_brsp_axis_tkeep(i),
                o_m_axis_tuser  => s_brsp_axis_tuser(i),
                i_m_axis_tready => s_brsp_axis_tready(i),
                o_adr           => o_tdc_adr(i),
                o_csn           => o_tdc_csn(i),
                o_rdn           => o_tdc_rdn(i),
                o_wrn           => o_tdc_wrn(i),
                o_oen           => o_tdc_oen(i),
                io_d            => io_tdc_d(i),
                i_ef1_pin       => i_tdc_ef1(i),
                i_ef2_pin       => i_tdc_ef2(i),
                i_lf1_pin       => i_tdc_lf1(i),
                i_lf2_pin       => i_tdc_lf2(i),
                i_irflag_pin    => i_tdc_irflag(i),
                i_errflag_pin   => i_tdc_errflag(i),
                o_ef1_sync      => s_ef1_sync(i),
                o_ef2_sync      => s_ef2_sync(i),
                o_lf1_sync      => s_lf1_sync(i),
                o_lf2_sync      => s_lf2_sync(i),
                o_irflag_sync   => s_irflag_sync(i),
                o_errflag_sync  => s_errflag_sync(i)
            );

        -- ----- skid buffer: bus_phy -> chip_ctrl (32b tdata + 8b tuser = 40b) -----
        u_sk_brsp : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk     => i_axis_aclk,
                i_rst_n   => i_axis_aresetn,
                i_flush   => '0',
                i_s_valid => s_brsp_axis_tvalid(i),
                o_s_ready => s_brsp_axis_tready(i),
                i_s_data  => s_brsp_axis_tdata(i) & s_brsp_axis_tuser(i),
                o_m_valid => s_brsp_sk_tvalid(i),
                i_m_ready => s_brsp_sk_tready(i),
                o_m_data(39 downto 8) => s_brsp_sk_tdata(i),
                o_m_data(7 downto 0)  => s_brsp_sk_tuser(i)
            );

        -- ----- chip_ctrl: single-shot FSM (powerup/cfg/arm/capture/drain) -----
        u_chip_ctrl : entity work.tdc_gpx_chip_ctrl
            generic map (
                g_CHIP_ID        => i,
                g_POWERUP_CLKS   => g_POWERUP_CLKS,
                g_RECOVERY_CLKS  => g_RECOVERY_CLKS,
                g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
            )
            port map (
                i_clk               => i_axis_aclk,
                i_rst_n             => i_axis_aresetn,
                i_cfg               => s_cfg_merged,
                i_cfg_image         => o_cfg_image,
                i_cmd_start         => i_cmd_start_accepted,
                i_cmd_stop          => i_cmd_stop,
                i_cmd_soft_reset    => i_cmd_soft_reset,
                i_cmd_soft_reset_err => s_err_cmd_soft_reset(i),
                i_cmd_cfg_write     => o_cmd_cfg_write_g,
                i_cmd_reg_read      => s_cmd_reg_read_g(i),
                i_cmd_reg_write     => s_cmd_reg_write_g(i),
                i_cmd_reg_addr      => s_cmd_reg_addr_mux,  -- muxed: err_handler or CSR
                i_cmd_reg_wdata     => s_cmd_reg_wdata,
                o_cmd_reg_rdata     => s_cmd_reg_rdata(i),
                o_cmd_reg_rvalid    => s_cmd_reg_rvalid(i),
                o_cmd_reg_done      => s_cmd_reg_done(i),
                i_shot_start        => i_shot_start_per_chip(i),
                i_max_range_clks    => s_cfg_merged.max_range_clks,
                i_stop_tdc          => i_stop_tdc,
                i_expected_ififo1   => s_expected_ififo1(i),
                i_expected_ififo2   => s_expected_ififo2(i),
                o_bus_req_valid     => s_bus_req_valid(i),
                o_bus_req_rw        => s_bus_req_rw(i),
                o_bus_req_addr      => s_bus_req_addr(i),
                o_bus_req_wdata     => s_bus_req_wdata(i),
                o_bus_oen_permanent => s_bus_oen_perm(i),
                o_bus_req_burst     => s_bus_req_burst(i),
                o_bus_ticks_snap    => s_bus_ticks_snap(i),
                i_s_axis_tvalid     => s_brsp_sk_tvalid(i),
                i_s_axis_tdata      => s_brsp_sk_tdata(i),
                i_s_axis_tuser      => s_brsp_sk_tuser(i),
                o_s_axis_tready     => s_brsp_sk_tready(i),
                i_bus_busy          => s_bus_busy(i),
                i_ef1_sync          => s_ef1_sync(i),
                i_ef2_sync          => s_ef2_sync(i),
                i_irflag_sync       => s_irflag_sync(i),
                i_lf1_sync          => s_lf1_sync(i),
                i_lf2_sync          => s_lf2_sync(i),
                o_tick_en           => s_tick_en(i),
                o_stopdis           => o_tdc_stopdis(i),
                o_alutrigger        => o_tdc_alutrigger(i),
                o_puresn            => o_tdc_puresn(i),
                o_m_raw_axis_tvalid => s_raw_axis_tvalid(i),
                o_m_raw_axis_tdata  => s_raw_axis_tdata(i),
                o_m_raw_axis_tuser  => s_raw_axis_tuser(i),
                i_m_raw_axis_tready => s_raw_axis_tready(i),
                o_drain_done        => s_drain_done(i),
                o_shot_seq          => s_chip_shot_seq(i),
                o_busy              => s_chip_busy(i),
                o_err_drain_timeout => s_err_drain_timeout(i),
                o_err_sequence      => s_err_sequence(i),
                o_err_rsp_mismatch  => s_err_rsp_mismatch(i)
            );

        -- ----- skid buffer: chip_ctrl -> decode_pipe (32b + 8b = 40b) -----
        u_sk_raw : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk     => i_axis_aclk,
                i_rst_n   => i_axis_aresetn,
                i_flush   => i_pipeline_abort,
                i_s_valid => s_raw_axis_tvalid(i),
                o_s_ready => s_raw_axis_tready(i),
                i_s_data  => s_raw_axis_tdata(i) & s_raw_axis_tuser(i),
                o_m_valid => s_sk_raw_tvalid(i),
                i_m_ready => i_raw_sk_tready(i),
                o_m_data(39 downto 8) => s_sk_raw_tdata(i),
                o_m_data(7 downto 0)  => s_sk_raw_tuser(i)
            );

    end generate gen_chip;

end architecture rtl;
