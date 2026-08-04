library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

-- Checkpoint-E configuration boundary: AXI4-Lite CSR, atomic manager and both
-- destination-domain gateways. Functional cores consume only the typed active
-- records and enables exported here; they never contain local AXI logic.
entity lidar_csr_config_subsystem is
    generic (
        G_BUILD_CONFIG     : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_CSR_CLK_MHZ      : positive := 100;
        G_PHASE_TIMEOUT_US : positive := 1000
    );
    port (
        i_csr_clk   : in  std_logic;
        i_csr_rst_n : in  std_logic;
        i_proc_clk  : in  std_logic;
        i_proc_rst_n : in std_logic;
        i_tdc_clk   : in  std_logic;
        i_tdc_rst_n : in  std_logic;

        s_axi_awaddr  : in  std_logic_vector(8 downto 0);
        s_axi_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        s_axi_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        s_axi_araddr  : in  std_logic_vector(8 downto 0);
        s_axi_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;

        i_proc_safe : in  std_logic;
        i_tdc_safe  : in  std_logic;

        o_irq                : out std_logic;
        o_clear_status       : out std_logic;
        o_soft_reset_request : out std_logic;
        o_busy               : out std_logic;
        o_done               : out std_logic;
        o_commit_rejected    : out std_logic;
        o_reject_error       : out lidar_cfg_error_t;
        o_error              : out lidar_cfg_error_t;
        o_recovery_required  : out std_logic;
        o_active_valid       : out std_logic;
        o_active             : out lidar_active_config_t;
        o_proc_enable        : out std_logic;
        o_proc_active_valid  : out std_logic;
        o_proc_active        : out lidar_active_config_t;
        o_tdc_enable         : out std_logic;
        o_tdc_active_valid   : out std_logic;
        o_tdc_active         : out lidar_active_config_t;
        o_prepare_req        : out std_logic;
        o_activate_req       : out std_logic;
        o_release_req        : out std_logic
    );
end entity lidar_csr_config_subsystem;

architecture rtl of lidar_csr_config_subsystem is

    signal w_shadow : lidar_runtime_config_t;
    signal w_commit : std_logic;
    signal w_busy   : std_logic;
    signal w_done   : std_logic;
    signal w_rejected : std_logic;
    signal w_reject_error : lidar_cfg_error_t;
    signal w_error          : lidar_cfg_error_t;
    signal w_recovery       : std_logic;
    signal w_active_valid   : std_logic;
    signal w_active         : lidar_active_config_t;

begin

    o_busy              <= w_busy;
    o_done              <= w_done;
    o_commit_rejected   <= w_rejected;
    o_reject_error      <= w_reject_error;
    o_error             <= w_error;
    o_recovery_required <= w_recovery;
    o_active_valid      <= w_active_valid;
    o_active            <= w_active;

    u_csr : entity work.lidar_csr_bank
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk    => i_csr_clk,
            i_rst_n  => i_csr_rst_n,
            s_axi_awaddr  => s_axi_awaddr,
            s_axi_awprot  => s_axi_awprot,
            s_axi_awvalid => s_axi_awvalid,
            s_axi_awready => s_axi_awready,
            s_axi_wdata   => s_axi_wdata,
            s_axi_wstrb   => s_axi_wstrb,
            s_axi_wvalid  => s_axi_wvalid,
            s_axi_wready  => s_axi_wready,
            s_axi_bresp   => s_axi_bresp,
            s_axi_bvalid  => s_axi_bvalid,
            s_axi_bready  => s_axi_bready,
            s_axi_araddr  => s_axi_araddr,
            s_axi_arprot  => s_axi_arprot,
            s_axi_arvalid => s_axi_arvalid,
            s_axi_arready => s_axi_arready,
            s_axi_rdata   => s_axi_rdata,
            s_axi_rresp   => s_axi_rresp,
            s_axi_rvalid  => s_axi_rvalid,
            s_axi_rready  => s_axi_rready,
            i_cfg_busy              => w_busy,
            i_cfg_done              => w_done,
            i_cfg_commit_rejected   => w_rejected,
            i_cfg_reject_error      => w_reject_error,
            i_cfg_error             => w_error,
            i_cfg_recovery_required => w_recovery,
            i_cfg_active_valid      => w_active_valid,
            i_cfg_active            => w_active,
            o_shadow             => w_shadow,
            o_commit             => w_commit,
            o_clear_status       => o_clear_status,
            o_soft_reset_request => o_soft_reset_request,
            o_irq                => o_irq
        );

    u_config : entity work.lidar_config_subsystem
        generic map (
            G_BUILD_CONFIG     => G_BUILD_CONFIG,
            G_CSR_CLK_MHZ      => G_CSR_CLK_MHZ,
            G_PHASE_TIMEOUT_US => G_PHASE_TIMEOUT_US
        )
        port map (
            i_csr_clk          => i_csr_clk,
            i_csr_rst_n        => i_csr_rst_n,
            i_proc_clk         => i_proc_clk,
            i_proc_rst_n       => i_proc_rst_n,
            i_tdc_clk          => i_tdc_clk,
            i_tdc_rst_n        => i_tdc_rst_n,
            i_commit           => w_commit,
            i_shadow           => w_shadow,
            i_proc_safe        => i_proc_safe,
            i_tdc_safe         => i_tdc_safe,
            o_busy             => w_busy,
            o_done             => w_done,
            o_commit_rejected  => w_rejected,
            o_reject_error     => w_reject_error,
            o_error            => w_error,
            o_recovery_required => w_recovery,
            o_active_valid     => w_active_valid,
            o_active           => w_active,
            o_proc_enable      => o_proc_enable,
            o_proc_active_valid => o_proc_active_valid,
            o_proc_active      => o_proc_active,
            o_tdc_enable       => o_tdc_enable,
            o_tdc_active_valid => o_tdc_active_valid,
            o_tdc_active       => o_tdc_active,
            o_prepare_req      => o_prepare_req,
            o_activate_req     => o_activate_req,
            o_release_req      => o_release_req
        );

end architecture rtl;
