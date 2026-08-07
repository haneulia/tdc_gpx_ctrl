library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_status_pkg.all;

-- Checkpoint-E configuration boundary: AXI4-Lite CSR, atomic manager and both
-- destination-domain gateways. Functional cores consume only the typed active
-- records and enables exported here; they never contain local AXI logic.
entity lidar_csr_config_subsystem is
    generic (
        G_BUILD_CONFIG     : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_CSR_CLK_MHZ      : positive := 100;
        G_PHASE_TIMEOUT_US : positive := 1000;
        G_PROC_DEFER_ACTIVATE_ACK : boolean := false;
        G_TDC_DEFER_ACTIVATE_ACK : boolean := false
    );
    port (
        i_csr_clk   : in  std_logic;
        i_csr_rst_n : in  std_logic;
        i_proc_clk  : in  std_logic;
        i_proc_rst_n : in std_logic;
        i_proc_soft_reset : in std_logic := '0';
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
        i_external_laser_permit : in std_logic;
        i_tdc_config_ready : in std_logic := '0';
        i_tdc_config_done  : in std_logic := '0';
        i_tdc_config_fault : in std_logic := '0';
        i_proc_activate_complete : in std_logic := '0';
        i_proc_activate_fault    : in std_logic := '0';
        i_system_command_ready    : in std_logic := '1';
        i_system_command_rejected : in std_logic := '0';
        i_runtime_irq : in lidar_runtime_irq_t := C_RUNTIME_IRQ_CLEAR;

        o_diag_request_valid : out std_logic;
        i_diag_request_ready : in  std_logic := '0';
        o_diag_request_index : out lidar_diag_index_t;
        i_diag_response_valid : in  std_logic := '0';
        o_diag_response_ready : out std_logic;
        i_diag_response       : in  lidar_diag_response_t :=
            (others => '0');

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
        o_tdc_register_image : out gpx_register_image_t;
        o_tdc_config_apply   : out std_logic;
        o_proc_activate_start : out std_logic;
        o_prepare_req        : out std_logic;
        o_activate_req       : out std_logic;
        o_release_req        : out std_logic;
        o_operation_state    : out operation_state_t;
        o_operation_command_accepted : out std_logic;
        o_operation_command_rejected : out std_logic;
        o_operation_permit_trip : out std_logic;
        o_operation_safe_to_prepare : out std_logic
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
    signal w_proc_enable       : std_logic;
    signal w_proc_active_valid : std_logic;
    signal w_proc_active       : lidar_active_config_t;
    signal w_tdc_enable        : std_logic;
    signal w_tdc_active_valid  : std_logic;
    signal w_tdc_active        : lidar_active_config_t;
    signal w_gpx_shadow_image    : gpx_register_image_t;
    signal w_gpx_candidate_image : gpx_register_image_t;
    signal w_gpx_active_image    : gpx_register_image_t;
    signal w_tdc_register_image  : gpx_register_image_t;
    signal w_tdc_config_apply    : std_logic;
    signal w_tdc_activate_start  : std_logic;
    signal w_tdc_activate_complete : std_logic;
    signal w_tdc_activate_fault    : std_logic;

    signal w_operation_command_valid : std_logic;
    signal w_operation_command       : operation_command_t;
    signal w_operation_command_ready : std_logic;
    signal w_operation_command_busy  : std_logic;
    signal w_operation_cdc_rejected  : std_logic;
    signal w_operation_state_proc    : operation_state_t;
    signal w_operation_state_csr     : operation_state_t;
    signal w_operation_safe          : std_logic;

begin

    o_busy              <= w_busy;
    o_done              <= w_done;
    o_commit_rejected   <= w_rejected;
    o_reject_error      <= w_reject_error;
    o_error             <= w_error;
    o_recovery_required <= w_recovery;
    o_active_valid      <= w_active_valid;
    o_active            <= w_active;
    o_proc_enable       <= w_proc_enable;
    o_proc_active_valid <= w_proc_active_valid;
    o_proc_active       <= w_proc_active;
    o_tdc_enable        <= w_tdc_enable;
    o_tdc_active_valid  <= w_tdc_active_valid;
    o_tdc_active        <= w_tdc_active;
    o_tdc_register_image <= w_tdc_register_image;
    o_tdc_config_apply <= w_tdc_config_apply;
    o_operation_state   <= w_operation_state_proc;
    o_operation_safe_to_prepare <= w_operation_safe;

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
            i_cfg_active_gpx_image  => w_gpx_active_image,
            i_operation_status      => w_operation_state_csr,
            i_operation_command_ready => w_operation_command_ready,
            i_operation_command_busy  => w_operation_command_busy,
            i_operation_command_rejected => w_operation_cdc_rejected,
            i_system_command_ready    => i_system_command_ready,
            i_system_command_rejected => i_system_command_rejected,
            i_runtime_irq          => i_runtime_irq,
            o_diag_request_valid   => o_diag_request_valid,
            i_diag_request_ready   => i_diag_request_ready,
            o_diag_request_index   => o_diag_request_index,
            i_diag_response_valid  => i_diag_response_valid,
            o_diag_response_ready  => o_diag_response_ready,
            i_diag_response        => i_diag_response,
            o_shadow             => w_shadow,
            o_gpx_image_shadow   => w_gpx_shadow_image,
            o_commit             => w_commit,
            o_clear_status       => o_clear_status,
            o_soft_reset_request => o_soft_reset_request,
            o_operation_command_valid => w_operation_command_valid,
            o_operation_command       => w_operation_command,
            o_irq                => o_irq
        );

    u_gpx_image_transaction : entity work.lidar_gpx_image_transaction
        port map (
            i_clk             => i_csr_clk,
            i_rst_n           => i_csr_rst_n,
            i_commit          => w_commit,
            i_cfg_busy        => w_busy,
            i_cfg_done        => w_done,
            i_cfg_error       => w_error,
            i_shadow_image    => w_gpx_shadow_image,
            o_candidate_image => w_gpx_candidate_image,
            o_active_image    => w_gpx_active_image
        );

    u_operation : entity work.lidar_operation_subsystem
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_csr_clk               => i_csr_clk,
            i_csr_rst_n             => i_csr_rst_n,
            i_csr_command_valid     => w_operation_command_valid,
            i_csr_command           => w_operation_command,
            o_csr_command_ready     => w_operation_command_ready,
            o_csr_command_busy      => w_operation_command_busy,
            o_csr_command_rejected  => w_operation_cdc_rejected,
            i_proc_clk              => i_proc_clk,
            i_proc_rst_n            => i_proc_rst_n,
            i_proc_soft_reset       => i_proc_soft_reset,
            i_external_laser_permit => i_external_laser_permit,
            i_config_enable         => w_proc_enable,
            i_active_valid          => w_proc_active_valid,
            i_active_config         => w_proc_active,
            i_pipeline_idle         => i_proc_safe,
            o_state_proc            => w_operation_state_proc,
            o_state_csr             => w_operation_state_csr,
            o_command_accepted_proc => o_operation_command_accepted,
            o_command_rejected_proc => o_operation_command_rejected,
            o_permit_trip_proc      => o_operation_permit_trip,
            o_safe_to_prepare       => w_operation_safe
        );

    u_config : entity work.lidar_config_subsystem
        generic map (
            G_BUILD_CONFIG     => G_BUILD_CONFIG,
            G_CSR_CLK_MHZ      => G_CSR_CLK_MHZ,
            G_PHASE_TIMEOUT_US => G_PHASE_TIMEOUT_US,
            G_PROC_DEFER_ACTIVATE_ACK => G_PROC_DEFER_ACTIVATE_ACK,
            G_TDC_DEFER_ACTIVATE_ACK => G_TDC_DEFER_ACTIVATE_ACK
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
            i_proc_safe        => w_operation_safe,
            i_tdc_safe         => i_tdc_safe,
            i_proc_activate_complete => i_proc_activate_complete,
            i_proc_activate_fault    => i_proc_activate_fault,
            i_tdc_activate_complete => w_tdc_activate_complete,
            i_tdc_activate_fault    => w_tdc_activate_fault,
            o_busy             => w_busy,
            o_done             => w_done,
            o_commit_rejected  => w_rejected,
            o_reject_error     => w_reject_error,
            o_error            => w_error,
            o_recovery_required => w_recovery,
            o_active_valid     => w_active_valid,
            o_active           => w_active,
            o_proc_enable      => w_proc_enable,
            o_proc_active_valid => w_proc_active_valid,
            o_proc_active      => w_proc_active,
            o_tdc_enable       => w_tdc_enable,
            o_tdc_active_valid => w_tdc_active_valid,
            o_tdc_active       => w_tdc_active,
            o_proc_activate_start => o_proc_activate_start,
            o_tdc_activate_start => w_tdc_activate_start,
            o_prepare_req      => o_prepare_req,
            o_activate_req     => o_activate_req,
            o_release_req      => o_release_req
        );

    gen_tdc_external_apply : if G_TDC_DEFER_ACTIVATE_ACK generate
        u_gpx_activation : entity work.lidar_gpx_config_activation
            port map (
                i_clk               => i_tdc_clk,
                i_rst_n             => i_tdc_rst_n,
                i_activate_start    => w_tdc_activate_start,
                i_candidate_image   => w_gpx_candidate_image,
                i_apply_ready       => i_tdc_config_ready,
                i_apply_done        => i_tdc_config_done,
                i_apply_fault       => i_tdc_config_fault,
                o_register_image    => w_tdc_register_image,
                o_apply             => w_tdc_config_apply,
                o_activate_complete => w_tdc_activate_complete,
                o_activate_fault    => w_tdc_activate_fault,
                o_busy              => open
            );
    end generate gen_tdc_external_apply;

    gen_tdc_immediate_apply : if not G_TDC_DEFER_ACTIVATE_ACK generate
        w_tdc_register_image <= w_gpx_candidate_image;
        w_tdc_config_apply <= '0';
        w_tdc_activate_complete <= '1';
        w_tdc_activate_fault <= '0';
    end generate gen_tdc_immediate_apply;

end architecture rtl;
