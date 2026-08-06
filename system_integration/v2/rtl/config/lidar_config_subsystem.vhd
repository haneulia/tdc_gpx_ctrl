library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

entity lidar_config_subsystem is
    generic (
        G_BUILD_CONFIG       : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_CSR_CLK_MHZ        : positive := 100;
        G_PHASE_TIMEOUT_US   : positive := 1000;
        G_PROC_DEFER_ACTIVATE_ACK : boolean := false;
        G_TDC_DEFER_ACTIVATE_ACK : boolean := false
    );
    port (
        i_csr_clk          : in  std_logic;
        i_csr_rst_n        : in  std_logic;
        i_proc_clk         : in  std_logic;
        i_proc_rst_n       : in  std_logic;
        i_tdc_clk          : in  std_logic;
        i_tdc_rst_n        : in  std_logic;
        i_commit           : in  std_logic;
        i_shadow           : in  lidar_runtime_config_t;
        i_proc_safe        : in  std_logic;
        i_tdc_safe         : in  std_logic;
        i_proc_activate_complete : in std_logic := '0';
        i_proc_activate_fault    : in std_logic := '0';
        i_tdc_activate_complete : in std_logic := '0';
        i_tdc_activate_fault    : in std_logic := '0';
        o_busy             : out std_logic;
        o_done             : out std_logic;
        o_commit_rejected  : out std_logic;
        o_reject_error     : out lidar_cfg_error_t;
        o_error            : out lidar_cfg_error_t;
        o_recovery_required : out std_logic;
        o_active_valid     : out std_logic;
        o_active           : out lidar_active_config_t;
        o_proc_enable      : out std_logic;
        o_proc_active_valid : out std_logic;
        o_proc_active      : out lidar_active_config_t;
        o_tdc_enable       : out std_logic;
        o_tdc_active_valid : out std_logic;
        o_tdc_active       : out lidar_active_config_t;
        o_proc_activate_start : out std_logic;
        o_tdc_activate_start : out std_logic;
        o_prepare_req      : out std_logic;
        o_activate_req     : out std_logic;
        o_release_req      : out std_logic
    );
end entity lidar_config_subsystem;

architecture rtl of lidar_config_subsystem is

    signal prepare_req   : std_logic;
    signal activate_req  : std_logic;
    signal release_req   : std_logic;
    signal candidate     : lidar_active_config_t;
    signal proc_prepare_ack  : std_logic;
    signal proc_activate_ack : std_logic;
    signal proc_release_ack  : std_logic;
    signal proc_fault        : std_logic;
    signal tdc_prepare_ack   : std_logic;
    signal tdc_activate_ack  : std_logic;
    signal tdc_release_ack   : std_logic;
    signal tdc_fault         : std_logic;

begin

    o_prepare_req  <= prepare_req;
    o_activate_req <= activate_req;
    o_release_req  <= release_req;

    u_manager : entity work.lidar_config_manager
        generic map (
            G_BUILD_CONFIG       => G_BUILD_CONFIG,
            G_CSR_CLK_MHZ        => G_CSR_CLK_MHZ,
            G_PHASE_TIMEOUT_US   => G_PHASE_TIMEOUT_US
        )
        port map (
            i_clk               => i_csr_clk,
            i_rst_n             => i_csr_rst_n,
            i_commit            => i_commit,
            i_shadow            => i_shadow,
            i_proc_prepare_ack  => proc_prepare_ack,
            i_proc_activate_ack => proc_activate_ack,
            i_proc_release_ack  => proc_release_ack,
            i_proc_fault        => proc_fault,
            i_tdc_prepare_ack   => tdc_prepare_ack,
            i_tdc_activate_ack  => tdc_activate_ack,
            i_tdc_release_ack   => tdc_release_ack,
            i_tdc_fault         => tdc_fault,
            o_prepare_req       => prepare_req,
            o_activate_req      => activate_req,
            o_release_req       => release_req,
            o_candidate         => candidate,
            o_busy              => o_busy,
            o_done              => o_done,
            o_commit_rejected   => o_commit_rejected,
            o_reject_error      => o_reject_error,
            o_error             => o_error,
            o_recovery_required => o_recovery_required,
            o_active_valid      => o_active_valid,
            o_active            => o_active
        );

    u_proc_gateway : entity work.lidar_config_gateway
        generic map (
            G_DEFER_ACTIVATE_ACK => G_PROC_DEFER_ACTIVATE_ACK
        )
        port map (
            i_csr_clk         => i_csr_clk,
            i_csr_rst_n       => i_csr_rst_n,
            i_domain_clk      => i_proc_clk,
            i_domain_rst_n    => i_proc_rst_n,
            i_prepare_req     => prepare_req,
            i_activate_req    => activate_req,
            i_release_req     => release_req,
            i_candidate       => candidate,
            i_safe_to_prepare => i_proc_safe,
            i_activate_complete => i_proc_activate_complete,
            i_activate_fault    => i_proc_activate_fault,
            o_prepare_ack     => proc_prepare_ack,
            o_activate_ack    => proc_activate_ack,
            o_release_ack     => proc_release_ack,
            o_activate_start  => o_proc_activate_start,
            o_fault_csr       => proc_fault,
            o_domain_enable   => o_proc_enable,
            o_active_valid    => o_proc_active_valid,
            o_active          => o_proc_active
        );

    u_tdc_gateway : entity work.lidar_config_gateway
        generic map (
            G_DEFER_ACTIVATE_ACK => G_TDC_DEFER_ACTIVATE_ACK
        )
        port map (
            i_csr_clk         => i_csr_clk,
            i_csr_rst_n       => i_csr_rst_n,
            i_domain_clk      => i_tdc_clk,
            i_domain_rst_n    => i_tdc_rst_n,
            i_prepare_req     => prepare_req,
            i_activate_req    => activate_req,
            i_release_req     => release_req,
            i_candidate       => candidate,
            i_safe_to_prepare => i_tdc_safe,
            i_activate_complete => i_tdc_activate_complete,
            i_activate_fault    => i_tdc_activate_fault,
            o_prepare_ack     => tdc_prepare_ack,
            o_activate_ack    => tdc_activate_ack,
            o_release_ack     => tdc_release_ack,
            o_activate_start  => o_tdc_activate_start,
            o_fault_csr       => tdc_fault,
            o_domain_enable   => o_tdc_enable,
            o_active_valid    => o_tdc_active_valid,
            o_active          => o_tdc_active
        );

end architecture rtl;
