library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- F3a operation boundary: acknowledged command CDC plus the single
-- Processing-domain operation/safety owner.
entity lidar_operation_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_csr_clk                 : in  std_logic;
        i_csr_rst_n               : in  std_logic;
        i_csr_command_valid       : in  std_logic;
        i_csr_command             : in  operation_command_t;
        o_csr_command_ready       : out std_logic;
        o_csr_command_busy        : out std_logic;
        o_csr_command_rejected    : out std_logic;

        i_proc_clk                : in  std_logic;
        i_proc_rst_n              : in  std_logic;
        i_proc_soft_reset         : in  std_logic := '0';
        i_external_laser_permit   : in  std_logic;
        i_config_enable           : in  std_logic;
        i_active_valid            : in  std_logic;
        i_active_config           : in  lidar_active_config_t;
        i_pipeline_idle           : in  std_logic;

        o_state_proc              : out operation_state_t;
        o_state_csr               : out operation_state_t;
        o_command_accepted_proc   : out std_logic;
        o_command_rejected_proc   : out std_logic;
        o_permit_trip_proc        : out std_logic;
        o_safe_to_prepare         : out std_logic
    );
end entity lidar_operation_subsystem;

architecture rtl of lidar_operation_subsystem is

    signal command_valid_proc : std_logic;
    signal command_proc       : operation_command_t;
    signal command_source_online_proc : std_logic;
    signal state_proc         : operation_state_t;

    signal status_source_r    : operation_status_bits_t := (others => '0');
    signal status_meta_r      : operation_status_bits_t := (others => '0');
    signal status_sync_r      : operation_status_bits_t := (others => '0');
    signal state_csr_c        : operation_state_t := C_OPERATION_STATE_SAFE;

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of status_meta_r : signal is "TRUE";
    attribute ASYNC_REG of status_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of status_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of status_sync_r : signal is "NO";

begin

    o_state_proc <= state_proc;
    o_state_csr <= state_csr_c;

    -- scheduler_enable은 physical/simulation 실행 허가의 OR 결과다. 이 값을
    -- 별도 CDC bit로 다시 넘기면 시뮬레이션 모드에서 동일 source FF가 두
    -- synchronizer로 fan-out되어 CDC-11이 발생한다. 독립 상태 bit만 2-FF로
    -- 동기화한 뒤 CSR 도메인에서 동일 의미를 다시 도출한다.
    p_state_csr : process (all)
        variable value : operation_state_t;
    begin
        value := fn_unpack_operation_state(status_sync_r);
        value.scheduler_enable := value.physical_fire_enable or
                                  value.simulation_enable;
        state_csr_c <= value;
    end process p_state_csr;

    u_command_cdc : entity work.lidar_operation_command_cdc
        port map (
            i_source_clk      => i_csr_clk,
            i_source_rst_n    => i_csr_rst_n,
            i_command_valid   => i_csr_command_valid,
            i_command         => i_csr_command,
            o_source_ready    => o_csr_command_ready,
            o_source_busy     => o_csr_command_busy,
            o_source_rejected => o_csr_command_rejected,
            i_domain_clk      => i_proc_clk,
            i_domain_rst_n    => i_proc_rst_n,
            o_domain_source_online => command_source_online_proc,
            o_command_valid   => command_valid_proc,
            o_command         => command_proc
        );

    u_manager : entity work.lidar_operation_manager
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                   => i_proc_clk,
            i_rst_n                 => i_proc_rst_n,
            i_soft_reset            => i_proc_soft_reset,
            i_command_valid         => command_valid_proc,
            i_command               => command_proc,
            i_command_source_online => command_source_online_proc,
            i_external_laser_permit => i_external_laser_permit,
            i_config_enable         => i_config_enable,
            i_active_valid          => i_active_valid,
            i_active_config         => i_active_config,
            i_pipeline_idle         => i_pipeline_idle,
            o_state                 => state_proc,
            o_command_accepted      => o_command_accepted_proc,
            o_command_rejected      => o_command_rejected_proc,
            o_permit_trip           => o_permit_trip_proc,
            o_safe_to_prepare       => o_safe_to_prepare
        );

    -- Register every status bit before the asynchronous crossing. This avoids
    -- placing decoded combinational operation logic ahead of a synchronizer.
    p_status_source : process (i_proc_clk, i_proc_rst_n)
    begin
        if i_proc_rst_n = '0' then
            status_source_r <= (others => '0');
        elsif rising_edge(i_proc_clk) then
            status_source_r <= fn_pack_operation_state(state_proc);
            status_source_r(5) <= '0';
        end if;
    end process p_status_source;

    p_status_sync : process (i_csr_clk, i_csr_rst_n)
    begin
        if i_csr_rst_n = '0' then
            status_meta_r <= (others => '0');
            status_sync_r <= (others => '0');
        elsif rising_edge(i_csr_clk) then
            status_meta_r <= status_source_r;
            status_sync_r <= status_meta_r;
        end if;
    end process p_status_sync;

end architecture rtl;
