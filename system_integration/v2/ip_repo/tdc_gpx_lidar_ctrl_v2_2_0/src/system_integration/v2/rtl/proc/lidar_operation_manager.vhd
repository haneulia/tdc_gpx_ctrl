library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Single Processing-domain owner of RUN/STOP, ARM/DISARM and laser permission.
-- Configuration validity can gate operation but never creates operation state.
entity lidar_operation_manager is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                   : in  std_logic;
        i_rst_n                 : in  std_logic;
        i_soft_reset            : in  std_logic := '0';
        i_command_valid         : in  std_logic;
        i_command               : in  operation_command_t;
        i_command_source_online : in  std_logic;
        i_external_laser_permit : in  std_logic;
        i_config_enable         : in  std_logic;
        i_active_valid          : in  std_logic;
        i_active_config         : in  lidar_active_config_t;
        i_pipeline_idle         : in  std_logic;

        o_state                 : out operation_state_t;
        o_command_accepted      : out std_logic;
        o_command_rejected      : out std_logic;
        o_permit_trip           : out std_logic;
        o_safe_to_prepare       : out std_logic
    );
end entity lidar_operation_manager;

architecture rtl of lidar_operation_manager is

    signal permit_meta_r : std_logic := '0';
    signal permit_sync_r : std_logic := '0';

    signal running_r : std_logic := '0';
    signal armed_r   : std_logic := '0';
    signal command_accepted_r : std_logic := '0';
    signal command_rejected_r : std_logic := '0';
    signal permit_trip_r      : std_logic := '0';

    signal raw_permit_c         : std_logic;
    signal synced_permit_c      : std_logic;
    signal permit_qualified_c   : std_logic;
    signal config_ready_c       : std_logic;
    signal simulation_mode_c    : std_logic;
    signal processing_enable_c  : std_logic;
    signal physical_enable_c    : std_logic;
    signal simulation_enable_c  : std_logic;
    signal scheduler_enable_c   : std_logic;
    signal state_c              : operation_state_t;

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of permit_meta_r : signal is "TRUE";
    attribute ASYNC_REG of permit_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of permit_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of permit_sync_r : signal is "NO";

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-OP-001 invalid build configuration"
        severity failure;

    -- Assertion requires the synchronized permission. Deassertion also uses
    -- the raw pin so a LOW, X, Z or unconnected input closes the physical gate
    -- without waiting for another Processing clock.
    raw_permit_c <= '1' when i_external_laser_permit = '1' else '0';
    synced_permit_c <= '1' when permit_sync_r = '1' else '0';
    permit_qualified_c <= raw_permit_c and synced_permit_c;
    config_ready_c <= '1' when i_config_enable = '1' and
        i_active_valid = '1' else '0';
    simulation_mode_c <= i_active_config.source.motor.simulation_mode
        when config_ready_c = '1' else '0';

    processing_enable_c <= '1' when running_r = '1' and
        config_ready_c = '1' else '0';
    physical_enable_c <= '1' when running_r = '1' and armed_r = '1' and
        config_ready_c = '1' and simulation_mode_c = '0' and
        permit_qualified_c = '1' else '0';
    simulation_enable_c <= '1' when running_r = '1' and armed_r = '1' and
        config_ready_c = '1' and simulation_mode_c = '1' else '0';
    scheduler_enable_c <= physical_enable_c or simulation_enable_c;

    state_c.running              <= running_r;
    state_c.armed                <= armed_r;
    state_c.external_permit      <= permit_qualified_c;
    state_c.config_ready         <= config_ready_c;
    state_c.processing_enable    <= processing_enable_c;
    state_c.scheduler_enable     <= scheduler_enable_c;
    state_c.physical_fire_enable <= physical_enable_c;
    state_c.simulation_enable    <= simulation_enable_c;

    o_state            <= state_c;
    o_command_accepted <= command_accepted_r;
    o_command_rejected <= command_rejected_r;
    o_permit_trip      <= permit_trip_r;
    o_safe_to_prepare  <= '1' when scheduler_enable_c = '0' and
        i_pipeline_idle = '1' else '0';

    p_permit_sync : process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            permit_meta_r <= '0';
            permit_sync_r <= '0';
        elsif rising_edge(i_clk) then
            permit_meta_r <= i_external_laser_permit;
            permit_sync_r <= permit_meta_r;
        end if;
    end process p_permit_sync;

    p_operation_state : process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            running_r          <= '0';
            armed_r            <= '0';
            command_accepted_r <= '0';
            command_rejected_r <= '0';
            permit_trip_r      <= '0';
        elsif rising_edge(i_clk) then
            command_accepted_r <= '0';
            command_rejected_r <= '0';
            permit_trip_r      <= '0';

            if i_soft_reset = '1' then
                running_r <= '0';
                armed_r   <= '0';
                if i_command_valid = '1' then
                    command_rejected_r <= '1';
                end if;

            -- Losing the active configuration is equivalent to a cold safe
            -- state. A normal atomic commit only lowers config_enable and
            -- therefore preserves RUN/ARM while the pipeline drains.
            elsif i_command_source_online /= '1' or i_active_valid /= '1' then
                running_r <= '0';
                armed_r   <= '0';
                if i_command_valid = '1' then
                    case i_command is
                        when OP_COMMAND_STOP | OP_COMMAND_DISARM =>
                            command_accepted_r <= '1';
                        when others =>
                            command_rejected_r <= '1';
                    end case;
                end if;
            else
                -- A physical permit loss requires an explicit later ARM. The
                -- combinational output gate above has already closed.
                if running_r = '1' and armed_r = '1' and
                   config_ready_c = '1' and simulation_mode_c = '0' and
                   permit_qualified_c /= '1' then
                    armed_r       <= '0';
                    permit_trip_r <= '1';
                end if;

                if i_command_valid = '1' then
                    case i_command is
                        when OP_COMMAND_STOP =>
                            -- STOP cannot leave a latent ARM that would resume
                            -- laser operation after a later RUN.
                            running_r          <= '0';
                            armed_r            <= '0';
                            command_accepted_r <= '1';

                        when OP_COMMAND_DISARM =>
                            armed_r            <= '0';
                            command_accepted_r <= '1';

                        when OP_COMMAND_RUN =>
                            if config_ready_c = '1' then
                                running_r          <= '1';
                                command_accepted_r <= '1';
                            else
                                command_rejected_r <= '1';
                            end if;

                        when OP_COMMAND_ARM =>
                            if running_r = '1' and config_ready_c = '1' and
                               (simulation_mode_c = '1' or
                                permit_qualified_c = '1') then
                                armed_r            <= '1';
                                command_accepted_r <= '1';
                            else
                                armed_r            <= '0';
                                command_rejected_r <= '1';
                            end if;

                        when others =>
                            command_rejected_r <= '1';
                    end case;
                end if;
            end if;
        end if;
    end process p_operation_state;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert not (physical_enable_c = '1' and
                            simulation_enable_c = '1')
                    report "V2-OP-002 physical and simulation enable overlap"
                    severity failure;
                if physical_enable_c = '1' then
                    assert raw_permit_c = '1' and permit_sync_r = '1'
                        report "V2-OP-003 physical fire without permit"
                        severity failure;
                end if;
                if scheduler_enable_c = '1' then
                    assert running_r = '1' and armed_r = '1' and
                        config_ready_c = '1'
                        report "V2-OP-004 scheduler enable without operation"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
