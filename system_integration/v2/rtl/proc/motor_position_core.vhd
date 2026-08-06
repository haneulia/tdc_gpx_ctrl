library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- B0 Processing boundary. Physical encoder pins use explicit four-stage
-- synchronizers. Virtual A/B/Z use one common register boundary so the
-- combinational Z qualification cannot extend into the position decoder.
-- The selected source is decoded into one registered event record.
entity motor_position_core is
    port (
        i_clk                       : in  std_logic;
        i_rst_n                     : in  std_logic;
        i_enable                    : in  std_logic;
        i_active_valid              : in  std_logic;
        i_active_config             : in  lidar_active_config_t;
        i_enc_a                     : in  std_logic;
        i_enc_b                     : in  std_logic;
        i_enc_z                     : in  std_logic;
        i_clear_diagnostics         : in  std_logic;
        o_position_event            : out position_event_t;
        o_current_position          : out position_t;
        o_current_direction         : out direction_t;
        o_invalid_transition        : out std_logic;
        o_invalid_transition_sticky : out std_logic;
        o_invalid_transition_count  : out u32_t;
        o_source_switch             : out std_logic;
        o_virtual_a                 : out std_logic;
        o_virtual_b                 : out std_logic;
        o_virtual_z                 : out std_logic;
        o_virtual_z_fault           : out virtual_z_fault_t
    );
end entity motor_position_core;

architecture rtl of motor_position_core is

    type transition_decode_t is record
        motion : boolean;
        cw     : boolean;
    end record transition_decode_t;

    function fn_decode_transition(
        mode      : decode_mode_t;
        previous_a: std_logic;
        previous_b: std_logic;
        current_a : std_logic;
        current_b : std_logic
    ) return transition_decode_t is
        variable result : transition_decode_t := (
            motion => false,
            cw     => true
        );
    begin
        case mode is
            when DECODE_X1 =>
                if previous_a = '0' and current_a = '1' then
                    result.motion := true;
                    result.cw := current_b = '0';
                end if;

            when DECODE_X2 =>
                if previous_a = '0' and current_a = '1' then
                    result.motion := true;
                    result.cw := current_b = '0';
                elsif previous_a = '1' and current_a = '0' then
                    result.motion := true;
                    result.cw := current_b = '1';
                end if;

            when DECODE_X4 =>
                if previous_a = '0' and current_a = '1' then
                    result.motion := true;
                    result.cw := current_b = '0';
                elsif previous_a = '1' and current_a = '0' then
                    result.motion := true;
                    result.cw := current_b = '1';
                elsif previous_b = '0' and current_b = '1' then
                    result.motion := true;
                    result.cw := current_a = '1';
                elsif previous_b = '1' and current_b = '0' then
                    result.motion := true;
                    result.cw := current_a = '0';
                end if;
        end case;
        return result;
    end function fn_decode_transition;

    signal phy_a_sync_r : std_logic_vector(3 downto 0) := (others => '0');
    signal phy_b_sync_r : std_logic_vector(3 downto 0) := (others => '0');
    signal phy_z_sync_r : std_logic_vector(3 downto 0) := (others => '0');

    signal virtual_a_c       : std_logic;
    signal virtual_b_c       : std_logic;
    signal virtual_z_c       : std_logic;
    signal virtual_a_pipe_r  : std_logic := '0';
    signal virtual_b_pipe_r  : std_logic := '0';
    signal virtual_z_pipe_r  : std_logic := '0';
    signal virtual_z_fault_c : virtual_z_fault_t;
    signal virtual_enable_c  : std_logic;
    signal selected_a_c      : std_logic;
    signal selected_b_c      : std_logic;
    signal selected_z_c      : std_logic;
    signal invalid_c         : std_logic;
    signal z_rise_c          : std_logic;
    signal transition_c      : transition_decode_t;

    signal decode_mode_r     : decode_mode_t := DECODE_X4;
    signal physical_invert_r : direction_t := DIRECTION_CW;
    signal simulation_mode_r : std_logic := '0';
    signal count_max_r       : position_t := (others => '0');
    signal active_version_r  : u16_t := (others => '0');

    signal previous_a_r : std_logic := '0';
    signal previous_b_r : std_logic := '0';
    signal previous_z_r : std_logic := '0';

    signal position_r  : position_t := (others => '0');
    signal direction_r : direction_t := DIRECTION_CW;
    signal event_r     : position_event_t := C_POSITION_EVENT_IDLE;

    signal invalid_r        : std_logic := '0';
    signal invalid_sticky_r : std_logic := '0';
    signal invalid_count_r  : u32_t := (others => '0');
    signal source_switch_r  : std_logic := '0';

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of phy_a_sync_r : signal is "TRUE";
    attribute ASYNC_REG of phy_b_sync_r : signal is "TRUE";
    attribute ASYNC_REG of phy_z_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of phy_a_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of phy_b_sync_r : signal is "NO";
    attribute SHREG_EXTRACT of phy_z_sync_r : signal is "NO";

begin

    o_position_event            <= event_r;
    o_current_position          <= position_r;
    o_current_direction         <= direction_r;
    o_invalid_transition        <= invalid_r;
    o_invalid_transition_sticky <= invalid_sticky_r;
    o_invalid_transition_count  <= invalid_count_r;
    o_source_switch             <= source_switch_r;
    o_virtual_a                 <= virtual_a_pipe_r;
    o_virtual_b                 <= virtual_b_pipe_r;
    o_virtual_z                 <= virtual_z_pipe_r;
    o_virtual_z_fault           <= virtual_z_fault_c;

    virtual_enable_c <= i_enable and i_active_valid and simulation_mode_r;
    selected_a_c <= virtual_a_pipe_r when simulation_mode_r = '1'
        else phy_a_sync_r(3);
    selected_b_c <= virtual_b_pipe_r when simulation_mode_r = '1'
        else phy_b_sync_r(3);
    selected_z_c <= virtual_z_pipe_r when simulation_mode_r = '1'
        else phy_z_sync_r(3);
    invalid_c <= '1' when
        selected_a_c /= previous_a_r and
        selected_b_c /= previous_b_r
        else '0';
    z_rise_c <= '1' when selected_z_c = '1' and previous_z_r = '0'
        else '0';
    transition_c <= fn_decode_transition(
        decode_mode_r,
        previous_a_r,
        previous_b_r,
        selected_a_c,
        selected_b_c);

    p_physical_sync : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                phy_a_sync_r <= (others => '0');
                phy_b_sync_r <= (others => '0');
                phy_z_sync_r <= (others => '0');
            else
                phy_a_sync_r <= phy_a_sync_r(2 downto 0) & i_enc_a;
                phy_b_sync_r <= phy_b_sync_r(2 downto 0) & i_enc_b;
                phy_z_sync_r <= phy_z_sync_r(2 downto 0) & i_enc_z;
            end if;
        end if;
    end process p_physical_sync;

    p_virtual_pipeline : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                virtual_a_pipe_r <= '0';
                virtual_b_pipe_r <= '0';
                virtual_z_pipe_r <= '0';
            else
                virtual_a_pipe_r <= virtual_a_c;
                virtual_b_pipe_r <= virtual_b_c;
                virtual_z_pipe_r <= virtual_z_c;
            end if;
        end if;
    end process p_virtual_pipeline;

    u_virtual_source : entity work.motor_virtual_source
        port map (
            i_clk           => i_clk,
            i_rst_n         => i_rst_n,
            i_active_valid  => i_active_valid,
            i_enable        => virtual_enable_c,
            i_active_config => i_active_config,
            o_a             => virtual_a_c,
            o_b             => virtual_b_c,
            o_z             => virtual_z_c,
            o_phase_tick    => open,
            o_position      => open,
            o_z_fault       => virtual_z_fault_c
        );

    p_config : process (i_clk)
        variable new_max_v : position_t;
        variable new_mode_v: std_logic;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                decode_mode_r     <= DECODE_X4;
                physical_invert_r <= DIRECTION_CW;
                simulation_mode_r <= '0';
                count_max_r       <= (others => '0');
                active_version_r  <= (others => '0');
                source_switch_r   <= '0';
            else
                source_switch_r <= '0';
                if i_enable = '0' and i_active_valid = '1' then
                    if i_active_config.derived.total_states <= 1 then
                        new_max_v := (others => '0');
                    else
                        new_max_v := resize(
                            i_active_config.derived.total_states - 1,
                            new_max_v'length);
                    end if;
                    new_mode_v :=
                        i_active_config.source.motor.simulation_mode;

                    if new_mode_v /= simulation_mode_r then
                        source_switch_r <= '1';
                    end if;
                    decode_mode_r <=
                        i_active_config.source.motor.decode_mode;
                    physical_invert_r <=
                        i_active_config.source.motor.direction;
                    simulation_mode_r <= new_mode_v;
                    count_max_r       <= new_max_v;
                    active_version_r  <= i_active_config.version;
                end if;
            end if;
        end if;
    end process p_config;

    p_decode : process (i_clk)
        variable event_v    : position_event_t;
        variable new_max_v  : position_t;
        variable next_pos_v : position_t;
        variable cw_v       : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                previous_a_r <= '0';
                previous_b_r <= '0';
                previous_z_r <= '0';
                position_r   <= (others => '0');
                direction_r  <= DIRECTION_CW;
                event_r      <= C_POSITION_EVENT_IDLE;
            else
                event_v := C_POSITION_EVENT_IDLE;
                event_v.position       := position_r;
                event_v.direction      := direction_r;
                event_v.source_sim     := simulation_mode_r;
                event_v.active_version := active_version_r;

                if i_enable = '0' or i_active_valid = '0' then
                    if i_active_valid = '1' then
                        if i_active_config.derived.total_states <= 1 then
                            new_max_v := (others => '0');
                        else
                            new_max_v := resize(
                                i_active_config.derived.total_states - 1,
                                new_max_v'length);
                        end if;
                        if position_r > new_max_v then
                            position_r <= new_max_v;
                        end if;

                        if i_active_config.source.motor.simulation_mode = '1' then
                            previous_a_r <= virtual_a_pipe_r;
                            previous_b_r <= virtual_b_pipe_r;
                            previous_z_r <= virtual_z_pipe_r;
                        else
                            previous_a_r <= phy_a_sync_r(3);
                            previous_b_r <= phy_b_sync_r(3);
                            previous_z_r <= phy_z_sync_r(3);
                        end if;
                    else
                        previous_a_r <= selected_a_c;
                        previous_b_r <= selected_b_c;
                        previous_z_r <= selected_z_c;
                    end if;
                else
                    previous_a_r <= selected_a_c;
                    previous_b_r <= selected_b_c;
                    previous_z_r <= selected_z_c;

                    event_v.source_latency_valid := '1';
                    if simulation_mode_r = '1' then
                        event_v.source_latency_clks :=
                            C_POSITION_VIRTUAL_LATENCY_CLKS;
                    else
                        event_v.source_latency_clks :=
                            C_POSITION_PHYSICAL_LATENCY_CLKS;
                    end if;

                    if z_rise_c = '1' then
                        position_r       <= (others => '0');
                        event_v.valid    := '1';
                        event_v.position := (others => '0');
                        event_v.z_event  := '1';
                    elsif invalid_c = '0' and transition_c.motion then
                        cw_v := transition_c.cw;
                        if simulation_mode_r = '0'
                           and physical_invert_r = DIRECTION_CCW then
                            cw_v := not cw_v;
                        end if;

                        if cw_v then
                            direction_r <= DIRECTION_CW;
                            event_v.direction := DIRECTION_CW;
                            if position_r >= count_max_r then
                                next_pos_v := (others => '0');
                            else
                                next_pos_v := position_r + 1;
                            end if;
                        else
                            direction_r <= DIRECTION_CCW;
                            event_v.direction := DIRECTION_CCW;
                            if position_r = 0 then
                                next_pos_v := count_max_r;
                            else
                                next_pos_v := position_r - 1;
                            end if;
                        end if;

                        position_r       <= next_pos_v;
                        event_v.valid    := '1';
                        event_v.position := next_pos_v;
                    end if;
                end if;
                event_r <= event_v;
            end if;
        end if;
    end process p_decode;

    p_diagnostics : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                invalid_r        <= '0';
                invalid_sticky_r <= '0';
                invalid_count_r  <= (others => '0');
            else
                invalid_r <= '0';
                if i_clear_diagnostics = '1' then
                    invalid_sticky_r <= '0';
                    invalid_count_r  <= (others => '0');
                end if;
                if i_enable = '1' and i_active_valid = '1'
                   and invalid_c = '1' then
                    invalid_r        <= '1';
                    invalid_sticky_r <= '1';
                    invalid_count_r  <= invalid_count_r + 1;
                end if;
            end if;
        end if;
    end process p_diagnostics;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_enable = '1' then
                assert i_active_valid = '1'
                    report "V2-MOTOR-004 enable without active configuration"
                    severity failure;
                assert i_active_config.version = active_version_r
                    report "V2-MOTOR-005 active version changed while enabled"
                    severity failure;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
