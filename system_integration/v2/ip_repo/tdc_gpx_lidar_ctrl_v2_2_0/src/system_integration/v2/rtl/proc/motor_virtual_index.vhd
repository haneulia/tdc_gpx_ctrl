library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Processing-domain virtual index generator. A complete revolution qualifies
-- Z after reset or configuration quiesce. Width zero preserves the legacy
-- contract: Z starts at wrap and remains active until the next phase event.
entity motor_virtual_index is
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;
        i_enable         : in  std_logic;
        i_z_offset       : in  position_t;
        i_z_width        : in  position_t;
        i_z_early        : in  std_logic;
        i_phase_tick     : in  std_logic;
        i_ticks_next     : in  u32_t;
        i_pre_wrap_event : in  std_logic;
        i_wrap_event     : in  std_logic;
        o_z              : out std_logic;
        o_fault          : out virtual_z_fault_t
    );
end entity motor_virtual_index;

architecture rtl of motor_virtual_index is

    type z_state_t is (Z_IDLE, Z_DELAY, Z_ACTIVE);

    signal guard_r        : std_logic := '1';
    signal state_r        : z_state_t := Z_IDLE;
    signal delay_count_r  : u32_t := (others => '0');
    signal active_count_r : position_t := (others => '0');
    signal z_hold_r       : std_logic := '0';
    signal fault_r        : virtual_z_fault_t := (others => '0');

    signal width_clamped_r   : position_t := (others => '0');
    signal offset_m2_r      : position_t := (others => '0');
    signal offset_zero_r    : std_logic := '1';
    signal offset_one_r     : std_logic := '0';
    signal early_delay_m2_r : u32_t := (others => '0');
    signal early_start_r    : std_logic := '0';
    signal early_ok_r       : std_logic := '0';

    signal legacy_c    : std_logic;
    signal start_now_c : std_logic;

begin

    legacy_c <= '1' when i_z_width = 0 else '0';

    -- A registered wrap event supplies the first output cycle directly when
    -- no late offset is requested. The state machine owns continuation only.
    start_now_c <= '1' when
        guard_r = '0' and i_wrap_event = '1' and
        (legacy_c = '1' or (i_z_early = '0' and i_z_offset = 0))
        else '0';

    o_z     <= start_now_c or z_hold_r;
    o_fault <= fault_r;

    -- These values are stable before the registered wrap/pre-wrap event is
    -- consumed. Keeping the wide subtract out of the FSM removes a long
    -- ticks_next-to-state-counter path without adding Z output latency.
    p_precompute : process (i_clk)
        variable width_v  : position_t;
        variable offset_v : u32_t;
        variable delay_v  : u32_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_enable = '0' then
                width_clamped_r  <= (others => '0');
                offset_m2_r      <= (others => '0');
                offset_zero_r    <= '1';
                offset_one_r     <= '0';
                early_delay_m2_r <= (others => '0');
                early_start_r    <= '0';
                early_ok_r       <= '0';
            else
                width_v := i_z_width;
                if i_ticks_next /= 0 and
                   resize(i_z_width, i_ticks_next'length) > i_ticks_next then
                    width_v := resize(i_ticks_next, width_v'length);
                end if;
                width_clamped_r <= width_v;

                if i_z_offset = 0 then
                    offset_zero_r <= '1';
                    offset_one_r  <= '0';
                    offset_m2_r   <= (others => '0');
                elsif i_z_offset = 1 then
                    offset_zero_r <= '0';
                    offset_one_r  <= '1';
                    offset_m2_r   <= (others => '0');
                else
                    offset_zero_r <= '0';
                    offset_one_r  <= '0';
                    offset_m2_r   <= i_z_offset - 2;
                end if;

                offset_v := resize(i_z_offset, offset_v'length);
                early_delay_m2_r <= i_ticks_next - offset_v - 2;
                if i_ticks_next <= offset_v then
                    early_start_r    <= '1';
                    early_ok_r       <= '0';
                else
                    delay_v := i_ticks_next - offset_v;
                    early_ok_r <= '1';
                    if delay_v <= 1 then
                        early_start_r    <= '1';
                    else
                        early_start_r    <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process p_precompute;

    p_index : process (i_clk)
        variable trigger_late_v  : boolean;
        variable trigger_early_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_enable = '0' then
                guard_r        <= '1';
                state_r        <= Z_IDLE;
                delay_count_r  <= (others => '0');
                active_count_r <= (others => '0');
                z_hold_r       <= '0';
                fault_r        <= (others => '0');
            else
                if guard_r = '1' and i_wrap_event = '1' then
                    guard_r <= '0';
                end if;

                if legacy_c = '1' then
                    state_r        <= Z_IDLE;
                    delay_count_r  <= (others => '0');
                    active_count_r <= (others => '0');
                    if guard_r = '1' then
                        z_hold_r <= '0';
                    elsif i_wrap_event = '1' then
                        z_hold_r <= '1';
                    elsif i_phase_tick = '1' then
                        z_hold_r <= '0';
                    end if;
                else
                    trigger_late_v := guard_r = '0'
                        and i_wrap_event = '1' and i_z_early = '0';
                    trigger_early_v := guard_r = '0'
                        and i_pre_wrap_event = '1' and i_z_early = '1';

                    case state_r is
                        when Z_IDLE =>
                            z_hold_r       <= '0';
                            delay_count_r  <= (others => '0');
                            active_count_r <= (others => '0');

                            if trigger_late_v then
                                if offset_zero_r = '1' then
                                    if width_clamped_r > 1 then
                                        state_r  <= Z_ACTIVE;
                                        active_count_r <= width_clamped_r - 2;
                                        z_hold_r <= '1';
                                    end if;
                                elsif offset_one_r = '1' then
                                    state_r  <= Z_ACTIVE;
                                    active_count_r <= width_clamped_r - 1;
                                    z_hold_r <= '1';
                                else
                                    state_r <= Z_DELAY;
                                    delay_count_r <= resize(
                                        offset_m2_r,
                                        delay_count_r'length);
                                end if;
                            elsif trigger_early_v then
                                if early_ok_r = '0' then
                                    fault_r(C_VIRTUAL_Z_EARLY_OVER_BIT) <= '1';
                                end if;
                                if early_start_r = '1' then
                                    state_r  <= Z_ACTIVE;
                                    active_count_r <= width_clamped_r - 1;
                                    z_hold_r <= '1';
                                else
                                    state_r <= Z_DELAY;
                                    delay_count_r <= early_delay_m2_r;
                                end if;
                            end if;

                        when Z_DELAY =>
                            z_hold_r <= '0';
                            if trigger_late_v or trigger_early_v then
                                fault_r(C_VIRTUAL_Z_COLLISION_BIT) <= '1';
                            end if;
                            if delay_count_r = 0 then
                                state_r  <= Z_ACTIVE;
                                active_count_r <= width_clamped_r - 1;
                                z_hold_r <= '1';
                            else
                                delay_count_r <= delay_count_r - 1;
                            end if;

                        when Z_ACTIVE =>
                            if trigger_late_v or trigger_early_v then
                                fault_r(C_VIRTUAL_Z_COLLISION_BIT) <= '1';
                            end if;
                            if active_count_r = 0 then
                                state_r  <= Z_IDLE;
                                z_hold_r <= '0';
                            else
                                active_count_r <= active_count_r - 1;
                                z_hold_r <= '1';
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_index;

end architecture rtl;
