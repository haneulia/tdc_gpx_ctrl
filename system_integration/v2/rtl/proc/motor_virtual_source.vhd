library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Registered Processing-domain x4 quadrature source. Fractional state timing
-- uses the committed low/high interval and Bresenham remainder. Configuration
-- is stable whenever i_enable is high; quiesce restarts the virtual phase at
-- 00 so a later release has a deterministic first transition.
entity motor_virtual_source is
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;
        i_active_valid  : in  std_logic;
        i_enable        : in  std_logic;
        i_active_config : in  lidar_active_config_t;
        o_a             : out std_logic;
        o_b             : out std_logic;
        o_z             : out std_logic;
        o_phase_tick    : out std_logic;
        o_position      : out position_t;
        o_z_fault       : out virtual_z_fault_t
    );
end entity motor_virtual_source;

architecture rtl of motor_virtual_source is

    signal remain_r      : u32_t := (others => '0');
    signal accumulator_r : u16_t := (others => '0');
    signal threshold_r   : u16_t := (others => '0');
    signal ticks_lo_m1_r : u32_t := (others => '0');
    signal ticks_hi_m1_r : u32_t := (others => '0');
    signal ticks_lo_cfg_r : u32_t := (others => '0');
    signal ticks_hi_cfg_r : u32_t := (others => '0');
    signal hi_count_cfg_r : u16_t := (others => '0');
    signal ticks_next_r  : u32_t := (others => '0');
    signal phase_tick_r  : std_logic := '0';

    signal phase_r          : unsigned(1 downto 0) := "11";
    signal a_r              : std_logic := '0';
    signal b_r              : std_logic := '0';
    signal position_r       : position_t := (others => '0');
    signal count_max_r      : position_t := (others => '0');
    signal direction_cfg_r  : direction_t := DIRECTION_CW;
    signal z_offset_cfg_r   : position_t := (others => '0');
    signal z_width_cfg_r    : position_t := (others => '0');
    signal z_early_cfg_r    : std_logic := '0';
    signal pre_wrap_event_r : std_logic := '0';
    signal wrap_event_r     : std_logic := '0';
    signal z_c              : std_logic;
    signal z_fault_c        : virtual_z_fault_t;

begin

    o_a          <= a_r;
    o_b          <= b_r;
    o_z          <= z_c;
    o_phase_tick <= phase_tick_r;
    o_position   <= position_r;
    o_z_fault    <= z_fault_c;

    p_timing : process (i_clk)
        variable total_v    : u16_t;
        variable hi_count_v : u16_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_active_valid = '0' then
                remain_r      <= (others => '0');
                accumulator_r <= (others => '0');
                threshold_r   <= to_unsigned(1, threshold_r'length);
                ticks_lo_m1_r <= (others => '0');
                ticks_hi_m1_r <= (others => '0');
                ticks_lo_cfg_r <= (others => '0');
                ticks_hi_cfg_r <= (others => '0');
                hi_count_cfg_r <= (others => '0');
                ticks_next_r  <= (others => '0');
                phase_tick_r  <= '0';
            elsif i_enable = '0' then
                total_v := i_active_config.derived.total_states;
                hi_count_v := resize(
                    i_active_config.source.motor.virtual_hi_count,
                    hi_count_v'length);
                hi_count_cfg_r <= hi_count_v;
                ticks_lo_cfg_r <=
                    i_active_config.source.motor.virtual_ticks_lo;
                ticks_hi_cfg_r <=
                    i_active_config.derived.virtual_ticks_hi;
                if i_active_config.source.motor.virtual_ticks_lo = 0 then
                    remain_r      <= (others => '0');
                    ticks_lo_m1_r <= (others => '0');
                else
                    remain_r <=
                        i_active_config.source.motor.virtual_ticks_lo - 1;
                    ticks_lo_m1_r <=
                        i_active_config.source.motor.virtual_ticks_lo - 1;
                end if;
                if i_active_config.derived.virtual_ticks_hi = 0 then
                    ticks_hi_m1_r <= (others => '0');
                else
                    ticks_hi_m1_r <=
                        i_active_config.derived.virtual_ticks_hi - 1;
                end if;
                accumulator_r <= hi_count_v;
                if total_v > hi_count_v then
                    threshold_r <= total_v - hi_count_v;
                else
                    threshold_r <= to_unsigned(1, threshold_r'length);
                end if;
                ticks_next_r <=
                    i_active_config.source.motor.virtual_ticks_lo;
                phase_tick_r <= '0';
            elsif remain_r = 0 then
                phase_tick_r <= '1';
                if accumulator_r >= threshold_r then
                    accumulator_r <= accumulator_r - threshold_r;
                    remain_r <= ticks_hi_m1_r;
                    ticks_next_r <= ticks_hi_cfg_r;
                else
                    accumulator_r <= accumulator_r + hi_count_cfg_r;
                    remain_r <= ticks_lo_m1_r;
                    ticks_next_r <= ticks_lo_cfg_r;
                end if;
            else
                phase_tick_r <= '0';
                remain_r <= remain_r - 1;
            end if;
        end if;
    end process p_timing;

    p_phase_position : process (i_clk)
        variable phase_next_v : unsigned(1 downto 0);
        variable count_max_v  : position_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_active_valid = '0' then
                phase_r          <= "11";
                a_r              <= '0';
                b_r              <= '0';
                position_r       <= (others => '0');
                count_max_r      <= (others => '0');
                direction_cfg_r  <= DIRECTION_CW;
                z_offset_cfg_r   <= (others => '0');
                z_width_cfg_r    <= (others => '0');
                z_early_cfg_r    <= '0';
                pre_wrap_event_r <= '0';
                wrap_event_r     <= '0';
            elsif i_enable = '0' then
                phase_r          <= "11";
                a_r              <= '0';
                b_r              <= '0';
                position_r       <= (others => '0');
                pre_wrap_event_r <= '0';
                wrap_event_r     <= '0';

                if i_active_config.derived.total_states <= 1 then
                    count_max_r <= (others => '0');
                else
                    count_max_r <= resize(
                        i_active_config.derived.total_states - 1,
                        count_max_r'length);
                end if;
                direction_cfg_r <=
                    i_active_config.source.motor.direction;
                z_offset_cfg_r <=
                    i_active_config.source.motor.z_offset;
                z_width_cfg_r <=
                    i_active_config.source.motor.z_width;
                z_early_cfg_r <=
                    i_active_config.source.motor.z_early;
            else
                pre_wrap_event_r <= '0';
                wrap_event_r     <= '0';

                if phase_tick_r = '1' then
                    if direction_cfg_r = DIRECTION_CW then
                        if phase_r = 3 then
                            phase_next_v := (others => '0');
                        else
                            phase_next_v := phase_r + 1;
                        end if;
                    else
                        if phase_r = 0 then
                            phase_next_v := to_unsigned(3, phase_next_v'length);
                        else
                            phase_next_v := phase_r - 1;
                        end if;
                    end if;

                    phase_r <= phase_next_v;
                    case to_integer(phase_next_v) is
                        when 0      => a_r <= '1'; b_r <= '0';
                        when 1      => a_r <= '1'; b_r <= '1';
                        when 2      => a_r <= '0'; b_r <= '1';
                        when others => a_r <= '0'; b_r <= '0';
                    end case;

                    count_max_v := count_max_r;

                    if direction_cfg_r = DIRECTION_CW then
                        if position_r >= count_max_v then
                            position_r   <= (others => '0');
                            wrap_event_r <= '1';
                        else
                            position_r <= position_r + 1;
                            if count_max_v > 0 and
                               position_r = count_max_v - 1 then
                                pre_wrap_event_r <= '1';
                            end if;
                        end if;
                    else
                        if position_r = 0 then
                            position_r <= count_max_v;
                        else
                            position_r <= position_r - 1;
                            if position_r = 1 then
                                wrap_event_r <= '1';
                            elsif position_r = 2 then
                                pre_wrap_event_r <= '1';
                            end if;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process p_phase_position;

    u_index : entity work.motor_virtual_index
        port map (
            i_clk            => i_clk,
            i_rst_n          => i_rst_n,
            i_enable         => i_enable and i_active_valid,
            i_z_offset       => z_offset_cfg_r,
            i_z_width        => z_width_cfg_r,
            i_z_early        => z_early_cfg_r,
            i_phase_tick     => phase_tick_r,
            i_ticks_next     => ticks_next_r,
            i_pre_wrap_event => pre_wrap_event_r,
            i_wrap_event     => wrap_event_r,
            o_z              => z_c,
            o_fault          => z_fault_c
        );

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_active_valid = '1'
               and i_enable = '1' then
                assert i_active_config.derived.total_states > 1
                    report "V2-MOTOR-001 active total_states is invalid"
                    severity failure;
                assert i_active_config.source.motor.virtual_ticks_lo > 0
                    report "V2-MOTOR-002 active virtual interval is zero"
                    severity failure;
                assert resize(
                    i_active_config.source.motor.virtual_hi_count, 16)
                    < i_active_config.derived.total_states
                    report "V2-MOTOR-003 active virtual remainder is invalid"
                    severity failure;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
