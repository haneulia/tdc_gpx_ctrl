library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;

-- Optional one-pulse-per-channel synthetic Echo source.
-- This entity is instantiated only in a simulation-enabled build.
entity echo_sim_source is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;
        i_shot_start     : in  shot_start_event_t;
        i_shot_result    : in  shot_result_t;
        i_profile_ready  : in  std_logic;
        i_active_clocks  : in  echo_delay_clocks_array_t;
        o_stop_pulse     : out std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);
        o_busy           : out std_logic;
        o_profile_not_ready : out std_logic
    );
end entity echo_sim_source;

architecture rtl of echo_sim_source is

    constant C_NUM_CHANNELS : positive :=
        fn_echo_channel_count(G_BUILD_CONFIG);
    constant C_ZERO_PENDING : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0) := (others => '0');

    signal counter_r : echo_delay_clocks_array_t :=
        C_ECHO_DELAY_CLOCKS_CLEAR;
    signal pending_r : std_logic_vector(C_NUM_CHANNELS - 1 downto 0) :=
        (others => '0');
    signal pulse_r : std_logic_vector(C_NUM_CHANNELS - 1 downto 0) :=
        (others => '0');
    signal profile_not_ready_r : std_logic := '0';

begin

    assert G_BUILD_CONFIG.enable_echo_simulation
        report "V2-ECHO-004 simulation source in a production build"
        severity failure;

    o_stop_pulse <= pulse_r;
    o_profile_not_ready <= profile_not_ready_r;
    o_busy <= '1' when pending_r /= C_ZERO_PENDING or
        pulse_r /= C_ZERO_PENDING else '0';

    p_source : process (i_clk)
    begin
        if rising_edge(i_clk) then
            pulse_r <= (others => '0');
            profile_not_ready_r <= '0';

            if i_rst_n = '0' then
                pending_r <= (others => '0');
            elsif i_shot_result.valid = '1' then
                pending_r <= (others => '0');
            elsif i_shot_start.valid = '1' then
                pending_r <= (others => '0');
                if i_shot_start.request.source_sim = '1' and
                   i_profile_ready = '1' then
                    for channel in 0 to C_NUM_CHANNELS - 1 loop
                        counter_r(channel) <= i_active_clocks(channel);
                        if i_active_clocks(channel) /= 0 then
                            pending_r(channel) <= '1';
                        end if;
                    end loop;
                elsif i_shot_start.request.source_sim = '1' then
                    profile_not_ready_r <= '1';
                end if;
            else
                for channel in 0 to C_NUM_CHANNELS - 1 loop
                    if pending_r(channel) = '1' then
                        if counter_r(channel) = 1 then
                            pulse_r(channel)   <= '1';
                            pending_r(channel) <= '0';
                        else
                            counter_r(channel) <= counter_r(channel) - 1;
                        end if;
                    end if;
                end loop;
            end if;
        end if;
    end process p_source;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert not (
                    i_shot_start.valid = '1' and
                    i_shot_result.valid = '1')
                    report "V2-ECHO-005 simultaneous Shot start/result"
                    severity failure;
                assert not (
                    i_shot_start.valid = '1' and
                    i_shot_start.request.source_sim = '1' and
                    i_profile_ready = '0')
                    report "V2-ECHO-013 simulation Shot before delay profile ready"
                    severity warning;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
