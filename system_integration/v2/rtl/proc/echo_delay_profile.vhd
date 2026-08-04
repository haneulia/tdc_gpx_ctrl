library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_echo_pkg.all;

-- Processing-domain expansion of the compact Echo simulation profile.
--
-- CTL20 carries only CH0_DELAY and CHANNEL_STEP. After each atomic config
-- activation, this block produces one channel entry per clock using:
--
--   delay_ticks(channel) = CH0_DELAY + channel * CHANNEL_STEP
--
-- The sequential accumulator keeps the 32-channel expansion and 5 ns tick
-- conversion away from the physical LVDS-to-STOP timing path.
entity echo_delay_profile is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;
        i_active_valid  : in  std_logic;
        i_active_config : in  lidar_active_config_t;
        o_ready         : out std_logic;
        o_busy          : out std_logic;
        o_profile_version : out u16_t;
        o_active_clocks : out echo_delay_clocks_array_t
    );
end entity echo_delay_profile;

architecture rtl of echo_delay_profile is

    constant C_NUM_CHANNELS : positive :=
        fn_echo_channel_count(G_BUILD_CONFIG);

    type profile_state_t is (
        PROFILE_WAIT_CONFIG,
        PROFILE_BUILD,
        PROFILE_READY
    );

    signal state_r : profile_state_t := PROFILE_WAIT_CONFIG;
    signal target_version_r : u16_t := (others => '0');
    signal profile_version_r : u16_t := (others => '0');
    signal build_index_r : natural range 0 to C_ECHO_MAX_CHANNELS - 1 := 0;
    signal tick_accumulator_r : echo_delay_ticks_t := (others => '0');
    signal channel_step_r : echo_delay_ticks_t := (others => '0');
    signal active_clocks_r : echo_delay_clocks_array_t :=
        C_ECHO_DELAY_CLOCKS_CLEAR;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-ECHO-001 invalid build configuration"
        severity failure;
    assert G_BUILD_CONFIG.enable_echo_receiver and
           G_BUILD_CONFIG.enable_echo_simulation
        report "V2-ECHO-002 delay profile instantiated outside Echo simulation"
        severity failure;

    o_ready <= '1' when state_r = PROFILE_READY and
        i_active_valid = '1' and
        profile_version_r = i_active_config.version else '0';
    o_busy <= '1' when state_r = PROFILE_BUILD else '0';
    o_profile_version <= profile_version_r;
    o_active_clocks <= active_clocks_r;

    p_profile : process (i_clk)
        variable first_ticks_v : echo_delay_ticks_t;
        variable step_ticks_v  : echo_delay_ticks_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r             <= PROFILE_WAIT_CONFIG;
                target_version_r    <= (others => '0');
                profile_version_r   <= (others => '0');
                build_index_r       <= 0;
                tick_accumulator_r  <= (others => '0');
                channel_step_r      <= (others => '0');
                active_clocks_r     <= C_ECHO_DELAY_CLOCKS_CLEAR;
            elsif i_active_valid = '0' then
                state_r <= PROFILE_WAIT_CONFIG;
            elsif state_r = PROFILE_WAIT_CONFIG or
                  i_active_config.version /= target_version_r then
                first_ticks_v := resize(
                    i_active_config.source.echo.channel_0_delay_5ns,
                    C_ECHO_DELAY_WIDTH);
                step_ticks_v := resize(
                    i_active_config.source.echo.channel_step_5ns,
                    C_ECHO_DELAY_WIDTH);
                target_version_r   <= i_active_config.version;
                build_index_r      <= 0;
                tick_accumulator_r <= first_ticks_v;
                channel_step_r     <= step_ticks_v;
                state_r            <= PROFILE_BUILD;
            elsif state_r = PROFILE_BUILD then
                active_clocks_r(build_index_r) <=
                    fn_echo_ticks_to_proc_clocks(
                        tick_accumulator_r,
                        G_BUILD_CONFIG.proc_clk_mhz);

                if build_index_r = C_NUM_CHANNELS - 1 then
                    profile_version_r <= target_version_r;
                    state_r           <= PROFILE_READY;
                else
                    build_index_r <= build_index_r + 1;
                    tick_accumulator_r <=
                        tick_accumulator_r + channel_step_r;
                end if;
            end if;
        end if;
    end process p_profile;

end architecture rtl;
