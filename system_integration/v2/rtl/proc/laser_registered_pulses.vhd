library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Registered pulse-width bank for B3. Trigger conditions are decided by the
-- executor; this entity owns only exact clock-width pulse generation. The
-- physical fire output has an immediate fail-safe permission gate, while an
-- already-started TDC START/STOP pulse is never truncated by operation state.
entity laser_registered_pulses is
    port (
        i_clk                  : in  std_logic;
        i_rst_n                : in  std_logic;
        i_physical_fire_enable : in  std_logic;
        i_fire_trigger         : in  std_logic;
        i_sim_start_trigger    : in  std_logic;
        i_stop_trigger         : in  std_logic;
        i_fire_width_clks      : in  unsigned(31 downto 0);
        i_start_width_clks     : in  unsigned(31 downto 0);
        i_stop_width_clks      : in  unsigned(31 downto 0);

        o_fire_pulse           : out std_logic;
        o_sim_start_pulse      : out std_logic;
        o_stop_pulse           : out std_logic;
        o_fire_busy            : out std_logic;
        o_sim_start_busy       : out std_logic;
        o_stop_busy            : out std_logic
    );
end entity laser_registered_pulses;

architecture rtl of laser_registered_pulses is

    signal fire_r        : std_logic := '0';
    signal fire_count_r  : unsigned(31 downto 0) := (others => '0');
    signal start_r       : std_logic := '0';
    signal start_count_r : unsigned(31 downto 0) := (others => '0');
    signal stop_r        : std_logic := '0';
    signal stop_count_r  : unsigned(31 downto 0) := (others => '0');

begin

    o_fire_pulse      <= fire_r and i_physical_fire_enable;
    o_sim_start_pulse <= start_r;
    o_stop_pulse      <= stop_r;
    o_fire_busy       <= fire_r;
    o_sim_start_busy  <= start_r;
    o_stop_busy       <= stop_r;

    p_fire_pulse : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                fire_r        <= '0';
                fire_count_r  <= (others => '0');
            else
                if fire_r = '1' then
                    if fire_count_r = 0 then
                        fire_r <= '0';
                    else
                        fire_count_r <= fire_count_r - 1;
                    end if;
                elsif i_fire_trigger = '1' then
                    fire_r       <= '1';
                    fire_count_r <= i_fire_width_clks - 1;
                end if;
            end if;
        end if;
    end process p_fire_pulse;

    p_sim_start_pulse : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                start_r       <= '0';
                start_count_r <= (others => '0');
            else
                if start_r = '1' then
                    if start_count_r = 0 then
                        start_r <= '0';
                    else
                        start_count_r <= start_count_r - 1;
                    end if;
                elsif i_sim_start_trigger = '1' then
                    start_r       <= '1';
                    start_count_r <= i_start_width_clks - 1;
                end if;
            end if;
        end if;
    end process p_sim_start_pulse;

    p_stop_pulse : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                stop_r       <= '0';
                stop_count_r <= (others => '0');
            else
                if stop_r = '1' then
                    if stop_count_r = 0 then
                        stop_r <= '0';
                    else
                        stop_count_r <= stop_count_r - 1;
                    end if;
                elsif i_stop_trigger = '1' then
                    stop_r       <= '1';
                    stop_count_r <= i_stop_width_clks - 1;
                end if;
            end if;
        end if;
    end process p_stop_pulse;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                if i_fire_trigger = '1' then
                    assert fire_r = '0' and i_fire_width_clks /= 0
                        report "V2-LASER-PULSE-001 invalid fire trigger"
                        severity failure;
                end if;
                if i_sim_start_trigger = '1' then
                    assert start_r = '0' and i_start_width_clks /= 0
                        report "V2-LASER-PULSE-002 invalid START trigger"
                        severity failure;
                end if;
                if i_stop_trigger = '1' then
                    assert stop_r = '0' and i_stop_width_clks /= 0
                        report "V2-LASER-PULSE-003 invalid STOP trigger"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
