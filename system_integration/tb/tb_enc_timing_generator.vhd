library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.enc_pkg.all;

entity tb_enc_timing_generator is
end entity;

architecture sim of tb_enc_timing_generator is
    constant C_CLK_PERIOD : time := 10 ns;

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal param_en     : std_logic := '0';
    signal ticks_lo     : std_logic_vector(c_TICK_CNT_W-1 downto 0) :=
                          std_logic_vector(to_unsigned(5, c_TICK_CNT_W));
    signal ticks_hi     : std_logic_vector(c_TICK_CNT_W-1 downto 0) :=
                          std_logic_vector(to_unsigned(6, c_TICK_CNT_W));
    signal hi_count     : std_logic_vector(c_POS_W-1 downto 0) :=
                          std_logic_vector(to_unsigned(3, c_POS_W));
    signal total_states : std_logic_vector(c_POS_W-1 downto 0) :=
                          std_logic_vector(to_unsigned(8, c_POS_W));
    signal phase_tick   : std_logic;
    signal ticks_next   : std_logic_vector(c_TICK_CNT_W-1 downto 0);
begin
    clk <= not clk after C_CLK_PERIOD / 2;

    dut : entity work.enc_timing_generator
        generic map (
            g_TICKS_LO     => 5,
            g_TICKS_HI     => 6,
            g_HI_COUNT     => 3,
            g_TOTAL_STATES => 8
        )
        port map (
            i_clk          => clk,
            i_rst_n        => rst_n,
            i_param_en     => param_en,
            i_ticks_lo     => ticks_lo,
            i_ticks_hi     => ticks_hi,
            i_hi_count     => hi_count,
            i_total_states => total_states,
            o_phase_tick   => phase_tick,
            o_ticks_next   => ticks_next
        );

    stim : process
        procedure check_epoch(
            constant lo_value       : positive;
            constant hi_value       : positive;
            constant hi_count_value : natural;
            constant total_value    : positive;
            constant interval_count : positive;
            constant reload         : boolean
        ) is
            variable cycles       : natural;
            variable expected     : positive;
            variable accumulator  : natural;
            variable threshold    : positive;
            variable high_seen    : natural;
        begin
            if reload then
                wait until falling_edge(clk);
                ticks_lo     <= std_logic_vector(to_unsigned(lo_value, ticks_lo'length));
                ticks_hi     <= std_logic_vector(to_unsigned(hi_value, ticks_hi'length));
                hi_count     <= std_logic_vector(to_unsigned(hi_count_value, hi_count'length));
                total_states <= std_logic_vector(to_unsigned(total_value, total_states'length));
                param_en     <= '1';
                wait until rising_edge(clk);
                wait for 1 ps;
                param_en <= '0';
                assert phase_tick = '0'
                    report "parameter apply did not cancel the old interval"
                    severity failure;
            end if;

            expected    := lo_value;
            accumulator := hi_count_value;
            threshold   := total_value - hi_count_value;
            high_seen   := 0;

            for interval_index in 1 to interval_count loop
                cycles := 0;
                loop
                    wait until rising_edge(clk);
                    wait for 1 ps;
                    cycles := cycles + 1;
                    exit when phase_tick = '1';
                    assert cycles <= hi_value
                        report "phase tick timeout" severity failure;
                end loop;

                assert cycles = expected
                    report "wrong phase interval at index " &
                           integer'image(interval_index) & ": got " &
                           integer'image(cycles) & ", expected " &
                           integer'image(expected)
                    severity failure;

                if expected = hi_value then
                    high_seen := high_seen + 1;
                end if;

                if accumulator >= threshold then
                    expected    := hi_value;
                    accumulator := accumulator - threshold;
                else
                    expected    := lo_value;
                    accumulator := accumulator + hi_count_value;
                end if;

            end loop;

            if (interval_count mod total_value) = 0 then
                assert high_seen = (interval_count / total_value) * hi_count_value
                    report "Bresenham high-interval count is not exact"
                    severity failure;
            end if;
        end procedure;
    begin
        for i in 1 to 4 loop
            wait until rising_edge(clk);
        end loop;
        wait until falling_edge(clk);
        rst_n <= '1';

        -- Baseline distribution, two exact revolutions.
        check_epoch(5, 6, 3, 8, 16, false);

        -- No fractional remainder: every interval is TICKS_LO.
        check_epoch(3, 4, 0, 7, 14, true);

        -- Shortest legal interval. This case was unsafe in the old three-stage
        -- feedback scheduler because phase events can occur every clock.
        check_epoch(1, 2, 2, 5, 15, true);

        report "ENC_TIMING_GENERATOR_PASS" severity note;
        std.env.stop;
        wait;
    end process;
end architecture;
