library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity lidar_u64_u32_divider_seq is
    port (
        i_clk         : in  std_logic;
        i_rst_n       : in  std_logic;
        i_start       : in  std_logic;
        i_numerator   : in  unsigned(63 downto 0);
        i_denominator : in  unsigned(31 downto 0);
        o_busy        : out std_logic;
        o_done        : out std_logic;
        o_div_zero    : out std_logic;
        o_quotient    : out unsigned(63 downto 0);
        o_remainder   : out unsigned(31 downto 0)
    );
end entity lidar_u64_u32_divider_seq;

architecture rtl of lidar_u64_u32_divider_seq is

    signal r_busy        : std_logic := '0';
    signal r_done        : std_logic := '0';
    signal r_div_zero    : std_logic := '0';
    signal r_dividend    : unsigned(63 downto 0);
    signal r_divisor     : unsigned(31 downto 0);
    signal r_remainder   : unsigned(32 downto 0);
    signal r_quotient    : unsigned(63 downto 0);
    signal r_result_q    : unsigned(63 downto 0);
    signal r_result_r    : unsigned(31 downto 0);
    signal r_bit_index   : natural range 0 to 63 := 0;

begin

    o_busy      <= r_busy;
    o_done      <= r_done;
    o_div_zero  <= r_div_zero;
    o_quotient  <= r_result_q;
    o_remainder <= r_result_r;

    p_divide : process (i_clk)
        variable v_remainder : unsigned(32 downto 0);
        variable v_quotient  : unsigned(63 downto 0);
    begin
        if rising_edge(i_clk) then
            r_done     <= '0';
            r_div_zero <= '0';

            if i_rst_n = '0' then
                r_busy      <= '0';
                r_done      <= '0';
                r_div_zero  <= '0';
                r_bit_index <= 0;
            elsif r_busy = '0' then
                if i_start = '1' then
                    if i_denominator = 0 then
                        r_result_q <= (others => '0');
                        r_result_r <= i_numerator(31 downto 0);
                        r_div_zero <= '1';
                        r_done     <= '1';
                    else
                        r_dividend  <= i_numerator;
                        r_divisor   <= i_denominator;
                        r_remainder <= (others => '0');
                        r_quotient  <= (others => '0');
                        r_bit_index <= 0;
                        r_busy      <= '1';
                    end if;
                end if;
            else
                v_remainder := shift_left(r_remainder, 1);
                v_remainder(0) := r_dividend(63);
                v_quotient := shift_left(r_quotient, 1);

                if v_remainder >= ('0' & r_divisor) then
                    v_remainder := v_remainder - ('0' & r_divisor);
                    v_quotient(0) := '1';
                end if;

                r_dividend <= shift_left(r_dividend, 1);
                if r_bit_index = 63 then
                    r_result_q  <= v_quotient;
                    r_result_r  <= v_remainder(31 downto 0);
                    r_busy      <= '0';
                    r_done      <= '1';
                else
                    r_remainder <= v_remainder;
                    r_quotient  <= v_quotient;
                    r_bit_index <= r_bit_index + 1;
                end if;
            end if;
        end if;
    end process p_divide;

end architecture rtl;
