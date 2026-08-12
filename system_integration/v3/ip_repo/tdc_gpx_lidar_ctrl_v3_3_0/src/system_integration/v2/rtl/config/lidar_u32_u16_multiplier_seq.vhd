library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity lidar_u32_u16_multiplier_seq is
    port (
        i_clk     : in  std_logic;
        i_rst_n   : in  std_logic;
        i_start   : in  std_logic;
        i_left    : in  unsigned(31 downto 0);
        i_right   : in  unsigned(15 downto 0);
        o_busy    : out std_logic;
        o_done    : out std_logic;
        o_product : out unsigned(63 downto 0)
    );
end entity lidar_u32_u16_multiplier_seq;

architecture rtl of lidar_u32_u16_multiplier_seq is

    signal r_busy         : std_logic := '0';
    signal r_done         : std_logic := '0';
    signal r_accumulator  : unsigned(47 downto 0);
    signal r_multiplicand : unsigned(47 downto 0);
    signal r_multiplier   : unsigned(15 downto 0);
    signal r_product      : unsigned(63 downto 0);
    signal r_bit_index    : natural range 0 to 15 := 0;

begin

    o_busy    <= r_busy;
    o_done    <= r_done;
    o_product <= r_product;

    p_multiply : process (i_clk)
        variable v_accumulator : unsigned(47 downto 0);
    begin
        if rising_edge(i_clk) then
            r_done <= '0';

            if i_rst_n = '0' then
                r_busy      <= '0';
                r_done      <= '0';
                r_bit_index <= 0;
            elsif r_busy = '0' then
                if i_start = '1' then
                    r_accumulator  <= (others => '0');
                    r_multiplicand <= resize(i_left, 48);
                    r_multiplier   <= i_right;
                    r_bit_index    <= 0;
                    r_busy         <= '1';
                end if;
            else
                v_accumulator := r_accumulator;
                if r_multiplier(0) = '1' then
                    v_accumulator := v_accumulator + r_multiplicand;
                end if;

                if r_bit_index = 15 then
                    r_product <= resize(v_accumulator, 64);
                    r_busy    <= '0';
                    r_done    <= '1';
                else
                    r_accumulator  <= v_accumulator;
                    r_multiplicand <= shift_left(r_multiplicand, 1);
                    r_multiplier   <= shift_right(r_multiplier, 1);
                    r_bit_index    <= r_bit_index + 1;
                end if;
            end if;
        end if;
    end process p_multiply;

end architecture rtl;
