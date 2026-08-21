library ieee;
use ieee.std_logic_1164.all;

--------------------------------------------------------------------------------
-- Optional four-register interrupt block.
--
-- Register offsets from c_INTR_BASE = num_ctl_regs + num_stat_regs:
--   +0 INTR_EN     R/W  Interrupt enable
--   +1 INTR_STATUS RO   Synchronized source level
--   +2 INTR_FLAG   W1C  Pending manual interrupt flags
--   +3 INTR_MODE   R/W  0=manual level, 1=automatic one-clock pulse
--
-- intrpt_src_in may be asynchronous. Each source passes through a two-flop
-- synchronizer before rising-edge detection. The detector compares only the
-- synchronized second stage with a separate history register; functional logic
-- never consumes the metastability-facing first stage. A source pulse must
-- remain high long enough to be sampled; narrow events require an upstream CDC
-- event bridge.
--
-- In manual mode a newly detected edge wins over a simultaneous W1C, ensuring
-- that software cannot silently clear an event arriving on the same clock.
--------------------------------------------------------------------------------
entity axil_intr_32 is
    generic (
        num_ctl_regs      : integer := 32;
        num_stat_regs     : integer := 32;
        num_intr_regs     : integer := 4;
        num_interrupt_src : integer := 32;
        num_data_bits     : integer := 32
    );
    port (
        aclk          : in  std_logic;
        aresetn       : in  std_logic;

        w_addr_num    : in  integer;
        w_data        : in  std_logic_vector(num_data_bits-1 downto 0);
        w_strb        : in  std_logic_vector((num_data_bits/8)-1 downto 0);
        w_we          : in  std_logic;

        intrpt_src_in : in  std_logic_vector(num_interrupt_src-1 downto 0);

        r_addr_num    : in  integer;
        rd_data       : out std_logic_vector(num_data_bits-1 downto 0);
        rd_hit        : out std_logic;
        irq           : out std_logic
    );
end entity axil_intr_32;

architecture Behavioral of axil_intr_32 is
    constant c_INTR_BASE       : integer := num_ctl_regs + num_stat_regs;
    constant c_INTR_EN_OFF     : integer := 0;
    constant c_INTR_STATUS_OFF : integer := 1;
    constant c_INTR_FLAG_OFF   : integer := 2;
    constant c_INTR_MODE_OFF   : integer := 3;

    signal s_enable_reg : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_mode_reg   : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_flag_reg   : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');

    signal s_sync_ff1   : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_sync_ff2   : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_sync_ff2_d : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_rise       : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');

    signal s_w_en_sel   : std_logic;
    signal s_w_flag_sel : std_logic;
    signal s_w_mode_sel : std_logic;
    signal s_sw_clear   : std_logic_vector(num_data_bits-1 downto 0);

    signal s_irq_level : std_logic_vector(num_data_bits-1 downto 0);
    signal s_irq_pulse : std_logic_vector(num_data_bits-1 downto 0);
    signal s_irq_next  : std_logic;

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of s_sync_ff1 : signal is "TRUE";
    attribute ASYNC_REG of s_sync_ff2 : signal is "TRUE";

    attribute SHREG_EXTRACT : string;
    attribute SHREG_EXTRACT of s_sync_ff1 : signal is "NO";
    attribute SHREG_EXTRACT of s_sync_ff2 : signal is "NO";
    attribute SHREG_EXTRACT of s_sync_ff2_d : signal is "NO";
begin
    assert num_data_bits = 32
        report "axil_intr_32 supports num_data_bits=32 only"
        severity failure;
    assert num_intr_regs = 4
        report "axil_intr_32 requires num_intr_regs=4"
        severity failure;
    assert num_interrupt_src >= 1 and num_interrupt_src <= num_data_bits
        report "axil_intr_32 num_interrupt_src must be in range 1..32"
        severity failure;

    s_w_en_sel <= '1' when
        w_we = '1' and w_addr_num = c_INTR_BASE + c_INTR_EN_OFF else '0';
    s_w_flag_sel <= '1' when
        w_we = '1' and w_addr_num = c_INTR_BASE + c_INTR_FLAG_OFF else '0';
    s_w_mode_sel <= '1' when
        w_we = '1' and w_addr_num = c_INTR_BASE + c_INTR_MODE_OFF else '0';

    p_enable : process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s_enable_reg <= (others => '0');
            elsif s_w_en_sel = '1' then
                for byte_index in 0 to (num_data_bits/8)-1 loop
                    if w_strb(byte_index) = '1' then
                        s_enable_reg(8*byte_index+7 downto 8*byte_index) <=
                            w_data(8*byte_index+7 downto 8*byte_index);
                    end if;
                end loop;
            end if;
        end if;
    end process p_enable;

    p_mode : process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s_mode_reg <= (others => '0');
            elsif s_w_mode_sel = '1' then
                for byte_index in 0 to (num_data_bits/8)-1 loop
                    if w_strb(byte_index) = '1' then
                        s_mode_reg(8*byte_index+7 downto 8*byte_index) <=
                            w_data(8*byte_index+7 downto 8*byte_index);
                    end if;
                end loop;
            end if;
        end if;
    end process p_mode;

    p_source_sync : process(aclk)
        variable v_async_source : std_logic_vector(num_data_bits-1 downto 0);
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s_sync_ff1   <= (others => '0');
                s_sync_ff2   <= (others => '0');
                s_sync_ff2_d <= (others => '0');
            else
                v_async_source := (others => '0');
                v_async_source(num_interrupt_src-1 downto 0) := intrpt_src_in;
                s_sync_ff1   <= v_async_source;
                s_sync_ff2   <= s_sync_ff1;
                s_sync_ff2_d <= s_sync_ff2;
            end if;
        end if;
    end process p_source_sync;

    -- One gate after fully synchronized registers preserves the existing event
    -- latency while removing all functional fanout from the first CDC stage.
    s_rise <= s_sync_ff2 and not s_sync_ff2_d;

    gen_sw_clear : for i in 0 to num_data_bits-1 generate
        s_sw_clear(i) <= w_data(i) when
            s_w_flag_sel = '1' and w_strb(i/8) = '1' else '0';
    end generate gen_sw_clear;

    gen_active_flags : for i in 0 to num_interrupt_src-1 generate
        p_flag : process(aclk)
        begin
            if rising_edge(aclk) then
                if aresetn = '0' then
                    s_flag_reg(i) <= '0';
                elsif s_rise(i) = '1' then
                    if s_mode_reg(i) = '1' then
                        s_flag_reg(i) <= '0';
                    else
                        s_flag_reg(i) <= '1';
                    end if;
                elsif s_sw_clear(i) = '1' then
                    s_flag_reg(i) <= '0';
                end if;
            end if;
        end process p_flag;
    end generate gen_active_flags;

    gen_inactive_flags : if num_interrupt_src < num_data_bits generate
        gen_flag : for i in num_interrupt_src to num_data_bits-1 generate
            s_flag_reg(i) <= '0';
        end generate gen_flag;
    end generate gen_inactive_flags;

    s_irq_level <= s_flag_reg and s_enable_reg and not s_mode_reg;
    s_irq_pulse <= s_rise and s_enable_reg and s_mode_reg;
    s_irq_next <= '1' when
        (s_irq_level or s_irq_pulse) /= (s_irq_level'range => '0') else '0';

    p_irq : process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                irq <= '0';
            else
                irq <= s_irq_next;
            end if;
        end if;
    end process p_irq;

    p_read : process(
        r_addr_num,
        s_enable_reg,
        s_sync_ff2,
        s_flag_reg,
        s_mode_reg
    )
    begin
        rd_data <= (others => '0');
        rd_hit  <= '0';
        if r_addr_num >= c_INTR_BASE and
           r_addr_num < c_INTR_BASE + num_intr_regs then
            case r_addr_num - c_INTR_BASE is
                when c_INTR_EN_OFF =>
                    rd_data <= s_enable_reg;
                when c_INTR_STATUS_OFF =>
                    rd_data <= s_sync_ff2;
                when c_INTR_FLAG_OFF =>
                    rd_data <= s_flag_reg;
                when c_INTR_MODE_OFF =>
                    rd_data <= s_mode_reg;
                when others =>
                    null;
            end case;
            rd_hit <= '1';
        end if;
    end process p_read;
end architecture Behavioral;
