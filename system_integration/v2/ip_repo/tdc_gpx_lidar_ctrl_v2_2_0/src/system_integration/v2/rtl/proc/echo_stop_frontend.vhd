library ieee;
use ieee.std_logic_1164.all;

library unisim;
use unisim.vcomponents.all;

-- Physical LVDS input owner and observation-only synchronizer.
--
-- The production generate is intentionally a direct IBUFDS-to-STOP path.
-- Only a simulation-enabled build contains the source mux and mode register.
entity echo_stop_frontend is
    generic (
        G_NUM_CHANNELS      : positive range 1 to 32 := 32;
        G_ENABLE_SIMULATION : boolean := false
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_mode_latch        : in  std_logic;
        i_select_simulation : in  std_logic;
        i_sim_stop          : in  std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        i_pd_lvds_p         : in  std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        i_pd_lvds_n         : in  std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        o_tdc_stop          : out std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        o_diag_rise_event   : out std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        o_diag_fall_event   : out std_logic_vector(
            G_NUM_CHANNELS - 1 downto 0);
        o_simulation_active : out std_logic
    );
end entity echo_stop_frontend;

architecture rtl of echo_stop_frontend is

    signal physical_stop_c : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0);
    signal physical_meta_r : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0) := (others => '0');
    signal physical_sync_r : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0) := (others => '0');
    signal physical_sync_d_r : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0) := (others => '0');
    signal physical_rise_c : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0);
    signal physical_fall_c : std_logic_vector(
        G_NUM_CHANNELS - 1 downto 0);

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of physical_meta_r : signal is "TRUE";
    attribute ASYNC_REG of physical_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of physical_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of physical_sync_r : signal is "NO";

begin

    gen_ibufds : for channel in 0 to G_NUM_CHANNELS - 1 generate
        u_ibufds : IBUFDS
            generic map (
                DIFF_TERM    => TRUE,
                IBUF_LOW_PWR => FALSE,
                IOSTANDARD   => "LVDS_25"
            )
            port map (
                I  => i_pd_lvds_p(channel),
                IB => i_pd_lvds_n(channel),
                O  => physical_stop_c(channel)
            );
    end generate gen_ibufds;

    p_observer : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                physical_meta_r   <= (others => '0');
                physical_sync_r   <= (others => '0');
                physical_sync_d_r <= (others => '0');
            else
                physical_meta_r   <= physical_stop_c;
                physical_sync_r   <= physical_meta_r;
                physical_sync_d_r <= physical_sync_r;
            end if;
        end if;
    end process p_observer;

    physical_rise_c <= physical_sync_r and not physical_sync_d_r;
    physical_fall_c <= not physical_sync_r and physical_sync_d_r;

    gen_production : if not G_ENABLE_SIMULATION generate
        -- This short combinational I/O path is the deliberate Echo exception
        -- to the registered-logic preference. It preserves the physical edge.
        o_tdc_stop          <= physical_stop_c;
        o_diag_rise_event   <= physical_rise_c;
        o_diag_fall_event   <= physical_fall_c;
        o_simulation_active <= '0';

        -- synthesis translate_off
        p_no_simulation : process (i_clk)
        begin
            if rising_edge(i_clk) then
                if i_rst_n = '1' and i_mode_latch = '1' then
                    assert i_select_simulation = '0'
                        report "V2-ECHO-006 simulation selected in production"
                        severity failure;
                end if;
            end if;
        end process p_no_simulation;
        -- synthesis translate_on
    end generate gen_production;

    gen_simulation : if G_ENABLE_SIMULATION generate
        signal simulation_active_r : std_logic := '0';
    begin
        p_mode : process (i_clk)
        begin
            if rising_edge(i_clk) then
                if i_rst_n = '0' then
                    simulation_active_r <= '0';
                elsif i_mode_latch = '1' then
                    simulation_active_r <= i_select_simulation;

                    -- synthesis translate_off
                    if i_select_simulation /= simulation_active_r then
                        assert physical_stop_c =
                            (physical_stop_c'range => '0')
                            report "V2-ECHO-007 mode change while LVDS active"
                            severity warning;
                    end if;
                    -- synthesis translate_on
                end if;
            end if;
        end process p_mode;

        o_tdc_stop <= i_sim_stop when simulation_active_r = '1'
            else physical_stop_c;
        o_diag_rise_event <= i_sim_stop when simulation_active_r = '1'
            else physical_rise_c;
        o_diag_fall_event <= (others => '0')
            when simulation_active_r = '1' else physical_fall_c;
        o_simulation_active <= simulation_active_r;
    end generate gen_simulation;

end architecture rtl;
