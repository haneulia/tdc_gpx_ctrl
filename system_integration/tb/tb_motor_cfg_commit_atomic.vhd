library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.enc_pkg.all;

entity tb_motor_cfg_commit_atomic is
end entity;

architecture sim of tb_motor_cfg_commit_atomic is
    constant C_CLK_PERIOD : time := 10 ns;

    signal clk               : std_logic := '0';
    signal rst_n             : std_logic := '0';
    signal cfg_apply         : std_logic := '0';
    signal dec_mode          : std_logic_vector(1 downto 0) := "00";
    signal enc_cpr           : std_logic_vector(15 downto 0) := (others => '0');
    signal enc_dir           : std_logic := '0';
    signal enc_ticks_lo      : std_logic_vector(31 downto 0) := (others => '0');
    signal enc_hi_count      : std_logic_vector(15 downto 0) := (others => '0');
    signal z_offset          : std_logic_vector(c_POS_W-1 downto 0) := (others => '0');
    signal z_early           : std_logic := '0';
    signal z_width           : std_logic_vector(c_POS_W-1 downto 0) := (others => '0');
    signal face_center       : std_logic_vector(c_POS_W-1 downto 0) := (others => '0');
    signal face_half         : std_logic_vector(c_POS_W-1 downto 0) := (others => '0');
    signal face_update_en    : std_logic_vector(4 downto 0) := (others => '0');
    signal enc_param_applied : std_logic := '0';

    signal cfg_busy          : std_logic;
    signal active_dec_mode   : std_logic_vector(1 downto 0);
    signal active_enc_cpr    : std_logic_vector(15 downto 0);
    signal enc_param_commit  : std_logic;
    signal enc_cfg           : t_enc_runtime_cfg;
    signal mirror_update_en  : std_logic_vector(4 downto 0);
    signal mirror_center     : std_logic_vector(c_POS_W-1 downto 0);
    signal mirror_half       : std_logic_vector(c_POS_W-1 downto 0);
    signal total_states_dec  : std_logic_vector(c_POS_W-1 downto 0);
    signal pos_overflow      : std_logic;
begin
    clk <= not clk after C_CLK_PERIOD / 2;

    dut : entity work.motor_cfg_commit_ctrl
        generic map (
            g_CPR      => 18,
            g_TICKS_LO => 138,
            g_TICKS_HI => 139,
            g_HI_COUNT => 64
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_cfg_apply         => cfg_apply,
            i_dec_mode          => dec_mode,
            i_enc_cpr           => enc_cpr,
            i_enc_dir           => enc_dir,
            i_enc_ticks_lo      => enc_ticks_lo,
            i_enc_hi_count      => enc_hi_count,
            i_z_offset          => z_offset,
            i_z_early           => z_early,
            i_z_width           => z_width,
            i_face_center       => face_center,
            i_face_half         => face_half,
            i_face_update_en    => face_update_en,
            i_enc_param_applied => enc_param_applied,
            o_cfg_busy          => cfg_busy,
            o_active_dec_mode   => active_dec_mode,
            o_active_enc_cpr    => active_enc_cpr,
            o_enc_param_commit  => enc_param_commit,
            o_enc_cfg           => enc_cfg,
            o_mirror_update_en  => mirror_update_en,
            o_mirror_center     => mirror_center,
            o_mirror_half       => mirror_half,
            o_total_states_dec  => total_states_dec,
            o_pos_overflow      => pos_overflow
        );

    stim : process
    begin
        wait for 5 * C_CLK_PERIOD;
        rst_n <= '1';
        wait for 5 * C_CLK_PERIOD;

        assert enc_param_commit = '0' and cfg_busy = '0'
            report "CSR values were committed without CFG_APPLY"
            severity failure;
        assert active_dec_mode = "10" and unsigned(active_enc_cpr) = 18
            report "generic boot geometry was not retained"
            severity failure;
        assert unsigned(enc_cfg.ticks_lo) = 138 and
               unsigned(enc_cfg.ticks_hi) = 139 and
               unsigned(enc_cfg.hi_count) = 64 and
               unsigned(enc_cfg.total_states) = 72
            report "generic encoder boot epoch was overwritten by CSR reset values"
            severity failure;

        -- Runtime request: all values belong to the CPR=20 configuration.
        dec_mode    <= "10";
        enc_cpr      <= std_logic_vector(to_unsigned(20, enc_cpr'length));
        enc_dir      <= '1';
        enc_ticks_lo <= std_logic_vector(to_unsigned(37, enc_ticks_lo'length));
        enc_hi_count <= std_logic_vector(to_unsigned(7, enc_hi_count'length));
        z_offset     <= std_logic_vector(to_unsigned(5, z_offset'length));
        z_early      <= '1';
        z_width      <= std_logic_vector(to_unsigned(11, z_width'length));
        cfg_apply    <= '1';
        wait until rising_edge(clk);
        cfg_apply    <= '0';
        wait for 1 ns;

        assert enc_param_commit = '1'
            report "commit pulse missing" severity failure;
        assert unsigned(enc_cfg.total_states) = 80
            report "mixed commit: new ticks are paired with old CPR total_states"
            severity failure;
        assert unsigned(enc_cfg.ticks_lo) = 37 and unsigned(enc_cfg.ticks_hi) = 38
            report "ticks_hi does not belong to the committed ticks_lo"
            severity failure;
        assert enc_cfg.dir = '1' and unsigned(enc_cfg.hi_count) = 7 and
               unsigned(enc_cfg.z_offset) = 5 and enc_cfg.z_early = '1' and
               unsigned(enc_cfg.z_width) = 11
            report "encoder commit bundle contains values from different epochs"
            severity failure;

        -- CSR writes while busy must not mutate the in-flight commit bundle.
        enc_cpr      <= std_logic_vector(to_unsigned(30, enc_cpr'length));
        enc_dir      <= '0';
        enc_ticks_lo <= std_logic_vector(to_unsigned(99, enc_ticks_lo'length));
        enc_hi_count <= std_logic_vector(to_unsigned(13, enc_hi_count'length));
        z_offset     <= std_logic_vector(to_unsigned(17, z_offset'length));
        z_early      <= '0';
        z_width      <= std_logic_vector(to_unsigned(19, z_width'length));
        wait until rising_edge(clk);
        wait for 1 ns;

        assert unsigned(enc_cfg.total_states) = 80
            report "in-flight total_states changed after CSR write" severity failure;
        assert unsigned(enc_cfg.ticks_lo) = 37 and unsigned(enc_cfg.ticks_hi) = 38
            report "in-flight ticks changed after CSR write" severity failure;
        assert enc_cfg.dir = '1' and unsigned(enc_cfg.hi_count) = 7 and
               unsigned(enc_cfg.z_offset) = 5 and enc_cfg.z_early = '1' and
               unsigned(enc_cfg.z_width) = 11
            report "in-flight encoder bundle changed after CSR write"
            severity failure;

        enc_param_applied <= '1';
        wait until rising_edge(clk);
        enc_param_applied <= '0';
        wait for 1 ns;

        assert unsigned(active_enc_cpr) = 20
            report "active CPR does not match acknowledged commit" severity failure;

        report "MOTOR_CFG_ATOMIC_PASS" severity note;
        std.env.stop;
        wait;
    end process;
end architecture;
