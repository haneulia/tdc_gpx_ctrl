library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.lidar_gpx_pkg.all;

entity tb_lidar_csr_config_subsystem is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_TDC_CLK_MHZ         : positive := 200;
        G_PROC_HALF_PERIOD_PS : positive := 3333;
        G_TDC_HALF_PERIOD_PS  : positive := 2500
    );
end entity tb_lidar_csr_config_subsystem;

architecture sim of tb_lidar_csr_config_subsystem is

    constant C_CSR_HALF_PERIOD  : time := 5 ns;
    constant C_PROC_HALF_PERIOD : time := G_PROC_HALF_PERIOD_PS * 1 ps;
    constant C_TDC_HALF_PERIOD  : time := G_TDC_HALF_PERIOD_PS * 1 ps;
    constant C_MAX_WAIT_CLKS    : positive := 5000;

    function fn_build_profile return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz  := G_TDC_CLK_MHZ;
        return result;
    end function fn_build_profile;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_profile;

    signal csr_clk   : std_logic := '0';
    signal proc_clk  : std_logic := '0';
    signal tdc_clk   : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';
    signal stop_clocks : boolean := false;

    signal awaddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal awvalid : std_logic := '0';
    signal awready : std_logic;
    signal wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb   : std_logic_vector(3 downto 0) := (others => '0');
    signal wvalid  : std_logic := '0';
    signal wready  : std_logic;
    signal bresp   : std_logic_vector(1 downto 0);
    signal bvalid  : std_logic;
    signal bready  : std_logic := '0';
    signal araddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal arvalid : std_logic := '0';
    signal arready : std_logic;
    signal rdata   : std_logic_vector(31 downto 0);
    signal rresp   : std_logic_vector(1 downto 0);
    signal rvalid  : std_logic;
    signal rready  : std_logic := '0';

    signal proc_safe : std_logic := '1';
    signal tdc_safe  : std_logic := '1';
    signal irq       : std_logic;
    signal clear_status : std_logic;
    signal soft_reset : std_logic;
    signal busy       : std_logic;
    signal done       : std_logic;
    signal rejected   : std_logic;
    signal reject_error : lidar_cfg_error_t;
    signal error_value  : lidar_cfg_error_t;
    signal recovery     : std_logic;
    signal active_valid : std_logic;
    signal active_cfg   : lidar_active_config_t;
    signal proc_enable  : std_logic;
    signal proc_active_valid : std_logic;
    signal proc_active  : lidar_active_config_t;
    signal tdc_enable   : std_logic;
    signal tdc_active_valid : std_logic;
    signal tdc_active   : lidar_active_config_t;
    signal prepare_req  : std_logic;
    signal activate_req : std_logic;
    signal release_req  : std_logic;
    signal external_laser_permit : std_logic := '0';
    signal operation_state : operation_state_t;
    signal operation_command_accepted : std_logic;
    signal operation_command_rejected : std_logic;
    signal operation_permit_trip : std_logic;
    signal operation_safe : std_logic;
    signal tdc_config_ready : std_logic := '1';
    signal tdc_config_done  : std_logic := '0';
    signal tdc_config_fault : std_logic := '0';
    signal tdc_register_image : gpx_register_image_t;
    signal tdc_config_apply : std_logic;
    signal tdc_apply_pending : std_logic := '0';
    signal tdc_apply_delay : natural range 0 to 7 := 0;
    signal tdc_apply_count : natural := 0;
    signal last_applied_reg5 : std_logic_vector(31 downto 0) :=
        (others => '0');

begin

    p_csr_clock : process
    begin
        while not stop_clocks loop
            csr_clk <= '0';
            wait for C_CSR_HALF_PERIOD;
            csr_clk <= '1';
            wait for C_CSR_HALF_PERIOD;
        end loop;
        wait;
    end process p_csr_clock;

    p_proc_clock : process
    begin
        while not stop_clocks loop
            proc_clk <= '0';
            wait for C_PROC_HALF_PERIOD;
            proc_clk <= '1';
            wait for C_PROC_HALF_PERIOD;
        end loop;
        wait;
    end process p_proc_clock;

    p_tdc_clock : process
    begin
        while not stop_clocks loop
            tdc_clk <= '0';
            wait for C_TDC_HALF_PERIOD;
            tdc_clk <= '1';
            wait for C_TDC_HALF_PERIOD;
        end loop;
        wait;
    end process p_tdc_clock;

    -- Deterministic stand-in for the H2B-2A coordinator. The coordinator's
    -- own test proves that o_config_done waits for every physically present
    -- Chip; this model proves that the central transaction waits for that
    -- aggregated completion handshake.
    p_tdc_programming_model : process (tdc_clk)
    begin
        if rising_edge(tdc_clk) then
            if tdc_rst_n = '0' then
                tdc_config_done <= '0';
                tdc_apply_pending <= '0';
                tdc_apply_delay <= 0;
                tdc_apply_count <= 0;
                last_applied_reg5 <= (others => '0');
            else
                tdc_config_done <= '0';
                if tdc_config_apply = '1' then
                    assert tdc_apply_pending = '0'
                        report "V2-CSR-INT overlapping GPX image apply"
                        severity failure;
                    assert tdc_enable = '0' and release_req = '0'
                        report "V2-CSR-INT TDC enabled before GPX programming"
                        severity failure;
                    tdc_apply_pending <= '1';
                    tdc_apply_delay <= 7;
                    tdc_apply_count <= tdc_apply_count + 1;
                    last_applied_reg5 <= tdc_register_image(5);
                elsif tdc_apply_pending = '1' then
                    assert tdc_enable = '0'
                        report "V2-CSR-INT TDC enabled while GPX apply pending"
                        severity failure;
                    if tdc_apply_delay = 0 then
                        tdc_config_done <= '1';
                        tdc_apply_pending <= '0';
                    else
                        tdc_apply_delay <= tdc_apply_delay - 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_tdc_programming_model;

    u_dut : entity work.lidar_csr_config_subsystem
        generic map (
            G_BUILD_CONFIG     => C_BUILD_CONFIG,
            G_CSR_CLK_MHZ      => 100,
            G_PHASE_TIMEOUT_US => 1,
            G_TDC_DEFER_ACTIVATE_ACK => true
        )
        port map (
            i_csr_clk   => csr_clk,
            i_csr_rst_n => csr_rst_n,
            i_proc_clk  => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_tdc_clk   => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            s_axi_awaddr  => awaddr,
            s_axi_awprot  => awprot,
            s_axi_awvalid => awvalid,
            s_axi_awready => awready,
            s_axi_wdata   => wdata,
            s_axi_wstrb   => wstrb,
            s_axi_wvalid  => wvalid,
            s_axi_wready  => wready,
            s_axi_bresp   => bresp,
            s_axi_bvalid  => bvalid,
            s_axi_bready  => bready,
            s_axi_araddr  => araddr,
            s_axi_arprot  => arprot,
            s_axi_arvalid => arvalid,
            s_axi_arready => arready,
            s_axi_rdata   => rdata,
            s_axi_rresp   => rresp,
            s_axi_rvalid  => rvalid,
            s_axi_rready  => rready,
            i_proc_safe => proc_safe,
            i_tdc_safe  => tdc_safe,
            i_external_laser_permit => external_laser_permit,
            i_tdc_config_ready => tdc_config_ready,
            i_tdc_config_done  => tdc_config_done,
            i_tdc_config_fault => tdc_config_fault,
            o_irq                => irq,
            o_clear_status       => clear_status,
            o_soft_reset_request => soft_reset,
            o_busy               => busy,
            o_done               => done,
            o_commit_rejected    => rejected,
            o_reject_error       => reject_error,
            o_error              => error_value,
            o_recovery_required  => recovery,
            o_active_valid       => active_valid,
            o_active             => active_cfg,
            o_proc_enable        => proc_enable,
            o_proc_active_valid  => proc_active_valid,
            o_proc_active        => proc_active,
            o_tdc_enable         => tdc_enable,
            o_tdc_active_valid   => tdc_active_valid,
            o_tdc_active         => tdc_active,
            o_tdc_register_image => tdc_register_image,
            o_tdc_config_apply   => tdc_config_apply,
            o_proc_activate_start => open,
            o_prepare_req        => prepare_req,
            o_activate_req       => activate_req,
            o_release_req        => release_req,
            o_operation_state    => operation_state,
            o_operation_command_accepted => operation_command_accepted,
            o_operation_command_rejected => operation_command_rejected,
            o_operation_permit_trip => operation_permit_trip,
            o_operation_safe_to_prepare => operation_safe
        );

    p_request_invariants : process (csr_clk)
    begin
        if rising_edge(csr_clk) and csr_rst_n = '1' then
            assert activate_req = '0' or prepare_req = '1'
                report "V2-CSR-INT ACTIVATE without PREPARE"
                severity failure;
            assert release_req = '0'
                or (prepare_req = '1' and activate_req = '1')
                report "V2-CSR-INT RELEASE without PREPARE+ACTIVATE"
                severity failure;
        end if;
    end process p_request_invariants;

    p_test : process
        variable v_word      : std_logic_vector(31 downto 0);
        variable v_preserved : lidar_active_config_t;
        variable v_expected  : lidar_runtime_config_t;
        variable v_cycles    : natural;

        procedure csr_tick is
        begin
            wait until rising_edge(csr_clk);
            wait for 1 ps;
        end procedure csr_tick;

        procedure axi_write(
            constant address : natural;
            constant value   : std_logic_vector(31 downto 0)
        ) is
            variable aw_sent : boolean := false;
            variable w_sent  : boolean := false;
        begin
            wait until falling_edge(csr_clk);
            awaddr  <= std_logic_vector(to_unsigned(address, awaddr'length));
            awvalid <= '1';
            wdata   <= value;
            wstrb   <= "1111";
            wvalid  <= '1';
            while not (aw_sent and w_sent) loop
                wait until rising_edge(csr_clk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_sent := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wvalid <= '0';
                    w_sent := true;
                end if;
            end loop;
            wstrb <= (others => '0');
            while bvalid = '0' loop
                csr_tick;
            end loop;
            assert bresp = "00"
                report "V2-CSR-INT AXI write response"
                severity failure;
            bready <= '1';
            csr_tick;
            bready <= '0';
        end procedure axi_write;

        procedure axi_read_value(
            constant address : natural;
            variable value   : out std_logic_vector(31 downto 0)
        ) is
        begin
            wait until falling_edge(csr_clk);
            araddr  <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(csr_clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                csr_tick;
            end loop;
            assert rresp = "00"
                report "V2-CSR-INT AXI read response"
                severity failure;
            value := rdata;
            rready <= '1';
            csr_tick;
            rready <= '0';
        end procedure axi_read_value;

        procedure axi_read(
            constant address  : natural;
            constant expected : std_logic_vector(31 downto 0)
        ) is
            variable actual : std_logic_vector(31 downto 0);
        begin
            axi_read_value(address, actual);
            assert actual = expected
                report "V2-CSR-INT read mismatch at " & integer'image(address)
                severity failure;
        end procedure axi_read;

        procedure command(constant bit_index : natural) is
            variable command_word : std_logic_vector(31 downto 0) :=
                (others => '0');
        begin
            command_word(bit_index) := '1';
            axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), command_word);
        end procedure command;

        procedure wait_done(
            constant case_name : string;
            constant expected_error : lidar_cfg_error_t
        ) is
            variable cycles : natural := 0;
        begin
            loop
                csr_tick;
                cycles := cycles + 1;
                exit when done = '1';
                assert cycles < C_MAX_WAIT_CLKS
                    report case_name & ": completion timeout"
                    severity failure;
            end loop;
            assert error_value = expected_error
                report case_name & ": error mismatch"
                severity failure;
        end procedure wait_done;

        procedure check_active(
            constant expected_source  : lidar_runtime_config_t;
            constant expected_version : natural
        ) is
            variable expected_derived : lidar_derived_config_t;
        begin
            expected_derived := fn_derive_runtime_config(
                C_BUILD_CONFIG, expected_source);
            assert active_valid = '1'
                and to_integer(active_cfg.version) = expected_version
                and active_cfg.source = expected_source
                and active_cfg.derived = expected_derived
                report "V2-CSR-INT manager active record mismatch"
                severity failure;
            assert proc_active_valid = '1' and tdc_active_valid = '1'
                and proc_active = active_cfg and tdc_active = active_cfg
                and proc_enable = '1' and tdc_enable = '1'
                report "V2-CSR-INT destination records are not atomic"
                severity failure;
        end procedure check_active;
    begin
        wait for 30 ns;
        csr_rst_n  <= '1';
        proc_rst_n <= '1';
        tdc_rst_n  <= '1';
        csr_tick;

        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000100");
        assert active_valid = '0' and proc_enable = '0' and tdc_enable = '0'
            report "V2-CSR-INT startup was not inhibited"
            severity failure;

        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000005");
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"001ABCDE");
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        command(C_CMD_COMMIT_BIT);
        wait_done("V2-CSR-INT first commit", CFG_OK);
        v_expected := fn_default_runtime_config(C_BUILD_CONFIG);
        v_expected.motor.simulation_mode := '1';
        check_active(v_expected, 1);
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000046");
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), x"01080001");
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_SOURCE_BASE), x"00120E10");
        assert tdc_apply_count = 1 and last_applied_reg5 = x"001ABCDE"
            report "V2-CSR-INT first GPX image apply mismatch"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000105");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"001ABCDE");
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000005");

        command(C_CMD_CLEAR_STATUS_BIT);
        axi_write(fn_ctl_byte_offset(C_CTL_SHOT_INTERVAL), x"000186A0");
        command(C_CMD_COMMIT_BIT);
        while busy /= '1' loop
            csr_tick;
        end loop;

        -- This edit is for the next transaction; the running transaction must
        -- keep its original snapshot and DIRTY must remain asserted afterward.
        axi_write(fn_ctl_byte_offset(C_CTL_SHOT_INTERVAL), x"000249F0");
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00222222");
        command(C_CMD_COMMIT_BIT);
        wait_done("V2-CSR-INT snapshot commit", CFG_OK);
        v_expected.laser.optical_shot_interval_udeg := to_unsigned(100_000, 30);
        check_active(v_expected, 2);
        axi_read(fn_ctl_byte_offset(C_CTL_SHOT_INTERVAL), x"000249F0");
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_SOURCE_BASE +
            C_CTL_SHOT_INTERVAL - 1), x"000186A0");
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"71000156");
        assert tdc_apply_count = 2 and last_applied_reg5 = x"001ABCDE"
            report "V2-CSR-INT in-flight GPX image snapshot changed"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000105");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"001ABCDE");
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000005");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00222222");

        command(C_CMD_CLEAR_STATUS_BIT);
        command(C_CMD_COMMIT_BIT);
        wait_done("V2-CSR-INT next shadow commit", CFG_OK);
        v_expected.laser.optical_shot_interval_udeg := to_unsigned(150_000, 30);
        check_active(v_expected, 3);
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000046");
        assert tdc_apply_count = 3 and last_applied_reg5 = x"00222222"
            report "V2-CSR-INT next GPX image snapshot was not applied"
            severity failure;

        -- Invalid source data completes with an error and cannot alter either
        -- destination's active version.
        command(C_CMD_CLEAR_STATUS_BIT);
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120000");
        v_preserved := active_cfg;
        command(C_CMD_COMMIT_BIT);
        wait_done("V2-CSR-INT invalid commit", CFG_RUNTIME_CPR);
        assert active_cfg = v_preserved and proc_active = v_preserved
            and tdc_active = v_preserved
            report "V2-CSR-INT invalid commit changed active data"
            severity failure;
        assert tdc_apply_count = 3
            report "V2-CSR-INT invalid commit programmed GPX image"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"0020014A");

        -- Restoring the source while PROC is unsafe holds PREPARE without
        -- exposing a partial active configuration.
        command(C_CMD_CLEAR_STATUS_BIT);
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        proc_safe <= '0';
        command(C_CMD_COMMIT_BIT);
        v_cycles := 0;
        while prepare_req /= '1' loop
            csr_tick;
            v_cycles := v_cycles + 1;
            assert v_cycles < C_MAX_WAIT_CLKS
                report "V2-CSR-INT PREPARE was not issued"
                severity failure;
        end loop;
        for index in 1 to 10 loop
            csr_tick;
        end loop;
        assert busy = '1' and active_cfg = v_preserved
            report "V2-CSR-INT unsafe domain activated early"
            severity failure;
        proc_safe <= '1';
        wait_done("V2-CSR-INT safe-point commit", CFG_OK);
        check_active(v_expected, 4);
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000046");
        assert tdc_apply_count = 4 and last_applied_reg5 = x"00222222"
            report "V2-CSR-INT safe-point GPX apply mismatch"
            severity failure;

        axi_read_value(fn_stat_byte_offset(C_STAT_CAPTURE_TDC_CLKS), v_word);
        assert unsigned(v_word) = active_cfg.derived.capture_window_tdc_clks
            report "V2-CSR-INT TDC-clock-derived readback mismatch"
            severity failure;
        assert recovery = '0'
            report "V2-CSR-INT unexpected recovery lock"
            severity failure;

        report "LIDAR_V2_CSR_CONFIG_SUBSYSTEM_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        stop_clocks <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
