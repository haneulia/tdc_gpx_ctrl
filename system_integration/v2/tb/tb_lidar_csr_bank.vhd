library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_status_pkg.all;

entity tb_lidar_csr_bank is
end entity tb_lidar_csr_bank;

architecture sim of tb_lidar_csr_bank is

    constant C_CLK_PERIOD : time := 10 ns;

    function fn_active_config(
        source_value : lidar_runtime_config_t;
        version_value : natural
    ) return lidar_active_config_t is
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(version_value, 16);
        result.source  := source_value;
        result.derived := fn_derive_runtime_config(
            C_DEFAULT_BUILD_CONFIG, source_value);
        return result;
    end function fn_active_config;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal stop_clock : boolean := false;

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

    signal cfg_busy     : std_logic := '0';
    signal cfg_done     : std_logic := '0';
    signal cfg_rejected : std_logic := '0';
    signal cfg_reject_error : lidar_cfg_error_t := CFG_TRANSACTION_BUSY;
    signal cfg_error    : lidar_cfg_error_t := CFG_OK;
    signal cfg_recovery : std_logic := '0';
    signal active_valid : std_logic := '0';
    signal active_cfg   : lidar_active_config_t := fn_active_config(
        C_DEFAULT_RUNTIME_CONFIG, 1);
    signal active_gpx_image : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;
    signal operation_status : operation_state_t := C_OPERATION_STATE_SAFE;
    signal operation_command_ready : std_logic := '1';
    signal operation_command_busy  : std_logic := '0';
    signal operation_command_rejected : std_logic := '0';

    signal shadow_cfg : lidar_runtime_config_t;
    signal shadow_gpx_image : gpx_register_image_t;
    signal commit_pulse : std_logic;
    signal clear_pulse  : std_logic;
    signal reset_pulse  : std_logic;
    signal operation_command_valid : std_logic;
    signal operation_command       : operation_command_t;
    signal irq          : std_logic;
    signal runtime_irq  : lidar_runtime_irq_t := C_RUNTIME_IRQ_CLEAR;
    signal diag_request_valid : std_logic;
    signal diag_request_ready : std_logic := '1';
    signal diag_request_index : lidar_diag_index_t;
    signal diag_response_valid : std_logic := '0';
    signal diag_response_ready : std_logic;
    signal diag_response : lidar_diag_response_t := (others => '0');

    signal commit_count : natural := 0;
    signal clear_count  : natural := 0;
    signal reset_count  : natural := 0;
    signal operation_command_count : natural := 0;
    signal last_operation_command  : operation_command_t := OP_COMMAND_NONE;

begin

    p_clock : process
    begin
        while not stop_clock loop
            clk <= '0';
            wait for C_CLK_PERIOD / 2;
            clk <= '1';
            wait for C_CLK_PERIOD / 2;
        end loop;
        wait;
    end process p_clock;

    p_command_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if commit_pulse = '1' then
                commit_count <= commit_count + 1;
            end if;
            if clear_pulse = '1' then
                clear_count <= clear_count + 1;
            end if;
            if reset_pulse = '1' then
                reset_count <= reset_count + 1;
            end if;
            if operation_command_valid = '1' then
                operation_command_count <= operation_command_count + 1;
                last_operation_command <= operation_command;
            end if;
        end if;
    end process p_command_monitor;

    p_diag_responder : process (clk)
        variable v_data : lidar_diag_word_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                diag_response_valid <= '0';
                diag_response <= (others => '0');
            else
                if diag_response_valid = '1' and
                        diag_response_ready = '1' then
                    diag_response_valid <= '0';
                end if;
                if diag_request_valid = '1' and
                        diag_request_ready = '1' then
                    v_data := x"A50000" & diag_request_index;
                    if diag_request_index = x"FF" then
                        diag_response <= fn_pack_diag_response(v_data, '1');
                    else
                        diag_response <= fn_pack_diag_response(v_data, '0');
                    end if;
                    diag_response_valid <= '1';
                end if;
            end if;
        end if;
    end process p_diag_responder;

    u_dut : entity work.lidar_csr_bank
        generic map (
            G_BUILD_CONFIG => C_DEFAULT_BUILD_CONFIG
        )
        port map (
            i_clk    => clk,
            i_rst_n  => rst_n,
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
            i_cfg_busy              => cfg_busy,
            i_cfg_done              => cfg_done,
            i_cfg_commit_rejected   => cfg_rejected,
            i_cfg_reject_error      => cfg_reject_error,
            i_cfg_error             => cfg_error,
            i_cfg_recovery_required => cfg_recovery,
            i_cfg_active_valid      => active_valid,
            i_cfg_active            => active_cfg,
            i_cfg_active_gpx_image  => active_gpx_image,
            i_operation_status      => operation_status,
            i_operation_command_ready => operation_command_ready,
            i_operation_command_busy  => operation_command_busy,
            i_operation_command_rejected => operation_command_rejected,
            i_runtime_irq          => runtime_irq,
            o_diag_request_valid   => diag_request_valid,
            i_diag_request_ready   => diag_request_ready,
            o_diag_request_index   => diag_request_index,
            i_diag_response_valid  => diag_response_valid,
            o_diag_response_ready  => diag_response_ready,
            i_diag_response        => diag_response,
            o_shadow             => shadow_cfg,
            o_gpx_image_shadow   => shadow_gpx_image,
            o_commit             => commit_pulse,
            o_clear_status       => clear_pulse,
            o_soft_reset_request => reset_pulse,
            o_operation_command_valid => operation_command_valid,
            o_operation_command       => operation_command,
            o_irq                => irq
        );

    p_stimulus : process
        variable v_runtime : lidar_runtime_config_t;
        variable v_word    : std_logic_vector(31 downto 0);

        procedure wait_cycles(constant count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
        end procedure wait_cycles;

        procedure axi_write(
            constant address  : natural;
            constant value    : std_logic_vector(31 downto 0);
            constant strobe   : std_logic_vector(3 downto 0) := "1111";
            constant aw_delay : natural := 0;
            constant w_delay  : natural := 0;
            constant b_delay  : natural := 0
        ) is
            variable aw_sent : boolean := false;
            variable w_sent  : boolean := false;
            variable cycles  : natural := 0;
            variable held_bresp : std_logic_vector(1 downto 0);
        begin
            wait until falling_edge(clk);
            awaddr <= std_logic_vector(to_unsigned(address, awaddr'length));
            wdata  <= value;
            wstrb  <= strobe;

            while not (aw_sent and w_sent) loop
                if cycles >= aw_delay and not aw_sent then
                    awvalid <= '1';
                end if;
                if cycles >= w_delay and not w_sent then
                    wvalid <= '1';
                end if;
                wait until rising_edge(clk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_sent := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wvalid <= '0';
                    w_sent := true;
                end if;
                cycles := cycles + 1;
            end loop;
            wstrb <= (others => '0');

            while bvalid = '0' loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
            assert bresp = "00"
                report "V2-CSR-BANK AXI write response is not OKAY"
                severity failure;
            held_bresp := bresp;
            for index in 1 to b_delay loop
                wait until rising_edge(clk);
                wait for 1 ps;
                assert bvalid = '1' and bresp = held_bresp
                    report "V2-CSR-BANK B channel changed under backpressure"
                    severity failure;
            end loop;
            bready <= '1';
            wait until rising_edge(clk);
            bready <= '0';
            wait for 1 ps;
        end procedure axi_write;

        procedure axi_read_value(
            constant address : natural;
            variable value   : out std_logic_vector(31 downto 0);
            constant ready_delay : natural := 0
        ) is
            variable held_data : std_logic_vector(31 downto 0);
        begin
            wait until falling_edge(clk);
            araddr  <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
            assert rresp = "00"
                report "V2-CSR-BANK AXI read response is not OKAY"
                severity failure;
            held_data := rdata;
            for index in 1 to ready_delay loop
                wait until rising_edge(clk);
                wait for 1 ps;
                assert rvalid = '1' and rdata = held_data and rresp = "00"
                    report "V2-CSR-BANK R channel changed under backpressure"
                    severity failure;
            end loop;
            value := held_data;
            rready <= '1';
            wait until rising_edge(clk);
            rready <= '0';
            wait for 1 ps;
        end procedure axi_read_value;

        procedure axi_read(
            constant address  : natural;
            constant expected : std_logic_vector(31 downto 0);
            constant ready_delay : natural := 0
        ) is
            variable actual : std_logic_vector(31 downto 0);
        begin
            axi_read_value(address, actual, ready_delay);
            assert actual = expected
                report "V2-CSR-BANK read mismatch at byte address " &
                    integer'image(address)
                severity failure;
        end procedure axi_read;

        procedure pulse_done(constant error_value : lidar_cfg_error_t) is
        begin
            wait until falling_edge(clk);
            cfg_error <= error_value;
            cfg_done  <= '1';
            wait until rising_edge(clk);
            wait until falling_edge(clk);
            cfg_done <= '0';
        end procedure pulse_done;

        procedure pulse_reject is
        begin
            wait until falling_edge(clk);
            cfg_rejected <= '1';
            wait until rising_edge(clk);
            wait until falling_edge(clk);
            cfg_rejected <= '0';
        end procedure pulse_reject;
    begin
        rst_n <= '0';
        wait_cycles(5);
        rst_n <= '1';
        wait_cycles(3);

        axi_read(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000000");
        axi_read(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00020E10");
        axi_read(fn_ctl_byte_offset(C_CTL_FACE_CENTER_0), x"000005A0");
        axi_read(fn_stat_byte_offset(C_STAT_CORE_INFO), x"3E250205");
        axi_read(fn_stat_byte_offset(C_STAT_BUILD_INFO), x"0C30C896");
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000100", 2);
        axi_read(fn_ctl_byte_offset(C_CTL_RESERVED_FIRST), x"00000000");
        assert shadow_cfg = fn_default_runtime_config(C_DEFAULT_BUILD_CONFIG)
            report "V2-CSR-BANK reset shadow mismatch"
            severity failure;

        -- CTL23/24 provide one request/response transaction at a time. The
        -- returned sequence increments only when a complete 33-bit response
        -- is accepted, so DATA and ERROR always describe the selected index.
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00000000");
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_DATA), x"00000000");
        axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00000110");
        wait_cycles(5);
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00010210");
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_DATA), x"A5000010");

        -- A second CAPTURE write cannot replace the selected index while the
        -- first request is held by downstream backpressure.
        diag_request_ready <= '0';
        axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00000120");
        wait_cycles(2);
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00010120");
        axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00000121");
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00010120");
        diag_request_ready <= '1';
        wait_cycles(5);
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"00020220");
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_DATA), x"A5000020");

        axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"000001FF");
        wait_cycles(5);
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), x"000306FF");
        axi_read(fn_ctl_byte_offset(C_CTL_DIAG_DATA), x"A50000FF");

        -- CTL21/22 form one indexed GPX image portal. Selection never changes
        -- the image; DATA writes edit only the staging view.
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000000");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA),
            C_GPX_REGISTER_IMAGE_DEFAULT(0));
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000005");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA),
            C_GPX_REGISTER_IMAGE_DEFAULT(5));
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00123456");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00123456");
        assert shadow_gpx_image(5) = x"00123456" and
            shadow_gpx_image(0) = C_GPX_REGISTER_IMAGE_DEFAULT(0)
            report "V2-CSR-BANK indexed staging image mismatch"
            severity failure;

        -- Active view is read-only and returns zero until the central
        -- transaction reports a valid active configuration.
        active_gpx_image(5) <= x"000ABCDE";
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000105");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00000000");
        active_valid <= '1';
        wait_cycles(1);
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"000ABCDE");
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00011111");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"000ABCDE");
        active_valid <= '0';
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_INDEX), x"00000005");

        -- GPX is a 28-bit bus. An upper-nibble write is rejected without
        -- silently changing the selected staging entry.
        axi_write(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"F0123456");
        axi_read(fn_ctl_byte_offset(C_CTL_GPX_IMAGE_DATA), x"00123456");
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_ACCESS_ERROR_BIT) = '1'
            report "V2-CSR-BANK invalid GPX image word was not rejected"
            severity failure;

        -- Simultaneous, AW-first and W-first writes all preserve byte strobes.
        axi_write(fn_ctl_byte_offset(C_CTL_LASER_FIRE_PROFILE),
            x"AABBCCDD", "0101", 0, 0, 2);
        axi_read(fn_ctl_byte_offset(C_CTL_LASER_FIRE_PROFILE), x"01BB00DD");
        axi_write(fn_ctl_byte_offset(C_CTL_VIRTUAL_TICKS_LO),
            x"00000258", "1111", 0, 3);
        axi_write(fn_ctl_byte_offset(C_CTL_SIM_START_DELAY),
            x"0000008C", "1111", 3, 0);
        assert shadow_cfg.laser.fire_width_5ns_ticks = to_unsigned(16#00DD#, 16)
            and shadow_cfg.laser.fire_done_timeout_5ns_ticks =
                to_unsigned(16#01BB#, 16)
            and shadow_cfg.motor.virtual_ticks_lo = to_unsigned(600, 32)
            and shadow_cfg.laser.simulation_start_delay_5ns =
                to_unsigned(140, 32)
            report "V2-CSR-BANK shadow field decode mismatch"
            severity failure;

        -- Source mode is writable, while reserved bits and decode 11 are
        -- rejected atomically without changing the previous legal word.
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        axi_read(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        assert shadow_cfg.motor.simulation_mode = '1'
            report "V2-CSR-BANK simulation mode decode mismatch"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00320E10");
        axi_read(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00130E10");
        axi_read(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00120E10");
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_ACCESS_ERROR_BIT) = '1'
            report "V2-CSR-BANK invalid encoding did not set access error"
            severity failure;

        axi_write(fn_ctl_byte_offset(C_CTL_MOTOR_PROFILE), x"00020E10");

        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        wait_cycles(2);
        assert clear_count = 1
            report "V2-CSR-BANK clear command was not a one-shot event"
            severity failure;
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000100");

        -- RUN/STOP/ARM/DISARM are independent W1S events in CTL0. They are
        -- never stored as command levels.
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000008");
        wait_cycles(2);
        assert operation_command_count = 1 and
            last_operation_command = OP_COMMAND_RUN and
            operation_command_valid = '0'
            report "V2-CSR-BANK RUN command event mismatch"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000010");
        wait_cycles(2);
        assert operation_command_count = 2 and
            last_operation_command = OP_COMMAND_STOP
            report "V2-CSR-BANK STOP command event mismatch"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000020");
        wait_cycles(2);
        assert operation_command_count = 3 and
            last_operation_command = OP_COMMAND_ARM
            report "V2-CSR-BANK ARM command event mismatch"
            severity failure;
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000040");
        wait_cycles(2);
        assert operation_command_count = 4 and
            last_operation_command = OP_COMMAND_DISARM
            report "V2-CSR-BANK DISARM command event mismatch"
            severity failure;

        operation_command_ready <= '0';
        operation_command_busy  <= '1';
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000008");
        wait_cycles(2);
        assert operation_command_count = 4
            report "V2-CSR-BANK busy operation command escaped"
            severity failure;
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_ACCESS_ERROR_BIT) = '1'
            report "V2-CSR-BANK busy operation command was not diagnosed"
            severity failure;
        operation_command_ready <= '1';
        operation_command_busy  <= '0';
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        wait_cycles(2);

        -- Multi-command writes are rejected; a zero W1S write is a no-op.
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000000");
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000005");
        wait_cycles(2);
        assert commit_count = 0 and reset_count = 0
            report "V2-CSR-BANK conflicting command escaped"
            severity failure;
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_ACCESS_ERROR_BIT) = '1'
            report "V2-CSR-BANK command conflict was not diagnosed"
            severity failure;

        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        axi_write(fn_ctl_byte_offset(C_CTL_LASER_FIRE_PROFILE),
            x"01200014");
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000001");
        wait_cycles(2);
        assert commit_count = 1 and commit_pulse = '0'
            report "V2-CSR-BANK commit command was not exactly one cycle"
            severity failure;

        cfg_busy <= '1';
        wait_cycles(1);
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_BUSY_BIT) = '1'
            report "V2-CSR-BANK live BUSY status missing"
            severity failure;
        cfg_busy <= '0';

        pulse_done(CFG_RUNTIME_FIRE_TIMEOUT);
        wait_cycles(1);
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"002C010A");
        axi_read(fn_stat_byte_offset(C_STAT_DERIVED_MASKS), x"00010000");

        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        v_runtime := shadow_cfg;
        active_cfg <= fn_active_config(v_runtime, 1);
        active_valid <= '1';
        wait_cycles(1);
        pulse_done(CFG_OK);
        wait_cycles(1);
        axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), x"00000046");
        operation_status <= (
            running              => '1',
            armed                => '1',
            external_permit      => '1',
            config_ready         => '1',
            processing_enable    => '1',
            scheduler_enable     => '1',
            physical_fire_enable => '1',
            simulation_enable    => '0'
        );
        wait_cycles(1);
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), x"017F0001");
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_SOURCE_BASE +
            C_CTL_LASER_FIRE_PROFILE - 1), x"01200014");
        axi_read(fn_stat_byte_offset(C_STAT_DERIVED_GEOMETRY), x"00013840");
        axi_read(fn_stat_byte_offset(C_STAT_DERIVED_FACE), x"09600961");
        axi_read(fn_stat_byte_offset(C_STAT_FACE_BOUNDS_0), x"0A5000F0");
        -- CTL12=288 common 5 ns ticks is rounded up to Reg7.MTimer=58
        -- 25 ns ticks, so the effective GPX capture window is 290 ticks.
        axi_read(fn_stat_byte_offset(C_STAT_CAPTURE_TDC_CLKS), x"00000122");
        axi_read(fn_stat_byte_offset(C_STAT_DERIVED_MASKS), x"00020C3F");

        pulse_reject;
        wait_cycles(1);
        axi_read_value(fn_stat_byte_offset(C_STAT_TRANSACTION), v_word);
        assert v_word(C_TXN_REJECTED_STICKY_BIT) = '1'
            and v_word(31 downto 24) = x"71"
            report "V2-CSR-BANK rejection status mismatch"
            severity failure;

        -- Enable all implemented IRQ sources, then create an access event.
        -- Pending flags retain events even while masked, so software clears
        -- the historical events before opening this test's enable mask.
        axi_write(fn_irq_byte_offset(2), x"FFFFFFFF");
        wait_cycles(3);
        axi_write(fn_irq_byte_offset(0), x"0000001F");
        axi_write(fn_ctl_byte_offset(C_CTL_RESERVED_FIRST), x"FFFFFFFF");
        wait_cycles(6);
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word(C_IRQ_ACCESS_ERROR) = '1' and irq = '1'
            report "V2-CSR-BANK manual IRQ flag did not latch"
            severity failure;
        axi_write(fn_irq_byte_offset(2), x"00000010");
        wait_cycles(3);
        axi_read(fn_irq_byte_offset(2), x"00000000");
        assert irq = '0'
            report "V2-CSR-BANK IRQ did not clear after W1C"
            severity failure;

        -- Soft reset remains a top-level request and is not stored in CTL0.
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000004");
        wait_cycles(2);
        assert reset_count = 1 and reset_pulse = '0'
            report "V2-CSR-BANK soft-reset request was not one cycle"
            severity failure;
        axi_read(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000000");

        report "LIDAR_V2_CSR_BANK_PASS" severity note;
        stop_clock <= true;
        wait;
    end process p_stimulus;

end architecture sim;
