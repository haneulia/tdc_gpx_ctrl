-- ============================================================================
-- 테스트 자산 목적: native status snapshot, CTL23/24 portal과 IRQ 통합 계약을 검증한다.
-- 핵심 검증 계약: 물리 GPX read, reset-abort, IRQ source map, CLEAR와 W1C 소유권이다.
-- 관련 RTL: lidar_csr_bank, lidar_status_snapshot_subsystem, system command CDC.
-- 실행 회귀: scripts/run_v2_k08_status_irq.ps1
-- 유지보수 주의: sticky owner 추가 시 fault 주입부터 CLEAR/W1C 순서까지 확장해야 한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_status_pkg.all;
use work.lidar_csr_map_pkg.all;

entity tb_lidar_status_irq_integration is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_TDC_CLK_MHZ         : positive := 200;
        G_PROC_HALF_PERIOD_PS : positive := 3333;
        G_TDC_HALF_PERIOD_PS  : positive := 2500
    );
end entity tb_lidar_status_irq_integration;

architecture sim of tb_lidar_status_irq_integration is

    constant C_CSR_PERIOD : time := 10 ns;
    constant C_PROC_HALF_PERIOD : time :=
        G_PROC_HALF_PERIOD_PS * 1 ps;
    constant C_TDC_HALF_PERIOD : time :=
        G_TDC_HALF_PERIOD_PS * 1 ps;

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    function fn_active_config(
        source_value  : lidar_runtime_config_t;
        version_value : natural
    ) return lidar_active_config_t is
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(version_value, 16);
        result.source := source_value;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, source_value);
        return result;
    end function fn_active_config;

    signal stop_clocks : boolean := false;
    signal csr_clk : std_logic := '0';
    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';

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

    signal shadow_cfg : lidar_runtime_config_t;
    signal shadow_gpx : gpx_register_image_t;
    signal commit_pulse : std_logic;
    signal csr_clear : std_logic;
    signal csr_soft_reset : std_logic;
    signal operation_command_valid : std_logic;
    signal operation_command : operation_command_t;
    signal irq : std_logic;

    signal diag_request_valid : std_logic;
    signal diag_request_ready : std_logic;
    signal diag_request_index : lidar_diag_index_t;
    signal diag_response_valid : std_logic;
    signal diag_response_ready : std_logic;
    signal diag_response : lidar_diag_response_t;
    signal runtime_irq : lidar_runtime_irq_t;

    signal command_ready : std_logic;
    signal command_rejected : std_logic;
    signal proc_clear : std_logic;
    signal proc_soft_reset : std_logic;
    signal tdc_clear : std_logic;
    signal tdc_soft_reset : std_logic;
    signal proc_clear_seen : std_logic := '0';
    signal tdc_clear_seen : std_logic := '0';

    signal processing_diagnostics : processing_diagnostics_t :=
        C_PROCESSING_DIAGNOSTICS_CLEAR;
    signal echo_diagnostics : echo_diagnostics_t :=
        C_ECHO_DIAGNOSTICS_CLEAR;
    signal face_close_overflow : std_logic := '0';
    signal gpx_context_fault : std_logic := '0';
    signal hit_faults : gpx_hit_decoder_faults_t :=
        C_GPX_HIT_DECODER_FAULTS_CLEAR;
    signal cell_faults : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
    signal frame_faults : gpx_frame_assembler_faults_t :=
        C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR;
    signal rise_profile : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;
    signal fall_profile : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;

    signal active_mask : chip_mask_t := "0011";
    signal terminal_mask : chip_mask_t := "0011";
    signal lane_status : gpx_lane_status_array_t :=
        (others => C_GPX_LANE_STATUS_RESET);
    signal lane_faults : gpx_lane_faults_array_t :=
        (others => C_GPX_LANE_FAULTS_CLEAR);
    signal shot_drop : std_logic := '0';
    signal stop_drop : std_logic := '0';

    signal operation_status : operation_state_t := C_OPERATION_STATE_SAFE;
    signal tdc_safe : std_logic := '1';
    signal tdc_run_enable : std_logic;
    signal register_service_pause : std_logic;
    signal register_read : gpx_register_read_request_t :=
        C_GPX_REGISTER_READ_REQUEST_IDLE;
    signal register_read_ready : std_logic;
    signal register_read_response : gpx_register_read_response_t :=
        C_GPX_REGISTER_READ_RESPONSE_IDLE;
    signal register_read_response_ready : std_logic;
    signal register_read_count : natural := 0;
    signal register_pause_age_r : natural range 0 to 3 := 0;

begin

    -- 실제 Top과 동일하게 maintenance read 동안 acquisition RUN만 내린다.
    -- CSR/Processing clock은 계속 동작하므로 PS가 완료 상태를 읽을 수 있다.
    tdc_run_enable <= not register_service_pause;
    register_read_ready <= register_service_pause and tdc_safe;

    p_csr_clock : process
    begin
        while not stop_clocks loop
            csr_clk <= '0';
            wait for C_CSR_PERIOD / 2;
            csr_clk <= '1';
            wait for C_CSR_PERIOD / 2;
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

    p_clear_monitors : process (proc_clk, tdc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                proc_clear_seen <= '0';
            elsif proc_clear = '1' then
                proc_clear_seen <= '1';
            end if;
        end if;
        if rising_edge(tdc_clk) then
            if tdc_rst_n = '0' then
                tdc_clear_seen <= '0';
            elsif tdc_clear = '1' then
                tdc_clear_seen <= '1';
            end if;
        end if;
    end process p_clear_monitors;

    -- 외부 TDC-GPX register read 응답 모델. Chip 1, Reg7 요청에 대해
    -- 28-bit 실제 readback 값을 반환하고 response backpressure를 지킨다.
    p_gpx_register_model : process (tdc_clk)
    begin
        if rising_edge(tdc_clk) then
            if tdc_rst_n = '0' then
                register_read_response <=
                    C_GPX_REGISTER_READ_RESPONSE_IDLE;
                register_read_count <= 0;
                register_pause_age_r <= 0;
            else
                if register_service_pause = '0' then
                    register_pause_age_r <= 0;
                elsif register_pause_age_r < 3 then
                    register_pause_age_r <= register_pause_age_r + 1;
                end if;

                if register_read_response.valid = '1' and
                   register_read_response_ready = '1' then
                    register_read_response <=
                        C_GPX_REGISTER_READ_RESPONSE_IDLE;
                end if;

                if register_read.valid = '1' and
                   register_read_ready = '1' then
                    -- pause 적용 전의 오래된 safe 값으로 유지보수 read가
                    -- 시작되지 않도록 두 번의 TDC edge가 지난 뒤 수락한다.
                    assert register_pause_age_r >= 2
                        report "K0-8 GPX read started before pause-safe pipeline"
                        severity failure;
                    assert register_read.chip = 1 and
                           register_read.address = x"7"
                        report "K0-8 unexpected physical GPX register request"
                        severity failure;
                    register_read_response.valid <= '1';
                    register_read_response.error <= '0';
                    register_read_response.chip <= register_read.chip;
                    register_read_response.address <= register_read.address;
                    register_read_response.read_data <= x"1ABCDE0";
                    register_read_count <= register_read_count + 1;
                end if;
            end if;
        end if;
    end process p_gpx_register_model;

    u_csr : entity work.lidar_csr_bank
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => csr_clk,
            i_rst_n => csr_rst_n,
            s_axi_awaddr => awaddr,
            s_axi_awprot => awprot,
            s_axi_awvalid => awvalid,
            s_axi_awready => awready,
            s_axi_wdata => wdata,
            s_axi_wstrb => wstrb,
            s_axi_wvalid => wvalid,
            s_axi_wready => wready,
            s_axi_bresp => bresp,
            s_axi_bvalid => bvalid,
            s_axi_bready => bready,
            s_axi_araddr => araddr,
            s_axi_arprot => arprot,
            s_axi_arvalid => arvalid,
            s_axi_arready => arready,
            s_axi_rdata => rdata,
            s_axi_rresp => rresp,
            s_axi_rvalid => rvalid,
            s_axi_rready => rready,
            i_cfg_busy => '0',
            i_cfg_done => '0',
            i_cfg_commit_rejected => '0',
            i_cfg_reject_error => CFG_OK,
            i_cfg_error => CFG_OK,
            i_cfg_recovery_required => '0',
            i_cfg_active_valid => '0',
            i_cfg_active => fn_active_config(
                fn_default_runtime_config(C_BUILD_CONFIG), 0),
            i_cfg_active_gpx_image => C_GPX_REGISTER_IMAGE_DEFAULT,
            i_operation_status => operation_status,
            i_operation_command_ready => '1',
            i_operation_command_busy => '0',
            i_operation_command_rejected => '0',
            i_system_command_ready => command_ready,
            i_system_command_rejected => command_rejected,
            i_runtime_irq => runtime_irq,
            o_diag_request_valid => diag_request_valid,
            i_diag_request_ready => diag_request_ready,
            o_diag_request_index => diag_request_index,
            i_diag_response_valid => diag_response_valid,
            o_diag_response_ready => diag_response_ready,
            i_diag_response => diag_response,
            o_shadow => shadow_cfg,
            o_gpx_image_shadow => shadow_gpx,
            o_commit => commit_pulse,
            o_clear_status => csr_clear,
            o_soft_reset_request => csr_soft_reset,
            o_operation_command_valid => operation_command_valid,
            o_operation_command => operation_command,
            o_irq => irq
        );

    u_command_cdc : entity work.lidar_system_command_cdc
        port map (
            i_source_clk => csr_clk,
            i_source_rst_n => csr_rst_n,
            i_clear_status => csr_clear,
            i_soft_reset => csr_soft_reset,
            o_source_ready => command_ready,
            o_source_busy => open,
            o_source_rejected => command_rejected,
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            o_proc_clear_status => proc_clear,
            o_proc_soft_reset => proc_soft_reset,
            i_tdc_clk => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            o_tdc_clear_status => tdc_clear,
            o_tdc_soft_reset => tdc_soft_reset
        );

    u_status : entity work.lidar_status_snapshot_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_csr_clk => csr_clk,
            i_csr_rst_n => csr_rst_n,
            i_csr_request_valid => diag_request_valid,
            o_csr_request_ready => diag_request_ready,
            i_csr_request_index => diag_request_index,
            o_csr_response_valid => diag_response_valid,
            i_csr_response_ready => diag_response_ready,
            o_csr_response => diag_response,
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_proc_clear_status => proc_clear,
            i_processing_diagnostics => processing_diagnostics,
            i_echo_diagnostics => echo_diagnostics,
            i_face_close_overflow_sticky => face_close_overflow,
            i_pipeline_idle => '1',
            i_echo_idle => '1',
            i_gpx_proc_idle => '1',
            i_gpx_axis_idle => '1',
            i_echo_profile_ready => '1',
            i_echo_profile_busy => '0',
            i_echo_profile_version => to_unsigned(16#1234#, 16),
            i_rise_profile => rise_profile,
            i_fall_profile => fall_profile,
            i_gpx_context_fault_sticky => gpx_context_fault,
            i_gpx_shot_drop_sticky => shot_drop,
            i_gpx_stop_drop_sticky => stop_drop,
            i_gpx_cdc_reset_busy => '0',
            i_hit_fault_sticky => hit_faults,
            i_cell_fault_sticky => cell_faults,
            i_frame_fault_sticky => frame_faults,
            i_b0_to_accept_clks => to_unsigned(5, 8),
            i_physical_to_fire_clks => to_unsigned(9, 8),
            i_virtual_to_accept_clks => to_unsigned(7, 8),
            i_fire_done_sync_clks => to_unsigned(3, 16),
            i_rearm_margin_clks => to_unsigned(2, 16),
            i_tdc_clk => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            i_tdc_clear_status => tdc_clear,
            i_active_mask => active_mask,
            i_terminal_mask => terminal_mask,
            i_lane_status => lane_status,
            i_lane_faults => lane_faults,
            i_tdc_safe => tdc_safe,
            i_tdc_run_enable => tdc_run_enable,
            i_tdc_active_valid => '1',
            i_tdc_config_ready => '1',
            o_tdc_register_service_pause => register_service_pause,
            o_tdc_register_read => register_read,
            i_tdc_register_read_ready => register_read_ready,
            i_tdc_register_read_response => register_read_response,
            o_tdc_register_read_response_ready =>
                register_read_response_ready,
            o_runtime_irq => runtime_irq
        );

    p_stimulus : process
        variable v_word : std_logic_vector(31 downto 0);
        variable v_sequence : natural := 0;
        variable v_control : std_logic_vector(31 downto 0);

        procedure wait_csr_cycles(constant count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(csr_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_csr_cycles;

        procedure axi_write(
            constant address : natural;
            constant value   : std_logic_vector(31 downto 0)
        ) is
            variable aw_sent : boolean := false;
            variable w_sent  : boolean := false;
        begin
            wait until falling_edge(csr_clk);
            awaddr <= std_logic_vector(to_unsigned(address, awaddr'length));
            wdata <= value;
            wstrb <= "1111";
            awvalid <= '1';
            wvalid <= '1';
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
                wait until rising_edge(csr_clk);
            end loop;
            assert bresp = "00"
                report "K0-8 AXI write response mismatch"
                severity failure;
            bready <= '1';
            wait until rising_edge(csr_clk);
            bready <= '0';
            wait for 1 ps;
        end procedure axi_write;

        procedure axi_read_value(
            constant address : natural;
            variable value   : out std_logic_vector(31 downto 0)
        ) is
        begin
            wait until falling_edge(csr_clk);
            araddr <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(csr_clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                wait until rising_edge(csr_clk);
            end loop;
            assert rresp = "00"
                report "K0-8 AXI read response mismatch"
                severity failure;
            value := rdata;
            rready <= '1';
            wait until rising_edge(csr_clk);
            rready <= '0';
            wait for 1 ps;
        end procedure axi_read_value;

        procedure capture_diag(
            constant index_value : natural;
            constant expected_data : std_logic_vector(31 downto 0);
            constant expected_error : std_logic := '0'
        ) is
            variable actual : std_logic_vector(31 downto 0);
            variable complete : boolean := false;
        begin
            axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX),
                std_logic_vector(to_unsigned(
                    16#100# + index_value, 32)));
            for attempt in 0 to 80 loop
                axi_read_value(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), actual);
                if actual(C_DIAG_VALID_BIT) = '1' then
                    complete := true;
                    exit;
                end if;
            end loop;
            assert complete
                report "K0-8 diagnostic portal did not complete"
                severity failure;

            v_sequence := v_sequence + 1;
            v_control := (others => '0');
            v_control(C_DIAG_INDEX_MSB downto C_DIAG_INDEX_LSB) :=
                std_logic_vector(to_unsigned(index_value, 8));
            v_control(C_DIAG_VALID_BIT) := '1';
            v_control(C_DIAG_ERROR_BIT) := expected_error;
            v_control(C_DIAG_SEQUENCE_MSB downto C_DIAG_SEQUENCE_LSB) :=
                std_logic_vector(to_unsigned(v_sequence, 16));
            assert actual = v_control
                report "K0-8 diagnostic control mismatch"
                severity failure;
            axi_read_value(fn_ctl_byte_offset(C_CTL_DIAG_DATA), actual);
            assert actual = expected_data
                report "K0-8 diagnostic data mismatch"
                severity failure;
        end procedure capture_diag;

        procedure abort_diag_with_remote_reset(
            signal remote_rst_n : out std_logic;
            constant index_value : natural
        ) is
            variable actual : std_logic_vector(31 downto 0);
            variable complete : boolean := false;
        begin
            axi_write(fn_ctl_byte_offset(C_CTL_DIAG_INDEX),
                std_logic_vector(to_unsigned(
                    16#100# + index_value, 32)));
            remote_rst_n <= '0';
            wait_csr_cycles(6);
            remote_rst_n <= '1';

            for attempt in 0 to 80 loop
                axi_read_value(fn_ctl_byte_offset(C_CTL_DIAG_INDEX), actual);
                if actual(C_DIAG_VALID_BIT) = '1' then
                    complete := true;
                    exit;
                end if;
            end loop;
            assert complete
                report "K0-8 reset-aborted diagnostic did not complete"
                severity failure;

            v_sequence := v_sequence + 1;
            v_control := (others => '0');
            v_control(C_DIAG_INDEX_MSB downto C_DIAG_INDEX_LSB) :=
                std_logic_vector(to_unsigned(index_value, 8));
            v_control(C_DIAG_VALID_BIT) := '1';
            v_control(C_DIAG_ERROR_BIT) := '1';
            v_control(C_DIAG_SEQUENCE_MSB downto C_DIAG_SEQUENCE_LSB) :=
                std_logic_vector(to_unsigned(v_sequence, 16));
            assert actual = v_control
                report "K0-8 reset-aborted diagnostic control mismatch"
                severity failure;
            axi_read_value(fn_ctl_byte_offset(C_CTL_DIAG_DATA), actual);
            assert actual = x"00000000"
                report "K0-8 reset-aborted diagnostic data was not zero"
                severity failure;
            wait_csr_cycles(20);
        end procedure abort_diag_with_remote_reset;

    begin
        csr_rst_n <= '0';
        proc_rst_n <= '0';
        tdc_rst_n <= '0';
        wait_csr_cycles(8);
        csr_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait_csr_cycles(30);

        -- A destination reset must terminate one accepted transaction with
        -- ERROR, then rebase the one-entry mailbox without replaying it.
        abort_diag_with_remote_reset(tdc_rst_n, C_DIAG_TDC_SUMMARY);
        capture_diag(C_DIAG_TDC_SUMMARY, x"00007433");
        abort_diag_with_remote_reset(proc_rst_n, C_DIAG_PROC_FLAGS);
        capture_diag(C_DIAG_PROC_INVALID_COUNT, x"00000000");

        -- 11CCAAAA에서 CC=01(Chip 1), AAAA=0111(Reg7)이다.
        -- scheduler가 켜져 있으면 물리 bus 요청 없이 ordering error로 거부한다.
        operation_status.scheduler_enable <= '1';
        capture_diag(16#D7#, x"70000000", '1');
        assert register_read_count = 0
            report "K0-8 armed physical GPX read reached the bus"
            severity failure;

        -- DISARM 뒤에는 acquisition만 자동 pause하고 실제 28-bit 값을 읽는다.
        operation_status.scheduler_enable <= '0';
        capture_diag(16#D7#, x"71ABCDE0");
        assert register_read_count = 1 and register_service_pause = '0' and
               tdc_run_enable = '1'
            report "K0-8 physical GPX read did not resume acquisition"
            severity failure;

        rise_profile.enabled <= '1';
        rise_profile.valid <= '1';

        -- Five independent runtime classes become stable native-domain
        -- levels before the explicit CSR-domain interrupt synchronizers.
        processing_diagnostics.invalid_transition_sticky <= '1';
        processing_diagnostics.invalid_transition_count <=
            to_unsigned(16#11223344#, 32);
        processing_diagnostics.source_switch_pulse <= '1';
        processing_diagnostics.laser.fire_done_timeout_sticky <= '1';
        processing_diagnostics.laser.fire_done_timeout_count <=
            to_unsigned(7, 32);
        gpx_context_fault <= '1';
        shot_drop <= '1';
        stop_drop <= '1';
        hit_faults.chip_index_error <= '1';
        echo_diagnostics.snapshot.timeout <= '1';
        lane_faults(0).drain_timeout_pulse <= '1';
        lane_faults(0).run_timeout_pulse <= '1';
        lane_faults(0).run_timeout_cause <= "101";
        -- K1-1: legacy controller가 소유하는 TDC lane sticky bit를 모두
        -- 올려 진단 page와 GPX_TRANSPORT IRQ bit 집계를 확인한다.
        lane_faults(0).response_mismatch_sticky <= '1';
        lane_faults(0).raw_drop_sticky <= '1';
        lane_faults(0).drain_cap_sticky <= '1';
        lane_faults(0).init_cfg_coalesced_sticky <= '1';
        lane_faults(0).command_collision_sticky <= '1';
        lane_faults(0).bus_fatal_sticky <= '1';
        wait until rising_edge(proc_clk);
        processing_diagnostics.source_switch_pulse <= '0';
        echo_diagnostics.snapshot.timeout <= '0';
        wait until rising_edge(tdc_clk);
        lane_faults(0).drain_timeout_pulse <= '0';
        lane_faults(0).run_timeout_pulse <= '0';
        lane_faults(0).run_timeout_cause <= "000";

        axi_write(fn_irq_byte_offset(2), x"FFFFFFFF");
        axi_write(fn_irq_byte_offset(0), x"000003E0");
        wait_csr_cycles(10);

        capture_diag(C_DIAG_PROC_FLAGS, x"0335F203");
        capture_diag(C_DIAG_PROC_INVALID_COUNT, x"11223344");
        capture_diag(C_DIAG_ECHO_FLAGS, x"00000040");
        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00060003");
        capture_diag(C_DIAG_TDC_LANE_FAULT_0, x"00001EDD");
        capture_diag(16#00#, x"00000000", '1');

        axi_read_value(fn_irq_byte_offset(1), v_word);
        assert v_word = x"000003E0"
            report "K0-8 runtime IRQ source map mismatch"
            severity failure;
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"000003F0" and irq = '1'
            report "K0-8 runtime IRQ pending/level mismatch"
            severity failure;

        -- Portal errors also own the existing ACCESS_ERROR event (bit 4).
        -- Clear that IRQ flag separately before checking runtime clear.
        axi_write(fn_irq_byte_offset(2), x"00000010");
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        processing_diagnostics.invalid_transition_sticky <= '0';
        processing_diagnostics.laser.fire_done_timeout_sticky <= '0';
        gpx_context_fault <= '0';
        shot_drop <= '0';
        stop_drop <= '0';
        hit_faults.chip_index_error <= '0';
        -- 실제 owner의 set/clear 우선순위는 전용 K1-1 fault-injection에서
        -- 검증한다. 이 통합 TB에서는 owner clear 후 보이는 level을 모델링해
        -- CSR source 하강과 IRQ_FLAG W1C 순서에 집중한다.
        lane_faults(0).response_mismatch_sticky <= '0';
        lane_faults(0).raw_drop_sticky <= '0';
        lane_faults(0).drain_cap_sticky <= '0';
        lane_faults(0).init_cfg_coalesced_sticky <= '0';
        lane_faults(0).command_collision_sticky <= '0';
        lane_faults(0).bus_fatal_sticky <= '0';
        wait_csr_cycles(20);
        assert proc_clear_seen = '1' and tdc_clear_seen = '1'
            report "K0-8 CLEAR_STATUS did not reach both destination domains"
            severity failure;

        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00000000");
        capture_diag(C_DIAG_ECHO_FLAGS, x"00000000");
        capture_diag(C_DIAG_TDC_LANE_FAULT_0, x"00000000");

        axi_read_value(fn_irq_byte_offset(1), v_word);
        assert v_word = x"00000000"
            report "K0-8 runtime IRQ source did not clear"
            severity failure;
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"000003E0" and irq = '1'
            report "K0-8 W1C IRQ ownership was lost on CLEAR_STATUS"
            severity failure;
        axi_write(fn_irq_byte_offset(2), x"000003E0");
        wait_csr_cycles(4);
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"00000000" and irq = '0'
            report "K0-8 runtime IRQ W1C clear mismatch"
            severity failure;

        -- K1-1 status-source priority: TDC pulse fault를 CLEAR_STATUS와 같은
        -- TDC clock에 겹친다. 새 사건은 clear에 유실되지 않아야 하며,
        -- 원인을 두 번째 clear로 내린 뒤에도 IRQ_FLAG는 W1C 전까지 남는다.
        lane_faults(0).drain_timeout_pulse <= '1';
        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        loop
            wait until rising_edge(tdc_clk);
            exit when tdc_clear = '1';
        end loop;
        lane_faults(0).drain_timeout_pulse <= '0';
        wait_csr_cycles(20);
        capture_diag(C_DIAG_TDC_LANE_FAULT_0, x"00000001");
        axi_read_value(fn_irq_byte_offset(1), v_word);
        assert v_word = x"00000100"
            report "K1-1 same-cycle TDC fault was lost by CLEAR_STATUS"
            severity failure;
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"00000100" and irq = '1'
            report "K1-1 same-cycle TDC fault did not preserve IRQ_FLAG"
            severity failure;

        axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), x"00000002");
        wait_csr_cycles(20);
        capture_diag(C_DIAG_TDC_LANE_FAULT_0, x"00000000");
        axi_read_value(fn_irq_byte_offset(1), v_word);
        assert v_word = x"00000000"
            report "K1-1 second CLEAR_STATUS did not remove TDC fault source"
            severity failure;
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"00000100" and irq = '1'
            report "K1-1 second CLEAR_STATUS incorrectly cleared IRQ_FLAG"
            severity failure;
        axi_write(fn_irq_byte_offset(2), x"00000100");
        wait_csr_cycles(4);
        axi_read_value(fn_irq_byte_offset(2), v_word);
        assert v_word = x"00000000" and irq = '0'
            report "K1-1 same-cycle fault IRQ W1C mismatch"
            severity failure;

        report "LIDAR_V2_K08_STATUS_IRQ_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
            severity note;
        stop_clocks <= true;
        wait;
    end process p_stimulus;

end architecture sim;
