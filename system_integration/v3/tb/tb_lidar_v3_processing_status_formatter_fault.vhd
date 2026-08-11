library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;
use work.lidar_echo_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;
use work.lidar_status_pkg.all;
use work.lidar_v3_status_pkg.all;

-- H6-B1 formatter 진단 전용 검증.
-- H4 Rise/Fall 포매터의 sticky 비트가 GPX_DATA IRQ, 요약 진단 0x1A,
-- 상세 진단 0x27에 동일하게 보존되고 CLEAR_STATUS로 함께 지워지는지 확인한다.
entity tb_lidar_v3_processing_status_formatter_fault is
end entity tb_lidar_v3_processing_status_formatter_fault;

architecture sim of tb_lidar_v3_processing_status_formatter_fault is
    constant C_CLK_PERIOD : time := 10 ns;

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal clear_status : std_logic := '0';

    signal request_valid : std_logic := '0';
    signal request_ready : std_logic;
    signal request_index : lidar_diag_index_t := (others => '0');
    signal response_valid : std_logic;
    signal response_ready : std_logic := '1';
    signal response       : lidar_diag_response_t;

    signal rise_formatter_fault : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal fall_formatter_fault : std_logic_vector(7 downto 0) :=
        (others => '0');

    signal irq_processing_warning : std_logic;
    signal irq_laser_timeout      : std_logic;
    signal irq_echo_diagnostic    : std_logic;
    signal irq_gpx_transport      : std_logic;
    signal irq_gpx_data           : std_logic;
begin
    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_v3_processing_status_source
        port map (
            i_clk          => clk,
            i_rst_n        => rst_n,
            i_clear_status => clear_status,

            i_request_valid  => request_valid,
            o_request_ready  => request_ready,
            i_request_index  => request_index,
            o_response_valid => response_valid,
            i_response_ready => response_ready,
            o_response       => response,

            i_processing_diagnostics => C_PROCESSING_DIAGNOSTICS_CLEAR,
            i_echo_diagnostics       => C_ECHO_DIAGNOSTICS_CLEAR,
            i_face_close_overflow_sticky => '0',
            i_pipeline_idle => '1',
            i_echo_idle     => '1',
            i_gpx_proc_idle => '1',
            i_gpx_axis_idle => '1',
            i_echo_profile_ready   => '1',
            i_echo_profile_busy    => '0',
            i_echo_profile_version => (others => '0'),
            i_rise_profile => C_GPX_VDMA_LANE_PROFILE_IDLE,
            i_fall_profile => C_GPX_VDMA_LANE_PROFILE_IDLE,
            i_gpx_context_fault_sticky => '0',
            i_gpx_shot_drop_sticky => '0',
            i_gpx_stop_drop_sticky => '0',
            i_gpx_cdc_reset_busy   => '0',
            i_hit_fault_sticky   => C_GPX_HIT_DECODER_FAULTS_CLEAR,
            i_cell_fault_sticky  => C_GPX_CELL_COLLECTOR_FAULTS_CLEAR,
            i_frame_fault_sticky => C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR,
            i_rise_formatter_fault_sticky => rise_formatter_fault,
            i_fall_formatter_fault_sticky => fall_formatter_fault,
            i_b0_to_accept_clks      => (others => '0'),
            i_physical_to_fire_clks  => (others => '0'),
            i_virtual_to_accept_clks => (others => '0'),
            i_fire_done_sync_clks    => (others => '0'),
            i_rearm_margin_clks      => (others => '0'),

            o_irq_processing_warning => irq_processing_warning,
            o_irq_laser_timeout      => irq_laser_timeout,
            o_irq_echo_diagnostic    => irq_echo_diagnostic,
            o_irq_gpx_transport      => irq_gpx_transport,
            o_irq_gpx_data           => irq_gpx_data
        );

    p_test : process
        procedure capture_diag(
            constant index_value : in natural;
            constant expected    : in std_logic_vector(31 downto 0)
        ) is
        begin
            loop
                wait until rising_edge(clk);
                exit when request_ready = '1';
            end loop;

            request_index <= std_logic_vector(to_unsigned(
                index_value, request_index'length));
            request_valid <= '1';
            wait until rising_edge(clk);
            assert request_ready = '1'
                report "V3-H6B diagnostic request was not accepted"
                severity failure;
            request_valid <= '0';

            loop
                wait until rising_edge(clk);
                exit when response_valid = '1';
            end loop;
            assert response(C_DIAG_RESPONSE_ERROR_BIT) = '0'
                report "V3-H6B diagnostic response unexpectedly reported error"
                severity failure;
            assert response(
                C_DIAG_RESPONSE_DATA_HI downto
                C_DIAG_RESPONSE_DATA_LO) = expected
                report "V3-H6B diagnostic data mismatch at index " &
                    integer'image(index_value)
                severity failure;
        end procedure capture_diag;
    begin
        rst_n <= '0';
        wait for 5 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);
        wait until rising_edge(clk);

        assert irq_processing_warning = '0' and
               irq_laser_timeout = '0' and
               irq_echo_diagnostic = '0' and
               irq_gpx_transport = '0' and
               irq_gpx_data = '0'
            report "V3-H6B IRQ was active before fault injection"
            severity failure;
        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00000000");
        capture_diag(C_DIAG_V3_GPX_FORMATTER_FAULTS, x"00000000");

        -- Rise lane bit 0, 2, 4 fault: summary bit19 and detail[7:0].
        rise_formatter_fault <= x"15";
        wait until rising_edge(clk);
        wait for 1 ns;
        assert irq_gpx_data = '1'
            report "V3-H6B Rise formatter fault did not latch GPX_DATA IRQ"
            severity failure;
        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00080000");
        capture_diag(C_DIAG_V3_GPX_FORMATTER_FAULTS, x"00000015");

        -- Add Fall lane bit 1, 5, 7: summary bit20 and detail[15:8].
        fall_formatter_fault <= x"A2";
        wait until rising_edge(clk);
        wait for 1 ns;
        assert irq_gpx_data = '1'
            report "V3-H6B Fall formatter fault did not preserve GPX_DATA IRQ"
            severity failure;
        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00180000");
        capture_diag(C_DIAG_V3_GPX_FORMATTER_FAULTS, x"0000A215");
        assert irq_processing_warning = '0' and
               irq_laser_timeout = '0' and
               irq_echo_diagnostic = '0' and
               irq_gpx_transport = '0'
            report "V3-H6B formatter fault contaminated an unrelated IRQ class"
            severity failure;

        -- The formatter sticky sources and the status owner receive the same
        -- clear pulse in the integrated top. Model that atomic clear here.
        rise_formatter_fault <= (others => '0');
        fall_formatter_fault <= (others => '0');
        clear_status <= '1';
        wait until rising_edge(clk);
        clear_status <= '0';
        wait until rising_edge(clk);
        wait for 1 ns;
        assert irq_gpx_data = '0'
            report "V3-H6B CLEAR_STATUS did not clear GPX_DATA IRQ"
            severity failure;
        capture_diag(C_DIAG_GPX_PROC_FAULTS, x"00000000");
        capture_diag(C_DIAG_V3_GPX_FORMATTER_FAULTS, x"00000000");

        report "LIDAR_V3_H6B_FORMATTER_STATUS_IRQ_PASS"
            severity note;
        std.env.stop;
        wait;
    end process p_test;
end architecture sim;
