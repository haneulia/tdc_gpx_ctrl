library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- V3 HLS Hit-to-Cell 블록의 OOC 합성·배치·배선 전용 Harness다.
-- 네 TDC-GPX Chip 모두 Rise/Fall 양 Edge를 허용하여 합성 도구가 최대 topology
-- 경로를 제거하지 못하게 한다. 기능 비교용 테스트벤치가 아니며, 이 Harness의
-- PASS는 150/200 MHz 내부 Timing과 Latch/DRC만 보장한다. Parent I/O 지연,
-- 상위 FIFO 점유율 및 전체 Shot 처리 여유는 H5/H6 통합 검증에서 별도로 닫는다.
entity lidar_gpx_cell_collector_hls_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150
    );
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;
        i_abort          : in  std_logic;
        i_clear_sticky   : in  std_logic;
        i_active_version : in  std_logic_vector(15 downto 0);
        i_visible_returns : in std_logic_vector(2 downto 0);
        i_rise_mask      : in  std_logic_vector(3 downto 0);
        i_fall_mask      : in  std_logic_vector(3 downto 0);

        i_hit_valid       : in  std_logic;
        i_hit_kind        : in  std_logic_vector(1 downto 0);
        i_chip_index      : in  std_logic_vector(1 downto 0);
        i_ififo_id        : in  std_logic;
        i_channel_code    : in  std_logic_vector(1 downto 0);
        i_stop_index      : in  std_logic_vector(2 downto 0);
        i_start_number    : in  std_logic_vector(7 downto 0);
        i_slope           : in  std_logic;
        i_hit              : in  std_logic_vector(16 downto 0);
        i_faulted          : in  std_logic;
        i_timeout_cause    : in  std_logic_vector(2 downto 0);
        i_shot_context     : in  gpx_shot_context_t;
        i_chip_shot_seq    : in  std_logic_vector(15 downto 0);
        o_hit_ready        : out std_logic;

        i_cell_ready       : in  std_logic;
        o_cell_valid       : out std_logic;
        o_cell_kind        : out std_logic_vector(1 downto 0);
        o_cell_chip_index  : out std_logic_vector(1 downto 0);
        o_cell_ififo_id    : out std_logic;
        o_cell_stop_index  : out std_logic_vector(2 downto 0);
        o_cell_slope       : out std_logic;
        o_cell_hit_count   : out std_logic_vector(2 downto 0);
        o_cell_max_hits    : out std_logic_vector(2 downto 0);
        o_cell_hits        : out std_logic_vector(118 downto 0);
        o_cell_hit_dropped : out std_logic;
        o_cell_return_overflow : out std_logic;
        o_cell_error_fill  : out std_logic;
        o_cell_faulted     : out std_logic;
        o_cell_timeout_cause : out std_logic_vector(2 downto 0);
        o_cell_shot_context  : out gpx_shot_context_t;
        o_cell_chip_shot_seq : out std_logic_vector(15 downto 0);
        o_fault_pulse        : out std_logic_vector(3 downto 0);
        o_fault_sticky       : out std_logic_vector(3 downto 0)
    );
end entity lidar_gpx_cell_collector_hls_impl;

architecture rtl of lidar_gpx_cell_collector_hls_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => 200,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "1111",
        fall_capability_mask   => "1111",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    signal hit_event_c    : gpx_hit_event_t;
    signal cell_event_c   : gpx_cell_event_t;
    signal fault_pulse_c  : gpx_cell_collector_faults_t;
    signal fault_sticky_c : gpx_cell_collector_faults_t;

begin

    p_hit_input : process (all)
        variable result : gpx_hit_event_t;
    begin
        result := C_GPX_HIT_EVENT_IDLE;
        result.valid := i_hit_valid;
        result.kind := gpx_hit_event_kind_t'val(
            to_integer(unsigned(i_hit_kind)));
        result.chip_index := unsigned(i_chip_index);
        result.ififo_id := i_ififo_id;
        result.channel_code := unsigned(i_channel_code);
        result.stop_index := unsigned(i_stop_index);
        result.start_number := unsigned(i_start_number);
        result.slope := fn_gpx_slope_from_bit(i_slope);
        result.hit := unsigned(i_hit);
        result.faulted := i_faulted;
        result.timeout_cause := i_timeout_cause;
        result.shot_context := fn_unpack_shot_context(i_shot_context);
        result.chip_shot_seq := unsigned(i_chip_shot_seq);
        hit_event_c <= result;
    end process p_hit_input;

    u_collector : entity work.lidar_gpx_cell_collector_hls_adapter
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_abort             => i_abort,
            i_clear_sticky      => i_clear_sticky,
            i_active_version    => unsigned(i_active_version),
            i_max_hits_per_stop => unsigned(i_visible_returns),
            i_active_rise_mask  => i_rise_mask,
            i_active_fall_mask  => i_fall_mask,
            i_hit_event         => hit_event_c,
            o_hit_ready         => o_hit_ready,
            o_cell_event        => cell_event_c,
            i_cell_ready        => i_cell_ready,
            o_idle              => open,
            o_fault_pulse       => fault_pulse_c,
            o_fault_sticky      => fault_sticky_c
        );

    o_cell_valid <= cell_event_c.valid;
    o_cell_kind <= std_logic_vector(to_unsigned(
        gpx_cell_event_kind_t'pos(cell_event_c.kind),
        o_cell_kind'length));
    o_cell_chip_index <= std_logic_vector(cell_event_c.chip_index);
    o_cell_ififo_id <= cell_event_c.ififo_id;
    o_cell_stop_index <= std_logic_vector(cell_event_c.stop_index);
    o_cell_slope <= fn_gpx_slope_to_bit(cell_event_c.slope);
    o_cell_hit_count <= std_logic_vector(cell_event_c.hit_count);
    o_cell_max_hits <= std_logic_vector(cell_event_c.max_hits);
    o_cell_hit_dropped <= cell_event_c.hit_dropped;
    o_cell_return_overflow <= cell_event_c.return_overflow;
    o_cell_error_fill <= cell_event_c.error_fill;
    o_cell_faulted <= cell_event_c.faulted;
    o_cell_timeout_cause <= cell_event_c.timeout_cause;
    o_cell_shot_context <= fn_pack_shot_context(cell_event_c.shot_context);
    o_cell_chip_shot_seq <= std_logic_vector(cell_event_c.chip_shot_seq);

    g_hit_output : for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 generate
        constant C_LO : natural := hit_index * C_GPX_HIT_WIDTH;
    begin
        o_cell_hits(C_LO + C_GPX_HIT_WIDTH - 1 downto C_LO) <=
            std_logic_vector(cell_event_c.hits(hit_index));
    end generate g_hit_output;

    o_fault_pulse <=
        fault_pulse_c.hit_capacity_drop &
        fault_pulse_c.start_number_nonzero &
        fault_pulse_c.return_overflow &
        fault_pulse_c.context_mismatch;
    o_fault_sticky <=
        fault_sticky_c.hit_capacity_drop &
        fault_sticky_c.start_number_nonzero &
        fault_sticky_c.return_overflow &
        fault_sticky_c.context_mismatch;

end architecture rtl;
