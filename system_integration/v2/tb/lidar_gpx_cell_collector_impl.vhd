-- ============================================================================
-- 테스트 자산 목적: B7 Cell collector의 RAM/ready/context 경로를 구현 점검한다.
-- 핵심 검증 계약: Return 저장 구조가 목표 clock과 자원 예산을 만족하고 latch가 없다.
-- 관련 RTL: lidar_gpx_cell_collector.
-- 실행 회귀: scripts/run_v2_gpx_cell_collector.ps1
-- 유지보수 주의: Return/fault 기능은 tb_lidar_gpx_cell_collector가 소유한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity lidar_gpx_cell_collector_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;
        i_active_version : in std_logic_vector(15 downto 0);
        i_max_hits       : in std_logic_vector(2 downto 0);

        i_hit_valid      : in  std_logic;
        i_hit_kind       : in  std_logic_vector(1 downto 0);
        i_chip_index     : in  std_logic_vector(1 downto 0);
        i_ififo_id       : in  std_logic;
        i_channel_code   : in  std_logic_vector(1 downto 0);
        i_stop_index     : in  std_logic_vector(2 downto 0);
        i_start_number   : in  std_logic_vector(7 downto 0);
        i_slope          : in  std_logic;
        i_hit            : in  std_logic_vector(16 downto 0);
        i_faulted        : in  std_logic;
        i_timeout_cause  : in  std_logic_vector(2 downto 0);
        i_shot_context   : in  gpx_shot_context_t;
        i_chip_shot_seq  : in  std_logic_vector(15 downto 0);
        o_hit_ready      : out std_logic;

        i_cell_ready     : in  std_logic;
        o_cell_valid     : out std_logic;
        o_cell_kind      : out std_logic_vector(1 downto 0);
        o_cell_chip      : out std_logic_vector(1 downto 0);
        o_cell_ififo     : out std_logic;
        o_cell_stop      : out std_logic_vector(2 downto 0);
        o_cell_slope     : out std_logic;
        o_cell_hit_count : out std_logic_vector(2 downto 0);
        o_cell_max_hits  : out std_logic_vector(2 downto 0);
        o_cell_hits      : out std_logic_vector(
            C_MAX_RETURNS_PER_STOP * C_GPX_HIT_WIDTH - 1 downto 0);
        o_cell_dropped   : out std_logic;
        o_cell_error_fill : out std_logic;
        o_cell_faulted   : out std_logic;
        o_cell_timeout_cause : out std_logic_vector(2 downto 0);
        o_cell_shot_context : out gpx_shot_context_t;
        o_cell_chip_seq  : out std_logic_vector(15 downto 0);
        o_fault_pulse    : out std_logic_vector(3 downto 0);
        o_fault_sticky   : out std_logic_vector(3 downto 0)
    );
end entity lidar_gpx_cell_collector_impl;

architecture rtl of lidar_gpx_cell_collector_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => 200,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
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
        result.valid         := i_hit_valid;
        result.kind          := gpx_hit_event_kind_t'val(
            to_integer(unsigned(i_hit_kind)));
        result.chip_index    := unsigned(i_chip_index);
        result.ififo_id      := i_ififo_id;
        result.channel_code  := unsigned(i_channel_code);
        result.stop_index    := unsigned(i_stop_index);
        result.start_number  := unsigned(i_start_number);
        result.slope         := fn_gpx_slope_from_bit(i_slope);
        result.hit           := unsigned(i_hit);
        result.faulted       := i_faulted;
        result.timeout_cause := i_timeout_cause;
        result.shot_context  := fn_unpack_shot_context(i_shot_context);
        result.chip_shot_seq := unsigned(i_chip_shot_seq);
        hit_event_c <= result;
    end process p_hit_input;

    u_collector : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_abort             => i_abort,
            i_clear_sticky      => i_clear_sticky,
            i_active_version    => unsigned(i_active_version),
            i_max_hits_per_stop => unsigned(i_max_hits),
            i_hit_event         => hit_event_c,
            o_hit_ready         => o_hit_ready,
            o_cell_event        => cell_event_c,
            i_cell_ready        => i_cell_ready,
            o_fault_pulse       => fault_pulse_c,
            o_fault_sticky      => fault_sticky_c
        );

    o_cell_valid     <= cell_event_c.valid;
    o_cell_kind      <= std_logic_vector(to_unsigned(
        gpx_cell_event_kind_t'pos(cell_event_c.kind), o_cell_kind'length));
    o_cell_chip      <= std_logic_vector(cell_event_c.chip_index);
    o_cell_ififo     <= cell_event_c.ififo_id;
    o_cell_stop      <= std_logic_vector(cell_event_c.stop_index);
    o_cell_slope     <= fn_gpx_slope_to_bit(cell_event_c.slope);
    o_cell_hit_count <= std_logic_vector(cell_event_c.hit_count);
    o_cell_max_hits  <= std_logic_vector(cell_event_c.max_hits);
    o_cell_dropped   <= cell_event_c.hit_dropped;
    o_cell_error_fill <= cell_event_c.error_fill;
    o_cell_faulted   <= cell_event_c.faulted;
    o_cell_timeout_cause <= cell_event_c.timeout_cause;
    o_cell_shot_context <= fn_pack_shot_context(cell_event_c.shot_context);
    o_cell_chip_seq  <= std_logic_vector(cell_event_c.chip_shot_seq);

    gen_hit_pack : for hit_index in 0 to C_MAX_RETURNS_PER_STOP - 1 generate
        constant C_LO : natural := hit_index * C_GPX_HIT_WIDTH;
        constant C_HI : natural := C_LO + C_GPX_HIT_WIDTH - 1;
    begin
        o_cell_hits(C_HI downto C_LO) <=
            std_logic_vector(cell_event_c.hits(hit_index));
    end generate gen_hit_pack;

    o_fault_pulse <= fault_pulse_c.hit_capacity_drop &
                     fault_pulse_c.start_number_nonzero &
                     fault_pulse_c.return_overflow &
                     fault_pulse_c.context_mismatch;
    o_fault_sticky <= fault_sticky_c.hit_capacity_drop &
                      fault_sticky_c.start_number_nonzero &
                      fault_sticky_c.return_overflow &
                      fault_sticky_c.context_mismatch;

end architecture rtl;
