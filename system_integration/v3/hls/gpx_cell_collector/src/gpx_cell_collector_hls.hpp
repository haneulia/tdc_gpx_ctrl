#ifndef GPX_CELL_COLLECTOR_HLS_HPP
#define GPX_CELL_COLLECTOR_HLS_HPP

#include <cstdint>
#include <hls_stream.h>

#include "lidar_v3_h2_cell_contract.hpp"

// The RTL adapter keeps ap_start asserted, so one accepted decoded Hit Event
// and all results belonging to it form one ap_ctrl_hs invocation. reset_epoch
// travels in the accepted AXI payload and cannot be missed during backpressure.
extern "C" void gpx_cell_collector_hls(
    hls::stream<lidar_v3::h2::collector_input_axis_t> &decoded_hit_event_in,
    hls::stream<lidar_v3::h2::collector_result_axis_t> &collector_result_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t build_max_return_count_per_stop,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask,
    std::uint8_t runtime_visible_return_count,
    std::uint16_t active_configuration_version);

#endif
