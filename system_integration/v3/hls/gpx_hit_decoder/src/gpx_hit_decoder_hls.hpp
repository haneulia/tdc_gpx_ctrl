#ifndef GPX_HIT_DECODER_HLS_HPP
#define GPX_HIT_DECODER_HLS_HPP

#include <cstdint>
#include <hls_stream.h>

#include "lidar_v3_h1_raw_hit_contract.hpp"

namespace lidar_v3 {
namespace h1 {

// Decode one accepted external TDC-GPX I-Mode Raw Event. The function keeps
// event identity and Shot context even when a topology fault suppresses the
// decoded Hit Event.
decoder_result_axis_t decode_gpx_raw_event(
    const raw_event_axis_t &raw_event,
    const decoder_configuration_t &configuration);

}  // namespace h1
}  // namespace lidar_v3

// One accepted Raw Event produces one decoder result. Chip/STOP counts are
// build-time structure snapshots; rise/fall masks are active Runtime topology
// snapshots and must stay stable while an event is accepted.
extern "C" void gpx_hit_decoder_hls(
    hls::stream<lidar_v3::h1::raw_event_axis_t> &raw_event_in,
    hls::stream<lidar_v3::h1::decoder_result_axis_t> &decoder_result_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask);

#endif
