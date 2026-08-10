#ifndef GPX_HIT_DECODER_HLS_HPP
#define GPX_HIT_DECODER_HLS_HPP

#include <hls_stream.h>

#include "lidar_v3_hls_contract.hpp"

namespace lidar_v3 {

decode_result_payload_t decode_gpx_raw_event(
    const raw_payload_t &raw,
    const decoder_config_t &config);

}  // namespace lidar_v3

// Keep the synthesizable top at global C linkage. Vitis HLS resolves this
// symbol by name, while all implementation types remain namespace-scoped.
extern "C" void gpx_hit_decoder_hls(
    hls::stream<lidar_v3::raw_payload_t> &raw_in,
    hls::stream<lidar_v3::decode_result_payload_t> &result_out,
    std::uint8_t num_chips,
    std::uint8_t stops_per_chip,
    std::uint8_t rise_mask,
    std::uint8_t fall_mask);

#endif
