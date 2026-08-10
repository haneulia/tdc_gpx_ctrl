#ifndef GPX_CELL_COLLECTOR_HLS_HPP
#define GPX_CELL_COLLECTOR_HLS_HPP

#include <hls_stream.h>

#include "lidar_v3_hls_contract.hpp"

// The RTL adapter keeps ap_start asserted so the ap_ctrl_hs top runs
// continuously with an explicit boundary for each accepted Hit event.
// Every input Hit carries reset_epoch in its AXI4-Stream sideband. Checking
// the epoch only after the Hit handshake prevents a short abort from being
// missed while the core is blocked on either input or output backpressure.
extern "C" void gpx_cell_collector_hls(
    hls::stream<lidar_v3::cell_hit_input_t> &hit_in,
    hls::stream<lidar_v3::collector_result_payload_t> &result_out,
    std::uint8_t num_chips,
    std::uint8_t stops_per_chip,
    std::uint8_t max_returns_per_stop,
    std::uint8_t rise_mask,
    std::uint8_t fall_mask,
    std::uint8_t visible_returns,
    std::uint16_t active_version);

#endif
