#ifndef GPX_FRAME_ASSEMBLER_HLS_HPP
#define GPX_FRAME_ASSEMBLER_HLS_HPP

#include <cstdint>
#include <hls_stream.h>

#include "lidar_v3_hls_contract.hpp"

// H3 converts width-independent H2 Cells into independently ordered Rise and
// Fall Frame lanes. The RTL adapter supplies one event per invocation and
// decouples both output lanes with dedicated FIFOs.
extern "C" void gpx_frame_assembler_hls(
    hls::stream<lidar_v3::frame_input_payload_t> &event_in,
    hls::stream<lidar_v3::frame_cell_payload_t> &rise_out,
    hls::stream<lidar_v3::frame_cell_payload_t> &fall_out,
    hls::stream<lidar_v3::frame_control_payload_t> &control_out,
    std::uint8_t num_chips, std::uint8_t stops_per_chip, std::uint8_t num_faces,
    std::uint16_t active_version, std::uint8_t rise_mask,
    std::uint8_t fall_mask, std::uint16_t columns_per_face);

#endif
