#ifndef GPX_FRAME_ASSEMBLER_HLS_HPP
#define GPX_FRAME_ASSEMBLER_HLS_HPP

#include <cstdint>
#include <hls_stream.h>

#include "lidar_v3_h3_frame_contract.hpp"

// H3 converts width-independent H2 Cell Events into independently ordered
// Rise and Fall Lane Cells. The RTL adapter supplies one event per ap_ctrl_hs
// invocation and decouples the two output lanes with dedicated FIFOs.
extern "C" void gpx_frame_assembler_hls(
    hls::stream<lidar_v3::h3::assembler_input_axis_t>
        &cell_or_face_close_event_in,
    hls::stream<lidar_v3::h3::ordered_lane_cell_axis_t>
        &ordered_rise_cell_out,
    hls::stream<lidar_v3::h3::ordered_lane_cell_axis_t>
        &ordered_fall_cell_out,
    hls::stream<lidar_v3::h3::assembler_control_axis_t>
        &assembler_control_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t build_mirror_face_count,
    std::uint16_t active_configuration_version,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask,
    std::uint16_t runtime_expected_shot_columns_per_face);

#endif
