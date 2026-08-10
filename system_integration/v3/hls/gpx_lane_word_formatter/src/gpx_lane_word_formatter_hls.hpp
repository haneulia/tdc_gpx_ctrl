#ifndef GPX_LANE_WORD_FORMATTER_HLS_HPP
#define GPX_LANE_WORD_FORMATTER_HLS_HPP

#include <hls_stream.h>

#include "lidar_v3_h4_word_contract.hpp"

// H4 converts one ordered Rise/Fall Lane into width-independent canonical
// 32-bit Words. The retained RTL stage after H4 is the sole owner of packing
// these Words into synthesized 32-bit or 64-bit AXI4-Stream Beats.
extern "C" void gpx_lane_word_formatter_hls(
    hls::stream<lidar_v3::h4::formatter_input_axis_t>
        &ordered_cell_or_face_close_in,
    hls::stream<lidar_v3::h4::canonical_line_word_axis_t>
        &canonical_line_word_out,
    hls::stream<lidar_v3::h4::formatter_control_axis_t>
        &formatter_control_out,
    lidar_v3::h4::lane_profile_t active_lane_profile);

#endif
