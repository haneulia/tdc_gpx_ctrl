#include "gpx_hit_decoder_hls.hpp"

namespace lidar_v3 {
namespace h1 {

namespace {

bool edge_slope_is_enabled(
    std::uint8_t tdc_chip_index,
    bool edge_slope_is_rise,
    const decoder_configuration_t &configuration) {
    const std::uint8_t enabled_chip_mask =
        edge_slope_is_rise
            ? configuration.runtime_enabled_rise_chip_mask
            : configuration.runtime_enabled_fall_chip_mask;
    return ((enabled_chip_mask >> tdc_chip_index) & 0x1U) != 0U;
}

}  // namespace

decoder_result_axis_t decode_gpx_raw_event(
    const raw_event_axis_t &raw_event,
    const decoder_configuration_t &configuration) {
    decoder_result_axis_t decoder_result = 0;
    decoded_hit_event_t decoded_hit_event = 0;

    const std::uint8_t event_kind =
        read_field<raw_event_layout::event_kind>(raw_event).to_uint();
    const std::uint8_t tdc_chip_index =
        read_field<raw_event_layout::tdc_chip_index>(raw_event).to_uint();
    const bool ififo_bank_select =
        read_flag<raw_event_layout::ififo_bank_select>(raw_event);

    write_field<decoded_hit_event_layout::event_kind>(
        decoded_hit_event, event_kind);
    write_field<decoded_hit_event_layout::tdc_chip_index>(
        decoded_hit_event, tdc_chip_index);
    write_flag<decoded_hit_event_layout::ififo_bank_select>(
        decoded_hit_event, ififo_bank_select);
    write_flag<decoded_hit_event_layout::upstream_event_faulted>(
        decoded_hit_event,
        read_flag<raw_event_layout::upstream_event_faulted>(raw_event));
    write_field<decoded_hit_event_layout::timeout_cause_bitmap>(
        decoded_hit_event,
        read_field<raw_event_layout::timeout_cause_bitmap>(raw_event));
    write_field<decoded_hit_event_layout::shot_context>(
        decoded_hit_event,
        read_field<raw_event_layout::shot_context>(raw_event));
    write_field<decoded_hit_event_layout::tdc_chip_shot_sequence>(
        decoded_hit_event,
        read_field<raw_event_layout::tdc_chip_shot_sequence>(raw_event));

    bool contains_hit_event = false;
    bool tdc_chip_index_fault = false;
    bool stop_channel_index_fault = false;
    bool edge_slope_assignment_fault = false;

    if (tdc_chip_index >= configuration.build_tdc_chip_count ||
        tdc_chip_index >= limits::kMaximumTdcGpxChipCount) {
        tdc_chip_index_fault = true;
    } else if (event_kind ==
               static_cast<std::uint8_t>(raw_event_kind_t::data)) {
        const ap_uint<kTdcGpxImodeWordBits> tdc_gpx_imode_word =
            read_field<raw_event_layout::tdc_gpx_imode_word>(raw_event);
        const std::uint8_t channel_index_within_ififo =
            read_field<tdc_gpx_imode_word_layout::channel_index_within_ififo>(
                tdc_gpx_imode_word)
                .to_uint();
        const std::uint8_t logical_stop_channel_index =
            static_cast<std::uint8_t>(
                channel_index_within_ififo + (ififo_bank_select ? 4U : 0U));
        const bool edge_slope_is_rise =
            read_flag<tdc_gpx_imode_word_layout::edge_slope_is_rise>(
                tdc_gpx_imode_word);

        if (logical_stop_channel_index >=
                configuration.build_stop_channels_per_chip ||
            logical_stop_channel_index >=
                limits::kMaximumStopChannelsPerChip) {
            stop_channel_index_fault = true;
        } else if (!edge_slope_is_enabled(
                       tdc_chip_index,
                       edge_slope_is_rise,
                       configuration)) {
            edge_slope_assignment_fault = true;
        } else {
            contains_hit_event = true;
            write_field<decoded_hit_event_layout::tdc_gpx_channel_index>(
                decoded_hit_event, channel_index_within_ififo);
            write_field<decoded_hit_event_layout::logical_stop_channel_index>(
                decoded_hit_event, logical_stop_channel_index);
            write_field<decoded_hit_event_layout::tdc_start_number>(
                decoded_hit_event,
                read_field<tdc_gpx_imode_word_layout::start_number>(
                    tdc_gpx_imode_word));
            write_flag<decoded_hit_event_layout::edge_slope_is_rise>(
                decoded_hit_event, edge_slope_is_rise);
            write_field<decoded_hit_event_layout::distance_hit_17bit>(
                decoded_hit_event,
                read_field<tdc_gpx_imode_word_layout::distance_hit_17bit>(
                    tdc_gpx_imode_word));
        }
    } else {
        contains_hit_event = true;
    }

    write_field<decoder_result_layout::decoded_hit_event>(
        decoder_result, decoded_hit_event);
    write_flag<decoder_result_layout::contains_hit_event>(
        decoder_result, contains_hit_event);
    write_flag<decoder_result_layout::tdc_chip_index_fault>(
        decoder_result, tdc_chip_index_fault);
    write_flag<decoder_result_layout::stop_channel_index_fault>(
        decoder_result, stop_channel_index_fault);
    write_flag<decoder_result_layout::edge_slope_assignment_fault>(
        decoder_result, edge_slope_assignment_fault);
    return decoder_result;
}

}  // namespace h1
}  // namespace lidar_v3

extern "C" void gpx_hit_decoder_hls(
    hls::stream<lidar_v3::h1::raw_event_axis_t> &raw_event_in,
    hls::stream<lidar_v3::h1::decoder_result_axis_t> &decoder_result_out,
    std::uint8_t build_tdc_chip_count,
    std::uint8_t build_stop_channels_per_chip,
    std::uint8_t runtime_enabled_rise_chip_mask,
    std::uint8_t runtime_enabled_fall_chip_mask) {
#pragma HLS INTERFACE axis port=raw_event_in
#pragma HLS INTERFACE axis port=decoder_result_out
#pragma HLS INTERFACE ap_none port=build_tdc_chip_count
#pragma HLS INTERFACE ap_none port=build_stop_channels_per_chip
#pragma HLS INTERFACE ap_none port=runtime_enabled_rise_chip_mask
#pragma HLS INTERFACE ap_none port=runtime_enabled_fall_chip_mask
#pragma HLS INTERFACE ap_ctrl_none port=return
    // TDC-GPX events can have long idle gaps. A flushable II=1 pipeline
    // retires the last accepted event without waiting for a following input.
#pragma HLS PIPELINE II=1 style=flp

    const lidar_v3::h1::decoder_configuration_t configuration = {
        build_tdc_chip_count,
        build_stop_channels_per_chip,
        runtime_enabled_rise_chip_mask,
        runtime_enabled_fall_chip_mask};
    const lidar_v3::h1::raw_event_axis_t raw_event = raw_event_in.read();
    decoder_result_out.write(
        lidar_v3::h1::decode_gpx_raw_event(raw_event, configuration));
}
