#include "gpx_hit_decoder_hls.hpp"

namespace lidar_v3 {

namespace {

bool slope_is_supported(
    std::uint8_t chip,
    std::uint8_t slope,
    const decoder_config_t &config) {
    const std::uint8_t mask = slope == static_cast<std::uint8_t>(slope_t::rise)
                                  ? config.rise_mask
                                  : config.fall_mask;
    return ((mask >> chip) & 0x1U) != 0U;
}

}  // namespace

decode_result_payload_t decode_gpx_raw_event(
    const raw_payload_t &raw,
    const decoder_config_t &config) {
    decode_result_payload_t result = 0;
    hit_payload_t hit = 0;

    const std::uint8_t kind = raw.range(kRawKindHi, kRawKindLo).to_uint();
    const std::uint8_t chip = raw.range(kRawChipHi, kRawChipLo).to_uint();
    const std::uint8_t ififo = raw[kRawIfifo] ? 1U : 0U;

    hit.range(kHitKindHi, kHitKindLo) = kind;
    hit.range(kHitChipHi, kHitChipLo) = chip;
    hit[kHitIfifo] = ififo;
    hit[kHitFaulted] = raw[kRawFaulted];
    hit.range(kHitTimeoutHi, kHitTimeoutLo) =
        raw.range(kRawTimeoutHi, kRawTimeoutLo);
    hit.range(kHitShotContextHi, kHitShotContextLo) =
        raw.range(kRawShotContextHi, kRawShotContextLo);
    hit.range(kHitChipShotSeqHi, kHitChipShotSeqLo) =
        raw.range(kRawChipShotSeqHi, kRawChipShotSeqLo);

    bool emit = false;
    bool chip_fault = false;
    bool stop_fault = false;
    bool slope_fault = false;

    if (chip >= config.num_chips || chip >= kMaxChips) {
        chip_fault = true;
    } else if (kind == static_cast<std::uint8_t>(raw_kind_t::data)) {
        const std::uint32_t word = raw_imode_word(raw);
        const std::uint8_t channel =
            static_cast<std::uint8_t>((word >> kImodeChannelLo) & 0x3U);
        const std::uint8_t stop =
            static_cast<std::uint8_t>(channel + (ififo != 0U ? 4U : 0U));
        const std::uint8_t slope =
            static_cast<std::uint8_t>((word >> kImodeSlope) & 0x1U);

        if (stop >= config.stops_per_chip || stop >= kMaxStopsPerChip) {
            stop_fault = true;
        } else if (!slope_is_supported(chip, slope, config)) {
            slope_fault = true;
        } else {
            emit = true;
            hit.range(kHitChannelHi, kHitChannelLo) = channel;
            hit.range(kHitStopHi, kHitStopLo) = stop;
            hit.range(kHitStartHi, kHitStartLo) =
                (word >> kImodeStartLo) & 0xFFU;
            hit[kHitSlope] = slope;
            hit.range(kHitValueHi, kHitValueLo) = word & 0x1FFFFU;
        }
    } else {
        emit = true;
    }

    result.range(kResultHitHi, kResultHitLo) = hit;
    result[kResultEmit] = emit;
    result[kResultChipFault] = chip_fault;
    result[kResultStopFault] = stop_fault;
    result[kResultSlopeFault] = slope_fault;
    return result;
}

}  // namespace lidar_v3

extern "C" void gpx_hit_decoder_hls(
    hls::stream<lidar_v3::raw_payload_t> &raw_in,
    hls::stream<lidar_v3::decode_result_payload_t> &result_out,
    std::uint8_t num_chips,
    std::uint8_t stops_per_chip,
    std::uint8_t rise_mask,
    std::uint8_t fall_mask) {
#pragma HLS INTERFACE axis port=raw_in
#pragma HLS INTERFACE axis port=result_out
#pragma HLS INTERFACE ap_none port=num_chips
#pragma HLS INTERFACE ap_none port=stops_per_chip
#pragma HLS INTERFACE ap_none port=rise_mask
#pragma HLS INTERFACE ap_none port=fall_mask
#pragma HLS INTERFACE ap_ctrl_none port=return
    // GPX events may arrive one at a time with long idle gaps. A flushable
    // pipeline must retire the last accepted event without waiting for a
    // following input while still sustaining one event per clock (II=1).
#pragma HLS PIPELINE II=1 style=flp

    const lidar_v3::decoder_config_t config = {
        num_chips, stops_per_chip, rise_mask, fall_mask};
    const lidar_v3::raw_payload_t raw = raw_in.read();
    result_out.write(lidar_v3::decode_gpx_raw_event(raw, config));
}
