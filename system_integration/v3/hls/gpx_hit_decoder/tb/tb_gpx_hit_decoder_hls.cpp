#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "gpx_hit_decoder_hls.hpp"

// 검증 목적:
//   1. TDC-GPX I-Mode 28-bit 워드가 17-bit 거리값, 채널, START 번호,
//      Rising/Falling slope로 정확히 분리되는지 확인한다.
//   2. Chip/STOP 수와 Runtime slope mask 위반이 각각 독립 fault가 되는지
//      확인한다.
//   3. 162-bit Shot 문맥과 Chip별 Shot 순번이 비트 단위로 보존되는지
//      확인한다. 이 Oracle은 CSim과 C/RTL CoSim이 함께 사용한다.

namespace {

namespace h1 = lidar_v3::h1;
namespace limits = lidar_v3::limits;

struct expected_result_t {
    bool contains_hit_event;
    bool tdc_chip_index_fault;
    bool stop_channel_index_fault;
    bool edge_slope_assignment_fault;
    std::uint8_t event_kind;
    std::uint8_t tdc_chip_index;
    std::uint8_t ififo_bank_select;
    std::uint8_t channel_index_within_ififo;
    std::uint8_t logical_stop_channel_index;
    std::uint8_t start_number;
    std::uint8_t edge_slope_is_rise;
    std::uint32_t distance_hit_17bit;
    std::uint8_t upstream_event_faulted;
    std::uint8_t timeout_cause_bitmap;
    std::uint16_t tdc_chip_shot_sequence;
};

h1::raw_event_axis_t make_raw_event(
    h1::raw_event_kind_t event_kind,
    std::uint8_t tdc_chip_index,
    std::uint8_t ififo_bank_select,
    std::uint8_t channel_index_within_ififo,
    std::uint8_t start_number,
    lidar_v3::tdc_edge_slope_t edge_slope,
    std::uint32_t distance_hit_17bit,
    std::uint16_t tdc_chip_shot_sequence,
    std::uint8_t upstream_event_faulted = 0U,
    std::uint8_t timeout_cause_bitmap = 0U) {
    h1::raw_event_axis_t raw_event = 0;
    ap_uint<h1::kTdcGpxImodeWordBits> imode_word = 0;

    lidar_v3::write_field<h1::tdc_gpx_imode_word_layout::distance_hit_17bit>(
        imode_word, distance_hit_17bit);
    lidar_v3::write_flag<h1::tdc_gpx_imode_word_layout::edge_slope_is_rise>(
        imode_word,
        edge_slope == lidar_v3::tdc_edge_slope_t::rise);
    lidar_v3::write_field<h1::tdc_gpx_imode_word_layout::start_number>(
        imode_word, start_number);
    lidar_v3::write_field<
        h1::tdc_gpx_imode_word_layout::channel_index_within_ififo>(
        imode_word, channel_index_within_ififo);

    lidar_v3::write_field<h1::raw_event_layout::event_kind>(
        raw_event, static_cast<std::uint8_t>(event_kind));
    lidar_v3::write_field<h1::raw_event_layout::tdc_chip_index>(
        raw_event, tdc_chip_index);
    lidar_v3::write_flag<h1::raw_event_layout::ififo_bank_select>(
        raw_event, ififo_bank_select != 0U);
    lidar_v3::write_field<h1::raw_event_layout::tdc_gpx_imode_word>(
        raw_event, imode_word);
    lidar_v3::write_flag<h1::raw_event_layout::upstream_event_faulted>(
        raw_event, upstream_event_faulted != 0U);
    lidar_v3::write_field<h1::raw_event_layout::timeout_cause_bitmap>(
        raw_event, timeout_cause_bitmap & 0x7U);

    h1::shot_context_t shot_context = 0;
    for (unsigned bit_index = 0;
         bit_index < h1::kShotContextRecordBits;
         ++bit_index) {
        shot_context[bit_index] =
            ((bit_index * 5U + tdc_chip_shot_sequence) % 11U) < 5U;
    }
    lidar_v3::write_field<h1::raw_event_layout::shot_context>(
        raw_event, shot_context);
    lidar_v3::write_field<h1::raw_event_layout::tdc_chip_shot_sequence>(
        raw_event, tdc_chip_shot_sequence);
    return raw_event;
}

expected_result_t calculate_expected_result(
    const h1::raw_event_axis_t &raw_event,
    const h1::decoder_configuration_t &configuration) {
    expected_result_t expected{};
    expected.event_kind =
        lidar_v3::read_field<h1::raw_event_layout::event_kind>(raw_event)
            .to_uint();
    expected.tdc_chip_index =
        lidar_v3::read_field<h1::raw_event_layout::tdc_chip_index>(raw_event)
            .to_uint();
    expected.ififo_bank_select =
        lidar_v3::read_flag<h1::raw_event_layout::ififo_bank_select>(raw_event)
            ? 1U
            : 0U;
    expected.upstream_event_faulted =
        lidar_v3::read_flag<h1::raw_event_layout::upstream_event_faulted>(
            raw_event)
            ? 1U
            : 0U;
    expected.timeout_cause_bitmap =
        lidar_v3::read_field<h1::raw_event_layout::timeout_cause_bitmap>(
            raw_event)
            .to_uint();
    expected.tdc_chip_shot_sequence =
        lidar_v3::read_field<h1::raw_event_layout::tdc_chip_shot_sequence>(
            raw_event)
            .to_uint();

    if (expected.tdc_chip_index >= configuration.build_tdc_chip_count ||
        expected.tdc_chip_index >= limits::kMaximumTdcGpxChipCount) {
        expected.tdc_chip_index_fault = true;
        return expected;
    }
    if (expected.event_kind !=
        static_cast<std::uint8_t>(h1::raw_event_kind_t::data)) {
        expected.contains_hit_event = true;
        return expected;
    }

    const ap_uint<h1::kTdcGpxImodeWordBits> imode_word =
        lidar_v3::read_field<h1::raw_event_layout::tdc_gpx_imode_word>(
            raw_event);
    expected.channel_index_within_ififo =
        lidar_v3::read_field<
            h1::tdc_gpx_imode_word_layout::channel_index_within_ififo>(
            imode_word)
            .to_uint();
    expected.logical_stop_channel_index =
        static_cast<std::uint8_t>(expected.channel_index_within_ififo +
                                  (expected.ififo_bank_select != 0U ? 4U : 0U));
    expected.start_number =
        lidar_v3::read_field<h1::tdc_gpx_imode_word_layout::start_number>(
            imode_word)
            .to_uint();
    expected.edge_slope_is_rise =
        lidar_v3::read_flag<h1::tdc_gpx_imode_word_layout::edge_slope_is_rise>(
            imode_word)
            ? 1U
            : 0U;
    expected.distance_hit_17bit =
        lidar_v3::read_field<h1::tdc_gpx_imode_word_layout::distance_hit_17bit>(
            imode_word)
            .to_uint();

    if (expected.logical_stop_channel_index >=
            configuration.build_stop_channels_per_chip ||
        expected.logical_stop_channel_index >=
            limits::kMaximumStopChannelsPerChip) {
        expected.stop_channel_index_fault = true;
        return expected;
    }
    const std::uint8_t enabled_chip_mask =
        expected.edge_slope_is_rise != 0U
            ? configuration.runtime_enabled_rise_chip_mask
            : configuration.runtime_enabled_fall_chip_mask;
    if (((enabled_chip_mask >> expected.tdc_chip_index) & 0x1U) == 0U) {
        expected.edge_slope_assignment_fault = true;
        return expected;
    }
    expected.contains_hit_event = true;
    return expected;
}

void check_one_event(
    const h1::raw_event_axis_t &raw_event,
    const h1::decoder_configuration_t &configuration) {
    hls::stream<h1::raw_event_axis_t> raw_event_stream;
    hls::stream<h1::decoder_result_axis_t> decoder_result_stream;
    raw_event_stream.write(raw_event);
    gpx_hit_decoder_hls(
        raw_event_stream,
        decoder_result_stream,
        configuration.build_tdc_chip_count,
        configuration.build_stop_channels_per_chip,
        configuration.runtime_enabled_rise_chip_mask,
        configuration.runtime_enabled_fall_chip_mask);
    assert(!decoder_result_stream.empty());

    const h1::decoder_result_axis_t decoder_result =
        decoder_result_stream.read();
    const h1::decoded_hit_event_t decoded_hit_event =
        h1::read_decoder_result_hit_event(decoder_result);
    const expected_result_t expected =
        calculate_expected_result(raw_event, configuration);

    assert(h1::decoder_result_contains_hit_event(decoder_result) ==
           expected.contains_hit_event);
    assert(lidar_v3::read_flag<h1::decoder_result_layout::tdc_chip_index_fault>(
               decoder_result) == expected.tdc_chip_index_fault);
    assert(lidar_v3::read_flag<
               h1::decoder_result_layout::stop_channel_index_fault>(
               decoder_result) == expected.stop_channel_index_fault);
    assert(lidar_v3::read_flag<
               h1::decoder_result_layout::edge_slope_assignment_fault>(
               decoder_result) == expected.edge_slope_assignment_fault);
    assert(lidar_v3::read_field<h1::decoder_result_layout::reserved_zero>(
               decoder_result) == 0U);
    assert(lidar_v3::read_field<h1::decoded_hit_event_layout::event_kind>(
               decoded_hit_event) == expected.event_kind);
    assert(lidar_v3::read_field<h1::decoded_hit_event_layout::tdc_chip_index>(
               decoded_hit_event) == expected.tdc_chip_index);
    assert(lidar_v3::read_flag<
               h1::decoded_hit_event_layout::ififo_bank_select>(
               decoded_hit_event) == (expected.ififo_bank_select != 0U));
    assert(lidar_v3::read_flag<
               h1::decoded_hit_event_layout::upstream_event_faulted>(
               decoded_hit_event) ==
           (expected.upstream_event_faulted != 0U));
    assert(lidar_v3::read_field<
               h1::decoded_hit_event_layout::timeout_cause_bitmap>(
               decoded_hit_event) == expected.timeout_cause_bitmap);
    assert(lidar_v3::read_field<
               h1::decoded_hit_event_layout::tdc_chip_shot_sequence>(
               decoded_hit_event) == expected.tdc_chip_shot_sequence);
    assert(lidar_v3::read_field<h1::decoded_hit_event_layout::shot_context>(
               decoded_hit_event) ==
           lidar_v3::read_field<h1::raw_event_layout::shot_context>(raw_event));

    if (expected.contains_hit_event &&
        expected.event_kind ==
            static_cast<std::uint8_t>(h1::raw_event_kind_t::data)) {
        assert(lidar_v3::read_field<
                   h1::decoded_hit_event_layout::tdc_gpx_channel_index>(
                   decoded_hit_event) == expected.channel_index_within_ififo);
        assert(lidar_v3::read_field<
                   h1::decoded_hit_event_layout::logical_stop_channel_index>(
                   decoded_hit_event) == expected.logical_stop_channel_index);
        assert(lidar_v3::read_field<
                   h1::decoded_hit_event_layout::tdc_start_number>(
                   decoded_hit_event) == expected.start_number);
        assert(lidar_v3::read_flag<
                   h1::decoded_hit_event_layout::edge_slope_is_rise>(
                   decoded_hit_event) ==
               (expected.edge_slope_is_rise != 0U));
        assert(lidar_v3::read_field<
                   h1::decoded_hit_event_layout::distance_hit_17bit>(
                   decoded_hit_event) == expected.distance_hit_17bit);
    }
}

void run_profile(const h1::decoder_configuration_t &configuration) {
    std::uint16_t sequence = 1U;
    for (std::uint8_t event_kind = 0U; event_kind < 4U; ++event_kind) {
        for (std::uint8_t chip_index = 0U; chip_index < 4U; ++chip_index) {
            for (std::uint8_t ififo_bank = 0U; ififo_bank < 2U; ++ififo_bank) {
                for (std::uint8_t channel = 0U; channel < 4U; ++channel) {
                    for (std::uint8_t slope = 0U; slope < 2U; ++slope) {
                        const std::uint32_t hit_value =
                            ((sequence * 977U) ^ 0x10001U) & 0x1FFFFU;
                        check_one_event(
                            make_raw_event(
                                static_cast<h1::raw_event_kind_t>(event_kind),
                                chip_index,
                                ififo_bank,
                                channel,
                                static_cast<std::uint8_t>(sequence),
                                static_cast<lidar_v3::tdc_edge_slope_t>(slope),
                                hit_value,
                                sequence,
                                (sequence & 1U) != 0U,
                                sequence & 0x7U),
                            configuration);
                        ++sequence;
                    }
                }
            }
        }
    }
}

}  // namespace

int main() {
    const h1::decoder_configuration_t dedicated{
        4U, 8U, 0x3U, 0xCU};
    const h1::decoder_configuration_t one_chip_dual{
        1U, 8U, 0x1U, 0x1U};
    const h1::decoder_configuration_t reduced{
        3U, 6U, 0x3U, 0x4U};
    const h1::decoder_configuration_t all_dual{
        4U, 8U, 0xFU, 0xFU};

    // ap_ctrl_none 설정 입력은 개별 이벤트 sideband가 아니라 한 Face/run
    // 동안 안정적으로 유지되는 설정 계약이므로 CoSim은 profile별 실행한다.
    const char *profile_value = std::getenv("V3_HLS_PROFILE");
    const std::string profile = profile_value == nullptr ? "" : profile_value;

    if (profile.empty() || profile == "dedicated") {
        run_profile(dedicated);
    }
    if (profile.empty() || profile == "one_chip_dual") {
        run_profile(one_chip_dual);
    }
    if (profile.empty() || profile == "reduced") {
        run_profile(reduced);
    }
    if (profile.empty() || profile == "all_dual") {
        run_profile(all_dual);
    }
    if (!profile.empty() &&
        profile != "dedicated" &&
        profile != "one_chip_dual" &&
        profile != "reduced" &&
        profile != "all_dual") {
        std::cerr << "Unknown V3_HLS_PROFILE: " << profile << std::endl;
        return 2;
    }

    // 물리 최대값 0x1FFFF의 상위 17번째 bit가 손실되지 않는지 별도 고정한다.
    if (profile.empty() || profile == "one_chip_dual") {
        check_one_event(
            make_raw_event(
                h1::raw_event_kind_t::data,
                0U,
                0U,
                2U,
                0x7EU,
                lidar_v3::tdc_edge_slope_t::rise,
                0x1FFFFU,
                0x55AAU),
            one_chip_dual);
    }

    std::cout << "LIDAR_V3_HLS_GPX_HIT_DECODER_CSIM_PASS";
    if (!profile.empty()) {
        std::cout << " profile=" << profile;
    }
    std::cout << std::endl;
    return 0;
}
