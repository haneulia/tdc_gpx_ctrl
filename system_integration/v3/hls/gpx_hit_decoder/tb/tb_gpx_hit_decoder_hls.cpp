#include <cassert>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>

#include "gpx_hit_decoder_hls.hpp"

// 검증 목적:
//   네 가지 Chip/STOP/Rise/Fall 구성에서 Raw28의 모든 조합을 독립 Oracle과
//   비교한다. 이 테스트는 HLS C simulation과 C/RTL co-simulation이 공유한다.

namespace {

using namespace lidar_v3;

struct expected_result_t {
    bool emit;
    bool chip_fault;
    bool stop_fault;
    bool slope_fault;
    std::uint8_t kind;
    std::uint8_t chip;
    std::uint8_t ififo;
    std::uint8_t channel;
    std::uint8_t stop;
    std::uint8_t start;
    std::uint8_t slope;
    std::uint32_t hit;
    std::uint8_t faulted;
    std::uint8_t timeout;
    std::uint16_t sequence;
};

raw_payload_t make_raw(
    raw_kind_t kind,
    std::uint8_t chip,
    std::uint8_t ififo,
    std::uint8_t channel,
    std::uint8_t start,
    slope_t slope,
    std::uint32_t hit,
    std::uint16_t sequence,
    std::uint8_t faulted = 0,
    std::uint8_t timeout = 0) {
    raw_payload_t raw = 0;
    std::uint32_t word = 0;
    word |= (static_cast<std::uint32_t>(channel) & 0x3U) << 26;
    word |= static_cast<std::uint32_t>(start) << 18;
    word |= (static_cast<std::uint32_t>(slope) & 0x1U) << 17;
    word |= hit & 0x1FFFFU;

    raw.range(kRawKindHi, kRawKindLo) = static_cast<std::uint8_t>(kind);
    raw.range(kRawChipHi, kRawChipLo) = chip;
    raw[kRawIfifo] = ififo != 0U;
    raw.range(kRawWordHi, kRawWordLo) = word;
    raw[kRawFaulted] = faulted != 0U;
    raw.range(kRawTimeoutHi, kRawTimeoutLo) = timeout & 0x7U;

    // Use a nontrivial opaque context pattern and require exact pass-through.
    ap_uint<kShotContextBits> context = 0;
    for (unsigned bit = 0; bit < kShotContextBits; ++bit) {
        context[bit] = ((bit * 5U + sequence) % 11U) < 5U;
    }
    raw.range(kRawShotContextHi, kRawShotContextLo) = context;
    raw.range(kRawChipShotSeqHi, kRawChipShotSeqLo) = sequence;
    return raw;
}

expected_result_t oracle(
    const raw_payload_t &raw,
    const decoder_config_t &config) {
    expected_result_t expected{};
    expected.kind = raw.range(kRawKindHi, kRawKindLo).to_uint();
    expected.chip = raw.range(kRawChipHi, kRawChipLo).to_uint();
    expected.ififo = raw[kRawIfifo] ? 1U : 0U;
    expected.faulted = raw[kRawFaulted] ? 1U : 0U;
    expected.timeout = raw.range(kRawTimeoutHi, kRawTimeoutLo).to_uint();
    expected.sequence =
        raw.range(kRawChipShotSeqHi, kRawChipShotSeqLo).to_uint();

    if (expected.chip >= config.num_chips || expected.chip >= kMaxChips) {
        expected.chip_fault = true;
        return expected;
    }
    if (expected.kind != static_cast<std::uint8_t>(raw_kind_t::data)) {
        expected.emit = true;
        return expected;
    }

    const std::uint32_t word = raw.range(kRawWordHi, kRawWordLo).to_uint();
    expected.channel = (word >> 26) & 0x3U;
    expected.stop = expected.channel + (expected.ififo != 0U ? 4U : 0U);
    expected.start = (word >> 18) & 0xFFU;
    expected.slope = (word >> 17) & 0x1U;
    expected.hit = word & 0x1FFFFU;

    if (expected.stop >= config.stops_per_chip ||
        expected.stop >= kMaxStopsPerChip) {
        expected.stop_fault = true;
        return expected;
    }
    const std::uint8_t mask = expected.slope != 0U
                                  ? config.rise_mask
                                  : config.fall_mask;
    if (((mask >> expected.chip) & 0x1U) == 0U) {
        expected.slope_fault = true;
        return expected;
    }
    expected.emit = true;
    return expected;
}

void check_one(
    const raw_payload_t &raw,
    const decoder_config_t &config) {
    hls::stream<raw_payload_t> input;
    hls::stream<decode_result_payload_t> output;
    input.write(raw);
    gpx_hit_decoder_hls(
        input,
        output,
        config.num_chips,
        config.stops_per_chip,
        config.rise_mask,
        config.fall_mask);
    assert(!output.empty());

    const decode_result_payload_t result = output.read();
    const hit_payload_t hit_payload = result_hit(result);
    const expected_result_t expected = oracle(raw, config);

    assert(result_emits_hit(result) == expected.emit);
    assert((result[kResultChipFault] != 0) == expected.chip_fault);
    assert((result[kResultStopFault] != 0) == expected.stop_fault);
    assert((result[kResultSlopeFault] != 0) == expected.slope_fault);
    assert(result.range(kResultReservedHi, kResultReservedLo) == 0U);
    assert(hit_payload.range(kHitKindHi, kHitKindLo).to_uint() == expected.kind);
    assert(hit_payload.range(kHitChipHi, kHitChipLo).to_uint() == expected.chip);
    assert((hit_payload[kHitIfifo] ? 1U : 0U) == expected.ififo);
    assert((hit_payload[kHitFaulted] ? 1U : 0U) == expected.faulted);
    assert(hit_payload.range(kHitTimeoutHi, kHitTimeoutLo).to_uint() ==
           expected.timeout);
    assert(hit_payload.range(kHitChipShotSeqHi, kHitChipShotSeqLo).to_uint() ==
           expected.sequence);
    assert(hit_payload.range(kHitShotContextHi, kHitShotContextLo) ==
           raw.range(kRawShotContextHi, kRawShotContextLo));

    if (expected.emit &&
        expected.kind == static_cast<std::uint8_t>(raw_kind_t::data)) {
        assert(hit_payload.range(kHitChannelHi, kHitChannelLo).to_uint() ==
               expected.channel);
        assert(hit_payload.range(kHitStopHi, kHitStopLo).to_uint() ==
               expected.stop);
        assert(hit_payload.range(kHitStartHi, kHitStartLo).to_uint() ==
               expected.start);
        assert((hit_payload[kHitSlope] ? 1U : 0U) == expected.slope);
        assert(hit_payload.range(kHitValueHi, kHitValueLo).to_uint() ==
               expected.hit);
    }
}

void run_profile(const decoder_config_t &config) {
    std::uint16_t sequence = 1;
    for (std::uint8_t kind = 0; kind < 4; ++kind) {
        for (std::uint8_t chip = 0; chip < 4; ++chip) {
            for (std::uint8_t ififo = 0; ififo < 2; ++ififo) {
                for (std::uint8_t channel = 0; channel < 4; ++channel) {
                    for (std::uint8_t slope = 0; slope < 2; ++slope) {
                        const std::uint32_t hit =
                            ((sequence * 977U) ^ 0x10001U) & 0x1FFFFU;
                        check_one(
                            make_raw(
                                static_cast<raw_kind_t>(kind),
                                chip,
                                ififo,
                                channel,
                                static_cast<std::uint8_t>(sequence),
                                static_cast<slope_t>(slope),
                                hit,
                                sequence,
                                (sequence & 1U) != 0U,
                                sequence & 0x7U),
                            config);
                        ++sequence;
                    }
                }
            }
        }
    }
}

}  // namespace

int main() {
    const decoder_config_t dedicated{4, 8, 0x3, 0xC};
    const decoder_config_t one_chip_dual{1, 8, 0x1, 0x1};
    const decoder_config_t reduced{3, 6, 0x3, 0x4};
    const decoder_config_t all_dual{4, 8, 0xF, 0xF};

    // C simulation runs all profiles. C/RTL co-simulation sets one fixed
    // profile per invocation because ap_ctrl_none scalar configuration is a
    // stable Face/run contract, not a transaction sideband.
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

    // Explicitly preserve the full 17-bit maximum Hit value.
    if (profile.empty() || profile == "one_chip_dual") {
        check_one(
            make_raw(raw_kind_t::data, 0, 0, 2, 0x7E,
                     slope_t::rise, 0x1FFFFU, 0x55AA),
            one_chip_dual);
    }

    std::cout << "LIDAR_V3_HLS_GPX_HIT_DECODER_CSIM_PASS";
    if (!profile.empty()) {
        std::cout << " profile=" << profile;
    }
    std::cout << std::endl;
    return 0;
}
