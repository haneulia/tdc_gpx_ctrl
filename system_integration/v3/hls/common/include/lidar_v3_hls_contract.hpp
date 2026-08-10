#ifndef LIDAR_V3_HLS_CONTRACT_HPP
#define LIDAR_V3_HLS_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

namespace lidar_v3 {

constexpr unsigned kMaxChips = 4;
constexpr unsigned kMaxStopsPerChip = 8;

// AXI4-Stream TDATA is byte-aligned. The semantic Raw payload uses bits
// [214:0]; bit 215 is reserved and must be driven to zero by the RTL adapter.
constexpr unsigned kRawSemanticBits = 215;
constexpr unsigned kRawPayloadBits = 216;
constexpr unsigned kHitPayloadBits = 218;
// The semantic decoder result uses bits [221:0]. Bits [223:222] are reserved
// so the HLS and RTL sides share an explicit 28-byte AXI4-Stream contract.
constexpr unsigned kDecodeResultSemanticBits = 222;
constexpr unsigned kDecodeResultBits = 224;
constexpr unsigned kShotContextBits = 162;

static_assert((kRawPayloadBits % 8U) == 0U, "Raw AXIS width must be byte aligned");
static_assert((kDecodeResultBits % 8U) == 0U,
              "Result AXIS width must be byte aligned");

using raw_payload_t = ap_uint<kRawPayloadBits>;
using hit_payload_t = ap_uint<kHitPayloadBits>;
using decode_result_payload_t = ap_uint<kDecodeResultBits>;

enum class raw_kind_t : std::uint8_t {
    data = 0,
    ififo1_done = 1,
    drain_done = 2,
    timeout = 3
};

enum class slope_t : std::uint8_t {
    fall = 0,
    rise = 1
};

// Raw payload layout. AXI TVALID owns event validity and is not in TDATA.
constexpr unsigned kRawKindLo = 0;
constexpr unsigned kRawKindHi = 1;
constexpr unsigned kRawChipLo = 2;
constexpr unsigned kRawChipHi = 3;
constexpr unsigned kRawIfifo = 4;
constexpr unsigned kRawWordLo = 5;
constexpr unsigned kRawWordHi = 32;
constexpr unsigned kRawFaulted = 33;
constexpr unsigned kRawTimeoutLo = 34;
constexpr unsigned kRawTimeoutHi = 36;
constexpr unsigned kRawShotContextLo = 37;
constexpr unsigned kRawShotContextHi = 198;
constexpr unsigned kRawChipShotSeqLo = 199;
constexpr unsigned kRawChipShotSeqHi = 214;
constexpr unsigned kRawReserved = 215;

// External GPX I-Mode 28-bit word layout.
constexpr unsigned kImodeHitLo = 0;
constexpr unsigned kImodeHitHi = 16;
constexpr unsigned kImodeSlope = 17;
constexpr unsigned kImodeStartLo = 18;
constexpr unsigned kImodeStartHi = 25;
constexpr unsigned kImodeChannelLo = 26;
constexpr unsigned kImodeChannelHi = 27;

// Hit payload layout. The payload is meaningful only when result.emit is one.
constexpr unsigned kHitKindLo = 0;
constexpr unsigned kHitKindHi = 1;
constexpr unsigned kHitChipLo = 2;
constexpr unsigned kHitChipHi = 3;
constexpr unsigned kHitIfifo = 4;
constexpr unsigned kHitChannelLo = 5;
constexpr unsigned kHitChannelHi = 6;
constexpr unsigned kHitStopLo = 7;
constexpr unsigned kHitStopHi = 9;
constexpr unsigned kHitStartLo = 10;
constexpr unsigned kHitStartHi = 17;
constexpr unsigned kHitSlope = 18;
constexpr unsigned kHitValueLo = 19;
constexpr unsigned kHitValueHi = 35;
constexpr unsigned kHitFaulted = 36;
constexpr unsigned kHitTimeoutLo = 37;
constexpr unsigned kHitTimeoutHi = 39;
constexpr unsigned kHitShotContextLo = 40;
constexpr unsigned kHitShotContextHi = 201;
constexpr unsigned kHitChipShotSeqLo = 202;
constexpr unsigned kHitChipShotSeqHi = 217;

// Decoder result layout. One result is produced for every accepted raw event.
constexpr unsigned kResultHitLo = 0;
constexpr unsigned kResultHitHi = 217;
constexpr unsigned kResultEmit = 218;
constexpr unsigned kResultChipFault = 219;
constexpr unsigned kResultStopFault = 220;
constexpr unsigned kResultSlopeFault = 221;
constexpr unsigned kResultReservedLo = 222;
constexpr unsigned kResultReservedHi = 223;

struct decoder_config_t {
    std::uint8_t num_chips;
    std::uint8_t stops_per_chip;
    std::uint8_t rise_mask;
    std::uint8_t fall_mask;
};

inline std::uint32_t bit_mask(unsigned width) {
    return width >= 32U ? 0xFFFFFFFFU : ((1U << width) - 1U);
}

inline std::uint32_t raw_imode_word(const raw_payload_t &raw) {
    return raw.range(kRawWordHi, kRawWordLo).to_uint();
}

inline hit_payload_t result_hit(const decode_result_payload_t &result) {
    return result.range(kResultHitHi, kResultHitLo);
}

inline bool result_emits_hit(const decode_result_payload_t &result) {
    return result[kResultEmit] != 0;
}

}  // namespace lidar_v3

#endif
