#ifndef LIDAR_V3_HLS_CONTRACT_HPP
#define LIDAR_V3_HLS_CONTRACT_HPP

#include <ap_int.h>
#include <cstdint>

namespace lidar_v3 {

constexpr unsigned kMaxChips = 4;
constexpr unsigned kMaxStopsPerChip = 8;
constexpr unsigned kMaxReturnsPerStop = 7;

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

// H2 carries the abort generation beside each accepted Hit. The generation
// is checked after the stream handshake, so an abort cannot be missed while
// the HLS core is waiting for input. Bits [223:218] remain reserved-zero and
// bits [231:224] carry reset_epoch.
constexpr unsigned kCellHitInputBits = 232;
using cell_hit_input_t = ap_uint<kCellHitInputBits>;
constexpr unsigned kCellInputReservedLo = 218;
constexpr unsigned kCellInputReservedHi = 223;
constexpr unsigned kCellInputResetEpochLo = 224;
constexpr unsigned kCellInputResetEpochHi = 231;

// One Cell is one Shot x one Chip x one STOP x one slope. The Cell payload
// carries all seven physical Return slots; hit_count states how many leading
// slots are visible for the active Runtime Return setting.
constexpr unsigned kCellPayloadBits = 319;
constexpr unsigned kCollectorResultSemanticBits = 324;
constexpr unsigned kCollectorResultBits = 328;
using cell_payload_t = ap_uint<kCellPayloadBits>;
using collector_result_payload_t = ap_uint<kCollectorResultBits>;

static_assert((kCellHitInputBits % 8U) == 0U,
              "Cell input AXIS width must be byte aligned");
static_assert((kCollectorResultBits % 8U) == 0U,
              "Cell result AXIS width must be byte aligned");

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

enum class cell_kind_t : std::uint8_t {
    data = 0,
    ififo1_done = 1,
    drain_done = 2,
    timeout = 3
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

// Fields inside the opaque 162-bit Shot context. These positions are owned by
// lidar_gpx_event_pkg.fn_pack_shot_context in V2 and are repeated here only as
// an explicit RTL/HLS ABI contract.
constexpr unsigned kContextFaceLo = 1;
constexpr unsigned kContextFaceHi = 3;
constexpr unsigned kContextShotIndexLo = 20;
constexpr unsigned kContextShotIndexHi = 35;
constexpr unsigned kContextSourceSim = 37;
constexpr unsigned kContextActiveVersionLo = 47;
constexpr unsigned kContextActiveVersionHi = 62;

// Width-independent Cell payload layout. AXI TVALID owns event validity.
constexpr unsigned kCellKindLo = 0;
constexpr unsigned kCellKindHi = 1;
constexpr unsigned kCellChipLo = 2;
constexpr unsigned kCellChipHi = 3;
constexpr unsigned kCellIfifo = 4;
constexpr unsigned kCellStopLo = 5;
constexpr unsigned kCellStopHi = 7;
constexpr unsigned kCellSlope = 8;
constexpr unsigned kCellHitCountLo = 9;
constexpr unsigned kCellHitCountHi = 11;
constexpr unsigned kCellMaxHitsLo = 12;
constexpr unsigned kCellMaxHitsHi = 14;
constexpr unsigned kCellHitsLo = 15;
constexpr unsigned kCellHitsHi = 133;
constexpr unsigned kCellHitDropped = 134;
constexpr unsigned kCellReturnOverflow = 135;
constexpr unsigned kCellErrorFill = 136;
constexpr unsigned kCellFaulted = 137;
constexpr unsigned kCellTimeoutLo = 138;
constexpr unsigned kCellTimeoutHi = 140;
constexpr unsigned kCellShotContextLo = 141;
constexpr unsigned kCellShotContextHi = 302;
constexpr unsigned kCellChipShotSeqLo = 303;
constexpr unsigned kCellChipShotSeqHi = 318;

// One collector result is emitted for every accepted Hit input. The first
// result is a non-Cell status acknowledgement; a terminal input may then emit
// several Cell results followed by one Cell control event.
constexpr unsigned kCollectorCellLo = 0;
constexpr unsigned kCollectorCellHi = 318;
constexpr unsigned kCollectorEmit = 319;
constexpr unsigned kCollectorContextFault = 320;
constexpr unsigned kCollectorReturnOverflowFault = 321;
constexpr unsigned kCollectorStartNonzeroFault = 322;
constexpr unsigned kCollectorCapacityDropFault = 323;
constexpr unsigned kCollectorReservedLo = 324;
constexpr unsigned kCollectorReservedHi = 327;

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

inline hit_payload_t cell_input_hit(const cell_hit_input_t &input) {
    return input.range(kHitPayloadBits - 1U, 0U);
}

inline cell_payload_t collector_result_cell(
    const collector_result_payload_t &result) {
    return result.range(kCollectorCellHi, kCollectorCellLo);
}

inline bool collector_result_emits_cell(
    const collector_result_payload_t &result) {
    return result[kCollectorEmit] != 0;
}

}  // namespace lidar_v3

#endif
