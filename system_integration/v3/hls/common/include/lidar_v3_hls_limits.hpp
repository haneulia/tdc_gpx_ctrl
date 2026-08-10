#ifndef LIDAR_V3_HLS_LIMITS_HPP
#define LIDAR_V3_HLS_LIMITS_HPP

#include <cstdint>

namespace lidar_v3 {

// H0-H4 소스 계약 버전이다. AXI payload에 자동 삽입되는 값이 아니므로,
// Release Manifest와 문서가 이 버전을 함께 기록해야 한다.
constexpr std::uint16_t kHlsContractAbiMajor = 3U;
constexpr std::uint16_t kHlsContractAbiMinor = 2U;

namespace limits {

// H0-H4 HLS에 합성되는 최대 구조 용량이다. 제품 Generic과 Runtime Active
// snapshot은 이보다 적게 활성화할 수 있지만 이 상한을 넘을 수 없다.
constexpr unsigned kMaximumTdcGpxChipCount = 4U;
constexpr unsigned kMaximumStopChannelsPerChip = 8U;
constexpr unsigned kMaximumReturnCountPerStop = 7U;
constexpr unsigned kMaximumMirrorFaceCount = 5U;

}  // namespace limits

// H1 decoded Hit, H2 Cell, H3 ordered Lane Cell, H4 Word가 공유하는 edge 의미다.
enum class tdc_edge_slope_t : std::uint8_t {
    fall = 0,
    rise = 1
};

}  // namespace lidar_v3

#endif
