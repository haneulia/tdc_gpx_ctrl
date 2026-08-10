#ifndef LIDAR_V3_HLS_CONTRACT_HPP
#define LIDAR_V3_HLS_CONTRACT_HPP

// H0 공통 계약의 안정된 진입점이다. H1~H4 전체 Bit ABI가 필요한 통합 코드와
// 테스트는 이 파일 하나만 include한다. 한 단계만 소유하는 구현은 의존 범위를
// 분명히 하기 위해 해당 stage contract를 직접 include한다.
#include "lidar_v3_hls_bit_field.hpp"
#include "lidar_v3_hls_limits.hpp"
#include "lidar_v3_h1_raw_hit_contract.hpp"
#include "lidar_v3_h2_cell_contract.hpp"
#include "lidar_v3_h3_frame_contract.hpp"
#include "lidar_v3_h4_word_contract.hpp"

#endif
