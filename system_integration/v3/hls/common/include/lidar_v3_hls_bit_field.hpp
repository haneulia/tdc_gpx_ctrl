#ifndef LIDAR_V3_HLS_BIT_FIELD_HPP
#define LIDAR_V3_HLS_BIT_FIELD_HPP

#include <ap_int.h>

namespace lidar_v3 {

// packed ABI의 한 필드를 컴파일 시점에 정의한다. end는 마지막 bit 다음 위치를
// 뜻하므로 다음 필드는 이전_field::end에서 시작한다. 숫자 중복과 경계 오타를
// 줄이고 static_assert로 payload 범위를 검사하기 위한 H0 공통 도구다.
template <unsigned Low, unsigned Width>
struct bit_field_t {
    static_assert(Width > 0U, "A packed ABI field must contain at least one bit");

    static constexpr unsigned low = Low;
    static constexpr unsigned width = Width;
    static constexpr unsigned high = Low + Width - 1U;
    static constexpr unsigned end = Low + Width;
};

template <typename Field, int PayloadBits>
inline ap_uint<Field::width> read_field(const ap_uint<PayloadBits> &payload) {
    static_assert(Field::high < PayloadBits,
                  "Packed ABI read exceeds the payload width");
    return payload.range(Field::high, Field::low);
}

template <typename Field, int PayloadBits, typename Value>
inline void write_field(ap_uint<PayloadBits> &payload, const Value &value) {
    static_assert(Field::high < PayloadBits,
                  "Packed ABI write exceeds the payload width");
    payload.range(Field::high, Field::low) = value;
}

template <typename Field, int PayloadBits>
inline bool read_flag(const ap_uint<PayloadBits> &payload) {
    static_assert(Field::width == 1U, "read_flag requires a one-bit field");
    static_assert(Field::high < PayloadBits,
                  "Packed ABI flag read exceeds the payload width");
    return payload[Field::low] != 0;
}

template <typename Field, int PayloadBits>
inline void write_flag(ap_uint<PayloadBits> &payload, bool value) {
    static_assert(Field::width == 1U, "write_flag requires a one-bit field");
    static_assert(Field::high < PayloadBits,
                  "Packed ABI flag write exceeds the payload width");
    payload[Field::low] = value ? 1U : 0U;
}

}  // namespace lidar_v3

#endif
