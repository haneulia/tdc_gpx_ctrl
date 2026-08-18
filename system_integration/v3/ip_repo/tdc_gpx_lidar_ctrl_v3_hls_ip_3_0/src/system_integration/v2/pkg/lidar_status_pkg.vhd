library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_gpx_pkg.all;

-- v2 통합 상태/진단 ABI의 단일 정의점이다.
-- 고정 32 CTL / 32 STAT 주소를 늘리지 않고 CTL23/24를 indexed read-only
-- portal로 사용한다. 이 파일의 INDEX와 bit 상수는 RTL과 PS software가
-- 함께 따라야 하는 ABI이므로 숫자 literal을 각 status source에 복제하지 않는다.
package lidar_status_pkg is

    constant C_DIAG_INDEX_WIDTH : positive := 8;
    subtype lidar_diag_index_t is std_logic_vector(
        C_DIAG_INDEX_WIDTH - 1 downto 0);
    subtype lidar_diag_word_t is std_logic_vector(31 downto 0);

    constant C_DIAG_RESPONSE_WIDTH : positive := 33;
    subtype lidar_diag_response_t is std_logic_vector(
        C_DIAG_RESPONSE_WIDTH - 1 downto 0);
    constant C_DIAG_RESPONSE_ERROR_BIT : natural := 32;
    constant C_DIAG_RESPONSE_DATA_HI   : natural := 31;
    constant C_DIAG_RESPONSE_DATA_LO   : natural := 0;

    -- Processing-domain words.
    constant C_DIAG_PROC_FIRST             : natural := 16#10#;
    constant C_DIAG_PROC_FLAGS             : natural := 16#10#;
    constant C_DIAG_PROC_INVALID_COUNT     : natural := 16#11#;
    constant C_DIAG_PROC_FACE_OVERLAP_COUNT: natural := 16#12#;
    constant C_DIAG_PROC_OVERRUN_COUNT     : natural := 16#13#;
    constant C_DIAG_PROC_MON_DROP_COUNT    : natural := 16#14#;
    constant C_DIAG_LASER_REQ_DROP_COUNT   : natural := 16#15#;
    constant C_DIAG_LASER_TIMEOUT_COUNT    : natural := 16#16#;
    constant C_DIAG_LASER_ABORT_COUNT      : natural := 16#17#;
    constant C_DIAG_LASER_UNEXPECTED_COUNT : natural := 16#18#;
    constant C_DIAG_PROC_LATENCY_CONTRACT  : natural := 16#19#;
    constant C_DIAG_GPX_PROC_FAULTS        : natural := 16#1A#;
    constant C_DIAG_PROC_PROFILE_STATE     : natural := 16#1B#;
    constant C_DIAG_RISE_GEOMETRY          : natural := 16#1C#;
    constant C_DIAG_RISE_STRIDE            : natural := 16#1D#;
    constant C_DIAG_FALL_GEOMETRY          : natural := 16#1E#;
    constant C_DIAG_FALL_STRIDE            : natural := 16#1F#;

    constant C_DIAG_ECHO_FLAGS             : natural := 16#20#;
    constant C_DIAG_ECHO_OUTSIDE_COUNT     : natural := 16#21#;
    constant C_DIAG_ECHO_OVERLAP_COUNT     : natural := 16#22#;
    constant C_DIAG_ECHO_NOT_READY_COUNT   : natural := 16#23#;
    constant C_DIAG_ECHO_TOTALS            : natural := 16#24#;
    constant C_DIAG_ECHO_RISE_MASK         : natural := 16#25#;
    constant C_DIAG_ECHO_FALL_MASK         : natural := 16#26#;
    constant C_DIAG_ECHO_CHANNEL_FIRST     : natural := 16#40#;
    constant C_DIAG_ECHO_CHANNEL_LAST      : natural := 16#5F#;
    constant C_DIAG_PROC_LAST              : natural :=
        C_DIAG_ECHO_CHANNEL_LAST;

    -- TDC-domain words. Four lane words remain allocated even in smaller
    -- builds; absent lanes return zero and cannot assert an interrupt.
    constant C_DIAG_TDC_FIRST          : natural := 16#80#;
    constant C_DIAG_TDC_SUMMARY        : natural := 16#80#;
    constant C_DIAG_TDC_LANE_STATUS_0  : natural := 16#84#;
    constant C_DIAG_TDC_LANE_STATUS_3  : natural := 16#87#;
    constant C_DIAG_TDC_LANE_FAULT_0   : natural := 16#88#;
    constant C_DIAG_TDC_LANE_FAULT_3   : natural := 16#8B#;
    constant C_DIAG_TDC_LAST           : natural :=
        C_DIAG_TDC_LANE_FAULT_3;

    -- 외부 TDC-GPX 실제 Register 읽기 인덱스: 11CCAAAA.
    -- CC는 합성된 Chip 번호(0..3), AAAA는 GPX Register 주소(0..15)다.
    -- CTL23으로 이 값을 CAPTURE하고 CTL24에서
    -- {요청 주소[3:0], 실제 읽은 값[27:0]}을 원자적으로 읽는다.
    constant C_DIAG_GPX_REGISTER_FIRST : natural := 16#C0#;
    constant C_DIAG_GPX_REGISTER_LAST  : natural := 16#FF#;
    constant C_DIAG_GPX_REGISTER_CHIP_MSB : natural := 5;
    constant C_DIAG_GPX_REGISTER_CHIP_LSB : natural := 4;
    constant C_DIAG_GPX_REGISTER_ADDR_MSB : natural := 3;
    constant C_DIAG_GPX_REGISTER_ADDR_LSB : natural := 0;

    -- CTL24 물리 GPX read 결과. 상위 nibble은 요청 Register 주소를 다시
    -- 돌려주므로 PS가 비동기 완료값이 자신이 요청한 주소인지 검증할 수 있다.
    constant C_GPX_REGISTER_RESULT_ADDR_MSB : natural := 31;
    constant C_GPX_REGISTER_RESULT_ADDR_LSB : natural := 28;
    constant C_GPX_REGISTER_RESULT_DATA_MSB : natural := 27;
    constant C_GPX_REGISTER_RESULT_DATA_LSB : natural := 0;

    -- TDC_SUMMARY(0x80) bit 계약. bit 15는 물리 Register service timeout,
    -- 응답 Chip/주소 불일치 또는 bus 응답 오류를 CLEAR_STATUS까지 보존한다.
    constant C_TDC_SUMMARY_ACTIVE_MASK_LSB : natural := 0;
    constant C_TDC_SUMMARY_ACTIVE_MASK_MSB : natural := 3;
    constant C_TDC_SUMMARY_TERMINAL_MASK_LSB : natural := 4;
    constant C_TDC_SUMMARY_TERMINAL_MASK_MSB : natural := 7;
    constant C_TDC_SUMMARY_SAFE_BIT : natural := 10;
    constant C_TDC_SUMMARY_RUN_ENABLE_BIT : natural := 12;
    constant C_TDC_SUMMARY_ACTIVE_VALID_BIT : natural := 13;
    constant C_TDC_SUMMARY_CONFIG_READY_BIT : natural := 14;
    constant C_TDC_SUMMARY_REGISTER_READ_ERROR_BIT : natural := 15;

    -- DISARM 후 GPX acquisition을 잠시 멈추고 물리 Register를 읽는 전체
    -- 서비스의 절대 상한. 정상 Register bus watchdog보다 충분히 길지만,
    -- 배선/응답 고장으로 CTL23.BUSY가 영구 고착되지는 않게 한다.
    constant C_GPX_REGISTER_SERVICE_TIMEOUT_NS : positive := 10_000_000;

    -- Existing IRQ source bits 0..4 remain configuration/AXI events. K0-8
    -- appends five stable runtime aggregates without moving the four IRQ
    -- registers or changing their W1C/level-high behavior.
    constant C_RUNTIME_IRQ_COUNT              : positive := 5;
    subtype lidar_runtime_irq_t is std_logic_vector(
        C_RUNTIME_IRQ_COUNT - 1 downto 0);
    -- Runtime IRQ는 모두 원인 sticky의 level-high 요약이다. CLEAR_STATUS로
    -- 원인을 먼저 지운 뒤 IRQ pending을 W1C로 지워야 재발 여부가 보인다.
    -- PROCESSING_WARNING: 위치/Face/Shot 일정/monitor/laser lifecycle 계약 위반.
    -- LASER_TIMEOUT: fire_pulse 뒤 FIRE_DONE_TIMEOUT 안에 fire_done 미수신.
    -- ECHO_DIAGNOSTIC: Echo window/profile/snapshot 계측 계약 위반.
    -- GPX_TRANSPORT: CDC drop, GPX bus/drain/controller 및 물리 read 서비스 오류.
    -- GPX_DATA: Raw28 해석, Hit17, Cell, Face 조립의 의미/순서 계약 위반.
    constant C_RUNTIME_IRQ_PROCESSING_WARNING : natural := 0;
    constant C_RUNTIME_IRQ_LASER_TIMEOUT      : natural := 1;
    constant C_RUNTIME_IRQ_ECHO_DIAGNOSTIC    : natural := 2;
    constant C_RUNTIME_IRQ_GPX_TRANSPORT      : natural := 3;
    constant C_RUNTIME_IRQ_GPX_DATA           : natural := 4;
    constant C_RUNTIME_IRQ_CLEAR : lidar_runtime_irq_t :=
        (others => '0');

    -- 진단 index가 어느 clock-domain owner에 속하는지 분류한다.
    function fn_diag_is_processing(index : lidar_diag_index_t)
        return boolean;
    function fn_diag_is_tdc(index : lidar_diag_index_t)
        return boolean;
    -- 11CCAAAA 물리 read index에서 Chip과 Register 주소를 검증·추출한다.
    function fn_diag_is_gpx_register_read(index : lidar_diag_index_t)
        return boolean;
    function fn_diag_gpx_register_chip(index : lidar_diag_index_t)
        return gpx_chip_select_t;
    function fn_diag_gpx_register_address(index : lidar_diag_index_t)
        return gpx_bus_address_t;
    -- CTL24의 단일 32-bit 물리 read 결과와 mailbox 응답을 조립한다.
    function fn_pack_gpx_register_read_word(
        address_value : gpx_bus_address_t;
        data_value    : gpx_bus_data_t
    ) return lidar_diag_word_t;
    function fn_pack_diag_response(
        data_value  : lidar_diag_word_t;
        error_value : std_logic
    ) return lidar_diag_response_t;

end package lidar_status_pkg;

package body lidar_status_pkg is

    function fn_diag_is_processing(index : lidar_diag_index_t)
        return boolean is
        variable value : natural;
    begin
        value := to_integer(unsigned(index));
        return value >= C_DIAG_PROC_FIRST and value <= C_DIAG_PROC_LAST;
    end function fn_diag_is_processing;

    function fn_diag_is_tdc(index : lidar_diag_index_t)
        return boolean is
        variable value : natural;
    begin
        value := to_integer(unsigned(index));
        return (value >= C_DIAG_TDC_FIRST and value <= C_DIAG_TDC_LAST) or
            (value >= C_DIAG_GPX_REGISTER_FIRST and
             value <= C_DIAG_GPX_REGISTER_LAST);
    end function fn_diag_is_tdc;

    function fn_diag_is_gpx_register_read(index : lidar_diag_index_t)
        return boolean is
        variable value : natural;
    begin
        value := to_integer(unsigned(index));
        return value >= C_DIAG_GPX_REGISTER_FIRST and
            value <= C_DIAG_GPX_REGISTER_LAST;
    end function fn_diag_is_gpx_register_read;

    function fn_diag_gpx_register_chip(index : lidar_diag_index_t)
        return gpx_chip_select_t is
    begin
        return unsigned(index(
            C_DIAG_GPX_REGISTER_CHIP_MSB downto
            C_DIAG_GPX_REGISTER_CHIP_LSB));
    end function fn_diag_gpx_register_chip;

    function fn_diag_gpx_register_address(index : lidar_diag_index_t)
        return gpx_bus_address_t is
    begin
        return index(
            C_DIAG_GPX_REGISTER_ADDR_MSB downto
            C_DIAG_GPX_REGISTER_ADDR_LSB);
    end function fn_diag_gpx_register_address;

    function fn_pack_gpx_register_read_word(
        address_value : gpx_bus_address_t;
        data_value    : gpx_bus_data_t
    ) return lidar_diag_word_t is
        variable result : lidar_diag_word_t := (others => '0');
    begin
        result(C_GPX_REGISTER_RESULT_ADDR_MSB downto
               C_GPX_REGISTER_RESULT_ADDR_LSB) := address_value;
        result(C_GPX_REGISTER_RESULT_DATA_MSB downto
               C_GPX_REGISTER_RESULT_DATA_LSB) := data_value;
        return result;
    end function fn_pack_gpx_register_read_word;

    function fn_pack_diag_response(
        data_value  : lidar_diag_word_t;
        error_value : std_logic
    ) return lidar_diag_response_t is
        variable result : lidar_diag_response_t := (others => '0');
    begin
        result(C_DIAG_RESPONSE_DATA_HI downto
               C_DIAG_RESPONSE_DATA_LO) := data_value;
        result(C_DIAG_RESPONSE_ERROR_BIT) := error_value;
        return result;
    end function fn_pack_diag_response;

end package body lidar_status_pkg;
