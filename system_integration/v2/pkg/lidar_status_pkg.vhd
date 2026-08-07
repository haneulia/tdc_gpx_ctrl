library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- K0-8 runtime diagnostic ABI. Configuration/status registers keep their
-- existing fixed 32/32 layout; CTL23/24 form one indexed read-only portal for
-- observations that originate in the Processing and TDC clock domains.
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

    -- Existing IRQ source bits 0..4 remain configuration/AXI events. K0-8
    -- appends five stable runtime aggregates without moving the four IRQ
    -- registers or changing their W1C/level-high behavior.
    constant C_RUNTIME_IRQ_COUNT              : positive := 5;
    subtype lidar_runtime_irq_t is std_logic_vector(
        C_RUNTIME_IRQ_COUNT - 1 downto 0);
    constant C_RUNTIME_IRQ_PROCESSING_WARNING : natural := 0;
    constant C_RUNTIME_IRQ_LASER_TIMEOUT      : natural := 1;
    constant C_RUNTIME_IRQ_ECHO_DIAGNOSTIC    : natural := 2;
    constant C_RUNTIME_IRQ_GPX_TRANSPORT      : natural := 3;
    constant C_RUNTIME_IRQ_GPX_DATA           : natural := 4;
    constant C_RUNTIME_IRQ_CLEAR : lidar_runtime_irq_t :=
        (others => '0');

    function fn_diag_is_processing(index : lidar_diag_index_t)
        return boolean;
    function fn_diag_is_tdc(index : lidar_diag_index_t)
        return boolean;
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
        return value >= C_DIAG_TDC_FIRST and value <= C_DIAG_TDC_LAST;
    end function fn_diag_is_tdc;

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
