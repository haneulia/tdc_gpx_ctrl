library ieee;
use ieee.std_logic_1164.all;

-- V3에서 새로 생긴 HLS 데이터 경로 진단 ABI만 소유한다.
-- 기존 V2 lidar_status_pkg의 0x00~0x26, 0x40~0xFF 계약은 변경하지 않는다.
package lidar_v3_status_pkg is

    -- CTL23 indexed 진단 요청으로 선택하고 CTL24에서 읽는다.
    -- Rise/Fall H4 formatter가 같은 fault 의미를 공유하므로 각 8-bit bitmap을
    -- 한 32-bit Word 안에 나란히 배치한다.
    constant C_DIAG_V3_GPX_FORMATTER_FAULTS : natural := 16#27#;
    constant C_DIAG_V3_FORMATTER_RISE_LO    : natural := 0;
    constant C_DIAG_V3_FORMATTER_RISE_HI    : natural := 7;
    constant C_DIAG_V3_FORMATTER_FALL_LO    : natural := 8;
    constant C_DIAG_V3_FORMATTER_FALL_HI    : natural := 15;

    -- 기존 C_DIAG_GPX_PROC_FAULTS(0x1A)의 비어 있던 Bit에 H4 요약을 추가한다.
    constant C_DIAG_V3_GPX_RISE_FORMATTER_ANY_BIT : natural := 19;
    constant C_DIAG_V3_GPX_FALL_FORMATTER_ANY_BIT : natural := 20;

end package lidar_v3_status_pkg;
