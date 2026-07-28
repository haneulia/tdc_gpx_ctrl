library ieee;
use ieee.std_logic_1164.all;

use work.lidar_unified_csr_pkg.all;

entity tb_lidar_unified_csr_pkg is
end entity tb_lidar_unified_csr_pkg;

architecture sim of tb_lidar_unified_csr_pkg is
begin
    p_contract : process
    begin
        assert C_UCSR_CTL_COUNT = 32 and C_UCSR_STAT_COUNT = 32
            report "unified CSR must retain 32 CTL and 32 STAT slots"
            severity failure;
        assert C_UCSR_INTR_COUNT = 4 and C_UCSR_INTR_SOURCE_COUNT = 32
            report "unified CSR interrupt geometry changed"
            severity failure;

        assert fn_ctl_byte_offset(0) = 16#000#
            and fn_ctl_byte_offset(31) = 16#07C#
            and fn_stat_byte_offset(0) = 16#080#
            and fn_stat_byte_offset(31) = 16#0FC#
            and fn_intr_byte_offset(0) = 16#100#
            and fn_intr_byte_offset(3) = C_UCSR_LAST_ADDR
            report "unified CSR byte-address geometry changed"
            severity failure;

        assert C_CTL_SYS_CTRL = 0
            and C_CTL_MOTOR_CFG = 2
            and C_CTL_LASER_FIRE_CFG = 8
            and C_CTL_ECHO_DELAY_CMD = 15
            and C_CTL_TDC_BUS_TIMING = 17
            and C_CTL_TDC_AUX_CMD + 1 = C_CTL_RESERVED_FIRST
            report "unified CTL ownership ranges overlap or contain a gap"
            severity failure;

        assert C_STAT_SYS_VERSION = 0
            and C_STAT_MOTOR_STATUS = 6
            and C_STAT_LASER_STATUS = 12
            and C_STAT_ECHO_RISE_MASK = 19
            and C_STAT_TDC_CHIP0_RESULT = 23
            and C_STAT_TDC_PIPELINE_STATUS = 27
            and C_STAT_TDC_STATUS_EXT2 + 1 = C_STAT_RESERVED_FIRST
            report "unified STAT ownership ranges overlap or contain a gap"
            severity failure;

        assert C_IRQ_SYS_FIRST = 0
            and C_IRQ_SYS_LAST + 1 = C_IRQ_MOTOR_FIRST
            and C_IRQ_MOTOR_LAST + 1 = C_IRQ_LASER_FIRST
            and C_IRQ_LASER_LAST + 1 = C_IRQ_ECHO_FIRST
            and C_IRQ_ECHO_LAST + 1 = C_IRQ_TDC_FIRST
            and C_IRQ_TDC_LAST + 1 = C_IRQ_RESERVED_FIRST
            and C_IRQ_RESERVED_LAST = C_UCSR_INTR_SOURCE_COUNT - 1
            report "unified interrupt-source ownership ranges overlap"
            severity failure;

        assert C_UCSR_ACTIVE_WORD_COUNT = 60
            report "unified active-register count changed"
            severity failure;

        report "UNIFIED_CSR_CONTRACT_PASS" severity note;
        wait;
    end process p_contract;
end architecture sim;
