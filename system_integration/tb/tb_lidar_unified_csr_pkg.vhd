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

        assert C_SYS_CTRL_RESET_EPOCH_HI - C_SYS_CTRL_RESET_EPOCH_LO + 1 = 8
            and C_SYS_CFG_EPOCH_HI - C_SYS_CFG_EPOCH_LO + 1 = 8
            report "system transaction epochs must remain 8 bits"
            severity failure;

        assert C_MOTOR_CFG_CPR_HI < C_MOTOR_CFG_DIR_BIT
            and C_MOTOR_CFG_DIR_BIT < C_MOTOR_CFG_DEC_MODE_LO
            and C_MOTOR_CFG_DEC_MODE_HI < C_MOTOR_CFG_Z_EARLY_BIT
            and C_MOTOR_CFG_Z_EARLY_BIT < C_MOTOR_CFG_VALID_HOLD_LO
            and C_MOTOR_CFG_VALID_HOLD_HI < 32
            report "MOTOR_CFG fields overlap or exceed one word"
            severity failure;

        assert C_MOTOR_SCHED_HI_COUNT_HI < C_MOTOR_SCHED_PHYS_LAT_LO
            and C_MOTOR_SCHED_PHYS_LAT_HI < C_MOTOR_SCHED_VIRT_LAT_LO
            and C_MOTOR_SCHED_VIRT_LAT_HI < 32
            report "MOTOR_SCHED_LATENCY fields overlap or exceed one word"
            severity failure;

        assert C_MOTOR_Z_OFFSET_HI < C_MOTOR_Z_WIDTH_LO
            and C_MOTOR_Z_WIDTH_HI < 32
            and C_MOTOR_FACE_CENTER_HI < C_MOTOR_FACE_HALF_LO
            and C_MOTOR_FACE_HALF_HI < C_MOTOR_FACE_VALID_BIT
            and C_MOTOR_FACE_VALID_BIT < 32
            report "Motor 15-bit pair packing changed"
            severity failure;

        assert C_MOTOR_FACE_WRITE_INDEX_HI < C_MOTOR_FACE_READ_INDEX_LO
            and C_MOTOR_FACE_READ_INDEX_HI < C_MOTOR_FACE_WRITE_EPOCH_LO
            and C_MOTOR_FACE_WRITE_EPOCH_HI < 32
            report "MOTOR_FACE_INDEX fields overlap"
            severity failure;

        assert C_MOTOR_CFG_STATUS_CFG_EPOCH_HI
                   < C_MOTOR_CFG_STATUS_FACE_EPOCH_LO
            and C_MOTOR_CFG_STATUS_FACE_EPOCH_HI
                   < C_MOTOR_CFG_STATUS_READ_INDEX_LO
            and C_MOTOR_CFG_STATUS_READ_INDEX_HI
                   < C_MOTOR_CFG_STATUS_GEOM_VALID_BIT
            and C_MOTOR_CFG_STATUS_VALID_BIT
                   < C_MOTOR_CFG_STATUS_RESET_EPOCH_LO
            and C_MOTOR_CFG_STATUS_RESET_EPOCH_HI = 31
            report "MOTOR_CFG_STATUS fields overlap or leave the word"
            severity failure;

        assert C_ECHO_DELAY_INDEX_HI < C_ECHO_DELAY_WRITE_TOGGLE_BIT
            and C_ECHO_DELAY_WRITE_TOGGLE_BIT
                   < C_ECHO_DELAY_APPLY_TOGGLE_BIT
            and C_ECHO_DELAY_DATA_HI < 32
            report "Echo indexed delay command fields changed"
            severity failure;

        assert C_IRQ_MOTOR_ACTIVE_ENTER = C_IRQ_MOTOR_FIRST
            and C_IRQ_MOTOR_DIAGNOSTIC = C_IRQ_MOTOR_LAST
            and C_IRQ_LASER_BLOCKING_CFG = C_IRQ_LASER_FIRST
            and C_IRQ_LASER_FRAME_OVERFLOW <= C_IRQ_LASER_LAST
            report "Stage 3 interrupt ownership changed"
            severity failure;

        report "UNIFIED_CSR_CONTRACT_PASS" severity note;
        wait;
    end process p_contract;
end architecture sim;
