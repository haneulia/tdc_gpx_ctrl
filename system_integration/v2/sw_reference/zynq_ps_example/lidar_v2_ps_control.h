#ifndef LIDAR_V2_PS_CONTROL_H
#define LIDAR_V2_PS_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_V2_CSR_ABI_MAJOR 2U
#define LIDAR_V2_CSR_ABI_MINOR 7U
#define LIDAR_V2_CTL_COUNT 32U
#define LIDAR_V2_STAT_COUNT 32U
#define LIDAR_V2_RUNTIME_SHADOW_LAST 20U
#define LIDAR_V2_IRQ_SOURCE_COUNT 10U

#define LIDAR_V2_CTL_OFFSET(index) ((uint32_t)(index) * 4U)
#define LIDAR_V2_STAT_OFFSET(index) (0x080U + (uint32_t)(index) * 4U)
#define LIDAR_V2_IRQ_ENABLE_OFFSET 0x100U
#define LIDAR_V2_IRQ_STATUS_OFFSET 0x104U
#define LIDAR_V2_IRQ_FLAG_OFFSET 0x108U
#define LIDAR_V2_IRQ_MODE_OFFSET 0x10CU

enum {
    LIDAR_V2_CTL_COMMAND = 0,
    LIDAR_V2_CTL_MOTOR_PROFILE = 1,
    LIDAR_V2_CTL_VIRTUAL_TICKS_LO = 2,
    LIDAR_V2_CTL_VIRTUAL_HI_COUNT = 3,
    LIDAR_V2_CTL_Z_PROFILE = 4,
    LIDAR_V2_CTL_FACE_CENTER_0 = 5,
    LIDAR_V2_CTL_FACE_CENTER_4 = 9,
    LIDAR_V2_CTL_FACE_PROFILE = 10,
    LIDAR_V2_CTL_LASER_FIRE_PROFILE = 11,
    LIDAR_V2_CTL_TARGET_RANGE = 12,
    LIDAR_V2_CTL_TDC_PULSE_WIDTHS = 13,
    LIDAR_V2_CTL_SIM_START_DELAY = 14,
    LIDAR_V2_CTL_SHOT_INTERVAL = 15,
    LIDAR_V2_CTL_TDC_BUS_PROFILE = 16,
    LIDAR_V2_CTL_TDC_START_OFFSET = 17,
    LIDAR_V2_CTL_TDC_SCAN_TIMEOUT = 18,
    LIDAR_V2_CTL_TDC_CAPTURE_ADJUST = 19,
    LIDAR_V2_CTL_ECHO_DELAY_PROFILE = 20,
    LIDAR_V2_CTL_GPX_IMAGE_INDEX = 21,
    LIDAR_V2_CTL_GPX_IMAGE_DATA = 22,
    LIDAR_V2_CTL_DIAG_INDEX = 23,
    LIDAR_V2_CTL_DIAG_DATA = 24,
    LIDAR_V2_CTL_VDMA_PROFILE_CONTROL = 25,
    LIDAR_V2_CTL_VDMA_RISE_GEOMETRY = 26,
    LIDAR_V2_CTL_VDMA_RISE_STRIDE = 27,
    LIDAR_V2_CTL_VDMA_FALL_GEOMETRY = 28,
    LIDAR_V2_CTL_VDMA_FALL_STRIDE = 29
};

enum {
    LIDAR_V2_STAT_CORE_INFO = 0,
    LIDAR_V2_STAT_BUILD_INFO = 1,
    LIDAR_V2_STAT_TRANSACTION = 2,
    LIDAR_V2_STAT_ACTIVE_VERSION = 3,
    LIDAR_V2_STAT_ACTIVE_SOURCE_BASE = 4,
    LIDAR_V2_STAT_DERIVED_GEOMETRY = 23,
    LIDAR_V2_STAT_DERIVED_FACE = 24,
    LIDAR_V2_STAT_FACE_BOUNDS_0 = 25,
    LIDAR_V2_STAT_CAPTURE_TDC_CLKS = 30,
    LIDAR_V2_STAT_DERIVED_MASKS = 31
};

#define LIDAR_V2_CMD_COMMIT (1U << 0)
#define LIDAR_V2_CMD_CLEAR_STATUS (1U << 1)
#define LIDAR_V2_CMD_SOFT_RESET (1U << 2)
#define LIDAR_V2_CMD_RUN (1U << 3)
#define LIDAR_V2_CMD_STOP (1U << 4)
#define LIDAR_V2_CMD_ARM (1U << 5)
#define LIDAR_V2_CMD_DISARM (1U << 6)

#define LIDAR_V2_TXN_BUSY (1U << 0)
#define LIDAR_V2_TXN_DONE (1U << 1)
#define LIDAR_V2_TXN_SUCCESS (1U << 2)
#define LIDAR_V2_TXN_ERROR (1U << 3)
#define LIDAR_V2_TXN_REJECTED (1U << 4)
#define LIDAR_V2_TXN_RECOVERY_REQUIRED (1U << 5)
#define LIDAR_V2_TXN_ACTIVE_VALID (1U << 6)
#define LIDAR_V2_TXN_ACCESS_ERROR (1U << 7)
#define LIDAR_V2_TXN_SHADOW_DIRTY (1U << 8)

#define LIDAR_V2_OP_RUNNING (1U << 16)
#define LIDAR_V2_OP_ARMED (1U << 17)
#define LIDAR_V2_OP_COMMAND_READY (1U << 24)
#define LIDAR_V2_OP_COMMAND_BUSY (1U << 25)

#define LIDAR_V2_VDMA_RISE_PENDING (1U << 0)
#define LIDAR_V2_VDMA_RISE_ENABLE (1U << 1)
#define LIDAR_V2_VDMA_FALL_PENDING (1U << 2)
#define LIDAR_V2_VDMA_FALL_ENABLE (1U << 3)
#define LIDAR_V2_VDMA_RISE_ACK (1U << 8)
#define LIDAR_V2_VDMA_FALL_ACK (1U << 9)

#define LIDAR_V2_IRQ_ALL_MASK ((1U << LIDAR_V2_IRQ_SOURCE_COUNT) - 1U)
#define LIDAR_V2_IRQ_COMMIT_SUCCESS (1U << 0)
#define LIDAR_V2_IRQ_COMMIT_ERROR (1U << 1)
#define LIDAR_V2_IRQ_COMMIT_REJECTED (1U << 2)
#define LIDAR_V2_IRQ_RECOVERY_REQUIRED (1U << 3)
#define LIDAR_V2_IRQ_ACCESS_ERROR (1U << 4)
#define LIDAR_V2_IRQ_PROCESSING_WARNING (1U << 5)
#define LIDAR_V2_IRQ_LASER_TIMEOUT (1U << 6)
#define LIDAR_V2_IRQ_ECHO_DIAGNOSTIC (1U << 7)
#define LIDAR_V2_IRQ_GPX_TRANSPORT (1U << 8)
#define LIDAR_V2_IRQ_GPX_DATA (1U << 9)

typedef uint32_t (*lidar_v2_mmio_read32_t)(void *context, uintptr_t address);
typedef void (*lidar_v2_mmio_write32_t)(
    void *context,
    uintptr_t address,
    uint32_t value);
typedef void (*lidar_v2_mmio_barrier_t)(void *context);

typedef struct {
    uintptr_t csr_base;
    void *context;
    lidar_v2_mmio_read32_t read32;
    lidar_v2_mmio_write32_t write32;
    lidar_v2_mmio_barrier_t barrier;
} lidar_v2_device_t;

typedef enum {
    LIDAR_V2_CONTROL_OK = 0,
    LIDAR_V2_CONTROL_ARGUMENT,
    LIDAR_V2_CONTROL_TIMEOUT,
    LIDAR_V2_CONTROL_COMMIT_ERROR,
    LIDAR_V2_CONTROL_COMMIT_REJECTED,
    LIDAR_V2_CONTROL_RECOVERY_REQUIRED,
    LIDAR_V2_CONTROL_ACCESS_ERROR,
    LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR,
    LIDAR_V2_CONTROL_GPX_READ_ERROR,
    LIDAR_V2_CONTROL_ABI_MISMATCH
} lidar_v2_control_status_t;

typedef struct {
    uint32_t ctl[LIDAR_V2_RUNTIME_SHADOW_LAST + 1U];
} lidar_v2_shadow_config_t;

typedef struct {
    uint8_t abi_major;
    uint8_t abi_minor;
    uint8_t num_faces;
    uint8_t num_chips;
    uint8_t stops_per_chip;
    uint8_t max_returns_per_stop;
    bool echo_receiver_enabled;
    bool echo_simulation_enabled;
    bool processing_tdc_clock_sync;
    uint16_t output_width_bits;
    uint8_t processing_clock_mhz;
    uint8_t tdc_clock_mhz;
    uint8_t rise_capability_mask;
    uint8_t fall_capability_mask;
} lidar_v2_build_info_t;

typedef struct {
    bool pending;
    bool enabled;
    uint16_t hsize_bytes;
    uint16_t vsize_lines;
    uint16_t stride_bytes;
} lidar_v2_vdma_lane_profile_t;

typedef struct {
    lidar_v2_vdma_lane_profile_t rise;
    lidar_v2_vdma_lane_profile_t fall;
} lidar_v2_vdma_profiles_t;

typedef struct {
    uint32_t transaction_word;
    uint16_t active_version;
    uint8_t error_code;
    uint8_t reject_code;
} lidar_v2_commit_result_t;

typedef int (*lidar_v2_poll_service_t)(void *context);

void lidar_v2_device_init(
    lidar_v2_device_t *device,
    uintptr_t csr_base,
    void *context,
    lidar_v2_mmio_read32_t read32,
    lidar_v2_mmio_write32_t write32,
    lidar_v2_mmio_barrier_t barrier);

uint32_t lidar_v2_read_ctl(const lidar_v2_device_t *device, uint8_t index);
uint32_t lidar_v2_read_stat(const lidar_v2_device_t *device, uint8_t index);
void lidar_v2_write_ctl(
    const lidar_v2_device_t *device,
    uint8_t index,
    uint32_t value);

lidar_v2_control_status_t lidar_v2_read_build_info(
    const lidar_v2_device_t *device,
    lidar_v2_build_info_t *info);
lidar_v2_control_status_t lidar_v2_read_shadow(
    const lidar_v2_device_t *device,
    lidar_v2_shadow_config_t *shadow);
lidar_v2_control_status_t lidar_v2_write_shadow(
    const lidar_v2_device_t *device,
    const lidar_v2_shadow_config_t *shadow);

lidar_v2_control_status_t lidar_v2_clear_status(
    const lidar_v2_device_t *device,
    uint32_t poll_limit);
lidar_v2_control_status_t lidar_v2_stop_and_disarm(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context);
lidar_v2_control_status_t lidar_v2_run_and_arm(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context);
lidar_v2_control_status_t lidar_v2_commit_shadow(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context,
    lidar_v2_commit_result_t *result);

lidar_v2_control_status_t lidar_v2_read_vdma_profiles(
    const lidar_v2_device_t *device,
    lidar_v2_vdma_profiles_t *profiles);
lidar_v2_control_status_t lidar_v2_ack_vdma_profiles(
    const lidar_v2_device_t *device,
    bool acknowledge_rise,
    bool acknowledge_fall);

void lidar_v2_irq_configure(
    const lidar_v2_device_t *device,
    uint32_t enable_mask,
    uint32_t automatic_pulse_mask);
uint32_t lidar_v2_irq_pending(const lidar_v2_device_t *device);
uint32_t lidar_v2_irq_source_level(const lidar_v2_device_t *device);
void lidar_v2_irq_ack(const lidar_v2_device_t *device, uint32_t mask);

lidar_v2_control_status_t lidar_v2_write_gpx_staging_word(
    const lidar_v2_device_t *device,
    uint8_t register_address,
    uint32_t data_28bit);
lidar_v2_control_status_t lidar_v2_read_gpx_register(
    const lidar_v2_device_t *device,
    uint8_t chip_index,
    uint8_t register_address,
    uint32_t poll_limit,
    uint32_t *data_28bit);

const char *lidar_v2_control_status_name(lidar_v2_control_status_t status);

#ifdef __cplusplus
}
#endif

#endif
