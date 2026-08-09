#include "lidar_v2_ps_control.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_CSR_BASE 0x40000000U
#define TEST_WORD_COUNT 68U

typedef struct {
    uint32_t words[TEST_WORD_COUNT];
    uint16_t diagnostic_sequence;
    uint32_t barrier_count;
    uint32_t last_vdma_ack;
} fake_csr_t;

static uint32_t fake_read32(void *context, uintptr_t address)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    uint32_t offset = (uint32_t)(address - TEST_CSR_BASE);

    assert((offset & 3U) == 0U);
    assert((offset / 4U) < TEST_WORD_COUNT);
    return csr->words[offset / 4U];
}

static void fake_write32(void *context, uintptr_t address, uint32_t value)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    uint32_t offset = (uint32_t)(address - TEST_CSR_BASE);
    uint32_t index = offset / 4U;

    assert((offset & 3U) == 0U);
    assert(index < TEST_WORD_COUNT);
    if (offset == LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_COMMAND)) {
        uint32_t *transaction =
            &csr->words[LIDAR_V2_STAT_OFFSET(LIDAR_V2_STAT_TRANSACTION) / 4U];
        uint32_t *operation =
            &csr->words[LIDAR_V2_STAT_OFFSET(LIDAR_V2_STAT_ACTIVE_VERSION) / 4U];

        if ((value & LIDAR_V2_CMD_CLEAR_STATUS) != 0U) {
            *transaction &= ~(LIDAR_V2_TXN_DONE | LIDAR_V2_TXN_SUCCESS |
                LIDAR_V2_TXN_ERROR | LIDAR_V2_TXN_REJECTED |
                LIDAR_V2_TXN_ACCESS_ERROR);
        }
        if ((value & LIDAR_V2_CMD_STOP) != 0U) {
            *operation &= ~(LIDAR_V2_OP_RUNNING | LIDAR_V2_OP_ARMED);
            *operation |= LIDAR_V2_OP_COMMAND_READY;
        }
        if ((value & LIDAR_V2_CMD_RUN) != 0U) {
            *operation |= LIDAR_V2_OP_RUNNING | LIDAR_V2_OP_COMMAND_READY;
        }
        if ((value & LIDAR_V2_CMD_ARM) != 0U) {
            *operation |= LIDAR_V2_OP_ARMED | LIDAR_V2_OP_COMMAND_READY;
        }
        if ((value & LIDAR_V2_CMD_COMMIT) != 0U) {
            uint16_t version = (uint16_t)(*operation + 1U);
            *operation = (*operation & 0xFFFF0000U) | version;
            *transaction = LIDAR_V2_TXN_DONE | LIDAR_V2_TXN_SUCCESS |
                LIDAR_V2_TXN_ACTIVE_VALID;
        }
        return;
    }
    if (offset == LIDAR_V2_CTL_OFFSET(
            LIDAR_V2_CTL_VDMA_PROFILE_CONTROL)) {
        csr->last_vdma_ack = value;
        if ((value & LIDAR_V2_VDMA_RISE_ACK) != 0U) {
            csr->words[index] &= ~LIDAR_V2_VDMA_RISE_PENDING;
        }
        if ((value & LIDAR_V2_VDMA_FALL_ACK) != 0U) {
            csr->words[index] &= ~LIDAR_V2_VDMA_FALL_PENDING;
        }
        return;
    }
    if (offset == LIDAR_V2_IRQ_FLAG_OFFSET) {
        csr->words[index] &= ~value;
        return;
    }
    if (offset == LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_DIAG_INDEX)) {
        uint8_t register_address = (uint8_t)(value & 0xFU);

        ++csr->diagnostic_sequence;
        csr->words[index] =
            ((uint32_t)csr->diagnostic_sequence << 16U) | (1U << 9U);
        csr->words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_DIAG_DATA) / 4U] =
            ((uint32_t)register_address << 28U) | 0x00ABCDEFU;
        return;
    }
    csr->words[index] = value;
}

static void fake_barrier(void *context)
{
    fake_csr_t *csr = (fake_csr_t *)context;
    ++csr->barrier_count;
}

static void initialize_build_contract(fake_csr_t *csr)
{
    uint32_t core = LIDAR_V2_CSR_ABI_MINOR |
        (LIDAR_V2_CSR_ABI_MAJOR << 8U) |
        (5U << 16U) | (4U << 19U) | (8U << 22U) | (7U << 26U) |
        (1U << 29U);
    uint32_t build = 150U | (200U << 8U) | (1U << 16U) |
        (3U << 20U) | (12U << 24U);

    csr->words[LIDAR_V2_STAT_OFFSET(LIDAR_V2_STAT_CORE_INFO) / 4U] = core;
    csr->words[LIDAR_V2_STAT_OFFSET(LIDAR_V2_STAT_BUILD_INFO) / 4U] = build;
    csr->words[LIDAR_V2_STAT_OFFSET(LIDAR_V2_STAT_ACTIVE_VERSION) / 4U] =
        LIDAR_V2_OP_COMMAND_READY;
}

int main(void)
{
    fake_csr_t csr;
    lidar_v2_device_t device;
    lidar_v2_build_info_t build;
    lidar_v2_shadow_config_t shadow;
    lidar_v2_vdma_profiles_t profiles;
    lidar_v2_commit_result_t commit;
    uint32_t gpx_data;

    memset(&csr, 0, sizeof(csr));
    initialize_build_contract(&csr);
    lidar_v2_device_init(&device, TEST_CSR_BASE, &csr,
        fake_read32, fake_write32, fake_barrier);

    assert(lidar_v2_read_build_info(&device, &build) ==
        LIDAR_V2_CONTROL_OK);
    assert(build.num_chips == 4U && build.stops_per_chip == 8U);
    assert(build.max_returns_per_stop == 7U);
    assert(build.processing_clock_mhz == 150U && build.tdc_clock_mhz == 200U);
    assert(build.output_width_bits == 64U);
    assert(build.rise_capability_mask == 3U &&
        build.fall_capability_mask == 12U);

    memset(&shadow, 0, sizeof(shadow));
    shadow.ctl[LIDAR_V2_CTL_TARGET_RANGE] = 4000U;
    shadow.ctl[LIDAR_V2_CTL_SHOT_INTERVAL] = 50000U;
    assert(lidar_v2_write_shadow(&device, &shadow) ==
        LIDAR_V2_CONTROL_OK);
    memset(&shadow, 0, sizeof(shadow));
    assert(lidar_v2_read_shadow(&device, &shadow) ==
        LIDAR_V2_CONTROL_OK);
    assert(shadow.ctl[LIDAR_V2_CTL_TARGET_RANGE] == 4000U);
    assert(shadow.ctl[LIDAR_V2_CTL_SHOT_INTERVAL] == 50000U);

    assert(lidar_v2_stop_and_disarm(&device, 8U, NULL, NULL) ==
        LIDAR_V2_CONTROL_OK);
    assert(lidar_v2_commit_shadow(&device, 8U, NULL, NULL, &commit) ==
        LIDAR_V2_CONTROL_OK);
    assert(commit.active_version == 1U);
    assert(lidar_v2_run_and_arm(&device, 8U, NULL, NULL) ==
        LIDAR_V2_CONTROL_OK);

    csr.words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_VDMA_PROFILE_CONTROL) / 4U] =
        LIDAR_V2_VDMA_RISE_PENDING | LIDAR_V2_VDMA_RISE_ENABLE |
        LIDAR_V2_VDMA_FALL_PENDING | LIDAR_V2_VDMA_FALL_ENABLE;
    csr.words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_VDMA_RISE_GEOMETRY) / 4U] =
        (1801U << 16U) | 512U;
    csr.words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_VDMA_RISE_STRIDE) / 4U] = 1024U;
    csr.words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_VDMA_FALL_GEOMETRY) / 4U] =
        (1801U << 16U) | 512U;
    csr.words[LIDAR_V2_CTL_OFFSET(LIDAR_V2_CTL_VDMA_FALL_STRIDE) / 4U] = 1024U;
    assert(lidar_v2_read_vdma_profiles(&device, &profiles) ==
        LIDAR_V2_CONTROL_OK);
    assert(profiles.rise.pending && profiles.fall.pending);
    assert(profiles.rise.hsize_bytes == 512U &&
        profiles.rise.vsize_lines == 1801U &&
        profiles.rise.stride_bytes == 1024U);
    assert(lidar_v2_ack_vdma_profiles(&device, true, true) ==
        LIDAR_V2_CONTROL_OK);
    assert(csr.last_vdma_ack ==
        (LIDAR_V2_VDMA_RISE_ACK | LIDAR_V2_VDMA_FALL_ACK));

    lidar_v2_irq_configure(&device,
        LIDAR_V2_IRQ_COMMIT_ERROR | LIDAR_V2_IRQ_GPX_DATA, 0U);
    csr.words[LIDAR_V2_IRQ_FLAG_OFFSET / 4U] =
        LIDAR_V2_IRQ_COMMIT_ERROR | LIDAR_V2_IRQ_GPX_DATA;
    csr.words[LIDAR_V2_IRQ_STATUS_OFFSET / 4U] = LIDAR_V2_IRQ_GPX_DATA;
    assert(lidar_v2_irq_pending(&device) ==
        (LIDAR_V2_IRQ_COMMIT_ERROR | LIDAR_V2_IRQ_GPX_DATA));
    assert(lidar_v2_irq_source_level(&device) == LIDAR_V2_IRQ_GPX_DATA);
    lidar_v2_irq_ack(&device, LIDAR_V2_IRQ_COMMIT_ERROR);
    assert(lidar_v2_irq_pending(&device) == LIDAR_V2_IRQ_GPX_DATA);

    assert(lidar_v2_write_gpx_staging_word(&device, 7U, 0x01234567U) ==
        LIDAR_V2_CONTROL_OK);
    assert(lidar_v2_read_ctl(&device, LIDAR_V2_CTL_GPX_IMAGE_INDEX) == 7U);
    assert(lidar_v2_read_ctl(&device, LIDAR_V2_CTL_GPX_IMAGE_DATA) ==
        0x01234567U);
    assert(lidar_v2_read_gpx_register(&device, 3U, 7U, 8U, &gpx_data) ==
        LIDAR_V2_CONTROL_OK);
    assert(gpx_data == 0x00ABCDEFU);
    assert(csr.barrier_count != 0U);

    puts("LIDAR_V2_PS_CONTROL_TEST_PASS");
    return 0;
}
