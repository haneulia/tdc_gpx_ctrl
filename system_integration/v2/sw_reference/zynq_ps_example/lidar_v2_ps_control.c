#include "lidar_v2_ps_control.h"

#include <string.h>

static bool device_valid(const lidar_v2_device_t *device)
{
    return device != NULL && device->read32 != NULL &&
        device->write32 != NULL;
}

static void barrier(const lidar_v2_device_t *device)
{
    if (device->barrier != NULL) {
        device->barrier(device->context);
    }
}

static uint32_t read_offset(
    const lidar_v2_device_t *device,
    uint32_t offset)
{
    uint32_t value = device->read32(
        device->context, device->csr_base + offset);
    barrier(device);
    return value;
}

static void write_offset(
    const lidar_v2_device_t *device,
    uint32_t offset,
    uint32_t value)
{
    device->write32(device->context, device->csr_base + offset, value);
    barrier(device);
}

static int service_poll(
    lidar_v2_poll_service_t poll_service,
    void *service_context)
{
    if (poll_service == NULL) {
        return 0;
    }
    return poll_service(service_context);
}

static lidar_v2_control_status_t wait_operation(
    const lidar_v2_device_t *device,
    uint32_t set_mask,
    uint32_t clear_mask,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context)
{
    uint32_t poll;

    for (poll = 0U; poll < poll_limit; ++poll) {
        uint32_t operation = lidar_v2_read_stat(
            device, LIDAR_V2_STAT_ACTIVE_VERSION);
        if ((operation & set_mask) == set_mask &&
            (operation & clear_mask) == 0U) {
            return LIDAR_V2_CONTROL_OK;
        }
        if (service_poll(poll_service, service_context) != 0) {
            return LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR;
        }
    }
    return LIDAR_V2_CONTROL_TIMEOUT;
}

void lidar_v2_device_init(
    lidar_v2_device_t *device,
    uintptr_t csr_base,
    void *context,
    lidar_v2_mmio_read32_t read32,
    lidar_v2_mmio_write32_t write32,
    lidar_v2_mmio_barrier_t barrier_fn)
{
    if (device != NULL) {
        device->csr_base = csr_base;
        device->context = context;
        device->read32 = read32;
        device->write32 = write32;
        device->barrier = barrier_fn;
    }
}

uint32_t lidar_v2_read_ctl(const lidar_v2_device_t *device, uint8_t index)
{
    if (!device_valid(device) || index >= LIDAR_V2_CTL_COUNT) {
        return 0U;
    }
    return read_offset(device, LIDAR_V2_CTL_OFFSET(index));
}

uint32_t lidar_v2_read_stat(const lidar_v2_device_t *device, uint8_t index)
{
    if (!device_valid(device) || index >= LIDAR_V2_STAT_COUNT) {
        return 0U;
    }
    return read_offset(device, LIDAR_V2_STAT_OFFSET(index));
}

void lidar_v2_write_ctl(
    const lidar_v2_device_t *device,
    uint8_t index,
    uint32_t value)
{
    if (device_valid(device) && index < LIDAR_V2_CTL_COUNT) {
        write_offset(device, LIDAR_V2_CTL_OFFSET(index), value);
    }
}

lidar_v2_control_status_t lidar_v2_read_build_info(
    const lidar_v2_device_t *device,
    lidar_v2_build_info_t *info)
{
    uint32_t core;
    uint32_t build;
    uint32_t width_code;

    if (!device_valid(device) || info == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    core = lidar_v2_read_stat(device, LIDAR_V2_STAT_CORE_INFO);
    build = lidar_v2_read_stat(device, LIDAR_V2_STAT_BUILD_INFO);
    memset(info, 0, sizeof(*info));
    info->abi_minor = (uint8_t)(core & 0xFFU);
    info->abi_major = (uint8_t)((core >> 8U) & 0xFFU);
    info->num_faces = (uint8_t)((core >> 16U) & 0x7U);
    info->num_chips = (uint8_t)((core >> 19U) & 0x7U);
    info->stops_per_chip = (uint8_t)((core >> 22U) & 0xFU);
    info->max_returns_per_stop = (uint8_t)((core >> 26U) & 0x7U);
    info->echo_receiver_enabled = ((core >> 29U) & 1U) != 0U;
    info->echo_simulation_enabled = ((core >> 30U) & 1U) != 0U;
    info->processing_tdc_clock_sync = ((core >> 31U) & 1U) != 0U;
    info->processing_clock_mhz = (uint8_t)(build & 0xFFU);
    info->tdc_clock_mhz = (uint8_t)((build >> 8U) & 0xFFU);
    width_code = (build >> 16U) & 0x3U;
    info->output_width_bits = width_code == 0U ? 32U :
        (width_code == 1U ? 64U : (width_code == 2U ? 128U : 0U));
    info->rise_capability_mask = (uint8_t)((build >> 20U) & 0xFU);
    info->fall_capability_mask = (uint8_t)((build >> 24U) & 0xFU);

    if (info->abi_major != LIDAR_V2_CSR_ABI_MAJOR ||
        info->abi_minor != LIDAR_V2_CSR_ABI_MINOR ||
        info->output_width_bits == 0U) {
        return LIDAR_V2_CONTROL_ABI_MISMATCH;
    }
    return LIDAR_V2_CONTROL_OK;
}

lidar_v2_control_status_t lidar_v2_read_shadow(
    const lidar_v2_device_t *device,
    lidar_v2_shadow_config_t *shadow)
{
    uint8_t index;

    if (!device_valid(device) || shadow == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    memset(shadow, 0, sizeof(*shadow));
    for (index = 1U; index <= LIDAR_V2_RUNTIME_SHADOW_LAST; ++index) {
        shadow->ctl[index] = lidar_v2_read_ctl(device, index);
    }
    return LIDAR_V2_CONTROL_OK;
}

lidar_v2_control_status_t lidar_v2_write_shadow(
    const lidar_v2_device_t *device,
    const lidar_v2_shadow_config_t *shadow)
{
    uint8_t index;

    if (!device_valid(device) || shadow == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    for (index = 1U; index <= LIDAR_V2_RUNTIME_SHADOW_LAST; ++index) {
        lidar_v2_write_ctl(device, index, shadow->ctl[index]);
    }
    return LIDAR_V2_CONTROL_OK;
}

lidar_v2_control_status_t lidar_v2_clear_status(
    const lidar_v2_device_t *device,
    uint32_t poll_limit)
{
    uint32_t poll;
    const uint32_t cleared_mask = LIDAR_V2_TXN_DONE |
        LIDAR_V2_TXN_SUCCESS | LIDAR_V2_TXN_ERROR |
        LIDAR_V2_TXN_REJECTED | LIDAR_V2_TXN_ACCESS_ERROR;

    if (!device_valid(device) || poll_limit == 0U) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_COMMAND,
        LIDAR_V2_CMD_CLEAR_STATUS);
    for (poll = 0U; poll < poll_limit; ++poll) {
        if ((lidar_v2_read_stat(device, LIDAR_V2_STAT_TRANSACTION) &
             cleared_mask) == 0U) {
            return LIDAR_V2_CONTROL_OK;
        }
    }
    return LIDAR_V2_CONTROL_TIMEOUT;
}

lidar_v2_control_status_t lidar_v2_stop_and_disarm(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context)
{
    lidar_v2_control_status_t status;

    if (!device_valid(device) || poll_limit == 0U) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    status = wait_operation(device, LIDAR_V2_OP_COMMAND_READY, 0U,
        poll_limit, poll_service, service_context);
    if (status != LIDAR_V2_CONTROL_OK) {
        return status;
    }
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_COMMAND, LIDAR_V2_CMD_STOP);
    return wait_operation(device, LIDAR_V2_OP_COMMAND_READY,
        LIDAR_V2_OP_RUNNING | LIDAR_V2_OP_ARMED,
        poll_limit, poll_service, service_context);
}

lidar_v2_control_status_t lidar_v2_run_and_arm(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context)
{
    lidar_v2_control_status_t status;

    if (!device_valid(device) || poll_limit == 0U) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    status = wait_operation(device, LIDAR_V2_OP_COMMAND_READY, 0U,
        poll_limit, poll_service, service_context);
    if (status != LIDAR_V2_CONTROL_OK) {
        return status;
    }
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_COMMAND, LIDAR_V2_CMD_RUN);
    status = wait_operation(device,
        LIDAR_V2_OP_COMMAND_READY | LIDAR_V2_OP_RUNNING, 0U,
        poll_limit, poll_service, service_context);
    if (status != LIDAR_V2_CONTROL_OK) {
        return status;
    }
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_COMMAND, LIDAR_V2_CMD_ARM);
    return wait_operation(device,
        LIDAR_V2_OP_COMMAND_READY | LIDAR_V2_OP_RUNNING |
            LIDAR_V2_OP_ARMED,
        0U, poll_limit, poll_service, service_context);
}

lidar_v2_control_status_t lidar_v2_commit_shadow(
    const lidar_v2_device_t *device,
    uint32_t poll_limit,
    lidar_v2_poll_service_t poll_service,
    void *service_context,
    lidar_v2_commit_result_t *result)
{
    uint32_t poll;

    if (!device_valid(device) || poll_limit == 0U || result == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_COMMAND, LIDAR_V2_CMD_COMMIT);

    for (poll = 0U; poll < poll_limit; ++poll) {
        uint32_t transaction;

        if (service_poll(poll_service, service_context) != 0) {
            return LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR;
        }
        transaction = lidar_v2_read_stat(
            device, LIDAR_V2_STAT_TRANSACTION);
        if ((transaction & LIDAR_V2_TXN_BUSY) == 0U &&
            (transaction & LIDAR_V2_TXN_DONE) != 0U) {
            result->transaction_word = transaction;
            result->active_version = (uint16_t)lidar_v2_read_stat(
                device, LIDAR_V2_STAT_ACTIVE_VERSION);
            result->error_code = (uint8_t)((transaction >> 16U) & 0xFFU);
            result->reject_code = (uint8_t)((transaction >> 24U) & 0xFFU);
            if ((transaction & LIDAR_V2_TXN_RECOVERY_REQUIRED) != 0U) {
                return LIDAR_V2_CONTROL_RECOVERY_REQUIRED;
            }
            if ((transaction & LIDAR_V2_TXN_REJECTED) != 0U) {
                return LIDAR_V2_CONTROL_COMMIT_REJECTED;
            }
            if ((transaction & LIDAR_V2_TXN_ERROR) != 0U) {
                return LIDAR_V2_CONTROL_COMMIT_ERROR;
            }
            if ((transaction & LIDAR_V2_TXN_ACCESS_ERROR) != 0U) {
                return LIDAR_V2_CONTROL_ACCESS_ERROR;
            }
            if ((transaction & LIDAR_V2_TXN_SUCCESS) != 0U &&
                (transaction & LIDAR_V2_TXN_ACTIVE_VALID) != 0U) {
                return LIDAR_V2_CONTROL_OK;
            }
            return LIDAR_V2_CONTROL_COMMIT_ERROR;
        }
    }
    return LIDAR_V2_CONTROL_TIMEOUT;
}

lidar_v2_control_status_t lidar_v2_read_vdma_profiles(
    const lidar_v2_device_t *device,
    lidar_v2_vdma_profiles_t *profiles)
{
    uint32_t retry;

    if (!device_valid(device) || profiles == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    for (retry = 0U; retry < 4U; ++retry) {
        uint32_t control_before = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_PROFILE_CONTROL);
        uint32_t rise_geometry = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_RISE_GEOMETRY);
        uint32_t rise_stride = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_RISE_STRIDE);
        uint32_t fall_geometry = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_FALL_GEOMETRY);
        uint32_t fall_stride = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_FALL_STRIDE);
        uint32_t control_after = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_VDMA_PROFILE_CONTROL);

        if ((control_before & 0xFU) != (control_after & 0xFU)) {
            continue;
        }
        memset(profiles, 0, sizeof(*profiles));
        profiles->rise.pending =
            (control_after & LIDAR_V2_VDMA_RISE_PENDING) != 0U;
        profiles->rise.enabled =
            (control_after & LIDAR_V2_VDMA_RISE_ENABLE) != 0U;
        profiles->rise.hsize_bytes = (uint16_t)rise_geometry;
        profiles->rise.vsize_lines = (uint16_t)(rise_geometry >> 16U);
        profiles->rise.stride_bytes = (uint16_t)rise_stride;
        profiles->fall.pending =
            (control_after & LIDAR_V2_VDMA_FALL_PENDING) != 0U;
        profiles->fall.enabled =
            (control_after & LIDAR_V2_VDMA_FALL_ENABLE) != 0U;
        profiles->fall.hsize_bytes = (uint16_t)fall_geometry;
        profiles->fall.vsize_lines = (uint16_t)(fall_geometry >> 16U);
        profiles->fall.stride_bytes = (uint16_t)fall_stride;
        return LIDAR_V2_CONTROL_OK;
    }
    return LIDAR_V2_CONTROL_TIMEOUT;
}

lidar_v2_control_status_t lidar_v2_ack_vdma_profiles(
    const lidar_v2_device_t *device,
    bool acknowledge_rise,
    bool acknowledge_fall)
{
    uint32_t control;
    uint32_t acknowledge = 0U;

    if (!device_valid(device)) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    control = lidar_v2_read_ctl(
        device, LIDAR_V2_CTL_VDMA_PROFILE_CONTROL);
    if (acknowledge_rise) {
        if ((control & LIDAR_V2_VDMA_RISE_PENDING) == 0U) {
            return LIDAR_V2_CONTROL_ACCESS_ERROR;
        }
        acknowledge |= LIDAR_V2_VDMA_RISE_ACK;
    }
    if (acknowledge_fall) {
        if ((control & LIDAR_V2_VDMA_FALL_PENDING) == 0U) {
            return LIDAR_V2_CONTROL_ACCESS_ERROR;
        }
        acknowledge |= LIDAR_V2_VDMA_FALL_ACK;
    }
    if (acknowledge != 0U) {
        lidar_v2_write_ctl(device, LIDAR_V2_CTL_VDMA_PROFILE_CONTROL,
            acknowledge);
    }
    return LIDAR_V2_CONTROL_OK;
}

void lidar_v2_irq_configure(
    const lidar_v2_device_t *device,
    uint32_t enable_mask,
    uint32_t automatic_pulse_mask)
{
    if (!device_valid(device)) {
        return;
    }
    write_offset(device, LIDAR_V2_IRQ_ENABLE_OFFSET,
        enable_mask & LIDAR_V2_IRQ_ALL_MASK);
    write_offset(device, LIDAR_V2_IRQ_MODE_OFFSET,
        automatic_pulse_mask & LIDAR_V2_IRQ_ALL_MASK);
}

uint32_t lidar_v2_irq_pending(const lidar_v2_device_t *device)
{
    if (!device_valid(device)) {
        return 0U;
    }
    return read_offset(device, LIDAR_V2_IRQ_FLAG_OFFSET) &
        read_offset(device, LIDAR_V2_IRQ_ENABLE_OFFSET) &
        LIDAR_V2_IRQ_ALL_MASK;
}

uint32_t lidar_v2_irq_source_level(const lidar_v2_device_t *device)
{
    if (!device_valid(device)) {
        return 0U;
    }
    return read_offset(device, LIDAR_V2_IRQ_STATUS_OFFSET) &
        LIDAR_V2_IRQ_ALL_MASK;
}

void lidar_v2_irq_ack(const lidar_v2_device_t *device, uint32_t mask)
{
    if (device_valid(device)) {
        write_offset(device, LIDAR_V2_IRQ_FLAG_OFFSET,
            mask & LIDAR_V2_IRQ_ALL_MASK);
    }
}

lidar_v2_control_status_t lidar_v2_write_gpx_staging_word(
    const lidar_v2_device_t *device,
    uint8_t register_address,
    uint32_t data_28bit)
{
    if (!device_valid(device) || register_address > 15U ||
        (data_28bit & 0xF0000000U) != 0U) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_GPX_IMAGE_INDEX,
        register_address);
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_GPX_IMAGE_DATA, data_28bit);
    return LIDAR_V2_CONTROL_OK;
}

lidar_v2_control_status_t lidar_v2_read_gpx_register(
    const lidar_v2_device_t *device,
    uint8_t chip_index,
    uint8_t register_address,
    uint32_t poll_limit,
    uint32_t *data_28bit)
{
    uint32_t before;
    uint16_t sequence_before;
    uint32_t request;
    uint32_t poll;

    if (!device_valid(device) || chip_index > 3U ||
        register_address > 15U || poll_limit == 0U || data_28bit == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    before = lidar_v2_read_ctl(device, LIDAR_V2_CTL_DIAG_INDEX);
    sequence_before = (uint16_t)(before >> 16U);
    request = 0x100U | 0xC0U | ((uint32_t)chip_index << 4U) |
        register_address;
    lidar_v2_write_ctl(device, LIDAR_V2_CTL_DIAG_INDEX, request);

    for (poll = 0U; poll < poll_limit; ++poll) {
        uint32_t status = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_DIAG_INDEX);
        uint16_t sequence = (uint16_t)(status >> 16U);

        if ((status & (1U << 8U)) != 0U || sequence == sequence_before) {
            continue;
        }
        if ((status & (1U << 10U)) != 0U ||
            (status & (1U << 9U)) == 0U) {
            return LIDAR_V2_CONTROL_GPX_READ_ERROR;
        }
        *data_28bit = lidar_v2_read_ctl(
            device, LIDAR_V2_CTL_DIAG_DATA);
        if (((*data_28bit >> 28U) & 0xFU) != register_address) {
            return LIDAR_V2_CONTROL_GPX_READ_ERROR;
        }
        *data_28bit &= 0x0FFFFFFFU;
        return LIDAR_V2_CONTROL_OK;
    }
    return LIDAR_V2_CONTROL_TIMEOUT;
}

const char *lidar_v2_control_status_name(lidar_v2_control_status_t status)
{
    static const char *const names[] = {
        "OK",
        "ARGUMENT",
        "TIMEOUT",
        "COMMIT_ERROR",
        "COMMIT_REJECTED",
        "RECOVERY_REQUIRED",
        "ACCESS_ERROR",
        "VDMA_SERVICE_ERROR",
        "GPX_READ_ERROR",
        "ABI_MISMATCH"
    };

    if ((unsigned int)status >= sizeof(names) / sizeof(names[0])) {
        return "UNKNOWN";
    }
    return names[status];
}
