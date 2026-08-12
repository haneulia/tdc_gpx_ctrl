#include "lidar_v3_control.h"

typedef struct {
    lidar_v3_vdma_transaction_t *transaction;
    bool service_failed;
} lidar_v3_commit_poll_context_t;

static int continue_until_hardware_terminal(void *context)
{
    lidar_v3_commit_poll_context_t *poll_context =
        (lidar_v3_commit_poll_context_t *)context;

    if (!poll_context->service_failed &&
        lidar_v3_vdma_transaction_poll(poll_context->transaction) != 0) {
        /*
         * Keep returning success to the shared CSR poller so it can collect
         * the RTL timeout, error code, and recovery-required state.
         */
        poll_context->service_failed = true;
    }
    return 0;
}

lidar_v2_control_status_t lidar_v3_commit_shadow(
    const lidar_v2_device_t *device,
    lidar_v3_vdma_transaction_t *vdma_transaction,
    uint32_t poll_limit,
    lidar_v2_commit_result_t *result)
{
    lidar_v3_commit_poll_context_t poll_context;
    lidar_v2_control_status_t status;

    if (device == NULL || vdma_transaction == NULL ||
        vdma_transaction->device != device || poll_limit == 0U ||
        result == NULL) {
        return LIDAR_V2_CONTROL_ARGUMENT;
    }
    /*
     * Drain the old VDMA geometry before CSR COMMIT starts. Ethernet or
     * Viewer processing must not consume G_PHASE_TIMEOUT_US after PREPARE.
     */
    if (lidar_v3_vdma_transaction_prepare(
            vdma_transaction, poll_limit) != 0) {
        return LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR;
    }
    lidar_v3_vdma_transaction_begin(vdma_transaction);
    poll_context.transaction = vdma_transaction;
    poll_context.service_failed = false;
    status = lidar_v2_commit_shadow(device, poll_limit,
        continue_until_hardware_terminal, &poll_context, result);
    if (poll_context.service_failed &&
        (status == LIDAR_V2_CONTROL_OK ||
         status == LIDAR_V2_CONTROL_TIMEOUT)) {
        return LIDAR_V2_CONTROL_VDMA_SERVICE_ERROR;
    }
    return status;
}
