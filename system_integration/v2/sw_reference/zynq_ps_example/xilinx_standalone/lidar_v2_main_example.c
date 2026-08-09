/*
 * Board integration template for Zynq-7000 standalone or FreeRTOS + lwIP.
 *
 * This translation unit is intentionally disabled by default.  Define
 * LIDAR_V2_BUILD_BOARD_MAIN and the board address/interrupt macros below in
 * the application build after Vivado exports xparameters.h.
 */
#if defined(LIDAR_V2_BUILD_BOARD_MAIN)

#include "lidar_v2_example_app.h"

#include <string.h>

#include "lwip/ip_addr.h"
#include "xaxivdma.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xstatus.h"

#ifndef LIDAR_V2_CSR_BASE_ADDRESS
#error "Define LIDAR_V2_CSR_BASE_ADDRESS from xparameters.h"
#endif
#ifndef LIDAR_V2_RISE_VDMA_DEVICE_ID
#error "Define LIDAR_V2_RISE_VDMA_DEVICE_ID from xparameters.h"
#endif
#ifndef LIDAR_V2_FALL_VDMA_DEVICE_ID
#error "Define LIDAR_V2_FALL_VDMA_DEVICE_ID from xparameters.h"
#endif
#ifndef LIDAR_V2_GIC_DEVICE_ID
#error "Define LIDAR_V2_GIC_DEVICE_ID from xparameters.h"
#endif
#ifndef LIDAR_V2_RISE_VDMA_IRQ_ID
#error "Define LIDAR_V2_RISE_VDMA_IRQ_ID from xparameters.h"
#endif
#ifndef LIDAR_V2_FALL_VDMA_IRQ_ID
#error "Define LIDAR_V2_FALL_VDMA_IRQ_ID from xparameters.h"
#endif
#ifndef LIDAR_V2_RISE_DDR_BASE
#error "Reserve non-overlapping Rise/Fall DDR frame-buffer ranges"
#endif
#ifndef LIDAR_V2_FALL_DDR_BASE
#error "Reserve non-overlapping Rise/Fall DDR frame-buffer ranges"
#endif
#ifndef LIDAR_V2_FRAME_CAPACITY_BYTES
#error "Define the synthesis-maximum bytes reserved for one VDMA frame"
#endif
#ifndef LIDAR_V2_BIN_RESOLUTION_PS
#error "Define the physical TDC-GPX bin resolution in picoseconds"
#endif

#define LIDAR_V2_FRAME_STORE_COUNT 3U

/* Implement these functions in the board's network/diagnostic layer. */
extern int lidar_board_network_init(void);
extern void lidar_board_network_poll(void);
/*
 * Read the STAT registers and accumulated diagnostic counters for every
 * pending source. Return only the IRQ bits whose causes are fully handled.
 */
extern uint32_t lidar_board_handle_irq_sources(
    lidar_v2_example_app_t *app,
    uint32_t pending_flags);

static XAxiVdma rise_vdma;
static XAxiVdma fall_vdma;
static XScuGic gic;
static lidar_v2_example_app_t lidar_app;

static const UINTPTR rise_frame_addresses[LIDAR_V2_FRAME_STORE_COUNT] = {
    LIDAR_V2_RISE_DDR_BASE,
    LIDAR_V2_RISE_DDR_BASE + LIDAR_V2_FRAME_CAPACITY_BYTES,
    LIDAR_V2_RISE_DDR_BASE + 2U * LIDAR_V2_FRAME_CAPACITY_BYTES
};

static const UINTPTR fall_frame_addresses[LIDAR_V2_FRAME_STORE_COUNT] = {
    LIDAR_V2_FALL_DDR_BASE,
    LIDAR_V2_FALL_DDR_BASE + LIDAR_V2_FRAME_CAPACITY_BYTES,
    LIDAR_V2_FALL_DDR_BASE + 2U * LIDAR_V2_FRAME_CAPACITY_BYTES
};

static int initialize_vdma(XAxiVdma *instance, uint16_t device_id)
{
    XAxiVdma_Config *config = XAxiVdma_LookupConfig(device_id);

    if (config == NULL) {
        return XST_FAILURE;
    }
    return XAxiVdma_CfgInitialize(instance, config, config->BaseAddress);
}

static int initialize_interrupts(void)
{
    XScuGic_Config *config = XScuGic_LookupConfig(LIDAR_V2_GIC_DEVICE_ID);

    if (config == NULL ||
        XScuGic_CfgInitialize(&gic, config,
            config->CpuBaseAddress) != XST_SUCCESS ||
        XScuGic_Connect(&gic, LIDAR_V2_RISE_VDMA_IRQ_ID,
            (Xil_InterruptHandler)XAxiVdma_WriteIntrHandler,
            &rise_vdma) != XST_SUCCESS ||
        XScuGic_Connect(&gic, LIDAR_V2_FALL_VDMA_IRQ_ID,
            (Xil_InterruptHandler)XAxiVdma_WriteIntrHandler,
            &fall_vdma) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(&gic, LIDAR_V2_RISE_VDMA_IRQ_ID);
    XScuGic_Enable(&gic, LIDAR_V2_FALL_VDMA_IRQ_ID);
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, &gic);
    Xil_ExceptionEnable();
    return XST_SUCCESS;
}

int main(void)
{
    lidar_v2_example_parameters_t parameters;

    if (lidar_board_network_init() != XST_SUCCESS ||
        initialize_vdma(&rise_vdma,
            LIDAR_V2_RISE_VDMA_DEVICE_ID) != XST_SUCCESS ||
        initialize_vdma(&fall_vdma,
            LIDAR_V2_FALL_VDMA_DEVICE_ID) != XST_SUCCESS ||
        initialize_interrupts() != XST_SUCCESS) {
        return XST_FAILURE;
    }

    memset(&parameters, 0, sizeof(parameters));
    parameters.csr_base = LIDAR_V2_CSR_BASE_ADDRESS;
    parameters.rise_vdma = &rise_vdma;
    parameters.fall_vdma = &fall_vdma;
    parameters.rise_frame_addresses = rise_frame_addresses;
    parameters.fall_frame_addresses = fall_frame_addresses;
    parameters.frame_store_count = LIDAR_V2_FRAME_STORE_COUNT;
    parameters.frame_capacity_bytes = LIDAR_V2_FRAME_CAPACITY_BYTES;
    parameters.bin_resolution_ps = LIDAR_V2_BIN_RESOLUTION_PS;
    IP_ADDR4(&parameters.viewer_address, 192, 168, 1, 50);
    parameters.viewer_udp_port = 5000U;

    /* All three time fields below use the common Runtime 5 ns tick unit. */
    parameters.fire_width_5ns_ticks = 4U;
    parameters.fire_done_timeout_5ns_ticks = 288U;
    /* Laser target round-trip time (2R/c, TARGET_RANGE_WINDOW_5NS). */
    parameters.target_range_window_5ns = 4000U;
    /* Requested optical angle between adjacent laser candidate points. */
    parameters.optical_shot_interval_udeg = 50000U;
    /* Runtime TDC-GPX bus read timing, not the Processing clock divider. */
    parameters.tdc_bus_clk_div = 1U;
    parameters.tdc_bus_ticks = 5U;
    parameters.active_chip_mask = 0xFU;
    parameters.falling_enable = true;
    parameters.visible_returns = 7U;

    if (lidar_v2_example_app_init(&lidar_app, &parameters) != XST_SUCCESS) {
        return XST_FAILURE;
    }

    for (;;) {
        lidar_board_network_poll();
        if (lidar_v2_example_app_step(&lidar_app) != XST_SUCCESS) {
            return XST_FAILURE;
        }
        /* Diagnose first; W1C only the causes that software fully handled. */
        if (lidar_app.last_irq_flags != 0U) {
            uint32_t handled_flags = lidar_board_handle_irq_sources(
                &lidar_app, lidar_app.last_irq_flags);

            if (handled_flags != 0U &&
                lidar_v2_example_acknowledge_irqs(
                    &lidar_app, handled_flags, false) != XST_SUCCESS) {
                return XST_FAILURE;
            }
        }
    }
}

#endif
