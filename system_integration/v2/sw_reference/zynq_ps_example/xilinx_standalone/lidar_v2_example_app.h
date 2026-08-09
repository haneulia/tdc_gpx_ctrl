#ifndef LIDAR_V2_EXAMPLE_APP_H
#define LIDAR_V2_EXAMPLE_APP_H

#include <stddef.h>
#include <stdint.h>

#include "xaxivdma.h"

#include "lidar_v2_lwip_udp.h"
#include "lidar_v2_xilinx_vdma.h"

typedef struct {
    uintptr_t csr_base;
    XAxiVdma *rise_vdma;
    XAxiVdma *fall_vdma;
    const UINTPTR *rise_frame_addresses;
    const UINTPTR *fall_frame_addresses;
    uint8_t frame_store_count;
    size_t frame_capacity_bytes;
    uint16_t bin_resolution_ps;
    ip_addr_t viewer_address;
    uint16_t viewer_udp_port;

    uint16_t fire_width_5ns_ticks;
    uint16_t fire_done_timeout_5ns_ticks;
    uint32_t target_range_window_5ns;
    uint32_t optical_shot_interval_udeg;
    uint8_t tdc_bus_clk_div;
    uint8_t tdc_bus_ticks;
    uint8_t active_chip_mask;
    bool falling_enable;
    uint8_t visible_returns;
} lidar_v2_example_parameters_t;

typedef struct {
    lidar_v2_device_t device;
    lidar_v2_build_info_t build;
    lidar_v2_xilinx_vdma_lane_t rise_lane;
    lidar_v2_xilinx_vdma_lane_t fall_lane;
    lidar_v2_xilinx_vdma_service_t vdma_service;
    lidar_v2_lwip_udp_t udp;
    lidar_ps_decode_summary_t last_rise_summary;
    lidar_ps_decode_summary_t last_fall_summary;
    uint16_t bin_resolution_ps;
    uint32_t last_irq_flags;
} lidar_v2_example_app_t;

/*
 * XAxiVdma instances and the lwIP interface must already be initialized.
 * The caller must connect XAxiVdma_WriteIntrHandler() to each VDMA IRQ.
 */
int lidar_v2_example_app_init(
    lidar_v2_example_app_t *app,
    const lidar_v2_example_parameters_t *parameters);

/* Call repeatedly from the lwIP/FreeRTOS processing task. */
int lidar_v2_example_app_step(lidar_v2_example_app_t *app);

/* Inspect source-specific diagnostics before calling this function. */
int lidar_v2_example_acknowledge_irqs(
    lidar_v2_example_app_t *app,
    uint32_t handled_mask,
    bool clear_diagnostic_history);

#endif
