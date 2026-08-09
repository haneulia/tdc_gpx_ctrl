#include "lidar_v2_example_app.h"

#include <string.h>

#include "xil_io.h"
#include "xpseudo_asm.h"
#include "xstatus.h"

static uint32_t xilinx_read32(void *context, uintptr_t address)
{
    (void)context;
    return Xil_In32((UINTPTR)address);
}

static void xilinx_write32(void *context, uintptr_t address, uint32_t value)
{
    (void)context;
    Xil_Out32((UINTPTR)address, value);
}

static void xilinx_barrier(void *context)
{
    (void)context;
    dmb();
}

static int update_runtime_shadow(
    lidar_v2_example_app_t *app,
    const lidar_v2_example_parameters_t *parameters)
{
    lidar_v2_shadow_config_t shadow;
    uint32_t bus_profile;

    if (parameters->fire_done_timeout_5ns_ticks >
            parameters->target_range_window_5ns ||
        parameters->tdc_bus_clk_div == 0U ||
        parameters->tdc_bus_ticks == 0U ||
        parameters->tdc_bus_clk_div > 63U ||
        parameters->tdc_bus_ticks > 63U ||
        parameters->active_chip_mask == 0U ||
        parameters->active_chip_mask > 15U ||
        parameters->visible_returns == 0U ||
        parameters->visible_returns > app->build.max_returns_per_stop ||
        parameters->optical_shot_interval_udeg == 0U ||
        parameters->optical_shot_interval_udeg > 0x3FFFFFFFU) {
        return XST_INVALID_PARAM;
    }
    if (lidar_v2_read_shadow(&app->device, &shadow) !=
        LIDAR_V2_CONTROL_OK) {
        return XST_FAILURE;
    }

    shadow.ctl[LIDAR_V2_CTL_LASER_FIRE_PROFILE] =
        ((uint32_t)parameters->fire_done_timeout_5ns_ticks << 16U) |
        parameters->fire_width_5ns_ticks;
    /* Laser target round-trip time (2R/c), in the common 5 ns runtime unit. */
    shadow.ctl[LIDAR_V2_CTL_TARGET_RANGE] =
        parameters->target_range_window_5ns;
    /* Requested optical angle between adjacent laser candidate points. */
    shadow.ctl[LIDAR_V2_CTL_SHOT_INTERVAL] =
        parameters->optical_shot_interval_udeg;
    bus_profile = ((uint32_t)parameters->tdc_bus_clk_div & 0x3FU) |
        (((uint32_t)parameters->tdc_bus_ticks & 0x3FU) << 6U) |
        (((uint32_t)parameters->active_chip_mask & 0xFU) << 12U) |
        ((parameters->falling_enable ? 1U : 0U) << 16U) |
        (((uint32_t)parameters->visible_returns & 0x7U) << 17U);
    shadow.ctl[LIDAR_V2_CTL_TDC_BUS_PROFILE] = bus_profile;

    return lidar_v2_write_shadow(&app->device, &shadow) ==
        LIDAR_V2_CONTROL_OK ? XST_SUCCESS : XST_FAILURE;
}

static int build_view_config(
    const lidar_v2_example_app_t *app,
    const lidar_ps_frame_info_t *frame_info,
    lidar_ps_view_config_t *view)
{
    uint32_t active_version;
    uint32_t derived_geometry;
    uint32_t bounds;
    uint32_t masks;
    uint32_t center;

    if (frame_info->face_index >= app->build.num_faces) {
        return XST_FAILURE;
    }
    active_version = lidar_v2_read_stat(
        &app->device, LIDAR_V2_STAT_ACTIVE_VERSION);
    if ((uint16_t)active_version != frame_info->active_config_version) {
        /* Production software should retain config snapshots by version. */
        return XST_FAILURE;
    }
    derived_geometry = lidar_v2_read_stat(
        &app->device, LIDAR_V2_STAT_DERIVED_GEOMETRY);
    bounds = lidar_v2_read_stat(&app->device,
        (uint8_t)(LIDAR_V2_STAT_FACE_BOUNDS_0 + frame_info->face_index));
    center = lidar_v2_read_stat(&app->device,
        (uint8_t)(LIDAR_V2_STAT_ACTIVE_SOURCE_BASE +
            LIDAR_V2_CTL_FACE_CENTER_0 + frame_info->face_index - 1U));
    masks = lidar_v2_read_stat(&app->device, LIDAR_V2_STAT_DERIVED_MASKS);

    memset(view, 0, sizeof(*view));
    view->t0_tick_hz = (uint32_t)app->build.processing_clock_mhz * 1000000U;
    view->bin_resolution_ps = app->bin_resolution_ps;
    view->total_states_per_revolution = derived_geometry & 0xFFFFU;
    view->face_lower_state = (uint16_t)(bounds & 0x7FFFU);
    view->face_center_state = (uint16_t)(center & 0x7FFFU);
    view->face_upper_state_exclusive =
        (uint16_t)((bounds >> 16U) & 0x7FFFU);
    view->configured_face_count = app->build.num_faces;
    view->lane_chip_mask = frame_info->slope_rise != 0U ?
        (uint8_t)((masks >> 4U) & 0xFU) :
        (uint8_t)((masks >> 8U) & 0xFU);
    view->stops_per_chip = app->build.stops_per_chip;
    return XST_SUCCESS;
}

static int process_lane(
    lidar_v2_example_app_t *app,
    lidar_v2_xilinx_vdma_lane_t *lane,
    bool expect_rise,
    lidar_ps_decode_summary_t *summary)
{
    uint32_t derived_face;
    uint16_t planned_shots;
    uint8_t frame_index;
    lidar_ps_ddr_frame_t frame;
    lidar_ps_frame_info_t frame_info;
    lidar_ps_view_config_t view;
    lidar_ps_status_t decode_status;
    int status;

    if (!lane->active_profile.enabled) {
        return XST_SUCCESS;
    }
    if (lane->overwrite_mask != 0U || lane->error_mask != 0U) {
        return XST_FAILURE;
    }
    derived_face = lidar_v2_read_stat(
        &app->device, LIDAR_V2_STAT_DERIVED_FACE);
    planned_shots = (uint16_t)(derived_face >> 16U);
    status = lidar_v2_xilinx_vdma_take_completed(
        lane, planned_shots, &frame_index, &frame);
    if (status == XST_NO_DATA) {
        return XST_SUCCESS;
    }
    if (status != XST_SUCCESS) {
        return status;
    }

    decode_status = lidar_ps_inspect_frame(&frame, &frame_info);
    if (decode_status == LIDAR_PS_OK &&
        (frame_info.slope_rise != 0U) == expect_rise &&
        build_view_config(app, &frame_info, &view) == XST_SUCCESS) {
        decode_status = lidar_ps_decode_and_packetize(
            &frame, &view, lidar_v2_lwip_udp_packet_sink,
            &app->udp, summary);
    } else if (decode_status == LIDAR_PS_OK) {
        decode_status = LIDAR_PS_ERR_GEOMETRY;
    }
    lidar_v2_xilinx_vdma_release_frame(lane, frame_index, &frame);
    return decode_status == LIDAR_PS_OK ? XST_SUCCESS : XST_FAILURE;
}

int lidar_v2_example_app_init(
    lidar_v2_example_app_t *app,
    const lidar_v2_example_parameters_t *parameters)
{
    lidar_v2_commit_result_t commit_result;
    lidar_v2_control_status_t control_status;

    if (app == NULL || parameters == NULL ||
        parameters->rise_vdma == NULL ||
        parameters->rise_frame_addresses == NULL ||
        parameters->frame_store_count == 0U ||
        parameters->frame_capacity_bytes == 0U ||
        parameters->bin_resolution_ps == 0U) {
        return XST_INVALID_PARAM;
    }
    memset(app, 0, sizeof(*app));
    lidar_v2_device_init(&app->device, parameters->csr_base, NULL,
        xilinx_read32, xilinx_write32, xilinx_barrier);
    control_status = lidar_v2_read_build_info(&app->device, &app->build);
    if (control_status != LIDAR_V2_CONTROL_OK) {
        return XST_FAILURE;
    }
    if (lidar_v2_xilinx_vdma_lane_init(&app->rise_lane,
            parameters->rise_vdma, parameters->rise_frame_addresses,
            parameters->frame_store_count, parameters->frame_capacity_bytes,
            app->build.output_width_bits) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    if (parameters->fall_vdma != NULL) {
        if (parameters->fall_frame_addresses == NULL ||
            lidar_v2_xilinx_vdma_lane_init(&app->fall_lane,
                parameters->fall_vdma, parameters->fall_frame_addresses,
                parameters->frame_store_count,
                parameters->frame_capacity_bytes,
                app->build.output_width_bits) != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }
    app->vdma_service.device = &app->device;
    app->vdma_service.rise = &app->rise_lane;
    app->vdma_service.fall = parameters->fall_vdma != NULL ?
        &app->fall_lane : NULL;
    if (lidar_v2_lwip_udp_open(&app->udp,
            &parameters->viewer_address,
            parameters->viewer_udp_port) != 0) {
        return XST_FAILURE;
    }
    app->bin_resolution_ps = parameters->bin_resolution_ps;

    control_status = lidar_v2_stop_and_disarm(&app->device, 1000000U,
        lidar_v2_xilinx_service_vdma_profiles, &app->vdma_service);
    if (control_status != LIDAR_V2_CONTROL_OK ||
        lidar_v2_clear_status(&app->device, 1000000U) !=
            LIDAR_V2_CONTROL_OK ||
        update_runtime_shadow(app, parameters) != XST_SUCCESS) {
        lidar_v2_lwip_udp_close(&app->udp);
        return XST_FAILURE;
    }
    control_status = lidar_v2_commit_shadow(&app->device, 5000000U,
        lidar_v2_xilinx_service_vdma_profiles, &app->vdma_service,
        &commit_result);
    if (control_status != LIDAR_V2_CONTROL_OK) {
        lidar_v2_lwip_udp_close(&app->udp);
        return XST_FAILURE;
    }

    /* Level-high IRQ is conservative on Zynq GIC: clear source, then W1C. */
    lidar_v2_irq_configure(&app->device, LIDAR_V2_IRQ_ALL_MASK, 0U);
    if (lidar_v2_clear_status(&app->device, 1000000U) !=
        LIDAR_V2_CONTROL_OK) {
        lidar_v2_lwip_udp_close(&app->udp);
        return XST_FAILURE;
    }
    lidar_v2_irq_ack(&app->device, LIDAR_V2_IRQ_ALL_MASK);
    control_status = lidar_v2_run_and_arm(&app->device, 1000000U,
        lidar_v2_xilinx_service_vdma_profiles, &app->vdma_service);
    if (control_status != LIDAR_V2_CONTROL_OK) {
        lidar_v2_lwip_udp_close(&app->udp);
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int lidar_v2_example_app_step(lidar_v2_example_app_t *app)
{
    int status;

    if (app == NULL) {
        return XST_INVALID_PARAM;
    }
    app->last_irq_flags |= lidar_v2_irq_pending(&app->device);
    status = process_lane(app, &app->rise_lane, true,
        &app->last_rise_summary);
    if (status != XST_SUCCESS) {
        return status;
    }
    if (app->vdma_service.fall != NULL) {
        status = process_lane(app, &app->fall_lane, false,
            &app->last_fall_summary);
        if (status != XST_SUCCESS) {
            return status;
        }
    }
    return XST_SUCCESS;
}

int lidar_v2_example_acknowledge_irqs(
    lidar_v2_example_app_t *app,
    uint32_t handled_mask,
    bool clear_diagnostic_history)
{
    uint32_t source_level;

    if (app == NULL) {
        return XST_INVALID_PARAM;
    }
    handled_mask &= LIDAR_V2_IRQ_ALL_MASK;
    if (clear_diagnostic_history &&
        lidar_v2_clear_status(&app->device, 1000000U) !=
            LIDAR_V2_CONTROL_OK) {
        return XST_FAILURE;
    }
    source_level = lidar_v2_irq_source_level(&app->device);
    handled_mask &= ~source_level;
    lidar_v2_irq_ack(&app->device, handled_mask);
    app->last_irq_flags &= ~handled_mask;
    return XST_SUCCESS;
}
