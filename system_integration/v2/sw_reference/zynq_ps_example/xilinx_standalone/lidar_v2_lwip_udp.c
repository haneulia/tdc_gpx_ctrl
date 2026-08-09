#include "lidar_v2_lwip_udp.h"

#include <string.h>

#include "lwip/err.h"
#include "lwip/pbuf.h"

int lidar_v2_lwip_udp_open(
    lidar_v2_lwip_udp_t *transport,
    const ip_addr_t *remote_address,
    uint16_t remote_port)
{
    if (transport == NULL || remote_address == NULL || remote_port == 0U) {
        return -1;
    }
    memset(transport, 0, sizeof(*transport));
    transport->pcb = udp_new_ip_type(IP_GET_TYPE(remote_address));
    if (transport->pcb == NULL) {
        return -1;
    }
    ip_addr_copy(transport->remote_address, *remote_address);
    transport->remote_port = remote_port;
    return 0;
}

void lidar_v2_lwip_udp_close(lidar_v2_lwip_udp_t *transport)
{
    if (transport != NULL && transport->pcb != NULL) {
        udp_remove(transport->pcb);
        transport->pcb = NULL;
    }
}

int lidar_v2_lwip_udp_packet_sink(
    const uint8_t *payload,
    size_t payload_bytes,
    void *context)
{
    lidar_v2_lwip_udp_t *transport = (lidar_v2_lwip_udp_t *)context;
    struct pbuf *packet;
    err_t status;

    if (transport == NULL || transport->pcb == NULL || payload == NULL ||
        payload_bytes == 0U || payload_bytes > 1440U) {
        return -1;
    }
    packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)payload_bytes, PBUF_RAM);
    if (packet == NULL) {
        ++transport->error_count;
        return -1;
    }
    status = pbuf_take(packet, payload, payload_bytes);
    if (status == ERR_OK) {
        status = udp_sendto(transport->pcb, packet,
            &transport->remote_address, transport->remote_port);
    }
    pbuf_free(packet);
    if (status != ERR_OK) {
        ++transport->error_count;
        return -1;
    }
    ++transport->packet_count;
    return 0;
}
