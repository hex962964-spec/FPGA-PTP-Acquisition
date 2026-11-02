#include <assert.h>
#include <stdint.h>

#define NET_ACQ_UDP_DATA_DST_MAC_B5 0x10
#define NET_ACQ_UDP_DATA_DST_MAC_B4 0x20
#define NET_ACQ_UDP_DATA_DST_MAC_B3 0x30
#define NET_ACQ_UDP_DATA_DST_MAC_B2 0x40
#define NET_ACQ_UDP_DATA_DST_MAC_B1 0x50
#define NET_ACQ_UDP_DATA_DST_MAC_B0 0x60
#define NET_ACQ_PTP_PEER_MAC_B5 0xAA
#define NET_ACQ_PTP_PEER_MAC_B4 0xBB
#define NET_ACQ_PTP_PEER_MAC_B3 0xCC
#define NET_ACQ_PTP_PEER_MAC_B2 0xDD
#define NET_ACQ_PTP_PEER_MAC_B1 0xEE
#define NET_ACQ_PTP_PEER_MAC_B0 0xF0
#define NET_ACQ_UDP_DATA_DST_IP_A   10
#define NET_ACQ_UDP_DATA_DST_IP_B   1
#define NET_ACQ_UDP_DATA_DST_IP_C   2
#define NET_ACQ_UDP_DATA_DST_IP_D   3
#define NET_ACQ_PTP_PEER_IP_A   192
#define NET_ACQ_PTP_PEER_IP_B   168
#define NET_ACQ_PTP_PEER_IP_C   9
#define NET_ACQ_PTP_PEER_IP_D   77
#define NET_ACQ_PTP_ADDR_SPLIT_ENABLE 1u

#define main net_acq_main_hidden
#include "main.c"
#undef main

int main(void)
{
    net_acq_eth_cfg_t cfg = build_eth_cfg();

    assert(cfg.ptp_addr_split_enable == 1u);
    assert((cfg.dst_mac & 0x0000FFFFFFFFFFFFull) == 0x0000102030405060ull);
    assert((cfg.ptp_peer_mac & 0x0000FFFFFFFFFFFFull) == 0x0000AABBCCDDEEF0ull);
    assert(cfg.dst_ip == ((10u << 24) | (1u << 16) | (2u << 8) | 3u));
    assert(cfg.ptp_peer_ip == ((192u << 24) | (168u << 16) | (9u << 8) | 77u));
    assert(net_acq_eth_validate_config(&cfg) == 0);

    cfg.ptp_peer_mac = 0ull;
    assert(net_acq_eth_validate_config(&cfg) != 0);

    return 0;
}
