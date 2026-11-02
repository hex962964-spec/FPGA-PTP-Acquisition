#include <assert.h>
#include <stdint.h>

#define NET_ACQ_UDP_DATA_DST_MAC_B5 0x10
#define NET_ACQ_UDP_DATA_DST_MAC_B4 0x22
#define NET_ACQ_UDP_DATA_DST_MAC_B3 0x33
#define NET_ACQ_UDP_DATA_DST_MAC_B2 0x44
#define NET_ACQ_UDP_DATA_DST_MAC_B1 0x55
#define NET_ACQ_UDP_DATA_DST_MAC_B0 0x66
#define NET_ACQ_UDP_DATA_DST_IP_A   192
#define NET_ACQ_UDP_DATA_DST_IP_B   168
#define NET_ACQ_UDP_DATA_DST_IP_C   0
#define NET_ACQ_UDP_DATA_DST_IP_D   100

#define main net_acq_main_hidden
#include "main.c"
#undef main

int main(void)
{
    net_acq_eth_cfg_t cfg;

    cfg = build_eth_cfg();

    assert(cfg.ptp_addr_split_enable == 1u);
    assert((cfg.ptp_peer_mac & 0x0000FFFFFFFFFFFFull) == 0x04CB01DAAADAull);
    assert(cfg.ptp_peer_ip == NET_ACQ_MAKE_IP4(192, 168, 0, 3));
    assert((cfg.dst_mac & 0x0000FFFFFFFFFFFFull) == 0x102233445566ull);
    assert(cfg.dst_ip == NET_ACQ_MAKE_IP4(192, 168, 0, 100));
    assert(cfg.ptp_peer_mac != cfg.dst_mac);
    assert(cfg.ptp_peer_ip != cfg.dst_ip);
    assert(net_acq_eth_validate_config(&cfg) == 0);

    return 0;
}
