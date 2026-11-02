#include <stdio.h>
#include <stdint.h>
#include "net_acq_hal.h"

static uint64_t net_acq_read64_consistent(uint32_t off_lo, uint32_t off_hi)
{
    uint32_t hi1;
    uint32_t lo;
    uint32_t hi2;

    do {
        hi1 = net_acq_read(off_hi);
        lo  = net_acq_read(off_lo);
        hi2 = net_acq_read(off_hi);
    } while (hi1 != hi2);

    return ((uint64_t)hi2 << 32) | (uint64_t)lo;
}

static uint64_t net_acq_read64_lo_hi(uint32_t off_lo, uint32_t off_hi)
{
    uint32_t lo;
    uint32_t hi;

    lo = net_acq_read(off_lo);
    hi = net_acq_read(off_hi);
    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

static uint16_t net_acq_effective_spp_internal(uint32_t samples_per_packet)
{
    uint32_t spp;

    spp = samples_per_packet;
    if (spp == 0u) {
        spp = 1u;
    }
    if (spp > 256u) {
        spp = 256u;
    }
    return (uint16_t)spp;
}

static void print_mac48(uint64_t mac)
{
    printf("%02x:%02x:%02x:%02x:%02x:%02x",
           (unsigned int)((mac >> 40) & 0xFFu),
           (unsigned int)((mac >> 32) & 0xFFu),
           (unsigned int)((mac >> 24) & 0xFFu),
           (unsigned int)((mac >> 16) & 0xFFu),
           (unsigned int)((mac >>  8) & 0xFFu),
           (unsigned int)( mac        & 0xFFu));
}

static void print_ip4(uint32_t ip)
{
    printf("%u.%u.%u.%u",
           (unsigned int)((ip >> 24) & 0xFFu),
           (unsigned int)((ip >> 16) & 0xFFu),
           (unsigned int)((ip >>  8) & 0xFFu),
           (unsigned int)( ip        & 0xFFu));
}

static void net_acq_print_u80_hex_internal(const net_acq_u80_t *value)
{
    if (!value) {
        printf("0x0000_00000000_00000000");
        return;
    }

    printf("0x%04x_%08x_%08x",
           (unsigned int)value->hi16,
           (unsigned int)(value->lo64 >> 32),
           (unsigned int)(value->lo64 & 0xFFFFFFFFu));
}

static int net_acq_wait_eth_link_up(uint32_t poll_delay_cycles, uint32_t max_polls)
{
    uint32_t i;

    if (net_acq_eth_link_is_up()) {
        return 0;
    }

    for (i = 0u; i < max_polls; i++) {
        if (net_acq_eth_link_is_up()) {
            return 0;
        }
        net_acq_delay_cycles(poll_delay_cycles);
    }

    return -177;
}

static int net_acq_verify_front_shadow(const net_acq_phase_cfg_t *cfg)
{
    net_acq_snapshot_t snap;
    uint16_t expect_spp;
    uint32_t expect_ctrl;

    if (!cfg) {
        return -100;
    }

    expect_spp  = net_acq_effective_spp_internal(cfg->samples_per_packet);
    expect_ctrl = net_acq_ctrl_build(0u, cfg->use_external_source);

    net_acq_read_snapshot(&snap);

    if ((snap.ctrl & (CTRL_ENABLE | CTRL_SRC_SEL_EXT)) != expect_ctrl) {
        printf("[strict][front] CTRL mismatch, got=0x%08x expect_masked=0x%08x\n",
               snap.ctrl, expect_ctrl);
        return -101;
    }
    if ((snap.mode & 0xFu) != (cfg->mode & 0xFu)) {
        printf("[strict][front] MODE mismatch, got=%u expect=%u\n",
               (unsigned int)(snap.mode & 0xFu),
               (unsigned int)(cfg->mode & 0xFu));
        return -102;
    }
    if (snap.sample_period != cfg->sample_period) {
        printf("[strict][front] SAMPLE_PERIOD mismatch, got=%u expect=%u\n",
               (unsigned int)snap.sample_period,
               (unsigned int)cfg->sample_period);
        return -103;
    }
    if ((snap.samples_per_packet & 0xFFFFu) != (uint32_t)expect_spp) {
        printf("[strict][front] SAMPLES_PER_PKT mismatch, got=%u expect=%u\n",
               (unsigned int)(snap.samples_per_packet & 0xFFFFu),
               (unsigned int)expect_spp);
        return -104;
    }
    if ((snap.channel_id & 0xFFFFu) != (cfg->channel_id & 0xFFFFu)) {
        printf("[strict][front] CHANNEL_ID mismatch, got=%u expect=%u\n",
               (unsigned int)(snap.channel_id & 0xFFFFu),
               (unsigned int)(cfg->channel_id & 0xFFFFu));
        return -105;
    }

    return 0;
}

static int net_acq_verify_eth_features(uint32_t require_backend,
                                       uint32_t require_ptp_addr_split)
{
    uint32_t features;
    uint32_t ptp_features;

    features = net_acq_read(NET_ACQ_ETH_FEATURES);

    if (NET_ACQ_ETH_FEATURE_VERSION(features) == 0u) {
        printf("[strict][eth] ETH_FEATURES.version is zero\n");
        return -110;
    }

    if (require_backend) {
        if ((features & ETH_FEATURE_BACKEND_PRESENT) == 0u) {
            printf("[strict][eth] backend feature missing\n");
            return -111;
        }
        if ((features & ETH_FEATURE_IPV4_UDP_TX_PRESENT) == 0u) {
            printf("[strict][eth] ipv4/udp tx feature missing\n");
            return -112;
        }
        if ((features & ETH_FEATURE_COMMIT_SHADOW) == 0u) {
            printf("[strict][eth] shadow commit feature missing\n");
            return -113;
        }
    }

    if (require_ptp_addr_split) {
        if ((features & ETH_FEATURE_PTP_ADDR_SPLIT_PRESENT) == 0u) {
            printf("[strict][eth] ETH_FEATURES missing dedicated PTP peer split support\n");
            return -114;
        }
        ptp_features = net_acq_read(NET_ACQ_PTP_FEATURES);
        if (NET_ACQ_PTP_FEATURE_VERSION(ptp_features) == 0u) {
            printf("[strict][ptp] PTP_FEATURES.version is zero while addr split is requested\n");
            return -115;
        }
        if ((ptp_features & PTP_FEATURE_ADDR_SPLIT_PRESENT) == 0u) {
            printf("[strict][ptp] PTP_FEATURES missing addr split support\n");
            return -116;
        }
    }

    return 0;
}
static int net_acq_verify_eth_shadow(const net_acq_eth_cfg_t *cfg)
{
    net_acq_eth_snapshot_t snap;
    uint32_t expect_ctrl;

    if (!cfg) {
        return 0;
    }

    net_acq_read_eth_snapshot(&snap);

    expect_ctrl = net_acq_eth_ctrl_build(cfg->backend_enable,
                                         cfg->tx_mirror_enable,
                                         cfg->append_fcs,
                                         cfg->udp_checksum_enable,
                                         cfg->stall_drop_enable,
                                         0u);

    if ((snap.eth_ctrl & 0x3Du) != (expect_ctrl & 0x3Du)) {
        printf("[strict][eth] ETH_CTRL mismatch, got=0x%08x expect=0x%08x\n",
               snap.eth_ctrl, expect_ctrl);
        return -120;
    }
    if ((snap.local_mac & 0x0000FFFFFFFFFFFFull) != (cfg->local_mac & 0x0000FFFFFFFFFFFFull)) {
        printf("[strict][eth] LOCAL_MAC mismatch\n");
        return -121;
    }
    if ((snap.dst_mac & 0x0000FFFFFFFFFFFFull) != (cfg->dst_mac & 0x0000FFFFFFFFFFFFull)) {
        printf("[strict][eth] DST_MAC mismatch\n");
        return -122;
    }
    if (snap.local_ip != cfg->local_ip) {
        printf("[strict][eth] LOCAL_IP mismatch\n");
        return -123;
    }
    if (snap.dst_ip != cfg->dst_ip) {
        printf("[strict][eth] DST_IP mismatch\n");
        return -124;
    }
    if ((snap.ptp_addr_split_enable ? 1u : 0u) != (cfg->ptp_addr_split_enable ? 1u : 0u)) {
        printf("[strict][eth] PTP_ADDR_CFG mismatch\n");
        return -132;
    }
    if ((snap.ptp_peer_mac & 0x0000FFFFFFFFFFFFull) != (cfg->ptp_peer_mac & 0x0000FFFFFFFFFFFFull)) {
        printf("[strict][eth] PTP_PEER_MAC mismatch\n");
        return -133;
    }
    if (snap.ptp_peer_ip != cfg->ptp_peer_ip) {
        printf("[strict][eth] PTP_PEER_IP mismatch\n");
        return -134;
    }
    if (snap.src_port != cfg->src_port || snap.dst_port != cfg->dst_port) {
        printf("[strict][eth] UDP_PORTS mismatch\n");
        return -125;
    }
    if (snap.ttl != cfg->ttl || snap.dscp_ecn != cfg->dscp_ecn || snap.ip_ident_seed != cfg->ip_ident_seed) {
        printf("[strict][eth] IPV4_CFG mismatch\n");
        return -126;
    }
    if ((snap.ptp_cfg & 0x0000FF0Fu) !=
        ((((uint32_t)cfg->ptp_domain & 0xFFu) << 8) |
         (cfg->ptp_enable ? PTP_CFG_ENABLE : 0u) |
         (cfg->ptp_one_step ? PTP_CFG_ONE_STEP : 0u) |
         (cfg->ptp_two_step ? PTP_CFG_TWO_STEP : 0u) |
         (cfg->ptp_event_priority_enable ? PTP_CFG_EVENT_PRIORITY_EN : 0u))) {
        printf("[strict][eth] PTP_CFG mismatch\n");
        return -127;
    }
    if (snap.ptp_clock_id != cfg->ptp_clock_id) {
        printf("[strict][eth] PTP_CLOCK_ID mismatch\n");
        return -128;
    }
    if (snap.ptp_port_num != cfg->ptp_port_num) {
        printf("[strict][eth] PTP_PORT_NUM mismatch\n");
        return -129;
    }
    if ((snap.eth_status & ETH_STATUS_CFG_ACTIVE) == 0u) {
        printf("[strict][eth] cfg_active is not set after commit\n");
        return -130;
    }
    if (((snap.eth_status & ETH_STATUS_BACKEND_SELECTED) ? 1u : 0u) != (cfg->backend_enable ? 1u : 0u)) {
        printf("[strict][eth] backend_selected mismatch after commit\n");
        return -131;
    }

    return 0;
}

static int net_acq_check_runtime_strict(const net_acq_phase_cfg_t *cfg,
                                        const net_acq_eth_cfg_t *eth_cfg,
                                        const net_acq_debug_snapshot_t *dbg,
                                        const net_acq_eth_snapshot_t *eth)
{
    uint16_t exp_udp_len;
    uint16_t exp_ip_len;

    if (!cfg || !dbg) {
        return -140;
    }

    if ((dbg->dbg_status & (DBG_STATUS_ERROR_OR | DBG_STATUS_CORE_ERROR_RAW | DBG_STATUS_SINK_ERROR_STICKY)) != 0u) {
        printf("[strict][dbg] DBG_STATUS indicates error, dbg_status=0x%08x\n", dbg->dbg_status);
        return -141;
    }
    if (dbg->dbg_bad_packet_count != 0u) {
        printf("[strict][dbg] BAD_PKT_CNT is non-zero, value=%u\n",
               (unsigned int)dbg->dbg_bad_packet_count);
        return -142;
    }
    if (dbg->dbg_last_error_code != NET_ACQ_ERR_NONE) {
        printf("[strict][dbg] LAST_ERROR is non-zero, code=%u\n",
               (unsigned int)dbg->dbg_last_error_code);
        return -143;
    }

    if (eth_cfg && eth_cfg->backend_enable) {
        if (!eth) {
            return -144;
        }

        if ((eth->eth_status & ETH_STATUS_CFG_ACTIVE) == 0u) {
            printf("[strict][eth] cfg_active dropped during runtime\n");
            return -145;
        }
        if ((eth->eth_status & ETH_STATUS_BACKEND_SELECTED) == 0u) {
            printf("[strict][eth] backend_selected dropped during runtime\n");
            return -146;
        }
        if (cfg->require_eth_link_up && ((eth->eth_status & ETH_STATUS_LINK_UP) == 0u)) {
            printf("[strict][eth] link_up is required but not asserted\n");
            return -152;
        }
        if ((eth->eth_status & (ETH_STATUS_TX_ERROR_STICKY |
                                ETH_STATUS_PKT_BUF_OVERFLOW |
                                ETH_STATUS_MAC_STALL_TIMEOUT)) != 0u) {
            printf("[strict][eth] ETH_STATUS indicates sticky error, eth_status=0x%08x\n",
                   eth->eth_status);
            return -147;
        }
        if (eth->drop_count != 0u) {
            printf("[strict][eth] DROP_COUNT is non-zero, value=%u\n",
                   (unsigned int)eth->drop_count);
            return -148;
        }
        if (eth->last_error_code != NET_ACQ_ETH_ERR_NONE) {
            printf("[strict][eth] LAST_ERROR is non-zero, code=%u\n",
                   (unsigned int)eth->last_error_code);
            return -149;
        }

        exp_udp_len = net_acq_expected_udp_length(cfg->samples_per_packet);
        exp_ip_len  = net_acq_expected_ip_total_length(cfg->samples_per_packet);
        if (eth->tx_packet_count != 0u) {
            if (eth->last_udp_length != exp_udp_len) {
                printf("[strict][eth] UDP length mismatch, got=%u expect=%u\n",
                       (unsigned int)eth->last_udp_length,
                       (unsigned int)exp_udp_len);
                return -150;
            }
            if (eth->last_ip_total_length != exp_ip_len) {
                printf("[strict][eth] IP total length mismatch, got=%u expect=%u\n",
                       (unsigned int)eth->last_ip_total_length,
                       (unsigned int)exp_ip_len);
                return -151;
            }
        }
    }

    return 0;
}

void net_acq_delay_cycles(volatile uint32_t cycles)
{
    while (cycles--) {
        __asm__ volatile ("nop");
    }
}

uint32_t net_acq_ctrl_build(uint32_t enable, uint32_t use_external_source)
{
    uint32_t ctrl;

    ctrl = 0u;
    if (enable) {
        ctrl |= CTRL_ENABLE;
    }
    if (use_external_source) {
        ctrl |= CTRL_SRC_SEL_EXT;
    }

    return ctrl;
}

void net_acq_reg_write(uint32_t addr, uint32_t value)
{
#ifdef NET_ACQ_UNIT_TEST
    extern void net_acq_test_reg_write(uint32_t addr, uint32_t value);
    net_acq_test_reg_write(addr, value);
#else
    *(volatile uint32_t *)(uintptr_t)addr = value;
#endif
}

uint32_t net_acq_reg_read(uint32_t addr)
{
    return *(volatile uint32_t *)(uintptr_t)addr;
}

void net_acq_write(uint32_t offset, uint32_t value)
{
    net_acq_reg_write(NET_ACQ_BASE + offset, value);
}

uint32_t net_acq_read(uint32_t offset)
{
    return net_acq_reg_read(NET_ACQ_BASE + offset);
}

void net_acq_disable(uint32_t use_external_source)
{
    net_acq_write(NET_ACQ_CTRL, net_acq_ctrl_build(0u, use_external_source));
}

void net_acq_enable(uint32_t use_external_source)
{
    net_acq_write(NET_ACQ_CTRL, net_acq_ctrl_build(1u, use_external_source));
}

void net_acq_soft_reset(uint32_t keep_enable, uint32_t use_external_source)
{
    net_acq_write(NET_ACQ_CTRL,
                  net_acq_ctrl_build(keep_enable, use_external_source) |
                  CTRL_SOFT_RESET);
}

void net_acq_clear_status(uint32_t keep_enable, uint32_t use_external_source)
{
    net_acq_write(NET_ACQ_CTRL,
                  net_acq_ctrl_build(keep_enable, use_external_source) |
                  CTRL_CLEAR_STATUS);
}

void net_acq_phy_restart_autoneg(void)
{
    net_acq_write(NET_ACQ_PHY_CTRL, PHY_CTRL_RESTART_AN_PULSE);
}

void net_acq_phy_soft_reset(void)
{
    net_acq_write(NET_ACQ_PHY_CTRL, PHY_CTRL_SOFT_RESET_PULSE);
}

void net_acq_set_time_quality(uint32_t state)
{
    net_acq_write(NET_ACQ_TIME_QUALITY, state & 0x3u);
}

void net_acq_apply_config(uint32_t mode,
                          uint32_t sample_period,
                          uint32_t samples_per_packet,
                          uint32_t channel_id,
                          uint32_t use_external_source)
{
    uint16_t spp;

    spp = net_acq_effective_spp_internal(samples_per_packet);
    if (samples_per_packet == 0u) {
        printf("[cfg][front] samples_per_packet=0 is unsafe, force to 1\n");
    }
    if (samples_per_packet > 256u) {
        printf("[cfg][front] samples_per_packet=%u exceeds sample_fifo depth, clamp to 256\n",
               (unsigned int)samples_per_packet);
    }

    net_acq_write(NET_ACQ_MODE, mode & 0xFu);
    net_acq_write(NET_ACQ_SAMPLE_PERIOD, sample_period);
    net_acq_write(NET_ACQ_SAMPLES_PER_PKT, (uint32_t)spp & 0xFFFFu);
    net_acq_write(NET_ACQ_CHANNEL_ID, channel_id & 0xFFFFu);
    net_acq_write(NET_ACQ_CTRL, net_acq_ctrl_build(0u, use_external_source));
}

uint64_t net_acq_last_timestamp(void)
{
    return net_acq_read64_consistent(NET_ACQ_LAST_TS_L, NET_ACQ_LAST_TS_H);
}

uint64_t net_acq_debug_last_timestamp(void)
{
    return net_acq_read64_consistent(NET_ACQ_DBG_LAST_TS_L, NET_ACQ_DBG_LAST_TS_H);
}

uint32_t net_acq_sample_fifo_level(uint32_t fifo_level)
{
    return NET_ACQ_FIFO_SAMPLE_LEVEL(fifo_level);
}

uint32_t net_acq_info_fifo_level(uint32_t fifo_level)
{
    return NET_ACQ_FIFO_INFO_LEVEL(fifo_level);
}

uint16_t net_acq_effective_samples_per_packet(uint32_t samples_per_packet)
{
    return net_acq_effective_spp_internal(samples_per_packet);
}

uint16_t net_acq_expected_udp_length(uint32_t samples_per_packet)
{
    uint16_t eff_spp;

    eff_spp = net_acq_effective_spp_internal(samples_per_packet);
    return (uint16_t)(8u + (uint16_t)((10u + (uint32_t)eff_spp) * 2u));
}

uint16_t net_acq_expected_ip_total_length(uint32_t samples_per_packet)
{
    return (uint16_t)(20u + net_acq_expected_udp_length(samples_per_packet));
}

const char *net_acq_source_name(uint32_t use_external_source)
{
    return use_external_source ? "EXTERNAL_SAMPLE_IN" : "INTERNAL_DATA_GEN";
}

const char *net_acq_mode_name(uint32_t mode)
{
    switch (mode & 0xFu) {
    case NET_ACQ_MODE_SAWTOOTH:
        return "SAWTOOTH";
    case NET_ACQ_MODE_TRIANGLE:
        return "TRIANGLE";
    case NET_ACQ_MODE_SQUARE:
        return "SQUARE";
    case NET_ACQ_MODE_CONSTANT:
        return "CONSTANT";
    default:
        return "RESERVED";
    }
}

const char *net_acq_debug_error_name(uint16_t err_code)
{
    switch (err_code) {
    case NET_ACQ_ERR_NONE:
        return "NONE";
    case NET_ACQ_ERR_MAGIC:
        return "MAGIC_MISMATCH";
    case NET_ACQ_ERR_VERSION:
        return "VERSION_MISMATCH";
    case NET_ACQ_ERR_EARLY_LAST:
        return "EARLY_LAST";
    case NET_ACQ_ERR_SEQ_MISMATCH:
        return "SEQ_MISMATCH";
    case NET_ACQ_ERR_TS_NON_MONO:
        return "TIMESTAMP_NON_MONOTONIC";
    case NET_ACQ_ERR_MISSING_LAST:
        return "MISSING_LAST";
    default:
        return "UNKNOWN";
    }
}

void net_acq_read_snapshot(net_acq_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    snap->ctrl               = net_acq_read(NET_ACQ_CTRL);
    snap->mode               = net_acq_read(NET_ACQ_MODE);
    snap->sample_period      = net_acq_read(NET_ACQ_SAMPLE_PERIOD);
    snap->samples_per_packet = net_acq_read(NET_ACQ_SAMPLES_PER_PKT);
    snap->channel_id         = net_acq_read(NET_ACQ_CHANNEL_ID);
    snap->status             = net_acq_read(NET_ACQ_STATUS);
    snap->packet_count       = net_acq_read(NET_ACQ_PACKET_COUNT);
    snap->fifo_level         = net_acq_read(NET_ACQ_FIFO_LEVEL);
    snap->last_timestamp     = net_acq_last_timestamp();
}

void net_acq_dump_snapshot(const net_acq_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[front] CTRL            = 0x%08x (enable=%u, soft_reset=%u, clear_status=%u, src_sel_ext=%u)\n",
           snap->ctrl,
           (snap->ctrl & CTRL_ENABLE) ? 1u : 0u,
           (snap->ctrl & CTRL_SOFT_RESET) ? 1u : 0u,
           (snap->ctrl & CTRL_CLEAR_STATUS) ? 1u : 0u,
           (snap->ctrl & CTRL_SRC_SEL_EXT) ? 1u : 0u);
    printf("[front] MODE            = 0x%08x (%s)\n",
           snap->mode, net_acq_mode_name(snap->mode));
    printf("[front] SAMPLE_PERIOD   = 0x%08x (%u)\n",
           snap->sample_period, snap->sample_period);
    printf("[front] SAMPLES_PER_PKT = 0x%08x (%u)\n",
           snap->samples_per_packet, snap->samples_per_packet & 0xFFFFu);
    printf("[front] CHANNEL_ID      = 0x%08x (%u)\n",
           snap->channel_id, snap->channel_id & 0xFFFFu);
    printf("[front] STATUS          = 0x%08x (busy=%u, error=%u)\n",
           snap->status,
           (snap->status & STATUS_BUSY) ? 1u : 0u,
           (snap->status & STATUS_ERROR) ? 1u : 0u);
    printf("[front] PACKET_COUNT    = %u\n", snap->packet_count);
    printf("[front] FIFO_LEVEL      = 0x%08x (sample_fifo=%u, info_fifo=%u)\n",
           snap->fifo_level,
           net_acq_sample_fifo_level(snap->fifo_level),
           net_acq_info_fifo_level(snap->fifo_level));
    printf("[front] LAST_TIMESTAMP  = 0x%08x_%08x\n",
           (uint32_t)(snap->last_timestamp >> 32),
           (uint32_t)(snap->last_timestamp & 0xFFFFFFFFu));
}

void net_acq_read_debug_snapshot(net_acq_debug_snapshot_t *snap)
{
    uint32_t last_counts;
    uint32_t last_error;

    if (!snap) {
        return;
    }

    snap->dbg_status              = net_acq_read(NET_ACQ_DBG_STATUS);
    snap->dbg_good_packet_count   = net_acq_read(NET_ACQ_DBG_GOOD_PKT_CNT);
    snap->dbg_bad_packet_count    = net_acq_read(NET_ACQ_DBG_BAD_PKT_CNT);
    snap->dbg_last_seq_id         = net_acq_read(NET_ACQ_DBG_LAST_SEQ_ID);
    snap->dbg_last_timestamp      = net_acq_debug_last_timestamp();

    last_counts = net_acq_read(NET_ACQ_DBG_LAST_COUNTS);
    last_error  = net_acq_read(NET_ACQ_DBG_LAST_ERROR);

    snap->dbg_last_sample_count     = NET_ACQ_DBG_LAST_SAMPLE_COUNT(last_counts);
    snap->dbg_last_payload_count    = NET_ACQ_DBG_LAST_PAYLOAD_COUNT(last_counts);
    snap->dbg_last_error_code       = NET_ACQ_DBG_LAST_ERROR_CODE(last_error);
    snap->dbg_last_error_word_index = NET_ACQ_DBG_LAST_ERROR_WORD_INDEX(last_error);
}

void net_acq_dump_debug_snapshot(const net_acq_debug_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[dbg ] DBG_STATUS       = 0x%08x (busy=%u, error_or=%u, core_err=%u, sink_err=%u)\n",
           snap->dbg_status,
           (snap->dbg_status & DBG_STATUS_BUSY) ? 1u : 0u,
           (snap->dbg_status & DBG_STATUS_ERROR_OR) ? 1u : 0u,
           (snap->dbg_status & DBG_STATUS_CORE_ERROR_RAW) ? 1u : 0u,
           (snap->dbg_status & DBG_STATUS_SINK_ERROR_STICKY) ? 1u : 0u);
    printf("[dbg ] GOOD_PKT_CNT     = %u\n", snap->dbg_good_packet_count);
    printf("[dbg ] BAD_PKT_CNT      = %u\n", snap->dbg_bad_packet_count);
    printf("[dbg ] LAST_SEQ_ID      = %u (0x%08x)\n",
           snap->dbg_last_seq_id, snap->dbg_last_seq_id);
    printf("[dbg ] LAST_TS          = 0x%08x_%08x\n",
           (uint32_t)(snap->dbg_last_timestamp >> 32),
           (uint32_t)(snap->dbg_last_timestamp & 0xFFFFFFFFu));
    printf("[dbg ] LAST_COUNTS      = sample_count=%u, payload_count=%u\n",
           (unsigned int)snap->dbg_last_sample_count,
           (unsigned int)snap->dbg_last_payload_count);
    printf("[dbg ] LAST_ERROR       = code=%u(%s), word_index=%u\n",
           (unsigned int)snap->dbg_last_error_code,
           net_acq_debug_error_name(snap->dbg_last_error_code),
           (unsigned int)snap->dbg_last_error_word_index);
}

uint32_t net_acq_eth_ctrl_build(uint32_t backend_enable,
                                uint32_t tx_mirror_enable,
                                uint32_t append_fcs,
                                uint32_t udp_checksum_enable,
                                uint32_t stall_drop_enable,
                                uint32_t pulse_commit)
{
    uint32_t ctrl;

    ctrl = 0u;
    if (backend_enable) {
        ctrl |= ETH_CTRL_BACKEND_ENABLE;
    }
    if (pulse_commit) {
        ctrl |= ETH_CTRL_COMMIT_PULSE;
    }
    if (tx_mirror_enable) {
        ctrl |= ETH_CTRL_TX_MIRROR_ENABLE;
    }
    if (append_fcs) {
        ctrl |= ETH_CTRL_APPEND_FCS;
    }
    if (udp_checksum_enable) {
        ctrl |= ETH_CTRL_UDP_CHECKSUM_EN;
    }
    if (stall_drop_enable) {
        ctrl |= ETH_CTRL_STALL_DROP_ENABLE;
    }

    return ctrl;
}


uint32_t net_acq_txs_ctrl_build(uint32_t enable,
                                uint32_t block_when_tod_invalid,
                                uint32_t strict_window)
{
    uint32_t ctrl;

    ctrl = 0u;
    if (enable) {
        ctrl |= NET_ACQ_TXS_CTRL_ENABLE;
    }
    if (block_when_tod_invalid) {
        ctrl |= NET_ACQ_TXS_CTRL_BLOCK_WHEN_TOD_INVALID;
    }
    if (strict_window) {
        ctrl |= NET_ACQ_TXS_CTRL_STRICT_WINDOW;
    }
    return ctrl;
}

int net_acq_txsched_validate_config(const net_acq_txsched_cfg_t *cfg)
{
    if (!cfg) {
        return 0;
    }

    if (cfg->period_ns == 0u) {
        printf("[cfg][txs] period_ns is zero\n");
        return -210;
    }
    if (cfg->slot_width_ns == 0u) {
        printf("[cfg][txs] slot_width_ns is zero\n");
        return -211;
    }
    if (cfg->slot_offset_ns >= cfg->period_ns) {
        printf("[cfg][txs] slot_offset_ns (%u) must be < period_ns (%u)\n",
               (unsigned int)cfg->slot_offset_ns,
               (unsigned int)cfg->period_ns);
        return -212;
    }
    if (cfg->slot_width_ns > cfg->period_ns) {
        printf("[cfg][txs] slot_width_ns (%u) must be <= period_ns (%u)\n",
               (unsigned int)cfg->slot_width_ns,
               (unsigned int)cfg->period_ns);
        return -213;
    }
    return 0;
}

void net_acq_txsched_write_shadow(const net_acq_txsched_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }

    net_acq_write(NET_ACQ_TXS_BASE_TIME_L, (uint32_t)(cfg->base_time_ns & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_TXS_BASE_TIME_H, (uint32_t)((cfg->base_time_ns >> 32) & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_TXS_PERIOD_NS, cfg->period_ns);
    net_acq_write(NET_ACQ_TXS_SLOT_OFFSET_NS, cfg->slot_offset_ns);
    net_acq_write(NET_ACQ_TXS_SLOT_WIDTH_NS, cfg->slot_width_ns);
    net_acq_write(NET_ACQ_TXS_ADVANCE_NS, cfg->advance_ns);
    net_acq_write(NET_ACQ_TXS_LATE_TOL_NS, cfg->late_tol_ns);
    net_acq_write(NET_ACQ_TXS_CTRL,
                  net_acq_txs_ctrl_build(cfg->enable,
                                         cfg->block_when_tod_invalid,
                                         cfg->strict_window));
}

uint32_t net_acq_ptp_cfg_build(uint32_t enable,
                               uint32_t one_step,
                               uint32_t two_step,
                               uint32_t event_priority_enable,
                               uint8_t domain)
{
    uint32_t v;

    v = (((uint32_t)domain & 0xFFu) << 8);
    if (enable) {
        v |= PTP_CFG_ENABLE;
    }
    if (one_step) {
        v |= PTP_CFG_ONE_STEP;
    }
    if (two_step) {
        v |= PTP_CFG_TWO_STEP;
    }
    if (event_priority_enable) {
        v |= PTP_CFG_EVENT_PRIORITY_EN;
    }
    return v;
}

uint32_t net_acq_ptp_ctrl_build(uint32_t tod_enable,
                                uint32_t tod_hold,
                                uint32_t txstamp_pop_pulse,
                                uint32_t tod_set_pulse,
                                uint32_t tod_step_pulse,
                                uint32_t tod_freq_pulse,
                                uint32_t tx_trigger_pulse,
                                uint32_t tx_need_timestamp,
                                uint8_t tx_message_type,
                                uint8_t tx_transport_specific,
                                uint16_t tx_sequence_id)
{
    uint32_t v;
    uint32_t active_need_timestamp;
    uint8_t  active_msg_type;

    /*
     * 当前 RTL 的 TXSTAMP 清除语义不是 PTP_CTRL.bit2 pop，
     * 而是按 TXSTAMP_L/TXSTAMP_H 的 low->high 读出顺序消费。
     * 因此这里继续保留 txstamp_pop_pulse 形参只为兼容已有调用点，
     * 但不再把它编码进 PTP_CTRL。
     */
    (void)txstamp_pop_pulse;

    /*
     * 与最新 RTL 对齐：比赛版主动 TX 已固定收口到
     *   Delay_Req + need_timestamp=1
     * 软件仍可通过 tspec / sequence_id 控制命令细节，但 msg_type / bit7
     * 不再由软件自由选择；读回时 CSR 也会固定反映这个策略。
     */
    (void)tx_need_timestamp;
    (void)tx_message_type;
    active_need_timestamp = 1u;
    active_msg_type       = (uint8_t)NET_ACQ_PTP_MSG_DELAY_REQ;

    v = 0u;
    if (tod_enable) {
        v |= PTP_CTRL_TOD_ENABLE;
    }
    if (tod_hold) {
        v |= PTP_CTRL_TOD_HOLD;
    }
    if (tod_set_pulse) {
        v |= PTP_CTRL_TOD_SET_PULSE;
    }
    if (tod_step_pulse) {
        v |= PTP_CTRL_TOD_STEP_PULSE;
    }
    if (tod_freq_pulse) {
        v |= PTP_CTRL_TOD_FREQ_PULSE;
    }
    if (tx_trigger_pulse) {
        v |= PTP_CTRL_TX_TRIGGER_PULSE;
    }
    if (active_need_timestamp) {
        v |= PTP_CTRL_TX_NEED_TIMESTAMP;
    }
    v |= (((uint32_t)active_msg_type & 0x0Fu) << 8);
    v |= (((uint32_t)tx_transport_specific & 0x0Fu) << 12);
    v |= (((uint32_t)tx_sequence_id & 0xFFFFu) << 16);
    return v;
}

uint32_t net_acq_ptp_tx_cfg_build(uint16_t tx_flags,
                                  int8_t tx_log_message_interval)
{
    uint32_t v;

    v  = ((uint32_t)tx_flags & 0xFFFFu);
    v |= (((uint32_t)((uint8_t)tx_log_message_interval)) << 16);
    return v;
}

void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }

    net_acq_write(NET_ACQ_PTP_CFG,
                  net_acq_ptp_cfg_build(cfg->enable,
                                        cfg->one_step,
                                        cfg->two_step,
                                        cfg->event_priority_enable,
                                        cfg->domain));
    net_acq_write(NET_ACQ_PTP_CLOCK_ID_L, (uint32_t)(cfg->clock_id & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_CLOCK_ID_H, (uint32_t)((cfg->clock_id >> 32) & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_PORT_NUM,   (uint32_t)(cfg->port_num & 0xFFFFu));
    net_acq_write(NET_ACQ_PTP_TOD_SET_L,  (uint32_t)(cfg->tod_set_ns & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_TOD_SET_H,  (uint32_t)((cfg->tod_set_ns >> 32) & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_STEP_NS,   (uint32_t)cfg->tod_step_ns);
    net_acq_write(NET_ACQ_PTP_FREQ_ADJ,   (uint32_t)cfg->tod_freq_adj_ppb);
    net_acq_write(NET_ACQ_PTP_TX_CFG,
                  net_acq_ptp_tx_cfg_build(cfg->tx_flags,
                                            cfg->tx_log_message_interval));
    net_acq_write(NET_ACQ_PTP_CTRL,
                  net_acq_ptp_ctrl_build(cfg->tod_enable,
                                         cfg->tod_hold,
                                         0u,
                                         0u,
                                         0u,
                                         0u,
                                         0u,
                                         cfg->tx_need_timestamp,
                                         cfg->tx_message_type,
                                         cfg->tx_transport_specific,
                                         cfg->tx_sequence_id));
}

void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *cfg,
                            uint32_t txstamp_pop_pulse,
                            uint32_t tod_set_pulse,
                            uint32_t tod_step_pulse,
                            uint32_t tod_freq_pulse,
                            uint32_t tx_trigger_pulse)
{
    if (!cfg) {
        return;
    }

    net_acq_write(NET_ACQ_PTP_CTRL,
                  net_acq_ptp_ctrl_build(cfg->tod_enable,
                                         cfg->tod_hold,
                                         txstamp_pop_pulse,
                                         tod_set_pulse,
                                         tod_step_pulse,
                                         tod_freq_pulse,
                                         tx_trigger_pulse,
                                         cfg->tx_need_timestamp,
                                         cfg->tx_message_type,
                                         cfg->tx_transport_specific,
                                         cfg->tx_sequence_id));
}

uint64_t net_acq_ptp_current_time(void)
{
    return net_acq_read64_consistent(NET_ACQ_PTP_CUR_TIME_L, NET_ACQ_PTP_CUR_TIME_H);
}

int net_acq_ptp_read_txstamp(uint64_t *txstamp_ns_out)
{
    uint32_t status;
    uint64_t txstamp_ns;

    status = net_acq_read(NET_ACQ_PTP_STATUS);
    if ((status & PTP_STATUS_TXSTAMP_VALID) == 0u) {
        return 0;
    }

    /*
     * TXSTAMP 的当前 CSR 语义是按 low -> high 读出后消费。
     * 这里不要再用 hi -> low -> hi 的 consistent reader，
     * 否则会多打一拍高 32 位读取，调试时也更难理解时序。
     */
    txstamp_ns = net_acq_read64_lo_hi(NET_ACQ_PTP_TXSTAMP_L, NET_ACQ_PTP_TXSTAMP_H);
    if (txstamp_ns_out) {
        *txstamp_ns_out = txstamp_ns;
    }
    return 1;
}

const char *net_acq_eth_error_name(uint16_t err_code)
{
    switch (err_code) {
    case NET_ACQ_ETH_ERR_NONE:
        return "NONE";
    case NET_ACQ_ETH_ERR_FRAME_TOO_LONG:
        return "INGRESS_FRAME_TOO_LONG";
    case NET_ACQ_ETH_ERR_SOFT_RESET_ABORT:
        return "INGRESS_SOFT_RESET_ABORT";
    default:
        return "UDP_OR_IPV4_RESERVED";
    }
}

int net_acq_eth_validate_config(const net_acq_eth_cfg_t *cfg)
{
    uint64_t local_mac;
    uint64_t dst_mac;
    uint64_t ptp_peer_mac;
    uint32_t need_data_path;
    uint32_t need_ptp_path;

    if (!cfg) {
        return 0;
    }

    need_data_path = cfg->backend_enable ? 1u : 0u;
    need_ptp_path  = cfg->ptp_enable ? 1u : 0u;

    if (!need_data_path && !need_ptp_path) {
        return 0;
    }

    local_mac    = cfg->local_mac    & 0x0000FFFFFFFFFFFFull;
    dst_mac      = cfg->dst_mac      & 0x0000FFFFFFFFFFFFull;
    ptp_peer_mac = cfg->ptp_peer_mac & 0x0000FFFFFFFFFFFFull;

    if (local_mac == 0ull) {
        printf("[cfg][eth] local_mac is zero, refuse active network/PTP config\n");
        return -170;
    }
    if (cfg->local_ip == 0u) {
        printf("[cfg][eth] local_ip is zero, refuse active network/PTP config\n");
        return -172;
    }

    if (need_data_path) {
        if (dst_mac == 0ull) {
            printf("[cfg][eth] dst_mac is zero, refuse backend enable\n");
            return -171;
        }
        if (cfg->dst_ip == 0u) {
            printf("[cfg][eth] dst_ip is zero, refuse backend enable\n");
            return -173;
        }
        if ((cfg->src_port == 0u) || (cfg->dst_port == 0u)) {
            printf("[cfg][eth] udp port is zero, refuse backend enable\n");
            return -174;
        }
        if (cfg->ttl == 0u) {
            printf("[cfg][eth] ttl is zero, refuse backend enable\n");
            return -175;
        }
    }

    if (need_ptp_path) {
        if (cfg->ptp_addr_split_enable) {
            if (ptp_peer_mac == 0ull) {
                printf("[cfg][eth] ptp_peer_mac is zero while split_enable=1\n");
                return -177;
            }
            if (cfg->ptp_peer_ip == 0u) {
                printf("[cfg][eth] ptp_peer_ip is zero while split_enable=1\n");
                return -178;
            }
        } else {
            if (dst_mac == 0ull) {
                printf("[cfg][eth] dst_mac is zero, cannot act as legacy PTP peer\n");
                return -179;
            }
            if (cfg->dst_ip == 0u) {
                printf("[cfg][eth] dst_ip is zero, cannot act as legacy PTP peer\n");
                return -180;
            }
        }
    }

    if (cfg->ptp_one_step && cfg->ptp_two_step) {
        printf("[cfg][eth] ptp one_step and two_step cannot both be 1\n");
        return -176;
    }

    return 0;
}

uint32_t net_acq_eth_link_is_up(void)
{
    return (net_acq_read(NET_ACQ_ETH_STATUS) & ETH_STATUS_LINK_UP) ? 1u : 0u;
}

void net_acq_eth_write_shadow(const net_acq_eth_cfg_t *cfg)
{
    uint64_t local_mac;
    uint64_t dst_mac;
    uint64_t ptp_peer_mac;

    if (!cfg) {
        return;
    }

    local_mac    = cfg->local_mac    & 0x0000FFFFFFFFFFFFull;
    dst_mac      = cfg->dst_mac      & 0x0000FFFFFFFFFFFFull;
    ptp_peer_mac = cfg->ptp_peer_mac & 0x0000FFFFFFFFFFFFull;

    net_acq_write(NET_ACQ_LOCAL_MAC_L, (uint32_t)(local_mac & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_LOCAL_MAC_H, (uint32_t)((local_mac >> 32) & 0xFFFFull));
    net_acq_write(NET_ACQ_DST_MAC_L,   (uint32_t)(dst_mac & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_DST_MAC_H,   (uint32_t)((dst_mac >> 32) & 0xFFFFull));
    net_acq_write(NET_ACQ_PTP_PEER_MAC_L, (uint32_t)(ptp_peer_mac & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_PEER_MAC_H, (uint32_t)((ptp_peer_mac >> 32) & 0xFFFFull));

    net_acq_write(NET_ACQ_LOCAL_IP, cfg->local_ip);
    net_acq_write(NET_ACQ_DST_IP,   cfg->dst_ip);
    net_acq_write(NET_ACQ_PTP_PEER_IP, cfg->ptp_peer_ip);
    net_acq_write(NET_ACQ_PTP_ADDR_CFG,
                  cfg->ptp_addr_split_enable ? PTP_ADDR_CFG_SPLIT_ENABLE : 0u);
    net_acq_write(NET_ACQ_UDP_PORTS,
                  (((uint32_t)cfg->src_port & 0xFFFFu) << 16) |
                   ((uint32_t)cfg->dst_port & 0xFFFFu));
    net_acq_write(NET_ACQ_IPV4_CFG,
                  (((uint32_t)cfg->ip_ident_seed & 0xFFFFu) << 16) |
                  (((uint32_t)cfg->dscp_ecn     & 0xFFu)   <<  8) |
                   ((uint32_t)cfg->ttl          & 0xFFu));

    net_acq_write(NET_ACQ_PTP_CFG,
                  (((uint32_t)cfg->ptp_domain & 0xFFu) << 8) |
                  ((cfg->ptp_event_priority_enable ? PTP_CFG_EVENT_PRIORITY_EN : 0u)) |
                  ((cfg->ptp_two_step              ? PTP_CFG_TWO_STEP         : 0u)) |
                  ((cfg->ptp_one_step              ? PTP_CFG_ONE_STEP         : 0u)) |
                  ((cfg->ptp_enable                ? PTP_CFG_ENABLE           : 0u)));
    net_acq_write(NET_ACQ_PTP_CLOCK_ID_L, (uint32_t)(cfg->ptp_clock_id & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_CLOCK_ID_H, (uint32_t)((cfg->ptp_clock_id >> 32) & 0xFFFFFFFFull));
    net_acq_write(NET_ACQ_PTP_PORT_NUM,   (uint32_t)(cfg->ptp_port_num & 0xFFFFu));

    net_acq_write(NET_ACQ_ETH_CTRL,
                  net_acq_eth_ctrl_build(cfg->backend_enable,
                                         cfg->tx_mirror_enable,
                                         cfg->append_fcs,
                                         cfg->udp_checksum_enable,
                                         cfg->stall_drop_enable,
                                         0u));
}
int net_acq_eth_apply_config_ex(const net_acq_eth_cfg_t *cfg,
                                const net_acq_txsched_cfg_t *txs_cfg,
                                uint32_t poll_delay_cycles,
                                uint32_t max_polls)
{
    uint32_t i;
    uint32_t status;
    uint32_t expect_backend_selected;
    int      rc;

    if (!cfg) {
        return 0;
    }

    rc = net_acq_txsched_validate_config(txs_cfg);
    if (rc != 0) {
        return rc;
    }

    if (txs_cfg) {
        net_acq_txsched_write_shadow(txs_cfg);
    }
    net_acq_eth_write_shadow(cfg);
    net_acq_write(NET_ACQ_ETH_CTRL,
                  net_acq_eth_ctrl_build(cfg->backend_enable,
                                         cfg->tx_mirror_enable,
                                         cfg->append_fcs,
                                         cfg->udp_checksum_enable,
                                         cfg->stall_drop_enable,
                                         1u));

    if (max_polls == 0u) {
        return 0;
    }

    expect_backend_selected = cfg->backend_enable ? ETH_STATUS_BACKEND_SELECTED : 0u;

    for (i = 0u; i < max_polls; i++) {
        status = net_acq_read(NET_ACQ_ETH_STATUS);
        if ((status & ETH_STATUS_CFG_ACTIVE) &&
            ((status & ETH_STATUS_BACKEND_SELECTED) == expect_backend_selected)) {
            return 0;
        }
        net_acq_delay_cycles(poll_delay_cycles);
    }

    return -10;
}

int net_acq_eth_apply_config(const net_acq_eth_cfg_t *cfg,
                             uint32_t poll_delay_cycles,
                             uint32_t max_polls)
{
    return net_acq_eth_apply_config_ex(cfg, (const net_acq_txsched_cfg_t *)0,
                                       poll_delay_cycles, max_polls);
}

void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *snap)
{
    uint32_t ports;
    uint32_t ipv4_cfg;
    uint32_t lengths;

    if (!snap) {
        return;
    }

    snap->eth_ctrl         = net_acq_read(NET_ACQ_ETH_CTRL);
    snap->eth_status       = net_acq_read(NET_ACQ_ETH_STATUS);
    snap->local_mac        = ((uint64_t)(net_acq_read(NET_ACQ_LOCAL_MAC_H) & 0xFFFFu) << 32) |
                             (uint64_t)net_acq_read(NET_ACQ_LOCAL_MAC_L);
    snap->local_ip         = net_acq_read(NET_ACQ_LOCAL_IP);
    snap->dst_mac          = ((uint64_t)(net_acq_read(NET_ACQ_DST_MAC_H) & 0xFFFFu) << 32) |
                             (uint64_t)net_acq_read(NET_ACQ_DST_MAC_L);
    snap->dst_ip           = net_acq_read(NET_ACQ_DST_IP);
    snap->ptp_peer_mac     = ((uint64_t)(net_acq_read(NET_ACQ_PTP_PEER_MAC_H) & 0xFFFFu) << 32) |
                             (uint64_t)net_acq_read(NET_ACQ_PTP_PEER_MAC_L);
    snap->ptp_peer_ip      = net_acq_read(NET_ACQ_PTP_PEER_IP);
    snap->ptp_addr_split_enable = (net_acq_read(NET_ACQ_PTP_ADDR_CFG) & PTP_ADDR_CFG_SPLIT_ENABLE) ? 1u : 0u;

    ports                  = net_acq_read(NET_ACQ_UDP_PORTS);
    snap->src_port         = (uint16_t)((ports >> 16) & 0xFFFFu);
    snap->dst_port         = (uint16_t)(ports & 0xFFFFu);

    ipv4_cfg               = net_acq_read(NET_ACQ_IPV4_CFG);
    snap->ttl              = (uint8_t)(ipv4_cfg & 0xFFu);
    snap->dscp_ecn         = (uint8_t)((ipv4_cfg >> 8) & 0xFFu);
    snap->ip_ident_seed    = (uint16_t)((ipv4_cfg >> 16) & 0xFFFFu);

    snap->tx_packet_count  = net_acq_read(NET_ACQ_ETH_TX_PKT_COUNT);
    snap->tx_byte_count    = net_acq_read(NET_ACQ_ETH_TX_BYTE_COUNT);
    snap->drop_count       = net_acq_read(NET_ACQ_ETH_DROP_COUNT);

    lengths                 = net_acq_read(NET_ACQ_ETH_LAST_LENGTHS);
    snap->last_udp_length   = NET_ACQ_ETH_LAST_UDP_LENGTH(lengths);
    snap->last_ip_total_length = NET_ACQ_ETH_LAST_IP_TOTAL_LENGTH(lengths);
    snap->last_error_code   = (uint16_t)(net_acq_read(NET_ACQ_ETH_LAST_ERROR) & 0xFFFFu);
    snap->features          = net_acq_read(NET_ACQ_ETH_FEATURES);

    snap->ptp_cfg           = net_acq_read(NET_ACQ_PTP_CFG);
    snap->ptp_clock_id      = net_acq_read64_consistent(NET_ACQ_PTP_CLOCK_ID_L, NET_ACQ_PTP_CLOCK_ID_H);
    snap->ptp_port_num      = (uint16_t)(net_acq_read(NET_ACQ_PTP_PORT_NUM) & 0xFFFFu);
    snap->ptp_status        = net_acq_read(NET_ACQ_PTP_STATUS);
    /*
     * 注意：TXSTAMP_L/H 的 low->high 读取会消费当前 latched timestamp。
     * 为了避免 heartbeat / dump 把 PTP 运行态真正需要的 t3 偷读掉，
     * snapshot 路径这里不主动读取 TXSTAMP 数据本体。
     */
    snap->ptp_txstamp_ns    = 0ull;
    snap->ptp_ctrl          = net_acq_read(NET_ACQ_PTP_CTRL);
    snap->ptp_tod_set_ns    = net_acq_read64_consistent(NET_ACQ_PTP_TOD_SET_L, NET_ACQ_PTP_TOD_SET_H);
    snap->ptp_tod_step_ns   = (int32_t)net_acq_read(NET_ACQ_PTP_STEP_NS);
    snap->ptp_tod_freq_adj_ppb = (int32_t)net_acq_read(NET_ACQ_PTP_FREQ_ADJ);
    snap->ptp_cur_time_ns   = net_acq_read64_consistent(NET_ACQ_PTP_CUR_TIME_L, NET_ACQ_PTP_CUR_TIME_H);
    snap->ptp_tx_cfg        = net_acq_read(NET_ACQ_PTP_TX_CFG);
    snap->ptp_features      = net_acq_read(NET_ACQ_PTP_FEATURES);
}

void net_acq_dump_eth_snapshot(const net_acq_eth_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[eth ] ETH_CTRL         = 0x%08x (backend_enable=%u, tx_mirror=%u, append_fcs=%u, udp_checksum=%u, stall_drop=%u)\n",
           snap->eth_ctrl,
           (snap->eth_ctrl & ETH_CTRL_BACKEND_ENABLE)    ? 1u : 0u,
           (snap->eth_ctrl & ETH_CTRL_TX_MIRROR_ENABLE)  ? 1u : 0u,
           (snap->eth_ctrl & ETH_CTRL_APPEND_FCS)        ? 1u : 0u,
           (snap->eth_ctrl & ETH_CTRL_UDP_CHECKSUM_EN)   ? 1u : 0u,
           (snap->eth_ctrl & ETH_CTRL_STALL_DROP_ENABLE) ? 1u : 0u);

    printf("[eth ] ETH_STATUS       = 0x%08x (tx_busy=%u, tx_error=%u, pkt_overflow=%u, mac_stall=%u, cfg_active=%u, backend_selected=%u, link_up=%u, last_err=%u(%s))\n",
           snap->eth_status,
           (snap->eth_status & ETH_STATUS_TX_BUSY)           ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_TX_ERROR_STICKY)   ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_PKT_BUF_OVERFLOW)  ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_MAC_STALL_TIMEOUT) ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_CFG_ACTIVE)        ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_BACKEND_SELECTED)  ? 1u : 0u,
           (snap->eth_status & ETH_STATUS_LINK_UP)           ? 1u : 0u,
           (unsigned int)NET_ACQ_ETH_STATUS_LAST_ERROR(snap->eth_status),
           net_acq_eth_error_name(NET_ACQ_ETH_STATUS_LAST_ERROR(snap->eth_status)));

    printf("[eth ] LOCAL_MAC        = ");
    print_mac48(snap->local_mac);
    printf("\n");

    printf("[eth ] DATA_DST_MAC     = ");
    print_mac48(snap->dst_mac);
    printf("\n");

    printf("[eth ] LOCAL_IP         = ");
    print_ip4(snap->local_ip);
    printf("\n");

    printf("[eth ] DATA_DST_IP      = ");
    print_ip4(snap->dst_ip);
    printf("\n");

    printf("[eth ] PTP_ADDR_SPLIT   = %u\n", (unsigned int)(snap->ptp_addr_split_enable ? 1u : 0u));
    printf("[eth ] PTP_PEER_MAC     = ");
    print_mac48(snap->ptp_peer_mac);
    printf("\n");
    printf("[eth ] PTP_PEER_IP      = ");
    print_ip4(snap->ptp_peer_ip);
    printf("\n");
    printf("[eth ] UDP_PORTS        = src=0x%04x (%u), dst=0x%04x (%u)\n",
           (unsigned int)snap->src_port, (unsigned int)snap->src_port,
           (unsigned int)snap->dst_port, (unsigned int)snap->dst_port);
    printf("[eth ] IPV4_CFG         = ttl=%u, dscp_ecn=0x%02x, ip_ident_seed=0x%04x\n",
           (unsigned int)snap->ttl,
           (unsigned int)snap->dscp_ecn,
           (unsigned int)snap->ip_ident_seed);
    printf("[eth ] TX_PKT_COUNT     = %u\n", snap->tx_packet_count);
    printf("[eth ] TX_BYTE_COUNT    = %u\n", snap->tx_byte_count);
    printf("[eth ] DROP_COUNT       = %u\n", snap->drop_count);
    printf("[eth ] LAST_LENGTHS     = ip_total=%u, udp=%u\n",
           (unsigned int)snap->last_ip_total_length,
           (unsigned int)snap->last_udp_length);
    printf("[eth ] LAST_ERROR       = code=%u(%s)\n",
           (unsigned int)snap->last_error_code,
           net_acq_eth_error_name(snap->last_error_code));
    printf("[eth ] ETH_FEATURES     = 0x%08x (version=%u, backend=%u, ipv4_udp_tx=%u, udp_csum_hw=%u, append_fcs_hw=%u, ptp_sideband=%u, commit_shadow=%u, legacy_mirror=%u, ptp_addr_split=%u)\n",
           snap->features,
           (unsigned int)NET_ACQ_ETH_FEATURE_VERSION(snap->features),
           (snap->features & ETH_FEATURE_BACKEND_PRESENT)     ? 1u : 0u,
           (snap->features & ETH_FEATURE_IPV4_UDP_TX_PRESENT) ? 1u : 0u,
           (snap->features & ETH_FEATURE_UDP_CHECKSUM_HW)     ? 1u : 0u,
           (snap->features & ETH_FEATURE_APPEND_FCS_HW)       ? 1u : 0u,
           (snap->features & ETH_FEATURE_PTP_RESERVED)        ? 1u : 0u,
           (snap->features & ETH_FEATURE_COMMIT_SHADOW)       ? 1u : 0u,
           (snap->features & ETH_FEATURE_LEGACY_MIRROR)       ? 1u : 0u,
           (snap->features & ETH_FEATURE_PTP_ADDR_SPLIT_PRESENT)      ? 1u : 0u);

    printf("[ptp ] PTP_CFG          = 0x%08x (enable=%u, one_step=%u, two_step=%u, event_prio=%u, domain=%u)\n",
           snap->ptp_cfg,
           (snap->ptp_cfg & PTP_CFG_ENABLE)            ? 1u : 0u,
           (snap->ptp_cfg & PTP_CFG_ONE_STEP)          ? 1u : 0u,
           (snap->ptp_cfg & PTP_CFG_TWO_STEP)          ? 1u : 0u,
           (snap->ptp_cfg & PTP_CFG_EVENT_PRIORITY_EN) ? 1u : 0u,
           (unsigned int)NET_ACQ_PTP_CFG_DOMAIN(snap->ptp_cfg));
    printf("[ptp ] CLOCK_ID         = 0x%08x_%08x\n",
           (uint32_t)(snap->ptp_clock_id >> 32),
           (uint32_t)(snap->ptp_clock_id & 0xFFFFFFFFu));
    printf("[ptp ] PORT_NUM         = %u\n", (unsigned int)snap->ptp_port_num);
    if ((snap->ptp_status & PTP_STATUS_TXSTAMP_VALID) != 0u) {
        printf("[ptp ] TXSTAMP_NS       = <valid pending; deferred to avoid destructive read>\n");
    } else {
        printf("[ptp ] TXSTAMP_NS       = 0x%08x_%08x\n",
               (uint32_t)(snap->ptp_txstamp_ns >> 32),
               (uint32_t)(snap->ptp_txstamp_ns & 0xFFFFFFFFu));
    }
    printf("[ptp ] PTP_STATUS       = 0x%08x (txstamp_valid=%u, txstamp_overflow=%u, tx_busy=%u, tx_cmd_pending=%u, cur_time_valid=%u, tod_enable=%u, tod_hold=%u, ptp_enable=%u)\n",
           snap->ptp_status,
           (snap->ptp_status & PTP_STATUS_TXSTAMP_VALID)    ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_TXSTAMP_OVERFLOW) ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_TX_BUSY)          ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_TX_CMD_PENDING)   ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_CUR_TIME_VALID)   ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_TOD_ENABLE)       ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_TOD_HOLD)         ? 1u : 0u,
           (snap->ptp_status & PTP_STATUS_PTP_ENABLE)       ? 1u : 0u);
    printf("[ptp ] PTP_CTRL         = 0x%08x (tod_enable=%u, tod_hold=%u, need_ts=%u, msg_type=%u, tspec=%u, seq_id=%u)\n",
           snap->ptp_ctrl,
           (snap->ptp_ctrl & PTP_CTRL_TOD_ENABLE)        ? 1u : 0u,
           (snap->ptp_ctrl & PTP_CTRL_TOD_HOLD)          ? 1u : 0u,
           (snap->ptp_ctrl & PTP_CTRL_TX_NEED_TIMESTAMP) ? 1u : 0u,
           (unsigned int)NET_ACQ_PTP_CTRL_MSG_TYPE(snap->ptp_ctrl),
           (unsigned int)NET_ACQ_PTP_CTRL_TSPEC(snap->ptp_ctrl),
           (unsigned int)NET_ACQ_PTP_CTRL_SEQ_ID(snap->ptp_ctrl));
    printf("[ptp ] TOD_SET_NS       = 0x%08x_%08x\n",
           (uint32_t)(snap->ptp_tod_set_ns >> 32),
           (uint32_t)(snap->ptp_tod_set_ns & 0xFFFFFFFFu));
    printf("[ptp ] TOD_STEP_NS      = %d\n", (int)snap->ptp_tod_step_ns);
    printf("[ptp ] FREQ_ADJ_PPB    = %d\n", (int)snap->ptp_tod_freq_adj_ppb);
    printf("[ptp ] CUR_TIME_NS      = 0x%08x_%08x\n",
           (uint32_t)(snap->ptp_cur_time_ns >> 32),
           (uint32_t)(snap->ptp_cur_time_ns & 0xFFFFFFFFu));
    printf("[ptp ] TX_CFG           = 0x%08x (flags=0x%04x, log_interval=%d)\n",
           snap->ptp_tx_cfg,
           (unsigned int)NET_ACQ_PTP_TX_CFG_FLAGS(snap->ptp_tx_cfg),
           (int8_t)NET_ACQ_PTP_TX_CFG_LOG_INT(snap->ptp_tx_cfg));
    printf("[ptp ] PTP_FEATURES     = 0x%08x (version=%u, tod_sys=%u, tod_eth=%u, tx_engine=%u, tx_arbiter=%u, txstamp_cdc=%u, tx_reg_if=%u, addr_split=%u)\n",
           snap->ptp_features,
           (unsigned int)NET_ACQ_PTP_FEATURE_VERSION(snap->ptp_features),
           (snap->ptp_features & PTP_FEATURE_TOD_SYS_PRESENT)     ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_TOD_ETH_PRESENT)     ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_TX_ENGINE_PRESENT)   ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_TX_ARBITER_PRESENT)  ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_TXSTAMP_CDC_PRESENT) ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_TX_REG_IF_PRESENT)   ? 1u : 0u,
           (snap->ptp_features & PTP_FEATURE_ADDR_SPLIT_PRESENT)  ? 1u : 0u);
}

void net_acq_read_dbgext_snapshot(net_acq_dbgext_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    snap->feature_bits                    = net_acq_read(NET_ACQ_DBGEXT_FEATURES);
    snap->arb_data_frame_count            = net_acq_read(NET_ACQ_DBGEXT_ARB_DATA_FRAME_COUNT);
    snap->arb_ptp_frame_count             = net_acq_read(NET_ACQ_DBGEXT_ARB_PTP_FRAME_COUNT);
    snap->arb_stall_cycles                = net_acq_read(NET_ACQ_DBGEXT_ARB_STALL_CYCLES);
    snap->arb_last_error                  = (uint16_t)(net_acq_read(NET_ACQ_DBGEXT_ARB_LAST_ERROR) & 0xFFFFu);
    snap->backend_backpressure_cycles     = net_acq_read(NET_ACQ_DBGEXT_BACKPRESSURE_CYCLES);
    snap->ptp_tx_trigger_req_count        = net_acq_read(NET_ACQ_DBGEXT_PTP_TX_TRIGGER_REQ_CNT);
    snap->ptp_tx_trigger_accept_count     = net_acq_read(NET_ACQ_DBGEXT_PTP_TX_TRIGGER_ACC_CNT);
    snap->ptp_tx_trigger_while_busy_count = net_acq_read(NET_ACQ_DBGEXT_PTP_TX_TRIGGER_BUSY_CNT);
    snap->ptp_tx_last_error               = (uint16_t)(net_acq_read(NET_ACQ_DBGEXT_PTP_TX_LAST_ERROR) & 0xFFFFu);
    snap->ptp_rx_accept_count             = net_acq_read(NET_ACQ_DBGEXT_PTP_RX_ACCEPT_COUNT);
    snap->ptp_rx_drop_count               = net_acq_read(NET_ACQ_DBGEXT_PTP_RX_DROP_COUNT);
    snap->ptp_rx_last_error               = (uint16_t)(net_acq_read(NET_ACQ_DBGEXT_PTP_RX_LAST_ERROR) & 0xFFFFu);
    snap->ptp_rx_status                   = net_acq_read(NET_ACQ_DBGEXT_PTP_RX_STATUS);
}

void net_acq_dump_dbgext_snapshot(const net_acq_dbgext_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[dbgext] FEATURES          = 0x%08x (ver=%u, ext=%u, arb=%u, backpress=%u, ptp_tx=%u, ptp_rx=%u, sched_ctrl=%u, sched_obs=%u)\n",
           snap->feature_bits,
           (unsigned int)NET_ACQ_DBGEXT_FEATURE_VERSION(snap->feature_bits),
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_ARB_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_BACKPRESS_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_PTP_TX_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_PTP_RX_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_SCHED_CTRL_PRESENT) ? 1u : 0u,
           (snap->feature_bits & NET_ACQ_DBGEXT_FEATURE_SCHED_OBS_PRESENT) ? 1u : 0u);
    printf("[dbgext] ARB data=%u, ptp=%u, stall=%u, arb_last_err=%u(%s)\n",
           (unsigned int)snap->arb_data_frame_count,
           (unsigned int)snap->arb_ptp_frame_count,
           (unsigned int)snap->arb_stall_cycles,
           (unsigned int)snap->arb_last_error,
           net_acq_eth_error_name(snap->arb_last_error));
    printf("[dbgext] BACKPRESSURE      = %u\n",
           (unsigned int)snap->backend_backpressure_cycles);
    printf("[dbgext] PTP TX trigger req=%u, acc=%u, while_busy=%u, last_err=%u(%s)\n",
           (unsigned int)snap->ptp_tx_trigger_req_count,
           (unsigned int)snap->ptp_tx_trigger_accept_count,
           (unsigned int)snap->ptp_tx_trigger_while_busy_count,
           (unsigned int)snap->ptp_tx_last_error,
           net_acq_eth_error_name(snap->ptp_tx_last_error));
    printf("[dbgext] PTP RX accept=%u, drop=%u, last_err=%u, fifo_overflow=%u\n",
           (unsigned int)snap->ptp_rx_accept_count,
           (unsigned int)snap->ptp_rx_drop_count,
           (unsigned int)snap->ptp_rx_last_error,
           (snap->ptp_rx_status & NET_ACQ_DBGEXT_PTP_RX_STATUS_FIFO_OVERFLOW) ? 1u : 0u);
}

void net_acq_read_txsched_snapshot(net_acq_txsched_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    snap->ctrl            = net_acq_read(NET_ACQ_TXS_CTRL);
    snap->status          = net_acq_read(NET_ACQ_TXS_STATUS);
    snap->base_time_ns    = net_acq_read64_consistent(NET_ACQ_TXS_BASE_TIME_L, NET_ACQ_TXS_BASE_TIME_H);
    snap->period_ns       = net_acq_read(NET_ACQ_TXS_PERIOD_NS);
    snap->slot_offset_ns  = net_acq_read(NET_ACQ_TXS_SLOT_OFFSET_NS);
    snap->slot_width_ns   = net_acq_read(NET_ACQ_TXS_SLOT_WIDTH_NS);
    snap->advance_ns      = net_acq_read(NET_ACQ_TXS_ADVANCE_NS);
    snap->late_tol_ns     = net_acq_read(NET_ACQ_TXS_LATE_TOL_NS);
    snap->held_count      = net_acq_read(NET_ACQ_TXS_HELD_COUNT);
    snap->pass_count      = net_acq_read(NET_ACQ_TXS_PASS_COUNT);
    snap->late_count      = net_acq_read(NET_ACQ_TXS_LATE_COUNT);
    snap->slot_miss_count = net_acq_read(NET_ACQ_TXS_SLOT_MISS_COUNT);
    snap->last_plan_ns    = net_acq_read64_consistent(NET_ACQ_TXS_LAST_PLAN_L, NET_ACQ_TXS_LAST_PLAN_H);
    snap->last_actual_ns  = net_acq_read64_consistent(NET_ACQ_TXS_LAST_ACTUAL_L, NET_ACQ_TXS_LAST_ACTUAL_H);
    snap->last_delta_ns   = net_acq_read(NET_ACQ_TXS_LAST_DELTA_NS);
}

void net_acq_dump_txsched_snapshot(const net_acq_txsched_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[txs   ] CTRL             = 0x%08x (enable=%u, block_bad_tod=%u, strict_window=%u)\n",
           snap->ctrl,
           (snap->ctrl & NET_ACQ_TXS_CTRL_ENABLE) ? 1u : 0u,
           (snap->ctrl & NET_ACQ_TXS_CTRL_BLOCK_WHEN_TOD_INVALID) ? 1u : 0u,
           (snap->ctrl & NET_ACQ_TXS_CTRL_STRICT_WINDOW) ? 1u : 0u);
    printf("[txs   ] STATUS           = 0x%08x (enable=%u, tod_valid=%u, pending=%u, armed=%u, active=%u, cfg_valid=%u, block_on_bad=%u, strict=%u, last_late=%u, last_miss=%u, window_open=%u, bypass=%u)\n",
           snap->status,
           (snap->status & NET_ACQ_TXS_STATUS_ENABLE) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_TOD_VALID) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_PENDING_FRAME) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_RELEASE_ARMED) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_FRAME_ACTIVE) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_CFG_VALID) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_BLOCK_ON_BAD_TIME) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_STRICT_WINDOW) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_LAST_LATE) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_LAST_SLOT_MISS) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_WINDOW_OPEN) ? 1u : 0u,
           (snap->status & NET_ACQ_TXS_STATUS_BYPASS) ? 1u : 0u);
    printf("[txs   ] BASE/PERIOD      = base=0x%08x_%08x, period=%u, offset=%u, width=%u, advance=%u, late_tol=%u\n",
           (uint32_t)(snap->base_time_ns >> 32),
           (uint32_t)(snap->base_time_ns & 0xFFFFFFFFu),
           (unsigned int)snap->period_ns,
           (unsigned int)snap->slot_offset_ns,
           (unsigned int)snap->slot_width_ns,
           (unsigned int)snap->advance_ns,
           (unsigned int)snap->late_tol_ns);
    printf("[txs   ] COUNTS           = held=%u, pass=%u, late=%u, miss=%u\n",
           (unsigned int)snap->held_count,
           (unsigned int)snap->pass_count,
           (unsigned int)snap->late_count,
           (unsigned int)snap->slot_miss_count);
    printf("[txs   ] LAST             = plan=0x%08x_%08x, actual=0x%08x_%08x, delta_ns=%u\n",
           (uint32_t)(snap->last_plan_ns >> 32),
           (uint32_t)(snap->last_plan_ns & 0xFFFFFFFFu),
           (uint32_t)(snap->last_actual_ns >> 32),
           (uint32_t)(snap->last_actual_ns & 0xFFFFFFFFu),
           (unsigned int)snap->last_delta_ns);
}

int net_acq_run_ptp_register_test(const net_acq_ptp_test_cfg_t *ptp_cfg,
                                  const net_acq_eth_cfg_t *base_eth_cfg,
                                  uint32_t poll_delay_cycles,
                                  uint32_t eth_commit_poll_rounds,
                                  uint32_t use_external_source)
{
    net_acq_eth_snapshot_t snap;
    net_acq_eth_cfg_t local_eth_cfg;
    uint32_t expect_cfg;
    uint32_t expect_ctrl;
    uint32_t expect_tx_cfg;
    uint64_t cur0;
    uint64_t cur1;
    uint64_t txstamp;
    uint32_t i;
    uint32_t status;
    uint32_t features;
    uint32_t have_ext_features;
    uint32_t backend_available;

    if (!ptp_cfg) {
        return 0;
    }

    printf("\n================ phase: PTP_REGISTER_TEST ================\n");
    printf("[ptp-test] enable=%u, one_step=%u, two_step=%u, event_prio=%u, domain=%u\n",
           ptp_cfg->enable ? 1u : 0u,
           ptp_cfg->one_step ? 1u : 0u,
           ptp_cfg->two_step ? 1u : 0u,
           ptp_cfg->event_priority_enable ? 1u : 0u,
           (unsigned int)ptp_cfg->domain);
    printf("[ptp-test] clock_id=0x%08x_%08x, port_num=%u\n",
           (uint32_t)(ptp_cfg->clock_id >> 32),
           (uint32_t)(ptp_cfg->clock_id & 0xFFFFFFFFu),
           (unsigned int)ptp_cfg->port_num);
    printf("[ptp-test] tod_set=0x%08x_%08x, step=%d, freq_adj_ppb=%d\n",
           (uint32_t)(ptp_cfg->tod_set_ns >> 32),
           (uint32_t)(ptp_cfg->tod_set_ns & 0xFFFFFFFFu),
           (int)ptp_cfg->tod_step_ns,
           (int)ptp_cfg->tod_freq_adj_ppb);
    printf("[ptp-test] tx msg_type=%u, tspec=%u, seq_id=%u, flags=0x%04x, need_ts=%u\n",
           (unsigned int)ptp_cfg->tx_message_type,
           (unsigned int)ptp_cfg->tx_transport_specific,
           (unsigned int)ptp_cfg->tx_sequence_id,
           (unsigned int)ptp_cfg->tx_flags,
           ptp_cfg->tx_need_timestamp ? 1u : 0u);

    net_acq_disable(use_external_source);
    net_acq_delay_cycles(poll_delay_cycles);
#if NET_ACQ_USE_CLEAR_STATUS
    net_acq_clear_status(0u, use_external_source);
    net_acq_delay_cycles(poll_delay_cycles);
#endif

    backend_available = (base_eth_cfg && base_eth_cfg->backend_enable) ? 1u : 0u;
    if (base_eth_cfg) {
        local_eth_cfg = *base_eth_cfg;
        local_eth_cfg.ptp_enable                = ptp_cfg->enable;
        local_eth_cfg.ptp_one_step              = ptp_cfg->one_step;
        local_eth_cfg.ptp_two_step              = ptp_cfg->two_step;
        local_eth_cfg.ptp_event_priority_enable = ptp_cfg->event_priority_enable;
        local_eth_cfg.ptp_domain                = ptp_cfg->domain;
        local_eth_cfg.ptp_clock_id              = ptp_cfg->clock_id;
        local_eth_cfg.ptp_port_num              = ptp_cfg->port_num;
        if (net_acq_eth_apply_config(&local_eth_cfg,
                                     poll_delay_cycles,
                                     eth_commit_poll_rounds) != 0) {
            printf("[ptp-test] eth commit for ptp static config failed\n");
            return -201;
        }
    }

    net_acq_ptp_write_shadow(ptp_cfg);
    net_acq_delay_cycles(poll_delay_cycles);
    net_acq_read_eth_snapshot(&snap);

    expect_cfg = net_acq_ptp_cfg_build(ptp_cfg->enable,
                                       ptp_cfg->one_step,
                                       ptp_cfg->two_step,
                                       ptp_cfg->event_priority_enable,
                                       ptp_cfg->domain);
    expect_ctrl = net_acq_ptp_ctrl_build(ptp_cfg->tod_enable,
                                         ptp_cfg->tod_hold,
                                         0u, 0u, 0u, 0u, 0u,
                                         ptp_cfg->tx_need_timestamp,
                                         ptp_cfg->tx_message_type,
                                         ptp_cfg->tx_transport_specific,
                                         ptp_cfg->tx_sequence_id);
    expect_tx_cfg = net_acq_ptp_tx_cfg_build(ptp_cfg->tx_flags,
                                             ptp_cfg->tx_log_message_interval);

    if ((snap.ptp_cfg & 0x0000FF0Fu) != (expect_cfg & 0x0000FF0Fu)) {
        printf("[ptp-test] PTP_CFG readback mismatch, got=0x%08x expect=0x%08x\n",
               snap.ptp_cfg, expect_cfg);
        return -202;
    }
    if (snap.ptp_clock_id != ptp_cfg->clock_id) {
        printf("[ptp-test] PTP_CLOCK_ID readback mismatch\n");
        return -203;
    }
    if (snap.ptp_port_num != ptp_cfg->port_num) {
        printf("[ptp-test] PTP_PORT_NUM readback mismatch\n");
        return -204;
    }

    features = snap.ptp_features;
    have_ext_features = (features != 0u);
    if (ptp_cfg->require_ext_features && !have_ext_features) {
        printf("[ptp-test] PTP_FEATURES is zero, but extended PTP regs are required\n");
        return -205;
    }
    if (!ptp_cfg->require_ext_features && !have_ext_features) {
        printf("[ptp-test] PTP_FEATURES reads zero, continue in compatibility mode\n");
    }

    if (have_ext_features || ptp_cfg->require_ext_features) {
        if ((snap.ptp_ctrl & 0xFFFFFF83u) != (expect_ctrl & 0xFFFFFF83u)) {
            printf("[ptp-test] PTP_CTRL base readback mismatch, got=0x%08x expect=0x%08x\n",
                   snap.ptp_ctrl, expect_ctrl);
            return -206;
        }
        if (snap.ptp_tod_set_ns != ptp_cfg->tod_set_ns) {
            printf("[ptp-test] PTP_TOD_SET readback mismatch\n");
            return -207;
        }
        if (snap.ptp_tod_step_ns != ptp_cfg->tod_step_ns) {
            printf("[ptp-test] PTP_TOD_STEP readback mismatch\n");
            return -208;
        }
        if (snap.ptp_tod_freq_adj_ppb != ptp_cfg->tod_freq_adj_ppb) {
            printf("[ptp-test] PTP_FREQ_ADJ readback mismatch\n");
            return -209;
        }
        if (snap.ptp_tx_cfg != expect_tx_cfg) {
            printf("[ptp-test] PTP_TX_CFG readback mismatch, got=0x%08x expect=0x%08x\n",
                   snap.ptp_tx_cfg, expect_tx_cfg);
            return -210;
        }

        if (ptp_cfg->issue_tod_set_pulse) {
            net_acq_ptp_write_ctrl(ptp_cfg, 0u, 1u, 0u, 0u, 0u);
            net_acq_delay_cycles(poll_delay_cycles);
        }
        if (ptp_cfg->issue_tod_step_pulse) {
            net_acq_ptp_write_ctrl(ptp_cfg, 0u, 0u, 1u, 0u, 0u);
            net_acq_delay_cycles(poll_delay_cycles);
        }
        if (ptp_cfg->issue_tod_freq_pulse) {
            net_acq_ptp_write_ctrl(ptp_cfg, 0u, 0u, 0u, 1u, 0u);
            net_acq_delay_cycles(poll_delay_cycles);
        }

        cur0 = net_acq_ptp_current_time();
        cur1 = cur0;
        for (i = 0u; i < ptp_cfg->cur_time_poll_rounds; i++) {
            status = net_acq_read(NET_ACQ_PTP_STATUS);
            cur1   = net_acq_ptp_current_time();
            if ((status & PTP_STATUS_CUR_TIME_VALID) != 0u) {
                break;
            }
            net_acq_delay_cycles(poll_delay_cycles);
        }

        status = net_acq_read(NET_ACQ_PTP_STATUS);
        if (ptp_cfg->require_cur_time_valid && ((status & PTP_STATUS_CUR_TIME_VALID) == 0u)) {
            printf("[ptp-test] CUR_TIME_VALID did not assert\n");
            return -211;
        }
        if ((status & PTP_STATUS_CUR_TIME_VALID) != 0u) {
            printf("[ptp-test] current time observed: 0x%08x_%08x -> 0x%08x_%08x\n",
                   (uint32_t)(cur0 >> 32), (uint32_t)(cur0 & 0xFFFFFFFFu),
                   (uint32_t)(cur1 >> 32), (uint32_t)(cur1 & 0xFFFFFFFFu));
        } else {
            printf("[ptp-test] current time not yet valid, continue\n");
        }

        if (ptp_cfg->issue_txstamp_pop_pulse) {
            (void)net_acq_ptp_read_txstamp((uint64_t *)0);
            net_acq_delay_cycles(poll_delay_cycles);
        }

        if (ptp_cfg->issue_tx_trigger_pulse) {
            if (!backend_available) {
                printf("[ptp-test] backend is disabled, skip tx_trigger observation\n");
                if (ptp_cfg->require_txstamp_valid) {
                    return -212;
                }
            } else {
                net_acq_ptp_write_ctrl(ptp_cfg, 0u, 0u, 0u, 0u, 1u);
                for (i = 0u; i < ptp_cfg->txstamp_poll_rounds; i++) {
                    status = net_acq_read(NET_ACQ_PTP_STATUS);
                    if ((status & PTP_STATUS_TXSTAMP_VALID) != 0u) {
                        break;
                    }
                    net_acq_delay_cycles(poll_delay_cycles);
                }
                status = net_acq_read(NET_ACQ_PTP_STATUS);
                if (ptp_cfg->require_txstamp_valid && ((status & PTP_STATUS_TXSTAMP_VALID) == 0u)) {
                    printf("[ptp-test] txstamp_valid did not assert after tx_trigger\n");
                    return -213;
                }
                if ((status & PTP_STATUS_TXSTAMP_VALID) != 0u) {
                    if (net_acq_ptp_read_txstamp(&txstamp) > 0) {
                        printf("[ptp-test] txstamp observed: 0x%08x_%08x\n",
                               (uint32_t)(txstamp >> 32),
                               (uint32_t)(txstamp & 0xFFFFFFFFu));
                    } else {
                        printf("[ptp-test] txstamp_valid was seen, but it disappeared before consume\n");
                    }
                } else {
                    printf("[ptp-test] tx_trigger issued, but txstamp_valid is still 0\n");
                }
            }
        }
    }

    net_acq_read_eth_snapshot(&snap);
    net_acq_dump_eth_snapshot(&snap);
    printf("[ptp-test] register self-test completed\n");
    return 0;
}

void net_acq_dump_all_registers(void)
{
    net_acq_snapshot_t basic;
    net_acq_debug_snapshot_t dbg;
    net_acq_eth_snapshot_t eth;
    net_acq_dbgext_snapshot_t dbgext;
    net_acq_txsched_snapshot_t txs;
    net_acq_ptp_rx_snapshot_t ptp_rx;

    net_acq_read_snapshot(&basic);
    net_acq_read_debug_snapshot(&dbg);
    net_acq_read_eth_snapshot(&eth);
    net_acq_read_dbgext_snapshot(&dbgext);
    net_acq_read_txsched_snapshot(&txs);
    net_acq_ptp_rx_read_snapshot(&ptp_rx);
    net_acq_dump_snapshot(&basic);
    net_acq_dump_debug_snapshot(&dbg);
    net_acq_dump_eth_snapshot(&eth);
    net_acq_dump_dbgext_snapshot(&dbgext);
    net_acq_dump_txsched_snapshot(&txs);
    net_acq_ptp_rx_dump_snapshot(&ptp_rx);
}

int net_acq_run_phase(const net_acq_phase_cfg_t *cfg)
{
    return net_acq_run_soc_phase(cfg, (const net_acq_eth_cfg_t *)0, 0u);
}

int net_acq_run_soc_phase(const net_acq_phase_cfg_t *cfg,
                          const net_acq_eth_cfg_t *eth_cfg,
                          uint32_t eth_commit_poll_rounds)
{
    net_acq_snapshot_t snap;
    net_acq_debug_snapshot_t dbg;
    net_acq_eth_snapshot_t eth;
    uint32_t last_packet_count;
    uint32_t last_eth_packet_count;
    uint32_t same_count_rounds;
    uint32_t rounds;
    uint32_t need_eth_target;
    int strict_rc;

    if (!cfg) {
        return -99;
    }

    printf("\n================ phase: %s ================\n", cfg->name);
    printf("[phase] source             = %s\n",
           net_acq_source_name(cfg->use_external_source));
    printf("[phase] mode               = %u (%s)\n",
           cfg->mode, net_acq_mode_name(cfg->mode));
    printf("[phase] sample_period      = %u\n", cfg->sample_period);
    printf("[phase] samples_per_pkt    = %u (effective=%u)\n",
           cfg->samples_per_packet,
           (unsigned int)net_acq_effective_samples_per_packet(cfg->samples_per_packet));
    printf("[phase] channel_id         = %u\n", cfg->channel_id);
    printf("[phase] target_packets     = %u\n", cfg->target_packets);
    printf("[phase] require_link_up    = %u\n", cfg->require_eth_link_up ? 1u : 0u);
    printf("[phase] poll_delay_cycles  = %u\n", cfg->poll_delay_cycles);
    printf("[phase] heartbeat_rounds   = %u\n", cfg->heartbeat_rounds);
    printf("[phase] timeout_rounds     = %u\n", cfg->timeout_rounds);
    printf("[phase] expect_udp_len     = %u\n",
           (unsigned int)net_acq_expected_udp_length(cfg->samples_per_packet));
    printf("[phase] expect_ip_total    = %u\n",
           (unsigned int)net_acq_expected_ip_total_length(cfg->samples_per_packet));
    if (eth_cfg) {
        printf("[phase] eth_backend_enable = %u\n", eth_cfg->backend_enable ? 1u : 0u);
        printf("[phase] eth_tx_mirror      = %u\n", eth_cfg->tx_mirror_enable ? 1u : 0u);
        printf("[phase] eth_commit_polls   = %u\n", eth_commit_poll_rounds);
    }

    printf("\n[1] stop module and issue reset pulse...\n");
    net_acq_disable(cfg->use_external_source);
    net_acq_soft_reset(0u, cfg->use_external_source);
#if NET_ACQ_USE_CLEAR_STATUS
    net_acq_clear_status(0u, cfg->use_external_source);
#endif
    net_acq_delay_cycles(cfg->poll_delay_cycles);
    net_acq_dump_all_registers();

    strict_rc = net_acq_verify_eth_features((eth_cfg && eth_cfg->backend_enable) ? 1u : 0u,
                                           (eth_cfg && eth_cfg->ptp_addr_split_enable) ? 1u : 0u);
    if (strict_rc != 0) {
        return strict_rc;
    }

    strict_rc = net_acq_eth_validate_config(eth_cfg);
    if (strict_rc != 0) {
        return strict_rc;
    }

    printf("\n[2] configure front-end...\n");
    net_acq_apply_config(cfg->mode,
                         cfg->sample_period,
                         cfg->samples_per_packet,
                         cfg->channel_id,
                         cfg->use_external_source);
    strict_rc = net_acq_verify_front_shadow(cfg);
    if (strict_rc != 0) {
        net_acq_dump_all_registers();
        return strict_rc;
    }
    net_acq_read_snapshot(&snap);
    net_acq_dump_snapshot(&snap);

    if (eth_cfg) {
        printf("\n[3] configure ethernet backend shadow regs and commit...\n");
        if (net_acq_eth_apply_config(eth_cfg,
                                     cfg->poll_delay_cycles,
                                     eth_commit_poll_rounds) != 0) {
            printf("[error] eth commit/cfg_active polling failed\n");
            net_acq_dump_all_registers();
            return -11;
        }
        strict_rc = net_acq_verify_eth_shadow(eth_cfg);
        if (strict_rc != 0) {
            net_acq_dump_all_registers();
            return strict_rc;
        }
        if (cfg->require_eth_link_up && eth_cfg->backend_enable) {
            strict_rc = net_acq_wait_eth_link_up(cfg->poll_delay_cycles, eth_commit_poll_rounds);
            if (strict_rc != 0) {
                printf("[strict][eth] link_up did not assert after commit\n");
                net_acq_dump_all_registers();
                return strict_rc;
            }
        }
        net_acq_read_eth_snapshot(&eth);
        net_acq_dump_eth_snapshot(&eth);
    }

    printf("\n[4] enable acquisition...\n");
    net_acq_enable(cfg->use_external_source);
    net_acq_delay_cycles(cfg->enable_settle_cycles);
    net_acq_dump_all_registers();

    printf("\n[5] polling front-end + monitor + ethernet status...\n");
    last_packet_count     = net_acq_read(NET_ACQ_PACKET_COUNT);
    last_eth_packet_count = (eth_cfg && eth_cfg->backend_enable) ? net_acq_read(NET_ACQ_ETH_TX_PKT_COUNT) : 0u;
    same_count_rounds     = 0u;
    rounds                = 0u;
    need_eth_target       = (eth_cfg && eth_cfg->backend_enable) ? 1u : 0u;

    while (1) {
        uint32_t status;
        uint32_t packet_cnt;
        uint32_t fifo_level;
        uint64_t last_ts;
        uint32_t eth_status;
        uint32_t eth_tx_pkt;
        uint32_t eth_drops;

        status     = net_acq_read(NET_ACQ_STATUS);
        packet_cnt = net_acq_read(NET_ACQ_PACKET_COUNT);
        fifo_level = net_acq_read(NET_ACQ_FIFO_LEVEL);
        last_ts    = net_acq_last_timestamp();

        eth_status = 0u;
        eth_tx_pkt = 0u;
        eth_drops  = 0u;
        if (eth_cfg) {
            eth_status = net_acq_read(NET_ACQ_ETH_STATUS);
            eth_tx_pkt = net_acq_read(NET_ACQ_ETH_TX_PKT_COUNT);
            eth_drops  = net_acq_read(NET_ACQ_ETH_DROP_COUNT);
        }

        net_acq_read_debug_snapshot(&dbg);
        if (eth_cfg) {
            net_acq_read_eth_snapshot(&eth);
        }

        strict_rc = net_acq_check_runtime_strict(cfg,
                                                 eth_cfg,
                                                 &dbg,
                                                 eth_cfg ? &eth : (const net_acq_eth_snapshot_t *)0);
        if (strict_rc != 0) {
            printf("[strict] runtime check failed, rc=%d\n", strict_rc);
            net_acq_dump_all_registers();
            return strict_rc;
        }

        if ((packet_cnt != last_packet_count) || (eth_tx_pkt != last_eth_packet_count)) {
            printf("[run ][%s] front_pkt=%u, eth_pkt=%u, eth_drop=%u, front_status=0x%08x, eth_status=0x%08x, sample_fifo=%u, info_fifo=%u, last_ts=0x%08x_%08x\n",
                   cfg->name,
                   packet_cnt,
                   eth_tx_pkt,
                   eth_drops,
                   status,
                   eth_status,
                   net_acq_sample_fifo_level(fifo_level),
                   net_acq_info_fifo_level(fifo_level),
                   (uint32_t)(last_ts >> 32),
                   (uint32_t)(last_ts & 0xFFFFFFFFu));

            if (cfg->dump_monitor_each_packet) {
                net_acq_dump_debug_snapshot(&dbg);
                if (eth_cfg) {
                    net_acq_dump_eth_snapshot(&eth);
                }
            }

            last_packet_count     = packet_cnt;
            last_eth_packet_count = eth_tx_pkt;
            same_count_rounds     = 0u;
        } else {
            same_count_rounds++;
        }

        if (status & STATUS_ERROR) {
            printf("[warn] front-end STATUS.error=1, dumping all registers...\n");
            net_acq_dump_all_registers();
            return -1;
        }

        if ((cfg->target_packets != 0u) && (packet_cnt >= cfg->target_packets)) {
            if (!need_eth_target || (eth_tx_pkt >= cfg->target_packets)) {
                printf("\n[phase] %s reached strict target_packets=%u\n",
                       cfg->name, cfg->target_packets);
                if (dbg.dbg_good_packet_count < cfg->target_packets) {
                    printf("[strict] GOOD_PKT_CNT=%u still below target=%u\n",
                           (unsigned int)dbg.dbg_good_packet_count,
                           (unsigned int)cfg->target_packets);
                    net_acq_dump_all_registers();
                    return -160;
                }
                if (need_eth_target && eth.tx_byte_count == 0u) {
                    printf("[strict] ETH_TX_BYTE_COUNT is zero after reaching target\n");
                    net_acq_dump_all_registers();
                    return -161;
                }
                net_acq_dump_all_registers();
                return 0;
            }
        }

        if ((cfg->heartbeat_rounds != 0u) && (same_count_rounds >= cfg->heartbeat_rounds)) {
            printf("[beat][%s] front_pkt=%u, eth_pkt=%u, eth_drop=%u, front_status=0x%08x, eth_status=0x%08x, sample_fifo=%u, info_fifo=%u\n",
                   cfg->name,
                   packet_cnt,
                   eth_tx_pkt,
                   eth_drops,
                   status,
                   eth_status,
                   net_acq_sample_fifo_level(fifo_level),
                   net_acq_info_fifo_level(fifo_level));

            if (cfg->dump_monitor_on_heartbeat) {
                net_acq_read_snapshot(&snap);
                net_acq_dump_snapshot(&snap);
                net_acq_dump_debug_snapshot(&dbg);
                if (eth_cfg) {
                    net_acq_dump_eth_snapshot(&eth);
                }
            }

            same_count_rounds = 0u;
        }

        rounds++;
        if ((cfg->timeout_rounds != 0u) && (rounds >= cfg->timeout_rounds)) {
            printf("[timeout][%s] timeout reached, dumping all registers...\n", cfg->name);
            net_acq_dump_all_registers();
            return -2;
        }

        net_acq_delay_cycles(cfg->poll_delay_cycles);
    }
}

const char *net_acq_ptp_msg_type_name(uint8_t msg_type)
{
    switch (msg_type & 0x0Fu) {
    case NET_ACQ_PTP_MSG_SYNC:
        return "Sync";
    case NET_ACQ_PTP_MSG_DELAY_REQ:
        return "Delay_Req";
    case NET_ACQ_PTP_MSG_FOLLOW_UP:
        return "Follow_Up";
    case NET_ACQ_PTP_MSG_DELAY_RESP:
        return "Delay_Resp";
    default:
        return "Unknown";
    }
}

void net_acq_print_u80_hex(const net_acq_u80_t *value)
{
    if (!value) {
        printf("0x0000_00000000_00000000");
        return;
    }

    printf("0x%04x_%08x_%08x",
           (unsigned int)value->hi16,
           (unsigned int)(value->lo64 >> 32),
           (unsigned int)(value->lo64 & 0xFFFFFFFFu));
}

void net_acq_ptp_rx_read_snapshot(net_acq_ptp_rx_snapshot_t *snap)
{
    uint32_t status;

    if (!snap) {
        return;
    }

    snap->status_raw                    = 0u;
    snap->valid                         = 0u;
    snap->overflow_sticky               = 0u;
    snap->msg_type                      = 0u;
    snap->sequence_id                   = 0u;
    snap->rx_timestamp_ns               = 0ull;
    snap->msg_timestamp.hi16            = 0u;
    snap->msg_timestamp.lo64            = 0ull;
    snap->requesting_port_identity.hi16 = 0u;
    snap->requesting_port_identity.lo64 = 0ull;

    status = net_acq_read(NET_ACQ_PTP_RX_MBOX_STATUS);

    snap->status_raw        = status;
    snap->valid             = (status & PTP_RX_MBOX_STATUS_VALID) ? 1u : 0u;
    snap->overflow_sticky   = (status & PTP_RX_MBOX_STATUS_OVERFLOW) ? 1u : 0u;
    if (!snap->valid) {
        return;
    }

    /*
     * PTP RX mailbox 只有单 outstanding 条目。
     * 因此在 valid=0 的空闲轮询路径里，只读状态寄存器就立即返回，
     * 避免每一轮都额外打满 RXTS/MSGTS/REQPORT 这些 CSR 读取，
     * 让 CPU 更快回到下一次 mailbox 轮询。
     */
    snap->msg_type                      = NET_ACQ_PTP_RX_MBOX_MSG_TYPE(status);
    snap->sequence_id                   = NET_ACQ_PTP_RX_MBOX_SEQUENCE_ID(status);
    snap->rx_timestamp_ns               = net_acq_read64_consistent(NET_ACQ_PTP_RX_RXTS_L,
                                                                    NET_ACQ_PTP_RX_RXTS_H);
    snap->msg_timestamp.lo64            = ((uint64_t)net_acq_read(NET_ACQ_PTP_RX_MSGTS_M) << 32) |
                                          (uint64_t)net_acq_read(NET_ACQ_PTP_RX_MSGTS_L);
    snap->msg_timestamp.hi16            = (uint16_t)(net_acq_read(NET_ACQ_PTP_RX_MSGTS_H) & 0xFFFFu);
    snap->requesting_port_identity.lo64 = ((uint64_t)net_acq_read(NET_ACQ_PTP_RX_REQPORT_M) << 32) |
                                          (uint64_t)net_acq_read(NET_ACQ_PTP_RX_REQPORT_L);
    snap->requesting_port_identity.hi16 = (uint16_t)(net_acq_read(NET_ACQ_PTP_RX_REQPORT_H) & 0xFFFFu);
}

void net_acq_ptp_rx_dump_snapshot(const net_acq_ptp_rx_snapshot_t *snap)
{
    if (!snap) {
        return;
    }

    printf("[ptp-rx] MBOX_STATUS      = 0x%08x (valid=%u, overflow=%u, msg_type=%u:%s, seq=%u)\n",
           snap->status_raw,
           (unsigned int)snap->valid,
           (unsigned int)snap->overflow_sticky,
           (unsigned int)snap->msg_type,
           net_acq_ptp_msg_type_name(snap->msg_type),
           (unsigned int)snap->sequence_id);
    printf("[ptp-rx] RX_TIMESTAMP_NS  = 0x%08x_%08x\n",
           (unsigned int)(snap->rx_timestamp_ns >> 32),
           (unsigned int)(snap->rx_timestamp_ns & 0xFFFFFFFFu));
    printf("[ptp-rx] MSG_TIMESTAMP_80 = ");
    net_acq_print_u80_hex(&snap->msg_timestamp);
    printf("\n");
    printf("[ptp-rx] REQ_PORT_ID_80   = ");
    net_acq_print_u80_hex(&snap->requesting_port_identity);
    printf("\n");
}

void net_acq_ptp_rx_mbox_pop(void)
{
    net_acq_write(NET_ACQ_PTP_RX_MBOX_CTRL, PTP_RX_MBOX_CTRL_POP);
}
