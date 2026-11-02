#ifndef NET_ACQ_HAL_H
#define NET_ACQ_HAL_H

#include <stdint.h>
#include "net_acq_regs.h"

#ifndef NET_ACQ_USE_CLEAR_STATUS
#define NET_ACQ_USE_CLEAR_STATUS 1
#endif

typedef struct {
    const char *name;
    uint32_t    mode;
    uint32_t    sample_period;
    uint32_t    samples_per_packet;
    uint32_t    channel_id;
    uint32_t    use_external_source;
    uint32_t    target_packets;          /* 0 表示不以包数作为退出条件 */
    uint32_t    require_eth_link_up;     /* 1 表示要求 ETH_STATUS.link_up=1 */
    uint32_t    poll_delay_cycles;
    uint32_t    enable_settle_cycles;
    uint32_t    heartbeat_rounds;        /* 0 表示关闭 heartbeat */
    uint32_t    timeout_rounds;          /* 0 表示不超时 */
    uint32_t    dump_monitor_each_packet;
    uint32_t    dump_monitor_on_heartbeat;
} net_acq_phase_cfg_t;

typedef struct {
    uint32_t ctrl;
    uint32_t mode;
    uint32_t sample_period;
    uint32_t samples_per_packet;
    uint32_t channel_id;
    uint32_t status;
    uint32_t packet_count;
    uint32_t fifo_level;
    uint64_t last_timestamp;
} net_acq_snapshot_t;

typedef struct {
    uint32_t dbg_status;
    uint32_t dbg_good_packet_count;
    uint32_t dbg_bad_packet_count;
    uint32_t dbg_last_seq_id;
    uint64_t dbg_last_timestamp;
    uint16_t dbg_last_sample_count;
    uint16_t dbg_last_payload_count;
    uint16_t dbg_last_error_code;
    uint16_t dbg_last_error_word_index;
} net_acq_debug_snapshot_t;

typedef struct {
    uint32_t backend_enable;
    uint32_t tx_mirror_enable;
    uint32_t append_fcs;
    uint32_t udp_checksum_enable;
    uint32_t stall_drop_enable;

    /* 一套共享的 DUT 本地身份：普通 UDP 数据面与 compact PTP 共用。 */
    uint64_t local_mac;                  /* 低 48 bit 有效 */
    uint32_t local_ip;

    /* 普通 UDP 数据面目的地址。 */
    uint64_t dst_mac;                    /* 低 48 bit 有效 */
    uint32_t dst_ip;

    /* 可选：PTP peer 与普通 UDP data dst 分离。 */
    uint32_t ptp_addr_split_enable;
    uint64_t ptp_peer_mac;               /* 低 48 bit 有效 */
    uint32_t ptp_peer_ip;

    uint16_t src_port;
    uint16_t dst_port;
    uint8_t  ttl;
    uint8_t  dscp_ecn;
    uint16_t ip_ident_seed;

    /* PTP compact 静态匹配配置：会进入 RX filter / TX 仲裁相关逻辑 */
    uint32_t ptp_enable;
    uint32_t ptp_one_step;
    uint32_t ptp_two_step;
    uint32_t ptp_event_priority_enable;
    uint8_t  ptp_domain;
    uint64_t ptp_clock_id;
    uint16_t ptp_port_num;
} net_acq_eth_cfg_t;

typedef struct {
    uint32_t eth_ctrl;
    uint32_t eth_status;
    uint64_t local_mac;
    uint32_t local_ip;
    uint64_t dst_mac;
    uint32_t dst_ip;
    uint32_t ptp_addr_split_enable;
    uint64_t ptp_peer_mac;
    uint32_t ptp_peer_ip;
    uint16_t src_port;
    uint16_t dst_port;
    uint8_t  ttl;
    uint8_t  dscp_ecn;
    uint16_t ip_ident_seed;
    uint32_t tx_packet_count;
    uint32_t tx_byte_count;
    uint32_t drop_count;
    uint16_t last_ip_total_length;
    uint16_t last_udp_length;
    uint16_t last_error_code;
    uint32_t features;
    uint32_t ptp_cfg;
    uint64_t ptp_clock_id;
    uint16_t ptp_port_num;
    uint64_t ptp_txstamp_ns;            /* snapshot path不主动消费 TXSTAMP；valid=1 时该值可能保持为 0 */
    uint32_t ptp_status;
    uint32_t ptp_ctrl;
    uint64_t ptp_tod_set_ns;
    int32_t  ptp_tod_step_ns;
    int32_t  ptp_tod_freq_adj_ppb;
    uint64_t ptp_cur_time_ns;
    uint32_t ptp_tx_cfg;
    uint32_t ptp_features;
} net_acq_eth_snapshot_t;

typedef struct {
    uint32_t feature_bits;
    uint32_t arb_data_frame_count;
    uint32_t arb_ptp_frame_count;
    uint32_t arb_stall_cycles;
    uint16_t arb_last_error;
    uint32_t backend_backpressure_cycles;
    uint32_t ptp_tx_trigger_req_count;
    uint32_t ptp_tx_trigger_accept_count;
    uint32_t ptp_tx_trigger_while_busy_count;
    uint16_t ptp_tx_last_error;
    uint32_t ptp_rx_accept_count;
    uint32_t ptp_rx_drop_count;
    uint16_t ptp_rx_last_error;
    uint32_t ptp_rx_status;
} net_acq_dbgext_snapshot_t;

typedef struct {
    uint32_t enable;
    uint32_t block_when_tod_invalid;
    uint32_t strict_window;
    uint64_t base_time_ns;
    uint32_t period_ns;
    uint32_t slot_offset_ns;
    uint32_t slot_width_ns;
    uint32_t advance_ns;
    uint32_t late_tol_ns;
} net_acq_txsched_cfg_t;

typedef struct {
    uint32_t ctrl;
    uint32_t status;
    uint64_t base_time_ns;
    uint32_t period_ns;
    uint32_t slot_offset_ns;
    uint32_t slot_width_ns;
    uint32_t advance_ns;
    uint32_t late_tol_ns;
    uint32_t held_count;
    uint32_t pass_count;
    uint32_t late_count;
    uint32_t slot_miss_count;
    uint64_t last_plan_ns;
    uint64_t last_actual_ns;
    uint32_t last_delta_ns;
} net_acq_txsched_snapshot_t;

typedef struct {
    uint16_t hi16;
    uint64_t lo64;
} net_acq_u80_t;

typedef struct {
    uint32_t    status_raw;
    uint32_t    valid;
    uint32_t    overflow_sticky;
    uint8_t     msg_type;
    uint16_t    sequence_id;
    uint64_t    rx_timestamp_ns;
    net_acq_u80_t msg_timestamp;
    net_acq_u80_t requesting_port_identity;
} net_acq_ptp_rx_snapshot_t;

typedef struct {
    uint32_t enable;
    uint32_t one_step;
    uint32_t two_step;
    uint32_t event_priority_enable;
    uint8_t  domain;
    uint64_t clock_id;
    uint16_t port_num;
    uint32_t tod_enable;
    uint32_t tod_hold;
    uint64_t tod_set_ns;
    int32_t  tod_step_ns;
    int32_t  tod_freq_adj_ppb;
    /* 兼容影子字段：当前 RTL 主动 TX 固定为 Delay_Req + need_timestamp=1 */
    uint32_t tx_need_timestamp;
    uint8_t  tx_message_type;
    uint8_t  tx_transport_specific;
    uint16_t tx_sequence_id;
    uint16_t tx_flags;
    int8_t   tx_log_message_interval;
    uint32_t issue_tod_set_pulse;
    uint32_t issue_tod_step_pulse;
    uint32_t issue_tod_freq_pulse;
    uint32_t issue_tx_trigger_pulse;
    /* 软件兼容位：当前不再通过 PTP_CTRL.bit2 pop TXSTAMP，读 TXSTAMP_L/H 即消费 */
    uint32_t issue_txstamp_pop_pulse;
    uint32_t require_ext_features;
    uint32_t require_cur_time_valid;
    uint32_t require_txstamp_valid;
    uint32_t cur_time_poll_rounds;
    uint32_t txstamp_poll_rounds;
} net_acq_ptp_test_cfg_t;

void     net_acq_delay_cycles(volatile uint32_t cycles);
uint32_t net_acq_ctrl_build(uint32_t enable, uint32_t use_external_source);
void     net_acq_reg_write(uint32_t addr, uint32_t value);
uint32_t net_acq_reg_read(uint32_t addr);
void     net_acq_write(uint32_t offset, uint32_t value);
uint32_t net_acq_read(uint32_t offset);
void     net_acq_disable(uint32_t use_external_source);
void     net_acq_enable(uint32_t use_external_source);
void     net_acq_soft_reset(uint32_t keep_enable, uint32_t use_external_source);
void     net_acq_clear_status(uint32_t keep_enable, uint32_t use_external_source);
void     net_acq_phy_restart_autoneg(void);
void     net_acq_phy_soft_reset(void);
void     net_acq_set_time_quality(uint32_t state);
void     net_acq_apply_config(uint32_t mode,
                              uint32_t sample_period,
                              uint32_t samples_per_packet,
                              uint32_t channel_id,
                              uint32_t use_external_source);
uint64_t net_acq_last_timestamp(void);
uint64_t net_acq_debug_last_timestamp(void);
uint32_t net_acq_sample_fifo_level(uint32_t fifo_level);
uint32_t net_acq_info_fifo_level(uint32_t fifo_level);
uint16_t net_acq_effective_samples_per_packet(uint32_t samples_per_packet);
uint16_t net_acq_expected_udp_length(uint32_t samples_per_packet);
uint16_t net_acq_expected_ip_total_length(uint32_t samples_per_packet);
const char *net_acq_source_name(uint32_t use_external_source);
const char *net_acq_mode_name(uint32_t mode);
const char *net_acq_debug_error_name(uint16_t err_code);
void     net_acq_read_snapshot(net_acq_snapshot_t *snap);
void     net_acq_dump_snapshot(const net_acq_snapshot_t *snap);
void     net_acq_read_debug_snapshot(net_acq_debug_snapshot_t *snap);
void     net_acq_dump_debug_snapshot(const net_acq_debug_snapshot_t *snap);

uint32_t net_acq_eth_ctrl_build(uint32_t backend_enable,
                                uint32_t tx_mirror_enable,
                                uint32_t append_fcs,
                                uint32_t udp_checksum_enable,
                                uint32_t stall_drop_enable,
                                uint32_t pulse_commit);
const char *net_acq_eth_error_name(uint16_t err_code);
int      net_acq_eth_validate_config(const net_acq_eth_cfg_t *cfg);
uint32_t net_acq_eth_link_is_up(void);
void     net_acq_eth_write_shadow(const net_acq_eth_cfg_t *cfg);
int      net_acq_eth_apply_config(const net_acq_eth_cfg_t *cfg,
                                  uint32_t poll_delay_cycles,
                                  uint32_t max_polls);
uint32_t net_acq_ptp_cfg_build(uint32_t enable,
                                uint32_t one_step,
                                uint32_t two_step,
                                uint32_t event_priority_enable,
                                uint8_t domain);
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
                                uint16_t tx_sequence_id);
uint32_t net_acq_ptp_tx_cfg_build(uint16_t tx_flags,
                                  int8_t tx_log_message_interval);
void     net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *cfg);
void     net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *cfg,
                                uint32_t txstamp_pop_pulse,
                                uint32_t tod_set_pulse,
                                uint32_t tod_step_pulse,
                                uint32_t tod_freq_pulse,
                                uint32_t tx_trigger_pulse);
uint64_t net_acq_ptp_current_time(void);
int      net_acq_ptp_read_txstamp(uint64_t *txstamp_ns_out);
void     net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *snap);
void     net_acq_read_dbgext_snapshot(net_acq_dbgext_snapshot_t *snap);
void     net_acq_dump_dbgext_snapshot(const net_acq_dbgext_snapshot_t *snap);
uint32_t net_acq_txs_ctrl_build(uint32_t enable,
                                uint32_t block_when_tod_invalid,
                                uint32_t strict_window);
int      net_acq_txsched_validate_config(const net_acq_txsched_cfg_t *cfg);
void     net_acq_txsched_write_shadow(const net_acq_txsched_cfg_t *cfg);
void     net_acq_read_txsched_snapshot(net_acq_txsched_snapshot_t *snap);
void     net_acq_dump_txsched_snapshot(const net_acq_txsched_snapshot_t *snap);
int      net_acq_eth_apply_config_ex(const net_acq_eth_cfg_t *cfg,
                                     const net_acq_txsched_cfg_t *txs_cfg,
                                     uint32_t poll_delay_cycles,
                                     uint32_t max_polls);
const char *net_acq_ptp_msg_type_name(uint8_t msg_type);
void     net_acq_print_u80_hex(const net_acq_u80_t *value);
void     net_acq_ptp_rx_read_snapshot(net_acq_ptp_rx_snapshot_t *snap);
void     net_acq_ptp_rx_dump_snapshot(const net_acq_ptp_rx_snapshot_t *snap);
void     net_acq_ptp_rx_mbox_pop(void);
void     net_acq_dump_eth_snapshot(const net_acq_eth_snapshot_t *snap);
int      net_acq_run_ptp_register_test(const net_acq_ptp_test_cfg_t *ptp_cfg,
                                       const net_acq_eth_cfg_t *base_eth_cfg,
                                       uint32_t poll_delay_cycles,
                                       uint32_t eth_commit_poll_rounds,
                                       uint32_t use_external_source);
void     net_acq_dump_all_registers(void);

int      net_acq_run_phase(const net_acq_phase_cfg_t *cfg);
int      net_acq_run_soc_phase(const net_acq_phase_cfg_t *cfg,
                               const net_acq_eth_cfg_t *eth_cfg,
                               uint32_t eth_commit_poll_rounds);

#endif
