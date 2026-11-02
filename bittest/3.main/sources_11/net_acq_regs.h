#ifndef NET_ACQ_REGS_H
#define NET_ACQ_REGS_H

#include <stdint.h>

/*
 * net_acq 挂载位置
 *   AXI Crossbar : axiOut_7
 *   physical     : 0x1f50_0000
 *   virtual      : 0xbf50_0000  (通过 DMW1 直映访问)
 */
#define NET_ACQ_BASE                 0xbf500000u

/* -------------------------------------------------------------------------- */
/* 基础 CSR（front-end）                                                       */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_CTRL                 0x00u
#define NET_ACQ_MODE                 0x04u
#define NET_ACQ_SAMPLE_PERIOD        0x08u
#define NET_ACQ_SAMPLES_PER_PKT      0x0Cu
#define NET_ACQ_CHANNEL_ID           0x10u
#define NET_ACQ_STATUS               0x14u
#define NET_ACQ_PACKET_COUNT         0x18u
#define NET_ACQ_LAST_TS_L            0x1Cu
#define NET_ACQ_LAST_TS_H            0x20u
#define NET_ACQ_FIFO_LEVEL           0x24u

/* -------------------------------------------------------------------------- */
/* 监测 / 调试寄存器（tx_debug_sink）                                           */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_DBG_STATUS           0x28u
#define NET_ACQ_DBG_GOOD_PKT_CNT     0x2Cu
#define NET_ACQ_DBG_BAD_PKT_CNT      0x30u
#define NET_ACQ_DBG_LAST_SEQ_ID      0x34u
#define NET_ACQ_DBG_LAST_TS_L        0x38u
#define NET_ACQ_DBG_LAST_TS_H        0x3Cu
#define NET_ACQ_DBG_LAST_COUNTS      0x40u
#define NET_ACQ_DBG_LAST_ERROR       0x44u
#define NET_ACQ_DBG_LAST_QUALITY     0x48u
#define NET_ACQ_DBG_LAST_QUALITY_STATE(v) ((uint8_t)((uint32_t)(v) & 0x3u))
#define NET_ACQ_DBG_LAST_PROTOCOL_VERSION(v) ((uint8_t)(((uint32_t)(v) >> 8) & 0xFFu))

/* -------------------------------------------------------------------------- */
/* 以太网后端 CSR（shadow config + status）                                     */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_ETH_CTRL             0x80u
#define NET_ACQ_ETH_STATUS           0x84u
#define NET_ACQ_LOCAL_MAC_L          0x88u
#define NET_ACQ_LOCAL_MAC_H          0x8Cu
#define NET_ACQ_DST_MAC_L            0x90u
#define NET_ACQ_DST_MAC_H            0x94u
#define NET_ACQ_LOCAL_IP             0x98u
#define NET_ACQ_DST_IP               0x9Cu
#define NET_ACQ_UDP_PORTS            0xA0u
#define NET_ACQ_IPV4_CFG             0xA4u
#define NET_ACQ_ETH_TX_PKT_COUNT     0xA8u
#define NET_ACQ_ETH_TX_BYTE_COUNT    0xACu
#define NET_ACQ_ETH_DROP_COUNT       0xB0u
#define NET_ACQ_ETH_LAST_LENGTHS     0xB4u
#define NET_ACQ_ETH_LAST_ERROR       0xB8u
#define NET_ACQ_ETH_FEATURES         0xBCu

/* -------------------------------------------------------------------------- */
/* PTP 预留 CSR                                                                 */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_PTP_CFG              0xC0u
#define NET_ACQ_PTP_CLOCK_ID_L       0xC4u
#define NET_ACQ_PTP_CLOCK_ID_H       0xC8u
#define NET_ACQ_PTP_PORT_NUM         0xCCu
#define NET_ACQ_PTP_TXSTAMP_L        0xD0u
#define NET_ACQ_PTP_TXSTAMP_H        0xD4u
#define NET_ACQ_PTP_STATUS           0xD8u
#define NET_ACQ_PTP_CTRL             0xDCu
#define NET_ACQ_PTP_TOD_SET_L        0xE0u
#define NET_ACQ_PTP_TOD_SET_H        0xE4u
#define NET_ACQ_PTP_STEP_NS         0xE8u
#define NET_ACQ_PTP_FREQ_ADJ        0xECu
/* 兼容旧命名 */
#define NET_ACQ_PTP_TOD_STEP        NET_ACQ_PTP_STEP_NS
#define NET_ACQ_PTP_TOD_FREQ        NET_ACQ_PTP_FREQ_ADJ
#define NET_ACQ_PTP_CUR_TIME_L       0xF0u
#define NET_ACQ_PTP_CUR_TIME_H       0xF4u
#define NET_ACQ_PTP_TX_CFG           0xF8u
#define NET_ACQ_PTP_FEATURES         0xFCu

/* -------------------------------------------------------------------------- */
/* CTRL bit 定义                                                                */
/* -------------------------------------------------------------------------- */
#define CTRL_ENABLE                  (1u << 0)
#define CTRL_SOFT_RESET              (1u << 1)
#define CTRL_CLEAR_STATUS            (1u << 2)
#define CTRL_SRC_SEL_EXT             (1u << 3)

/* -------------------------------------------------------------------------- */
/* STATUS bit 定义                                                              */
/* -------------------------------------------------------------------------- */
#define STATUS_BUSY                  (1u << 0)
#define STATUS_ERROR                 (1u << 1)

/* -------------------------------------------------------------------------- */
/* DBG_STATUS bit 定义                                                          */
/* -------------------------------------------------------------------------- */
#define DBG_STATUS_BUSY              (1u << 0)
#define DBG_STATUS_ERROR_OR          (1u << 1)
#define DBG_STATUS_CORE_ERROR_RAW    (1u << 2)
#define DBG_STATUS_SINK_ERROR_STICKY (1u << 3)

/* -------------------------------------------------------------------------- */
/* ETH_CTRL bit 定义                                                            */
/* bit1 为 commit pulse：写 1 触发，读回恒为 0                                  */
/* -------------------------------------------------------------------------- */
#define ETH_CTRL_BACKEND_ENABLE      (1u << 0)
#define ETH_CTRL_COMMIT_PULSE        (1u << 1)
#define ETH_CTRL_TX_MIRROR_ENABLE    (1u << 2)
#define ETH_CTRL_APPEND_FCS          (1u << 3)
#define ETH_CTRL_UDP_CHECKSUM_EN     (1u << 4)
#define ETH_CTRL_STALL_DROP_ENABLE   (1u << 5)

/* -------------------------------------------------------------------------- */
/* ETH_STATUS bit 定义                                                          */
/* -------------------------------------------------------------------------- */
#define ETH_STATUS_TX_BUSY           (1u << 0)
#define ETH_STATUS_TX_ERROR_STICKY   (1u << 1)
#define ETH_STATUS_PKT_BUF_OVERFLOW  (1u << 2)
#define ETH_STATUS_MAC_STALL_TIMEOUT (1u << 3)
#define ETH_STATUS_CFG_ACTIVE        (1u << 4)
#define ETH_STATUS_BACKEND_SELECTED  (1u << 5)
#define ETH_STATUS_LINK_UP           (1u << 6)
#define NET_ACQ_ETH_STATUS_LAST_ERROR(v)   ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* -------------------------------------------------------------------------- */
/* ETH_FEATURES bit 定义                                                        */
/* -------------------------------------------------------------------------- */
#define ETH_FEATURE_BACKEND_PRESENT      (1u << 0)
#define ETH_FEATURE_IPV4_UDP_TX_PRESENT  (1u << 1)
#define ETH_FEATURE_UDP_CHECKSUM_HW      (1u << 2)
#define ETH_FEATURE_APPEND_FCS_HW        (1u << 3)
#define ETH_FEATURE_PTP_SIDEBAND_PRESENT (1u << 4)
#define ETH_FEATURE_COMMIT_SHADOW        (1u << 5)
#define ETH_FEATURE_LEGACY_MIRROR        (1u << 6)
#define ETH_FEATURE_PTP_ADDR_SPLIT_PRESENT (1u << 7)
/* 兼容旧命名 */
#define ETH_FEATURE_PTP_PEER_SPLIT       ETH_FEATURE_PTP_ADDR_SPLIT_PRESENT
/* 兼容旧命名：bit4 现在明确表示 compact PTP sideband / helper path present */
#define ETH_FEATURE_PTP_RESERVED         ETH_FEATURE_PTP_SIDEBAND_PRESENT
#define NET_ACQ_ETH_FEATURE_VERSION(v)   ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* -------------------------------------------------------------------------- */
/* PTP_CFG / PTP_STATUS bit 定义                                                */
/* -------------------------------------------------------------------------- */
#define PTP_CFG_ENABLE                (1u << 0)
#define PTP_CFG_ONE_STEP              (1u << 1)
#define PTP_CFG_TWO_STEP              (1u << 2)
#define PTP_CFG_EVENT_PRIORITY_EN     (1u << 3)
#define NET_ACQ_PTP_CFG_DOMAIN(v)     ((uint8_t)(((uint32_t)(v) >> 8) & 0xFFu))

#define PTP_STATUS_TXSTAMP_VALID      (1u << 0)
#define PTP_STATUS_TXSTAMP_OVERFLOW   (1u << 1)
#define PTP_STATUS_TX_BUSY            (1u << 2)
#define PTP_STATUS_TX_CMD_PENDING     (1u << 3)
#define PTP_STATUS_CUR_TIME_VALID     (1u << 4)
#define PTP_STATUS_TOD_ENABLE         (1u << 5)
#define PTP_STATUS_TOD_HOLD           (1u << 6)
#define PTP_STATUS_PTP_ENABLE         (1u << 7)

#define PTP_CTRL_TOD_ENABLE           (1u << 0)
#define PTP_CTRL_TOD_HOLD             (1u << 1)
#define PTP_CTRL_TX_ABORT_PULSE       (1u << 2)
/* 兼容旧命名：bit2 已改为 tx_abort / clear_outstanding，不再是 TXSTAMP pop */
#define PTP_CTRL_TXSTAMP_POP_PULSE    PTP_CTRL_TX_ABORT_PULSE
#define PTP_CTRL_TOD_SET_PULSE        (1u << 3)
#define PTP_CTRL_TOD_STEP_PULSE       (1u << 4)
#define PTP_CTRL_TOD_FREQ_PULSE       (1u << 5)
#define PTP_CTRL_TX_TRIGGER_PULSE     (1u << 6)
#define PTP_CTRL_TX_NEED_TIMESTAMP    (1u << 7)
#define NET_ACQ_PTP_CTRL_MSG_TYPE(v)  ((uint8_t)(((uint32_t)(v) >> 8) & 0x0Fu))
#define NET_ACQ_PTP_CTRL_TSPEC(v)     ((uint8_t)(((uint32_t)(v) >> 12) & 0x0Fu))
#define NET_ACQ_PTP_CTRL_SEQ_ID(v)    ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

#define NET_ACQ_PTP_TX_CFG_FLAGS(v)   ((uint16_t)((uint32_t)(v) & 0xFFFFu))
#define NET_ACQ_PTP_TX_CFG_LOG_INT(v) ((uint8_t)(((uint32_t)(v) >> 16) & 0xFFu))

#define PTP_FEATURE_TOD_SYS_PRESENT       (1u << 0)
#define PTP_FEATURE_TOD_ETH_PRESENT       (1u << 1)
#define PTP_FEATURE_TX_ENGINE_PRESENT     (1u << 2)
#define PTP_FEATURE_TX_ARBITER_PRESENT    (1u << 3)
#define PTP_FEATURE_TXSTAMP_CDC_PRESENT   (1u << 4)
#define PTP_FEATURE_TX_REG_IF_PRESENT     (1u << 5)
#define PTP_FEATURE_ADDR_SPLIT_PRESENT    (1u << 6)
#define NET_ACQ_PTP_FEATURE_VERSION(v)    ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* 常用 PTP messageType 定义（当前比赛版主动 TX 固定收口到 Delay_Req） */
#define NET_ACQ_PTP_MSG_SYNC            0x0u
#define NET_ACQ_PTP_MSG_DELAY_REQ       0x1u
#define NET_ACQ_PTP_MSG_FOLLOW_UP       0x8u
#define NET_ACQ_PTP_MSG_DELAY_RESP      0x9u

/* -------------------------------------------------------------------------- */
/* PTP RX mailbox CSR                                                           */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_PTP_RX_MBOX_STATUS   0x100u
#define NET_ACQ_PTP_RX_RXTS_L        0x104u
#define NET_ACQ_PTP_RX_RXTS_H        0x108u
#define NET_ACQ_PTP_RX_MSGTS_L       0x10Cu
#define NET_ACQ_PTP_RX_MSGTS_M       0x110u
#define NET_ACQ_PTP_RX_MSGTS_H       0x114u
#define NET_ACQ_PTP_RX_REQPORT_L     0x118u
#define NET_ACQ_PTP_RX_REQPORT_M     0x11Cu
#define NET_ACQ_PTP_RX_REQPORT_H     0x120u
#define NET_ACQ_PTP_RX_MBOX_CTRL     0x124u

/* PHY management CSR */
#define NET_ACQ_PHY_CTRL             0x128u
#define NET_ACQ_PHY_STATUS           0x12Cu
#define NET_ACQ_PHY_ID               0x130u
#define NET_ACQ_PHY_BASIC            0x134u
#define NET_ACQ_PHY_AN               0x138u
#define NET_ACQ_PHY_GIG              0x13Cu

#define PHY_CTRL_SOFT_RESET_PULSE    (1u << 0)
#define PHY_CTRL_RESTART_AN_PULSE    (1u << 1)

/* -------------------------------------------------------------------------- */
/* 调度前可观测性 / debug-scheduler extension (RO, 0x140+)                     */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_DBGEXT_FEATURES                0x140u
#define NET_ACQ_DBGEXT_ARB_DATA_FRAME_COUNT    0x144u
#define NET_ACQ_DBGEXT_ARB_PTP_FRAME_COUNT     0x148u
#define NET_ACQ_DBGEXT_ARB_STALL_CYCLES        0x14Cu
#define NET_ACQ_DBGEXT_ARB_LAST_ERROR          0x150u
#define NET_ACQ_DBGEXT_BACKPRESSURE_CYCLES     0x154u
#define NET_ACQ_DBGEXT_PTP_TX_TRIGGER_REQ_CNT  0x158u
#define NET_ACQ_DBGEXT_PTP_TX_TRIGGER_ACC_CNT  0x15Cu
#define NET_ACQ_DBGEXT_PTP_TX_TRIGGER_BUSY_CNT 0x160u
#define NET_ACQ_DBGEXT_PTP_TX_LAST_ERROR       0x164u
#define NET_ACQ_DBGEXT_PTP_RX_ACCEPT_COUNT     0x168u
#define NET_ACQ_DBGEXT_PTP_RX_DROP_COUNT       0x16Cu
#define NET_ACQ_DBGEXT_PTP_RX_LAST_ERROR       0x170u
#define NET_ACQ_DBGEXT_PTP_RX_STATUS           0x174u
#define NET_ACQ_TIME_QUALITY                    0x178u

#define NET_ACQ_DBGEXT_FEATURE_PRESENT             (1u << 0)
#define NET_ACQ_DBGEXT_FEATURE_ARB_PRESENT         (1u << 1)
#define NET_ACQ_DBGEXT_FEATURE_BACKPRESS_PRESENT   (1u << 2)
#define NET_ACQ_DBGEXT_FEATURE_PTP_TX_PRESENT      (1u << 3)
#define NET_ACQ_DBGEXT_FEATURE_PTP_RX_PRESENT      (1u << 4)
#define NET_ACQ_DBGEXT_FEATURE_SCHED_CTRL_PRESENT  (1u << 5)
#define NET_ACQ_DBGEXT_FEATURE_SCHED_OBS_PRESENT   (1u << 6)
#define NET_ACQ_DBGEXT_FEATURE_VERSION(v)          ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* 兼容旧命名：当前 bit3 表示整组 PTP TX observability present */
#define NET_ACQ_DBGEXT_FEATURE_PTP_TRIG_PRESENT    NET_ACQ_DBGEXT_FEATURE_PTP_TX_PRESENT

#define NET_ACQ_DBGEXT_PTP_RX_STATUS_FIFO_OVERFLOW (1u << 0)

/* -------------------------------------------------------------------------- */
/* TSN-lite v1 scheduler shadow/config region (0x180+)                         */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_TXS_CTRL                0x180u
#define NET_ACQ_TXS_BASE_TIME_L         0x184u
#define NET_ACQ_TXS_BASE_TIME_H         0x188u
#define NET_ACQ_TXS_PERIOD_NS           0x18Cu
#define NET_ACQ_TXS_SLOT_OFFSET_NS      0x190u
#define NET_ACQ_TXS_SLOT_WIDTH_NS       0x194u
#define NET_ACQ_TXS_ADVANCE_NS          0x198u
#define NET_ACQ_TXS_LATE_TOL_NS         0x19Cu
#define NET_ACQ_TXS_STATUS              0x1A0u
#define NET_ACQ_TXS_HELD_COUNT          0x1A4u
#define NET_ACQ_TXS_PASS_COUNT          0x1A8u
#define NET_ACQ_TXS_LATE_COUNT          0x1ACu
#define NET_ACQ_TXS_SLOT_MISS_COUNT     0x1B0u
#define NET_ACQ_TXS_LAST_PLAN_L         0x1B4u
#define NET_ACQ_TXS_LAST_PLAN_H         0x1B8u
#define NET_ACQ_TXS_LAST_ACTUAL_L       0x1BCu
#define NET_ACQ_TXS_LAST_ACTUAL_H       0x1C0u
#define NET_ACQ_TXS_LAST_DELTA_NS       0x1C4u

/* -------------------------------------------------------------------------- */
/* Optional compact-PTP peer/data-dst address split shadow bank (0x1EC+)      */
/*   local_mac/local_ip: DUT local identity shared by normal UDP + compact PTP */
/*   dst_mac/dst_ip    : normal UDP data-plane destination                      */
/*   ptp_peer_mac/ip   : compact PTP peer address used by Delay_Req TX and      */
/*                       by PTP RX host-side matching when split is enabled.    */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_PTP_PEER_MAC_L         0x1ECu
#define NET_ACQ_PTP_PEER_MAC_H         0x1F0u
#define NET_ACQ_PTP_PEER_IP            0x1F4u
#define NET_ACQ_PTP_ADDR_CFG           0x1F8u
#define PTP_ADDR_CFG_SPLIT_ENABLE      (1u << 0)

#define NET_ACQ_TXS_CTRL_ENABLE                 (1u << 0)
#define NET_ACQ_TXS_CTRL_BLOCK_WHEN_TOD_INVALID (1u << 1)
#define NET_ACQ_TXS_CTRL_STRICT_WINDOW          (1u << 2)

#define NET_ACQ_TXS_STATUS_ENABLE            (1u << 0)
#define NET_ACQ_TXS_STATUS_TOD_VALID         (1u << 1)
#define NET_ACQ_TXS_STATUS_PENDING_FRAME     (1u << 2)
#define NET_ACQ_TXS_STATUS_RELEASE_ARMED     (1u << 3)
#define NET_ACQ_TXS_STATUS_FRAME_ACTIVE      (1u << 4)
#define NET_ACQ_TXS_STATUS_CFG_VALID         (1u << 5)
#define NET_ACQ_TXS_STATUS_BLOCK_ON_BAD_TIME (1u << 6)
#define NET_ACQ_TXS_STATUS_STRICT_WINDOW     (1u << 7)
#define NET_ACQ_TXS_STATUS_LAST_LATE         (1u << 8)
#define NET_ACQ_TXS_STATUS_LAST_SLOT_MISS    (1u << 9)
#define NET_ACQ_TXS_STATUS_WINDOW_OPEN       (1u << 10)
#define NET_ACQ_TXS_STATUS_BYPASS            (1u << 11)

#define PTP_RX_MBOX_STATUS_VALID        (1u << 0)
#define PTP_RX_MBOX_STATUS_OVERFLOW     (1u << 1)
#define NET_ACQ_PTP_RX_MBOX_MSG_TYPE(v)    ((uint8_t)(((uint32_t)(v) >> 4) & 0x0Fu))
#define NET_ACQ_PTP_RX_MBOX_SEQUENCE_ID(v) ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))
#define PTP_RX_MBOX_CTRL_POP            (1u << 0)

/* -------------------------------------------------------------------------- */
/* MODE 定义（按 data_gen.v 实现）                                              */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_MODE_SAWTOOTH        0u
#define NET_ACQ_MODE_TRIANGLE        1u
#define NET_ACQ_MODE_SQUARE          2u
#define NET_ACQ_MODE_CONSTANT        3u

/* 兼容旧命名 */
#define MODE_STW                     NET_ACQ_MODE_SAWTOOTH
#define MODE_SIN                     NET_ACQ_MODE_TRIANGLE
#define MODE_COS                     NET_ACQ_MODE_SQUARE

/* -------------------------------------------------------------------------- */
/* FIFO_LEVEL bitfield（按 acquisition_core.v 实现）                            */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_FIFO_INFO_SHIFT      16u
#define NET_ACQ_FIFO_INFO_MASK       0x1Fu
#define NET_ACQ_FIFO_SAMPLE_MASK     0x1FFu

#define NET_ACQ_FIFO_INFO_LEVEL(v)   (((uint32_t)(v) >> NET_ACQ_FIFO_INFO_SHIFT) & NET_ACQ_FIFO_INFO_MASK)
#define NET_ACQ_FIFO_SAMPLE_LEVEL(v) ((uint32_t)(v) & NET_ACQ_FIFO_SAMPLE_MASK)

/* -------------------------------------------------------------------------- */
/* DBG_LAST_COUNTS bitfield                                                     */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_DBG_LAST_SAMPLE_COUNT(v)   ((uint16_t)((uint32_t)(v) & 0xFFFFu))
#define NET_ACQ_DBG_LAST_PAYLOAD_COUNT(v)  ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* -------------------------------------------------------------------------- */
/* DBG_LAST_ERROR bitfield                                                      */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_DBG_LAST_ERROR_CODE(v)        ((uint16_t)((uint32_t)(v) & 0xFFFFu))
#define NET_ACQ_DBG_LAST_ERROR_WORD_INDEX(v)  ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* -------------------------------------------------------------------------- */
/* ETH_LAST_LENGTHS bitfield                                                    */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_ETH_LAST_UDP_LENGTH(v)       ((uint16_t)((uint32_t)(v) & 0xFFFFu))
#define NET_ACQ_ETH_LAST_IP_TOTAL_LENGTH(v)  ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))

/* -------------------------------------------------------------------------- */
/* tx_debug_sink 错误码                                                         */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_ERR_NONE              0u
#define NET_ACQ_ERR_MAGIC             1u
#define NET_ACQ_ERR_VERSION           2u
#define NET_ACQ_ERR_EARLY_LAST        3u
#define NET_ACQ_ERR_SEQ_MISMATCH      4u
#define NET_ACQ_ERR_TS_NON_MONO       5u
#define NET_ACQ_ERR_MISSING_LAST      6u
#define NET_ACQ_ERR_TIME_INVALID      7u

/* -------------------------------------------------------------------------- */
/* 以太网后端当前可见错误码（net_acq_pkt_ingress_adapter）                      */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_ETH_ERR_NONE             0u
#define NET_ACQ_ETH_ERR_FRAME_TOO_LONG   1u
#define NET_ACQ_ETH_ERR_SOFT_RESET_ABORT 2u

/* -------------------------------------------------------------------------- */
/* 便捷构造宏                                                                   */
/* -------------------------------------------------------------------------- */
#define NET_ACQ_MAKE_IP4(a,b,c,d) \
    ((((uint32_t)(a) & 0xFFu) << 24) | \
     (((uint32_t)(b) & 0xFFu) << 16) | \
     (((uint32_t)(c) & 0xFFu) <<  8) | \
      ((uint32_t)(d) & 0xFFu))

#define NET_ACQ_MAKE_MAC48(b5,b4,b3,b2,b1,b0) \
    ((((uint64_t)(b5) & 0xFFull) << 40) | \
     (((uint64_t)(b4) & 0xFFull) << 32) | \
     (((uint64_t)(b3) & 0xFFull) << 24) | \
     (((uint64_t)(b2) & 0xFFull) << 16) | \
     (((uint64_t)(b1) & 0xFFull) <<  8) | \
      ((uint64_t)(b0) & 0xFFull))

#endif
