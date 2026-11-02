#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include "net_acq_hal.h"
#include "confreg_time.h"
#include "lock_display.h"
#include "ptp_delay_guard.h"
#include "ptp_link_recovery.h"
#include "ptp_time_quality.h"
#include "ptp_fast50_servo.h"

/* Existing CONFREG DIGITAL_DATA register, low nibble drives the board adapter. */
static void ptp_lock_display_write(uint32_t digit)
{
    *(volatile uint32_t *)0xbf20f204u = digit ? 1u : 0u;
}

/* BSP / picolibc 依赖的全局变量 */
unsigned long UART_BASE               = 0xbf000000ul;
unsigned long CONFREG_TIMER_BASE      = 0xbf20f100ul;
unsigned long CONFREG_CLOCKS_PER_SEC  = 50000000ul;
unsigned long CORE_CLOCKS_PER_SEC     = 33000000ul;

/* -------------------------------------------------------------------------- */
/* 固定网络参数：请只在这里配置三组网络身份                                     */
/* 1) DUT 本机身份：本板卡自己的 MAC / IP                                      */
/* 2) UDP 数据对端：普通 UDP 数据流发往的上位机 MAC / IP                        */
/* 3) PTP 对端：用于时间同步交互的 observe 板 MAC / IP                          */
/*                                                                            */
/* 说明：                                                                     */
/* - 本版不再兼容旧的 NET_ACQ_HOST_* 宏，也不再做任何回退映射。                */
/* - 这三组参数请分别明确配置，避免 UDP 和 PTP 目标地址混淆。                  */
/* -------------------------------------------------------------------------- */

/* ============================== 1) DUT 本机身份 ============================ */
#ifndef NET_ACQ_DUT_LOCAL_MAC_B5
#define NET_ACQ_DUT_LOCAL_MAC_B5 0x00
#endif
#ifndef NET_ACQ_DUT_LOCAL_MAC_B4
#define NET_ACQ_DUT_LOCAL_MAC_B4 0x0A
#endif
#ifndef NET_ACQ_DUT_LOCAL_MAC_B3
#define NET_ACQ_DUT_LOCAL_MAC_B3 0x35
#endif
#ifndef NET_ACQ_DUT_LOCAL_MAC_B2
#define NET_ACQ_DUT_LOCAL_MAC_B2 0x01
#endif
#ifndef NET_ACQ_DUT_LOCAL_MAC_B1
#define NET_ACQ_DUT_LOCAL_MAC_B1 0xFE
#endif
#ifndef NET_ACQ_DUT_LOCAL_MAC_B0
#define NET_ACQ_DUT_LOCAL_MAC_B0 0xC0
#endif

#ifndef NET_ACQ_DUT_LOCAL_IP_A
#define NET_ACQ_DUT_LOCAL_IP_A 192
#endif
#ifndef NET_ACQ_DUT_LOCAL_IP_B
#define NET_ACQ_DUT_LOCAL_IP_B 168
#endif
#ifndef NET_ACQ_DUT_LOCAL_IP_C
#define NET_ACQ_DUT_LOCAL_IP_C 0
#endif
#ifndef NET_ACQ_DUT_LOCAL_IP_D
#define NET_ACQ_DUT_LOCAL_IP_D 14
#endif

/* ========================== 2) UDP 数据对端（上位机） ======================= */
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B5
#define NET_ACQ_UDP_DATA_DST_MAC_B5 0x00
#endif
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B4
#define NET_ACQ_UDP_DATA_DST_MAC_B4 0xE0
#endif
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B3
#define NET_ACQ_UDP_DATA_DST_MAC_B3 0x4C
#endif
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B2
#define NET_ACQ_UDP_DATA_DST_MAC_B2 0x20
#endif
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B1
#define NET_ACQ_UDP_DATA_DST_MAC_B1 0x52
#endif
#ifndef NET_ACQ_UDP_DATA_DST_MAC_B0
#define NET_ACQ_UDP_DATA_DST_MAC_B0 0x28
#endif

#ifndef NET_ACQ_UDP_DATA_DST_IP_A
#define NET_ACQ_UDP_DATA_DST_IP_A 192
#endif
#ifndef NET_ACQ_UDP_DATA_DST_IP_B
#define NET_ACQ_UDP_DATA_DST_IP_B 168
#endif
#ifndef NET_ACQ_UDP_DATA_DST_IP_C
#define NET_ACQ_UDP_DATA_DST_IP_C 0
#endif
#ifndef NET_ACQ_UDP_DATA_DST_IP_D
#define NET_ACQ_UDP_DATA_DST_IP_D 3
#endif

/* =========================== 3) PTP 对端（observe） ======================== */
#ifndef NET_ACQ_PTP_PEER_MAC_B5
#define NET_ACQ_PTP_PEER_MAC_B5 0x00
#endif
#ifndef NET_ACQ_PTP_PEER_MAC_B4
#define NET_ACQ_PTP_PEER_MAC_B4 0x0A
#endif
#ifndef NET_ACQ_PTP_PEER_MAC_B3
#define NET_ACQ_PTP_PEER_MAC_B3 0x35
#endif
#ifndef NET_ACQ_PTP_PEER_MAC_B2
#define NET_ACQ_PTP_PEER_MAC_B2 0x01
#endif
#ifndef NET_ACQ_PTP_PEER_MAC_B1
#define NET_ACQ_PTP_PEER_MAC_B1 0xFE
#endif
#ifndef NET_ACQ_PTP_PEER_MAC_B0
#define NET_ACQ_PTP_PEER_MAC_B0 0xAA
#endif

#ifndef NET_ACQ_PTP_PEER_IP_A
#define NET_ACQ_PTP_PEER_IP_A 192
#endif
#ifndef NET_ACQ_PTP_PEER_IP_B
#define NET_ACQ_PTP_PEER_IP_B 168
#endif
#ifndef NET_ACQ_PTP_PEER_IP_C
#define NET_ACQ_PTP_PEER_IP_C 0
#endif
#ifndef NET_ACQ_PTP_PEER_IP_D
#define NET_ACQ_PTP_PEER_IP_D 5
#endif

#ifndef NET_ACQ_PTP_ADDR_SPLIT_ENABLE
#define NET_ACQ_PTP_ADDR_SPLIT_ENABLE 1u
#endif
/* -------------------------------------------------------------------------- */
/* acquisition UDP 持续发送参数                                                */
/* -------------------------------------------------------------------------- */
#ifndef UDP_STREAM_SRC_PORT
#define UDP_STREAM_SRC_PORT 1234u
#endif
#ifndef UDP_STREAM_DST_PORT
#define UDP_STREAM_DST_PORT 6000u
#endif
#ifndef UDP_STREAM_MODE
#define UDP_STREAM_MODE NET_ACQ_MODE_SAWTOOTH
#endif
#ifndef UDP_STREAM_SAMPLE_PERIOD
/* PTP-only 第一轮不启用普通数据流；保留低速安全默认值，便于后续手动开流验证。 */
#define UDP_STREAM_SAMPLE_PERIOD 64u
#endif
#ifndef UDP_STREAM_SAMPLES_PER_PACKET
#define UDP_STREAM_SAMPLES_PER_PACKET 168u
#endif
#ifndef UDP_STREAM_CHANNEL_ID
#define UDP_STREAM_CHANNEL_ID 1u
#endif
#ifndef UDP_STREAM_USE_EXTERNAL_SOURCE
#define UDP_STREAM_USE_EXTERNAL_SOURCE 0u
#endif
#ifndef UDP_STREAM_TTL
#define UDP_STREAM_TTL 64u
#endif
#ifndef UDP_STREAM_DSCP_ECN
#define UDP_STREAM_DSCP_ECN 0u
#endif
#ifndef UDP_STREAM_IP_IDENT_SEED
#define UDP_STREAM_IP_IDENT_SEED 0x1357u
#endif
#ifndef UDP_STREAM_ENABLE_BACKEND
#define UDP_STREAM_ENABLE_BACKEND 1u
#endif
#ifndef UDP_STREAM_TX_MIRROR_ENABLE
#define UDP_STREAM_TX_MIRROR_ENABLE 1u
#endif
#ifndef UDP_STREAM_APPEND_FCS_REQUEST
#define UDP_STREAM_APPEND_FCS_REQUEST 0u
#endif
#ifndef UDP_STREAM_UDP_CHECKSUM_REQUEST
#define UDP_STREAM_UDP_CHECKSUM_REQUEST 0u
#endif
#ifndef UDP_STREAM_STALL_DROP_REQUEST
#define UDP_STREAM_STALL_DROP_REQUEST 0u
#endif

/* -------------------------------------------------------------------------- */
/* TSN-lite v1 —— 时隙调度器（默认关闭，按需启用）                               */
/* -------------------------------------------------------------------------- */
#ifndef TSN_LITE_V1_ENABLE
#define TSN_LITE_V1_ENABLE 0u
#endif
#ifndef TSN_LITE_V1_BLOCK_WHEN_TOD_INVALID
#define TSN_LITE_V1_BLOCK_WHEN_TOD_INVALID 1u
#endif
#ifndef TSN_LITE_V1_STRICT_WINDOW
#define TSN_LITE_V1_STRICT_WINDOW 1u
#endif
#ifndef TSN_LITE_V1_PERIOD_NS
#define TSN_LITE_V1_PERIOD_NS 1000000u
#endif
#ifndef TSN_LITE_V1_SLOT_OFFSET_NS
#define TSN_LITE_V1_SLOT_OFFSET_NS 0u
#endif
#ifndef TSN_LITE_V1_SLOT_WIDTH_NS
#define TSN_LITE_V1_SLOT_WIDTH_NS 200000u
#endif
#ifndef TSN_LITE_V1_ADVANCE_NS
#define TSN_LITE_V1_ADVANCE_NS 0u
#endif
#ifndef TSN_LITE_V1_LATE_TOL_NS
#define TSN_LITE_V1_LATE_TOL_NS 5000u
#endif
#ifndef TSN_LITE_V1_START_LEAD_PERIODS
#define TSN_LITE_V1_START_LEAD_PERIODS 2u
#endif
#ifndef TSN_LITE_V1_DUMP_ON_ARM
#define TSN_LITE_V1_DUMP_ON_ARM 1u
#endif

/* -------------------------------------------------------------------------- */
/* PTP 本地节点配置                                                            */
/* -------------------------------------------------------------------------- */
#ifndef PTP_LOCAL_DOMAIN
#define PTP_LOCAL_DOMAIN 0u
#endif
#ifndef PTP_LOCAL_PORT_NUM
#define PTP_LOCAL_PORT_NUM 1u
#endif
#ifndef PTP_LOCAL_CLOCK_ID
#define PTP_LOCAL_CLOCK_ID 0x000A3501FEC00001ull
#endif
#ifndef PTP_TOD_SEED_NS
#define PTP_TOD_SEED_NS 1000000000ull
#endif

/* -------------------------------------------------------------------------- */
/* 闭环验证参数                                                                */
/* -------------------------------------------------------------------------- */
#ifndef PTP_POLL_DELAY_CYCLES
#define PTP_POLL_DELAY_CYCLES 1000u
#endif
#ifndef PTP_CUR_TIME_VALID_POLLS
#define PTP_CUR_TIME_VALID_POLLS 256u
#endif
#ifndef PTP_TXSTAMP_WAIT_POLLS
#define PTP_TXSTAMP_WAIT_POLLS 5000u
#endif
#ifndef PTP_TX_IDLE_WAIT_POLLS
#define PTP_TX_IDLE_WAIT_POLLS 5000u
#endif
#ifndef PTP_ENABLE_SETTLE_CYCLES
#define PTP_ENABLE_SETTLE_CYCLES 100000u
#endif
#ifndef PTP_STARTUP_MBOX_DRAIN_LIMIT
#define PTP_STARTUP_MBOX_DRAIN_LIMIT 16u
#endif
#ifndef PTP_HEARTBEAT_POLLS
#define PTP_HEARTBEAT_POLLS 20000u
#endif
#ifndef PTP_FOLLOW_UP_TIMEOUT_POLLS
#define PTP_FOLLOW_UP_TIMEOUT_POLLS 300000u
#endif
#ifndef PTP_DELAY_RESP_TIMEOUT_POLLS
#define PTP_DELAY_RESP_TIMEOUT_POLLS 300000u
#endif
#ifndef PTP_STALL_HEARTBEATS
#define PTP_STALL_HEARTBEATS 8u
#endif
#ifndef PTP_REQUIRE_LINK_UP
#define PTP_REQUIRE_LINK_UP 0u
#endif
#ifndef PTP_RESTART_BACKOFF_CYCLES
#define PTP_RESTART_BACKOFF_CYCLES 500000u
#endif

/* -------------------------------------------------------------------------- */
/* 第一轮实验模式：只跑 PTP，不开启 acquisition/UDP stream                       */
/* -------------------------------------------------------------------------- */
#ifndef PTP_ONLY_MODE
#define PTP_ONLY_MODE 0u
#endif
#ifndef PTP_STREAM_REQUIRE_LOCK
#define PTP_STREAM_REQUIRE_LOCK 1u
#endif
#ifndef PTP_OFFSET_CAL_NS
/* 固定偏差补偿项。只用于最终校零：
 * raw offset 仍然照常打印与记录，servo 使用的是
 * offset_used = offset_raw - PTP_OFFSET_CAL_NS。
 * 若控制环已收敛到小范围后仍存在稳定 DC 偏差，再微调这里的值。
 */
#define PTP_OFFSET_CAL_NS (0ll)
#endif

/*
 * 最新 RTL 中，PTP_FREQ_ADJ 的统一语义已经改成 signed ppb。
 * 同一个 ppb 会在 sys(20ns) / eth_tx(8ns) / eth_rx(8ns) 域内分别换算，
 * 因此软件 servo 不应再把它当作 Q16 ns/tick。
 */
#ifndef PTP_STEP_THRESHOLD_NS
/*
 * 第一轮 PTP-only 使用硬件时间戳闭环，不再沿用 Windows/软件时间戳 bring-up 的 ms 级阈值。
 * 启动期只在 residual offset 超过 400us 时才做 full step；100~200us 级偏差优先交给阻尼 trim。
 */
#define PTP_STEP_THRESHOLD_NS 400000ll
#endif
#ifndef PTP_REACQUIRE_STEP_THRESHOLD_NS
/* 正常运行后只有超过 1ms 才重新 step；第一轮 PTP-only 默认不开流。 */
#define PTP_REACQUIRE_STEP_THRESHOLD_NS 1000000ll
#endif
#ifndef PTP_TRIM_THRESHOLD_NS
/* 10us 以上但尚未达到 full-step 区间时，使用阻尼 phase trim 快速拉近。 */
#define PTP_TRIM_THRESHOLD_NS 8000ll
#endif
#ifndef PTP_TRIM_GAIN_NUM
#define PTP_TRIM_GAIN_NUM 3ll
#endif
#ifndef PTP_TRIM_GAIN_DEN
#define PTP_TRIM_GAIN_DEN 4ll
#endif
#ifndef PTP_TRIM_MAX_NS
#define PTP_TRIM_MAX_NS 200000ll
#endif
#ifndef PTP_LOCK_THRESHOLD_NS
/* usable-ready 锁定阈值：10us。系统可用不必等待最终亚微秒稳态。 */
#define PTP_LOCK_THRESHOLD_NS 10000ll
#endif
#ifndef PTP_LOCK_STREAK_MIN
/* 需要连续若干轮满足 lock threshold，避免单次偶然样本直接置 lock。 */
#define PTP_LOCK_STREAK_MIN 8u
#endif
#ifndef PTP_FREQ_ADJ_LIMIT_PPB
/* 约等于旧 sys-domain 2048(Q16 ns/tick) 钳位对应的相对频偏上限 */
#define PTP_FREQ_ADJ_LIMIT_PPB 1562500
#endif
#ifndef PTP_SET_TIME_THRESHOLD_NS
/*
 * 大偏差时不要再用 32-bit step 硬拉，直接改用 64-bit set_time。
 * 这能解决本地 ToD 仍停留在 seed，而主机已经是 Unix epoch 时
 * 长时间追不上的问题。
 */
#define PTP_SET_TIME_THRESHOLD_NS 1000000000ll
#endif
#ifndef PTP_STREAM_START_AFTER_SERVO_CYCLES
/* 仅在非 PTP-only 模式下使用；当前至少等待 12 个完整 PTP exchange 后才允许开普通流。 */
#define PTP_STREAM_START_AFTER_SERVO_CYCLES 12u
#endif
#ifndef PTP_STREAM_START_MAX_OFFSET_NS
#define PTP_STREAM_START_MAX_OFFSET_NS 10000ll
#endif
#ifndef PTP_OFFSET_FILTER_NUM
#define PTP_OFFSET_FILTER_NUM 7ll
#endif
#ifndef PTP_OFFSET_FILTER_DEN
#define PTP_OFFSET_FILTER_DEN 8ll
#endif
#ifndef PTP_FREQ_GAIN_DIV
/* 相位到频偏的环路增益分母；值越大，调频越温和 */
#define PTP_FREQ_GAIN_DIV 32ll
#endif
#ifndef PTP_FREQ_FILTER_NUM
#define PTP_FREQ_FILTER_NUM 1ll
#endif
#ifndef PTP_FREQ_FILTER_DEN
#define PTP_FREQ_FILTER_DEN 2ll
#endif
#ifndef PTP_FREQ_FILTER_TRACK_NUM
/* ready/track 段不再额外低通输出频调，优先缩短长尾。 */
#define PTP_FREQ_FILTER_TRACK_NUM 0ll
#endif
#ifndef PTP_FREQ_FILTER_TRACK_DEN
#define PTP_FREQ_FILTER_TRACK_DEN 1ll
#endif
#ifndef PTP_FREQ_FILTER_FINE_ACQ_NUM
#define PTP_FREQ_FILTER_FINE_ACQ_NUM 0ll
#endif
#ifndef PTP_FREQ_FILTER_FINE_ACQ_DEN
#define PTP_FREQ_FILTER_FINE_ACQ_DEN 1ll
#endif
#ifndef PTP_FREQ_FILTER_FINE_NUM
#define PTP_FREQ_FILTER_FINE_NUM 1ll
#endif
#ifndef PTP_FREQ_FILTER_FINE_DEN
#define PTP_FREQ_FILTER_FINE_DEN 2ll
#endif
#ifndef PTP_DRIFT_FILTER_NUM
#define PTP_DRIFT_FILTER_NUM 3ll
#endif
#ifndef PTP_DRIFT_FILTER_DEN
#define PTP_DRIFT_FILTER_DEN 4ll
#endif
#ifndef PTP_FREQ_HOLD_DRIFT_PPB
#define PTP_FREQ_HOLD_DRIFT_PPB 200
#endif
#ifndef PTP_LOCK_DRIFT_THRESHOLD_PPB
#define PTP_LOCK_DRIFT_THRESHOLD_PPB 1500
#endif
#ifndef PTP_FINE_LOCK_THRESHOLD_NS
/* 精细锁定：用于标记高精度稳态完成，不作为“系统可用”的必要条件。 */
#define PTP_FINE_LOCK_THRESHOLD_NS (PTP_FAST50_ENABLE ? 50ll : 900ll)
#endif
#ifndef PTP_FINE_LOCK_STREAK_MIN
#define PTP_FINE_LOCK_STREAK_MIN (PTP_FAST50_ENABLE ? 8u : 12u)
#endif
#ifndef PTP_FINE_LOCK_DRIFT_THRESHOLD_PPB
#define PTP_FINE_LOCK_DRIFT_THRESHOLD_PPB 100
#endif
#ifndef PTP_DELAY_LOCK_MIN_NS
#define PTP_DELAY_LOCK_MIN_NS 0ll
#endif
#ifndef PTP_DELAY_LOCK_MAX_NS
#define PTP_DELAY_LOCK_MAX_NS 500000ll
#endif
#ifndef PTP_DELAY_LOCK_SLOPE_MAX_NS
#define PTP_DELAY_LOCK_SLOPE_MAX_NS 2000ll
#endif
#ifndef PTP_STREAM_START_MAX_DRIFT_PPB
#define PTP_STREAM_START_MAX_DRIFT_PPB 1500
#endif
#ifndef PTP_ACQ_OFFSET_FILTER_NUM
/* acquisition 段对 offset 采用更快的 LPF，减少粗对齐阶段因滤波滞后引起的过冲。 */
#define PTP_ACQ_OFFSET_FILTER_NUM 1ll
#endif
#ifndef PTP_ACQ_OFFSET_FILTER_DEN
#define PTP_ACQ_OFFSET_FILTER_DEN 2ll
#endif
#ifndef PTP_READY_OFFSET_FILTER_NUM
/* ready 之后先切到中等速度 LPF，避免一上来直接切到 7/8 造成长尾。 */
#define PTP_READY_OFFSET_FILTER_NUM 3ll
#endif
#ifndef PTP_READY_OFFSET_FILTER_DEN
#define PTP_READY_OFFSET_FILTER_DEN 4ll
#endif
#ifndef PTP_CONTROL_BUILD_ID
#define PTP_CONTROL_BUILD_ID 0x20261027u
#endif
/* 对称路径延迟的基本健康度窗口，用来屏蔽明显异常的四时间戳样本。 */
#ifndef PTP_DELAY_FILTER_NUM
#define PTP_DELAY_FILTER_NUM 7ll
#endif
#ifndef PTP_DELAY_FILTER_DEN
#define PTP_DELAY_FILTER_DEN 8ll
#endif
#ifndef PTP_DELAY_MIN_NS
#define PTP_DELAY_MIN_NS 0ll
#endif
#ifndef PTP_DELAY_MAX_NS
#define PTP_DELAY_MAX_NS 500000ll
#endif
#ifndef PTP_DELAY_OUTLIER_NS
#define PTP_DELAY_OUTLIER_NS 50000ll
#endif
#ifndef PTP_DELAY_REJECT_SLOPE_MAX_NS
/* 每个有效 exchange 的 delay 变化若超过 5us，优先按异常样本拒绝。 */
#define PTP_DELAY_REJECT_SLOPE_MAX_NS 5000ll
#endif
#ifndef PTP_FREQ_HOLD_THRESHOLD_NS
/* 进入 2us 内才认为相位足够接近；更大的 residual offset 仍需参与伺服。 */
#define PTP_FREQ_HOLD_THRESHOLD_NS 2000ll
#endif
#ifndef PTP_HOLD_OFFSET_THRESHOLD_NS
/* 已锁定后的保持区：小于该阈值时不再强积分相位误差，只保持/微调频偏 bias。 */
#define PTP_HOLD_OFFSET_THRESHOLD_NS 5000ll
#endif
#ifndef PTP_HOLD_DRIFT_THRESHOLD_PPB
#define PTP_HOLD_DRIFT_THRESHOLD_PPB 200
#endif
#ifndef PTP_FREQ_GAIN_DIV_LOCK
/* 已锁定后的相位到频偏增益分母；比 acquisition 阶段更温和，抑制长时间 hunting。 */
#define PTP_FREQ_GAIN_DIV_LOCK 64ll
#endif
/* Acquisition frequency observer uses accepted RAW offsets, removing the
 * frequency correction that was applied over that same measurement interval.
 * Avoid integrating filtered phase into oscillator bias during acquisition. */
#ifndef PTP_ACQUIRE_OBSERVER_DIV
#define PTP_ACQUIRE_OBSERVER_DIV 8ll
#endif
#ifndef PTP_ACQUIRE_BIAS_STEP_MAX_PPB
#define PTP_ACQUIRE_BIAS_STEP_MAX_PPB 2048ll
#endif
#ifndef PTP_ACQUIRE_PHASE_TIME_NS
#define PTP_ACQUIRE_PHASE_TIME_NS 2000000000ll
#endif
#ifndef PTP_ACQUIRE_DIRECT_MAX_PPB
#define PTP_ACQUIRE_DIRECT_MAX_PPB 4096ll
#endif
#ifndef PTP_BIAS_ACC_DIV
/* phase_term 注入长期频率 bias 的比例；值越大，积分越慢。 */
#define PTP_BIAS_ACC_DIV 4ll
#endif
#ifndef PTP_PHASE_DIRECT_DIV
/* phase_term 直接参与当轮输出的比例；长期偏置由 freq_bias_ppb 承担。 */
#define PTP_PHASE_DIRECT_DIV 8ll
#endif
#ifndef PTP_BIAS_UPDATE_MAX_PPB
/* 单个 PTP exchange 允许写入 bias 的最大幅度，防止瞬时样本造成积分 wind-up。 */
#define PTP_BIAS_UPDATE_MAX_PPB 256
#endif
#ifndef PTP_DIRECT_PHASE_MAX_PPB
#define PTP_DIRECT_PHASE_MAX_PPB 2048
#endif
#ifndef PTP_TRACK_BIAS_ACC_DIV
/* 已锁定但尚未进入 fine zone 时的慢速 bias 注入；比 acquisition 更温和。 */
#define PTP_TRACK_BIAS_ACC_DIV 8ll
#endif
#ifndef PTP_TRACK_BIAS_UPDATE_MAX_PPB
#define PTP_TRACK_BIAS_UPDATE_MAX_PPB 96
#endif
#ifndef PTP_TRACK_PHASE_DIRECT_DIV
#define PTP_TRACK_PHASE_DIRECT_DIV 4ll
#endif
#ifndef PTP_TRACK_DIRECT_PHASE_MAX_PPB
#define PTP_TRACK_DIRECT_PHASE_MAX_PPB 512
#endif
#ifndef PTP_FINE_OFFSET_THRESHOLD_NS
/* 细跟踪区：进入该范围后切入尾段控制；达到 fine_lock 之前先用稍快的 fine-acq 参数收尾。 */
#define PTP_FINE_OFFSET_THRESHOLD_NS 2500ll
#endif
#ifndef PTP_FINE_DRIFT_THRESHOLD_PPB
#define PTP_FINE_DRIFT_THRESHOLD_PPB 200
#endif
/* Fine acquisition and hold share a damped PI tuning at the nominal 200 ms interval.
 * Sub-ppb integral residue is preserved; changing the lock flag must not switch gains. */
#ifndef PTP_FREQ_GAIN_DIV_FINE_ACQ
#define PTP_FREQ_GAIN_DIV_FINE_ACQ 32ll
#endif
#ifndef PTP_FINE_ACQ_BIAS_ACC_DIV
#define PTP_FINE_ACQ_BIAS_ACC_DIV 64ll
#endif
#ifndef PTP_FINE_ACQ_BIAS_UPDATE_MAX_PPB
#define PTP_FINE_ACQ_BIAS_UPDATE_MAX_PPB 48
#endif
#ifndef PTP_FINE_ACQ_PHASE_DIRECT_DIV
#define PTP_FINE_ACQ_PHASE_DIRECT_DIV 1ll
#endif
#ifndef PTP_FINE_ACQ_DIRECT_PHASE_MAX_PPB
#define PTP_FINE_ACQ_DIRECT_PHASE_MAX_PPB 256
#endif
#ifndef PTP_FREQ_GAIN_DIV_FINE
#define PTP_FREQ_GAIN_DIV_FINE 32ll
#endif
#ifndef PTP_FINE_BIAS_ACC_DIV
#define PTP_FINE_BIAS_ACC_DIV 64ll
#endif
#ifndef PTP_FINE_BIAS_UPDATE_MAX_PPB
#define PTP_FINE_BIAS_UPDATE_MAX_PPB 8
#endif
#ifndef PTP_FINE_PHASE_DIRECT_DIV
#define PTP_FINE_PHASE_DIRECT_DIV 1ll
#endif
#ifndef PTP_FINE_DIRECT_PHASE_MAX_PPB
#define PTP_FINE_DIRECT_PHASE_MAX_PPB 256
#endif
#ifndef PTP_FREEZE_AFTER_DROP_CYCLES
/* 出现 cycle drop / timeout / malformed 后，至少冻结若干个有效 exchange，避免坏样本刚过去就立刻重积分。 */
#define PTP_FREEZE_AFTER_DROP_CYCLES 4u
#endif
#ifndef PTP_POST_ADJUST_HOLDOFF_CYCLES
/* step / set64 后额外留两个 exchange 让时间基和 mailbox 状态先稳定，再恢复 drift/phase 更新。 */
#define PTP_POST_ADJUST_HOLDOFF_CYCLES 2u
#endif
/* 收到有效 mailbox 后，优先立刻再轮询一次，减少单 outstanding mailbox 溢出概率。 */
#ifndef PTP_IMMEDIATE_REPOLL_AFTER_RX
#define PTP_IMMEDIATE_REPOLL_AFTER_RX 1u
#endif
#ifndef PTP_VERBOSE_RX
#define PTP_VERBOSE_RX 0u
#endif
#ifndef PTP_UART_EXCHANGE_TRACE
#define PTP_UART_EXCHANGE_TRACE 0u
#endif
#ifndef PTP_VALIDATE_RX_MSG_TIMESTAMP
#define PTP_VALIDATE_RX_MSG_TIMESTAMP 1u
#endif
#ifndef PTP_DELAY_REQ_SEQ_START
#define PTP_DELAY_REQ_SEQ_START 1u
#endif
#ifndef PTP_TX_TRIGGER_FAIL_RESTART_THRESHOLD
#define PTP_TX_TRIGGER_FAIL_RESTART_THRESHOLD 8u
#endif

typedef enum {
    PTP_WAIT_SYNC = 0,
    PTP_WAIT_FOLLOW_UP,
    PTP_WAIT_DELAY_RESP
} ptp_slave_state_t;

typedef enum {
    PTP_SERVO_QUAL_OK = 0,
    PTP_SERVO_QUAL_DELAY_RANGE,
    PTP_SERVO_QUAL_DELAY_OUTLIER,
    PTP_SERVO_QUAL_DELAY_SLOPE,
    PTP_SERVO_QUAL_PDV_SPIKE,
    PTP_SERVO_QUAL_PATH_REBASE
} ptp_servo_quality_t;

typedef struct {
    ptp_slave_state_t state;
    ptp_delay_guard_t delay_guard;
    uint32_t last_sample_valid;
    ptp_servo_quality_t last_sample_quality;
    int64_t last_delay_ref_ns, last_delay_margin_ns;
    uint16_t sync_seq_id;
    uint16_t delay_req_seq_id;
    uint16_t next_delay_req_seq_id;
    uint32_t delay_req_trigger_fail_count;
    uint32_t invalid_follow_up_count;
    uint32_t invalid_delay_resp_count;
    uint32_t delay_resp_port_mismatch_count;
    uint32_t cycle_drop_count;
    uint32_t txstamp_timeout_streak;
    uint64_t t1_master_ns;
    uint64_t t2_slave_rx_ns;
    uint64_t t3_slave_tx_ns;
    uint64_t t4_master_rx_ns;
    uint64_t prev_t1_master_ns;
    uint32_t have_prev_t1;
    int32_t  freq_adj_ppb;
    int32_t  freq_bias_ppb;
    int64_t  fine_bias_remainder; /* numerator remainder; one ppb = PTP_FINE_BIAS_ACC_DIV */
    int32_t  filtered_drift_ppb;
    uint32_t have_filtered_drift;
    int64_t  filtered_offset_ns;
    uint32_t have_filtered_offset;
    int64_t  filtered_delay_ns;
    uint32_t have_filtered_delay;
    int64_t  last_offset_ns;
    int64_t  last_delay_ns;
    int64_t  last_d1_ns;
    int64_t  last_d2_ns;
    int64_t  last_drift_ppb;
    int64_t  last_phase_ppb;
    int64_t  last_direct_phase_ppb;
    int64_t  last_bias_update_ppb;
    int64_t  last_target_freq_ppb;
    int64_t  last_delay_slope_ns;
    uint32_t servo_cycle_count;
    uint32_t rejected_exchange_count;
    uint32_t delay_outlier_count;
    uint32_t freeze_servo_cycles;
    uint32_t servo_freeze_event_count;
    uint32_t post_adjust_holdoff_cycles;
    uint32_t lock_streak;
    uint32_t locked;
    uint32_t fine_lock_streak;
    uint32_t fine_locked;
    uint32_t startup_done;
    ptp_fast50_servo_t fast50;
    uint32_t stream_started;
    uint32_t ptp_rx_count;
    uint32_t mailbox_overflow_count;
    uint32_t follow_up_timeout_polls;
    uint32_t delay_resp_timeout_polls;
} ptp_slave_ctx_t;

static void print_mac48(uint64_t mac)
{
    printf("%02x-%02x-%02x-%02x-%02x-%02x",
           (unsigned int)((mac >> 40) & 0xFFu),
           (unsigned int)((mac >> 32) & 0xFFu),
           (unsigned int)((mac >> 24) & 0xFFu),
           (unsigned int)((mac >> 16) & 0xFFu),
           (unsigned int)((mac >> 8) & 0xFFu),
           (unsigned int)(mac & 0xFFu));
}

static void print_ip4(uint32_t ip)
{
    printf("%u.%u.%u.%u",
           (unsigned int)((ip >> 24) & 0xFFu),
           (unsigned int)((ip >> 16) & 0xFFu),
           (unsigned int)((ip >> 8) & 0xFFu),
           (unsigned int)(ip & 0xFFu));
}

static uint32_t ptp_u80_equal(const net_acq_u80_t *a,
                              const net_acq_u80_t *b)
{
    if (!a || !b) {
        return 0u;
    }

    return ((a->hi16 == b->hi16) && (a->lo64 == b->lo64)) ? 1u : 0u;
}

static uint32_t ptp_u80_timestamp_is_valid(const net_acq_u80_t *value)
{
    uint32_t nanoseconds;

    if (!value) {
        return 0u;
    }

    nanoseconds = (uint32_t)(value->lo64 & 0xFFFFFFFFull);
    return (nanoseconds < 1000000000u) ? 1u : 0u;
}

static net_acq_u80_t ptp_build_request_port_identity(const net_acq_ptp_test_cfg_t *ptp_cfg)
{
    net_acq_u80_t id;
    uint64_t clock_id;

    id.hi16 = 0u;
    id.lo64 = 0ull;
    if (!ptp_cfg) {
        return id;
    }

    clock_id = ptp_cfg->clock_id;
    id.hi16 = (uint16_t)((clock_id >> 48) & 0xFFFFull);
    id.lo64 = ((clock_id & 0x0000FFFFFFFFFFFFull) << 16) |
              (uint64_t)(ptp_cfg->port_num & 0xFFFFu);
    return id;
}

static uint16_t ptp_next_delay_req_seq_id(ptp_slave_ctx_t *ctx)
{
    uint16_t seq_id;

    if (!ctx) {
        return (uint16_t)PTP_DELAY_REQ_SEQ_START;
    }

    seq_id = ctx->next_delay_req_seq_id;
    if (seq_id == 0u) {
        seq_id = (uint16_t)PTP_DELAY_REQ_SEQ_START;
    }

    ctx->next_delay_req_seq_id = (uint16_t)(seq_id + 1u);
    if (ctx->next_delay_req_seq_id == 0u) {
        ctx->next_delay_req_seq_id = (uint16_t)PTP_DELAY_REQ_SEQ_START;
    }

    return seq_id;
}

static uint64_t ptp_u80_to_ns(const net_acq_u80_t *value)
{
    uint64_t seconds;
    uint32_t nanoseconds;

    if (!value) {
        return 0ull;
    }

    seconds     = (((uint64_t)value->hi16) << 32) | (value->lo64 >> 32);
    nanoseconds = (uint32_t)(value->lo64 & 0xFFFFFFFFull);
    return seconds * 1000000000ull + (uint64_t)nanoseconds;
}

static uint32_t ptp_u80_to_ns_checked(const net_acq_u80_t *value, uint64_t *ns_out)
{
    uint64_t seconds;
    uint32_t nanoseconds;

    if (!ptp_u80_timestamp_is_valid(value)) {
        return 0u;
    }

    seconds     = (((uint64_t)value->hi16) << 32) | (value->lo64 >> 32);
    nanoseconds = (uint32_t)(value->lo64 & 0xFFFFFFFFull);
    if (seconds > (UINT64_MAX - (uint64_t)nanoseconds) / 1000000000ull) {
        return 0u;
    }

    if (ns_out) {
        *ns_out = seconds * 1000000000ull + (uint64_t)nanoseconds;
    }
    return 1u;
}

static void print_ptp_status_brief(const char *tag, uint32_t status)
{
    printf("[%s] PTP_STATUS=0x%08x (txstamp_valid=%u, tx_busy=%u, tx_cmd_pending=%u, cur_time_valid=%u, ptp_enable=%u)\n",
           tag ? tag : "ptp",
           (unsigned int)status,
           (status & PTP_STATUS_TXSTAMP_VALID) ? 1u : 0u,
           (status & PTP_STATUS_TX_BUSY) ? 1u : 0u,
           (status & PTP_STATUS_TX_CMD_PENDING) ? 1u : 0u,
           (status & PTP_STATUS_CUR_TIME_VALID) ? 1u : 0u,
           (status & PTP_STATUS_PTP_ENABLE) ? 1u : 0u);
}
static int64_t i64_abs(int64_t v)
{
    return (v < 0) ? -v : v;
}

static int64_t div_round_nearest_s64(int64_t num, int64_t den)
{
    int64_t half;

    if (den == 0) {
        return 0;
    }

    half = den / 2;
    if (num >= 0) {
        return (num + half) / den;
    }
    return (num - half) / den;
}

static int32_t saturate_i64_to_i32(int64_t v)
{
    if (v > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (v < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)v;
}

static int32_t clamp_freq_ppb(int64_t ppb)
{
    if (ppb > (int64_t)PTP_FREQ_ADJ_LIMIT_PPB) {
        return (int32_t)PTP_FREQ_ADJ_LIMIT_PPB;
    }
    if (ppb < -(int64_t)PTP_FREQ_ADJ_LIMIT_PPB) {
        return -(int32_t)PTP_FREQ_ADJ_LIMIT_PPB;
    }
    return (int32_t)ppb;
}

static int64_t clamp_i64_abs_limit(int64_t v, int64_t limit)
{
    if (limit < 0) {
        limit = -limit;
    }
    if (v > limit) {
        return limit;
    }
    if (v < -limit) {
        return -limit;
    }
    return v;
}

/* Carry sub-ppb fine integral updates instead of rounding each sample to zero.
 * The remainder is signed so opposite errors cancel, with unchanged loop gains. */
static int64_t ptp_fine_bias_step_ppb(ptp_slave_ctx_t *ctx, int64_t phase_ppb)
{
    int64_t total;
    int64_t step;
    int64_t limited;

    if (PTP_FINE_BIAS_ACC_DIV <= 0ll) {
        ctx->fine_bias_remainder = 0ll;
        return 0ll;
    }
    total = ctx->fine_bias_remainder + phase_ppb;
    step = div_round_nearest_s64(total, PTP_FINE_BIAS_ACC_DIV);
    limited = clamp_i64_abs_limit(step, (int64_t)PTP_FINE_BIAS_UPDATE_MAX_PPB);
    ctx->fine_bias_remainder = (step == limited)
        ? total - step * PTP_FINE_BIAS_ACC_DIV : 0ll;
    return limited;
}

static void ptp_freeze_servo(ptp_slave_ctx_t *ctx, uint32_t cycles)
{
    if (!ctx || cycles == 0u) {
        return;
    }
    ctx->fine_bias_remainder = 0ll;
    if (ctx->freeze_servo_cycles < cycles) {
        ctx->freeze_servo_cycles = cycles;
    }
    if (ctx->servo_freeze_event_count < 0xFFFFFFFFu) {
        ctx->servo_freeze_event_count++;
    }
}

static int64_t iir_filter_s64(int64_t prev,
                              int64_t sample,
                              int64_t num,
                              int64_t den,
                              uint32_t have_prev)
{
    if (!have_prev || den == 0) {
        return sample;
    }
    return div_round_nearest_s64(prev * num + sample, den);
}

static ptp_servo_quality_t ptp_check_exchange_quality(const ptp_slave_ctx_t *ctx,
                                                      int64_t mean_path_delay_ns,
                                                      int64_t *filtered_delay_ns_out)
{
    int64_t filtered_delay_ns;

    if (filtered_delay_ns_out) {
        *filtered_delay_ns_out = mean_path_delay_ns;
    }

    if (!ctx) {
        return PTP_SERVO_QUAL_DELAY_RANGE;
    }

    if ((mean_path_delay_ns < PTP_DELAY_MIN_NS) ||
        (mean_path_delay_ns > PTP_DELAY_MAX_NS)) {
        return PTP_SERVO_QUAL_DELAY_RANGE;
    }

    filtered_delay_ns = iir_filter_s64(ctx->filtered_delay_ns,
                                       mean_path_delay_ns,
                                       PTP_DELAY_FILTER_NUM,
                                       PTP_DELAY_FILTER_DEN,
                                       ctx->have_filtered_delay);
    if (filtered_delay_ns_out) {
        *filtered_delay_ns_out = filtered_delay_ns;
    }

    if (ctx->have_filtered_delay &&
        (i64_abs(mean_path_delay_ns - ctx->filtered_delay_ns) > PTP_DELAY_OUTLIER_NS)) {
        return PTP_SERVO_QUAL_DELAY_OUTLIER;
    }

    if ((ctx->servo_cycle_count > 0u) &&
        (i64_abs(mean_path_delay_ns - ctx->last_delay_ns) > PTP_DELAY_REJECT_SLOPE_MAX_NS)) {
        return PTP_SERVO_QUAL_DELAY_SLOPE;
    }

    return PTP_SERVO_QUAL_OK;
}

static const char *ptp_servo_quality_name(ptp_servo_quality_t quality)
{
    switch (quality) {
    case PTP_SERVO_QUAL_OK:
        return "OK";
    case PTP_SERVO_QUAL_DELAY_RANGE:
        return "DELAY_RANGE";
    case PTP_SERVO_QUAL_DELAY_OUTLIER:
        return "DELAY_OUTLIER";
    case PTP_SERVO_QUAL_DELAY_SLOPE:
        return "DELAY_SLOPE";
    case PTP_SERVO_QUAL_PDV_SPIKE:
        return "PDV_SPIKE";
    case PTP_SERVO_QUAL_PATH_REBASE:
        return "PATH_REBASE";
    default:
        return "UNKNOWN";
    }
}

static const char *ptp_state_name(ptp_slave_state_t state)
{
    switch (state) {
    case PTP_WAIT_SYNC:
        return "WAIT_SYNC";
    case PTP_WAIT_FOLLOW_UP:
        return "WAIT_FOLLOW_UP";
    case PTP_WAIT_DELAY_RESP:
        return "WAIT_DELAY_RESP";
    default:
        return "UNKNOWN";
    }
}

static void ptp_select_offset_filter_coeffs(const ptp_slave_ctx_t *ctx,
                                            int64_t *num_out,
                                            int64_t *den_out)
{
    int64_t num;
    int64_t den;

    num = PTP_ACQ_OFFSET_FILTER_NUM;
    den = PTP_ACQ_OFFSET_FILTER_DEN;
    if (ctx) {
        if (ctx->fine_locked) {
            num = PTP_OFFSET_FILTER_NUM;
            den = PTP_OFFSET_FILTER_DEN;
        } else if (ctx->locked) {
            num = PTP_READY_OFFSET_FILTER_NUM;
            den = PTP_READY_OFFSET_FILTER_DEN;
        }
    }

    if (num_out) {
        *num_out = num;
    }
    if (den_out) {
        *den_out = den;
    }
}

static void ptp_select_freq_filter_coeffs(const ptp_slave_ctx_t *ctx,
                                          uint32_t fine_zone,
                                          uint32_t track_zone,
                                          int64_t *num_out,
                                          int64_t *den_out)
{
    int64_t num;
    int64_t den;

    num = PTP_FREQ_FILTER_NUM;
    den = PTP_FREQ_FILTER_DEN;
    if (ctx) {
        if (ctx->fine_locked) {
            num = PTP_FREQ_FILTER_FINE_NUM;
            den = PTP_FREQ_FILTER_FINE_DEN;
        } else if (fine_zone) {
            num = PTP_FREQ_FILTER_FINE_ACQ_NUM;
            den = PTP_FREQ_FILTER_FINE_ACQ_DEN;
        } else if (track_zone || ctx->locked) {
            num = PTP_FREQ_FILTER_TRACK_NUM;
            den = PTP_FREQ_FILTER_TRACK_DEN;
        }
    }

    if (den == 0ll) {
        den = 1ll;
    }
    if (num_out) {
        *num_out = num;
    }
    if (den_out) {
        *den_out = den;
    }
}

static int32_t ptp_compute_trim_step_ns(int64_t filtered_offset_ns)
{
    int64_t step_ns;

    if (PTP_TRIM_GAIN_DEN == 0ll) {
        return 0;
    }

    step_ns = -div_round_nearest_s64(filtered_offset_ns * PTP_TRIM_GAIN_NUM,
                                     PTP_TRIM_GAIN_DEN);
    step_ns = clamp_i64_abs_limit(step_ns, PTP_TRIM_MAX_NS);
    return saturate_i64_to_i32(step_ns);
}

static void ptp_update_lock_flags(ptp_slave_ctx_t *ctx,
                                  int64_t filtered_offset_ns,
                                  uint32_t delay_lock_ok,
                                  uint32_t suppress_lock_update)
{
    uint32_t ready_ok;
    uint32_t fine_ok;

    if (!ctx) {
        return;
    }

    if (suppress_lock_update) {
        ctx->lock_streak = 0u;
        ctx->locked = 0u;
        ctx->fine_lock_streak = 0u;
        ctx->fine_locked = 0u;
        ctx->fine_bias_remainder = 0ll;
        return;
    }

    ready_ok = ((i64_abs(filtered_offset_ns) <= PTP_LOCK_THRESHOLD_NS) &&
                delay_lock_ok &&
                ((!ctx->have_filtered_drift) ||
                 (i64_abs((int64_t)ctx->filtered_drift_ppb) <= (int64_t)PTP_LOCK_DRIFT_THRESHOLD_PPB))) ? 1u : 0u;

    if (ready_ok) {
        if (ctx->lock_streak < 0xFFFFFFFFu) {
            ctx->lock_streak++;
        }
    } else {
        ctx->lock_streak = 0u;
    }
    ctx->locked = (ctx->lock_streak >= PTP_LOCK_STREAK_MIN) ? 1u : 0u;
    if (ctx->locked) {
        ctx->startup_done = 1u;
    }

    fine_ok = (ctx->locked &&
               (i64_abs(filtered_offset_ns) <= PTP_FINE_LOCK_THRESHOLD_NS) &&
               ((!PTP_FAST50_ENABLE) || (ctx->last_sample_valid && i64_abs(ctx->last_offset_ns)<=PTP_FINE_LOCK_THRESHOLD_NS)) &&
               delay_lock_ok &&
               ((!ctx->have_filtered_drift) ||
                (i64_abs((int64_t)ctx->filtered_drift_ppb) <= (int64_t)PTP_FINE_LOCK_DRIFT_THRESHOLD_PPB))) ? 1u : 0u;

    if (fine_ok) {
        if (ctx->fine_lock_streak < 0xFFFFFFFFu) {
            ctx->fine_lock_streak++;
        }
    } else {
        ctx->fine_lock_streak = 0u;
    }
    ctx->fine_locked = (ctx->fine_lock_streak >= PTP_FINE_LOCK_STREAK_MIN) ? 1u : 0u;
    if (!ctx->fine_locked) {
        ctx->fine_bias_remainder = 0ll;
    }
}

static void ptp_expire_sync_lock(ptp_slave_ctx_t *ctx,uint32_t last_good,
                                  uint32_t now,uint32_t timeout_ticks)
{
    if(ctx && timeout_ticks && (uint32_t)(now-last_good)>=timeout_ticks)
        ptp_update_lock_flags(ctx,0ll,0u,1u);
}

static void ptp_ctx_reset_cycle(ptp_slave_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->state                    = PTP_WAIT_SYNC;
    ctx->sync_seq_id              = 0u;
    ctx->delay_req_seq_id         = 0u;
    ctx->t1_master_ns             = 0ull;
    ctx->t2_slave_rx_ns           = 0ull;
    ctx->t3_slave_tx_ns           = 0ull;
    ctx->t4_master_rx_ns          = 0ull;
    ctx->follow_up_timeout_polls  = 0u;
    ctx->delay_resp_timeout_polls = 0u;
}

static void ptp_ctx_init(ptp_slave_ctx_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->state                          = PTP_WAIT_SYNC;
    ptp_delay_guard_init(&ctx->delay_guard);
    ctx->last_sample_valid=0u;
    ctx->last_sample_quality=PTP_SERVO_QUAL_OK;
    ctx->last_delay_ref_ns=0ll;
    ctx->last_delay_margin_ns=0ll;
    ctx->sync_seq_id                    = 0u;
    ctx->delay_req_seq_id               = 0u;
    ctx->next_delay_req_seq_id          = (uint16_t)PTP_DELAY_REQ_SEQ_START;
    ctx->delay_req_trigger_fail_count   = 0u;
    ctx->invalid_follow_up_count        = 0u;
    ctx->invalid_delay_resp_count       = 0u;
    ctx->delay_resp_port_mismatch_count = 0u;
    ctx->cycle_drop_count               = 0u;
    ctx->txstamp_timeout_streak         = 0u;
    ctx->t1_master_ns                   = 0ull;
    ctx->t2_slave_rx_ns                 = 0ull;
    ctx->t3_slave_tx_ns                 = 0ull;
    ctx->t4_master_rx_ns                = 0ull;
    ctx->prev_t1_master_ns              = 0ull;
    ctx->have_prev_t1                   = 0u;
    ctx->freq_adj_ppb                   = 0;
    ctx->freq_bias_ppb                  = 0;
    ctx->fine_bias_remainder            = 0ll;
    ctx->filtered_drift_ppb             = 0;
    ctx->have_filtered_drift            = 0u;
    ctx->filtered_offset_ns             = 0ll;
    ctx->have_filtered_offset           = 0u;
    ctx->filtered_delay_ns              = 0ll;
    ctx->have_filtered_delay            = 0u;
    ctx->last_offset_ns                 = 0ll;
    ctx->last_delay_ns                  = 0ll;
    ctx->last_d1_ns                     = 0ll;
    ctx->last_d2_ns                     = 0ll;
    ctx->last_drift_ppb                 = 0ll;
    ctx->last_phase_ppb                 = 0ll;
    ctx->last_direct_phase_ppb          = 0ll;
    ctx->last_bias_update_ppb           = 0ll;
    ctx->last_target_freq_ppb           = 0ll;
    ctx->last_delay_slope_ns            = 0ll;
    ctx->servo_cycle_count              = 0u;
    ctx->rejected_exchange_count        = 0u;
    ctx->delay_outlier_count            = 0u;
    ctx->freeze_servo_cycles            = 0u;
    ctx->servo_freeze_event_count       = 0u;
    ctx->post_adjust_holdoff_cycles     = 0u;
    ctx->lock_streak                    = 0u;
    ctx->locked                         = 0u;
    ctx->fine_lock_streak               = 0u;
    ctx->fine_locked                    = 0u;
    ctx->startup_done                   = 0u;
    ptp_fast50_init(&ctx->fast50);
    ctx->stream_started                 = 0u;
    ctx->ptp_rx_count                   = 0u;
    ctx->mailbox_overflow_count         = 0u;
    ctx->follow_up_timeout_polls        = 0u;
    ctx->delay_resp_timeout_polls       = 0u;
}

static net_acq_eth_cfg_t build_eth_cfg(void)
{
    net_acq_eth_cfg_t cfg;

    cfg.backend_enable             = UDP_STREAM_ENABLE_BACKEND;
    cfg.tx_mirror_enable           = UDP_STREAM_TX_MIRROR_ENABLE;
    cfg.append_fcs                 = UDP_STREAM_APPEND_FCS_REQUEST;
    cfg.udp_checksum_enable        = UDP_STREAM_UDP_CHECKSUM_REQUEST;
    cfg.stall_drop_enable          = UDP_STREAM_STALL_DROP_REQUEST;
    cfg.local_mac                  = NET_ACQ_MAKE_MAC48(NET_ACQ_DUT_LOCAL_MAC_B5,
                                                        NET_ACQ_DUT_LOCAL_MAC_B4,
                                                        NET_ACQ_DUT_LOCAL_MAC_B3,
                                                        NET_ACQ_DUT_LOCAL_MAC_B2,
                                                        NET_ACQ_DUT_LOCAL_MAC_B1,
                                                        NET_ACQ_DUT_LOCAL_MAC_B0);
    cfg.local_ip                   = NET_ACQ_MAKE_IP4(NET_ACQ_DUT_LOCAL_IP_A,
                                                      NET_ACQ_DUT_LOCAL_IP_B,
                                                      NET_ACQ_DUT_LOCAL_IP_C,
                                                      NET_ACQ_DUT_LOCAL_IP_D);
    cfg.dst_mac                    = NET_ACQ_MAKE_MAC48(NET_ACQ_UDP_DATA_DST_MAC_B5,
                                                        NET_ACQ_UDP_DATA_DST_MAC_B4,
                                                        NET_ACQ_UDP_DATA_DST_MAC_B3,
                                                        NET_ACQ_UDP_DATA_DST_MAC_B2,
                                                        NET_ACQ_UDP_DATA_DST_MAC_B1,
                                                        NET_ACQ_UDP_DATA_DST_MAC_B0);
    cfg.dst_ip                     = NET_ACQ_MAKE_IP4(NET_ACQ_UDP_DATA_DST_IP_A,
                                                      NET_ACQ_UDP_DATA_DST_IP_B,
                                                      NET_ACQ_UDP_DATA_DST_IP_C,
                                                      NET_ACQ_UDP_DATA_DST_IP_D);
    cfg.ptp_addr_split_enable      = NET_ACQ_PTP_ADDR_SPLIT_ENABLE;
    cfg.ptp_peer_mac               = NET_ACQ_MAKE_MAC48(NET_ACQ_PTP_PEER_MAC_B5,
                                                        NET_ACQ_PTP_PEER_MAC_B4,
                                                        NET_ACQ_PTP_PEER_MAC_B3,
                                                        NET_ACQ_PTP_PEER_MAC_B2,
                                                        NET_ACQ_PTP_PEER_MAC_B1,
                                                        NET_ACQ_PTP_PEER_MAC_B0);
    cfg.ptp_peer_ip                = NET_ACQ_MAKE_IP4(NET_ACQ_PTP_PEER_IP_A,
                                                      NET_ACQ_PTP_PEER_IP_B,
                                                      NET_ACQ_PTP_PEER_IP_C,
                                                      NET_ACQ_PTP_PEER_IP_D);
    cfg.src_port                   = (uint16_t)UDP_STREAM_SRC_PORT;
    cfg.dst_port                   = (uint16_t)UDP_STREAM_DST_PORT;
    cfg.ttl                        = (uint8_t)UDP_STREAM_TTL;
    cfg.dscp_ecn                   = (uint8_t)UDP_STREAM_DSCP_ECN;
    cfg.ip_ident_seed              = (uint16_t)UDP_STREAM_IP_IDENT_SEED;

    /* 关键点：active config 里必须携带 ptp_rx_filter 的固定匹配条件。 */
    cfg.ptp_enable                 = 1u;
    cfg.ptp_one_step               = 0u;
    cfg.ptp_two_step               = 1u;
    cfg.ptp_event_priority_enable  = 1u;
    cfg.ptp_domain                 = (uint8_t)PTP_LOCAL_DOMAIN;
    cfg.ptp_clock_id               = PTP_LOCAL_CLOCK_ID;
    cfg.ptp_port_num               = (uint16_t)PTP_LOCAL_PORT_NUM;

    return cfg;
}

static net_acq_ptp_test_cfg_t build_ptp_cfg(void)
{
    net_acq_ptp_test_cfg_t cfg;

    cfg.enable                     = 1u;
    cfg.one_step                   = 0u;
    cfg.two_step                   = 1u;
    cfg.event_priority_enable      = 1u;
    cfg.domain                     = (uint8_t)PTP_LOCAL_DOMAIN;
    cfg.clock_id                   = PTP_LOCAL_CLOCK_ID;
    cfg.port_num                   = (uint16_t)PTP_LOCAL_PORT_NUM;
    cfg.tod_enable                 = 1u;
    cfg.tod_hold                   = 0u;
    cfg.tod_set_ns                 = PTP_TOD_SEED_NS;
    cfg.tod_step_ns                = 0;
    cfg.tod_freq_adj_ppb           = 0;
    cfg.tx_need_timestamp          = 1u;
    cfg.tx_message_type            = NET_ACQ_PTP_MSG_DELAY_REQ;
    cfg.tx_transport_specific      = 0u;
    cfg.tx_sequence_id             = 0u;
    cfg.tx_flags                   = 0u;
    cfg.tx_log_message_interval    = 0;
    cfg.issue_tod_set_pulse        = 0u;
    cfg.issue_tod_step_pulse       = 0u;
    cfg.issue_tod_freq_pulse       = 0u;
    cfg.issue_tx_trigger_pulse     = 0u;
    cfg.issue_txstamp_pop_pulse    = 0u;
    cfg.require_ext_features       = 0u;
    cfg.require_cur_time_valid     = 0u;
    cfg.require_txstamp_valid      = 0u;
    cfg.cur_time_poll_rounds       = 0u;
    cfg.txstamp_poll_rounds        = 0u;

    return cfg;
}

static net_acq_txsched_cfg_t build_txsched_cfg(void)
{
    net_acq_txsched_cfg_t cfg;

    cfg.enable                 = TSN_LITE_V1_ENABLE;
    cfg.block_when_tod_invalid = TSN_LITE_V1_BLOCK_WHEN_TOD_INVALID;
    cfg.strict_window          = TSN_LITE_V1_STRICT_WINDOW;
    cfg.base_time_ns           = 0ull; /* 运行期基于当前 ToD 重对齐 */
    cfg.period_ns              = (uint32_t)TSN_LITE_V1_PERIOD_NS;
    cfg.slot_offset_ns         = (uint32_t)TSN_LITE_V1_SLOT_OFFSET_NS;
    cfg.slot_width_ns          = (uint32_t)TSN_LITE_V1_SLOT_WIDTH_NS;
    cfg.advance_ns             = (uint32_t)TSN_LITE_V1_ADVANCE_NS;
    cfg.late_tol_ns            = (uint32_t)TSN_LITE_V1_LATE_TOL_NS;
    return cfg;
}

static uint64_t txsched_align_base_time(uint64_t now_ns,
                                        uint32_t period_ns,
                                        uint32_t slot_offset_ns,
                                        uint32_t lead_periods)
{
    uint64_t future_ns;
    uint64_t period_base_ns;

    if (period_ns == 0u) {
        return now_ns;
    }

    future_ns = now_ns + (uint64_t)lead_periods * (uint64_t)period_ns;
    period_base_ns = (future_ns / (uint64_t)period_ns) * (uint64_t)period_ns;
    if ((period_base_ns + (uint64_t)slot_offset_ns) <= future_ns) {
        period_base_ns += (uint64_t)period_ns;
    }
    return period_base_ns;
}

static int net_acq_tsnlite_v1_arm(const net_acq_eth_cfg_t *eth_cfg,
                                  net_acq_txsched_cfg_t *txs_cfg,
                                  uint32_t poll_delay_cycles,
                                  uint32_t eth_commit_poll_rounds)
{
    net_acq_txsched_snapshot_t txs;
    net_acq_dbgext_snapshot_t dbgext;
    uint64_t cur_time_ns;
    int rc;

    if (!eth_cfg || !txs_cfg) {
        return 0;
    }
    if (!txs_cfg->enable) {
        return 0;
    }

    rc = net_acq_txsched_validate_config(txs_cfg);
    if (rc != 0) {
        printf("[txs   ] validate failed, rc=%d\n", rc);
        return rc;
    }

    cur_time_ns = net_acq_ptp_current_time();
    txs_cfg->base_time_ns = txsched_align_base_time(cur_time_ns,
                                                    txs_cfg->period_ns,
                                                    txs_cfg->slot_offset_ns,
                                                    (uint32_t)TSN_LITE_V1_START_LEAD_PERIODS);

    printf("[txs   ] arm scheduler: now=0x%08x_%08x, base=0x%08x_%08x, period=%u, slot_offset=%u, slot_width=%u\n",
           (uint32_t)(cur_time_ns >> 32),
           (uint32_t)(cur_time_ns & 0xFFFFFFFFu),
           (uint32_t)(txs_cfg->base_time_ns >> 32),
           (uint32_t)(txs_cfg->base_time_ns & 0xFFFFFFFFu),
           (unsigned int)txs_cfg->period_ns,
           (unsigned int)txs_cfg->slot_offset_ns,
           (unsigned int)txs_cfg->slot_width_ns);

    rc = net_acq_eth_apply_config_ex(eth_cfg,
                                     txs_cfg,
                                     poll_delay_cycles,
                                     eth_commit_poll_rounds);
    if (rc != 0) {
        printf("[txs   ] commit failed, rc=%d\n", rc);
        return rc;
    }

    if (TSN_LITE_V1_DUMP_ON_ARM) {
        net_acq_read_txsched_snapshot(&txs);
        net_acq_read_dbgext_snapshot(&dbgext);
        net_acq_dump_txsched_snapshot(&txs);
        net_acq_dump_dbgext_snapshot(&dbgext);
    }

    return 0;
}

static void print_banner(const net_acq_eth_cfg_t *eth_cfg,
                         const net_acq_ptp_test_cfg_t *ptp_cfg,
                         const net_acq_txsched_cfg_t *txs_cfg)
{
    printf("\n================ net_acq_ptp_closed_loop ================\n");
    printf("UART_BASE               = 0x%08lx\n", UART_BASE);
    printf("NET_ACQ_BASE            = 0x%08x\n", NET_ACQ_BASE);
    printf("Mount path              = axiOut_7 -> 0x1f500000 -> 0xbf500000\n");
#ifdef SIMU
    printf("Build mode              = SIMU\n");
#else
    printf("Build mode              = FPGA\n");
#endif
    printf("Control scheme          = PTP dual-stage ready/fine servo (ready fast, fine hold)\n");
    printf("Source                  = %s\n", net_acq_source_name(UDP_STREAM_USE_EXTERNAL_SOURCE));
    printf("Mode                    = %s\n", net_acq_mode_name(UDP_STREAM_MODE));
    printf("Sample period           = %u\n", (unsigned int)UDP_STREAM_SAMPLE_PERIOD);
    printf("Samples per packet      = %u (effective=%u)\n",
           (unsigned int)UDP_STREAM_SAMPLES_PER_PACKET,
           (unsigned int)net_acq_effective_samples_per_packet(UDP_STREAM_SAMPLES_PER_PACKET));
    printf("Channel ID              = %u\n", (unsigned int)UDP_STREAM_CHANNEL_ID);
    printf("Expected UDP length     = %u bytes\n",
           (unsigned int)net_acq_expected_udp_length(UDP_STREAM_SAMPLES_PER_PACKET));
    printf("Expected IPv4 length    = %u bytes\n",
           (unsigned int)net_acq_expected_ip_total_length(UDP_STREAM_SAMPLES_PER_PACKET));
    printf("Local MAC               = ");
    print_mac48(eth_cfg->local_mac);
    printf("\n");
    printf("Local IP                = ");
    print_ip4(eth_cfg->local_ip);
    printf("\n");
    printf("UDP dst MAC             = ");
    print_mac48(eth_cfg->dst_mac);
    printf("\n");
    printf("UDP dst IP              = ");
    print_ip4(eth_cfg->dst_ip);
    printf("\n");
    printf("PTP peer split enable   = %u\n", (unsigned int)(eth_cfg->ptp_addr_split_enable ? 1u : 0u));
    printf("PTP peer MAC            = ");
    print_mac48(eth_cfg->ptp_peer_mac);
    printf("\n");
    printf("PTP peer IP             = ");
    print_ip4(eth_cfg->ptp_peer_ip);
    printf("\n");
    printf("UDP data ports          = src=%u dst=%u\n",
           (unsigned int)eth_cfg->src_port,
           (unsigned int)eth_cfg->dst_port);
    printf("PTP domain              = %u\n", (unsigned int)ptp_cfg->domain);
    printf("PTP local clock_id      = 0x%08x_%08x\n",
           (uint32_t)(ptp_cfg->clock_id >> 32),
           (uint32_t)(ptp_cfg->clock_id & 0xFFFFFFFFu));
    printf("PTP local port_num      = %u\n", (unsigned int)ptp_cfg->port_num);
    printf("PTP initial ToD         = 0x%08x_%08x ns\n",
           (uint32_t)(ptp_cfg->tod_set_ns >> 32),
           (uint32_t)(ptp_cfg->tod_set_ns & 0xFFFFFFFFu));
    printf("backend_enable          = %u\n", (unsigned int)eth_cfg->backend_enable);
    printf("tx_mirror_enable        = %u\n", (unsigned int)eth_cfg->tx_mirror_enable);
    if (txs_cfg) {
        printf("TSN-lite v1 enable      = %u\n", (unsigned int)txs_cfg->enable);
        printf("TSN-lite period_ns      = %u\n", (unsigned int)txs_cfg->period_ns);
        printf("TSN-lite slot_offset_ns = %u\n", (unsigned int)txs_cfg->slot_offset_ns);
        printf("TSN-lite slot_width_ns  = %u\n", (unsigned int)txs_cfg->slot_width_ns);
        printf("TSN-lite advance_ns     = %u\n", (unsigned int)txs_cfg->advance_ns);
        printf("TSN-lite late_tol_ns    = %u\n", (unsigned int)txs_cfg->late_tol_ns);
    }
    printf("PTP RX accepted msgType = Sync / Follow_Up / Delay_Resp\n");
    printf("PTP RX accepted UDP dst = 319 / 320 (peer routed by split cfg when enabled)\n");
    printf("Slave TX msgType        = Delay_Req (event, timestamped)\n");
    printf("PTP UART exchange trace = [ptp-ts ] cycle/.../freeze/ready/fine\n");
    printf("Control build ID        = 0x%08x\n", (unsigned int)PTP_CONTROL_BUILD_ID);
    printf("Servo startup-step threshold = %lld ns, trim=%lld ns, trim_gain=%lld/%lld, trim_max=%lld ns\n",
           (long long)PTP_STEP_THRESHOLD_NS,
           (long long)PTP_TRIM_THRESHOLD_NS,
           (long long)PTP_TRIM_GAIN_NUM,
           (long long)PTP_TRIM_GAIN_DEN,
           (long long)PTP_TRIM_MAX_NS);
    printf("Servo reacquire-step threshold = %lld ns\n", (long long)PTP_REACQUIRE_STEP_THRESHOLD_NS);
    printf("Servo ready-lock threshold = %lld ns, streak=%u, drift<=%d ppb\n",
           (long long)PTP_LOCK_THRESHOLD_NS,
           (unsigned int)PTP_LOCK_STREAK_MIN,
           (int)PTP_LOCK_DRIFT_THRESHOLD_PPB);
    printf("Servo fine-lock threshold  = %lld ns, streak=%u, drift<=%d ppb\n",
           (long long)PTP_FINE_LOCK_THRESHOLD_NS,
           (unsigned int)PTP_FINE_LOCK_STREAK_MIN,
           (int)PTP_FINE_LOCK_DRIFT_THRESHOLD_PPB);
    printf("Large-offset set threshold = %lld ns\n", (long long)PTP_SET_TIME_THRESHOLD_NS);
    printf("PTP-only mode           = %u (1 means acquisition/UDP stream will stay disabled)\n",
           (unsigned int)PTP_ONLY_MODE);
    printf("PTP data stream gate    = after %u cycles, ready_lock_required=%u, |offset| <= %lld ns, |drift| <= %d ppb\n",
           (unsigned int)PTP_STREAM_START_AFTER_SERVO_CYCLES,
           (unsigned int)PTP_STREAM_REQUIRE_LOCK,
           (long long)PTP_STREAM_START_MAX_OFFSET_NS,
           (int)PTP_STREAM_START_MAX_DRIFT_PPB);
    printf("PTP offset calibration  = %lld ns (raw offset is still printed)\n",
           (long long)PTP_OFFSET_CAL_NS);
    printf("PTP_FREQ_ADJ semantic   = signed ppb (shared across sys/eth domains)\n");
    printf("Servo freq clamp        = %d ppb, gain_div_acq=%lld, gain_div_lock=%lld, offset_lpf_fine=%lld/%lld, offset_lpf_ready=%lld/%lld, offset_lpf_acq=%lld/%lld, drift_lpf=%lld/%lld, freq_lpf_acq=%lld/%lld, track=%lld/%lld, fine_acq=%lld/%lld, fine_hold=%lld/%lld\n",
           (int)PTP_FREQ_ADJ_LIMIT_PPB,
           (long long)PTP_FREQ_GAIN_DIV,
           (long long)PTP_FREQ_GAIN_DIV_LOCK,
           (long long)PTP_OFFSET_FILTER_NUM,
           (long long)PTP_OFFSET_FILTER_DEN,
           (long long)PTP_READY_OFFSET_FILTER_NUM,
           (long long)PTP_READY_OFFSET_FILTER_DEN,
           (long long)PTP_ACQ_OFFSET_FILTER_NUM,
           (long long)PTP_ACQ_OFFSET_FILTER_DEN,
           (long long)PTP_DRIFT_FILTER_NUM,
           (long long)PTP_DRIFT_FILTER_DEN,
           (long long)PTP_FREQ_FILTER_NUM,
           (long long)PTP_FREQ_FILTER_DEN,
           (long long)PTP_FREQ_FILTER_TRACK_NUM,
           (long long)PTP_FREQ_FILTER_TRACK_DEN,
           (long long)PTP_FREQ_FILTER_FINE_ACQ_NUM,
           (long long)PTP_FREQ_FILTER_FINE_ACQ_DEN,
           (long long)PTP_FREQ_FILTER_FINE_NUM,
           (long long)PTP_FREQ_FILTER_FINE_DEN);
    printf("Servo bias control      = bias_acc_div=%lld, phase_direct_div=%lld, bias_step_max=%d ppb, direct_phase_max=%d ppb, hold_offset=%lld ns, freeze_after_drop=%u\n",
           (long long)PTP_BIAS_ACC_DIV,
           (long long)PTP_PHASE_DIRECT_DIV,
           (int)PTP_BIAS_UPDATE_MAX_PPB,
           (int)PTP_DIRECT_PHASE_MAX_PPB,
           (long long)PTP_HOLD_OFFSET_THRESHOLD_NS,
           (unsigned int)PTP_FREEZE_AFTER_DROP_CYCLES);
    printf("Servo fine/track        = fine_zone=%lld ns, fine_drift=%d ppb, fine_acq(g=%lld,bias_div=%lld,bias_max=%d,direct_div=%lld,direct_max=%d), fine_hold(g=%lld,bias_div=%lld,bias_max=%d,direct_div=%lld,direct_max=%d), track(g=%lld,bias_div=%lld,bias_max=%d,direct_div=%lld,direct_max=%d)\n",
           (long long)PTP_FINE_OFFSET_THRESHOLD_NS,
           (int)PTP_FINE_DRIFT_THRESHOLD_PPB,
           (long long)PTP_FREQ_GAIN_DIV_FINE_ACQ,
           (long long)PTP_FINE_ACQ_BIAS_ACC_DIV,
           (int)PTP_FINE_ACQ_BIAS_UPDATE_MAX_PPB,
           (long long)PTP_FINE_ACQ_PHASE_DIRECT_DIV,
           (int)PTP_FINE_ACQ_DIRECT_PHASE_MAX_PPB,
           (long long)PTP_FREQ_GAIN_DIV_FINE,
           (long long)PTP_FINE_BIAS_ACC_DIV,
           (int)PTP_FINE_BIAS_UPDATE_MAX_PPB,
           (long long)PTP_FINE_PHASE_DIRECT_DIV,
           (int)PTP_FINE_DIRECT_PHASE_MAX_PPB,
           (long long)PTP_FREQ_GAIN_DIV_LOCK,
           (long long)PTP_TRACK_BIAS_ACC_DIV,
           (int)PTP_TRACK_BIAS_UPDATE_MAX_PPB,
           (long long)PTP_TRACK_PHASE_DIRECT_DIV,
           (int)PTP_TRACK_DIRECT_PHASE_MAX_PPB);
    printf("Delay guard: window=%u warmup=%u margin>=%lld ns MADx%lld rebase=%u stable samples\n",
           PTP_DELAY_GUARD_WINDOW,PTP_DELAY_GUARD_WARMUP,
           (long long)PTP_DELAY_GUARD_MARGIN_NS,(long long)PTP_DELAY_GUARD_MAD_MULT,
           PTP_DELAY_GUARD_REBASE);
    printf("Delay sanity window     = [%lld, %lld] ns, outlier=%lld ns, reject_slope=%lld ns, delay_lpf=%lld/%lld, lock_delay=[%lld,%lld], slope<=%lld ns\n",
           (long long)PTP_DELAY_MIN_NS,
           (long long)PTP_DELAY_MAX_NS,
           (long long)PTP_DELAY_OUTLIER_NS,
           (long long)PTP_DELAY_REJECT_SLOPE_MAX_NS,
           (long long)PTP_DELAY_FILTER_NUM,
           (long long)PTP_DELAY_FILTER_DEN,
           (long long)PTP_DELAY_LOCK_MIN_NS,
           (long long)PTP_DELAY_LOCK_MAX_NS,
           (long long)PTP_DELAY_LOCK_SLOPE_MAX_NS);
}

static int wait_ptp_cur_time_valid(uint32_t max_polls)
{
    uint32_t i;

    for (i = 0u; i < max_polls; i++) {
        uint32_t st;

        st = net_acq_read(NET_ACQ_PTP_STATUS);
        if ((st & PTP_STATUS_CUR_TIME_VALID) != 0u) {
            return 0;
        }
        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    }

    printf("[bringup] PTP_STATUS.cur_time_valid stayed 0\n");
    return -101;
}

static void drain_startup_ptp_rx_mailbox(void)
{
    uint32_t i;

    for (i = 0u; i < PTP_STARTUP_MBOX_DRAIN_LIMIT; i++) {
        net_acq_ptp_rx_snapshot_t mbox;

        net_acq_ptp_rx_read_snapshot(&mbox);
        if (!mbox.valid) {
            if (mbox.overflow_sticky) {
                printf("[bringup] RX mailbox overflow_sticky was already set before run loop, pop once to release it\n");
                net_acq_ptp_rx_mbox_pop();
                net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
                continue;
            }
            return;
        }

        printf("[bringup] stale PTP RX mailbox entry found before run loop, pop it first\n");
        net_acq_ptp_rx_dump_snapshot(&mbox);
        net_acq_ptp_rx_mbox_pop();
        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    }

    printf("[bringup] startup PTP RX mailbox drain limit reached (%u)\n",
           (unsigned int)PTP_STARTUP_MBOX_DRAIN_LIMIT);
}

static int wait_ptp_tx_idle(uint32_t max_polls)
{
    uint32_t i;
    uint32_t st;

    st = net_acq_read(NET_ACQ_PTP_STATUS);
    if ((st & (PTP_STATUS_TX_BUSY | PTP_STATUS_TX_CMD_PENDING)) == 0u) {
        return 0;
    }

    for (i = 0u; i < max_polls; i++) {
        st = net_acq_read(NET_ACQ_PTP_STATUS);
        if ((st & (PTP_STATUS_TX_BUSY | PTP_STATUS_TX_CMD_PENDING)) == 0u) {
            return 0;
        }
        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    }

    printf("[ptp] timeout waiting TX path idle before Delay_Req trigger\n");
    print_ptp_status_brief("ptp", st);
    return -141;
}

static int clear_stale_txstamp(net_acq_ptp_test_cfg_t *ptp_cfg)
{
    uint64_t stale_txstamp;
    uint32_t i;

    (void)ptp_cfg;

    if (net_acq_ptp_read_txstamp(&stale_txstamp) <= 0) {
        return 0;
    }

    for (i = 0u; i < 8u; i++) {
        uint32_t st;

        st = net_acq_read(NET_ACQ_PTP_STATUS);
        if ((st & PTP_STATUS_TXSTAMP_VALID) == 0u) {
            return 0;
        }
        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    }

    printf("[ptp] stale TXSTAMP remained valid after consume read\n");
    print_ptp_status_brief("ptp", net_acq_read(NET_ACQ_PTP_STATUS));
    return -142;
}

static int wait_txstamp_valid(uint64_t *txstamp_ns_out)
{
    uint32_t i;
    uint32_t first_status;
    uint32_t last_status;
    uint32_t saw_busy;
    uint32_t saw_cmd_pending;

    first_status = net_acq_read(NET_ACQ_PTP_STATUS);
    last_status = first_status;
    saw_busy = (first_status & PTP_STATUS_TX_BUSY) ? 1u : 0u;
    saw_cmd_pending = (first_status & PTP_STATUS_TX_CMD_PENDING) ? 1u : 0u;

    for (i = 0u; i < PTP_TXSTAMP_WAIT_POLLS; i++) {
        uint32_t st;

        st = net_acq_read(NET_ACQ_PTP_STATUS);
        last_status = st;
        if ((st & PTP_STATUS_TX_BUSY) != 0u) {
            saw_busy = 1u;
        }
        if ((st & PTP_STATUS_TX_CMD_PENDING) != 0u) {
            saw_cmd_pending = 1u;
        }
        if (net_acq_ptp_read_txstamp(txstamp_ns_out) > 0) {
            return 0;
        }
        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    }

    printf("[ptp] timeout waiting for TXSTAMP_VALID after Delay_Req trigger\n");
    printf("[ptp] tx wait stats: first=0x%08x last=0x%08x saw_busy=%u saw_cmd_pending=%u ctrl=0x%08x\n",
           (unsigned int)first_status,
           (unsigned int)last_status,
           (unsigned int)saw_busy,
           (unsigned int)saw_cmd_pending,
           (unsigned int)net_acq_read(NET_ACQ_PTP_CTRL));
    return -140;
}

static int ptp_send_delay_req(net_acq_ptp_test_cfg_t *ptp_cfg,
                              uint16_t sequence_id,
                              uint64_t *t3_ns_out)
{
    net_acq_ptp_test_cfg_t tx_cfg;
    int rc;

    if (!ptp_cfg || !t3_ns_out) {
        return -1;
    }

    tx_cfg = *ptp_cfg;
    tx_cfg.tx_message_type         = NET_ACQ_PTP_MSG_DELAY_REQ;
    tx_cfg.tx_transport_specific   = 0u;
    tx_cfg.tx_sequence_id          = sequence_id;
    tx_cfg.tx_need_timestamp       = 1u;
    tx_cfg.tx_flags                = 0u;
    tx_cfg.tx_log_message_interval = 0;

    rc = wait_ptp_tx_idle(PTP_TX_IDLE_WAIT_POLLS);
    if (rc != 0) {
        return rc;
    }

    rc = clear_stale_txstamp(&tx_cfg);
    if (rc != 0) {
        return rc;
    }

    net_acq_ptp_write_shadow(&tx_cfg);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    net_acq_ptp_write_ctrl(&tx_cfg, 0u, 0u, 0u, 0u, 1u);

    return wait_txstamp_valid(t3_ns_out);
}

static void ptp_apply_step(net_acq_ptp_test_cfg_t *ptp_cfg, int32_t step_ns)
{
    if (!ptp_cfg) {
        return;
    }

    ptp_cfg->tod_step_ns = step_ns;
    net_acq_ptp_write_shadow(ptp_cfg);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    net_acq_ptp_write_ctrl(ptp_cfg, 0u, 0u, 1u, 0u, 0u);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
}

static void ptp_apply_set_time64(net_acq_ptp_test_cfg_t *ptp_cfg, uint64_t tod_ns)
{
    if (!ptp_cfg) {
        return;
    }

    ptp_cfg->tod_set_ns = tod_ns;
    net_acq_ptp_write_shadow(ptp_cfg);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    net_acq_ptp_write_ctrl(ptp_cfg, 0u, 1u, 0u, 0u, 0u);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
}

static void ptp_apply_freq_ppb(net_acq_ptp_test_cfg_t *ptp_cfg, int32_t freq_ppb)
{
    if (!ptp_cfg) {
        return;
    }

    ptp_cfg->tod_freq_adj_ppb = freq_ppb;
    net_acq_ptp_write_shadow(ptp_cfg);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
    net_acq_ptp_write_ctrl(ptp_cfg, 0u, 0u, 0u, 1u, 0u);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
}

static void ptp_servo_clear_frequency_state(ptp_slave_ctx_t *ctx,
                                            net_acq_ptp_test_cfg_t *ptp_cfg,
                                            uint32_t apply_hw_zero)
{
    uint32_t need_apply_zero;

    if (!ctx) {
        return;
    }

    need_apply_zero = (ctx->freq_adj_ppb != 0) ? 1u : 0u;
    ctx->freq_adj_ppb          = 0;
    ctx->freq_bias_ppb         = 0;
    ctx->fine_bias_remainder   = 0ll;
    ctx->filtered_drift_ppb    = 0;
    ctx->have_filtered_drift   = 0u;
    ctx->last_drift_ppb        = 0ll;
    ctx->last_phase_ppb        = 0ll;
    ctx->last_direct_phase_ppb = 0ll;
    ctx->last_bias_update_ppb  = 0ll;
    ctx->last_target_freq_ppb  = 0ll;

    if (ptp_cfg) {
        ptp_cfg->tod_freq_adj_ppb = 0;
    }

    if (apply_hw_zero && need_apply_zero && ptp_cfg) {
        ptp_apply_freq_ppb(ptp_cfg, 0);
    }
}

static void ptp_invalidate_exchange_history(ptp_slave_ctx_t *ctx)
{
    ptp_ctx_reset_cycle(ctx);
    ptp_fast50_init(&ctx->fast50);
    ctx->prev_t1_master_ns=0ull;ctx->have_prev_t1=0u;
    ctx->have_filtered_offset=0u;ctx->have_filtered_drift=0u;
    ctx->have_filtered_delay=0u;ctx->filtered_drift_ppb=0;
    ctx->fine_bias_remainder=0ll;
    ptp_delay_guard_init(&ctx->delay_guard);
    /* Preserve freq_bias/freq_adj: clock continues while PTP is absent. */
}

static int runtime_fault_check(const net_acq_snapshot_t *front,
                               const net_acq_debug_snapshot_t *dbg,
                               const net_acq_eth_snapshot_t *eth,
                               const ptp_link_recovery_ctx_t *link_recovery)
{
    if ((front->status & STATUS_ERROR) != 0u) {
        printf("[fault] STATUS.error asserted\n");
        return -1;
    }
    if (dbg->dbg_bad_packet_count != 0u) {
        printf("[fault] DBG_BAD_PKT_CNT=%u\n", (unsigned int)dbg->dbg_bad_packet_count);
        return -2;
    }
    if (dbg->dbg_last_error_code != NET_ACQ_ERR_NONE) {
        printf("[fault] DBG_LAST_ERROR=%u (%s) at word=%u\n",
               (unsigned int)dbg->dbg_last_error_code,
               net_acq_debug_error_name(dbg->dbg_last_error_code),
               (unsigned int)dbg->dbg_last_error_word_index);
        return -3;
    }
    if ((eth->eth_status & ETH_STATUS_CFG_ACTIVE) == 0u) {
        printf("[fault] ETH_STATUS.cfg_active dropped to 0\n");
        return -4;
    }
    if ((eth->eth_status & ETH_STATUS_BACKEND_SELECTED) == 0u) {
        printf("[fault] ETH_STATUS.backend_selected dropped to 0\n");
        return -5;
    }
    if (PTP_REQUIRE_LINK_UP &&
        (link_recovery == NULL ||
         link_recovery->state == PTP_LINK_RECOVERY_MONITOR) &&
        ((eth->eth_status & ETH_STATUS_LINK_UP) == 0u)) {
        printf("[fault] ETH_STATUS.link_up=0\n");
        return -6;
    }
    if ((eth->eth_status & ETH_STATUS_TX_ERROR_STICKY) != 0u) {
        printf("[fault] ETH_STATUS.tx_error_sticky=1\n");
        return -7;
    }
    if (eth->drop_count != 0u) {
        printf("[fault] ETH_DROP_COUNT=%u\n", (unsigned int)eth->drop_count);
        return -8;
    }
    if (eth->last_error_code != NET_ACQ_ETH_ERR_NONE) {
        printf("[fault] ETH_LAST_ERROR=%u (%s)\n",
               (unsigned int)eth->last_error_code,
               net_acq_eth_error_name(eth->last_error_code));
        return -9;
    }
    if ((eth->ptp_status & PTP_STATUS_PTP_ENABLE) == 0u) {
        printf("[fault] PTP_STATUS.ptp_enable dropped to 0\n");
        return -10;
    }
    if ((eth->ptp_status & PTP_STATUS_TOD_ENABLE) == 0u) {
        printf("[fault] PTP_STATUS.tod_enable dropped to 0\n");
        return -11;
    }
    if ((eth->ptp_status & PTP_STATUS_CUR_TIME_VALID) == 0u) {
        printf("[fault] PTP_STATUS.cur_time_valid dropped to 0\n");
        return -12;
    }
    return 0;
}

static void ptp_time_quality_runtime_init(ptp_time_quality_ctx_t *quality)
{
    ptp_time_quality_init(quality);
    net_acq_set_time_quality(PTP_TIME_INVALID);
}

static uint32_t ptp_apply_time_quality_event(
    ptp_slave_ctx_t *ctx,
    ptp_time_quality_ctx_t *quality)
{
    ptp_time_quality_event_t event;

    if ((ctx == NULL) || (quality == NULL) ||
        (ptp_time_quality_take_event(quality, &event) == 0u)) {
        return 0u;
    }

    if ((event.actions & PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS) != 0u) {
        ctx->locked = 0u;
        ctx->lock_streak = 0u;
        ctx->fine_locked = 0u;
        ctx->fine_lock_streak = 0u;
        ctx->fine_bias_remainder = 0ll;
    }

    if ((event.to == PTP_TIME_HOLDOVER) || (event.to == PTP_TIME_INVALID)) {
        ptp_invalidate_exchange_history(ctx);
    }

    if (((event.actions & PTP_TIME_QUALITY_ACTION_STOP_STREAM) != 0u) &&
        (ctx->stream_started != 0u)) {
        net_acq_disable(UDP_STREAM_USE_EXTERNAL_SOURCE);
        ctx->stream_started = 0u;
        printf("[quality] acquisition/UDP stream disabled because time is INVALID\n");
    }

    net_acq_set_time_quality((uint32_t)event.to);
    printf("[quality] %s -> %s, missed=%u, changes=%u, freq_hold_ppb=%d\n",
           ptp_time_quality_name(event.from),
           ptp_time_quality_name(event.to),
           (unsigned int)quality->missed_health_ticks,
           (unsigned int)quality->transition_count,
           (int)ctx->freq_adj_ppb);
    return 1u;
}

static uint32_t ptp_suspend_stream_for_link_loss(
    ptp_slave_ctx_t *ctx,
    const ptp_time_quality_ctx_t *quality,
    uint32_t eth_status)
{
    if ((ctx == NULL) ||
        (ptp_time_quality_should_suspend_stream_on_link_down(
             quality,
             (eth_status & ETH_STATUS_LINK_UP) ? 1u : 0u,
             ctx->stream_started) == 0u)) {
        return 0u;
    }

    net_acq_disable(UDP_STREAM_USE_EXTERNAL_SOURCE);
    net_acq_clear_status(0u, UDP_STREAM_USE_EXTERNAL_SOURCE);
    ctx->stream_started = 0u;
    printf("[recover] acquisition/UDP stream suspended because ETH link is down\n");
    return 1u;
}

static void ptp_apply_link_recovery_action(
    ptp_slave_ctx_t *ctx,
    ptp_link_recovery_action_t action)
{
    if ((ctx == NULL) || (action == PTP_LINK_RECOVERY_ACTION_NONE)) {
        return;
    }

    ctx->locked = 0u;
    ctx->fine_locked = 0u;
    ctx->lock_streak = 0u;
    ctx->fine_lock_streak = 0u;
    ctx->fine_bias_remainder = 0ll;
    ctx->stream_started = 0u;
    ptp_invalidate_exchange_history(ctx);

    if (action == PTP_LINK_RECOVERY_ACTION_RESET_PATH) {
        printf("[recover] stable link-up detected: reset acquisition/PTP RX path\n");
        net_acq_disable(UDP_STREAM_USE_EXTERNAL_SOURCE);
        net_acq_soft_reset(0u, UDP_STREAM_USE_EXTERNAL_SOURCE);
        net_acq_clear_status(0u, UDP_STREAM_USE_EXTERNAL_SOURCE);
        return;
    }
    if (action == PTP_LINK_RECOVERY_ACTION_RESTART_AN) {
        printf("[recover] no PTP RX progress after path reset: restart PHY autoneg\n");
        net_acq_phy_restart_autoneg();
        return;
    }
    if (action == PTP_LINK_RECOVERY_ACTION_RESET_PHY) {
        printf("[recover] no PTP RX progress after autoneg restart: soft-reset PHY once\n");
        net_acq_phy_soft_reset();
    }
}

static void ptp_observe_link_edge(ptp_slave_ctx_t *ctx,
                                   ptp_time_quality_ctx_t *quality,
                                   uint32_t *previous_up,uint32_t link_up)
{
    if(*previous_up && !link_up){
        ptp_update_lock_flags(ctx,0ll,0u,1u);
        ptp_invalidate_exchange_history(ctx);
        ptp_time_quality_on_link_down(quality);
        (void)ptp_apply_time_quality_event(ctx,quality);
        printf("[recover] link-down edge: discard incomplete PTP exchange/history\n");
    }
    *previous_up=link_up;
}

static int bringup_once(const net_acq_eth_cfg_t *eth_cfg,
                        const net_acq_txsched_cfg_t *txs_cfg,
                        net_acq_ptp_test_cfg_t *ptp_cfg,
                        ptp_slave_ctx_t *ctx)
{
    net_acq_snapshot_t front;
    net_acq_debug_snapshot_t dbg;
    net_acq_eth_snapshot_t eth;
    int rc;

    if (!eth_cfg || !ptp_cfg || !ctx) {
        return -1;
    }

    ptp_ctx_init(ctx);

    printf("\n[bringup] disable -> soft_reset -> clear_status\n");
    net_acq_disable(UDP_STREAM_USE_EXTERNAL_SOURCE);
    net_acq_soft_reset(0u, UDP_STREAM_USE_EXTERNAL_SOURCE);
    net_acq_clear_status(0u, UDP_STREAM_USE_EXTERNAL_SOURCE);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);

    printf("[bringup] configure acquisition front-end\n");
    net_acq_apply_config(UDP_STREAM_MODE,
                         UDP_STREAM_SAMPLE_PERIOD,
                         UDP_STREAM_SAMPLES_PER_PACKET,
                         UDP_STREAM_CHANNEL_ID,
                         UDP_STREAM_USE_EXTERNAL_SOURCE);

    printf("[bringup] commit ETH active config (data UDP + PTP static match)\n");
    rc = net_acq_eth_validate_config(eth_cfg);
    if (rc != 0) {
        printf("[bringup] net_acq_eth_validate_config failed, rc=%d\n", rc);
        return rc;
    }
    rc = net_acq_txsched_validate_config(txs_cfg);
    if (rc != 0) {
        printf("[bringup] net_acq_txsched_validate_config failed, rc=%d\n", rc);
        return rc;
    }
    rc = net_acq_eth_apply_config_ex(eth_cfg,
                                     (const net_acq_txsched_cfg_t *)0,
                                     PTP_POLL_DELAY_CYCLES,
                                     PTP_CUR_TIME_VALID_POLLS);
    if (rc != 0) {
        printf("[bringup] net_acq_eth_apply_config_ex failed, rc=%d\n", rc);
        return rc;
    }

    printf("[bringup] write PTP shadow (enable/domain/clock_id/port_num)\n");
    net_acq_ptp_write_shadow(ptp_cfg);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);

    printf("[bringup] pulse set_time so ToD starts from the shared seed\n");
    net_acq_ptp_write_ctrl(ptp_cfg, 0u, 1u, 0u, 0u, 0u);
    net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);

    rc = wait_ptp_cur_time_valid(PTP_CUR_TIME_VALID_POLLS);
    if (rc != 0) {
        return rc;
    }
    drain_startup_ptp_rx_mailbox();
    rc = clear_stale_txstamp(ptp_cfg);
    if (rc != 0) {
        printf("[bringup] warning: stale TXSTAMP cleanup returned rc=%d\n", rc);
    }

    if (PTP_REQUIRE_LINK_UP && !net_acq_eth_link_is_up()) {
        printf("[bringup] link_up required but not asserted\n");
        return -103;
    }

    printf("[bringup] keep acquisition disabled for now; wait until PTP reaches usable ready-lock, then keep refining toward fine-lock\n");

    net_acq_read_snapshot(&front);
    net_acq_read_debug_snapshot(&dbg);
    net_acq_read_eth_snapshot(&eth);
    net_acq_dump_snapshot(&front);
    net_acq_dump_debug_snapshot(&dbg);
    net_acq_dump_eth_snapshot(&eth);
    if (txs_cfg && txs_cfg->enable) {
        net_acq_txsched_snapshot_t txs;
        net_acq_read_txsched_snapshot(&txs);
        net_acq_dump_txsched_snapshot(&txs);
    }

    return 0;
}

static void print_rx_summary(const net_acq_ptp_rx_snapshot_t *mbox)
{
    if (!mbox) {
        return;
    }

    printf("[ptp-rx] type=%s(%u) seq=%u rx_ts=0x%08x_%08x msg_ts=",
           net_acq_ptp_msg_type_name(mbox->msg_type),
           (unsigned int)mbox->msg_type,
           (unsigned int)mbox->sequence_id,
           (unsigned int)(mbox->rx_timestamp_ns >> 32),
           (unsigned int)(mbox->rx_timestamp_ns & 0xFFFFFFFFu));
    net_acq_print_u80_hex(&mbox->msg_timestamp);
    printf(" req_port=");
    net_acq_print_u80_hex(&mbox->requesting_port_identity);
    printf("\n");
}

static void print_ptp_uart_exchange(const ptp_slave_ctx_t *ctx,
                                    int64_t d1_ns,
                                    int64_t d2_ns,
                                    int64_t mean_path_delay_ns,
                                    int64_t offset_raw_ns,
                                    int64_t offset_used_ns,
                                    int64_t filtered_offset_ns)
{
    if (!ctx) {
        return;
    }

    printf("[ptp-ts ] cycle=%u,sync_seq=%u,delay_req_seq=%u,t1=0x%08x_%08x,t2=0x%08x_%08x,t3=0x%08x_%08x,t4=0x%08x_%08x,d1_ns=%lld,d2_ns=%lld,delay_ns=%lld,offset_raw_ns=%lld,offset_cal_ns=%lld,offset_ns=%lld,filt_offset_ns=%lld,drift_ppb=%lld,phase_ppb=%lld,direct_ppb=%lld,bias_step_ppb=%lld,bias_ppb=%d,target_ppb=%lld,freq_ppb=%d,freeze=%u,ready=%u,ready_streak=%u,fine_lock=%u,fine_streak=%u,sample_valid=%u,quality=%s,delay_ref_ns=%lld,delay_margin_ns=%lld,spike_count=%u,rebase_count=%u\n",
           (unsigned int)ctx->servo_cycle_count,
           (unsigned int)ctx->sync_seq_id,
           (unsigned int)ctx->delay_req_seq_id,
           (unsigned int)(ctx->t1_master_ns >> 32),
           (unsigned int)(ctx->t1_master_ns & 0xFFFFFFFFu),
           (unsigned int)(ctx->t2_slave_rx_ns >> 32),
           (unsigned int)(ctx->t2_slave_rx_ns & 0xFFFFFFFFu),
           (unsigned int)(ctx->t3_slave_tx_ns >> 32),
           (unsigned int)(ctx->t3_slave_tx_ns & 0xFFFFFFFFu),
           (unsigned int)(ctx->t4_master_rx_ns >> 32),
           (unsigned int)(ctx->t4_master_rx_ns & 0xFFFFFFFFu),
           (long long)d1_ns,
           (long long)d2_ns,
           (long long)mean_path_delay_ns,
           (long long)offset_raw_ns,
           (long long)PTP_OFFSET_CAL_NS,
           (long long)offset_used_ns,
           (long long)filtered_offset_ns,
           (long long)ctx->filtered_drift_ppb,
           (long long)(ctx->last_sample_valid?ctx->last_phase_ppb:0ll),
           (long long)(ctx->last_sample_valid?ctx->last_direct_phase_ppb:0ll),
           (long long)(ctx->last_sample_valid?ctx->last_bias_update_ppb:0ll),
           (int)ctx->freq_bias_ppb,
           (long long)(ctx->last_sample_valid?ctx->last_target_freq_ppb:ctx->freq_adj_ppb),
           (int)ctx->freq_adj_ppb,
           (unsigned int)ctx->freeze_servo_cycles,
           (unsigned int)ctx->locked,
           (unsigned int)ctx->lock_streak,
           (unsigned int)ctx->fine_locked,
           (unsigned int)ctx->fine_lock_streak,
           (unsigned int)ctx->last_sample_valid,
           ptp_servo_quality_name(ctx->last_sample_quality),
           (long long)ctx->last_delay_ref_ns,(long long)ctx->last_delay_margin_ns,
           (unsigned int)ctx->delay_guard.spike_count,
           (unsigned int)ctx->delay_guard.rebase_count);
}

static void ptp_handle_servo_result(ptp_slave_ctx_t *ctx,
                                    net_acq_ptp_test_cfg_t *ptp_cfg)
{
    int64_t d1;
    int64_t d2;
    int64_t mean_path_delay_ns;
    int64_t filtered_delay_ns;
    int64_t offset_raw_ns;
    int64_t offset_used_ns;
    int64_t filtered_offset_ns;
    int64_t dt_master_ns;
    int64_t raw_correction_ppb;
    int64_t phase_term_ppb;
    int64_t direct_phase_ppb;
    int64_t bias_update_ppb;
    int64_t target_freq_ppb_dbg;
    int64_t drift_ppb_raw;
    int64_t filtered_drift_ppb;
    int64_t prev_filtered_offset_ns;
    int64_t prev_offset_used_ns;
    int64_t delay_slope_ns;
    int64_t offset_filter_num;
    int64_t offset_filter_den;
    int64_t freq_filter_num;
    int64_t freq_filter_den;
    uint32_t have_prev_filtered_offset;
    uint32_t delay_lock_ok;
    uint32_t drift_lock_ok;
    uint32_t freeze_bias_update;
    uint32_t hold_zone;
    uint32_t fine_zone;
    uint32_t track_zone;
    int32_t applied_step_ns;
    int32_t desired_freq_ppb;
    int32_t filtered_freq_ppb;
    uint32_t reset_prev_t1;
    uint32_t suppress_lock_update;
    int64_t step_threshold_ns;
    const char *mode_name;
    ptp_servo_quality_t quality;
    ptp_delay_guard_result_t guard_result;
    uint32_t path_rebased=0u;
    int64_t delay_ref_ns=0ll,delay_margin_ns=0ll;
    net_acq_eth_snapshot_t eth;

    if (!ctx || !ptp_cfg) {
        return;
    }

    d1 = (int64_t)ctx->t2_slave_rx_ns - (int64_t)ctx->t1_master_ns;
    d2 = (int64_t)ctx->t4_master_rx_ns - (int64_t)ctx->t3_slave_tx_ns;

    mean_path_delay_ns = div_round_nearest_s64(d1 + d2, 2ll);
    offset_raw_ns      = div_round_nearest_s64(d1 - d2, 2ll);
    offset_used_ns     = offset_raw_ns - (int64_t)PTP_OFFSET_CAL_NS;

    quality = ptp_check_exchange_quality(ctx,
                                         mean_path_delay_ns,
                                         &filtered_delay_ns);
    guard_result=PTP_DELAY_GUARD_ACCEPT;
    delay_ref_ns=ctx->have_filtered_delay?ctx->filtered_delay_ns:mean_path_delay_ns;
    delay_margin_ns=PTP_DELAY_GUARD_MARGIN_NS;
    if(mean_path_delay_ns>=PTP_DELAY_MIN_NS && mean_path_delay_ns<=PTP_DELAY_MAX_NS){
        guard_result=ptp_delay_guard_check_qualified(&ctx->delay_guard,mean_path_delay_ns,
                                           &delay_ref_ns,&delay_margin_ns,(quality!=PTP_SERVO_QUAL_OK));
    }else{
        ctx->delay_guard.candidate_count=0u;
    }
    if(guard_result==PTP_DELAY_GUARD_SPIKE){
        quality=PTP_SERVO_QUAL_PDV_SPIKE;
    }else if(guard_result==PTP_DELAY_GUARD_PATH_REBASE){
        /* A persistent stable path change must not cause permanent rejection.
         * Keep oscillator compensation, but require a fresh phase/drift lock. */
        path_rebased=1u;quality=PTP_SERVO_QUAL_PATH_REBASE;
        ptp_fast50_init(&ctx->fast50);
        filtered_delay_ns=mean_path_delay_ns;
        ctx->have_filtered_offset=0u;ctx->have_filtered_drift=0u;
        ctx->have_filtered_delay=0u;ctx->have_prev_t1=0u;
        ctx->filtered_drift_ppb=0;ctx->prev_t1_master_ns=0ull;
        ctx->fine_bias_remainder=0ll;
        ctx->locked=0u;ctx->lock_streak=0u;
        ctx->fine_locked=0u;ctx->fine_lock_streak=0u;
    }
    ctx->last_delay_ref_ns=delay_ref_ns;
    ctx->last_delay_margin_ns=delay_margin_ns;
    ctx->last_sample_quality=quality;
    if (quality != PTP_SERVO_QUAL_OK && quality != PTP_SERVO_QUAL_PATH_REBASE) {
        ctx->last_sample_valid=0u;
        ctx->rejected_exchange_count++;
        if (quality == PTP_SERVO_QUAL_DELAY_OUTLIER || quality == PTP_SERVO_QUAL_PDV_SPIKE) {
            ctx->delay_outlier_count++;
        }
        if(quality!=PTP_SERVO_QUAL_PDV_SPIKE)ptp_freeze_servo(ctx,PTP_FREEZE_AFTER_DROP_CYCLES);
        printf("[servo ] cycle=%u sync_seq=%u delay_req_seq=%u d1=%lld ns d2=%lld ns delay=%lld ns filt_delay=%lld ns offset_raw=%lld ns offset_cal=%lld ns offset=%lld ns filt_offset=%lld ns mode=REJECT phase_ppb=0 direct_ppb=0 bias_step_ppb=0 drift_ppb=%d raw_ppb=0 bias_ppb=%d target_ppb=%d freq_ppb=%d ready=%u ready_streak=%u fine_lock=%u fine_streak=%u stream=%u reject=%u holdoff=%u sample_valid=0 quality=%s delay_ref_ns=%lld delay_margin_ns=%lld spike_count=%u rebase_count=%u\n",
               (unsigned int)ctx->servo_cycle_count,(unsigned int)ctx->sync_seq_id,
               (unsigned int)ctx->delay_req_seq_id,(long long)d1,(long long)d2,
               (long long)mean_path_delay_ns,(long long)ctx->filtered_delay_ns,
               (long long)offset_raw_ns,(long long)PTP_OFFSET_CAL_NS,(long long)offset_used_ns,
               (long long)ctx->filtered_offset_ns,(int)ctx->filtered_drift_ppb,
               (int)ctx->freq_bias_ppb,(int)ctx->freq_adj_ppb,(int)ctx->freq_adj_ppb,
               (unsigned int)ctx->locked,(unsigned int)ctx->lock_streak,
               (unsigned int)ctx->fine_locked,(unsigned int)ctx->fine_lock_streak,
               (unsigned int)ctx->stream_started,(unsigned int)ctx->rejected_exchange_count,
               (unsigned int)ctx->post_adjust_holdoff_cycles,ptp_servo_quality_name(quality),
               (long long)delay_ref_ns,(long long)delay_margin_ns,
               (unsigned int)ctx->delay_guard.spike_count,(unsigned int)ctx->delay_guard.rebase_count);
#if PTP_UART_EXCHANGE_TRACE
        print_ptp_uart_exchange(ctx,d1,d2,mean_path_delay_ns,offset_raw_ns,
                                offset_used_ns,ctx->filtered_offset_ns);
#endif
        return;
    }
    ctx->last_sample_valid=1u;
    if(!path_rebased)ptp_delay_guard_accept(&ctx->delay_guard,mean_path_delay_ns);

    prev_offset_used_ns = ctx->last_offset_ns;
    prev_filtered_offset_ns = ctx->filtered_offset_ns;
    have_prev_filtered_offset = ctx->have_filtered_offset;
    ptp_select_offset_filter_coeffs(ctx, &offset_filter_num, &offset_filter_den);
    filtered_offset_ns = iir_filter_s64(ctx->filtered_offset_ns,
                                        offset_used_ns,
                                        offset_filter_num,
                                        offset_filter_den,
                                        ctx->have_filtered_offset);

    delay_slope_ns = ctx->have_filtered_delay ? (mean_path_delay_ns - ctx->last_delay_ns) : 0ll;
    filtered_drift_ppb = ctx->filtered_drift_ppb;
    drift_ppb_raw = 0ll;
    phase_term_ppb = 0ll;
    direct_phase_ppb = 0ll;
    bias_update_ppb = 0ll;
    target_freq_ppb_dbg = (int64_t)ctx->freq_adj_ppb;

    ctx->filtered_delay_ns    = filtered_delay_ns;
    ctx->have_filtered_delay  = 1u;
    ctx->filtered_offset_ns   = filtered_offset_ns;
    ctx->have_filtered_offset = 1u;
    ctx->last_delay_ns        = mean_path_delay_ns;
    ctx->last_offset_ns       = offset_used_ns;
    ctx->last_d1_ns           = d1;
    ctx->last_d2_ns           = d2;
    ctx->last_delay_slope_ns  = delay_slope_ns;
    ctx->servo_cycle_count++;
    /* These fields describe this accepted action, not a prior PI update. */
    ctx->last_phase_ppb=0ll;ctx->last_direct_phase_ppb=0ll;
    ctx->last_bias_update_ppb=0ll;ctx->last_target_freq_ppb=ctx->freq_adj_ppb;
    ctx->last_drift_ppb=0ll;

    applied_step_ns = 0;
    desired_freq_ppb = ctx->freq_adj_ppb;
    filtered_freq_ppb = ctx->freq_adj_ppb;
    raw_correction_ppb = 0ll;
    reset_prev_t1 = 0u;
    suppress_lock_update = path_rebased;
    delay_lock_ok = 0u;
    drift_lock_ok = 0u;
    freeze_bias_update = 0u;
    hold_zone = 0u;
    fine_zone = 0u;
    track_zone = 0u;
    mode_name = "HOLD";

    if (i64_abs(offset_used_ns) >= PTP_SET_TIME_THRESHOLD_NS) {
        uint64_t cur_time_ns;
        int64_t corrected_ns;

        cur_time_ns = net_acq_ptp_current_time();
        corrected_ns = (int64_t)cur_time_ns - offset_used_ns;
        if (corrected_ns < 0) {
            corrected_ns = 0;
        }

        ptp_servo_clear_frequency_state(ctx, ptp_cfg, 1u);
        ptp_apply_set_time64(ptp_cfg, (uint64_t)corrected_ns);
        printf("[servo ] large offset -> set_time64 cur=0x%08x_%08x target=0x%08x_%08x\n",
               (unsigned int)(cur_time_ns >> 32),
               (unsigned int)(cur_time_ns & 0xFFFFFFFFu),
               (unsigned int)(((uint64_t)corrected_ns) >> 32),
               (unsigned int)(((uint64_t)corrected_ns) & 0xFFFFFFFFu));
        mode_name = "SET64";
        ctx->startup_done = 1u;
        reset_prev_t1 = 1u;
        suppress_lock_update = 1u;
        ctx->filtered_offset_ns = 0ll;
        ctx->have_filtered_offset = 0u;
        ctx->post_adjust_holdoff_cycles = PTP_POST_ADJUST_HOLDOFF_CYCLES;
        filtered_offset_ns = 0ll;
    } else {
        step_threshold_ns = ctx->startup_done ? PTP_REACQUIRE_STEP_THRESHOLD_NS
                                               : PTP_STEP_THRESHOLD_NS;

        if ((step_threshold_ns > 0) &&
            (i64_abs(filtered_offset_ns) >= step_threshold_ns)) {
            /*
             * 仅对更大的启动偏差执行 full step；100~200us 级残差改由阻尼 trim
             * 逐步拉近，减少上电阶段的来回摆动。
             */
            ptp_servo_clear_frequency_state(ctx, ptp_cfg, 1u);
            applied_step_ns = saturate_i64_to_i32(-filtered_offset_ns);
            ptp_apply_step(ptp_cfg, applied_step_ns);
            mode_name = ctx->startup_done ? "RESTEP" : "STEP";
            ctx->startup_done = 1u;
            reset_prev_t1 = 1u;
            suppress_lock_update = 1u;
            ctx->filtered_offset_ns = 0ll;
            ctx->have_filtered_offset = 0u;
            ctx->post_adjust_holdoff_cycles = PTP_POST_ADJUST_HOLDOFF_CYCLES;
            filtered_offset_ns = 0ll;
        } else if (ctx->post_adjust_holdoff_cycles != 0u) {
            ctx->post_adjust_holdoff_cycles--;
            mode_name = "SETTLE";
        } else if ((!PTP_FAST50_ENABLE) && (i64_abs(filtered_offset_ns) >= PTP_TRIM_THRESHOLD_NS) &&
                   !ctx->fine_locked) {
            /*
             * coarse trim 只修相位，不清空已经学到的频率 bias/freq_adj；
             * 这样能更快把 10us~几百 us 的残差压到 track/fine 区，同时避免 full step 的过冲感。
             */
            applied_step_ns = ptp_compute_trim_step_ns(filtered_offset_ns);
            if (applied_step_ns != 0) {
                ptp_apply_step(ptp_cfg, applied_step_ns);
                mode_name = ctx->startup_done ? "RTRIM" : "TRIM";
                ctx->startup_done = 1u;
                reset_prev_t1 = 1u;
                suppress_lock_update = 1u;
                ctx->filtered_offset_ns = 0ll;
                ctx->have_filtered_offset = 0u;
                ctx->post_adjust_holdoff_cycles = PTP_POST_ADJUST_HOLDOFF_CYCLES;
                filtered_offset_ns = 0ll;
            } else {
                mode_name = "TRIM_SKIPPED";
            }
        } else {
            dt_master_ns = 0ll;
            if (ctx->have_prev_t1) {
                dt_master_ns = (int64_t)(ctx->t1_master_ns - ctx->prev_t1_master_ns);
            }

            if (PTP_FAST50_ENABLE) {
                if(ctx->freeze_servo_cycles){
                    ctx->freeze_servo_cycles--;ptp_fast50_init(&ctx->fast50);mode_name="FAST50_FREEZE";
                } else {
                    ptp_fast50_result_t fr=ptp_fast50_update(&ctx->fast50,ctx->t1_master_ns,
                        offset_used_ns,ctx->freq_adj_ppb,ctx->freq_bias_ppb);
                    bias_update_ppb=(int64_t)fr.bias_ppb-ctx->freq_bias_ppb;
                    ctx->freq_bias_ppb=fr.bias_ppb;ctx->filtered_drift_ppb=-fr.residual_ppb;
                    ctx->have_filtered_drift=fr.reanchored?0u:1u;
                    filtered_offset_ns=fr.phase_ns;ctx->filtered_offset_ns=filtered_offset_ns;
                    phase_term_ppb=fr.phase_pull_ppb;direct_phase_ppb=fr.phase_pull_ppb;
                    ctx->last_phase_ppb=phase_term_ppb;ctx->last_direct_phase_ppb=direct_phase_ppb;
                    ctx->last_bias_update_ppb=bias_update_ppb;ctx->last_target_freq_ppb=fr.command_ppb;
                    ctx->fine_bias_remainder=0ll;target_freq_ppb_dbg=fr.command_ppb;
                    if(fr.command_ppb!=ctx->freq_adj_ppb){ctx->freq_adj_ppb=fr.command_ppb;ptp_apply_freq_ppb(ptp_cfg,ctx->freq_adj_ppb);}
                    mode_name=fr.reanchored?"FAST50_INIT":"FAST50_TRACK";
                }
            } else if (dt_master_ns > 0 && have_prev_filtered_offset) {
                int64_t phase_gain_div;
                int64_t lp;
                int64_t bias_candidate_ppb;

                drift_ppb_raw = div_round_nearest_s64((-(filtered_offset_ns - prev_filtered_offset_ns)) * 1000000000ll,
                                                      dt_master_ns);
                filtered_drift_ppb = iir_filter_s64((int64_t)ctx->filtered_drift_ppb,
                                                    drift_ppb_raw,
                                                    PTP_DRIFT_FILTER_NUM,
                                                    PTP_DRIFT_FILTER_DEN,
                                                    ctx->have_filtered_drift);
                ctx->filtered_drift_ppb  = clamp_freq_ppb(filtered_drift_ppb);
                ctx->have_filtered_drift = 1u;
                ctx->last_drift_ppb      = drift_ppb_raw;

                fine_zone = ((i64_abs(filtered_offset_ns) <= PTP_FINE_OFFSET_THRESHOLD_NS) &&
                             (i64_abs((int64_t)ctx->filtered_drift_ppb) <= (int64_t)PTP_FINE_DRIFT_THRESHOLD_PPB)) ? 1u : 0u;
                hold_zone = ((i64_abs(filtered_offset_ns) <= PTP_HOLD_OFFSET_THRESHOLD_NS) &&
                             (i64_abs((int64_t)ctx->filtered_drift_ppb) <= (int64_t)PTP_HOLD_DRIFT_THRESHOLD_PPB)) ? 1u : 0u;
                track_zone = (hold_zone && !fine_zone) ? 1u : 0u;

                if (fine_zone) {
                    phase_gain_div = ctx->fine_locked ? PTP_FREQ_GAIN_DIV_FINE
                                                      : PTP_FREQ_GAIN_DIV_FINE_ACQ;
                } else if (ctx->locked || ctx->lock_streak >= PTP_LOCK_STREAK_MIN) {
                    phase_gain_div = PTP_FREQ_GAIN_DIV_LOCK;
                } else {
                    phase_gain_div = PTP_FREQ_GAIN_DIV;
                }
                if (phase_gain_div <= 0) {
                    phase_gain_div = PTP_FREQ_GAIN_DIV;
                }
                phase_term_ppb = div_round_nearest_s64((-filtered_offset_ns) * 1000000000ll,
                                                       dt_master_ns * phase_gain_div);
                ctx->last_phase_ppb = phase_term_ppb;
                raw_correction_ppb = phase_term_ppb;

                freeze_bias_update = (ctx->freeze_servo_cycles != 0u) ? 1u : 0u;
                if (ctx->freeze_servo_cycles != 0u) {
                    ctx->freeze_servo_cycles--;
                }

                if (freeze_bias_update) {
                    ctx->fine_bias_remainder = 0ll;
                    bias_update_ppb = 0ll;
                    direct_phase_ppb = 0ll;
                    target_freq_ppb_dbg = (int64_t)ctx->freq_adj_ppb;
                    desired_freq_ppb = ctx->freq_adj_ppb;
                    mode_name = "FREEZE";
                } else if (!ctx->fine_locked) {
                    /* freq_adj_ppb was held since the last accepted t1, including
                     * rejected PDV exchanges. Thus freq_adj - d(offset)/dt is an
                     * estimate of oscillator compensation, not phase integral.
                     * set/trim/rebase invalidate have_prev_t1 before this branch. */
                    int64_t measured_comp_ppb = (int64_t)ctx->freq_adj_ppb -
                        div_round_nearest_s64((offset_used_ns-prev_offset_used_ns)*1000000000ll,
                                              dt_master_ns);
                    bias_update_ppb = div_round_nearest_s64(
                        (int64_t)clamp_freq_ppb(measured_comp_ppb)-ctx->freq_bias_ppb,
                        PTP_ACQUIRE_OBSERVER_DIV);
                    bias_update_ppb = clamp_i64_abs_limit(bias_update_ppb,PTP_ACQUIRE_BIAS_STEP_MAX_PPB);
                    ctx->freq_bias_ppb = clamp_freq_ppb((int64_t)ctx->freq_bias_ppb+bias_update_ppb);
                    ctx->fine_bias_remainder = 0ll;
                    direct_phase_ppb = div_round_nearest_s64(-filtered_offset_ns*1000000000ll,
                                                           PTP_ACQUIRE_PHASE_TIME_NS);
                    direct_phase_ppb = clamp_i64_abs_limit(direct_phase_ppb,PTP_ACQUIRE_DIRECT_MAX_PPB);
                    target_freq_ppb_dbg = (int64_t)ctx->freq_bias_ppb+direct_phase_ppb;
                    desired_freq_ppb = clamp_freq_ppb(target_freq_ppb_dbg);
                    mode_name = "ACQUIRE";
                } else if (fine_zone) {
                    if (ctx->fine_locked) {
                        bias_update_ppb = ptp_fine_bias_step_ppb(ctx, phase_term_ppb);
                        bias_candidate_ppb = (int64_t)ctx->freq_bias_ppb + bias_update_ppb;
                        ctx->freq_bias_ppb = clamp_freq_ppb(bias_candidate_ppb);
                        if ((int64_t)ctx->freq_bias_ppb != bias_candidate_ppb) {
                            ctx->fine_bias_remainder = 0ll; /* no fractional wind-up at a frequency limit */
                        }

                        direct_phase_ppb = div_round_nearest_s64(phase_term_ppb, PTP_FINE_PHASE_DIRECT_DIV);
                        direct_phase_ppb = clamp_i64_abs_limit(direct_phase_ppb, (int64_t)PTP_FINE_DIRECT_PHASE_MAX_PPB);
                        target_freq_ppb_dbg = (int64_t)ctx->freq_bias_ppb +
                                              (int64_t)ctx->filtered_drift_ppb +
                                              direct_phase_ppb;
                        desired_freq_ppb = clamp_freq_ppb(target_freq_ppb_dbg);
                        mode_name = "FINE_HOLD";
                    } else {
                        bias_update_ppb = div_round_nearest_s64(phase_term_ppb, PTP_FINE_ACQ_BIAS_ACC_DIV);
                        bias_update_ppb = clamp_i64_abs_limit(bias_update_ppb, (int64_t)PTP_FINE_ACQ_BIAS_UPDATE_MAX_PPB);
                        bias_candidate_ppb = (int64_t)ctx->freq_bias_ppb + bias_update_ppb;
                        ctx->freq_bias_ppb = clamp_freq_ppb(bias_candidate_ppb);

                        direct_phase_ppb = div_round_nearest_s64(phase_term_ppb, PTP_FINE_ACQ_PHASE_DIRECT_DIV);
                        direct_phase_ppb = clamp_i64_abs_limit(direct_phase_ppb, (int64_t)PTP_FINE_ACQ_DIRECT_PHASE_MAX_PPB);
                        target_freq_ppb_dbg = (int64_t)ctx->freq_bias_ppb +
                                              (int64_t)ctx->filtered_drift_ppb +
                                              direct_phase_ppb;
                        desired_freq_ppb = clamp_freq_ppb(target_freq_ppb_dbg);
                        mode_name = "FINE_ACQ";
                    }
                } else if (track_zone) {
                    bias_update_ppb = div_round_nearest_s64(phase_term_ppb, PTP_TRACK_BIAS_ACC_DIV);
                    bias_update_ppb = clamp_i64_abs_limit(bias_update_ppb, (int64_t)PTP_TRACK_BIAS_UPDATE_MAX_PPB);
                    bias_candidate_ppb = (int64_t)ctx->freq_bias_ppb + bias_update_ppb;
                    ctx->freq_bias_ppb = clamp_freq_ppb(bias_candidate_ppb);

                    direct_phase_ppb = div_round_nearest_s64(phase_term_ppb, PTP_TRACK_PHASE_DIRECT_DIV);
                    direct_phase_ppb = clamp_i64_abs_limit(direct_phase_ppb, (int64_t)PTP_TRACK_DIRECT_PHASE_MAX_PPB);
                    target_freq_ppb_dbg = (int64_t)ctx->freq_bias_ppb +
                                          (int64_t)ctx->filtered_drift_ppb +
                                          direct_phase_ppb;
                    desired_freq_ppb = clamp_freq_ppb(target_freq_ppb_dbg);
                    mode_name = (i64_abs(filtered_offset_ns) <= PTP_FREQ_HOLD_THRESHOLD_NS) ? "TRACK_HOLD" : "TRACK";
                } else {
                    bias_update_ppb = div_round_nearest_s64(phase_term_ppb, PTP_BIAS_ACC_DIV);
                    bias_update_ppb = clamp_i64_abs_limit(bias_update_ppb, (int64_t)PTP_BIAS_UPDATE_MAX_PPB);
                    bias_candidate_ppb = (int64_t)ctx->freq_bias_ppb + bias_update_ppb;
                    ctx->freq_bias_ppb = clamp_freq_ppb(bias_candidate_ppb);

                    direct_phase_ppb = div_round_nearest_s64(phase_term_ppb, PTP_PHASE_DIRECT_DIV);
                    direct_phase_ppb = clamp_i64_abs_limit(direct_phase_ppb, (int64_t)PTP_DIRECT_PHASE_MAX_PPB);
                    target_freq_ppb_dbg = (int64_t)ctx->freq_bias_ppb +
                                          (int64_t)ctx->filtered_drift_ppb +
                                          direct_phase_ppb;
                    desired_freq_ppb = clamp_freq_ppb(target_freq_ppb_dbg);
                    mode_name = "FREQ";
                }

                ptp_select_freq_filter_coeffs(ctx,
                                              fine_zone,
                                              track_zone,
                                              &freq_filter_num,
                                              &freq_filter_den);
                lp = div_round_nearest_s64((int64_t)ctx->freq_adj_ppb * freq_filter_num +
                                           (int64_t)desired_freq_ppb,
                                           freq_filter_den);
                filtered_freq_ppb = clamp_freq_ppb(lp);

                ctx->last_direct_phase_ppb = direct_phase_ppb;
                ctx->last_bias_update_ppb  = bias_update_ppb;
                ctx->last_target_freq_ppb  = target_freq_ppb_dbg;

                if (filtered_freq_ppb != ctx->freq_adj_ppb) {
                    ctx->freq_adj_ppb = filtered_freq_ppb;
                    ptp_apply_freq_ppb(ptp_cfg, ctx->freq_adj_ppb);
                }
            } else if (i64_abs(filtered_offset_ns) <= PTP_FREQ_HOLD_THRESHOLD_NS) {
                mode_name = "DEADBAND";
            }
        }
    }

    if (reset_prev_t1) {
        ptp_fast50_init(&ctx->fast50);
        ctx->fine_bias_remainder = 0ll;
        ctx->prev_t1_master_ns   = 0ull;
        ctx->have_prev_t1        = 0u;
        ctx->filtered_drift_ppb  = 0;
        ctx->have_filtered_drift = 0u;
        ctx->last_drift_ppb      = 0ll;
        ctx->last_phase_ppb      = 0ll;
        ctx->last_direct_phase_ppb = 0ll;
        ctx->last_bias_update_ppb  = 0ll;
        ctx->last_target_freq_ppb  = (int64_t)ctx->freq_adj_ppb;
    } else {
        ctx->prev_t1_master_ns = ctx->t1_master_ns;
        ctx->have_prev_t1      = 1u;
    }

    delay_lock_ok = ((mean_path_delay_ns >= PTP_DELAY_LOCK_MIN_NS) &&
                     (mean_path_delay_ns <= PTP_DELAY_LOCK_MAX_NS) &&
                     ((!ctx->have_filtered_delay) || (i64_abs(delay_slope_ns) <= PTP_DELAY_LOCK_SLOPE_MAX_NS))) ? 1u : 0u;
    drift_lock_ok = ((!ctx->have_filtered_drift) ||
                     (i64_abs((int64_t)ctx->filtered_drift_ppb) <= (int64_t)PTP_LOCK_DRIFT_THRESHOLD_PPB)) ? 1u : 0u;

    ptp_update_lock_flags(ctx,
                          filtered_offset_ns,
                          delay_lock_ok && drift_lock_ok,
                          suppress_lock_update);

    net_acq_read_eth_snapshot(&eth);
    printf("[servo ] cycle=%u sync_seq=%u delay_req_seq=%u d1=%lld ns d2=%lld ns delay=%lld ns filt_delay=%lld ns delay_slope=%lld ns offset_raw=%lld ns offset_cal=%lld ns offset=%lld ns filt_offset=%lld ns mode=%s step=%d ns phase_ppb=%lld direct_ppb=%lld bias_step_ppb=%lld drift_ppb=%lld raw_ppb=%lld bias_ppb=%d target_ppb=%lld freq_ppb=%d cur_time=0x%08x_%08x ready=%u ready_streak=%u fine_lock=%u fine_streak=%u delay_ok=%u drift_ok=%u fine_zone=%u track=%u hold=%u freeze=%u stream=%u reject=%u holdoff=%u sample_valid=1 quality=%s delay_ref_ns=%lld delay_margin_ns=%lld spike_count=%u rebase_count=%u\n",
           (unsigned int)ctx->servo_cycle_count,
           (unsigned int)ctx->sync_seq_id,
           (unsigned int)ctx->delay_req_seq_id,
           (long long)d1,
           (long long)d2,
           (long long)mean_path_delay_ns,
           (long long)ctx->filtered_delay_ns,
           (long long)delay_slope_ns,
           (long long)offset_raw_ns,
           (long long)PTP_OFFSET_CAL_NS,
           (long long)offset_used_ns,
           (long long)filtered_offset_ns,
           mode_name,
           (int)applied_step_ns,
           (long long)phase_term_ppb,
           (long long)ctx->last_direct_phase_ppb,
           (long long)ctx->last_bias_update_ppb,
           (long long)ctx->filtered_drift_ppb,
           (long long)raw_correction_ppb,
           (int)ctx->freq_bias_ppb,
           (long long)ctx->last_target_freq_ppb,
           (int)ctx->freq_adj_ppb,
           (unsigned int)(eth.ptp_cur_time_ns >> 32),
           (unsigned int)(eth.ptp_cur_time_ns & 0xFFFFFFFFu),
           (unsigned int)ctx->locked,
           (unsigned int)ctx->lock_streak,
           (unsigned int)ctx->fine_locked,
           (unsigned int)ctx->fine_lock_streak,
           (unsigned int)delay_lock_ok,
           (unsigned int)drift_lock_ok,
           (unsigned int)fine_zone,
           (unsigned int)track_zone,
           (unsigned int)hold_zone,
           (unsigned int)ctx->freeze_servo_cycles,
           (unsigned int)ctx->stream_started,
           (unsigned int)ctx->rejected_exchange_count,
           (unsigned int)ctx->post_adjust_holdoff_cycles,
           ptp_servo_quality_name(quality),
           (long long)delay_ref_ns,(long long)delay_margin_ns,
           (unsigned int)ctx->delay_guard.spike_count,
           (unsigned int)ctx->delay_guard.rebase_count);
#if PTP_UART_EXCHANGE_TRACE
    print_ptp_uart_exchange(ctx,
                            d1,
                            d2,
                            mean_path_delay_ns,
                            offset_raw_ns,
                            offset_used_ns,
                            filtered_offset_ns);
#endif
}

static int handle_ptp_rx_event(ptp_slave_ctx_t *ctx,
                               net_acq_ptp_test_cfg_t *ptp_cfg,
                               const net_acq_ptp_rx_snapshot_t *mbox)
{
    int rc;

    if (!ctx || !ptp_cfg || !mbox) {
        return -1;
    }

    ctx->ptp_rx_count++;

    if (PTP_VERBOSE_RX) {
        print_rx_summary(mbox);
    }

    switch (mbox->msg_type) {
    case NET_ACQ_PTP_MSG_SYNC:
        if (ctx->state != PTP_WAIT_SYNC) {
            printf("[ptp   ] new Sync arrived while state=%s, drop old cycle and re-sync\n",
                   ptp_state_name(ctx->state));
            ctx->cycle_drop_count++;
            ptp_freeze_servo(ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
        }
        ctx->state                    = PTP_WAIT_FOLLOW_UP;
        ctx->sync_seq_id              = mbox->sequence_id;
        ctx->delay_req_seq_id         = 0u;
        ctx->t2_slave_rx_ns           = mbox->rx_timestamp_ns;
        ctx->follow_up_timeout_polls  = 0u;
        ctx->delay_resp_timeout_polls = 0u;
        printf("[ptp   ] Sync accepted: seq=%u, t2=0x%08x_%08x\n",
               (unsigned int)ctx->sync_seq_id,
               (unsigned int)(ctx->t2_slave_rx_ns >> 32),
               (unsigned int)(ctx->t2_slave_rx_ns & 0xFFFFFFFFu));
        return 0;

    case NET_ACQ_PTP_MSG_FOLLOW_UP:
        if (ctx->state != PTP_WAIT_FOLLOW_UP) {
            printf("[ptp   ] ignore Follow_Up seq=%u because state=%s\n",
                   (unsigned int)mbox->sequence_id,
                   ptp_state_name(ctx->state));
            return 0;
        }
        if (mbox->sequence_id != ctx->sync_seq_id) {
            printf("[ptp   ] ignore Follow_Up seq=%u because waiting seq=%u\n",
                   (unsigned int)mbox->sequence_id,
                   (unsigned int)ctx->sync_seq_id);
            return 0;
        }

#if PTP_VALIDATE_RX_MSG_TIMESTAMP
        if (!ptp_u80_to_ns_checked(&mbox->msg_timestamp, &ctx->t1_master_ns)) {
            ctx->invalid_follow_up_count++;
            ctx->cycle_drop_count++;
            ptp_freeze_servo(ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
            printf("[ptp   ] ignore malformed Follow_Up seq=%u: preciseOriginTimestamp=",
                   (unsigned int)mbox->sequence_id);
            net_acq_print_u80_hex(&mbox->msg_timestamp);
            printf(" has invalid ns field\n");
            ptp_ctx_reset_cycle(ctx);
            return 0;
        }
#else
        ctx->t1_master_ns = ptp_u80_to_ns(&mbox->msg_timestamp);
#endif
        ctx->delay_req_seq_id = ptp_next_delay_req_seq_id(ctx);
        printf("[ptp   ] Follow_Up matched: sync_seq=%u delay_req_seq=%u t1=0x%08x_%08x\n",
               (unsigned int)ctx->sync_seq_id,
               (unsigned int)ctx->delay_req_seq_id,
               (unsigned int)(ctx->t1_master_ns >> 32),
               (unsigned int)(ctx->t1_master_ns & 0xFFFFFFFFu));

        rc = ptp_send_delay_req(ptp_cfg, ctx->delay_req_seq_id, &ctx->t3_slave_tx_ns);
        if (rc != 0) {
            ctx->delay_req_trigger_fail_count++;
            ctx->txstamp_timeout_streak++;
            ctx->cycle_drop_count++;
            ptp_freeze_servo(ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
            printf("[ptp   ] Delay_Req trigger failed, rc=%d (fail_count=%u, streak=%u). Drop only this cycle.\n",
                   rc,
                   (unsigned int)ctx->delay_req_trigger_fail_count,
                   (unsigned int)ctx->txstamp_timeout_streak);
            clear_stale_txstamp(ptp_cfg);
            ptp_ctx_reset_cycle(ctx);
            if (ctx->txstamp_timeout_streak >= PTP_TX_TRIGGER_FAIL_RESTART_THRESHOLD) {
                printf("[ptp   ] Delay_Req trigger failures reached threshold (%u), request restart\n",
                       (unsigned int)PTP_TX_TRIGGER_FAIL_RESTART_THRESHOLD);
                return rc;
            }
            return 0;
        }

        ctx->txstamp_timeout_streak   = 0u;
        ctx->state                    = PTP_WAIT_DELAY_RESP;
        ctx->delay_resp_timeout_polls = 0u;
        printf("[ptp   ] Delay_Req sent: seq=%u, t3=0x%08x_%08x\n",
               (unsigned int)ctx->delay_req_seq_id,
               (unsigned int)(ctx->t3_slave_tx_ns >> 32),
               (unsigned int)(ctx->t3_slave_tx_ns & 0xFFFFFFFFu));
        return 0;

    case NET_ACQ_PTP_MSG_DELAY_RESP:
        if (ctx->state != PTP_WAIT_DELAY_RESP) {
            printf("[ptp   ] ignore Delay_Resp seq=%u because state=%s\n",
                   (unsigned int)mbox->sequence_id,
                   ptp_state_name(ctx->state));
            return 0;
        }
        if (mbox->sequence_id != ctx->delay_req_seq_id) {
            printf("[ptp   ] ignore Delay_Resp seq=%u because waiting seq=%u\n",
                   (unsigned int)mbox->sequence_id,
                   (unsigned int)ctx->delay_req_seq_id);
            return 0;
        }

        {
            net_acq_u80_t expected_requestor;

            expected_requestor = ptp_build_request_port_identity(ptp_cfg);
            if (!ptp_u80_equal(&mbox->requesting_port_identity, &expected_requestor)) {
                ctx->delay_resp_port_mismatch_count++;
                printf("[ptp   ] ignore Delay_Resp seq=%u because requestingPortIdentity mismatched, got=",
                       (unsigned int)mbox->sequence_id);
                net_acq_print_u80_hex(&mbox->requesting_port_identity);
                printf(", expect=");
                net_acq_print_u80_hex(&expected_requestor);
                printf("\n");
                return 0;
            }
        }

#if PTP_VALIDATE_RX_MSG_TIMESTAMP
        if (!ptp_u80_to_ns_checked(&mbox->msg_timestamp, &ctx->t4_master_rx_ns)) {
            ctx->invalid_delay_resp_count++;
            ctx->cycle_drop_count++;
            ptp_freeze_servo(ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
            printf("[ptp   ] ignore malformed Delay_Resp seq=%u: receiveTimestamp=",
                   (unsigned int)mbox->sequence_id);
            net_acq_print_u80_hex(&mbox->msg_timestamp);
            printf(" has invalid ns field\n");
            ptp_ctx_reset_cycle(ctx);
            return 0;
        }
#else
        ctx->t4_master_rx_ns = ptp_u80_to_ns(&mbox->msg_timestamp);
#endif
        printf("[ptp   ] Delay_Resp matched: seq=%u, t4=0x%08x_%08x\n",
               (unsigned int)ctx->delay_req_seq_id,
               (unsigned int)(ctx->t4_master_rx_ns >> 32),
               (unsigned int)(ctx->t4_master_rx_ns & 0xFFFFFFFFu));

        ctx->txstamp_timeout_streak = 0u;
        ptp_handle_servo_result(ctx, ptp_cfg);
        ptp_ctx_reset_cycle(ctx);
        return 0;

    default:
        printf("[ptp   ] ignore unsupported mailbox msg_type=%u\n",
               (unsigned int)mbox->msg_type);
        return 0;
    }
}

static int run_forever(const net_acq_eth_cfg_t *eth_cfg,
                       net_acq_txsched_cfg_t *txs_cfg,
                       net_acq_ptp_test_cfg_t *ptp_cfg)
{
    ptp_slave_ctx_t ctx;
    ptp_lock_display_t display = {0u,0u,0u,0u};
    uint32_t last_health_tick=(uint32_t)get_cpu_clock_count();
    uint32_t previous_link_up=1u;
    ptp_time_quality_ctx_t time_quality;
    ptp_link_recovery_ctx_t link_recovery;
    net_acq_snapshot_t front;
    net_acq_debug_snapshot_t dbg;
    net_acq_eth_snapshot_t eth;
    net_acq_dbgext_snapshot_t dbgext;
    net_acq_txsched_snapshot_t txs;
    uint32_t heartbeat_polls;
    uint32_t stall_heartbeats;
    uint32_t last_front_pkt;
    uint32_t last_eth_pkt;
    uint32_t last_eth_bytes;
    int rc;

    if (!eth_cfg || !ptp_cfg || !txs_cfg) {
        return -1;
    }

    ptp_ctx_init(&ctx);
    ptp_lock_display_write(0u);
    ptp_time_quality_runtime_init(&time_quality);
    ptp_link_recovery_init(&link_recovery);
    heartbeat_polls = 0u;
    stall_heartbeats = 0u;
    last_front_pkt = net_acq_read(NET_ACQ_PACKET_COUNT);
    last_eth_pkt = net_acq_read(NET_ACQ_ETH_TX_PKT_COUNT);
    last_eth_bytes = net_acq_read(NET_ACQ_ETH_TX_BYTE_COUNT);

    while (1) {
        net_acq_ptp_rx_snapshot_t mbox;
        uint32_t handled_mailbox;
        uint32_t eth_status_now;

        eth_status_now=net_acq_read(NET_ACQ_ETH_STATUS);
        ptp_observe_link_edge(&ctx,&time_quality,&previous_link_up,
                             (eth_status_now & ETH_STATUS_LINK_UP)?1u:0u);
        (void)ptp_suspend_stream_for_link_loss(&ctx,&time_quality,eth_status_now);

        /* Health time is elapsed CPU time, independent of traffic/UART polls. */
        {
            uint32_t now=(uint32_t)get_cpu_clock_count();
            uint32_t due=(uint32_t)(now-last_health_tick)/(uint32_t)CORE_CLOCKS_PER_SEC;
            if(due){
                uint32_t status=net_acq_read(NET_ACQ_ETH_STATUS);
                net_acq_dbgext_snapshot_t health_dbg;
                last_health_tick += due*(uint32_t)CORE_CLOCKS_PER_SEC;
                net_acq_read_dbgext_snapshot(&health_dbg);
                if(due>PTP_TIME_INVALID_AFTER_TICKS+1u)due=PTP_TIME_INVALID_AFTER_TICKS+1u;
                while(due--){
                    ptp_time_quality_on_health_tick(&time_quality);
                    (void)ptp_apply_time_quality_event(&ctx,&time_quality);
                }
                printf("[precision] precision_limit_ns=%lld precision_streak=%u controller=%s servo_build=0x%08x\n",
                       (long long)PTP_FINE_LOCK_THRESHOLD_NS,(unsigned)PTP_FINE_LOCK_STREAK_MIN,
                       PTP_FAST50_ENABLE?"FAST50":"LEGACY",(unsigned)PTP_CONTROL_BUILD_ID);
                /* A single present link snapshot is not several seconds of
                 * continuous stable-up evidence. Advance recovery only once. */
                {
                    ptp_link_recovery_action_t action=ptp_link_recovery_on_health_tick(
                        &link_recovery,(status & ETH_STATUS_LINK_UP)?1u:0u,
                        health_dbg.ptp_rx_accept_count,ctx.servo_cycle_count);
                    ptp_apply_link_recovery_action(&ctx,action);
                }
            }
        }

        handled_mailbox = 0u;
        net_acq_ptp_rx_read_snapshot(&mbox);
        if (mbox.valid) {
            handled_mailbox = 1u;
            net_acq_ptp_rx_mbox_pop();
            if (mbox.overflow_sticky) {
                ctx.mailbox_overflow_count++;
            }
            if ((eth_status_now & ETH_STATUS_LINK_UP) &&
                handle_ptp_rx_event(&ctx, ptp_cfg, &mbox) != 0) {
                net_acq_dump_all_registers();
                return -200;
            }
        } else if (mbox.overflow_sticky) {
            handled_mailbox = 1u;
            ctx.mailbox_overflow_count++;
            printf("[ptp-rx] warning: overflow_sticky asserted, CPU 曾经没及时 pop mailbox (count=%u)\n",
                   (unsigned int)ctx.mailbox_overflow_count);
            net_acq_ptp_rx_mbox_pop();
        }

        if (ptp_time_quality_observe_servo(&time_quality,
                                           ctx.servo_cycle_count,
                                           ctx.locked)) {
            (void)ptp_apply_time_quality_event(&ctx, &time_quality);
        }

        /* Stop the data stream immediately on a link loss so its FIFO cannot
         * overflow while the bounded recovery state machine stays alive. */
        eth_status_now = net_acq_read(NET_ACQ_ETH_STATUS);
        (void)ptp_suspend_stream_for_link_loss(&ctx, &time_quality, eth_status_now);

        if (!PTP_ONLY_MODE &&
            !ctx.stream_started &&
            ptp_time_quality_stream_start_allowed(
                &time_quality,
                ((eth_status_now & ETH_STATUS_LINK_UP) != 0u) ? 1u : 0u) &&
            (ctx.servo_cycle_count >= PTP_STREAM_START_AFTER_SERVO_CYCLES) &&
            ((!PTP_STREAM_REQUIRE_LOCK) || ctx.locked) &&
            (i64_abs(ctx.filtered_offset_ns) <= PTP_STREAM_START_MAX_OFFSET_NS) &&
            (i64_abs((int64_t)ctx.filtered_drift_ppb) <= (int64_t)PTP_STREAM_START_MAX_DRIFT_PPB) &&
            (ctx.last_delay_ns >= PTP_DELAY_LOCK_MIN_NS) &&
            (ctx.last_delay_ns <= PTP_DELAY_LOCK_MAX_NS)) {
            if (txs_cfg->enable) {
                rc = net_acq_tsnlite_v1_arm(eth_cfg,
                                            txs_cfg,
                                            PTP_POLL_DELAY_CYCLES,
                                            PTP_CUR_TIME_VALID_POLLS);
                if (rc != 0) {
                    printf("[run   ] TSN-lite v1 arm failed, rc=%d\n", rc);
                    net_acq_dump_all_registers();
                    return rc;
                }
            }

            printf("[run   ] PTP stream gate passed (cycles=%u, ready=%u, fine=%u, filt_offset=%lld ns, drift=%d ppb, delay=%lld ns), enable acquisition/UDP stream now\n",
                   (unsigned int)ctx.servo_cycle_count,
                   (unsigned int)ctx.locked,
                   (unsigned int)ctx.fine_locked,
                   (long long)ctx.filtered_offset_ns,
                   (int)ctx.filtered_drift_ppb,
                   (long long)ctx.last_delay_ns);
            net_acq_enable(UDP_STREAM_USE_EXTERNAL_SOURCE);
            net_acq_delay_cycles(PTP_ENABLE_SETTLE_CYCLES);
            ctx.stream_started = 1u;
            last_front_pkt = net_acq_read(NET_ACQ_PACKET_COUNT);
            last_eth_pkt = net_acq_read(NET_ACQ_ETH_TX_PKT_COUNT);
            last_eth_bytes = net_acq_read(NET_ACQ_ETH_TX_BYTE_COUNT);
            stall_heartbeats = 0u;
        }

        if (ctx.state == PTP_WAIT_FOLLOW_UP) {
            ctx.follow_up_timeout_polls++;
            if (ctx.follow_up_timeout_polls >= PTP_FOLLOW_UP_TIMEOUT_POLLS) {
                printf("[ptp   ] timeout waiting Follow_Up for seq=%u, reset state\n",
                       (unsigned int)ctx.sync_seq_id);
                ptp_freeze_servo(&ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
                ptp_ctx_reset_cycle(&ctx);
            }
        } else if (ctx.state == PTP_WAIT_DELAY_RESP) {
            ctx.delay_resp_timeout_polls++;
            if (ctx.delay_resp_timeout_polls >= PTP_DELAY_RESP_TIMEOUT_POLLS) {
                printf("[ptp   ] timeout waiting Delay_Resp for seq=%u, reset state\n",
                       (unsigned int)ctx.delay_req_seq_id);
                ptp_freeze_servo(&ctx, PTP_FREEZE_AFTER_DROP_CYCLES);
                ptp_ctx_reset_cycle(&ctx);
            }
        }

        {
            uint32_t now=(uint32_t)get_cpu_clock_count();
            uint32_t digit = ptp_lock_display_poll(
                &display, ctx.servo_cycle_count, ctx.fine_locked,
                (ctx.freeze_servo_cycles != 0u || ctx.post_adjust_holdoff_cycles != 0u),
                now,
                (uint32_t)CORE_CLOCKS_PER_SEC); /* about 1 s without an accepted exchange */
            ptp_expire_sync_lock(&ctx,display.last_ticks,now,(uint32_t)CORE_CLOCKS_PER_SEC);
            if (digit != display.shown) {
                ptp_lock_display_write(digit);
                display.shown = digit;
            }
        }

#if PTP_IMMEDIATE_REPOLL_AFTER_RX
        if (handled_mailbox) {
            continue;
        }
#endif

        net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
        heartbeat_polls++;
        if (heartbeat_polls < PTP_HEARTBEAT_POLLS) {
            continue;
        }
        heartbeat_polls = 0u;


        net_acq_read_snapshot(&front);
        net_acq_read_debug_snapshot(&dbg);
        net_acq_read_eth_snapshot(&eth);
        net_acq_read_dbgext_snapshot(&dbgext);
        if (txs_cfg->enable) {
            net_acq_read_txsched_snapshot(&txs);
        }

        /* Cover a down edge between the fast poll and this snapshot, then
         * refresh sticky status after clearing it. */
        if (ptp_suspend_stream_for_link_loss(&ctx, &time_quality, eth.eth_status)) {
            net_acq_delay_cycles(PTP_POLL_DELAY_CYCLES);
            net_acq_read_snapshot(&front);
            net_acq_read_debug_snapshot(&dbg);
            net_acq_read_eth_snapshot(&eth);
            net_acq_read_dbgext_snapshot(&dbgext);
            if (txs_cfg->enable) {
                net_acq_read_txsched_snapshot(&txs);
            }
        }

        {
            int fault_rc;
            fault_rc = runtime_fault_check(&front, &dbg, &eth, &link_recovery);
            if (fault_rc != 0) {
                printf("[run] runtime fault rc=%d, requesting restart\n", fault_rc);
                net_acq_dump_all_registers();
                return fault_rc;
            }
        }

        {
            uint32_t front_delta;
            uint32_t eth_delta;
            uint32_t eth_byte_delta;
            uint32_t fifo_level;
            uint32_t sample_fifo;
            uint32_t info_fifo;

            front_delta = front.packet_count - last_front_pkt;
            eth_delta = eth.tx_packet_count - last_eth_pkt;
            eth_byte_delta = eth.tx_byte_count - last_eth_bytes;
            fifo_level = front.fifo_level;
            sample_fifo = net_acq_sample_fifo_level(fifo_level);
            info_fifo = net_acq_info_fifo_level(fifo_level);

            printf("[hb    ] state=%s time_quality=%s recovery=%s quality_miss=%u quality_changes=%u servo_cycles=%u ready=%u fine=%u stream=%u front_pkt=%u (+%u) eth_pkt=%u (+%u) eth_bytes=%u (+%u) good=%u bad=%u sample_fifo=%u info_fifo=%u\n",
                   ptp_state_name(ctx.state),
                   ptp_time_quality_name(time_quality.state),
                   ptp_link_recovery_state_name(link_recovery.state),
                   (unsigned int)time_quality.missed_health_ticks,
                   (unsigned int)time_quality.transition_count,
                   (unsigned int)ctx.servo_cycle_count,
                   (unsigned int)ctx.locked,
                   (unsigned int)ctx.fine_locked,
                   (unsigned int)ctx.stream_started,
                   (unsigned int)front.packet_count,
                   (unsigned int)front_delta,
                   (unsigned int)eth.tx_packet_count,
                   (unsigned int)eth_delta,
                   (unsigned int)eth.tx_byte_count,
                   (unsigned int)eth_byte_delta,
                   (unsigned int)dbg.dbg_good_packet_count,
                   (unsigned int)dbg.dbg_bad_packet_count,
                   (unsigned int)sample_fifo,
                   (unsigned int)info_fifo);

            printf("[hb    ] last_acq_ts=0x%08x_%08x ptp_cur_time=0x%08x_%08x d1=%lld d2=%lld offset=%lld filt_offset=%lld delay=%lld filt_delay=%lld delay_slope=%lld drift_ppb=%lld freq_ppb=%d ready_streak=%u fine_streak=%u ptp_rx=%u overflow=%u reject=%u delay_outlier=%u holdoff=%u drops=%u tx_fail=%u bad_fup=%u bad_dr=%u port_mis=%u eth_status=0x%08x\n",
                   (unsigned int)(front.last_timestamp >> 32),
                   (unsigned int)(front.last_timestamp & 0xFFFFFFFFu),
                   (unsigned int)(eth.ptp_cur_time_ns >> 32),
                   (unsigned int)(eth.ptp_cur_time_ns & 0xFFFFFFFFu),
                   (long long)ctx.last_d1_ns,
                   (long long)ctx.last_d2_ns,
                   (long long)ctx.last_offset_ns,
                   (long long)ctx.filtered_offset_ns,
                   (long long)ctx.last_delay_ns,
                   (long long)ctx.filtered_delay_ns,
                   (long long)ctx.last_delay_slope_ns,
                   (long long)ctx.filtered_drift_ppb,
                   (int)ctx.freq_adj_ppb,
                   (unsigned int)ctx.lock_streak,
                   (unsigned int)ctx.fine_lock_streak,
                   (unsigned int)ctx.ptp_rx_count,
                   (unsigned int)ctx.mailbox_overflow_count,
                   (unsigned int)ctx.rejected_exchange_count,
                   (unsigned int)ctx.delay_outlier_count,
                   (unsigned int)ctx.post_adjust_holdoff_cycles,
                   (unsigned int)ctx.cycle_drop_count,
                   (unsigned int)ctx.delay_req_trigger_fail_count,
                   (unsigned int)ctx.invalid_follow_up_count,
                   (unsigned int)ctx.invalid_delay_resp_count,
                   (unsigned int)ctx.delay_resp_port_mismatch_count,
                   (unsigned int)eth.eth_status);

            if (txs_cfg->enable) {
                printf("[hb-tsn] txs_status=0x%08x held=%u pass=%u late=%u miss=%u last_delta=%u trig_req=%u trig_acc=%u trig_busy=%u arb_data=%u arb_ptp=%u\n",
                       (unsigned int)txs.status,
                       (unsigned int)txs.held_count,
                       (unsigned int)txs.pass_count,
                       (unsigned int)txs.late_count,
                       (unsigned int)txs.slot_miss_count,
                       (unsigned int)txs.last_delta_ns,
                       (unsigned int)dbgext.ptp_tx_trigger_req_count,
                       (unsigned int)dbgext.ptp_tx_trigger_accept_count,
                       (unsigned int)dbgext.ptp_tx_trigger_while_busy_count,
                       (unsigned int)dbgext.arb_data_frame_count,
                       (unsigned int)dbgext.arb_ptp_frame_count);
            }

            printf("[hb-rx ] dbgext_accept=%u dbgext_drop=%u dbgext_last_err=%u fifo_overflow=%u ptp_trig_req=%u ptp_trig_acc=%u ptp_busy=%u\n",
                   (unsigned int)dbgext.ptp_rx_accept_count,
                   (unsigned int)dbgext.ptp_rx_drop_count,
                   (unsigned int)dbgext.ptp_rx_last_error,
                   (dbgext.ptp_rx_status & NET_ACQ_DBGEXT_PTP_RX_STATUS_FIFO_OVERFLOW) ? 1u : 0u,
                   (unsigned int)dbgext.ptp_tx_trigger_req_count,
                   (unsigned int)dbgext.ptp_tx_trigger_accept_count,
                   (unsigned int)dbgext.ptp_tx_trigger_while_busy_count);

            if (ctx.stream_started) {
                if (eth_delta == 0u) {
                    stall_heartbeats++;
                    printf("[hb    ] warning: no new ETH packets during this heartbeat (%u/%u)\n",
                           (unsigned int)stall_heartbeats,
                           (unsigned int)PTP_STALL_HEARTBEATS);
                    if (stall_heartbeats >= PTP_STALL_HEARTBEATS) {
                        printf("[hb    ] ETH packet counter stalled too long, requesting restart\n");
                        net_acq_dump_all_registers();
                        return -30;
                    }
                } else {
                    stall_heartbeats = 0u;
                }
            } else {
                stall_heartbeats = 0u;
            }

            last_front_pkt = front.packet_count;
            last_eth_pkt = eth.tx_packet_count;
            last_eth_bytes = eth.tx_byte_count;
        }
    }
}

int main(int argc, char **argv)
{
    net_acq_eth_cfg_t eth_cfg;
    net_acq_txsched_cfg_t txs_cfg;
    net_acq_ptp_test_cfg_t ptp_cfg;
    ptp_slave_ctx_t dummy_ctx;
    uint32_t restart_count;
    int rc;

    (void)argc;
    (void)argv;

    ptp_lock_display_write(0u);
    eth_cfg = build_eth_cfg();
    txs_cfg = build_txsched_cfg();
    ptp_cfg = build_ptp_cfg();
    ptp_ctx_init(&dummy_ctx);
    print_banner(&eth_cfg, &ptp_cfg, &txs_cfg);

    restart_count = 0u;
    while (1) {
        ptp_lock_display_write(0u);
        txs_cfg = build_txsched_cfg();
        ptp_cfg = build_ptp_cfg();
        printf("\n[main  ] bringup round = %u\n", (unsigned int)restart_count);
        rc = bringup_once(&eth_cfg, &txs_cfg, &ptp_cfg, &dummy_ctx);
        if (rc != 0) {
            printf("[main  ] bringup failed, rc=%d; backoff then retry\n", rc);
            net_acq_delay_cycles(PTP_RESTART_BACKOFF_CYCLES);
            restart_count++;
            continue;
        }

        rc = run_forever(&eth_cfg, &txs_cfg, &ptp_cfg);
        ptp_lock_display_write(0u);
        printf("[main  ] run loop exited, rc=%d; backoff then retry\n", rc);
        net_acq_delay_cycles(PTP_RESTART_BACKOFF_CYCLES);
        restart_count++;
    }
}
