#ifndef PTP_LINK_RECOVERY_H
#define PTP_LINK_RECOVERY_H

#include <stdint.h>

#define PTP_LINK_RECOVERY_STABLE_UP_TICKS       2u
#define PTP_LINK_RECOVERY_PATH_TIMEOUT_TICKS    4u
#define PTP_LINK_RECOVERY_PHY_TIMEOUT_TICKS     8u

typedef enum {
    PTP_LINK_RECOVERY_MONITOR = 0,
    PTP_LINK_RECOVERY_WAIT_STABLE_UP,
    PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS,
    PTP_LINK_RECOVERY_WAIT_PHY_PROGRESS,
    PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS,
    PTP_LINK_RECOVERY_EXHAUSTED
} ptp_link_recovery_state_t;

typedef enum {
    PTP_LINK_RECOVERY_ACTION_NONE = 0,
    PTP_LINK_RECOVERY_ACTION_RESET_PATH = 1,
    PTP_LINK_RECOVERY_ACTION_RESTART_AN = 2,
    PTP_LINK_RECOVERY_ACTION_RESET_PHY = 3
} ptp_link_recovery_action_t;

typedef struct {
    ptp_link_recovery_state_t state;
    uint32_t stable_up_ticks;
    uint32_t wait_ticks;
    uint32_t baseline_rx_accept_count;
    uint32_t baseline_servo_cycle_count;
    uint32_t recovery_count;
} ptp_link_recovery_ctx_t;

void ptp_link_recovery_init(ptp_link_recovery_ctx_t *ctx);

ptp_link_recovery_action_t ptp_link_recovery_on_health_tick(
    ptp_link_recovery_ctx_t *ctx,
    uint32_t link_up,
    uint32_t rx_accept_count,
    uint32_t servo_cycle_count);

const char *ptp_link_recovery_state_name(ptp_link_recovery_state_t state);

#endif
