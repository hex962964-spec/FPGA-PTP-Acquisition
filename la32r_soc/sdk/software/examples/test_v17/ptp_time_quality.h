#ifndef PTP_TIME_QUALITY_H
#define PTP_TIME_QUALITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PTP_TIME_HOLDOVER_AFTER_TICKS 2u
#define PTP_TIME_INVALID_AFTER_TICKS  8u

#define PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS (1u << 0)
#define PTP_TIME_QUALITY_ACTION_STOP_STREAM (1u << 1)

typedef enum {
    PTP_TIME_INVALID = 0,
    PTP_TIME_REACQUIRE = 1,
    PTP_TIME_LOCKED = 2,
    PTP_TIME_HOLDOVER = 3
} ptp_time_quality_state_t;

typedef struct {
    ptp_time_quality_state_t from;
    ptp_time_quality_state_t to;
    uint32_t actions;
} ptp_time_quality_event_t;

typedef struct {
    ptp_time_quality_state_t state;
    ptp_time_quality_state_t transition_from;
    ptp_time_quality_state_t transition_to;
    uint32_t missed_health_ticks;
    uint32_t transition_count;
    uint32_t last_servo_cycle_count;
    uint32_t exchange_seen_since_tick;
    uint32_t transition_pending;
    uint32_t transition_actions;
    uint32_t stream_continuity_allowed;
} ptp_time_quality_ctx_t;

void ptp_time_quality_init(ptp_time_quality_ctx_t *ctx);
uint32_t ptp_time_quality_observe_servo(ptp_time_quality_ctx_t *ctx,
                                        uint32_t servo_cycle_count,
                                        uint32_t servo_locked);
void ptp_time_quality_on_link_down(ptp_time_quality_ctx_t *ctx);
void ptp_time_quality_on_health_tick(ptp_time_quality_ctx_t *ctx);
uint32_t ptp_time_quality_take_event(ptp_time_quality_ctx_t *ctx,
                                     ptp_time_quality_event_t *event);
uint32_t ptp_time_quality_stream_may_continue(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up);
uint32_t ptp_time_quality_stream_start_allowed(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up);
uint32_t ptp_time_quality_should_suspend_stream_on_link_down(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up,
    uint32_t stream_started);
const char *ptp_time_quality_name(ptp_time_quality_state_t state);

#ifdef __cplusplus
}
#endif

#endif
