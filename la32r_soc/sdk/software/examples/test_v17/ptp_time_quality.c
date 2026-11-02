#include "ptp_time_quality.h"

#include <stddef.h>

static void ptp_time_quality_transition(ptp_time_quality_ctx_t *ctx,
                                        ptp_time_quality_state_t next)
{
    uint32_t actions;

    if ((ctx == NULL) || (ctx->state == next)) {
        return;
    }

    actions = (next == PTP_TIME_LOCKED)
                  ? 0u
                  : PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS;
    if (next == PTP_TIME_INVALID) {
        actions |= PTP_TIME_QUALITY_ACTION_STOP_STREAM;
        ctx->stream_continuity_allowed = 0u;
    } else if (next == PTP_TIME_LOCKED) {
        ctx->stream_continuity_allowed = 1u;
    }

    ctx->transition_from = ctx->state;
    ctx->transition_to = next;
    ctx->state = next;
    ctx->transition_actions = actions;
    ctx->transition_pending = 1u;
    if (ctx->transition_count != UINT32_MAX) {
        ctx->transition_count++;
    }
}

void ptp_time_quality_init(ptp_time_quality_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->state = PTP_TIME_INVALID;
    ctx->transition_from = PTP_TIME_INVALID;
    ctx->transition_to = PTP_TIME_INVALID;
    ctx->missed_health_ticks = 0u;
    ctx->transition_count = 0u;
    ctx->last_servo_cycle_count = 0u;
    ctx->exchange_seen_since_tick = 0u;
    ctx->transition_pending = 0u;
    ctx->transition_actions = 0u;
    ctx->stream_continuity_allowed = 0u;
}

uint32_t ptp_time_quality_observe_servo(ptp_time_quality_ctx_t *ctx,
                                        uint32_t servo_cycle_count,
                                        uint32_t servo_locked)
{
    ptp_time_quality_state_t state_before_exchange;

    if ((ctx == NULL) ||
        (servo_cycle_count == ctx->last_servo_cycle_count)) {
        return 0u;
    }

    ctx->last_servo_cycle_count = servo_cycle_count;
    ctx->exchange_seen_since_tick = 1u;
    ctx->missed_health_ticks = 0u;
    state_before_exchange = ctx->state;

    if ((state_before_exchange == PTP_TIME_INVALID) ||
        (state_before_exchange == PTP_TIME_HOLDOVER)) {
        ptp_time_quality_transition(ctx, PTP_TIME_REACQUIRE);
    } else if (servo_locked != 0u) {
        ptp_time_quality_transition(ctx, PTP_TIME_LOCKED);
    } else {
        ptp_time_quality_transition(ctx, PTP_TIME_REACQUIRE);
    }

    return 1u;
}

/* A physical down edge is known immediately; it is not a missed-packet guess. */
void ptp_time_quality_on_link_down(ptp_time_quality_ctx_t *ctx)
{
    if(ctx==NULL)return;
    ctx->exchange_seen_since_tick=0u;
    if(ctx->state==PTP_TIME_LOCKED)
        ptp_time_quality_transition(ctx,PTP_TIME_HOLDOVER);
}

void ptp_time_quality_on_health_tick(ptp_time_quality_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->exchange_seen_since_tick != 0u) {
        ctx->exchange_seen_since_tick = 0u;
        ctx->missed_health_ticks = 0u;
        return;
    }

    if (ctx->missed_health_ticks != UINT32_MAX) {
        ctx->missed_health_ticks++;
    }

    if ((ctx->state == PTP_TIME_LOCKED) &&
        (ctx->missed_health_ticks >= PTP_TIME_HOLDOVER_AFTER_TICKS)) {
        ptp_time_quality_transition(ctx, PTP_TIME_HOLDOVER);
    }

    if (((ctx->state == PTP_TIME_HOLDOVER) ||
         (ctx->state == PTP_TIME_REACQUIRE)) &&
        (ctx->missed_health_ticks >= PTP_TIME_INVALID_AFTER_TICKS)) {
        ptp_time_quality_transition(ctx, PTP_TIME_INVALID);
    }
}

uint32_t ptp_time_quality_take_event(ptp_time_quality_ctx_t *ctx,
                                     ptp_time_quality_event_t *event)
{
    if ((ctx == NULL) || (ctx->transition_pending == 0u)) {
        return 0u;
    }

    if (event != NULL) {
        event->from = ctx->transition_from;
        event->to = ctx->transition_to;
        event->actions = ctx->transition_actions;
    }
    ctx->transition_pending = 0u;
    ctx->transition_actions = 0u;
    return 1u;
}

uint32_t ptp_time_quality_stream_may_continue(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up)
{
    return ((ctx != NULL) && (link_up != 0u) &&
            (ctx->stream_continuity_allowed != 0u) &&
            (ctx->state != PTP_TIME_INVALID))
               ? 1u
               : 0u;
}

uint32_t ptp_time_quality_stream_start_allowed(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up)
{
    return ((ctx != NULL) && (link_up != 0u) &&
            (ctx->state == PTP_TIME_LOCKED))
               ? 1u
               : 0u;
}

uint32_t ptp_time_quality_should_suspend_stream_on_link_down(
    const ptp_time_quality_ctx_t *ctx,
    uint32_t link_up,
    uint32_t stream_started)
{
    return ((ctx != NULL) && (link_up == 0u) && (stream_started != 0u))
               ? 1u
               : 0u;
}

const char *ptp_time_quality_name(ptp_time_quality_state_t state)
{
    switch (state) {
    case PTP_TIME_INVALID:
        return "INVALID";
    case PTP_TIME_REACQUIRE:
        return "REACQUIRE";
    case PTP_TIME_LOCKED:
        return "LOCKED";
    case PTP_TIME_HOLDOVER:
        return "HOLDOVER";
    default:
        return "UNKNOWN";
    }
}
