#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "ptp_time_quality.h"

static void consume_event(ptp_time_quality_ctx_t *ctx,
                          ptp_time_quality_state_t from,
                          ptp_time_quality_state_t to,
                          uint32_t actions)
{
    ptp_time_quality_event_t event;

    assert(ptp_time_quality_take_event(ctx, &event) == 1u);
    assert(event.from == from);
    assert(event.to == to);
    assert(event.actions == actions);
    assert(ptp_time_quality_take_event(ctx, &event) == 0u);
}

static void enter_locked(ptp_time_quality_ctx_t *ctx)
{
    assert(ptp_time_quality_observe_servo(ctx, 1u, 0u) == 1u);
    consume_event(ctx,
                  PTP_TIME_INVALID,
                  PTP_TIME_REACQUIRE,
                  PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS);
    assert(ptp_time_quality_observe_servo(ctx, 2u, 1u) == 1u);
    consume_event(ctx, PTP_TIME_REACQUIRE, PTP_TIME_LOCKED, 0u);
}

int main(void)
{
    ptp_time_quality_ctx_t ctx;
    ptp_time_quality_event_t event;

    ptp_time_quality_init(&ctx);
    assert(ctx.state == PTP_TIME_INVALID);
    assert(ctx.missed_health_ticks == 0u);
    assert(ctx.transition_count == 0u);
    assert(ctx.stream_continuity_allowed == 0u);
    assert(ptp_time_quality_take_event(&ctx, &event) == 0u);

    /* Servo cycle zero is the baseline, not an accepted exchange. */
    assert(ptp_time_quality_observe_servo(&ctx, 0u, 1u) == 0u);
    assert(ctx.state == PTP_TIME_INVALID);

    enter_locked(&ctx);
    assert(ctx.state == PTP_TIME_LOCKED);
    assert(ctx.stream_continuity_allowed == 1u);

    /* The exchange flag consumes one health tick before misses accumulate. */
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.missed_health_ticks == 0u);
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.missed_health_ticks == 1u);
    assert(ctx.state == PTP_TIME_LOCKED);
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.missed_health_ticks == PTP_TIME_HOLDOVER_AFTER_TICKS);
    assert(ctx.state == PTP_TIME_HOLDOVER);
    assert(ctx.stream_continuity_allowed == 1u);
    consume_event(&ctx,
                  PTP_TIME_LOCKED,
                  PTP_TIME_HOLDOVER,
                  PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS);

    while (ctx.missed_health_ticks < (PTP_TIME_INVALID_AFTER_TICKS - 1u)) {
        ptp_time_quality_on_health_tick(&ctx);
    }
    assert(ctx.state == PTP_TIME_HOLDOVER);
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.missed_health_ticks == PTP_TIME_INVALID_AFTER_TICKS);
    assert(ctx.state == PTP_TIME_INVALID);
    assert(ctx.stream_continuity_allowed == 0u);
    consume_event(&ctx,
                  PTP_TIME_HOLDOVER,
                  PTP_TIME_INVALID,
                  PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS |
                      PTP_TIME_QUALITY_ACTION_STOP_STREAM);

    /* Recovery never jumps directly from INVALID/HOLDOVER to LOCKED. */
    assert(ptp_time_quality_observe_servo(&ctx, 3u, 1u) == 1u);
    assert(ctx.state == PTP_TIME_REACQUIRE);
    assert(ctx.stream_continuity_allowed == 0u);
    consume_event(&ctx,
                  PTP_TIME_INVALID,
                  PTP_TIME_REACQUIRE,
                  PTP_TIME_QUALITY_ACTION_CLEAR_LOCKS);
    assert(ptp_time_quality_observe_servo(&ctx, 4u, 1u) == 1u);
    assert(ctx.state == PTP_TIME_LOCKED);

    /* A context that never locked cannot enter HOLDOVER. */
    ptp_time_quality_init(&ctx);
    assert(ptp_time_quality_observe_servo(&ctx, 9u, 0u) == 1u);
    assert(ctx.state == PTP_TIME_REACQUIRE);
    ptp_time_quality_on_health_tick(&ctx);
    for (uint32_t i = 0u; i < PTP_TIME_HOLDOVER_AFTER_TICKS; ++i) {
        ptp_time_quality_on_health_tick(&ctx);
    }
    assert(ctx.state == PTP_TIME_REACQUIRE);
    while (ctx.missed_health_ticks < PTP_TIME_INVALID_AFTER_TICKS) {
        ptp_time_quality_on_health_tick(&ctx);
    }
    assert(ctx.state == PTP_TIME_INVALID);

    /* Counters saturate instead of wrapping. */
    ptp_time_quality_init(&ctx);
    ctx.missed_health_ticks = UINT32_MAX;
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.missed_health_ticks == UINT32_MAX);
    ctx.transition_count = UINT32_MAX;
    assert(ptp_time_quality_observe_servo(&ctx, 1u, 0u) == 1u);
    assert(ctx.transition_count == UINT32_MAX);

    assert(ptp_time_quality_name(PTP_TIME_INVALID)[0] == 'I');
    assert(ptp_time_quality_name(PTP_TIME_REACQUIRE)[0] == 'R');
    assert(ptp_time_quality_name(PTP_TIME_LOCKED)[0] == 'L');
    assert(ptp_time_quality_name(PTP_TIME_HOLDOVER)[0] == 'H');
    assert(ptp_time_quality_name((ptp_time_quality_state_t)99)[0] == 'U');

    /* Public calls are intentionally NULL-safe for the runtime fault path. */
    ptp_time_quality_init(NULL);
    assert(ptp_time_quality_observe_servo(NULL, 1u, 1u) == 0u);
    ptp_time_quality_on_health_tick(NULL);
    assert(ptp_time_quality_take_event(NULL, &event) == 0u);
    assert(ptp_time_quality_take_event(&ctx, NULL) == 1u);

    return 0;
}
