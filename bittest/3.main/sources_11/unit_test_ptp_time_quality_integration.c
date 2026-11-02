#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "ptp_time_quality.h"

static void enter_locked(ptp_time_quality_ctx_t *ctx)
{
    ptp_time_quality_event_t event;

    assert(ptp_time_quality_observe_servo(ctx, 10u, 0u) == 1u);
    assert(ptp_time_quality_take_event(ctx, &event) == 1u);
    assert(ptp_time_quality_observe_servo(ctx, 11u, 1u) == 1u);
    assert(ptp_time_quality_take_event(ctx, &event) == 1u);
    assert(ctx->state == PTP_TIME_LOCKED);
}

int main(void)
{
    ptp_time_quality_ctx_t ctx;
    ptp_time_quality_event_t event;

    ptp_time_quality_init(&ctx);
    enter_locked(&ctx);

    /* Polling one completed cycle cannot fabricate Observer activity. */
    assert(ptp_time_quality_observe_servo(&ctx, 11u, 1u) == 0u);
    ptp_time_quality_on_health_tick(&ctx);
    ptp_time_quality_on_health_tick(&ctx);
    ptp_time_quality_on_health_tick(&ctx);
    assert(ctx.state == PTP_TIME_HOLDOVER);
    assert(ptp_time_quality_stream_may_continue(&ctx, 1u) == 1u);
    assert(ptp_time_quality_stream_may_continue(&ctx, 0u) == 0u);
    assert(ptp_time_quality_should_suspend_stream_on_link_down(&ctx, 0u, 1u) == 1u);
    assert(ptp_time_quality_should_suspend_stream_on_link_down(&ctx, 1u, 1u) == 0u);
    assert(ptp_time_quality_take_event(&ctx, &event) == 1u);

    /* HOLDOVER recovery keeps an already-running stream through REACQUIRE. */
    assert(ptp_time_quality_observe_servo(&ctx, 12u, 1u) == 1u);
    assert(ctx.state == PTP_TIME_REACQUIRE);
    assert(ptp_time_quality_stream_may_continue(&ctx, 1u) == 1u);
    assert(ptp_time_quality_stream_start_allowed(&ctx, 1u) == 0u);

    /* Loss after recovery reaches INVALID even though the Ethernet link is up. */
    ptp_time_quality_on_health_tick(&ctx);
    for (uint32_t i = 0u; i < PTP_TIME_INVALID_AFTER_TICKS; ++i) {
        ptp_time_quality_on_health_tick(&ctx);
    }
    assert(ctx.state == PTP_TIME_INVALID);
    assert(ptp_time_quality_stream_may_continue(&ctx, 1u) == 0u);
    assert(ptp_time_quality_stream_start_allowed(&ctx, 1u) == 0u);
    assert(ptp_time_quality_take_event(&ctx, &event) == 1u);
    assert((event.actions & PTP_TIME_QUALITY_ACTION_STOP_STREAM) != 0u);
    assert(ptp_time_quality_take_event(&ctx, &event) == 0u);

    /* INVALID recovery cannot restart until a later cycle reports LOCKED. */
    assert(ptp_time_quality_observe_servo(&ctx, 13u, 1u) == 1u);
    assert(ctx.state == PTP_TIME_REACQUIRE);
    assert(ptp_time_quality_stream_may_continue(&ctx, 1u) == 0u);
    assert(ptp_time_quality_observe_servo(&ctx, 14u, 1u) == 1u);
    assert(ctx.state == PTP_TIME_LOCKED);
    assert(ptp_time_quality_stream_start_allowed(&ctx, 1u) == 1u);
    assert(ptp_time_quality_stream_start_allowed(&ctx, 0u) == 0u);

    assert(ptp_time_quality_stream_may_continue(NULL, 1u) == 0u);
    assert(ptp_time_quality_stream_start_allowed(NULL, 1u) == 0u);
    assert(ptp_time_quality_should_suspend_stream_on_link_down(NULL, 0u, 1u) == 0u);

    return 0;
}
