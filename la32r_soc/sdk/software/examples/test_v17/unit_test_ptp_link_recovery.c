#include <assert.h>
#include <stdint.h>

#include "ptp_link_recovery.h"

static void tick_n(ptp_link_recovery_ctx_t *ctx,
                   uint32_t count,
                   uint32_t link_up,
                   uint32_t rx_count,
                   uint32_t servo_count,
                   ptp_link_recovery_action_t expected_last)
{
    uint32_t i;
    ptp_link_recovery_action_t action = PTP_LINK_RECOVERY_ACTION_NONE;

    for (i = 0u; i < count; ++i) {
        action = ptp_link_recovery_on_health_tick(
            ctx, link_up, rx_count, servo_count);
    }
    assert(action == expected_last);
}

int main(void)
{
    ptp_link_recovery_ctx_t ctx;
    ptp_link_recovery_action_t action;

    ptp_link_recovery_init(&ctx);
    tick_n(&ctx, 20u, 1u, 100u, 10u, PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_MONITOR);

    action = ptp_link_recovery_on_health_tick(&ctx, 0u, 100u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_STABLE_UP);
    action = ptp_link_recovery_on_health_tick(&ctx, 1u, 100u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    action = ptp_link_recovery_on_health_tick(&ctx, 0u, 100u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);

    action = ptp_link_recovery_on_health_tick(&ctx, 1u, 100u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    action = ptp_link_recovery_on_health_tick(&ctx, 1u, 100u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_RESET_PATH);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS);

    action = ptp_link_recovery_on_health_tick(&ctx, 1u, 0u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS);
    action = ptp_link_recovery_on_health_tick(&ctx, 1u, 1u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_MONITOR);
    assert(ctx.recovery_count == 1u);

    ptp_link_recovery_on_health_tick(&ctx, 0u, 101u, 10u);
    tick_n(&ctx,
           PTP_LINK_RECOVERY_STABLE_UP_TICKS,
           1u,
           101u,
           10u,
           PTP_LINK_RECOVERY_ACTION_RESET_PATH);
    tick_n(&ctx,
           PTP_LINK_RECOVERY_PATH_TIMEOUT_TICKS,
           1u,
           101u,
           10u,
           PTP_LINK_RECOVERY_ACTION_RESTART_AN);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_PHY_PROGRESS);

    tick_n(&ctx,
           PTP_LINK_RECOVERY_PHY_TIMEOUT_TICKS,
           1u,
           101u,
           10u,
           PTP_LINK_RECOVERY_ACTION_RESET_PHY);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS);
    tick_n(&ctx, 20u, 0u, 0u, 10u, PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS);

    tick_n(&ctx,
           PTP_LINK_RECOVERY_PHY_TIMEOUT_TICKS,
           1u,
           0u,
           10u,
           PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_EXHAUSTED);

    action = ptp_link_recovery_on_health_tick(&ctx, 0u, 101u, 10u);
    assert(action == PTP_LINK_RECOVERY_ACTION_NONE);
    assert(ctx.state == PTP_LINK_RECOVERY_WAIT_STABLE_UP);

    return 0;
}
