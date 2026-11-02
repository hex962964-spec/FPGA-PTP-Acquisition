#include "ptp_link_recovery.h"

#include <stddef.h>

static uint32_t has_progress(const ptp_link_recovery_ctx_t *ctx,
                             uint32_t rx_accept_count,
                             uint32_t servo_cycle_count)
{
    return ((rx_accept_count > ctx->baseline_rx_accept_count) ||
            (servo_cycle_count > ctx->baseline_servo_cycle_count)) ? 1u : 0u;
}

static void arm_recovery(ptp_link_recovery_ctx_t *ctx,
                         uint32_t rx_accept_count,
                         uint32_t servo_cycle_count)
{
    ctx->state = PTP_LINK_RECOVERY_WAIT_STABLE_UP;
    ctx->stable_up_ticks = 0u;
    ctx->wait_ticks = 0u;
    ctx->baseline_rx_accept_count = rx_accept_count;
    ctx->baseline_servo_cycle_count = servo_cycle_count;
}

static void complete_recovery(ptp_link_recovery_ctx_t *ctx)
{
    ctx->state = PTP_LINK_RECOVERY_MONITOR;
    ctx->stable_up_ticks = 0u;
    ctx->wait_ticks = 0u;
    if (ctx->recovery_count != UINT32_MAX) {
        ctx->recovery_count++;
    }
}

void ptp_link_recovery_init(ptp_link_recovery_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->state = PTP_LINK_RECOVERY_MONITOR;
    ctx->stable_up_ticks = 0u;
    ctx->wait_ticks = 0u;
    ctx->baseline_rx_accept_count = 0u;
    ctx->baseline_servo_cycle_count = 0u;
    ctx->recovery_count = 0u;
}

ptp_link_recovery_action_t ptp_link_recovery_on_health_tick(
    ptp_link_recovery_ctx_t *ctx,
    uint32_t link_up,
    uint32_t rx_accept_count,
    uint32_t servo_cycle_count)
{
    if (ctx == NULL) {
        return PTP_LINK_RECOVERY_ACTION_NONE;
    }

    switch (ctx->state) {
    case PTP_LINK_RECOVERY_MONITOR:
        if (link_up == 0u) {
            arm_recovery(ctx, rx_accept_count, servo_cycle_count);
        }
        break;
    case PTP_LINK_RECOVERY_WAIT_STABLE_UP:
        if (link_up == 0u) {
            ctx->stable_up_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            break;
        }
        if (ctx->stable_up_ticks != UINT32_MAX) {
            ctx->stable_up_ticks++;
        }
        if (ctx->stable_up_ticks >= PTP_LINK_RECOVERY_STABLE_UP_TICKS) {
            ctx->state = PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS;
            ctx->wait_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            return PTP_LINK_RECOVERY_ACTION_RESET_PATH;
        }
        break;
    case PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS:
        if (link_up == 0u) {
            arm_recovery(ctx, rx_accept_count, servo_cycle_count);
            break;
        }
        if (rx_accept_count < ctx->baseline_rx_accept_count) {
            ctx->baseline_rx_accept_count = rx_accept_count;
        }
        if (has_progress(ctx, rx_accept_count, servo_cycle_count)) {
            complete_recovery(ctx);
            break;
        }
        if (ctx->wait_ticks != UINT32_MAX) {
            ctx->wait_ticks++;
        }
        if (ctx->wait_ticks >= PTP_LINK_RECOVERY_PATH_TIMEOUT_TICKS) {
            ctx->state = PTP_LINK_RECOVERY_WAIT_PHY_PROGRESS;
            ctx->wait_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            return PTP_LINK_RECOVERY_ACTION_RESTART_AN;
        }
        break;
    case PTP_LINK_RECOVERY_WAIT_PHY_PROGRESS:
        if (link_up == 0u) {
            ctx->wait_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            break;
        }
        if (rx_accept_count < ctx->baseline_rx_accept_count) {
            ctx->baseline_rx_accept_count = rx_accept_count;
        }
        if (has_progress(ctx, rx_accept_count, servo_cycle_count)) {
            complete_recovery(ctx);
            break;
        }
        if (ctx->wait_ticks != UINT32_MAX) {
            ctx->wait_ticks++;
        }
        if (ctx->wait_ticks >= PTP_LINK_RECOVERY_PHY_TIMEOUT_TICKS) {
            ctx->state = PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS;
            ctx->wait_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            return PTP_LINK_RECOVERY_ACTION_RESET_PHY;
        }
        break;
    case PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS:
        if (link_up == 0u) {
            ctx->wait_ticks = 0u;
            ctx->baseline_rx_accept_count = rx_accept_count;
            ctx->baseline_servo_cycle_count = servo_cycle_count;
            break;
        }
        if (rx_accept_count < ctx->baseline_rx_accept_count) {
            ctx->baseline_rx_accept_count = rx_accept_count;
        }
        if (has_progress(ctx, rx_accept_count, servo_cycle_count)) {
            complete_recovery(ctx);
            break;
        }
        if (ctx->wait_ticks != UINT32_MAX) {
            ctx->wait_ticks++;
        }
        if (ctx->wait_ticks >= PTP_LINK_RECOVERY_PHY_TIMEOUT_TICKS) {
            ctx->state = PTP_LINK_RECOVERY_EXHAUSTED;
            ctx->wait_ticks = 0u;
        }
        break;
    case PTP_LINK_RECOVERY_EXHAUSTED:
        if (link_up == 0u) {
            arm_recovery(ctx, rx_accept_count, servo_cycle_count);
        }
        break;
    default:
        ptp_link_recovery_init(ctx);
        break;
    }
    return PTP_LINK_RECOVERY_ACTION_NONE;
}

const char *ptp_link_recovery_state_name(ptp_link_recovery_state_t state)
{
    switch (state) {
    case PTP_LINK_RECOVERY_MONITOR: return "MONITOR";
    case PTP_LINK_RECOVERY_WAIT_STABLE_UP: return "WAIT_STABLE_UP";
    case PTP_LINK_RECOVERY_WAIT_PATH_PROGRESS: return "WAIT_PATH_PROGRESS";
    case PTP_LINK_RECOVERY_WAIT_PHY_PROGRESS: return "WAIT_PHY_PROGRESS";
    case PTP_LINK_RECOVERY_WAIT_FINAL_PROGRESS: return "WAIT_FINAL_PROGRESS";
    case PTP_LINK_RECOVERY_EXHAUSTED: return "EXHAUSTED";
    default: return "UNKNOWN";
    }
}
