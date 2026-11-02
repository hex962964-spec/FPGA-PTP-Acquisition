#include <assert.h>
#include <stdint.h>
#include <string.h>

#define main net_acq_main_hidden
#include "main.c"
#undef main

static net_acq_eth_snapshot_t g_fake_eth_snapshot;

void net_acq_delay_cycles(volatile uint32_t cycles) { (void)cycles; }
void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *cfg) { (void)cfg; }
void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *cfg,
                            uint32_t txstamp_pop_pulse,
                            uint32_t tod_set_pulse,
                            uint32_t tod_step_pulse,
                            uint32_t tod_freq_pulse,
                            uint32_t tx_trigger_pulse)
{
    (void)cfg; (void)txstamp_pop_pulse; (void)tod_set_pulse;
    (void)tod_step_pulse; (void)tod_freq_pulse; (void)tx_trigger_pulse;
}
uint64_t net_acq_ptp_current_time(void) { return 0ull; }
void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *snap) { if (snap) { *snap = g_fake_eth_snapshot; } }

int main(void)
{
    ptp_slave_ctx_t ctx;

    memset(&g_fake_eth_snapshot, 0, sizeof(g_fake_eth_snapshot));
    ptp_ctx_init(&ctx);
    ctx.have_filtered_drift = 1u;
    ctx.filtered_drift_ppb = 0;

    for (uint32_t i = 0; i < PTP_LOCK_STREAK_MIN; ++i) {
        ptp_update_lock_flags(&ctx, PTP_LOCK_THRESHOLD_NS - 1000ll, 1u, 0u);
    }
    assert(ctx.locked == 1u);
    assert(ctx.fine_locked == 0u);
    assert(ctx.startup_done == 1u);

    ctx.filtered_drift_ppb = 50;
    for (uint32_t i = 0; i < PTP_FINE_LOCK_STREAK_MIN; ++i) {
        ptp_update_lock_flags(&ctx, PTP_FINE_LOCK_THRESHOLD_NS - 100ll, 1u, 0u);
    }
    assert(ctx.locked == 1u);
    assert(ctx.fine_locked == 1u);

    ptp_update_lock_flags(&ctx, PTP_FINE_LOCK_THRESHOLD_NS + 5000ll, 1u, 1u);
    assert(ctx.locked == 0u);
    assert(ctx.fine_locked == 0u);

    return 0;
}
