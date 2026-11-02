#include <assert.h>
#include <stdint.h>

#define main net_acq_main_hidden
#include "main.c"
#undef main

static uint32_t g_disable_count;
static uint32_t g_quality_write_count;
static uint32_t g_last_quality;
static uint32_t g_call_count;
static char g_call_order[64];
static uint32_t g_check_invalid_order;
static ptp_slave_ctx_t *g_servo_under_test;

void net_acq_disable(uint32_t use_external_source)
{
    (void)use_external_source;
    assert(g_call_count < sizeof(g_call_order));
    g_call_order[g_call_count++] = 'D';
    g_disable_count++;
}

void net_acq_set_time_quality(uint32_t state)
{
    assert(g_call_count < sizeof(g_call_order));
    g_call_order[g_call_count++] = 'Q';
    if (g_check_invalid_order && state == PTP_TIME_INVALID) {
        /* Publishing INVALID while acquisition remains enabled would let the
         * V2 packet checker latch ERR_TIME_INVALID and restart the runtime. */
        assert(g_disable_count == 1u);
        assert(g_servo_under_test->stream_started == 0u);
    }
    g_quality_write_count++;
    g_last_quality = state;
}

static void apply_pending(ptp_slave_ctx_t *servo,
                          ptp_time_quality_ctx_t *quality)
{
    assert(ptp_apply_time_quality_event(servo, quality) == 1u);
}

/* Populate a half-finished exchange and a previous phase epoch. A lost
 * reference must discard these even while oscillator compensation is held. */
static void seed_old_exchange_history(ptp_slave_ctx_t *servo)
{
    servo->state = PTP_WAIT_DELAY_RESP;
    servo->sync_seq_id = 123u;
    servo->delay_req_seq_id = 456u;
    servo->next_delay_req_seq_id = 457u;
    servo->t1_master_ns = 1000000000ull;
    servo->t2_slave_rx_ns = 1000002136ull;
    servo->t3_slave_tx_ns = 1012800000ull;
    servo->t4_master_rx_ns = 1012802136ull;
    servo->follow_up_timeout_polls = 7u;
    servo->delay_resp_timeout_polls = 9u;
    servo->prev_t1_master_ns = 800000000ull;
    servo->have_prev_t1 = 1u;
    servo->have_filtered_offset = 1u;
    servo->filtered_offset_ns = 51ll;
    servo->have_filtered_drift = 1u;
    servo->filtered_drift_ppb = 13;
    servo->have_filtered_delay = 1u;
    servo->filtered_delay_ns = 2136ll;
    servo->fine_bias_remainder = 7ll;
    ptp_delay_guard_init(&servo->delay_guard);
    for (uint32_t i = 0u; i < PTP_DELAY_GUARD_WINDOW; ++i) {
        ptp_delay_guard_accept(&servo->delay_guard, 2136ll);
    }
    assert(servo->delay_guard.count == PTP_DELAY_GUARD_WINDOW);
}

static void expect_old_exchange_discarded(const ptp_slave_ctx_t *servo)
{
    assert(servo->state == PTP_WAIT_SYNC);
    assert(servo->sync_seq_id == 0u && servo->delay_req_seq_id == 0u);
    assert(servo->t1_master_ns == 0ull && servo->t2_slave_rx_ns == 0ull);
    assert(servo->t3_slave_tx_ns == 0ull && servo->t4_master_rx_ns == 0ull);
    assert(servo->follow_up_timeout_polls == 0u);
    assert(servo->delay_resp_timeout_polls == 0u);
    assert(servo->have_prev_t1 == 0u && servo->prev_t1_master_ns == 0ull);
    assert(servo->have_filtered_offset == 0u);
    assert(servo->have_filtered_drift == 0u && servo->filtered_drift_ppb == 0);
    assert(servo->have_filtered_delay == 0u);
    assert(servo->fine_bias_remainder == 0ll);
    assert(servo->delay_guard.count == 0u && servo->delay_guard.candidate_count == 0u);
    /* Keep request sequence advancing, avoiding immediate old-response reuse. */
    assert(servo->next_delay_req_seq_id == 457u);
    assert(servo->freq_adj_ppb == 321 && servo->freq_bias_ppb == 654);
}

int main(void)
{
    ptp_slave_ctx_t servo;
    ptp_time_quality_ctx_t quality;

    g_disable_count = 0u;
    g_quality_write_count = 0u;
    g_last_quality = 99u;
    g_call_count = 0u;
    g_check_invalid_order = 0u;
    g_servo_under_test = &servo;
    ptp_ctx_init(&servo);
    servo.freq_adj_ppb = 321;
    servo.freq_bias_ppb = 654;

    ptp_time_quality_runtime_init(&quality);
    assert(quality.state == PTP_TIME_INVALID);
    assert(g_quality_write_count == 1u);
    assert(g_last_quality == PTP_TIME_INVALID);

    servo.locked = 1u;
    servo.lock_streak = 9u;
    servo.fine_locked = 1u;
    servo.fine_lock_streak = 17u;
    assert(ptp_time_quality_observe_servo(&quality, 1u, 0u) == 1u);
    apply_pending(&servo, &quality);
    assert(servo.locked == 0u);
    assert(servo.lock_streak == 0u);
    assert(servo.fine_locked == 0u);
    assert(servo.fine_lock_streak == 0u);
    assert(servo.freq_adj_ppb == 321);
    assert(servo.stream_started == 0u);
    assert(g_last_quality == PTP_TIME_REACQUIRE);

    assert(ptp_time_quality_observe_servo(&quality, 2u, 1u) == 1u);
    apply_pending(&servo, &quality);
    assert(g_last_quality == PTP_TIME_LOCKED);
    servo.locked = 1u;
    servo.lock_streak = 10u;
    servo.fine_locked = 1u;
    servo.fine_lock_streak = 20u;
    servo.stream_started = 1u;
    seed_old_exchange_history(&servo);

    ptp_time_quality_on_health_tick(&quality);
    ptp_time_quality_on_health_tick(&quality);
    ptp_time_quality_on_health_tick(&quality);
    apply_pending(&servo, &quality);
    assert(quality.state == PTP_TIME_HOLDOVER);
    assert(servo.locked == 0u);
    assert(servo.fine_locked == 0u);
    assert(servo.stream_started == 1u);
    assert(servo.freq_adj_ppb == 321);
    assert(g_disable_count == 0u);
    assert(g_last_quality == PTP_TIME_HOLDOVER);
    expect_old_exchange_discarded(&servo);

    /* The first fresh accepted exchange after HOLDOVER provides the start of
     * the new observer epoch. Entering REACQUIRE must not erase it again. */
    servo.have_prev_t1 = 1u;
    servo.prev_t1_master_ns = 2000000000ull;
    servo.have_filtered_offset = 1u;
    servo.filtered_offset_ns = 23ll;
    ptp_delay_guard_accept(&servo.delay_guard, 2140ll);
    assert(ptp_time_quality_observe_servo(&quality, 3u, 1u) == 1u);
    apply_pending(&servo, &quality);
    assert(quality.state == PTP_TIME_REACQUIRE);
    assert(servo.stream_started == 1u);
    assert(servo.have_prev_t1 == 1u && servo.prev_t1_master_ns == 2000000000ull);
    assert(servo.have_filtered_offset == 1u && servo.filtered_offset_ns == 23ll);
    assert(servo.delay_guard.count == 1u);
    assert(servo.freq_adj_ppb == 321 && servo.freq_bias_ppb == 654);
    /* Re-polling a completed cycle must not fabricate recovery progress. */
    assert(ptp_time_quality_observe_servo(&quality, 3u, 1u) == 0u);
    assert(ptp_apply_time_quality_event(&servo, &quality) == 0u);
    assert(servo.have_prev_t1 == 1u && servo.delay_guard.count == 1u);
    seed_old_exchange_history(&servo);

    ptp_time_quality_on_health_tick(&quality);
    for (uint32_t i = 0u; i < PTP_TIME_INVALID_AFTER_TICKS; ++i) {
        ptp_time_quality_on_health_tick(&quality);
    }
    uint32_t before_invalid_calls = g_call_count;
    g_check_invalid_order = 1u;
    apply_pending(&servo, &quality);
    g_check_invalid_order = 0u;
    assert(g_call_count == before_invalid_calls + 2u);
    assert(g_call_order[before_invalid_calls] == 'D');
    assert(g_call_order[before_invalid_calls + 1u] == 'Q');
    expect_old_exchange_discarded(&servo);
    assert(quality.state == PTP_TIME_INVALID);
    assert(servo.stream_started == 0u);
    assert(g_disable_count == 1u);
    assert(g_last_quality == PTP_TIME_INVALID);
    uint32_t after_invalid_calls = g_call_count;
    assert(ptp_apply_time_quality_event(&servo, &quality) == 0u);
    assert(g_call_count == after_invalid_calls);
    assert(g_disable_count == 1u);

    assert(ptp_time_quality_observe_servo(&quality, 4u, 1u) == 1u);
    apply_pending(&servo, &quality);
    assert(quality.state == PTP_TIME_REACQUIRE);
    assert(servo.stream_started == 0u);
    assert(ptp_time_quality_observe_servo(&quality, 5u, 1u) == 1u);
    apply_pending(&servo, &quality);
    assert(quality.state == PTP_TIME_LOCKED);
    assert(servo.stream_started == 0u);

    assert(g_quality_write_count == quality.transition_count + 1u);
    assert(ptp_apply_time_quality_event(NULL, &quality) == 0u);
    assert(ptp_apply_time_quality_event(&servo, NULL) == 0u);

    return 0;
}
