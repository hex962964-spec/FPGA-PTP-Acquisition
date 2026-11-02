#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#define main net_acq_main_hidden
#include "main.c"
#undef main

static int32_t g_last_freq_ppb;
static int32_t g_last_step_ns;
static uint64_t g_last_set_ns;
static uint32_t g_freq_pulse_count;
static uint32_t g_step_pulse_count;
static uint32_t g_set_pulse_count;
static net_acq_eth_snapshot_t g_fake_eth_snapshot;

static void reset_stub_state(void)
{
    g_last_freq_ppb = INT32_MIN;
    g_last_step_ns = INT32_MIN;
    g_last_set_ns = 0ull;
    g_freq_pulse_count = 0u;
    g_step_pulse_count = 0u;
    g_set_pulse_count = 0u;
    memset(&g_fake_eth_snapshot, 0, sizeof(g_fake_eth_snapshot));
}

void net_acq_delay_cycles(volatile uint32_t cycles)
{
    (void)cycles;
}

void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *cfg)
{
    (void)cfg;
}

void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *cfg,
                            uint32_t txstamp_pop_pulse,
                            uint32_t tod_set_pulse,
                            uint32_t tod_step_pulse,
                            uint32_t tod_freq_pulse,
                            uint32_t tx_trigger_pulse)
{
    (void)txstamp_pop_pulse;
    (void)tx_trigger_pulse;

    if (tod_freq_pulse) {
        g_freq_pulse_count++;
        g_last_freq_ppb = cfg ? cfg->tod_freq_adj_ppb : INT32_MIN;
    }
    if (tod_step_pulse) {
        g_step_pulse_count++;
        g_last_step_ns = cfg ? cfg->tod_step_ns : INT32_MIN;
    }
    if (tod_set_pulse) {
        g_set_pulse_count++;
        g_last_set_ns = cfg ? cfg->tod_set_ns : 0ull;
    }
}

uint64_t net_acq_ptp_current_time(void)
{
    return 2000000000ull;
}

void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *snap)
{
    if (snap) {
        *snap = g_fake_eth_snapshot;
    }
}

static void prepare_trim_case(ptp_slave_ctx_t *ctx)
{
    ptp_ctx_init(ctx);
    ctx->t1_master_ns = 0ull;
    ctx->t2_slave_rx_ns = 320000ull;
    ctx->t3_slave_tx_ns = 300000ull;
    ctx->t4_master_rx_ns = 400000ull;
}

static void prepare_full_step_case(ptp_slave_ctx_t *ctx)
{
    ptp_ctx_init(ctx);
    ctx->t1_master_ns = 0ull;
    ctx->t2_slave_rx_ns = 1000000ull;
    ctx->t3_slave_tx_ns = 400000ull;
    ctx->t4_master_rx_ns = 400000ull;
}

static void prepare_freeze_case(ptp_slave_ctx_t *ctx)
{
    ptp_ctx_init(ctx);
    ctx->startup_done = 1u;
    ctx->have_prev_t1 = 1u;
    ctx->prev_t1_master_ns = 0ull;
    ctx->have_filtered_offset = 1u;
    ctx->filtered_offset_ns = 1000ll;
    ctx->freq_adj_ppb = 2000;
    ctx->freeze_servo_cycles = 1u;
    ctx->t1_master_ns = 100000000ull;
    ctx->t2_slave_rx_ns = 100100000ull;
    ctx->t3_slave_tx_ns = 100200000ull;
    ctx->t4_master_rx_ns = 100296000ull;
}

int main(void)
{
    ptp_slave_ctx_t ctx;
    net_acq_ptp_test_cfg_t ptp_cfg;
    int64_t num, den;

    ptp_cfg = build_ptp_cfg();

    /* 100us 级偏差不再做 full STEP，而是先用阻尼 trim 拉近。 */
    reset_stub_state();
    prepare_trim_case(&ctx);
    ctx.freq_adj_ppb = 1234;
    ctx.freq_bias_ppb = 567;
    ctx.filtered_drift_ppb = 89;
    ctx.have_filtered_drift = 1u;
    ptp_handle_servo_result(&ctx, &ptp_cfg);
    assert(g_step_pulse_count == 1u);
    assert(g_last_step_ns == -82500);
    assert(g_freq_pulse_count == 0u);
    assert(ctx.freq_adj_ppb == 1234);
    assert(ctx.freq_bias_ppb == 567);
    assert(ctx.filtered_drift_ppb == 0);
    assert(ctx.have_filtered_drift == 0u);
    assert(ctx.startup_done == 1u);

    /* 更大的启动偏差仍然需要 full STEP，并且先清空旧频率状态。 */
    reset_stub_state();
    prepare_full_step_case(&ctx);
    ctx.freq_adj_ppb = 1234;
    ctx.freq_bias_ppb = 567;
    ctx.filtered_drift_ppb = 89;
    ctx.have_filtered_drift = 1u;
    ptp_handle_servo_result(&ctx, &ptp_cfg);
    assert(g_step_pulse_count == 1u);
    assert(g_last_step_ns == -500000);
    assert(g_freq_pulse_count == 1u);
    assert(g_last_freq_ppb == 0);
    assert(ctx.freq_adj_ppb == 0);
    assert(ctx.freq_bias_ppb == 0);
    assert(ctx.filtered_drift_ppb == 0);
    assert(ctx.have_filtered_drift == 0u);

    /* FREEZE 模式应保持当前频率，不应把输出频率塌回到 bias-only。 */
    reset_stub_state();
    prepare_freeze_case(&ctx);
    ptp_handle_servo_result(&ctx, &ptp_cfg);
    assert(g_step_pulse_count == 0u);
    assert(g_set_pulse_count == 0u);
    assert(g_freq_pulse_count == 0u);
    assert(ctx.freq_adj_ppb == 2000);
    assert(ctx.freeze_servo_cycles == 0u);

    /* offset LPF 采用三段：acq -> ready -> fine hold。 */
    ptp_ctx_init(&ctx);
    ptp_select_offset_filter_coeffs(&ctx, &num, &den);
    assert(num == PTP_ACQ_OFFSET_FILTER_NUM);
    assert(den == PTP_ACQ_OFFSET_FILTER_DEN);
    ctx.locked = 1u;
    ptp_select_offset_filter_coeffs(&ctx, &num, &den);
    assert(num == PTP_READY_OFFSET_FILTER_NUM);
    assert(den == PTP_READY_OFFSET_FILTER_DEN);
    ctx.fine_locked = 1u;
    ptp_select_offset_filter_coeffs(&ctx, &num, &den);
    assert(num == PTP_OFFSET_FILTER_NUM);
    assert(den == PTP_OFFSET_FILTER_DEN);

    /* 频率输出 LPF 也按阶段切换：track/fine_acq 直达，fine_hold 再回到稳态低通。 */
    ptp_ctx_init(&ctx);
    ptp_select_freq_filter_coeffs(&ctx, 0u, 0u, &num, &den);
    assert(num == PTP_FREQ_FILTER_NUM);
    assert(den == PTP_FREQ_FILTER_DEN);
    ctx.locked = 1u;
    ptp_select_freq_filter_coeffs(&ctx, 0u, 1u, &num, &den);
    assert(num == PTP_FREQ_FILTER_TRACK_NUM);
    assert(den == PTP_FREQ_FILTER_TRACK_DEN);
    ptp_select_freq_filter_coeffs(&ctx, 1u, 0u, &num, &den);
    assert(num == PTP_FREQ_FILTER_FINE_ACQ_NUM);
    assert(den == PTP_FREQ_FILTER_FINE_ACQ_DEN);
    ctx.fine_locked = 1u;
    ptp_select_freq_filter_coeffs(&ctx, 1u, 0u, &num, &den);
    assert(num == PTP_FREQ_FILTER_FINE_NUM);
    assert(den == PTP_FREQ_FILTER_FINE_DEN);

    /* trim 计算应带阻尼并受最大幅度限制。 */
    assert(ptp_compute_trim_step_ns(10000ll) == -7500);
    assert(ptp_compute_trim_step_ns(-10000ll) == 7500);
    assert(ptp_compute_trim_step_ns(1000000ll) == -(int32_t)PTP_TRIM_MAX_NS);

    return 0;
}
