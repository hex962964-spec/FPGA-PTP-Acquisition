#include <assert.h>
#include <stdint.h>

#define main net_acq_main_hidden
#include "main.c"
#undef main

int main(void)
{
    ptp_slave_ctx_t ctx;
    int64_t filtered_delay_ns;

    ptp_ctx_init(&ctx);

    assert(ptp_check_exchange_quality(&ctx, 100000ll, &filtered_delay_ns) == PTP_SERVO_QUAL_OK);
    assert(filtered_delay_ns == 100000ll);

    ctx.filtered_delay_ns = 100000ll;
    ctx.have_filtered_delay = 1u;

    assert(ptp_check_exchange_quality(&ctx, 105000ll, &filtered_delay_ns) == PTP_SERVO_QUAL_OK);
    assert(filtered_delay_ns >= 100000ll);

    assert(ptp_check_exchange_quality(&ctx, -1ll, &filtered_delay_ns) == PTP_SERVO_QUAL_DELAY_RANGE);
    assert(ptp_check_exchange_quality(&ctx, 400000ll, &filtered_delay_ns) == PTP_SERVO_QUAL_DELAY_OUTLIER);

    return 0;
}
