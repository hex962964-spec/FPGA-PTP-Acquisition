#include <assert.h>
#include <stdint.h>

#include "net_acq_hal.h"

static uint32_t g_write_count;
static uint32_t g_last_addr;
static uint32_t g_last_value;

void net_acq_test_reg_write(uint32_t addr, uint32_t value)
{
    g_write_count++;
    g_last_addr = addr;
    g_last_value = value;
}

int main(void)
{
    g_write_count = 0u;
    g_last_addr = 0u;
    g_last_value = 0u;

    assert(NET_ACQ_TIME_QUALITY == 0x178u);
    net_acq_set_time_quality(0xFFFFFFFFu);
    assert(g_write_count == 1u);
    assert(g_last_addr == NET_ACQ_BASE + NET_ACQ_TIME_QUALITY);
    assert(g_last_value == 3u);

    net_acq_set_time_quality(0u);
    assert(g_write_count == 2u);
    assert(g_last_value == 0u);

    return 0;
}
