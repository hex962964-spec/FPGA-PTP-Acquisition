#include <assert.h>
#include <stdint.h>

#define main net_acq_main_hidden
#include "main.c"
#undef main

int main(void)
{
    net_acq_txsched_cfg_t cfg;

    cfg = build_txsched_cfg();
    assert(net_acq_txsched_validate_config(&cfg) == 0);

    cfg.period_ns = 0u;
    assert(net_acq_txsched_validate_config(&cfg) == -210);

    cfg = build_txsched_cfg();
    cfg.slot_width_ns = 0u;
    assert(net_acq_txsched_validate_config(&cfg) == -211);

    cfg = build_txsched_cfg();
    cfg.slot_offset_ns = cfg.period_ns;
    assert(net_acq_txsched_validate_config(&cfg) == -212);

    cfg = build_txsched_cfg();
    cfg.slot_width_ns = cfg.period_ns + 1u;
    assert(net_acq_txsched_validate_config(&cfg) == -213);

    assert(txsched_align_base_time(1500ull, 1000u, 100u, 2u) == 4000ull);
    assert(txsched_align_base_time(3000ull, 1000u, 200u, 1u) == 4000ull);

    return 0;
}
