#ifndef PTP_LOCK_DISPLAY_H
#define PTP_LOCK_DISPLAY_H
#include <stdint.h>

/* Display-only freshness tracking; never changes PTP servo/stream state. */
typedef struct {
    uint32_t last_cycle;
    uint32_t last_ticks;
    uint32_t fresh;
    uint32_t shown;
} ptp_lock_display_t;

static inline uint32_t ptp_lock_display_poll(ptp_lock_display_t *d,
                                             uint32_t cycle,
                                             uint32_t fine_locked,
                                             uint32_t fault,
                                             uint32_t now,
                                             uint32_t timeout_ticks)
{
    if (cycle != d->last_cycle) {
        d->last_cycle = cycle;
        d->last_ticks = now;
        d->fresh = 1u;
    }
    if (fault || (uint32_t)(now - d->last_ticks) >= timeout_ticks) {
        /* Latch expiry until another exchange; counter wrap must not relight 1. */
        d->fresh = 0u;
    }
    return (fine_locked && d->fresh) ? 1u : 0u;
}
#endif
