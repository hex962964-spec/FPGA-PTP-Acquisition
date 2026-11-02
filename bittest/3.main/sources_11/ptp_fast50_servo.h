#ifndef PTP_FAST50_SERVO_H
#define PTP_FAST50_SERVO_H
#include <stdint.h>
#ifndef PTP_FAST50_ENABLE
#define PTP_FAST50_ENABLE 1u
#endif
#define PTP_FAST50_Q 65536ll
#define PTP_FAST50_LIMIT_PPB 1562500ll
#define PTP_FAST50_PULL_LIMIT_PPB 200000ll
#define PTP_FAST50_TAU_NS 400000000ll
/* alpha-beta observer: separate oscillator rate from the applied correction.
 * Input: accepted, unshifted calibrated offset and actual master interval.
 * Q16 preserves fractional state; integer ppb hardware commands remain bounded.
 */
typedef struct {int64_t phase_q16,rate_q16;uint64_t previous_ns;uint32_t initialized;} ptp_fast50_servo_t;
typedef struct {int32_t command_ppb,bias_ppb,residual_ppb;int64_t phase_ns,phase_pull_ppb;uint32_t reanchored;} ptp_fast50_result_t;
void ptp_fast50_init(ptp_fast50_servo_t *s);
ptp_fast50_result_t ptp_fast50_update(ptp_fast50_servo_t *s,uint64_t master_ns,int64_t offset_ns,int32_t applied_ppb,int32_t seed_comp_ppb);
#endif
