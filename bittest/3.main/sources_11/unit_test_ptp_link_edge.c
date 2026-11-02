#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static int quiet_printf(const char *fmt,...){(void)fmt;return 0;}
#define printf quiet_printf
#define main hidden_firmware_main
#include "main.c"
#undef main
#undef printf
static unsigned quality_writes;
void net_acq_disable(uint32_t v){(void)v;}
void net_acq_set_time_quality(uint32_t state){(void)state;quality_writes++;}
int main(void){
 ptp_slave_ctx_t c;ptp_time_quality_ctx_t q;uint32_t up=1;
 ptp_ctx_init(&c);ptp_time_quality_init(&q);
 c.state=PTP_WAIT_DELAY_RESP;c.have_prev_t1=1;c.prev_t1_master_ns=123;
 c.have_filtered_offset=1;c.have_filtered_drift=1;c.have_filtered_delay=1;
 c.locked=1;c.fine_locked=1;c.freq_adj_ppb=456;c.freq_bias_ppb=321;
 c.stream_started=0; /* Physical down must clear pending even before acquisition starts. */
 q.state=PTP_TIME_LOCKED;q.stream_continuity_allowed=1;
 ptp_observe_link_edge(&c,&q,&up,0);
 assert(!up && c.state==PTP_WAIT_SYNC && !c.have_prev_t1);
 assert(!c.have_filtered_offset && !c.have_filtered_drift && !c.have_filtered_delay);
 assert(!c.locked && !c.fine_locked && q.state==PTP_TIME_HOLDOVER);
 assert(c.freq_adj_ppb==456 && c.freq_bias_ppb==321 && quality_writes==1);
 ptp_observe_link_edge(&c,&q,&up,0);assert(quality_writes==1);
 ptp_observe_link_edge(&c,&q,&up,1);assert(up && q.state==PTP_TIME_HOLDOVER);
 return 0;
}
