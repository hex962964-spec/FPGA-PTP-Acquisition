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
static uint64_t master;
void net_acq_delay_cycles(volatile uint32_t n){(void)n;}
void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *c){(void)c;}
void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *c,uint32_t a,uint32_t b,uint32_t d,uint32_t e,uint32_t f){(void)c;(void)a;(void)b;(void)d;(void)e;(void)f;}
uint64_t net_acq_ptp_current_time(void){return master;}
void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *s){memset(s,0,sizeof(*s));}
static void prepare(ptp_slave_ctx_t *c){
 ptp_ctx_init(c);master=1000000000000ull;c->startup_done=1;
 c->have_filtered_offset=1;c->filtered_offset_ns=1000;c->last_offset_ns=1000;
 c->have_filtered_drift=1;c->filtered_drift_ppb=0;
 c->have_filtered_delay=1;c->filtered_delay_ns=2136;c->last_delay_ns=2136;
 c->have_prev_t1=1;c->prev_t1_master_ns=master;c->freq_adj_ppb=1000;c->freq_bias_ppb=0;
 for(unsigned i=0;i<PTP_DELAY_GUARD_WINDOW;i++)ptp_delay_guard_accept(&c->delay_guard,2136);
}
static void feed(ptp_slave_ctx_t *c,net_acq_ptp_test_cfg_t *cfg,int64_t offset,int64_t queue){
 master+=200000000ull;c->t1_master_ns=master;c->t2_slave_rx_ns=master+2136+offset;
 c->t3_slave_tx_ns=master+12800000ull+offset;c->t4_master_rx_ns=master+12800000ull+2136+queue;
 ptp_handle_servo_result(c,cfg);
}
int main(void){
 ptp_slave_ctx_t c,before;net_acq_ptp_test_cfg_t cfg=build_ptp_cfg();
 /* Applied +1000 ppb with observed +100 ns / .2s means +500 ppb compensation.
  * Acquisition learns +63 ppb (500/8), rather than integrating positive phase
  * as a negative oscillator bias. The phase pull is separate and negative. */
 prepare(&c);feed(&c,&cfg,1100,0);assert(c.last_sample_valid && !c.fine_locked);
 assert(c.freq_bias_ppb==63 && c.last_bias_update_ppb==63);
 assert(c.last_direct_phase_ppb<0 && c.last_target_freq_ppb==c.freq_bias_ppb+c.last_direct_phase_ppb);
 /* Rejecting two exchanges must preserve bias, compensation and RAW history;
  * resumed slope spans .6s, not .2s. */
 prepare(&c);before=c;feed(&c,&cfg,1100,2500);feed(&c,&cfg,1200,2500);
 assert(!c.last_sample_valid && c.freq_bias_ppb==before.freq_bias_ppb && c.freq_adj_ppb==before.freq_adj_ppb);
 assert(c.prev_t1_master_ns==before.prev_t1_master_ns && c.last_offset_ns==before.last_offset_ns);
 feed(&c,&cfg,1300,0);assert(c.last_sample_valid && c.freq_bias_ppb==63);
 /* No previous t1 after rebase/phase correction: never estimate a frequency
  * using the previous phase epoch. */
 prepare(&c);c.have_prev_t1=0;feed(&c,&cfg,1100,0);assert(c.freq_bias_ppb==0);
 /* The observer is limited at extreme startup mismatch. */
 prepare(&c);c.freq_adj_ppb=PTP_FREQ_ADJ_LIMIT_PPB;feed(&c,&cfg,1000,0);
 assert(c.last_bias_update_ppb==PTP_ACQUIRE_BIAS_STEP_MAX_PPB);
 return 0;
}
