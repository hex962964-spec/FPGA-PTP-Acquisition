#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static int quiet_printf(const char *fmt, ...) { (void)fmt; return 0; }
#define printf quiet_printf
#define main hidden_firmware_main
#include "main.c"
#undef main
#undef printf
void net_acq_delay_cycles(volatile uint32_t n){(void)n;}
void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *c){(void)c;}
void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *c,uint32_t a,uint32_t b,uint32_t d,uint32_t e,uint32_t f){(void)c;(void)a;(void)b;(void)d;(void)e;(void)f;}
uint64_t net_acq_ptp_current_time(void){return 2000000000ull;}
void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *s){memset(s,0,sizeof(*s));}

static void prepare_fine(ptp_slave_ctx_t *c,int64_t offset){
 ptp_ctx_init(c);c->startup_done=1;c->locked=1;c->fine_locked=1;
 c->lock_streak=100;c->fine_lock_streak=100;
 c->have_filtered_offset=1;c->filtered_offset_ns=offset;
 c->have_filtered_drift=1;c->filtered_drift_ppb=0;
 c->have_filtered_delay=1;c->filtered_delay_ns=2136;c->last_delay_ns=2136;
 c->have_prev_t1=1;c->prev_t1_master_ns=1000000000ull;
}
static void feed(ptp_slave_ctx_t *c,net_acq_ptp_test_cfg_t *cfg,int64_t off){
 uint64_t t=c->prev_t1_master_ns+200000000ull;
 c->t1_master_ns=t;c->t2_slave_rx_ns=t+2136+off;
 c->t3_slave_tx_ns=t+12800000ull+off;c->t4_master_rx_ns=t+12800000ull+2136;
 ptp_handle_servo_result(c,cfg);
}
int main(void){
 ptp_slave_ctx_t c;net_acq_ptp_test_cfg_t cfg=build_ptp_cfg();
 int64_t sum=0;ptp_ctx_init(&c);
 /* A repeated 1/div ppb correction must eventually reach the hardware integer. */
 for(int i=0;i<PTP_FINE_BIAS_ACC_DIV;i++)sum+=ptp_fine_bias_step_ppb(&c,1);
 assert(sum==1 && c.fine_bias_remainder==0);
 sum=0;
 for(int i=0;i<PTP_FINE_BIAS_ACC_DIV;i++)sum+=ptp_fine_bias_step_ppb(&c,-1);
 assert(sum==-1 && c.fine_bias_remainder==0);
 /* Opposite small errors cancel instead of creating a DC bias. */
 for(int i=0;i<1000;i++){
  sum+=ptp_fine_bias_step_ppb(&c,1);sum+=ptp_fine_bias_step_ppb(&c,-1);
 }
 assert(sum==-1 && c.fine_bias_remainder==0);
 /* Exercise the actual servo: small fixed errors integrate in both directions. */
 prepare_fine(&c,200);
 for(int i=0;i<256;i++)feed(&c,&cfg,200);
 assert(c.freq_bias_ppb<0 && c.fine_locked);
 int negative=c.freq_bias_ppb;
 prepare_fine(&c,-200);
 for(int i=0;i<256;i++)feed(&c,&cfg,-200);
 assert(c.freq_bias_ppb==-negative && c.fine_locked);
 /* A rejected sample/freeze cannot carry a correction into a new tracking period. */
 c.fine_bias_remainder=17;ptp_freeze_servo(&c,2);
 assert(c.fine_bias_remainder==0);
 c.fine_bias_remainder=17;ptp_update_lock_flags(&c,0,1,1);
 assert(c.fine_bias_remainder==0 && !c.fine_locked);
 c.fine_bias_remainder=17;ptp_servo_clear_frequency_state(&c,&cfg,0);
 assert(c.fine_bias_remainder==0);
 /* Saturation limits both whole and fractional accumulated corrections. */
 ptp_ctx_init(&c);
 assert(ptp_fine_bias_step_ppb(&c,PTP_FINE_BIAS_ACC_DIV*(PTP_FINE_BIAS_UPDATE_MAX_PPB+2))==PTP_FINE_BIAS_UPDATE_MAX_PPB);
 assert(c.fine_bias_remainder==0);
 prepare_fine(&c,-800);c.freq_bias_ppb=PTP_FREQ_ADJ_LIMIT_PPB;
 c.fine_bias_remainder=PTP_FINE_BIAS_ACC_DIV/2-1;feed(&c,&cfg,-800);
 assert(c.freq_bias_ppb==PTP_FREQ_ADJ_LIMIT_PPB && c.fine_bias_remainder==0);
 /* A phase step starts with no fractional memory from the old phase epoch. */
 prepare_fine(&c,0);c.fine_bias_remainder=17;feed(&c,&cfg,500000);
 assert(c.fine_bias_remainder==0 && !c.fine_locked);
 return 0;
}
