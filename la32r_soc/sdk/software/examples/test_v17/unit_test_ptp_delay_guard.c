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
static unsigned writes;
static uint64_t master_time;
static unsigned req;
void net_acq_delay_cycles(volatile uint32_t n){(void)n;}
void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *c){(void)c;}
void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *c,uint32_t a,uint32_t b,uint32_t d,uint32_t e,uint32_t f){(void)c;(void)a;(void)b;(void)d;(void)e;(void)f;writes++;}
uint64_t net_acq_ptp_current_time(void){return master_time;}
void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *s){memset(s,0,sizeof(*s));}
static void feed(ptp_slave_ctx_t *c,net_acq_ptp_test_cfg_t *cfg,int64_t off,int64_t delay,int64_t extra1,int64_t extra2){
 master_time+=200000000ull;req++;
 c->sync_seq_id=(uint16_t)(req+100u);c->delay_req_seq_id=(uint16_t)req;
 c->t1_master_ns=master_time;c->t2_slave_rx_ns=master_time+delay+off+extra1;
 c->t3_slave_tx_ns=master_time+12800000ull+off;c->t4_master_rx_ns=master_time+12800000ull+delay+extra2;
 ptp_handle_servo_result(c,cfg);
}
static void prepare(ptp_slave_ctx_t *c,net_acq_ptp_test_cfg_t *cfg){
 ptp_ctx_init(c);master_time=1000000000ull;req=0;writes=0;
 c->startup_done=1;c->locked=1;c->fine_locked=1;c->lock_streak=100;c->fine_lock_streak=100;
 c->have_filtered_offset=1;c->filtered_offset_ns=100;
 c->have_filtered_delay=1;c->filtered_delay_ns=2136;c->last_delay_ns=2136;
 c->have_filtered_drift=1;c->filtered_drift_ppb=0;c->have_prev_t1=1;c->prev_t1_master_ns=master_time;
 for(int i=0;i<32;i++)feed(c,cfg,100,2136,0,0);
 assert(c->fine_locked && c->delay_guard.count==PTP_DELAY_GUARD_WINDOW);
}
static void unchanged_control(const ptp_slave_ctx_t *a,const ptp_slave_ctx_t *b){
 #define SAME(x) assert(a->x==b->x)
 SAME(filtered_offset_ns);SAME(filtered_delay_ns);SAME(filtered_drift_ppb);
 SAME(last_offset_ns);SAME(last_delay_ns);SAME(last_d1_ns);SAME(last_d2_ns);
 SAME(freq_adj_ppb);SAME(freq_bias_ppb);SAME(fine_bias_remainder);
 SAME(prev_t1_master_ns);SAME(have_prev_t1);SAME(servo_cycle_count);
 SAME(last_phase_ppb);SAME(last_direct_phase_ppb);SAME(last_bias_update_ppb);SAME(last_target_freq_ppb);
 #undef SAME
}
int main(void){
 ptp_slave_ctx_t c,before;net_acq_ptp_test_cfg_t cfg=build_ptp_cfg();unsigned oldwrites;
 /* Both delay directions are screened using path delay, not clipping offset. */
 prepare(&c,&cfg);before=c;oldwrites=writes;
 feed(&c,&cfg,100,2136,0,2500);
 assert(!c.last_sample_valid && c.last_sample_quality==PTP_SERVO_QUAL_PDV_SPIKE);
 unchanged_control(&c,&before);assert(writes==oldwrites);
 feed(&c,&cfg,100,2136,2500,0);
 assert(!c.last_sample_valid);unchanged_control(&c,&before);assert(writes==oldwrites);
 /* The first clean return uses elapsed time from the last accepted sample. */
 feed(&c,&cfg,100,2136,0,0);
 assert(c.last_sample_valid && c.servo_cycle_count==before.servo_cycle_count+1);
 assert(c.prev_t1_master_ns==master_time && c.last_phase_ppb==div_round_nearest_s64(-100ll*1000000000ll,600000000ll*PTP_FREQ_GAIN_DIV_FINE));
 /* A real clock phase shift leaves mean delay unchanged and is not filtered out. */
 before=c;feed(&c,&cfg,5000,2136,0,0);
 assert(c.last_sample_valid && c.last_offset_ns==5000 && c.servo_cycle_count==before.servo_cycle_count+1);
 /* Stable increases and decreases rebase on the eighth sample, not forever reject. */
 for(int direction=0;direction<2;direction++){
  prepare(&c,&cfg);int64_t path=direction?500:10000;int freq=c.freq_adj_ppb;int bias=c.freq_bias_ppb;
  for(unsigned i=0;i<PTP_DELAY_GUARD_REBASE-1;i++){feed(&c,&cfg,0,path,0,0);assert(!c.last_sample_valid);assert(c.freq_adj_ppb==freq);}
  feed(&c,&cfg,0,path,0,0);
  assert(c.last_sample_valid && c.last_sample_quality==PTP_SERVO_QUAL_PATH_REBASE);
  assert(c.delay_guard.count==PTP_DELAY_GUARD_REBASE && c.delay_guard.rebase_count==1);
  assert(!c.locked && !c.fine_locked && !c.have_filtered_drift);
  assert(c.freq_adj_ppb==freq && c.freq_bias_ppb==bias);
  assert(c.last_phase_ppb==0 && c.last_direct_phase_ppb==0 && c.last_bias_update_ppb==0 && c.last_target_freq_ppb==freq);
  for(int i=0;i<30;i++)feed(&c,&cfg,0,path,0,0);
  assert(c.fine_locked && c.last_sample_valid);
 }
 /* A bad very first sample must not trap the old gross quality thresholds. */
 for(int initial=10000;initial<=100000;initial*=10){
  ptp_ctx_init(&c);master_time=1000000000ull;req=0;
  feed(&c,&cfg,0,initial,0,0);assert(c.last_sample_valid);
  for(unsigned i=0;i<PTP_DELAY_GUARD_REBASE;i++)feed(&c,&cfg,0,2136,0,0);
  assert(c.last_sample_valid && c.last_sample_quality==PTP_SERVO_QUAL_PATH_REBASE);
  assert(c.delay_guard.rebase_count==1 && c.last_delay_ns==2136);
 }
 /* Unstable persistent disturbances cannot keep a stale fine lock indefinitely. */
 prepare(&c,&cfg);
 for(int i=0;i<24;i++){feed(&c,&cfg,100,4000+(i%2)*1000,0,0);assert(!c.last_sample_valid);}
 assert(c.delay_guard.rebase_count==0);
 ptp_expire_sync_lock(&c,0xffffffc0u,0xffffffc0u+99u,100u);assert(c.fine_locked);
 ptp_expire_sync_lock(&c,0xffffffc0u,0xffffffc0u+100u,100u);assert(!c.fine_locked && !c.locked && c.fine_lock_streak==0);
 feed(&c,&cfg,100,2136,0,0);assert(c.last_sample_valid);
 /* Gross invalid delay is still rejected before the guard; original protection remains. */
 prepare(&c,&cfg);unsigned cycles=c.servo_cycle_count;oldwrites=writes;
 feed(&c,&cfg,0,-1,0,0);assert(!c.last_sample_valid && c.last_sample_quality==PTP_SERVO_QUAL_DELAY_RANGE);
 assert(c.servo_cycle_count==cycles && writes==oldwrites && c.freeze_servo_cycles>0);
 return 0;
}