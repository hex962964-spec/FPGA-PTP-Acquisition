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
static uint64_t master=1000000000000ull;
void net_acq_delay_cycles(volatile uint32_t n){(void)n;}
void net_acq_ptp_write_shadow(const net_acq_ptp_test_cfg_t *c){(void)c;}
void net_acq_ptp_write_ctrl(const net_acq_ptp_test_cfg_t *c,uint32_t a,uint32_t b,uint32_t d,uint32_t e,uint32_t f){(void)c;(void)a;(void)b;(void)d;(void)e;(void)f;}
uint64_t net_acq_ptp_current_time(void){return master;}
void net_acq_read_eth_snapshot(net_acq_eth_snapshot_t *s){memset(s,0,sizeof(*s));}
static void feed(ptp_slave_ctx_t *c,net_acq_ptp_test_cfg_t *cfg,int64_t off,int64_t queue){master+=200000000ull;c->t1_master_ns=master;c->t2_slave_rx_ns=master+2136+off;c->t3_slave_tx_ns=master+12800000ull+off;c->t4_master_rx_ns=master+12800000ull+2136+queue;ptp_handle_servo_result(c,cfg);}
int main(void){
 ptp_slave_ctx_t c,before;net_acq_ptp_test_cfg_t cfg=build_ptp_cfg();ptp_fast50_servo_t ob;ptp_fast50_result_t rr;
 assert(PTP_FAST50_ENABLE&&PTP_FINE_LOCK_THRESHOLD_NS==50&&PTP_FINE_LOCK_STREAK_MIN==8);
 ptp_ctx_init(&c);c.lock_streak=100;c.have_filtered_drift=1;c.filtered_drift_ppb=0;c.last_sample_valid=1;c.last_offset_ns=100;
 for(unsigned i=0;i<20;i++)ptp_update_lock_flags(&c,0,1,0);assert(!c.fine_locked); // filtered zero cannot hide RAW error
 c.last_offset_ns=40;for(unsigned i=0;i<8;i++)ptp_update_lock_flags(&c,40,1,0);assert(c.fine_locked);
 c.last_offset_ns=51;ptp_update_lock_flags(&c,0,1,0);assert(!c.fine_locked);
 ptp_fast50_init(&ob);rr=ptp_fast50_update(&ob,1000000000ull,1000,0,0);assert(rr.reanchored&&rr.command_ppb<0);
 rr=ptp_fast50_update(&ob,1200000000ull,500,rr.command_ppb,0);assert(!rr.reanchored);
 rr=ptp_fast50_update(&ob,1100000000ull,500,rr.command_ppb,0);assert(rr.reanchored);
 rr=ptp_fast50_update(&ob,10000000000ull,INT64_MIN,INT32_MIN,INT32_MIN);assert(rr.reanchored&&rr.command_ppb>=-1562500&&rr.command_ppb<=1562500);
 ptp_ctx_init(&c);for(unsigned i=0;i<PTP_DELAY_GUARD_WINDOW;i++)ptp_delay_guard_accept(&c.delay_guard,2136);
 feed(&c,&cfg,1000,0);assert(c.fast50.initialized);before=c;feed(&c,&cfg,2000,2500);assert(!c.last_sample_valid);assert(memcmp(&before.fast50,&c.fast50,sizeof(c.fast50))==0);assert(c.freq_adj_ppb==before.freq_adj_ppb);
 ptp_invalidate_exchange_history(&c);assert(!c.fast50.initialized);
 return 0;
}
