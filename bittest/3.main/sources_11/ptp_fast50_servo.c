#include "ptp_fast50_servo.h"
#include <string.h>
static int64_t bound(int64_t v,int64_t n){return v>n?n:v< -n?-n:v;}
static int64_t rounded(int64_t a,int64_t d){return a>=0?(a+d/2)/d:-((-a+d/2)/d);}
/* Split before multiplying: no rate_Q16 * interval_ns overflow. */
static int64_t muldiv(int64_t a,int64_t b,int64_t d){return (a/d)*b+(a%d)*b/d;}
void ptp_fast50_init(ptp_fast50_servo_t *s){if(s)memset(s,0,sizeof(*s));}
ptp_fast50_result_t ptp_fast50_update(ptp_fast50_servo_t *s,uint64_t now,int64_t offset,int32_t applied,int32_t seed){
 ptp_fast50_result_t r={0};int64_t dt=0,prediction,innovation,tau,pull;
 offset=bound(offset,2000000ll);applied=(int32_t)bound(applied,PTP_FAST50_LIMIT_PPB);seed=(int32_t)bound(seed,PTP_FAST50_LIMIT_PPB);
 if(s->initialized&&now>s->previous_ns&&now-s->previous_ns<=2000000000ull)dt=(int64_t)(now-s->previous_ns);
 if(!s->initialized||dt<1000000ll){s->phase_q16=offset*PTP_FAST50_Q;s->rate_q16=-(int64_t)seed*PTP_FAST50_Q;r.reanchored=1;}
 else{
  prediction=s->phase_q16+muldiv(s->rate_q16+(int64_t)applied*PTP_FAST50_Q,dt,1000000000ll);
  innovation=offset*PTP_FAST50_Q-prediction;
  s->phase_q16=prediction+rounded(innovation*3ll,4ll);
  s->rate_q16+=muldiv(innovation,1000000000ll,dt)/8ll;
  s->rate_q16=bound(s->rate_q16,PTP_FAST50_LIMIT_PPB*PTP_FAST50_Q);
 }
 s->initialized=1;s->previous_ns=now;
 tau=dt*2ll;if(tau<PTP_FAST50_TAU_NS)tau=PTP_FAST50_TAU_NS;
 pull=-rounded(muldiv(s->phase_q16,1000000000ll,tau),PTP_FAST50_Q);
 pull=bound(pull,PTP_FAST50_PULL_LIMIT_PPB);
 r.phase_ns=rounded(s->phase_q16,PTP_FAST50_Q);r.phase_pull_ppb=pull;
 r.bias_ppb=(int32_t)bound(-rounded(s->rate_q16,PTP_FAST50_Q),PTP_FAST50_LIMIT_PPB);
 r.residual_ppb=(int32_t)bound(rounded(s->rate_q16,PTP_FAST50_Q)+applied,PTP_FAST50_LIMIT_PPB);
 r.command_ppb=(int32_t)bound((int64_t)r.bias_ppb+pull,PTP_FAST50_LIMIT_PPB);
 return r;
}
