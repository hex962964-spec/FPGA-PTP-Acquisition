#ifndef PTP_DELAY_GUARD_H
#define PTP_DELAY_GUARD_H
#include <stdint.h>

/* Reject abnormal path delay, never the offset itself. All times are ns.
 * Median/MAD reference contains accepted measurements only. A stable sustained
 * path change rebases after a bounded run; raw observations remain diagnostic. */
#define PTP_DELAY_GUARD_WINDOW 17u
#define PTP_DELAY_GUARD_WARMUP 8u
#define PTP_DELAY_GUARD_REBASE 8u
#define PTP_DELAY_GUARD_MARGIN_NS 128ll
#define PTP_DELAY_GUARD_MAD_MULT 6ll

typedef enum {
    PTP_DELAY_GUARD_ACCEPT = 0,
    PTP_DELAY_GUARD_SPIKE,
    PTP_DELAY_GUARD_PATH_REBASE
} ptp_delay_guard_result_t;

typedef struct {
    int64_t samples[PTP_DELAY_GUARD_WINDOW];
    int64_t candidates[PTP_DELAY_GUARD_REBASE];
    uint32_t count, next, candidate_count;
    uint32_t spike_count, rebase_count;
} ptp_delay_guard_t;

static inline void ptp_delay_guard_init(ptp_delay_guard_t *g)
{
    g->count=0u;g->next=0u;g->candidate_count=0u;
    g->spike_count=0u;g->rebase_count=0u;
}
static inline void ptp_delay_guard_sort(int64_t *v,uint32_t n)
{
    uint32_t i,j;int64_t t;
    for(i=1u;i<n;i++){
        t=v[i];j=i;
        while(j>0u && v[j-1u]>t){v[j]=v[j-1u];j--;}
        v[j]=t;
    }
}
static inline void ptp_delay_guard_accept(ptp_delay_guard_t *g,int64_t delay)
{
    g->samples[g->next]=delay;
    g->next=(g->next+1u)%PTP_DELAY_GUARD_WINDOW;
    if(g->count<PTP_DELAY_GUARD_WINDOW)g->count++;
}
static inline ptp_delay_guard_result_t ptp_delay_guard_check_qualified(
    ptp_delay_guard_t *g,int64_t delay,int64_t *reference,int64_t *margin,uint32_t force_probe)
{
    int64_t sorted[PTP_DELAY_GUARD_WINDOW],dev[PTP_DELAY_GUARD_WINDOW];
    int64_t ref=delay,bound=PTP_DELAY_GUARD_MARGIN_NS,low,high;
    uint32_t i;
    if(g->count){
        for(i=0u;i<g->count;i++)sorted[i]=g->samples[i];
        ptp_delay_guard_sort(sorted,g->count);ref=sorted[g->count/2u];
        for(i=0u;i<g->count;i++)dev[i]=(sorted[i]>=ref)?sorted[i]-ref:ref-sorted[i];
        ptp_delay_guard_sort(dev,g->count);
        if(dev[g->count/2u]*PTP_DELAY_GUARD_MAD_MULT>bound)
            bound=dev[g->count/2u]*PTP_DELAY_GUARD_MAD_MULT;
    }
    *reference=ref;*margin=bound;
    if((g->count<PTP_DELAY_GUARD_WARMUP && !force_probe) || ((delay>=ref)?delay-ref:ref-delay)<=bound){
        g->candidate_count=0u;return PTP_DELAY_GUARD_ACCEPT;
    }
    low=delay;high=delay;
    for(i=0u;i<g->candidate_count;i++){
        if(g->candidates[i]<low)low=g->candidates[i];
        if(g->candidates[i]>high)high=g->candidates[i];
    }
    if(high-low>bound)g->candidate_count=0u;
    g->candidates[g->candidate_count++]=delay;
    if(g->candidate_count>=PTP_DELAY_GUARD_REBASE){
        g->count=0u;g->next=0u;
        for(i=0u;i<PTP_DELAY_GUARD_REBASE;i++)ptp_delay_guard_accept(g,g->candidates[i]);
        g->candidate_count=0u;
        if(g->rebase_count<UINT32_MAX)g->rebase_count++;
        return PTP_DELAY_GUARD_PATH_REBASE;
    }
    if(g->spike_count<UINT32_MAX)g->spike_count++;
    return PTP_DELAY_GUARD_SPIKE;
}
static inline ptp_delay_guard_result_t ptp_delay_guard_check(
    ptp_delay_guard_t *g,int64_t delay,int64_t *reference,int64_t *margin)
{
    return ptp_delay_guard_check_qualified(g,delay,reference,margin,0u);
}
#endif
