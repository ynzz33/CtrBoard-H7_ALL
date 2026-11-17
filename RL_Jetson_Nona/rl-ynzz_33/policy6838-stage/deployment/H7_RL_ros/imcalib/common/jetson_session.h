/* 可在主机执行的会话/动作门控；硬件许可在调用处再次检查。 */
#ifndef JETSON_SESSION_H
#define JETSON_SESSION_H
#include "jetson_protocol.h"
#include <math.h>
typedef struct {
    uint64_t session, last_rx_us, source_us, accepted_source_us, published_us, applied_us;
    uint32_t reset, sequence, ack, applied, latency_us, rejected, tx_drops;
    uint8_t blocked, warmup, accepted, warmup_action;
} jrl_session_t;
static inline void jrl_clear_action(jrl_session_t *s,uint32_t reset) {
    s->reset=reset;s->ack=0;s->applied=0;s->applied_us=0;s->latency_us=0;
    s->warmup=0;s->accepted=0;s->warmup_action=0;
}
static inline void jrl_disconnect(jrl_session_t *s) { s->session=0;s->blocked=1;jrl_clear_action(s,s->reset); }
static inline int jrl_expire(jrl_session_t *s,uint64_t now,int request) {
    if(!request) s->blocked=0; /* 断连后必须先退出遥控投入，再重新握手。 */
    if(s->session && now-s->last_rx_us>=100000u) { jrl_disconnect(s);return 1; }
    return 0;
}
/* 1握手，2有效动作，3停止；0拒绝。payload已经过CRC检查。 */
static inline int jrl_receive(jrl_session_t *s,const uint8_t *p,uint64_t now,int request,int permit) {
    uint32_t warmup=0;
    uint8_t type=p[5];uint64_t session=jrl_u64(p+8);uint32_t seq=jrl_u32(p+20);
    if(type==JRL_STOP && jrl_u16(p+6)==0 && session && session==s->session) { jrl_disconnect(s);return 3; }
    if(type==JRL_HELLO && jrl_u16(p+6)==4 && session && jrl_u32(p+32)==JRL_CONTRACT &&
       seq==s->sequence && now-s->published_us<10000u && !s->blocked) {
        if(session!=s->session) {
            if(request) goto reject;
            jrl_clear_action(s,s->reset);s->session=session;
        }
        s->last_rx_us=now;return 1;
    }
    if(type!=JRL_ACTION || jrl_u16(p+6)!=JRL_ACTION_BYTES || !permit || s->blocked || !session ||
       session!=s->session || jrl_u32(p+16)!=s->reset || seq!=s->sequence ||
       (s->accepted && seq==s->ack) || jrl_u64(p+24)!=s->source_us || now<s->source_us || now-s->source_us>=10000u) goto reject;
    for(unsigned i=0;i<6;i++) if(!isfinite(jrl_f32(p+32+4*i)) || fabsf(jrl_f32(p+32+4*i))>100.0f) goto reject;
    warmup=jrl_u32(p+56);
    if(warmup>1u || (!s->accepted && !warmup) || (s->accepted && s->warmup<5u) || (s->warmup>=5u && warmup)) goto reject;
    if(warmup) for(unsigned i=0;i<6;i++) if(jrl_f32(p+32+4*i)!=0.0f) goto reject;
    s->ack=seq;s->accepted_source_us=s->source_us;s->accepted=1;s->warmup_action=(uint8_t)warmup;s->last_rx_us=now;return 2;
reject:
    s->rejected++;return 0;
}
static inline void jrl_pd(jrl_session_t *s,uint64_t now) {
    if(!s->accepted) return;
    if(s->warmup_action && s->warmup<5u) s->warmup++;
    if(s->applied!=s->ack) {
        s->applied=s->ack;s->applied_us=now;
        s->latency_us=(uint32_t)(now-s->accepted_source_us);
    }
}
#endif
