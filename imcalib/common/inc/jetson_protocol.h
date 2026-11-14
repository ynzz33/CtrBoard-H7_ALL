/* Jetson/H7共用字节协议。显式小端编码，不发送C结构体或依赖对齐。 */
#ifndef JETSON_PROTOCOL_H
#define JETSON_PROTOCOL_H
#include <stdint.h>
#include <string.h>
#define JRL_VERSION 2u
#define JRL_CONTRACT 1767902u
#define JRL_MAX_FRAME 384u
#define JRL_HEADER 32u
#define JRL_STATE_BYTES 224u
#define JRL_ACTION_BYTES 32u
#define JRL_HELLO 1u
#define JRL_STATE 2u
#define JRL_ACTION 3u
#define JRL_STOP 4u
#define JRL_VALID 1u
#define JRL_SELECTED 2u
#define JRL_CONNECTED 4u
#define JRL_BLOCKED 8u
#define JRL_TRACKING 16u
#define JRL_WARMUP 1u
/* STATE payload: flags/fault/ack/warmup/applied/latency/contract/rejects (8*u32),
 * gyro3/gravity3/q4/dq6/command3/accepted6/target6/request6 (37*f32),
 * quaternion4, applied_time_us(u64), tx_drops/u32, output sequence/policy/flags/valid (4*u32).
 * ACTION payload: action6/f32, warmup flag/u32, inference_us/u32. */
static inline uint16_t jrl_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static inline uint32_t jrl_u32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static inline uint64_t jrl_u64(const uint8_t *p) { return jrl_u32(p)|((uint64_t)jrl_u32(p+4)<<32); }
static inline void jrl_p16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static inline void jrl_p32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static inline void jrl_p64(uint8_t *p,uint64_t v) { jrl_p32(p,(uint32_t)v);jrl_p32(p+4,(uint32_t)(v>>32)); }
static inline float jrl_f32(const uint8_t *p) { uint32_t v=jrl_u32(p);float f;memcpy(&f,&v,4);return f; }
static inline void jrl_pf32(uint8_t *p,float f) { uint32_t v;memcpy(&v,&f,4);jrl_p32(p,v); }
static inline uint32_t jrl_crc(const uint8_t *p,unsigned n) {
    uint32_t c=0xffffffffu;
    for(unsigned i=0;i<n;i++) { c^=p[i];for(unsigned b=0;b<8;b++) c=(c>>1)^((0u-(c&1u))&0xedb88320u); }
    return ~c;
}
static inline void jrl_header(uint8_t *p,uint8_t type,uint16_t bytes,uint64_t session,uint32_t reset,uint32_t seq,uint64_t us) {
    memcpy(p,"JRL2",4);p[4]=JRL_VERSION;p[5]=type;jrl_p16(p+6,bytes);
    jrl_p64(p+8,session);jrl_p32(p+16,reset);jrl_p32(p+20,seq);jrl_p64(p+24,us);
}
static inline unsigned jrl_finish(uint8_t *p) { unsigned n=JRL_HEADER+jrl_u16(p+6);jrl_p32(p+n,jrl_crc(p,n));return n+4; }
typedef struct { uint8_t data[JRL_MAX_FRAME];uint16_t used;uint32_t errors; } jrl_parser_t;
/* 返回完整包长；调用者须在喂下一字节前消费data。坏包滑动重同步。 */
static inline unsigned jrl_feed(jrl_parser_t *s,uint8_t b) {
    if(s->used==JRL_MAX_FRAME) { memmove(s->data,s->data+1,--s->used);s->errors++; }
    s->data[s->used++]=b;
    for(;;) {
        if(s->used<4) return 0;
        if(memcmp(s->data,"JRL2",4)) { memmove(s->data,s->data+1,--s->used);continue; }
        if(s->used<8) return 0;
        unsigned n=JRL_HEADER+jrl_u16(s->data+6)+4;
        if(s->data[4]!=JRL_VERSION || n>JRL_MAX_FRAME) { s->errors++;memmove(s->data,s->data+1,--s->used);continue; }
        if(s->used<n) return 0;
        if(jrl_u32(s->data+n-4)!=jrl_crc(s->data,n-4)) { s->errors++;memmove(s->data,s->data+1,--s->used);continue; }
        s->used=0;return n;
    }
}
#endif
