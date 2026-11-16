/* H7与Jetson的JRL2实控和遥测协议，字段显式按小端编码。 */
#ifndef JETSON_PROTOCOL_H
#define JETSON_PROTOCOL_H
#include <stdint.h>
#include <string.h>
#define JRL_VERSION 2u
#define JRL_MAX_FRAME 384u
#define JRL_HEADER 32u
#define JRL_EXTERNAL_CONTRACT 3000201u

/* 输入两帧共150个float，顺序是obs[25]、history[125]。
 * 动作回包携带原请求标识、6路原始动作及Jetson推理耗时us。 */
enum {
    JRL_TELEMETRY = 5,
    JRL_INPUT_FIRST = 6,
    JRL_INPUT_SECOND = 7,
    JRL_REPLY = 8,
    JRL_READY = 10,
    JRL_EXTERNAL_PD = 11,
    JRL_EXTERNAL_RESTART = 12, /* 发给旧会话，要求结束当前推理进程。 */
    JRL_SWING_SAMPLE = 13,
    JRL_STAIR_SAMPLE = 14,
    JRL_TELEMETRY_BYTES = 240,
    JRL_READY_BYTES = 4,
    JRL_EXTERNAL_PD_BYTES = 16,
    JRL_EXTERNAL_RESTART_BYTES = 4,
    /* 摆矩采集样本，保留独立类型以兼容现有 type 5 导出。
     * 帧头 session / reset / seq / us 对应采集会话 / 试次 / 控制拍 / 板端时间。
     * 负载按下列顺序逐字段编码，双关节数组均为前、后。
     * u32: flags, board_drops, trial_config, fault, feedback_age_us[2]。
     * trial_config 低三个字节依次为测试侧（1=左，2=右）、档位、档内序号，后两项从1开始。
     * f32: swing_nm, axial_n, angle_rad[2], velocity_rad_s[2],
     *      feedback_nm[2], mapped_nm[2], prelimit_nm[2], submitted_nm[2],
     *      length_m, length_rate_m_s, swing_rad, swing_rate_rad_s, swing_ref_rad_s。 */
    JRL_SWING_FLOATS = 19,
    JRL_SWING_SAMPLE_BYTES = 24 + JRL_SWING_FLOATS * 4,
    JRL_STAIR_FLOATS = 45,
    JRL_STAIR_SAMPLE_BYTES = 24 + JRL_STAIR_FLOATS * 4,
    JRL_HALF_FLOATS = 75,
    JRL_INPUT_BYTES = 4 + JRL_HALF_FLOATS * 4,
    JRL_REPLY_BYTES = 4 + 6 * 4 + 4
};
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
typedef struct { uint8_t data[JRL_MAX_FRAME];uint16_t used; } jrl_parser_t;
/* 返回完整包长；调用者须在喂下一字节前消费data。坏包滑动重同步。 */
static inline unsigned jrl_feed(jrl_parser_t *s,uint8_t b) {
    if(s->used==JRL_MAX_FRAME) { memmove(s->data,s->data+1,--s->used); }
    s->data[s->used++]=b;
    for(;;) {
        if(s->used<4) return 0;
        if(memcmp(s->data,"JRL2",4)) { memmove(s->data,s->data+1,--s->used);continue; }
        if(s->used<8) return 0;
        unsigned n=JRL_HEADER+jrl_u16(s->data+6)+4;
        if(s->data[4]!=JRL_VERSION || n>JRL_MAX_FRAME) { memmove(s->data,s->data+1,--s->used);continue; }
        if(s->used<n) return 0;
        if(jrl_u32(s->data+n-4)!=jrl_crc(s->data,n-4)) { memmove(s->data,s->data+1,--s->used);continue; }
        s->used=0;return n;
    }
}
#endif
