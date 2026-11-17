#pragma once
#include "jetson_protocol.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
namespace jrl {
using Input=std::array<float,150>;
using Action=std::array<float,6>;
inline float clip(float x,float limit) { return std::max(-limit,std::min(limit,x)); }
/* 与H7_RL RL_Observation_Build保持相同计算顺序；H7已完成物理映射。 */
inline std::array<float,25> frame(const uint8_t *p) {
    std::array<float,25> raw{};
    constexpr float nominal[4]={-.604720554f,.608324173f,.604720554f,-.608324173f};
    constexpr unsigned order[6]={0,1,3,4,2,5};
    for(unsigned i=0;i<3;i++) raw[i]=jrl_f32(p+32+4*i)*.25f;
    const float pitch=-2.f*(3.14159265358979323846f/180.f);
    const float si=std::sin(pitch),co=std::cos(pitch);
    raw[3]=co*jrl_f32(p+44)+si*jrl_f32(p+52);raw[4]=jrl_f32(p+48);
    raw[5]=co*jrl_f32(p+52)-si*jrl_f32(p+44);
    const bool tracking=(jrl_u32(p)&JRL_TRACKING)!=0;
    raw[6]=tracking ? clip(jrl_f32(p+96),1.5f)*2.f:0.f;
    raw[7]=tracking ? clip(jrl_f32(p+100),3.f)*.25f:0.f;
    raw[8]=std::clamp(jrl_f32(p+104)+(.06f-.02352f),.22f,.30f)*5.f;
    for(unsigned i=0;i<4;i++) raw[9+i]=jrl_f32(p+56+4*i)-nominal[i];
    for(unsigned i=0;i<6;i++) { raw[13+order[i]]=jrl_f32(p+72+4*i)*.05f;raw[19+i]=jrl_f32(p+108+4*i); }
    for(float f:raw) if(!std::isfinite(f)) throw std::runtime_error("non-finite observation");
    return raw;
}
class Observation {
    bool initialized_=false;
    std::array<float,125> history_{};
public:
    bool initialized() const { return initialized_; }
    void reset() { initialized_=false;history_.fill(0); }
    Input build(const uint8_t *p) {
        auto raw=frame(p);
        if(!initialized_) { for(unsigned i=0;i<5;i++) std::copy(raw.begin(),raw.end(),history_.begin()+i*25);initialized_=true; }
        else { std::memmove(history_.data(),history_.data()+25,100*sizeof(float));std::copy(raw.begin(),raw.end(),history_.begin()+100); }
        Input input;for(unsigned i=0;i<25;i++) input[i]=clip(raw[i],100.f);
        std::copy(history_.begin(),history_.end(),input.begin()+25);return input;
    }
};
}
