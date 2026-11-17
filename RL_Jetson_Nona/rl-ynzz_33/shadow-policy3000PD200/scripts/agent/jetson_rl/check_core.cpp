/* 执行实际协议解析/会话门控和C++观测代码，不用替身重新实现算法。 */
#include "jetson_session.h"
#include "policy.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>
#include <random>
static std::vector<uint8_t> packet(uint8_t type,uint64_t session,uint32_t reset,uint32_t seq,uint64_t us,bool warm=false) {
    std::vector<uint8_t> p(68);jrl_header(p.data(),type,type==JRL_ACTION?32:(type==JRL_HELLO?4:0),session,reset,seq,us);
    if(type==JRL_HELLO) jrl_p32(p.data()+32,JRL_CONTRACT);
    if(type==JRL_ACTION) jrl_p32(p.data()+56,warm);
    p.resize(jrl_finish(p.data()));return p;
}
int main(int argc,char** argv) {
    assert(jrl_crc(reinterpret_cast<const uint8_t*>("123456789"),9)==0xcbf43926u);
    auto hello=packet(JRL_HELLO,123,4,1,1000);jrl_parser_t parser{};unsigned count=0;
    for(unsigned i=0;i<37;i++) assert(!jrl_feed(&parser,0xff));
    for(auto b:hello) { count+=bool(jrl_feed(&parser,b)); }
    assert(count==1);
    auto bad=hello;bad[15]^=1;for(auto b:bad) assert(!jrl_feed(&parser,b));
    for(auto b:hello) { count+=bool(jrl_feed(&parser,b)); }
    assert(count==2&&parser.errors==1);
    for(int i=0;i<2;i++) { for(auto b:hello) { count+=bool(jrl_feed(&parser,b)); } }
    assert(count==4);
    auto large=hello;jrl_p16(large.data()+6,65535);for(auto b:large) jrl_feed(&parser,b);
    for(auto b:hello) { count+=bool(jrl_feed(&parser,b)); }
    assert(count==5);
    jrl_session_t s{};s.sequence=1;s.reset=4;s.published_us=s.source_us=1000;
    assert(jrl_receive(&s,hello.data(),2000,1,0)==0); // 投入时不能开始新会话。
    assert(jrl_receive(&s,hello.data(),2000,0,0)==1);
    auto action=packet(JRL_ACTION,123,4,1,1000,true);
    assert(jrl_receive(&s,action.data(),2100,1,0)==0);
    assert(jrl_receive(&s,action.data(),2100,1,1)==2);
    assert(jrl_receive(&s,action.data(),2200,1,1)==0); // 同序重复不喂看门狗。
    s.sequence=2;s.source_us=11000;auto next=packet(JRL_ACTION,123,4,2,11000);
    assert(jrl_receive(&s,next.data(),12000,1,1)==0);
    for(unsigned i=0;i<5;i++) jrl_pd(&s,3000+2000*i);
    assert(s.warmup==5&&s.applied==1&&s.latency_us==2000); // source_us更新不污染旧动作首PD。
    assert(jrl_receive(&s,next.data(),12000,1,1)==2);
    s.sequence=3;s.source_us=21000;next=packet(JRL_ACTION,123,4,3,21000);
    assert(jrl_receive(&s,next.data(),31000,1,1)==0);
    jrl_pf32(next.data()+32,NAN);assert(jrl_receive(&s,next.data(),22000,1,1)==0);
    jrl_pf32(next.data()+32,101);assert(jrl_receive(&s,next.data(),22000,1,1)==0);
    jrl_clear_action(&s,5);next=packet(JRL_ACTION,123,4,3,21000,true);assert(jrl_receive(&s,next.data(),22000,1,1)==0);
    assert(jrl_expire(&s,112000,1)&&s.blocked&&!s.session);
    hello=packet(JRL_HELLO,456,5,3,21000);s.published_us=112000;
    assert(jrl_receive(&s,hello.data(),112001,1,1)==0);
    jrl_expire(&s,112002,0);assert(!s.blocked);assert(jrl_receive(&s,hello.data(),112003,0,0)==1);
    auto stop=packet(JRL_STOP,456,5,3,21000);assert(jrl_receive(&s,stop.data(),112004,1,1)==3&&s.blocked);
    jrl::Observation obs;uint8_t state[JRL_STATE_BYTES]={0};jrl_p32(state,JRL_TRACKING);jrl_pf32(state+52,-1);jrl_pf32(state+104,.22352f);
    jrl_pf32(state+32,1000);auto x=obs.build(state);assert(x[0]==100&&x[25]==250&&x[125]==250);
    jrl_pf32(state+32,4);x=obs.build(state);assert(x[0]==1&&x[25]==250&&x[125]==1);
    obs.reset();x=obs.build(state);assert(x[25]==1&&x[125]==1);
    jrl_p32(state,0);jrl_pf32(state+96,1);jrl_pf32(state+100,1);
    x=obs.build(state);assert(x[6]==0&&x[7]==0); // ROS运动解锁保护独立保留。
    jrl_pf32(state+32,INFINITY);bool threw=false;try { obs.build(state); } catch(...) { threw=true; }assert(threw);
    /* 供实际主工程RL_Observation_Build连续序列对照。 */
    if(argc==3) {
        std::ofstream inputs(argv[1],std::ios::binary),outputs(argv[2],std::ios::binary);assert(inputs&&outputs);
        std::mt19937 rng(17679);std::uniform_real_distribution<float> uniform(-1,1);obs.reset();
        for(unsigned n=0;n<256;n++) {
            float row[26];for(float& f:row) f=uniform(rng);row[25]=n%31==0;
            if(n==10) row[0]=1000; // 历史保存裁剪前帧。
            if(row[25]) { obs.reset(); }
            memset(state,0,sizeof(state));jrl_p32(state,JRL_TRACKING);
            for(unsigned i=0;i<16;i++) jrl_pf32(state+32+4*i,row[i]);
            for(unsigned i=0;i<3;i++) jrl_pf32(state+96+4*i,row[16+i]);
            for(unsigned i=0;i<6;i++) jrl_pf32(state+108+4*i,row[19+i]);
            auto input=obs.build(state);inputs.write(reinterpret_cast<char*>(row),sizeof(row));outputs.write(reinterpret_cast<char*>(input.data()),sizeof(input));
        }
    }
    std::cout<<"protocol/session/deadline/rearm/warmup/history checks passed\n";
}
