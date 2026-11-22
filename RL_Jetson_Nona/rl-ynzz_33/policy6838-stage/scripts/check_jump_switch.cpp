#include "jump_switch.hpp"
#include <cassert>
#include <iostream>

int main() {
    jrl::JumpSwitch state;
    jrl::Input walk{};
    walk[6]=2.f; walk[8]=1.5f;
    uint64_t time=1000000;
    auto tick=[&](unsigned flags,float force) {
        const bool active=state.update(flags,1,time,force);
        const auto x=state.input(walk);
        assert(x[6]==3.f && x[8]==5.0f);
        assert(x[25+100+8]==5.0f);
        time+=10000;
        return active;
    };
    assert(!tick(5,80)); // 选择Jump，拨轮未到顶。
    assert(tick(7,80) && state.phase==jrl::JumpSwitch::Takeoff);
    for(int i=0;i<4;i++) tick(7,0);
    assert(state.phase==jrl::JumpSwitch::Flight);
    for(int i=0;i<6;i++) tick(7,80);
    assert(state.phase==jrl::JumpSwitch::Landing);
    tick(7,0); // 接触反弹，重新确认落地。
    assert(state.phase==jrl::JumpSwitch::Flight);
    for(int i=0;i<6;i++) tick(7,80);
    for(int i=0;i<8;i++) tick(7,80);
    assert(state.phase==jrl::JumpSwitch::Armed);
    assert(!tick(7,80)); // 一直顶住不会再次起跳。
    tick(5,80);
    assert(tick(7,80));
    tick(3,80); // 支持力失效，故障锁止。
    assert(state.phase==jrl::JumpSwitch::Fault);
    assert(!tick(7,80));
    tick(4,80); // 退出跳跃选择。
    assert(state.phase==jrl::JumpSwitch::Walk);
    tick(5,80);
    tick(7,80);
    for(int i=0;i<141;i++) tick(7,80);
    assert(state.phase==jrl::JumpSwitch::Fault); // 未起跳不能以超时冒充落地。
    std::cout<<"jump switch transitions passed\n";
}
