#pragma once
#include "policy.hpp"

namespace jrl {
/* 阶段只选择模型，H7继续拥有最终出力许可。 */
class JumpSwitch {
public:
    enum Phase { Walk, Armed, Takeoff, Flight, Landing, Fault };
    Phase phase = Walk;
private:
    bool at_top_ = true;
    uint64_t begin_ = 0, landed_ = 0, last_ = 0, off_ = 0, on_ = 0;
    uint32_t reset_ = 0;
    bool seeded_ = false;
    std::array<float,125> history_{};
public:
    bool update(uint32_t flags, uint32_t reset, uint64_t now, float force) {
        const bool selected = flags & 1u, top = flags & 2u;
        const bool valid = (flags & 4u) && std::isfinite(force);
        const bool unloaded = force < 35.0f, loaded = force > 35.0f;
        if (!seeded_ || reset != reset_ || now <= last_ || now-last_ > 100000u) {
            phase = selected ? Armed : Walk;
            at_top_ = top; seeded_ = false; begin_ = landed_ = off_ = on_ = 0;
        }
        reset_ = reset; last_ = now;
        if (!selected) {
            phase = Walk; at_top_ = top; off_ = on_ = 0;
            return false;
        }
        if (phase == Walk) phase = Armed;
        if (phase == Armed && top && !at_top_ && valid && loaded) {
            phase = Takeoff; begin_ = now; off_ = on_ = 0;
        }
        at_top_ = top;
        // 支持力估计由H7发布；跳跃计时独立于传统AIR的200ms离地判据。
        if (phase == Takeoff) {
            if (valid && unloaded) {
                if (!off_) off_ = now;
                if (now-off_ >= 30000u) { phase = Flight; on_ = 0; }
            } else off_ = 0;
        } else if (phase == Flight) {
            if (valid && loaded) {
                if (!on_) on_ = now;
                if (now-on_ >= 50000u) { phase = Landing; landed_ = now; }
            } else on_ = 0;
        } else if (phase == Landing) {
            if (!valid || !loaded) { phase = Flight; on_ = landed_ = 0; }
            else if (now-landed_ >= 80000u) phase = Armed;
        }
        // 训练配置max_jump_ticks=140，每策略拍10ms；超时不冒充落地。
        if (phase >= Takeoff && phase <= Landing && now-begin_ >= 1400000u)
            phase = Fault;
        if (phase >= Takeoff && phase <= Landing && !valid) phase = Fault;
        return phase == Takeoff || phase == Flight || phase == Landing;
    }
    Input input(const Input& walk) {
        std::array<float,25> raw{};
        std::copy_n(walk.begin(),25,raw.begin());
        raw[6] *= 1.5f; // WALK前进命令乘2，Jump乘3。
        raw[8] = 1.0f * 5.0f;
        if (!seeded_) {
            for (unsigned i=0;i<5;i++) std::copy(raw.begin(),raw.end(),history_.begin()+25*i);
            seeded_=true;
        } else {
            std::move(history_.begin()+25,history_.end(),history_.begin());
            std::copy(raw.begin(),raw.end(),history_.begin()+100);
        }
        Input result{};
        std::copy(raw.begin(),raw.end(),result.begin());
        std::copy(history_.begin(),history_.end(),result.begin()+25);
        return result;
    }
};
}
