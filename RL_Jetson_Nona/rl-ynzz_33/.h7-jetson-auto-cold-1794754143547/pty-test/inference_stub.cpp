
#include "inference.hpp"
namespace jrl {
struct Inference::Impl {};
Inference::Inference(const std::string&) : impl_(new Impl) {}
Inference::~Inference() = default;
void Inference::convert(const std::string&, const std::string&) {}
Action Inference::run(const Input& input) {
    Action out{};
    for (unsigned i=0;i<out.size();++i) out[i]=input[i];
    return out;
}
}
