#pragma once
#include "policy.hpp"
#include <memory>
#include <string>
namespace jrl {
std::string sha256(const std::string& path);
class Inference {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    static void convert(const std::string& onnx,const std::string& engine, const std::string& expected_sha = {});
    explicit Inference(const std::string& engine, const std::string& expected_sha = {});
    ~Inference();
    Action run(const Input& input);
};
}
