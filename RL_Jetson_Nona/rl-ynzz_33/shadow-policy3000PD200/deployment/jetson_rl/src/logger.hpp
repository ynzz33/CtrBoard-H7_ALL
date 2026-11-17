#pragma once
#include "jetson_protocol.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>
namespace jrl {
inline uint64_t monotonic_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Record { uint32_t kind;uint64_t us;std::vector<uint8_t> data; };
/* 有界后台写入。缺失记录单独保存在summary；故障由控制线程执行STOP。 */
class Logger {
    std::filesystem::path path_;std::ofstream stream_;std::thread writer_;
    std::deque<Record> queue_;std::mutex mutex_;std::condition_variable wake_;bool closing_=false;
    std::atomic<bool> failed_{false};std::atomic<uint64_t> dropped_{0},written_{0};
    void loop() noexcept {
        try {
            uint64_t check=0;
            for(;;) {
                Record r;
                { std::unique_lock<std::mutex> lock(mutex_);wake_.wait(lock,[&]{return closing_||!queue_.empty();});
                  if(queue_.empty()) break;
                  r=std::move(queue_.front());queue_.pop_front(); }
                uint8_t h[16];jrl_p32(h,r.kind);jrl_p32(h+4,uint32_t(r.data.size()));jrl_p64(h+8,r.us);
                stream_.write(reinterpret_cast<char*>(h),sizeof(h));stream_.write(reinterpret_cast<const char*>(r.data.data()),r.data.size());written_++;
                if(monotonic_us()-check>=1000000u) {
                    stream_.flush();check=monotonic_us();
                    if(!stream_||std::filesystem::space(path_).available<(1ull<<30)) throw std::runtime_error("log IO/space failure");
                }
            }
            stream_.flush();if(!stream_) failed_=true;
        } catch(...) { failed_=true; }
    }
public:
    explicit Logger(const std::string& path):path_(path) {
        auto parent=path_.parent_path();if(parent.empty()) parent=".";
        if(std::filesystem::space(parent).available<5ull*(1ull<<30)) throw std::runtime_error("logging requires at least 5 GiB available");
        if(!std::filesystem::create_directory(path_)) throw std::runtime_error("refuse to reuse run directory");
        stream_.open(path_/"records.bin",std::ios::binary);if(!stream_) throw std::runtime_error("cannot open log");
        writer_=std::thread([this]{loop();});
    }
    bool failed() const { return failed_; }
    void push(uint32_t kind,const void* data,size_t size,uint64_t us=monotonic_us()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(queue_.size()>=4096||failed_||closing_) { dropped_++;return; }
        const auto* p=static_cast<const uint8_t*>(data);queue_.push_back({kind,us,{p,p+size}});wake_.notify_one();
    }
    void event(const std::string& s) { push(4,s.data(),s.size()); }
    void close() {
        { std::lock_guard<std::mutex> lock(mutex_);closing_=true; }wake_.notify_all();if(writer_.joinable()) writer_.join();
        stream_.close();std::ofstream summary(path_/"summary.txt");summary<<"records="<<written_<<"\ndropped="<<dropped_<<"\nwrite_failed="<<failed_<<"\n";
    }
    ~Logger() { close(); }
};
}
