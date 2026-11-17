#pragma once
#include "jetson_protocol.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace jrl {
inline uint64_t monotonic_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Record {
    uint32_t kind;
    uint64_t us;
    std::vector<uint8_t> data;
};

/* Keep disk IO off the serial/inference thread. */
class Logger {
    static constexpr uint64_t SegmentUs = 300'000'000;

    std::filesystem::path path_, pending_;
    std::ofstream stream_;
    std::thread writer_;
    std::deque<Record> queue_;
    std::mutex mutex_;
    std::condition_variable wake_;
    bool closing_ = false;
    bool directory_created_ = false;
    std::atomic<bool> failed_{false};
    std::atomic<uint64_t> dropped_{0};
    uint64_t written_ = 0;
    uint64_t segment_started_us_ = 0;
    uint64_t last_flush_us_ = 0;

    void open_segment(uint64_t us) {
        if (!directory_created_) {
            if (!std::filesystem::create_directory(path_)) throw std::runtime_error("refuse to reuse run directory");
            directory_created_ = true;
        }
        using std::chrono::system_clock;
        const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
            system_clock::now().time_since_epoch()).count();
        const std::time_t second = wall_us / 1'000'000;
        std::tm local{};
        if (!localtime_r(&second, &local)) throw std::runtime_error("log timestamp");
        std::ostringstream name;
        name << "records-" << std::put_time(&local, "%Y%m%d-%H%M%S") << '-'
             << std::setfill('0') << std::setw(6) << wall_us % 1'000'000 << '-'
             << us * 1000 << ".partial";
        pending_ = path_ / name.str();
        if (std::filesystem::exists(pending_)) throw std::runtime_error("refuse to overwrite log segment");
        stream_.open(pending_, std::ios::binary);
        if (!stream_) throw std::runtime_error("open log segment");
        segment_started_us_ = last_flush_us_ = us;
    }

    void seal_segment() {
        if (!stream_.is_open()) return;
        stream_.close();
        if (!stream_) throw std::runtime_error("close log segment");
        auto sealed = pending_;
        sealed.replace_extension(".bin");
        if (std::filesystem::exists(sealed)) throw std::runtime_error("refuse to overwrite sealed log");
        std::filesystem::rename(pending_, sealed);
    }

    void write_record(uint32_t kind, uint64_t us, const uint8_t* data, size_t size) {
        if (!stream_.is_open()) open_segment(us);
        if (us >= segment_started_us_ + SegmentUs) {
            seal_segment();
            open_segment(us);
        }
        uint8_t header[16];
        jrl_p32(header, kind);
        jrl_p32(header + 4, uint32_t(size));
        jrl_p64(header + 8, us);
        stream_.write(reinterpret_cast<const char*>(header), sizeof(header));
        stream_.write(reinterpret_cast<const char*>(data), size);
        ++written_;
        if (us >= last_flush_us_ + 1'000'000) {
            stream_.flush();
            last_flush_us_ = us;
            if (!stream_ || std::filesystem::space(path_).available < (1ull << 30))
                throw std::runtime_error("log IO/space failure");
        }
    }

    void loop() noexcept {
        try {
            for (;;) {
                Record record;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    const auto pending = [&] { return closing_ || !queue_.empty(); };
                    wake_.wait(lock, pending);
                    if (queue_.empty()) break;
                    record = std::move(queue_.front());
                    queue_.pop_front();
                }
                write_record(record.kind, record.us, record.data.data(), record.data.size());
            }
            seal_segment();
        } catch (...) {
            failed_ = true;
        }
    }

public:
    explicit Logger(const std::string& path) : path_(path) {
        auto parent = path_.parent_path();
        if (parent.empty()) parent = ".";
        if (std::filesystem::space(parent).available < 5ull * (1ull << 30))
            throw std::runtime_error("logging requires at least 5 GiB available");
        if (std::filesystem::exists(path_)) throw std::runtime_error("refuse to reuse run directory");
        writer_ = std::thread([this] { loop(); });
    }
    bool failed() const { return failed_; }
    void push(uint32_t kind, const void* data, size_t size, uint64_t us=monotonic_us()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= 4096 || failed_ || closing_) {
            ++dropped_;
            return;
        }
        const auto* bytes = static_cast<const uint8_t*>(data);
        queue_.push_back({kind, us, {bytes, bytes + size}});
        wake_.notify_one();
    }
    void event(const std::string& text) { push(4, text.data(), text.size()); }
    void close() {
        if (!writer_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closing_ = true;
        }
        wake_.notify_all();
        writer_.join();
        if (directory_created_) {
            std::ofstream summary(path_ / "summary.txt");
            summary << "records=" << written_ << "\ndropped=" << dropped_
                    << "\nwrite_failed=" << failed_ << '\n';
        }
    }
    ~Logger() { close(); }
};
}
