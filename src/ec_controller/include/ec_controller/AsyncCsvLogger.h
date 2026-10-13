// Copyright (c) 2026 Phanerox. All rights reserved.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>

namespace ec
{

// Single producer (controller update), single consumer (writer thread).
// start/stop must run while the producer is quiescent, as in controller lifecycle callbacks.
class AsyncCsvLogger
{
public:
  struct Row
  {
    int64_t time_ns{0};
    std::vector<double> values;
  };

  ~AsyncCsvLogger() {stop();}

  bool start(const std::string & path, const std::string & header, size_t columns)
  {
    stop();
    head_.store(0);
    tail_.store(0);
    dropped_.store(0);
    failed_.store(false);
    rows_.resize(kCapacity);
    for (auto & row : rows_) {row.values.assign(columns, 0.0);}
    file_.clear();
    file_.open(path, std::ios::out | std::ios::trunc);
    file_ << std::setprecision(8) << header;
    file_.flush();
    if (!file_) {
      failed_.store(true);
      if (file_.is_open()) {file_.close();}
      return false;
    }
    running_.store(true);
    try {
      writer_ = std::thread([this]() {writeLoop();});
    } catch (...) {
      running_.store(false);
      file_.close();
      throw;
    }
    return true;
  }

  // No allocation, locks, formatting, disk access, or waiting in the producer.
  Row * beginRow()
  {
    if (!running_.load(std::memory_order_relaxed) || failed_.load(std::memory_order_relaxed)) {
      return nullptr;
    }
    const size_t head = head_.load(std::memory_order_relaxed);
    if ((head + 1) % kCapacity == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return nullptr;
    }
    return &rows_[head];
  }

  void commitRow()
  {
    head_.store((head_.load(std::memory_order_relaxed) + 1) % kCapacity,
      std::memory_order_release);
  }

  void stop()
  {
    running_.store(false, std::memory_order_release);
    if (writer_.joinable()) {writer_.join();}
    if (file_.is_open()) {
      file_.close();
      if (!file_) {failed_.store(true);}
    }
  }

  uint64_t dropped() const {return dropped_.load(std::memory_order_relaxed);}
  bool failed() const {return failed_.load(std::memory_order_relaxed);}

private:
  void writeLoop()
  {
    // Drain a bounded batch before flushing; stop drains all accepted samples.
    for (;;) {
      const bool running = running_.load(std::memory_order_acquire);
      size_t tail = tail_.load(std::memory_order_relaxed);
      const size_t end = head_.load(std::memory_order_acquire);
      if (!running && tail == end) {break;}
      const bool have_rows = tail != end;
      while (tail != end) {
        const auto & row = rows_[tail];
        file_ << row.time_ns;
        for (double value : row.values) {file_ << ',' << value;}
        file_ << '\n';
        tail = (tail + 1) % kCapacity;
        tail_.store(tail, std::memory_order_release);
      }
      if (have_rows) {
        file_.flush();
        if (!file_) {
          failed_.store(true, std::memory_order_relaxed);
          return;
        }
      }
      if (running) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    }
  }

  static_assert(std::atomic<size_t>::is_always_lock_free);
  static_assert(std::atomic<uint64_t>::is_always_lock_free);
  static constexpr size_t kCapacity = 4096;
  std::vector<Row> rows_;
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<bool> running_{false};
  std::atomic<bool> failed_{false};
  std::ofstream file_;
  std::thread writer_;
};

}  // namespace ec
