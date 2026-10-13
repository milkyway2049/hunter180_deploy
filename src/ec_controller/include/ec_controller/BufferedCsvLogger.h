// Copyright (c) 2026 Phanerox. All rights reserved.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace ec
{

// All operations run on the calling thread. The stream buffer reduces write calls,
// but formatting and buffer-full disk writes still occur in the control update.
class BufferedCsvLogger
{
public:
  struct Row
  {
    int64_t time_ns{0};
    std::vector<double> values;
  };

  ~BufferedCsvLogger() {stop();}

  bool start(const std::string & path, const std::string & header, size_t columns)
  {
    stop();
    failed_ = false;
    row_.values.assign(columns, 0.0);
    file_.clear();
    file_.rdbuf()->pubsetbuf(buffer_.data(), buffer_.size());
    file_.open(path, std::ios::out | std::ios::trunc);
    file_ << std::setprecision(8) << header;
    file_.flush();
    if (!file_) {
      failed_ = true;
      if (file_.is_open()) {file_.close();}
      return false;
    }
    return true;
  }

  Row * beginRow()
  {
    return file_.is_open() && !failed_ ? &row_ : nullptr;
  }

  void commitRow()
  {
    file_ << row_.time_ns;
    for (double value : row_.values) {file_ << ',' << value;}
    file_ << '\n';
    // Do not flush each row: the stream writes when its buffer fills.
    if (!file_) {failed_ = true;}
  }

  void stop()
  {
    if (file_.is_open()) {
      file_.close();  // Flush buffered output before closing.
      if (!file_) {failed_ = true;}
    }
  }

  bool failed() const {return failed_;}

private:
  Row row_;
  bool failed_{false};
  // Must outlive file_, which retains a pointer to this buffer.
  std::array<char, 64 * 1024> buffer_{};
  std::ofstream file_;
};

}  // namespace ec
