#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/types.h"
#include "rosbagloader.h"

class ImuFileLoader {
public:
  ImuFileLoader() = delete;
  ImuFileLoader(const std::string &filename, const std::string &label,
                int rate = 200, bool if_increment = false)
      : dataset_(RosbagDataset::load(filename)), data_(&dataset_->imu(label)),
        dt_(1.0 / static_cast<double>(rate)), if_increment_(if_increment) {}

  void set_increment_mode(bool if_increment) { if_increment_ = if_increment; }

  const IMU &next() {
    if (isEof()) {
      throw std::out_of_range("attempted to read past the end of an IMU topic");
    }

    const IMU previous = imu_;
    imu_ = (*data_)[index_++];

    const double dt = imu_.timestamp - previous.timestamp;
    imu_.dt = dt > 0.0 && dt < 0.1 ? dt : dt_;
    if (if_increment_) {
      imu_.angular_velocity /= imu_.dt;
      imu_.acceleration /= imu_.dt;
    }
    return imu_;
  }

  bool isEof() const { return index_ >= data_->size(); }

  double starttime() const { return data_->front().timestamp; }
  double endtime() const { return data_->back().timestamp; }

private:
  std::shared_ptr<const RosbagDataset> dataset_;
  const std::vector<IMU> *data_;
  std::size_t index_ = 0;
  double dt_;
  bool if_increment_;
  IMU imu_;
};
