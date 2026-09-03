#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/types.h"
#include "rosbagloader.h"

class RobotSensorLoader {
public:
  RobotSensorLoader() = delete;
  explicit RobotSensorLoader(const std::string &filename)
      : dataset_(RosbagDataset::load(filename)),
        data_(&dataset_->robotSensors()) {}

  void setRotmat(Eigen::Matrix3d rotmat = Eigen::Matrix3d::Identity()) {
    robot_body_rotmat_ = rotmat;
  }

  const RobotSensor &next() {
    if (isEof()) {
      throw std::out_of_range(
          "attempted to read past the end of the robot sensor topic");
    }
    return (*data_)[index_++];
  }

  bool isEof() const { return index_ >= data_->size(); }

  double starttime() const { return data_->front().timestamp; }
  double endtime() const { return data_->back().timestamp; }

private:
  std::shared_ptr<const RosbagDataset> dataset_;
  const std::vector<RobotSensor> *data_;
  std::size_t index_ = 0;
  Eigen::Matrix3d robot_body_rotmat_ = Eigen::Matrix3d::Identity();
};
