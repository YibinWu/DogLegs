#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/types.h"

// Dataset view for the uncompressed ROS1 bags produced by bin_to_rosbag.py.
// Instances are cached by path so all six topic loaders share one parse.
class RosbagDataset {
public:
  static std::shared_ptr<const RosbagDataset> load(const std::string &filename);

  const std::vector<IMU> &imu(const std::string &label) const;
  const std::vector<RobotSensor> &robotSensors() const {
    return robot_sensors_;
  }

private:
  explicit RosbagDataset(const std::string &filename);

  std::map<std::string, std::vector<IMU>> imus_;
  std::vector<RobotSensor> robot_sensors_;
};
