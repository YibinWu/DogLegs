#include "rosbagloader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace {

using Fields = std::unordered_map<std::string, std::string>;

class Cursor {
public:
  explicit Cursor(const std::vector<std::uint8_t> &data)
      : data_(data.data()), size_(data.size()) {}

  std::size_t remaining() const { return size_ - pos_; }

  std::uint32_t uint32() {
    std::uint32_t value = scalar<std::uint32_t>();
    return value;
  }

  std::uint8_t uint8() { return scalar<std::uint8_t>(); }

  double float64() { return scalar<double>(); }

  std::string string() {
    const auto length = uint32();
    require(length);
    std::string value(reinterpret_cast<const char *>(data_ + pos_), length);
    pos_ += length;
    return value;
  }

  std::vector<std::uint8_t> bytes(std::size_t length) {
    require(length);
    std::vector<std::uint8_t> result(data_ + pos_, data_ + pos_ + length);
    pos_ += length;
    return result;
  }

  void skip(std::size_t length) {
    require(length);
    pos_ += length;
  }

private:
  template <typename T> T scalar() {
    require(sizeof(T));
    T value;
    std::memcpy(&value, data_ + pos_, sizeof(T));
    pos_ += sizeof(T);
    return value;
  }

  void require(std::size_t length) const {
    if (length > remaining()) {
      throw std::runtime_error("truncated ROS bag record");
    }
  }

  const std::uint8_t *data_;
  std::size_t size_;
  std::size_t pos_ = 0;
};

struct Record {
  Fields header;
  std::vector<std::uint8_t> data;
};

Fields parseHeader(const std::vector<std::uint8_t> &raw) {
  Fields fields;
  Cursor cursor(raw);
  while (cursor.remaining() > 0) {
    const auto field_length = cursor.uint32();
    auto field = cursor.bytes(field_length);
    const auto separator = std::find(field.begin(), field.end(), '=');
    if (separator == field.end()) {
      throw std::runtime_error("invalid ROS bag header field");
    }
    const std::string name(field.begin(), separator);
    const std::string value(separator + 1, field.end());
    fields[name] = value;
  }
  return fields;
}

template <typename T> T fieldScalar(const Fields &fields, const char *name) {
  const auto iter = fields.find(name);
  if (iter == fields.end() || iter->second.size() != sizeof(T)) {
    throw std::runtime_error(std::string("invalid ROS bag header field: ") +
                             name);
  }
  T value;
  std::memcpy(&value, iter->second.data(), sizeof(T));
  return value;
}

std::string fieldString(const Fields &fields, const char *name) {
  const auto iter = fields.find(name);
  if (iter == fields.end()) {
    throw std::runtime_error(std::string("missing ROS bag header field: ") +
                             name);
  }
  return iter->second;
}

bool readUint32(std::istream &stream, std::uint32_t &value) {
  stream.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (stream.gcount() == 0 && stream.eof()) {
    return false;
  }
  if (!stream) {
    throw std::runtime_error("truncated ROS bag file");
  }
  return true;
}

std::vector<std::uint8_t> readBytes(std::istream &stream,
                                    std::uint32_t length) {
  std::vector<std::uint8_t> data(length);
  stream.read(reinterpret_cast<char *>(data.data()), length);
  if (!stream) {
    throw std::runtime_error("truncated ROS bag file");
  }
  return data;
}

bool readRecord(std::istream &stream, Record &record) {
  std::uint32_t header_length;
  if (!readUint32(stream, header_length)) {
    return false;
  }
  record.header = parseHeader(readBytes(stream, header_length));

  std::uint32_t data_length;
  if (!readUint32(stream, data_length)) {
    throw std::runtime_error("missing ROS bag record data length");
  }
  record.data = readBytes(stream, data_length);
  return true;
}

bool readRecord(Cursor &cursor, Record &record) {
  if (cursor.remaining() == 0) {
    return false;
  }
  const auto header_length = cursor.uint32();
  record.header = parseHeader(cursor.bytes(header_length));
  record.data = cursor.bytes(cursor.uint32());
  return true;
}

IMU parseImu(const std::vector<std::uint8_t> &raw, std::uint64_t bag_time) {
  Cursor cursor(raw);
  cursor.skip(sizeof(std::uint32_t)); // Header.seq
  const auto sec = cursor.uint32();
  const auto nsec = cursor.uint32();
  cursor.skip(cursor.uint32()); // Header.frame_id

  cursor.skip(4 * sizeof(double)); // orientation
  cursor.skip(9 * sizeof(double)); // orientation covariance

  IMU imu;
  imu.timestamp = static_cast<double>(bag_time) * 1e-9;
  for (int i = 0; i < 3; ++i) {
    imu.angular_velocity[i] = cursor.float64();
  }
  cursor.skip(9 * sizeof(double)); // angular velocity covariance
  for (int i = 0; i < 3; ++i) {
    imu.acceleration[i] = cursor.float64();
  }

  const double header_time = static_cast<double>(sec) + nsec * 1e-9;
  if (std::abs(header_time - imu.timestamp) > 5e-9) {
    throw std::runtime_error("IMU header and bag timestamps differ");
  }
  return imu;
}

RobotSensor parseRobotSensor(const std::vector<std::uint8_t> &raw,
                             std::uint64_t bag_time) {
  Cursor cursor(raw);
  const auto dimensions = cursor.uint32();
  for (std::uint32_t i = 0; i < dimensions; ++i) {
    cursor.string();
    cursor.skip(2 * sizeof(std::uint32_t)); // size and stride
  }
  cursor.skip(sizeof(std::uint32_t)); // layout.data_offset

  const auto values = cursor.uint32();
  if (values != 29) {
    throw std::runtime_error("robot sensor message must contain 29 doubles");
  }

  RobotSensor sensor;
  sensor.timestamp = cursor.float64();
  for (int i = 0; i < 12; ++i) {
    sensor.joint_angular_position[i] = cursor.float64();
  }
  for (int i = 0; i < 12; ++i) {
    sensor.joint_angular_velocity[i] = cursor.float64();
  }
  for (int i = 0; i < 4; ++i) {
    sensor.footforce[i] = cursor.float64();
  }

  const double message_time = static_cast<double>(bag_time) * 1e-9;
  if (std::abs(sensor.timestamp - message_time) > 5e-9) {
    throw std::runtime_error("robot sensor and bag timestamps differ");
  }
  return sensor;
}

std::uint64_t fieldTime(const Fields &fields) {
  const auto iter = fields.find("time");
  if (iter == fields.end() || iter->second.size() != 8) {
    throw std::runtime_error("invalid ROS bag message timestamp");
  }
  std::uint32_t sec;
  std::uint32_t nsec;
  std::memcpy(&sec, iter->second.data(), sizeof(sec));
  std::memcpy(&nsec, iter->second.data() + sizeof(sec), sizeof(nsec));
  return static_cast<std::uint64_t>(sec) * 1000000000ULL + nsec;
}

std::string labelForTopic(const std::string &topic) {
  static const std::unordered_map<std::string, std::string> labels = {
      {"/doglegs/imu/body", "Body"}, {"/doglegs/imu/fl", "FL"},
      {"/doglegs/imu/fr", "FR"},     {"/doglegs/imu/rl", "RL"},
      {"/doglegs/imu/rr", "RR"},
  };
  const auto iter = labels.find(topic);
  return iter == labels.end() ? std::string() : iter->second;
}

} // namespace

std::shared_ptr<const RosbagDataset>
RosbagDataset::load(const std::string &filename) {
  static std::mutex mutex;
  static std::unordered_map<std::string, std::weak_ptr<const RosbagDataset>>
      cache;

  const auto key =
      std::filesystem::absolute(filename).lexically_normal().string();
  std::lock_guard<std::mutex> lock(mutex);
  if (const auto cached = cache[key].lock()) {
    return cached;
  }
  auto dataset =
      std::shared_ptr<const RosbagDataset>(new RosbagDataset(filename));
  cache[key] = dataset;
  return dataset;
}

RosbagDataset::RosbagDataset(const std::string &filename) {
  std::ifstream stream(filename, std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open ROS bag: " + filename);
  }

  std::string version;
  std::getline(stream, version);
  if (version != "#ROSBAG V2.0") {
    throw std::runtime_error("unsupported ROS bag version in: " + filename);
  }

  struct Connection {
    std::string topic;
    std::string type;
  };
  std::unordered_map<std::uint32_t, Connection> connections;

  auto handleRecord = [&](const Record &record) {
    constexpr std::uint8_t kConnection = 7;
    constexpr std::uint8_t kMessage = 2;
    const auto operation = fieldScalar<std::uint8_t>(record.header, "op");
    if (operation == kConnection) {
      const auto id = fieldScalar<std::uint32_t>(record.header, "conn");
      const auto metadata = parseHeader(record.data);
      connections[id] =
          {fieldString(record.header, "topic"), fieldString(metadata, "type")};
      return;
    }
    if (operation != kMessage) {
      return;
    }

    const auto id = fieldScalar<std::uint32_t>(record.header, "conn");
    const auto connection = connections.find(id);
    if (connection == connections.end()) {
      throw std::runtime_error("ROS bag message precedes its connection");
    }
    const auto time = fieldTime(record.header);
    const auto label = labelForTopic(connection->second.topic);
    if (!label.empty()) {
      if (connection->second.type != "sensor_msgs/Imu") {
        throw std::runtime_error("unexpected IMU message type");
      }
      imus_[label].push_back(parseImu(record.data, time));
    } else if (connection->second.topic == "/doglegs/robot_sensor") {
      if (connection->second.type != "std_msgs/Float64MultiArray") {
        throw std::runtime_error("unexpected robot sensor message type");
      }
      robot_sensors_.push_back(parseRobotSensor(record.data, time));
    }
  };

  Record top_record;
  while (readRecord(stream, top_record)) {
    constexpr std::uint8_t kChunk = 5;
    const auto operation =
        fieldScalar<std::uint8_t>(top_record.header, "op");
    if (operation == kChunk) {
      const auto compression = fieldString(top_record.header, "compression");
      if (compression != "none") {
        throw std::runtime_error("compressed ROS bags are not supported: " +
                                 compression);
      }
      Cursor chunk(top_record.data);
      Record chunk_record;
      while (readRecord(chunk, chunk_record)) {
        handleRecord(chunk_record);
      }
    } else if (operation == 7) {
      handleRecord(top_record);
    }
  }

  const std::size_t count = robot_sensors_.size();
  if (count == 0) {
    throw std::runtime_error("ROS bag contains no /doglegs/robot_sensor data");
  }
  for (const auto *label : {"Body", "FL", "FR", "RL", "RR"}) {
    const auto iter = imus_.find(label);
    if (iter == imus_.end() || iter->second.size() != count) {
      throw std::runtime_error(std::string("ROS bag topic count mismatch: ") +
                               label);
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (std::abs(iter->second[i].timestamp - robot_sensors_[i].timestamp) >
          5e-9) {
        throw std::runtime_error(std::string("ROS bag timestamps differ: ") +
                                 label);
      }
    }
  }
}

const std::vector<IMU> &RosbagDataset::imu(const std::string &label) const {
  const auto iter = imus_.find(label);
  if (iter == imus_.end()) {
    throw std::runtime_error("unknown IMU label: " + label);
  }
  return iter->second;
}
