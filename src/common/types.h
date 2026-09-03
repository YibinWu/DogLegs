#pragma once

#include <math.h>

#include <Eigen/Geometry>
#include <map>

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);

const int IMU_RATE = 200;
const int RANK = 21;
const int NOISERANK = 18;

const int IMU_NUM = 5;

// imu measurement sliding window length, use the middle one
const int window_length = 11;
const int halfwindow_length = (int)(window_length / 2);

static constexpr double NormG = 9.782940329221166;

namespace Eigen {
using Vector12d = Eigen::Matrix<double, 12, 1>;
}

enum LegID { FL = 0, FR = 1, RL = 2, RR = 3 };

enum StateID {
  P_ID = 0,
  V_ID = 3,
  PHI_ID = 6,
  BG_ID = 9,
  BA_ID = 12,
  SG_ID = 15,
  SA_ID = 18
};
enum NoiseID {
  VRW_ID = 0,
  ARW_ID = 3,
  BGSTD_ID = 6,
  BASTD_ID = 9,
  SGSTD_ID = 12,
  SASTD_ID = 15
};

typedef struct RobotSensor {
  double timestamp = 0.0;
  Eigen::Vector12d joint_angular_position = Eigen::Vector12d::Zero();
  Eigen::Vector12d joint_angular_velocity = Eigen::Vector12d::Zero();
  Eigen::Vector4d footforce = Eigen::Vector4d::Zero();
} RobotSensor;

typedef struct IMU {
  double timestamp = 0.0;
  double dt = 0.0;

  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

} IMU;

typedef struct Attitude {
  Eigen::Quaterniond qbn = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d cbn = Eigen::Matrix3d::Identity();
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();
} Attitude;

typedef struct PVA {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Attitude att;
} PVA;

typedef struct ImuError {
  Eigen::Vector3d gyrbias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accbias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyrscale = Eigen::Vector3d::Zero();
  Eigen::Vector3d accscale = Eigen::Vector3d::Zero();
} ImuError;

typedef struct NavState {
  PVA pva;
  ImuError imuerror;
} NavState;

typedef struct ImuNoise {
  Eigen::Vector3d gyr_arw = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_vrw = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyrbias_std = Eigen::Vector3d::Zero();
  Eigen::Vector3d accbias_std = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyrscale_std = Eigen::Vector3d::Zero();
  Eigen::Vector3d accscale_std = Eigen::Vector3d::Zero();
  double corr_time = 0.0;
} ImuNoise;

typedef struct RobotPara {
  double ox = 0.0;
  double oy = 0.0;
  double ot = 0.0;
  double lc = 0.0;
  double lt = 0.0;
} RobotPara;

typedef struct Paras {
  // initial state and state standard deviation
  NavState initstate;
  NavState initstate_std;

  // imu noise parameters
  ImuNoise imunoise;

  std::vector<Eigen::Vector3d> legimus_leverarm;

  std::vector<Eigen::Vector3d> legimus_mountingangle;

  Eigen::Vector3d zupt_std = Eigen::Vector3d::Zero();
  double zihr_std = 0.0;

  double zupt_dt = 0.0;

  double starttime, endtime = 0.0;

  int imudatarate, imudatalen = 0;

  int initAlignmentTime = 0;

  double relpos_dt = 0.1;
  double relpos_std = 0.05;

  std::string bagpath, outputpath;

  Eigen::Matrix3d robotbody_rotmat = Eigen::Matrix3d::Identity();

  Eigen::Vector3d base_in_bodyimu = Eigen::Vector3d::Zero();

  bool if_use_multi_imu_constraint = false;
  bool if_estimate_leverarm = false;

  RobotPara robotpara;

} Paras;

typedef struct MeasUpdateInfo {
  double timestamp = 0.0;
  Eigen::Vector3d zuptInno = Eigen::Vector3d::Zero();
  Eigen::Vector3d relPosInno = Eigen::Vector3d::Zero();
} MeasUpdateInfo;
