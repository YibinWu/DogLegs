#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

#include "common/types.h"

#include "common/rotation.h"
#include "fileio/filesaver.h"
#include "fileio/imufileloader.h"

class INSPropagation {
public:
  FileSaver navfile_, imuerrfile_, measUpdateEpochfile_, allzuptEpochfile_,
      measUpdateInfofile_;

  std::vector<double> all_zupt_epoch_, meas_update_epoch_, zupt_intevals_,
      meas_update_info_;

public:
  explicit INSPropagation(Paras &options, std::string lable);

  ~INSPropagation() = default;

  inline void setInitGyroBias(const Vector3d &gyro_bias) {
    imuerror_.gyrbias = gyro_bias;
  }
  inline void setInitAttitude(const double &roll, const double &pitch) {
    pvacur_.att.euler(0) = roll;
    pvacur_.att.euler(1) = pitch;
    pvacur_.att.euler(2) = 0.0;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

    pvapre_ = pvacur_;
  }

  double timestamp() const { return imucur_.timestamp; }

  NavState getNavState();
  NavState getPreNavState();

  IMU getImuCur() { return imucur_; }

  void initStaticAlignment();

  void newImuProcess();

  void stateFeedback(const Eigen::MatrixXd &delta_x);

  void writeState();

  void writeMeasUpdateInfo(MeasUpdateInfo &info) {
    std::vector<double> result;
    result.clear();
    result.push_back(info.timestamp);
    result.push_back(info.zuptInno[0]);
    result.push_back(info.zuptInno[1]);
    result.push_back(info.zuptInno[2]);
    result.push_back(info.relPosInno[0]);
    result.push_back(info.relPosInno[1]);
    result.push_back(info.relPosInno[2]);
    measUpdateInfofile_.dump(result);
  }

  std::vector<double> getZUPTintervals() { return zupt_intevals_; }

  bool getZuptFlag() { return if_ZUPT_available_; }
  void setZuptFlag(bool flag) { if_ZUPT_available_ = flag; }
  bool getZIHRFlag() { return if_ZIHR_available_; }
  void setZIHRFlag(bool flag) { if_ZIHR_available_ = flag; }

  int getZIHRNum() { return ZIHR_num_; }
  double getPreHeading() { return zihr_preheading_; }
  void setZIHRPreHeading(double heading) { zihr_preheading_ = heading; }

  void setUpdate_t(double t) { last_velocity_update_t = t; }
  double getLastUpdate_t() { return last_velocity_update_t; }
  std::vector<double> getZUPTepoch() { return all_zupt_epoch_; }
  void setRelPosUpdateT(double t) { last_relpos_update_t = t; }
  double getLastRelPosUpdateT() { return last_relpos_update_t; }

  void updatePreState() {
    pvapre_ = pvacur_;
    imupre_ = imucur_;
  }

private:
  void initialize(const NavState &initstate, const NavState &initstate_std);
  void openOutputFiles();

  void detectZUPT();

  void insPropagation(IMU &imupre, IMU &imucur);

  void addImuData() {
    IMU imu = imufile_.next();
    if (imuBuff_.size() < window_length) {
      imuBuff_.push_back(imu);
    } else {
      imuBuff_.pop_front();
      imuBuff_.push_back(imu);

      imucur_ = imuBuff_[halfwindow_length];
      imupre_ = imuBuff_[halfwindow_length - 1];
    }
  }

  void insMech();

  void imuCompensate();

private:
  Paras paras_;

  std::string label_;

  double last_velocity_update_t = 0.0;
  double last_relpos_update_t = 0.0;

  int ZIHR_num_ = 0;
  double zihr_preheading_ = 0.0;

  ImuFileLoader imufile_;

  IMU imupre_;
  IMU imucur_;

  PVA pvacur_;
  PVA pvapre_;
  ImuError imuerror_;

  std::deque<IMU> imuBuff_;

  bool if_ZUPT_available_ = false;
  bool if_ZIHR_available_ = false;

  int zuptcount_ = 0;

  enum StateID {
    P_ID = 0,
    V_ID = 3,
    PHI_ID = 6,
    BG_ID = 9,
    BA_ID = 12,
    SG_ID = 15,
    SA_ID = 18,
    LegIMU_leverarm_ID = 21
  };
  enum NoiseID {
    VRW_ID = 0,
    ARW_ID = 3,
    BGSTD_ID = 6,
    BASTD_ID = 9,
    SGSTD_ID = 12,
    SASTD_ID = 15,
    LegIMU_leverarm_Noise_ID = 18
  };
};
