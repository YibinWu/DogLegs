#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

#include "common/types.h"

#include "common/rotation.h"
#include "fileio/loadanddump.h"
#include "fileio/robotsensorloader.h"
#include "inspropagation.h"
#include <Eigen/Sparse>
class IMUFusion {
public:
  explicit IMUFusion(Paras &options);

  ~IMUFusion() = default;

  void run();

  double gettimestamp() const { return inspropagation_[1]->timestamp(); }

private:
  void initializePQ(const ImuNoise &imunoise, const NavState &initstate_std);

  void covPropagation();
  void measUpdate();
  void fullStateFeedback();

  void initialization();

  void detectZIHR();

  void checkCov() {
    for (int i = 0; i < full_state_rank_; i++) {
      if (Cov_(i, i) < 0) {
        std::cout << "Covariance is negative at " << std::setprecision(10)
                  << gettimestamp() << " !" << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }

  void writeResults() {
    for (int i = 0; i < IMU_NUM; i++) {
      inspropagation_[i]->writeState();
    }
    double timestamp = gettimestamp();
    writeSTD(timestamp, Cov_, stdfile_);

    std::vector<double> result;

    result.clear();
    result.push_back(timestamp);
    for (int i = 1; i < IMU_NUM; i++) {
      int legid = LegID(i - 1);
      result.push_back(R_wf_wb_[legid].euler[2]);
    }
    relative_heading_file_.dump(result);
  }

  void computeRelFootPosVel(RobotSensor &robotsensor, int legid);
  void EKFUpdate(Eigen::MatrixXd &dz, Eigen::SparseMatrix<double> &H,
                            Eigen::MatrixXd &R, Eigen::MatrixXd &Cov_,
                            Eigen::MatrixXd &delta_x_);

      private : Paras paras_;

  RobotSensor robotsensor_;
  RobotSensorLoader robotsensorfile_;

  INSPropagation bodyins_;
  INSPropagation fl_legins_;
  INSPropagation fr_legins_;
  INSPropagation rl_legins_;
  INSPropagation rr_legins_;

  INSPropagation *inspropagation_[IMU_NUM] = {
      &bodyins_, &fl_legins_, &fr_legins_, &rl_legins_, &rr_legins_};

  Eigen::MatrixXd Cov_;
  Eigen::SparseMatrix<double> Qc_;
  Eigen::MatrixXd delta_x_;

  int full_state_rank_;
  int full_noise_rank_;

  enum IMUID { Body_ID = 0, FL_ID = 1, FR_ID = 2, RL_ID = 3, RR_ID = 4 };

  // Transformation matrix from foot world-frame to body world-frame
  std::vector<Attitude> R_wf_wb_;
  std::vector<Eigen::Vector3d> p_wf_wb_;

  bool if_initialized_ = false;

  std::vector<Eigen::Vector3d> initFootPosinBody;

  FileSaver stdfile_, relative_heading_file_;

  double sequence_interval = 0.0;

  bool measument_updated_ = false;

  Eigen::Matrix<double, 3, 4> foot_pos_rel;
  Eigen::Matrix<double, 3, 4> foot_vel_rel;

  std::vector<std::pair<double, PVA>> bodystate_buffer_;
};