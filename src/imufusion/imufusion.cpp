#include "imufusion.h"

#include <Eigen/Sparse>
#include <absl/time/clock.h>
#include <omp.h>

extern std::ofstream timing;

auto create_covariance_block = [](const auto &v1) {
  return v1.cwiseAbs2().asDiagonal();
};

static auto square = [](auto x) { return x * x; };

IMUFusion::IMUFusion(Paras &paras)
    : paras_(paras), robotsensorfile_(paras.bagpath),
      bodyins_(paras, "Body"), fl_legins_(paras, "FL"), fr_legins_(paras, "FR"),
      rl_legins_(paras, "RL"), rr_legins_(paras, "RR") {

  // relative heading drift between foot-imus and body-imu
  full_state_rank_ = IMU_NUM * RANK + 4;
  full_noise_rank_ = IMU_NUM * NOISERANK + 4;

  Cov_.resize(full_state_rank_, full_state_rank_);
  Qc_.resize(full_noise_rank_, full_noise_rank_);
  delta_x_.resize(full_state_rank_, 1);
  Cov_.setZero();
  Qc_.setZero();
  delta_x_.setZero();

  initializePQ(paras_.imunoise, paras_.initstate_std);

  robotsensorfile_.setRotmat(paras.robotbody_rotmat);

  stdfile_.open(paras.outputpath + "/fullstd.txt", FileSaver::TEXT);
  relative_heading_file_.open(paras.outputpath + "/relative_heading.txt",
                              FileSaver::TEXT);

  sequence_interval =
      paras_.endtime - (paras_.starttime + paras_.initAlignmentTime);
}

void IMUFusion::initializePQ(const ImuNoise &imunoise,
                             const NavState &initstate_std) {
  ImuError imuerror_std = initstate_std.imuerror;
  Cov_.block(P_ID, P_ID, 3, 3) = create_covariance_block(initstate_std.pva.pos);
  Cov_.block(V_ID, V_ID, 3, 3) = create_covariance_block(initstate_std.pva.vel);
  Cov_.block(PHI_ID, PHI_ID, 3, 3) =
      create_covariance_block(initstate_std.pva.att.euler);
  Cov_.block(BG_ID, BG_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov_.block(BA_ID, BA_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
  Cov_.block(SG_ID, SG_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrscale);
  Cov_.block(SA_ID, SA_ID, 3, 3) =
      create_covariance_block(imuerror_std.accscale);

  for (int i = 1; i < IMU_NUM; i++) {
    Cov_.block(i * RANK, i * RANK, RANK, RANK) = Cov_.block(0, 0, RANK, RANK);
  }

  Cov_.block(IMU_NUM * RANK, IMU_NUM * RANK, 4, 4) =
      Eigen::Matrix4d::Identity() * square(0.1 * D2R);

  std::vector<Eigen::Triplet<double>> qc_triplets;
  auto addNoiseBlock = [&](int base, const Eigen::Matrix3d &cov) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        double val = cov(i, j);
        if (std::abs(val) > 1e-12)
          qc_triplets.emplace_back(base + i, base + j, val);
      }
  };

  int noise_block_size = NOISERANK;
  for (int i = 0; i < IMU_NUM; ++i) {
    int offset = i * noise_block_size;

    addNoiseBlock(offset + ARW_ID, create_covariance_block(imunoise.gyr_arw));
    addNoiseBlock(offset + VRW_ID, create_covariance_block(imunoise.acc_vrw));
    addNoiseBlock(offset + BGSTD_ID,
                  2 / imunoise.corr_time *
                      create_covariance_block(imunoise.gyrbias_std));
    addNoiseBlock(offset + BASTD_ID,
                  2 / imunoise.corr_time *
                      create_covariance_block(imunoise.accbias_std));
    addNoiseBlock(offset + SGSTD_ID,
                  2 / imunoise.corr_time *
                      create_covariance_block(imunoise.gyrscale_std));
    addNoiseBlock(offset + SASTD_ID,
                  2 / imunoise.corr_time *
                      create_covariance_block(imunoise.accscale_std));
  }

  for (int i = 0; i < 4; ++i)
    qc_triplets.emplace_back(IMU_NUM * NOISERANK + i, IMU_NUM * NOISERANK + i,
                             1e-6);

  Qc_.resize(full_noise_rank_, full_noise_rank_);
  Qc_.setFromTriplets(qc_triplets.begin(), qc_triplets.end());

  for (int i = 0; i < IMU_NUM; i++) {
    inspropagation_[i]->setUpdate_t(paras_.starttime);
    inspropagation_[i]->setZuptFlag(false);
    inspropagation_[i]->setRelPosUpdateT(paras_.starttime);
  }
}

void IMUFusion::covPropagation() {

  Eigen::SparseMatrix<double> G_sparse, Phi_sparse;
  Phi_sparse.resize(full_state_rank_, full_state_rank_);
  G_sparse.resize(full_state_rank_, full_noise_rank_);

  std::vector<Eigen::Triplet<double>> phi_triplets, g_triplets;

  double corrtime = paras_.imunoise.corr_time;
  Eigen::Matrix3d guassMarkov = -1 / corrtime * Eigen::Matrix3d::Identity();
  double dt = inspropagation_[0]->getImuCur().dt;

  auto addBlock = [](std::vector<Eigen::Triplet<double>> &triplets, int r,
                     int c, const Eigen::Matrix3d &m, double scale = 1.0) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        double val = m(i, j) * scale;
        if (std::abs(val) > 1e-12)
          triplets.emplace_back(r + i, c + j, val);
      }
  };

  for (int i = 0; i < full_state_rank_; ++i)
    phi_triplets.emplace_back(i, i, 1.0);

  for (int i = 0; i < IMU_NUM; i++) {

    const int offset = i * RANK;
    const int noise_offset = i * NOISERANK;

    const auto &rotmat = inspropagation_[i]->getNavState().pva.att.cbn;
    const auto &acc = inspropagation_[i]->getImuCur().acceleration;
    const auto &gyro = inspropagation_[i]->getImuCur().angular_velocity;

    Eigen::Matrix3d skew = Rotation::skewSymmetric(rotmat * acc);

    addBlock(phi_triplets, P_ID + offset, V_ID + offset,
             Eigen::Matrix3d::Identity(), dt);
    addBlock(phi_triplets, V_ID + offset, PHI_ID + offset, skew, dt);
    addBlock(phi_triplets, V_ID + offset, BA_ID + offset, rotmat, dt);
    addBlock(phi_triplets, V_ID + offset, SA_ID + offset,
             rotmat * acc.asDiagonal(), dt);
    addBlock(phi_triplets, PHI_ID + offset, BG_ID + offset, -rotmat, dt);
    addBlock(phi_triplets, PHI_ID + offset, SG_ID + offset,
             -rotmat * gyro.asDiagonal(), dt);
    addBlock(phi_triplets, BG_ID + offset, BG_ID + offset, guassMarkov, dt);
    addBlock(phi_triplets, BA_ID + offset, BA_ID + offset, guassMarkov, dt);
    addBlock(phi_triplets, SG_ID + offset, SG_ID + offset, guassMarkov, dt);
    addBlock(phi_triplets, SA_ID + offset, SA_ID + offset, guassMarkov, dt);

    addBlock(g_triplets, V_ID + offset, VRW_ID + noise_offset, rotmat);
    addBlock(g_triplets, PHI_ID + offset, ARW_ID + noise_offset, rotmat);
    addBlock(g_triplets, BG_ID + offset, BGSTD_ID + noise_offset,
             Eigen::Matrix3d::Identity());
    addBlock(g_triplets, BA_ID + offset, BASTD_ID + noise_offset,
             Eigen::Matrix3d::Identity());
    addBlock(g_triplets, SG_ID + offset, SGSTD_ID + noise_offset,
             Eigen::Matrix3d::Identity());
    addBlock(g_triplets, SA_ID + offset, SASTD_ID + noise_offset,
             Eigen::Matrix3d::Identity());
  }

  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      g_triplets.emplace_back(IMU_NUM * RANK + i, IMU_NUM * NOISERANK + j,
                              (i == j ? 1.0 : 0.0));

  Phi_sparse.setFromTriplets(phi_triplets.begin(), phi_triplets.end());
  G_sparse.setFromTriplets(g_triplets.begin(), g_triplets.end());

  Eigen::SparseMatrix<double> GQ =
      G_sparse * Qc_.selfadjointView<Eigen::Lower>();

  Eigen::SparseMatrix<double> Qd_sparse = (GQ * G_sparse.transpose()) * dt;

  Eigen::SparseMatrix<double> PhiQd = Phi_sparse * Qd_sparse;
  Qd_sparse = (PhiQd * Phi_sparse.transpose() + Qd_sparse) * 0.5;

  const Eigen::MatrixXd PhiCov = (Phi_sparse * Cov_).eval();
  Cov_ = PhiCov * Phi_sparse.transpose() + Qd_sparse;
}

void IMUFusion::measUpdate() {

  for (int i = 1; i < IMU_NUM; i++) {
    double ts = inspropagation_[i]->timestamp();
    std::vector<double> zupt_intervals = inspropagation_[i]->getZUPTintervals();

    int zupt_interval_size = zupt_intervals.size();

    NavState cur_state = inspropagation_[i]->getNavState();

    int legid = LegID(i - 1);

    if (inspropagation_[i]->getZuptFlag()) {

      Eigen::Vector3d vel_leverarm =
          cur_state.pva.att.cbn *
          Rotation::skewSymmetric(
              inspropagation_[i]->getImuCur().angular_velocity) *
          paras_.legimus_leverarm[legid];

      Eigen::MatrixXd Z_zupt = cur_state.pva.vel + vel_leverarm;

      Eigen::SparseMatrix<double> H_zupt_sparse(3, full_state_rank_);
      std::vector<Eigen::Triplet<double>> triplets;

      for (int j = 0; j < 3; ++j) {
        triplets.emplace_back(j, V_ID + i * RANK + j, 1.0);
      }

      H_zupt_sparse.setFromTriplets(triplets.begin(), triplets.end());

      Eigen::MatrixXd R_zupt = create_covariance_block(paras_.zupt_std);
      EKFUpdate(Z_zupt, H_zupt_sparse, R_zupt, Cov_, delta_x_);

      MeasUpdateInfo measupdateinfo;
      measupdateinfo.timestamp = ts;
      measupdateinfo.zuptInno = Z_zupt.block(0, 0, 3, 1);

      // multi-IMU fusion
      if (paras_.if_use_multi_imu_constraint) {

        Eigen::Matrix3d R_wb_b = bodyins_.getNavState().pva.att.cbn.transpose();
        Eigen::Vector3d p_b = bodyins_.getNavState().pva.pos;

        computeRelFootPosVel(robotsensor_, legid);

        // leg-imu's position in body-imu's frame
        Eigen::Vector3d p_f_b =
            foot_pos_rel.col(legid) + paras_.base_in_bodyimu;
        // leg-imu's position in leg-imu world frame
        Eigen::Vector3d p_f = inspropagation_[i]->getNavState().pva.pos;

        Eigen::Vector3d p_bf_wb =
            R_wf_wb_[legid].cbn * p_f + p_wf_wb_[legid] - p_b;
        Eigen::MatrixXd Z_relpos = R_wb_b * p_bf_wb - p_f_b;

        Eigen::SparseMatrix<double> H_relpos_sparse(3, full_state_rank_);
        std::vector<Eigen::Triplet<double>> triplets;

        Eigen::Matrix3d R_bf = R_wf_wb_[legid].cbn;
        Eigen::Vector3d lever = paras_.legimus_leverarm[legid];
        Eigen::Matrix3d skew_bf_lever = Rotation::skewSymmetric(R_bf * lever);
        Eigen::Matrix3d skew_pbf_wb = Rotation::skewSymmetric(p_bf_wb);
        Eigen::Matrix3d R_wb_b_R_bf = R_wb_b * R_bf;

        // Block 1: H_relpos.block(0, P_ID, 3, 3) = -R_wb_b;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            triplets.emplace_back(r, P_ID + c, -R_wb_b(r, c));

        // Block 2: H_relpos.block(0, PHI_ID, 3, 3) = -R_wb_b * skew(p_bf_wb);
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            triplets.emplace_back(r, PHI_ID + c, -(R_wb_b * skew_pbf_wb)(r, c));

        // Block 3: H_relpos.block(0, P_ID + i * RANK, 3, 3) = R_wb_b * R_bf;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            triplets.emplace_back(r, P_ID + i * RANK + c, R_wb_b_R_bf(r, c));

        // Block 4: H_relpos.block(0, PHI_ID + i * RANK, 3, 3) = R_wb_b * R_bf *
        // skew(R_bf * lever);
        // for (int r = 0; r < 3; ++r)
        //   for (int c = 0; c < 3; ++c)
        //     triplets.emplace_back(r, PHI_ID + i * RANK + c,
        //                           (R_wb_b * R_bf * skew_bf_lever)(r, c));

        // Block 5: H_relpos.block(0, IMU_NUM * RANK + legid, 3, 1) = (R_wb_b *
        // skew(R_bf * p_f)).col(2);
        Eigen::Vector3d col2 =
            (R_wb_b * Rotation::skewSymmetric(R_bf * p_f)).col(2);
        for (int r = 0; r < 3; ++r)
          triplets.emplace_back(r, IMU_NUM * RANK + legid, col2(r));

        H_relpos_sparse.setFromTriplets(triplets.begin(), triplets.end());
        Eigen::MatrixXd R_relpos = create_covariance_block(
            paras_.relpos_std * Eigen::Vector3d::Ones());

        EKFUpdate(Z_relpos, H_relpos_sparse, R_relpos, Cov_, delta_x_);
        inspropagation_[i]->setRelPosUpdateT(robotsensor_.timestamp);

        measupdateinfo.relPosInno = Z_relpos;
      }
      // set zupt flag to false after update
      inspropagation_[i]->setZuptFlag(false);
      inspropagation_[i]->setZIHRFlag(false);
      inspropagation_[i]->setUpdate_t(ts);
      inspropagation_[i]->meas_update_epoch_.push_back(ts);

      bodyins_.meas_update_epoch_.push_back(ts);
      bodyins_.writeMeasUpdateInfo(measupdateinfo);

      measument_updated_ = true;
      inspropagation_[i]->writeMeasUpdateInfo(measupdateinfo);
    }
  }
}

void IMUFusion::fullStateFeedback() {
  if (measument_updated_ == false)
    return;

  for (int i = 0; i < IMU_NUM; i++) {
    Eigen::Matrix<double, 21, 1> deltax_block =
        delta_x_.block(i * RANK, 0, RANK, 1);
    inspropagation_[i]->stateFeedback(deltax_block);
  }
  for (int i = 0; i < IMU_NUM - 1; i++) {

    Eigen::Vector3d delta_att;
    delta_att << 0, 0, delta_x_(IMU_NUM * RANK + i, 0);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(delta_att);
    R_wf_wb_[i].qbn = qpn * R_wf_wb_[i].qbn;
    R_wf_wb_[i].cbn = Rotation::quaternion2matrix(R_wf_wb_[i].qbn);
    R_wf_wb_[i].euler = Rotation::matrix2euler(R_wf_wb_[i].cbn);
  }

  delta_x_.setZero();
  measument_updated_ = false;
}

void IMUFusion::run() {

  initialization();

  auto start_t = absl::Now();

  int percent = 0, lastpercent = 0;
  double cur_ts = gettimestamp();
  while (cur_ts < paras_.endtime) {

    if (robotsensor_.timestamp < cur_ts && !robotsensorfile_.isEof()) {
      robotsensor_ = robotsensorfile_.next();
    }

    for (int i = 0; i < IMU_NUM; i++) {
      inspropagation_[i]->newImuProcess();
    }

    // try buffer robotsensor_
    robotsensor_ = robotsensorfile_.next();

    cur_ts = gettimestamp();
    covPropagation();
    measUpdate();

    fullStateFeedback();
    checkCov();
    writeResults();

    // check zihr availability
    detectZIHR();

    for (int i = 0; i < IMU_NUM; i++) {
      inspropagation_[i]->updatePreState();
    }

    percent = int((cur_ts - paras_.starttime - paras_.initAlignmentTime) /
                  sequence_interval * 100);
    if (percent - lastpercent >= 1) {
      std::cout << "Processing: " << std::setw(3) << percent << "%\r"
                << std::flush;
      lastpercent = percent;
    }

  }
  stdfile_.close();
  relative_heading_file_.close();

  for (int i = 0; i < IMU_NUM; i++) {
    if (!inspropagation_[i]->meas_update_epoch_.empty()) {
      inspropagation_[i]->measUpdateEpochfile_.dump(
          inspropagation_[i]->meas_update_epoch_);
    }
    inspropagation_[i]->measUpdateEpochfile_.close();

    if (i > 0) {
      inspropagation_[i]->allzuptEpochfile_.dump(
          inspropagation_[i]->all_zupt_epoch_);
      inspropagation_[i]->allzuptEpochfile_.close();
    }
    inspropagation_[i]->navfile_.close();
    inspropagation_[i]->imuerrfile_.close();
  }

  auto end_t = absl::Now();

  std::cout << std::endl << std::endl << "DogLegs Process Finish! ";
  std::cout << "From " << (paras_.starttime + paras_.initAlignmentTime)
            << " s to " << paras_.endtime << " s, total " << sequence_interval
            << " s! " << absl::ToDoubleSeconds(end_t - start_t)
            << " s used in total!" << std::endl;
}

void IMUFusion::detectZIHR() {

  for (int i = 0; i < IMU_NUM; i++) {

    int zihr_num = inspropagation_[i]->getZIHRNum();
    double last_update_t = inspropagation_[i]->getLastUpdate_t();
    double cur_ts = inspropagation_[i]->timestamp();
    bool zihr_flag = inspropagation_[i]->getZIHRFlag();

    if (zihr_num >= 2 && last_update_t == cur_ts && !zihr_flag) {
      inspropagation_[i]->setZIHRFlag(true);
      double preheading = inspropagation_[i]->getNavState().pva.att.euler[2];

      inspropagation_[i]->setZIHRPreHeading(preheading);
    }
  }
}

void IMUFusion::initialization() {

  for (int i = 0; i < IMU_NUM; i++) {
    inspropagation_[i]->initStaticAlignment();
    inspropagation_[i]->setZIHRPreHeading(
        inspropagation_[i]->getNavState().pva.att.euler[2]);
  }

  // get the relative position of the body-imu and the leg-imus
  do {
    robotsensor_ = robotsensorfile_.next();
  } while (robotsensor_.timestamp < paras_.starttime);

  int k = 0;

  foot_pos_rel.setZero();
  foot_vel_rel.setZero();

  Eigen::Matrix<double, 3, 4> footposition_inbody;
  footposition_inbody.setZero();

  while (robotsensor_.timestamp < paras_.starttime + paras_.initAlignmentTime) {

    for (int i = 1; i < IMU_NUM; i++) {
      computeRelFootPosVel(robotsensor_, i - 1);
    }
    footposition_inbody += foot_pos_rel;
    robotsensor_ = robotsensorfile_.next();
    k++;
  }
  footposition_inbody /= k;

  for (int i = 0; i < 4; i++) {
    p_wf_wb_.push_back(footposition_inbody.col(i) + paras_.base_in_bodyimu);
    Attitude R_wf_wb;
    R_wf_wb_.push_back(R_wf_wb);
  }
}

void IMUFusion::computeRelFootPosVel(RobotSensor &robotsensor, int legid) {

  int lfoot = -1, ffoot = -1;
  if (legid < 2)
    ffoot = 1;
  if (legid == 0 || legid == 2)
    lfoot = 1;

  double ox = paras_.robotpara.ox;
  double oy = paras_.robotpara.oy;
  double ot = paras_.robotpara.ot;
  double lc = paras_.robotpara.lc - paras_.legimus_leverarm[legid](2);
  double lt = paras_.robotpara.lt;

  Eigen::Vector3d joint_angular_position =
      robotsensor.joint_angular_position.segment(legid * 3, 3);
  Eigen::Vector3d joint_angular_velocity =
      robotsensor.joint_angular_velocity.segment(legid * 3, 3);

  double s1 = sin(joint_angular_position(0));
  double s2 = sin(joint_angular_position(1));
  double s23 = sin(joint_angular_position(1) + joint_angular_position(2));

  double c1 = cos(joint_angular_position(0));
  double c2 = cos(joint_angular_position(1));
  double c23 = cos(joint_angular_position(1) + joint_angular_position(2));

  foot_pos_rel(0, legid) = -lt * s2 - lc * s23 + ffoot * ox;
  foot_pos_rel(1, legid) =
      lfoot * ot * c1 + lc * s1 * c23 + lt * c2 * s1 + lfoot * oy;
  foot_pos_rel(2, legid) = lfoot * ot * s1 - lc * c1 * c23 - lt * c1 * c2;

  Eigen::Matrix3d J;
  J(0, 0) = 0.0;
  J(0, 1) = -lc * c23 - lt * c2;
  J(0, 2) = -lc * c23;
  J(1, 0) = lt * c1 * c2 - lfoot * ot * s1 + lc * c1 * c23;
  J(1, 1) = -s1 * (lc * s23 + lt * s2);
  J(1, 2) = -lc * s23 * s1;
  J(2, 0) = lt * c2 * s1 + lfoot * ot * c1 + lc * s1 * c23;
  J(2, 1) = c1 * (lc * s23 + lt * s2);
  J(2, 2) = lc * s23 * c1;

  foot_vel_rel(0, legid) =
      J(0, 1) * joint_angular_velocity(1) + J(0, 2) * joint_angular_velocity(2);
  foot_vel_rel(1, legid) = J(1, 0) * joint_angular_velocity(0) +
                           J(1, 1) * joint_angular_velocity(1) +
                           J(1, 2) * joint_angular_velocity(2);
  foot_vel_rel(2, legid) = J(2, 0) * joint_angular_velocity(0) +
                           J(2, 1) * joint_angular_velocity(1) +
                           J(2, 2) * joint_angular_velocity(2);

  foot_pos_rel.block(0, legid, 3, 1) =
      paras_.robotbody_rotmat * foot_pos_rel.block(0, legid, 3, 1);
  foot_vel_rel.block(0, legid, 3, 1) =
      paras_.robotbody_rotmat * foot_vel_rel.block(0, legid, 3, 1);
}

void IMUFusion::EKFUpdate(Eigen::MatrixXd &dz, Eigen::SparseMatrix<double> &H,
                          Eigen::MatrixXd &R, Eigen::MatrixXd &Cov_,
                          Eigen::MatrixXd &delta_x_) {
  assert(H.cols() == Cov_.rows());
  assert(dz.rows() == H.rows());
  assert(dz.rows() == R.rows());
  assert(dz.cols() == 1);

  Eigen::MatrixXd S = H * Cov_ * H.transpose();
  S += R;

  // Step 2: Kalman gain
  Eigen::LLT<Eigen::MatrixXd> llt(S); // Cholesky decomposition
  Eigen::MatrixXd K =
      Cov_ * H.transpose() * llt.solve(Eigen::Matrix3d::Identity());

  // Step 3: State update
  delta_x_ += K * (dz - H * delta_x_);

  // Step 4: Covariance update (Joseph form is more stable)
  Eigen::MatrixXd I_KH =
      Eigen::MatrixXd::Identity(Cov_.rows(), Cov_.cols()) - K * H;
  Cov_ = I_KH * Cov_;
}
