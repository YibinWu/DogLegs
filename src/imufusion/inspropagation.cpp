#include "inspropagation.h"

#include <math.h>

#include <fstream>

#include "common/rotation.h"
#include "common/types.h"
#include "fileio/loadanddump.h"

extern std::ofstream update_;
namespace Eigen {
using Vector6d = Eigen::Matrix<double, 6, 1>;
} // namespace Eigen

static auto square = [](auto x) { return x * x; };

INSPropagation::INSPropagation(Paras &paras, std::string label)
    : paras_(paras), imufile_(paras.bagpath, label, paras.imudatarate) {
  label_ = label;
  initialize(paras_.initstate, paras_.initstate_std);
  openOutputFiles();
}

void INSPropagation::openOutputFiles() {
  // open output stream for individual system
  std::string navfilepath = paras_.outputpath + "/" + label_ + "_traj.txt";
  std::string imuerrorfilepath =
      paras_.outputpath + "/" + label_ + "_imuerror.txt";
  std::string stdfilepath = paras_.outputpath + "/" + label_ + "_std.txt";
  std::string zuptEpochpath =
      paras_.outputpath + "/" + label_ + "_zupt_epoch.txt";
  std::string measUpdateEpochpath =
      paras_.outputpath + "/" + label_ + "_meas_update_epoch.txt";

  navfile_.open(navfilepath, FileSaver::TEXT);
  imuerrfile_.open(imuerrorfilepath, FileSaver::TEXT);

  if (!navfile_.isOpen() || !imuerrfile_.isOpen()) {
    std::cout << label_ + "-IMU: Failed to open output files!" << std::endl;
  }
  measUpdateEpochfile_.open(measUpdateEpochpath, FileSaver::TEXT);
  measUpdateInfofile_.open(paras_.outputpath + "/" + label_ +
                               "_meas_update_info.txt",
                           FileSaver::TEXT);

  // ZUPT ONLY for leg-imus
  if (label_ != "Body") {
    allzuptEpochfile_.open(zuptEpochpath, FileSaver::TEXT);

    if (!allzuptEpochfile_.isOpen() || !measUpdateEpochfile_.isOpen()) {
      std::cout << label_ + "-IMU: Failed to open zupt epochs / measument "
                            "update output file!"
                << std::endl;
    }
  }
}

void INSPropagation::initialize(const NavState &initstate,
                                const NavState &initstate_std) {
  pvacur_.pos = initstate.pva.pos;
  pvacur_.vel = initstate.pva.vel;
  pvacur_.att.euler = initstate.pva.att.euler;
  pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
  pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

  imuerror_ = initstate.imuerror;

  pvapre_ = pvacur_;

  ImuError imuerror_std = initstate_std.imuerror;
}

void INSPropagation::newImuProcess() {

  addImuData();

  imuCompensate();

  insMech();

  // Only the leg-imus detect zupt
  if (label_ != "Body") {
    detectZUPT();
  }
}

void INSPropagation::detectZUPT() {
  if (imuBuff_.size() < window_length) {
    return;
  }

  double sigma_acc = square(0.03);
  double glrt_acc = 0.0;

  Eigen::VectorXd gy, az;
  gy.resize(imuBuff_.size());
  az.resize(imuBuff_.size());

  for (auto it = imuBuff_.begin(); it != imuBuff_.end(); ++it) {
    glrt_acc += square(it->acceleration.norm() - NormG);
    gy(it - imuBuff_.begin()) = it->angular_velocity.y();
    az(it - imuBuff_.begin()) = it->acceleration.norm();
  }

  double gy_std = std::sqrt((gy.array() - gy.mean()).square().mean());
  double az_std = std::sqrt((az.array() - az.mean()).square().mean());

  glrt_acc = glrt_acc / sigma_acc / window_length;

  double abs_y_gyro = abs(imucur_.angular_velocity.y());

  if (glrt_acc < 1200000 && abs_y_gyro < 6.0 && gy_std < 0.3 && az_std < 5.0) {
    zuptcount_++;
    ZIHR_num_++;
    zupt_intevals_.push_back(imucur_.timestamp);
    all_zupt_epoch_.push_back(imucur_.timestamp);
    if (zuptcount_ % 5 == 0) {
      if_ZUPT_available_ = true;
    }
  } else {
    zupt_intevals_.clear();
    zuptcount_ = 0;
    ZIHR_num_ = 0;
    if_ZUPT_available_ = false;
    if_ZIHR_available_ = false;
  }
}

NavState INSPropagation::getNavState() {
  NavState state;

  state.pva = pvacur_;
  state.imuerror = imuerror_;

  return state;
}

NavState INSPropagation::getPreNavState() {
  NavState state;

  state.pva = pvapre_;
  state.imuerror = imuerror_;

  return state;
}

void INSPropagation::initStaticAlignment() {
  do {
    imucur_ = imufile_.next();

  } while (imucur_.timestamp < paras_.starttime);

  int k = 0;
  Vector3d init_gyro_mean = Vector3d::Zero();
  Vector3d init_acc_mean = Vector3d::Zero();

  while (imucur_.timestamp < paras_.starttime + paras_.initAlignmentTime) {

    addImuData();
    init_gyro_mean += imucur_.angular_velocity;
    init_acc_mean += imucur_.acceleration;
    k++;
  }

  init_gyro_mean /= k;
  init_acc_mean /= k;
  double init_roll = atan2(-init_acc_mean[1], -init_acc_mean[2]);
  double init_pitch =
      atan2(init_acc_mean[0], sqrt(init_acc_mean[1] * init_acc_mean[1] +
                                   init_acc_mean[2] * init_acc_mean[2]));
  setInitGyroBias(init_gyro_mean);
  setInitAttitude(init_roll, init_pitch);
}

void INSPropagation::stateFeedback(const Eigen::MatrixXd &delta_x) {
  {
    pvacur_.pos -= delta_x.block(P_ID, 0, 3, 1);
    pvacur_.vel -= delta_x.block(V_ID, 0, 3, 1);

    Vector3d delta_att = delta_x.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(delta_att);
    pvacur_.att.qbn = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

    imuerror_.gyrbias += delta_x.block(BG_ID, 0, 3, 1);
    imuerror_.accbias += delta_x.block(BA_ID, 0, 3, 1);
    imuerror_.gyrscale += delta_x.block(SG_ID, 0, 3, 1);
    imuerror_.accscale += delta_x.block(SA_ID, 0, 3, 1);

    pvapre_ = pvacur_;
  }
}

void INSPropagation::insMech() {

  Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl;
  Eigen::Vector3d temp1, temp2, temp3;
  Eigen::Vector3d imucur_dvel, imucur_dtheta, imupre_dvel, imupre_dtheta;

  imucur_dvel = imucur_.acceleration * imucur_.dt;
  imucur_dtheta = imucur_.angular_velocity * imucur_.dt;
  imupre_dvel = imupre_.acceleration * imupre_.dt;
  imupre_dtheta = imupre_.angular_velocity * imupre_.dt;

  temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
  temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
  temp3 = imupre_dvel.cross(imucur_dtheta) / 12;

  d_vfb = imucur_dvel + temp1 + temp2 + temp3;

  d_vfn = pvapre_.att.cbn * d_vfb;

  gl << 0, 0, NormG;
  d_vgn = gl * imucur_.dt;

  pvacur_.vel = pvapre_.vel + d_vfn + d_vgn;

  Eigen::Vector3d midvel;

  midvel = (pvacur_.vel + pvapre_.vel) / 2;
  pvacur_.pos = pvapre_.pos + midvel * imucur_.dt;

  Eigen::Quaterniond qbb;
  Eigen::Vector3d rot_bframe;

  rot_bframe = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;
  qbb = Rotation::rotvec2quaternion(rot_bframe);

  pvacur_.att.qbn = pvapre_.att.qbn * qbb;
  pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
  pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);
}

void INSPropagation::imuCompensate() {

  imucur_.angular_velocity -= imuerror_.gyrbias;
  imucur_.acceleration -= imuerror_.accbias;

  Eigen::Vector3d gyrscale, accscale;
  gyrscale = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
  accscale = Eigen::Vector3d::Ones() + imuerror_.accscale;
  imucur_.angular_velocity =
      imucur_.angular_velocity.cwiseProduct(gyrscale.cwiseInverse());
  imucur_.acceleration =
      imucur_.acceleration.cwiseProduct(accscale.cwiseInverse());
}

void INSPropagation::writeState() {
  NavState navstate = getNavState();
  writeNavResult(imucur_.timestamp, navstate, navfile_, imuerrfile_);
}
