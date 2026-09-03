#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <filesystem>
#include <iomanip>
#include <iostream>

#include "common/types.h"
#include "filesaver.h"

inline void createOutputDir(std::string &outputpath) {
  try {
    std::filesystem::create_directories(outputpath);
  } catch (const std::filesystem::filesystem_error &exception) {
    std::cerr << "Failed to create output directory '" << outputpath
              << "': " << exception.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

inline bool getFiles(YAML::Node &config, Paras &paras) {
  try {
    paras.bagpath = config["bagpath"].as<std::string>();
    paras.outputpath = config["outputpath"].as<std::string>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration: " << exception.what()
              << std::endl;
    return false;
  }
  createOutputDir(paras.outputpath);
  return true;
}

inline bool loadConfig(YAML::Node &config, Paras &paras) {
  std::vector<double> initposstd_vec, initvelstd_vec, initattstd_vec;

  try {
    initposstd_vec = config["initposstd"].as<std::vector<double>>();
    initvelstd_vec = config["initvelstd"].as<std::vector<double>>();
    initattstd_vec = config["initattstd"].as<std::vector<double>>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check initial std "
                 "of position, velocity, and attitude!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.initstate_std.pva.pos[i] = initposstd_vec[i];
    paras.initstate_std.pva.vel[i] = initvelstd_vec[i];
    paras.initstate_std.pva.att.euler[i] = initattstd_vec[i] * D2R;
  }

  double arw, vrw, gbstd, abstd, gsstd, asstd;

  try {
    arw = config["imunoise"]["arw"].as<double>();
    vrw = config["imunoise"]["vrw"].as<double>();
    gbstd = config["imunoise"]["gbstd"].as<double>();
    abstd = config["imunoise"]["abstd"].as<double>();
    gsstd = config["imunoise"]["gsstd"].as<double>();
    asstd = config["imunoise"]["asstd"].as<double>();
    paras.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check IMU noise!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.imunoise.gyr_arw[i] = arw * (D2R / 60.0);
    paras.imunoise.acc_vrw[i] = vrw / 60.0;
    paras.imunoise.gyrbias_std[i] = gbstd * (D2R / 3600.0);
    paras.imunoise.accbias_std[i] = abstd * 1e-5;
    paras.imunoise.gyrscale_std[i] = gsstd * 1e-6;
    paras.imunoise.accscale_std[i] = asstd * 1e-6;

    paras.initstate_std.imuerror.gyrbias[i] = gbstd * (D2R / 3600.0);
    paras.initstate_std.imuerror.accbias[i] = abstd * 1e-5;
    paras.initstate_std.imuerror.gyrscale[i] = gsstd * 1e-6;
    paras.initstate_std.imuerror.accscale[i] = asstd * 1e-6;
  }

  paras.imunoise.corr_time *= 3600;

  std::vector<double> leverArm, mountingangle, zupt_std;

  double odo_update_interval, wheelradius;
  bool ifCompensateVelocity;

  leverArm = config["fl_imu_leverarm"].as<std::vector<double>>();
  paras.legimus_leverarm.push_back(Eigen::Vector3d(leverArm.data()));
  leverArm = config["fr_imu_leverarm"].as<std::vector<double>>();
  paras.legimus_leverarm.push_back(Eigen::Vector3d(leverArm.data()));
  leverArm = config["rl_imu_leverarm"].as<std::vector<double>>();
  paras.legimus_leverarm.push_back(Eigen::Vector3d(leverArm.data()));
  leverArm = config["rr_imu_leverarm"].as<std::vector<double>>();
  paras.legimus_leverarm.push_back(Eigen::Vector3d(leverArm.data()));

  mountingangle = config["fl_imu_mountingangle"].as<std::vector<double>>();
  paras.legimus_mountingangle.push_back(Eigen::Vector3d(mountingangle.data()));
  mountingangle = config["fr_imu_mountingangle"].as<std::vector<double>>();
  paras.legimus_mountingangle.push_back(Eigen::Vector3d(mountingangle.data()));
  mountingangle = config["rl_imu_mountingangle"].as<std::vector<double>>();
  paras.legimus_mountingangle.push_back(Eigen::Vector3d(mountingangle.data()));
  mountingangle = config["rr_imu_mountingangle"].as<std::vector<double>>();
  paras.legimus_mountingangle.push_back(Eigen::Vector3d(mountingangle.data()));

  zupt_std = config["zupt_std"].as<std::vector<double>>();

  paras.zupt_std = Eigen::Vector3d(zupt_std.data());

  paras.zihr_std = config["zihr_std"].as<double>();

  paras.zupt_dt = config["zupt_dt"].as<double>();

  paras.starttime = config["starttime"].as<double>();

  paras.endtime = config["endtime"].as<double>();

  paras.initAlignmentTime = config["initAlignmentTime"].as<int>();

  paras.imudatalen = config["imudatalen"].as<int>();
  paras.imudatarate = config["imudatarate"].as<int>();

  std::vector<double> robot_b_rotmat_vec;
  robot_b_rotmat_vec = config["rotmat"].as<std::vector<double>>();
  paras.robotbody_rotmat =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          robot_b_rotmat_vec.data());

  if (!getFiles(config, paras)) {
    std::cout << "Failed to get bag and output paths!" << std::endl;
    return false;
  }

  std::vector<double> base_in_bodyimu_vec;

  base_in_bodyimu_vec = config["base_in_bodyimu"].as<std::vector<double>>();
  paras.base_in_bodyimu = Eigen::Vector3d(base_in_bodyimu_vec.data());

  paras.robotpara.ox = config["robotpara"]["ox"].as<double>();
  paras.robotpara.oy = config["robotpara"]["oy"].as<double>();
  paras.robotpara.ot = config["robotpara"]["ot"].as<double>();
  paras.robotpara.lc = config["robotpara"]["lc"].as<double>();
  paras.robotpara.lt = config["robotpara"]["lt"].as<double>();

  paras.if_use_multi_imu_constraint =
      config["if_use_multi_imu_constraint"].as<bool>();
  paras.relpos_std = config["relpos_std"].as<double>();
  paras.relpos_dt = config["relpos_dt"].as<double>();

  return true;
}

inline void writeNavResult(double time, NavState &navstate, FileSaver &navfile,
                           FileSaver &imuerrfile) {
  std::vector<double> result;

  result.clear();
  result.push_back(time);
  result.push_back(navstate.pva.pos[0]);
  result.push_back(navstate.pva.pos[1]);
  result.push_back(navstate.pva.pos[2]);
  result.push_back(navstate.pva.vel[0]);
  result.push_back(navstate.pva.vel[1]);
  result.push_back(navstate.pva.vel[2]);
  result.push_back(navstate.pva.att.euler[0] * R2D);
  result.push_back(navstate.pva.att.euler[1] * R2D);
  result.push_back(navstate.pva.att.euler[2] * R2D);
  navfile.dump(result);

  auto imuerr = navstate.imuerror;
  result.clear();
  result.push_back(time);
  result.push_back(imuerr.gyrbias[0] * R2D * 3600);
  result.push_back(imuerr.gyrbias[1] * R2D * 3600);
  result.push_back(imuerr.gyrbias[2] * R2D * 3600);
  result.push_back(imuerr.accbias[0] * 1e5);
  result.push_back(imuerr.accbias[1] * 1e5);
  result.push_back(imuerr.accbias[2] * 1e5);
  result.push_back(imuerr.gyrscale[0] * 1e6);
  result.push_back(imuerr.gyrscale[1] * 1e6);
  result.push_back(imuerr.gyrscale[2] * 1e6);
  result.push_back(imuerr.accscale[0] * 1e6);
  result.push_back(imuerr.accscale[1] * 1e6);
  result.push_back(imuerr.accscale[2] * 1e6);
  imuerrfile.dump(result);
}

inline void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile) {
  std::vector<double> result;

  result.clear();
  result.push_back(time);

  for (int k = 0; k < IMU_NUM; k++) {
    int idx = k * RANK;
    for (int i = 0 + idx; i < 6 + idx; i++) {
      result.push_back(sqrt(cov(i, i)));
    }
    for (int i = 6 + idx; i < 9 + idx; i++) {
      result.push_back(sqrt(cov(i, i)) * R2D);
    }

    for (int i = 9 + idx; i < 12 + idx; i++) {
      result.push_back(sqrt(cov(i, i)) * R2D * 3600);
    }
    for (int i = 12 + idx; i < 15 + idx; i++) {
      result.push_back(sqrt(cov(i, i)) * 1e5);
    }
    for (int i = 15 + idx; i < 21 + idx; i++) {
      result.push_back(sqrt(cov(i, i)) * 1e6);
    }
  }
  stdfile.dump(result);
}

inline void writeMeasUpdateInfo(MeasUpdateInfo &info, FileSaver &file) {
  std::vector<double> result;
  result.clear();
  result.push_back(info.timestamp);
  result.push_back(info.zuptInno[0]);
  result.push_back(info.zuptInno[1]);
  result.push_back(info.zuptInno[2]);
  result.push_back(info.relPosInno[0]);
  result.push_back(info.relPosInno[1]);
  result.push_back(info.relPosInno[2]);
  file.dump(result);
}
