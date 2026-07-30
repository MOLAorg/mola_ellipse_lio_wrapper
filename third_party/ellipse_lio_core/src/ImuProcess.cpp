// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's src/imu_processing.cpp
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. See ImuProcess.h and
// THIRD_PARTY_NOTICES.md for what was ported/removed.
#include <ellipse_lio_core/ImuProcess.h>
#include <ikfom/so3_math.h>

namespace ellipse_lio_core
{
ImuProcess::~ImuProcess() {}

ImuProcess::ImuProcess(IkfomSPtr kf, ImuParams params, LogFn log)
: kf_(kf), params_(params), log_(std::move(log)), imu_states_(params.rate)
{
  acc_noise_ << params.acc_noise, params.acc_noise, params.acc_noise;
  gyr_noise_ << params.gyr_noise, params.gyr_noise, params.gyr_noise;
  acc_bias_ << params.acc_bias, params.acc_bias, params.acc_bias;
  gyr_bias_ << params.gyr_bias, params.gyr_bias, params.gyr_bias;

  q_ = ProcessNoiseCov();
  q_.block<3, 3>(0, 0).diagonal() = gyr_noise_;
  q_.block<3, 3>(3, 3).diagonal() = acc_noise_;
  q_.block<3, 3>(6, 6).diagonal() = gyr_bias_;
  q_.block<3, 3>(9, 9).diagonal() = acc_bias_;
}

void ImuProcess::feedImu(const ImuSample & s)
{
  imu_counter_++;

  if (s.t < imu_end_time_) {
    if (log_) {
      log_(2, "ImuProcess: IMU time out of order, sample dropped");
    }
    return;
  }

  std::lock_guard<std::mutex> lck(imu_mutex_);
  Process(s);
}

void ImuProcess::Process(const ImuSample & s)
{
  double dt;
  input_ikfom in;
  ImuState imu_state;
  V3D gyr_avr, acc_avr;

  if (imu_need_init_) {
    InitImu(s);
    return;
  }

  if (!imu_states_.size()) {
    acc_avr = mean_acc_ * kGravityMetersPerSecondSquared / mean_acc_.norm();
    gyr_avr = mean_gyr_;
  } else {
    gyr_avr = s.gyro;
    gyr_avr += imu_states_.back().gyr;
    gyr_avr *= 0.5;
    acc_avr = s.acc;
    acc_avr += imu_states_.back().acc;
    acc_avr *= 0.5 * kGravityMetersPerSecondSquared / mean_acc_.norm();
  }

  dt = s.t - kf_state_.time;

  in.acc = acc_avr;
  in.gyro = gyr_avr;
  kf_->predict(dt, q_, in);

  kf_state_.state = kf_->get_x();
  kf_state_.cov = kf_->get_P();
  kf_state_.time = s.t;
  kf_state_.gyr = s.gyro;

  SetKfState();

  imu_state.state = kf_state_;
  imu_state.acc = s.acc;
  imu_state.gyr = s.gyro;
  imu_state.acc_avr = kf_state_.state.rot * (acc_avr - kf_state_.state.ba);
  imu_state.acc_avr += kf_state_.state.grav.get_vect();
  imu_state.gyr_avr = gyr_avr - kf_state_.state.bg;

  imu_states_.push_back(imu_state);
  imu_start_time_ = imu_states_.front().state.time;
  imu_end_time_ = imu_states_.back().state.time;
  imu_has_data_ = true;
}

void ImuProcess::InitImu(const ImuSample & s)
{
  if (b_first_frame_) {
    init_iter_num_ = 1;
    b_first_frame_ = false;
    mean_acc_ = s.acc;
    mean_gyr_ = s.gyro;
  } else {
    mean_acc_ += s.acc;
    mean_gyr_ += s.gyro;
    init_iter_num_++;
  }

  if (init_iter_num_ > params_.rate && lidar_ready_) {
    imu_need_init_ = false;
    mean_acc_ /= init_iter_num_;
    mean_gyr_ /= init_iter_num_;

    kf_state_.time = s.t;

    kf_state_.state = kf_->get_x();
    kf_state_.state.grav = S2(-mean_acc_ / mean_acc_.norm() * kGravityMetersPerSecondSquared);
    kf_state_.state.bg = mean_gyr_;
    kf_state_.state.offset_T_L_I = params_.t_imu_lidar;
    kf_state_.state.offset_R_L_I = params_.r_imu_lidar;
    kf_->change_x(kf_state_.state);

    kf_state_.cov = PCov();
    kf_->change_P(kf_state_.cov);
    kf_state_.gyr = mean_gyr_;
  }
}

void ImuProcess::SyncWithLidar(double * imu_start_time, double * imu_end_time)
{
  std::lock_guard<std::mutex> lck(imu_mutex_);
  synced_imu_states_ = imu_states_;
  *imu_start_time = imu_start_time_;
  *imu_end_time = imu_end_time_;
}

void ImuProcess::GetTimeMatch(
  int * match_idx, double match_time, const boost::circular_buffer<ImuState> & imu_states)
{
  double time_diff;
  bool match_flag = false;

  time_diff = match_time - imu_states.front().state.time;
  *match_idx = static_cast<int>(std::floor(time_diff * params_.rate));
  *match_idx = std::max(*match_idx, 0);
  *match_idx = std::min(*match_idx, static_cast<int>(imu_states.size()) - 1);

  while (!match_flag) {
    if (imu_states[*match_idx].state.time > match_time) {
      if (*match_idx == 0) {
        match_flag = true;
      } else if (imu_states[*match_idx - 1].state.time > match_time) {
        (*match_idx)--;
      } else {
        match_flag = true;
      }
    } else if (*match_idx == static_cast<int>(imu_states.size()) - 1) {
      match_flag = true;
    } else {
      (*match_idx)++;
    }
  }
}

void ImuProcess::UndistortPointCloud(
  EllipseLioPointCloudPtr pc, KfState * kf_state, double lidar_start_time, double lidar_end_time)
{
  (void)lidar_start_time;
  int match_idx;
  Eigen::Isometry3d T_imu_lidar, T_world_imu_e;

  GetTimeMatch(&match_idx, lidar_end_time, synced_imu_states_);

  *kf_state = synced_imu_states_[match_idx].state;

  T_imu_lidar.linear() = synced_imu_states_[match_idx].state.state.offset_R_L_I.toRotationMatrix();
  T_imu_lidar.translation() = synced_imu_states_[match_idx].state.state.offset_T_L_I;
  T_world_imu_e.linear() = synced_imu_states_[match_idx].state.state.rot.toRotationMatrix();
  T_world_imu_e.translation() = synced_imu_states_[match_idx].state.state.pos;

#pragma omp parallel for
  for (size_t i = 0; i < pc->points.size(); i++) {
    int head_idx, tail_idx;
    Eigen::Isometry3d T_world_imu_p, T_imu_e_imu_p;

    const double pt_time = static_cast<double>(pc->points[i].time_secs) +
                           static_cast<double>(pc->points[i].time_nsecs) * 1e-9;

    GetTimeMatch(&tail_idx, pt_time, synced_imu_states_);
    head_idx = std::max(tail_idx - 1, 0);

    M3D R_imu = synced_imu_states_[head_idx].state.state.rot.toRotationMatrix();
    V3D vel_imu = synced_imu_states_[head_idx].state.state.vel;
    V3D pos_imu = synced_imu_states_[head_idx].state.state.pos;
    V3D acc_avr = synced_imu_states_[tail_idx].acc_avr;
    V3D gyr_avr = synced_imu_states_[tail_idx].gyr_avr;

    double dt = pt_time - synced_imu_states_[head_idx].state.time;

    T_world_imu_p.linear() = R_imu * Exp(gyr_avr, dt);
    T_world_imu_p.translation() = pos_imu + vel_imu * dt + 0.5 * acc_avr * dt * dt;
    T_imu_e_imu_p = T_world_imu_e.inverse() * T_world_imu_p;

    pc->points[i].getVector3fMap() = (T_imu_lidar.inverse() * T_imu_e_imu_p * T_imu_lidar *
                                      pc->points[i].getVector3fMap().cast<double>())
                                       .cast<float>();
  }
}

void ImuProcess::UpdateStatesWithLidar(
  KfState * kf_state, double lidar_end_time, double max_solve_time)
{
  int match_idx;
  IkfomSPtr kf(new Ikfom());

  {
    std::lock_guard<std::mutex> lck(imu_mutex_);
    *kf = *kf_;
  }

  kf->change_x(kf_state->state);
  kf->change_P(kf_state->cov);
  if (!kf->update_iterated_dyn_share_modified_R(kLidarPointCovariance, max_solve_time)) {
    if (log_) {
      log_(3, "ImuProcess: insufficient matches for iEKF update");
    }
    return;
  }

  std::lock_guard<std::mutex> lck(imu_mutex_);

  *kf_ = *kf;
  GetTimeMatch(&match_idx, lidar_end_time, imu_states_);
  imu_states_[match_idx].state.state = kf_->get_x();
  imu_states_[match_idx].state.cov = kf_->get_P();

  *kf_state = imu_states_[match_idx].state;

  for (size_t i = match_idx + 1; i < imu_states_.size(); i++) {
    double dt;
    int head_idx, tail_idx;
    input_ikfom in;
    V3D gyr_avr, acc_avr;

    tail_idx = static_cast<int>(i);
    head_idx = static_cast<int>(i) - 1;

    gyr_avr = 0.5 * (imu_states_[tail_idx].gyr + imu_states_[head_idx].gyr);
    acc_avr = 0.5 * (imu_states_[tail_idx].acc + imu_states_[head_idx].acc);
    acc_avr *= kGravityMetersPerSecondSquared / mean_acc_.norm();

    dt = imu_states_[tail_idx].state.time - imu_states_[head_idx].state.time;

    in.acc = acc_avr;
    in.gyro = gyr_avr;
    kf_->predict(dt, q_, in);

    imu_states_[tail_idx].state.state = kf_->get_x();
    imu_states_[tail_idx].state.cov = kf_->get_P();
    imu_states_[tail_idx].acc_avr =
      imu_states_[tail_idx].state.state.rot * (acc_avr - imu_states_[tail_idx].state.state.ba);
    imu_states_[tail_idx].acc_avr += imu_states_[tail_idx].state.state.grav.get_vect();
    imu_states_[tail_idx].gyr_avr = gyr_avr - imu_states_[tail_idx].state.state.bg;
  }

  kf_state_.state = imu_states_.back().state.state;
  kf_state_.cov = imu_states_.back().state.cov;
  kf_state_.time = imu_states_.back().state.time;
  kf_state_.gyr = imu_states_.back().gyr;

  SetKfState();
}

void ImuProcess::SetKfState()
{
  std::lock_guard<std::mutex> lck(pub_mutex_);
  pub_kf_state_ = kf_state_;
}

void ImuProcess::GetKfState(KfState * kf_state) const
{
  std::lock_guard<std::mutex> lck(pub_mutex_);
  *kf_state = pub_kf_state_;
}

}  // namespace ellipse_lio_core
