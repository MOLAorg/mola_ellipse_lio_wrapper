// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's src/map_processing.cpp
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. See EllipseLioCore.h and
// THIRD_PARTY_NOTICES.md for what was ported/removed.
#include <ellipse_lio_core/EllipseLioCore.h>
#include <ikfom/so3_math.h>
#include <omp.h>
#include <sys/times.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace ellipse_lio_core
{
EllipseLioCore::EllipseLioCore(const Config & cfg, LogFn log) : cfg_(cfg), log_(std::move(log))
{
  lidar_params_.type = cfg_.lidar_type;
  lidar_params_.rate = cfg_.lidar_rate;
  lidar_params_.scan_lines = cfg_.lidar_scan_lines;
  lidar_params_.min_range = cfg_.lidar_min_range;
  lidar_params_.max_range = cfg_.lidar_max_range;
  lidar_params_.vertical_fov = cfg_.lidar_vertical_fov;

  imu_params_.rate = cfg_.imu_rate;
  imu_params_.gyr_noise = cfg_.imu_gyr_noise;
  imu_params_.acc_noise = cfg_.imu_acc_noise;
  imu_params_.gyr_bias = cfg_.imu_gyr_bias;
  imu_params_.acc_bias = cfg_.imu_acc_bias;
  imu_params_.t_imu_lidar = cfg_.t_imu_lidar;
  imu_params_.r_imu_lidar = cfg_.r_imu_lidar;

  map_resolution_ = std::max(cfg_.map_resolution, kMinMapRes);

  map_cloud_->reserve(kMaxMapPoints);
  raw_cloud_->reserve(kMaxProcPoints);
  scan_cloud_->reserve(kMaxProcPoints);
  scan_cloud_grav_->reserve(kMaxProcPoints);
  filter_cloud_->reserve(kMaxProcPoints);
  buffer_cloud_->reserve(kMaxProcPoints);

  ioctree_.SetBucketSize(1);
  ioctree_.SetMaxOctants(kMaxMapPoints);
  ioctree_.SetMaxNewPoints(kMaxProcPoints);
  ioctree_.SetMinExtent(static_cast<float>(map_resolution_));

  ekfom_data_sb_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_om_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_oe_ = Eigen::ArrayXd::Zero(100);
  ekfom_data_w_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_ot_ = Eigen::ArrayXXd(kMaxProcPoints, 3);
  ekfom_data_or_ = Eigen::ArrayXXd(kMaxProcPoints, 3);
  ekfom_data_h_ = Eigen::VectorXd(kMaxProcPoints);
  ekfom_data_w_x_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_h_x_ = Eigen::MatrixXd(kMaxProcPoints, 6);
  ekfom_data_h_x_r_ = Eigen::MatrixXd(6, kMaxProcPoints);
  ekfom_data_h_v_ = Eigen::ArrayXd(kMaxProcPoints);
  ekfom_data_h_x_v_ = Eigen::MatrixXd(kMaxProcPoints, 6);

  update_idx_.reserve(kMaxMapPoints);
  saliency_idxs_.reserve(kMaxMapPoints);
  neighbours_.reserve(kMaxMapPoints);
  filters_.reserve(kMaxMapPoints);

  tensors_p1_.reserve(kMaxMapPoints);
  tensors_p2_.reserve(kMaxMapPoints);
  salivalues_.reserve(kMaxMapPoints);
  eigenvalues_.reserve(kMaxMapPoints);
  eigenvectors_.reserve(kMaxMapPoints);

  updated_pt_ = std::vector<std::atomic<int>>(kMaxMapPoints);
  new_neighbours_map_idx_ = std::vector<int>(kMaxScanPoints);
  new_neighbours_size_ = std::vector<std::atomic<int>>(kMaxScanPoints);
  new_neighbours_ = std::vector<std::vector<int>>(kMaxScanPoints, std::vector<int>(kMaxNeighbours));

  kf_->init_dyn_share(
    GetF, DfDx, DfDw,
    [this](state_ikfom & s, esekfom::dyn_share_datastruct<double> & d) { hShareModel(s, d); }, 10,
    1e-3);

  // Built eagerly (unlike upstream's lazy first-timer-tick construction,
  // which only existed because it needed shared_from_this()): num_bins_ is
  // known immediately from lidar_params_.
  imu_process_ = std::make_unique<ImuProcess>(kf_, imu_params_, log_);
  lid_process_ =
    std::make_unique<LidarProcess>(lidar_params_, static_cast<float>(map_resolution_), log_);

  n_means_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
  n_cnts_ = Eigen::ArrayXXi::Zero(kMaxProcPoints, lid_process_->num_bins_);
  n_bins_ = Eigen::ArrayXXi::Zero(kMaxProcPoints, lid_process_->num_bins_);

  raw_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
  scan_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
  buffer_cloud_bins_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
  scan_bin_sizes_ = std::vector<std::atomic<int>>(lid_process_->num_bins_);
  filter_bin_sizes_ = std::vector<std::atomic<int>>(lid_process_->num_bins_);

  char line[128];
  struct tms timeSample;
  last_cpu_ = times(&timeSample);
  last_sys_cpu_ = timeSample.tms_stime;
  last_user_cpu_ = timeSample.tms_utime;

  num_processors_ = 0;
  if (FILE * file = std::fopen("/proc/cpuinfo", "r")) {
    while (std::fgets(line, sizeof(line), file) != nullptr) {
      if (std::strncmp(line, "processor", 9) == 0) {
        num_processors_++;
      }
    }
    std::fclose(file);
  }
  num_processors_ = std::max(num_processors_, 1);

  start_time_ = omp_get_wtime();
}

void EllipseLioCore::addImu(const ImuSample & s)
{
  imu_process_->feedImu(s);
  tryProcess();
}

void EllipseLioCore::addLidarScan(const LidarPointVector & points, double t_beg, double t_end)
{
  lid_process_->feedScan(points, t_beg, t_end);
  tryProcess();
}

std::vector<OdometryOutput> EllipseLioCore::drainReady()
{
  std::vector<OdometryOutput> out;
  out.swap(ready_outputs_);
  return out;
}

std::shared_ptr<const XYZIPointVector> EllipseLioCore::currentMap() const
{
  auto out = std::make_shared<XYZIPointVector>();
  out->reserve(map_cloud_->size());
  for (const auto & p : map_cloud_->points) {
    out->push_back(XYZIPoint{p.x, p.y, p.z, p.intensity});
  }
  return out;
}

bool EllipseLioCore::SyncPackages()
{
  const double lidar_scan_time = 1.0 / lidar_params_.rate;
  const double inter_sync_time = omp_get_wtime() - last_sync_time_;
  const double lidar_scan_duration = lidar_scan_time;

  if (!lid_process_->lidar_has_data_ && buffer_cloud_->empty() && raw_cloud_->empty()) {
    return false;
  }
  imu_process_->lidar_ready_ = true;
  if (imu_process_->imu_end_time_ <= last_imu_time_) {
    return false;
  }
  if (lid_process_->lidar_start_time_ < imu_process_->imu_start_time_) {
    lid_process_->ClearPointCloud();
    if (log_) {
      log_(2, "EllipseLioCore: Lidar start time is before IMU start time");
    }
    return false;
  }

  lid_process_->GetPointCloud(
    raw_cloud_, &raw_start_time_, &raw_end_time_, &raw_cloud_bins_, &start_bin_, &mean_bin_);

  if (raw_cloud_->empty() && buffer_cloud_->empty()) {
    return false;
  }

  imu_start_time_ = imu_process_->imu_start_time_;
  imu_end_time_ = imu_process_->imu_end_time_;
  last_imu_time_ = imu_process_->imu_end_time_;

  scan_start_time_ = buffer_start_time_;
  scan_end_time_ = raw_end_time_;

  if (buffer_cloud_->empty()) {
    scan_start_time_ = raw_start_time_;
  }
  if (raw_cloud_->empty()) {
    scan_end_time_ = buffer_end_time_;
  }
  if (imu_start_time_ > scan_start_time_) {
    scan_start_time_ = imu_start_time_;
    if (log_) {
      log_(2, "EllipseLioCore: IMU start time later than scan start time");
    }
    buffer_cloud_->clear();
    return false;
  }
  if (scan_end_time_ - scan_start_time_ < lidar_scan_duration) {
    if (!buffer_cloud_->empty() || scan_end_time_ > imu_end_time_) {
      return false;
    }
  } else {
    if (scan_start_time_ + lidar_scan_duration > imu_end_time_) {
      return false;
    }
  }

  SyncRawCloudWithImu();

  const int cur_imu_freq =
    static_cast<int>(std::round(imu_process_->imu_counter_ / inter_sync_time));
  const int cur_lid_freq =
    static_cast<int>(std::round(lid_process_->lidar_counter_ / inter_sync_time));
  imu_process_->imu_counter_ = 0;
  lid_process_->lidar_counter_ = 0;

  analytics_.imu_freq = cur_imu_freq;
  analytics_.lid_freq = cur_lid_freq;
  analytics_.odom_freq = static_cast<int64_t>(std::round(1.0 / inter_sync_time));

  last_sync_time_ = omp_get_wtime();
  return true;
}

void EllipseLioCore::SyncRawCloudWithImu()
{
  std::atomic<int> scan_idx = 0, filter_idx = 0;

  imu_time_offset_ = 0;
  imu_process_->SyncWithLidar(&imu_start_time_, &imu_end_time_);
  lid_time_offset_ = lid_process_->lidar_time_offset_;

  if (!raw_cloud_->empty()) {
    scan_start_time_ = raw_start_time_;
    scan_end_time_ = raw_end_time_;
  }
  if (!buffer_cloud_->empty()) {
    scan_start_time_ = buffer_start_time_;
  }
  if (imu_end_time_ > scan_end_time_ && imu_start_time_ < scan_start_time_) {
    *scan_cloud_ = *buffer_cloud_;
    *scan_cloud_ += *raw_cloud_;
    scan_cloud_bins_ = buffer_cloud_bins_ + raw_cloud_bins_;
    buffer_cloud_->clear();
    buffer_cloud_bins_.setZero();
    buffer_start_time_ = 0;
    buffer_end_time_ = 0;
  } else {
    if (imu_end_time_ < scan_end_time_) {
      imu_time_offset_ = scan_end_time_ - imu_end_time_;
      buffer_end_time_ = scan_end_time_;
      buffer_start_time_ = imu_end_time_;
      scan_end_time_ = imu_end_time_;
    }
    if (imu_start_time_ > scan_start_time_) {
      scan_start_time_ = imu_start_time_;
    }

    std::fill(scan_bin_sizes_.begin(), scan_bin_sizes_.end(), 0);
    std::fill(filter_bin_sizes_.begin(), filter_bin_sizes_.end(), 0);
    const int total_size = static_cast<int>(buffer_cloud_->size() + raw_cloud_->size());

    scan_cloud_->resize(total_size);
    filter_cloud_->resize(total_size);

    const int buffer_size = static_cast<int>(buffer_cloud_->size());

#pragma omp parallel for
    for (int i = 0; i < total_size; i++) {
      const EllipseLioPoint & pt =
        i < buffer_size ? buffer_cloud_->points[i] : raw_cloud_->points[i - buffer_size];
      const double pt_time =
        static_cast<double>(pt.time_secs) + static_cast<double>(pt.time_nsecs) * 1e-9;
      if (pt_time < scan_start_time_) {
        continue;
      }
      if (pt_time > scan_end_time_) {
        filter_bin_sizes_[pt.bin_idx]++;
        filter_cloud_->points[filter_idx++] = pt;
      } else {
        scan_bin_sizes_[pt.bin_idx]++;
        scan_cloud_->points[scan_idx++] = pt;
      }
    }

    scan_cloud_->resize(scan_idx);
    filter_cloud_->resize(filter_idx);
    *buffer_cloud_ = *filter_cloud_;

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(scan_bin_sizes_.size()); i++) {
      scan_cloud_bins_[i] = scan_bin_sizes_[i].load();
      buffer_cloud_bins_[i] = filter_bin_sizes_[i].load();
    }
  }

  raw_cloud_->clear();
  raw_cloud_bins_.setZero();
  raw_start_time_ = 0;
  raw_end_time_ = 0;

  if (!ekfom_data_sb_.sum()) {
    ekfom_data_sb_ += std::min(std::max(start_bin_ / 10.0, 1e-4), 1.0);
  } else {
    ekfom_data_sb_[start_bin_cnt_] = std::min(std::max(start_bin_ / 10.0, 1e-4), 1.0);
  }
  start_bin_cnt_ = (start_bin_cnt_ + 1) % static_cast<int>(ekfom_data_sb_.size());

  analytics_.imu_offset = imu_time_offset_;
  analytics_.lid_offset = lid_time_offset_;
  analytics_.scan_size = static_cast<int64_t>(scan_cloud_->size());
  analytics_.buffer_size = static_cast<int64_t>(buffer_cloud_->size());
  analytics_.scan_time = scan_end_time_ - scan_start_time_;
}

void EllipseLioCore::ComputeTensorVote(int i, int j, M3F * A_j, bool first_pass)
{
  V3F p_i = map_cloud_->points[i].getVector3fMap();
  V3F p_j = map_cloud_->points[j].getVector3fMap();
  const int & bin_idx = map_cloud_->points[i].bin_idx;
  float search_rad = lid_process_->search_radii_[bin_idx];
  float d_ij = (p_i - p_j).norm();
  float c_ij = std::exp(-std::pow(d_ij, 2) / search_rad);
  V3F r_ij = (p_i - p_j).normalized();
  M3F rrt = r_ij * r_ij.transpose();
  M3F R_ij = Eye3f - 2.0f * rrt;
  M3F Rp_ij = (Eye3f - 0.5f * rrt) * R_ij;
  M3F K_j = Eye3f;
  if (!first_pass) {
    K_j = tensors_p2_[j];
  }
  *A_j = c_ij * R_ij * K_j * Rp_ij;
}

void EllipseLioCore::ComputeTensorEigen(int i, M3F * tensor, bool first_pass)
{
  V3F eig_val, sali_val;
  M3F eig_vec, tensor_i2;
  Eigen::SelfAdjointEigenSolver<M3F> eig_solver;

  eig_solver.computeDirect(*tensor);
  eig_vec = eig_solver.eigenvectors();
  eig_val = eig_solver.eigenvalues().cwiseAbs();

  const int & bin_idx = map_cloud_->points[i].bin_idx;
  const float & search_rad = lid_process_->search_radii_[bin_idx];

  if (first_pass) {
    tensor_i2 = (eig_val(2) - eig_val(1)) * eig_vec.col(2) * eig_vec.col(2).transpose();
    tensor_i2 += (eig_val(1) - eig_val(0)) * (eig_vec.col(2) * eig_vec.col(2).transpose() +
                                              eig_vec.col(1) * eig_vec.col(1).transpose());
    tensors_p2_[i] = tensor_i2;
  } else {
    sali_val(0) = eig_val(2) - eig_val(1);
    sali_val(1) = eig_val(1) - eig_val(0);
    sali_val(2) = eig_val(0);
    sali_val.maxCoeff(&saliency_idxs_[i]);

    filters_[i][1] = true;
    salivalues_[i] = sali_val;
    eigenvalues_[i] = (1.0 / (eig_val.array() + 1e-10)).matrix().normalized();
    eigenvalues_[i] *= search_rad;
    eigenvectors_[i] = eig_vec;
    map_cloud_->points[i].prim_type = (saliency_idxs_[i] + 1) * 85;
  }
}

void EllipseLioCore::TensorVotePass1(
  int old_map_size, std::vector<int> & added_idxs, std::vector<int> & updated_idxs)
{
  const int added_size = static_cast<int>(added_idxs.size());
  std::atomic<int> upd_idx = 0, new_neighbours_idx = 0;

#pragma omp parallel for
  for (int i = 0; i < added_size; i++) {
    const int map_i = added_idxs[i];
    updated_pt_[map_i] = 0;
    map_cloud_->points[map_i].prim_type = 0;

    const int & bin_idx = map_cloud_->points[map_i].bin_idx;
    const int & bucket_size = lid_process_->bucket_sizes_[bin_idx];
    const float search_rad = lid_process_->search_radii_[bin_idx];

    neighbours_[map_i].reserve(lid_process_->max_neighbours_[bin_idx]);
    ioctree_.RadiusNeighbors(
      map_cloud_->points[map_i], search_rad, neighbours_[map_i], bucket_size);

    n_bins_.row(i).setZero();
    n_cnts_.row(i).setZero();
    n_bins_(i, bin_idx) = 1;
    n_cnts_(i, bin_idx) = static_cast<int>(neighbours_[map_i].size());
  }

#pragma omp parallel for
  for (int i = 0; i < lid_process_->num_bins_; i++) {
    const int n_bins_sum = n_bins_.col(i).head(added_size).sum();
    const int tot_sum = lid_process_->cnt_neighbours_[i] + n_bins_sum;
    if (!n_bins_sum) {
      continue;
    }
    n_means_(i) = static_cast<int>(std::floor(n_means_(i) * lid_process_->cnt_neighbours_[i]));
    n_means_(i) += n_cnts_.col(i).head(added_size).sum();
    n_means_(i) = static_cast<int>(std::floor(static_cast<double>(n_means_(i)) / tot_sum));
    n_means_(i) = std::min(std::max(n_means_(i), kMinNeighbours), kMaxNeighbours);
    lid_process_->min_neighbours_[i] = n_means_(i);
    lid_process_->max_neighbours_[i] = std::min(2 * n_means_(i), kMaxNeighbours);
    lid_process_->cnt_neighbours_[i] += n_bins_sum;
  }

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(added_idxs.size()); i++) {
    Eigen::MatrixXf K;
    M3F tensor_i1;

    const int map_i = added_idxs[i];

    const int & bin_idx = map_cloud_->points[map_i].bin_idx;
    const int & min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int & max_neigh = lid_process_->max_neighbours_[bin_idx];

    const int loop_cnt = std::min(static_cast<int>(neighbours_[map_i].size()), max_neigh);
    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      const int map_j = neighbours_[map_i][j];
      ComputeTensorVote(map_i, map_j, &A_j, true);
      K.row(j) = A_j.reshaped(1, 9);

      if (map_j >= old_map_size) {
        continue;
      }

      const int & bin_idx_j = map_cloud_->points[map_j].bin_idx;
      const float search_rad_j = lid_process_->search_radii_[bin_idx_j];
      EllipseLioPoint & pt_i = map_cloud_->points[map_i];
      EllipseLioPoint & pt_j = map_cloud_->points[map_j];
      const float d_ij = (pt_i.getVector3fMap() - pt_j.getVector3fMap()).norm();

      if (d_ij > search_rad_j) {
        continue;
      }

      if (!(updated_pt_[map_j]++)) {
        update_idx_[map_j] = new_neighbours_idx++;
        new_neighbours_map_idx_[update_idx_[map_j]] = map_j;
        new_neighbours_size_[update_idx_[map_j]] = 0;
        new_neighbours_[update_idx_[map_j]][new_neighbours_size_[update_idx_[map_j]]++] = map_i;
      } else if (new_neighbours_size_[update_idx_[map_j]] < kMaxNeighbours) {
        new_neighbours_[update_idx_[map_j]][new_neighbours_size_[update_idx_[map_j]]++] = map_i;
      }
    }
    tensors_p1_[map_i] = K.colwise().sum().reshaped(3, 3);

    filters_[map_i][0] = loop_cnt >= min_neigh;
    if (!filters_[map_i][0]) {
      continue;
    }

    tensor_i1 = tensors_p1_[map_i] / static_cast<float>(loop_cnt);
    ComputeTensorEigen(map_i, &tensor_i1, true);
  }

  updated_idxs.resize(new_neighbours_idx);

#pragma omp parallel for
  for (int i = 0; i < new_neighbours_idx; i++) {
    Eigen::MatrixXf K;
    M3F tensor_i1;

    const int map_i = new_neighbours_map_idx_[i];
    updated_pt_[map_i] = 0;

    const int & bin_idx = map_cloud_->points[map_i].bin_idx;
    const int & min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int & max_neigh = lid_process_->max_neighbours_[bin_idx];
    if (static_cast<int>(neighbours_[map_i].size()) >= max_neigh) {
      continue;
    }

    updated_idxs[upd_idx++] = map_i;

    const int max_loop = max_neigh - static_cast<int>(neighbours_[map_i].size());
    const int loop_cnt = std::min(static_cast<int>(new_neighbours_size_[i]), max_loop);

    const int old_size = static_cast<int>(neighbours_[map_i].size());
    neighbours_[map_i].resize(old_size + loop_cnt);

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      M3F A_j;
      const int map_j = new_neighbours_[i][j];
      neighbours_[map_i][old_size + j] = map_j;
      ComputeTensorVote(map_i, map_j, &A_j, true);
      K.row(j) = A_j.reshaped(1, 9);
    }

    tensors_p1_[map_i] += K.colwise().sum().reshaped(3, 3);

    filters_[map_i][0] = static_cast<int>(neighbours_[map_i].size()) >= min_neigh;
    if (!filters_[map_i][0]) {
      continue;
    }

    tensor_i1 = tensors_p1_[map_i] / static_cast<float>(neighbours_[map_i].size());
    ComputeTensorEigen(map_i, &tensor_i1, true);
  }
  updated_idxs.resize(upd_idx);
}

void EllipseLioCore::TensorVotePass2(std::vector<int> & added_idxs, std::vector<int> & updated_idxs)
{
  const int total_size = static_cast<int>(added_idxs.size() + updated_idxs.size());

#pragma omp parallel for
  for (int i = 0; i < total_size; i++) {
    M3F tensor_i2;
    Eigen::MatrixXf K;
    Eigen::VectorXi K_filter;

    const int map_i =
      i < static_cast<int>(added_idxs.size()) ? added_idxs[i] : updated_idxs[i - added_idxs.size()];

    const int & bin_idx = map_cloud_->points[map_i].bin_idx;
    const int & min_neigh = lid_process_->min_neighbours_[bin_idx];
    const int & max_neigh = lid_process_->max_neighbours_[bin_idx];
    const int loop_cnt = std::min(static_cast<int>(neighbours_[map_i].size()), max_neigh);

    if (!filters_[map_i][0]) {
      continue;
    }

    K = Eigen::MatrixXf::Zero(loop_cnt, 9);
    K_filter = Eigen::VectorXi::Zero(loop_cnt);

#pragma omp parallel for
    for (int j = 0; j < loop_cnt; j++) {
      const int map_j = neighbours_[map_i][j];
      if (!filters_[map_j][0]) {
        continue;
      }

      M3F A_j;
      ComputeTensorVote(map_i, map_j, &A_j, false);
      K.row(j) = A_j.reshaped(1, 9);
      K_filter(j) = 1;
    }

    const int filter_cnt = K_filter.sum();
    if (filter_cnt < min_neigh) {
      continue;
    }

    tensor_i2 = K.colwise().sum().reshaped(3, 3);
    tensor_i2 /= static_cast<float>(filter_cnt);
    ComputeTensorEigen(map_i, &tensor_i2, false);
  }
}

void EllipseLioCore::MapIncremental()
{
  int start_idx, end_idx;
  std::vector<int> new_idxs, updated_idxs, added_idxs, map_idxs;

  poses_.push_back(kf_state_.state.pos.cast<float>());
  rotes_.push_back(kf_state_.state.rot.cast<float>());

  if (traj_dist_.empty()) {
    traj_dist_.push_back(0.0f);
  } else {
    const float curr_dist = (poses_.back() - poses_[poses_.size() - 2]).norm();
    traj_dist_.push_back(traj_dist_.back() + curr_dist);
  }

  if (kf_state_.state.vel.norm() > 0.1 || vel_poses_.empty()) {
    vel_poses_.push_back(kf_state_.state.pos.cast<float>());
  }

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(scan_cloud_->size()); i++) {
    scan_cloud_->points[i].scan_idx = map_counter_;
    const int & bin_idx = scan_cloud_->points[i].bin_idx;
    scan_cloud_->points[i].bin_idx = std::max(bin_idx, start_bin_);
    scan_cloud_->points[i].getVector3fMap() =
      (kf_state_.state.rot *
         (kf_state_.state.offset_R_L_I * scan_cloud_->points[i].getVector3fMap().cast<double>() +
          kf_state_.state.offset_T_L_I) +
       kf_state_.state.pos)
        .cast<float>();
  }

  start_idx = 0;
  end_idx = 0;
  old_map_size_ = static_cast<int>(map_cloud_->size());

  for (int i = 0; i < static_cast<int>(scan_cloud_bins_.size()); i++) {
    end_idx += scan_cloud_bins_[i];
    if (!scan_cloud_bins_[i]) {
      continue;
    }
    if (end_idx > static_cast<int>(scan_cloud_->size())) {
      break;
    }

    ioctree_.SetBucketSize(lid_process_->bucket_sizes_[std::max(i, start_bin_)]);
    ioctree_.Update(
      *scan_cloud_, added_idxs, map_idxs, start_idx, end_idx, static_cast<float>(map_resolution_));
    *map_cloud_ += EllipseLioPointCloud(*scan_cloud_, added_idxs);
    new_idxs.insert(new_idxs.end(), map_idxs.begin(), map_idxs.end());
    start_idx = end_idx;
  }

  new_map_size_ = static_cast<int>(map_cloud_->size());

  update_idx_.resize(map_cloud_->size(), 0);
  saliency_idxs_.resize(map_cloud_->size(), 0);
  neighbours_.resize(map_cloud_->size(), std::vector<int>());
  filters_.resize(map_cloud_->size(), Eigen::Vector2i::Zero());

  tensors_p1_.resize(map_cloud_->size(), M3F::Zero());
  tensors_p2_.resize(map_cloud_->size(), M3F::Zero());
  salivalues_.resize(map_cloud_->size(), V3F::Zero());
  eigenvalues_.resize(map_cloud_->size(), V3F::Zero());
  eigenvectors_.resize(map_cloud_->size(), M3F::Zero());

  if (new_idxs.size() > 0) {
    TensorVotePass1(old_map_size_, new_idxs, updated_idxs);
    TensorVotePass2(new_idxs, updated_idxs);
  }

  analytics_.map_size = static_cast<int64_t>(map_cloud_->size());
  analytics_.oct_num = static_cast<int64_t>(ioctree_.OctantSize());
  analytics_.new_idxs = static_cast<int64_t>(new_idxs.size());
  analytics_.upd_idxs = static_cast<int64_t>(updated_idxs.size());
  analytics_.traj_dist = static_cast<int64_t>(traj_dist_.back());
}

void EllipseLioCore::hShareModel(
  state_ikfom & s, esekfom::dyn_share_datastruct<double> & ekfom_data)
{
  double t0, res_mean;
  int feat_tot, reject_cnt, start_bin_scale;
  float grav_check, vert_score;
  float wt_min, wt_max, wt_mean, wt_std, obs_min;
  float rng_min, rng_max, rng_mean, rng_min_scale, rng_max_scale;

  Eigen::Array3i cnts(3);
  std::atomic<int> feat_cnt;
  std::vector<std::atomic<int>> prim_cnts(3);

  M3F cov_mat;
  V3D rot_obs, tran_obs, cov_scales;
  V3F grav_norm, poses_diff, poses_orth, grav_x, grav_y, grav_z;

  Eigen::Quaternionf gq;
  Eigen::Matrix4f tf_grav;
  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();

  feat_cnt = 0;
  prim_cnts[0] = 0;
  prim_cnts[1] = 0;
  prim_cnts[2] = 0;

  grav_norm = kf_state_.state.grav.get_vect().normalized().cast<float>();
  poses_diff = s.pos.cast<float>();
  poses_diff -= vel_poses_[std::max(static_cast<int>(vel_poses_.size()) - 100, 0)];
  grav_check = std::fabs(grav_norm.dot(poses_diff));
  grav_check *= std::fabs(grav_norm.dot(poses_diff.normalized()));

  poses_orth = poses_diff - grav_norm * grav_norm.dot(poses_diff);
  vert_score = 1.0f - (((100.0f / mean_bin_) * grav_check) / poses_orth.norm());

  scan_cloud_grav_->resize(scan_cloud_->size());
  gq = Eigen::Quaternionf::FromTwoVectors(grav_norm, V3F::UnitZ());
  gq = gq * s.rot.cast<float>() * s.offset_R_L_I.cast<float>();

  tf_grav = Eigen::Matrix4f::Identity();
  tf_grav.block<3, 3>(0, 0) = gq.toRotationMatrix();

  gq = Eigen::Quaternionf::FromTwoVectors(V3F::UnitZ(), grav_norm);
  grav_x = gq.toRotationMatrix().col(0);
  grav_y = gq.toRotationMatrix().col(1);
  grav_z = gq.toRotationMatrix().col(2);

  pcl::transformPointCloud(*scan_cloud_, *scan_cloud_grav_, tf_grav);
  pcl::compute3DCentroid(*scan_cloud_grav_, centroid);
  pcl::computeCovarianceMatrixNormalized(*scan_cloud_grav_, centroid, cov_mat);

  const float mean_bin_arc =
    static_cast<float>(lidar_params_.vertical_fov * kPi * mean_bin_ / 180.0);
  const float cov_score = std::fmin(std::fabs(cov_mat(2, 2)) / mean_bin_arc, 1.0f);

  cov_scales = cov_mat.diagonal().cast<double>();
  cov_scales(2) *= cov_score;
  cov_scales(2) *= 180.0 / lidar_params_.vertical_fov;
  cov_scales /= cov_scales.maxCoeff();

  start_bin_scale =
    static_cast<int>(std::floor(kMaxSearchRes / lid_process_->match_radii_.front()));

  t0 = omp_get_wtime();
  (void)t0;

#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(scan_cloud_->size()); i++) {
    Eigen::VectorXd h_x_vec(6);
    Eigen::VectorXd::Index tran_idx, rot_idx;
    std::vector<int> N_idxs;
    std::vector<float> N_dst;
    int sali_idx, map_i, feat_num, prim_num;
    float inc_search_rad, residual, time_score, traj_diff, curr_traj_dist, bin_scale;

    M3D P_skew;
    V3D p_lidar, p_imu, a, obs_trans, obs_rot, obs_idx_trans, obs_idx_rot;
    V3F scores, p_world, n_world, p_dash, q, q_dash, norm_vec, a_world;

    const EllipseLioPoint & pt = scan_cloud_->points[i];

    p_lidar = pt.getVector3fMap().cast<double>();
    p_imu = s.offset_R_L_I * p_lidar + s.offset_T_L_I;
    p_world = (s.rot * p_imu + s.pos).cast<float>();

    const int & bin_idx = scan_cloud_->points[i].bin_idx;
    const float & match_rad = lid_process_->match_radii_[bin_idx];
    const float & search_rad = lid_process_->search_radii_[bin_idx];
    const float & min_oct_res = lid_process_->octree_resolutions_.front();

    inc_search_rad = match_rad / (ekfom_iter_cnt_ + 1);
    inc_search_rad =
      std::fmax(std::fmin(inc_search_rad, static_cast<float>(kMaxSearchRes)), min_oct_res);

    curr_traj_dist = traj_dist_.back();
    curr_traj_dist += (s.pos.cast<float>() - poses_.back()).norm();
    if (inc_search_rad > curr_traj_dist) {
      inc_search_rad = std::fmax(0.5f * inc_search_rad, static_cast<float>(kMinSearchRes));
    }

    ioctree_.KnnNeighbors(p_world, 1, N_idxs, N_dst, inc_search_rad);
    if (N_idxs.empty()) {
      continue;
    }

    map_i = N_idxs[0];
    if (!filters_[map_i][1]) {
      continue;
    }

    traj_diff = curr_traj_dist;
    traj_diff -= traj_dist_[map_cloud_->points[map_i].scan_idx];
    bin_scale = std::fmax(std::fmin(bin_idx / 4.0f, 10.0f), static_cast<float>(start_bin_scale));
    if (
      map_cloud_->points[map_i].scan_idx && s.vel.norm() > 0.1 &&
      match_rad > bin_scale * search_rad && traj_diff < match_rad / bin_scale) {
      continue;
    }

    scores = salivalues_[map_i] / salivalues_[map_i].sum();
    n_world = map_cloud_->points[map_i].getVector3fMap();
    q = p_world - n_world;

    q_dash = q.dot(eigenvectors_[map_i].col(2)) * eigenvectors_[map_i].col(2);
    p_dash = scores(0) * (p_world - q_dash);

    q_dash = q.dot(eigenvectors_[map_i].col(0)) * eigenvectors_[map_i].col(0);
    p_dash += scores(1) * (n_world + q_dash);

    p_dash += scores(2) * n_world;

    norm_vec = p_world - p_dash;
    residual = norm_vec.norm();
    norm_vec.normalize();

    time_score = 1.0f / (traj_diff + 1.0f);
    time_score *= std::fmax(1.0f - std::fabs(grav_norm.dot(norm_vec)), 1e-4f);
    time_score = 1.0f / std::fmin(std::fmax(time_score, 1e-4f), 1.0f);

    P_skew = SkewSymMat(p_imu);
    a = P_skew * s.rot.conjugate() * norm_vec.cast<double>();
    a_world = (s.rot * a).normalized().cast<float>();
    h_x_vec << norm_vec(0), norm_vec(1), norm_vec(2), a[0], a[1], a[2];

    obs_trans = cov_scales * std::pow(scores(0), 2);
    obs_trans(0) *= std::fabs(norm_vec.dot(grav_x));
    obs_trans(1) *= std::fabs(norm_vec.dot(grav_y));
    obs_trans(2) *= std::fabs(norm_vec.dot(grav_z));
    obs_rot(0) = std::fabs(a_world.dot(grav_x));
    obs_rot(1) = std::fabs(a_world.dot(grav_y));
    obs_rot(2) = std::fabs(a_world.dot(grav_z));

    obs_trans.maxCoeff(&tran_idx);
    obs_idx_trans = V3D::Zero();
    obs_idx_trans(tran_idx) = 1;
    obs_trans = obs_trans.cwiseProduct(obs_idx_trans);

    obs_rot.maxCoeff(&rot_idx);
    obs_idx_rot = V3D::Zero();
    obs_idx_rot(rot_idx) = 1;
    obs_rot = obs_rot.cwiseProduct(obs_idx_rot);

    feat_num = ++feat_cnt;
    sali_idx = saliency_idxs_[map_i];
    prim_num = ++prim_cnts[sali_idx];
    (void)prim_num;

    ekfom_data_ot_.row(feat_num - 1) = obs_trans;
    ekfom_data_or_.row(feat_num - 1) = obs_rot;
    ekfom_data_w_.row(feat_num - 1) = time_score;

    ekfom_data_h_v_(feat_num - 1) = -residual;
    ekfom_data_h_x_v_.row(feat_num - 1) = h_x_vec;
  }

  cnts << prim_cnts[0].load(), prim_cnts[1].load(), prim_cnts[2].load();
  feat_tot = feat_cnt.load();
  reject_cnt = static_cast<int>(scan_cloud_->size()) - feat_tot;

  wt_min = ekfom_data_w_.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w_.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w_.head(feat_tot).mean();
  wt_std = (ekfom_data_w_.head(feat_tot) - wt_mean).square().sum();
  wt_std = std::sqrt(wt_std / (feat_tot - 1));

  rng_min = wt_mean - wt_std;
  rng_min = std::fmax(rng_min, wt_min);
  rng_min_scale = rng_min / wt_min;
  rng_mean = wt_mean - rng_min;
  rng_max = wt_max - rng_min;
  rng_max = std::fmin(rng_mean + wt_std, rng_max);
  rng_max_scale = rng_max / ((wt_max * rng_min_scale) - rng_min);

  if (wt_std) {
    ekfom_data_w_.head(feat_tot) *= rng_min_scale;
    ekfom_data_w_.head(feat_tot) -= rng_min;
    ekfom_data_w_.head(feat_tot) *= rng_max_scale;
    ekfom_data_w_.head(feat_tot) += 1.0;
  }

  tran_obs = ekfom_data_ot_.topRows(feat_tot).colwise().sum() + 1e-4;
  rot_obs = ekfom_data_or_.topRows(feat_tot).colwise().sum() + 1e-4;

  tran_obs /= tran_obs.maxCoeff();
  rot_obs /= rot_obs.maxCoeff();

  obs_min = 1e4f * static_cast<float>(rot_obs.minCoeff() * tran_obs.minCoeff());

  if (!ekfom_data_om_.sum()) {
    ekfom_data_om_ += std::fmin(std::fmax(obs_min, 1e-4f), 1.0f);
    ekfom_data_oe_ += std::fmin(std::fmax(vert_score, 1e-4f), 1.0f);
  } else {
    ekfom_data_om_[ekfom_obs_cnt_] = std::fmin(std::fmax(obs_min, 1e-4f), 1.0f);
    ekfom_data_oe_[ekfom_vert_cnt_] = std::fmin(std::fmax(vert_score, 1e-4f), 1.0f);
  }
  ekfom_obs_cnt_ = (ekfom_obs_cnt_ + 1) % static_cast<int>(ekfom_data_om_.size());
  ekfom_vert_cnt_ = (ekfom_vert_cnt_ + 1) % static_cast<int>(ekfom_data_oe_.size());

  obs_min =
    static_cast<float>(ekfom_data_om_.mean() * ekfom_data_oe_.mean() * ekfom_data_sb_.mean());
  obs_min = std::fmax(obs_min, 0.1f);
  ekfom_data_w_.head(feat_tot) = ekfom_data_w_.head(feat_tot).pow(obs_min);
  ekfom_data_h_.block(0, 0, feat_tot, 1) = ekfom_data_h_v_.head(feat_tot);
  ekfom_data_w_x_.block(0, 0, feat_tot, 1) = ekfom_data_w_.head(feat_tot);
  ekfom_data_h_x_.block(0, 0, feat_tot, 6) = ekfom_data_h_x_v_.topRows(feat_tot);

  wt_min = ekfom_data_w_.head(feat_tot).minCoeff();
  wt_max = ekfom_data_w_.head(feat_tot).maxCoeff();
  wt_mean = ekfom_data_w_.head(feat_tot).mean();
  wt_std = (ekfom_data_w_.head(feat_tot) - wt_mean).square().sum();
  wt_std = std::sqrt(wt_std / (feat_tot - 1));

  rng_min = 1;
  rng_max += 1;
  rng_mean += 1;

  ekfom_data_h_x_r_.leftCols(feat_tot) =
    (ekfom_data_h_x_.topRows(feat_tot).array().colwise() * ekfom_data_w_x_.head(feat_tot))
      .transpose();

  res_mean = -ekfom_data_h_.head(feat_tot).sum() / feat_tot;
  ekfom_data.h = ekfom_data_h_.head(feat_tot);
  ekfom_data.h_x = ekfom_data_h_x_.topRows(feat_tot);
  ekfom_data.h_x_R = ekfom_data_h_x_r_.leftCols(feat_tot);

  ekfom_iter_cnt_++;

  analytics_.num_planes = cnts(0);
  analytics_.num_lines = cnts(1);
  analytics_.num_balls = cnts(2);
  analytics_.wt_std = wt_std;
  analytics_.wt_min = wt_min;
  analytics_.wt_max = wt_max;
  analytics_.obs_min = obs_min;
  analytics_.wt_mean = wt_mean;
  analytics_.rng_min = rng_min;
  analytics_.rng_max = rng_max;
  analytics_.rng_mean = rng_mean;
  analytics_.res_mean = res_mean;
  analytics_.mean_bin = mean_bin_;
  analytics_.start_bin = start_bin_;
  analytics_.num_feats = feat_tot;
  analytics_.num_reject = reject_cnt;
  analytics_.kf_iterations = ekfom_iter_cnt_;
  analytics_.obs_score = ekfom_data_om_.mean();
  analytics_.vert_score = ekfom_data_oe_.mean();
  analytics_.bin_score = ekfom_data_sb_.mean();
}

void EllipseLioCore::tryProcess()
{
  if (!SyncPackages()) {
    return;
  }

  const double t1 = omp_get_wtime();
  imu_process_->UndistortPointCloud(scan_cloud_, &kf_state_, scan_start_time_, scan_end_time_);
  const double t2 = omp_get_wtime();

  if (map_counter_) {
    ekfom_iter_cnt_ = 0;
    imu_process_->UpdateStatesWithLidar(&kf_state_, scan_end_time_, 0.5 / lidar_params_.rate);
  }
  const double t3 = omp_get_wtime();

  // Snapshot the deskewed scan in LiDAR body frame BEFORE MapIncremental()
  // transforms scan_cloud_ into world frame in place: this keeps
  // OdometryOutput::deskewed_points_body's contract ("LiDAR body frame")
  // consistent with the other three wrappers, even though upstream's own
  // PublishScan() happens to publish the world-frame version (a publish-time
  // choice, not part of the estimation math -- see EllipseLioCore.h).
  auto deskewedBody = std::make_shared<XYZIPointVector>();
  deskewedBody->reserve(scan_cloud_->size());
  for (const auto & p : scan_cloud_->points) {
    deskewedBody->push_back(XYZIPoint{p.x, p.y, p.z, p.intensity});
  }

  MapIncremental();
  const double t4 = omp_get_wtime();

  const double imu_time = t2 - t1;
  const double state_time = t3 - t2;
  const double map_time = t4 - t3;
  const double total_time = t4 - t1;

  max_imu_time_ = std::max(max_imu_time_, imu_time);
  max_state_time_ = std::max(max_state_time_, state_time);
  max_map_time_ = std::max(max_map_time_, map_time);
  max_total_time_ = std::max(max_total_time_, total_time);

  mean_imu_time_ += imu_time;
  mean_state_time_ += state_time;
  mean_map_time_ += map_time;
  mean_total_time_ += total_time;

  ComputeRamUsage();
  ComputeCpuUsage();

  analytics_.run_time = static_cast<int64_t>(std::round(t4 - start_time_));
  analytics_.imu_time = imu_time;
  analytics_.state_time = state_time;
  analytics_.map_time = map_time;
  analytics_.total_time = total_time;
  analytics_.imu_mean = mean_imu_time_ / (map_counter_ + 1);
  analytics_.state_mean = mean_state_time_ / (map_counter_ + 1);
  analytics_.map_mean = mean_map_time_ / (map_counter_ + 1);
  analytics_.total_mean = mean_total_time_ / (map_counter_ + 1);
  analytics_.imu_max = max_imu_time_;
  analytics_.state_max = max_state_time_;
  analytics_.map_max = max_map_time_;
  analytics_.total_max = max_total_time_;

  static bool DUMP_GRAV = ::getenv("MOLA_ELLIPSELIO_DUMP_ESTIMATED_GRAV") != nullptr;
  if (DUMP_GRAV) {
    static std::ofstream f("grav.txt");
    char buf[100];
    std::snprintf(buf, sizeof(buf), "%.03f ", scan_end_time_);
    f << buf << kf_state_.state.grav << "\n";
  }

  static bool DEBUG_DUMP = ::getenv("MOLA_ELLIPSELIO_DEBUG_DUMP") != nullptr;
  if (DEBUG_DUMP) {
    static std::ofstream df("debug_dump.txt");
    df << scan_end_time_ << " pos=" << kf_state_.state.pos.transpose()
       << " vel=" << kf_state_.state.vel.transpose() << " ba=" << kf_state_.state.ba.transpose()
       << " bg=" << kf_state_.state.bg.transpose()
       << " grav=" << kf_state_.state.grav.get_vect().transpose()
       << " num_feats=" << analytics_.num_feats << " num_reject=" << analytics_.num_reject
       << " res_mean=" << analytics_.res_mean << " obs_min=" << analytics_.obs_min
       << " vert_score=" << analytics_.vert_score << " map_size=" << analytics_.map_size
       << " mean_bin=" << mean_bin_ << " start_bin=" << start_bin_ << " kf_iter=" << ekfom_iter_cnt_
       << "\n";
  }

  OdometryOutput out;
  out.timestamp = scan_end_time_;
  out.pose_imu.setIdentity();
  out.pose_imu.linear() = kf_state_.state.rot.toRotationMatrix();
  out.pose_imu.translation() = kf_state_.state.pos;
  out.vel = kf_state_.state.vel;
  out.bias_gyro = kf_state_.state.bg;
  out.bias_acc = kf_state_.state.ba;
  out.gravity = kf_state_.state.grav.get_vect();
  out.deskewed_points_body = std::move(deskewedBody);
  out.analytics = analytics_;

  ready_outputs_.push_back(std::move(out));
  map_counter_++;
}

void EllipseLioCore::ComputeRamUsage()
{
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >>
    minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >>
    nice >> num_threads >> itrealvalue >> starttime >> vsize >> rss;
  stat_stream.close();
  const long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
  resident_set = static_cast<double>(rss) * static_cast<double>(page_size_kb);

  analytics_.ram_usage = static_cast<int64_t>(std::round(resident_set / 1000.0));
}

void EllipseLioCore::ComputeCpuUsage()
{
  struct tms timeSample;
  std::clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (
    now <= last_cpu_ || timeSample.tms_stime < last_sys_cpu_ ||
    timeSample.tms_utime < last_user_cpu_) {
    cpu_percent = -1.0;
  } else {
    cpu_percent = static_cast<double>(
      (timeSample.tms_stime - last_sys_cpu_) + (timeSample.tms_utime - last_user_cpu_));
    cpu_percent /= static_cast<double>(now - last_cpu_);
    cpu_percent /= num_processors_;
    cpu_percent *= 100.0;
  }
  last_cpu_ = now;
  last_sys_cpu_ = timeSample.tms_stime;
  last_user_cpu_ = timeSample.tms_utime;

  analytics_.cpu_usage = static_cast<int64_t>(std::round(cpu_percent));
}

}  // namespace ellipse_lio_core
