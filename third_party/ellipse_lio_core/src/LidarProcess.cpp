// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's src/lidar_processing.cpp
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. See LidarProcess.h and
// THIRD_PARTY_NOTICES.md for what was ported/removed.
#include <ellipse_lio_core/LidarProcess.h>

#include <algorithm>
#include <cmath>

namespace ellipse_lio_core
{
LidarProcess::~LidarProcess() {}

LidarProcess::LidarProcess(LidarParams params, float map_resolution, LogFn log)
: log_(std::move(log)),
  process_pc_(new EllipseLioPointCloud()),
  ellipselio_pc_(new EllipseLioPointCloud()),
  params_(params)
{
  num_bins_ = static_cast<int>(std::ceil(params_.max_range + 1));
  scan_res_ = static_cast<float>(kPi * (params_.vertical_fov / (params_.scan_lines - 1)) / 180.0);
  max_octree_res_ = 10;

  bin_pcs_i_ = Eigen::ArrayXi::Zero(num_bins_);
  bin_pcs_sizes_ = Eigen::ArrayXi::Zero(num_bins_);
  bin_sizes_ = std::vector<std::atomic<int>>(num_bins_);
  bin_octrees_ = std::vector<iOctree::Octree>(num_bins_);
  bin_idxs_ = std::vector<std::vector<int>>(num_bins_, std::vector<int>(kMaxScanPoints));
  bin_pcs_ = std::vector<EllipseLioPointCloud>(num_bins_);
  bin_min_times_ = std::vector<double>(num_bins_, 0.0);
  bin_max_times_ = std::vector<double>(num_bins_, 0.0);

  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);

  process_pc_->resize(kMaxScanPoints);
  ellipselio_pc_->reserve(kMaxProcPoints);

  bucket_sizes_ = std::vector<int>(num_bins_, 1);
  cnt_neighbours_ = std::vector<int>(num_bins_, 1);
  scan_line_sep_ = std::vector<float>(num_bins_, scan_res_);
  min_neighbours_ = std::vector<int>(num_bins_, kMinNeighbours);
  max_neighbours_ = std::vector<int>(num_bins_, kMaxNeighbours);
  search_radii_ = std::vector<float>(num_bins_, kMaxSearchRes);
  match_radii_ = std::vector<float>(num_bins_, kMaxSearchRes);
  octree_resolutions_ = std::vector<float>(num_bins_, kMinSearchRes);

#pragma omp parallel for
  for (int i = 0; i < num_bins_; i++) {
    float octree_res, search_rad, bucket_size, scan_line_sep, match_rad;

    scan_line_sep = (i + 1) * scan_res_;
    scan_line_sep = std::floor(scan_line_sep * 100.0f) / 100.0f;
    scan_line_sep = std::fmax(scan_line_sep, static_cast<float>(kMinBinRes));

    octree_res = (i + 1) * scan_res_;
    octree_res = std::floor(octree_res * 100.0f) / 100.0f;
    octree_res = std::fmax(octree_res, static_cast<float>(kMinBinRes));

    match_rad = std::fmax(10.0f * octree_res, static_cast<float>(kMinSearchRes));
    search_rad = std::fmin(
      std::fmax(10.0f * octree_res, static_cast<float>(kMinSearchRes)),
      static_cast<float>(kMaxSearchRes));

    bucket_size = kMaxNeighbours * std::pow(map_resolution, 2);
    bucket_size /= static_cast<float>(kPi) * std::pow(search_rad, 2);
    bucket_size = std::fmin(std::ceil(bucket_size), static_cast<float>(kMinNeighbours));

    bucket_sizes_[i] = static_cast<int>(bucket_size);
    search_radii_[i] = search_rad;
    match_radii_[i] = match_rad;
    scan_line_sep_[i] = scan_line_sep;
    octree_resolutions_[i] = octree_res;

    bin_pcs_i_(i) = i;
    bin_pcs_[i].reserve(kMaxScanPoints);
    bin_octrees_[i].SetMaxNewPoints(kMaxScanPoints);
    bin_octrees_[i].SetMaxOctants(static_cast<int>(0.1 * kMaxScanPoints));
  }
}

void LidarProcess::ConvertScan(const LidarPointVector & points, double t_beg)
{
  std::atomic<int> rej_points = 0;
  const int num_points = std::min(static_cast<int>(points.size()), kMaxScanPoints);

  std::fill(bin_sizes_.begin(), bin_sizes_.end(), 0);

#pragma omp parallel for
  for (int i = 0; i < num_points; i++) {
    const LidarPoint & sp = points[i];
    EllipseLioPoint & dp = process_pc_->points[i];

    dp.x = sp.x;
    dp.y = sp.y;
    dp.z = sp.z;
    dp.intensity = sp.intensity;

    const double pt_time = t_beg + sp.t_offset_sec;
    dp.time_secs = static_cast<uint32_t>(std::floor(pt_time));
    dp.time_nsecs = static_cast<uint32_t>(std::llround((pt_time - std::floor(pt_time)) * 1e9));

    const float range = std::sqrt(dp.x * dp.x + dp.y * dp.y + dp.z * dp.z);

    if (
      range < params_.min_range || range > params_.max_range || std::isnan(range) ||
      std::isinf(range)) {
      rej_points++;
      continue;
    }

    const int bin_idx = static_cast<int>(std::floor(range));
    dp.bin_idx = bin_idx;
    bin_idxs_[bin_idx][bin_sizes_[bin_idx]++] = i;
  }

  const int proc_points = num_points - rej_points.load();

  float mean_bin = 0, mean_num = 0;
  for (int i = 0; i < num_bins_; i++) {
    if (mean_num > 0.5 * proc_points && bin_sizes_[i] < 0.01 * proc_points) {
      break;
    }
    mean_num += bin_sizes_[i];
    mean_bin += i * bin_sizes_[i];
  }

  mean_bin_ = static_cast<int>(std::floor(mean_bin / mean_num));
  start_bin_ = std::min(mean_bin_, static_cast<int>(max_octree_res_));
}

void LidarProcess::feedScan(const LidarPointVector & points, double t_beg, double t_end)
{
  lidar_counter_++;

  lidar_time_offset_ = 0.0;
  if (t_end < lidar_end_time_) {
    lidar_time_offset_ = lidar_end_time_ - t_end;
    if (log_) {
      log_(2, "LidarProcess: WARNING: Lidar time offset detected!");
    }
  }

  std::lock_guard<std::mutex> lck(lidar_mutex_);

  ConvertScan(points, t_beg);

  int end_bin = num_bins_;
  int prev_start_bin = -1;
  while (start_bin_ > prev_start_bin && prev_start_bin < max_octree_res_) {
#pragma omp parallel for
    for (int i = 0; i < end_bin; i++) {
      std::vector<int> new_idxs, added_idxs;
      if (!bin_sizes_[i]) {
        continue;
      }

      bin_pcs_sizes_[i] = 0;
      bin_pcs_[i].clear();
      bin_octrees_[i].clear();

      bin_octrees_[i].SetBucketSize(1);
      bin_octrees_[i].SetMinExtent(octree_resolutions_[std::max(i, start_bin_)]);
      bin_octrees_[i].Update(*process_pc_, bin_sizes_[i], bin_idxs_[i], added_idxs, new_idxs);

      if (!added_idxs.size()) {
        continue;
      }
      bin_pcs_[i] = EllipseLioPointCloud(*process_pc_, added_idxs);
      bin_pcs_sizes_[i] = static_cast<int>(bin_pcs_[i].size());
      SetMinMaxTime(i);

      bin_idxs_[i] = added_idxs;
      bin_sizes_[i] = bin_pcs_sizes_[i];
    }

    float mean_bin = 0, mean_num = 0;
    int proc_points = bin_pcs_sizes_.sum();
    for (int i = 0; i < num_bins_; i++) {
      if (mean_num > 0.5 * proc_points && bin_sizes_[i] < 0.01 * proc_points) {
        break;
      }
      mean_num += bin_sizes_[i];
      mean_bin += i * bin_sizes_[i];
    }

    prev_start_bin = start_bin_;
    mean_bin_ = static_cast<int>(std::floor(mean_bin / mean_num));
    start_bin_ = std::min(mean_bin_, static_cast<int>(max_octree_res_));
    end_bin = start_bin_;
  }

  ellipselio_pc_->clear();
  bool init_time = true;
  for (int i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) {
      continue;
    }
    if ((ellipselio_pc_->size() + bin_pcs_[i].size()) > kMaxProcPoints) {
      break;
    }

    *ellipselio_pc_ += bin_pcs_[i];

    if (init_time) {
      lidar_start_time_ = bin_min_times_[i];
      lidar_end_time_ = bin_max_times_[i];
      init_time = false;
    } else {
      lidar_start_time_ = std::min(lidar_start_time_, bin_min_times_[i]);
      lidar_end_time_ = std::max(lidar_end_time_, bin_max_times_[i]);
    }
  }
  lidar_has_data_ = true;
}

void LidarProcess::ClearBins()
{
#pragma omp parallel for
  for (int i = 0; i < num_bins_; i++) {
    if (!bin_pcs_sizes_[i]) {
      continue;
    }
    bin_pcs_sizes_[i] = 0;
    bin_pcs_[i].clear();
    bin_octrees_[i].clear();
  }
}

void LidarProcess::ClearPointCloud()
{
  std::lock_guard<std::mutex> lck(lidar_mutex_);
  ClearBins();
  lidar_has_data_ = false;
}

bool LidarProcess::GetPointCloud(
  EllipseLioPointCloudPtr pc, double * start_time, double * end_time,
  Eigen::ArrayXi * bin_pcs_sizes, int * start_bin, int * mean_bin)
{
  if (!lidar_has_data_) {
    return false;
  }

  std::lock_guard<std::mutex> lck(lidar_mutex_);
  *mean_bin = mean_bin_;
  *start_bin = start_bin_;
  if (pc->empty()) {
    *start_time = lidar_start_time_;
  }
  *end_time = lidar_end_time_;
  *pc += *ellipselio_pc_;
  *bin_pcs_sizes += bin_pcs_sizes_;
  ClearBins();
  lidar_has_data_ = false;

  return true;
}

void LidarProcess::SetMinMaxTime(int bin_idx)
{
  const auto pointTime = [](const EllipseLioPoint & p) -> double {
    return static_cast<double>(p.time_secs) + static_cast<double>(p.time_nsecs) * 1e-9;
  };

  double t1 = pointTime(bin_pcs_[bin_idx].points[0]);
  bin_min_times_[bin_idx] = t1;
  bin_max_times_[bin_idx] = t1;
  for (size_t i = 1; i < bin_pcs_[bin_idx].size(); i++) {
    const double t2 = pointTime(bin_pcs_[bin_idx].points[i]);
    bin_min_times_[bin_idx] = std::min(bin_min_times_[bin_idx], t2);
    bin_max_times_[bin_idx] = std::max(bin_max_times_[bin_idx], t2);
  }
}

}  // namespace ellipse_lio_core
