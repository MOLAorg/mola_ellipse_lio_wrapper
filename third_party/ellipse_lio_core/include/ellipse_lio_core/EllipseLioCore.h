// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's include/map_processing.h and src/map_processing.cpp
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. `MappingNode` (an `rclcpp::Node`) is
// replaced by this stateful, push-based class: the four ROS timers collapse
// into `addImu()`/`addLidarScan()` triggering the same processing steps
// `TimerCallback()` ran (`SyncPackages()`, IMU propagation/undistortion, the
// iterated-EKF tensor-voting update, `MapIncremental()`); the five ROS
// publishers and the `tf2_ros` broadcaster are replaced by the returned
// `OdometryOutput` and the `currentMap()`/`currentAnalytics()` accessors;
// `SplitMap()` (PointCloud2 message chunking) and the whole camera pipeline
// are dropped. `imu_process_`/`lid_process_` are built directly in the
// constructor instead of lazily on the first timer tick (upstream deferred
// this only because it needed `shared_from_this()`, unavailable inside a
// `rclcpp::Node` constructor -- not needed here). See
// THIRD_PARTY_NOTICES.md and the plan's E5 for the full task list this
// implements.
#pragma once

#include <ellipse_lio_core/ImuProcess.h>
#include <ellipse_lio_core/LidarProcess.h>

#include <ctime>
#include <deque>
#include <ellipse_lio_core/Types.hpp>
#include <mutex>
#include <vector>

namespace ellipse_lio_core
{
/// EllipseLIO's algorithm core: IMU-propagated, iterated-ESKF LiDAR-inertial
/// odometry with an incremental i-Octree map and a tensor-voting ellipsoid
/// point-to-map residual. No ROS, no MOLA dependency.
///
/// Usage mirrors fast_lio2_core::FastLio2Core: call `addImu()` for every IMU
/// sample and `addLidarScan()` for every LiDAR scan, in timestamp order
/// (interleaving is fine). Call `drainReady()` to collect newly-completed
/// scan results.
class EllipseLioCore
{
public:
  explicit EllipseLioCore(const Config & cfg, LogFn log = {});

  void addImu(const ImuSample & s);
  void addLidarScan(const LidarPointVector & points, double t_beg, double t_end);

  std::vector<OdometryOutput> drainReady();

  /// Snapshot of the incremental i-Octree map, in world frame.
  std::shared_ptr<const XYZIPointVector> currentMap() const;

  bool isInitialized() const { return map_counter_ > 0; }

  /// Only meant to be called by the h_share_model lambda passed to
  /// `kf_->init_dyn_share()`.
  void hShareModel(state_ikfom & s, esekfom::dyn_share_datastruct<double> & ekfom_data);

private:
  bool SyncPackages();
  void SyncRawCloudWithImu();

  void ComputeTensorVote(int i, int j, M3F * A_j, bool first_pass);
  void ComputeTensorEigen(int i, M3F * tensor, bool first_pass);
  void TensorVotePass1(
    int old_map_size, std::vector<int> & added_idxs, std::vector<int> & updated_idxs);
  void TensorVotePass2(std::vector<int> & added_idxs, std::vector<int> & updated_idxs);
  void MapIncremental();

  void ComputeRamUsage();
  void ComputeCpuUsage();

  /// Runs one full processing cycle if SyncPackages() reports new
  /// synchronized data, pushing an OdometryOutput to ready_outputs_.
  void tryProcess();

  Config cfg_;
  LogFn log_;

  double last_sync_time_ = 0;
  double max_imu_time_ = 0, max_state_time_ = 0, max_map_time_ = 0, max_total_time_ = 0;
  double mean_imu_time_ = 0, mean_state_time_ = 0, mean_map_time_ = 0, mean_total_time_ = 0;

  int map_counter_ = 0;
  int old_map_size_ = 0, new_map_size_ = 0, last_map_size_ = 0;

  double start_time_ = 0;

  double map_resolution_ = 0.1;
  int start_bin_ = 0, mean_bin_ = 0;

  int num_processors_ = 1;
  std::clock_t last_cpu_ = 0, last_sys_cpu_ = 0, last_user_cpu_ = 0;

  int start_bin_cnt_ = 0;
  int ekfom_vert_cnt_ = 0;
  int ekfom_obs_cnt_ = 0;
  int ekfom_iter_cnt_ = 0;

  Eigen::ArrayXi n_means_;
  Eigen::ArrayXXi n_cnts_;
  Eigen::ArrayXXi n_bins_;

  Eigen::ArrayXd ekfom_data_w_;
  Eigen::ArrayXd ekfom_data_sb_;
  Eigen::ArrayXd ekfom_data_om_;
  Eigen::ArrayXd ekfom_data_oe_;
  Eigen::ArrayXXd ekfom_data_ot_;
  Eigen::ArrayXXd ekfom_data_or_;
  Eigen::VectorXd ekfom_data_h_;
  Eigen::ArrayXd ekfom_data_w_x_;
  Eigen::MatrixXd ekfom_data_h_x_;
  Eigen::MatrixXd ekfom_data_h_x_r_;
  Eigen::ArrayXd ekfom_data_h_v_;
  Eigen::MatrixXd ekfom_data_h_x_v_;

  std::vector<float> traj_dist_;
  std::vector<Eigen::Vector3f> vel_poses_;
  std::vector<Eigen::Vector3f> poses_;
  std::vector<Eigen::Quaternionf> rotes_;

  std::vector<M3F> tensors_p1_;
  std::vector<M3F> tensors_p2_;
  std::vector<M3F> eigenvectors_;
  std::vector<V3F> eigenvalues_;
  std::vector<V3F> salivalues_;

  Eigen::ArrayXi raw_cloud_bins_;
  Eigen::ArrayXi scan_cloud_bins_;
  Eigen::ArrayXi buffer_cloud_bins_;
  std::vector<std::atomic<int>> scan_bin_sizes_;
  std::vector<std::atomic<int>> filter_bin_sizes_;

  std::vector<int> new_neighbours_map_idx_;
  std::vector<std::atomic<int>> updated_pt_;
  std::vector<std::vector<int>> new_neighbours_;
  std::vector<std::atomic<int>> new_neighbours_size_;

  std::vector<int> update_idx_;
  std::vector<int> saliency_idxs_;
  std::vector<std::vector<int>> neighbours_;
  std::vector<Eigen::Vector2i> filters_;

  EllipseLioPointCloudPtr map_cloud_{new EllipseLioPointCloud()};
  EllipseLioPointCloudPtr raw_cloud_{new EllipseLioPointCloud()};
  EllipseLioPointCloudPtr scan_cloud_{new EllipseLioPointCloud()};
  EllipseLioPointCloudPtr scan_cloud_grav_{new EllipseLioPointCloud()};
  EllipseLioPointCloudPtr filter_cloud_{new EllipseLioPointCloud()};
  EllipseLioPointCloudPtr buffer_cloud_{new EllipseLioPointCloud()};

  iOctree::Octree ioctree_;

  ImuParams imu_params_;
  LidarParams lidar_params_;

  IkfomSPtr kf_{new Ikfom()};
  KfState kf_state_;

  double imu_start_time_ = 0, imu_end_time_ = 0;
  double raw_start_time_ = 0, raw_end_time_ = 0;
  double scan_start_time_ = 0, scan_end_time_ = 0;
  double buffer_start_time_ = 0, buffer_end_time_ = 0;
  double last_imu_time_ = 0;

  double imu_time_offset_ = 0, lid_time_offset_ = 0;

  Analytics analytics_;

  std::unique_ptr<ImuProcess> imu_process_;
  std::unique_ptr<LidarProcess> lid_process_;

  std::vector<OdometryOutput> ready_outputs_;
};

}  // namespace ellipse_lio_core
