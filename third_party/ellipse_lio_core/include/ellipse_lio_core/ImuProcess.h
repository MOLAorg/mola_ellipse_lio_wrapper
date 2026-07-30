// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's include/imu_processing.h
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. Ported: `rclcpp::Time` -> `double`
// seconds; `sensor_msgs::msg::Imu` -> `ellipse_lio_core::ImuSample`; the ROS
// subscription/callback-group machinery is replaced by a public `feedImu()`
// entry point the caller (the MOLA module, or the offline CLI) drives
// directly; `RCLCPP_*` logging replaced by an optional `LogFn` sink; the
// camera-colorization path (`GetMatchingImages`/`ColorisePoint`, the `cams`
// parameter of `UndistortPointCloud`) was removed together with camera
// support (see THIRD_PARTY_NOTICES.md). Also dropped: `SetGyrCov`,
// `SetAccCov`, `SetGyrBiasCov`, `SetAccBiasCov`, `SetExtrinsic` -- declared
// in the upstream header but never defined or called anywhere in the
// upstream repository (dead API surface, confirmed by inspection). Gravity/
// bias initialization and IMU propagation/undistortion logic is otherwise
// unchanged.
#pragma once

#include <ellipse_lio_core/CommonLib.h>

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <ellipse_lio_core/Types.hpp>
#include <mutex>

namespace ellipse_lio_core
{
/// Standard gravity acceleration (m/s^2), and default LiDAR-point covariance
/// used by the iterated update -- same values EllipseLIO's imu_processing.h
/// declares.
inline constexpr double kGravityMetersPerSecondSquared = 9.80665;
inline constexpr double kLidarPointCovariance = 0.001;

/// IMU sensor parameters and calibration data (mirrors upstream's
/// `ImuParams`, minus the ROS topic name).
struct ImuParams
{
  int rate = 400;
  double gyr_noise = 0.1;
  double acc_noise = 0.1;
  double gyr_bias = 1e-4;
  double acc_bias = 1e-4;
  V3D t_imu_lidar = V3D::Zero();
  M3D r_imu_lidar = M3D::Identity();
};

/// Processes IMU measurements and integrates them with the EKF: buffers
/// samples, performs gravity/bias initialization, IMU-based scan
/// undistortion, and the iterated-EKF LiDAR measurement update. No ROS
/// dependency; see EllipseLioCore for the orchestration this class used to
/// get from `MappingNode`.
class ImuProcess
{
public:
  ImuProcess(IkfomSPtr kf, ImuParams params, LogFn log = {});
  ~ImuProcess();

  /// Feeds one IMU sample: buffers it and, if initialized, immediately runs
  /// the EKF prediction step (mirrors upstream's per-message `ImuCallback`).
  void feedImu(const ImuSample & s);

  /// Undistorts a LiDAR scan (in place) using the IMU trajectory in
  /// `synced_imu_states_` (see SyncWithLidar()), and reports the filter
  /// state matched to `lidar_end_time`.
  void UndistortPointCloud(
    EllipseLioPointCloudPtr pc, KfState * kf_state, double lidar_start_time, double lidar_end_time);

  /// Runs the iterated-EKF LiDAR measurement update (via the h_share_model
  /// callback registered on `kf`), re-propagating every buffered IMU sample
  /// after the matched state.
  void UpdateStatesWithLidar(KfState * kf_state, double lidar_end_time, double max_solve_time);

  /// Thread-safe accessor for the latest publishable filter state.
  void GetKfState(KfState * kf_state) const;

  /// Snapshots the IMU state buffer for use by UndistortPointCloud().
  void SyncWithLidar(double * imu_start_time, double * imu_end_time);

  std::atomic<int> imu_counter_{0};
  bool imu_has_data_ = false;
  bool lidar_ready_ = false;
  double imu_start_time_ = 0;
  double imu_end_time_ = 0;
  Eigen::Matrix<double, 12, 12> q_;

private:
  void SetKfState();
  void Process(const ImuSample & s);
  void InitImu(const ImuSample & s);
  void GetTimeMatch(
    int * match_idx, double match_time, const boost::circular_buffer<ImuState> & imu_states);

  IkfomSPtr kf_;
  KfState kf_state_, pub_kf_state_;

  ImuParams params_;
  LogFn log_;

  std::mutex imu_mutex_;
  mutable std::mutex pub_mutex_;

  boost::circular_buffer<ImuState> imu_states_, synced_imu_states_;

  V3D mean_acc_ = V3D::Zero();
  V3D mean_gyr_ = V3D::Zero();
  V3D acc_noise_ = V3D::Zero();
  V3D gyr_noise_ = V3D::Zero();
  V3D acc_bias_ = V3D::Zero();
  V3D gyr_bias_ = V3D::Zero();

  int init_iter_num_ = 1;
  double last_imu_time_ = 0;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

}  // namespace ellipse_lio_core
