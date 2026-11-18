// SPDX-License-Identifier: GPL-3.0-only
// mola_ellipse_lio_wrapper: original code (not derived from EllipseLIO), the
// plain data types used at the ellipse_lio_core boundary. See
// THIRD_PARTY_NOTICES.md at the repository root for the licensing situation
// of the vendored EllipseLIO / i-Octree / IKFoM code this library wraps.
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ellipse_lio_core
{
/// Standard gravity, m/s^2 (same constant EllipseLIO / Fast-LIO2 use).
inline constexpr double G_m_s2 = 9.80665;

/// One input LiDAR point, decoupled from any PCL type.
struct LidarPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
  double t_offset_sec = 0;  //!< relative to the scan's start time
};
using LidarPointVector = std::vector<LidarPoint>;

/// One output (map/deskewed-scan) point: XYZI only, no per-point timing.
struct XYZIPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
};
using XYZIPointVector = std::vector<XYZIPoint>;

/// A single IMU sample, decoupled from any ROS message type.
struct ImuSample
{
  double t = 0;                                    //!< seconds, same clock as scan timestamps
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();   //!< m/s^2
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  //!< rad/s
};

/// Algorithm parameters, mirroring the schema of EllipseLIO's
/// `config/*.yaml` (upstream parameter names, dots replaced by underscores).
/// Fields under upstream's `cameras.*` are intentionally absent: this port
/// drops camera colorization entirely (see THIRD_PARTY_NOTICES.md).
struct Config
{
  // mapping.*
  double map_resolution = 0.1;
  int pub_map_n_secs = 10;

  // lidar.*
  /// LidType: 1=Livox, 2=Velodyne, 3=Ouster, 4=Hesai, 5=Gazebo. Kept for
  /// documentation/config-schema parity with upstream; this port receives
  /// already-decoded points (see LidarPointVector) so no vendor-specific
  /// point decoding happens here anymore.
  int lidar_type = 4;
  int lidar_rate = 10;
  int lidar_scan_lines = 64;
  double lidar_min_range = 1.0;
  double lidar_max_range = 100.0;
  double lidar_vertical_fov = 104.0;

  // imu.*
  int imu_rate = 400;
  double imu_gyr_noise = 0.1;
  double imu_acc_noise = 0.1;
  double imu_gyr_bias = 1e-4;
  double imu_acc_bias = 1e-4;

  /// Port addition, not in upstream's schema. Upstream ends the iterated
  /// update of a scan once it has run for a quarter of the scan period, to
  /// keep up in real time, so the number of iterations depends on the CPU
  /// speed and load and runs are not reproducible. `false` lifts that limit:
  /// the update is then bounded by its iteration count only, and offline runs
  /// are deterministic.
  bool iekf_wall_clock_limit = true;

  /// LiDAR-to-IMU extrinsics: p_imu = r_imu_lidar * p_lidar + t_imu_lidar.
  /// Despite the upstream field names ("t_imu_lidar"/"r_imu_lidar"), this was
  /// verified (see the pipeline YAML comment and the plan's E10 cross-check)
  /// to hold the LiDAR pose expressed in the IMU frame -- the exact same
  /// convention fast_lio2_core's extrinsic_R/T use.
  Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();
  Eigen::Matrix3d r_imu_lidar = Eigen::Matrix3d::Identity();
};

/// Plain-struct transcription of upstream's `msg/EllipseLioAnalytics.msg`,
/// field for field (same names), minus `cams_freq` (camera support was
/// dropped, see THIRD_PARTY_NOTICES.md). This is EllipseLIO's own per-scan
/// instrumentation payload: registration quality, primitive-type counts,
/// adaptive-weight statistics, map/octree size, and per-stage timings.
struct Analytics
{
  int64_t imu_freq = 0;
  int64_t lid_freq = 0;
  int64_t odom_freq = 0;

  int64_t map_size = 0;
  int64_t oct_num = 0;
  int64_t new_idxs = 0;
  int64_t upd_idxs = 0;
  int64_t mean_bin = 0;
  int64_t start_bin = 0;
  int64_t scan_size = 0;
  int64_t buffer_size = 0;
  double scan_time = 0;

  double res_mean = 0;
  double obs_min = 0;
  int64_t kf_iterations = 0;

  double obs_score = 0;
  double vert_score = 0;
  double bin_score = 0;

  int64_t num_planes = 0;
  int64_t num_lines = 0;
  int64_t num_balls = 0;
  int64_t num_feats = 0;
  int64_t num_reject = 0;

  double wt_mean = 0;
  double wt_std = 0;
  double wt_min = 0;
  double wt_max = 0;

  double rng_min = 0;
  double rng_max = 0;
  double rng_mean = 0;

  double imu_time = 0;
  double state_time = 0;
  double map_time = 0;
  double total_time = 0;

  double imu_mean = 0;
  double state_mean = 0;
  double map_mean = 0;
  double total_mean = 0;

  double imu_max = 0;
  double state_max = 0;
  double map_max = 0;
  double total_max = 0;

  double lid_offset = 0;
  double imu_offset = 0;

  int64_t run_time = 0;
  int64_t traj_dist = 0;
  int64_t ram_usage = 0;
  int64_t cpu_usage = 0;
};

/// Output of processing one LiDAR scan.
struct OdometryOutput
{
  double timestamp = 0;  //!< == the scan's end time

  /// world_from_imu, i.e. the pose of the IMU frame in the (arbitrary,
  /// gravity-unaligned) odometry world frame.
  Eigen::Isometry3d pose_imu = Eigen::Isometry3d::Identity();

  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

  /// Deskewed scan in LiDAR body frame (at the scan end pose).
  std::shared_ptr<const XYZIPointVector> deskewed_points_body;

  /// Per-scan instrumentation (see Analytics above): the research payload.
  Analytics analytics;
};

/// Simple log sink used instead of RCLCPP_* logging macros; level follows
/// MRPT's convention loosely (0=debug, 1=info, 2=warn, 3=error) but callers
/// may treat it as an opaque severity ordinal. Defaults to a no-op.
using LogFn = std::function<void(int level, const std::string & msg)>;

}  // namespace ellipse_lio_core
