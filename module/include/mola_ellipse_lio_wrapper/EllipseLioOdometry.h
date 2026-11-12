/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: wraps EllipseLIO (third_party/ellipse_lio_core),
 copyright Rowan Border and Margarita Chli (V4RL, University of Cyprus).
*/

/**
 * @file   EllipseLioOdometry.h
 * @brief  MOLA front-end module wrapping the EllipseLIO algorithm
 */
#pragma once

// Only the light, PCL/i-Octree/IKFoM-free plain-data header: this keeps
// consumers of this public header from having to add ellipse_lio_core's
// (internal, non-installed) include dirs to their own build. The full
// ellipse_lio_core/EllipseLioCore.h is only included from the .cpp file.
#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/viz/CSetOfLines.h>

#include <atomic>
#include <ellipse_lio_core/Types.hpp>
#include <memory>
#include <mutex>
#include <regex>
#include <vector>

namespace ellipse_lio_core
{
class EllipseLioCore;
}

namespace mola
{
/** MOLA online front-end wrapping the EllipseLIO LiDAR-inertial odometry
 * algorithm (see `ellipse_lio_core`, in `third_party/`).
 *
 * Consumes `mrpt::obs::CObservationPointCloud` (LiDAR) and
 * `mrpt::obs::CObservationIMU` (IMU) observations and publishes live
 * localization + map updates via `LocalizationSourceBase`/`MapSourceBase`
 * (consumed e.g. by `mola_bridge_ros2` to republish `/tf` and topics).
 *
 * For the interactive MOLA GUI (`mola_viz`/`mola_viz_imgui`), this class
 * additionally drives the `VizInterface` directly (like `mola::LidarOdometry`
 * and `mola::FastLio2Odometry` do), drawing the current vehicle pose, the
 * estimated trajectory and the growing local map in the 3-D scene, plus a
 * small status sub-window with EllipseLIO's own per-scan instrumentation
 * (registration iterations, mean residual, primitive-type counts, map size).
 *
 * LiDAR scans are processed on a single-thread worker pool with the
 * `POLICY_DROP_OLD` policy: fine for live use, NOT loss-free. The offline
 * CLI (`mola-ellipse-lio-cli`) achieves the loss-free guarantee by feeding
 * one scan at a time and busy-waiting on `isBusy()` before advancing.
 *
 * \ingroup mola_ellipse_lio_wrapper_grp
 */
class EllipseLioOdometry : public FrontEndBase, public LocalizationSourceBase, public MapSourceBase
{
  DEFINE_MRPT_OBJECT(EllipseLioOdometry, mola)

public:
  EllipseLioOdometry();
  ~EllipseLioOdometry() override;

  // ExecutableBase
  void spinOnce() override;

  // RawDataConsumer
  void onNewObservation(const mrpt::obs::CObservation::ConstPtr & o) override;

  /** True while the LiDAR worker still has a scan queued or running.
   * Polled by `mola-ellipse-lio-cli` to guarantee no scan is ever dropped:
   * it feeds one observation, then waits for `isBusy()==false` before
   * feeding the next. */
  bool isBusy() const;

  /** Trajectory accumulated so far from this module's own localization
   * updates (`map` -> `lidar`, i.e. deskewed-scan-frame poses). */
  mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

  /** The pose this module reports, from the IMU pose EllipseLIO estimates:
   *
   *   T_w_base = T_w_imu . T_lidar_imu^-1 . T_base_lidar^-1
   *
   * @param worldFromImu        what EllipseLIO estimates, `T_w_imu`.
   * @param imuPoseInLidar      `T_lidar_imu`, YAML `imu_pose_in_lidar`.
   * @param lidarPoseInBaseLink `T_base_lidar`, YAML `baselink2lidar_pose_str`.
   */
  static mrpt::poses::CPose3D worldFromBaseLink(
    const mrpt::poses::CPose3D & worldFromImu, const mrpt::poses::CPose3D & imuPoseInLidar,
    const mrpt::poses::CPose3D & lidarPoseInBaseLink);

protected:
  void initialize_frontend(const Yaml & cfg) override;

private:
  void onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void onImuObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void processLidarScan(mrpt::obs::CObservation::ConstPtr o);
  void processLidarScanImpl(const mrpt::obs::CObservation::ConstPtr & o);
  void publishOutput(const ellipse_lio_core::OdometryOutput & out);

  // --- GUI / 3D scene visualization (mola_viz / mola_viz_imgui) ---
  void updateVisualization(
    const mrpt::poses::CPose3D & lidarPose, const ellipse_lio_core::OdometryOutput & out);
  void updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose);
  void updateVisualizationLocalMap();
  void internalBuildGUI();
  void updateVisualizationTextLabels(const ellipse_lio_core::OdometryOutput & out);

  std::regex lidar_sensor_label_regex_{"lidar"};
  std::regex imu_sensor_label_regex_{"imu"};

  /// Pose of the IMU frame wrt the LiDAR frame (same convention as the
  /// `fixed_sensor_pose` entries dataset YAMLs already use for the IMU
  /// sensor when the LiDAR is the reference/base_link frame).
  mrpt::poses::CPose3D imu_pose_in_lidar_;

  /// Pose of the LiDAR frame wrt `base_link`, i.e. `T_base_lidar`. Identity
  /// when the LiDAR itself is the reference frame.
  mrpt::poses::CPose3D baselink_pose_of_lidar_;

  /// Constant pose of the output frame wrt the IMU, see worldFromBaseLink().
  mrpt::poses::CPose3D imu_to_output_frame_;

  /// Name of the frame the reported pose lands in.
  std::string output_frame_id_ = "lidar";

  /// Used only as a fallback when an incoming point cloud has no per-point
  /// timestamp channel (synthesizes one from azimuth angle, assuming a
  /// constant angular velocity across this many seconds per scan).
  double fallback_scan_period_ = 0.1;

  ellipse_lio_core::Config core_cfg_;
  std::unique_ptr<ellipse_lio_core::EllipseLioCore> core_;

  mrpt::WorkerThreadsPool worker_lidar_{
    1, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "ellipselio_lidar"};
  std::atomic_int tasks_in_flight_{0};

  mutable std::mutex trajectory_mtx_;
  mrpt::poses::CPose3DInterpolator trajectory_;

  /// Toggles read by the "View" GUI tab checkboxes and by the viz update
  /// code, both potentially running on different threads (GUI thread vs.
  /// the LiDAR worker thread), hence `std::atomic`.
  struct VisualizationParams
  {
    std::atomic_bool show_trajectory{true};
    std::atomic_bool show_localmap{true};
    std::atomic_bool camera_follows_vehicle{true};

    float current_pose_corner_size = 1.0f;
    float local_map_point_size = 2.0f;

    /// How many processed scans to wait between local map viz refreshes
    /// (rebuilding the point cloud GL object is the costliest viz step).
    int map_update_decimation = 10;
  } visualization_params_;

  /// Only touched from the (single-threaded) LiDAR worker, since that is
  /// the only thread that ever calls updateVisualization().
  mrpt::viz::CSetOfLines::Ptr gl_estimated_path_;
  int map_viz_update_counter_ = 0;
  bool gui_created_ = false;

  struct Gui
  {
    mola::gui::LiveString::Ptr lbStatus;
  } gui_;
};

}  // namespace mola
