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
 * @file   EllipseLioOdometry.cpp
 * @brief  MOLA front-end module wrapping the EllipseLIO algorithm
 */
// clang-format off
// EllipseLioOdometry_Adapters.h must come first: it pulls in
// ellipse_lio_core/Types.hpp (PCL headers) before mrpt/maps/CPointsMap.h
// (below, via the mrpt/obs headers), which the latter needs to enable its
// inline PCL PCD I/O methods. See EllipseLioOdometry_Adapters.h's own
// comment for the full explanation.
#include "EllipseLioOdometry_Adapters.h"
// clang-format on

#include <ellipse_lio_core/EllipseLioCore.h>
#include <mola_ellipse_lio_wrapper/EllipseLioOdometry.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/stock_objects.h>

#include <sstream>

namespace mola
{
IMPLEMENTS_MRPT_OBJECT(EllipseLioOdometry, FrontEndBase, mola)

namespace
{
/// Parses "x y z yaw_deg pitch_deg roll_deg".
mrpt::poses::CPose3D parsePose6D(const std::string & s)
{
  std::istringstream iss(s);
  double x, y, z, yaw, pitch, roll;
  if (!(iss >> x >> y >> z >> yaw >> pitch >> roll)) {
    THROW_EXCEPTION_FMT(
      "Could not parse pose string '%s' (expected: 'x y z "
      "yaw_deg pitch_deg roll_deg')",
      s.c_str());
  }
  return {x, y, z, mrpt::DEG2RAD(yaw), mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll)};
}

mrpt::poses::CPose3D isometryToPose(const Eigen::Isometry3d & iso)
{
  mrpt::math::CMatrixDouble44 m;
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++) m(r, c) = iso.matrix()(r, c);
  return mrpt::poses::CPose3D(m);
}
}  // namespace

EllipseLioOdometry::EllipseLioOdometry() = default;
EllipseLioOdometry::~EllipseLioOdometry() { worker_lidar_.clear(); }

void EllipseLioOdometry::initialize_frontend(const Yaml & cfg)
{
  MRPT_TRY_START

  std::string lidar_sensor_label = "lidar", imu_sensor_label = "imu";
  YAML_LOAD_OPT(lidar_sensor_label, std::string);
  YAML_LOAD_OPT(imu_sensor_label, std::string);
  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);
  imu_sensor_label_regex_ = std::regex(imu_sensor_label);

  // The YAML key is the variable name: "imu_pose_in_lidar", as in the shipped
  // pipelines and the other LIO wrappers.
  std::string imu_pose_in_lidar = "0 0 0 0 0 0";
  YAML_LOAD_OPT(imu_pose_in_lidar, std::string);
  imu_pose_in_lidar_ = parsePose6D(imu_pose_in_lidar);

  // Pose of the LiDAR frame wrt base_link (T_base_lidar): same key name and
  // convention as the DLIO, Fast-LIO2 and GLIM wrappers'.
  std::string baselink2lidar_pose_str = "0 0 0 0 0 0";
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);
  baselink_pose_of_lidar_ = parsePose6D(baselink2lidar_pose_str);

  imu_to_output_frame_ = worldFromBaseLink(
    mrpt::poses::CPose3D::Identity(), imu_pose_in_lidar_, baselink_pose_of_lidar_);
  output_frame_id_ =
    baselink_pose_of_lidar_ == mrpt::poses::CPose3D::Identity() ? "lidar" : "base_link";

  YAML_LOAD_MEMBER_OPT(fallback_scan_period, double);

  YAML_LOAD_OPT3(core_cfg_, map_resolution, double);
  YAML_LOAD_OPT3(core_cfg_, pub_map_n_secs, int);

  YAML_LOAD_OPT3(core_cfg_, lidar_type, int);
  YAML_LOAD_OPT3(core_cfg_, lidar_rate, int);
  YAML_LOAD_OPT3(core_cfg_, lidar_scan_lines, int);
  YAML_LOAD_OPT3(core_cfg_, lidar_min_range, double);
  YAML_LOAD_OPT3(core_cfg_, lidar_max_range, double);
  YAML_LOAD_OPT3(core_cfg_, lidar_vertical_fov, double);

  YAML_LOAD_OPT3(core_cfg_, imu_rate, int);
  YAML_LOAD_OPT3(core_cfg_, imu_gyr_noise, double);
  YAML_LOAD_OPT3(core_cfg_, imu_acc_noise, double);
  YAML_LOAD_OPT3(core_cfg_, imu_gyr_bias, double);
  YAML_LOAD_OPT3(core_cfg_, imu_acc_bias, double);

  YAML_LOAD_OPT3(visualization_params_, show_trajectory, bool);
  YAML_LOAD_OPT3(visualization_params_, show_localmap, bool);
  YAML_LOAD_OPT3(visualization_params_, camera_follows_vehicle, bool);
  YAML_LOAD_OPT3(visualization_params_, current_pose_corner_size, float);
  YAML_LOAD_OPT3(visualization_params_, local_map_point_size, float);
  YAML_LOAD_OPT3(visualization_params_, map_update_decimation, int);

  // EllipseLIO's r_imu_lidar/t_imu_lidar are "LiDAR wrt IMU"
  // (p_imu = r_imu_lidar*p_lidar + t_imu_lidar), the exact same convention
  // fast_lio2_core's extrinsic_R/T use, verified in the pipeline YAML's
  // comment -- i.e. the inverse of the "IMU wrt LiDAR" pose used above
  // (which matches the `fixed_sensor_pose` convention already used for IMU
  // sensors in MOLA dataset YAMLs, so pipeline authors can reuse the same
  // numbers).
  mrpt::poses::CPose3D lidarPoseInImu = imu_pose_in_lidar_;
  lidarPoseInImu.inverse();
  core_cfg_.r_imu_lidar = lidarPoseInImu.getRotationMatrix().asEigen();
  core_cfg_.t_imu_lidar =
    Eigen::Vector3d(lidarPoseInImu.x(), lidarPoseInImu.y(), lidarPoseInImu.z());

  auto logSink = [this](int level, const std::string & msg) {
    switch (level) {
      case 3:
        MRPT_LOG_ERROR_STREAM(msg);
        break;
      case 2:
        MRPT_LOG_WARN_STREAM(msg);
        break;
      default:
        MRPT_LOG_INFO_STREAM(msg);
        break;
    }
  };

  core_ = std::make_unique<ellipse_lio_core::EllipseLioCore>(core_cfg_, logSink);

  MRPT_LOG_INFO_STREAM(
    "EllipseLioOdometry initialized. lidar_sensor_label='"
    << lidar_sensor_label << "' imu_sensor_label='" << imu_sensor_label << "'");

  MRPT_TRY_END
}

void EllipseLioOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  ASSERT_(o);

  if (std::regex_match(o->sensorLabel, imu_sensor_label_regex_)) {
    onImuObservation(o);
  } else if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidarObservation(o);
  }
  MRPT_TRY_END
}

void EllipseLioOdometry::onImuObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
  if (!imu || !core_) {
    return;
  }

  // Cheap: run synchronously. IMU is the deskew clock, it must never be
  // dropped -- not even in live/online mode.
  core_->addImu(toEllipseLioImu(*imu));

  for (const auto & out : core_->drainReady()) {
    publishOutput(out);
  }
}

void EllipseLioOdometry::onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!core_) return;

  tasks_in_flight_++;
  auto fut = worker_lidar_.enqueue(&EllipseLioOdometry::processLidarScan, this, o);
  (void)fut;
}

void EllipseLioOdometry::processLidarScan(mrpt::obs::CObservation::ConstPtr o)
{
  // On the worker thread an exception would be lost in the discarded future:
  // report it, and always release the in-flight count, or isBusy() would
  // stay true forever.
  try {
    processLidarScanImpl(o);
  } catch (const std::exception & e) {
    MRPT_LOG_ERROR_STREAM("Error processing a LiDAR scan: " << e.what());
  } catch (...) {
    MRPT_LOG_ERROR("Unknown error processing a LiDAR scan");
  }
  tasks_in_flight_--;
}

void EllipseLioOdometry::processLidarScanImpl(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (pc && core_) {
    double t_beg, t_end;
    auto cloud = toEllipseLioCloud(*pc, fallback_scan_period_, t_beg, t_end);
    core_->addLidarScan(cloud, t_beg, t_end);
    for (const auto & out : core_->drainReady()) {
      publishOutput(out);
    }
  }
}

void EllipseLioOdometry::publishOutput(const ellipse_lio_core::OdometryOutput & out)
{
  // EllipseLIO estimates the IMU pose; report base_link (or the LiDAR, when
  // it is the reference frame) as the other MOLA modules do.
  const mrpt::poses::CPose3D imuPose = isometryToPose(out.pose_imu);
  const mrpt::poses::CPose3D outputPose = imuPose + imu_to_output_frame_;

  const auto timestamp = mrpt::Clock::fromDouble(out.timestamp);

  {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    trajectory_.insert(timestamp, outputPose);
  }

  if (anyUpdateLocalizationSubscriber()) {
    LocalizationUpdate lu;
    lu.timestamp = timestamp;
    lu.reference_frame = "map";
    lu.child_frame = output_frame_id_;
    lu.method = "ellipse_lio";
    lu.pose = outputPose.asTPose();
    advertiseUpdatedLocalization(lu);
  }

  if (anyUpdateMapSubscriber()) {
    MapUpdate mu;
    mu.timestamp = timestamp;
    mu.reference_frame = "map";
    mu.method = "ellipse_lio";
    mu.map_name = "local_map";
    mu.map = toMrptPointsMap(*core_->currentMap());
    mu.keep_last_one_only = true;
    advertiseUpdatedMap(mu);
  }

  updateVisualization(outputPose, out);
}

mrpt::poses::CPose3D EllipseLioOdometry::worldFromBaseLink(
  const mrpt::poses::CPose3D & worldFromImu, const mrpt::poses::CPose3D & imuPoseInLidar,
  const mrpt::poses::CPose3D & lidarPoseInBaseLink)
{
  mrpt::poses::CPose3D lidarPoseInImu = imuPoseInLidar;
  lidarPoseInImu.inverse();
  mrpt::poses::CPose3D baseLinkPoseInLidar = lidarPoseInBaseLink;
  baseLinkPoseInLidar.inverse();
  return worldFromImu + lidarPoseInImu + baseLinkPoseInLidar;
}

void EllipseLioOdometry::updateVisualization(
  const mrpt::poses::CPose3D & lidarPose, const ellipse_lio_core::OdometryOutput & out)
{
  if (!visualizer_) return;

  // Vehicle pose corner:
  if (visualization_params_.current_pose_corner_size > 0) {
    auto glVehicle = mrpt::viz::CSetOfObjects::Create();
    glVehicle->insert(
      mrpt::viz::stock_objects::CornerXYZ(visualization_params_.current_pose_corner_size));
    glVehicle->setPose(lidarPose);
    visualizer_->update_3d_object("ellipselio/vehicle", glVehicle);
  }

  updateVisualizationPath(lidarPose);
  updateVisualizationLocalMap();

  if (visualization_params_.camera_follows_vehicle) {
    const mrpt::math::TPoint3Df lookAt(lidarPose.x(), lidarPose.y(), lidarPose.z());
    visualizer_->update_viewport_look_at(lookAt);
  }

  if (!gui_created_) {
    internalBuildGUI();
    gui_created_ = true;
  }
  updateVisualizationTextLabels(out);
}

void EllipseLioOdometry::updateVisualizationPath(const mrpt::poses::CPose3D & lidarPose)
{
  if (!visualization_params_.show_trajectory) {
    visualizer_->update_3d_object("ellipselio/path", mrpt::viz::CSetOfObjects::Create());
    return;
  }

  if (!gl_estimated_path_) {
    gl_estimated_path_ = mrpt::viz::CSetOfLines::Create();
    gl_estimated_path_->setColor_u8(0x20, 0x20, 0xe0, 0xff);
  }

  const auto t = lidarPose.translation();
  if (gl_estimated_path_->empty()) {
    gl_estimated_path_->appendLine(t, t);
  } else {
    gl_estimated_path_->appendLineStrip(t);
  }

  // Hand a fresh copy to the GUI thread so this (worker-thread-owned) buffer
  // can keep growing on later ticks without racing with the GUI thread's
  // read of the previously-published object.
  auto pathGrp = mrpt::viz::CSetOfObjects::Create();
  pathGrp->insert(mrpt::viz::CSetOfLines::Create(*gl_estimated_path_));
  visualizer_->update_3d_object("ellipselio/path", pathGrp);
}

void EllipseLioOdometry::updateVisualizationLocalMap()
{
  if (!visualization_params_.show_localmap) {
    visualizer_->update_3d_object("ellipselio/localmap", mrpt::viz::CSetOfObjects::Create());
    return;
  }

  if (++map_viz_update_counter_ < visualization_params_.map_update_decimation) return;
  map_viz_update_counter_ = 0;

  const auto mapPts = core_->currentMap();
  if (!mapPts || mapPts->empty()) return;

  const auto pointsMap = toMrptPointsMap(*mapPts);

  auto glCloud = mrpt::viz::CPointCloudColoured::Create();
  glCloud->loadFromPointsMap(pointsMap.get());
  glCloud->setPointSize(visualization_params_.local_map_point_size);

  const auto bbox = pointsMap->boundingBox();
  glCloud->recolorizeByCoordinate(bbox.min.z, bbox.max.z, 2 /*Z*/, mrpt::img::TColormap::cmJET);

  auto grp = mrpt::viz::CSetOfObjects::Create();
  grp->insert(glCloud);
  visualizer_->update_3d_object("ellipselio/localmap", grp);
}

void EllipseLioOdometry::internalBuildGUI()
{
  using namespace mola::gui;

  gui_.lbStatus = std::make_shared<LiveString>(" ");

  Tab tab;
  tab.title = "Status";
  tab.widgets.emplace_back(Label{gui_.lbStatus});
  tab.widgets.emplace_back(CheckBox{
    "Show trajectory", visualization_params_.show_trajectory,
    [this](bool checked) { visualization_params_.show_trajectory = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Show local map", visualization_params_.show_localmap,
    [this](bool checked) { visualization_params_.show_localmap = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Camera follows vehicle", visualization_params_.camera_follows_vehicle,
    [this](bool checked) { visualization_params_.camera_follows_vehicle = checked; }});

  WindowDescription desc;
  desc.title = "EllipseLIO";
  desc.position = {5, 700};
  desc.size = {320, 0};
  desc.tabs.emplace_back(std::move(tab));

  visualizer_->create_subwindow_from_description(desc);
}

void EllipseLioOdometry::updateVisualizationTextLabels(const ellipse_lio_core::OdometryOutput & out)
{
  if (!gui_.lbStatus) return;

  const auto & a = out.analytics;
  const auto mapPts = core_->currentMap();
  gui_.lbStatus->set(mrpt::format(
    "t=%.03f | map points: %zu | busy: %s\n"
    "kf_iter=%lld res_mean=%.04f\n"
    "planes=%lld lines=%lld balls=%lld reject=%lld",
    out.timestamp, mapPts ? mapPts->size() : std::size_t{0}, isBusy() ? "yes" : "no",
    static_cast<long long>(a.kf_iterations), a.res_mean, static_cast<long long>(a.num_planes),
    static_cast<long long>(a.num_lines), static_cast<long long>(a.num_balls),
    static_cast<long long>(a.num_reject)));
}

void EllipseLioOdometry::spinOnce() {}

bool EllipseLioOdometry::isBusy() const
{
  return tasks_in_flight_.load() > 0 || worker_lidar_.pendingTasks() > 0;
}

mrpt::poses::CPose3DInterpolator EllipseLioOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
