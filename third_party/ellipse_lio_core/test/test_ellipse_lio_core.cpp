// SPDX-License-Identifier: GPL-3.0-only
// mola_ellipse_lio_wrapper: original test code (not derived from EllipseLIO).
#include <ellipse_lio_core/EllipseLioCore.h>
#include <gtest/gtest.h>

namespace
{
using ellipse_lio_core::Config;
using ellipse_lio_core::EllipseLioCore;
using ellipse_lio_core::G_m_s2;
using ellipse_lio_core::ImuSample;
using ellipse_lio_core::LidarPoint;
using ellipse_lio_core::LidarPointVector;
using ellipse_lio_core::OdometryOutput;

// A hollow box (floor, ceiling, four walls) centered on the sensor,
// simulating a LiDAR inside a small room: gives the tensor-voting map
// enough dense, multi-bin structure to build planes. Every point gets a
// synthetic (linearly spaced) per-point relative timestamp.
LidarPointVector makeBoxScan(double t_scan_period)
{
  LidarPointVector cloud;
  int idx = 0;

  const auto addPoint = [&](float x, float y, float z) {
    LidarPoint p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 0.5f;
    p.t_offset_sec = t_scan_period * (idx % 100) / 100.0;
    cloud.push_back(p);
    idx++;
  };

  constexpr float kStep = 0.25f;
  constexpr float kHalfSide = 2.5f;
  constexpr float kFloorZ = -2.0f;
  constexpr float kCeilZ = 2.0f;

  for (float x = -kHalfSide; x <= kHalfSide; x += kStep) {
    for (float y = -kHalfSide; y <= kHalfSide; y += kStep) {
      addPoint(x, y, kFloorZ);
      addPoint(x, y, kCeilZ);
    }
  }
  for (float x = -kHalfSide; x <= kHalfSide; x += kStep) {
    for (float z = kFloorZ; z <= kCeilZ; z += kStep) {
      addPoint(x, -kHalfSide, z);
      addPoint(x, kHalfSide, z);
    }
  }
  for (float y = -kHalfSide; y <= kHalfSide; y += kStep) {
    for (float z = kFloorZ; z <= kCeilZ; z += kStep) {
      addPoint(-kHalfSide, y, z);
      addPoint(kHalfSide, y, z);
    }
  }
  return cloud;
}

}  // namespace

TEST(EllipseLioCore, StationaryRunConvergesAndBuildsMap)
{
  Config cfg;
  cfg.map_resolution = 0.2;
  cfg.lidar_min_range = 0.3;
  cfg.lidar_max_range = 20.0;
  cfg.lidar_scan_lines = 32;
  cfg.lidar_vertical_fov = 90.0;
  cfg.imu_rate = 200;
  cfg.lidar_rate = 10;
  cfg.imu_gyr_noise = 0.01;
  cfg.imu_acc_noise = 0.01;
  cfg.imu_gyr_bias = 1e-5;
  cfg.imu_acc_bias = 1e-5;

  EllipseLioCore core(cfg);

  constexpr double kImuRateHz = 200.0;
  constexpr double kScanRateHz = 10.0;
  constexpr double kImuDt = 1.0 / kImuRateHz;
  constexpr double kScanDt = 1.0 / kScanRateHz;
  constexpr int kNumScans = 20;

  double t = 0.0;
  double t_scan = 0.0;
  bool got_any_output = false;

  OdometryOutput last_output;

  for (int scan_i = 0; scan_i < kNumScans; scan_i++) {
    const double t_scan_end = t_scan + kScanDt;

    while (t <= t_scan_end + kImuDt) {
      ImuSample s;
      s.t = t;
      s.acc = Eigen::Vector3d(0, 0, G_m_s2);
      s.gyro = Eigen::Vector3d::Zero();
      core.addImu(s);
      for (const auto & out : core.drainReady()) {
        got_any_output = true;
        last_output = out;
      }
      t += kImuDt;
    }

    core.addLidarScan(makeBoxScan(kScanDt), t_scan, t_scan_end);
    for (const auto & out : core.drainReady()) {
      got_any_output = true;
      last_output = out;
      ASSERT_TRUE(out.pose_imu.matrix().allFinite());
      ASSERT_TRUE(out.vel.allFinite());
    }

    t_scan = t_scan_end;
  }

  EXPECT_TRUE(got_any_output) << "expected at least one OdometryOutput after " << kNumScans
                              << " scans";
  if (got_any_output) {
    // Purely stationary run: position should stay close to the origin.
    EXPECT_LT(last_output.pose_imu.translation().norm(), 2.0);
    // Gravity magnitude should converge close to G_m_s2.
    EXPECT_NEAR(last_output.gravity.norm(), G_m_s2, 2.0);

    // The rolling registration scores are means of values clamped to [1e-4, 1].
    const auto & a = last_output.analytics;
    for (const double score : {a.obs_score, a.vert_score, a.cov_score, a.wt_score, a.bin_score}) {
      EXPECT_GT(score, 0.0);
      EXPECT_LE(score, 1.0 + 1e-9);
    }
    EXPECT_GE(a.kf_iterations, 1);
  }

  auto map = core.currentMap();
  ASSERT_TRUE(static_cast<bool>(map));
  EXPECT_GT(map->size(), 0u);
}
