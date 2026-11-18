// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's include/common_lib.h
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. Ported: `rclcpp::Time` fields replaced by
// plain `double` seconds; the ROS-parameter-array conversion helpers
// (Vec3dFromArray/QuaterniondFromArray/Mat3dFromArray) were dropped since
// this port receives already-parsed Eigen types via Config (see Types.hpp).
// Constants and the KfState/ImuState struct shapes are otherwise unchanged.
// See THIRD_PARTY_NOTICES.md.
#pragma once

#include <ellipse_lio_core/CommonPcl.h>
#include <ikfom/use_ikfom.h>

#include <Eigen/Eigen>

/// @name Physical Constants
/// @{
inline constexpr double kMaxStartBin = 10.0;
inline constexpr int kMinNeighbours = 6;
inline constexpr int kMaxNeighbours = 60;
inline constexpr double kMinMapRes = 0.1;
inline constexpr double kMinBinRes = 0.01;
inline constexpr double kMinSearchRes = 0.1;
inline constexpr double kMaxSearchRes = 1.0;
inline constexpr int kMinProcPoints = 1000;
inline constexpr int kMaxProcPoints = 30000;
inline constexpr int kMaxScanPoints = 200000;
inline constexpr int kMaxMapPoints = 10000000;
/// @}

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using V3F = Eigen::Vector3f;
using M3F = Eigen::Matrix3f;

inline M3D Eye3d(M3D::Identity());
inline M3F Eye3f(M3F::Identity());
inline V3D Zero3d(0, 0, 0);
inline V3F Zero3f(0, 0, 0);

/// Complete state of the Kalman filter at a given time: IMU-centric state
/// (position, velocity, rotation), covariance, and latest gyro measurement.
struct KfState
{
  double time = 0;  //!< seconds
  state_ikfom state;
  Ikfom::cov cov;
  V3D gyr;
};
using KfStateSPtr = std::shared_ptr<KfState>;

/// IMU measurement and corresponding filter state at a point in time, used
/// to reconstruct the trajectory during scan undistortion.
struct ImuState
{
  KfState state;
  V3D acc;
  V3D gyr;
  V3D acc_avr;  //!< time-averaged acceleration over the integration step
  V3D gyr_avr;  //!< time-averaged gyro over the integration step
};
