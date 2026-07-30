/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   EllipseLioOdometry_Adapters.cpp
 * @brief  Conversions between MRPT/MOLA and ellipse_lio_core plain types
 */
#include "EllipseLioOdometry_Adapters.h"

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <algorithm>
#include <cmath>

namespace mola
{
using ellipse_lio_core::LidarPoint;
using ellipse_lio_core::LidarPointVector;
using ellipse_lio_core::XYZIPoint;
using ellipse_lio_core::XYZIPointVector;

/** Converts an incoming LiDAR observation into EllipseLIO points, and
 * computes the scan's absolute [t_beg, t_end] time span.
 *
 * If the point cloud carries a per-point relative-time channel
 * (`CPointsMap::POINT_FIELD_TIMESTAMP`, seconds, any convention -- centered
 * on the observation timestamp, start-relative, etc.), it is used directly:
 * each point's `t_offset_sec` (relative to scan start) is computed by
 * re-normalizing relative to this scan's own min/max, so no assumption
 * about the specific dataset's time convention is needed. This is the same
 * absolute-per-point-timestamp semantics EllipseLIO's own Hesai decoding
 * path (`SetPoint(HesaiPoint, ...)`) uses -- see LidarProcess.h.
 *
 * Otherwise (no per-point timestamps), falls back to synthesizing an offset
 * from each point's azimuth angle, assuming constant angular velocity
 * across `fallback_scan_period` seconds and that the observation's
 * timestamp marks the scan's end (last, forward-facing point).
 *
 * Unlike the Fast-LIO2 adapter, no minimum-range cut is applied here:
 * EllipseLIO's own `LidarProcess` performs `min_range`/`max_range`
 * filtering internally (see ellipse_lio_core/LidarProcess.cpp), so cutting
 * here too would just duplicate upstream's own behavior.
 */
LidarPointVector toEllipseLioCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end)
{
  LidarPointVector out;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    t_beg = t_end = mrpt::Clock::toDouble(obs.timestamp);
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();

  const auto * is =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  out.resize(n);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  if (ts && ts->size() == n && n > 0) {
    float tMin = (*ts)[0], tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }
    t_beg = obsTime + tMin;
    t_end = obsTime + tMax;

    for (size_t i = 0; i < n; i++) {
      LidarPoint & p = out[i];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i] - tMin);
    }
  } else {
    // Fallback: assume `obsTime` is the scan's end (forward-facing point),
    // and synthesize each point's offset from its azimuth angle.
    t_beg = obsTime - fallback_scan_period;
    t_end = obsTime;

    for (size_t i = 0; i < n; i++) {
      LidarPoint & p = out[i];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;

      // Azimuth in [0, 2*pi), 0 at the scan start (fallback_scan_period ago).
      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t_offset_sec = azimuth01 * fallback_scan_period;
    }
  }

  return out;
}

ellipse_lio_core::ImuSample toEllipseLioImu(const mrpt::obs::CObservationIMU & obs)
{
  ellipse_lio_core::ImuSample s;
  s.t = mrpt::Clock::toDouble(obs.timestamp);
  s.acc = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_X_ACC), obs.get(mrpt::obs::IMU_Y_ACC), obs.get(mrpt::obs::IMU_Z_ACC));
  s.gyro = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_WX), obs.get(mrpt::obs::IMU_WY), obs.get(mrpt::obs::IMU_WZ));
  return s;
}

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const XYZIPointVector & points)
{
  auto out = mrpt::maps::CSimplePointsMap::Create();
  out->reserve(points.size());
  for (const auto & p : points) {
    out->insertPointFast(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace mola
