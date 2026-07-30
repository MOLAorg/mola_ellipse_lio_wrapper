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
 * @file   EllipseLioOdometry_Adapters.h
 * @brief  Conversions between MRPT/MOLA and ellipse_lio_core plain types
 *         (private to this module, not installed).
 */
#pragma once

#include <ellipse_lio_core/Types.hpp>

// clang-format off
// ellipse_lio_core/Types.hpp (above) already includes enough PCL headers to
// define PCL_LINEAR_VERSION, which makes mrpt/maps/CPointsMap.h (below)
// conditionally compile inline PCL PCD I/O methods -- but only this header
// actually declares the `pcl::io::savePCDFile`/`loadPCDFile` symbols those
// methods call. MRPT's own comment on those methods: "requires user code to
// include PCL before MRPT headers". Included here (not in the .cpp) so
// every translation unit that includes this header gets it consistently.
#include <pcl/io/pcd_io.h>
// clang-format on

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>

namespace mola
{
/** See the .cpp file for the full documentation of the conversion logic.
 * Unlike Fast-LIO2's adapter, this one does NOT apply a minimum-range cut:
 * EllipseLIO's own LidarProcess performs range filtering internally
 * (`Config::lidar_min_range`/`lidar_max_range`), exactly as upstream does. */
ellipse_lio_core::LidarPointVector toEllipseLioCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end);

ellipse_lio_core::ImuSample toEllipseLioImu(const mrpt::obs::CObservationIMU & obs);

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const ellipse_lio_core::XYZIPointVector & points);

}  // namespace mola
