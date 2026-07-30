// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's include/common_pcl.h
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. Ported: the vendor-specific PCL point
// types (LivoxPoint, VelodynePoint, OusterPoint, HesaiPoint, GazeboPoint) and
// the `pcl_conversions` include were removed, since this port never parses a
// raw `sensor_msgs::PointCloud2` -- it receives already-decoded points (see
// ellipse_lio_core/Types.hpp). `PointXYZNRGBIT` (the internal working point
// type) is otherwise unchanged. See THIRD_PARTY_NOTICES.md.
//
// PCL_NO_PRECOMPILE: see the comment in ellipse_lio_core/CMakeLists.txt and
// fast_lio2_core's own CommonPcl-equivalent note; ABI-safety measure, not an
// algorithmic change.
#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/impl/point_types.hpp>

/// EllipseLio unified point type: position, RGBA color, intensity,
/// timestamp, range-bin index, scan index, and primitive type (the
/// tensor-voting ellipsoid classification: 0=point, 1=line, 2=plane,
/// 3=ellipsoid). Aligned to 16 bytes for SIMD.
struct EIGEN_ALIGN16 PointXYZNRGBIT
{
  PCL_ADD_POINT4D;
  union {
    struct
    {
      uint32_t bin_idx;
      uint32_t scan_idx;
      uint32_t has_rgb;
      uint32_t prim_type;
    };
    float data_n[4];
  };
  union {
    struct
    {
      PCL_ADD_UNION_RGB
      float intensity;
      /// Per-point absolute timestamp, seconds/nanoseconds split (matches
      /// upstream's `time_secs`/`time_nsecs`, previously fed from
      /// `rclcpp::Time`; now fed directly as a split double, see
      /// ImuProcess.cpp/LidarProcess.cpp for the conversion helpers).
      uint32_t time_secs;
      uint32_t time_nsecs;
    };
    float data_c[4];
  };
  PCL_ADD_EIGEN_MAPS_RGB
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZNRGBIT,
  (float, x, x)(float, y, y)(float, z, z)(uint32_t, bin_idx, bin_idx)(uint32_t, scan_idx, scan_idx)(
    uint32_t, has_rgb, has_rgb)(uint32_t, prim_type, prim_type)(float, rgb, rgb)(
    float, intensity, intensity)(uint32_t, time_secs, time_secs)(uint32_t, time_nsecs, time_nsecs))

using EllipseLioPoint = PointXYZNRGBIT;
using EllipseLioPointCloud = pcl::PointCloud<EllipseLioPoint>;
using EllipseLioPointCloudPtr = pcl::PointCloud<EllipseLioPoint>::Ptr;
