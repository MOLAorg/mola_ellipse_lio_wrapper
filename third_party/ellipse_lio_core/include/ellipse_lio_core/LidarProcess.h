// SPDX-License-Identifier: MIT
//
// Derived from EllipseLIO's include/lidar_processing.h
// (https://github.com/v4rl-ucy/ellipselio), MIT License, Copyright (c) 2026
// Rowan Border and Margarita Chli. Ported: the ROS `PointCloud2` subscription
// and the vendor-specific point-decoding templates (`PointCloudHandler<T>`,
// `ConvertPoint<T>`, the per-vendor `SetPoint()` overloads for Livox,
// Velodyne, Ouster and Gazebo) were removed -- this port receives points
// already decoded into `LidarPointVector` (see Types.hpp and
// mola_ellipse_lio_wrapper's MRPT adapters), so there is no raw
// `PointCloud2` to parse. The Hesai decoding path is preserved in spirit:
// upstream's `SetPoint(HesaiPoint, ...)` used the point's own absolute
// timestamp field, ignoring the message-header-based time entirely, which is
// exactly what `feedScan()`'s `t_beg + t_offset_sec` reconstructs. The
// range-binning and per-bin octree-indexing logic (the adaptive-filtering
// contribution this method is being studied for) is UNCHANGED. See
// THIRD_PARTY_NOTICES.md.
#pragma once

#include <ellipse_lio_core/CommonLib.h>
#include <ioctree/ioctree.h>

#include <atomic>
#include <ellipse_lio_core/Types.hpp>
#include <mutex>
#include <vector>

namespace ellipse_lio_core
{
/// LiDAR sensor parameters (mirrors upstream's `LidarParams`, minus the ROS
/// topic name). `type` is kept for config-schema parity with upstream
/// (LidType: 1=Livox, 2=Velodyne, 3=Ouster, 4=Hesai, 5=Gazebo) even though
/// this port no longer branches on it -- see the file comment above.
struct LidarParams
{
  int type = 4;
  int rate = 10;
  int scan_lines = 64;
  double min_range = 1.0;
  double max_range = 100.0;
  double vertical_fov = 104.0;
};

/// Processes incoming LiDAR scans: range-based binning with per-bin
/// octree spatial indexing (i-Octree) and temporal tracking. No ROS
/// dependency; see EllipseLioCore for the orchestration this class used to
/// get from `MappingNode`.
class LidarProcess
{
public:
  LidarProcess(LidarParams params, float map_resolution, LogFn log = {});
  ~LidarProcess();

  void ClearPointCloud();

  /// Feeds one already-decoded LiDAR scan. `points` are in LiDAR body frame,
  /// `t_offset_sec` relative to `t_beg` (scan start, absolute seconds);
  /// `t_end` is the scan's absolute end time. Performs the same
  /// range-binning / octree-indexing upstream's `Process()` did.
  void feedScan(const LidarPointVector & points, double t_beg, double t_end);

  /// Retrieves the processed point cloud and temporal/spatial metadata
  /// accumulated since the last call; returns false if no new scan data is
  /// available.
  bool GetPointCloud(
    EllipseLioPointCloudPtr pc, double * start_time, double * end_time,
    Eigen::ArrayXi * bin_pcs_sizes, int * start_bin, int * mean_bin);

  int num_bins_ = 0;
  bool lidar_has_data_ = false;

  float scan_res_ = 0;
  float min_scan_res_ = 0;
  float max_search_rad_ = 0;
  float max_octree_res_ = 10;

  std::atomic<int> lidar_counter_{0};
  /// Detected backward time jump between consecutive scans (bag replay
  /// glitch), seconds; kept for the analytics dump only -- see the file
  /// comment on why it no longer corrects per-point times for Hesai.
  double lidar_time_offset_ = 0;
  double lidar_start_time_ = 0;
  double lidar_end_time_ = 0;

  std::vector<int> bucket_sizes_;
  std::vector<int> cnt_neighbours_;
  std::vector<int> min_neighbours_;
  std::vector<int> max_neighbours_;
  std::vector<float> search_radii_;
  std::vector<float> match_radii_;
  std::vector<float> scan_line_sep_;
  std::vector<float> octree_resolutions_;

private:
  void ConvertScan(const LidarPointVector & points, double t_beg);
  void SetMinMaxTime(int bin_idx);
  void ClearBins();

  LogFn log_;

  Eigen::ArrayXi bin_pcs_sizes_;
  Eigen::ArrayXi bin_pcs_i_;

  std::vector<std::vector<int>> bin_idxs_;
  std::vector<std::atomic<int>> bin_sizes_;
  std::vector<iOctree::Octree> bin_octrees_;
  std::vector<EllipseLioPointCloud> bin_pcs_;

  std::vector<double> bin_min_times_;
  std::vector<double> bin_max_times_;

  EllipseLioPointCloudPtr process_pc_;
  EllipseLioPointCloudPtr ellipselio_pc_;

  int start_bin_ = 0;
  int mean_bin_ = 0;
  double last_lidar_time_ = 0;
  std::mutex lidar_mutex_;

  LidarParams params_;
};

}  // namespace ellipse_lio_core
