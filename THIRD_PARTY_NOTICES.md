# Third-party notices

This repository is a [MOLA](https://github.com/MOLAorg/mola) wrapper that integrates
**EllipseLIO**, a LiDAR-inertial odometry algorithm developed by V4RL, University of
Cyprus. The code under `third_party/` is derived from, or vendored verbatim from, the
following upstream projects. None of it is original work of this repository's author
except where explicitly noted.

## EllipseLIO

- Upstream: https://github.com/v4rl-ucy/ellipselio
- Commit pinned: `bbc2a0a29a0fbe703c379ace7ea9a6dc14008c1c` (2026-06-12)
- License: **MIT**.
- Copyright (c) 2026 Rowan Border and Margarita Chli.
- Paper: Border & Chli, *EllipseLIO: Adaptive LiDAR Inertial Odometry with an Ellipsoid
  Representation*, arXiv:2605.21150.
- Files derived from it in this repository:
  - `third_party/ellipse_lio_core/include/ellipse_lio_core/CommonLib.h` and
    `CommonPcl.h` (ported from `include/common_lib.h`/`common_pcl.h`): `rclcpp::Time`
    fields replaced by `double` seconds; ROS-parameter-array conversion helpers
    (`Vec3dFromArray`/`QuaterniondFromArray`/`Mat3dFromArray`) dropped (this port
    receives already-parsed Eigen types); the vendor-specific PCL point types
    (`LivoxPoint`, `VelodynePoint`, `OusterPoint`, `HesaiPoint`, `GazeboPoint`) and the
    `pcl_conversions` include were removed, since this port never parses a raw
    `sensor_msgs::PointCloud2` -- see `ImuProcess.h`/`LidarProcess.h` below.
    `PointXYZNRGBIT` (the internal working point type) is otherwise unchanged.
  - `third_party/ellipse_lio_core/include/ellipse_lio_core/ImuProcess.h` and
    `src/ImuProcess.cpp` (ported from `include/imu_processing.h` /
    `src/imu_processing.cpp`): `sensor_msgs::msg::Imu` -> plain `ImuSample`;
    `rclcpp::Time` -> `double`; the ROS subscription/callback-group machinery replaced
    by a public `feedImu()` entry point; `RCLCPP_*` logging replaced by an optional
    `LogFn` sink; the camera-colorization path (`GetMatchingImages`/`ColorisePoint`,
    the `cams` parameter of `UndistortPointCloud`) removed together with camera
    support (see below). Also dropped: `SetGyrCov`, `SetAccCov`, `SetGyrBiasCov`,
    `SetAccBiasCov`, `SetExtrinsic` -- declared in the upstream header but never
    defined or called anywhere in the upstream repository (confirmed by inspection;
    dead API surface, not algorithm logic). Gravity/bias initialization and IMU
    propagation/undistortion/iterated-update logic is otherwise **unchanged**.
  - `third_party/ellipse_lio_core/include/ellipse_lio_core/LidarProcess.h` and
    `src/LidarProcess.cpp` (ported from `include/lidar_processing.h` /
    `src/lidar_processing.cpp`): the ROS `PointCloud2` subscription and the
    vendor-specific point-decoding templates (`PointCloudHandler<T>`,
    `ConvertPoint<T>`, the per-vendor `SetPoint()` overloads for Livox, Velodyne,
    Ouster and Gazebo) removed -- this port receives points already decoded into
    `LidarPointVector` (see `Types.hpp` and the MOLA module's adapters), so there is
    no raw `PointCloud2` to parse. The Hesai decoding path is preserved in spirit:
    upstream's `SetPoint(HesaiPoint, ...)` used the point's own absolute timestamp
    field, ignoring the message-header-based time entirely, which is exactly what
    `feedScan()`'s `t_beg + t_offset_sec` reconstructs. **The range-binning and
    per-bin octree-indexing logic (the adaptive-filtering contribution this method is
    being studied for) is unchanged.**
  - `third_party/ellipse_lio_core/include/ellipse_lio_core/EllipseLioCore.h` and
    `src/EllipseLioCore.cpp` (ported from `include/map_processing.h` /
    `src/map_processing.cpp`): `MappingNode` (an `rclcpp::Node`) replaced by a
    stateful, push-based class. The four ROS timers collapse into
    `addImu()`/`addLidarScan()` triggering the same processing steps
    `TimerCallback()` ran (`SyncPackages()`, IMU propagation/undistortion, the
    iterated-EKF tensor-voting update, `MapIncremental()`); the five ROS publishers
    and the `tf2_ros` broadcaster are replaced by the returned `OdometryOutput` and
    the `currentMap()` accessor. `SplitMap()` (PointCloud2 message chunking, needed
    only for ROS message size limits) and the whole camera pipeline are dropped.
    `imu_process_`/`lid_process_` are built directly in the constructor instead of
    lazily on the first timer tick (upstream deferred this only because it needed
    `shared_from_this()`, unavailable inside an `rclcpp::Node` constructor -- not
    needed here). Also dropped as dead code (declared, assigned, never read anywhere
    in upstream): `n_res_`, `map_search_rad_`, `vel_pose_counter_`, `ekfom_upd_cnt_`.
    **The tensor-voting map construction (`ComputeTensorVote`, `ComputeTensorEigen`,
    `TensorVotePass1`/`TensorVotePass2`) and the point-to-ellipsoid measurement model
    (`TensorRegistration`, ported as `hShareModel`) are transcribed as closely to
    upstream as the ROS-type removal allows** -- this is the core algorithmic
    contribution under study and no intentional logic change was made.
- Cameras are dropped structurally (not just by config): `cam_processing.{h,cpp}` was
  not vendored at all, and every reference to `CamProcessVec`/`InitCamProcess()`/the
  `cameras.*` parameters/the colorization branch of `SyncRawCloudWithImu()`'s ancestor
  was removed. This drops the `cv_bridge`, `image_transport` and OpenCV dependencies.

## i-Octree

- Upstream: bundled inside the EllipseLIO repository at `include/ioctree/ioctree.h`.
  The file's own header attributes it to Jun Zhu, Tsinghua University (2023); the
  upstream i-Octree project is https://github.com/zhujun3753/i-octree.
- License: **MIT** (per the license text embedded at the top of `ioctree.h`; no
  separate `LICENSE` file is bundled with this single-header copy).
- Copyright (c) 2023 Jun Zhu, Tsinghua University.
- Vendored at `third_party/ioctree/ioctree.h`, **unmodified**.

## IKFoM (Iterated Kalman Filter on Manifolds) toolkit

- Bundled inside the EllipseLIO repository at `include/ikfom/` (itself vendored from
  the same IKFoM lineage `mola_fast_lio2_wrapper` uses, but as its own independent copy
  under EllipseLIO's repository, with EllipseLIO-specific `so3_math.h` naming).
- License: **BSD 3-Clause** (`esekfom/`, `mtk/`), same terms as
  `mola_fast_lio2_wrapper`'s `IKFoM_toolkit`.
- Copyright (c) 2019-2023, The University of Hong Kong. Author: Dongjiao HE
  (IKFoM is itself a fork of the MTK manifold toolkit; some files retain an older
  University of Bremen attribution from that lineage).
- Vendored at `third_party/ikfom/{esekfom,mtk}`, **unmodified**.
- `third_party/ikfom/so3_math.h` and `use_ikfom.h` (EllipseLIO's own small header
  defining the state vector on top of IKFoM, distinct from the toolkit itself, itself
  ultimately derived from Fast-LIO2's `use-ikfom.hpp`) are vendored **unmodified** --
  unlike `mola_fast_lio2_wrapper`'s copy, no `inline` fix was needed here since
  EllipseLIO's version already declares its free functions `inline`.
- `esekfom.hpp`'s measurement-model callback (`init_dyn_share`'s `h_share_model`
  parameter) is a `std::function`, not a plain function pointer as in the vanilla
  Fast-LIO2 `IKFoM_toolkit` -- EllipseLIO's fork of the toolkit already supports
  binding a member function directly (`EllipseLioCore::hShareModel` via a capturing
  lambda), so no trampoline indirection is needed here (unlike
  `mola_fast_lio2_wrapper`'s `g_active_core` thread-local pointer, which exists to work
  around exactly that plain-function-pointer limitation in vanilla IKFoM).

## Licensing note: no GPLv2/GPLv3 incompatibility here

Unlike `mola_fast_lio2_wrapper` (which wraps GPL-2.0-only Fast-LIO2/ikd-Tree code
inside a GPL-3.0-only MOLA module -- a real license incompatibility, documented in that
repository's own `THIRD_PARTY_NOTICES.md`), **this repository has no such issue**:
EllipseLIO is MIT-licensed, i-Octree is MIT-licensed, and IKFoM is BSD-3-Clause. MIT and
BSD-3 are both permissive licenses fully compatible with combination into a GPL-3.0-only
work (this repository's own license). A future reader should not assume the fastlio2
wrapper's GPLv2-vs-GPLv3 caveat applies here -- it does not.

This repository is currently **private and used for internal research only**.

## Known issue

See `README.md`'s "Known issue: divergence on Oxford Spires" section: this port
diverges early on both Oxford Spires validation sequences tried. The algorithm logic
was transcribed as faithfully as the ROS-type removal allows (see the file-by-file
notes above), and a divergence-hunting session did not conclusively identify a root
cause (see the implementation plan's Open items). This is recorded here because it
bears on how much weight to put on this port when comparing EllipseLIO's true published
behavior against this wrapper's output.
