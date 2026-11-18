# mola_ellipse_lio_wrapper

A [MOLA](https://github.com/MOLAorg/mola) wrapper around
[EllipseLIO](https://github.com/v4rl-ucy/ellipselio) that mimics the interface of
[`mola-lidar-odometry-cli`](https://docs.mola-slam.org/latest/mola_lo_apps.html#mola-lidar-odometry-cli),
so EllipseLIO can be run on any MOLA-supported dataset for comparison against other
LiDAR-odometry methods.

**Status:** research wrapper, work in progress. Read the known issue below before relying
on its results.

## What this provides

1. **`mola::EllipseLioOdometry`** — an online module loadable from a `mola-cli` YAML
   pipeline (`type: mola::EllipseLioOdometry`), publishing live localization and map
   updates so the existing MOLA GUI (`mola_viz`/`mola_viz_imgui`) draws trajectory and
   the growing map in real time, plus a status sub-window with EllipseLIO's own
   per-scan instrumentation (registration iterations, mean residual, primitive-type
   counts).
2. **`mola-ellipse-lio-cli`** — an offline command-line tool that processes a dataset
   scan-by-scan, waiting for each scan to finish before feeding the next, so no scan
   is ever dropped.

Both are built on top of `ellipse_lio_core`, a small library containing the EllipseLIO
algorithm with all ROS dependencies stripped out (plain C++ structs in/out, no camera
support), so the same core logic backs both the live module and the offline CLI.

## Datasets

- **Oxford Spires**: via the generic `mola::Rosbag2Dataset` (package `mola_input_rosbag2`),
  same approach as `mola_lidar_odometry`'s own Oxford Spires launch files.
- MulRan is not wired up for this wrapper yet.
- **LiDAR-only rawlogs** (e.g. MVSim synthetic datasets): `--input-rawlog`
  plus `--synthetic-imu`, which interleaves a constant-gravity, zero-rate IMU
  sample before each scan (the same decorator the other LIO wrappers use).
  See `pipelines/ellipselio-mvsim.yaml`:

  ```bash
  mola-ellipse-lio-cli -c pipelines/ellipselio-mvsim.yaml --synthetic-imu \
    --input-rawlog dataset.rawlog --output-tum-path out.tum
  ```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select mola_ellipse_lio_wrapper --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## Usage

```bash
# Online, live GUI:
scripts/mola-ellipselio-gui-oxford-spires /path/to/oxford-spires/data/sequences/<seq>/
# Headless (no GUI):
scripts/mola-ellipselio-gui-oxford-spires /path/to/oxford-spires/data/sequences/<seq>/ --headless

# Offline, lossless, with TUM trajectory output:
SEQ=/path/to/oxford-spires/2024-03-13-observatory-quarter-01
BAG=$SEQ/raw/ros2bag/1710338090_2024-03-13-13-54-51
LIDAR_POSE_X=0 LIDAR_POSE_Y=0 LIDAR_POSE_Z=0 LIDAR_POSE_YAW=0 LIDAR_POSE_PITCH=0 LIDAR_POSE_ROLL=0 \
MOLA_USE_FIXED_LIDAR_POSE=true \
IMU_POSE_X=0.018771 IMU_POSE_Y=-0.008218 IMU_POSE_Z=-0.070474 \
IMU_POSE_YAW=-90.6263 IMU_POSE_PITCH=-0.1665 IMU_POSE_ROLL=-0.1287 \
MOLA_USE_FIXED_IMU_POSE=true \
mola-ellipse-lio-cli -c pipelines/ellipselio-oxford-spires.yaml \
  --input-rosbag2 "$BAG" \
  --lidar-topic /hesai/pandar --imu-topic /alphasense_driver_ros/imu \
  --output-tum-path ellipselio_oq01.tum
```

See `pipelines/ellipselio-oxford-spires.yaml` for the EllipseLIO algorithm parameters
(LiDAR-IMU extrinsics, IMU noise characterization, map/octree resolution, range gates),
ported from upstream's own `config/qt64_spires.yaml` -- see the comments in that file
for the two deliberate deviations (minimum range, a few extra env-var knobs) and the
extrinsic-convention cross-check.

## Parameters (`pipelines/*.yaml`)

Mirrors upstream's `mapping.*`/`lidar.*`/`imu.*` YAML schema (dots replaced by
underscores), plus the module-level `imu_pose_in_lidar` (MOLA `x y z yaw_deg pitch_deg
roll_deg` convention), `baselink2lidar_pose_str` (pose of the LiDAR wrt `base_link`, same
key and convention as the DLIO, Fast-LIO2 and GLIM wrappers) and
`lidar_sensor_label`/`imu_sensor_label` regexes. The reported pose is `base_link`'s, or
the LiDAR's when `baselink2lidar_pose_str` is left at identity. See
`third_party/ellipse_lio_core/include/ellipse_lio_core/Types.hpp`'s `Config` struct for
the authoritative field list.

## Determinism

Upstream ends the iterated update of each scan once its wall-clock time exceeds a
quarter of the scan period (25 ms at 10 Hz), which keeps the filter real-time on a
multi-core CPU. The number of iterations, and so the result, then depends on the CPU
speed and on its load: two runs of the same input differ, and on a single thread the
limit cuts the update short on most scans of geometrically demanding sequences.

`iekf_wall_clock_limit: false` (env `ELLIPSELIO_IEKF_WALL_CLOCK_LIMIT=false` in the
shipped pipelines) lifts it. The update is then bounded by its iteration count only,
and two runs of the same input produce byte-identical output. This changes the results
(more iterations per scan), and runs no longer keep up in real time on a slow CPU, so
the default stays `true`, upstream's behavior. Use `false` for offline benchmarks and
for perturbation or sensitivity studies.

## Instrumentation

`ellipse_lio_core::OdometryOutput::analytics` mirrors upstream's own
`EllipseLioAnalytics` message field for field (residual mean, adaptive-weight
statistics, primitive-type counts, map/octree size, per-stage timings): the single
highest-value instrumentation payload of the MOLA LIO wrappers, since it is EllipseLIO's
own internal research telemetry, unmodified.

Two extra, env-gated, zero-cost-when-off debug dumps (`third_party/ellipse_lio_core/src/EllipseLioCore.cpp`):

- `MOLA_ELLIPSELIO_DUMP_ESTIMATED_GRAV=1`: appends `<time> <gx> <gy> <gz>` to
  `grav.txt` after every scan (same format as `mola_fast_lio2_wrapper`'s own dump, so
  the two can be diffed directly).
- `MOLA_ELLIPSELIO_DEBUG_DUMP=1`: appends a fuller per-scan state line (position,
  velocity, biases, gravity, feature/rejection counts, residual mean, observability
  scores, map size) to `debug_dump.txt`. Added while investigating the divergence
  documented below; kept as a standing diagnostic aid for future investigation.

## Known issue: divergence on Oxford Spires (both sequences tested)

**This port diverges early** (within roughly the first 15-30 s) on both
`2024-03-13-observatory-quarter-01` and `2024-03-14-blenheim-palace-01`: the
accelerometer bias and gravity-vector estimates run away during the iterated-EKF
update, well before any aggressive motion occurs. Diagnostic steps taken so far: the
extrinsic was cross-checked numerically, an adaptive-weighting ablation test was
performed, both sequences were tried, and upstream's own `min_range: 0.0` was tried
against our `1.0` default. No root cause has been identified yet; this is recorded as a
known-broken item, not silently worked around.

Note that until the IMU extrinsic was read from the documented
`imu_pose_in_lidar` key, the module silently used an identity extrinsic, while
this dataset's IMU is rotated by about -90 deg in yaw wrt the LiDAR. Both
sequences remain to be re-checked with that fix.

## Notes

- `ellipse_lio_core` (`third_party/ellipse_lio_core/`) is EllipseLIO's algorithm with
  every ROS dependency stripped out (plain C++ structs in/out, no MOLA dependency
  either, no camera colorization support), and is unit-tested standalone
  (`test_ellipse_lio_core`, a synthetic stationary-room scan sequence). It builds with
  `-DPCL_NO_PRECOMPILE` for the same ABI-safety reason `fast_lio2_core` does -- see that
  package's README for the full explanation.
- EllipseLIO and its i-Octree/IKFoM dependencies are MIT and BSD-3, both compatible
  with this repository's GPL-3.0. See `THIRD_PARTY_NOTICES.md`.
