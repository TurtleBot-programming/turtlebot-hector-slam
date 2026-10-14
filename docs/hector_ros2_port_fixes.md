# Fixes to the community ROS 2 port of Hector SLAM

This project runs Hector SLAM from a community ROS 2 Humble port, not from the
official ROS 1 package. Getting a usable map out of it on the TurtleBot 4
needed two bug fixes in that port's `hector_mapping` node. This document
records what was wrong, how it was found, what was changed, and how the fix
was verified. Both fixes have been offered upstream.

| | |
| --- | --- |
| Affected package | `hector_mapping`, file `hector_mapping/src/HectorMappingRos.cpp` |
| Port | [KiiiLin/hector_slam_humble](https://github.com/KiiiLin/hector_slam_humble), branch `humble`, commit `38ad94a` ("migrate to ROS2 Humble") |
| Upstream pull request | [KiiiLin/hector_slam_humble#1](https://github.com/KiiiLin/hector_slam_humble/pull/1) (branch `fix/scan-qos-and-laser-transform` on our fork) |
| Our fork | [TurtleBot-programming/hector_slam_humble](https://github.com/TurtleBot-programming/hector_slam_humble), branch `humble`, commit `2ede869` (both fixes in one commit) |

## 1. Where the code comes from

```
tu-darmstadt-ros-pkg/hector_slam       official ROS 1 package (catkin, noetic-devel)
        │  ROS 2 migration, 2025 (single commit 38ad94a)
        ▼
KiiiLin/hector_slam_humble             community ROS 2 Humble port   ← bugs introduced here
        │  fork
        ▼
TurtleBot-programming/hector_slam_humble   our fork, used by this project
```

The ROS 1 package does not have either bug. Both come from the migration to
ROS 2, where one ROS 1 feature (`laser_geometry`) was stubbed out and the QoS
default was carried over unchanged.

## 2. Bug 1: scan subscription QoS

### Symptom

`hector_mapping` started without errors but `/map` stayed empty and Hector
never published a pose. `/scan` was being published at ~10 Hz.

### Cause

The port subscribed to the scan with a plain queue depth
(`HectorMappingRos.cpp:236` at `38ad94a`):

```cpp
scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    p_scan_topic_, p_scan_subscriber_queue_size_, ...);
```

A depth-only QoS in ROS 2 means **reliable** delivery. The TurtleBot 4's
RPLIDAR, and our Webots bridge which mimics it, publish `/scan` with
sensor-data QoS, which is **best-effort**. DDS does not connect a reliable
subscriber to a best-effort publisher (the subscriber asks for a guarantee the
publisher does not offer), so not a single scan reached Hector. ROS 2 doesn't
raise an error for this; it only shows up in `ros2 topic info /scan -v`, where
the two endpoints have different reliability.

### Fix

Subscribe with sensor-data QoS, keeping the configured queue size:

```cpp
scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_,
    rclcpp::SensorDataQoS().keep_last(p_scan_subscriber_queue_size_), ...);
```

A best-effort subscriber matches **both** best-effort and reliable publishers,
so the change cannot break a setup that already worked.

## 3. Bug 2: the laser mounting transform was ignored

This was the serious one: Hector ran and produced a map, but the map and the
pose estimate were wrong.

### Symptom

With the QoS fix in place, Hector built a map and Nav2 could drive on it,
but compared with the simulator's ground truth the pose was off by
**2.5 m on average and up to 5.1 m** after three minutes. The error had a
specific shape:

- the **heading** estimate was correct (mean error 1.7°, the same as after the fix);
- the **position** estimate moved at 90° to the direction the robot was
  facing: the logged positions satisfied `est_x ≈ gt_y`, `est_y ≈ −gt_x`;
- the estimated trajectory left the building, which is physically
  impossible.

### Diagnosis

1. **Rule out the inputs.** The bridge's scan was checked against the
   ground-truth map: projecting each scan point through the true robot pose
   and the lidar mounting (x = −0.04 m, yaw = +90°) put 95 % of the points
   on a real wall. So the scan and the TF tree were correct.
2. **Check the estimate for internal consistency.** Between two log samples
   the true robot moved along its heading (direction −50°, heading −50°),
   but Hector's estimate moved in direction −142° while reporting the same
   −50° heading: its translation was 90° away from its own heading. A
   pose estimate whose heading and direction of motion disagree by a
   constant angle means the scan points are being interpreted in a frame
   rotated by that angle.
3. **Read the scan callback.** That angle is exactly the lidar's mounting
   yaw, which pointed to the base → laser transform. In `scanCallback`
   (lines 312–340 at `38ad94a`):

```cpp
// If we are using the tf tree to find the transform between the base frame and laser frame,
// let's get that transform
laser_transform = tf_buffer_->lookupTransform(p_base_frame_, scan->header.frame_id, ...);  // line 317

// Convert the laser scan to point cloud
// TODO: Check laser_geometry ROS2 API
// projector_.projectLaser(*scan, laser_point_cloud_, 30.0);
// For now, skip point cloud conversion and use direct scan conversion
this->rosLaserScanToDataContainer(*scan, laserScanContainer, slamProcessor->getScaleToMap());  // line 330

// Convert the point cloud to our data container (using direct scan conversion for now)
// this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, ...);  // line 339
```

The transform is looked up and then **never used**. The ROS 1 code projected
the scan with `laser_geometry` and passed it through
`rosPointCloudToDataContainer()`, which moves every point into `base_frame`.
The port commented both steps out (`laser_geometry` was not ported) and
called `rosLaserScanToDataContainer()` instead, which keeps the points in the
laser frame and sets the scan origin to zero. `use_tf_scan_transformation`
defaults to `true` (line 83), so this affects every user who did not change it.

### Why it produces exactly this error

Let *M* be the laser's mounting transform in the base frame (here a rotation
by φ = +90° plus a 4 cm offset), and *B*ₖ the true base pose at scan *k*.
The scan points are expressed in the laser frame, whose true pose is
*L*ₖ = *B*ₖ · *M*. Because the points were treated as if they were already in
`base_frame`, the scan matcher actually tracks the **laser** frame and
reports it as the base. Hector anchors its map at the first scan, so it
estimates

  *E*ₖ = *L*₀⁻¹ · *L*ₖ = *M*⁻¹ · (*B*₀⁻¹ *B*ₖ) · *M*.

In 2D, rotations commute, so the rotation part of *E*ₖ equals the true
relative heading: **the heading is correct**. The translation part is the
true relative translation rotated by −φ: **positions are rotated by −90°**,
giving `est_x = gt_y`, `est_y = −gt_x`, exactly as logged. The map is built
in that rotated frame too, so the map and the robot's motion through it
disagree, and the pose drifts sideways whenever the robot drives.

A laser mounted with zero yaw at the base origin hides the bug completely,
which is presumably why it survived the migration. Any yaw makes it large,
and any offset makes it smaller but still real.

### What else was skipped

`rosLaserScanToDataContainer()` filters only by the scan message's own
`range_min`/`range_max`. Hector's own range and height filters,
`laser_min_dist`, `laser_max_dist`, `laser_z_min_value` and
`laser_z_max_value`, live in `rosPointCloudToDataContainer()` (line 601), so
on the default code path they were silently ignored as well.

### Fix

Do what `laser_geometry::LaserProjection::projectLaser()` did in ROS 1:
project the scan into a point cloud in the laser frame, then pass it to the
existing `rosPointCloudToDataContainer()`, which applies the transform, sets
the scan origin to the laser position and applies the filters.

```cpp
// Project the scan into a point cloud in the laser frame (what
// laser_geometry's projectLaser() did in the ROS 1 version)
laser_point_cloud_.header = scan->header;
laser_point_cloud_.points.clear();
laser_point_cloud_.points.reserve(scan->ranges.size());
float angle = scan->angle_min;
for (float range : scan->ranges)
{
  if (std::isfinite(range) && range > scan->range_min && range < scan->range_max)
  {
    geometry_msgs::msg::Point32 p;
    p.x = std::cos(angle) * range;
    p.y = std::sin(angle) * range;
    laser_point_cloud_.points.push_back(p);
  }
  angle += scan->angle_increment;
}

// Convert the point cloud to our data container, transforming it into the base frame
this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, slamProcessor->getScaleToMap());
```

It reuses the member `laser_point_cloud_` and the conversion function that
the port already had (both unused until now), so no new dependency on
`laser_geometry` is needed. The `use_tf_scan_transformation = false` path is
unchanged.

## 4. Verification

All runs: ROS 2 Humble, Webots R2025a, simulated TurtleBot 4 (RPLIDAR
yawed 90°, best-effort `/scan`), Hector + Nav2 + our frontier explorer,
autonomous exploration of the 10 m × 8 m test world from a blank map.
Position error is Hector's `map → base_link` against the simulator's
ground-truth pose, logged once per second (`webots/tools/pose_logger.py`).
Map quality is the saved map against the ground-truth map
(`webots/tools/evaluate_map.py`).

| | port as released | + QoS fix | + QoS and transform fix |
| --- | --- | --- | --- |
| scans reaching Hector | none | yes | yes |
| `/map` | empty | built, but skewed | correct |
| position error, mean | n/a | 2.48 m | **0.028 m** |
| position error, max | n/a | 5.11 m | **0.075 m** |
| heading error, mean | n/a | 1.7° (unaffected, see §3) | **1.8°** |
| map walls within 10 cm of a true wall | n/a | not measured (map unusable) | **99.2 %** |
| true walls within 10 cm of a map wall | n/a | not measured | **98.9 %** |

The figure `docs/images/map_vs_ground_truth.png` shows the final map and
trajectory over the ground truth. The upstream branch was also built on its
own from the port's `humble` branch
(`colcon build --packages-up-to hector_mapping`): it builds with no warnings.

## 5. Status

| Where | State |
| --- | --- |
| Our fork, `TurtleBot-programming/hector_slam_humble` branch `humble` | both fixes, commit `2ede869`; this project builds Hector from here (see the top-level README, Setup step 1) |
| Upstream, `KiiiLin/hector_slam_humble` | [pull request #1](https://github.com/KiiiLin/hector_slam_humble/pull/1), two commits (`f603979` QoS, `9f78f0a` transform) on branch `fix/scan-qos-and-laser-transform` of our fork; the port has no contribution guide and issues are disabled, so a PR was the only channel |

If the PR is merged, the fork can switch back to tracking upstream.

## 6. Notes and possible follow-ups (not changed)

- **Points behind the laser are dropped.** `rosPointCloudToDataContainer()`
  discards points with `x < 0` and range < 0.71 m *in the laser frame*
  (inherited unchanged from ROS 1, meant to remove the robot's own body). On
  the TurtleBot 4 the laser is yawed 90°, so this removes close points on the
  robot's **right-hand side**, not behind it. It has not caused problems in
  our worlds, but it costs some close-range data in narrow corridors.
- **The point-cloud debug publisher is still disabled.** The port's
  `slam_cloud` publisher is declared as `PointCloud2` while the internal
  cloud is a `sensor_msgs/PointCloud`. The PR leaves that block as it was.
- **Other ROS 1 features may have the same pattern.** This bug was a
  ROS 1 dependency stubbed with "for now" during migration. Other
  `TODO`/"disabled for now" comments in the port are worth a check before
  relying on them.

## 7. Not part of the port

Other problems found while bringing up the simulation were in **this**
repository, not in Hector, and were fixed here (see git history and the
README's troubleshooting table):

| Problem | Where | Fix |
| --- | --- | --- |
| Mirrored scan (robot wobbled in place) | Webots bridge | reverse the ray order: Webots orders rays clockwise, `LaserScan` is counter-clockwise |
| `Transform data too old` on every control cycle | `config/nav2.yaml` | `transform_tolerance` 0.2 → 0.5 s; Hector publishes `map → odom` once per scan |
| Explorer blacklisting good goals | `frontier_explorer.cpp` | don't blacklist goals rejected while Nav2 is starting; ignore late results of abandoned goals |
| RViz freezing, slow planner | `config/hector_slam.yaml` | `map_size` 2048 → 512 (102 m → 25.6 m grid) |
