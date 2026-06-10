# bridgePGO — FastLIVO2 ↔ SCPGO Data Relay

A ROS catkin package that bridges **FastLIVO2** (LiDAR-inertial-visual odometry) output to **SCPGO** (pose graph optimization).

## Overview

FastLIVO2 produces point clouds and odometry poses during SLAM. SCPGO consumes these for global pose graph optimization. `bridgePGO` sits in between — it reads FastLIVO2's output offline and replays them as ROS topics so SCPGO can subscribe and optimize without modifying either system.

```
FastLIVO2 output                  bridgePGO                     SCPGO
┌──────────────┐    ┌──────────────────────────────┐    ┌──────────────┐
│  PCD files   │───▶│  /velodyne_cloud_registered   │───▶│              │
│  + poses.txt │    │           _local               │    │  Pose Graph  │
│              │───▶│  /aft_mapped_to_init           │───▶│ Optimization │
└──────────────┘    └──────────────────────────────┘    └──────────────┘
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | Odometry pose in TUM format (map → base_link) |
| `/velodyne_cloud_registered_local` | `sensor_msgs/PointCloud2` | Registered point cloud from FastLIVO2 |

## Dependencies

- ROS Noetic
- PCL 1.10+
- `pcl_conversions`, `pcl_ros`
- `roscpp`, `sensor_msgs`, `std_msgs`, `nav_msgs`

## Build

```bash
cd catkin_2SCPGO_ws
catkin_make
source devel/setup.bash
```

## Generating FastLIVO2 Output

Enable PCD saving in FastLIVO2's config YAML:

```yaml
pcd_save:
  pcd_save_en: true
  type: 1
```

This produces the `all_pcd_body/` directory containing:

- **`<timestamp>.pcd`** — registered point cloud per frame
- **`lidar_poses.txt`** — TUM-format odometry poses for all frames

Copy the entire `all_pcd_body/` directory into the package before running the bridge.

## Usage

1. Place FastLIVO2 output (`.pcd` files + `lidar_poses.txt`) under:
   ```
   src/bridgePGO/all_pcd_body/
   ```

2. Launch:
   ```bash
   rosrun bridgePGO pubmsg
   ```

The node replays all frames at 10 Hz and keeps running after completion so RViz / SCPGO can inspect the final result.

## Data Format

`lidar_poses.txt` uses TUM format (timestamp tx ty tz qx qy qz qw):

```
1780450907.183845 0.00377 0.00112 -0.06737 0.00252 -0.00809 -0.00074 0.99996
```

Each line corresponds to a `.pcd` file named `<timestamp>.pcd`.
