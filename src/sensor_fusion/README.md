# Sensor fusion

Radar-led camera-radar fusion. The algorithm modules are plain C++ (no ROS) and are built by any
platform: CMake/colcon on ROS 2 (PC, Orin), concerto on TDA4.

## Layout

```
common/              fusion_common       camera model + calibration, Hungarian, class table, types, ego motion
fusion/              sensor_fusion_core  radar tracks + 2D box tracks -> fused objects, ego path, stop zone
radar_tracker/
  ekf/               ekf_tracker         EKF tracker of the AEB pipeline (aeb/ = byte-identical copy, SOURCE.md)
  imm/               imm_tracker         clustering + IMM tracker (with the EKF gating, mirror and ground checks)
camera/
  detector/          object_detection    YOLOX 18 classes / YOLOv8 + lane / road segmentation (TensorRT on Orin,
                                         ONNX Runtime / replay of a dump on PC), ROS node
  tracker/           byte_tracker        ByteTrack 2D box tracker
  lane/              lane_estimation     segmentation mask -> road edges / lane markings / driving path
tests/               sensor_fusion_tests unit tests of the modules above (no ROS)
adapters/ros2/       ROS 2 nodes, messages, launch, RViz (sensor_fusion, radar_tracker, perception_msgs,
                     fusion_visualization, fusion_bringup)
vehicles/            fusion_vehicles     common/: sensor_fusion.yaml, radar_tracker.yaml (all vehicles)
                                         <vehicle>/: calibration (camera intrinsics, radar-camera extrinsics)
```

The fusion does no radar or camera tracking itself. It takes two inputs:

- **radar tracks** (`ExternalRadarTrack`, `FusionCore::processRadarTracks`) from the EKF tracker, the IMM
  tracker or a platform's own tracker;
- **2D box tracks** (`CameraFrame` with track ids, `FusionCore::processCamera`) from ByteTrack
  (`trackBoxes`) or a platform's own camera tracker.

```
radar targets ──> EKF tracker | IMM tracker | platform tracker ──> radar tracks ──┐
                                                                                  ├──> FusionCore ──> objects
camera image ──> detector ──> ByteTrack | platform tracker ──> 2D box tracks ─────┘
camera image ──> lane segmentation ──> mask ──> LaneEstimator ──> road edges / lane markings / path
```

The lane boundaries are published with the objects (`FusedObjectArray`: `lane_*` / `road_*` in the
output frame, `*_base` in the vehicle frame). Flat-ground projection: reliable to ~25 m.

Road edges follow the road through curves (the row search starts on the vehicle centre line and
follows the middle of the road upwards; each edge is fitted along its own direction). Boxes of people
and two-wheelers from the detector (`lane.occluder_classes`) are never a road edge.

From the edges a tracked driving path for the planner (`PathTracker`, `lane.path.*`): 21 points, 1 m
apart from `base_link`, kept on the ground between masks (ego motion) and moved gently towards the
road edges (gain, step limit, gate for one-off jumps). The path keeps the vehicle's distance to the
nearer edge (it is not pulled to the middle of a wide road), stays half a lane inside both edges, and
its curvature stays within the steering (wheelbase 1.6 m, 20 deg: radius >= 4.4 m). Lane = path +-
(vehicle width / 2 + 0.5 m). Without edges the points stay where they are; after 15 m it restarts
from the steering.

Published as `FusedObjectArray.lane_path`, `lane_path_left` / `lane_path_right` (+ `*_base`,
`lane_path_unmeasured`) and as `nav_msgs/Path` `/sensor_fusion/lane_path` in the base frame.

To look at the lane chain of a bag on the camera image (optional, command line):

```bash
./view_lane.sh ~/data/calibration_ros_bag/bag_record_3_intersept_v1 --start 55 --duration 15
./view_lane.sh <bag> --lane-model models/road_bisenetv2_ade20k.onnx    # open-source road-only model
```

## Stages per platform

A stage that the platform provides itself is not compiled.

| Stage          | ROS 2 (PC / Orin)                          | TDA4                                   |
|----------------|--------------------------------------------|----------------------------------------|
| detector       | `object_detection`                         | the app's TIDL detector                |
| camera tracker | ByteTrack (`SF_WITH_BYTETRACK`)            | the app's HOG tracker                  |
| radar tracker  | EKF node + IMM (`SF_WITH_IMM`), runtime `radar.tracker` | the app's EKF, or the IMM     |
| lane           | segmentation + `lane_estimation` (`SF_WITH_LANE`) | the app's mask, if any (`lane_estimation`) |
| fusion         | `sensor_fusion_core`                       | `sensor_fusion_core`                   |

## ROS 2

```bash
./build.sh --orin --packages-up-to fusion_bringup              # or --pc
ros2 launch fusion_bringup fusion_pipeline.launch.xml radar_tracker:=ekf|imm vehicle:=buggy
colcon test --packages-select sensor_fusion_tests               # unit tests
```

Compile-time stages of the `sensor_fusion` node (all ON by default):

```bash
colcon build --packages-select sensor_fusion --cmake-args -DSF_WITH_IMM=OFF -DSF_WITH_BYTETRACK=OFF -DSF_WITH_LANE=OFF
```

- `SF_WITH_IMM=OFF`: no IMM tracker in the node; `radar.tracker imm` stops with an error.
- `SF_WITH_BYTETRACK=OFF`: the camera objects must already carry track ids.
- `SF_WITH_LANE=OFF`: no lane / road boundaries.
- No EKF tracker: do not build `radar_tracker` / `ekf_tracker` (`--packages-skip`).

## A new vehicle

Copy `vehicles/buggy/` to `vehicles/<name>/` with that vehicle's calibration, then launch with
`vehicle:=<name>`. Tuning shared by all vehicles stays in `vehicles/common/`; the radar mount and
height are launch arguments (`radar_height`, `radar_mount_x`, ...).
