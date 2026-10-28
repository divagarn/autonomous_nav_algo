# buggy_unified

One self-contained folder for the buggy's **PCD + Lanelet2 stack**: map loading, NDT/EKF
localisation on the live VLP-16, route planning, LiDAR obstacle detection, AUTO driving over UART,
brake calibration, map import, optional radar + camera object names, and a Gazebo closed-loop simulator.

Everything is a **copy**. Nothing was moved or deleted in `divagar_test1_ws`, `buggy_map_ws`
or `autoware_ndt_ws`; they still work as before.

```
buggy_unified/
├── buggy                  the one command (see below)
├── setup.bash             `source setup.bash` to use plain ros2 commands in this environment
├── src/
│   ├── buggy_pcd_lanelet_map/   map, localisation, routing, detection, AUTO, UART writer, tools
│   ├── buggy_rviz_plugins/      "Buggy State" panel + Add Car/Bus/Pedestrian tools (C++)
│   ├── buggy_perception/        wheel odometry, ground segmentation, self-hit filter, static TFs
│   ├── buggy_uart/              uart_bridge (ECU feedback) + UartInterface (frame encoder)
│   ├── ground_segmentation_msgs/   SteerSpeed message
│   ├── buggy_description/       URDF, Gazebo worlds (simulation only)
│   ├── velodyne/                VLP-16 driver + point cloud conversion
│   ├── radar_fusion/ fusion_common/ fusion_visualization/ perception_msgs/   radar tracker (--radar)
│   ├── umrr_ros2_driver/ umrr_ros2_msgs/   UMRR-A4 CAN radar driver (vendor SDK trimmed to x86_64)
│   └── camera_processing/       camera capture + YOLOX 18-class detector (--camera)
└── data/                  maps, models and calibration (portable; override with BUGGY_DATA_DIR)
    ├── maps/{new_delhi_map, office_loop1, office_track_sim}/
    └── models/yolox_s_ti_18class_ti_today_qat.onnx
```

## 1. Install on a machine

Needs **ROS 2 Humble** (the Autoware packages are the apt ones: `ros-humble-autoware-*`).

```bash
sudo apt install ros-humble-autoware-map-loader ros-humble-autoware-map-projection-loader \
     ros-humble-autoware-ndt-scan-matcher ros-humble-autoware-ekf-localizer \
     ros-humble-autoware-perception-msgs ros-humble-autoware-internal-localization-msgs \
     ros-humble-autoware-internal-debug-msgs ros-humble-rviz2 ros-humble-tf-transformations \
     ros-humble-cv-bridge python3-scipy python3-serial python3-yaml nlohmann-json3-dev
# only for --camera:  pip install onnxruntime   (and python3-opencv)
# only for ./buggy sim and ./buggy sim-map:
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
     ros-humble-ackermann-steering-controller ros-humble-xacro ros-humble-joint-state-broadcaster

cd buggy_unified
./buggy build          # first time, and after any change in src/ (use --clean for a fresh build)
./buggy doctor         # checks everything above + maps + the hardware
```
To move it to another computer, copy this whole folder, then run the three lines above.

## 2. Commands

| Command | What it does |
|---|---|
| `./buggy doctor` | Checks ROS packages, Python modules, build, maps, ECU serial port and LiDAR network link. Run first. |
| `./buggy maps` | Lists the maps in `data/maps` (lanelets, PCD size). |
| `./buggy view [MAP]` | Map + RViz + route + simulated AUTO. No sensors, no hardware. |
| `./buggy live [MAP]` | **Real buggy**: VLP-16, ECU feedback, NDT/EKF localisation, obstacle detection. Never moves the buggy. |
| `./buggy live [MAP] --drive` | Same, plus AUTO armed. Steering/stop frames are built and logged, **nothing is written** (loopback). |
| `./buggy live [MAP] --drive --real` | AUTO armed and the UART writer **really writes to the ECU**. Asks you to type `YES`. |
| `./buggy replay BAG [MAP] [--drive]` | The same stack on a recorded rosbag. Real writing is refused. |
| `./buggy live/replay ... --radar` | Adds radar obstacles (UMRR-A4 on CAN + radar_fusion IMM tracker). Not in a bag replay (no radar topic). |
| `./buggy live/replay ... --camera` | Adds camera + YOLOX: LiDAR/radar obstacles get a class name instead of "unknown". |
| `./buggy sim [MAP] [--radar] [gui:=true]` | Gazebo vehicle + emulated ECU, closed loop. Default map `office_track_sim`. `--radar` = synthetic radar. |
| `./buggy calibrate-brake [--preset 2\|4] [--live]` | Measures the real stopping distance. Dry run unless `--live`. |
| `./buggy add-map NAME FILE.osm [--pcd FILE.pcd]` | Creates `data/maps/NAME/` from a Vector Map Builder export and checks it. |
| `./buggy sim-map` / `./buggy sim-map --save` | Drives the Gazebo lane and records a PCD (`--save` writes it). |
| `./buggy params-check [FILE]` | Checks `config/buggy_params.yaml` (or your own file) against the Python and C++ nodes: typos, wrong types, values that must agree. |
| `./buggy stop` | Clean Ctrl+C to the running launch. |
| `./buggy ros2 ...` | Any ros2 command in this environment, e.g. `./buggy ros2 topic echo /localization/state`. |

`MAP` is a folder name in `data/maps` (default `new_delhi_map`) or a full path.
Extra `key:=value` arguments go straight to `ros2 launch`:
`voxel_size:=0.5  ndt_rate_hz:=5.0  corridor_half_width:=1.0  feedback_port:=/dev/ttyUSB1
sender_port:=/dev/ttyUSB1  bag_start_delay:=12.0  bag_rate:=0.35  can_interface:=can0
detector_model:=<onnx>  camera_device:=/dev/video0  labeler_debug:=true`.
Options: `--no-rviz`, `--no-detector`, `--radar`, `--camera`, `--params FILE`.

### Parameters: `config/buggy_params.yaml`
All tuning parameters of the nodes (125 of them: stop distances, detour, obstacle detection, ground filter, vehicle and
LiDAR geometry, ECU serial ports, ...) are in **one file**, `config/buggy_params.yaml`, one section per node. The values in
it are the built-in defaults, with a comment for each. The Python and the C++ nodes (`--cpp`) read the same file.
* **Change a value:** edit the file and restart `./buggy ...`. No rebuild.
* **Your own file:** `./buggy live --params my_params.yaml` - list only the keys you want to change, e.g.
  ```yaml
  operation_manager:
    ros__parameters:
      avoid_stop_margin: 9.0
  uart_bridge:
    ros__parameters:
      port: "/dev/ttyUSB1"
  ```
* **Priority** (highest wins): command line `key:=value` (e.g. `corridor_half_width:=1.0`, `feedback_port:=...`) >
  `--params FILE` > `config/buggy_params.yaml` > the default in the code. A launch argument you do not give does nothing.
* **Check it:** `./buggy params-check` (also part of `./buggy doctor`) reports unknown names, wrong types (write `5.0`,
  not `5`, for a float) and warns when the same vehicle dimension differs between nodes (LiDAR mount, wheelbase,
  steering limit, bumper offset).
* **Look at the running value:** `./buggy ros2 param get /operation_manager avoid_stop_margin`.
* **Not in the file** (they are switches or wiring of the `./buggy` options): `live`, `drive_enabled`, `--avoid`,
  `radar` / `camera`, `base_link_at_center`, map paths. `stop_react_sec` / `stop_decel` are replaced by
  `data/buggy_brake_calibration.yaml` once you have calibrated the brakes.
The path of the folder is `$BUGGY_CONFIG_DIR` (set by `setup.bash`, default `<folder>/config`).

## 3. Real buggy: step by step

1. `./buggy doctor` — the ECU serial port and the LiDAR link must both be `[OK]`.
2. `./buggy live` — localisation only. In RViz: **2D Pose Estimate** near where the buggy really is
   (right heading). The panel's Localization row should go `aligning → initialized → converged`
   with an NDT score above 1.4 (the bag replay gave about 3).
3. Check the data: `./buggy ros2 topic hz /velodyne_points` (about 20 Hz),
   `./buggy ros2 topic echo /wheel_uart --once`, `/localization/state`.
4. **Calibrate braking before any drive** (section 4). Until then the stop model uses guesses.
5. `./buggy live --drive` — dry run of AUTO. Set a **2D Goal Pose**, press AUTO and watch
   `/steering_angle`, `/final_red_detected`, `/final_yellow_detected` and the logged frames
   (`/uart/frame`). Nothing reaches the ECU.
6. With a safety driver, wheels clear or clear space ahead: `./buggy live --drive --real`.

### What AUTO does (live)
* Follows the smoothed lane with pure pursuit (max steering 20 deg, wheelbase 1.6 m) at the ECU's
  **4 km/h** preset, or the **2 km/h** preset (yellow=1) when slowing.
* **Obstacle** (LiDAR, ground removed, 0.2–2.0 m high, inside 1.25 m of the route centre line):
  slows to 2 km/h inside `stopping distance at 4 km/h + 6 m`, sends red (stop) when the gap
  (front bumper to obstacle edge) is within the stopping distance plus a 2.5 m margin, holds, and
  resumes 1 s after it is clear. Stopping distance = `v·react + v²/(2·decel)`.
* **Goal**: slows 6 m before and stops with the rear axle ~0.5 m short of the goal point
  (the front bumper ends up about 1.7 m past it).
* Immediate stop (red, steering 0) on: STOP/EMERGENCY, localisation not converged > 1 s, pose older
  than 0.5 s, more than 2.5 m off the route, obstacle detector silent > 0.7 s, no route, and the UART
  writer's own watchdog (no fresh command for 0.5 s).
* The ECU has **no deceleration command** (only red=1 and the 2/4 km/h presets), so the actual
  braking is the ECU's.

### Auto goal / closed loop (return to where you started)
The panel's **Auto goal (closed loop)** box (or `/system/request`):

| Button | Request | Does |
|---|---|---|
| Loop back to HOME (laps: n) | `loop:<n>` | Plans the route to HOME and n-1 more laps (n laps if the buggy is already at HOME); the route ends at HOME. Then press AUTO. |
| Return HOME | `home:go` | Route from here back to HOME. |
| Set HOME here | `home:set` | Makes the current pose HOME. |
| - | `home:clear` | Forgets HOME (also forgotten on a new 2D Pose Estimate). |

**HOME** = the pose where AUTO was first engaged (or where you pressed *Set HOME here*); it is drawn as a
green disc labelled HOME. Without the panel: `./buggy ros2 topic pub --once /system/request std_msgs/msg/String "{data: 'loop:2'}"`.
The multi-lap route is one continuous path, so the buggy does not stop between laps; it stops at HOME
(rear axle ~0.5 m before the point, as for any goal).

**Goals that used to give "No route"**: a lane that ends where it starts (a ring, as exported by Vector Map
Builder with separate start/end nodes) was not connected in the lane graph, so a goal *behind* the buggy
had no route (on `small_loop` 40 of 53 goals failed from some start positions). Lane ends within
`link_gap` (4 m) of a lane start and heading within 60 degrees are now linked, and a goal behind the buggy
routes around the loop (status: `[goal is behind: around the loop]`). A goal far from any lane is snapped to
the nearest lane and the status says how far (`[goal is 27 m from the lane: snapped to it]`).
Random start/goal pairs on all four maps: 0 of 300 without a route.

### Sensor drop-outs: obstacle memory
A LiDAR/radar obstacle that has been seen for >= 0.5 s (5 scans) and then disappears is **remembered**
(`obstacle_memory_sec:=4`, `0` = off): the slow-down and the stop stay active, with the gap dead-reckoned from
the buggy's own progress, until the buggy has stopped at the stop line (then held for `obstacle_memory_sec`),
and for the next 10 s the buggy creeps at 2 km/h. A drop-out right before the stop line therefore no longer
cancels the stop. One-scan flickers are ignored. The trade-off: an object that really left (a person walking
away) can hold the buggy for a few seconds. The log line `AUTO: v=.. gap=.. stop_below=.. hold=.. -> RED/SLOW/GO`
(once a second) and `/system/state` (`obs_gap`, `stop_below`) show why the buggy goes, slows or stops.

### C++ runtime core (`buggy_core`) - migration from Python
Policy: **C++ for the runtime core, Python for non-runtime work** (tools, CLI, map import, calibration, simulation, tests).
The C++ nodes are drop-in replacements for the Python nodes of the same name (same topics, parameters and output);
the Python versions stay as the reference and fallback. Select with `--cpp` (or `core:=cpp`) on
`live` / `replay` / `sim` / `calibrate-brake`.

| Node | C++ (`buggy_core`) | Python (reference) |
|---|---|---|
| `uart_writer` (only serial writer, watchdog, final STOP) | done | `buggy_pcd_lanelet_map` |
| `lidar_obstacle_detector` | done | `buggy_pcd_lanelet_map` |
| `operation_manager` (AUTO control loop, **live mode only**: real buggy, Gazebo, replay) | done (the RViz-only simulated vehicle, `view`, stays in Python) | `buggy_pcd_lanelet_map` |
| `segment_ground` (the `segment` backend used on the vehicle), `self_hit_filter` | done (the GPU / RANSAC backends stay Python-only, not used live) | `buggy_perception` |
| `wheel_odometry` (`/wheel_uart` -> `/odom`) | done | `buggy_perception` |
| `uart_bridge` (ECU feedback reader, `/wheel_uart`) | done | `buggy_uart` |
| `route_planner` (goal -> lane route, smoothing, HOME / loop) | done (works in `view`, `live`, `replay`, `sim`) | `buggy_pcd_lanelet_map` |
| `avoidance_planner` (detour around a blocked lane, lane + LiDAR free space) | done | `buggy_pcd_lanelet_map` |
| `localization_manager` (click -> NDT align -> seed EKF, state, TF-conflict and pose-jump watchdog) | done | `buggy_pcd_lanelet_map` |
| `lidar_preprocess` (NDT input: crop + voxel downsample), `odom_to_twist`, `velodyne_static_tf` | done | `buggy_pcd_lanelet_map`, `buggy_perception` |

**Map loaders (own, `buggy_core/mapping`):** `map_projection_loader` (publishes `map_projector_info.yaml` on `/map/map_projector_info`)
and `pointcloud_map_loader` (publishes the PCD on `/map/pointcloud_map` and serves the partial / differential map services that
`ndt_scan_matcher` uses) replace `autoware_map_projection_loader` and `autoware_map_loader/pointcloud_map_loader`; same topics, services and
parameters, so nothing else changed. Checked against the Autoware nodes on `small_loop`: the 2,945,602-point cloud is identical point for point
(same message layout), the projector message is the same, and a bag replay with the new loaders gives `initialized` then `converged` (NDT score 3.1).
`autoware_map_loaders:=true` switches back to the Autoware ones. Not implemented: downsampled-whole and selected loading (unused). Still Autoware:
`lanelet2_map_loader`, `ndt_scan_matcher`, `ekf_localizer`.

**The whole real-buggy runtime path is now C++** (`./buggy live --cpp`): 11 nodes of `buggy_core` plus the Autoware C++ nodes
(NDT, EKF, map loaders) and the Velodyne driver. What stays Python on purpose: the RViz-only simulated vehicle (`view`),
tools and calibration, the simulation helpers, map-display helpers (`lanelet_marker_publisher`, `map_display_cloud`,
`initial_pose_vehicle`, `dummy_objects`), and the optional radar / camera nodes (`radar_object_bridge`, `object_labeler`,
colleague's `radar_fusion` and detector).

Verification of what is ported (the Python code generates the golden data, `src/buggy_core/tools/golden/gen_golden.py`):
* `colcon test --packages-select buggy_core`: 72 UART frames **byte-identical** to `UartInterface.create_message`, LiDAR
  clustering identical to the Python algorithm on the same points, corridor / height filters.
* `tools/actuation/test_uart_pty.py <package>` (pseudo-terminal, no hardware): commands -> bytes, watchdog -> STOP, SIGINT -> final STOP.
  Both implementations pass; the C++ one sends 3 final STOP frames and exits 0 (the Python one 1 frame, exit code 1).
* Bag replay, both detectors on the same scans: 124 of 124 messages with the same objects, worst position / size
  difference 0.04 m (float32 vs double at cell boundaries).
* `operation_manager`: tracking maths (nearest-point match, pure pursuit, cross-track, obstacle-in-corridor distance,
  stop distance) **bit-identical** to the Python methods on 60 poses / 960 object checks; behaviour tests for GO / SLOW /
  STOP, obstacle memory (the lost-obstacle bug), detector / pose / localisation faults, arrival and brake-calibration hand-over.
* Bag replay with both `operation_manager` versions running side by side on the same TF / objects / requests
  (C++ remapped to `/cpp/...`): the **same GO / SLOW / RED sequence at the same instants**, same gap values, same messages.
* Ground filter + self-hit filter on the same 332 real VLP-16 scans from the bag (C++ side by side with Python): ground,
  non-ground and filtered clouds **identical point for point, including the colour values**; the synthetic golden set has 0 differences.
* `wheel_odometry`: 1,199 integration steps through the real Python callback (fake clock) vs C++: worst position difference 3.6e-15 m.
  Bag replay side by side: every C++ (speed, yaw-rate) pair is also published by Python, same distance travelled (33.49 m).
  Note: in a replay the Python node's first `dt` spans the jump from clock 0 to bag time, so its /odom pose starts with a huge
  offset (x ~ 1e9); only the twist reaches the EKF, so localisation is not affected, and on the real buggy there is no such jump.
* `route_planner`: on all 4 maps in `data/maps` - lane geometry, successor lists (including the ring links), 56 random
  start/goal plans (same lane chains, same smoothed paths, worst point difference 8e-14 m) and 12 one-lap paths identical to Python.
  Node level (`view`, C++ beside Python, same inputs): paths, status texts and markers identical for goals, HOME, `loop:1`, `loop:2`,
  a bad lap count, clear route (worst path difference 1e-13).
* `avoidance_planner`: 56 detour scenes from real maps (small / vehicle-sized obstacles, wide barriers, with and without the
  lane model and LiDAR grids): 39 detours and 17 refusals - same success flag, same refusal text, same side / offset / ib, every
  detour point **bit-identical** (difference 0). The detour result applied by `operation_manager` (path splice, status, message)
  matches the Python `on_avoid_result` on 40 results (30 spliced detours).
  Closed loop in `view small_loop --cpp` (C++ route + avoidance planner): "Detour right by 1.5 m", passes the pedestrian at 1.5 m,
  "detour finished: back on the lanelet path", reaches the goal - same offset as the Python planner.
  Speed: about the same as Python (tens to a few hundred ms per request); planning on request is not time-critical.
* `localization_manager`: 40 random event sequences (TF edges from several parents, NDT poses, scores, state changes) through
  the real Python methods vs C++: 403 `/localization/state` texts identical, covering converged / degraded / lost / initialized,
  99 with a TF conflict and 296 with pose jumps. Live on the bag replay: align (reliable=True), EKF and NDT activated, `converged`;
  a second publisher of `base_link` (fake odometry) is reported as `base_link<-map+odom` with the jump count and clears again.
* `uart_bridge`: 60 byte streams cut at random places (valid packets, garbage, bad end bytes) through the real Python
  `timer_callback` vs the C++ parser: 758 packets identical, in order, with the same resync behaviour. Pseudo-terminal test
  `tools/actuation/test_feedback_pty.py <package>` (the test plays the ECU): C++ passes all checks (decode, garbage, split packet,
  +-20 deg clamp, 100 packets at 50 Hz, adapter unplugged); the Python bridge lost 1 of 100 packets at 50 Hz in one run
  (its serial read blocks up to 50 ms; the C++ node polls without blocking).
  Not tested on the real adapter (no hardware here).
* `lidar_preprocess`: 12 synthetic scans (NaN / inf beams, the buggy's own body, voxel sizes 0.2 - 1.0 m, one too sparse) through
  the real Python `on_cloud` vs C++: 29,250 voxel centroids **bit-identical**. On the real bag (C++ beside Python, same scans):
  283 of 283 downsampled clouds identical; `odom_to_twist`: 337 of 337 messages identical (frame, twist, covariance);
  `velodyne_static_tf`: same transform (base_link -> velodyne, 1.73 / 0 / 1.00 m, no rotation).
* Replay with `--drive --cpp`: the same GO -> SLOW -> RED sequence (stop at 4.9 m with a 5.0 m threshold).
* **Timing:** the Python `uart_writer` was *not* a timing problem: frame interval at 10 Hz, worst case 103 ms idle, 108 ms
  with every core busy, 113 ms at twice the cores - C++ is the same (103 / 108 / 116 ms). The reasons to port it are
  determinism guarantees, no interpreter on the vehicle and easier safety review, not measured speed.


### Ground is not an obstacle (ground-aware detector)
The detector used to measure "height above ground" from a flat plane 1.0 m below the sensor. On a slope, over a bump, or
while the buggy pitches under braking, road surface then rose above the 0.2 m threshold and became an obstacle. Now (default on,
Python and C++ detectors alike):
* the ground points of the same scan (`/ground_points`) give a **local ground height**: median per 0.5 m cell, smoothed over
  3 m; where no ground point is near (far range) a **plane fitted to all ground points**; only if there is no ground at all the old
  flat plane. Obstacle points must be `ground_min_height` (0.30 m) above that local ground.
* a cluster that is both **thin** (vertical extent < `thin_extent` 0.08 m) and **low** (< `low_height` 0.5 m above ground) is one
  beam ring on the road, not an object, and is dropped (`reject_thin_low`).
Switches: `ground_aware:=false` (old behaviour), `reject_thin_low:=false` (needed if you must see single-ring objects below
0.5 m such as cones). Parameters of the node: `ground_min_height`, `thin_extent`, `low_height`, `ground_max_age`.
Tested: synthetic slopes (a 4 deg rise with the far ground leaked into the non-ground cloud is no longer an obstacle, a box, a wall and a
0.45 m cone on the slope still are, with their heights corrected), 60 random scenes Python vs C++ identical, and on the real bag
(Python and C++ detector side by side, 249 scans, same objects). Side effect to know: the real ground in that bag is not at the
assumed -1.0 m (about -0.75 m 3-6 m ahead, -1.4 m 6-12 m ahead), so the ground-aware height also reports real far / low objects the flat
plane used to hide (+93 clusters in 868 scans, median 1.4 m tall). Record a bag at the place where the false obstacles
appeared (`ros2 bag record /velodyne_points /wheel_uart /tf /tf_static`) to tune the three thresholds on that data.

### Going around a blocked lane (detour / obstacle avoidance)
`--avoid` (default **on** in `view`/`sim`, **off** in `live`/`replay`; `--no-avoid` turns it off).

**How it runs (stop first, then replan):**
1. The buggy slows down and **stops** in front of the obstacle as usual, at `stop_margin` (2.5 m) from the bumper.
   *Optional, off by default:* `avoid_stop_early: true` (in `config/buggy_params.yaml`, section `operation_manager`) makes the
   buggy stop **exactly `avoid_stop_margin` metres** (bumper to the obstacle's near edge) before an obstacle, so a detour has room.
   You choose the distance; the planner never overrides it. Why a distance is needed at all: the buggy only drives forward (no
   reverse) and turns no tighter than ~4.4 m radius. Measured: on a straight lane with space beside it a detour was found from a
   2.5 to 4 m gap; on `small_loop` (a bend) a 2 m stop found none and a 3.5 m stop (`avoid_stop_margin: 4.0`) worked.
   * `avoid_probe: true` (default): while approaching, the planner is asked how much room it thinks a detour needs. The answer is
     shown as `detour_room` in `/system/state` and, if your stop line is closer, one warning is logged ("the detour may be
     refused"). The estimate is conservative: the detour in the example above worked although it said 6.6 m. `false` = no probing.
   * `avoid_stop_follow_planner: true` (default `false`): additionally move the stop line back to that estimate + 0.5 m (up to
     20 m), so a wide obstacle stops further away. Off, the stop line is always exactly `avoid_stop_margin`.
   The planner itself does not need 7 m. What it needs is *room beside the lane* (the front corners swing wide during the arc; without
   LiDAR only `offlane_allow_m` = 0.8 m outside the lane counts, set 3.0 in `avoidance_planner` for `view`) and *no tight bend* right
   there (an offset towards the inside of a bend pushes the steering past the limit).
   With the default (off) a detour from 2-3 m in front of the obstacle mostly fails ("needs more steering" / "needs ~7 m
   before the obstacle") and the buggy simply stays stopped.
2. The panel message counts it down (`... | replanning in 3 s`). Only after the vehicle has **stood still for 5 s** (`avoid_after_sec`) the
   planner is asked for a detour (before, while it was still driving, nothing is replanned).
3. The detour is **inside the lane** when the buggy fits (smallest shift that clears the obstacle). If the lane is
   blocked completely, the planner looks for free ground **outside the lane** (LiDAR): the part of the detour inside the lane is drawn
   **cyan**, the part outside the lane **magenta** with the label *DETOUR OUTSIDE THE LANE*, and the free ground the LiDAR
   has seen outside the lane around the obstacle is drawn as **green** cells (displays *Detour* and *Free space outside
   the lane*; the green cells need LiDAR data, i.e. `live` / `replay` / `sim`, not plain `view`).
4. **If the obstacle disappears while the buggy is still on the detour** (and it has not yet reached the obstacle),
   after `avoid_clear_sec` (2 s) the buggy goes **back to the original route** - a smooth S-curve from where it is onto
   the original lanelet path, not along the old detour - and the panel says *Route restored*. Beside or past the
   obstacle it finishes the detour instead (the detector no longer sees an obstacle behind it).
5. The original route is always kept; when the detour is finished the buggy is back on it.

A detour is accepted only if the whole swept body footprint is clear of every known obstacle, of solid LiDAR
cells, and inside the drivable area: the lane, or up to 0.8 m outside it (2.0 m where the LiDAR has actually
seen free ground; without LiDAR data, e.g. in `view`, only 0.8 m is assumed). The detour is drawn in cyan
(display *Planning / Detour*), the buggy creeps at the 2 km/h preset while on it, and the normal
slow/stop logic still applies to anything else near it.
If there is no room it stays stopped, says why in the panel (`Blocked, no detour: ...`) and retries every 5 s (6 tries).
Limits to know:
* it needs about **7-12 m** in front of the obstacle (see the stop line above); if the planner says it needs more room than
  the buggy has, it stays stopped and says how much room it would need;
* a **tight curve** or a **narrow lane** (3.2 m lane, 0.6 m pedestrian: ~1.4 m shift) often leaves no room:
  on `small_loop` the planner found a detour for a pedestrian at 5 of 16 positions with lane-only data;
* a **car** (1.8 m wide) in a 3.2 m lane needs ~2.4 m outside the lane: only possible with LiDAR confirmation;
* **Lane completely blocked** (a barrier or wide vehicle across it): with LiDAR data the detour leaves the lane
  into the free space the LiDAR has actually seen - ground that was seen (>= 65 % of the body samples that fall
  outside the lane) and no solid cell - up to `max_offset_m` (6.5 m) from the route, passes the end of the
  obstacle and returns to the lane. The extent of the barrier comes from the connected solid LiDAR cells, not
  only from the detected boxes (which are clipped to the corridor). Wider offsets are tried in 0.5 m steps, on
  both sides, the smallest first. Tested with a synthetic LiDAR world (bus 10.5 m across the lane: detour 6.6 m
  around its end, back on the route within 0.14 m); **not yet with real VLP-16 data**. The VLP-16's ground
  returns are sparse at range (rings ~0.7 m apart at 10 m) and nothing is seen behind the obstacle, so the
  ground check may need tuning (`ground_fraction`, ground dilation). In `view` there is no LiDAR: pass
  `avoid_use_sensors:=true` together with something that publishes `/non_ground_points_filtered` and `/ground_points`;
* it never reverses and does not look for a different lanelet (no lane change / re-route).

### RViz window: maximised and resizable
RViz opens **maximised** (`rviz_maximize.py` sends the standard window-manager request once the window exists;
`rviz_maximize:=false` keeps the saved size). It stays resizable: restore/resize with the title bar or drag
the window edges; the dock panels can be resized/floated as usual in RViz. The "Buggy State" panel is laid out
in **two columns** (status + operation mode on the left, vehicle + auto goal on the right, buttons and message
below) instead of one tall column - no scroll area. Its old single-column height made the RViz window taller
than a 1200 px screen (minimum height 1220), so it could not fit or be resized; the window minimum is now about
836 x 946 and the left dock is about 780 px wide.

### Look: splash screen and royal-blue theme
A "BUGGY" splash window (the image's own 960x540 size, centred, always on top, progress bar) is shown for
`splash_sec:=5` seconds; **RViz is started only after it has closed**, so nothing else is on screen during the
splash and the RViz main page is what appears afterwards (RViz needs about a second to load after that). Esc skips
the splash. (RViz always shows its own logo splash unless it is given one, so it is started with a 1x1 transparent `-s rviz/no_splash.png`, which hides it; ours is
`splash_screen.py`: Tk, no ROS. With no display/Tk it is skipped and RViz starts at once; `rviz:=false` has no splash.) RViz is started with
`-stylesheet <theme.qss>` (Qt style sheet: royal blue #4169E1 toolbar, menus and buttons, darker royal blue
panels, navy 3D background #0C163C). Files: `src/buggy_pcd_lanelet_map/rviz/buggy_splash.png`,
`buggy_theme.qss`. Change the image/colours there, or per run: `splash:=/path/img.png`, `theme:=/path/x.qss`;
`splash:=none` / `theme:=none` give the plain RViz look; `splash_sec:=8` changes the time.

## 3b. What RViz shows (Autoware-style planning display)

* **Route**: the route lanelets in translucent green, the smoothed path in yellow (raw lane centre in grey).
* **Stop / Slow Zones** (display *Planning / Stop-Slow Zones*, topic `/planning/stop_zone_marker`), drawn on the
  road ahead of the front bumper for the nearest obstacle and for the goal:
  * **orange** - slow zone: from here the buggy drops to the 2 km/h preset;
  * **blue** - stop zone: the red (stop) command goes out at its start, the buggy comes to rest inside it;
  * **light red** - keep-clear margin between the stop line and the obstacle (2.5 m);
  * a **red wall** at the stop line with a label (`STOP: pedestrian 4.1 m`, `GOAL 3.0 m`).
  The zones come from the same stopping model as the logic (`v*react + v^2/(2*decel)`), so they grow
  as you calibrate the brakes. They are shown even before AUTO is engaged (preview).
* **LiDAR Objects** (red boxes), **Radar Objects** (orange boxes, velocity arrow, `radar <class> <km/h>`),
  **Labeled Objects** (class-coloured wireframe + `name score`, with `--camera`).

## 3c. Radar and camera (optional, off by default)

```bash
./buggy live --drive --radar            # radar obstacles also stop/slow the buggy
./buggy live --camera                   # names for the obstacles
./buggy live --drive --radar --camera   # both
./buggy sim --radar                     # Gazebo: synthetic radar (radar_sim); there is no camera model
```
* **Radar**: `umrr_ros2_driver` (CAN, `can_interface:=can0`) -> `/smart_radar/can_targets_0` -> `radar_fusion`
  (your IMM tracker, output in the `map` frame) -> `radar_object_bridge` -> `/perception/radar_objects`.
  The AUTO logic treats radar objects like LiDAR ones (same corridor, same stop model). A silent radar is
  **not** a fault (the LiDAR detector is the safety heartbeat); the panel's Sensors row shows `radar STALE`.
* **Camera**: `camera_raw_node` -> `yolox_detector_node` (18-class IDD model, `/camera/detections/array`) ->
  `object_labeler`, which projects every LiDAR/radar obstacle box into the image (fisheye model of
  `camera_lidar_fusion_node`, camera pose from `camera_static_tf`) and matches it to a detection by IoU.
  It only **adds** names (to the RViz labels, the zone label and the panel message); it never removes or delays
  an obstacle, so stopping behaves the same with the camera off, blocked or wrong.
* CPU: the detector is limited to 2 threads and 5 frames/s (`detector_threads:=`, `detector_rate_hz:=`) so
  NDT keeps up. On a laptop CPU, use `bag_rate:=0.35` for replay with `--camera`.
* **Accuracy**: `base_link -> camera_link` in `camera_static_tf.py` is a rough estimate, not a calibration.

## 3d. Mission: named stops (home, A, B ...) with a wait at each

The buggy drives to stop 1, stands there `wait_sec` (default 10 s), drives on to stop 2, ... and after the last
stop goes back to the first (`loop: true`) or ends. The stops are a file in the map folder,
`data/maps/<map>/mission.yaml`:

```yaml
loop: true            # after the last stop go to the first again
wait_sec: 10.0        # standing time at every stop
start_as_home: true   # "home" is optional, see below
stops:
  - {name: A, x: 25.04, y: 33.21}
  - {name: B, x: -7.55, y: 61.82, wait_sec: 20.0}     # own waiting time
```

**Home is optional.** With `start_as_home: true` there is no home in the file: home is wherever the buggy stands
when you press `mission start` - set it in RViz with **2D Pose Estimate**, the usual start flow. It becomes the last
stop "home" (A -> B -> home -> A ...), so the buggy returns to where it started. With `false` the loop is just
A -> B -> A. A stop that is really named `home` in the file (fixed place) wins over the start pose.

x, y are map metres (what RViz shows). The route planner snaps a stop that is off the lane to the nearest lane.
`data/maps/small_loop/mission.yaml` holds example stops - replace them with your real places.

| Command | What it does |
|---|---|
| `./buggy mission here NAME [MAP] [--wait SEC]` | add the vehicle's position NOW as a stop (stack running and localized) - drive to the place, run it |
| `./buggy mission pick NAME [MAP] [--wait SEC]` | add the next RViz "2D Goal Pose" click as a stop |
| `./buggy mission list [MAP]` / `check [MAP]` | show / validate the file (fields, duplicate names, stops closer than 4 m) |
| `./buggy view\|live\|sim MAP --mission` | start the stack with the mission node (stops shown in RViz: yellow, the target green) |
| RViz panel **Mission (stops A, B ...)** | **Start mission / Stop mission / Resume** buttons and the live state ("DRIVING: stop 2 of 3 \"B\"", "WAITING ... 6 s left") - the same as the three commands below; shows "mission node not running" when the stack was started without `--mission` |
| `./buggy mission start [N]` | begin, at the first stop or at stop N (counted from 0); a stop the buggy already stands at is skipped |
| `./buggy mission stop` / `resume` / `status` | stop the buggy and the mission / go on (only with `auto_restart: false`) / show the state |

How it drives: the `mission_manager` node does what the operator does by hand - it sends the next stop as the goal
(`/goal_pose`), waits for the new route, asks for AUTO (`/system/request mode:auto`), and sees `routing=arrived` in
`/system/state`. Obstacle stops and detours happen inside a leg, as always. It works with the Python and the `--cpp`
nodes. Behaviour (config/buggy_params.yaml, `mission_manager`): `auto_restart` (drive on by itself after the wait),
`autostart`, `plan_timeout_sec`, `abort_stop_sec`, `at_stop_radius`.

Safety: the mission only presses AUTO - the vehicle still needs `--drive` (and `--real` for the ECU). It is
aborted (the buggy stays stopped, it never restarts by itself) when there is no route to the next stop, when AUTO is
refused, or when AUTO drops to stop on the way for `abort_stop_sec` (operator STOP, emergency, a fault).
`./buggy mission stop` also stops the buggy. Not tested on the real buggy.

### The Buggy Manager app (set the stops with the mouse)

```bash
./buggy app
```

1. Pick the map and click **Waypoint Set**: the map opens in RViz (`./buggy view MAP`, its output is shown in the window).
2. **A** / **B** -> then RViz **2D Goal Pose** on the lane. "Other stop" adds more (name, click type, optional
   wait seconds). **Home is optional**: by default it is simply where the buggy starts - set it in RViz with
   **2D Pose Estimate** before `mission start` (checkbox "Home = where the buggy starts"). "Fixed home" stores a
   fixed place instead.
3. Every click is written to `<map>/mission.yaml` at once and drawn in RViz (yellow discs with the name). The table
   lets you move, delete, set the wait of a stop and switch the loop on/off.
4. **Close map**, then drive it: `./buggy live MAP --drive --mission` and `./buggy mission start`.

The app needs a desktop (Tk) and the ROS environment (`./buggy app` sets it). Test: `python3 -m pytest src/buggy_pcd_lanelet_map/test`.

### Camera views panel (RViz)

The **Camera Views** panel (RViz window, dockable; nothing is shown and nothing subscribed until you press a button)
has three toggle buttons - press the active one again to hide it:

| Button | Topic | Shows |
|---|---|---|
| Camera feed | `/object_detection/image_rect` | the plain rectified camera image (`--camera`) |
| Camera overlay feed | `/fusion_visualization/overlay` | boxes, lane mask and path drawn on the image (`--camera --overlay`) |
| Lane path curvature | `/sensor_fusion/lane_path` | top view of the tracked lane path (x forward) and its curvature near / mid / far (1/m, radius, left/right), the steering that follows it (+right, like `/steering_angle`) and the tightest point (red when tighter than the buggy's 4.4 m minimum radius) |

Without data it says which option to start with (e.g. "no data on ... (start with --camera --overlay)").

## 4. Brake calibration

```bash
./buggy calibrate-brake --preset 4           # dry run: checks feedback + pose, sends nothing
./buggy calibrate-brake --preset 4 --live    # drives straight, then red; asks for YES twice
./buggy calibrate-brake --preset 2 --live
```
Needs 15 m of clear flat space and a safety driver. Repeat 3+ times per preset. Each run is
appended to `data/buggy_brake_runs.jsonl`; `data/buggy_brake_calibration.yaml` is rewritten from the
worst case over all runs times 1.2, and AUTO reads it at start-up (the panel tooltip on
"Obstacle detector" shows `measured` instead of `default (NOT measured)`).
To practise without touching the real calibration: `BUGGY_DATA_DIR=/tmp/x ./buggy calibrate-brake`.

## 5. Adding a new map (Vector Map Builder export)

```bash
./buggy add-map my_new_map ~/Downloads/my_map.osm                 # uses the new_delhi_map PCD
./buggy add-map my_new_map ~/Downloads/my_map.osm --pcd /path/to/pointcloud.pcd
./buggy live my_new_map
```
The script copies the .osm, writes `map_projector_info.yaml` (`Local`, for exports with empty
lat/lon and `local_x/local_y` tags), links the PCD and prints: lanelet count, extent, how many lanes
connect, and **how much of the lane the PCD covers** (below 90 % means NDT cannot localise there).
A map folder is: `lanelet2_map.osm`, `pointcloud_map.pcd`, `map_projector_info.yaml`,
`pointcloud_map_metadata.yaml`. The PCD and the .osm must use the same coordinate frame.

## 6. Simulation (no hardware)

* `./buggy sim` — Gazebo `office_track.world` (spawn 0, -3, 0) + the same localisation/AUTO code, with an
  **emulated ECU** (3.5 s start delay, 0.4 s reaction, 1.2 m/s² decel — assumptions, not measurements).
  Click 2D Pose Estimate at the spawn, then 2D Goal Pose, then AUTO. Dummy cars/buses/pedestrians
  come from the RViz toolbar (`c`, `b`, `p`).
* `./buggy sim-map`, then in a second terminal `./buggy sim-map --save` — records `office_track_sim`'s PCD.
  Do not kill a running `pointcloud_map_accumulator`: it saves its partial map over the PCD on exit.

## 7. Main topics

`/localization/state` (state, NDT score, pose), `/system/state` (mode, routing, drive, detector),
`/system/request` (`mode:auto`, `emergency:true`, `velocity_limit:4`...), `/planning/route_path`,
`/perception/lidar_objects`, `/steering_angle` `/final_red_detected` `/final_yellow_detected`
(commands), `/uart/frame` and `/uart/status` (what the writer sends), `/wheel_uart` (ECU feedback).

## 8. Not measured / limits (read before driving)

* Stop model defaults (react 1.0 s, decel 0.8 m/s²), the 40 deg/s steering-rate limit, the 2.5 m
  off-path limit and the 0.5 m goal margin are **placeholders** until measured.
* The VLP-16 is sparse: a person beyond ~10 m, or a thin branch, can be missed. The detector needs
  5 points confirmed in two consecutive scans. It does not classify or track objects.
* Vehicle dimensions: wheelbase 1.6 m, front bumper ~2.2 m ahead of the rear axle (URDF) — the
  self-hit filter has 2.26 m and a second, larger 4.9 x 2.9 m figure; confirm on the vehicle.
* `base_link` is the **rear axle** on the buggy and the chassis centre in Gazebo (handled by
  `base_link_at_center` in the sim launch).
* **Sim map quality:** `office_track_sim`'s PCD was recorded with Gazebo *wheel odometry*, which drifts
  over the 260 m loop. Measured: with the vehicle at rest at the spawn (truth 0, 0, 0 deg), a click at the
  spawn made NDT converge at about (-2.1, -1.65, 14 deg) with score 3.0, i.e. ~2.7 m / 14 deg off
  while reporting `converged` (it corrected itself once the vehicle moved). Use the sim for the control
  and stop logic, not for judging localisation accuracy. A fix (record the map with Gazebo ground-truth TF) is not done.
* `office_loop1` has only a lanelet: its PCD is the `new_delhi_map` one (the lane lies inside it, but it was not driven).

## 9. Troubleshooting

* **RViz view / vehicle / map jumping or "flickering"**: the usual cause is a second publisher of the same TF
  frame on the same ROS domain (an old stack's `kiss_icp` odometry, a bag-play container, a second `./buggy`).
  `./buggy live|replay|sim|view|calibrate-brake` now refuses to start when other ROS nodes already run in
  the domain (`BUGGY_FORCE=1` overrides), and while running the localisation monitor reports
  `TF CONFLICT: base_link<-map+odom` in the log and in the panel's Localization row, and AUTO stops with
  `FAULT: TF conflict`. The panel also shows `(n pose jumps/30s)` when the pose jumps > 0.5 m repeatedly.
  Re-clicking *2D Pose Estimate* re-seeds the EKF and legitimately makes one jump.
* **Other ROS programs on the same network/PC interfere** (e.g. the old stack's `kiss_icp` or a bag-play
  container still running): they share the ROS graph, load the CPU and can publish TF. Use a private domain:
  `ROS_DOMAIN_ID=77 ./buggy live` (also for any `./buggy ros2 ...` you run next to it).

* Topics missing / `ros2 topic list` empty: a stale daemon — `./buggy` stops it for you; by hand `ros2 daemon stop`.
* No `/velodyne_points`: LiDAR cable/IP (`enp2s0` needs 192.168.1.77/24), or a leftover driver
  holding UDP port 2368 (`./buggy live` clears it).
* `align_failed`: the clicked pose is too far from the truth; click closer with the right heading.
* NDT `degraded`/`lost`: raise `voxel_size` (0.5) or lower `ndt_rate_hz` (5), check CPU with `top`.
* AUTO button greyed: needs a route + converged localisation (+ `--drive` and a running obstacle detector in live).
* Logs of the calibration helper: `data/calibration_base.log`.
