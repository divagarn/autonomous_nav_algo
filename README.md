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
| `./buggy stop` | Clean Ctrl+C to the running launch. |
| `./buggy ros2 ...` | Any ros2 command in this environment, e.g. `./buggy ros2 topic echo /localization/state`. |

`MAP` is a folder name in `data/maps` (default `new_delhi_map`) or a full path.
Extra `key:=value` arguments go straight to `ros2 launch`:
`voxel_size:=0.5  ndt_rate_hz:=5.0  corridor_half_width:=1.0  feedback_port:=/dev/ttyUSB1
sender_port:=/dev/ttyUSB1  bag_start_delay:=12.0  bag_rate:=0.35  can_interface:=can0
detector_model:=<onnx>  camera_device:=/dev/video0  labeler_debug:=true`.
Options: `--no-rviz`, `--no-detector`, `--radar`, `--camera`.

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

### Going around a blocked lane (detour / obstacle avoidance)
`--avoid` (default **on** in `view`/`sim`, **off** in `live`/`replay`; `--no-avoid` turns it off).
If an obstacle has blocked the route for **5 s** (`avoid_after_sec`), `operation_manager` asks the
`avoidance_planner` for a detour. The planner builds a smooth lateral S-curve off the lane centre line
(arc radius from the steering limit, forward driving only - the ECU has no reverse), past the obstacle
with 0.35 m body clearance, and back onto the **same route**, so the buggy carries on along its lanelet path.
A detour is accepted only if the whole swept body footprint is clear of every known obstacle, of solid LiDAR
cells, and inside the drivable area: the lane, or up to 0.8 m outside it (2.0 m where the LiDAR has actually
seen free ground; without LiDAR data, e.g. in `view`, only 0.8 m is assumed). The detour is drawn in cyan
(display *Planning / Detour*), the buggy creeps at the 2 km/h preset while on it, and the normal
slow/stop logic still applies to anything else near it.
If there is no room it stays stopped, says why in the panel (`Blocked, no detour: ...`) and retries every 5 s (6 tries).
Limits to know:
* it needs about **10 m** in front of the obstacle (a 1.4 m shift needs ~9.5 m of arc plus 1 m lead), so the
  5 s timer starts when the obstacle is first seen, not when the buggy has already stopped at 3 m;
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
