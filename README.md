# buggy_unified

One self-contained folder for the buggy's **PCD + Lanelet2 stack**: map loading, NDT/EKF
localisation on the live VLP-16, route planning, LiDAR obstacle detection, AUTO driving over UART,
brake calibration, map import, and a Gazebo closed-loop simulator.

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
│   └── velodyne/                VLP-16 driver + point cloud conversion
└── data/                  maps and calibration (portable; override with BUGGY_DATA_DIR)
    └── maps/{new_delhi_map, office_loop1, office_track_sim}/
```

## 1. Install on a machine

Needs **ROS 2 Humble** (the Autoware packages are the apt ones: `ros-humble-autoware-*`).

```bash
sudo apt install ros-humble-autoware-map-loader ros-humble-autoware-map-projection-loader \
     ros-humble-autoware-ndt-scan-matcher ros-humble-autoware-ekf-localizer \
     ros-humble-autoware-perception-msgs ros-humble-autoware-internal-localization-msgs \
     ros-humble-autoware-internal-debug-msgs ros-humble-rviz2 ros-humble-tf-transformations \
     ros-humble-cv-bridge python3-scipy python3-serial python3-yaml
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
| `./buggy sim [MAP] [gui:=true]` | Gazebo vehicle + emulated ECU, closed loop. Default map `office_track_sim`. |
| `./buggy calibrate-brake [--preset 2\|4] [--live]` | Measures the real stopping distance. Dry run unless `--live`. |
| `./buggy add-map NAME FILE.osm [--pcd FILE.pcd]` | Creates `data/maps/NAME/` from a Vector Map Builder export and checks it. |
| `./buggy sim-map` / `./buggy sim-map --save` | Drives the Gazebo lane and records a PCD (`--save` writes it). |
| `./buggy stop` | Clean Ctrl+C to the running launch. |
| `./buggy ros2 ...` | Any ros2 command in this environment, e.g. `./buggy ros2 topic echo /localization/state`. |

`MAP` is a folder name in `data/maps` (default `new_delhi_map`) or a full path.
Extra `key:=value` arguments go straight to `ros2 launch`:
`voxel_size:=0.5  ndt_rate_hz:=5.0  corridor_half_width:=1.0  feedback_port:=/dev/ttyUSB1
sender_port:=/dev/ttyUSB1  bag_start_delay:=12.0`. Options: `--no-rviz`, `--no-detector`.

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

* Topics missing / `ros2 topic list` empty: a stale daemon — `./buggy` stops it for you; by hand `ros2 daemon stop`.
* No `/velodyne_points`: LiDAR cable/IP (`enp2s0` needs 192.168.1.77/24), or a leftover driver
  holding UDP port 2368 (`./buggy live` clears it).
* `align_failed`: the clicked pose is too far from the truth; click closer with the right heading.
* NDT `degraded`/`lost`: raise `voxel_size` (0.5) or lower `ndt_rate_hz` (5), check CPU with `top`.
* AUTO button greyed: needs a route + converged localisation (+ `--drive` and a running obstacle detector in live).
* Logs of the calibration helper: `data/calibration_base.log`.
