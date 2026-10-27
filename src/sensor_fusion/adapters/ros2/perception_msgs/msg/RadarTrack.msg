# One radar track from radar_tracker (the EKF tracker), radar sensor frame:
# x forward, y left (header.frame_id of the RadarTrackArray).
uint32 id
float32 px
float32 py
float32 pz                      # radar frame (mount_z removed again)
float32 vx                      # EKF velocity [m/s] (EkfTracker state x(2), x(3))
float32 vy
float32 yaw                     # atan2(vy, vx) [rad]
float32 length                  # smoothed extent along x [m]
float32 width                   # smoothed extent along y [m]
float32 poe                     # probability of existence [0, 1]
int32 hits
int32 age                       # frames
int32 missed_frames
bool confirmed
bool is_dynamic                 # |v| > dynamic_speed_thresh (EkfTracker)
bool is_mirror_ghost            # MirrorDetector: N-fold range/velocity ghost
bool is_stationary_reflector    # MirrorDetector: anchors >= 1 ghost
bool is_ground_reflection       # low, weak return (excluded from MIO)
bool is_mio                     # MioSelector::selectMio_simple pick (AEB target)
float32 radial_speed            # last mean radial speed [m/s]
float32 avg_speed               # mean ego-compensated radial speed [m/s]
float32 avg_rcs
float32 avg_snr
int32 n_pts                     # points associated this frame
float32[16] covariance          # P of [px, py, vx, vy], row-major
