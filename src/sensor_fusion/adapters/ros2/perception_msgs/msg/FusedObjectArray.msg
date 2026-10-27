std_msgs/Header header   # frame_id of Cartesian fields/path, stamp = time of the cycle
string trigger           # sensor that triggered this cycle: "radar" | "camera"
string radar_tracker     # radar tracking of all objects: "ekf" (EKF tracker node) | "imm" (fusion's IMM)
string radar_frame_id    # frame of the *_radar fields of each object (the radar, e.g. umrr)
FusedObject[] objects
# predicted ego path in header.frame_id, at ground level, used for in_path
float32 path_curvature   # [1/m], + left
string path_source       # steering | odom
geometry_msgs/Point[] path_left
geometry_msgs/Point[] path_right
# stop zone: corridor from the radar to stop_zone_length ahead (closed outline, header.frame_id)
float32 stop_zone_length # [m] = stop_zone.min_distance + ego speed * stop_zone.time_headway
geometry_msgs/Point[] stop_zone
# lane from the camera segmentation (ground plane, every 1 m ahead of the vehicle); empty = not seen.
# lane_*: lane markings left / right of the vehicle; road_*: edges of the road surface around it
float32 lane_age         # [s] stamp - stamp of the segmentation the lane comes from (-1: no lane)
geometry_msgs/Point[] lane_left       # header.frame_id
geometry_msgs/Point[] lane_right
geometry_msgs/Point[] road_left
geometry_msgs/Point[] road_right
# tracked driving path (lane.path.*): lane.path.points points every lane.path.spacing from base_link, kept
# lane.path's distance from the nearer road edge, curvature within the steering; lane borders around it
geometry_msgs/Point[] lane_path
geometry_msgs/Point[] lane_path_left    # lane border: path + (vehicle width / 2 + lane margin) to the left
geometry_msgs/Point[] lane_path_right
float32 lane_path_unmeasured   # [m] driven since road edges last corrected the path (large: path is a guess)
float32 road_width       # [m] median width between the road edges in the last mask (-1: not seen)
string base_frame_id     # frame of the *_base fields (vehicle, e.g. base_link)
geometry_msgs/Pose radar_pose_base   # radar origin in base_frame_id (draws *_base in the camera image)
geometry_msgs/Point[] lane_left_base
geometry_msgs/Point[] lane_right_base
geometry_msgs/Point[] road_left_base
geometry_msgs/Point[] road_right_base
geometry_msgs/Point[] lane_path_base
geometry_msgs/Point[] lane_path_left_base
geometry_msgs/Point[] lane_path_right_base
