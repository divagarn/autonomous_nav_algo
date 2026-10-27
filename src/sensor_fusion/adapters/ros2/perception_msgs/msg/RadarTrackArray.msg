# radar_tracker output: the tracks aeb_process_frame_mois() exports (confirmed, poe >= 0.6),
# ghosts and ground reflections flagged, not removed (as in the AEB pipeline).
std_msgs/Header header
float32 ego_speed               # m/s used for this frame
float32 dt                      # s used for this frame
uint32 frame_number
int32 mio_id                    # -1: no MIO
RadarTrack[] tracks
