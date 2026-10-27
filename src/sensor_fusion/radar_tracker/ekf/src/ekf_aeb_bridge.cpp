/**
 * @file ekf_aeb_bridge.cpp
 * @brief Compiles the EKF tracker's aeb_wrapper.cpp (aeb/, byte-identical copy) and adds one accessor.
 *
 * aeb_process_frame_mois() keeps the full EkfTracker output in a file-static vector
 * (g_last_tracks) and exports only position/|v| as RadarData. Including the .cpp here puts
 * this accessor in the same translation unit, so the ROS node can publish the complete
 * RadarTrack state (velocity, covariance, ghost flags) of exactly the frame just processed,
 * without changing a line of the original file. Do not also compile aeb_wrapper.cpp separately.
 */
#include "../aeb/aeb_wrapper.cpp"

#include "radar_tracker/ekf_aeb_bridge.hpp"

namespace radar_tracker
{

const std::vector<RadarTrack> & aebLastTracks()
{
  return g_last_tracks;
}

int64_t aebMioId()
{
  // selectMio_simple() clears candidates_ on every call and fills candidates_[0] only when it
  // returns a MIO; it is only called when the tracker returned tracks (stale otherwise).
  if (!g_mio_selector || g_last_tracks.empty()) {return -1;}
  const auto & c = g_mio_selector->getCandidates();
  return c.empty() ? -1 : static_cast<int64_t>(c[0].track_id);
}

}  // namespace radar_tracker
