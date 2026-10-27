#pragma once

#include <cstdint>
#include <vector>

#include "aeb_top5.h"
#include "datatype/radar_point.hpp"

namespace radar_tracker
{

/// All tracks returned by EkfTracker::update() in the last aeb_process_frame_mois() call.
const std::vector<RadarTrack> & aebLastTracks();
/// MIO (selectMio_simple) of the last frame, -1 if none.
int64_t aebMioId();

}  // namespace radar_tracker
