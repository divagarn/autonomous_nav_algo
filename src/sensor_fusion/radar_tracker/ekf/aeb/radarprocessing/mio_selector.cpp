/**
 * @file mio_selector.cpp
 * @brief Implementation of production-grade MIO selection logic
 */

#include "radarprocessing/mio_selector.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>

float MioSelector::calculateTTC(float px, float py, float vx, float vy, float ego_speed) const {
    // Relative velocity in vehicle frame
    const float rel_vx = vx - ego_speed;
    //const float rel_vy = vy;
    
    // Closing speed (negative rel_vx = approaching)
    const float closing_speed = -rel_vx;
    
    if (closing_speed <= 0.01f) {
        return std::numeric_limits<float>::infinity();
    }
    
    // Simple TTC: time when longitudinal distance becomes zero
    // For more accuracy, consider using trajectory intersection
    const float ttc = px / closing_speed;
    
    // Reject negative or unreasonably large TTC
    if (ttc < 0.0f || ttc > config_.max_ttc_threshold) {
        return std::numeric_limits<float>::infinity();
    }
    
    return ttc;
}

float MioSelector::calculateTHW(float range, float ego_speed) const {
    if (ego_speed < 0.5f) {
        return std::numeric_limits<float>::infinity();
    }
    return range / ego_speed;
}

float MioSelector::calculatePathOverlap(float py, float vy, float ego_speed, float prediction_time) const {
    // Predict lateral position at collision point
    const float predicted_lateral = py + vy * prediction_time;
    
    // Calculate ego path boundaries (with safety margin)
    const float ego_half_width = (config_.ego_width / 2.0f) + config_.lateral_margin;
    
    // Object assumed to have some width (conservative: 2m for typical vehicle)
    const float obj_half_width = 1.0f;
    
    // Check overlap
    const float ego_left = -ego_half_width;
    const float ego_right = ego_half_width;
    const float obj_left = predicted_lateral - obj_half_width;
    const float obj_right = predicted_lateral + obj_half_width;
    
    // Calculate overlap amount
    const float overlap_left = std::max(ego_left, obj_left);
    const float overlap_right = std::min(ego_right, obj_right);
    const float overlap_width = std::max(0.0f, overlap_right - overlap_left);
    
    // Normalize to [0..1]
    const float max_overlap = 2.0f * ego_half_width;
    const float overlap_ratio = overlap_width / max_overlap;
    
    return std::min(1.0f, overlap_ratio);
}

float MioSelector::calculateThreatScore(const MioCandidate& candidate, float ego_speed) const {
    // Range-only mode: let quality gates do all ghost filtering; score simply
    // picks the closest surviving candidate.  Deterministic, no poe volatility.
    if (config_.use_range_only_score) {
        return (1.0f - std::min(1.0f, candidate.range / config_.max_range)) * 100.0f;
    }

    float score = 0.0f;

    // 1. TTC component (40%) - Lower TTC = Higher threat
    if (candidate.ttc < config_.max_ttc_threshold && candidate.ttc >= config_.min_ttc_threshold) {
        // Inverse relationship: TTC of 0.5s = max score, 8s = min score
        const float ttc_normalized = 1.0f - ((candidate.ttc - config_.min_ttc_threshold) / 
                                             (config_.max_ttc_threshold - config_.min_ttc_threshold));
        score += config_.weight_ttc * std::max(0.0f, ttc_normalized);
    }
    
    // 2. Path overlap component (30%) - Higher overlap = Higher threat
    score += config_.weight_path_overlap * candidate.path_overlap;
    
    // 3. Distance component (20%) - Closer = Higher threat
    const float range_normalized = 1.0f - std::min(1.0f, candidate.range / config_.max_range);
    score += config_.weight_distance * range_normalized;
    
    // 4. Closing speed component (10%)
    const float track_speed = std::sqrt(candidate.vx * candidate.vx + candidate.vy * candidate.vy);
    const float rel_speed = ego_speed - track_speed;
    if (rel_speed > 0.5f) {
        const float speed_normalized = std::min(1.0f, std::max(0.0f, rel_speed / 20.0f));
        score += config_.weight_closing_speed * speed_normalized;
    }
    
    // Confidence scaling - critical for reducing false positives
    score *= candidate.poe;
    
    // Age bonus (stable tracks preferred) - reduces MIO flickering
    if (candidate.age > 10) {
        score *= 1.05f; // 5% bonus for mature tracks
    }
    
    return std::min(100.0f, score);
}

bool MioSelector::meetsQualityThreshold(const RadarTrack& track) const {
   return track.confirmed 
        && track.poe >= config_.min_poe
        && static_cast<uint32_t>(track.age) >= config_.min_age
        && static_cast<uint32_t>(track.hits) >= config_.min_hits;
}

bool MioSelector::shouldSwitchMio(const MioCandidate& new_candidate) const {
    if (!state_.is_valid) {
        return true; // No current MIO, always switch
    }
    
    // Enforce minimum hold time - prevents rapid oscillation
    if (state_.frames_held < config_.min_hold_frames) {
        return false;
    }
    
    // Check if new candidate is significantly more threatening
    const float threat_delta = new_candidate.threat_score - state_.last_threat_score;
    
    // Apply hysteresis: new candidate must be significantly better
    // This is the key to stable MIO selection
    const float required_delta = config_.mio_switch_threshold / config_.switch_hysteresis_factor;
    
    return threat_delta > required_delta;
}

const MioCandidate* MioSelector::selectMio(const std::vector<RadarTrack>& tracks, 
                                            float ego_speed, 
                                            float dt) {
    candidates_.clear();
    
    // Build candidate list with comprehensive filtering and threat assessment
    for (const RadarTrack& track : tracks) {
        // Quality gate 1: Track maturity and confidence
        if (!meetsQualityThreshold(track)) {
            continue;
        }
        
        // Quality gate 2: Range filtering
        const float range = std::sqrt(track.px * track.px + track.py * track.py);
        if (range > config_.max_range || range < 1.0f) {
            continue;
        }
        
        // Quality gate 3: Must be in front (positive px)
        if (track.px < 0.5f) {
            continue;
        }
        
        // ═══════════════════════════════════════════════════════════════
        // CRITICAL FILTER 1: Hard lateral boundary (current position)
        // Only consider tracks within truck lateral range RIGHT NOW
        // ═══════════════════════════════════════════════════════════════
        if (config_.use_hard_lateral_bounds) {
            if (track.py < config_.lateral_min || track.py > config_.lateral_max) {
                continue;
            }
        }
        if (track.is_stationary_reflector) {
            continue;
        }
        
        // Build candidate structure
        MioCandidate candidate;
        candidate.track_id = track.id;
        candidate.px = track.px;
        candidate.py = track.py;
        candidate.vx = track.vx;
        candidate.vy = track.vy;
        candidate.range = range;
        candidate.poe = track.poe;
        candidate.is_dynamic = (std::sqrt(track.vx * track.vx + track.vy * track.vy) > 0.5f);
        candidate.age = track.age;
        candidate.hits = track.hits;
        
        // Calculate TTC using constant velocity model
        candidate.ttc = calculateTTC(track.px, track.py, track.vx, track.vy, ego_speed);
        candidate.thw = calculateTHW(range, ego_speed);
        
        // ═══════════════════════════════════════════════════════════════
        // CRITICAL FILTER 2: Predicted lateral position check
        // Ensure object will STAY in lane (not drift out)
        // ═══════════════════════════════════════════════════════════════
        if (config_.use_predicted_lateral && candidate.ttc < config_.max_ttc_threshold) {
            const float prediction_time = std::min(candidate.ttc, 3.0f); // Cap at 3s
            const float predicted_py = track.py + track.vy * prediction_time;
            
            // Add margin for predicted position (slightly more lenient)
            const float margin = config_.predicted_lateral_margin;
            if (predicted_py < (config_.lateral_min - margin) || 
                predicted_py > (config_.lateral_max + margin)) {
                continue;  // Object will drift out of lane
            }
        }
        
        // Calculate path overlap at predicted collision point
        const float prediction_time = std::min(candidate.ttc, 3.0f);
        candidate.path_overlap = calculatePathOverlap(track.py, track.vy, ego_speed, prediction_time);
        
        // Quality gate 4: Skip if no path overlap (triple safety check)
        if (candidate.path_overlap < 0.1f) {
            continue;
        }
        
        // Calculate comprehensive threat score
        candidate.threat_score = calculateThreatScore(candidate, ego_speed);
        
        // Quality gate 5: Only consider significant threats
        if (candidate.threat_score < 10.0f) {
            continue;
        }
        
        candidates_.push_back(candidate);
    }
    
    // Sort by threat score (descending) - most threatening first
    std::sort(candidates_.begin(), candidates_.end(),
              [](const MioCandidate& a, const MioCandidate& b) {
                  return a.threat_score > b.threat_score;
              });
    
    if (candidates_.empty()) {
        // No valid MIO found - reset state
        state_.is_valid = false;
        state_.current_mio_id = 0;
        state_.frames_held = 0;
        return nullptr;
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // Apply hysteresis logic to prevent MIO oscillation
    // This is the KEY to stable MIO selection
    // ═══════════════════════════════════════════════════════════════════
    const MioCandidate* selected_mio = nullptr;
    
    if (state_.is_valid) {
        // Check if current MIO still exists in candidates
        auto current_it = std::find_if(candidates_.begin(), candidates_.end(),
                                       [this](const MioCandidate& c) {
                                           return c.track_id == state_.current_mio_id;
                                       });
        
        if (current_it != candidates_.end()) {
            // Current MIO still valid
            const MioCandidate& current_mio = *current_it;
            
            // Check if we should switch to a new MIO
            if (candidates_[0].track_id != state_.current_mio_id && 
                shouldSwitchMio(candidates_[0])) {
                // Switch to new MIO (must be significantly better)
                selected_mio = &candidates_[0];
                state_.current_mio_id = selected_mio->track_id;
                state_.frames_held = 0;
            } else {
                // Keep current MIO (hysteresis in action)
                selected_mio = &current_mio;
                state_.frames_held++;
            }
        } else {
            // Current MIO lost (track deleted), switch to best candidate
            selected_mio = &candidates_[0];
            state_.current_mio_id = selected_mio->track_id;
            state_.frames_held = 0;
        }
    } else {
        // No previous MIO, select best candidate
        selected_mio = &candidates_[0];
        state_.current_mio_id = selected_mio->track_id;
        state_.frames_held = 0;
    }
    
    state_.is_valid = true;
    state_.last_threat_score = selected_mio->threat_score;

    return selected_mio;
}

// ─────────────────────────────────────────────────────────────────────────────
// FIX 2: shouldSwitchMio_fixed
//
// Original bug: shouldSwitchMio() reads state_.last_threat_score, which is the
// selected MIO's score from the PREVIOUS frame.  If the current MIO's score
// changes this frame (poe fluctuation, partial lane exit) the hysteresis delta
// is computed against a stale baseline — making it either too sticky or too easy
// to switch.
//
// Fix: accept the current MIO candidate (recalculated this frame) and diff
// directly against its live threat_score.
// ─────────────────────────────────────────────────────────────────────────────
bool MioSelector::shouldSwitchMio_fixed(const MioCandidate& current_mio,
                                         const MioCandidate& new_candidate) const {
    if (!state_.is_valid) {
        return true;
    }

    if (state_.frames_held < config_.min_hold_frames) {
        return false;
    }

    // Live delta: both scores recalculated in the current frame
    const float threat_delta = new_candidate.threat_score - current_mio.threat_score;
    const float required_delta = config_.mio_switch_threshold / config_.switch_hysteresis_factor;

    return threat_delta > required_delta;
}

// ─────────────────────────────────────────────────────────────────────────────
// selectMio_simple
//
// Mirrors the known-working pipeline:
//   confirmed only → |py| <= lateral_max → closest range = MIO
//
// No scoring, no hysteresis, no poe/age/hits gates beyond confirmed.
// This is the baseline that is known to work.
// ─────────────────────────────────────────────────────────────────────────────
const MioCandidate* MioSelector::selectMio_simple(const std::vector<RadarTrack>& tracks) {
    candidates_.clear();

    float      min_range    = std::numeric_limits<float>::infinity();
    const MioCandidate* mio = nullptr;

    // Debug: print every 30 frames so we can diagnose what the tracker is seeing
    static int dbg_frame = 0;
    const bool dbg_print = 0;//((dbg_frame++ % 30) == 0);

    if (dbg_print)
        printf("[MIO_DBG] --- frame %d | %zu tracks total ---\n", dbg_frame, tracks.size());

    for (const RadarTrack& track : tracks) {
        const float range = std::hypot(track.px, track.py);

        if (dbg_print)
            printf("[MIO_DBG]  id=%-4u px=%5.1f py=%5.1f rng=%5.1f poe=%.2f age=%-3u hits=%-3u "
                   "conf=%d ghost=%d refl=%d\n",
                   track.id, track.px, track.py, range, track.poe,
                   track.age, track.hits,
                   (int)track.confirmed, (int)track.is_mirror_ghost,
                   (int)track.is_stationary_reflector);

        if (!track.confirmed)
            continue;

        // Skip mirror ghosts — they are false detections reflected off structures.
        // Do NOT skip is_stationary_reflector: stationary real obstacles (stopped
        // vehicles, AEB test barriers) are also stationary and must remain as MIO.
        if (track.is_mirror_ghost )
            continue;

        // Ground/overdrivable height classification (see
        // EkfTrackerConfig::ground_z_threshold's doc comment) -- excluded
        // from MIO only, not from tracking: the track keeps updating and
        // stays visible everywhere else, it just can't be picked as the
        // single most-important object.
        if (track.is_ground_reflection)
            continue;

        if (std::fabs(track.py) > config_.lateral_max)
            continue;

        if (range < min_range) {
            min_range = range;

            MioCandidate c{};
            c.track_id   = track.id;
            c.px         = track.px;
            c.py         = track.py;
            c.vx         = track.vx;
            c.vy         = track.vy;
            c.range      = range;
            c.poe        = track.poe;
            c.is_dynamic = (std::hypot(track.vx, track.vy) > 0.5f);
            c.age        = track.age;
            c.hits       = track.hits;
            c.ttc        = std::numeric_limits<float>::infinity();
            c.thw        = std::numeric_limits<float>::infinity();
            c.path_overlap  = 0.0f;
            c.threat_score  = (1.0f - std::min(1.0f, range / config_.max_range)) * 100.0f;

            if (candidates_.empty())
                candidates_.push_back(c);
            else
                candidates_[0] = c;

            mio = &candidates_[0];
        }
    }

    // ── Hold logic ────────────────────────────────────────────────────────────
    // At close range radar measurements are noisy: a barrier track can briefly
    // lose confirmation for 1–3 frames while a farther stable track remains
    // confirmed.  The hold prevents the MIO jumping to the farther track.
    //
    // If the previous MIO track is still in the track list (even if temporarily
    // unconfirmed) AND is CLOSER than the current confirmed best, prefer the
    // held track and consume one hold frame.
    {
        // Look up the held track (may be unconfirmed)
        const RadarTrack* held_track = nullptr;
        if (simple_hold_remaining_ > 0 && simple_last_mio_id_ != 0) {
            for (const RadarTrack& t : tracks) {
                if (t.id != simple_last_mio_id_)
                    continue;
                if (t.is_mirror_ghost || std::fabs(t.py) > config_.lateral_max
                    || t.is_ground_reflection)
                    break;
                held_track = &t;
                break;
            }
        }

        const float held_range = held_track
            ? std::hypot(held_track->px, held_track->py)
            : std::numeric_limits<float>::infinity();
        const float best_range = mio
            ? mio->range
            : std::numeric_limits<float>::infinity();

        if (held_track != nullptr && held_range < best_range) {
            // Held track is closer — use it instead of the confirmed best
            simple_held_cand_.track_id   = held_track->id;
            simple_held_cand_.px         = held_track->px;
            simple_held_cand_.py         = held_track->py;
            simple_held_cand_.vx         = held_track->vx;
            simple_held_cand_.vy         = held_track->vy;
            simple_held_cand_.range      = held_range;
            simple_held_cand_.poe        = held_track->poe;
            simple_held_cand_.is_dynamic = (std::hypot(held_track->vx, held_track->vy) > 0.5f);
            simple_held_cand_.age        = held_track->age;
            simple_held_cand_.hits       = held_track->hits;
            simple_held_cand_.threat_score =
                (1.0f - std::min(held_range / config_.max_range, 1.0f)) * 100.0f;
            if (candidates_.empty())
                candidates_.push_back(simple_held_cand_);
            else
                candidates_[0] = simple_held_cand_;
            mio = &candidates_[0];
            --simple_hold_remaining_;
            if (dbg_print)
                printf("[MIO_DBG]  => HELD id=%-4u range=%.1fm (%u hold frames left)\n",
                       mio->track_id, mio->range, simple_hold_remaining_);
        } else if (mio) {
            // Confirmed best accepted — reset hold counter
            simple_last_mio_id_    = mio->track_id;
            simple_hold_remaining_ = SIMPLE_HOLD_FRAMES;
            simple_held_cand_      = candidates_[0];
        } else {
            // No MIO at all — drain hold counter
            if (simple_hold_remaining_ > 0)
                --simple_hold_remaining_;
            else
                simple_last_mio_id_ = 0;
        }
    }

    if (dbg_print) {
        if (mio)
            printf("[MIO_DBG]  => MIO id=%-4u range=%.1fm py=%.1fm\n",
                   mio->track_id, mio->range, mio->py);
        else
            printf("[MIO_DBG]  => MIO = NULL (no qualifying track)\n");
    }

    return mio;
}

// ─────────────────────────────────────────────────────────────────────────────
// FIX 1 + FIX 2 + FIX 3 combined: selectMio_fixed
//
// Fix 1 (low-speed ghost toggle): predicted lateral check was gated on
//   "candidate.ttc < max_ttc_threshold".  At low/zero ego speed ALL objects get
//   TTC = infinity, so the entire lateral-drift filter was silently bypassed.
//   Now the check always runs.  When TTC is infinite a fixed 2-second horizon
//   is used instead so laterally drifting objects are still caught.
//
// Fix 2 (stale hysteresis): delegates to shouldSwitchMio_fixed() which diffs
//   against the current MIO's live score instead of the cached previous-frame
//   value.
//
// Fix 3 (low-speed quality hardening):
//   At low/zero ego speed TTC=inf and closing_speed=0 for everything, so the
//   threat score collapses to pure (range x poe) — the nearest confirmed track
//   always wins, ghost or not.  Three sub-fixes applied only in low-speed mode:
//     3a. Bumper exclusion: reject px < config_.min_px (default 2.5 m).
//         Blocks self-reflections and under-bumper clutter which score highest
//         simply because they are closest to the radar.
//     3b. POE floor: require poe >= low_speed_min_poe (default 0.75) instead of
//         the normal min_poe.  Ghosts have low, fluctuating POE; raising the bar
//         filters most of them without affecting well-confirmed real targets.
//     3c. Age floor: require age >= 2 x min_age so only tracks that have been
//         consistently observed across many frames pass through.
// ─────────────────────────────────────────────────────────────────────────────
const MioCandidate* MioSelector::selectMio_fixed(const std::vector<RadarTrack>& tracks,
                                                   float ego_speed,
                                                   float dt) {
    candidates_.clear();

    // FIX 3: derive tighter thresholds when ego is slow / stationary
    const bool     low_speed_mode    = (ego_speed < config_.low_speed_threshold);
    const float    effective_min_poe = low_speed_mode ? config_.low_speed_min_poe
                                                      : config_.min_poe;
    const uint32_t effective_min_age = low_speed_mode ? (config_.min_age * 2u)
                                                      : config_.min_age;

    for (const RadarTrack& track : tracks) {
        if (!meetsQualityThreshold(track)) {
            continue;
        }

        // FIX 3b: tighter POE floor in low-speed mode
        if (track.poe < effective_min_poe) {
            continue;
        }

        // FIX 3c: tighter age floor in low-speed mode
        if (static_cast<uint32_t>(track.age) < effective_min_age) {
            continue;
        }

        const float range = std::sqrt(track.px * track.px + track.py * track.py);
        if (range > config_.max_range || range < 1.0f) {
            continue;
        }

        // FIX 3a: bumper exclusion zone — blocks clutter that scores high
        // purely from being the closest object to the radar
        if (track.px < config_.min_px) {
            continue;
        }

        if (config_.use_hard_lateral_bounds) {
            if (track.py < config_.lateral_min || track.py > config_.lateral_max) {
                continue;
            }
        }

        if (track.is_stationary_reflector || track.is_mirror_ghost) {
            continue;
        }

        MioCandidate candidate;
        candidate.track_id = track.id;
        candidate.px       = track.px;
        candidate.py       = track.py;
        candidate.vx       = track.vx;
        candidate.vy       = track.vy;
        candidate.range    = range;
        candidate.poe      = track.poe;
        candidate.is_dynamic = (std::sqrt(track.vx * track.vx + track.vy * track.vy) > 0.5f);
        candidate.age      = track.age;
        candidate.hits     = track.hits;

        candidate.ttc = calculateTTC(track.px, track.py, track.vx, track.vy, ego_speed);
        candidate.thw = calculateTHW(range, ego_speed);

        // FIX 1: always run the predicted lateral check.
        // Original: gated on (candidate.ttc < max_ttc_threshold) — silently skipped
        //           for every object when ego is slow/stationary (all TTC = inf).
        if (config_.use_predicted_lateral) {
            const float pred_time = (candidate.ttc < config_.max_ttc_threshold)
                                     ? std::min(candidate.ttc, 3.0f)
                                     : 2.0f;  // fixed 2s horizon when TTC = infinity
            const float predicted_py = track.py + track.vy * pred_time;
            const float margin = config_.predicted_lateral_margin;
            if (predicted_py < (config_.lateral_min - margin) ||
                predicted_py > (config_.lateral_max + margin)) {
                continue;
            }
        }

        const float prediction_time = std::min(candidate.ttc, 3.0f);
        candidate.path_overlap = calculatePathOverlap(track.py, track.vy, ego_speed, prediction_time);

        if (candidate.path_overlap < 0.1f) {
            continue;
        }

        candidate.threat_score = calculateThreatScore(candidate, ego_speed);

        if (candidate.threat_score < 10.0f) {
            continue;
        }

        candidates_.push_back(candidate);
    }

    std::sort(candidates_.begin(), candidates_.end(),
              [](const MioCandidate& a, const MioCandidate& b) {
                  return a.threat_score > b.threat_score;
              });

    if (candidates_.empty()) {
        state_.is_valid = false;
        state_.current_mio_id = 0;
        state_.frames_held = 0;
        return nullptr;
    }

    const MioCandidate* selected_mio = nullptr;

    if (state_.is_valid) {
        auto current_it = std::find_if(candidates_.begin(), candidates_.end(),
                                       [this](const MioCandidate& c) {
                                           return c.track_id == state_.current_mio_id;
                                       });

        if (current_it != candidates_.end()) {
            const MioCandidate& current_mio = *current_it;

            // FIX 2: pass current_mio so hysteresis diffs against live score
            if (candidates_[0].track_id != state_.current_mio_id &&
                shouldSwitchMio_fixed(current_mio, candidates_[0])) {

                printf("Switching MIO from track %u (score %.1f) to track %u (score %.1f)\n",
                       current_mio.track_id, current_mio.threat_score,
                       candidates_[0].track_id, candidates_[0].threat_score);
                selected_mio = &candidates_[0];
                state_.current_mio_id = selected_mio->track_id;
                state_.frames_held = 0;
            } else {
                selected_mio = &current_mio;
                state_.frames_held++;
            }
        } else {
            selected_mio = &candidates_[0];
            state_.current_mio_id = selected_mio->track_id;
            state_.frames_held = 0;
        }
    } else {
        selected_mio = &candidates_[0];
        state_.current_mio_id = selected_mio->track_id;
        state_.frames_held = 0;
    }

    state_.is_valid = true;
    state_.last_threat_score = selected_mio->threat_score;

    return selected_mio;
}