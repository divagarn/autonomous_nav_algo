/**
 * @file mirror_detector.cpp
 * @brief N-fold range/velocity mirror ghost detector — implementation.
 *
 * Algorithm per frame
 * ───────────────────
 * Step 1  Reset all is_stationary_reflector flags; preserve is_mirror_ghost
 *         from previous frame (for hysteresis).
 *
 * Step 2  Build reflector candidate list:
 *           confirmed track  AND
 *           |ego-frame radial speed| ≤ stationary_speed_thresh  AND
 *           |py| ≤ lane_half_width_m  AND
 *           hits ≥ min_reflector_hits
 *
 * Step 3  For every non-reflector track G and every reflector R, test each
 *         fold order N ∈ {2,3,4,5}:
 *
 *           (a) Range fold
 *               ref_range  = √(R.px²+R.py²)
 *               ghost_range= √(G.px²+G.py²)
 *               match if |ghost_range − N·ref_range| ≤ max(range_rel_tol·N·ref_range,
 *                                                           range_abs_tol)
 *
 *           (b) Lateral alignment
 *               match if |G.py − R.py| ≤ lateral_tol_m
 *               (same azimuth column, mirrors cannot appear off-axis)
 *
 *           (c) Velocity fold  (skipped when |ego_speed| < ego_stationary_thresh)
 *               ref_vrad   = ego-compensated radial speed of R
 *               ghost_vrad = ego-compensated radial speed of G
 *               match if |ghost_vrad − N·ref_vrad| ≤ speed_abs_tol
 *               NOTE: ref_vrad ≈ 0 when ego is stationary, so this criterion
 *               would always pass — we skip it to avoid false positives from
 *               unrelated dynamic objects.
 *
 * Step 4  On geometry match: mark G as mirror ghost (mirror_miss_frames = 0).
 *         On geometry mismatch for an already-flagged mirror: increment
 *         mirror_miss_frames; clear flag only when > mirror_forget_frames.
 *
 * Step 5  Mark every reflector that has at least one live ghost.
 */

#include "radarprocessing/mirror_detector.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace radarprocessing
{

// ─────────────────────────────────────────────────────────────────────────────
// Static helpers
// ─────────────────────────────────────────────────────────────────────────────

float MirrorDetector::trackRange(const RadarTrack &t)
{
    return std::hypot(t.px, t.py);
}
// ─────────────────────────────────────────────────────────────────────────────
// Reflector candidate test
// ─────────────────────────────────────────────────────────────────────────────
bool MirrorDetector::isReflectorCandidate(const RadarTrack &t, float ego_speed) const
{
    // Must be a confirmed, sufficiently-old track
    if (!t.confirmed || t.hits < cfg_.min_reflector_hits)
        return false;

    // Must be approximately stationary in the world frame
    // float vrad_world = egoFrameRadialSpeed(t, ego_speed);
    // if (std::fabs(vrad_world) > cfg_.stationary_speed_thresh)
    //     return false;
    float abs_speed = std::sqrt(t.vabsx * t.vabsx + t.vabsy * t.vabsy);
    if (abs_speed > cfg_.stationary_speed_thresh)
        return false;

    // Must be laterally close to the ego lane (guard-rail, bridge, etc.)
    if (std::fabs(t.py) > cfg_.lane_half_width_m)
        return false;

    // Must not already be classified as a ghost itself (avoid chaining)
    if (t.is_mirror_ghost)
        return false;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// N-fold geometry test
// ─────────────────────────────────────────────────────────────────────────────
bool MirrorDetector::isNFoldMirror(const RadarTrack &ref,
                                    const RadarTrack &ghost,
                                    int N,
                                    float ego_speed) const
{
    const float Nf = static_cast<float>(N);

    // ── (a) Range fold ────────────────────────────────────────────────────────
    float ref_range   = trackRange(ref);
    float ghost_range = trackRange(ghost);

    // Ghost should be at N × ref_range
    float min_expected_range = (Nf * ref_range) - 2.5;
    float max_expected_range = (Nf * ref_range) + 2.5;
    
    if (ghost_range < min_expected_range || ghost_range > max_expected_range)
        return false;

    // ── (b) Lateral alignment ─────────────────────────────────────────────────
    // Mirrors cannot appear at a different azimuth than the reflector; their
    // lateral displacement grows only slightly with range (same azimuth column).
    // We use a generous absolute tolerance to handle azimuth estimation error.
    if (std::fabs(ghost.py - ref.py) > cfg_.lateral_tol_m)
        return false;

    // ── (c) Velocity fold
    float ref_vrad   = ref.avg_speed;
    float ghost_vrad = ghost.avg_speed;
    float expected_vrad = Nf * ref_vrad;
    if (std::fabs(ghost_vrad - expected_vrad) > cfg_.speed_abs_tol)
    {
        return false;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main classify pass
// ─────────────────────────────────────────────────────────────────────────────
void MirrorDetector::classify(std::vector<RadarTrack> &tracks, float ego_speed)
{
    // ── Step 1: Reset reflector flag; preserve ghost flag for hysteresis ──────
    for (auto &t : tracks)
        t.is_stationary_reflector = false;

    const bool ego_stationary =
    std::fabs(ego_speed) < cfg_.ego_stationary_thresh;

    if (ego_stationary)
    {
        // Freeze already-classified mirrors.
        // Do not create new mirrors.
        // Do not decay existing mirrors.

        return;
    }

    // ── Step 2: Collect reflector candidates ──────────────────────────────────
    // Store indices into `tracks` so we can mark them later.
    std::vector<int> reflector_idxs;
    float best_range = std::numeric_limits<float>::max();
    int best_idx = -1;

    for (int i = 0; i < static_cast<int>(tracks.size()); ++i)
    {
        if (!isReflectorCandidate(tracks[i], ego_speed))
            continue;

        float r = trackRange(tracks[i]);

        // choose nearest reflector
        if (r < best_range)
        {
            best_range = r;
            best_idx = i;
        }
    }

    if (best_idx >= 0)
        reflector_idxs.push_back(best_idx);


    if (reflector_idxs.empty())
    {
        // No reflectors found: decay all existing mirror flags
        for (auto &t : tracks)
        {
            if (t.is_mirror_ghost)
            {
                ++t.mirror_miss_frames;
                if (t.mirror_miss_frames > cfg_.mirror_forget_frames)
                {
                    t.is_mirror_ghost  = false;
                    t.mirror_source_id = 0;
                    t.mirror_order     = 0;
                }
            }
        }
        return;
    }

    // ── Step 3: Match each track against reflectors ───────────────────────────
    // We track which reflectors generated at least one ghost (for Step 5).
    std::vector<bool> reflector_has_ghost(reflector_idxs.size(), false);

    for (int gi = 0; gi < static_cast<int>(tracks.size()); ++gi)
    {
        auto &ghost = tracks[gi];

        // A reflector cannot be its own ghost; skip reflector candidates
        bool is_reflector = false;
        for (int ri : reflector_idxs)
            if (ri == gi) { is_reflector = true; break; }
        if (is_reflector) continue;

        bool matched = false;

        for (int rk = 0; rk < static_cast<int>(reflector_idxs.size()); ++rk)
        {
            int ri = reflector_idxs[rk];
            const auto &ref = tracks[ri];

            for (int N : cfg_.fold_orders)
            {
                if (isNFoldMirror(ref, ghost, N, ego_speed))
                {
                    // ── Geometry match ───────────────────────────────────────
                    ghost.is_mirror_ghost    = true;
                    ghost.mirror_source_id   = ref.id;
                    ghost.mirror_order       = N;
                    ghost.mirror_miss_frames = 0;
                    reflector_has_ghost[rk]  = true;
                    matched = true;
                    break;   // use first (lowest-order) match
                }
            }
            if (matched) break;   // assigned to first reflector that matches
        }

        // ── Step 4: Hysteresis for unmatched ghosts ───────────────────────────
        if (!matched && ghost.is_mirror_ghost)
        {
            ++ghost.mirror_miss_frames;
            if (ghost.mirror_miss_frames > cfg_.mirror_forget_frames)
            {
                ghost.is_mirror_ghost  = false;
                ghost.mirror_source_id = 0;
                ghost.mirror_order     = 0;
            }
            // While within the forget window, keep the old source/order info so
            // callers can still identify which reflector this ghost is tied to.
        }
    }

    // ── Step 5: Mark reflectors that have active ghost children ───────────────
    for (int rk = 0; rk < static_cast<int>(reflector_idxs.size()); ++rk)
        if (reflector_has_ghost[rk])
            tracks[reflector_idxs[rk]].is_stationary_reflector = true;
}

} // namespace radarprocessing
