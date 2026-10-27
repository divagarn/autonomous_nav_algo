/**
 * @file mirror_detector.hpp
 * @brief N-fold range/velocity mirror ghost detection for stationary radar targets.
 *
 * Physical background
 * ───────────────────
 * When a strong stationary reflector (guard-rail, bridge, wall) is in the radar's
 * field-of-view, the transmitted pulse can make 2, 3, … N round-trips before being
 * recorded.  The radar time-of-flight logic then reports a "ghost" object at
 *
 *   range_ghost  = N * range_reflector        (N = 2, 3, 4, 5 …)
 *   radial_speed_ghost ≈ N * radial_speed_reflector   (frequency wrapping)
 *
 * Because the reflector is stationary in the world frame, in the ego frame its
 * radial speed equals –ego_speed·cos(bearing).  Ghost objects therefore appear at
 * integer multiples of that range (and wrapped velocity).
 *
 * Ego-stationary persistence
 * ──────────────────────────
 * When ego speed → 0 the ego-compensated radial speed of the reflector → 0,
 * making velocity-based discrimination harder.  The module therefore preserves the
 * mirror classification flag across frames: once a track is marked as a mirror it
 * stays a mirror until its reflector track disappears or the geometry is violated
 * for more than `mirror_forget_frames` consecutive frames.
 *
 * Integration with EkfTracker
 * ───────────────────────────
 * Call MirrorDetector::classify(tracks, ego_speed) once per update() cycle,
 * **after** the tracker has finished its predict/update/spawn pipeline.
 * The method sets RadarTrack::is_mirror_ghost = true on ghost tracks and
 * RadarTrack::is_stationary_reflector = true on the anchor track.
 *
 * Required additions to RadarTrack (datatypes/radar_point.hpp):
 *   bool     is_mirror_ghost        = false;
 *   bool     is_stationary_reflector= false;
 *   uint32_t mirror_source_id       = 0;      ///< track ID of the reflector
 *   int      mirror_order           = 0;       ///< N for which this is the ghost
 *   int      mirror_miss_frames     = 0;       ///< consecutive frames without geometry match
 */

#pragma once

#include "datatype/radar_point.hpp"   // RadarTrack, RadarPoint
#include <vector>
#include <cstdint>

namespace radarprocessing
{

// ─────────────────────────────────────────────────────────────────────────────
// Tuning parameters (all distances in metres, speeds in m/s)
// ─────────────────────────────────────────────────────────────────────────────
struct MirrorDetectorConfig
{
    // ── Reflector qualification ───────────────────────────────────────────────
    /// Max ego-compensated radial speed for a track to be considered stationary [m/s]
    float stationary_speed_thresh   = 1.5f;

    /// Min number of confirmed hits before a track can become a reflector anchor
    int   min_reflector_hits        = 5;

    /// Only look for reflectors within this lateral distance of the ego lane [m]
    /// (|py| ≤ lane_half_width_m)
    float lane_half_width_m         = 4.5f;

    // ── N-fold geometry matching ──────────────────────────────────────────────
    /// Multipliers to test (default: 2×, 3×, 4×, 5×)
    std::vector<int> fold_orders    = {2, 3, 4, 5, 6};

    /// Fractional tolerance on range match: |ghost_range - N*ref_range| / ref_range
    float range_rel_tol             = 0.12f;   ///< 12 %

    /// Absolute fallback tolerance on range match [m]
    float range_abs_tol             = 2.5f;

    /// Absolute tolerance on radial-speed match [m/s]
    float speed_abs_tol             = 1.0f;

    /// Lateral alignment tolerance between ghost and reflector [m]
    /// (they should share roughly the same azimuth / lateral position)
    float lateral_tol_m             = 2.0f;

    // ── Persistence ──────────────────────────────────────────────────────────
    /// Number of consecutive frames a mirror track can fail geometry check
    /// before losing its classification (hysteresis for ego-stationary case)
    int   mirror_forget_frames      = 8;

    /// When |ego_speed| < this, skip velocity matching (use range + lateral only)
    float ego_stationary_thresh     = 0.3f;    ///< [m/s]
};

// ─────────────────────────────────────────────────────────────────────────────
class MirrorDetector
{
public:
    explicit MirrorDetector(const MirrorDetectorConfig &cfg = {}) : cfg_(cfg) {}

    /**
     * @brief Classify mirror ghost tracks in-place.
     *
     * Must be called once per tracker update cycle, after EkfTracker::update()
     * returns the track list.
     *
     * @param tracks     Mutable track list from EkfTracker::update()
     * @param ego_speed  Host vehicle forward speed [m/s]  (signed, +ve = forward)
     */
    void classify(std::vector<RadarTrack> &tracks, float ego_speed);

    void setConfig(const MirrorDetectorConfig &cfg) { cfg_ = cfg; }
    const MirrorDetectorConfig &config() const { return cfg_; }

private:
    MirrorDetectorConfig cfg_;

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// True if a track qualifies as a stationary reflector anchor.
    bool isReflectorCandidate(const RadarTrack &t, float ego_speed) const;

    /**
     * @brief Test whether `ghost` is the N-th order mirror of `ref`.
     *
     * Checks:
     *   1. Range fold:    |ghost_range - N * ref_range| ≤ tol
     *   2. Lateral align: |ghost.py   - ref.py|         ≤ lateral_tol
     *   3. Speed fold:    |ghost_vrad - N * ref_vrad|   ≤ speed_tol
     *      (skipped when ego is stationary)
     *
     * @param ref        Reflector (anchor) track
     * @param ghost      Candidate ghost track
     * @param N          Fold order to test
     * @param ego_speed  Host speed [m/s]
     * @return true if all active criteria pass
     */
    bool isNFoldMirror(const RadarTrack &ref,
                       const RadarTrack &ghost,
                       int N,
                       float ego_speed) const;

    /// Compute the ego-frame radial speed of a track: projection of absolute
    /// velocity onto the bearing direction minus ego contribution.
    static float egoFrameRadialSpeed(const RadarTrack &t, float ego_speed);

    /// Euclidean range from radar origin.
    static float trackRange(const RadarTrack &t);
};

} // namespace radarprocessing
