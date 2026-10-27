/**
 * @file tracker_config.hpp
 * @brief Configuration parameters for the EKF multi-object radar tracker.
 *
 * Field set chosen to satisfy the references in ekf_tracker.cpp,
 * gating_module.cpp, and the init code in main.c (init_radar_tracker).
 */
#pragma once

#include <cstdint>

namespace radarprocessing
{

struct EkfTrackerConfig
{
    // ── Master switch ─────────────────────────────────────────────────────────
    bool  enabled              = true;

    // ── Kalman filter noise ───────────────────────────────────────────────────
    float sigma_a              = 3.0f;   ///< Acceleration noise [m/s²]
    float meas_noise_pos       = 1.0f;   ///< Position measurement noise [m]

    // ── Association gating ────────────────────────────────────────────────────
    float radial_gate          = 3.0f;   ///< Radial velocity gate [m/s]
    float dbscan_eps           = 0.7f;   ///< DBSCAN clustering radius [m]
    float gate_maha_dynamic    = 9.0f;   ///< Mahalanobis gate (dynamic)
    float gate_maha_static     = 16.0f;  ///< Mahalanobis gate (static, looser)
    float dynamic_speed_thresh = 0.5f;   ///< Dynamic vs static threshold [m/s]

    // ── Track management ──────────────────────────────────────────────────────
    float merge_dist           = 2.0f;   ///< Merge tracks closer than this [m]
    float dbscan_outlier_scale = 2.5f;   ///< MAD outlier rejection scale

    // ── POE (Probability of Existence) lifecycle ──────────────────────────────
    float poe_gain             = 0.15f;  ///< +POE per successful hit
    float poe_decay            = 0.15f;  ///< -POE per missed frame
    float poe_confirm          = 0.70f;  ///< Confirm track at this POE
    float poe_unconfirm        = 0.40f;  ///< Un-confirm below this POE
    float poe_delete           = 0.10f;  ///< Delete below this POE
    float poe_init             = 0.50f;  ///< Initial POE for new tracks
    int   max_missed_frames    = 5;      ///< Delete after this many misses

    // ── Ground/overdrivable classification (MIO exclusion only) ──────────────
    float ground_z_threshold   = 0.12f;  ///< [m] below this (height, relative to sensor mount), track is classified ground/overdrivable
    float ground_rcs_max       = -5.0f;  ///< [dBsm] rcs_ema_mean must also be below this, or the track is never classified ground

    // ── Static-track lateral rate limit ───────────────────────────────────────
    // Caps how far a single real update can move a stationary track's
    // lateral position (py), and clamps its fitted lateral velocity (vy) to
    // match -- a stationary track's wide association gate can otherwise pull
    // in points from a wider span of an elongated structure (guardrail/
    // barrier) frame to frame, walking the fitted centroid several meters
    // sideways purely from which points happen to fall in-gate. Longitudinal
    // (px/vx) and dynamic tracks are untouched.
    float static_lateral_rate_limit = 1.0f;  ///< [m/s] max |py| change per real update for a static track
};

} // namespace radarprocessing
