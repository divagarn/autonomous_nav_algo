/**
 * @file aeb_wrapper.cpp
 * @brief extern "C" bridge between C-only main.c and the C++ EkfTracker.
 *        Integrated with production-grade MIO selector.
 */

#include "aeb_top5.h"

#include "radarprocessing/ekf_tracker.hpp"
#include "radarprocessing/tracker_config.hpp"
#include "radarprocessing/gating_module.hpp"
#include "radarprocessing/mio_selector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

using radarprocessing::EkfTracker;
using radarprocessing::EkfTrackerConfig;
using radarprocessing::gating::GatingMode;

static EkfTracker *g_tracker = nullptr;
static MioSelector *g_mio_selector = nullptr;
static uint32_t g_frame_count = 0;
static std::vector<RadarTrack> g_last_tracks; /* Store tracks from last update for visualization */

static EkfTrackerConfig make_default_config()
{
    EkfTrackerConfig cfg;
    cfg.enabled = true;
    cfg.sigma_a = 3.0f;
    cfg.meas_noise_pos = 1.0f;
    cfg.radial_gate = 3.0f;
    cfg.dbscan_eps = 0.7f;
    cfg.gate_maha_dynamic = 9.0f;
    cfg.gate_maha_static = 16.0f;
    cfg.dynamic_speed_thresh = 0.5f;
    cfg.merge_dist = 2.0f;
    cfg.dbscan_outlier_scale = 2.5f;
    cfg.poe_gain = 0.15f;
    cfg.poe_decay = 0.15f;
    cfg.poe_confirm = 0.70f;
    cfg.poe_unconfirm = 0.40f;
    cfg.poe_delete = 0.10f;
    cfg.poe_init = 0.50f;
    cfg.max_missed_frames = 5;
    return cfg;
}

/* Importance score in [0..100] — distance, closing speed, lateral, confidence. */
static float calculate_importance_score(const RadarTrack &t, float ego_speed)
{
    const float range = std::sqrt(t.px * t.px + t.py * t.py);

    float distance_score;
    if (range < 5.0f)
        distance_score = 30.0f;
    else if (range < 15.0f)
        distance_score = 30.0f * (15.0f - range) / 10.0f;
    else if (range < 40.0f)
        distance_score = 15.0f * (40.0f - range) / 25.0f;
    else
        distance_score = 0.0f;

    const float track_speed = std::sqrt(t.vx * t.vx + t.vy * t.vy);
    const float rel_speed = ego_speed - track_speed;
    float speed_score;
    if (rel_speed > 10.0f)
        speed_score = 30.0f;
    else if (rel_speed > 0.5f)
        speed_score = 30.0f * (rel_speed / 10.0f);
    else
        speed_score = 0.0f;

    const float lat = std::fabs(t.py);
    float lateral_score;
    if (lat < 1.0f)
        lateral_score = 25.0f;
    else if (lat < 3.0f)
        lateral_score = 25.0f * (3.0f - lat) / 2.0f;
    else
        lateral_score = 0.0f;

    const float confidence_score = t.poe * 15.0f;

    float total = distance_score + speed_score + lateral_score + confidence_score;
    if (total > 100.0f)
        total = 100.0f;
    if (total < 0.0f)
        total = 0.0f;
    return total;
}

extern "C" int aeb_init(void)
{
    if (g_tracker)
        return 0;
        
    g_tracker = new (std::nothrow) EkfTracker(make_default_config());
    if (!g_tracker)
        return -1;
    g_tracker->setGatingMode(GatingMode::ADAPTIVE);
    
    // ═══════════════════════════════════════════════════════════════════
    // Initialize production-grade MIO selector
    // ═══════════════════════════════════════════════════════════════════
    MioSelector::Config mio_config;
    
    // Truck dimensions (adjust to your actual truck specifications)
    mio_config.ego_width = 2.5f;          // Full truck width [m]
    mio_config.ego_length = 6.0f;         // Truck length [m]
    
    // Hard lateral boundaries (critical for lane-specific MIO)
    mio_config.lateral_min = -1.4f;       // Left boundary [m]
    mio_config.lateral_max = 1.4f;        // Right boundary [m]
    mio_config.use_hard_lateral_bounds = true;
    mio_config.use_predicted_lateral = true;  // Also check predicted position
    mio_config.predicted_lateral_margin = 0.5f; // Margin for predicted pos [m]
    
    // Hysteresis parameters (key to stable MIO - no dangling)
    mio_config.min_hold_frames = 3;       // Hold MIO for minimum 5 frames
    mio_config.mio_switch_threshold = 20.0f;  // Require 20pt improvement to switch
    mio_config.switch_hysteresis_factor = 1.3f;  // 30% bonus to current MIO
    
    // Track quality filters (stricter for MIO than Top-5)
    mio_config.min_poe   = 0.65f;         // Minimum POE at normal speed
    mio_config.min_age   = 5;             // At least 5 frames old
    mio_config.min_hits  = 8;             // At least 8 detections

    // Threat assessment weights (tunable for your application)
    // BUG: weight_ttc and weight_path_overlap are zeroed — score is 100% distance-based.
    // Any ghost closer than the real target will outscore it regardless of lane position or TTC.
    // Switch to selectMio_fixed() in aeb_process_frame_mois() and use the corrected weights below.
    // mio_config.weight_ttc = 00.0f;           // BUG: should be 10.0f (TTC component)
    // mio_config.weight_path_overlap = 00.0f;  // BUG: should be 20.0f (path overlap component)
    // mio_config.weight_distance = 100.0f;     // dominates entirely because TTC+overlap are zero
    // mio_config.weight_closing_speed = 10.0f;

    mio_config.weight_ttc           = 10.0f;
    mio_config.weight_path_overlap  = 20.0f;
    mio_config.weight_distance      = 40.0f;
    mio_config.weight_closing_speed = 10.0f;
    
    // Physical thresholds
    mio_config.max_range = 120.0f;         // Maximum relevant range [m]
    mio_config.min_ttc_threshold = 0.5f;  // Minimum TTC to consider [s]
    mio_config.max_ttc_threshold = 8.0f;  // Maximum TTC to consider [s]
    
    g_mio_selector = new (std::nothrow) MioSelector(mio_config);
    if (!g_mio_selector) {
        delete g_tracker;
        g_tracker = nullptr;
        return -1;
    }
    
    return 0;
}

extern "C" void aeb_shutdown(void)
{
    delete g_tracker;
    g_tracker = nullptr;
    delete g_mio_selector;
    g_mio_selector = nullptr;
}

extern "C" int aeb_process_frame_mois(const RadarData *mois_pts,
                                      int n_pts,
                                      float ego_speed,
                                      float dt_s,
                                      Top5Objects *out_top5,
                                      RadarData *out_tracks,
                                      int max_tracks)
{
    if (!out_top5)
        return -1;
    std::memset(out_top5, 0, sizeof(*out_top5));
    out_top5->ego_speed = ego_speed;
    out_top5->frame_number = ++g_frame_count;

    if (!g_tracker || !g_mio_selector)
        return -2;

    /* RadarData → RadarPoint */
    std::vector<RadarPoint> pts;
    pts.reserve(n_pts > 0 ? n_pts : 0);
    for (int i = 0; i < n_pts; ++i)
    {
        const RadarData &s = mois_pts[i];
        RadarPoint p{};
        p.x = s.x;
        p.y = s.y;
        p.z = s.z;
        p.range = s.range;
        p.radial_speed = s.radial_speed;
        p.power = s.power;
        p.azimuth = s.azimuth;
        p.elevation = s.elevation;
        p.rcs = s.rcs;
        p.snr = s.snr;
        p.noise = s.noise;
        pts.push_back(p);
    }

    const std::vector<RadarTrack> tracks = g_tracker->update(pts, ego_speed, dt_s);

    /* Store tracks for later visualization */
    g_last_tracks = tracks;

    /* ═══════════════════════════════════════════════════════════════════════
     * PART 1: Select Most Important Object (MIO) using production-grade logic
     * ═══════════════════════════════════════════════════════════════════════ */
    int track_count = 0;
    if (tracks.size() > 0 ) {
        // Three implementations available — swap the active line to compare:
        // const MioCandidate* mio = g_mio_selector->selectMio(tracks, ego_speed, dt_s);         // original (buggy)
        // const MioCandidate* mio = g_mio_selector->selectMio_fixed(tracks, ego_speed, dt_s);   // full fix
        const MioCandidate* mio = g_mio_selector->selectMio_simple(tracks);                      // closest confirmed in lane
    
        // Store MIO as the FIRST object in Top5 (highest priority)
        if (mio) {
            ImportantObject &mio_obj = out_top5->objects[0];
            mio_obj.track_id = mio->track_id;
            mio_obj.px = mio->px;
            mio_obj.py = mio->py;
            mio_obj.vx = mio->vx;
            mio_obj.vy = mio->vy;
            mio_obj.range = mio->range;
            mio_obj.relative_speed = ego_speed - std::sqrt(mio->vx * mio->vx + mio->vy * mio->vy);
            mio_obj.lateral_position = mio->py;
            mio_obj.heading = std::atan2(mio->vy, mio->vx);
            mio_obj.importance_score = mio->threat_score;  // Use threat score
            mio_obj.confidence = mio->poe;
            mio_obj.is_dynamic = mio->is_dynamic;
            mio_obj.age = mio->age;
            mio_obj.hits = mio->hits;
            
            out_top5->count = 1;  // At least MIO is present
        }

        /* ═══════════════════════════════════════════════════════════════════════
        * PART 2: Convert ALL confirmed tracks to out_tracks[] for visualization
        * ═══════════════════════════════════════════════════════════════════════ */
        if (out_tracks && max_tracks > 0)
        {
            for (const RadarTrack &t : tracks)
            {
                /* Only export confirmed tracks with reasonable POE */
                if (!t.confirmed || t.poe < 0.6f)
                    continue;

                if (track_count >= max_tracks)
                    break;

                /* Convert RadarTrack → RadarData for visualization */
                RadarData &rd = out_tracks[track_count];
                rd.x = t.px;
                rd.y = t.py;
                rd.z = t.pz;
                rd.range = std::sqrt(t.px * t.px + t.py * t.py);
                rd.radial_speed = std::sqrt(t.vx * t.vx + t.vy * t.vy);
                rd.azimuth = std::atan2(t.py, t.px);
                rd.elevation = 0.0f;
                rd.power = t.avg_snr;
                rd.snr = t.avg_snr;
                rd.rcs = t.avg_rcs;
                rd.noise = 0.0f;
                rd.track_id = static_cast<int>(t.id);
                rd.cluster_id = static_cast<int>(t.id);

                track_count++;
            }
        }

        /* ═══════════════════════════════════════════════════════════════════════
        * PART 3: Score confirmed tracks and fill remaining Top-5 slots (2-5)
        * ═══════════════════════════════════════════════════════════════════════ */
        struct Scored
        {
            ImportantObject obj;
        };
        std::vector<Scored> cand;
        cand.reserve(tracks.size());

        /* Truck lateral range limits (half-width = 1.3m) */
        const float TRUCK_LATERAL_MIN = -1.4f; // Left boundary
        const float TRUCK_LATERAL_MAX = 1.4f;  // Right boundary

        for (const RadarTrack &t : tracks)
        {
            if (!t.confirmed || t.poe < 0.6f)
                continue;

            // Skip MIO (already in slot 0)
            if (mio && t.id == mio->track_id)
                continue;

            /* Filter: Only consider tracks within truck lateral range */
            if (t.py < TRUCK_LATERAL_MIN || t.py > TRUCK_LATERAL_MAX)
                continue;

            const float score = calculate_importance_score(t, ego_speed);
            if (score < 5.0f)
                continue;

            Scored s{};
            s.obj.track_id = t.id;
            s.obj.px = t.px;
            s.obj.py = t.py;
            s.obj.vx = t.vx;
            s.obj.vy = t.vy;
            s.obj.range = std::sqrt(t.px * t.px + t.py * t.py);
            s.obj.relative_speed = ego_speed - std::sqrt(t.vx * t.vx + t.vy * t.vy);
            s.obj.lateral_position = t.py;
            s.obj.heading = t.yaw;
            s.obj.importance_score = score;
            s.obj.confidence = t.poe;
            s.obj.is_dynamic = (std::sqrt(t.vx * t.vx + t.vy * t.vy) > 0.5f);
            s.obj.age = t.age;
            s.obj.hits = t.hits;
            cand.push_back(s);
        }

        std::sort(cand.begin(), cand.end(),
                [](const Scored &a, const Scored &b)
                {
                    return a.obj.importance_score > b.obj.importance_score;
                });

        // Fill remaining slots (up to 5 total including MIO)
        const int start_idx = mio ? 1 : 0;
        const int n = (int)std::min<size_t>(cand.size(), AEB_MAX_TOP_OBJECTS - start_idx);
        
        for (int i = 0; i < n; ++i)
            out_top5->objects[start_idx + i] = cand[i].obj;
        
        out_top5->count = start_idx + n;

    }
    
    /* Return number of tracks written to out_tracks[] */
    return track_count;
}

/* Optional: Export MIO state for debugging/logging */
extern "C" int aeb_get_mio_state(uint32_t *out_mio_id, 
                                  uint32_t *out_frames_held,
                                  float *out_threat_score,
                                  int *out_is_valid)
{
    if (!g_mio_selector)
        return -1;
        
    const MioState& state = g_mio_selector->getState();
    
    if (out_mio_id)
        *out_mio_id = state.current_mio_id;
    if (out_frames_held)
        *out_frames_held = state.frames_held;
    if (out_threat_score)
        *out_threat_score = state.last_threat_score;
    if (out_is_valid)
        *out_is_valid = state.is_valid ? 1 : 0;
        
    return 0;
}