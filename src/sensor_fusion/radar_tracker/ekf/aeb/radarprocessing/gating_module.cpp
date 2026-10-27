/**
 * @file gating_module.cpp
 * @brief Implementations of gating strategies and the factory.
 *
 * State vector: [px, py, vx, vy]  (row-major M4 layout)
 * Measurement : [mx, my]          (position only)
 */

#include "radarprocessing/gating_module.hpp"
#include "radarprocessing/tracker_config.hpp"

#include <cmath>
#include <memory>

namespace radarprocessing
{
    namespace gating
    {

        // ─────────────────────────────────────────────────────────────────────────────
        // Helper: 2-D Mahalanobis distance²
        //   d² = (z - Hx)ᵀ (HPHᵀ + r²·I₂)⁻¹ (z - Hx)
        //   H selects rows 0 and 1 of the state (position only).
        // ─────────────────────────────────────────────────────────────────────────────
        float computeMahalanobisDistance2D(const Eigen::Matrix4f &P,
                                           float pred_px, float pred_py,
                                           float meas_x, float meas_y,
                                           float r_factor)
        {
            // Innovation (residual)
            float dx = meas_x - pred_px;
            float dy = meas_y - pred_py;

            // S = top-left 2×2 of P  +  r²·I₂
            float r2 = r_factor * r_factor;
            float S00 = P(0, 0) + r2;
            float S01 = P(0, 1);
            float S10 = P(1, 0);
            float S11 = P(1, 1) + r2;

            // Invert 2×2
            float det = S00 * S11 - S01 * S10;
            if (std::fabs(det) < 1e-12f)
                return 1e9f; // singular — return large distance

            float inv = 1.f / det;
            float Si00 = S11 * inv;
            float Si01 = -S01 * inv;
            float Si10 = -S10 * inv;
            float Si11 = S00 * inv;

            // d² = [dx, dy] · S⁻¹ · [dx, dy]ᵀ
            return dx * (Si00 * dx + Si01 * dy) + dy * (Si10 * dx + Si11 * dy);
        }

        // ─────────────────────────────────────────────────────────────────────────────
        // SpatialGating
        //   Gate on plain Euclidean distance.  Score = d² (lower is better).
        //   Threshold: gate_scale * sqrt(P diagonal) averaged.
        // ─────────────────────────────────────────────────────────────────────────────
        bool SpatialGating::evaluate(const RadarTrack &track,
                                     const RadarPoint &point,
                                     float /*point_speed*/,
                                     float /*ego_speed*/,
                                     float &out_score) const
        {
            float dx = point.x - track.px;
            float dy = point.y - track.py;
            float d2 = dx * dx + dy * dy;

            // Adaptive threshold based on track position uncertainty
            float sigma_x = std::sqrt(std::max(track.P(0, 0), 0.01f));
            float sigma_y = std::sqrt(std::max(track.P(1, 1), 0.01f));
            float gate_r = gate_scale_ * 0.5f * (sigma_x + sigma_y);
            float gate_r2 = gate_r * gate_r;

            out_score = d2;
            return d2 <= gate_r2;
        }

        // ─────────────────────────────────────────────────────────────────────────────
        // MahalanobisGating
        //   Score = Mahalanobis distance squared.  Pass if score ≤ gate_threshold_.
        // ─────────────────────────────────────────────────────────────────────────────
        bool MahalanobisGating::evaluate(const RadarTrack &track,
                                         const RadarPoint &point,
                                         float /*point_speed*/,
                                         float /*ego_speed*/,
                                         float &out_score) const
        {
            out_score = computeMahalanobisDistance2D(track.P,
                                                     track.px, track.py,
                                                     point.x, point.y,
                                                     meas_noise_pos_);
            return out_score <= gate_threshold_;
        }

        // ─────────────────────────────────────────────────────────────────────────────
        // RadialGating
        //   Score = |Δradial_speed|.  Pass if ≤ radial_gate_.
        // ─────────────────────────────────────────────────────────────────────────────
        bool RadialGating::evaluate(const RadarTrack &track,
                                    const RadarPoint & /*point*/,
                                    float point_speed,
                                    float /*ego_speed*/,
                                    float &out_score) const
        {
            float dv = std::fabs(point_speed - track.radial_speed);
            out_score = dv;
            return dv <= radial_gate_;
        }

        // ─────────────────────────────────────────────────────────────────────────────
        // AdaptiveGating  — exact port of multi_radar_main.py
        //
        // Python reference (TrackedObject.gating_distance + process_all_radars loop):
        //
        //   alpha        = atan2(trk.y, trk.x)
        //   ego_on_los   = ego_speed * cos(alpha)
        //   abs_rel_speed = abs(trk.radial_speed + ego_on_los)
        //   is_dynamic   = abs_rel_speed > 0.3
        //
        //   Gate factors (3 levels):
        //     fast-dynamic  (abs_rel_speed > 5): Gx_factor=25, Gy_factor=16
        //     slow-dynamic  (abs_rel_speed ≤ 5): Gx_factor=9,  Gy_factor=9
        //     static                           : Gx_factor=4,  Gy_factor=4
        //
        //   L  = max(track.length, 0.5),  W = max(track.width, 0.5)
        //   Gx = Gx_factor + L,           Gy = Gy_factor + W
        //   var_x = max(P[0,0], 0.2),     var_y = max(P[1,1], 0.2)
        //   S = dx²/(var_x·Gx) + dy²/(var_y·Gy)   — pass if S ≤ 1
        //
        //   Radial consistency (from Python association loop):
        //     pred_pos   = (track.px, track.py)
        //     direction  = pred_pos / (|pred_pos| + 1e-6)
        //     pred_rad   = dot(track_vel, direction)
        //     pass if |pred_rad − point.radial_speed| ≤ radial_gate   (default 3.0)
        //
        // ─────────────────────────────────────────────────────────────────────────────
        bool AdaptiveGating::evaluate(const RadarTrack &track,
                                      const RadarPoint &point,
                                      float point_speed,
                                      float ego_speed,
                                      float &out_score) const
        {
            // ── 1. Dynamic classification — Python: abs_rel_speed = |trk.radial_speed + ego_on_los|
            //       Uses track bearing, not point bearing.
            float alpha = std::atan2(track.py, track.px);
            float ego_on_los = ego_speed * std::cos(alpha);
            float abs_rel_speed = std::fabs(track.radial_speed + ego_on_los);
            bool is_dynamic = abs_rel_speed > 0.3f;

            // ── 2. Three-level gate factors (Python: if/elif/else on abs_rel_speed)
            float gx_factor, gy_factor;
            if (is_dynamic)
            {
                if (abs_rel_speed > 5.0f)
                {
                    gx_factor = 25.f; // fast-dynamic
                    gy_factor = 16.f;
                }
                else
                {
                    gx_factor = 9.f; // slow-dynamic
                    gy_factor = 9.f;
                }
            }
            else
            {
                gx_factor = 4.f; // static
                gy_factor = 4.f;
            }

            // ── 3. Object-size-adaptive gate (Python: Gx = factor + L,  Gy = factor + W)
            float L = std::max(track.length, 0.5f);
            float W = std::max(track.width, 0.5f);

            float G_long = gx_factor + L; // along motion
            float G_lat = gy_factor + W;  // perpendicular

            // 4. covariance
            float var_x = std::max(track.P(0, 0), 0.2f);
            float var_y = std::max(track.P(1, 1), 0.2f);

            // 5. Residual
            float dx = point.x - track.px;
            float dy = point.y - track.py;

            // ── 6. Compute velocity direction
            float speed_sq = track.vx * track.vx + track.vy * track.vy;

            float cos_t, sin_t;

            if (speed_sq > 1e-4f)
            {
                cos_t = std::cos(track.yaw);
                sin_t = std::sin(track.yaw);
            }
            else
            {
                // Fallback: use bearing (important for static objects)
                float theta = std::atan2(track.py, track.px);
                cos_t = std::cos(theta);
                sin_t = std::sin(theta);
            }

            // ── 7. Rotate residual into motion frame
            float d_long = cos_t * dx + sin_t * dy; // along velocity
            float d_lat = -sin_t * dx + cos_t * dy; // perpendicular

            // ── 8. Approximate variance in rotated frame (lightweight)
            // NOTE: ignoring cross-covariance for efficiency
            float var_long = var_x * cos_t * cos_t + var_y * sin_t * sin_t;
            float var_lat = var_x * sin_t * sin_t + var_y * cos_t * cos_t;

            var_long = std::max(var_long, 0.2f);
            var_lat = std::max(var_lat, 0.2f);

            // ── 9. Rotated elliptical gating
            float S = (d_long * d_long) / (var_long * G_long) +
                      (d_lat * d_lat) / (var_lat * G_lat);

            if (S > 1.0f)
            {
                out_score = 1e9f;
                return false;
            }

            // ── 10. Radial consistency (unchanged)
            float range = std::sqrt(track.px * track.px + track.py * track.py) + 1e-6f;
            float dir_x = track.px / range;
            float dir_y = track.py / range;
            float pred_radial = track.vx * dir_x + track.vy * dir_y;

            if (std::fabs(pred_radial - point.radial_speed) > radial_gate_)
            {
                out_score = 1e9f;
                return false;
            }

            out_score = S;
            return true;
        }

        // ─────────────────────────────────────────────────────────────────────────────
        // GatingStrategyFactory
        // ─────────────────────────────────────────────────────────────────────────────
        std::unique_ptr<IGatingStrategy>
        GatingStrategyFactory::create(GatingMode mode, const EkfTrackerConfig &cfg)
        {
            switch (mode)
            {
            case GatingMode::SPATIAL:
                return std::make_unique<SpatialGating>(3.0f);

            case GatingMode::MAHALANOBIS:
                return std::make_unique<MahalanobisGating>(
                    cfg.meas_noise_pos, cfg.gate_maha_dynamic);

            case GatingMode::RADIAL:
                return std::make_unique<RadialGating>(cfg.radial_gate);

            case GatingMode::ADAPTIVE:
            default:
                // Python-style adaptive gating — gate factors and thresholds are
                // hard-coded to match multi_radar_main.py; only radial_gate is
                // configurable (default 3.0 m/s, same as Python).
                return std::make_unique<AdaptiveGating>(cfg.radial_gate);
            }
        }

    } // namespace gating
} // namespace radarprocessing
