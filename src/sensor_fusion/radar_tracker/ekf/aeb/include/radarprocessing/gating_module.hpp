// gating_module.hpp
#pragma once
#include "tracker_config.hpp"
// NOTE: Do NOT include ekf_tracker.hpp here — it includes gating_module.hpp,
//       creating a circular dependency. EkfTrackerConfig comes from tracker_config.hpp.
#include "datatype/radar_point.hpp"
#include <cmath>
#include <algorithm>

namespace radarprocessing
{
    namespace gating
    {

        enum class GatingMode
        {
            SPATIAL,
            MAHALANOBIS,
            RADIAL,
            ADAPTIVE
        };

        // Gating strategy interface
        //
        // evaluate() parameters:
        //   track       — predicted track state
        //   point       — incoming radar point (point.radial_speed is RAW, not ego-compensated)
        //   point_speed — ego-compensated radial speed of this point (pre-computed by caller)
        //   ego_speed   — host vehicle forward speed [m/s] (used by AdaptiveGating)
        //   out_score   — association score (lower = better match); set to 1e9 on rejection
        //
        class IGatingStrategy
        {
        public:
            virtual ~IGatingStrategy() = default;
            virtual bool evaluate(const RadarTrack &track,
                                  const RadarPoint &point,
                                  float point_speed,
                                  float ego_speed,
                                  float &out_score) const = 0;
            virtual const char *name() const = 0;
        };

        // Spatial/Euclidean gating
        class SpatialGating : public IGatingStrategy
        {
        public:
            explicit SpatialGating(float gate_scale = 3.0f) : gate_scale_(gate_scale) {}
            bool evaluate(const RadarTrack &track, const RadarPoint &point,
                          float point_speed, float ego_speed, float &out_score) const override;
            const char *name() const override { return "Spatial"; }

        private:
            float gate_scale_;
        };

        // Mahalanobis distance gating
        class MahalanobisGating : public IGatingStrategy
        {
        public:
            MahalanobisGating(float meas_noise_pos, float gate_threshold)
                : meas_noise_pos_(meas_noise_pos), gate_threshold_(gate_threshold) {}
            bool evaluate(const RadarTrack &track, const RadarPoint &point,
                          float point_speed, float ego_speed, float &out_score) const override;
            const char *name() const override { return "Mahalanobis"; }

        private:
            float meas_noise_pos_;
            float gate_threshold_;
        };

        // Radial velocity gating
        class RadialGating : public IGatingStrategy
        {
        public:
            explicit RadialGating(float radial_gate) : radial_gate_(radial_gate) {}
            bool evaluate(const RadarTrack &track, const RadarPoint &point,
                          float point_speed, float ego_speed, float &out_score) const override;
            const char *name() const override { return "Radial"; }

        private:
            float radial_gate_;
        };

        // Adaptive gating — Python-style (multi_radar_main.py)
        //
        // Replicates the exact algorithm from TrackedObject.gating_distance() +
        // the velocity-consistency check in process_all_radars():
        //   1. Dynamic classification via ego-compensated track radial (track bearing)
        //   2. 3-level gate factors: fast-dynamic (25/16), slow-dynamic (9/9), static (4/4)
        //   3. Object-size-adaptive gate dimensions: Gx = factor + L,  Gy = factor + W
        //   4. Diagonal variance floor: var = max(P[i,i], 0.2)
        //   5. Score: S = dx²/(var_x·Gx) + dy²/(var_y·Gy)  — pass if S ≤ 1
        //   6. Radial consistency: |dot(track_vel, track_dir) − point_raw_radial| ≤ radial_gate
        //
        class AdaptiveGating : public IGatingStrategy
        {
        public:
            // radial_gate: max allowed |predicted_radial − measured_radial| [m/s] (default 3.0)
            explicit AdaptiveGating(float radial_gate = 3.0f) : radial_gate_(radial_gate) {}

            bool evaluate(const RadarTrack &track, const RadarPoint &point,
                          float point_speed, float ego_speed, float &out_score) const override;
            const char *name() const override { return "Adaptive"; }

        private:
            float radial_gate_; ///< Radial consistency gate [m/s]
        };

        // Gating factory and composite
        class GatingStrategyFactory
        {
        public:
            static std::unique_ptr<IGatingStrategy> create(GatingMode mode,
                                                           const EkfTrackerConfig &cfg);
        };

        // Helper function to compute mahalanobis distance (used by multiple strategies)
        float computeMahalanobisDistance2D(const Eigen::Matrix4f &P, float px, float py,
                                           float mx, float my, float r_factor);

    } // namespace gating
} // namespace radarprocessing
