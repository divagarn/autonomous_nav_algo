/**
 * @file ekf_tracker.hpp
 * @ingroup radar
 * @author Dineshkumar
 * @brief EKF multi-object radar tracker (constant-velocity, 2-D).
 */
#pragma once

#include "radarprocessing/radar_clusterer.hpp"
#include "datatype/radar_point.hpp"
#include "radarprocessing/tracker_config.hpp"
#include "radarprocessing/gating_module.hpp"
#include "radarprocessing/mirror_detector.hpp"
#include "utils.hpp"
#include <vector>
#include <array>
#include <cstdint>
#include <memory>
#include <Eigen/Dense>

namespace radarprocessing
{

    class EkfTracker
    {
    public:
        explicit EkfTracker(const EkfTrackerConfig &cfg = {});

        /**
         * @brief Run one tracking cycle
         * @param points    Raw radar detections
         * @param ego_speed Host vehicle speed [m/s]
         * @param dt_s      Time delta [s]
         * @return Current track list (all tracks)
         */
        std::vector<RadarTrack> update(const std::vector<RadarPoint> &points,
                                       float ego_speed,
                                       float dt_s);
        void reset();
        void setConfig(const EkfTrackerConfig &cfg) { cfg_ = cfg; }
        const EkfTrackerConfig &config() const { return cfg_; }
        void setGatingMode(gating::GatingMode mode);

        /// Internal DBSCAN clusters formed during the last update() call.
        const std::vector<RadarCluster> &getClusters() const { return clusters_; }

        /**
         * @brief Save current tracker state for later restoration
         */
        struct TrackerState
        {
            std::vector<RadarTrack> tracks;
            uint32_t next_id;
            std::vector<RadarCluster> clusters;
        };

        TrackerState saveState() const;
        void restoreState(const TrackerState &state);

        int createdTracksCount = 0;
        int deletedTracksCount = 0;

    private:
        void predict(RadarTrack &t, float dt) const;
        void updateEKF(RadarTrack &t, float mx, float my) const;
        
        MirrorDetector mirror_detector_;
        
        /// Mahalanobis-like distance (squared) using S = H*P*H^T + R.
        /// Compare against gate_maha_dynamic / gate_maha_static.
        float mahalanobisDistance(const RadarTrack &t, float mx, float my) const;

        /// DBSCAN on XY plane; min_samples=1 (every point is a core point).
        std::vector<int> dbscanXY(const std::vector<RadarPoint> &pts) const;

        /// Compute axis-aligned bbox from a set of radar points.
        static RadarBBox3D computeBBoxFromPoints(const std::vector<RadarPoint> &pts);

        uint32_t next_id_ = 1;
        uint32_t newId() { return next_id_++; }

    public:
        uint32_t getNextId() const { return next_id_; }

    private:
        EkfTrackerConfig cfg_;
        gating::GatingMode gating_mode_ = gating::GatingMode::ADAPTIVE;
        std::vector<RadarTrack> tracks_;
        std::vector<RadarCluster> clusters_; ///< Last internal DBSCAN result
        std::unique_ptr<gating::IGatingStrategy> gating_strategy_;
        float adaptiveEps(const RadarPoint &p) const;
        // ═════════════════════════════════════════════════════════════════════════
        // Pre-computed Matrices (Constants)
        // ═════════════════════════════════════════════════════════════════════════
        Matrix2x4f H_; // Observation matrix [I_2x2, 0_2x2]
        Matrix2f R_;   // Measurement noise covariance

        // ═════════════════════════════════════════════════════════════════════════
        // Workspace (Reuse to avoid allocations in hot loop)
        // ═════════════════════════════════════════════════════════════════════════
        mutable EKFWorkspace workspace_;
    };

} // namespace radarprocessing
