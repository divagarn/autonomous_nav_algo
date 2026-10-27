#pragma once
#include <cstdint>
#include <vector>
#include <limits>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <array>

/**
 * @author Dineshkumar
 * @struct RadarPoint
 * @brief Represents a single radar detection in 3D Cartesian space.
 *
 * This structure encapsulates both geometric and signal-related attributes
 * of a radar return. It is commonly used in perception and sensor fusion
 * pipelines for tasks such as object detection, tracking, and state estimation.
 *
 * The point is primarily represented in Cartesian coordinates, with optional
 * angular measurements depending on sensor availability.
 *
 * @section coordinate_frame Coordinate Frame
 * - x: Forward direction (meters)
 * - y: Lateral direction (meters)
 * - z: Vertical direction (meters)
 *
 * @section attributes Radar Attributes
 * - Range and radial speed represent physical measurements from the radar sensor.
 * - Signal-related fields (RCS, SNR, noise) describe detection quality.
 *
 * @section optional_fields Optional Fields
 * - Azimuth and elevation may be unavailable depending on radar configuration.
 * - If unavailable, they should be set to NaN.
 *
 * @section usage Usage
 * Typically used within a RadarFrame:
 * @code
 * RadarPoint pt;
 * pt.x = 10.0f;
 * pt.radial_speed = -2.5f;
 * @endcode
 */
struct RadarPoint
{
    float x; ///< Cartesian forward coordinate (meters)
    float y; ///< Cartesian lateral coordinate (meters)
    float z; ///< Cartesian height (meters)

    // Physics
    float range;        ///< Radial distance from radar (meters)
    float radial_speed; ///< Radial velocity relative to sensor (m/s)
    float power;        ///< Signal power (unit depends on sensor)

    // Optional angular info
    float azimuth;   ///< Azimuth angle (radians), NaN if unavailable
    float elevation; ///< Elevation angle (radians), NaN if unavailable

    float rcs;   ///< Radar Cross Section (dBsm)
    float snr;   ///< Signal-to-Noise Ratio (dB)
    float noise; ///< Noise level (sensor-specific units)

    uint32_t flags; ///< Bitmask for status/quality flags (sensor-specific)
};


/**
 * @struct RadarBBox3D
 * @brief Axis-aligned 3-D bounding box around a radar cluster.
 *
 * All coordinates are in the **radar sensor frame**.
 */
struct RadarBBox3D
{
    float x_min, x_max;   ///< Forward axis
    float y_min, y_max;   ///< Lateral axis
    float z_min, z_max;   ///< Height axis

    float center_x() const { return 0.5f * (x_min + x_max); }
    float center_y() const { return 0.5f * (y_min + y_max); }
    float center_z() const { return 0.5f * (z_min + z_max); }
    float width()    const { return x_max - x_min; }
    float depth()    const { return y_max - y_min; }
    float height()   const { return z_max - z_min; }

    /** Returns the 8 corners of this bbox as 3-D points (radar frame). */
    std::vector<cv::Point3f> corners() const;
};

/**
 * @struct RadarCluster
 * @brief A group of radar points belonging to the same object, with bbox.
 */
struct RadarCluster
{
    std::vector<RadarPoint> points;
    RadarBBox3D             bbox;
    int                     id = -1;
};


/**
 * @struct RadarTrack
 * @brief EKF state + lifecycle + aggregated radar signal data from matched cluster.
 */
struct RadarTrack {
    // ═════════════════════════════════════════════════════════════════════════
    // Identity & Lifecycle
    // ═════════════════════════════════════════════════════════════════════════
    uint32_t id        = 0;
    bool     confirmed = false;

     // EKF state  (radar sensor frame: x=forward, y=left)
    float px = 0.f, py = 0.f;   ///< Position estimate [m]
    float pz = 0.f;          ///< Height estimate [m] (optional, may be unused)
    float vx = 0.f, vy = 0.f;   ///< Velocity estimate [m/s]
    float vabsx = 0.f, vabsy = 0.f; ///< Velocity in body frame (for dynamic classification)
    float prev_px = 0.f, prev_py = 0.f;  ///< Previous centroid for displacement velocity
    // py at the START of this frame, before predict() advances it -- lets
    // the static-track lateral rate limit (ekf_tracker.cpp, see
    // EkfTrackerConfig::static_lateral_rate_limit) bound the TOTAL per-frame
    // py movement (predict's own step + this frame's update correction),
    // not just the update correction in isolation. Snapshotted once per
    // frame in the "predict all tracks" loop, right before predict() runs.
    float py_pre_predict = 0.f;
    float yaw;   // orientation (rad)
    bool is_dynamic = false; ///< Dynamic classification (true if likely a moving object)


     // Lifecycle (POE-based — mirrors multi_radar_main.py TrackedObject)
    float poe           = 0.5f;   ///< Probability of Existence [0, 1]
    int   hits          = 0;      ///< Total successful associations
    int   missed_frames = 0;      ///< Consecutive frames without association
    int   age           = 0;      ///< Total frames since creation

    // Physical dimensions (exponentially smoothed from point-cloud bbox)
    float length = 0.5f;          ///< Extent along forward axis [m]
    float width  = 0.5f;          ///< Extent along lateral axis [m]
    // Most-recent radial speed (for gating / dynamic classification)
    float radial_speed = 0.f;     ///< Last measured mean radial speed [m/s]

    // Last matched point-cloud bbox (radar frame)
    RadarBBox3D bbox{};

    // ── Aggregated signal data (mean over associated points) ──────────────────
    float avg_rcs       = 0.f;   ///< Mean RCS [dBsm]
    float avg_snr       = 0.f;   ///< Mean SNR [dB]
    float avg_range     = 0.f;   ///< Mean range [m]
    float avg_speed     = 0.f;   ///< Mean |radial speed| [m/s]
    float avg_azimuth   = std::numeric_limits<float>::quiet_NaN();  ///< Mean azimuth angle [rad], NaN if unavailable
    float avg_elevation = std::numeric_limits<float>::quiet_NaN();  ///< Mean elevation angle [rad], NaN if unavailable
    int   n_pts         = 0;     ///< Number of associated points this frame

    // ── RCS consistency (running EMA mean/variance across this track's own
    // hits, NOT the per-frame avg_rcs above) -- feeds is_ground_reflection.
    float rcs_ema_mean = 0.f;
    float rcs_ema_var  = 0.f;

    // ── Ground/overdrivable classification (MIO exclusion only) ────────────
    // z_ema_mean: running mean of pz (per-frame avg height of associated
    // points, relative to the sensor's own mount). is_ground_reflection is
    // set in ekf_tracker.cpp once z_ema_mean drops below
    // cfg_.ground_z_threshold AND rcs_ema_mean stays below
    // cfg_.ground_rcs_max (height alone can't tell a real low-riding vehicle
    // return apart from ground clutter, so a strong RCS is never treated as
    // ground regardless of height). NEVER used to block track
    // creation/confirmation -- only to keep a track out of MIO selection
    // (mio_selector.cpp); it's still tracked and still visible everywhere
    // else.
    float z_ema_mean           = 0.f;
    bool  is_ground_reflection = false;

      // ── Mirror ghost classification ─────────────────────────────────────────
    bool     is_mirror_ghost         = false; ///< True if this is an N-fold ghost
    bool     is_stationary_reflector = false; ///< True if this anchors ≥1 ghost
    uint32_t mirror_source_id        = 0;     ///< Track ID of the reflector anchor
    int      mirror_order            = 0;     ///< Fold order N (2,3,4,5)
    int      mirror_miss_frames      = 0;     ///< Consecutive frames without geom match
    
    // ═════════════════════════════════════════════════════════════════════════
    // State Vector (EIGEN - Primary storage)
    // ═════════════════════════════════════════════════════════════════════════
    // State: x = [px, py, vx, vy]^T
    Eigen::Vector4f x{Eigen::Vector4f::Zero()};
    
    // Covariance: P (4×4 positive semi-definite)
    Eigen::Matrix4f P{Eigen::Matrix4f::Identity()};
    
    // ═════════════════════════════════════════════════════════════════════════
    // SIMD Alignment (critical for performance)
    // ═════════════════════════════════════════════════════════════════════════
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // ═════════════════════════════════════════════════════════════════════════
    // Synchronization Methods
    // ═════════════════════════════════════════════════════════════════════════
    
    /**
     * @brief Update scalar fields from Eigen state vector
     * Call this after any EKF update that modifies x
     */
    inline void syncFromEigen() {
        px = x(0);
        py = x(1);
        vx = x(2);
        vy = x(3);
        yaw = std::atan2(vy, vx);
    }
    
    /**
     * @brief Update Eigen state vector from scalar fields
     * Call this if you modify px, py, vx, vy directly (legacy code)
     */
    inline void syncToEigen() {
        x << px, py, vx, vy;
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // Legacy Compatibility - Flat Array Access for OLD Code
    // ═════════════════════════════════════════════════════════════════════════
    
    /**
     * @brief Get covariance as flat array (legacy compatibility)
     * Only use this for interfacing with old code that expects std::array
     */
    std::array<float, 16> getPAsArray() const {
        std::array<float, 16> arr;
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                arr[i*4 + j] = P(i, j);
        return arr;
    }
    
    /**
     * @brief Set covariance from flat array (legacy compatibility)
     * Only use this for interfacing with old code that provides std::array
     */
    void setPFromArray(const std::array<float, 16>& arr) {
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j)
                P(i, j) = arr[i*4 + j];
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // Convenient Constructors
    // ═════════════════════════════════════════════════════════════════════════
    
    RadarTrack() = default;
    
    /**
     * @brief Initialize track with position and velocity
     */
    RadarTrack(uint32_t track_id, float pos_x, float pos_y, 
               float vel_x, float vel_y, float init_poe = 0.1f)
        : id(track_id), poe(init_poe)
    {
        x << pos_x, pos_y, vel_x, vel_y;
        syncFromEigen();
        prev_px = pos_x;
        prev_py = pos_y;
    }
};