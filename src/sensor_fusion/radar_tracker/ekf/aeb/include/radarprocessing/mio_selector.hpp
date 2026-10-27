/**
 * @file mio_selector.hpp
 * @brief Production-grade Most Important Object (MIO) selection for AEB
 *
 * Implements hysteresis, path prediction, and threat assessment to minimize
 * MIO switching and false positives, based on tier-1 supplier best practices.
 *
 * Key Features:
 * - Hysteresis-based selection to prevent MIO oscillation
 * - TTC (Time-to-Collision) and THW (Time Headway) computation
 * - Path overlap prediction with lateral boundaries
 * - Multi-frame persistence and track quality filtering
 * - Configurable threat assessment weights
 */

#ifndef MIO_SELECTOR_HPP
#define MIO_SELECTOR_HPP

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>
#include "datatype/radar_point.hpp"  // For full RadarTrack definition (from include path)

/**
 * @brief MIO candidate structure with threat assessment
 */
struct MioCandidate
{
    uint32_t track_id;
    float px, py;       // Position [m]
    float vx, vy;       // Velocity [m/s]
    float range;        // Distance [m]
    float ttc;          // Time-to-Collision [s]
    float thw;          // Time Headway [s]
    float path_overlap; // Lateral path overlap [0..1]
    float threat_score; // Combined threat metric [0..100]
    float poe;          // Probability of Existence
    bool is_dynamic;
    uint32_t age;
    uint32_t hits;
};

/**
 * @brief Internal MIO state for hysteresis tracking
 */
struct MioState
{
    uint32_t current_mio_id;
    uint32_t frames_held;
    float last_threat_score;
    bool is_valid;
};

/**
 * @brief Production-grade MIO selector with hysteresis and threat assessment
 */
class MioSelector
{
public:
    /**
     * @brief Configuration parameters for MIO selection
     */
    struct Config
    {
        // Hysteresis parameters
        float mio_switch_threshold;     // Threat score delta to switch MIO [default: 20.0]
        uint32_t min_hold_frames;       // Min frames to hold MIO [default: 5]
        float switch_hysteresis_factor; // Multiplier for current MIO [default: 1.3]

        // Track quality filters
        float min_poe;     // Minimum POE for MIO [default: 0.75]
        uint32_t min_age;  // Minimum track age [default: 3]
        uint32_t min_hits; // Minimum hits [default: 5]

        // Threat assessment weights
        float weight_ttc;           // [default: 40.0]
        float weight_path_overlap;  // [default: 30.0]
        float weight_distance;      // [default: 20.0]
        float weight_closing_speed; // [default: 10.0]

        // Physical limits
        float max_range;         // Max relevant range [m, default: 80.0]
        float min_ttc_threshold; // Min TTC to consider [s, default: 0.5]
        float max_ttc_threshold; // Max TTC to consider [s, default: 8.0]
        float ego_width;         // Vehicle width [m, default: 2.5]
        float ego_length;        // Vehicle length [m, default: 6.0]
        float lateral_margin;    // Safety margin [m, default: 0.5]

        // Lateral filtering (critical for lane-specific MIO)
        float lateral_min;              // Left boundary [m, default: -2.2]
        float lateral_max;              // Right boundary [m, default: 2.2]
        bool use_hard_lateral_bounds;   // Enforce hard boundaries [default: true]
        bool use_predicted_lateral;     // Also check predicted position [default: true]
        float predicted_lateral_margin; // Margin for predicted position [m, default: 0.5]

        // Low-speed hardening (used by selectMio_fixed)
        float min_px;              // Bumper exclusion zone [m, default: 2.5]
        float low_speed_threshold; // Ego speed below which stricter filters apply [m/s, default: 0.5]
        float low_speed_min_poe;   // Min POE at low speed [default: 0.75]
        bool use_range_only_score; // Use range-only scoring instead of full weighted score [default: false]

        Config()
            : mio_switch_threshold(20.0f), min_hold_frames(5), switch_hysteresis_factor(1.3f), min_poe(0.75f), min_age(3), min_hits(5), weight_ttc(10.0f), weight_path_overlap(20.0f), weight_distance(40.0f), weight_closing_speed(10.0f), max_range(120.0f), min_ttc_threshold(0.5f), max_ttc_threshold(8.0f), ego_width(1.4f), ego_length(30.0f), lateral_margin(0.5f), lateral_min(-1.4f), lateral_max(1.4f), use_hard_lateral_bounds(true), use_predicted_lateral(true), predicted_lateral_margin(0.5f), min_px(2.5f), low_speed_threshold(0.5f), low_speed_min_poe(0.75f), use_range_only_score(false)
        {
        }
    };

    explicit MioSelector(const Config &config = Config())
        : config_(config), state_{}
    {
        state_.current_mio_id = 0;
        state_.frames_held = 0;
        state_.last_threat_score = 0.0f;
        state_.is_valid = false;
    }

    /**
     * @brief Select the Most Important Object from confirmed tracks
     *
     * @param tracks Vector of confirmed radar tracks
     * @param ego_speed Current ego vehicle speed [m/s]
     * @param dt Time delta since last update [s]
     * @return Pointer to MIO candidate or nullptr if none qualified
     */
    const MioCandidate *selectMio(const std::vector<struct RadarTrack> &tracks,
                                  float ego_speed,
                                  float dt);

    /**
     * @brief Select MIO using simple closest-confirmed-in-lane logic with hold hysteresis
     */
    const MioCandidate *selectMio_simple(const std::vector<struct RadarTrack> &tracks);

    /**
     * @brief Select MIO with full fixes: low-speed hardening, live hysteresis, predicted lateral
     */
    const MioCandidate *selectMio_fixed(const std::vector<struct RadarTrack> &tracks,
                                        float ego_speed,
                                        float dt);

    /**
     * @brief Get current MIO state for debugging
     */
    const MioState &getState() const { return state_; }

    /**
     * @brief Get all candidates from last selection (for debugging/visualization)
     */
    const std::vector<MioCandidate> &getCandidates() const { return candidates_; }

    /**
     * @brief Reset MIO selection (e.g., on mode change or system restart)
     */
    void reset()
    {
        state_.current_mio_id = 0;
        state_.frames_held = 0;
        state_.last_threat_score = 0.0f;
        state_.is_valid = false;
        candidates_.clear();
    }

private:
    static constexpr uint32_t SIMPLE_HOLD_FRAMES = 3;

    Config config_;
    MioState state_;
    std::vector<MioCandidate> candidates_;

    // Hold state for selectMio_simple
    uint32_t     simple_hold_remaining_ = 0;
    uint32_t     simple_last_mio_id_    = 0;
    MioCandidate simple_held_cand_      = {};

    /**
     * @brief Calculate Time-to-Collision using constant velocity model
     */
    float calculateTTC(float px, float py, float vx, float vy, float ego_speed) const;

    /**
     * @brief Calculate Time Headway (distance / ego_speed)
     */
    float calculateTHW(float range, float ego_speed) const;

    /**
     * @brief Calculate path overlap (lateral collision potential)
     * Returns [0..1] where 1.0 = full overlap, 0.0 = no overlap
     */
    float calculatePathOverlap(float py, float vy, float ego_speed, float prediction_time) const;

    /**
     * @brief Calculate comprehensive threat score [0..100]
     */
    float calculateThreatScore(const MioCandidate &candidate, float ego_speed) const;

    /**
     * @brief Check if track meets minimum quality requirements
     */
    bool meetsQualityThreshold(const struct RadarTrack &track) const;

    /**
     * @brief Apply hysteresis logic to prevent rapid MIO switching
     */
    bool shouldSwitchMio(const MioCandidate &new_candidate) const;

    /**
     * @brief Hysteresis using live scores (fix for stale hysteresis bug)
     */
    bool shouldSwitchMio_fixed(const MioCandidate &current_mio,
                               const MioCandidate &new_candidate) const;
};

#endif // MIO_SELECTOR_HPP