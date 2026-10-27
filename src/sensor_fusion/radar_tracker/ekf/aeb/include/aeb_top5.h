/**
 * @file aeb_top5.h
 * @brief C-callable AEB / top-5 most-important-object API.
 *
 * Defines plain-C structures plus an extern "C" interface around the C++
 * EkfTracker so main.c (compiled as C) can drive radar processing without
 * touching any C++ types.
 */
#ifndef AEB_TOP5_H
#define AEB_TOP5_H

#include <stdbool.h>
#include <stdint.h>

#include "radar.h" /* RadarData */

#ifdef __cplusplus
extern "C" {
#endif

#define AEB_MAX_TOP_OBJECTS 5

typedef struct {
  uint32_t track_id;      /* Unique track identifier */
  float px, py;           /* Position [m] in vehicle frame */
  float vx, vy;           /* Velocity [m/s] in vehicle frame */
  float range;            /* Distance from ego [m] */
  float relative_speed;   /* Closing speed [m/s] (positive = approaching) */
  float lateral_position; /* Lateral offset from ego centerline [m] */
  float heading;          /* Heading angle [rad] */
  float importance_score; /* Combined importance metric [0-100] */
  bool is_dynamic;        /* True if object is moving */
  float confidence;       /* Track confidence (POE) [0-1] */
  int age;                /* Track age [frames] */
  int hits;               /* Total measurement associations */
} ImportantObject;

typedef struct {
  ImportantObject objects[AEB_MAX_TOP_OBJECTS];
  int count;             /* Number of valid objects (0..AEB_MAX_TOP_OBJECTS) */
  uint32_t frame_number; /* Frame counter */
  float ego_speed;       /* Ego vehicle speed [m/s] */
} Top5Objects;

/* ── Wrapper API (implemented in aeb_wrapper.cpp) ───────────────────────────
 * aeb_init():           one-time tracker construction.  Safe to call twice.
 * aeb_shutdown():       free internal tracker.
 * aeb_process_frame_mois():
 *    Convert MOIS RadarData → RadarPoint, run one EKF update, score and
 *    select the top AEB_MAX_TOP_OBJECTS by importance, write into out_top5.
 *    dt_s is the time delta since the previous call.  ego_speed is in m/s.
 *    Returns 0 on success, non-zero on error.
 */
int aeb_init(void);
void aeb_shutdown(void);

/**
 * @brief Process MOIS radar frame through AEB tracker
 * @param mois_pts        Input raw radar points
 * @param n_pts           Number of input points
 * @param ego_speed       Ego vehicle speed [m/s]
 * @param dt_s            Time delta since last frame [s]
 * @param out_top5        Output: Top-5 most important objects
 * @param out_tracks      Output: ALL confirmed tracks for visualization
 * @param max_tracks      Size of out_tracks array
 * @return Number of tracks written to out_tracks (or -1 on error)
 */
int aeb_process_frame_mois(const RadarData *mois_pts, int n_pts,
                           float ego_speed, float dt_s, Top5Objects *out_top5,
                           RadarData *out_tracks, int max_tracks);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AEB_TOP5_H */
