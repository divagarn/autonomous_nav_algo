#ifndef RADAR_H
#define RADAR_H

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
/* stdatomic.h is C11-only — C++ uses <atomic> instead.
 * This guard lets radar.h be safely included from both .c and .cpp files. */
#ifdef __cplusplus
#include <atomic>
#else
#include <stdatomic.h>
#endif
#include <stdbool.h>

#define EPSILON 0.5f // DBSCAN distance threshold (metres) - using better params
#define MIN_POINTS 1 // minimum points to form a cluster
#define MAX_TRACKS 1280 // maximum number of tracked clusters
#define MAX_LOST_FRAMES 5
#define MAX_POINTS 10
// Raw radar data structure (direct from sensor parsing)
typedef struct {
  float x, y, z;
  float range;     // direct radar-reported range
  float azimuth;   // angle in radians or degrees
  float elevation; // angle in radians or degrees
  float radial_speed;     // radial speed (m/s)
  float power; // signal level
  float snr;       // signal-to-noise ratio
  float rcs;       // reflectivity
  float noise;
  int track_id;   // assigned by Kalman filter / tracker
  int cluster_id; // optional: assigned by DBSCAN
  bool association_only;
  int radar_id;  /* NEW — which physical mount (1..NUM_REPLAY_RADARS) produced this point */
} RadarData;

typedef uint16_t can_state_t;

#define MOIS_MASK 0x00FF
#define BSIS_MASK 0xFF00

/* MOIS bits */
#define MOIS_NORMAL 0x0000
#define MOIS_FAULT 0x0001
#define MOIS_INFO 0x0002
#define MOIS_COLLISION 0x0004

/* BSIS bits */
#define BSIS_NORMAL 0x0000
#define BSIS_FAULT 0x0100
#define BSIS_LEFT_INFO 0x0200
#define BSIS_RIGHT_INFO 0x0300
#define BSIS_LEFT_COLLISION 0x0400
#define BSIS_RIGHT_COLLISION 0x0500
#define BSIS_LEFT_RIGHT_INFO 0x0600
#define BSIS_LEFT_RIGHT_COLLISION 0x0700
#define BSIS_LEFT_COLLISION_RIGHT_INFO 0x0800
#define BSIS_LEFT_INFO_RIGHT_COLLISION 0x0900

/* LDW state bits */
#define LDW_NORMAL 0x00
#define LDW_FAULT 0x01
#define LDW_LEFT_WARNING 0x02
#define LDW_RIGHT_WARNING 0x03

/* DDAS state bits */
#define DDAS_NORMAL 0x00
#define DDAS_FAULT 0x01
#define DDAS_DROWSINESS 0x02
#define DDAS_DISTRACTION 0x03

/* ── AEB CAN frame signal definitions (0x04FD18E8, 8 bytes, 100 ms) ─────────
 *
 *  Bit [ 3: 0] — AEB Radar fault status   (4-bit, 0 = no fault, 1 = fault)
 *  Bit [ 7: 4] — AEB Camera fault status  (4-bit, 0 = no fault, 1 = fault)
 *  Bit [23: 8] — Relative Distance        (16-bit, 0–120 m)
 *  Bit [39:24] — Relative Velocity        (16-bit, -60 to +60 m/s,
 * offset-encoded) Bit [55:52] — Alive Counter            (4-bit, 0–15) Bit
 * [63:56] — Checksum                 (8-bit, 0–255)
 *
 * ───────────────────────────────────────────────────────────────────────────
 */

/* AEB Radar fault status (bits [3:0]) */
#define AEB_RADAR_FAULT_NONE 0x0
#define AEB_RADAR_FAULT_ACTIVE 0x1

/* AEB Camera fault status (bits [7:4]) */
#define AEB_CAMERA_FAULT_NONE 0x0
#define AEB_CAMERA_FAULT_ACTIVE 0x1

/* Relative Distance: 16-bit unsigned, unit = 0.01 m, range 0–120 m
 * Raw = distance_m * 100  →  max raw = 12000 (0x2EE0) */
#define AEB_DIST_SCALE 100.0f
#define AEB_DIST_MAX_RAW 12000u

/* Relative Velocity: 16-bit offset-encoded, unit = 0.01 m/s, offset = 60 m/s
 * Raw = (velocity_m_s + 60.0f) * 100  →  range 0–12000
 * Approaching object → velocity negative → raw < 6000
 * Receding  object  → velocity positive → raw > 6000 */
#define AEB_VEL_OFFSET_MS 60.0f
#define AEB_VEL_SCALE 100.0f
#define AEB_VEL_MAX_RAW 12000u

static inline uint64_t get_time_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}

/* ── extern "C" guard ────────────────────────────────────────────────────────
 * All declarations below are for functions and variables defined in C files
 * (radar.c, main.c).  Wrapping them in extern "C" stops the C++ compiler
 * from mangling their names, so the linker can match the C++ call sites to
 * the C-compiled symbols.  When included from a .c file the guard is a no-op.
 * ───────────────────────────────────────────────────────────────────────────
 */
#ifdef __cplusplus
extern "C" {
#endif

extern can_state_t current_state;
extern can_state_t last_state;

/* FCW-to-MOIS bridge flag.
 * When set to 1 (via "FCW_enable 1" in the config), the MOIS CAN state is
 * driven entirely by the FCW camera tracker instead of radar zone checks.
 * If FCW is active → MOIS_COLLISION is reported.
 * If FCW is inactive → MOIS_NORMAL is reported.
 * When 0, the original radar zone check (check_mois_zones) is used. */
extern int g_fcw_enable;

/* FCW zone states — written every draw frame by app_draw_graphics() in
 * main.c after hysteresis.  Read by processRadarFrame() in radar.c to
 * drive the MOIS CAN state when g_fcw_enable == 1.
 *
 *   g_fcw_coll_for_can : 1 = collision zone occupied (confirmed track inside
 *                             near trapezoid zone)
 *   g_fcw_info_for_can : 1 = info zone occupied (confirmed track inside far
 *                             trapezoid zone, only when collision is inactive)
 *
 * Priority enforced in main.c: collision takes precedence — if coll is 1,
 * info is forced to 0 before these globals are written.
 *
 * MOIS CAN mapping when g_fcw_enable == 1:
 *   coll == 1              → MOIS_COLLISION
 *   coll == 0, info == 1  → MOIS_INFO
 *   coll == 0, info == 0  → MOIS_NORMAL
 */
extern int g_fcw_coll_for_can;
extern int g_fcw_info_for_can;

extern int can_tx_sock;
extern struct sockaddr_can can_tx_addr;

#define VEHICLE_DATA_CAN_ID 0x08FEBFE8
#define DATA_CAN_ID 0x0CFD18E8
#define AEB_CAN_ID 0x04FD18E8 /* AEB status frame */
#define SPEED_HOLD_TH 10
#define SPEED_RESUME_TH 8
#define CAN_DATA_TIME_DELAY 100

#define MAX_RADAR_POINTS 500

int send_can(uint32_t can_id, uint8_t dlc, uint8_t *data);

// Buffer for storing parsed radar points (main.c can use this)
extern RadarData mois_arr[MAX_RADAR_POINTS];
extern RadarData bsis_arr[MAX_RADAR_POINTS];
extern RadarData mois_display_arr[MAX_RADAR_POINTS];
extern RadarData bsis_display_arr[MAX_RADAR_POINTS];
extern int numBSISRadarPoints;
extern int numMOISRadarPoints;
extern FILE *radarPipe;
/* Returns the physical mount position (vehicle-frame x/y) for a given
 * radar. Implemented differently per build:
 *   - replay app: radar_mounts.c, static gRadarMounts[] array, 1-based radar_idx
 *   - vision-apps: main.c, gAppObj.radar_pos[][], 0-based radar_idx
 * Returns false (and leaves mx/my untouched by the caller's own defaults)
 * if radar_idx is out of range for that build. */
bool get_radar_mount(int radar_idx, float *mx, float *my);
// Simplified hazard/object structure
typedef struct {
  bool hazard_occupied; // maybe rename later to "object_detected"
  float x;
  float y;
  float z;
  float range;
  float velocity;
} RadarObject;
// Zone definition structure
typedef struct {
  float x_min, x_max;
  float y_min, y_max;
  float z_min, z_max;
} Zone;
// MOIS zone status
typedef struct {
  bool collision; // Object in front hazard zone
  bool info;
} MOISZoneStatus;

// BSIS zone status
typedef struct {
  bool left_collision;  // Object in left warning zone
  bool left_info;       // Object in left info zone
  bool right_collision; // Object in right warning zone
  bool right_info;      // Object in right info zone
} BSISZoneStatus;

// AEB frame data — holds the signals packed into the 0x04FD18E8 CAN frame
typedef struct {
  uint8_t radar_fault;  /* AEB_RADAR_FAULT_NONE / AEB_RADAR_FAULT_ACTIVE */
  uint8_t camera_fault; /* AEB_CAMERA_FAULT_NONE / AEB_CAMERA_FAULT_ACTIVE */
  float distance_m;     /* Closest detected object distance, 0–120 m      */
  float velocity_ms;    /* Relative velocity m/s, -60 (approach) to +60   */
} AEBFrameData;
// Global radar object (one per frame)
extern RadarObject radarObject;
// Global zone status
extern MOISZoneStatus g_mois_status;
extern BSISZoneStatus g_bsis_status;
extern AEBFrameData g_aeb_frame;
// API
void resetRadar();
void updateRadar(float x, float y, float z, float range, float velocity);

// CAN transmit — call every 100 ms from main loop, independent of radar frames.
// Issues two frames per cycle:
//   send_can(DATA_CAN_ID, …) — MOIS / BSIS / LDW / DDAS
//   send_can(AEB_CAN_ID,  …) — AEB  (dedicated payload)
void sendCANStatus(void);
void setMOISState(uint16_t state);
void setBSISState(uint16_t state);
void setLDWState(uint8_t state);
uint8_t getLDWState(void);
void setDDASState(uint8_t state);
uint8_t getDDASState(void);
uint8_t getAEBRadarFault(void);        /* Get AEB radar  fault nibble      */
void setAEBRadarFault(uint8_t fault);  /* Set AEB radar  fault nibble      */
void setAEBCameraFault(uint8_t fault); /* Set AEB camera fault nibble      */

/* Update AEB frame data from the top-1 critical object produced by
 * aeb_process_frame_mois() in main.c.
 *   distance_m  : objects[0].range         (metres,  0–120 m)
 *   velocity_ms : objects[0].relative_speed (m/s, –60 approaching … +60
 * receding) The values are clamped and scaled to 0.01 resolution inside
 * sendCANStatus(). */
void updateAEBData(float distance_m, float velocity_ms);

// Processing functions
int processRadarFrame(float ego_signed_speed_ms);
void updateArrayWithTracks(void);

// Get tracking statistics
int getNumMOISTracks(void);
int getNumBSISTracks(void);
int getNumTotalTracks(void);

/* CAN TX health: 0=never sent, 1=last TX ok, -1=last TX failed */
int getCanTxStatus(void);

// Zone checking functions
void check_mois_zones(RadarData *clusters, int num_clusters);

// Override all six detection zones from parsed config (call after cfg load)
void setRadarZones(Zone mois_warn, Zone mois_info,
                   Zone bsis_lw,   Zone bsis_li,
                   Zone bsis_rw,   Zone bsis_ri);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RADAR_H */
