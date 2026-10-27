#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "fusion_common/hungarian.hpp"
#include "fusion_common/object_classes.hpp"
#include "sensor_fusion/byte_tracker.hpp"
#include "sensor_fusion/fusion_core.hpp"
#include "sensor_fusion/imm_tracker.hpp"

using namespace sensor_fusion;

namespace
{
/// Fusion configuration plus the IMM tracker's, as sensor_fusion.yaml holds them.
struct TestConfig : FusionConfig
{
  TrackerConfig tracker;
  RadarPreprocessorConfig radar;
  ByteTrackerConfig camera_tracker;
};

/// IMM tracker -> fusion, ByteTrack -> fusion, wired like sensor_fusion_node (radar.tracker imm).
class Pipeline
{
public:
  explicit Pipeline(const TestConfig & c)
  : core_(fusionConfig(c)), imm_(immConfig(c)), boxes_(c.camera_tracker) {}

  void setCamera(const fusion_common::CameraModel & cam) {core_.setCamera(cam);}
  void addEgo(const EgoSample & s) {core_.addEgo(s); imm_.addEgo(s);}
  void processRadar(Time t, const std::vector<RadarDetection> & dets)
  {
    double ego_speed = 0;
    const auto tracks = imm_.process(t, dets, &ego_speed);
    core_.processRadarTracks(t, tracks, ego_speed, "imm");
  }
  void processRadarTracks(Time t, const std::vector<ExternalRadarTrack> & tracks, double ego_speed)
  {
    core_.processRadarTracks(t, tracks, ego_speed, "ekf");
  }
  void processCamera(const CameraFrame & f) {core_.processCamera(trackBoxes(boxes_, f, &camera_tracks_));}
  std::vector<FusedTrack> objects() const {return core_.objects();}
  std::vector<FusedTrack> confirmedTracks() const {return core_.confirmedTracks();}
  const std::vector<TrackedBox> & cameraTracks() const {return camera_tracks_;}
  double stopZoneLength() const {return core_.stopZoneLength();}

private:
  static FusionConfig fusionConfig(const TestConfig & c)
  {
    FusionConfig f = c;
    f.external.moving_speed_threshold = c.tracker.moving_speed_threshold;
    f.external.moving_min_sigma = c.tracker.moving_min_sigma;
    f.external.motion_alpha = c.tracker.motion_alpha;
    f.external.motion_enter = c.tracker.motion_enter;
    f.external.motion_exit = c.tracker.motion_exit;
    f.external.ground_check = c.tracker.ground_check;
    f.external.ground_z_threshold = c.tracker.ground_z_threshold;
    return f;
  }
  static ImmTrackerConfig immConfig(const TestConfig & c)
  {
    ImmTrackerConfig i;
    i.tracker = c.tracker;
    i.radar = c.radar;
    i.ego_speed_source = c.ego_speed_source;
    i.mount = c.fusion_frame_mount;
    return i;
  }
  FusionCore core_;
  ImmTracker imm_;
  ByteTracker boxes_;
  std::vector<TrackedBox> camera_tracks_;
};
}  // namespace

// Minimum-cost assignment on a 3x2 matrix.
TEST(Hungarian, SimpleAssignment)
{
  Eigen::MatrixXd c(3, 2);
  c << 1, 10, 10, 1, 5, 5;
  auto a = fusion_common::hungarian(c, 8.0);
  EXPECT_EQ(a[0], 0);
  EXPECT_EQ(a[1], 1);
  EXPECT_EQ(a[2], -1);
}

// Driving straight moves a static point backwards in the sensor frame by v * dt.
TEST(EgoMotion, StraightDriveMovesStaticPointBack)
{
  EgoMotion e;
  e.add({0.0, 2.0, 0.0});
  e.add({1.0, 2.0, 0.0});
  Eigen::Matrix2d R;
  Eigen::Vector2d t;
  e.frameTransform(0.0, 1.0, R, t);
  const Eigen::Vector2d p = R * Eigen::Vector2d(10, 0) + t;
  EXPECT_NEAR(p.x(), 8.0, 1e-6);
  EXPECT_NEAR(p.y(), 0.0, 1e-6);
}

// The over-ground Doppler of a static target is ~0 after ego compensation.
TEST(Radar, DopplerCompensationStaticTarget)
{
  RadarDetection d;
  d.x = 10; d.y = 5;
  const double az = std::atan2(d.y, d.x);
  d.radial_speed = -2.0 * std::cos(az);           // vehicle at 2 m/s towards a static target
  EXPECT_NEAR(RadarPreprocessor::groundDoppler(d, Eigen::Vector2d(2.0, 0.0)), 0.0, 1e-9);
}

// The IMM follows a crossing target and gives the CT model the highest probability in a turn.
TEST(Imm, TracksCrossingTargetAndPrefersCtInTurn)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 0.1);
  // target: straight 3 s then turning at 0.4 rad/s, speed 3 m/s; ego static.
  // 100 cycles keeps it inside the radar field of view (|az| < 1 rad).
  double x = 15, y = -6, vx = 0, vy = 3, w = 0;
  const double dt = 0.066;
  for (int k = 0; k < 100; ++k) {
    const double t = k * dt;
    if (t > 3.0) {w = 0.4;}
    const double nvx = vx * std::cos(w * dt) - vy * std::sin(w * dt);
    const double nvy = vx * std::sin(w * dt) + vy * std::cos(w * dt);
    vx = nvx; vy = nvy;
    x += vx * dt; y += vy * dt;
    RadarDetection d;
    d.x = x + n(rng); d.y = y + n(rng);
    const double r = std::hypot(x, y);
    d.radial_speed = (x * vx + y * vy) / r;
    d.range = std::hypot(d.x, d.y);
    d.rcs = 10;
    core.addEgo({t, 0.0, 0.0});
    core.processRadar(t, {d});
  }
  auto tr = core.confirmedTracks();
  ASSERT_EQ(tr.size(), 1u);
  EXPECT_NEAR(tr[0].pos.x(), x, 0.5);
  EXPECT_NEAR(tr[0].pos.y(), y, 0.5);
  EXPECT_NEAR(tr[0].vel.norm(), 3.0, 0.5);
  EXPECT_GT(tr[0].model_probs[2], tr[0].model_probs[1]);   // CT (st, cv, ct) dominates while turning
  EXPECT_NEAR(tr[0].yaw_rate, 0.4, 0.2);
  EXPECT_EQ(tr[0].motion_state, "turning");
}

// Final list: every object in front within range, in_path only an attribute.
TEST(Output, FinalListAllObjectsInRange)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.output_max_range = 40.0;
  cfg.path.curvature_source = "odom";
  Pipeline core(cfg);
  for (int k = 0; k < 15; ++k) {
    std::vector<RadarDetection> dets;
    for (int i = 0; i < 8; ++i) {   // static, 6 .. 55 m, 2.5 m left / right of the path
      RadarDetection d;
      d.x = 6.0 + 7.0 * i;
      d.y = (i % 2) ? 2.5 : -2.5;
      d.range = std::hypot(d.x, d.y);
      d.rcs = 10;
      dets.push_back(d);
    }
    RadarDetection c;   // static object right ahead
    c.x = 10.0; c.y = 0.3; c.range = std::hypot(c.x, c.y); c.rcs = 10;
    dets.push_back(c);
    core.addEgo({k * 0.066, 0.0, 0.0});
    core.processRadar(k * 0.066, dets);
  }
  const auto all = core.objects();
  ASSERT_EQ(all.size(), 6u);   // 6, 10, 13, 20, 27, 34 m (41+ m cut), off-path ones included
  for (std::size_t i = 1; i < all.size(); ++i) {EXPECT_LE(all[i - 1].range(), all[i].range());}
  int in_path = 0;
  for (const auto & o : all) {
    EXPECT_EQ(o.motion_state, "stationary");
    EXPECT_GE(o.existence, cfg.output_min_existence);
    in_path += o.in_path;
  }
  EXPECT_EQ(in_path, 1);
}

// Relative-motion labels use object and ego over-ground velocities. In particular,
// a slower lead vehicle remains "same_direction" while closing_speed tells the
// planner whether the gap is shrinking; a static obstacle approached by ego is
// correctly "approaching", not merely "stationary".
TEST(Output, RelativeMotionAndPrediction)
{
  auto run = [](double ego_speed, double object_speed) {
      TestConfig cfg;
      cfg.ego_speed_source = "odom";
      cfg.relative_motion.prediction_horizon = 0.5;
      Pipeline core(cfg);
      constexpr double dt = 0.066;
      double relative_x = 15.0;
      for (int k = 0; k < 25; ++k) {
        const double time = k * dt;
        core.addEgo({time, ego_speed, 0.0});
        RadarDetection d;
        d.x = relative_x;
        d.range = relative_x;
        d.radial_speed = object_speed - ego_speed;
        d.rcs = 10.0;
        core.processRadar(time, {d});
        relative_x += (object_speed - ego_speed) * dt;
      }
      const auto objects = core.objects();
      EXPECT_EQ(objects.size(), 1u);
      return objects.at(0);
    };

  const auto following = run(2.0, 1.0);
  EXPECT_EQ(following.relative_motion_state, "same_direction");
  EXPECT_GT(following.closing_speed, 0.5);
  EXPECT_GT(following.time_to_collision, 0.0);
  // predicted_pos is over ground in the current frame: the lead keeps driving forward
  EXPECT_GT(following.predicted_pos.x(), following.pos.x());

  const auto oncoming = run(1.0, -2.0);
  EXPECT_EQ(oncoming.relative_motion_state, "approaching");
  EXPECT_GT(oncoming.closing_speed, 2.0);
  EXPECT_LT(oncoming.predicted_pos.x(), oncoming.pos.x());

  const auto receding = run(0.0, 2.0);
  EXPECT_EQ(receding.relative_motion_state, "moving_away");
  EXPECT_LT(receding.closing_speed, -1.0);
  EXPECT_EQ(receding.time_to_collision, -1.0);
  EXPECT_GT(receding.predicted_pos.x(), receding.pos.x());

  const auto static_ahead = run(2.0, 0.0);
  EXPECT_EQ(static_ahead.relative_motion_state, "approaching");
  EXPECT_FALSE(static_ahead.is_moving);
  EXPECT_GT(static_ahead.closing_speed, 1.0);
  // a stationary object does not move over ground (the old prediction moved it with the ego)
  EXPECT_NEAR((static_ahead.predicted_pos - static_ahead.pos).norm(), 0.0, 1e-6);
}

// Stop zone: corridor from the radar to min_distance + v_ego * headway ahead. Objects are
// predicted with constant velocity (ego holding its speed) to find their entry time.
TEST(Output, StopZoneEntry)
{
  // object at (x, y) moving (vx, vy) over ground, ego driving straight at ego_speed
  auto run = [](double ego_speed, double x, double y, double vx, double vy) {
      TestConfig cfg;
      cfg.ego_speed_source = "odom";
      cfg.path.curvature_source = "odom";
      cfg.stop_zone.min_distance = 3.0;
      cfg.stop_zone.time_headway = 1.5;
      cfg.stop_zone.horizon = 3.0;
      Pipeline core(cfg);
      constexpr double dt = 0.066;
      for (int k = 0; k < 25; ++k) {
        const double time = k * dt;
        core.addEgo({time, ego_speed, 0.0});
        RadarDetection d;
        d.x = x;
        d.y = y;
        d.range = std::hypot(x, y);
        d.azimuth = std::atan2(y, x);
        // relative radial speed = (v_obj - v_ego) . p / |p|
        d.radial_speed = ((vx - ego_speed) * x + vy * y) / d.range;
        d.rcs = 10.0;
        core.processRadar(time, {d});
        x += (vx - ego_speed) * dt;
        y += vy * dt;
      }
      const auto objects = core.objects();
      EXPECT_EQ(objects.size(), 1u);
      return std::make_pair(objects.at(0), core.stopZoneLength());
    };

  // standstill, static object 2 m ahead: inside the 3 m zone now
  const auto [inside, l0] = run(0.0, 2.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(l0, 3.0, 1e-6);
  EXPECT_TRUE(inside.in_stop_zone);
  EXPECT_EQ(inside.time_to_stop_zone, 0.0);

  // 2 m/s towards a static object (~11.7 m ahead at the end): zone 6 m, entry after ~2.9 s
  const auto [static_ahead, l1] = run(2.0, 15.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(l1, 6.0, 0.3);
  EXPECT_FALSE(static_ahead.in_stop_zone);
  EXPECT_GT(static_ahead.time_to_stop_zone, 2.0);
  EXPECT_LT(static_ahead.time_to_stop_zone, 3.01);

  // lead object moving away faster than the ego: never enters
  const auto [receding, l2] = run(1.0, 5.0, 0.0, 3.0, 0.0);
  EXPECT_EQ(receding.time_to_stop_zone, -1.0);

  // ego at 1 m/s (zone 4.5 m); pedestrian crossing from the left at 1.5 m/s, ends ~(4.4, 2.5):
  // reaches the corridor edge (0.9 m + half width) after ~0.75 s, not in it now
  const auto [crossing, l3] = run(1.0, 6.0, 5.0, 0.0, -1.5);
  EXPECT_NEAR(l3, 4.5, 0.3);
  EXPECT_FALSE(crossing.in_stop_zone);
  EXPECT_GT(crossing.time_to_stop_zone, 0.3);
  EXPECT_LT(crossing.time_to_stop_zone, 2.0);
}

// Double bounce: a return on the bearing of a real object at 2x range and 2x radial speed.
TEST(Tracker, DoubleBounceGhostLeavesFinalList)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  for (int k = 0; k < 20; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    const double r = 8.0 - 2.0 * t;   // real object approaching at 2 m/s
    RadarDetection a, g;
    a.x = r; a.radial_speed = -2.0; a.range = r; a.rcs = 10;
    g.x = 2 * r; g.radial_speed = -4.0; g.range = 2 * r; g.rcs = 0;
    core.processRadar(t, {a, g});
  }
  int ghosts = 0;
  for (const auto & t : core.confirmedTracks()) {ghosts += t.ghost;}
  EXPECT_EQ(ghosts, 1);
  const auto obj = core.objects();
  ASSERT_EQ(obj.size(), 1u);
  EXPECT_NEAR(obj[0].range(), 8.0 - 2.0 * 19 * 0.066, 0.5);
  EXPECT_EQ(obj[0].motion_state, "moving");
}

// Slow ego (3 km/h): a static object at twice the range of another one on the same bearing is
// not a ghost (the Doppler cannot tell 2x from 1x at this speed).
TEST(Tracker, NoGhostAtLowRelativeSpeed)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  const double v = 0.83;
  for (int k = 0; k < 20; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, v, 0.0});
    RadarDetection a, b;
    a.x = 8.0 - v * t; a.radial_speed = -v; a.range = a.x; a.rcs = 10;
    b.x = 2 * a.x; b.radial_speed = -v; b.range = b.x; b.rcs = 10;
    core.processRadar(t, {a, b});
  }
  for (const auto & t : core.confirmedTracks()) {EXPECT_FALSE(t.ghost);}
  EXPECT_EQ(core.objects().size(), 2u);
}

// The EKF tracker's MirrorDetector (tracker.mirror_check): at speed, a return at 3x the range and 3x the
// relative radial speed of a static reflector (the double-bounce check only knows 2x) on the same lateral position is a ghost and leaves
// the object list; the reflector stays.
TEST(Tracker, MirrorGhostAtSpeedIsDropped)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  const double v = 10.0;
  for (int k = 0; k < 10; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, v, 0.0});
    RadarDetection a, b;
    a.x = 15.0 - v * t; a.radial_speed = -v; a.range = a.x; a.rcs = 10;
    b.x = 3 * a.x; b.radial_speed = -3 * v; b.range = b.x; b.rcs = 10;
    core.processRadar(t, {a, b});
  }
  int ghosts = 0;
  for (const auto & t : core.confirmedTracks()) {ghosts += t.ghost ? 1 : 0;}
  EXPECT_EQ(ghosts, 1);
  ASSERT_EQ(core.objects().size(), 1u);
  EXPECT_LT(core.objects()[0].pos.x(), 12.0);   // the reflector, not the ghost
}

// radar.tracker ekf: EKF tracker tracks become the radar tracks (no IMM): tracker "ekf", its id,
// no IMM mode probabilities / turn rate; radar.tracker imm keeps the IMM outputs.
TEST(Tracker, EkfTracksHaveEkfDefaults)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  for (int k = 0; k < 5; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    ExternalRadarTrack e;
    e.id = 42; e.px = 10.0; e.py = 0.5; e.vx = 1.5; e.existence = 0.9; e.hits = 10; e.age = 10;
    e.P = Eigen::Matrix4d::Identity() * 0.05;
    core.processRadarTracks(t, {e}, 0.0);
  }
  const auto objs = core.objects();   // returned by value: keep it alive
  ASSERT_EQ(objs.size(), 1u);
  const auto & o = objs[0];
  EXPECT_EQ(o.tracker, "ekf");
  EXPECT_EQ(o.ekf_track_id, 42);
  EXPECT_TRUE(o.model_probs.empty());
  EXPECT_EQ(o.yaw_rate, 0.0);
  EXPECT_NE(o.motion_state, "turning");

  Pipeline imm(cfg);
  // a stationary track is listed once ground_min_samples (8) detection heights are known
  for (int k = 0; k < 10; ++k) {
    const double t = k * 0.066;
    imm.addEgo({t, 0.0, 0.0});
    RadarDetection d;
    d.x = 10.0; d.range = 10.0; d.rcs = 10;
    imm.processRadar(t, {d});
  }
  ASSERT_FALSE(imm.objects().empty());
  EXPECT_EQ(imm.objects()[0].tracker, "imm");
  EXPECT_EQ(imm.objects()[0].ekf_track_id, -1);
  EXPECT_EQ(imm.objects()[0].model_probs.size(), 3u);
}

// bag_record_3 track 7124: a strong static return measured ~0.1 m above the ground (drain cover)
// ran into the stop zone. Stationary tracks averaging below ground_z_threshold are not listed;
// a person-height static return and a low MOVING target stay.
TEST(Output, GroundLevelStaticTrackNotListed)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.radar.radar_height = 0.41;
  Pipeline core(cfg);
  for (int k = 0; k < 30; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    RadarDetection cover, person, mover;
    cover.x = 3.0; cover.y = 0.0; cover.z = -0.31; cover.range = 3.0; cover.rcs = 8;    // 0.10 m above ground, strong
    person.x = 5.0; person.y = 2.0; person.z = 0.4; person.range = std::hypot(5.0, 2.0); person.rcs = 0;
    mover.x = 12.0 - 1.5 * t; mover.y = -3.0; mover.z = -0.31; mover.range = std::hypot(mover.x, mover.y);
    mover.radial_speed = -1.5 * mover.x / mover.range; mover.rcs = 5;                    // low but moving
    core.processRadar(t, {cover, person, mover});
  }
  bool cover_listed = false, person_listed = false, mover_listed = false;
  for (const auto & o : core.objects()) {
    if (std::abs(o.pos.x() - 3.0) < 0.5 && std::abs(o.pos.y()) < 0.5) {cover_listed = true;}
    if (std::abs(o.pos.y() - 2.0) < 0.5) {person_listed = true;}
    if (std::abs(o.pos.y() + 3.0) < 0.5) {mover_listed = true;}
  }
  EXPECT_FALSE(cover_listed);
  EXPECT_TRUE(person_listed);
  EXPECT_TRUE(mover_listed);
}

// A weak target (pedestrian) that misses two scans stays in the final list (existence
// hysteresis: enter at min_existence, leave below keep_existence); after four misses it leaves.
TEST(Output, WeakTargetSurvivesMissedScans)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  Pipeline core(cfg);
  RadarDetection p;
  p.x = 10.0; p.y = 0.5; p.range = std::hypot(p.x, p.y); p.rcs = -5;
  int k = 0;
  auto scan = [&](bool hit) {
      const double t = k++ * 0.066;
      core.addEgo({t, 0.0, 0.0});
      core.processRadar(t, hit ? std::vector<RadarDetection>{p} : std::vector<RadarDetection>{});
    };
  for (int i = 0; i < 10; ++i) {scan(true);}
  ASSERT_EQ(core.objects().size(), 1u);
  scan(false);
  scan(false);
  ASSERT_EQ(core.objects().size(), 1u);
  EXPECT_LT(core.objects()[0].existence, cfg.output_min_existence);   // below the entry level
  scan(false);
  scan(false);
  EXPECT_TRUE(core.objects().empty());
}

// Bicycle-model arc from the steering angle: objects on / beside the arc.
TEST(Path, SteeringArc)
{
  PathConfig pc;
  pc.curvature_source = "steering";
  pc.wheelbase = 2.0;
  pc.steering_in_degrees = true;
  pc.half_width = 1.0;
  pc.smoothing = 1.0;   // single sample
  PathPredictor p(pc, MountPose2D{});
  p.addSteering(std::atan(2.0 / 10.0) * 180.0 / M_PI);   // R = 10 m to the left
  EXPECT_NEAR(p.curvature(), 0.1, 1e-6);
  // quarter circle: (10, 10) is on the path after 15.7 m
  const auto on = p.check({10.0, 10.0}, 0.0);
  EXPECT_TRUE(on.in_path);
  EXPECT_NEAR(on.distance, M_PI * 5.0, 1e-3);
  EXPECT_NEAR(on.lateral, 0.0, 1e-6);
  EXPECT_FALSE(p.check({10.0, 0.0}, 0.0).in_path);   // straight ahead is off a tight left turn
  EXPECT_FALSE(p.check({-3.0, 0.0}, 0.0).in_path);   // behind
  EXPECT_NEAR(p.border(1, 1.0).front().y(), 1.0, 1e-9);
}

// Radar with a 0.6 m lateral bias + exact camera boxes of a car: the camera cycle must
// classify the track, build a box and pull the bearing towards the camera (radar keeps
// at least radar_min_weight of it).
TEST(CameraFusion, ClassifiesAndCorrectsBearing)
{
  fusion_common::CameraCalibration c;
  c.width = 960; c.height = 540;
  c.fx = c.fy = 500; c.cx = 480; c.cy = 270;
  c.lens = fusion_common::LensModel::PINHOLE;
  c.R << 0, -1, 0,
    0, 0, -1,
    1, 0, 0;                      // radar axes -> optical axes, camera at the radar origin
  const fusion_common::CameraModel cam(c);

  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.camera.rectified = false;
  cfg.camera.radar_height_above_ground = 0.5;
  Pipeline core(cfg);
  core.setCamera(cam);

  const double x = 15.0, y_true = 1.0;
  const auto & car = fusion_common::classInfo("car");
  for (int k = 0; k < 60; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    RadarDetection d;
    d.x = x; d.y = y_true + 0.6; d.range = std::hypot(d.x, d.y); d.rcs = 10;
    core.processRadar(t, {d});
    // car box: centre half a length behind the radar return, on the ground
    double u0 = 1e9, v0 = 1e9, u1 = -1e9, v1 = -1e9;
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3d p(x + car.length * ((i & 1) ? 1.0 : 0.0), y_true + ((i & 2) ? 0.5 : -0.5) * car.width,
        -0.5 + ((i & 4) ? car.height : 0.0));
      const auto uv = cam.projectRadarPoint(p, 1e4);
      ASSERT_TRUE(uv);
      u0 = std::min(u0, uv->x()); v0 = std::min(v0, uv->y());
      u1 = std::max(u1, uv->x()); v1 = std::max(v1, uv->y());
    }
    CameraFrame f;
    f.stamp = t + 0.033;
    f.width = 960; f.height = 540;
    f.detections.push_back(CameraDetection{0, 2, "car", 0.8f, static_cast<float>(u0), static_cast<float>(v0),
        static_cast<float>(u1), static_cast<float>(v1)});
    core.processCamera(f);
  }
  const auto tr = core.confirmedTracks();
  ASSERT_EQ(tr.size(), 1u);
  EXPECT_EQ(tr[0].class_name, "car");
  EXPECT_GT(tr[0].class_confidence, 0.9);
  EXPECT_TRUE(tr[0].camera_associated);
  EXPECT_EQ(tr[0].camera_track_id, core.cameraTracks().at(0).track_id);   // 2D track from ByteTrack
  EXPECT_TRUE(tr[0].in_path);   // 1 m left of a straight path
  EXPECT_EQ(tr[0].status, "radar+camera");
  EXPECT_GT(tr[0].assoc_prob, 0.5);
  EXPECT_TRUE(tr[0].has_bbox);
  ASSERT_EQ(tr[0].class_probs.size(), fusion_common::fusedClasses().size());
  // bearing pulled from the biased radar (1.6 m) towards the camera (1.0 m); a confident radar
  // track keeps most of the weight (Bayes), and never less than radar_min_weight (60 %)
  EXPECT_LT(tr[0].pos.y(), y_true + 0.6 - 0.02);
  EXPECT_GT(tr[0].pos.y(), y_true + 0.6 * 0.6 - 0.05);
}

// Ground clutter, below-ground and overhead returns are removed; a standing pedestrian is kept.
TEST(Radar, GroundAndOverheadFilter)
{
  RadarPreprocessorConfig c;
  c.radar_height = 0.4;
  RadarPreprocessor pre(c);
  auto det = [](double x, double z, double rcs) {
      RadarDetection d;
      d.x = x; d.z = z; d.rcs = rcs; d.range = std::hypot(x, z);
      return d;
    };
  EXPECT_TRUE(pre.isGroundOrOverhead(det(5, -1.0, 10), 0.0));    // 0.6 m below the ground
  EXPECT_FALSE(pre.isGroundOrOverhead(det(30, -1.0, 10), 0.0));  // within elevation noise at 30 m
  EXPECT_TRUE(pre.isGroundOrOverhead(det(5, -0.3, -5), 0.0));    // weak static return at ground level
  EXPECT_FALSE(pre.isGroundOrOverhead(det(5, -0.3, -5), 2.0));   // ... unless it moves
  EXPECT_FALSE(pre.isGroundOrOverhead(det(5, 0.5, -5), 0.0));    // standing pedestrian (torso)
  EXPECT_TRUE(pre.isGroundOrOverhead(det(5, 4.0, 10), 0.0));     // overhead
}

/**
 * @brief Pinhole test camera (960 x 540) looking forward from the radar.
 * @return camera model
 */
static fusion_common::CameraModel testCamera()
{
  fusion_common::CameraCalibration c;
  c.width = 960; c.height = 540;
  c.fx = c.fy = 500; c.cx = 480; c.cy = 270;
  c.lens = fusion_common::LensModel::PINHOLE;
  c.R << 0, -1, 0,
    0, 0, -1,
    1, 0, 0;
  return fusion_common::CameraModel(c);
}

// Camera boxes alone never create fused objects; one box goes to at most one radar track.
TEST(CameraFusion, RadarLedOneBoxPerTrack)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.camera.rectified = false;
  Pipeline core(cfg);
  core.setCamera(testCamera());
  // person box at ~10 m straight ahead (1.75 m tall from the ground at -0.5 m)
  const float top = 270 - 500 * 1.25 / 10.0, bottom = 270 + 500 * 0.5 / 10.0;
  const CameraDetection person{0, 0, "person", 0.9f, 465, top, 495, bottom};
  for (int k = 0; k < 20; ++k) {   // camera only
    core.processCamera(CameraFrame{k * 0.033, 960, 540, {person}});
  }
  EXPECT_TRUE(core.objects().empty());
  EXPECT_FALSE(core.cameraTracks().empty());

  // two radar tracks inside the same box (10.0 and 10.4 m)
  for (int k = 0; k < 20; ++k) {
    const double t = 1.0 + k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    RadarDetection a, b;
    a.x = 10.0; a.y = 0.0; a.range = 10.0; a.rcs = 5;
    b.x = 11.6; b.y = 0.2; b.range = std::hypot(b.x, b.y); b.rcs = 5;
    core.processRadar(t, {a, b});
    core.processCamera(CameraFrame{t + 0.01, 960, 540, {person}});
  }
  const auto obj = core.objects();
  ASSERT_EQ(obj.size(), 2u);
  int fused = 0;
  for (const auto & o : obj) {fused += o.status == "radar+camera";}
  EXPECT_EQ(fused, 1);
  EXPECT_EQ(obj[0].status, "radar+camera");   // the closer one (range preference)
  EXPECT_EQ(obj[0].class_name, "pedestrian");
  EXPECT_EQ(obj[1].class_name, "unknown");
}

// bag_record_3, t 143 s: a strong ground-level return (kerb) at 4 m in front of a pedestrian at
// 6.5 m took the pedestrian box (nearest wins) and kept "pedestrian" for good. Now a ground-level
// track gets no camera box (min_track_height) and a class fades after class_keep_s without camera.
TEST(CameraFusion, GroundReturnGetsNoPersonBoxAndClassFades)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.camera.rectified = false;
  cfg.camera.radar_height_above_ground = 0.5;
  cfg.radar.radar_height = 0.5;
  Pipeline core(cfg);
  core.setCamera(testCamera());
  // person box at 6.5 m straight ahead (1.75 m tall, ground at -0.5 m)
  const float top = 270 - 500 * 1.25 / 6.5, bottom = 270 + 500 * 0.5 / 6.5;
  const CameraDetection person{0, 0, "person", 0.9f, 470, top, 490, bottom};
  double t = 0;
  for (int k = 0; k < 30; ++k) {
    t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    RadarDetection kerb, ped;
    kerb.x = 4.0; kerb.z = -0.45; kerb.range = 4.0; kerb.rcs = 5;   // 0.05 m above ground, strong
    ped.x = 6.5; ped.z = 0.4; ped.range = 6.5; ped.rcs = 0;         // 0.9 m above ground
    core.processRadar(t, {kerb, ped});
    core.processCamera(CameraFrame{t + 0.01, 960, 540, {person}});
  }
  // the kerb is still tracked, without the person box (min_track_height) ...
  bool kerb_tracked = false;
  for (const auto & tr : core.confirmedTracks()) {
    if (std::abs(tr.pos.x() - 4.0) < 0.3) {
      kerb_tracked = true;
      EXPECT_EQ(tr.class_name, "unknown");
      EXPECT_EQ(tr.status, "radar");
    }
  }
  EXPECT_TRUE(kerb_tracked);
  // ... but as stationary ground clutter it is not in the object list (tracker.ground_check)
  auto obj = core.objects();
  ASSERT_EQ(obj.size(), 1u);
  EXPECT_NEAR(obj[0].pos.x(), 6.5, 0.3);
  EXPECT_EQ(obj[0].class_name, "pedestrian");   // the person keeps the box
  // camera loses the person: after class_keep_s the label is gone again
  for (int k = 1; k <= 25; ++k) {
    const double tk = t + k * 0.066;
    core.addEgo({tk, 0.0, 0.0});
    RadarDetection ped;
    ped.x = 6.5; ped.z = 0.4; ped.range = 6.5; ped.rcs = 0;
    core.processRadar(tk, {ped});
  }
  obj = core.objects();
  ASSERT_FALSE(obj.empty());
  EXPECT_EQ(obj.back().class_name, "unknown");
}

// A camera frame older than the last radar scan is still fused (tracks predicted back to
// its stamp); the camera never moves the tracker.
TEST(CameraFusion, LateFrameIsFusedWithoutWaiting)
{
  TestConfig cfg;
  cfg.ego_speed_source = "odom";
  cfg.camera.rectified = false;
  Pipeline core(cfg);
  core.setCamera(testCamera());
  const float top = 270 - 500 * 1.25 / 10.0, bottom = 270 + 500 * 0.5 / 10.0;
  const CameraDetection person{0, 0, "person", 0.9f, 465, top, 495, bottom};
  for (int k = 0; k < 20; ++k) {
    const double t = k * 0.066;
    core.addEgo({t, 0.0, 0.0});
    RadarDetection a;
    a.x = 10.0; a.range = 10.0; a.rcs = 5;
    core.processRadar(t, {a});
    core.processCamera(CameraFrame{t - 0.05, 960, 540, {person}});   // 50 ms older than the scan
  }
  const auto obj = core.objects();
  ASSERT_EQ(obj.size(), 1u);
  EXPECT_EQ(obj[0].status, "radar+camera");
  EXPECT_EQ(obj[0].class_name, "pedestrian");
}
