// Mission state machine: stops one after the other, wait at each stop, loop, abort, resume.
#include <gtest/gtest.h>

#include "buggy_core/control/mission_core.hpp"

using namespace buggy_core;

namespace
{
struct Rig
{
  MissionCore core;
  double t = 100.0;
  std::string mode = "stop", routing = "set", msg;
  std::vector<std::string> goals;
  int autos = 0, stops = 0;
  std::optional<std::pair<double, double>> pose;

  explicit Rig(bool loop = true, bool auto_restart = true, size_t n = 3, bool start_as_home = false)
  : core([&] {
        MissionParams p;
        p.loop = loop; p.auto_restart = auto_restart; p.start_as_home = start_as_home;
        const char * names[] = {"home", "A", "B"};
        for (size_t i = 0; i < n; ++i) {p.stops.push_back({names[i], 10.0 * i, 0.0, 10.0});}
        return p;
      }()) {}

  // one 0.1 s step; new_path = the route planner answered
  void tick(bool new_path = false)
  {
    t += 0.1;
    MissionInput in;
    in.now = t; in.mode = mode; in.routing = routing; in.msg = msg;
    in.state_fresh = true; in.new_path = new_path; in.pose = pose;
    const auto a = core.step(in);
    if (a.send_goal) {goals.push_back(a.send_goal->name);}
    if (a.request_auto) {++autos; mode = "auto";}      // the operation manager accepts AUTO
    if (a.request_stop) {++stops; mode = "stop";}
  }
  void run(double sec) {for (int i = 0; i < static_cast<int>(sec * 10); ++i) {tick();}}
  // the planner answers: new path, state routing=set
  void route_ready() {routing = "set"; tick(true);}
  void arrive() {routing = "arrived"; mode = "stop"; tick();}
};
}  // namespace

TEST(Mission, StartsWithTheFirstStopAndDrives)
{
  Rig r;
  r.core.on_request("start");
  r.tick();
  ASSERT_EQ(r.goals.size(), 1u);
  EXPECT_EQ(r.goals[0], "home");
  EXPECT_EQ(r.core.phase(), MissionPhase::Planning);
  r.route_ready();
  EXPECT_EQ(r.core.phase(), MissionPhase::Driving);
  EXPECT_EQ(r.autos, 1);
}

TEST(Mission, WaitsTenSecondsAtTheStopThenGoesToTheNextOne)
{
  Rig r;
  r.core.on_request("start");
  r.tick(); r.route_ready();
  r.run(5.0);
  r.arrive();
  EXPECT_EQ(r.core.phase(), MissionPhase::Waiting);
  r.run(9.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Waiting) << "not before 10 s";
  EXPECT_EQ(r.goals.size(), 1u);
  r.run(1.5);
  EXPECT_EQ(r.core.phase(), MissionPhase::Planning);
  ASSERT_EQ(r.goals.size(), 2u);
  EXPECT_EQ(r.goals[1], "A");
}

TEST(Mission, LoopsBackToTheFirstStop)
{
  Rig r(true);
  r.core.on_request("start");
  r.tick();
  for (int leg = 0; leg < 4; ++leg) {r.route_ready(); r.run(1.0); r.arrive(); r.run(10.5);}
  ASSERT_GE(r.goals.size(), 5u);
  EXPECT_EQ(r.goals[0], "home"); EXPECT_EQ(r.goals[1], "A"); EXPECT_EQ(r.goals[2], "B");
  EXPECT_EQ(r.goals[3], "home"); EXPECT_EQ(r.goals[4], "A");
}

TEST(Mission, EndsAfterTheLastStopWithoutLoop)
{
  Rig r(false);
  r.core.on_request("start");
  r.tick();
  for (int leg = 0; leg < 3; ++leg) {r.route_ready(); r.run(1.0); r.arrive(); r.run(10.5);}
  EXPECT_EQ(r.core.phase(), MissionPhase::Done);
  EXPECT_EQ(r.goals.size(), 3u);
}

TEST(Mission, StaleArrivedStateIsIgnoredUntilTheNewRouteIsThere)
{
  Rig r;
  r.core.on_request("start");
  r.tick(); r.route_ready(); r.run(1.0); r.arrive(); r.run(10.5);          // now planning the next leg
  r.routing = "arrived";                                                    // state still shows the old arrival
  r.run(2.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Planning) << "no AUTO before the new path";
  EXPECT_EQ(r.autos, 1);
}

TEST(Mission, NoRouteAbortsAfterTheTimeout)
{
  Rig r;
  r.core.on_request("start");
  r.tick();
  r.run(6.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Aborted);
  EXPECT_NE(r.core.message().find("no route"), std::string::npos);
}

TEST(Mission, OperatorStopOnTheWayAbortsTheMission)
{
  Rig r;
  r.core.on_request("start");
  r.tick(); r.route_ready(); r.run(5.0);
  r.mode = "stop"; r.msg = "EMERGENCY";                                     // operator / fault
  r.run(1.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Driving) << "not for a moment";
  r.run(2.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Aborted);
  EXPECT_NE(r.core.message().find("EMERGENCY"), std::string::npos);
  r.run(20.0);
  EXPECT_EQ(r.goals.size(), 1u) << "an aborted mission never restarts by itself";
}

TEST(Mission, AutoRefusedIsRetriedThenAborted)
{
  Rig r;
  r.core.on_request("start");
  r.tick();
  r.routing = "set"; r.mode = "stop";
  r.tick(true);
  // the operation manager refuses AUTO (not armed): mode stays stop, msg says why
  r.msg = "AUTO disabled: not armed (launch with drive:=true)";
  for (int i = 0; i < 80; ++i) {r.mode = "stop"; r.tick();}
  EXPECT_EQ(r.core.phase(), MissionPhase::Aborted);
  EXPECT_NE(r.core.message().find("not armed"), std::string::npos);
  EXPECT_GE(r.autos, 3) << "the request is repeated during the grace time";
}

TEST(Mission, StopRequestStopsTheBuggy)
{
  Rig r;
  r.core.on_request("start");
  r.tick(); r.route_ready(); r.run(3.0);
  r.core.on_request("stop");
  r.tick();
  EXPECT_EQ(r.core.phase(), MissionPhase::Idle);
  EXPECT_EQ(r.stops, 1);
}

TEST(Mission, WithoutAutoRestartItWaitsForResume)
{
  Rig r(true, false);
  r.core.on_request("start");
  r.tick(); r.route_ready(); r.run(1.0); r.arrive(); r.run(11.0);
  EXPECT_EQ(r.core.phase(), MissionPhase::Paused);
  r.run(20.0);
  EXPECT_EQ(r.goals.size(), 1u);
  r.core.on_request("resume");
  r.tick();
  ASSERT_EQ(r.goals.size(), 2u);
  EXPECT_EQ(r.goals[1], "A");
}

TEST(Mission, StartingAtTheFirstStopGoesStraightToTheNext)
{
  Rig r;
  r.pose = std::make_pair(1.0, 0.5);                 // standing at "home"
  r.core.on_request("start");
  r.tick();
  ASSERT_EQ(r.goals.size(), 1u);
  EXPECT_EQ(r.goals[0], "A");
}

TEST(Mission, StartAtAGivenStop)
{
  Rig r;
  r.core.on_request("start:2");
  r.tick();
  ASSERT_EQ(r.goals.size(), 1u);
  EXPECT_EQ(r.goals[0], "B");
}

TEST(Mission, EmptyMissionIsRefused)
{
  Rig r(true, true, 0);
  r.core.on_request("start");
  r.tick();
  EXPECT_TRUE(r.goals.empty());
  EXPECT_EQ(r.core.phase(), MissionPhase::Aborted);
}

// ---- home is optional: without a "home" stop the pose set in RViz at the start is the home ----------------------
TEST(Mission, StartAsHomeAddsTheVehiclePoseAsTheLastStop)
{
  MissionParams p;
  p.start_as_home = true;
  p.stops = {{"A", 20.0, 0.0, 10.0}, {"B", 40.0, 0.0, 10.0}};
  MissionCore core(p);
  double t = 100.0;
  std::vector<std::string> goals;
  auto tick = [&](bool new_path, const std::string & routing, const std::string & mode) {
      t += 0.1;
      MissionInput in;
      in.now = t; in.mode = mode; in.routing = routing; in.state_fresh = true; in.new_path = new_path;
      in.pose = std::make_pair(3.0, 4.0);
      const auto a = core.step(in);
      if (a.send_goal) {goals.push_back(a.send_goal->name);}
    };
  core.on_request("start");
  tick(false, "unset", "stop");
  ASSERT_EQ(core.stops().size(), 3u);
  EXPECT_EQ(core.stops().back().name, "home");
  EXPECT_DOUBLE_EQ(core.stops().back().x, 3.0);
  EXPECT_DOUBLE_EQ(core.stops().back().wait_sec, 10.0);
  for (int leg = 0; leg < 4; ++leg) {
    tick(true, "set", "stop"); for (int i = 0; i < 10; ++i) {tick(false, "set", "auto");}
    tick(false, "arrived", "stop"); for (int i = 0; i < 105; ++i) {tick(false, "arrived", "stop");}
  }
  ASSERT_GE(goals.size(), 4u);
  EXPECT_EQ(goals[0], "A"); EXPECT_EQ(goals[1], "B"); EXPECT_EQ(goals[2], "home"); EXPECT_EQ(goals[3], "A");
}

TEST(Mission, WithoutStartAsHomeNoHomeIsAdded)
{
  MissionParams p;
  p.stops = {{"A", 20.0, 0.0, 10.0}, {"B", 40.0, 0.0, 10.0}};
  MissionCore core(p);
  MissionInput in;
  in.now = 1.0; in.state_fresh = true; in.mode = "stop"; in.routing = "unset"; in.pose = std::make_pair(3.0, 4.0);
  core.on_request("start");
  core.step(in);
  EXPECT_EQ(core.stops().size(), 2u);
}

TEST(Mission, AFixedHomeStopWinsOverTheStartPose)
{
  MissionParams p;
  p.start_as_home = true;
  p.stops = {{"home", 0.0, 0.0, 10.0}, {"A", 20.0, 0.0, 10.0}};
  MissionCore core(p);
  MissionInput in;
  in.now = 1.0; in.state_fresh = true; in.mode = "stop"; in.routing = "unset"; in.pose = std::make_pair(50.0, 4.0);
  core.on_request("start");
  core.step(in);
  EXPECT_EQ(core.stops().size(), 2u);
}

TEST(Mission, StartAsHomeNeedsAPose)
{
  MissionParams p;
  p.start_as_home = true;
  p.stops = {{"A", 20.0, 0.0, 10.0}};
  MissionCore core(p);
  MissionInput in;
  in.now = 1.0; in.state_fresh = true; in.mode = "stop"; in.routing = "unset";
  core.on_request("start");
  core.step(in);
  EXPECT_EQ(core.phase(), MissionPhase::Aborted);
  EXPECT_NE(core.message().find("2D Pose Estimate"), std::string::npos);
}
