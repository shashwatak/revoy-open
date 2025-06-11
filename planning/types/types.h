#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <vector>

/// needed for windows
#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

/// [-pi, pi]
double fixRadian(double value);

namespace planning {

using Point = Eigen::Vector2d;
using Footprint = std::vector<Eigen::Vector2d>;
using Footprints = std::vector<Footprint>;
using Path = std::vector<Eigen::Vector2d>;

struct HookedPose {
  Point position = {0, 0};
  double yaw = 0;
  double trailerYaw = 0;
};

struct Pose {
  Point position = {0, 0};
  double yaw = 0;
};

struct Entity {
  // fixed frame
  Pose pose;

  // TODO: footprint should be about origin, then rotated and
  // translated to get coords in fixed frame
  Footprint footprint;

  double lifetime = std::numeric_limits<double>::max();
};

using Entities = std::vector<Entity>;

struct Bounds {
  // meters
  double upperX = 50;
  double upperY = 50;
  double lowerX = -50;
  double lowerY = -50;
};

struct TimeParams {
  // TODO use units.h
  // microseconds

  // 100,000 us == 100ms == 0.1s
  double dt = 1e5;

  // using 0 sometimes causes problems
  // 1.7e15 is GMT Tuesday, November 14, 2023 10:13:20 PM
  double startTime = 1.7e15;

  // 6,000,000 us == 6,000 ms == 6 s
  double timeout = 6e6;
};

struct BodyParams {
  // meters
  // TODO uses units.h
  double revoyLength = 6;
  double revoyWidth = 2.5;
  double trailerLength = 20;
  double trailerWidth = 2.5;
};

// definition of what to run in the Simpl simulation
struct Scenario {

  // descriptive name of this scenario
  std::string name;

  // unmoving unchanging obstacles
  Footprints walls;

  // moving obstacles
  Entities entities;

  // revoy start position
  HookedPose start;

  // revoy will try to move to here
  HookedPose goal;

  // cannot plan / search outside this
  Bounds bounds;

  // start / end times, delta time
  TimeParams timeParams;

  // length, width, etc
  BodyParams bodyParams;
};

struct Results {

  // the source of these results
  Scenario scenario;

  // exceeded time bound
  bool timeout = false;

  // contacted obstacle
  bool collision = false;

  // revoy reached goal
  bool goalMet = false;

  // revoy min/max over the course of the scenario
  double minSpeed = 0;
  double maxSpeed = 0;

  // TODO: planner specific aggregate results maybe?
};

struct Controls {
  double speed = 0;
  double steer = 0;
  double duration = 0;
};

struct Graph {
  std::vector<Point> nodes;
  std::vector<std::tuple<size_t, size_t>> edges;
};

} // namespace planning
