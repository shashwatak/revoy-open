#include "planning/make-scenario.h"

#include <numbers>

namespace planning {

Scenario MakeDisappearingObstacleScenario(double dir, double dist,
                                          double disappear, double timeout,
                                          std::string name) {

  Scenario scenario;
  scenario.name = name;

  constexpr double DEBRIS = 0.5;

  const HookedPose start = {{0, 0}, dir, dir};
  const Point obst{dist * cos(dir), dist * sin(dir)};
  const HookedPose goal = {obst, dir, dir};

  /// this is useful while debugging the visuals, this should appear in the
  /// correct place
  const Point notInWay{dist * sin(dir), -dist * cos(dir)};

  const Entity disappearing = {{obst, 0},
                               {
                                   {-DEBRIS, -DEBRIS},
                                   {DEBRIS, -DEBRIS},
                                   {DEBRIS, DEBRIS},
                                   {-DEBRIS, DEBRIS},
                               },
                               disappear};

  const Entity notInTheWay = {{notInWay, 0},
                              {
                                  {-DEBRIS, -DEBRIS},
                                  {DEBRIS, -DEBRIS},
                                  {DEBRIS, DEBRIS},
                                  {-DEBRIS, DEBRIS},
                              }};

  scenario.entities.push_back(disappearing);
  scenario.entities.push_back(notInTheWay);
  scenario.start = start;
  scenario.goal = goal;
  scenario.timeParams.timeout = timeout;

  return scenario;
};

Scenario MakeYardScenario(const std::string &name) {

  Scenario scenario;
  scenario.name = name;

  const HookedPose start = {{0, 0}, 0, 0};
  const HookedPose goal = {{1, 0}, 0, 0};

  const Entity walls_left = {{{15, 15}, 0},
                             {
                                 {-10, -10},
                                 {-10, 10},
                                 {10, 10},
                                 {10, -10},
                             }};

  scenario.entities.push_back(walls_left);
  scenario.start = start;
  scenario.goal = goal;

  scenario.timeParams.timeout = 1e7;
  return scenario;
};

Scenario MakeTurningScenario(const std::string &name) {

  Scenario scenario;
  scenario.name = name;

  const HookedPose start = {{0, 0}, 0, 0};
  const HookedPose goal = {{15, 15}, std::numbers::pi, std::numbers::pi};

  scenario.start = start;
  scenario.goal = goal;

  scenario.timeParams.timeout = 1e7;
  return scenario;
};

} // namespace planning
