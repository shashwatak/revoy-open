#include "planning/make-scenario.h"

#include <numbers>

namespace planning {

Scenario MakeDisappearingObstacleScenario(double dir, double dist) {

  Scenario scenario;
  scenario.name = "disappearing-obstacle_" + std::to_string(dir) + "_" +
                  std::to_string(dist);

  constexpr double DISAPPEAR = 5e6;
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
                               DISAPPEAR};

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

  return scenario;
};

Scenario MakeYardScenario() {

  Scenario scenario;
  scenario.name = "yard";

  const HookedPose start = {{0, 0}, 0, 0};
  const HookedPose goal = {{20, 30}, std::numbers::pi*0.99, std::numbers::pi*0.99};

  const Entity walls_left = {{{15, 15}, 0},
                             {
                                 {-7, -7},
                                 {-7, 7},
                                 {7, 7},
                                 {7, -7},
                             }};

  scenario.entities.push_back(walls_left);
  // scenario.entities.push_back(notInTheWay);
  scenario.start = start;
  scenario.goal = goal;

  scenario.timeParams.timeout = 1e6;

  scenario.bounds = {50, 50, -50, -50};
  return scenario;
};

} // namespace planning
