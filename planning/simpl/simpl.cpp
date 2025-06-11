#include "planning/simpl.h"
#include "planning/add-footprint-to-grid.h"
#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <cstdint>
#include <memory>
#include <string>

namespace planning {

namespace {
/// 200 * 200 cells = 40000 cells
/// 0.1m * 200 cells means grid total diagonal is ~28m, effective range ~14m
static constexpr uint16_t NUM_CELLS = 200;
static constexpr double CELL_LENGTH = 0.1; // meters
static constexpr double GRID_OFFSET =
    CELL_LENGTH * ((float)NUM_CELLS) / 2.0; // meters
} // namespace

Simpl::Simpl(Scenario scenario)
    : scenario_(scenario), revoyEv_(scenario_.start),
      planningPipeline_(scenario_.bounds, scenario_.bodyParams),
      perception_({}, {NUM_CELLS, NUM_CELLS, CELL_LENGTH, CELL_LENGTH,
                       GRID_OFFSET, GRID_OFFSET}) {};

void Simpl::update(int64_t time) {

  // get simulated obstacle footprints from simulated observers
  const Footprints footprints = getVisibleFootprints(time);

  // insert footprints into occupancy grid
  FootprintsToOccupancyGrid(perception_.occupancy, footprints,
                            revoyEv_.getHookedPose());

  // update plan w/ latest revoy pose and occupancy grid
  const Plan plan = planningPipeline_.getNextPlan(revoyEv_.getHookedPose(),
                                                  scenario_.goal, perception_);

  const Controls &controls = plan.targetControls;

  // update revoy w/ controls
  revoyEv_.update(controls, scenario_.timeParams.dt / 1e6);

  results_.maxSpeed = std::fmax(results_.maxSpeed, controls.speed);
  results_.minSpeed = std::fmin(results_.minSpeed, controls.speed);

  results_.collision |= isColliding(time);
  results_.timeout |= isTimeout(time);
  results_.goalMet |= isGoalMet();
}

bool Simpl::isColliding(int64_t time) const {
  // Use full polygon intersection, from the occupancy grid used in
  // planning. this check fails correctly for edge cases not caught by
  // occupancy grid.
  const Footprints revoy = revoyEv_.getBody(scenario_.bodyParams);
  const Footprints obsts = getVisibleFootprints(time);
  return IsBodyCollidingAnyObstacles(revoy, obsts);
}

bool Simpl::isTimeout(int64_t time) const {
  return time > scenario_.timeParams.timeout + scenario_.timeParams.startTime;
}

bool Simpl::isGoalMet() const {
  // goal is met, scenario is complete
  // TODO: use GoalRegion comparison
  static const double GOAL_TOLERANCE = 0.1;
  return (revoyEv_.getHookedPose().position - scenario_.goal.position).norm() <=
             GOAL_TOLERANCE &&
         fabs(fixRadian(revoyEv_.getHookedPose().yaw - scenario_.goal.yaw)) <=
             GOAL_TOLERANCE &&
         fabs(fixRadian(revoyEv_.getHookedPose().trailerYaw -
                        scenario_.goal.trailerYaw)) <= GOAL_TOLERANCE;
}

bool Simpl::isDone(int64_t time) const {
  return isTimeout(time) || isGoalMet() ||
         (time > scenario_.timeParams.startTime && !getPlanningPipeline()
                                                        .getCoarsePlanner()
                                                        ->getSetup()
                                                        .haveSolutionPath());
}

const Footprints Simpl::getVisibleFootprints(int64_t time) const {
  Footprints footprints;
  for (const auto &entity : scenario_.entities) {
    if (static_cast<double>(time) >
        scenario_.timeParams.startTime + entity.lifetime) {
      continue;
    }

    const auto tfFootprint = TransformFootprint(entity.footprint, entity.pose);
    footprints.push_back(tfFootprint);
  }
  return footprints;
}

const Perception &Simpl::getLastPerception() const { return perception_; }

const Scenario &Simpl::getScenario() const { return scenario_; }
const Results &Simpl::getResults() const { return results_; }
const PlanningPipeline &Simpl::getPlanningPipeline() const {
  return planningPipeline_;
};
const Holonomic &Simpl::getRevoyEv() const { return revoyEv_; };

} // namespace planning
