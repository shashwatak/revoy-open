#include "planning/coarse-planner.h"

#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"
#include "planning/types.h"

#include <ompl/base/State.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace planning {

ompl::base::ValidStateSamplerPtr
AllocOBValidStateSampler(const ompl::base::SpaceInformation *si) {
  return std::make_shared<ompl::base::ObstacleBasedValidStateSampler>(si);
}

CoarsePlanner::CoarsePlanner(const Bounds &bounds, const BodyParams &bodyParams)
    : bounds_(bounds), space_(std::make_shared<Flatland>(bounds)),
      setup_(space_), validityChecker_(std::make_shared<ValidityChecker>(
                          setup_.getSpaceInformation(), bodyParams)) {
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // set state validity checking for this space
  setup_.setStateValidityChecker(validityChecker_);

  // register default projection
  setup_.getStateSpace()->registerProjections();
  setup_.getStateSpace()->getDefaultProjection()->setCellSizes({0.1, 0.1});

  // set the planner
  setup_.setPlanner(
      std::make_shared<ompl::geometric::RRTstar>(setup_.getSpaceInformation()));

  // set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds rbounds(2);
  rbounds.low[0] = bounds_.lowerX;
  rbounds.low[1] = bounds_.lowerY;
  rbounds.high[0] = bounds_.upperX;
  rbounds.high[1] = bounds_.upperY;
  space_->setBounds(rbounds);
};

void CoarsePlanner::plan(const HookedPose &start_, const HookedPose &goal_,
                         const Footprints &undrivableAreas) {

  setup_.clear();

  // create a start state
  ompl::base::ScopedState<Flatland> start(space_);
  start->setX(start_.position.x());
  start->setY(start_.position.y());
  start->setYaw(start_.yaw);

  // create goal state, move forward a little only
  ompl::base::ScopedState<Flatland> goal(space_);
  goal->setX(goal_.position.x());
  goal->setY(goal_.position.y());
  goal->setYaw(goal_.yaw);

  setup_.setStartAndGoalStates(start, goal, 1);

  validityChecker_->setUndrivableAreas(undrivableAreas);

  setup_.setup();

  ompl::base::PlannerStatus solved = setup_.solve(0.1);
  setup_.getPathSimplifier()->simplifyMax(setup_.getSolutionPath());

  if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION &&
      solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
    std::cout << "no coarse solution: " << solved << std::endl;
  }
}

const ompl::geometric::SimpleSetup &CoarsePlanner::getSetup() const {
  return setup_;
}

CoarsePlanner::ValidityChecker::ValidityChecker(
    const ompl::base::SpaceInformationPtr &si, const BodyParams &bodyParams)
    : ompl::base::StateValidityChecker(si), bodyParams_(bodyParams) {}

void CoarsePlanner::ValidityChecker::setUndrivableAreas(
    const Footprints &undrivableAreas) {
  undrivableAreas_ = undrivableAreas;
}

bool CoarsePlanner::ValidityChecker::isValid(
    const ompl::base::State *state_) const {

  const auto *state = state_->as<Flatland::StateType>();
  bool isValid = si_->satisfiesBounds(state);

  const HookedPose pose = {
      {state->getX(), state->getY()}, state->getYaw(), state->getTrailerYaw()};
  const Footprints body = FootprintsFromPose(pose, bodyParams_);

  // compare the body parts against undrivable areas
  for (const Footprint &part : body) {
    for (const Footprint &area : undrivableAreas_) {
      isValid &= !AreFootprintsOverlapping(part, area);
    }
  }

  return isValid;
};

} // namespace planning
