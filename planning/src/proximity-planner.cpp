#include "planning/proximity-planner.h"

#include "planning/fill-graph.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <ompl/base/State.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>

namespace planning {

ProximityPlanner::ProximityPlanner(const Bounds &bounds,
                                   const BodyParams &bodyParams)
    : bounds_(bounds), space_(std::make_shared<Flatland>(bounds)), setup_(space_),
      validityChecker_(std::make_shared<ValidityChecker>(
          setup_.getSpaceInformation(), bodyParams)) {
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // set state validity checking for this space
  setup_.setStateValidityChecker(validityChecker_);

  // register default projection
  setup_.getStateSpace()->registerProjections();
  setup_.getStateSpace()->getDefaultProjection()->setCellSizes({0.1, 0.1});

  // set the planner
  setup_.setPlanner(
      std::make_shared<ompl::geometric::LazyPRMstar>(setup_.getSpaceInformation()));

  // set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds rbounds(2);
  rbounds.low[0] = bounds_.lowerX;
  rbounds.low[1] = bounds_.lowerY;
  rbounds.high[0] = bounds_.upperX;
  rbounds.high[1] = bounds_.upperY;
  space_->setBounds(rbounds);
};

void ProximityPlanner::plan(const HookedPose &start_, const HookedPose &goal_,
                            std::shared_ptr<OccupancyGrid> grid) {

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

  path_.clear();
  controls_ = {};
  grid_ = grid;

  const Pose &gridPose{{start->getX(), start->getY()}, start->getYaw()};
  validityChecker_->setOccupancyGrid(grid_, gridPose);

  setup_.setup();
  // setup_.print();
  ompl::base::PlannerStatus solved = setup_.solve(0.01);

  if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION &&
      solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
    std::cout << "no solution: " << solved << std::endl;
  } else {
    std::cout << "solution: " << solved << std::endl;
    auto &solution = setup_.getSolutionPath();
    for (const auto baseState : solution.getStates()) {
      const auto state = baseState->as<Flatland::StateType>();
      path_.push_back({state->getX(), state->getY()});
    }

    if (solution.getStateCount() > 0) {
      // const auto ctrl =
      //     solution.getControl(0)
      //         ->as<ompl::control::DiscreteControlSpace::ControlType>();
      // controls_.speed = ctrl->value;
      // controls_.steer = 0;
      // controls_.duration = solution.getControlDuration(0);
    }
  }

  FillGraph<ompl::geometric::SimpleSetup, Flatland::StateType>(graph_, setup_);
  setup_.clear();
}

const Path &ProximityPlanner::getLastSolution() const { return path_; };
const Controls &ProximityPlanner::getControls() const { return controls_; }
const Graph &ProximityPlanner::getLastGraph() const { return graph_; }
const std::shared_ptr<OccupancyGrid> &
ProximityPlanner::getLastOccupancyGrid() const {
  return grid_;
}

ProximityPlanner::ValidityChecker::ValidityChecker(
    const ompl::base::SpaceInformationPtr &si, const BodyParams &bodyParams)
    : ompl::base::StateValidityChecker(si), bodyParams_(bodyParams) {}

void ProximityPlanner::ValidityChecker::setOccupancyGrid(
    const std::shared_ptr<OccupancyGrid> grid, const Pose &pose) {
  grid_ = grid;
  currentPose_ = pose;
}

bool ProximityPlanner::ValidityChecker::isValid(
    const ompl::base::State *state_) const {
  if (!grid_) {
    std::cout << "ERROR: no occupancy grid set, no validity check" << std::endl;
    assert(false);
    return false;
  }
  const auto *state = state_->as<Flatland::StateType>();
  bool isValid = si_->satisfiesBounds(state);

  const HookedPose pose = {{state->getX(), state->getY()}, state->getYaw(), 0};
  const Footprints body = FootprintsFromPose(pose, bodyParams_);

  /// put the footprint into the occupancy frame to check if its hitting
  /// anything
  for (const Footprint &part : body) {
    const Footprint partInRevoyFrame =
        ReverseTransformFootprint(part, currentPose_);
    isValid &= !grid_->isFootprintOccupied(partInRevoyFrame);
  }

  return isValid;
};

} // namespace planning
