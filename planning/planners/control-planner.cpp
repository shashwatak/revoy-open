#include "planning/control-planner.h"

#include "planning/fill-graph.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <ompl/base/State.h>
#include <ompl/control/planners/rrt/RRT.h>

namespace planning {

ControlPlanner::ControlPlanner(const Bounds &bounds,
                               const BodyParams &bodyParams)
    : bounds_(bounds), space_(std::make_shared<RevoySpace>()),
      cspace_(
          std::make_shared<ompl::control::RealVectorControlSpace>(space_, 2)),
      setup_(cspace_), validityChecker_(std::make_shared<ValidityChecker>(
                           setup_.getSpaceInformation(), bodyParams)),
      propagator_(std::make_shared<Propagator>(setup_.getSpaceInformation())) {

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEBUG);

  // set bounds for speed and steer
  ompl::base::RealVectorBounds cbounds(2);
  cbounds.low[0] = -10;
  cbounds.high[0] = 10;
  cbounds.low[1] = (-std::numbers::pi) / 2.0;
  cbounds.high[1] = (std::numbers::pi) / 2.0;
  cspace_->setBounds(cbounds);

  // set the state propagation routine
  setup_.setStatePropagator(propagator_);

  // set state validity checking for this space
  setup_.setStateValidityChecker(validityChecker_);

  // register default projection
  setup_.getStateSpace()->registerProjections();
  setup_.getStateSpace()->getDefaultProjection()->setCellSizes({0.1, 0.1});

  // set the planner
  setup_.setPlanner(
      std::make_shared<ompl::control::RRT>(setup_.getSpaceInformation()));

  // control steps per propogation
  setup_.getSpaceInformation()->setMinMaxControlDuration(1, 5);

  // set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds rbounds(2);
  rbounds.low[0] = bounds_.lowerX;
  rbounds.low[1] = bounds_.lowerY;
  rbounds.high[0] = bounds_.upperX;
  rbounds.high[1] = bounds_.upperY;
  space_->setBounds(rbounds);
};

void ControlPlanner::plan(const HookedPose &start_,
                          const std::vector<Pose> &goals,
                          std::shared_ptr<OccupancyGrid> grid) {

  path_.clear();
  controls_ = {};
  controlsVector_ = {};
  grid_ = grid;
  graph_.nodes.clear();
  graph_.edges.clear();

  const size_t numGoals = goals.size();
  for (size_t i = 0; i < numGoals; i++) {
    Pose start = {start_.position, start_.yaw};
    if (i > 0) {
      start = goals[i - 1];
    }

    // create a start state
    ompl::base::ScopedState<RevoySpace> revoyStart(space_);
    revoyStart->setX(start.position.x());
    revoyStart->setY(start.position.y());
    revoyStart->setYaw(start.yaw);
    revoyStart->setTrailerYaw(start.yaw);

    // create goal state
    Pose goal = goals[i];
    ompl::base::ScopedState<RevoySpace> revoyGoal(space_);
    revoyGoal->setX(goal.position.x());
    revoyGoal->setY(goal.position.y());
    revoyGoal->setYaw(goal.yaw);
    revoyGoal->setTrailerYaw(goal.yaw);

    setup_.setStartAndGoalStates(revoyStart, revoyGoal, 1);

    const Pose &gridPose{{revoyStart->getX(), revoyStart->getY()},
                         revoyStart->getYaw()};
    validityChecker_->setOccupancyGrid(grid_, gridPose);

    setup_.setup();
    // setup_.print();

    ompl::base::PlannerStatus solved = setup_.solve(0.1);
    if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION &&
        solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {

      std::cout << "no control solution: " << solved << std::endl;

    } else {

      std::cout << "control solution: " << solved << std::endl;
      auto &solution = setup_.getSolutionPath();
      for (const auto baseState : solution.getStates()) {
        const auto state = baseState->as<RevoySpace::StateType>();
        path_.push_back({{state->getX(), state->getY()}, state->getYaw()});
      }

      if (solution.getStateCount() > 0) {

        for (auto ctrlBase : solution.getControls()) {
          const auto ctrl =
              ctrlBase
                  ->as<ompl::control::RealVectorControlSpace::ControlType>();
          controlsVector_.push_back({ctrl->values[0], ctrl->values[1], 1.0});
        }

        controls_ = controlsVector_[0];
      }

      // FillGraph<ompl::control::SimpleSetup, RevoySpace::StateType>(graph_,
      //                                                              setup_);
      setup_.clear();
    }
  }
}

const std::vector<HookedPose> &ControlPlanner::getLastSolution() const {
  return path_;
};
const Controls &ControlPlanner::getControls() const { return controls_; }
const std::vector<Controls> &ControlPlanner::getControlsVector() const {
  return controlsVector_;
}
const Graph &ControlPlanner::getLastGraph() const { return graph_; }

ControlPlanner::ValidityChecker::ValidityChecker(
    const ompl::control::SpaceInformationPtr &si, const BodyParams &bodyParams)
    : ompl::base::StateValidityChecker(si), bodyParams_(bodyParams) {}

void ControlPlanner::ValidityChecker::setOccupancyGrid(
    const std::shared_ptr<OccupancyGrid> grid, const Pose &pose) {
  grid_ = grid;
  currentPose_ = pose;
}

bool ControlPlanner::ValidityChecker::isValid(
    const ompl::base::State *state_) const {
  if (!grid_) {
    std::cout << "ERROR: no occupancy grid set, no validity check" << std::endl;
    assert(false);
    return false;
  }
  const auto *state = state_->as<RevoySpace::StateType>();
  bool isValid = si_->satisfiesBounds(state);
  // std::cout << "bounds? : " << (isValid ? "true" : "false") << std::endl;

  const HookedPose pose = {
      {state->getX(), state->getY()}, state->getYaw(), state->getTrailerYaw()};
  const Footprints body = FootprintsFromPose(pose, bodyParams_);

  /// put the footprint into the occupancy frame to check if its hitting
  /// anything
  for (const Footprint &part : body) {
    const Footprint partInRevoyFrame =
        ReverseTransformFootprint(part, currentPose_);
    isValid &= !grid_->isFootprintOccupied(partInRevoyFrame);
  }

  isValid &= fabs(state->getHitchAngle()) < (M_PI / 2.0);
  return isValid;
}

ControlPlanner::Propagator::Propagator(
    const std::shared_ptr<ompl::control::SpaceInformation> si)
    : ompl::control::StatePropagator(si) {}

void ControlPlanner::Propagator::propagate(
    const ompl::base::State *start, const ompl::control::Control *control,
    const double duration, ompl::base::State *result) const {

  const auto ctrl =
      control->as<ompl::control::RealVectorControlSpace::ControlType>();
  const double speed = ctrl->values[0];
  const double steer = ctrl->values[1];
  RevoySpace::Propagate(start->as<RevoySpace::StateType>(), {speed, steer},
                        duration, result->as<RevoySpace::StateType>());
};

} // namespace planning
