#include "planning/planning-pipeline.h"

namespace planning {

PlanningPipeline::PlanningPipeline(const Bounds &bounds,
                                   const BodyParams &bodyParams)
    : bounds_(bounds),
      // coarsePlanner_(std::make_shared<CoarsePlanner>(bounds, bodyParams)),
      // controlPlanner_(std::make_shared<ControlPlanner>(bounds, bodyParams)),
      proximityPlanner_(
          std::make_shared<ProximityPlanner>(bounds, bodyParams)) {};

Plan PlanningPipeline::getNextPlan(const HookedPose &start, const HookedPose &_,
                                   const Perception &perception) {

  // Coarse Plan: First see if its even possible for a simple Point in R^2 to
  // reach the goal. If not, no plan is possible. Otherwise, use the Coarse
  // Plan to inform the Control Plan
  // coarsePlanner_->plan(start, goal, grid);

  // Control Plan: Explores SE(2)+SO(2) w/ non-holonomic constraint,
  // articulated vehicle with a single hook-point. If possible, will find a
  // path to goal that respects vehicle artuculation dynamics.
  // controlPlanner_->plan(start, coarsePlanner_->getLastSolution(), grid);

  /// Proximity Plan: Explores 1D along travel to stop for obstacles,
  proximityPlanner_->plan(start, {}, perception.occupancy);

  // Plan
  return {
      .targetControls = proximityPlanner_->getControls(),
  };
}

// const std::shared_ptr<CoarsePlanner>
// PlanningPipeline::getCoarsePlanner() const {
//   return coarsePlanner_;
// }
// const std::shared_ptr<ControlPlanner>
// PlanningPipeline::getControlPlanner() const {
//   return controlPlanner_;
// }
const std::shared_ptr<ProximityPlanner>
PlanningPipeline::getProximityPlanner() const {
  return proximityPlanner_;
}

} // namespace planning
