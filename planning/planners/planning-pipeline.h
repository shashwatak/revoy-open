#pragma once

// #include "planning/coarse-planner.h"
// #include "planning/control-planner.h"
#include "planning/proximity-planner.h"

#include "planning/perception.h"
#include "planning/plan.h"

namespace planning {

// NOTE: currently not using ompl::multilevel because non-holonomic constraints
// are more easily captured by ompl::control, which isn't readily compatible
// with ompl::multilevel (only seems to support ompl::geometric)

// A series of planners, coarse -> fine.

// TODO: configure the rates at which each planner runs: currently they run
// serially one after the other, but we will want some to run more often, or in
// parallel. Perhaps move this to a scheduler of some kind, possibly RTOS?

// Planning Levels

// Coarse Plan: First see if its even possible for a simple Point in R^2 to
// reach the goal. If not, no plan is possible. Otherwise, use the Coarse
// Plan to inform the Control Plan.

// Control Plan: Explores SE(2)+SO(2) w/ non-holonomic constraint,
// articulated vehicle with a single hook-point. If possible, will find a
// path to goal that respects vehicle artuculation dynamics.

// Proximity Plan: Explores path (1D travel) and time to stop for dynamic /
// moving obstacles. Real-time planning.

class PlanningPipeline {
public:
  PlanningPipeline() = delete;
  PlanningPipeline(const Bounds &bounds, const BodyParams &bodyParams);

  Plan getNextPlan(const HookedPose &start, const HookedPose &goal,
                   const Perception &perception);

  // TODO: leaky abstraction, directly access the planner is weird.
  // we really only need this for visualization, so we should maybe
  // do something with a friend function?
  const std::shared_ptr<ProximityPlanner> getProximityPlanner() const;

private:
  // Params, Inputs, Outputs
  Bounds bounds_ = {};

  // Planners
  // std::shared_ptr<CoarsePlanner> coarsePlanner_;
  // std::shared_ptr<ControlPlanner> controlPlanner_;
  std::shared_ptr<ProximityPlanner> proximityPlanner_;
};
} // namespace planning
