#pragma once

#include "planning/coarse-planner.h"
#include "planning/control-planner.h"

namespace planning {

/// A series of planners, coarse search (low dimension) -> fine (high dimension)
/// similar to "multilevel planning" but not using ompl::multilevel because
/// non-holonomic constraints are more easily captured by ompl::control.
/// TODO: possibly replace this w/ orchestrator i.e. roslaunch or RTOS scheduler
/// tasks.
class PlanningPipeline {
public:
  PlanningPipeline() = delete;
  PlanningPipeline(const Bounds &bounds, const BodyParams &bodyParams);

  /// getters used for output / debug
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;

  /// get const planners, to access their debug info
  const std::shared_ptr<CoarsePlanner> getCoarsePlanner() const;
  const std::shared_ptr<ControlPlanner> getControlPlanner() const;

  void plan(const HookedPose &start, const HookedPose &goal,
            std::shared_ptr<OccupancyGrid> grid
            //, std::vector<Footprints> footprints
  );

private:
  /// Params, Inputs, Outputs
  Bounds bounds_ = {};

  /// Obstacles
  std::shared_ptr<OccupancyGrid> grid_;
  std::vector<Footprint> footprints_;

  /// Planners
  std::shared_ptr<CoarsePlanner> coarsePlanner_;
  std::shared_ptr<ControlPlanner> controlPlanner_;
};
} // namespace planning
