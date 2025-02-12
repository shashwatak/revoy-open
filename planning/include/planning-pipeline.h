#pragma once

#include "planning/coarse-planner.h"

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
  const std::vector<Path> &getLastSolutions() const;
  const std::vector<Graph> &getLastGraphs() const;
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;

  /// out to controls
  const Controls &getControls() const;

  void plan(const HookedPose &start, const HookedPose &goal,
            std::shared_ptr<OccupancyGrid> grid
            //, std::vector<Footprints> footprints
  );

private:
  /// Params, Inputs, Outputs
  Bounds bounds_ = {};
  std::vector<Path> paths_ = {};
  std::vector<Graph> graphs_ = {};
  Controls controls_ = {};

  /// Obstacles
  std::shared_ptr<OccupancyGrid> grid_;
  std::vector<Footprint> footprints_;

  /// Planners
  std::shared_ptr<ProximityPlanner> proximityPlanner_;
};
} // namespace planning
