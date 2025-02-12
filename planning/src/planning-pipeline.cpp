#include "planning/planning-pipeline.h"

namespace planning {

PlanningPipeline::PlanningPipeline(const Bounds &bounds,
                                   const BodyParams &bodyParams)
    : bounds_(bounds),
      coarsePlanner_(std::make_shared<CoarsePlanner>(bounds, bodyParams)),
      controlPlanner_(std::make_shared<ControlPlanner>(bounds, bodyParams)) {};

void PlanningPipeline::plan(const HookedPose &start, const HookedPose &goal,
                            std::shared_ptr<OccupancyGrid> grid) {

  paths_.clear();
  graphs_.clear();
  controls_ = {};
  grid_ = grid;

  /// Coarse Plan: First see if its even possible for a simple Point in R^2 to
  /// reach the goal. If not, no plan is possible. Otherwise, use the Coarse
  /// Plan to inform the Control Plan
  coarsePlanner_->plan(start, goal, grid);
  paths_.push_back(coarsePlanner_->getLastSolution());
  graphs_.push_back(coarsePlanner_->getLastGraph());

  /// Control Plan: Explores SE(2)+SO(2) w/ non-holonomic constraint, articulated vehicle with a single
  /// hook-point. If possible, will find a path to goal that respects vehicle
  /// artuculation dynamics.
  controlPlanner_->plan(start, goal, grid);
  paths_.push_back(controlPlanner_->getLastSolution());
  graphs_.push_back(controlPlanner_->getLastGraph());

  controls_ = controlPlanner_->getControls();
}

const std::vector<Path> &PlanningPipeline::getLastSolutions() const {
  return paths_;
};
const std::vector<Graph> &PlanningPipeline::getLastGraphs() const {
  return graphs_;
}

const std::shared_ptr<OccupancyGrid> &
PlanningPipeline::getLastOccupancyGrid() const {
  return grid_;
}

const Controls &PlanningPipeline::getControls() const { return controls_; }

} // namespace planning
