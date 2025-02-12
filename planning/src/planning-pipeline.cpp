#include "planning/planning-pipeline.h"

namespace planning {

PlanningPipeline::PlanningPipeline(const Bounds &bounds,
                                   const BodyParams &bodyParams)
    : bounds_(bounds), proximityPlanner_(std::make_shared<ProximityPlanner>(
                           bounds, bodyParams)) {};

void PlanningPipeline::plan(const HookedPose &start, const HookedPose &goal,
                            std::shared_ptr<OccupancyGrid> grid) {

  paths_.clear();
  graphs_.clear();
  controls_ = {};
  grid_ = grid;

  proximityPlanner_->plan(start, goal, grid);

  paths_.push_back(proximityPlanner_->getLastSolution());
  graphs_.push_back(proximityPlanner_->getLastGraph());
  controls_ = proximityPlanner_->getControls();
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
