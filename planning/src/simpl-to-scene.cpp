#include "planning/simpl-to-scene.h"

namespace planning {
Scene SimplToScene(std::unique_ptr<Simpl> &simpl, int64_t time) {
  assert(simpl);

  Scene scene;
  scene.scenario = simpl->getScenario();
  scene.coarseSolution =
      simpl->getPlanningPipeline().getCoarsePlanner()->getLastSolution();
  scene.controlSolution =
      simpl->getPlanningPipeline().getControlPlanner()->getLastSolution();
  scene.coarseGraph =
      simpl->getPlanningPipeline().getCoarsePlanner()->getLastGraph();
  scene.controlGraph =
      simpl->getPlanningPipeline().getControlPlanner()->getLastGraph();
  scene.revoy = simpl->getRevoyEv().getBody(scene.scenario.bodyParams);
  scene.revoyPose = simpl->getRevoyEv().getHookedPose();
  scene.visibleEntities = simpl->getVisibleFootprints(time);
  scene.grid = simpl->getPlanningPipeline().getLastOccupancyGrid();

  return scene;
}
} // namespace planning
