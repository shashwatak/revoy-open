
#include "planning/make-scenario.h"
#include "planning/control-planner.h"
#include "planning/mock-revoy-ev.h"
#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"

#include "planning/simpl.h"
#include "planning/mcap-utils.h"
#include <cstdint>

using namespace planning;


int main(int argc, char **argv) {

  const std::vector<std::string> args(argv + 1, argv + argc);

  const Scenario scenario = MakeYardScenario();

  auto controlPlanner = std::make_unique<ControlPlanner>(scenario.bounds, scenario.bodyParams);
  auto mockRevoyEv = std::make_unique<MockRevoyEv>(scenario.start);

  Footprints footprints;
  for (const Entity& entity: scenario.entities) {
    const auto tfFootprint = TransformFootprint(entity.footprint, entity.pose);
    footprints.push_back(tfFootprint);
  }

  const std::shared_ptr<OccupancyGrid> grid =
      FootprintsToOccupancyGrid(footprints, mockRevoyEv->getHookedPose());

  // grid->print();
  controlPlanner->plan(scenario.start, scenario.goal, grid);

  std::unique_ptr<McapWrapper> mcapWrapper =
      std::make_unique<McapWrapper>("control-planner-only-" + scenario.name + ".mcap");

  int64_t time = scenario.timeParams.startTime;
  bool collision = false;
  size_t controlsIdx = 0;
  auto controlsVector = controlPlanner->getControlsVector();
  double currControlEnd = controlsVector[0].duration;

  Scene scene;
  scene.visibleEntities = footprints;
  scene.scenario = scenario;
  scene.graphs.clear();
  scene.graphs.push_back({});
  scene.graphs.push_back({});

  scene.plannedPaths.clear();
  scene.plannedPaths.push_back({});
  scene.plannedPaths.push_back({});
  scene.revoy = mockRevoyEv->getBody(scenario.bodyParams);
  scene.revoyPose = mockRevoyEv->getHookedPose();
  scene.grid = grid;

  mcapWrapper->write(scene, time);

  while (time <= scenario.timeParams.timeout + scenario.timeParams.startTime &&
         controlsIdx < controlsVector.size()) {

    const auto controls = controlsVector[controlsIdx];

    mockRevoyEv->update(controls, scenario.bounds, scenario.timeParams.dt / 1e6);

    // record mcap
    scene.revoy = mockRevoyEv->getBody(scenario.bodyParams);
    scene.revoyPose = mockRevoyEv->getHookedPose();

    scene.graphs.clear();
    scene.graphs.push_back({});
    scene.graphs.push_back(controlPlanner->getLastGraph());

    scene.plannedPaths.clear();
    scene.plannedPaths.push_back({});
    scene.plannedPaths.push_back(controlPlanner->getLastSolution());

    mcapWrapper->write(scene, time);

    collision |= IsBodyCollidingAnyObstacles(scene.revoy, scene.visibleEntities);

    // tick
    std::cout << "---------------------------" << std::endl;
    std::cout << "time: " << std::to_string(time) << std::endl;
    std::cout << "dt: " << std::to_string(scenario.timeParams.dt) << std::endl;
    time += scenario.timeParams.dt;
    std::cout << "time now: " << std::to_string(time) << std::endl;
    double next = scenario.timeParams.startTime + (currControlEnd * 1e6);

    if (time > next) {
      std::cout << "time > next: " << std::to_string(time) << " > " << std::to_string(next) << std::endl;
      controlsIdx++;
      if ( controlsIdx < controlsVector.size()) { 
        currControlEnd += controlsVector[controlsIdx].duration; 
        std::cout << "new next: " << std::to_string(currControlEnd) << std::endl;
      };
    }
  }

  mcapWrapper.reset();

  std::cout << "collision: " << (collision ? "yes" : "no") << std::endl;
  return 0;
}

