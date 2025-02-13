
#include "planning/make-scenario.h"
#include "planning/control-planner.h"
#include "planning/mock-revoy-ev.h"
#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"

#include "planning/simpl.h"
#include "planning/mcap-utils.h"
#include <cstdint>

using namespace planning;


/// exercise control-planner by applying solution controls (speed, steer, duration) to the
/// vehicle model.

int main(int argc, char **argv) {

  /// todo: args
  const std::vector<std::string> args(argv + 1, argv + argc);

  /// todo: configure scenario params
  const Scenario scenario = MakeYardScenario();

  /// setup planners and mock revoy
  auto coarsePlanner = std::make_unique<CoarsePlanner>(scenario.bounds, scenario.bodyParams);
  auto controlPlanner = std::make_unique<ControlPlanner>(scenario.bounds, scenario.bodyParams);
  auto mockRevoyEv = std::make_unique<MockRevoyEv>(scenario.start);

  /// add obstacles to occupancy grid
  Footprints footprints;
  for (const Entity& entity: scenario.entities) {
    const auto tfFootprint = TransformFootprint(entity.footprint, entity.pose);
    footprints.push_back(tfFootprint);
  }
  const std::shared_ptr<OccupancyGrid> grid =
      FootprintsToOccupancyGrid(footprints, mockRevoyEv->getHookedPose());
  // grid->print();

  /// coarse plan
  std::cout << "plan" << std::endl;
  coarsePlanner->plan(scenario.start, scenario.goal, grid);

  // const auto& solution = coarsePlanner->getLastSolution();

  /// vehicle kinematic plan
  controlPlanner->plan(scenario.start, scenario.goal, grid);

  /// init visualizer
  std::unique_ptr<McapWrapper> mcapWrapper =
      std::make_unique<McapWrapper>("control-planner-only-" + scenario.name + ".mcap");

  /// verify plan by moving vehicle along
  int64_t time = scenario.timeParams.startTime;
  bool collision = false;
  size_t controlsIdx = 0;
  auto controlsVector = controlPlanner->getControlsVector();
  double currControlEnd = controlsVector[0].duration;

  /// visualization
  /// todo: replace bug-prone vector push_back stuff
  Scene scene;
  scene.visibleEntities = footprints;
  scene.scenario = scenario;
  scene.coarseSolution = coarsePlanner->getLastSolution();
  scene.controlSolution = controlPlanner->getLastSolution();
  scene.coarseGraph = coarsePlanner->getLastGraph();
  scene.controlGraph = controlPlanner->getLastGraph();
  scene.revoy = mockRevoyEv->getBody(scenario.bodyParams);
  scene.revoyPose = mockRevoyEv->getHookedPose();
  scene.grid = grid;

  /// write initial scene to mcap
  mcapWrapper->write(scene, time);

  /// apply the resulting controls plan to the vehicle model, it should work.
  while (time <= scenario.timeParams.timeout + scenario.timeParams.startTime &&
         controlsIdx < controlsVector.size()) {

    /// get current controls node
    const auto controls = controlsVector[controlsIdx];

    /// update vehicle model 
    mockRevoyEv->update(controls, scenario.bounds, scenario.timeParams.dt / 1e6);

    /// update visualization
    /// todo: replace bug-prone push_back weirdness
    scene.revoy = mockRevoyEv->getBody(scenario.bodyParams);
    scene.revoyPose = mockRevoyEv->getHookedPose();
    scene.coarseGraph = coarsePlanner->getLastGraph();
    scene.controlGraph = controlPlanner->getLastGraph();
    scene.coarseSolution = coarsePlanner->getLastSolution();
    scene.controlSolution = controlPlanner->getLastSolution();

    /// write scene to mcap
    mcapWrapper->write(scene, time);

    /// true if ever in collision
    collision |= IsBodyCollidingAnyObstacles(scene.revoy, scene.visibleEntities);

    /// tick
    // std::cout << "---------------------------" << std::endl;
    // std::cout << "time: " << std::to_string(time) << std::endl;
    // std::cout << "dt: " << std::to_string(scenario.timeParams.dt) << std::endl;
    time += scenario.timeParams.dt;
    // std::cout << "time now: " << std::to_string(time) << std::endl;
    double next = scenario.timeParams.startTime + (currControlEnd * 1e6);

    /// when this control-nodes duration has expired, move onto the next node in the solution path
    if (time > next) {
      // std::cout << "time > next: " << std::to_string(time) << " > " << std::to_string(next) << std::endl;
      controlsIdx++;
      if ( controlsIdx < controlsVector.size()) { 
        currControlEnd += controlsVector[controlsIdx].duration; 
        // std::cout << "new next: " << std::to_string(currControlEnd) << std::endl;
      };
    }
  }

  /// close file write
  mcapWrapper.reset();

  /// report error
  std::cout << "collision: " << (collision ? "yes" : "no") << std::endl;
  return (collision) ? 0 : 1;
}

