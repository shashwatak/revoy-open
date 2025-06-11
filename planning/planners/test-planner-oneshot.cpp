
#include <catch2/catch_test_macros.hpp>

// #include "planning/control-planner.h"
#include "planning/fill-graph.h"
#include "planning/fill-path.h"

#include "planning/add-footprint-to-grid.h"
#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"
#include "planning/holonomic.h"
#include "planning/make-scenario.h"

#include "planning/simpl-mcap.h"

using namespace planning;

// exercise control-planner by applying solution controls (speed, steer,
// duration) to the vehicle model.

TEST_CASE("run the coarse planner once, then control planner along") {

  // todo: configure scenario params
  const Scenario scenario = MakeTurningScenario("oneshot-turn");

  // setup planners and mock revoy
  CoarsePlanner coarsePlanner(scenario.bounds, scenario.bodyParams);
  // ControlPlanner controlPlanner (scenario.bounds, scenario.bodyParams);
  Holonomic mockRevoyEv(scenario.start);

  // add obstacles to occupancy grid
  Footprints footprints;
  for (const Entity &entity : scenario.entities) {
    const auto tfFootprint = TransformFootprint(entity.footprint, entity.pose);
    footprints.push_back(tfFootprint);
  }

  // static constexpr uint16_t NUM_CELLS = 200;
  // static constexpr double CELL_LENGTH = 0.1; // meters
  // static constexpr double GRID_OFFSET =
  //     CELL_LENGTH * ((float)NUM_CELLS) / 2.0; // meters
  // OccupancyGrid grid(NUM_CELLS, NUM_CELLS, CELL_LENGTH, CELL_LENGTH,
  //                    GRID_OFFSET, GRID_OFFSET);
  // FootprintsToOccupancyGrid(grid, footprints, mockRevoyEv->getHookedPose());

  // coarse plan
  std::cout << "plan" << std::endl;
  coarsePlanner.plan(scenario.start, scenario.goal, footprints);

  if (!coarsePlanner.getSetup().haveSolutionPath()) {

    std::cout << "no plan made" << std::endl;

  } else {

    // const auto &solution = coarsePlanner.getSetup().getSolutionPath();
    // // vehicle kinematic plan
    // controlPlanner->plan(scenario.start, solution, grid);

    // init visualizer
    SimplMcap mcap("test-planner-oneshot-" + scenario.name + ".mcap");

    // verify plan by moving vehicle along
    int64_t time = scenario.timeParams.startTime;
    bool collision = false;
    // size_t controlsIdx = 0;
    // auto controlsVector = controlPlanner->getControlsVector();
    // double currControlEnd = controlsVector[0].duration;

    // visualization
    // todo: replace bug-prone vector push_back stuff
    Scene scene;
    scene.visibleEntities = footprints;
    scene.scenario = scenario;
    FillPath<ompl::geometric::SimpleSetup, CoarsePlanner::Flatland>(
        scene.planners["coarse"].solution, coarsePlanner.getSetup());
    FillGraph<ompl::geometric::SimpleSetup, CoarsePlanner::Flatland::StateType>(
        scene.planners["coarse"].graph, coarsePlanner.getSetup());

    scene.revoy = mockRevoyEv.getBody(scenario.bodyParams);
    scene.revoyPose = mockRevoyEv.getHookedPose();
    scene.occupancy = nullptr;

    // write initial scene to mcap
    mcap.write(scene, time);

    // apply the resulting controls plan to the vehicle model, it should work.
    while (time <= scenario.timeParams.timeout + scenario.timeParams.startTime
           // && controlsIdx < controlsVector.size()) {
    ) {

      //   // get current controls node
      //   const auto controls = controlsVector[controlsIdx];

      //   // update vehicle model
      //   mockRevoyEv->update(controls, scenario.timeParams.dt / 1e6);

      //   // update visualization
      //   // todo: replace bug-prone push_back weirdness
      //   scene.revoy = mockRevoyEv->getBody(scenario.bodyParams);
      //   scene.revoyPose = mockRevoyEv->getHookedPose();
      //   scene.coarseGraph = coarsePlanner->getLastGraph();
      //   scene.controlGraph = controlPlanner->getLastGraph();
      //   scene.coarseSolution = coarsePlanner->getLastSolution();
      //   scene.controlSolution = controlPlanner->getLastSolution();

      //   // write scene to mcap
      //   mcap->write(scene, time);

      // true if ever in collision
      // collision |=
      //     IsBodyCollidingAnyObstacles(scene.revoy, scene.visibleEntities);

      // tick
      // std::cout << "---------------------------" << std::endl;
      // std::cout << "time: " << std::to_string(time) << std::endl;
      // std::cout << "dt: " << std::to_string(scenario.timeParams.dt) <<
      // std::endl;
      time += scenario.timeParams.dt;
      // std::cout << "time now: " << std::to_string(time) << std::endl;
      //   double next = scenario.timeParams.startTime + (currControlEnd * 1e6);

      //   // when this control-nodes duration has expired, move onto the next
      //   node in
      //   // the solution path
      //   if (time > next) {
      //     // std::cout << "time > next: " << std::to_string(time) << " > " <<
      //     // std::to_string(next) << std::endl;
      //     controlsIdx++;
      //     if (controlsIdx < controlsVector.size()) {
      //       currControlEnd += controlsVector[controlsIdx].duration;
      //       // std::cout << "new next: " << std::to_string(currControlEnd) <<
      //       // std::endl;
      //     };
      //   }
    }

    // report error
    CHECK(!collision);
  }

  // close file write, otherwise mcap will be corrupted
  // mcap.reset();
}
