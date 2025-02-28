#pragma once

#include "planning/mock-revoy-ev.h"
#include "planning/planning-pipeline.h"
#include "planning/types.h"

namespace planning {

// Simpl: Simulator for OMPL, focusing on SE(2) and similar "2D" spaces
class Simpl {

  // defines what will happen in the simulation
  Scenario scenario_;

  // simple vehicle model to move around in the simulation
  MockRevoyEv revoyEv_;

  // the planner, outputs commands to the vehicle model
  PlanningPipeline planningPipeline_;

  // obstacles, will be filled with simulated values
  Perception perception_;

  // result, continue to update it as the scenario runs
  Results results_;

  // true if currently the revoy is in contact with any obstacle
  bool isColliding(int64_t time) const;

  // true if time has exceeded scenario defined time bounds
  bool isTimeout(int64_t time) const;

  // true if the revoy has reached the goal state
  bool isGoalMet() const;

public:
  // initialize simulation with the given scenario, the only permitted
  // way to initialize Simpl
  Simpl(Scenario scenario);
  Simpl() = delete;

  // called in a tight loop
  void update(int64_t time);

  // true when exit condition is met
  bool isDone(int64_t time) const;

  // TODO: leaky abstraction, directly access the planner is weird.
  // we really only need this for visualization, so we should maybe
  // do something with a friend function?
  const PlanningPipeline &getPlanningPipeline() const;

  // the following ar used for debug visualization or analysis
  const MockRevoyEv &getRevoyEv() const;
  const Scenario &getScenario() const;
  const Results &getResults() const;
  const Footprints getVisibleFootprints(int64_t time) const;
  const Perception &getLastPerception() const;
};

} // namespace planning
