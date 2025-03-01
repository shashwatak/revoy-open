#pragma once

#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>

#include "planning/types.h"

namespace planning {

// interpolate between states in the path, to get more dense XYs, to 
// better display the actual path in visualizer.
// The template syntax is insane, there may be a better way to do this.
template <typename SimpleSetup, typename StateSpace>
void FillPath(Path &path, const SimpleSetup &setup) {

  // skip when there is no exact or approximate solution
  if (!setup.haveSolutionPath()) {
    return;
  }

  auto &solution = setup.getSolutionPath();
  for (size_t i = 1; i < setup.getSolutionPath().getStateCount(); i++) {

    // get the two states we need to interpolate between
    const auto &prevState = solution.getState(i - 1)->template as<typename StateSpace::StateType>();
    const auto &state = solution.getState(i)->template as<typename StateSpace::StateType>();

    // frac is the interpolation factor [0,1]
    static constexpr double frac = 0.1;

    // t is the current interpolation iter
    double t = 0;
    while (t < 1) {

      // using the state space to allocate a new state, we will fill it with the interpolated values
      auto space = setup.getStateSpace()->template as<StateSpace>();
      auto newState = space->allocState()->template as<typename StateSpace::StateType>();

      // do the interpolation
      space->interpolate(prevState, state, t, newState);

      // save the interpolated state to output
      path.push_back({newState->getX(), newState->getY()});

      // increment and continue
      t += frac;
    }
  }
};





    // const size_t count = solution.getStateCount();
    // std::cout << "    count states: " << std::to_string(count) << std::endl;





} // namespace planning
