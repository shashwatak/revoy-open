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

  // using the state space to allocate a new state, we will fill it with the
  // interpolated values
  auto space = setup.getStateSpace()->template as<StateSpace>();

  // write into this state during interpolation, remember to free it at the end
  auto tempState =
      space->allocState()->template as<typename StateSpace::StateType>();

  // interpolate between solution[i-1] and solution[i]
  auto &solution = setup.getSolutionPath();

  for (size_t i = 1; i < setup.getSolutionPath().getStateCount(); i++) {

    // get the two states we need to interpolate between
    const auto &prevState =
        solution.getState(i - 1)->template as<typename StateSpace::StateType>();
    const auto &state =
        solution.getState(i)->template as<typename StateSpace::StateType>();

    // interp is the current interpolation factor [0, 1]
    double interp = 0;

    // update interp by this much each step, moving it 0 -> 1
    static constexpr double interpStep = 0.1;

    // interpolation loop
    while (interp < 1) {

      // do the interpolation
      space->interpolate(prevState, state, interp, tempState);

      // save the interpolated state to output
      path.push_back({tempState->getX(), tempState->getY()});

      // increment and continue
      interp += interpStep;
    }
  }

  // need to free the temp state, otherwise leak
  space->freeState(tempState);
};

} // namespace planning
