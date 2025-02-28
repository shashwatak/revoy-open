#pragma once

#include "planning/types.h"

namespace planning {

// the output of Planning
struct Plan {

  // tells the ECU the desired speed and steering angle of the vehicle.
  Controls targetControls;
};
} // namespace planning
