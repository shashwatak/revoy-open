#pragma once

#include "planning/occupancy-grid.h"
#include "planning/types.h"

namespace planning {

// the input to Planning
struct Perception {

  // some perception / tracking systems more naturally output boxes / polygons
  // that are mapped to the floor.
  Footprints obstacles;

  // some perception / tracking systems more naturally output grid of pixels
  // that are mapped to the floor.
  OccupancyGrid occupancy;
};
} // namespace planning
