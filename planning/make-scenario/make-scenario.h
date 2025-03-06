#pragma once

#include "planning/types.h"

namespace planning {

// returns Scenario where there is an obstacle blocking the way, and it
// disappears after a while.
Scenario MakeDisappearingObstacleScenario(double dir, double dist,
                                          double disappear, double timeout,
                                          std::string name);

// return Yard Scenario (WIP)
Scenario MakeYardScenario(const std::string& name);

// return simple left turn
Scenario MakeTurningScenario(const std::string& name);

} // namespace planning
