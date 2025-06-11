#pragma once

#include "planning/types.h"

namespace planning {

class Holonomic {

public:
  Holonomic() = default;
  Holonomic(HookedPose start);

  void update(const Controls &controls, double duration);

  const Footprints getBody(const BodyParams &params) const;
  const HookedPose getHookedPose() const;

  void print(const BodyParams &params) const;

private:
  HookedPose pose_;
};

} // namespace planning
