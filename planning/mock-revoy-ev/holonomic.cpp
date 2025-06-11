#include "planning/holonomic.h"

#include "planning/footprint-transform.h"
#include "planning/types.h"

#include <iostream>
#include <limits>

namespace planning {

Holonomic::Holonomic(HookedPose start) : pose_(start) {};

void Holonomic::update(const Controls &controls, double duration) {

  const double travel = controls.speed * duration;
  pose_.yaw = fixRadian(pose_.yaw + controls.steer);
  pose_.position.x() = pose_.position.x() + (travel * cos(pose_.yaw));
  pose_.position.y() = pose_.position.y() + (travel * sin(pose_.yaw));
}

const HookedPose Holonomic::getHookedPose() const { return pose_; }

const Footprints Holonomic::getBody(const BodyParams &params) const {

  return FootprintsFromPose(getHookedPose(), params);
}

void Holonomic::print(const BodyParams &params) const {
  std::cout << "revoy feet" << std::endl;
  for (const auto &bodyPart : getBody(params)) {
    for (const auto &point : bodyPart) {
      std::cout << "    (" << point.x() << "," << point.y() << ")\n";
    }
  }
  std::cout << std::endl;
}

} // namespace planning
