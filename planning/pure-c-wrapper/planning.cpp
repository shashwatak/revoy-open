#include "planning/planning.h"

#include "planning/occupancy-grid.h"
#include "planning/perception.h"
#include "planning/plan.h"
#include "planning/planning-pipeline.h"
#include "planning/types.h"

#include <bitset>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>

// Somewhat unfortunate name-collision, I want the Pure C interface and
// the C++ interface to use the same names (Plan and Perception).
// In order to support both, this file has to absorb the weirdness.
using CPerception = ::Perception;
using CPlan = ::Plan;

using CppPerception = planning::Perception;
using CppPlan = planning::Plan;

using namespace planning;

static std::unique_ptr<PlanningPipeline> planner_ = nullptr;

static std::unique_ptr<CppPerception> cppPerception_ = nullptr;

static const Bounds BOUNDS;
static const BodyParams BODY;

CPerception MakeEmptyPerception() { return {}; }

void StartPlanning() {
  std::cout << "Initalize Planning" << std::endl;
  planner_ = std::make_unique<PlanningPipeline>(BOUNDS, BODY);
  cppPerception_ = std::make_unique<CppPerception>();
};

void EndPlanning() {
  std::cout << "Terminate Planning" << std::endl;
  planner_.reset();
};

/// for now, these dont matter because there is no localization, and we assume
/// to move forward a little
static const HookedPose START{{0, 0}, 0, 0};
static const HookedPose GOAL{{1, 0}, 0, 0};

CPlan GetNextPlan(const CPerception *perception) {

  if (!perception) {
    return {};
  }

  // std::cout << "perception->numBytes: " <<
  // std::to_string(perception->numBytes)
  //           << std::endl;
  // std::cout << "perception->numBytesInRow: "
  //           << std::to_string(perception->numBytesInRow) << std::endl;
  // std::cout << "perception->bitsPerMeter: "
  //           << std::to_string(perception->bitsPerMeter) << std::endl;
  // std::cout << "perception->offsetX: " << std::to_string(perception->offsetX)
  //           << std::endl;
  // std::cout << "perception->offsetY: " << std::to_string(perception->offsetY)
  //           << std::endl;

  /// prevent divide-by-zero, assert because execution won't be possible
  assert(perception->numBytes != 0);
  assert(perception->numBytesInRow != 0);
  assert(perception->bitsPerMeter != 0);

  const uint32_t N = perception->numBytes / perception->numBytesInRow;
  const uint32_t M = perception->numBytesInRow * 8;

  const double cellX = 1.0 / perception->bitsPerMeter;
  const double cellY = 1.0 / perception->bitsPerMeter;
  const double offsetX = perception->offsetX;
  const double offsetY = perception->offsetY;

  // std::cout << "N: " << std::to_string(N) << std::endl;
  // std::cout << "M: " << std::to_string(M) << std::endl;
  // std::cout << "cellX: " << std::to_string(cellX) << std::endl;
  // std::cout << "cellY: " << std::to_string(cellY) << std::endl;
  // std::cout << "offsetX: " << std::to_string(offsetX) << std::endl;
  // std::cout << "offsetY: " << std::to_string(offsetY) << std::endl;

  // run planning
  cppPerception_->occupancy.reset(perception->occupancy, perception->numBytes,
                                  N, M, cellX, cellY, offsetX, offsetY);
  CppPlan cppPlan = planner_->getNextPlan(START, GOAL, *cppPerception_);
  const Controls controls = cppPlan.targetControls;

  // gather planning output into C struct
  CPlan plan = {.setSpeed = controls.speed,
                .setSteer = controls.steer,
                .gridWidth = cppPerception_->occupancy.getN(),
                .gridHeight = cppPerception_->occupancy.getM(),
                .cellSizeX = cppPerception_->occupancy.getCellX(),
                .cellSizeY = cppPerception_->occupancy.getCellY(),
                .originX = cppPerception_->occupancy.getOffsetX(),
                .originY = cppPerception_->occupancy.getOffsetY(),
                .gridData = nullptr,
                .gridDataSize = 0};

  // Copy grid data
  std::string gridData = cppPerception_->occupancy.fillData();
  if (!gridData.empty()) {
    plan.gridData = new char[gridData.size()];
    memcpy(plan.gridData, gridData.data(), gridData.size());
    plan.gridDataSize = gridData.size();
  }

  return plan;
}
