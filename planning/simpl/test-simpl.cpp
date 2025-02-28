#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>

#include "planning/footprint-overlap.h"
#include "planning/make-scenario.h"
#include "planning/mock-revoy-ev.h"
#include "planning/simpl-mcap.h"
#include "planning/simpl.h"
#include "planning/types.h"

using namespace planning;

namespace {
Results DoSimplLoop(const Scenario &scenario);

const double MIN_AVOIDABLE_DIST = (3 * sqrt(BodyParams().revoyLength));
const double UNAVOIDABLE_DIST = MIN_AVOIDABLE_DIST - 1;
const double AVOIDABLE_DIST = MIN_AVOIDABLE_DIST + 3;
const double EAST = 0;
const double SOUTH_WEST = -3 * M_PI / 4.0;
const double TIMEOUT = 1e7;
const double PERMANENT_OBSTACLE = 1e100;
const double TEMP_OBSTACLE = 1e6;
} // namespace

TEST_CASE("revoy moves only after unavoidable obstacle disappears") {

  const std::string name = std::string("go-after-obstacle-disappears");

  Scenario scenario = MakeDisappearingObstacleScenario(
      EAST, UNAVOIDABLE_DIST, TEMP_OBSTACLE, TIMEOUT, name);

  Results results = DoSimplLoop(scenario);

  CHECK(results.collision);
  CHECK(results.maxSpeed > 0);
  CHECK(results.goalMet);
}

TEST_CASE("revoy never moves because unavoidable obstacle is blocking it") {
  const std::string name = std::string("dont-go-obstacle-wont-disappear");

  Scenario scenario = MakeDisappearingObstacleScenario(
      SOUTH_WEST, UNAVOIDABLE_DIST, PERMANENT_OBSTACLE, TIMEOUT, name);

  Results results = DoSimplLoop(scenario);

  CHECK(results.collision);
  CHECK(results.maxSpeed == 0);
  CHECK(!results.goalMet);
}

TEST_CASE("revoy moves closer to a obstacle but stops before it") {

  const std::string name = std::string("move-up-to-obstacle-and-stop");

  Scenario scenario = MakeDisappearingObstacleScenario(
      SOUTH_WEST, AVOIDABLE_DIST, PERMANENT_OBSTACLE, TIMEOUT, name);

  Results results = DoSimplLoop(scenario);

  CHECK(!results.collision);
  CHECK(results.maxSpeed > 0);
  CHECK(!results.goalMet);
}

namespace {
Results DoSimplLoop(const Scenario &scenario) {

  std::unique_ptr<Simpl> simpl = std::make_unique<Simpl>(scenario);

  std::unique_ptr<SimplMcap> mcap =
      std::make_unique<SimplMcap>("test-simpl-" + scenario.name + ".mcap");

  int64_t time = scenario.timeParams.startTime;

  while (!simpl->isDone(time)) {

    // sim + plan
    simpl->update(time);

    // record mcap
    mcap->write(*simpl, time);

    // tick
    time += scenario.timeParams.dt;
  }

  const Results results = simpl->getResults();

  simpl.reset();
  mcap.reset();

  return results;
}
} // namespace
