#pragma once

#include "planning/occupancy-grid.h"
#include "planning/simpl.h"
#include "planning/types.h"

#include <map>
#include <vector>

#include "mcap/writer.hpp"

namespace planning {

// debug visualization from a planner
struct PlannerViz {
  Path solution;
  Graph graph;
};

// debug visualization of the simulation
struct Scene {
  Footprints revoy;
  HookedPose revoyPose;
  Scenario scenario;
  Footprints visibleEntities;
  const OccupancyGrid *occupancy;
  std::map<std::string, PlannerViz> planners;
};

// responsible for managing the topics, writing, and converting into
// foxglove_schema messages that are rendered in Foxglove
class SimplMcap final {

public:
  // initialize a new mcap file at outputFilename
  SimplMcap(const std::string outputFilename);
  ~SimplMcap();

  void write(const Simpl &simpl, int64_t writeTime);

  // TODO: scene is a leaky abstaction, move it into SimplMcap, and pass
  // a reference to Simpl here instead of Scene
  void write(const Scene &scene, int64_t writeTime);

private:
  // file handle, file writes
  mcap::McapWriter writer;

  // required by mcapWriter, topic-name to channel-id map
  std::map<std::string, mcap::ChannelId> channelIds;

  // we will increment this once per frame
  size_t frameIndex = 0;

  // convenience wrapper to add topic to mcap writer for later data writing.
  void addTopic(const std::string &type, const std::string &desc,
                const std::string &topic);

  // convenience wrapper to write data to a topic that was added prior.
  void writeTopic(const std::string &serialized, const std::string &topic,
                  double writeTime);

  // some massaging is required to go from Simpl to visualizable things,
  // e.g. some paths need to be interpolated (to show curvature), graph nodes
  // need to be collected, etc.
  static Scene SimplToScene(const Simpl &simpl, int64_t time);
};

} // namespace planning
