#pragma once

#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <map>
#include <vector>

#include "mcap/writer.hpp"

namespace planning {

struct Scene {
  Footprints revoy;
  HookedPose revoyPose;
  std::vector<Pose> coarseSolution;
  std::vector<HookedPose> controlSolution;
  Scenario scenario;
  Footprints visibleEntities;
  std::shared_ptr<OccupancyGrid> grid;
  Graph coarseGraph;
  Graph controlGraph;
};

class McapWrapper {
private:
  mcap::McapWriter writer;
  std::map<std::string, mcap::ChannelId> channelIds;
  size_t frameIndex = 0;
  void addTopic(const std::string &type, const std::string &desc,
                const std::string &topic);
  void writeTopic(const std::string &serialized, const std::string &topic,
                  double writeTime);

public:
  McapWrapper(const std::string outputFilename);
  ~McapWrapper();
  void write(const Scene &scene, int64_t writeTime);
};
} // namespace planning
