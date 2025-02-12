#define MCAP_IMPLEMENTATION

#include "planning/mcap-utils.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"

#include "foxglove/Grid.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PackedElementField.pb.h"
#include "foxglove/PosesInFrame.pb.h"
#include "foxglove/SceneUpdate.pb.h"

#include "build-proto-fds/build-proto-fds.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>

namespace planning {

namespace {

/// eigen helper
Eigen::Quaterniond MakeQuaternion(double angle);

/// foxglove proto helpers
foxglove::SceneUpdate MakeActorSceneUpdate(const Scene &scene, int64_t time);
foxglove::SceneUpdate MakeGraphSceneUpdate(const Graph &graph, int64_t time);
foxglove::SceneUpdate MakePathSceneUpdate(const Path &path, int64_t time);
foxglove::SceneUpdate MakeScenarioSceneUpdate(const Scenario &scenario,
                                              double writeTime);
foxglove::LinePrimitive MakePath(const Path &path);
foxglove::LinePrimitive MakeFootprint(const Footprint &footprint);
foxglove::ArrowPrimitive MakeArrowPrimitive(const Pose &pose);
foxglove::Grid MakeGrid(const std::shared_ptr<OccupancyGrid> &grid,
                        const HookedPose &revoyPose, int64_t writeTime);

const static std::string GRID_TOPIC = "occupancy";
const static std::string ACTORS_TOPIC = "actors";
const static std::string COARSE_GRAPH_TOPIC = "coarse-graph";
const static std::string COARSE_PATH_TOPIC = "coarse-path";
const static std::string CONTROL_GRAPH_TOPIC = "control-graph";
const static std::string CONTROL_PATH_TOPIC = "control-path";
const static std::string SCENARIO_TOPIC = "scenario";

} // namespace

McapWrapper::McapWrapper(const std::string outputFilename) {

  auto options = mcap::McapWriterOptions("");
  const auto res = writer.open(outputFilename, options);

  if (!res.ok()) {
    std::cerr << "Failed to open " << outputFilename
              << " for writing: " << res.message << std::endl;
    exit(1);
  }

  addTopic("foxglove.Grid",
           revoy::BuildFileDescriptorSet(foxglove::Grid::descriptor())
               .SerializeAsString(),
           GRID_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           ACTORS_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           COARSE_GRAPH_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           COARSE_PATH_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           CONTROL_GRAPH_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           CONTROL_PATH_TOPIC);

  addTopic("foxglove.SceneUpdate",
           revoy::BuildFileDescriptorSet(foxglove::SceneUpdate::descriptor())
               .SerializeAsString(),
           SCENARIO_TOPIC);
}

void McapWrapper::write(const Scene &scene, int64_t writeTime) {

std::cout << "mcap write" << std::endl;
  
  assert(scene.graphs.size() == 2);
  assert(scene.plannedPaths.size() == 2);

  const auto actors =
      MakeActorSceneUpdate(scene, writeTime).SerializeAsString();
  writeTopic(actors, ACTORS_TOPIC, writeTime);

  /// COARSE PLAN
  const auto coarseGraph =
      MakeGraphSceneUpdate(scene.graphs[0], writeTime).SerializeAsString();
  writeTopic(coarseGraph, COARSE_GRAPH_TOPIC, writeTime);
  const auto coarsePath =
      MakePathSceneUpdate(scene.plannedPaths[0], writeTime).SerializeAsString();
  writeTopic(coarsePath, COARSE_PATH_TOPIC, writeTime);

  // /// CONTROL PLAN
  const auto controlGraph =
      MakeGraphSceneUpdate(scene.graphs[1], writeTime).SerializeAsString();
  writeTopic(controlGraph, CONTROL_GRAPH_TOPIC, writeTime);
  const auto controlPath =
      MakePathSceneUpdate(scene.plannedPaths[1], writeTime).SerializeAsString();
  writeTopic(controlPath, CONTROL_PATH_TOPIC, writeTime);

  /// GRID
  const auto grid =
      MakeGrid(scene.grid, scene.revoyPose, writeTime).SerializeAsString();
  writeTopic(grid, GRID_TOPIC, writeTime);

  /// SCENARIO
  const auto scenario =
      MakeScenarioSceneUpdate(scene.scenario, writeTime).SerializeAsString();
  writeTopic(scenario, SCENARIO_TOPIC, writeTime);

  frameIndex++;

std::cout << "mcap write end" << std::endl;

  return;
}

void McapWrapper::addTopic(const std::string &type, const std::string &desc,
                           const std::string &topic) {
  mcap::Schema schema(type, "protobuf", desc);
  writer.addSchema(schema);

  mcap::Channel channel(topic, "protobuf", schema.id);
  writer.addChannel(channel);
  channelIds.insert({std::string(topic), channel.id});
};

void McapWrapper::writeTopic(const std::string &serialized,
                             const std::string &topic, double writeTime) {

  mcap::Message msg;
  msg.channelId = channelIds.at(topic);
  msg.sequence = static_cast<uint32_t>(frameIndex);
  msg.publishTime =
      static_cast<uint64_t>(writeTime) * static_cast<int64_t>(1e3);
  msg.logTime = static_cast<uint64_t>(writeTime) * static_cast<int64_t>(1e3);
  msg.data = reinterpret_cast<const std::byte *>(serialized.data());
  msg.dataSize = serialized.size();
  const auto res = writer.write(msg);
  if (!res.ok()) {
    std::cerr << "Failed to write message: " << res.message << "\n";
    writer.terminate();
    return;
  }
}

McapWrapper::~McapWrapper() { writer.close(); }

namespace {

Eigen::Quaterniond MakeQuaternion(double angle) {
  Eigen::Vector3d axis(0, 0, 1);
  Eigen::AngleAxisd angleAxis(angle, axis);
  Eigen::Quaterniond quaternion(angleAxis);
  return quaternion;
}

foxglove::Pose MakePose(const Pose &pose) {

  foxglove::Pose fgPose;

  // 3D position
  fgPose.mutable_position()->set_x(pose.position.x());
  fgPose.mutable_position()->set_y(pose.position.y());

  // 3D orientation
  const Eigen::Quaterniond quaternion = MakeQuaternion(pose.yaw);
  fgPose.mutable_orientation()->set_w(quaternion.w());
  fgPose.mutable_orientation()->set_x(quaternion.x());
  fgPose.mutable_orientation()->set_y(quaternion.y());
  fgPose.mutable_orientation()->set_z(quaternion.z());

  return fgPose;
}

foxglove::SceneUpdate MakeActorSceneUpdate(const Scene &scene,
                                           int64_t writeTime) {
  foxglove::SceneUpdate sceneUpdate;

  foxglove::SceneEntity fgActors;
  fgActors.mutable_lifetime()->set_seconds(0);
  fgActors.mutable_lifetime()->set_nanos(1e8);
  fgActors.set_frame_id("fixed");
  fgActors.set_id("entities");
  fgActors.mutable_timestamp()->set_seconds(writeTime);
  for (const auto &obstacle : scene.scenario.entities) {
    if (obstacle.lifetime + scene.scenario.timeParams.startTime < writeTime) {
      continue;
    }
    const auto tfFootprint =
        TransformFootprint(obstacle.footprint, obstacle.pose);
    const uint8_t index = static_cast<uint8_t>(fgActors.lines_size());
    fgActors.mutable_lines()->Add(MakeFootprint(tfFootprint));
    fgActors.mutable_lines()->Mutable(index)->mutable_color()->set_r(1);
    fgActors.mutable_lines()->Mutable(index)->mutable_color()->set_g(0);
    fgActors.mutable_lines()->Mutable(index)->mutable_color()->set_b(0);
    fgActors.mutable_lines()->Mutable(index)->mutable_color()->set_a(0.5);
  }
  sceneUpdate.mutable_entities()->Add(std::move(fgActors));

  foxglove::SceneEntity fgVisibleEntities;
  fgVisibleEntities.mutable_lifetime()->set_seconds(0);
  fgVisibleEntities.mutable_lifetime()->set_nanos(1e8);
  fgVisibleEntities.set_frame_id("fixed");
  fgVisibleEntities.set_id("visible_entities");
  fgVisibleEntities.mutable_timestamp()->set_seconds(writeTime);
  for (const auto &footprint : scene.visibleEntities) {
    const uint8_t index = static_cast<uint8_t>(fgVisibleEntities.lines_size());
    fgVisibleEntities.mutable_lines()->Add(MakeFootprint(footprint));
    fgVisibleEntities.mutable_lines()->Mutable(index)->mutable_color()->set_r(
        1);
    fgVisibleEntities.mutable_lines()->Mutable(index)->mutable_color()->set_a(
        1);
  }
  sceneUpdate.mutable_entities()->Add(std::move(fgVisibleEntities));

  foxglove::SceneEntity fgRevoy;
  fgRevoy.set_frame_id("fixed");
  fgRevoy.set_id("revoy");
  fgRevoy.mutable_timestamp()->set_seconds(writeTime);

  assert(scene.revoy.size() > 0);
  const Footprint revoy = scene.revoy[0];
  fgRevoy.mutable_lines()->Add(MakeFootprint(revoy));

  // first segment (revoy) should be orange
  fgRevoy.mutable_lines()->Mutable(0)->mutable_color()->set_r(1);
  fgRevoy.mutable_lines()->Mutable(0)->mutable_color()->set_g(0.5);
  fgRevoy.mutable_lines()->Mutable(0)->mutable_color()->set_a(1);
  fgRevoy.mutable_lines()->Mutable(0)->set_thickness(0.05);
  fgRevoy.mutable_lines()->Mutable(0)->set_scale_invariant(false);
  sceneUpdate.mutable_entities()->Add(std::move(fgRevoy));

  for (uint8_t index = 1; index < scene.revoy.size(); index++) {
    foxglove::SceneEntity fgTrailer;
    fgTrailer.set_frame_id("fixed");
    fgTrailer.set_id("trailer_" + std::to_string(index));
    fgTrailer.mutable_timestamp()->set_seconds(writeTime);
    fgTrailer.mutable_lines()->Add(MakeFootprint(scene.revoy[index]));

    // second segment (i.e. trailer) should be white
    fgTrailer.mutable_lines()->Mutable(0)->mutable_color()->set_r(1);
    fgTrailer.mutable_lines()->Mutable(0)->mutable_color()->set_g(1);
    fgTrailer.mutable_lines()->Mutable(0)->mutable_color()->set_b(1);
    fgTrailer.mutable_lines()->Mutable(0)->mutable_color()->set_a(1);
    fgTrailer.mutable_lines()->Mutable(0)->set_thickness(0.05);
    fgTrailer.mutable_lines()->Mutable(0)->set_scale_invariant(false);
    sceneUpdate.mutable_entities()->Add(std::move(fgTrailer));
  }

  return sceneUpdate;
}

foxglove::SceneUpdate MakeGraphSceneUpdate(const Graph &graph,
                                           int64_t writeTime) {
  foxglove::SceneUpdate sceneUpdate;

  foxglove::SceneEntity fgGraph;

  fgGraph.set_frame_id("fixed");
  fgGraph.set_id("graph");
  fgGraph.mutable_timestamp()->set_seconds(writeTime);

  foxglove::LinePrimitive nodePoints;
  nodePoints.set_type(foxglove::LinePrimitive::LINE_LIST);
  nodePoints.set_thickness(0.125);
  nodePoints.set_scale_invariant(false);

  for (const auto &node : graph.nodes) {
    foxglove::Point3 fgPoint;
    fgPoint.set_x(node.x());
    fgPoint.set_y(node.y());
    fgPoint.set_z(0.0);
    nodePoints.mutable_points()->Add(std::move(fgPoint));
  }

  nodePoints.mutable_color()->set_r(1.0);
  nodePoints.mutable_color()->set_g(1.0);
  nodePoints.mutable_color()->set_b(1.0);
  nodePoints.mutable_color()->set_a(0.3);

  fgGraph.mutable_lines()->Add(std::move(nodePoints));

  for (const auto &[start_idx, end_idx] : graph.edges) {
    foxglove::LinePrimitive edgeLine;
    edgeLine.set_type(foxglove::LinePrimitive::LINE_LIST);
    edgeLine.set_thickness(0.01);
    edgeLine.set_scale_invariant(false);

    foxglove::Point3 startPoint;
    startPoint.set_x(graph.nodes[start_idx].x());
    startPoint.set_y(graph.nodes[start_idx].y());
    startPoint.set_z(0.0);
    edgeLine.mutable_points()->Add(std::move(startPoint));

    foxglove::Point3 endPoint;
    endPoint.set_x(graph.nodes[end_idx].x());
    endPoint.set_y(graph.nodes[end_idx].y());
    endPoint.set_z(0.0);
    edgeLine.mutable_points()->Add(std::move(endPoint));

    edgeLine.mutable_color()->set_r(1.0);
    edgeLine.mutable_color()->set_g(1.0);
    edgeLine.mutable_color()->set_b(1.0);
    edgeLine.mutable_color()->set_a(1.0);

    fgGraph.mutable_lines()->Add(std::move(edgeLine));
  }

  sceneUpdate.mutable_entities()->Add(std::move(fgGraph));

  return sceneUpdate;
}

foxglove::SceneUpdate MakeScenarioSceneUpdate(const Scenario &scenario,
                                              double writeTime) {
  foxglove::SceneUpdate sceneUpdate;
  const std::vector<Point> limits{
      {scenario.bounds.upperX, scenario.bounds.upperY},
      {scenario.bounds.lowerX, scenario.bounds.upperY},
      {scenario.bounds.lowerX, scenario.bounds.lowerY},
      {scenario.bounds.upperX, scenario.bounds.lowerY},
      {scenario.bounds.upperX, scenario.bounds.upperY},
  };

  foxglove::SceneEntity fgLimits;
  fgLimits.set_frame_id("fixed");
  fgLimits.set_id("limits");
  fgLimits.mutable_timestamp()->set_seconds(writeTime);
  fgLimits.mutable_lines()->Add(MakeFootprint(limits));
  fgLimits.mutable_lines()->Mutable(0)->mutable_color()->set_a(1);
  sceneUpdate.mutable_entities()->Add(std::move(fgLimits));

  foxglove::SceneEntity fgStart;
  fgStart.set_frame_id("fixed");
  fgStart.set_id("start");
  fgStart.mutable_timestamp()->set_seconds(writeTime);
  fgStart.mutable_arrows()->Add(
      MakeArrowPrimitive({scenario.start.position, scenario.start.yaw}));
  fgStart.mutable_arrows()->Mutable(0)->mutable_color()->set_b(1);
  fgStart.mutable_arrows()->Mutable(0)->mutable_color()->set_a(0.5);
  sceneUpdate.mutable_entities()->Add(std::move(fgStart));

  foxglove::SceneEntity fgGoal;
  fgGoal.set_frame_id("fixed");
  fgGoal.set_id("goal");
  fgGoal.mutable_timestamp()->set_seconds(writeTime);
  fgGoal.mutable_arrows()->Add(
      MakeArrowPrimitive({scenario.goal.position, scenario.goal.yaw}));
  fgGoal.mutable_arrows()->Mutable(0)->mutable_color()->set_g(1);
  fgGoal.mutable_arrows()->Mutable(0)->mutable_color()->set_a(0.5);
  sceneUpdate.mutable_entities()->Add(std::move(fgGoal));

  return sceneUpdate;
}

foxglove::SceneUpdate MakePathSceneUpdate(const Path &path, int64_t writeTime) {
  foxglove::SceneUpdate sceneUpdate;

  foxglove::SceneEntity fgPath;
  fgPath.set_frame_id("fixed");
  fgPath.set_id("path");
  fgPath.mutable_timestamp()->set_seconds(writeTime);
  fgPath.mutable_lines()->Add(MakePath(path));
  fgPath.mutable_lines()->Mutable(0)->mutable_color()->set_r(1);
  fgPath.mutable_lines()->Mutable(0)->mutable_color()->set_g(1);
  fgPath.mutable_lines()->Mutable(0)->mutable_color()->set_b(0);
  fgPath.mutable_lines()->Mutable(0)->mutable_color()->set_a(1);
  sceneUpdate.mutable_entities()->Add(std::move(fgPath));

  return sceneUpdate;
}

foxglove::LinePrimitive MakePath(const std::vector<Point> &path) {
  foxglove::LinePrimitive line;

  line.set_type(foxglove::LinePrimitive::LINE_STRIP);
  line.set_thickness(0.1);
  line.set_scale_invariant(false);

  for (const auto &point : path) {
    foxglove::Point3 fgPoint;
    fgPoint.set_x(point.x());
    fgPoint.set_y(point.y());
    line.mutable_points()->Add(std::move(fgPoint));
  }

  return line;
}

foxglove::LinePrimitive MakeFootprint(const Footprint &footprint) {
  foxglove::LinePrimitive line;

  line.set_type(foxglove::LinePrimitive::LINE_STRIP);
  line.set_thickness(0.25);
  line.set_scale_invariant(false);

  for (const auto &point : footprint) {
    foxglove::Point3 fgPoint;
    fgPoint.set_x(point.x());
    fgPoint.set_y(point.y());
    line.mutable_points()->Add(std::move(fgPoint));
  }

  // close the footprint
  foxglove::Point3 fgPoint;
  fgPoint.set_x(footprint[0].x());
  fgPoint.set_y(footprint[0].y());
  line.mutable_points()->Add(std::move(fgPoint));
  return line;
}

foxglove::ArrowPrimitive MakeArrowPrimitive(const Pose &pose) {
  foxglove::ArrowPrimitive arrow;

  *arrow.mutable_pose() = MakePose(pose);
  arrow.set_shaft_length(1);
  arrow.set_shaft_diameter(0.1);
  arrow.set_head_length(0.5);
  arrow.set_head_diameter(0.3);

  return arrow;
}

foxglove::Grid MakeGrid(const std::shared_ptr<OccupancyGrid> &grid,
                        const HookedPose &revoyPose, int64_t writeTime) {
  foxglove::Grid fgGrid;
  fgGrid.mutable_timestamp()->set_seconds(writeTime);
  fgGrid.set_frame_id("fixed");
  fgGrid.set_column_count(grid->getM());
  fgGrid.set_cell_stride(1);
  fgGrid.set_row_stride(grid->getM());
  fgGrid.mutable_cell_size()->set_x(grid->getCellX());
  fgGrid.mutable_cell_size()->set_y(grid->getCellY());

  /// "Origin of grid's corner relative to frame of reference; grid is
  /// positioned in the x-y plane relative to this origin"

  // first we orient the grid to face the same way as ego
  Pose gridPose = {{}, -M_PI / 2.0};
  gridPose.yaw = fixRadian(gridPose.yaw + revoyPose.yaw);
  gridPose.position = revoyPose.position;

  // then we offset the grid so that it is centered on ego
  Point xyOffset = {-grid->getOffsetX(), grid->getOffsetY()};
  xyOffset = RotatePoint(xyOffset, revoyPose.yaw);
  gridPose.position += xyOffset;
  *fgGrid.mutable_pose() = MakePose(gridPose);

  foxglove::PackedElementField fgFields;
  fgFields.set_name("isOccupied");
  fgFields.set_offset(0);
  fgFields.set_type(foxglove::PackedElementField::NumericType::
                        PackedElementField_NumericType_UINT8);

  fgGrid.mutable_fields()->Add(std::move(fgFields));
  fgGrid.set_data(grid->fillData());
  return fgGrid;
}

} // namespace
} // namespace planning
