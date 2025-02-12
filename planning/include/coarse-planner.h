#pragma once

#include "planning/occupancy-grid.h"
#include "planning/revoy-space.h"

#include "planning/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace planning {

/// Makes a coarse plan towards the Goal, not guaranteed to actually be
/// followable. Use the output of this to guide a more high-dimensional search.
class CoarsePlanner {
public:
  CoarsePlanner() = delete;
  CoarsePlanner(const Bounds &bounds, const BodyParams &bodyParams);

  /// Completes ompl setup, calls ompl solve
  void plan(const HookedPose &start, const HookedPose &goal,
            std::shared_ptr<OccupancyGrid> grid);

  /// getters used for output / debug
  const std::vector<Pose> &getLastSolution() const;
  const Graph &getLastGraph() const;

  /// ompl hooks that we define, pass data / query the StateSpace
  class ValidityChecker;

private:
  /// Params, Inputs, Outputs
  Bounds bounds_ = {};
  std::vector<Pose> path_ = {};
  Graph graph_ = {};

  /// OMPL stuff
  std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
  ompl::geometric::SimpleSetup setup_;
  std::shared_ptr<ValidityChecker> validityChecker_;

  /// Obstacles
  std::shared_ptr<OccupancyGrid> grid_;

public:
  /// OMPL will use this to decide if a State in the StateSpace is
  /// valid, i.e. outside all obstacles.
  class ValidityChecker : public ompl::base::StateValidityChecker {
  public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                    const BodyParams &bodyParams);

    /// used by OMPL during search
    bool isValid(const ompl::base::State *state_) const override;

    /// used by us to pass in latest obstacle positions
    void setOccupancyGrid(const std::shared_ptr<OccupancyGrid> grid,
                          const Pose &revoyPose);

    // void setFootprints(const Footprints &obstacles);

  private:
    BodyParams bodyParams_ = {};
    std::shared_ptr<OccupancyGrid> grid_;
    Pose currentPose_ = {};
  };

  class Flatland : public ompl::base::RealVectorStateSpace {
  public:
    Flatland(const Bounds &bounds) : ompl::base::RealVectorStateSpace(2) {
      ompl::base::RealVectorBounds rbounds(2);
      rbounds.low[0] = bounds.lowerX;
      rbounds.low[1] = bounds.lowerY;
      rbounds.high[0] = bounds.upperX;
      rbounds.high[1] = bounds.upperY;
      setBounds(rbounds);
    }

    class StateType : public ompl::base::RealVectorStateSpace::StateType {

    public:
      StateType() = default;
      double getX() const { return values[0]; };
      double getY() const { return values[1]; };
      double getYaw() const { return 0; };
      double getTrailerYaw() const { return 0; };
      double getHitchAngle() const { return 0; };
      void setX(double x) { values[0] = x; };
      void setY(double y) { values[1] = y; };
      void setXY(double x, double y) {
        setX(x);
        setY(y);
      };
      void setYaw(double _) {};
      void setTrailerYaw(double _) {};
    };
  };
};
} // namespace planning
