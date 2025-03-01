#pragma once

#include "planning/occupancy-grid.h"

#include "planning/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace planning {

// Makes a coarse plan towards the Goal, not guaranteed to actually be
// followable. Use the output of this to guide a more high-dimensional search.
class CoarsePlanner {
public:
  CoarsePlanner() = delete;
  CoarsePlanner(const Bounds &bounds, const BodyParams &bodyParams);

  // Completes ompl setup, calls ompl solve
  void plan(const HookedPose &start, const HookedPose &goal,
            const Footprints &undriveableAreas);

  // ompl hooks that we define, pass data / query the StateSpace
  class ValidityChecker;

  /// getters used for debug
  const ompl::geometric::SimpleSetup &getSetup() const;

private:
  // Params, Inputs, Outputs
  Bounds bounds_ = {};

  // OMPL stuff
  std::shared_ptr<ompl::base::ReedsSheppStateSpace> space_;
  ompl::geometric::SimpleSetup setup_;
  std::shared_ptr<ValidityChecker> validityChecker_;

public:
  // OMPL will use this to decide if a State in the StateSpace is
  // valid, i.e. outside all undriveableAreas.
  class ValidityChecker : public ompl::base::StateValidityChecker {
  public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                    const BodyParams &bodyParams);

    // used by OMPL during search
    bool isValid(const ompl::base::State *state_) const override;

    // sets where OMPL should not search
    void setUndrivableAreas(const Footprints &undrivableAreas);

  private:
    BodyParams bodyParams_ = {};
    Footprints undrivableAreas_ = {};
  };

  class Flatland : public ompl::base::ReedsSheppStateSpace {
  public:
    Flatland(const Bounds &bounds) : ompl::base::ReedsSheppStateSpace(15.0) {
      ompl::base::RealVectorBounds rbounds(2);
      rbounds.low[0] = bounds.lowerX;
      rbounds.low[1] = bounds.lowerY;
      rbounds.high[0] = bounds.upperX;
      rbounds.high[1] = bounds.upperY;
      setBounds(rbounds);
    }

    class StateType : public ompl::base::ReedsSheppStateSpace::StateType {

    public:
      StateType() = default;
      double getTrailerYaw() const { return 0; };
      double getHitchAngle() const { return 0; };
      void setTrailerYaw(double _) {};
    };
  };
};
} // namespace planning
