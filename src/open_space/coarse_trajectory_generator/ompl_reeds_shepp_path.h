#ifndef OMPL_REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H
#define OMPL_REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H

#include "common/basic_type.hpp"
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

class OmplRSGenerator
{
    public:

      OmplRSGenerator(double min_radius);
      ~OmplRSGenerator();

      bool planPath(
        const common::State& startPose,
        const common::State& goalPose,
        std::vector<common::State>& pathPoses);

      double getBX() {return bx_;}
      double getBY() {return by_;}
      void setMinTurningRadius(double rho) {min_turn_radius_ = rho;}
      void setBoundaries(double bx, double by);

    private:

      void state2pose(
        const ompl::base::State* state, common::State& pose);

      void pose2state(
        const common::State& pose, ompl::base::State* state);

      bool isStateValid(
        const ompl::base::SpaceInformation* si, const ompl::base::State *state);

    private:

      ompl::base::StateSpacePtr reedsSheppStateSpace_;
      ompl::geometric::SimpleSetupPtr simpleSetup_;
      ompl::base::RealVectorBounds bounds_;             // 2-d bound

      double min_turn_radius_;
      double max_plan_time_;
      int interpolation_num_poses_;
      int skip_poses;
      bool display_planner_output_;

      double bx_;   // x size in meters
      double by_;
};
#endif  // OMPL_REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H