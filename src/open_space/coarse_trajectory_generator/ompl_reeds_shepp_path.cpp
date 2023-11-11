#include "ompl_reeds_shepp_path.h"
#include <algorithm>

OmplRSGenerator::OmplRSGenerator(double min_radius)
: min_turn_radius_(min_radius),
    bx_(10), by_(10), bounds_(2)
{
    reedsSheppStateSpace_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turn_radius_);
    simpleSetup_ = std::make_shared<ompl::geometric::SimpleSetup>(reedsSheppStateSpace_);
    max_plan_time_ = 0.2;
    display_planner_output_ = true;
    interpolation_num_poses_ = 30;

    setBoundaries(40.0, 40.0);
}

OmplRSGenerator::~OmplRSGenerator() {}

void OmplRSGenerator::setBoundaries(const double bx, const double by)
{
    bx_ = bx;
    by_ = by;
    bounds_.low[0] = -by_ / 2 - 0.1;
    bounds_.low[1] = -bx_ / 2 - 0.1;
    bounds_.high[0] = by_ / 2 + 0.1;
    bounds_.high[1] = bx_ / 2 + 0.1;
    reedsSheppStateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds_);
}


void OmplRSGenerator::state2pose( const ompl::base::State* state, common::State& pose)
{
    const ompl::base::SE2StateSpace::StateType *s =
        state->as<ompl::base::SE2StateSpace::StateType>();
    pose.set_x(s->getX());
    pose.set_y(s->getY());
    pose.set_theta(s->getYaw());
}


void OmplRSGenerator::pose2state(
    const common::State& pose, ompl::base::State* state)
{
    ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    s->setX(pose.x());
    s->setY(pose.y());
    s->setYaw(pose.theta());
}


bool OmplRSGenerator::isStateValid(
    const ompl::base::SpaceInformation* si, const ompl::base::State *state)
{
    // check if state is inside boundary
    if (!si->satisfiesBounds(state))
      return false;

    return true;
}


bool OmplRSGenerator::planPath(
    const common::State& start_pose,
    const common::State& goal_pose,
    std::vector<common::State>& result_path)
{
    // disable planner console output
    if (!display_planner_output_)
    {
      std::cout.setstate(std::ios_base::failbit);
      std::cerr.setstate(std::ios_base::failbit);
    }

    // create start and goal states
    ompl::base::ScopedState<> start(reedsSheppStateSpace_);
    ompl::base::ScopedState<> goal(reedsSheppStateSpace_);

    // initialize state valididy checker
    ompl::base::SpaceInformationPtr si(simpleSetup_->getSpaceInformation());
    simpleSetup_->setStateValidityChecker(
      std::bind(&OmplRSGenerator::isStateValid, this, si.get(), std::placeholders::_1));

    // convert start and goal poses to ompl base states
    pose2state(start_pose, start());
    pose2state(goal_pose, goal());

    // clear all planning data
    simpleSetup_->clear();
    // set new start and goal states
    simpleSetup_->setStartAndGoalStates(start, goal);

    if (!simpleSetup_->solve(max_plan_time_)) {
        std::cerr << "[ompl rs_planner] failed !!!!" << std::endl;
        return false;
    }

    // simplify solution
    simpleSetup_->simplifySolution();

    // get solution path
    ompl::geometric::PathGeometric path = simpleSetup_->getSolutionPath();
    // interpolate between poses
    path.interpolate(interpolation_num_poses_);

    if (path.getStateCount() > interpolation_num_poses_)
    {
        std::cerr << "ompl rs path poses too less" << std::endl;
        return false;
    }

    // resize result_path
    result_path.resize(path.getStateCount());

    // convert each state to a pose and store it in result_path vector
    for (unsigned int i = 0; i < path.getStateCount(); i++)
    {
      const ompl::base::State* state = path.getState(i);
      state2pose(state, result_path[i]);
    }

    // enable console output
    std::cout.clear();
    std::cerr.clear();

    return true;
  }