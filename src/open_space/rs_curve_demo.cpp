#include <vector>
#include <string>
#include <memory>

#include "receiver/receiver.h"
#include "rviz_tool/planner_viz.h"
#include "common/vehicle_config_helper.h"
#include "tools/timer_tool.hpp"

#include "open_space/planner_open_space_config.h"
#include "open_space/coarse_trajectory_generator/node3d.h"
#include "open_space/coarse_trajectory_generator/reeds_sheep_path.h"
#include "open_space/coarse_trajectory_generator/ompl_reeds_shepp_path.h"

#include "ros/ros.h"
#include "tf2/utils.h"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

using namespace planning;

int main(int argc, char** argv) {
    ros::init(argc, argv, "reed_sheep_planner");
    ros::NodeHandle node;

    common::VehicleParam veh_param = common::VehicleConfigHelper::GetConfig().vehicle_param;
    AINFO << "L  " << veh_param.wheel_base();
    AINFO << "max steering angle " << veh_param.max_steer_angle();
    AINFO << "default min radius" << veh_param.min_turn_radius();
    AINFO << "fixed min radius" << cal_vehicle_min_radius(veh_param);

    PlannerOpenSpaceConfig planner_open_space_config;

    ReedShepp reed_shepp_generator(veh_param, planner_open_space_config);
    OmplRSGenerator ompl_rs_generator(veh_param.min_turn_radius());

    Receiver receiver(node);
    PlannerViz viz(node);

    ros::Rate r(3);
    common::Timer timer;

    while (ros::ok()) {

        if (receiver.isReady()) {
            auto& start = receiver.getStart();
            auto& goal = receiver.getGoal();
            std::shared_ptr<Node3d> start_node =
                std::make_shared<Node3d>(start.position.x, start.position.y, tf2::getYaw(start.orientation));
            std::shared_ptr<Node3d> goal_node =
                std::make_shared<Node3d>(goal.position.x, goal.position.y, tf2::getYaw(goal.orientation));

            std::shared_ptr<ReedSheppPath> rs_path = std::make_shared<ReedSheppPath>();
            timer.begin();
            reed_shepp_generator.ShortestRSP(start_node, goal_node, rs_path);
            AINFO << "rs result " << rs_path->x.size() << ", time: " << timer.end() << " ms";
            std::vector<common::State> traj;
            for (size_t i=0; i<rs_path->x.size(); ++i) {
                traj.emplace_back(rs_path->x[i], rs_path->y[i], rs_path->phi[i]);
            }
            viz.showTrajectoryPath(traj, 0);

            // ompl get reedsshepp path
            // typedef ompl::base::SE2StateSpace::StateType Se2State;
            // ompl::base::ReedsSheppStateSpace reedsSheppPath(veh_param.min_turn_radius());
            // Se2State* rsStart = (Se2State*)reedsSheppPath.allocState();
            // Se2State* rsEnd = (Se2State*)reedsSheppPath.allocState();
            // rsStart->setXY(start_node->GetX(), start_node->GetY());
            // rsStart->setYaw(start_node->GetPhi());
            // rsEnd->setXY(goal_node->GetX(), goal_node->GetY());
            // rsEnd->setYaw(goal_node->GetPhi());
            // double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
            // AINFO << "ompl rs distance " << reedsSheppCost;
            std::vector<common::State> traj2;
            timer.begin();
            ompl_rs_generator.planPath(common::State(start.position.x, start.position.y, tf2::getYaw(start.orientation)),
                                       common::State(goal.position.x, goal.position.y, tf2::getYaw(goal.orientation)),
                                       traj2
                                      );
            AINFO << "ompl rs reuslt " <<  traj2.size() << ", time: " << timer.end() << " ms";
            viz.showTrajectoryPath(traj2, 1);
         }
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return EXIT_SUCCESS;
}