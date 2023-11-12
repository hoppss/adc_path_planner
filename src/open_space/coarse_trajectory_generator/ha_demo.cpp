#include "common/basic_type.hpp"
#include "common/vehicle_config_helper.h"
#include "common/line_segment2d.h"

#include "open_space_map.h"
#include "rviz_tool/planner_viz.h"
#include "open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "open_space/planner_open_space_config.h"

#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hybrid_astar_demo");
    ros::NodeHandle node;
    common::VehicleParam  veh_param = common::VehicleConfigHelper::GetConfig().vehicle_param;
    AINFO << veh_param.debugString();

    PlannerViz viz(node);

    OpenSpaceMap osm;
    if (!osm.loadMap()) {
        AWARN << "openspace load map failed";
        return 1;
    }
    common::State goal = osm.getGoal();
    std::vector<std::vector<common::LineSegment2d>> bounds = osm.getBound();
    viz.showBounds(bounds);


    // decide astar bound
    std::vector<double> bound;
    bound.resize(4);
    double max_x = std::numeric_limits<double>::lowest();
    double min_x = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();

    for (size_t i=0; i<bounds[0].size(); ++i) {
        if (bounds[0][i].start().x() < min_x) {
            min_x = bounds[0][i].start().x();
        }
        if (bounds[0][i].start().x() > max_x) {
            max_x = bounds[0][i].start().x();
        }
        if (bounds[0][i].start().y() < min_y) {
            min_y = bounds[0][i].start().y();
        }
        if (bounds[0][i].start().y() > max_y) {
            max_y = bounds[0][i].start().y();
        }

        if (bounds[0][i].end().x() < min_x) {
            min_x = bounds[0][i].end().x();
        }
        if (bounds[0][i].end().x() > max_x) {
            max_x = bounds[0][i].end().x();
        }
        if (bounds[0][i].end().y() < min_y) {
            min_y = bounds[0][i].end().y();
        }
        if (bounds[0][i].end().y() > max_y) {
            max_y = bounds[0][i].end().y();
        }
    }
    bound[0] = min_x;
    bound[1] = max_x;
    bound[2] = min_y;
    bound[3] = max_y;
    AINFO << "Bound: xmin " << min_x << ", xmax: " << max_x << ", ymin: " << min_y << ", ymax: " << max_y;
    AINFO << "goal x: " << goal.x() << ", y: " << goal.y() << ", theta: " << goal.theta();

    PlannerOpenSpaceConfig config;
    planning::HybridAStar hastar(config);

    planning::HybridAStartResult result;
    if (!hastar.Plan(0, 0, 0, 10, 10, 1.57, bound, bounds, &result)) {
        AERROR << "hastar plan failed";
        return -1;
    }
    std::vector<common::State> path;

    for(size_t i=0; i<result.x.size(); ++i) {
        path.emplace_back(result.x[i], result.y[i], result.phi[i]);
    }
    ros::Rate r(3);
    while(ros::ok()) {
        viz.showBounds(osm.getBound());
        viz.showTrajectoryPath(path, 0);
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 1;
}