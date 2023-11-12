#include "open_space_map.h"
#include "rviz_tool/planner_viz.h"

#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hybrid_astar_demo");
    ros::NodeHandle node;

    PlannerViz viz(node);

    OpenSpaceMap osm;
    if (!osm.loadMap()) {
        AWARN << "openspace load map failed";
        return 1;
    }
    ros::Rate r(3);
    while(ros::ok()) {
        viz.showBounds(osm.getBound());
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 1;
}