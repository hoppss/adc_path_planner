cmake_minimum_required(VERSION 2.8.3)
project(adc_path_planner)

add_compile_options(--std=c++14)

find_package(catkin REQUIRED COMPONENTS 
                            roscpp
                            rospy
                            std_msgs
                            geometry_msgs
                            nav_msgs
                            abseil_cpp
                            tf2_ros
                            tf2_geometry_msgs
)
find_package (glog REQUIRED)   #0.6.0 源码安装
find_package(osqp REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES planner_common
  DEPENDS
)

find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${glog_INCLUDE_DIRS}
    /usr/local/include
)

link_directories(/usr/local/lib/)

set(COMMON_SRC  src/common/vec2d.cpp
                src/common/aabox2d.cpp
                src/common/box2d.cpp
                src/common/line_segment2d.cpp
                src/common/math_util.cpp
                src/common/linear_interpolation.cpp
                src/common/polygon2d.cpp
                src/common/obstacle.cpp

                src/common/discrete_points_math.cpp
                src/common/discretized_path.cpp
                src/common/type_tool.cpp
                src/common/vehicle_config_helper.cpp
                src/receiver/receiver.cpp
                src/rviz_tool/rviz_tool.cpp
                src/rviz_tool/planner_viz.cpp

                src/reference_line/traj_file_tool.cpp
                src/reference_line/uos_traj_file_tool.cpp

                src/reference_line/discrete_points_smoother.cpp

                src/reference_line/fem_pose_deviation_osqp_interface.cpp
                src/reference_line/fem_pose_deviation_sqp_interface.cpp

                # collision check
                src/common/collision_checker.cpp

                # open space
                src/open_space/coarse_trajectory_generator/node3d.cpp
                src/open_space/coarse_trajectory_generator/reeds_sheep_path.cpp
)
# basic library
add_library(planner_common ${COMMON_SRC})
target_link_libraries(planner_common ${catkin_LIBRARIES}
                              ${abseil_cpp_LIBRARIES}
                              glog::glog
                              osqp::osqp
)

########## sub directory project ##########
## glog
add_subdirectory(src/app)

# discrete points smoother
add_subdirectory(src/reference_line)

#tk spline demo
add_subdirectory(src/tkspline)

#apa parking plan
add_subdirectory(src/open_space)
add_subdirectory(src/open_space/geometry_plan)