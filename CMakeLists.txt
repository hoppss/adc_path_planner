cmake_minimum_required(VERSION 2.8.3)
project(adc_path_planner)

add_compile_options(--std=c++11)

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
  LIBRARIES
  DEPENDS
)

find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    # ${glog_INCLUDE_DIRS}
    /usr/local/include
)

link_directories(/usr/local/lib/)
# message("glog found ?  ${glog_FOUND}")  ## 1
# message("glog include dir: ${glog_INCLUDE_DIRS}   ${glog_INCLUDES}")  ## empty
# message("glog lib dir: ${glog_LIBRARIES}")  ## empty


# add_subdirectory(src/tkspline)
# add_subdirectory(src/tinyspline)

set(TKSPLINE_SRC    src/tkspline/spline.cpp)
set(TINYSPLINE_SRC  src/tineyspline/tinysplinecpp.cpp  
                    src/tineyspline/tinyspline.c)

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

## tk_test
add_executable(tk_test  src/app/tk_test.cpp
                        ${COMMON_SRC}
                        ${TKSPLINE_SRC}
)
add_dependencies(tk_test  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                          ${catkin_EXPORTED_TARGETS}
                          )
target_link_libraries(tk_test ${catkin_LIBRARIES}
                              ${abseil_cpp_LIBRARIES}
                              glog::glog
                              osqp::osqp
)

# discrete points smoother
add_executable(smoother_demo src/reference_line/smoother_demo.cpp
                            ${COMMON_SRC}
)
add_dependencies(smoother_demo ${${PROJECT_NAME}_EXPORTED_TARGETS}
                              ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(smoother_demo  ${catkin_LIBRARIES}
                                    ${abseil_cpp_LIBRARIES}
                                    glog::glog
                                    # /usr/local/lib/libglog.so
                                    osqp::osqp
)

## show uviz traj
add_executable(uos_viz_demo src/reference_line/uos_viz_demo.cpp
                            ${COMMON_SRC}
)
add_dependencies(uos_viz_demo ${${PROJECT_NAME}_EXPORTED_TARGETS}
                              ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(uos_viz_demo  ${catkin_LIBRARIES}
                                    ${abseil_cpp_LIBRARIES}
                                    glog::glog
                                    # /usr/local/lib/libglog.so
                                    osqp::osqp
)


## geometry apa
add_executable(apa_plan     src/open_space/geometry_plan/main.cc
                            src/open_space/geometry_plan/geometry_parking_utils.cc
                            src/open_space/geometry_plan/line_circle_connection.cc
                            src/open_space/geometry_plan/reverse_verticle_parking.cc
                            ${COMMON_SRC}
)
# add_dependencies(apa_plan ${${PROJECT_NAME}_EXPORTED_TARGETS}
#                               ${catkin_EXPORTED_TARGETS}
# )
target_link_libraries(apa_plan  ${catkin_LIBRARIES}
                                    ${abseil_cpp_LIBRARIES}
                                    glog::glog
                                    osqp::osqp
)












## glog
add_executable(glog_demo src/app/glog_demo.cpp
)
add_dependencies(glog_demo ${${PROJECT_NAME}_EXPORTED_TARGETS}
                              ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(glog_demo  ${catkin_LIBRARIES}
                                    ${abseil_cpp_LIBRARIES}
                                    glog::glog
)



# install(TARGETS  tk_test
#         ARCHIVE DESTINATION
#         LIBRARY DESTINATION
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_DEVEL_DESTINATION}
# )