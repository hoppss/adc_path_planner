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
)
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
)

# add_subdirectory(src/tkspline)
# add_subdirectory(src/tinyspline)

set(TKSPLINE_SRC    src/tkspline/spline.cpp)
set(TINYSPLINE_SRC  src/tineyspline/tinysplinecpp.cpp  
                    src/tineyspline/tinyspline.c)

set(COMMON_SRC  src/common/vec2d.cpp
                src/common/box2d.cpp
                src/common/line_segment2d.cpp
                src/common/math_util.cpp
                src/common/type_tool.cpp
                src/receiver/receiver.cpp
                src/rviz_tool/rviz_tool.cpp
                src/rviz_tool/planner_viz.cpp
)

add_executable(tk_test  src/app/tk_test.cpp
                        ${COMMON_SRC}
                        ${TKSPLINE_SRC}
)
add_dependencies(tk_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tk_test   ${catkin_LIBRARIES}
                                # tkspline
                                ${abseil_cpp_LIBRARIES}
)

install(TARGETS  tk_test
        ARCHIVE DESTINATION
        LIBRARY DESTINATION
        RUNTIME DESTINATION ${CATKIN_PACKAGE_DEVEL_DESTINATION}
)