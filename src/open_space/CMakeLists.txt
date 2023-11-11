## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

include_directories(include ${OMPL_INCLUDE_DIRS})

add_executable(rs_demo      rs_curve_demo.cpp
                            coarse_trajectory_generator/ompl_reeds_shepp_path.cpp)
target_link_libraries(rs_demo  planner_common)
target_link_libraries(rs_demo ${OMPL_LIBRARIES})