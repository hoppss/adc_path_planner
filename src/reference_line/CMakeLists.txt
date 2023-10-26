# discrete points smoother
add_executable(smoother_demo smoother_demo.cpp)
target_link_libraries(smoother_demo  planner_common)

# show uviz traj
add_executable(uos_viz_demo uos_viz_demo.cpp)
target_link_libraries(uos_viz_demo  planner_common)