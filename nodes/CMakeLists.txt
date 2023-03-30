#-----------------------------------------------------------------------
# MoveIt Planner Executor Server Node:
# for planning and executing MoveIt trajectories
#-----------------------------------------------------------------------
add_executable(moveit_planner_executor_server_node moveit_planner_executor_server_node.cpp)
target_link_libraries(moveit_planner_executor_server_node ${catkin_LIBRARIES})
