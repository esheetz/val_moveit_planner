#-----------------------------------------------------------------------
# MoveIt Planner Executor Server Node:
# for planning and executing MoveIt trajectories
#-----------------------------------------------------------------------
add_executable(moveit_planner_executor_server_node moveit_planner_executor_server_node.cpp)
target_link_libraries(moveit_planner_executor_server_node ${catkin_LIBRARIES})


#-----------------------------------------------------------------------
# MoveIt Filter Joint States Node:
# for filtering joint state messages for MoveIt
#-----------------------------------------------------------------------
add_executable(moveit_filter_joint_states_node moveit_filter_joint_states_node.cpp)
target_link_libraries(moveit_filter_joint_states_node ${catkin_LIBRARIES})

#-----------------------------------------------------------------------
# Arm Goal Interactive Marker Node:
# for (hack-ily) setting targets MoveIt plans
#-----------------------------------------------------------------------
install(PROGRAMS
	arm_goal_interactive_marker_node.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
