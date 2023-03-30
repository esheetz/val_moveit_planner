# Valkyrie MoveIt Planner and Executor
Services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.

This package contains a node that provides services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.  The package depends on MoveIt, including `moveit_msgs`, `moveit_core`, `moveit_ros_planning`, and `moveit_ros_planning_interface`.  The server node also stores planned arm trajectories internally so that nodes using these services do not have to.

For an example of a node that uses these services, please see...
