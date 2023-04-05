# Valkyrie MoveIt Planner and Executor
Services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.

This package contains a node that provides services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.  The package depends on MoveIt, including `moveit_msgs`, `moveit_core`, `moveit_ros_planning`, and `moveit_ros_planning_interface`.  The package works best when the [`val_moveit_config`](https://js-er-code.jsc.nasa.gov/vs/val_moveit_config) and [IHMC Message Interface](https://github.com/esheetz/IHMCMsgInterface) are installed.  The server node also stores planned arm trajectories internally so that nodes using these services do not have to.

For an example of a node that uses these services, please see...



## Test MoveIt Planning
To launch the node, run:
```
roslaunch val_moveit_planner_executor moveit_planner_executor_server_node.launch
```
This launch file automatically launches the `val_moveit_config` demo for visualization purposes.

There are several scripts that will allow you to test making service calls to the node.  Run any of the following scripts to verify the node works and see the plan visualized in RViz:
```
./scripts/test_plan_left_arm_goal_service.sh
./scripts/test_plan_right_arm_goal_service.sh
./scripts/test_execute_stored_trajectory_service.sh
```
Note that the final script tests execution, which amounts to publishing a message for the IHMCMsgInterface node.  For right now, this will verify that the node handles execution services properly without executing anything on the robot.



## Test IHMC Execution of MoveIt Trajectories
Launch the IHMC Message Interface and the Valkyrie MoveIt node:
```
roslaunch ihmc_msg_interface ihmc_interface_node.launch
roslaunch val_moveit_planner_executor moveit_planner_executor_server_node.launch
```

Run any of the scripts to plan and execute a trajectory.  For initial testing, we recommend very small movements, which can be done using the following scripts:
```
./scripts/small_tests/test_plan_left_arm_small_goal_service.sh
./scripts/small_tests/test_plan_right_arm_small_goal_service.sh
./scripts/test_execute_stored_trajectory_service.sh
```
