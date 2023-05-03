# Valkyrie MoveIt Planner and Executor
Services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.

This package contains a node that provides services for planning and executing MoveIt arm trajectories on NASA's Valkyrie robot.  The package depends on MoveIt, including `moveit_msgs`, `moveit_core`, `moveit_ros_planning`, and `moveit_ros_planning_interface`, and [`val_safety_exception_reporter`](https://github.com/esheetz/val_safety_exception_reporter).  The package uses launch and config files from [`val_moveit_config`](https://js-er-code.jsc.nasa.gov/vs/val_moveit_config) and [`val_description`](https://js-er-code.jsc.nasa.gov/vs/val_description).  This package also allows execution of planned trajectories when the [IHMC Message Interface](https://github.com/esheetz/IHMCMsgInterface) is installed.  The server node also stores planned arm trajectories internally so that nodes using these services do not have to.

For an example of a node that uses these services, please see the `SemanticFrameControllerNode` in the [`val_dynacore` package](https://github.com/esheetz/val_dynacore).



## MoveIt Robot Demo

To launch a MoveIt demo on the robot, run:
```
roslaunch val_moveit_planner_executor val_moveit_utils.launch # optional: allow_sensors:=false to ignore map data
```

The `val_moveit_utils.launch` file launches several nodes that help run MoveIt on the robot.  RViz will launch with two interactive markers.  Use the MoveIt marker to move the robot around.  Plan and execute trajectories on Valkyrie using the larger interactive marker.



## MoveIt SCS Demo

To launch a MoveIt demo while the SCS sim is running, run:
```
roslaunch val_moveit_planner_executor val_moveit_utils.launch robot_running:=false simulate_robot_running:=true scs_running:=true allow_sensors:=false
```

The demo runs as usual.  Executing a trajectory will command the simulated robot in SCS to execute.



## MoveIt RViz Demo

To launch a MoveIt demo in RViz only, run:
```
roslaunch val_moveit_planner_executor val_moveit_utils.launch robot_running:=false
```

This launches the original MoveIt demo in `val_moveit_config`, but tests the services provided in this package.
