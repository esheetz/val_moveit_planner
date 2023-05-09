#---------------------------------------------------------------------------
#     Package Setup
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(val_moveit_planner_executor)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  sensor_msgs
  visualization_msgs
  val_safety_exception_reporter
  message_generation
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

#------------------------------------------------------------------------
#     dynamic_reconfigure Param Config Setup
#------------------------------------------------------------------------
generate_dynamic_reconfigure_options(
  cfg/IMPlanningRequestParams.cfg
)

message("generated param config files!")

#------------------------------------------------------------------------
#     ROS Messages and Services Setup
#------------------------------------------------------------------------
# generate messages
add_message_files(
  FILES
  ModifyAllowedCollision.msg
)

# generate services
add_service_files(
  FILES
  PlanToArmGoal.srv
  PlanToArmWaypoints.srv
  ExecuteToArmGoal.srv
  ModifyAllowedPlanningSceneCollisions.srv
)

# dependencies for generated messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
  moveit_msgs
)

message("generated message files!")
message("generated service files!")

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS message_runtime moveit_core moveit_ros_planning_interface roscpp rospy tf geometry_msgs moveit_msgs sensor_msgs visualization_msgs val_safety_exception_reporter
  DEPENDS EIGEN3
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
message("Eigen include directories: ${EIGEN3_INCLUDE_DIRS}")

#------------------------------------------------------------------------
#     Include Directories
#------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")
include_directories ("${PROJECT_SOURCE_DIR}/srv")

#---------------------------------------------------------------------------
#     Directory Selection
#---------------------------------------------------------------------------
add_subdirectory (nodes)
