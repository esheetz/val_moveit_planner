#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
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
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  geometry_msgs
  moveit_msgs
  sensor_msgs
  visualization_msgs
  controller_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

#------------------------------------------------------------------------
#     ROS Setup
#------------------------------------------------------------------------
add_subdirectory (srv)

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS message_runtime moveit_core moveit_ros_planning_interface roscpp rospy geometry_msgs moveit_msgs sensor_msgs visualization_msgs controller_msgs
  DEPENDS EIGEN3
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
message("Eigen include directories: ${EIGEN3_INCLUDE_DIRS}")

#------------------------------------------------------------------------
#     Find ROS Services
#     Note:
#         ${CATKIN_DEVEL_PREFIX} is devel/.private/${PROJECT_NAME}
#         services are in devel/include
#         also works: ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#------------------------------------------------------------------------
set(SERVICE_DIR ${CATKIN_DEVEL_PREFIX}/../../${CATKIN_GLOBAL_INCLUDE_DESTINATION})
set(ANOTHER_SERVICE_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
message("")
message("Service directory: ${SERVICE_DIR}")
message("Another service directory: ${ANOTHER_SERVICE_DIR}")
message("")
include_directories ("${SERVICE_DIR}")
include_directories("${ANOTHER_SERVICE_DIR}")

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
