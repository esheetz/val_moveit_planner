/**
 * MoveIt Planner Executor Server Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _MOVEIT_PLANNER_EXECUTOR_SERVER_NODE_
#define _MOVEIT_PLANNER_EXECUTOR_SERVER_NODE_

#include <chrono>
#include <map>
#include <utility> // std::pair

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <val_moveit_planner_executor/PlanToArmGoal.h>
#include <val_moveit_planner_executor/ExecuteToArmGoal.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveItPlannerExecutorServerNode
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	MoveItPlannerExecutorServerNode(const ros::NodeHandle& nh);
	~MoveItPlannerExecutorServerNode();

	// CONNECTIONS
	bool initializeConnections();

	// ADVERTISE SERVICES
	void advertiseServices();

	// CALLBACKS
	void octomapPointsCallback(const sensor_msgs::PointCloud2& msg);
	void octomapObstaclesCallback(const visualization_msgs::MarkerArray& msg);

	// SERVICE CALLBACKS
	bool planToArmGoalCallback(val_moveit_planner_executor::PlanToArmGoal::Request&  req,
							   val_moveit_planner_executor::PlanToArmGoal::Response& res);
	bool executeToArmGoalCallback(val_moveit_planner_executor::ExecuteToArmGoal::Request&  req,
								  val_moveit_planner_executor::ExecuteToArmGoal::Response& res);

	// GETTERS/SETTERS
	double getLoopRate();
	std::string getTabString();

	// OCTOMAP HELPERS
	bool initializeOctomapVariables();
	void markOctomapDataRecieved();
	bool checkOctomapTimeout();

	// MOVEIT HELPERS
	void initializeMoveItVariables();
	bool addPlanningSceneObstacles();
	bool addPlanningSceneObstacles(sensor_msgs::PointCloud2& point_cloud);
	bool addPlanningSceneObstacles(visualization_msgs::MarkerArray& obstacles);
	void clearPlanningSceneObstacles();

private:
	ros::NodeHandle nh_; // node handler
	ros::ServiceServer plan_to_arm_goal_service_; // PlanToArmGoal service
	ros::ServiceServer execute_to_arm_goal_service_; // ExecuteToArmGoal service

	std::string octomap_topic_;
	ros::Subscriber octomap_subscriber_;
	std::string ihmc_msg_interface_moveit_topic_;
	ros::Publisher ihmc_msg_interface_pub_;

	// point cloud information
	sensor_msgs::PointCloud2 point_cloud_;
	visualization_msgs::MarkerArray obstacles_;
	bool volumetric_points_;
	bool received_octomap_data_;
	double octomap_timeout_;
	std::chrono::system_clock::time_point last_octomap_data_received_;

	double loop_rate_; // loop rate for publishing

	// MoveIt interfaces
	static const std::string left_arm_planning_group;
	static const std::string right_arm_planning_group;
	moveit::planning_interface::MoveGroupInterface move_group_interface_larm_;
	moveit::planning_interface::MoveGroupInterface move_group_interface_rarm_;
	const moveit::core::JointModelGroup* joint_model_group_larm_;
	const moveit::core::JointModelGroup* joint_model_group_rarm_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
	bool plan_exists_;
	moveit_msgs::RobotTrajectory planned_trajectory_;


}; // end class MoveItPlannerExecutorServerNode

#endif
