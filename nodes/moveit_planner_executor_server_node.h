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
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <val_moveit_planner_executor/PlanToArmGoal.h>
#include <val_moveit_planner_executor/ExecuteToArmGoal.h>

#include <val_safety_exception_reporter/CannotExecuteMotion.h>
#include <val_safety_exception_reporter/CannotGetMotionPlan.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class MoveItPlannerExecutorServerNode
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	MoveItPlannerExecutorServerNode(const ros::NodeHandle& nh);
	~MoveItPlannerExecutorServerNode();

	// CONNECTIONS
	bool initializeConnections();

	// SERVICE CLIENTS
	void initializeServiceClients();

	// ADVERTISE SERVICES
	void advertiseServices();

	// SERVICE CALLBACKS
	bool planToArmGoalCallback(val_moveit_planner_executor::PlanToArmGoal::Request&  req,
							   val_moveit_planner_executor::PlanToArmGoal::Response& res);
	bool executeToArmGoalCallback(val_moveit_planner_executor::ExecuteToArmGoal::Request&  req,
								  val_moveit_planner_executor::ExecuteToArmGoal::Response& res);

	// GETTERS/SETTERS
	double getLoopRate();
	std::string getTabString();

	// TRANSFORM HELPERS
	bool transformPoseToTargetFrame(std::string target_frame,
									geometry_msgs::PoseStamped source_pose,
									geometry_msgs::PoseStamped& target_pose);

	// MOVEIT HELPERS
	void initializeMoveItVariables();

	// IHMCMsgInterface HELPERS
	void tellIHMCMsgInterfaceReceiveMoveItTrajectories();

	// SAFETY REPORTER HELPERS
	void publishSafetyReportClosestArm(geometry_msgs::PoseStamped requested_target);
	void publishSafetyReportInvalidGroup(geometry_msgs::PoseStamped requested_target);
	void publishSafetyReportCollisionAwarePlanning(std::string planning_group, geometry_msgs::PoseStamped requested_target);
	void publishSafetyReportWorldFrameTransform(std::string planning_group, geometry_msgs::PoseStamped requested_target);
	void publishSafetyReportNoPlanFound(std::string planning_group, geometry_msgs::PoseStamped requested_target);
	void publishCannotGetMotionPlanMessage(std::string planning_exception,
										   std::string planning_group,
										   geometry_msgs::PoseStamped requested_target);
	void publishSafetyReportNoStoredTrajectory();
	void publishCannotExecuteMotionMessage(std::string execution_exception);

private:
	ros::NodeHandle nh_; // node handler
	tf::TransformListener tf_; // transforms between frames

	ros::ServiceServer plan_to_arm_goal_service_; // PlanToArmGoal service
	ros::ServiceServer execute_to_arm_goal_service_; // ExecuteToArmGoal service
	ros::ServiceClient plan_kinematic_path_client_; // GetMotionPlan client

	std::string ihmc_msg_interface_moveit_topic_;
	ros::Publisher ihmc_msg_interface_pub_;
	std::string ihmc_msg_interface_recv_moveit_traj_topic_;
	ros::Publisher ihmc_msg_interface_recv_moveit_traj_pub_;
	int ihmc_msg_interface_recv_pub_counter_;

	ros::Publisher safety_reporter_plan_pub_;
	ros::Publisher safety_reporter_execute_pub_;

	// MoveIt interfaces
	static const std::string left_arm_planning_group;
	static const std::string right_arm_planning_group;
	moveit::planning_interface::MoveGroupInterface move_group_interface_larm_;
	moveit::planning_interface::MoveGroupInterface move_group_interface_rarm_;

	// internal storage for MoveIt plans
	bool plan_exists_;
	moveit_msgs::RobotTrajectory planned_trajectory_;

	double loop_rate_; // loop rate for publishing

}; // end class MoveItPlannerExecutorServerNode

#endif
