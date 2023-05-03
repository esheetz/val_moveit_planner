/**
 * MoveIt Planning Scene Collision Detector Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _MOVEIT_PLANNING_SCENE_COLLISION_DETECTOR_
#define _MOVEIT_PLANNING_SCENE_COLLISION_DETECTOR_

#include <map>
#include <utility> // std::pair

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

#include <val_moveit_planner_executor/ModifyAllowedCollision.h>
#include <val_moveit_planner_executor/ModifyAllowedPlanningSceneCollisions.h>

#include <val_safety_exception_reporter/Collisions.h>
#include <val_safety_exception_reporter/Collision.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class MoveItPlanningSceneCollisionDetector
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	MoveItPlanningSceneCollisionDetector(const ros::NodeHandle& nh);
	~MoveItPlanningSceneCollisionDetector();

	// CONNECTIONS
	bool initializeConnections();

	// ADVERTISE SERVICES
	void advertiseServices();

	// SERVICE CALLBACKS
	bool modifyAllowedCollisionsCallback(val_moveit_planner_executor::ModifyAllowedPlanningSceneCollisions::Request&  req,
										 val_moveit_planner_executor::ModifyAllowedPlanningSceneCollisions::Response& res);

	// GETTERS/SETTERS
	double getLoopRate();

	// MOVEIT HELPERS
	void initializeMoveItVariables();
	planning_scene_monitor::LockedPlanningSceneRO getCurrentPlanningSceneRO();
	planning_scene_monitor::LockedPlanningSceneRW getCurrentPlanningSceneRW();

	// SAFETY REPORTER HELPERS
	std::string getCollisionMessageBodyType(collision_detection::BodyType body_type);
	geometry_msgs::Point getCollisionMessagePoint(Eigen::Vector3d contact_pos);
	void makeCollisionMessage(collision_detection::Contact contact, val_safety_exception_reporter::Collision& collision_msg);
	void publishSafetyReportCollisionMessage(collision_detection::CollisionResult::ContactMap collision_contacts);

	// COLLISION DETECTION
	void checkPlanningSceneCollisions();
	bool allowPlanningSceneCollisions(std::vector<val_moveit_planner_executor::ModifyAllowedCollision> modify_collisions);

private:
	ros::NodeHandle nh_; // node handler

	ros::Publisher safety_reporter_pub_;

	ros::ServiceServer modify_allowed_collisions_service_; // ModifyAllowedPlanningSceneCollisions service
	ros::ServiceClient apply_planning_scene_client_; // ApplyPlanningScene client

	// MoveIt interfaces
	std::string planning_scene_service_;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

	double loop_rate_; // loop rate for publishing

}; // end class MoveItPlanningSceneCollisionDetector

#endif
