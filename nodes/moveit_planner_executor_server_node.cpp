/**
 * MoveIt Planner Executor Server Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/moveit_planner_executor_server_node.h>

// STATIC MEMBERS
const std::string MoveItPlannerExecutorServerNode::left_arm_planning_group = std::string("left_arm");
const std::string MoveItPlannerExecutorServerNode::right_arm_planning_group = std::string("right_arm");

// CONSTRUCTORS/DESTRUCTORS
MoveItPlannerExecutorServerNode::MoveItPlannerExecutorServerNode(const ros::NodeHandle& nh)
    : move_group_interface_larm_(left_arm_planning_group),
      move_group_interface_rarm_(right_arm_planning_group) {
    
    nh_ = nh;

    // set up parameters
    nh_.param("octomap_topic", octomap_topic_, std::string("occupied_cells_vis_array"));
    nh_.param("ihmc_msg_interface_moveit_topic", ihmc_msg_interface_moveit_topic_, std::string("moveit_planned_robot_trajectory"));

    loop_rate_ = 10.0; // Hz

    initializeMoveItVariables();

    initializeOctomapVariables();

    initializeConnections();

    std::cout << "[MoveIt Planner Executor Server Node] Constructed" << std::endl;
}

MoveItPlannerExecutorServerNode::~MoveItPlannerExecutorServerNode() {
    std::cout << "[MoveIt Planner Executor Server Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool MoveItPlannerExecutorServerNode::initializeConnections() {
    if( volumetric_points_ ) {
        octomap_subscriber_ = nh_.subscribe(octomap_topic_, 1, &MoveItPlannerExecutorServerNode::octomapObstaclesCallback, this);
    }
    else {
        octomap_subscriber_ = nh_.subscribe(octomap_topic_, 1, &MoveItPlannerExecutorServerNode::octomapPointsCallback, this);
    }

    ihmc_msg_interface_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>(ihmc_msg_interface_moveit_topic_, 1);

    return true;
}

// ADVERTISE SERVICES
void MoveItPlannerExecutorServerNode::advertiseServices() {
    plan_to_arm_goal_service_ = nh_.advertiseService("plan_to_arm_goal", &MoveItPlannerExecutorServerNode::planToArmGoalCallback, this);
    execute_to_arm_goal_service_ = nh_.advertiseService("execute_to_arm_goal", &MoveItPlannerExecutorServerNode::executeToArmGoalCallback, this);

    ROS_INFO("[MoveIt Planner Executor Server Node] Providing services for planning and executing MoveIt trajectories!");
    
    return;
}

// CALLBACKS
void MoveItPlannerExecutorServerNode::octomapPointsCallback(const sensor_msgs::PointCloud2& msg) {
    if( volumetric_points_ ) {
        // ignore message
        return;
    }

    // store message
    point_cloud_ = msg;

    markOctomapDataRecieved();

    return;
}

void MoveItPlannerExecutorServerNode::octomapObstaclesCallback(const visualization_msgs::MarkerArray& msg) {
    if( !volumetric_points_ ) {
        // ignore message
        return;
    }

    // store message
    obstacles_ = msg;

    markOctomapDataRecieved();

    return;
}

// SERVICE CALLBACKS
bool MoveItPlannerExecutorServerNode::planToArmGoalCallback(val_moveit_planner_executor::PlanToArmGoal::Request&  req,
                                                            val_moveit_planner_executor::PlanToArmGoal::Response& res) {
    // ***** INITIALIZE VARIABLES *****

    // new planning request received; reset internally stored plan
    planned_trajectory_ = moveit_msgs::RobotTrajectory();
    plan_exists_ = false;

    // ***** PROCESS REQUESTED PLANNING GROUP ARM *****

    // initialize flag for move group
    bool use_left_move_group = false;

    // check which move group should handle this request
    if( req.planning_group_arm == req.LEFT_ARM ) {
        // set flag to use left move group
        use_left_move_group = true;
    }
    else if( req.planning_group_arm == req.RIGHT_ARM ) {
        // set flag to use right move group (not the left)
        use_left_move_group = false;
    }
    else if( req.planning_group_arm == req.ASSIGN_CLOSEST_ARM ) {
        // check which arm is closest based on y-coordinate of pose request
        if( req.arm_goal_pose.position.y > 0 ) {
            // set flag to use left move group
            use_left_move_group = true;
        }
        else { // req.arm_goal_pose.position.y <= 0
            // set flag to use right move group (not the left)
            use_left_move_group = false;
        }
    }
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Invalid planning group arm %d requested; expect 0,1,2", req.planning_group_arm);
        // invalid request, set response success to false
        res.success = false;
        return res.success;
    }

    // ***** PROCESS REQUESTED PLANNING SCENE *****

    // clear map
    clearPlanningSceneObstacles();

    // check if collision-aware planning is requested
    if( req.collision_aware_planning ) {
        // collision-aware planning requested; update planning scene with collision objects from octomap
        bool updated_planning_scene = addPlanningSceneObstacles();

        if( !updated_planning_scene ) {
            ROS_WARN("[MoveIt Planner Executor Server Node] Could not update planning scene; ignoring request");
            // cannot update planning scene, set response success to false
            res.success = false;
            return res.success;
        }
    }

    // ***** PROCESS PLANNING REQUEST *****

    // initialize plan
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
    // initialize success flag
    bool plan_success = false;

    // set pose for correct move group
    if( use_left_move_group ) {
        // set pose target for left move group
        move_group_interface_larm_.setPoseTarget(req.arm_goal_pose);

        // plan using left move group
        plan_success = (move_group_interface_larm_.plan(moveit_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    else { // !use_left_move_group, so use right move group
        // set pose target for right move group
        move_group_interface_rarm_.setPoseTarget(req.arm_goal_pose);

        // plan using right move group
        plan_success = (move_group_interface_rarm_.plan(moveit_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // check success
    if( plan_success ) {
        ROS_INFO("[MoveIt Planner Executor Server Node] Found plan in %f seconds", moveit_plan.planning_time_);
    }
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Could not find plan");
        // could not find plan, set response success to false
        res.success = plan_success;
        return res.success;
    }

    // ***** SET RESPONSE *****

    // successful plan, set response success and planned trajectory
    res.success = plan_success;
    res.planned_trajectory = moveit_plan.trajectory_;

    // ***** PROCESS INTERNAL STORAGE *****

    // store plan internally
    plan_exists_ = true;
    planned_trajectory_ = moveit_plan.trajectory_;

    return res.success;
}

bool MoveItPlannerExecutorServerNode::executeToArmGoalCallback(val_moveit_planner_executor::ExecuteToArmGoal::Request&  req,
                                                               val_moveit_planner_executor::ExecuteToArmGoal::Response& res) {
    // ***** PROCESS REQUESTED TRAJECTORY *****

    // initialize robot trajectory
    moveit_msgs::RobotTrajectory moveit_robot_traj_msg;

    // check if stored robot trajectory requested
    if( req.use_stored_robot_trajectory ) {
        // check that plan exists
        if( plan_exists_ ) {
            // use internally stored planned trajectory
            moveit_robot_traj_msg = planned_trajectory_;
        }
        else { // !plan_exists_
            ROS_WARN("[MoveIt Planner Executor Server Node] Requested to execute stored robot trajectory, but no planned trajectory exists; ignoring request");
            // no plan exists for execution, set response success to false
            res.success = false;
            return res.success;
        }
    }
    else {
        // use planned trajectory from request
        moveit_robot_traj_msg = req.planned_trajectory;
    }

    // ***** SEND ROBOT TRAJECTORY TO IHMCMsgInterfaceNode *****

    // publish robot trajectory for IHMCMsgInterfaceNode
    ihmc_msg_interface_pub_.publish(moveit_robot_traj_msg);

    // ***** SET RESPONSE *****

    // successfully sent robot trajectory, set response success to true
    res.success = true;

    // ***** PROCESS INTERNAL STORAGE *****

    // interanlly stored plan executed; reset internally stored plan
    planned_trajectory_ = moveit_msgs::RobotTrajectory();
    plan_exists_ = false;

    return res.success;
}

// GETTERS/SETTERS
double MoveItPlannerExecutorServerNode::getLoopRate() {
    return loop_rate_;
}

std::string MoveItPlannerExecutorServerNode::getTabString() {
    return std::string("\t\t\t\t\t\t\t\t\t\t");
}

// OCTOMAP HELPERS
bool MoveItPlannerExecutorServerNode::initializeOctomapVariables() {
    // initialize helpers for possible octomap topics
    std::string points_topic("octomap_point_cloud_centers");
    std::string occupancy_topic("occupied_cells_vis_array");

    // check topic
    if( octomap_topic_ == points_topic ) {
        volumetric_points_ = false;
    }
    else {
        volumetric_points_ = true;
    }

    // set octomap flag
    received_octomap_data_ = false;

    // set timeout
    octomap_timeout_ = 3.0; // seconds // TODO timeout time?

    return true;
}

void MoveItPlannerExecutorServerNode::markOctomapDataRecieved() {
    // update flag
    received_octomap_data_ = true;

    // store message time
    last_octomap_data_received_ = std::chrono::system_clock::now();

    return;
}

bool MoveItPlannerExecutorServerNode::checkOctomapTimeout() {
    if( !received_octomap_data_ ) {
        // data not received
        return true;
    }

    // get current time
    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();

    // compute duration since last octomap
    double time_since_octomap_data = std::chrono::duration_cast<std::chrono::seconds>(t - last_octomap_data_received_).count();

    // check for timeout
    bool timed_out = time_since_octomap_data > octomap_timeout_;

    // if timed out, update received octomap data flag
    if( timed_out ) {
        received_octomap_data_ = false;
    }

    return timed_out;
}

// MOVEIT HELPERS
void MoveItPlannerExecutorServerNode::initializeMoveItVariables() {
    // move group interfaces for left and right arms
    move_group_interface_larm_ = moveit::planning_interface::MoveGroupInterface(left_arm_planning_group);
    move_group_interface_rarm_ = moveit::planning_interface::MoveGroupInterface(right_arm_planning_group);

    // joint model group pointers for left and right arms
    joint_model_group_larm_ = move_group_interface_larm_.getCurrentState()->getJointModelGroup(left_arm_planning_group);
    joint_model_group_rarm_ = move_group_interface_rarm_.getCurrentState()->getJointModelGroup(right_arm_planning_group);

    // planning scene interface
    planning_scene_interface_ = moveit::planning_interface::PlanningSceneInterface();

    // set flag for internally stored plan
    plan_exists_ = false;

    // print information
    ROS_INFO("[MoveIt Planner Executor Server Node] LEFT ARM MOVEIT INFO:");
    ROS_INFO("%sValkyrie left arm planning frame: %s", getTabString().c_str(), move_group_interface_larm_.getPlanningFrame().c_str());
    ROS_INFO("%sLeft arm end-effector link: %s", getTabString().c_str(), move_group_interface_larm_.getEndEffectorLink().c_str());
    ROS_INFO("[MoveIt Planner Executor Server Node] RIGHT ARM MOVEIT INFO:");
    ROS_INFO("%sValkyrie right arm planning frame: %s", getTabString().c_str(), move_group_interface_rarm_.getPlanningFrame().c_str());
    ROS_INFO("%sRight arm end-effector link: %s", getTabString().c_str(), move_group_interface_rarm_.getEndEffectorLink().c_str());

    return;
}

bool MoveItPlannerExecutorServerNode::addPlanningSceneObstacles() {
    // check if octomap data is recent (function call implicitly checks if data is received)
    if( checkOctomapTimeout() ) {
        // old octomap data
        ROS_WARN("[MoveIt Planner Executor Server Node] Octomap data more than %f seconds old; cannot update planning scene", octomap_timeout_);
        return false;
    }

    // current octomap data, can proceed with updating planning scene
    if( volumetric_points_ ) {
        // use stored visualizatoin_msgs::MarkerArray
        return addPlanningSceneObstacles(obstacles_);
    }
    else { // !volumetric_points_
        return addPlanningSceneObstacles(point_cloud_);
    }
}

bool MoveItPlannerExecutorServerNode::addPlanningSceneObstacles(sensor_msgs::PointCloud2& point_cloud) {
    ROS_WARN("[MoveIt Planner Executor Server Node] Node requires octomap as visualization_msgs::MarkerArray, not sensor_msgs::PointCloud2 since volumetric points are required for collision-aware motion planning; please restart node with volumetric_points:=true");
    return false;

    /*
    // initialize vector of collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // CREATE OBJECTS FROM POINT CLOUD
    // working with PointCloud2 is complicated since data cannot easily be used directly
    // would need to convert from sensor_msgs::PointCloud2 object to pcl point cloud object
    // skipping this for now; assume using volumetric points

    // add collision objects to world
    planning_scene_interface_.addCollisionObjects(collision_objects);

    return true;
    */
}

bool MoveItPlannerExecutorServerNode::addPlanningSceneObstacles(visualization_msgs::MarkerArray& obstacles) {
    // initialize vector of collision objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // loop through octomap obstacles
    for( int i = 0 ; i < obstacles.markers.size() ; i++ ) {
        // get marker
        visualization_msgs::Marker marker = obstacles.markers[i];

        // initialize corresponding collision object
        moveit_msgs::CollisionObject collision_obj;

        // set frame id
        collision_obj.header.frame_id = marker.header.frame_id;

        // set id to identify object
        collision_obj.id = marker.ns + std::string("/") + std::to_string(marker.id);

        // create primitive shape for collision object
        shape_msgs::SolidPrimitive primitive;
        // set primitive type; make sure marker type and primitive type match
        if( marker.type == marker.CUBE ) {
            primitive.type = primitive.BOX;
            // set dimensions based on marker scale
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = marker.scale.x;
            primitive.dimensions[primitive.BOX_Y] = marker.scale.y;
            primitive.dimensions[primitive.BOX_Z] = marker.scale.z;
        }
        else {
            // marker and primitive types also have SPHERE and CYLINDER in common, but setting dimensions is more complicated
            // assume type must be CUBE/BOX
            ROS_WARN("[MoveIt Planner Executor Server Node] Expected Octomap markers to be of type %d, but got %d instead", marker.CUBE, marker.type);
            return false;
        }

        // create primitive pose for collision object
        geometry_msgs::Pose primitive_pose;
        // set primitive pose based on marker pose
        primitive_pose = marker.pose;

        // add primitive shape and primitive pose to collision object
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(primitive_pose);

        // set operation; make sure marker operation and primitive operation match
        if( marker.action == marker.ADD ) {
            collision_obj.operation = collision_obj.ADD;
        }
        else {
            // marker and primitive have other actions/operations available
            // assume action/operation must be ADD
            ROS_WARN("[MoveIt Planner Executor Server Node] Expected Octomap markers to have action %d, but got %d instead", marker.ADD, marker.action);
            return false;
        }

        // add collision object to vector
        collision_objects.push_back(collision_obj);
    }

    // add collision objects to world
    planning_scene_interface_.addCollisionObjects(collision_objects);

    return true;
}

void MoveItPlannerExecutorServerNode::clearPlanningSceneObstacles() {
    // get all objects from planning scene
    std::map<std::string, moveit_msgs::CollisionObject> collision_obj_map;
    collision_obj_map = planning_scene_interface_.getObjects();

    // initialize vector of object ids
    std::vector<std::string> object_ids;

    // get all object ids from map
    for( std::pair<std::string, moveit_msgs::CollisionObject> collision_obj_item : collision_obj_map ) {
        object_ids.push_back(collision_obj_item.first);
    }

    // remove collision objects from planning scene
    planning_scene_interface_.removeCollisionObjects(object_ids);

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "MoveItPlannerExecutorServerNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    MoveItPlannerExecutorServerNode moveit_server_node(nh);
    ROS_INFO("[MoveIt Planner Executor Server Node] Node started!");

    // advertise services
    moveit_server_node.advertiseServices();

    // run node, wait for requests
    ros::Rate rate(moveit_server_node.getLoopRate());
    while( ros::ok() ) {
        // spin
        ros::spin();
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Node stopped, all done!");

    return 0;
}
