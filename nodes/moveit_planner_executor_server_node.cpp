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

    ROS_INFO("[MoveIt Planner Executor Server Node] Constructed");

    nh_ = nh;

    loop_rate_ = 10.0; // Hz

    ihmc_msg_interface_recv_pub_counter_ = 5;

    // set up parameters
    nh_.param("ihmc_msg_interface_moveit_topic", ihmc_msg_interface_moveit_topic_, std::string("/IHMCInterfaceNode/moveit_planned_robot_trajectory"));
    nh_.param("ihmc_msg_interface_receive_moveit_traj_topic", ihmc_msg_interface_recv_moveit_traj_topic_, std::string("/IHMCInterfaceNode/receive_moveit_trajectories"));

    // initialize variables, connections, service clients
    initializeMoveItVariables();
    initializeConnections();
    initializeServiceClients();
}

MoveItPlannerExecutorServerNode::~MoveItPlannerExecutorServerNode() {
    ROS_INFO("[MoveIt Planner Executor Server Node] Destroyed");
}

// CONNECTIONS
bool MoveItPlannerExecutorServerNode::initializeConnections() {
    ihmc_msg_interface_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>(ihmc_msg_interface_moveit_topic_, 1);
    ihmc_msg_interface_recv_moveit_traj_pub_ = nh_.advertise<std_msgs::Bool>(ihmc_msg_interface_recv_moveit_traj_topic_, 1);

    safety_reporter_plan_pub_ = nh_.advertise<val_safety_exception_reporter::CannotGetMotionPlan>("/valkyrie_safety_reporter/cannot_get_motion_plan", 10);
    safety_reporter_execute_pub_ = nh_.advertise<val_safety_exception_reporter::CannotExecuteMotion>("/valkyrie_safety_reporter/cannot_execute_motion", 10);

    return true;
}

// SERVICE CLIENTS
void MoveItPlannerExecutorServerNode::initializeServiceClients() {
    // create motion planning client
    plan_kinematic_path_client_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
    // block until service is advertised and available
    plan_kinematic_path_client_.waitForExistence();
    ROS_INFO("[MoveIt Planner Executor Server Node] Service /plan_kinematic_path exists!");

    // create Cartesian path client
    plan_cartesian_path_client_ = nh_.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");
    // block until service is advertised and available
    plan_cartesian_path_client_.waitForExistence();
    ROS_INFO("[MoveIt Planner Executor Server Node] Service /compute_cartesian_path exists!");

    ROS_INFO("[MoveIt Planner Executor Server Node] Created motion planning clients!");

    return;
}

// ADVERTISE SERVICES
void MoveItPlannerExecutorServerNode::advertiseServices() {
    plan_to_arm_goal_service_ = nh_.advertiseService("plan_to_arm_goal", &MoveItPlannerExecutorServerNode::planToArmGoalCallback, this);
    plan_to_arm_waypoints_service_ = nh_.advertiseService("plan_to_arm_waypoints", &MoveItPlannerExecutorServerNode::planToArmWaypointsCallback, this);
    execute_trajectory_service_ = nh_.advertiseService("execute_trajectory", &MoveItPlannerExecutorServerNode::executeTrajectoryCallback, this);

    ROS_INFO("[MoveIt Planner Executor Server Node] Providing services for planning and executing MoveIt trajectories!");
    
    return;
}

// SERVICE CALLBACKS
bool MoveItPlannerExecutorServerNode::planToArmGoalCallback(val_moveit_planner_executor::PlanToArmGoal::Request&  req,
                                                            val_moveit_planner_executor::PlanToArmGoal::Response& res) {
    ROS_INFO("[MoveIt Planner Executor Server Node] Received new arm goal motion planning request");

    // ***** INITIALIZE VARIABLES *****

    // new planning request received; reset internally stored plan
    planned_trajectory_ = moveit_msgs::RobotTrajectory();
    plan_exists_ = false;

    // create service call message, request, and response
    moveit_msgs::GetMotionPlan motion_plan_srv;
    moveit_msgs::MotionPlanRequest motion_plan_req;
    moveit_msgs::MotionPlanResponse motion_plan_res;

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
        // initialized transformed pelvis pose
        geometry_msgs::PoseStamped arm_goal_pelvis;

        // get pose in pelvis frame
        bool tf_succ = transformPoseToTargetFrame(std::string("pelvis"), req.arm_goal_pose, arm_goal_pelvis);

        if( !tf_succ ) {
            ROS_ERROR("[MoveIt Planner Executor Server Node] Cannot determine closest arm to handle motion planning request; ignoring request");
            ROS_ERROR("[MoveIt Planner Executor Server Node] Please consider explicitly setting the requested planning arm to either left or right.");
            // publish message for safety reporter
            publishSafetyReportClosestArm(req.arm_goal_pose);
            // ignoring request, set response success to false
            res.success = false;
            return true; // service communication succeeded
        }

        // check which arm is closest based on y-coordinate of pose request in pelvis frame
        if( arm_goal_pelvis.pose.position.y > 0 ) {
            // set flag to use left move group
            use_left_move_group = true;
        }
        else { // arm_goal_pelvis.pose.position.y <= 0
            // set flag to use right move group (not the left)
            use_left_move_group = false;
        }
    }
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Invalid planning group arm %d requested (expect 0,1,2); ignoring request", req.planning_group_arm);
        // publish message for safety reporter
        publishSafetyReportInvalidGroup(req.arm_goal_pose);
        // invalid request, set response success to false
        res.success = false;
        return true; // service communication succeeded
    }

    // initialize end-effector link and planning frame
    std::string ee_name;
    std::string planning_frame;

    // set group name, end-effector link, and planning frame based on left/right arm
    if( use_left_move_group ) {
        // set left arm joint group, left end-effector, and planning frame
        motion_plan_req.group_name = left_arm_planning_group;
        ee_name = move_group_interface_larm_.getEndEffectorLink();
        planning_frame = move_group_interface_larm_.getPlanningFrame();
    }
    else { // !use_left_move_group
        // set right arm joint group, right end-effector, and planning frame
        motion_plan_req.group_name = right_arm_planning_group;
        ee_name = move_group_interface_rarm_.getEndEffectorLink();
        planning_frame = move_group_interface_rarm_.getPlanningFrame();
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Requesting plan for %s joing group with end-effector %s in %s frame", motion_plan_req.group_name.c_str(), ee_name.c_str(), planning_frame.c_str());

    // ***** PROCESS REQUESTED COLLISION AWARENESS *****

    if( !req.collision_aware_planning ) {
        // collision-aware planning not requested, but node defaults to collision-aware planning
        ROS_WARN("[MoveIt Planner Executor Server Node] Requested plan without collision awareness, but collision-aware planning performed by default; ignoring request");
        // publish message for safety reporter
        publishSafetyReportCollisionAwarePlanning(motion_plan_req.group_name, req.arm_goal_pose);
        // invalid request, set response success to false
        res.success = false;
        return true; // service communication succeeded
    }

    // ***** PROCESS REQUESTED PLANNING TIME *****

    if( req.allowed_planning_time != 0.0 ) {
        // non-default planning time requested
        motion_plan_req.allowed_planning_time = req.allowed_planning_time;
    }
    else { // req.allowed_planning_time == 0.0
        // set some default planning time; this node will use 2.0 seconds
        motion_plan_req.allowed_planning_time = 2.0;
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Requesting plan in %f seconds or less", motion_plan_req.allowed_planning_time);

    // ***** PROCESS REQUESTED ARM GOAL POSE *****

    // initialized transformed world pose
    geometry_msgs::PoseStamped arm_goal_world;

    // get pose in pelvis frame
    bool tf_succ = transformPoseToTargetFrame(std::string("world"), req.arm_goal_pose, arm_goal_world);

    if( !tf_succ ) {
        ROS_ERROR("[MoveIt Planner Executor Server Node] Cannot transform motion planning request into world frame; ignoring request");
        // publish message for safety reporter
        publishSafetyReportWorldFrameTransform(motion_plan_req.group_name, req.arm_goal_pose);
        // ignoring request, set response success to false
        res.success = false;
        return true; // service communication succeeded
    }

    // set position and angular tolerances
    std::vector<double> tolerance_pos(3, 0.01);
    std::vector<double> tolerance_ang(3, 0.01);
    
    // set target pose as a goal constraint
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(ee_name, arm_goal_world, tolerance_pos, tolerance_ang);
    
    // add requested pose as goal constraint
    motion_plan_req.goal_constraints.push_back(pose_goal);

    // set motion plan request in service call message
    motion_plan_srv.request.motion_plan_request = motion_plan_req;

    // ***** CALL MOTION PLANNING SERVICE *****

    ROS_INFO("[MoveIt Planner Executor Server Node] Calling motion planning client...");

    // call service
    plan_kinematic_path_client_.call(motion_plan_srv);

    // get response in service call message
    motion_plan_res = motion_plan_srv.response.motion_plan_response;

    // check success
    bool plan_success = (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS);

    if( plan_success ) {
        ROS_INFO("[MoveIt Planner Executor Server Node] Found plan with %ld trajectory points in %f seconds",
                 motion_plan_res.trajectory.joint_trajectory.points.size(),
                 motion_plan_res.planning_time);
    }
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Could not find plan");
        // publish message for safety reporter
        publishSafetyReportNoPlanFound(motion_plan_req.group_name, req.arm_goal_pose);
        // could not find plan, set response success to false
        res.success = plan_success;
        return true; // service communication succeeded
    }

    // ***** SET RESPONSE *****

    // successful plan, set response success and planned trajectory
    res.success = plan_success;
    res.planned_trajectory = motion_plan_res.trajectory;

    // check length of trajectory
    if( res.planned_trajectory.joint_trajectory.points.size() > 50 ) {
        ROS_WARN("[MoveIt Planner Executor Server Node] Planned joint trajectory has %ld points and trajectories with more than 50 points may not be ignored by IHMC controllers",
                 res.planned_trajectory.joint_trajectory.points.size());
        ROS_WARN("[MoveIt Planner Executor Server Node] Please consider replanning or using a different target to get a shorter joint trajectory");
    }

    // ***** PROCESS INTERNAL STORAGE *****

    // store plan internally
    plan_exists_ = true;
    planned_trajectory_ = res.planned_trajectory;

    return true; // service communication succeeded
}

bool MoveItPlannerExecutorServerNode::planToArmWaypointsCallback(val_moveit_planner_executor::PlanToArmWaypoints::Request&  req,
                                                                 val_moveit_planner_executor::PlanToArmWaypoints::Response& res) {
    // get number of waypoints in request
    int num_waypoints = req.arm_waypoints.size();
    ROS_INFO("[MoveIt Planner Executor Server Node] Received new arm waypoints motion planning request with %d waypoints", num_waypoints);

    // ***** INITIALIZE VARIABLES *****

    // new planning request received; reset internally stored plan
    planned_trajectory_ = moveit_msgs::RobotTrajectory();
    plan_exists_ = false;

    // create service call message
    moveit_msgs::GetCartesianPath cart_plan_srv;

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
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Invalid planning group arm %d requested (expect 0,1); ignoring request", req.planning_group_arm);
        // publish message for safety reporter
        publishSafetyReportInvalidGroup(req.arm_waypoints[num_waypoints-1]);
        // invalid request, set response success to false
        res.success = false;
        return true; // service communication succeeded
    }

    // initialize end-effector link and planning frame
    std::string ee_name;
    std::string planning_frame;

    // set group name, end-effector link, and planning frame based on left/right arm
    if( use_left_move_group ) {
        // set left arm joint group, left end-effector, and planning frame
        cart_plan_srv.request.group_name = left_arm_planning_group;
        ee_name = move_group_interface_larm_.getEndEffectorLink();
        planning_frame = move_group_interface_larm_.getPlanningFrame();
    }
    else { // !use_left_move_group
        // set right arm joint group, right end-effector, and planning frame
        cart_plan_srv.request.group_name = right_arm_planning_group;
        ee_name = move_group_interface_rarm_.getEndEffectorLink();
        planning_frame = move_group_interface_rarm_.getPlanningFrame();
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Requesting plan for %s joing group with end-effector %s in %s frame", cart_plan_srv.request.group_name.c_str(), ee_name.c_str(), planning_frame.c_str());

    // ***** PROCESS REQUESTED WAYPOINTS *****

    // clear waypoints
    cart_plan_srv.request.waypoints.clear();

    // verify non-empty list of waypoints
    if( num_waypoints == 0 ) {
        ROS_ERROR("[MoveIt Planner Executor Server Node] Cannot plan with empty list of waypoints; ignoring request");
        // ignoring request, set response success to false
        res.success = false;
        return true; // service communication succeeded
    }

    // loop through requested waypoints and add them to service request
    for( int i = 0 ; i < req.arm_waypoints.size() ; i++ ) {
        // initialized transformed world pose
        geometry_msgs::PoseStamped arm_waypoint_world;

        // get pose in pelvis frame
        bool tf_succ = transformPoseToTargetFrame(std::string("world"), req.arm_waypoints[i], arm_waypoint_world);

        if( !tf_succ ) {
            ROS_ERROR("[MoveIt Planner Executor Server Node] Cannot transform motion planning waypoint into world frame; ignoring request");
            // publish message for safety reporter
            publishSafetyReportWorldFrameTransform(cart_plan_srv.request.group_name, req.arm_waypoints[i]);
            // ignoring request, set response success to false
            res.success = false;
            return true; // service communication succeeded
        }

        // add transformed waypoint pose to service request
        cart_plan_srv.request.waypoints.push_back(arm_waypoint_world.pose);
    }

    // ***** PROCESS REQUESTED PLANNING PARAMETERS *****

    // set max step between consecutive points (in Cartesian space) in the returned path
    if( req.max_cart_step != 0.0 ) {
        // non-default max Cartesian step requested
        cart_plan_srv.request.max_step = req.max_cart_step;
    }
    else { // req.max_cart_step == 0.0
        // set default max Cartesian step
        cart_plan_srv.request.max_step = req.DEFAULT_MAX_CART_STEP;
    }

    // set max allowed distance between consecutive points (in configuration space) in the returned path
    if( req.max_config_step != 0.0 ) {
        // non-default max configuration step requested
        cart_plan_srv.request.jump_threshold = req.max_config_step;
    }
    else { // req.max_config_step == 0.0
        // set default max configuration step
        cart_plan_srv.request.jump_threshold = req.DEFAULT_MAX_CONFIG_STEP;
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Requesting plan with max distance of %f (m) between waypoints in Cartesian space",
             cart_plan_srv.request.max_step);
    ROS_INFO("[MoveIt Planner Executor Server Node] Requesting plan with max distance of %f (radians) between waypoints in configuration space",
             cart_plan_srv.request.jump_threshold);

    // ***** CALL MOTION PLANNING SERVICE *****

    ROS_INFO("[MoveIt Planner Executor Server Node] Calling motion planning client...");

    // call service
    plan_cartesian_path_client_.call(cart_plan_srv);

    // check success
    bool plan_success = (cart_plan_srv.response.error_code.val == cart_plan_srv.response.error_code.SUCCESS);

    if( plan_success ) {
        ROS_INFO("[MoveIt Planner Executor Server Node] Found plan with %ld trajectory points",
                 cart_plan_srv.response.solution.joint_trajectory.points.size());
        if( cart_plan_srv.response.fraction != 1.0 ) {
            ROS_WARN("[MoveIt Planner Executor Server Node] Planned trajectory completes %f%% of requested trajectory",
                     cart_plan_srv.response.fraction * 100.0);
        }
        else { // cart_plan_srv.response.fraction == 1.0
            ROS_INFO("[MoveIt Planner Executor Server Node] Planned trajectory completes %f%% of requested trajectory!",
                     cart_plan_srv.response.fraction * 100.0);
        }
    }
    else {
        ROS_WARN("[MoveIt Planner Executor Server Node] Could not find plan");
        // publish message for safety reporter
        publishSafetyReportNoPlanFound(cart_plan_srv.request.group_name, req.arm_waypoints[num_waypoints-1]);
        // could not find plan, set response success to false
        res.success = plan_success;
        return true; // service communication succeeded
    }

    // ***** SET RESPONSE *****

    // successful plan, set response success and planned trajectory
    res.success = plan_success;
    res.fraction = cart_plan_srv.response.fraction;
    res.planned_trajectory = cart_plan_srv.response.solution;

    // check length of trajectory
    if( res.planned_trajectory.joint_trajectory.points.size() > 50 ) {
        ROS_WARN("[MoveIt Planner Executor Server Node] Planned joint trajectory has %ld points and trajectories with more than 50 points may not be ignored by IHMC controllers",
                 res.planned_trajectory.joint_trajectory.points.size());
        ROS_WARN("[MoveIt Planner Executor Server Node] Please consider replanning or using a different target to get a shorter joint trajectory");
    }

    // ***** PROCESS INTERNAL STORAGE *****

    // store plan internally
    plan_exists_ = true;
    planned_trajectory_ = res.planned_trajectory;

    return true; // service communication succeeded
}

bool MoveItPlannerExecutorServerNode::executeTrajectoryCallback(val_moveit_planner_executor::ExecuteTrajectory::Request&  req,
                                                                val_moveit_planner_executor::ExecuteTrajectory::Response& res) {
    ROS_INFO("[MoveIt Planner Executor Server Node] Received new motion trajectory execution request");

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
            // publish message for safety reporter
            publishSafetyReportNoStoredTrajectory();
            // no plan exists for execution, set response success to false
            res.success = false;
            return true; // service communication succeeded
        }
    }
    else {
        // use planned trajectory from request
        moveit_robot_traj_msg = req.planned_trajectory;
    }

    // ***** SEND ROBOT TRAJECTORY TO IHMCMsgInterfaceNode *****

    ROS_INFO("[MoveIt Planner Executor Server Node] Sending robot trajectory for execution...");

    // publish robot trajectory for IHMCMsgInterfaceNode
    ihmc_msg_interface_pub_.publish(moveit_robot_traj_msg);

    // ***** SET RESPONSE *****

    // successfully sent robot trajectory, set response success to true
    res.success = true;

    ROS_INFO("[MoveIt Planner Executor Server Node] Sent planned robot trajectory to IHMCMsgInterfaceNode for execution");

    // ***** PROCESS INTERNAL STORAGE *****

    // interanlly stored plan executed; reset internally stored plan
    planned_trajectory_ = moveit_msgs::RobotTrajectory();
    plan_exists_ = false;

    return true; // service communication succeeded
}

// GETTERS/SETTERS
double MoveItPlannerExecutorServerNode::getLoopRate() {
    return loop_rate_;
}

std::string MoveItPlannerExecutorServerNode::getTabString() {
    return std::string("\t");
}

// TRANSFORM HELPERS
bool MoveItPlannerExecutorServerNode::transformPoseToTargetFrame(std::string target_frame,
                                                                 geometry_msgs::PoseStamped source_pose,
                                                                 geometry_msgs::PoseStamped& target_pose) {
    // get frame name
    std::string msg_frame_name = source_pose.header.frame_id;

    // check if arm goal is given in pelvis frame
    if( msg_frame_name != target_frame ) {
        // not in pelvis frame, transformation needed to determine closest arm
        std::string err_msg;
        geometry_msgs::PoseStamped transformed_goal_pose;
        try {
            // check if transform exists
            if( !tf_.waitForTransform(target_frame, msg_frame_name, ros::Time(0), ros::Duration(1.0), // wait for transform from target frame to source frame
                                      ros::Duration(0.01), &err_msg) ) { // default polling sleep duration
                ROS_ERROR("[MoveIt Planner Executor Server Node] No transform from %s to %s",
                          target_frame.c_str(), msg_frame_name.c_str());
                ROS_ERROR("[MoveIt Planner Executor Server Node] Transform error: %s", err_msg.c_str());
                return false;
            }
            else {
                // transform planning goal into pelvis frame
                tf_.transformPose(target_frame, source_pose, transformed_goal_pose);
                // update target pose
                target_pose = transformed_goal_pose;
            }
        }
        catch( tf::TransformException ex ) {
            ROS_ERROR("[MoveIt Planner Executor Server Node] Trouble getting transform from %s to %s",
                      target_frame.c_str(), msg_frame_name.c_str());
            ROS_ERROR("[MoveIt Planner Executor Server Node] Transform exception: %s", ex.what());
            return false;
        }
    }
    else {
        // no transform needed, given pose in target frame; update target pose
        target_pose = source_pose;
    }

    return true;
}

// MOVEIT HELPERS
void MoveItPlannerExecutorServerNode::initializeMoveItVariables() {
    // move group interfaces for left and right arms
    move_group_interface_larm_ = moveit::planning_interface::MoveGroupInterface(left_arm_planning_group);
    move_group_interface_rarm_ = moveit::planning_interface::MoveGroupInterface(right_arm_planning_group);

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

// IHMCMsgInterface HELPERS
void MoveItPlannerExecutorServerNode::tellIHMCMsgInterfaceReceiveMoveItTrajectories() {
    std_msgs::Bool bool_msg;
    bool_msg.data = true;

    if( ihmc_msg_interface_recv_pub_counter_ > 0 ) {
        ROS_INFO("[MoveIt Planner Executor Server Node] Telling IHMCMsgInterfaceNode to accept MoveIt trajectories");
        ihmc_msg_interface_recv_moveit_traj_pub_.publish(bool_msg);
        ihmc_msg_interface_recv_pub_counter_--;
    }

    return;
}

// SAFETY REPORTER HELPERS
void MoveItPlannerExecutorServerNode::publishSafetyReportClosestArm(geometry_msgs::PoseStamped requested_target) {
    // set up exception for safety reporter
    std::string error_message = std::string("Cannot determine closest arm to handle motion planning request; may need to explicitly request planning arm to be either left or right");

    // publish message for safety reporter
    publishCannotGetMotionPlanMessage(error_message, std::string("closest arm"), requested_target);

    return;
}

void MoveItPlannerExecutorServerNode::publishSafetyReportInvalidGroup(geometry_msgs::PoseStamped requested_target) {
    // set up exception for safety reporter
    std::string error_message = std::string("Invalid planning group requested");

    // publish message for safety reporter
    publishCannotGetMotionPlanMessage(error_message, std::string("unknown group"), requested_target);

    return;
}

void MoveItPlannerExecutorServerNode::publishSafetyReportCollisionAwarePlanning(std::string planning_group, geometry_msgs::PoseStamped requested_target) {
    // set up exception for safety reporter
    std::string error_message = std::string("Cannot ignore collisions because planning scene initialized; try restarting launch file with `allow_sensors:=false` to ignore sensor data");

    // publish message for safety reporter
    publishCannotGetMotionPlanMessage(error_message, planning_group, requested_target);

    return;
}

void MoveItPlannerExecutorServerNode::publishSafetyReportWorldFrameTransform(std::string planning_group, geometry_msgs::PoseStamped requested_target) {
    // set up exception for safety reporter
    std::string error_message = std::string("Cannot transform motion planning request into world frame");

    // publish message for safety reporter
    publishCannotGetMotionPlanMessage(error_message, planning_group, requested_target);

    return;
}

void MoveItPlannerExecutorServerNode::publishSafetyReportNoPlanFound(std::string planning_group, geometry_msgs::PoseStamped requested_target) {
    // set up exception for safety reporter
    std::string error_message = std::string("Could not find motion plan");

    // publish message for safety reporter
    publishCannotGetMotionPlanMessage(error_message, planning_group, requested_target);

    return;
}

void MoveItPlannerExecutorServerNode::publishCannotGetMotionPlanMessage(std::string planning_exception,
                                                                        std::string planning_group,
                                                                        geometry_msgs::PoseStamped requested_target) {
    // create cannot get motion plan message
    val_safety_exception_reporter::CannotGetMotionPlan motion_plan_msg;

    // set message fields
    motion_plan_msg.planning_exception = planning_exception;
    motion_plan_msg.planning_group = planning_group;
    motion_plan_msg.requested_plan_target = requested_target;

    // publish message
    safety_reporter_plan_pub_.publish(motion_plan_msg);

    return;
}

void MoveItPlannerExecutorServerNode::publishSafetyReportNoStoredTrajectory() {
    // set up exception for safety reporter
    std::string error_message = std::string("Received request to execute stored robot trajectory, but no trajectory is stored; try replanning and executing");

    // publish message for safety reporter
    publishCannotExecuteMotionMessage(error_message);

    return;
}

void MoveItPlannerExecutorServerNode::publishCannotExecuteMotionMessage(std::string execution_exception) {
    // create cannot execute motion message
    val_safety_exception_reporter::CannotExecuteMotion execute_msg;

    // set message fields
    execute_msg.execution_exception = execution_exception;

    // publish message
    safety_reporter_execute_pub_.publish(execute_msg);

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieMoveItPlannerExecutorServerNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    MoveItPlannerExecutorServerNode moveit_server_node(nh);
    ROS_INFO("[MoveIt Planner Executor Server Node] Node started!");

    // advertise services
    moveit_server_node.advertiseServices();

    // get loop rate
    ros::Rate rate(moveit_server_node.getLoopRate());

    // run node, wait for requests
    while( ros::ok() ) {
        // make sure IHMCMsgInterface is listening for MoveIt trajectories
        moveit_server_node.tellIHMCMsgInterfaceReceiveMoveItTrajectories();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[MoveIt Planner Executor Server Node] Node stopped, all done!");

    return 0;
}
