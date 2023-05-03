/**
 * MoveIt Planning Scene Collision Detector Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

 #include <nodes/moveit_planning_scene_collision_detector.h>

// CONSTRUCTORS/DESTRUCTORS
MoveItPlanningSceneCollisionDetector::MoveItPlanningSceneCollisionDetector(const ros::NodeHandle& nh) {
    ROS_INFO("[MoveIt Planning Scene Collision Detector] Constructed");

    nh_ = nh;

    loop_rate_ = 10.0; // Hz

    // initialize connections and MoveIt
    initializeMoveItVariables();
    initializeConnections();
}

MoveItPlanningSceneCollisionDetector::~MoveItPlanningSceneCollisionDetector() {
    ROS_INFO("[MoveIt Planning Scene Collision Detector] Destroyed");
}

// CONNECTIONS
bool MoveItPlanningSceneCollisionDetector::initializeConnections() {
    safety_reporter_pub_ = nh_.advertise<val_safety_exception_reporter::Collisions>("/valkyrie_safety_reporter/collisions", 10);

    return true;
}

// ADVERTISE SERVICES
void MoveItPlanningSceneCollisionDetector::advertiseServices() {
    modify_allowed_collisions_service_ = nh_.advertiseService("modify_allowed_collisions", &MoveItPlanningSceneCollisionDetector::modifyAllowedCollisionsCallback, this);

    ROS_INFO("[MoveIt Planning Scene Collision Detector] Providing services for modifying planning scene collisions!");

    return;
}

// SERVICE CALLBACKS
bool MoveItPlanningSceneCollisionDetector::modifyAllowedCollisionsCallback(val_moveit_planner_executor::ModifyAllowedPlanningSceneCollisions::Request&  req,
                                                                           val_moveit_planner_executor::ModifyAllowedPlanningSceneCollisions::Response& res) {
    ROS_INFO("[MoveIt Planning Scene Collision Detector] Received request to modify allowed planning scene collisions");

    // update planning scene
    bool update_success = allowPlanningSceneCollisions(req.modify_collisions);

    // set response
    res.success = update_success;
    return true; // service communication succeeded
}

// GETTERS/SETTERS
double MoveItPlanningSceneCollisionDetector::getLoopRate() {
    return loop_rate_;
}

// MOVEIT HELPERS
void MoveItPlanningSceneCollisionDetector::initializeMoveItVariables() {
    // initialize service for getting planning scene
    planning_scene_service_ = std::string("/get_planning_scene");

    // get name of robot description
    std::string robot_desc_name;
    nh_.param("robot_desc_name", robot_desc_name, std::string("/moveit/robot_description"));

    // initialize shared planning scene monitor pointer; needs move_group node to already be running
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_desc_name);

    // create client for updating planning scene
    apply_planning_scene_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
    // block until service is advertised and available
    apply_planning_scene_client_.waitForExistence();

    ROS_INFO("[MoveIt Planning Scene Collision Detector] Service /apply_planning_scene exists!");
    ROS_INFO("[MoveIt Planning Scene Collision Detector] Created planning scene update client!");

    return;
}

planning_scene_monitor::LockedPlanningSceneRO MoveItPlanningSceneCollisionDetector::getCurrentPlanningSceneRO() {
    // update planning scene monitor pointer with current state
    planning_scene_monitor_->requestPlanningSceneState(planning_scene_service_);

    // create read-only planning scene
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);

    return ps;
}

planning_scene_monitor::LockedPlanningSceneRW MoveItPlanningSceneCollisionDetector::getCurrentPlanningSceneRW() {
    // update planning scene monitor pointer with current state
    planning_scene_monitor_->requestPlanningSceneState(planning_scene_service_);

    // create read-write planning scene
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update(); // updates all transforms for robot model // TODO is this necessary?

    return ps;
}

// SAFETY REPORTER HELPERS
std::string MoveItPlanningSceneCollisionDetector::getCollisionMessageBodyType(collision_detection::BodyType body_type) {
    // create empty collision message, which will only be used for names of body types
    val_safety_exception_reporter::Collision collision_msg;

    // initialize body type string
    std::string collision_msg_body_type("");

    // check contact body type
    switch( body_type ) {
    case collision_detection::BodyType::ROBOT_LINK:
        collision_msg_body_type = collision_msg.ROBOT_LINK;
        break;
    case collision_detection::BodyType::ROBOT_ATTACHED:
        collision_msg_body_type = collision_msg.ROBOT_ATTACHED;
        break;
    case collision_detection::BodyType::WORLD_OBJECT:
        collision_msg_body_type = collision_msg.WORLD_OBJECT;
        break;
    }

    return collision_msg_body_type;
}

geometry_msgs::Point MoveItPlanningSceneCollisionDetector::getCollisionMessagePoint(Eigen::Vector3d contact_pos) {
    // create point message
    geometry_msgs::Point collision_point;

    // set point message from contact position
    collision_point.x = contact_pos.x();
    collision_point.y = contact_pos.y();
    collision_point.z = contact_pos.z();

    return collision_point;
}

void MoveItPlanningSceneCollisionDetector::makeCollisionMessage(collision_detection::Contact contact, val_safety_exception_reporter::Collision& collision_msg) {
    // set body names
    collision_msg.body1_name = contact.body_name_1;
    collision_msg.body2_name = contact.body_name_2;

    // set body types
    collision_msg.body1_type = getCollisionMessageBodyType(contact.body_type_1);
    collision_msg.body2_type = getCollisionMessageBodyType(contact.body_type_2);

    // set point
    collision_msg.collision_point = getCollisionMessagePoint(contact.pos);

    return;
}

void MoveItPlanningSceneCollisionDetector::publishSafetyReportCollisionMessage(collision_detection::CollisionResult::ContactMap collision_contacts) {
    /*
     * See documentation on collision detection contacts:
     * http://docs.ros.org/en/indigo/api/moveit_core/html/structcollision__detection_1_1Contact.html
     */

    // create collisions message
    val_safety_exception_reporter::Collisions collisions_msg;
    collisions_msg.collisions.clear();

    // iterate through contact map
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for( it = collision_contacts.begin() ; it != collision_contacts.end() ; it++ ) {
        std::string body_1_name = it->first.first;
        std::string body_2_name = it->first.second;
        std::vector<collision_detection::Contact> contacts = it->second;
        // loop through contacts associated with the two bodies
        for( int i = 0 ; i < contacts.size() ; i++ ) {
            // get contact
            collision_detection::Contact c = contacts[i];
            // initialize collision message
            val_safety_exception_reporter::Collision c_msg;
            // create collision message from contact
            makeCollisionMessage(c, c_msg);
            // add to vector of collisions
            collisions_msg.collisions.push_back(c_msg);
        }
    }

    // publish message
    safety_reporter_pub_.publish(collisions_msg);

    return;
}

// COLLISION DETECTION
void MoveItPlanningSceneCollisionDetector::checkPlanningSceneCollisions() {
    /*
     * See source for example of getting current planning scene and checking collisions:
     * https://groups.google.com/g/moveit-users/c/a3dIWP7hdqo?pli=1
     *
     * See planning scene tutorial:
     * https://ros-planning.github.io/moveit_tutorials/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
     *
     * See planning scene monitor tutorial (and links to LockedPlanningSceneRW/LockedPlanningSceneRO classes):
     * https://ros-planning.github.io/moveit_tutorials/doc/planning_scene_monitor/planning_scene_monitor_tutorial.html
     */

    // get current planning scene
    planning_scene_monitor::LockedPlanningSceneRO ps = getCurrentPlanningSceneRO();
    // this is a normal planning scene, look at planning_scene::PlanningScene documentation for reference

    // initialize collision detection request and result
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    /*
     * See CollisionRequest documentation: http://docs.ros.org/en/indigo/api/moveit_core/html/structcollision__detection_1_1CollisionRequest.html
     * See CollisionResult documentation: http://docs.ros.org/en/indigo/api/moveit_core/html/structcollision__detection_1_1CollisionResult.html
     */

    // can check collisions for a specific group, by setting collision_request.group_name
    // but we want to check collisions with the full robot, so we will leave this unset

    // request contact information, set max number to a large number
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    // check collisions
    /*
     * NOTE: checks for self-collisions using unpadded robot
     *       and collisions with environment using padded robot
     */
    ps->checkCollision(collision_request, collision_result);

    // check collision result
    if( collision_result.collision ) {
        ROS_WARN("[MoveIt Planning Scene Collision Detector] Found %ld collision contact points", collision_result.contact_count);
        publishSafetyReportCollisionMessage(collision_result.contacts);
    }

    return;
}

bool MoveItPlanningSceneCollisionDetector::allowPlanningSceneCollisions(std::vector<val_moveit_planner_executor::ModifyAllowedCollision> modify_collisions) {
    /*
     * See source for example of changing ACM and updating planning scene monitor:
     * https://groups.google.com/g/moveit-users/c/3OXynhBJleA
     */

    // create service call message, request, and response to update planning scene
    moveit_msgs::ApplyPlanningScene ps_srv;
    moveit_msgs::PlanningScene ps_req;
    bool ps_res;

    // get current planning scene
    planning_scene_monitor::LockedPlanningSceneRW ps = getCurrentPlanningSceneRW();
    // this is a normal planning scene, look at planning_scene::PlanningScene documentation for reference

    // get allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrixNonConst();

    // loop through collisions to modify
    for( int i = 0 ; i < modify_collisions.size() ; i++ ) {
        // get modify collision message
        val_moveit_planner_executor::ModifyAllowedCollision c_msg = modify_collisions[i];
        
        // update entry in allowed collision matrix to allow collisions between given bodies
        acm.setEntry(c_msg.body1_name, c_msg.body2_name, c_msg.allow_collisions);
    }

    // update planning scene requested update with new allowed collision matrix
    acm.getMessage(ps_req.allowed_collision_matrix);
    ps_req.is_diff = true;

    // set planning scene request
    ps_srv.request.scene = ps_req;

    // call service
    apply_planning_scene_client_.call(ps_srv);

    // get response in service call message
    ps_res = ps_srv.response.success;

    if( ps_res ) {
        ROS_INFO("[MoveIt Planning Scene Collision Detector] Successfully updated collisions allowed by planning scene monitor!");
    }
    else {
        ROS_WARN("[MoveIt Planning Scene Collision Detector] Could not update collisions allowed by planning scene monitor");
        ROS_WARN("[MoveIt Planning Scene Collision Detector] Proceed with caution!");
    }

    return ps_res;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieMoveItPlanningSceneCollisionDetector");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    MoveItPlanningSceneCollisionDetector moveit_collision_detector(nh);
    ROS_INFO("[MoveIt Planning Scene Collision Detector] Node started!");

    // advertise services
    moveit_collision_detector.advertiseServices();

    // get loop rate
    ros::Rate rate(moveit_collision_detector.getLoopRate());

    // run node, check for collisions
    while( ros::ok() ) {
        // check for collisions
        moveit_collision_detector.checkPlanningSceneCollisions();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[MoveIt Planning Scene Collision Detector] Node stopped, all done!");

    return 0;
}