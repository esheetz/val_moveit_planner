/**
 * MoveIt Filter Joint States Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

 #include <nodes/moveit_filter_joint_states_node.h>

// CONSTRUCTORS/DESTRUCTORS
MoveItFilterJointStatesNode::MoveItFilterJointStatesNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("filter_js_topic", filter_js_topic_, std::string("/moveit_filtered_joint_states"));
    nh_.param("js_topic", js_topic_, std::string("/ihmc_ros/valkyrie/output/joint_states"));
    bool success = nh_.getParam("/filter_joints", filter_joints_);
    ROS_INFO("[MoveIt Filter Joint States Node] Got filtered joints param: %s", (success ? std::string("true").c_str() : std::string("FALSE").c_str()));

    // initialize connections
    initializeConnections();
    
    ROS_INFO("[MoveIt Filter Joint States Node] Constructed");
}

MoveItFilterJointStatesNode::~MoveItFilterJointStatesNode() {
    ROS_INFO("[MoveIt Filter Joint States Node] Destroyed");
}

// CONNECTIONS
bool MoveItFilterJointStatesNode::initializeConnections() {
    filter_js_pub_ = nh_.advertise<sensor_msgs::JointState>(filter_js_topic_, 1);
    js_sub_ = nh_.subscribe(js_topic_, 1, &MoveItFilterJointStatesNode::jointStateCallback, this);

    return true;
}

// CALLBACKS
void MoveItFilterJointStatesNode::jointStateCallback(const sensor_msgs::JointState& msg) {
    // create filtered message
    sensor_msgs::JointState filtered_js_msg;

    // set header
    filtered_js_msg.header = msg.header;

    // look through received joint state message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // try to find joint name in vector of joints to be filtered
        std::vector<std::string>::iterator it = std::find(filter_joints_.begin(), filter_joints_.end(), msg.name[i]);
        // check if joint should be filtered out (joint is not in vector)
        if( it == filter_joints_.end() ) {
            // add joint information to filtered message
            filtered_js_msg.name.push_back(msg.name[i]);
            filtered_js_msg.position.push_back(msg.position[i]);
            filtered_js_msg.velocity.push_back(msg.velocity[i]);
            filtered_js_msg.effort.push_back(msg.effort[i]);
        }
    }

    // publish filtered message
    filter_js_pub_.publish(filtered_js_msg);

    return;
}

// HELPERS
void MoveItFilterJointStatesNode::printFilteredJoints() {
    // print names of joints to be filtered
    ROS_INFO("[MoveIt Filter Joint States Node] Found %ld joints to filter. Filtering out the following joints:", filter_joints_.size());

    for( int i = 0 ; i < filter_joints_.size() ; i++ ) {
        ROS_INFO("\t%s", filter_joints_[i].c_str());
    }

    return;
}

int main(int argc, char** argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieMoveItFilterJointStatesNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    MoveItFilterJointStatesNode fjsnode(nh);
    ROS_INFO("[MoveIt Filter Joint States Node] Node started!");

    // print names of filtered joints
    fjsnode.printFilteredJoints();

    // spin, wait for joint state messages
    while( ros::ok() ) {
        ros::spin();
    }

    return 0;
}
