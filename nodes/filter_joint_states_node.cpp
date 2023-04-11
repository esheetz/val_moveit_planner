/**
 * Filter Joint States Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

 #include <nodes/filter_joint_states_node.h>

// CONSTRUCTORS/DESTRUCTORS
FilterJointStatesNode::FilterJointStatesNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("filter_js_topic", filter_js_topic_, std::string("/moveit_filtered_joint_states"));
    nh_.param("js_topic", js_topic_, std::string("/ihmc_ros/valkyrie/output/joint_states"));

    // initialize connections
    initializeConnections();
    
    ROS_INFO("[Filter Joint States Node] Constructed");
}

FilterJointStatesNode::~FilterJointStatesNode() {
    ROS_INFO("[Filter Joint States Node] Destroyed");
}

// CONNECTIONS
bool FilterJointStatesNode::initializeConnections() {
    filter_js_pub_ = nh_.advertise<sensor_msgs::JointState>(filter_js_topic_, 1);
    js_sub_ = nh_.subscribe(js_topic_, 1, &FilterJointStatesNode::jointStateCallback, this);

    return true;
}

// CALLBACKS
void FilterJointStatesNode::jointStateCallback(const sensor_msgs::JointState& msg) {
    // create filtered message
    sensor_msgs::JointState filtered_js_msg;

    // set header
    filtered_js_msg.header = msg.header;

    // look through received joint state message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // check if joint is hokuyo_joint
        if( msg.name[i] != std::string("hokuyo_joint") ) {
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

int main(int argc, char** argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieMoveItFilterJointStatesNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    FilterJointStatesNode fjsnode(nh);
    ROS_INFO("[Filter Joint States Node] Node started!");

    // spin, wait for joint state messages
    while( ros::ok() ) {
        ros::spin();
    }

    return 0;
}
