/**
 * MoveIt Filter Joint States Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _MOVEIT_FILTER_JOINT_STATES_NODE_
#define _MOVEIT_FILTER_JOINT_STATES_NODE_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class MoveItFilterJointStatesNode {
public:
    // CONSTRUCTORS/DESTRUCTORS
    MoveItFilterJointStatesNode(const ros::NodeHandle& nh);
    ~MoveItFilterJointStatesNode();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACKS
    void jointStateCallback(const sensor_msgs::JointState& msg);

    // HELPERS
    void printFilteredJoints();

private:
    ros::NodeHandle nh_; // node handler
    std::string filter_js_topic_; // topic for filtered joint states
    ros::Publisher filter_js_pub_; // publisher for filtered joint states
    std::string js_topic_; // topic for unfiltered joint states
    ros::Subscriber js_sub_; // subscriber for unfiltered joint states

    std::vector<std::string> filter_joints_; // vector of joints to filter out of joint state message

}; // end class MoveItFilterJointStatesNode

#endif
