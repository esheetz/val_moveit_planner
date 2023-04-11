/**
 * Filter Joint States Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _FILTER_JOINT_STATES_NODE_
#define _FILTER_JOINT_STATES_NODE_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class FilterJointStatesNode {
public:
    // CONSTRUCTORS/DESTRUCTORS
    FilterJointStatesNode(const ros::NodeHandle& nh);
    ~FilterJointStatesNode();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACKS
    void jointStateCallback(const sensor_msgs::JointState& msg);

private:
    ros::NodeHandle nh_;
    std::string filter_js_topic_;
    ros::Publisher filter_js_pub_;
    std::string js_topic_;
    ros::Subscriber js_sub_;

}; // end class FilterJointStatesNode

#endif
