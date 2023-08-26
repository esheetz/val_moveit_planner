#!/usr/bin/env python3
"""
(Kind of Hacky) Arm Goal Interactive Marker Node
Emily Sheetz, Winter 2023
"""

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate, InteractiveMarker, InteractiveMarkerControl, Marker
from moveit_msgs.msg import RobotTrajectory

# services
from val_moveit_planner_executor.srv import PlanToArmGoal, PlanToArmGoalRequest, PlanToArmGoalResponse
from val_moveit_planner_executor.srv import PlanToArmWaypoints, PlanToArmWaypointsRequest, PlanToArmWaypointsResponse
from val_moveit_planner_executor.srv import ExecuteTrajectory, ExecuteTrajectoryRequest, ExecuteTrajectoryResponse

# dynamic reconfigure
from dynamic_reconfigure.server import Server
from val_moveit_planner_executor.cfg import IMPlanningRequestParamsConfig

# Interactive Marker imports
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import MenuHandler

class ArmGoalInteractiveMarkerNode:
    def __init__(self):
        # initialize names
        self.node_name = "Hacky MoveIt Arm Goal IM Node"
        self.im_server_name = "hacky_moveit_arm_goals"
        self.int_marker_name = "Hacky MoveIt Arm Goal Target"
        self.moveit_im_topic_name = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update"
        self.moveit_planner_executor_plan_service_name = "/ValkyrieMoveItPlannerExecutorServerNode/plan_to_arm_goal"
        self.moveit_planner_executor_plan_waypoints_service_name = "/ValkyrieMoveItPlannerExecutorServerNode/plan_to_arm_waypoints"
        self.moveit_planner_executor_execute_service_name = "/ValkyrieMoveItPlannerExecutorServerNode/execute_trajectory"
        self.left_arm_name = "EE:goal_leftPalm"
        self.right_arm_name = "EE:goal_rightPalm"
        self.arm_names = [self.left_arm_name, self.right_arm_name]

        # initialize interactive marker server
        self.im_server = InteractiveMarkerServer(self.im_server_name)

        # initialize MoveIt IM update subscriber
        self.moveit_im_sub = rospy.Subscriber(self.moveit_im_topic_name, InteractiveMarkerUpdate, self.moveit_im_feedback_callback)

        # initialize val_moveit_planner_executor services
        rospy.loginfo("[%s] Waiting for service plan_to_arm_goal..." % self.node_name)
        rospy.wait_for_service(self.moveit_planner_executor_plan_service_name) # blocks until service exists
        self.moveit_plan_client = rospy.ServiceProxy(self.moveit_planner_executor_plan_service_name,
                                                     PlanToArmGoal)
        rospy.loginfo("[%s] Service plan_to_arm_goal is ready!" % self.node_name)

        rospy.loginfo("[%s] Waiting for service plan_to_arm_waypoints..." % self.node_name)
        rospy.wait_for_service(self.moveit_planner_executor_plan_waypoints_service_name) # blocks until service exists
        self.moveit_plan_waypoints_client = rospy.ServiceProxy(self.moveit_planner_executor_plan_waypoints_service_name,
                                                               PlanToArmWaypoints)
        rospy.loginfo("[%s] Service plan_to_arm_waypoints is ready!" % self.node_name)

        rospy.loginfo("[%s] Waiting for service execute_trajectory..." % self.node_name)
        rospy.wait_for_service(self.moveit_planner_executor_execute_service_name) # blocks until service exists
        self.moveit_execute_client = rospy.ServiceProxy(self.moveit_planner_executor_execute_service_name,
                                                        ExecuteTrajectory)
        rospy.loginfo("[%s] Service execute_to_arm_goal is ready!" % self.node_name)

        # initialize interactive marker
        self.initialize_interactive_marker()

        # internal planning params
        self.curr_arm_name = None
        self.arm_waypoints = []
        self.max_cart_step = 0.1
        self.max_config_step = 5.0
        # initialization does not matter, will immediately be dynamically reconfigured

        # set up dynamic reconfigure server
        self.reconfigure_server = Server(IMPlanningRequestParamsConfig, self.param_reconfigure_callback)

    def initialize_interactive_marker(self):
        # based on stance gen marker in reachability_server/marker_test_client.py

        # initialize marker
        int_marker = InteractiveMarker()

        # set frame, scale, name, and description
        int_marker.header.frame_id = "world"
        int_marker.scale = 1
        int_marker.name = self.int_marker_name
        int_marker.description = self.int_marker_name
        # offset z-coordinate of pose, normalize quaternion
        int_marker.pose.position.z = 1.0
        int_marker.pose.orientation.w = 1.0

        # add control
        move_control = InteractiveMarkerControl()
        move_control.always_visible = True

        # add coordinate marker
        coordinate_markers = []
        x_axis = Marker()
        x_axis.type = Marker.ARROW
        x_axis.scale.x = 0.2
        x_axis.scale.y = 0.05
        x_axis.scale.z = 0.05
        x_axis.color.r = 1.0
        x_axis.color.a = 1.0
        x_axis.pose.orientation.x = 1.0

        y_axis = Marker()
        y_axis.type = Marker.ARROW
        y_axis.scale.x = 0.2
        y_axis.scale.y = 0.05
        y_axis.scale.z = 0.05
        y_axis.color.g = 1.0
        y_axis.color.a = 1.0
        y_axis.pose.orientation.x = 0.5
        y_axis.pose.orientation.y = 0.5
        y_axis.pose.orientation.z = 0.5
        y_axis.pose.orientation.w = 0.5

        z_axis = Marker()
        z_axis.type = Marker.ARROW
        z_axis.scale.x = 0.2
        z_axis.scale.y = 0.05
        z_axis.scale.z = 0.05
        z_axis.color.b = 1.0
        z_axis.color.a = 1.0
        z_axis.pose.orientation.y = -0.707
        z_axis.pose.orientation.w = 0.707
        coordinate_markers.append(x_axis)
        coordinate_markers.append(y_axis)
        coordinate_markers.append(z_axis)

        # add coordinate marker to control
        move_control.markers = coordinate_markers

        # add control to marker
        int_marker.controls.append(move_control)

        # set interaction mode
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MENU #MOVE_3D

        # get 6D controls
        move_controls = self.getMoveMarkerControl()
        rotate_controls = self.getRotateMarkerControl()

        # add 6D control
        full_control = InteractiveMarkerControl()
        full_control.interaction_mode = InteractiveMarkerControl.BUTTON
        full_control.always_visible = True
        int_marker.controls.append(full_control)
        #int_marker.controls += move_controls + rotate_controls

        # add interactive marker to server
        self.im_server.insert(int_marker, self.process_feedback)
        self.initialize_menu_handler()
        self.menu_handler.apply(self.im_server, int_marker.name)
        self.im_server.applyChanges()

        return

    def getMoveMarkerControl(self):
        controls = []
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(control)
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(control)
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(control)
        # int_marker.controls.append(control)
        return controls

    def getRotateMarkerControl(self):
        controls = []
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(control)
        return controls

    def initialize_menu_handler(self):
        # add items to menu handler
        self.setup_menu_handler(closest_arm_checked=True, waypoints_checked=False)

        return

    def setup_menu_handler(self, closest_arm_checked=True, waypoints_checked=True):
        # clear menu handler
        self.menu_handler = MenuHandler()

        # store most recent checked state internall
        self.curr_closest_arm_checked_state = closest_arm_checked
        self.curr_waypoints_checked_state = waypoints_checked

        # add checkbox
        self.closest_arm_checkbox_handle = self.menu_handler.insert("Use Closest Arm for Planning Requests", callback=self.checkbox_callback)

        # depending on whether closest arm is used or not, different planning options available
        if closest_arm_checked:
            # set appropriate check state
            self.menu_handler.setCheckState(self.closest_arm_checkbox_handle, MenuHandler.CHECKED)
            # add closest arm planning button
            self.menu_handler.insert("Request Plan for Closest Arm", callback=self.closest_arm_plan_callback)
        else:
            # set appropriate check state
            self.menu_handler.setCheckState(self.closest_arm_checkbox_handle, MenuHandler.UNCHECKED)
            # add left/right arm planning buttons
            self.menu_handler.insert("Request Plan", callback=self.arm_plan_callback)

        # # add checkbox
        self.waypoints_checkbox_handle = self.menu_handler.insert("Create Sequence of Waypoints", callback=self.checkbox_callback)

        # depending on whether waypoints is checked or not, different planning options available
        if waypoints_checked:
            # set appropriate check state
            self.menu_handler.setCheckState(self.waypoints_checkbox_handle, MenuHandler.CHECKED)
            # add waypoint buttons
            self.menu_handler.insert("        Add Waypoint " + str(len(self.arm_waypoints)+1) + " to Trajectory", callback=self.add_waypoint_callback)
            self.menu_handler.insert("        Remove Last Waypoint from Trajectory", callback=self.remove_last_waypoint_callback)
            self.menu_handler.insert("        Clear All Trajectory Waypoints", callback=self.clear_all_waypoints_callback)
            self.menu_handler.insert("Request Waypoint Plan", callback=self.arm_waypoint_plan_callback)
        else:
            # set appropriate check state
            self.menu_handler.setCheckState(self.waypoints_checkbox_handle, MenuHandler.UNCHECKED)
            # no buttons to add

        # add execute button
        self.menu_handler.insert("Execute Stored Plan", callback=self.execute_callback)

        return

    def param_reconfigure_callback(self, config, level):
        # take params from reconfigure request and store them internally
        self.max_cart_step = config.max_cartesian_step
        self.max_config_step = config.max_configuration_step

        rospy.loginfo("[%s] Reconfigured max Cartesian step to be %f (m) and max configuration step to be %f (radians)" %
                      (self.node_name, self.max_cart_step, self.max_config_step))

        return config

    def process_feedback(self, feedback):
        # callback to process feedback of unknown types
        return

    def checkbox_callback(self, feedback):
        # single callback used to change state of all checkboxes; get handle actually changed
        checkbox_handle = feedback.menu_entry_id

        # get state of changed checkbox
        checkbox_state = self.menu_handler.getCheckState(checkbox_handle)

        if checkbox_handle == self.closest_arm_checkbox_handle:
            # toggle closest arm checkbox state, use stored waypoint checkbox state
            self.setup_menu_handler(closest_arm_checked=(checkbox_state != MenuHandler.CHECKED),
                                    waypoints_checked=self.curr_waypoints_checked_state)
        elif checkbox_handle == self.waypoints_checkbox_handle:
            # use stored closest arm checkbox state, toggle waypoint checkbox state
            self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                    waypoints_checked=(checkbox_state != MenuHandler.CHECKED))
        else:
            # this should never happen, but checking just to be sure
            rospy.logwarn("[%s] Unknown menu entry handle %d, expected %d or %d" %
                          (self.node_name, checkbox_handle, self.closest_arm_checkbox_handle, self.waypoints_checkbox_handle))
            return

        # make sure menu reflects changes
        self.menu_handler.apply(self.im_server, feedback.marker_name)
        self.im_server.applyChanges()

        return

    def request_plan_from_marker_feedback(self, feedback, planning_arm=PlanToArmGoalRequest.LEFT_ARM, plan_time=0.0):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # create pose stamped
            pose_msg = PoseStamped()
            pose_msg.header = marker.header
            pose_msg.pose = marker.pose

            # try calling plan service
            try:
                # request plan based on marker pose
                res = self.moveit_plan_client(planning_arm,
                                              True,
                                              plan_time,
                                              pose_msg)
            except rospy.ServiceException as e:
                rospy.logwarn("[%s] Plan to arm goal service call failed: %s" % self.node_name, e)
                return

            # check success
            if res.success:
                rospy.loginfo("[%s] Planning successful!" % self.node_name)
            else:
                rospy.logwarn("[%s] Planning failed :'(" % self.node_name)

        return

    def arm_plan_callback(self, feedback):
        # make sure current arm name received from IM updates is valid
        if self.curr_arm_name is None:
            rospy.logwarn("[%s] Unknown end-effector name None, expected %s or %s" %
                          (self.node_name, self.left_arm_name, self.right_arm_name))
            return

        if self.curr_arm_name not in self.arm_names:
            rospy.logwarn("[%s] Unknown end-effector name %s, expected %s or %s" %
                          (self.node_name, self.curr_arm_name, self.left_arm_name, self.right_arm_name))
            return

        # set planning arm based on current arm name received from IM updates
        if self.curr_arm_name == self.left_arm_name:
            self.request_plan_from_marker_feedback(feedback, planning_arm=PlanToArmGoalRequest.LEFT_ARM)
        else: # self.curr_arm_name == self.right_arm_name
            self.request_plan_from_marker_feedback(feedback, planning_arm=PlanToArmGoalRequest.RIGHT_ARM)

        return

    def closest_arm_plan_callback(self, feedback):
        self.request_plan_from_marker_feedback(feedback, planning_arm=PlanToArmGoalRequest.ASSIGN_CLOSEST_ARM)

        return

    def add_waypoint_callback(self, feedback):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # create pose stamped
            pose_msg = PoseStamped()
            pose_msg.header = marker.header
            pose_msg.pose = marker.pose

            # add pose to stored waypoints
            self.arm_waypoints.append(pose_msg)

            # update menu and number of waypoints
            self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                    waypoints_checked=self.curr_waypoints_checked_state)

            # make sure menu reflects changes
            self.menu_handler.apply(self.im_server, feedback.marker_name)
            self.im_server.applyChanges()

        return

    def remove_last_waypoint_callback(self, feedback):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # remove last waypoint from stored waypoints
            self.arm_waypoints.pop()

            # update menu and number of waypoints
            self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                    waypoints_checked=self.curr_waypoints_checked_state)

            # make sure menu reflects changes
            self.menu_handler.apply(self.im_server, feedback.marker_name)
            self.im_server.applyChanges()

        return

    def clear_all_waypoints_callback(self, feedback):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # clear all waypoints
            self.arm_waypoints = []

            # update menu and number of waypoints
            self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                    waypoints_checked=self.curr_waypoints_checked_state)

            # make sure menu reflects changes
            self.menu_handler.apply(self.im_server, feedback.marker_name)
            self.im_server.applyChanges()

        return

    def request_waypoint_plan(self, feedback, planning_arm=PlanToArmWaypointsRequest.LEFT_ARM):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # try calling plan waypoints service
            try:
                # request plan based on stored list of waypoints
                res = self.moveit_plan_waypoints_client(planning_arm,
                                                        self.max_cart_step,
                                                        self.max_config_step,
                                                        self.arm_waypoints)
            except rospy.ServiceException as e:
                rospy.logwarn("[%s] Plan to arm waypoints service call failed: %s" % self.node_name, e)
                return

            # check success
            if res.success:
                rospy.loginfo("[%s] Planning successful!" % self.node_name)
            else:
                rospy.logwarn("[%s] Planning failed :'(" % self.node_name)

        return

    def arm_waypoint_plan_callback(self, feedback):
        # make sure current arm name received from IM updates is valid
        if self.curr_arm_name is None:
            rospy.logwarn("[%s] Unknown end-effector name None, expected %s or %s" %
                          (self.node_name, self.left_arm_name, self.right_arm_name))
            return

        if self.curr_arm_name not in self.arm_names:
            rospy.logwarn("[%s] Unknown end-effector name %s, expected %s or %s" %
                          (self.node_name, self.curr_arm_name, self.left_arm_name, self.right_arm_name))
            return

        # set planning arm based on current arm name received from IM updates
        if self.curr_arm_name == self.left_arm_name:
            self.request_waypoint_plan(feedback, planning_arm=PlanToArmGoalRequest.LEFT_ARM)
        else: # self.curr_arm_name == self.right_arm_name
            self.request_waypoint_plan(feedback, planning_arm=PlanToArmGoalRequest.RIGHT_ARM)

        return

    def execute_callback(self, feedback):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # try calling execute service
            try:
                # create empty robot trajectory
                empty_robot_traj = RobotTrajectory()

                # request execution
                res = self.moveit_execute_client(True,
                                                 empty_robot_traj)
            except rospy.ServiceException as e:
                rospy.logwarn("[%s] Execute to arm goal service call failed: %s" % self.node_name, e)
                return

            # check success
            if res.success:
                rospy.loginfo("[%s] Execution successful!" % self.node_name)
            else:
                rospy.logwarn("[%s] Execution failed :'(" % self.node_name)

            # clear all internally stored waypoints
            self.arm_waypoints = []

            # update menu and number of waypoints
            self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                    waypoints_checked=self.curr_waypoints_checked_state)

        return

    def moveit_im_feedback_callback(self, msg):
        # make sure pose exists
        if len(msg.poses) > 0:
            # get pose out of message
            pose = msg.poses[0].pose

            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[%s] Interactive Marker Server does not exist" % self.node_name)
                return

            # get the marker from the server
            marker = self.im_server.get(self.int_marker_name)

            # update marker pose
            marker.pose = pose
            self.im_server.insert(marker)
            self.menu_handler.apply(self.im_server, marker.name)
            self.im_server.applyChanges()

            # check if marker name has changed
            if self.curr_arm_name != msg.poses[0].name:
                # changed arm, clear all internally stored waypoints
                self.arm_waypoints = []
                # update menu and number of waypoints
                self.setup_menu_handler(closest_arm_checked=self.curr_closest_arm_checked_state,
                                        waypoints_checked=self.curr_waypoints_checked_state)

            # store name of marker
            self.curr_arm_name = msg.poses[0].name

        return

if __name__ == '__main__':
    # set node name
    node_name = "arm_goal_interactive_marker_node"

    # initialize node
    rospy.init_node("ArmGoalInteractiveMarkerNode")

    # create IM node
    im_node = ArmGoalInteractiveMarkerNode()
    rospy.loginfo("[%s] Node started!" % im_node.node_name)
    rospy.loginfo("[%s] Remember that rqt_reconfigure may need to be launched separately and/or refreshed to pick up all nodes" % im_node.node_name)

    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[%s] Node stopped, all done!" % im_node.node_name)
