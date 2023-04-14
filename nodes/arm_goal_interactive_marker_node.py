#!/usr/bin/env python3
"""
(Kind of Hacky) Arm Goal Interactive Marker Node
Emily Sheetz, Winter 2023
"""

import rospy
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate, InteractiveMarker, InteractiveMarkerControl, Marker
from moveit_msgs.msg import RobotTrajectory

# services
from val_moveit_planner_executor.srv import PlanToArmGoal, PlanToArmGoalRequest, PlanToArmGoalResponse
from val_moveit_planner_executor.srv import ExecuteToArmGoal, ExecuteToArmGoalRequest, ExecuteToArmGoalResponse

# Interactive Marker imports
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import MenuHandler

class ArmGoalInteractiveMarkerNode:
    def __init__(self):
        # initialize names
        self.im_server_name = "hacky_moveit_arm_goals"
        self.moveit_im_topic_name = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update"
        self.moveit_planner_executor_plan_service_name = "/ValkyrieMoveItPlannerExecutorServerNode/plan_to_arm_goal"
        self.moveit_planner_executor_execute_service_name = "/ValkyrieMoveItPlannerExecutorServerNode/execute_to_arm_goal"

        # initialize interactive marker server
        self.im_server = InteractiveMarkerServer(self.im_server_name)

        # initialize MoveIt IM update subscriber
        self.moveit_im_sub = rospy.Subscriber(self.moveit_im_topic_name, InteractiveMarkerUpdate, self.moveit_im_feedback_callback)

        # initialize val_moveit_planner_executor services
        rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Waiting for service plan_to_arm_goal...")
        rospy.wait_for_service(self.moveit_planner_executor_plan_service_name) # blocks until service exists
        self.moveit_plan_client = rospy.ServiceProxy(self.moveit_planner_executor_plan_service_name,
                                                     PlanToArmGoal)
        rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Service plan_to_arm_goal is ready!")

        rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Waiting for service execute_to_arm_goal...")
        rospy.wait_for_service(self.moveit_planner_executor_execute_service_name) # blocks until service exists
        self.moveit_execute_client = rospy.ServiceProxy(self.moveit_planner_executor_execute_service_name,
                                                        ExecuteToArmGoal)
        rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Service execute_to_arm_goal is ready!")

        # initialize interactive marker
        self.initialize_interactive_marker()

    def initialize_interactive_marker(self):
        # based on stance gen marker in reachability_server/marker_test_client.py

        # initialize marker
        int_marker = InteractiveMarker()

        # set frame, scale, name, and description
        int_marker.header.frame_id = "world"
        int_marker.scale = 1
        int_marker.name = "Hacky MoveIt Arm Goal Target"
        self.int_marker_name = "Hacky MoveIt Arm Goal Target"
        int_marker.description = "Hacky MoveIt Arm Goal Target"
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
        x_axis.scale.x = 0.3
        x_axis.scale.y = 0.05
        x_axis.scale.z = 0.05
        x_axis.color.r = 1.0
        x_axis.color.a = 1.0
        x_axis.pose.orientation.x = 1.0

        y_axis = Marker()
        y_axis.type = Marker.ARROW
        y_axis.scale.x = 0.3
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
        z_axis.scale.x = 0.3
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
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

        # get 6D controls
        move_controls = self.getMoveMarkerControl()
        rotate_controls = self.getRotateMarkerControl()

        # add 6D control
        full_control = InteractiveMarkerControl()
        full_control.interaction_mode = InteractiveMarkerControl.BUTTON
        full_control.always_visible = True
        int_marker.controls.append(full_control)
        int_marker.controls += move_controls + rotate_controls

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
        # initialize menu handler
        self.menu_handler = MenuHandler()

        # add items to menu handler
        self.menu_handler.insert("Request Plan for Left Arm", callback=self.left_arm_plan_callback)
        self.menu_handler.insert("Request Plan for Right Arm", callback=self.right_arm_plan_callback)
        self.menu_handler.insert("Execute Stored Plan", callback=self.execute_callback)

        return

    def process_feedback(self, feedback):
        # callback to process feedback of unknown types
        return

    def request_plan_from_marker_feedback(self, feedback, left_arm=True, plan_time=0.0):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Interactive Marker Server does not exist")
                return

            # get the marker from the server
            marker = self.im_server.get(feedback.marker_name)

            # try calling plan service
            try:
                # set arm for planning request
                planning_arm = -1
                if left_arm:
                    planning_arm = PlanToArmGoalRequest.LEFT_ARM
                else:
                    planning_arm = PlanToArmGoalRequest.RIGHT_ARM

                # request plan based on marker pose
                res = self.moveit_plan_client(planning_arm,
                                              True,
                                              plan_time,
                                              marker.pose)
            except rospy.ServiceException as e:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Plan to arm goal service call failed: %s" % e)
                return

            # check success
            if res.success:
                rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Planning successful!")
            else:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Planning failed :'(")

        return

    def left_arm_plan_callback(self, feedback):
        self.request_plan_from_marker_feedback(feedback, left_arm=True)

        return

    def right_arm_plan_callback(self, feedback):
        self.request_plan_from_marker_feedback(feedback, left_arm=False)

        return

    def execute_callback(self, feedback):
        # check type of feedback event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Interactive Marker Server does not exist")
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
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Execute to arm goal service call failed: %s" % e)
                return

            # check success
            if res.success:
                rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Execution successful!")
            else:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Execution failed :'(")

        return

    def moveit_im_feedback_callback(self, msg):
        # make sure pose exists
        if len(msg.poses) > 0:
            # get pose out of message
            pose = msg.poses[0].pose

            # verify that server exists
            if self.im_server is None:
                rospy.logwarn("[Hacky MoveIt Arm Goal IM Node] Interactive Marker Server does not exist")
                return

            # get the marker from the server
            marker = self.im_server.get(self.int_marker_name)

            # update marker pose
            marker.pose = pose
            self.im_server.insert(marker)
            self.im_server.applyChanges()

if __name__ == '__main__':
    # set node name
    node_name = "arm_goal_interactive_marker_node"

    # initialize node
    rospy.init_node("ArmGoalInteractiveMarkerNode")

    # create IM node
    im_node = ArmGoalInteractiveMarkerNode()
    rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Node started!")

    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("[Hacky MoveIt Arm Goal IM Node] Node stopped, all done!")
