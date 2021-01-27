#!/usr/bin/env python2

import rospy
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg
#import message_filters

class GripperCommandAction(object):
    """Create a GripperCommandAction as a standard interface for the simulated gripper.

    This is pretty much just a wrapper to use both simulated finger controllers via one
    action simultaneously. The max_effort parameter of the command will be ignored as
    well as no stalling will be detected.
    """
    _left_finger_index = -1
    _right_finger_index = -1
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        self._joint_state_subscriber = rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.state_cb, queue_size=1)
        self._command_publisher_left = rospy.Publisher("franka_left_finger_controller/command", std_msgs.msg.Float64, queue_size=1)
        self._command_publisher_right = rospy.Publisher("franka_right_finger_controller/command", std_msgs.msg.Float64, queue_size=1)
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()

    def state_cb(self, joint_state):
        # get indices where to find values for each finger in joint state message
        if self._left_finger_index == -1 or self._right_finger_index == -1:
            for index, name in enumerate(joint_state.name):
                if name.endswith("finger_joint1"):
                    self._left_finger_index = index
                elif name.endswith("finger_joint2"):
                    self._right_finger_index = index
        else:
            self._feedback.position = (joint_state.position[self._left_finger_index] + joint_state.position[self._right_finger_index]) / 2
            self._feedback.effort = joint_state.effort[self._left_finger_index] + joint_state.effort[self._right_finger_index]

    def execute_cb(self, goal):
        # helper variables
        success = True

        self._feedback.stalled = False
        self._feedback.reached_goal = False

        # limit goal to joint limits
        #TODO get limits from somewhere else than hardcoded values
        if goal.command.position > 0.04:
            limited_goal = 0.04
        else:
            limited_goal = goal.command.position

        # start executing the action
        self._command_publisher_left.publish(std_msgs.msg.Float64(limited_goal))
        self._command_publisher_right.publish(std_msgs.msg.Float64(limited_goal))
        while self._feedback.reached_goal == False:
            # check that preempt has not been requested by the client
            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                success = False
                break

            error = limited_goal - self._feedback.position
            if abs(error) < 0.005: #TODO also consider goal reached when max_effor reached?
                self._feedback.reached_goal = True

            # publish the feedback
            self._action_server.publish_feedback(self._feedback)

        if success:
            self._result.position = self._feedback.position
            self._result.effort = self._feedback.effort
            self._result.stalled = self._feedback.stalled
            self._result.reached_goal = self._feedback.reached_goal
            self._action_server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('franka_gripper')
    server = GripperCommandAction(rospy.get_name() + "/gripper_action")
    rospy.spin()
