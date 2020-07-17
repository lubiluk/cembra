#!/usr/bin/env python
import math as m
import random

import geometry_msgs
import numpy as np
import rospy
import tmc_msgs.msg
import trajectory_msgs.msg
import gazebo_msgs.msg
import sensor_msgs.msg

import cembra.srv
import config
import robot_utils
import simulator_utils

class Environment2:
    """
        An environment with a robot and one cube on a workspace.
        The robot is rewarded when it touches the cube.
    """
    def __init__(self):
        rospy.init_node("environment_2")
        rospy.loginfo("Environment 2")

        self._collision_registered = False

    def start(self):
        self.arm_pub = rospy.Publisher(
            config.UNFILTERED_VELOCITY_TOPIC, tmc_msgs.msg.JointVelocity, queue_size=1)
        self.base_pub = rospy.Publisher(
            config.UNFILTERED_TWIST_TOPIC, geometry_msgs.msg.Twist, queue_size=1)
        self.collision_sub = rospy.Subscriber(
            '/cube_contact_sensor_state', gazebo_msgs.msg.ContactsState, self._handle_collision)

        rospy.loginfo("Waiting for controllers to attach")
        # wait to establish connection between the controller
        while (self.arm_pub.get_num_connections() == 0 
                or self.base_pub.get_num_connections() == 0):
            rospy.sleep(0.1)

        self.reset_srv = rospy.Service(
            config.RESET_SERVICE, cembra.srv.ResetEnv2, self._handle_reset)
        self.action_srv = rospy.Service(
            config.ACTION_SERVICE, cembra.srv.ActionEnv2, self._handle_action)
        
        simulator_utils.go_turbo()

        rospy.loginfo("Environment ready")

        rospy.spin()

    def _handle_reset(self, msg):
        robot_utils.move_to_start_pose()
        simulator_utils.set_model_position('hsrb', 0, 0, 0)
        simulator_utils.set_model_position('wood_cube_5cm', 0.7, 0, 0)
        self._collision_registered = False

        image = robot_utils.get_image(drain=True)

        return image

    def _handle_action(self, msg):
        """
        Handles action in form of 8 integers.

        - 5 for arm
        - 3 for base
        """
        actions = msg.action
        
        self._send_velocities(actions)

        state = robot_utils.get_image()

        reward = 1 if self._collision_registered else -0.01
        is_done = self._collision_registered

        if is_done:
            self._send_velocities([0, 0, 0, 0, 0, 0, 0, 0])

        return (state, reward, is_done)

    def _send_velocities(self, actions):
        assert(len(actions) == 8)

        # Arm velocities
        msg = tmc_msgs.msg.JointVelocity()
        msg.name = [
                'arm_flex_joint',
                'arm_lift_joint',
                'arm_roll_joint',
                'wrist_flex_joint',
                'wrist_roll_joint'
                ]
        msg.velocity = [a * config.SLOW for a in actions[:5]]
        msg.header.stamp = rospy.Time.now()

        self.arm_pub.publish(msg)

        # Base velocity
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = actions[5] * config.SLOW
        msg.linear.y = actions[6] * config.SLOW
        msg.angular.z = actions[7] * config.SLOW
        
        self.base_pub.publish(msg)

        rospy.loginfo("sent velocities")

    def _handle_collision(self, msg):
        for s in msg.states:
            if s.collision2_name.startswith('hsrb::hand') or s.collision2_name.startswith('hsrb::wrist'):
                self._collision_registered = True

def run():
    env = Environment2()
    env.start()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass