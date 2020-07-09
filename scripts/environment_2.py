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
        self._image = None

    def start(self):
        self.arm_pub = rospy.Publisher(
            config.UNFILTERED_VELOCITY_TOPIC, tmc_msgs.msg.JointVelocity, queue_size=1)
        self.base_pub = rospy.Publisher(
            config.UNFILTERED_TWIST_TOPIC, geometry_msgs.msg.Twist, queue_size=1)
        self.collision_sub = rospy.Subscriber(
            '/cube_contact_sensor_state', gazebo_msgs.msg.ContactsState, self._handle_collision)
        self.camera_sub = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/rgb/image_rect_color', sensor_msgs.msg.Image, self._handle_image)

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

        return self._image

    def _handle_action(self, msg):
        """
        Handles action in form of 8 integers.

        - 5 for arm
        - 3 for base
        """
        actions = msg.action
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

        # let it move for a bit
        rospy.sleep(0.02)

        state = self._image

        reward = 1 if self._collision_registered else -0.01
        is_done = self._collision_registered

        return (state, reward, is_done)

    def _handle_collision(self, msg):
        for s in msg.states:
            if s.collision2_name.split('::')[0] == 'hsrb':
                self._collision_registered = True

    def _handle_image(self, msg):
        self._image = msg

def run():
    env = Environment2()
    env.start()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass