#!/usr/bin/env python

import math as m
import random

import geometry_msgs
import numpy as np
import rospy
import tmc_msgs.msg
import trajectory_msgs.msg

import cembra.srv
import config
import robot_utils
import simulator_utils


class Environment1:
    def __init__(self):
        rospy.init_node("environment_1")
        rospy.loginfo("Environment 1")

    def start(self):
        self.vel_pub = rospy.Publisher(config.FILTERED_VELOCITY_TOPIC, tmc_msgs.msg.JointVelocity, queue_size=1)
        self.twist_pub = rospy.Publisher(config.UNFILTERED_TWIST_TOPIC, geometry_msgs.msg.Twist, queue_size=1)

        rospy.loginfo("Waiting for filters to attach")
        # wait to establish connection between the controller
        while (self.vel_pub.get_num_connections() == 0 
                or self.twist_pub.get_num_connections() == 0):
            rospy.sleep(0.1)

        self.reset_srv = rospy.Service(config.RESET_SERVICE, cembra.srv.Reset, self.handle_reset)
        self.action_srv = rospy.Service(config.ACTION_SERVICE, cembra.srv.Action, self.handle_action)
        
        simulator_utils.go_turbo()

        rospy.loginfo("Environment ready")

        rospy.spin()

    def handle_reset(self, _):
        robot_utils.move_to_start_pose()
        # simulator_utils.pause()
        simulator_utils.set_model_position('hsrb', 0, 0, 0)
        self.distribute_objects()
        rospy.sleep(2)
        self.save_goal_positions()
        # simulator_utils.resume()
        goal = robot_utils.get_image()
        # simulator_utils.pause()
        self.distribute_objects()
        # simulator_utils.resume()
        rospy.sleep(2)
        state = robot_utils.get_image()
        # simulator_utils.resume()

        return (goal, state)


    def handle_action(self, msg):
        """
        Handles action in form of 8 integers.

        - 5 for arm
        - 1 for base
        """
        actions = msg.action
        assert(len(actions) == 6)

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

        self.vel_pub.publish(msg)

        # Base velocity
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = actions[5] * config.SLOW
        
        self.twist_pub.publish(msg)

        rospy.sleep(0.02)

        state = robot_utils.get_image()
        reward = 1 if self.is_goal_reached() else 0
        is_done = reward

        return (state, reward, is_done)

    def distribute_objects(self):
        for i in range(1, 3):
            # Choose random, reachable location of the cube
            r = random.uniform(config.MIN_ROBOT_DISTANCE, config.MAX_ROBOT_DISTANCE)
            theta = random.uniform(config.MIN_ROBOT_ANGLE, config.MAX_ROBOT_ANGLE)
            x = r * m.cos(theta)
            y = r * m.sin(theta)
            
            simulator_utils.set_model_position('cube' + str(i), x, y, 0)

    def save_goal_positions(self):
        self.goal_positions = [simulator_utils.get_model_position('cube' + str(i)) for i in range(1, 3)]

    def is_goal_reached(self):
        self.current_positions = [simulator_utils.get_model_position('cube' + str(i)) for i in range(1, 3)]
        
        return np.allclose(self.goal_positions, self.current_positions, 0.01)

def run():
    env = Environment1()
    env.start()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
