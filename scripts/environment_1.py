#!/usr/bin/env python

import math as m
import random

import geometry_msgs
import rospy
import tmc_msgs.msg

import config
import utils
import cembra.srv


class Environment1:
    def __init__(self):
        rospy.init_node("environment_1")
        rospy.loginfo("Environment 1")

    def start(self):
        self.vel_pub = rospy.Publisher(config.VELOCITY_TOPIC, tmc_msgs.msg.JointVelocity, queue_size=1)
        self.twist_pub = rospy.Publisher(config.TWIST_TOPIC, geometry_msgs.msg.Twist, queue_size=1)

        rospy.loginfo("Waiting for filters to attach")
        # wait to establish connection between the controller
        while self.vel_pub.get_num_connections() == 0 or self.twist_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        self.reset_srv = rospy.Service(config.RESET_SERVICE, cembra.srv.Reset, self.handle_reset)
        self.action_srv = rospy.Service(config.ACTION_SERVICE, cembra.srv.Action, self.handle_action)
        rospy.loginfo("Environment ready")

        rospy.spin()

    def handle_reset(self, _):
        utils.move_to_start_pose()
        utils.set_model_position('hsrb', 0, 0, 0)
        self.distribute_objects()
        rospy.sleep(2)
        goal = utils.get_image()
        self.distribute_objects()
        rospy.sleep(2)
        state = utils.get_image()

        return (goal, state)


    def handle_action(self):
        pass

    def distribute_objects(self):
        for i in range(1, 3):
            # Choose random, reachable location of the cube
            r = random.uniform(config.MIN_ROBOT_DISTANCE, config.MAX_ROBOT_DISTANCE)
            theta = random.uniform(config.MIN_ROBOT_ANGLE, config.MAX_ROBOT_ANGLE)
            x = r * m.cos(theta)
            y = r * m.sin(theta)
            
            utils.set_model_position('cube' + str(i), x, y, 0)

def run():
    env = Environment1()
    env.start()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
