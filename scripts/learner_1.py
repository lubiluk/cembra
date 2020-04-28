#!/usr/bin/env python

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

import cembra.srv


class Learner1:
    def __init__(self):
        rospy.init_node("learner_1")
        rospy.loginfo("Learner 1")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action)


    def start(self):
        # Taking action takes at least 0.02 of a second
        # so max rate is 50 hz
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            rospy.loginfo("Resetting environment")

            (goal, state) = self.reset_env()

            # Save images
            try:
                bridge = CvBridge()
                img = bridge.imgmsg_to_cv2(goal, "bgr8")
                cv2.imwrite("goal.png", img)
                img = bridge.imgmsg_to_cv2(state, "bgr8")
                cv2.imwrite("state.png", img)
            except CvBridgeError as cv_bridge_exception:
                rospy.logerr(cv_bridge_exception)

            rospy.loginfo("Acting...")

            while not rospy.is_shutdown():
                (state, reward, is_done) = self.take_action()

                if is_done == 1:
                    rospy.loginfo("Episode ended")
                    break

                rate.sleep()

    def reset_env(self):
        response = self.reset_proxy()

        return (response.goal, response.state)

    def take_action(self):
        action = (-1, -1, -1, -1, -1, -1)
        response = self.action_proxy(action)

        return (response.state, response.reward, response.is_done)


if __name__ == '__main__':
    try:
        learner = Learner1()
        learner.start()
    except rospy.ROSInterruptException:
        pass
