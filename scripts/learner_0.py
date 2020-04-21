#!/usr/bin/env python

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

import rospy
import cembra.srv

def reset_env():
    try:
        perform_action = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset)
        response = perform_action()

        return response.reward
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def take_action():
    try:
        action = (1, 1, 1, 1, 1, 1)

        perform_action = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action)
        response = perform_action(action)

        return response.reward
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: {}".format(e))

def run():
    rospy.init_node('learner_0')
    rospy.wait_for_service(ACTION_SERVICE)

    # Taking action takes at least 0.02 of a second
    # so max rate is 50 hz
    rate = rospy.Rate(50)

    reset_env()

    while not rospy.is_shutdown():
        take_action()
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass