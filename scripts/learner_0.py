#!/usr/bin/env python

import rospy
from cembra.srv import Action, ActionResponse

def take_action():
    try:
        action = (1, 1, 1, 1, 1, 1)

        perform_action = rospy.ServiceProxy('action', Action)
        response = perform_action(action)

        return response.reward
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def run():
    rospy.init_node('learner_0')
    rospy.wait_for_service('action')

    # Taking action takes at least 0.02 of a second
    # so max rate is 50 hz
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        take_action()
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass