#!/usr/bin/env python

import rospy
import cembra.srv
import config

class Teleop2:
    def __init__(self):
        rospy.init_node("teleop_2")
        rospy.loginfo("Teleop 2")

        rospy.wait_for_service(config.RESET_SERVICE)
        rospy.wait_for_service(config.ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(config.RESET_SERVICE, cembra.srv.ResetEnv2)
        self.action_proxy = rospy.ServiceProxy(config.ACTION_SERVICE, cembra.srv.ActionEnv2)

        self.actions = [1, 1, 1, 1, 1, 1, 1, 1]

    def start(self):
        self.reset_proxy()
    
        while not rospy.is_shutdown():
            self.action_proxy(self.actions)


if __name__ == '__main__':
    try:
        teleop = Teleop2()
        teleop.start()
    except rospy.ROSInterruptException:
        pass
