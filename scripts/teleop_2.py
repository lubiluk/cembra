#!/usr/bin/env python

import rospy
import cembra.srv
import config
from pynput import keyboard

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
        # self.reset_proxy()

        listener = keyboard.Listener(
            on_press=self.on_press)
        listener.start()
    
        while not rospy.is_shutdown():
            passd

        listener.stop()

    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
        except AttributeError:
            print('special key {0} pressed'.format(
                key))



if __name__ == '__main__':
    try:
        teleop = Teleop2()
        teleop.start()
    except rospy.ROSInterruptException:
        pass
