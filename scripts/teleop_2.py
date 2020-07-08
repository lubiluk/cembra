#!/usr/bin/env python

import rospy
import cembra.srv
import config
from pynput import keyboard

KEY_MAP = {
    'q': (0, -1),
    'a': (0, 1),
    'w': (1, 1),
    's': (1, -1),
    'e': (2, 1),
    'd': (2, -1),
    'r': (3, 1),
    'f': (3, -1),
    't': (4, 1),
    'g': (4, -1),
    'y': (5, 1),
    'h': (5, -1),
    'u': (6, 1),
    'j': (6, -1),
    'i': (7, 1),
    'k': (7, -1)
}

class Teleop2:
    def __init__(self):
        rospy.init_node("teleop_2")
        rospy.loginfo("Teleop 2")

        rospy.wait_for_service(config.RESET_SERVICE)
        rospy.wait_for_service(config.ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(config.RESET_SERVICE, cembra.srv.ResetEnv2)
        self.action_proxy = rospy.ServiceProxy(config.ACTION_SERVICE, cembra.srv.ActionEnv2)

        self.actions = [0, 0, 0, 0, 0, 0, 0, 0]

    def start(self):
        self.reset_proxy()

        listener = keyboard.Listener(
            on_press=self.on_press)
        listener.start()
    
        while not rospy.is_shutdown():
            self.action_proxy(self.actions)
            self.actions = [0, 0, 0, 0, 0, 0, 0, 0]

        listener.stop()

    def on_press(self, key):
        a = KEY_MAP[key.char]

        if a is not None:
            self.actions[a[0]] = a[1]




if __name__ == '__main__':
    try:
        teleop = Teleop2()
        teleop.start()
    except rospy.ROSInterruptException:
        pass
