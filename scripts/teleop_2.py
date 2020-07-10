#!/usr/bin/env python

import rospy
import cembra.srv
import config
from pynput import keyboard
import sys
import h5py
import uuid
import numpy as np
from ros_numpy import numpify

KEY_MAP = {
    'q': (0, 1),
    'a': (0, -1),
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

        self.pressed_keys = set()

    def start(self):
        stop = False

        while not stop:
            self.play_and_record()

            answer = raw_input("Repeat? [Y/n]")

            if answer == 'n':
                stop = True


    def play_and_record(self):
        states = np.zeros((10000, 480, 640, 3), dtype='uint8')
        actions = np.zeros((10000, 8), dtype='i')
        rewards = np.zeros((10000, 1), dtype='f')
        is_dones = np.zeros((10000, 1), dtype='?')
        step = 0

        result = self.reset_proxy()
        state = numpify(result.state)

        rospy.loginfo("Start!")

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()
    
        while not rospy.is_shutdown():
            action = [0, 0, 0, 0, 0, 0, 0, 0]

            for k in self.pressed_keys:
                mapping = KEY_MAP[k]
                action[mapping[0]] = mapping[1]


            # rospy.loginfo(actions)
            result = self.action_proxy(action)
            states[step] = state
            actions[step] = action
            rewards[step] = result.reward
            is_dones[step] = result.is_done

            step += 1
            state = numpify(result.state)

            if result.is_done:
                rospy.loginfo("Success!")
                break

        listener.stop()

        path = "/tmp/{}.hdf5".format(uuid.uuid4())

        with h5py.File(path, 'w') as f:
            f.create_dataset("states", data=states[:step])
            f.create_dataset("actions", data=actions[:step])
            f.create_dataset("rewards", data=rewards[:step])
            f.create_dataset("is_dones", data=is_dones[:step])

    def on_press(self, key):
        if key.char in KEY_MAP.keys():
            self.pressed_keys.add(key.char)
        
        sys.stdout.write('\b')

    def on_release(self, key):
        if key.char in self.pressed_keys:
            self.pressed_keys.remove(key.char)




if __name__ == '__main__':
    try:
        teleop = Teleop2()
        teleop.start()
    except rospy.ROSInterruptException:
        pass
