#!/usr/bin/env python

import cembra.srv
import rospy

from frame_buffer import FrameBuffer
from observation_processor import ObservationProcessor

RESET_SERVICE = '/cembra/reset'
ACTION_SERVICE = '/cembra/action'

class Learner3:
    def __init__(self):
        rospy.init_node("learner_3_ddpg")
        rospy.loginfo("Learner 3 DDPG")

        rospy.wait_for_service(RESET_SERVICE)
        rospy.wait_for_service(ACTION_SERVICE)

        self.reset_proxy = rospy.ServiceProxy(RESET_SERVICE, cembra.srv.Reset3)
        self.action_proxy = rospy.ServiceProxy(ACTION_SERVICE, cembra.srv.Action3)

        self.observation_processor = ObservationProcessor()
        self.frame_buffer = FrameBuffer(640, 480, 4)

        self.n_actions = 8

    def start(self):
        pass

    def _reset_env(self):
        response = self.reset_proxy()

        self.frame_buffer.reset()

        state = self.observation_processor.process(response.state)
        state = self.frame_buffer.update(state)

        return state

    def _take_action(self, velocities):
        response = self.action_proxy(velocities)

        state = self.observation_processor.process(response.state)
        state = self.frame_buffer.update(state)

        return (state, response.reward, response.is_done)

if __name__ == '__main__':
    try:
        learner = Learner3()
        learner.start()
    except rospy.ROSInterruptException:
        pass
