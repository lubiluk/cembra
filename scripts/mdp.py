#!/usr/bin/env python

import rospy
import sensor_msgs.msg
from cembra.srv import Action

CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'

def get_image():
    msg = rospy.wait_for_message(CAMERA_TOPIC, sensor_msgs.msg.Image)
    print(msg)

def handle_action():
    pass

def run():
    rospy.init_node('mdp')
    s = rospy.Service('action', Action, handle_action)
    print "Ready to execute actions."
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass