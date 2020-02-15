#!/usr/bin/env python

import rospy
import sensor_msgs.msg

CAMERA_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'

rospy.init_node('perceptor')

def get_image():
    msg = rospy.wait_for_message(CAMERA_TOPIC, sensor_msgs.msg.Image)
    print(msg)

def run():
    get_image()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass